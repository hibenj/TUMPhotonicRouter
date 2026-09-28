"""Abstract one-sided pad-slot planning for electrical routing."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .individual_topology import bundle_route_side
from .types import (
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    EscapeBundle,
    IndividualEscapeTopologyResult,
    PadAssignment,
    PadPlan,
    PadSlot,
    Side,
)
from .pitch_grid import bbox_to_grid_cells


@dataclass(frozen=True)
class _PadInterval:
    items: tuple[tuple[ElectricalTerminal, int | None, int | None], ...]
    preferred_x_um: float
    route_side: str | None = None


def plan_pad_slots(
    common_bus: CommonBusRoutingResult,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    escape_topology: IndividualEscapeTopologyResult | None = None,
) -> PadPlan:
    """Assign used electrical pads to legal pitch slots without creating geometry.

    The plan is abstract: all slots describe legal candidate locations, while
    only ``assignments`` become bondpad rectangles in a later realization stage.
    If escape topology is available, individual control terminals are assigned
    bundle-by-bundle using the order induced by the escape corridors. Otherwise
    they fall back to x-coordinate order. The common bus receives one assigned
    slot at the left or right end of the assignment sequence.
    """

    config.validate()
    if not common_bus.success:
        raise ValueError(
            "cannot plan pad slots until the common bus connected every heater; "
            f"failed heaters: {common_bus.failed_heaters}"
        )

    first_assignment_index = config.pad_extra_slots_left
    individual_items = _individual_assignment_items(common_bus, escape_topology)
    intervals = _pad_intervals(individual_items, obstacle_map, escape_topology)
    individual_indices = _assign_interval_indices(intervals, first_assignment_index, config)
    origin_x = _resolve_origin_x(
        intervals,
        individual_indices,
        obstacle_map,
        config,
        escape_topology,
    )
    common_bus_index = _common_bus_pad_slot_index(
        origin_x,
        individual_indices,
        obstacle_map,
        config,
    )
    assignment_indices = [*individual_indices, common_bus_index]
    min_slot_index = min(0, min(assignment_indices, default=0))
    max_slot_index = max(0, max(assignment_indices, default=0)) + config.pad_extra_slots_right
    pad_offset_um = max(
        config.pad_offset_um,
        _required_pad_channel_offset_um(
            _pad_channel_route_count(
                intervals,
                len(individual_indices),
                config,
            ),
            config,
        ),
    )

    slot_by_index = {
        index: _make_slot(index, origin_x, obstacle_map, config, pad_offset_um)
        for index in range(min_slot_index, max_slot_index + 1)
    }
    slot_by_index[common_bus_index] = _make_slot(
        common_bus_index,
        origin_x,
        obstacle_map,
        config,
        pad_offset_um,
        width_um=config.common_bus_bondpad_width_um,
        length_um=config.common_bus_bondpad_length_um,
        center_x_um=_snap_x_to_grid_center(
            origin_x + common_bus_index * config.pad_pitch_um,
            obstacle_map,
        ),
    )

    assignments: list[PadAssignment] = []
    for assignment_index, (terminal, bundle_id, topology_rank) in zip(
        individual_indices,
        individual_items,
    ):
        slot = slot_by_index[assignment_index]
        assignments.append(
            PadAssignment(
                slot=slot,
                net_id=f"individual:{terminal.heater_id}",
                kind="individual",
                terminal=terminal,
                heater_id=terminal.heater_id,
                topology_bundle_id=bundle_id,
                topology_rank=topology_rank,
            )
        )
    assignments.append(
        PadAssignment(
            slot=slot_by_index[common_bus_index],
            net_id="common_bus",
            kind="common_bus",
            terminal=None,
            heater_id=None,
        )
    )

    assigned_indices = {assignment.slot.index for assignment in assignments}
    slots = tuple(slot_by_index[index] for index in sorted(slot_by_index))
    empty_slots = tuple(slot for slot in slots if slot.index not in assigned_indices)
    return PadPlan(
        side=config.pad_side,
        pitch_um=config.pad_pitch_um,
        origin_x_um=origin_x,
        slots=slots,
        assignments=tuple(assignments),
        empty_slots=empty_slots,
    )


def _individual_assignment_items(
    common_bus: CommonBusRoutingResult,
    escape_topology: IndividualEscapeTopologyResult | None,
) -> tuple[tuple[ElectricalTerminal, int | None, int | None], ...]:
    if escape_topology is None or not escape_topology.terminal_order:
        return tuple(
            (terminal, None, None)
            for terminal in sorted(
                common_bus.unselected_terminals.values(),
                key=lambda terminal: (terminal.center[0], terminal.center[1], terminal.id),
            )
        )

    metadata_by_terminal_id: dict[str, tuple[int | None, int | None]] = {}
    for bundle in escape_topology.bundles:
        for rank, terminal in enumerate(bundle.ordered_terminals):
            metadata_by_terminal_id[terminal.id] = (bundle.bundle_id, rank)

    assigned_terminal_ids: set[str] = set()
    items: list[tuple[ElectricalTerminal, int | None, int | None]] = []
    for terminal in escape_topology.terminal_order:
        assigned_terminal_ids.add(terminal.id)
        bundle_id, topology_rank = metadata_by_terminal_id.get(terminal.id, (None, None))
        items.append((terminal, bundle_id, topology_rank))

    # Keep topology order first, but do not drop failed/unordered terminals.
    for terminal in sorted(
        common_bus.unselected_terminals.values(),
        key=lambda terminal: (terminal.center[0], terminal.center[1], terminal.id),
    ):
        if terminal.id in assigned_terminal_ids:
            continue
        items.append((terminal, None, None))

    return tuple(items)


def _pad_intervals(
    individual_items: tuple[tuple[ElectricalTerminal, int | None, int | None], ...],
    obstacle_map: ElectricalObstacleMap,
    escape_topology: IndividualEscapeTopologyResult | None,
) -> tuple[_PadInterval, ...]:
    if not individual_items:
        return ()
    preferred_x_by_terminal_id = _preferred_pad_x_by_terminal_id(obstacle_map, escape_topology)
    bundle_by_id = (
        {bundle.bundle_id: bundle for bundle in escape_topology.bundles}
        if escape_topology is not None
        else {}
    )
    intervals: list[_PadInterval] = []
    current_items: list[tuple[ElectricalTerminal, int | None, int | None]] = []
    current_bundle_id: int | None | object = object()
    for item in individual_items:
        terminal, bundle_id, _ = item
        if current_items and bundle_id != current_bundle_id:
            intervals.append(
                _make_interval(
                    tuple(current_items),
                    preferred_x_by_terminal_id,
                    bundle_by_id,
                    obstacle_map,
                )
            )
            current_items = []
        current_items.append(item)
        current_bundle_id = bundle_id
    if current_items:
        intervals.append(
            _make_interval(
                tuple(current_items),
                preferred_x_by_terminal_id,
                bundle_by_id,
                obstacle_map,
            )
        )
    return tuple(intervals)


def _make_interval(
    items: tuple[tuple[ElectricalTerminal, int | None, int | None], ...],
    preferred_x_by_terminal_id: dict[str, float],
    bundle_by_id: dict[int, EscapeBundle],
    obstacle_map: ElectricalObstacleMap,
) -> _PadInterval:
    preferred_values = tuple(
        preferred_x_by_terminal_id.get(terminal.id, terminal.center[0]) for terminal, _, _ in items
    )
    bundle_id = items[0][1] if items else None
    bundle = bundle_by_id.get(bundle_id) if bundle_id is not None else None
    route_side = bundle_route_side(bundle, obstacle_map) if bundle is not None else None
    return _PadInterval(
        items=items,
        preferred_x_um=_median_float(preferred_values),
        route_side=route_side,
    )


def _assign_interval_indices(
    intervals: tuple[_PadInterval, ...],
    first_assignment_index: int,
    config: ElectricalRoutingConfig,
) -> list[int]:
    if not intervals:
        return []
    if config.pad_origin_x_um is not None:
        stride = config.pad_empty_slots_between_assignments + 1
        return [
            first_assignment_index + item_index * stride
            for item_index in range(sum(len(interval.items) for interval in intervals))
        ]

    preferred_starts = [
        _interval_preferred_start(interval, config.pad_pitch_um) for interval in intervals
    ]
    min_preferred_start = min(preferred_starts)
    previous_end = first_assignment_index - 1 - config.pad_empty_slots_between_assignments
    assigned_indices: list[int] = []
    for interval, preferred_start in zip(intervals, preferred_starts):
        shifted_start = preferred_start - min_preferred_start + first_assignment_index
        start_index = max(
            shifted_start,
            previous_end + config.pad_empty_slots_between_assignments + 1,
        )
        assigned_indices.extend(range(start_index, start_index + len(interval.items)))
        previous_end = start_index + len(interval.items) - 1
    return assigned_indices


def _interval_preferred_start(
    interval: _PadInterval,
    pitch_um: float,
) -> int:
    """Return the desired first slot index for one bundle's pad block.

    A bundle whose terminals share a column and exit to the same side ("L's
    nest") is packed entirely on the exit side of the terminal column, one
    slot outward from it, so the terminal nearest the pad row gets the pad
    nearest the terminal column and each terminal further from the pad row
    gets the next pad column outward. A lone terminal (nothing to nest
    against) or a terminal with no known exit side keeps the terminal's own
    column, which gives it a zero-detour wire when the column is free.
    """

    terminal_col_index = round(interval.preferred_x_um / pitch_um)
    if len(interval.items) <= 1 or interval.route_side is None:
        return round(interval.preferred_x_um / pitch_um - (len(interval.items) - 1) / 2.0)
    if interval.route_side == "left":
        return terminal_col_index - len(interval.items)
    return terminal_col_index + 1


def _interval_reference_index(
    interval: _PadInterval,
    interval_indices: list[int],
) -> float:
    """Return the abstract index that ``interval.preferred_x_um`` should map to.

    Mirrors ``_interval_preferred_start``: for a nested multi-terminal bundle
    the terminal column sits one slot outside the block's inner (nearest)
    edge rather than at the block's center.
    """

    if len(interval.items) <= 1 or interval.route_side is None:
        return (interval_indices[0] + interval_indices[-1]) / 2.0
    if interval.route_side == "left":
        return interval_indices[-1] + 1
    return interval_indices[0] - 1


def _resolve_origin_x(
    intervals: tuple[_PadInterval, ...],
    individual_indices: list[int],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    escape_topology: IndividualEscapeTopologyResult | None,
) -> float:
    if config.pad_origin_x_um is not None:
        return float(config.pad_origin_x_um)
    if not intervals:
        return 0.0
    desired_origins = []
    item_cursor = 0
    for interval in intervals:
        interval_indices = individual_indices[item_cursor : item_cursor + len(interval.items)]
        item_cursor += len(interval.items)
        if not interval_indices:
            continue
        reference_index = _interval_reference_index(interval, interval_indices)
        desired_origins.append(interval.preferred_x_um - reference_index * config.pad_pitch_um)
    return _median_float(tuple(desired_origins))


def _preferred_pad_x_by_terminal_id(
    obstacle_map: ElectricalObstacleMap,
    escape_topology: IndividualEscapeTopologyResult | None,
) -> dict[str, float]:
    if escape_topology is None:
        return {}
    origin_x, _ = obstacle_map.grid.origin
    grid_size = obstacle_map.grid.grid_size_um
    preferred: dict[str, float] = {}
    for route in escape_topology.routes:
        if route.exit_cell is None:
            continue
        preferred[route.terminal.id] = origin_x + (route.exit_cell[0] + 0.5) * grid_size
    return preferred


def _median_float(values: tuple[float, ...]) -> float:
    if not values:
        return 0.0
    sorted_values = sorted(values)
    mid = len(sorted_values) // 2
    if len(sorted_values) % 2 == 1:
        return sorted_values[mid]
    return (sorted_values[mid - 1] + sorted_values[mid]) / 2.0


def _common_bus_pad_slot_index(
    origin_x: float,
    individual_indices: list[int],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    layout_xmin, _, layout_xmax, _ = obstacle_map.layout_bbox
    if config.common_bus_pad_position == "left":
        side_x = layout_xmin - max(
            config.bus_x_margin_um,
            config.common_bus_bondpad_width_um,
        )
        side_index = math.floor((side_x - origin_x) / config.pad_pitch_um)
        next_sequence_index = min(individual_indices, default=0) - (
            config.pad_empty_slots_between_assignments + 1
        )
        return min(side_index, next_sequence_index)

    side_x = layout_xmax + max(
        config.bus_x_margin_um,
        config.common_bus_bondpad_width_um,
    )
    side_index = math.ceil((side_x - origin_x) / config.pad_pitch_um)
    next_sequence_index = max(individual_indices, default=-1) + (
        config.pad_empty_slots_between_assignments + 1
    )
    return max(side_index, next_sequence_index)


def _make_slot(
    index: int,
    origin_x: float,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    pad_offset_um: float,
    *,
    width_um: float | None = None,
    length_um: float | None = None,
    center_x_um: float | None = None,
    center_y_um: float | None = None,
    side: Side | None = None,
) -> PadSlot:
    slot_side = config.pad_side if side is None else side
    center_x = origin_x + index * config.pad_pitch_um if center_x_um is None else center_x_um
    half_width = (config.bondpad_width_um if width_um is None else width_um) / 2.0
    half_length = (config.bondpad_length_um if length_um is None else length_um) / 2.0
    _, layout_ymin, _, layout_ymax = obstacle_map.layout_bbox
    if center_y_um is not None:
        center_y = center_y_um
    elif slot_side == "top":
        center_y = layout_ymax + pad_offset_um + half_length
    else:
        center_y = layout_ymin - pad_offset_um - half_length
    bbox = (
        center_x - half_width,
        center_y - half_length,
        center_x + half_width,
        center_y + half_length,
    )
    return PadSlot(
        index=index,
        center=(center_x, center_y),
        bbox=bbox,
        side=slot_side,
    )


def _snap_x_to_grid_center(
    x_um: float,
    obstacle_map: ElectricalObstacleMap,
) -> float:
    grid = obstacle_map.grid
    grid_size = grid.grid_size_um
    origin_x = grid.origin[0]
    grid_x = round((x_um - origin_x) / grid_size - 0.5)
    return origin_x + (grid_x + 0.5) * grid_size


def _required_pad_channel_offset_um(
    individual_route_count: int,
    config: ElectricalRoutingConfig,
) -> float:
    if individual_route_count <= 0:
        return config.pad_offset_um
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    return individual_route_count * track_pitch_um + config.wire_width_um


def _pad_channel_route_count(
    intervals: tuple[_PadInterval, ...],
    fallback_route_count: int,
    config: ElectricalRoutingConfig,
) -> int:
    if config.pad_origin_x_um is not None:
        return fallback_route_count
    if not intervals:
        return fallback_route_count
    return max(len(interval.items) for interval in intervals)


def pad_access_bbox(
    slot: PadSlot,
    config: ElectricalRoutingConfig,
    *,
    width_um: float | None = None,
) -> tuple[float, float, float, float]:
    """Return the centered chip-facing port region for a pad slot."""

    xmin, ymin, xmax, ymax = slot.bbox
    center_x = (xmin + xmax) / 2.0
    access_width = config.wire_width_um if width_um is None else width_um
    half_access_width = min(access_width, xmax - xmin) / 2.0
    depth = min(config.pad_access_depth_um, ymax - ymin)
    if slot.side == "top":
        return (
            center_x - half_access_width,
            ymin,
            center_x + half_access_width,
            ymin + depth,
        )
    return (
        center_x - half_access_width,
        ymax - depth,
        center_x + half_access_width,
        ymax,
    )


def pad_access_cells(
    slot: PadSlot,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    width_um: float | None = None,
) -> frozenset[tuple[int, int]]:
    """Return grid cells where routes may connect to this pad slot."""

    return bbox_to_grid_cells(
        pad_access_bbox(slot, config, width_um=width_um),
        obstacle_map.grid,
    )
