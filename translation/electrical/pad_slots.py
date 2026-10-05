"""Abstract one-sided pad-slot planning for electrical routing."""

from __future__ import annotations

import math
from itertools import pairwise

from .individual_topology import bundle_route_side
from .pitch_grid import bbox_to_grid_cells
from .river_routing import (
    ChannelWire,
    assign_tracks,
    bundle_opening_edge,
    clearance_in_cells,
    corridor_lane_column,
    pad_column_cell,
    track_pitch_in_cells,
)
from .types import (
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    IndividualEscapeTopologyResult,
    PadAssignment,
    PadPlan,
    PadSlot,
    Side,
)
from .wire_geometry import wire_reservation_radius_cells


def plan_pad_slots(
    common_bus: CommonBusRoutingResult,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    escape_topology: IndividualEscapeTopologyResult | None = None,
) -> PadPlan:
    """Assign used electrical pads to legal pitch slots without creating geometry.

    The plan is abstract: all slots describe legal candidate locations, while
    only ``assignments`` become bondpad rectangles in a later realization stage.

    Pad choice (pad-choice plan, 2026-10-05): every individual terminal has
    one corridor lane, the column its pad wire rises on beside its heater
    (``river_routing.corridor_lane_column`` of its bundle's opening edge and
    its row, exactly as the channel router will build it; a terminal without
    an escape bundle takes its own column). The pads are assigned to the
    wires in the order of those lanes, one per slot of the pad pitch, so
    that the sum of the horizontal runs (lane to pad column) is as small as
    possible, and the pad row's origin is chosen by the same measure
    (``assign_pad_slots_with_origin``). The lane order is forced by the one
    metal layer: two wires whose pads are out of lane order always have
    runs that must cross. An eight-row bundle therefore gets half its pads on
    each side of its lane band, never one long block on its exit side. The
    common bus receives one assigned slot at the configured end of the
    sequence with the gap its escape band needs (``common_bus_pad_slot_gap``).
    A forced ``pad_origin_x_um`` keeps the owner's placement: pads from the
    first index in topology order, one per stride.

    The channel's height (``pad_offset_um``, the gap between the layout and
    the pad row) is sized from the channel router's own track assignment
    run on these lanes and pad columns (``_pad_channel_track_count``):
    tracks depend on pad columns only, not on the pad row's height, so the
    count is known before the slots are made.
    """

    config.validate()
    if not common_bus.success:
        raise ValueError(
            "cannot plan pad slots until the common bus connected every heater; "
            f"failed heaters: {common_bus.failed_heaters}"
        )

    first_assignment_index = config.pad_extra_slots_left
    individual_items = _individual_assignment_items(common_bus, escape_topology)
    lane_by_terminal_id = _lane_by_terminal_id(
        individual_items, obstacle_map, config, escape_topology
    )
    if config.pad_origin_x_um is not None:
        origin_x = float(config.pad_origin_x_um)
        stride = config.pad_empty_slots_between_assignments + 1
        individual_indices = [
            first_assignment_index + item_index * stride
            for item_index in range(len(individual_items))
        ]
    else:
        order = sorted(
            range(len(individual_items)),
            key=lambda item_index: (
                lane_by_terminal_id[individual_items[item_index][0].id][0],
                item_index,
            ),
        )
        origin_x, sorted_indices = assign_pad_slots_with_origin(
            [lane_by_terminal_id[individual_items[item_index][0].id][0] for item_index in order],
            obstacle_map,
            config,
        )
        individual_indices = [0] * len(individual_items)
        for position, item_index in enumerate(order):
            individual_indices[item_index] = sorted_indices[position]
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
            _pad_channel_track_count(
                individual_items,
                lane_by_terminal_id,
                individual_indices,
                origin_x,
                obstacle_map,
                config,
            ),
            obstacle_map,
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
        strict=True,
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


def common_bus_pad_slot_gap(config: ElectricalRoutingConfig) -> int:
    """Return the empty slots required between the last individual pad and the bus pad.

    2026-09-28 (eight-row-bundles plan, Rule B): the bus pad's escape is
    realized at ``bus_width_um``, centered on the bus pad slot -- wider than
    the bus pad's own bondpad rectangle by construction (see
    ``common_bus_escape_rects``). One default ``pad_empty_slots_between_
    assignments + 1`` gap (one pad pitch) is not always enough for that
    band to clear the neighbouring individual pad's own
    ``bondpad_width_um``-wide metal by ``obstacle_clearance_um``: two slots
    ``pad_pitch_um`` apart already give ``pad_pitch_um`` of center-to-center
    separation, so the extra slots beyond that one required is
    ``ceil((bus_width_um / 2 + bondpad_width_um / 2 + obstacle_clearance_um
    - pad_pitch_um) / pad_pitch_um)``, at least 0. With the defaults (bus
    400, individual pad 80, clearance 10, pitch 130) that is
    ceil((200 + 40 + 10 - 130) / 130) = 1 extra empty slot.
    """

    required_um = (
        config.bus_width_um / 2.0
        + config.bondpad_width_um / 2.0
        + config.obstacle_clearance_um
        - config.pad_pitch_um
    )
    extra_slots = max(0, math.ceil(required_um / config.pad_pitch_um))
    return config.pad_empty_slots_between_assignments + 1 + extra_slots


def _common_bus_pad_slot_index(
    origin_x: float,
    individual_indices: list[int],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    gap = common_bus_pad_slot_gap(config)
    layout_xmin, _, layout_xmax, _ = obstacle_map.layout_bbox
    if config.common_bus_pad_position == "left":
        side_x = layout_xmin - max(
            config.bus_x_margin_um,
            config.common_bus_bondpad_width_um,
        )
        side_index = math.floor((side_x - origin_x) / config.pad_pitch_um)
        next_sequence_index = min(individual_indices, default=0) - gap
        return min(side_index, next_sequence_index)

    side_x = layout_xmax + max(
        config.bus_x_margin_um,
        config.common_bus_bondpad_width_um,
    )
    side_index = math.ceil((side_x - origin_x) / config.pad_pitch_um)
    next_sequence_index = max(individual_indices, default=-1) + gap
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
    track_count: int,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> float:
    """Return the channel height ``track_count`` tracks need between the layout and the pads.

    One track pitch per track, plus the room the lowest track's reservation
    needs above the layout's top row: the channel router checks a wire's
    centerline dilated by ``wire_reservation_radius_cells`` (wire width plus
    obstacle clearance, rounded up to cells) against the obstacle map, so
    track 1 must lie ``radius + 1`` rows above the layout's top row or a
    run on it that passes over a heater column touches the column's top
    cells (measured on the multi-heater fixture at a 20 um grid, pad-choice
    plan 2026-10-05: with pads on both sides of a bundle's lanes such runs
    are the rule, not the exception). In physical terms this is one wire
    width plus one cell of rounding, where it used to be one wire width.
    """

    if track_count <= 0:
        return config.pad_offset_um
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    radius_cells = wire_reservation_radius_cells(obstacle_map, config)
    return track_count * track_pitch_um + (radius_cells + 1) * obstacle_map.grid.grid_size_um


def _lane_by_terminal_id(
    individual_items: tuple[tuple[ElectricalTerminal, int | None, int | None], ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    escape_topology: IndividualEscapeTopologyResult | None,
) -> dict[str, tuple[int, int]]:
    """Return every individual terminal's (corridor lane column, row) in grid cells.

    The lane is the column the channel router will rise on for that
    terminal (``river_routing.corridor_lane_column`` of its bundle's
    opening edge, exit side and row; row 1 the terminal nearest the pad
    row: the first of a "right" bundle's ordered terminals, the last of a
    "left" one's, mirroring ``bundle_detail_router``'s member order). A
    terminal without a bundle, or whose bundle has no opening edge, takes
    its own port-access anchor column (or its center's column) with row 0,
    meaning "no lane".
    """

    grid = obstacle_map.grid
    pitch_cells = track_pitch_in_cells(obstacle_map, config)
    clearance_cells = clearance_in_cells(obstacle_map, config)
    lanes: dict[str, tuple[int, int]] = {}
    bundles = escape_topology.bundles if escape_topology is not None else ()
    for bundle in bundles:
        exit_dx = -1 if bundle_route_side(bundle, obstacle_map) == "left" else 1
        terminal_ids = [terminal.id for terminal in bundle.ordered_terminals]
        opening_edge = bundle_opening_edge(obstacle_map, terminal_ids, exit_dx)
        if opening_edge is None:
            continue
        row_count = len(terminal_ids)
        for rank, terminal_id in enumerate(terminal_ids):
            row = rank + 1 if exit_dx > 0 else row_count - rank
            lanes[terminal_id] = (
                corridor_lane_column(opening_edge, exit_dx, row, pitch_cells, clearance_cells),
                row,
            )
    for terminal, _bundle_id, _rank in individual_items:
        if terminal.id in lanes:
            continue
        access = obstacle_map.individual_port_accesses.get(terminal.id)
        if access is not None:
            column = access.anchor_cell[0]
        else:
            column = int((terminal.center[0] - grid.origin[0]) // grid.grid_size_um)
        lanes[terminal.id] = (column, 0)
    return lanes


def _pad_channel_track_count(
    individual_items: tuple[tuple[ElectricalTerminal, int | None, int | None], ...],
    lane_by_terminal_id: dict[str, tuple[int, int]],
    individual_indices: list[int],
    origin_x: float,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    """Return the number of tracks the channel router will use, for the channel's height.

    2026-09-29 (channel routing plan, second slice): the channel used to be
    sized from the widest bundle, but the joint track assignment can need
    more tracks than any one bundle has rows (cascade_12_folded, before the
    pad-choice plan: a lone heater whose corridor lane lay within a pitch
    of a neighbouring bundle's outermost pad column had to sit below that
    bundle's six nested rows -- seven tracks for a channel of six, and the
    lowest track ran inside the layout into an obstacle). So the same pure
    ``assign_tracks`` runs here, on the same inputs the router will build:
    one ``ChannelWire`` per terminal with a lane (``_lane_by_terminal_id``)
    and its pad column from the slot's access cells (``pad_column_cell``,
    whose x does not depend on the pad row's height). Terminals without a
    lane count one track each as a floor (they have no lane to assign). A
    forced ``pad_origin_x_um`` keeps the old global count: its pads lie
    wherever the owner put them and most wires take the fallback.
    """

    if config.pad_origin_x_um is not None or not individual_items:
        return len(individual_indices)

    pitch_cells = track_pitch_in_cells(obstacle_map, config)
    wires: list[ChannelWire] = []
    laneless = 0
    for (terminal, _bundle_id, _rank), index in zip(
        individual_items, individual_indices, strict=True
    ):
        lane, row = lane_by_terminal_id[terminal.id]
        if row == 0:
            laneless += 1
            continue
        slot = _make_slot(index, origin_x, obstacle_map, config, config.pad_offset_um)
        target_cells = pad_access_cells(slot, obstacle_map, config)
        if not target_cells:
            continue
        wires.append(
            ChannelWire(
                wire_id=terminal.id,
                c=lane,
                p=pad_column_cell(target_cells, config)[0],
                row=row,
            )
        )
    placed, _unplaced = assign_tracks(wires, pitch_cells)
    return max(max(placed.values(), default=0), laneless)


def assign_pad_slots(lane_xs: list[int], slot_xs: list[int], gap_slots: int = 0) -> list[int]:
    """Assign one slot per wire, in lane order, minimizing the sum of the horizontal runs.

    ``lane_xs`` are the wires' corridor lanes in grid cells, ascending;
    ``slot_xs`` the candidate slot columns in grid cells, ascending, one
    per slot index; ``gap_slots`` the empty slots required between two
    assigned pads (consecutive chosen indices differ by at least
    ``gap_slots + 1``). Returns one slot index per wire, strictly
    increasing, minimizing the sum of ``abs(lane - slot_xs[index])``.

    The order is forced, not preferred (pad-choice plan, 2026-10-05): two
    wires whose pad order differs from their lane order always have
    overlapping runs on the one metal layer, and every such overlap is a
    constraint cycle of ``river_routing.assign_tracks``. The minimum is
    exact, by dynamic programming over (wire, slot): ``best[i][k]`` is the
    least cost with wire ``i`` on slot ``k``, the inner minimum over the
    earlier wire's slots ``j <= k - gap - 1`` kept as a running minimum,
    so the table costs wires times slots. Raises ``ValueError`` when the
    slots cannot hold the wires with their gaps.
    """

    if gap_slots < 0:
        raise ValueError("gap_slots must be non-negative")
    wire_count = len(lane_xs)
    slot_count = len(slot_xs)
    if wire_count == 0:
        return []
    stride = gap_slots + 1
    if slot_count < (wire_count - 1) * stride + 1:
        raise ValueError(
            f"{slot_count} slots cannot hold {wire_count} wires "
            f"with {gap_slots} empty slots between"
        )
    if any(a > b for a, b in pairwise(lane_xs)):
        raise ValueError("lane_xs must be ascending")

    infinity = float("inf")
    best = [abs(lane_xs[0] - slot_x) for slot_x in slot_xs]
    came_from: list[list[int]] = []
    for lane_x in lane_xs[1:]:
        previous = best
        best = [infinity] * slot_count
        back = [-1] * slot_count
        running_min = infinity
        running_arg = -1
        for k in range(slot_count):
            j = k - stride
            if j >= 0 and previous[j] < running_min:
                running_min = previous[j]
                running_arg = j
            if running_min < infinity:
                best[k] = running_min + abs(lane_x - slot_xs[k])
                back[k] = running_arg
        came_from.append(back)

    k = min(range(slot_count), key=lambda index: (best[index], index))
    indices = [k]
    for back in reversed(came_from):
        k = back[k]
        indices.append(k)
    indices.reverse()
    return indices


def assign_pad_slots_with_origin(
    lane_xs: list[int],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[float, list[int]]:
    """Choose the pad row's origin and one slot per wire by the minimum run sum.

    ``lane_xs`` ascending, in grid cells. For every origin offset within
    one pad pitch at the grid resolution, the candidate slots are laid out
    to cover every lane with room for every wire on either side
    (``wire_count + extra + 2`` pitches beyond the outermost lanes: the
    prototype that lacked this room squeezed every pad to one end), each
    slot's column taken from the real slot geometry (``_make_slot`` ->
    ``pad_access_cells`` -> ``pad_column_cell``, the chain the channel
    router reads; a pitch that is not a whole number of cells, such as
    150 um on a 20 um grid, drifts by a cell every other slot, which a
    rounded pitch in cells would miss by 400 um over forty slots), the
    assignment is computed (``assign_pad_slots``), and the best run sum
    wins (ties: the smallest offset). The indices are then shifted so the
    first assigned index is ``config.pad_extra_slots_left`` (the pad
    plan's convention), which only moves the origin. Returns
    ``(origin_x_um, indices)``; ``indices`` are in the order of
    ``lane_xs``.
    """

    grid = obstacle_map.grid
    grid_size = grid.grid_size_um
    if not lane_xs:
        return grid.origin[0], []

    pitch_um = config.pad_pitch_um
    gap_slots = config.pad_empty_slots_between_assignments
    first_assignment_index = config.pad_extra_slots_left
    wire_count = len(lane_xs)
    margin_left = (wire_count + config.pad_extra_slots_left + 2) * (gap_slots + 1)
    margin_right = (wire_count + config.pad_extra_slots_right + 2) * (gap_slots + 1)
    lowest_x_um = grid.origin[0] + lane_xs[0] * grid_size - margin_left * pitch_um
    highest_x_um = grid.origin[0] + (lane_xs[-1] + 1) * grid_size + margin_right * pitch_um
    slot_count = int((highest_x_um - lowest_x_um) // pitch_um) + 1
    offset_count = max(1, math.ceil(pitch_um / grid_size))

    def slot_column(origin_x_um: float, index: int) -> int:
        slot = _make_slot(index, origin_x_um, obstacle_map, config, config.pad_offset_um)
        target_cells = pad_access_cells(slot, obstacle_map, config)
        if target_cells:
            return pad_column_cell(target_cells, config)[0]
        # a candidate slot outside the grid (the margins reach beyond it on a
        # small layout): its geometric column keeps the candidates monotone,
        # and no wire is assigned to it unless every nearer slot costs more
        return int((slot.center[0] - grid.origin[0]) // grid_size)

    best: tuple[float, int, float, list[int]] | None = None
    for offset_index in range(offset_count):
        origin_x_um = lowest_x_um + offset_index * grid_size
        slot_xs = [slot_column(origin_x_um, k) for k in range(slot_count)]
        indices = assign_pad_slots(lane_xs, slot_xs, gap_slots)
        run_sum = float(
            sum(abs(lane - slot_xs[k]) for lane, k in zip(lane_xs, indices, strict=True))
        )
        if best is None or run_sum < best[0]:
            best = (run_sum, offset_index, origin_x_um, indices)
    assert best is not None
    _, _, origin_x_um, indices = best
    shift = indices[0] - first_assignment_index
    return origin_x_um + shift * pitch_um, [k - shift for k in indices]


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
