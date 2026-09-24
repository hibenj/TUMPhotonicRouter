"""Phase 4: the crossing plan and the port footprints."""

from __future__ import annotations

import math

from typing import Any, Iterable

from gdsfactory.typings import Port

from photonic_router.static_obstacle_builder import physical_to_grid

from translation.crossing_modes import is_collision_mode
from translation.route_rust_crossing_components import _bbox_bounds_um
from translation.route_rust_crossing_plan import _build_crossing_plan_info, _port_center_um
from translation.route_rust_obstacle_config import _port_type_name
from translation.route_rust_types import RouteJob

from translation.routing.route_jobs import (
    _angle_to_step,
    _dense_fanout_min_ports,
    _fanout_stub_bend_steps,
    _grid_cell_center_um,
    _in_bounds,
    _is_dense_source_fanout_instance,
    _orientation_to_angle,
    _port_access_rule_for,
    port_to_grid_state,
)
from translation.routing import timing
from translation.routing.settings import SessionSettings
from translation.routing.stages import PlannedJobs
from translation.routing.state import SessionState

def _rect_ranges_by_y(
    settings,
    state,
    rects: Iterable[tuple[int, int, int, int]],
) -> dict[int, list[tuple[int, int]]]:
    ranges_by_y: dict[int, list[tuple[int, int]]] = {}
    for rect_min_x, rect_min_y, rect_max_x, rect_max_y in rects:
        min_x = max(0, rect_min_x)
        max_x = min(state.grid_width - 1, rect_max_x)
        min_y = max(0, rect_min_y)
        max_y = min(state.grid_height - 1, rect_max_y)
        if min_x > max_x or min_y > max_y:
            continue
        for y in range(min_y, max_y + 1):
            ranges_by_y.setdefault(y, []).append((min_x, max_x))
    for y, ranges in list(ranges_by_y.items()):
        ranges.sort()
        merged_ranges: list[tuple[int, int]] = []
        for min_x, max_x in ranges:
            if not merged_ranges or min_x > merged_ranges[-1][1] + 1:
                merged_ranges.append((min_x, max_x))
            else:
                prev_min_x, prev_max_x = merged_ranges[-1]
                merged_ranges[-1] = (prev_min_x, max(prev_max_x, max_x))
        ranges_by_y[y] = merged_ranges
    return ranges_by_y


def _raw_static_rect_ranges_by_y(settings, state) -> dict[int, list[tuple[int, int]]]:
    if state.raw_static_rect_ranges_by_y is not None:
        return state.raw_static_rect_ranges_by_y
    ranges_by_y = _rect_ranges_by_y(settings, state, state.raw_static_rects_for_openings)
    state.raw_static_rect_ranges_by_y = ranges_by_y
    return ranges_by_y


def _heater_opening_rect_ranges_by_y(settings, state) -> dict[int, list[tuple[int, int]]]:
    if state.heater_opening_rect_ranges_by_y is not None:
        return state.heater_opening_rect_ranges_by_y
    ranges_by_y = _rect_ranges_by_y(settings, state, state.heater_opening_rects_for_openings)
    state.heater_opening_rect_ranges_by_y = ranges_by_y
    return ranges_by_y


def _raw_static_cells_by_y(settings, state) -> dict[int, set[int]]:
    if state.raw_static_cells_by_y is not None:
        return state.raw_static_cells_by_y
    cells_by_y: dict[int, set[int]] = {}
    for cell_x, cell_y in state.raw_static_cells:
        cells_by_y.setdefault(int(cell_y), set()).add(int(cell_x))
    state.raw_static_cells_by_y = cells_by_y
    return cells_by_y


def _cell_in_raw_static(settings, state, cell: tuple[int, int]) -> bool:
    if cell in state.raw_static_cells:
        return True
    x, y = cell
    return any(
        rect_min_x <= x <= rect_max_x
        for rect_min_x, rect_max_x in _raw_static_rect_ranges_by_y(settings, state).get(y, ())
    )


def _cells_in_raw_static_geometry(
    settings,
    state,
    cells: set[tuple[int, int]],
) -> set[tuple[int, int]]:
    return {cell for cell in cells if _cell_in_raw_static(settings, state, cell)}


def _resolve_port_footprint_cells(
    settings,
    state,
    *,
    instance_name: str,
    port_name: str,
    port: object,
) -> tuple[int, int]:
    """Return (length_cells, half_width_cells) sizing this port's access/keepout region."""
    rule = _port_access_rule_for(settings, state,
        instance_name=instance_name,
        port_name=port_name,
        port=port,
    )
    if rule is not None:
        grid_size = float(state.grid.grid_size_um)
        length_cells = max(
            1,
            int(math.ceil(max(0.0, float(rule.access_length_um)) / grid_size)),
        )
        half_width_cells = max(
            0,
            int(math.ceil((max(0.0, float(rule.access_width_um)) / 2.0) / grid_size)),
        )
        return length_cells, half_width_cells

    if _is_dense_source_fanout_instance(settings, state, instance_name):
        return int(state.stub_port_lane_length_cells), int(state.stub_port_lane_half_width_cells)
    # A dense TARGET port with a real, pre-committed static stub
    # (`_build_static_fanout_target_anchors`) is in exactly the same
    # position as a dense source port with one: the stub's own committed
    # waveguide already protects the approach, so the generic port-lane
    # reservation below would only be redundant. Reuse the same
    # stub-scoped knobs (still real, still overridable via the same
    # `PHOTONIC_ROUTER_STUB_PORT_LANE_*` environment variables) rather
    # than introducing separate target-only ones -- see
    # `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
    if f"{instance_name},{port_name}" in getattr(state, "fanout_anchor_by_port_spec", {}):
        return int(state.stub_port_lane_length_cells), int(state.stub_port_lane_half_width_cells)
    return int(state.port_lane_length_cells), int(state.port_lane_half_width_cells)


def _instance_ref_by_name(settings, state, instance_name: str) -> Any | None:
    try:
        instances = state.routed_layout.insts
    except (AttributeError, TypeError):
        return None
    for instance in instances:
        if getattr(instance, "name", None) == instance_name:
            return instance
    return None


def _instance_static_geometry_open_cells(
    settings,
    state,
    *,
    instance_name: str,
    center_um: tuple[float, float],
    orientation: float | None,
) -> set[tuple[int, int]]:
    """Static cells of the port's own instance to open on the port-facing side.

    Only called for ports whose access rule declares
    ``opens_instance_static_geometry`` (today: heater optical ports, whose
    metal pad sits on the port-facing side and would otherwise block the
    access runway). The margin covers the clearance-expanded blocked
    rectangles as well, so the opening matches what the obstacle map
    actually blocks. Never call this on the strength of a rule merely
    existing: the pre-placed crossing grids register a zero-size runway
    rule, and opening their interior let a net hook through the grid
    (``benes_16x16`` grid mode, route 74, 2026-08-27).
    """
    if orientation is None:
        return set()
    ref = _instance_ref_by_name(settings, state, instance_name)
    if ref is None:
        return set()
    bounds = _bbox_bounds_um(ref)
    if bounds is None:
        return set()
    left, bottom, right, top = bounds
    grid_size = float(state.grid.grid_size_um)
    if grid_size <= 0.0:
        return set()

    angle_rad = math.radians(float(orientation))
    dir_x = math.cos(angle_rad)
    dir_y = math.sin(angle_rad)
    if not math.isfinite(dir_x) or not math.isfinite(dir_y):
        return set()

    bbox_margin = 0.5 * grid_size
    min_bbox_cell = physical_to_grid(
        float(left) - bbox_margin,
        float(bottom) - bbox_margin,
        state.grid,
    )
    max_bbox_cell = physical_to_grid(
        float(right) + bbox_margin,
        float(top) + bbox_margin,
        state.grid,
    )
    min_x = max(
        0,
        min_bbox_cell[0],
    )
    max_x = min(
        state.grid_width - 1,
        max_bbox_cell[0],
    )
    min_y = max(
        0,
        min_bbox_cell[1],
    )
    max_y = min(
        state.grid_height - 1,
        max_bbox_cell[1],
    )
    if min_x > max_x or min_y > max_y:
        return set()

    heater_clearance_um = getattr(state.resolved_obstacle_config, "heater_clearance_um", None)
    opening_margin_um = max(
        float(state.route_clearance_um),
        0.0 if heater_clearance_um is None else float(heater_clearance_um),
    )
    opening_margin_cells = int(math.ceil(opening_margin_um / grid_size)) + 1
    search_min_x = max(0, min_x - opening_margin_cells)
    search_max_x = min(state.grid_width - 1, max_x + opening_margin_cells)
    search_min_y = max(0, min_y - opening_margin_cells)
    search_max_y = min(state.grid_height - 1, max_y + opening_margin_cells)
    opening_margin_distance = float(opening_margin_cells) * grid_size + bbox_margin
    opening_left = float(left) - opening_margin_distance
    opening_right = float(right) + opening_margin_distance
    opening_bottom = float(bottom) - opening_margin_distance
    opening_top = float(top) + opening_margin_distance

    candidate_cells: set[tuple[int, int]] = set()
    explicit_cells_by_y = _raw_static_cells_by_y(settings, state)
    heater_rect_ranges_by_y = _heater_opening_rect_ranges_by_y(settings, state)
    for cell_y in range(search_min_y, search_max_y + 1):
        xs: set[int] = set()
        xs.update(
            cell_x
            for cell_x in explicit_cells_by_y.get(cell_y, set())
            if search_min_x <= int(cell_x) <= search_max_x
        )
        for rect_min_x, rect_max_x in heater_rect_ranges_by_y.get(cell_y, ()):
            start_x = max(search_min_x, int(rect_min_x))
            end_x = min(search_max_x, int(rect_max_x))
            if start_x <= end_x:
                xs.update(range(start_x, end_x + 1))
        for cell_x in xs:
            inside_instance_bbox = (
                min_x <= int(cell_x) <= max_x and min_y <= int(cell_y) <= max_y
            )
            cell_center = _grid_cell_center_um(settings, state, int(cell_x), int(cell_y))
            if (
                opening_left <= cell_center[0] <= opening_right
                and opening_bottom <= cell_center[1] <= opening_top
            ):
                candidate_cells.add((int(cell_x), int(cell_y)))
                continue
            if inside_instance_bbox:
                if cell_center[0] < left - bbox_margin or cell_center[0] > right + bbox_margin:
                    continue
                if cell_center[1] < bottom - bbox_margin or cell_center[1] > top + bbox_margin:
                    continue
            outward_distance = (cell_center[0] - float(center_um[0])) * dir_x + (
                cell_center[1] - float(center_um[1])
            ) * dir_y
            if outward_distance >= -bbox_margin:
                candidate_cells.add((int(cell_x), int(cell_y)))

    return candidate_cells


def _endpoint_state_for_lane_assignment(settings, state, port: Port, *, as_target: bool):
    return port_to_grid_state(settings, state,
        port,
        state.origin_x_um,
        state.origin_y_um,
        float(state.grid.grid_size_um),
        as_target=as_target,
    )


def build_crossing_plan_and_port_footprints(
    settings: SessionSettings,
    state: SessionState,
    route_jobs: list[RouteJob],
    endpoint_port_specs_by_instance: dict[str, set[str]],
    dense_port_runway_length_by_spec: dict[str, int],
    obstacle_map: Any,
    crossing_device_info: dict[str, Any],
) -> PlannedJobs:
    """Build `self.crossing_plan_info`, batch port-opening/footprint and foreign-keepout cells, and assign dense-port lane geometry.

    Enables collision-crossing routing on `self.router` if configured. Populates
    `self.port_access_cells_by_spec`/`self.port_access_candidate_cells_by_spec`/
    `self.port_runway_cells_by_spec` (one raw footprint per port, from the single
    unified sizing computation), `self.foreign_port_keepout_cells_by_spec`,
    `self.dense_port_lateral_windows`/`self.dense_port_lateral_owner_groups` (lateral
    room allocated to each port in a dense multi-port group sharing a facing angle),
    and `self.port_state_lane_offsets` (grid-lane spreading for endpoints that would
    otherwise share the exact same source/target cell and angle). Returns `route_jobs`
    (unchanged by this phase, passed straight through) and
    `foreign_port_keepout_cells_by_instance`, which the next phase needs to build the
    static-cell handoff to Rust.
    """
    port_rule_extra_open_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    t_crossing_context_start = timing.pipeline_timer_start(settings, state)
    state.crossing_plan_info = _build_crossing_plan_info(
        rust_backend=state.rust_backend,
        router=state.router,
        schematic=settings.schematic,
        route_jobs=route_jobs,
        enable_crossings=settings.enable_crossings,
        crossing_mode=settings.crossing_mode,
        node_depths=settings.node_depths,
        node_ranks=settings.node_ranks,
        edge_ranks=settings.edge_ranks,
        crossing_loss=float(settings.crossing_loss),
        crossing_search_loss=float(settings.crossing_search_loss),
        crossing_half_size_cells=int(state.resolved_crossing_half_size_cells),
        min_straight_cells_per_crossing=int(settings.min_straight_cells_per_crossing),
        allow_only_expected_crossings=settings.effective_allow_only_expected_crossings,
        guidance_net_names=settings.crossing_guidance_net_names,
        config=settings.config.crossing_plan,
    )
    state.crossing_plan_info["crossing_mode"] = settings.crossing_mode
    if bool(settings.enable_crossings):
        # The exact configuration must be visible in stdout (harness step 0):
        # which of baseline / contribution 1 ran, and with which prices.
        guidance = state.crossing_plan_info.get("guidance")
        guidance_text = (
            f" planned_pairs={guidance['planned_pair_count']}"
            f" planned_loss={guidance['planned_crossing_loss']:.1f}"
            f" budget={'1/pair' if guidance.get('single_discounted_crossing_per_pair', True) else 'unlimited'}"
            if isinstance(guidance, dict)
            else ""
        )
        print(
            f"      - crossing search: mode={settings.crossing_mode}"
            f" search_loss={float(state.crossing_plan_info.get('crossing_search_loss', 0.0)):.1f}"
            f"{guidance_text}"
        )
    state.crossing_plan_info["requested_allow_only_expected_crossings"] = bool(
        settings.allow_only_expected_crossings
    )
    state.crossing_plan_info["bend_runout_cells_per_crossing"] = int(state.bend_radius_cells)
    state.crossing_plan_info["fanout_stub_bend_degrees"] = 45 * int(
        _fanout_stub_bend_steps(settings, state)
    )
    state.crossing_plan_info["required_straight_margin_cells_per_crossing"] = int(
        state.resolved_crossing_half_size_cells
    ) + int(state.bend_radius_cells)
    state.crossing_plan_info["fanout_access_mode"] = (
        settings.fanout_access_mode_normalized
    )
    state.crossing_plan_info["fanout_anchor_port_count"] = len(state.fanout_anchor_by_port_spec)
    state.crossing_plan_info["fanout_anchor_net_ids"] = sorted(state.fanout_anchor_net_ids)
    state.crossing_plan_info["fanout_anchor_source_net_ids"] = sorted(
        state.fanout_anchor_source_net_ids
    )
    state.crossing_plan_info["fanout_anchor_target_net_ids"] = sorted(
        state.fanout_anchor_target_net_ids
    )
    state.crossing_plan_info["fanout_stub_center_cell_count"] = len(
        state.fanout_stub_center_cells
    )
    state.crossing_plan_info["fanout_stub_static_cell_count"] = len(
        state.fanout_stub_static_cells
    )
    state.crossing_plan_info["fanout_stub_centerlines_um"] = [
        {
            "port_spec": anchor.port_spec,
            "anchor_cell": [int(anchor.state_x), int(anchor.state_y)],
            "physical_angle": int(anchor.physical_angle) % 8,
            "centerline_um": [
                [float(point[0]), float(point[1])] for point in anchor.stub_centerline_um
            ],
        }
        for anchor in sorted(
            state.fanout_anchor_by_port_spec.values(),
            key=lambda item: item.port_spec,
        )
    ]
    state.crossing_plan_info["crossing_device"] = crossing_device_info
    if bool(settings.enable_crossings) and is_collision_mode(
        settings.crossing_mode
    ):
        if not hasattr(state.router, "set_collision_crossing_routing"):
            extension_path = getattr(state.rust_backend, "__file__", "<unknown>")
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.set_collision_crossing_routing. Rebuild it with "
                "`maturin develop --release`. "
                f"Loaded extension: {extension_path}"
            )
        state.router.set_collision_crossing_routing(True)
    elif hasattr(state.router, "set_collision_crossing_routing"):
        state.router.set_collision_crossing_routing(False)
    timing.record_pipeline_timing(settings, state, "crossing_context", t_crossing_context_start)

    if not hasattr(state.router, "build_port_footprint_cells"):
        extension_path = getattr(state.rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.build_port_footprint_cells. Rebuild it with "
            "`maturin develop --release`. "
            f"Loaded extension: {extension_path}"
        )

    t_port_opening_prep_start = timing.pipeline_timer_start(settings, state)
    port_opening_inputs: list[
        tuple[str, float, float, float | None, str | None, float | None, float | None]
    ] = []
    for port_spec, (instance_name, port_name, port) in state.endpoint_ports_by_spec.items():
        fanout_anchor = state.fanout_anchor_by_port_spec.get(port_spec)
        center = fanout_anchor.center_um if fanout_anchor is not None else _port_center_um(port)
        if center is None:
            raise ValueError(f"Port {port_spec!r} has no finite center coordinate")
        orientation_value = getattr(port, "orientation", None)
        orientation = None if orientation_value is None else float(orientation_value)
        port_type = _port_type_name(port)
        rule = _port_access_rule_for(settings, state,
            instance_name=instance_name,
            port_name=port_name,
            port=port,
        )
        access_length_um = None if rule is None else float(rule.access_length_um)
        access_width_um = None if rule is None else float(rule.access_width_um)
        state.port_access_rule_by_spec[port_spec] = (
            None if rule is None else rule.component_name_pattern
        )
        port_rule_extra_open_cells_by_spec[port_spec] = (
            _instance_static_geometry_open_cells(settings, state,
                instance_name=instance_name,
                center_um=(float(center[0]), float(center[1])),
                orientation=orientation,
            )
            if rule is not None and rule.opens_instance_static_geometry
            else set()
        )
        port_opening_inputs.append(
            (
                port_spec,
                float(center[0]),
                float(center[1]),
                orientation,
                port_type,
                access_length_um,
                access_width_um,
            )
        )
    timing.record_pipeline_timing(settings, state, "port_opening_prep", t_port_opening_prep_start)

    t_port_opening_batch_start = timing.pipeline_timer_start(settings, state)

    # One raw footprint per port, from the single unified sizing computation
    # (Milestone 1's _resolve_port_footprint_cells plus the dense-runway
    # length override, matching the precedence the old code already used:
    # an explicit custom access rule wins over a dense-runway override).
    # Self-opening and foreign-keepout both derive from this same dict
    # below, so they can never drift out of sync with each other again --
    # see .agent/execplans/2026-08-18-unify-port-access-region-computation.md.
    raw_footprint_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_footprint_inputs: list[tuple[str, float, float, float | None, int, int]] = []
    for item in port_opening_inputs:
        port_spec = str(item[0])
        raw_footprint_cells_by_spec[port_spec] = set()
        _spec, x_um, y_um, orientation, port_type, access_length_um, access_width_um = item
        instance_name, port_name, port = state.endpoint_ports_by_spec[port_spec]
        custom_access = access_length_um is not None or access_width_um is not None
        if not custom_access and port_type is not None and str(port_type) != "optical":
            continue
        length_cells, half_width_cells = _resolve_port_footprint_cells(settings, state,
            instance_name=instance_name,
            port_name=port_name,
            port=port,
        )
        custom_runway_length = dense_port_runway_length_by_spec.get(port_spec)
        if custom_runway_length is not None and not custom_access:
            length_cells = max(1, int(custom_runway_length))
        port_footprint_inputs.append(
            (
                port_spec,
                float(x_um),
                float(y_um),
                orientation,
                int(length_cells),
                int(half_width_cells),
            )
        )

    for port_spec, raw_cells in state.router.build_port_footprint_cells(port_footprint_inputs):
        raw_footprint_cells_by_spec[str(port_spec)] = {
            (int(cell[0]), int(cell[1])) for cell in raw_cells
        }

    for port_spec, raw_footprint_cells in raw_footprint_cells_by_spec.items():
        open_cells = raw_footprint_cells - _cells_in_raw_static_geometry(settings, state,
            raw_footprint_cells
        )
        state.port_access_cells_by_spec[port_spec] = set(open_cells)
        state.port_access_candidate_cells_by_spec[port_spec] = set(raw_footprint_cells)
        extra_open_cells = port_rule_extra_open_cells_by_spec.get(port_spec, set())
        if extra_open_cells:
            state.port_access_cells_by_spec[port_spec].update(extra_open_cells)
            state.port_access_candidate_cells_by_spec[port_spec].update(extra_open_cells)
        state.port_runway_cells_by_spec[port_spec] = set(raw_footprint_cells)
    timing.record_pipeline_timing(settings, state, "port_opening_batch", t_port_opening_batch_start)

    # Foreign-keepout reuses the exact same raw footprint each port already
    # got for its own opening above -- a keepout region is the same shape,
    # just unfiltered (no must-stay-blocked subtraction), since it exists to
    # say "nothing else may enter here," not to describe what this port's
    # own net may route through. foreign_port_keepout_cells now only gates
    # whether keepout logic runs at all; it no longer controls size.
    state.foreign_port_keepout_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
    foreign_port_keepout_nonstatic_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
    if settings.foreign_port_keepout_cells > 0:
        t_foreign_keepout_start = timing.pipeline_timer_start(settings, state)
        for port_spec, raw_footprint_cells in raw_footprint_cells_by_spec.items():
            instance_name = port_spec.split(",", 1)[0]
            cells_for_spec = set(raw_footprint_cells)
            state.foreign_port_keepout_cells_by_spec[port_spec] = cells_for_spec
            foreign_port_keepout_cells_by_instance.setdefault(instance_name, set()).update(
                cells_for_spec
            )
            nonstatic_cells_for_spec = cells_for_spec - _cells_in_raw_static_geometry(
                settings, state,
                cells_for_spec
            )
            foreign_port_keepout_nonstatic_cells_by_instance.setdefault(
                instance_name,
                set(),
            ).update(nonstatic_cells_for_spec)
        timing.record_pipeline_timing(settings, state, "foreign_port_keepout_batch", t_foreign_keepout_start)

    state.dense_port_lateral_windows: dict[str, tuple[float, float, float, float, float]] = {}
    state.dense_port_lateral_owner_groups: dict[
        str,
        tuple[float, float, tuple[tuple[str, float], ...]],
    ] = {}
    for instance_name, port_specs in endpoint_port_specs_by_instance.items():
        if len(port_specs) < _dense_fanout_min_ports(settings, state):
            continue
        groups: dict[int, list[tuple[str, float]]] = {}
        for port_spec in port_specs:
            _inst, _port_name, port = state.endpoint_ports_by_spec[port_spec]
            fanout_anchor = state.fanout_anchor_by_port_spec.get(port_spec)
            angle = (
                int(fanout_anchor.physical_angle) % 8
                if fanout_anchor is not None
                else _orientation_to_angle(
                    settings, state, getattr(port, "orientation", None), flip=False
                )
            )
            step_x, step_y = _angle_to_step(settings, state, angle)
            lateral_x, lateral_y = -step_y, step_x
            center = (
                fanout_anchor.center_um if fanout_anchor is not None else _port_center_um(port)
            )
            if center is None or (lateral_x == 0 and lateral_y == 0):
                continue
            lateral_position = float(center[0]) * lateral_x + float(center[1]) * lateral_y
            groups.setdefault(angle, []).append((port_spec, lateral_position))
        for angle, group in groups.items():
            if len(group) <= 1:
                continue
            step_x, step_y = _angle_to_step(settings, state, angle)
            lateral_x, lateral_y = -step_y, step_x
            ordered = sorted(group, key=lambda item: item[1])
            owner_group = tuple(ordered)
            for owned_port_spec, _lateral_position in ordered:
                state.dense_port_lateral_owner_groups[owned_port_spec] = (
                    float(lateral_x),
                    float(lateral_y),
                    owner_group,
                )
            for index, (port_spec, lateral_position) in enumerate(ordered):
                previous_position = ordered[index - 1][1] if index > 0 else None
                next_position = ordered[index + 1][1] if index + 1 < len(ordered) else None
                if previous_position is None and next_position is None:
                    continue
                if previous_position is None:
                    gap = abs(next_position - lateral_position)
                    lower = lateral_position - gap * 0.5
                else:
                    lower = (previous_position + lateral_position) * 0.5
                if next_position is None:
                    gap = abs(lateral_position - previous_position)
                    upper = lateral_position + gap * 0.5
                else:
                    upper = (lateral_position + next_position) * 0.5
                lane_margin_um = 0.0
                state.dense_port_lateral_windows[port_spec] = (
                    float(lateral_x),
                    float(lateral_y),
                    float(lower),
                    float(upper),
                    lane_margin_um,
                )

    state.normal_port_runway_cells: set[tuple[int, int]] = set()
    for cells in state.port_runway_cells_by_spec.values():
        state.normal_port_runway_cells.update(cells)

    endpoint_ports_by_key: dict[tuple[int, int, int], list[tuple[str, bool, Port]]] = {}
    for job in route_jobs:
        for port_spec, port, as_target in (
            (f"{job.inst1},{job.port1}", job.source_port, False),
            (f"{job.inst2},{job.port2}", job.target_port, True),
        ):
            grid_state = _endpoint_state_for_lane_assignment(
                settings, state, port, as_target=as_target
            )
            key = (int(grid_state.x), int(grid_state.y), int(grid_state.angle) % 8)
            endpoint_ports_by_key.setdefault(key, []).append((port_spec, as_target, port))

    state.port_state_lane_offsets: dict[tuple[str, bool], tuple[int, int]] = {}
    for (base_x, base_y, angle), endpoints in endpoint_ports_by_key.items():
        unique_endpoints = list(
            dict.fromkeys((spec, is_target) for spec, is_target, _ in endpoints)
        )
        if len(unique_endpoints) <= 1:
            continue
        step_x, step_y = _angle_to_step(settings, state, angle)
        lateral_x, lateral_y = -step_y, step_x
        if lateral_x == 0 and lateral_y == 0:
            continue

        def _lateral_position_for_lane_assignment(item: tuple[str, bool, Port]) -> float:
            center = _port_center_um(item[2])
            if center is None:
                return 0.0
            return center[0] * lateral_x + center[1] * lateral_y

        sorted_endpoints = sorted(endpoints, key=_lateral_position_for_lane_assignment)
        seen_endpoint_keys: set[tuple[str, bool]] = set()
        lane_index = 0
        for port_spec, is_target, _ in sorted_endpoints:
            endpoint_key = (port_spec, is_target)
            if endpoint_key in seen_endpoint_keys:
                continue
            seen_endpoint_keys.add(endpoint_key)
            candidate_x = base_x + lateral_x * lane_index
            candidate_y = base_y + lateral_y * lane_index
            if _in_bounds(settings, state, candidate_x, candidate_y):
                state.port_state_lane_offsets[endpoint_key] = (
                    lateral_x * lane_index,
                    lateral_y * lane_index,
                )
            lane_index += 1

    return PlannedJobs(route_jobs, foreign_port_keepout_cells_by_instance)
