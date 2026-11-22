"""Phase 4: the crossing plan and the port footprints."""

from __future__ import annotations

import math
from typing import Any, NamedTuple

from gdsfactory.typings import Port

from photonic_router.crossings.route_rust_crossing_components import bbox_bounds_um
from photonic_router.crossings.route_rust_crossing_plan import (
    CrossingPlanRequest,
    build_crossing_plan_info,
)
from photonic_router.records.crossing_plan_info import CrossingPlanInfo
from photonic_router.records.route_rust_obstacle_config import port_type_name
from photonic_router.records.route_rust_records import center_of_port_um
from photonic_router.records.route_rust_types import RouteJob
from photonic_router.routing import timing
from photonic_router.routing.products import PortFootprints
from photonic_router.routing.route_jobs import (
    angle_to_step,
    cell_in_grid,
    dense_fanout_min_ports,
    fanout_stub_bend_steps,
    is_dense_source_fanout_instance,
    orientation_to_angle,
    port_access_rule_for,
    port_to_grid_state,
    state_cell_center_um,
)
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.stages import PlannedJobs
from photonic_router.routing.state import SessionState
from photonic_router.static_obstacle_builder import physical_to_grid


def _cell_in_raw_static(state: SessionState, cell: tuple[int, int]) -> bool:
    if cell in state.kernel.static.raw_static_cells:
        return True
    x, y = cell
    return any(
        rect_min_x <= x <= rect_max_x
        for rect_min_x, rect_max_x in state.kernel.raw_static_rect_ranges_by_y.get(y, ())
    )


def cells_in_raw_static_geometry(
    state: SessionState,
    cells: set[tuple[int, int]],
) -> set[tuple[int, int]]:
    """Return those of ``cells`` that lie in the raw static obstacle geometry of the kernel state.

    A cell counts if it is one of the raw static cells or lies inside one of the raw
    static rectangles.
    """
    return {cell for cell in cells if _cell_in_raw_static(state, cell)}


def _resolve_port_footprint_cells(
    settings: SessionSettings,
    state: SessionState,
    *,
    instance_name: str,
    port_name: str,
    port: Port,
) -> tuple[int, int]:
    """Return (length_cells, half_width_cells) sizing this port's access/keepout region."""
    rule = port_access_rule_for(
        settings,
        state,
        instance_name=instance_name,
        port_name=port_name,
        port=port,
    )
    if rule is not None:
        grid_size = float(state.obstacles.grid.grid_size_um)
        length_cells = max(
            1,
            math.ceil(max(0.0, float(rule.access_length_um)) / grid_size),
        )
        half_width_cells = max(
            0,
            math.ceil((max(0.0, float(rule.access_width_um)) / 2.0) / grid_size),
        )
        return length_cells, half_width_cells

    if is_dense_source_fanout_instance(settings, state, instance_name):
        return int(state.kernel.lanes.stub_port_lane_length_cells), int(
            state.kernel.lanes.stub_port_lane_half_width_cells
        )
    # A dense TARGET port with a real, pre-committed static stub
    # (`_build_static_fanout_target_anchors`) is in exactly the same
    # position as a dense source port with one: the stub's own committed
    # waveguide already protects the approach, so the generic port-lane
    # reservation below would only be redundant. Reuse the same
    # stub-scoped knobs (still real, still overridable via the same
    # `PHOTONIC_ROUTER_STUB_PORT_LANE_*` environment variables) rather
    # than introducing separate target-only ones -- see
    # `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
    if f"{instance_name},{port_name}" in state.fanout.anchor_by_port_spec:
        return int(state.kernel.lanes.stub_port_lane_length_cells), int(
            state.kernel.lanes.stub_port_lane_half_width_cells
        )
    return int(state.kernel.lanes.port_lane_length_cells), int(
        state.kernel.lanes.port_lane_half_width_cells
    )


def _instance_ref_by_name(state: SessionState, instance_name: str) -> Any | None:
    try:
        instances = state.routed_layout.insts
    except (AttributeError, TypeError):
        return None
    for instance in instances:
        if getattr(instance, "name", None) == instance_name:
            return instance
    return None


def _static_columns_in_row(state: SessionState, cell_y: int, min_x: int, max_x: int) -> set[int]:
    """Columns ``min_x..max_x`` of row ``cell_y`` that hold static geometry.

    The explicit static cells of the row and the columns its heater opening
    rectangles cover.
    """

    xs: set[int] = set()
    xs.update(
        cell_x
        for cell_x in state.kernel.raw_static_cells_by_y.get(cell_y, set())
        if min_x <= int(cell_x) <= max_x
    )
    for rect_min_x, rect_max_x in state.kernel.heater_opening_rect_ranges_by_y.get(cell_y, ()):
        start_x = max(min_x, int(rect_min_x))
        end_x = min(max_x, int(rect_max_x))
        if start_x <= end_x:
            xs.update(range(start_x, end_x + 1))
    return xs


def _instance_static_geometry_open_cells(
    state: SessionState,
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
    ref = _instance_ref_by_name(state, instance_name)
    if ref is None:
        return set()
    bounds = bbox_bounds_um(ref)
    if bounds is None:
        return set()
    left, bottom, right, top = bounds
    grid_size = float(state.obstacles.grid.grid_size_um)
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
        state.obstacles.grid,
    )
    max_bbox_cell = physical_to_grid(
        float(right) + bbox_margin,
        float(top) + bbox_margin,
        state.obstacles.grid,
    )
    min_x = max(
        0,
        min_bbox_cell[0],
    )
    max_x = min(
        state.kernel.frame.grid_width - 1,
        max_bbox_cell[0],
    )
    min_y = max(
        0,
        min_bbox_cell[1],
    )
    max_y = min(
        state.kernel.frame.grid_height - 1,
        max_bbox_cell[1],
    )
    if min_x > max_x or min_y > max_y:
        return set()

    heater_clearance_um = getattr(
        state.obstacles.resolved_obstacle_config, "heater_clearance_um", None
    )
    opening_margin_um = max(
        float(state.kernel.radii.route_clearance_um),
        0.0 if heater_clearance_um is None else float(heater_clearance_um),
    )
    opening_margin_cells = math.ceil(opening_margin_um / grid_size) + 1
    search_min_x = max(0, min_x - opening_margin_cells)
    search_max_x = min(state.kernel.frame.grid_width - 1, max_x + opening_margin_cells)
    search_min_y = max(0, min_y - opening_margin_cells)
    search_max_y = min(state.kernel.frame.grid_height - 1, max_y + opening_margin_cells)
    opening_margin_distance = float(opening_margin_cells) * grid_size + bbox_margin
    opening_left = float(left) - opening_margin_distance
    opening_right = float(right) + opening_margin_distance
    opening_bottom = float(bottom) - opening_margin_distance
    opening_top = float(top) + opening_margin_distance

    candidate_cells: set[tuple[int, int]] = set()
    for cell_y in range(search_min_y, search_max_y + 1):
        for cell_x in _static_columns_in_row(state, cell_y, search_min_x, search_max_x):
            inside_instance_bbox = min_x <= int(cell_x) <= max_x and min_y <= int(cell_y) <= max_y
            cell_center = state_cell_center_um(state, int(cell_x), int(cell_y))
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


class _PortAccess(NamedTuple):
    """Step 2's tables, before they join the phase's record."""

    cells_by_spec: dict[str, set[tuple[int, int]]]
    candidate_cells_by_spec: dict[str, set[tuple[int, int]]]
    rule_by_spec: dict[str, str | None]
    runway_cells_by_spec: dict[str, set[tuple[int, int]]]


def _endpoint_state_for_lane_assignment(state: SessionState, port: Port, *, as_target: bool) -> Any:
    return port_to_grid_state(
        state,
        port,
        as_target=as_target,
    )


def _build_crossing_plan(
    settings: SessionSettings,
    state: SessionState,
    route_jobs: list[RouteJob],
) -> CrossingPlanInfo:
    """Step 1: the crossing plan of the session, with the fan-out facts it records.

    Prints the crossing configuration when crossings are enabled.
    """
    t_crossing_context_start = timing.pipeline_timer_start(settings)
    plan = CrossingPlanInfo.from_dict(
        build_crossing_plan_info(
            rust_backend=state.rust_backend,
            router=state.kernel.router,
            schematic=settings.schematic,
            unrouted_layout=settings.unrouted_layout,
            placement_analysis=settings.placement_analysis,
            route_jobs=route_jobs,
            request=CrossingPlanRequest(
                enable_crossings=settings.enable_crossings,
                crossing_mode=settings.crossing_mode,
                crossing_loss=float(settings.crossing_loss),
                crossing_search_loss=float(settings.crossing_search_loss),
                crossing_half_size_cells=int(state.obstacles.resolved_crossing_half_size_cells),
                guidance_net_names=settings.crossing_guidance_net_names,
                plan_config=settings.config.crossing_plan,
            ),
        )
    )
    plan.crossing_mode = settings.crossing_mode
    if bool(settings.enable_crossings):
        # The exact configuration must be visible in stdout (harness step 0):
        # which of baseline / contribution 1 ran, and with which prices.
        guidance = plan.guidance
        guidance_text = (
            f" planned_pairs={guidance['planned_pair_count']}"
            f" planned_loss={guidance['planned_crossing_loss']:.1f}"
            f" budget={'1/pair' if guidance.get('single_discounted_crossing_per_pair', True) else 'unlimited'}"
            if isinstance(guidance, dict)
            else ""
        )
        print(
            f"      - crossing search: mode={settings.crossing_mode}"
            f" search_loss={float(plan.crossing_search_loss):.1f}"
            f"{guidance_text}"
        )
    plan.bend_runout_cells_per_crossing = int(state.kernel.radii.bend_radius_cells)
    plan.fanout_stub_bend_degrees = 45 * int(fanout_stub_bend_steps(settings))
    plan.required_straight_margin_cells_per_crossing = int(
        state.obstacles.resolved_crossing_half_size_cells
    ) + int(state.kernel.radii.bend_radius_cells)
    plan.fanout_access_mode = settings.fanout_access_mode_normalized
    plan.fanout_anchor_port_count = len(state.fanout.anchor_by_port_spec)
    plan.fanout_anchor_net_ids = sorted(state.fanout.anchor_net_ids)
    plan.fanout_anchor_source_net_ids = sorted(state.fanout.anchor_source_net_ids)
    plan.fanout_anchor_target_net_ids = sorted(state.fanout.anchor_target_net_ids)
    plan.fanout_stub_center_cell_count = len(state.fanout.stub_center_cells)
    plan.fanout_stub_static_cell_count = len(state.fanout.stub_static_cells)
    plan.fanout_stub_centerlines_um = [
        {
            "port_spec": anchor.port_spec,
            "anchor_cell": [int(anchor.state_x), int(anchor.state_y)],
            "physical_angle": int(anchor.physical_angle) % 8,
            "centerline_um": [
                [float(point[0]), float(point[1])] for point in anchor.stub_centerline_um
            ],
        }
        for anchor in sorted(
            state.fanout.anchor_by_port_spec.values(),
            key=lambda item: item.port_spec,
        )
    ]
    plan.crossing_device = state.obstacles.crossing_device_info
    timing.record_pipeline_timing(settings, state, "crossing_context", t_crossing_context_start)
    return plan


def _build_port_access(
    settings: SessionSettings,
    state: SessionState,
    dense_port_runway_length_by_spec: dict[str, int],
) -> tuple[_PortAccess, dict[str, set[tuple[int, int]]]]:
    """Step 2: one footprint per port, and what the port's own net may open of it.

    Returns the access tables and the raw footprint per port spec, which the
    foreign keep-out reuses unchanged.

    Raises:
        RuntimeError: the loaded kernel has no `build_port_footprint_cells`.
        ValueError: a port has no finite center coordinate.
    """
    if not hasattr(state.kernel.router, "build_port_footprint_cells"):
        extension_path = getattr(state.rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.build_port_footprint_cells. Rebuild it with "
            "`maturin develop --release`. "
            f"Loaded extension: {extension_path}"
        )

    port_rule_extra_open_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    access_rule_by_spec: dict[str, str | None] = {}
    access_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    runway_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    t_port_opening_prep_start = timing.pipeline_timer_start(settings)
    port_opening_inputs: list[
        tuple[str, float, float, float | None, str | None, float | None, float | None]
    ] = []
    for port_spec, (instance_name, port_name, port) in state.endpoints.ports_by_spec.items():
        fanout_anchor = state.fanout.anchor_by_port_spec.get(port_spec)
        center = fanout_anchor.center_um if fanout_anchor is not None else center_of_port_um(port)
        if center is None:
            raise ValueError(f"Port {port_spec!r} has no finite center coordinate")
        orientation_value = getattr(port, "orientation", None)
        orientation = None if orientation_value is None else float(orientation_value)
        port_type = port_type_name(port)
        rule = port_access_rule_for(
            settings,
            state,
            instance_name=instance_name,
            port_name=port_name,
            port=port,
        )
        access_length_um = None if rule is None else float(rule.access_length_um)
        access_width_um = None if rule is None else float(rule.access_width_um)
        access_rule_by_spec[port_spec] = None if rule is None else rule.component_name_pattern
        port_rule_extra_open_cells_by_spec[port_spec] = (
            _instance_static_geometry_open_cells(
                state,
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

    t_port_opening_batch_start = timing.pipeline_timer_start(settings)

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
        instance_name, port_name, port = state.endpoints.ports_by_spec[port_spec]
        custom_access = access_length_um is not None or access_width_um is not None
        if not custom_access and port_type is not None and str(port_type) != "optical":
            continue
        length_cells, half_width_cells = _resolve_port_footprint_cells(
            settings,
            state,
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

    for port_spec, raw_cells in state.kernel.router.build_port_footprint_cells(
        port_footprint_inputs
    ):
        raw_footprint_cells_by_spec[str(port_spec)] = {
            (int(cell[0]), int(cell[1])) for cell in raw_cells
        }

    for port_spec, raw_footprint_cells in raw_footprint_cells_by_spec.items():
        open_cells = raw_footprint_cells - cells_in_raw_static_geometry(state, raw_footprint_cells)
        access_cells_by_spec[port_spec] = set(open_cells)
        access_candidate_cells_by_spec[port_spec] = set(raw_footprint_cells)
        extra_open_cells = port_rule_extra_open_cells_by_spec.get(port_spec, set())
        if extra_open_cells:
            access_cells_by_spec[port_spec].update(extra_open_cells)
            access_candidate_cells_by_spec[port_spec].update(extra_open_cells)
        runway_cells_by_spec[port_spec] = set(raw_footprint_cells)
    timing.record_pipeline_timing(settings, state, "port_opening_batch", t_port_opening_batch_start)

    return (
        _PortAccess(
            access_cells_by_spec,
            access_candidate_cells_by_spec,
            access_rule_by_spec,
            runway_cells_by_spec,
        ),
        raw_footprint_cells_by_spec,
    )


def _build_foreign_keepouts(
    settings: SessionSettings,
    state: SessionState,
    raw_footprint_cells_by_spec: dict[str, set[tuple[int, int]]],
) -> tuple[dict[str, set[tuple[int, int]]], dict[str, set[tuple[int, int]]]]:
    """Step 3: the cells foreign nets must keep clear, per port spec and per instance."""
    # Foreign-keepout reuses the exact same raw footprint each port already
    # got for its own opening above -- a keepout region is the same shape,
    # just unfiltered (no must-stay-blocked subtraction), since it exists to
    # say "nothing else may enter here," not to describe what this port's
    # own net may route through. foreign_port_keepout_cells now only gates
    # whether keepout logic runs at all; it no longer controls size.
    keepout_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    keepout_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
    if settings.foreign_port_keepout_cells > 0:
        t_foreign_keepout_start = timing.pipeline_timer_start(settings)
        for port_spec, raw_footprint_cells in raw_footprint_cells_by_spec.items():
            instance_name = port_spec.split(",", 1)[0]
            cells_for_spec = set(raw_footprint_cells)
            keepout_cells_by_spec[port_spec] = cells_for_spec
            keepout_cells_by_instance.setdefault(instance_name, set()).update(cells_for_spec)
        timing.record_pipeline_timing(
            settings, state, "foreign_port_keepout_batch", t_foreign_keepout_start
        )

    return keepout_cells_by_spec, keepout_cells_by_instance


def _lateral_positions_by_angle(
    state: SessionState, port_specs: set[str]
) -> dict[int, list[tuple[str, float]]]:
    """Group an instance's ports by heading, each with its position across that heading.

    A port with a static fan-out stub is taken at the stub's anchor.
    """
    groups: dict[int, list[tuple[str, float]]] = {}
    for port_spec in port_specs:
        _inst, _port_name, port = state.endpoints.ports_by_spec[port_spec]
        fanout_anchor = state.fanout.anchor_by_port_spec.get(port_spec)
        angle = (
            int(fanout_anchor.physical_angle) % 8
            if fanout_anchor is not None
            else orientation_to_angle(getattr(port, "orientation", None), flip=False)
        )
        step_x, step_y = angle_to_step(angle)
        lateral_x, lateral_y = -step_y, step_x
        center = fanout_anchor.center_um if fanout_anchor is not None else center_of_port_um(port)
        if center is None or (lateral_x == 0 and lateral_y == 0):
            continue
        lateral_position = float(center[0]) * lateral_x + float(center[1]) * lateral_y
        groups.setdefault(angle, []).append((port_spec, lateral_position))
    return groups


def _window_between_neighbours(positions: list[float], index: int) -> tuple[float, float]:
    """The lateral window of the port at `index`: halfway to each neighbour.

    At the end of a group the window is as wide on the open side as on the
    side of the one neighbour. `positions` is sorted and has at least two
    entries.
    """
    position = positions[index]
    if index == 0:
        lower = position - abs(positions[1] - position) * 0.5
    else:
        lower = (positions[index - 1] + position) * 0.5
    if index + 1 == len(positions):
        upper = position + abs(position - positions[index - 1]) * 0.5
    else:
        upper = (position + positions[index + 1]) * 0.5
    return lower, upper


def _build_dense_lateral_windows(
    settings: SessionSettings,
    state: SessionState,
    endpoint_port_specs_by_instance: dict[str, set[str]],
) -> tuple[
    dict[str, tuple[float, float, float, float, float]],
    dict[str, tuple[float, float, tuple[tuple[str, float], ...]]],
]:
    """Step 4: the lateral room of each port in a dense group facing one way.

    Returns the window per port spec and the owner group per port spec.
    """
    windows: dict[str, tuple[float, float, float, float, float]] = {}
    owner_groups: dict[
        str,
        tuple[float, float, tuple[tuple[str, float], ...]],
    ] = {}
    for _instance_name, port_specs in endpoint_port_specs_by_instance.items():
        if len(port_specs) < dense_fanout_min_ports(settings):
            continue
        groups = _lateral_positions_by_angle(state, port_specs)
        for angle, group in groups.items():
            if len(group) <= 1:
                continue
            step_x, step_y = angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x
            ordered = sorted(group, key=lambda item: item[1])
            owner_group = tuple(ordered)
            for owned_port_spec, _lateral_position in ordered:
                owner_groups[owned_port_spec] = (
                    float(lateral_x),
                    float(lateral_y),
                    owner_group,
                )
            positions = [position for _spec, position in ordered]
            for index, (port_spec, _position) in enumerate(ordered):
                lower, upper = _window_between_neighbours(positions, index)
                lane_margin_um = 0.0
                windows[port_spec] = (
                    float(lateral_x),
                    float(lateral_y),
                    float(lower),
                    float(upper),
                    lane_margin_um,
                )
    return windows, owner_groups


def _endpoints_by_grid_state(
    state: SessionState, route_jobs: list[RouteJob]
) -> dict[tuple[int, int, int], list[tuple[str, bool, Port]]]:
    """The endpoints of all jobs by the grid state (x, y, heading) they map to."""
    endpoint_ports_by_key: dict[tuple[int, int, int], list[tuple[str, bool, Port]]] = {}
    for job in route_jobs:
        for port_spec, port, as_target in (
            (f"{job.inst1},{job.port1}", job.source_port, False),
            (f"{job.inst2},{job.port2}", job.target_port, True),
        ):
            grid_state = _endpoint_state_for_lane_assignment(state, port, as_target=as_target)
            key = (int(grid_state.x), int(grid_state.y), int(grid_state.angle) % 8)
            endpoint_ports_by_key.setdefault(key, []).append((port_spec, as_target, port))
    return endpoint_ports_by_key


def _assign_lane_offsets(
    state: SessionState, route_jobs: list[RouteJob]
) -> dict[tuple[str, bool], tuple[int, int]]:
    """Step 5: spread the endpoints that would share one grid cell and heading."""
    endpoint_ports_by_key = _endpoints_by_grid_state(state, route_jobs)
    lane_offsets: dict[tuple[str, bool], tuple[int, int]] = {}
    for (base_x, base_y, angle), endpoints in endpoint_ports_by_key.items():
        unique_endpoints = list(
            dict.fromkeys((spec, is_target) for spec, is_target, _ in endpoints)
        )
        if len(unique_endpoints) <= 1:
            continue
        step_x, step_y = angle_to_step(angle)
        lateral_x, lateral_y = -step_y, step_x
        if lateral_x == 0 and lateral_y == 0:
            continue

        def _lateral_position_for_lane_assignment(
            item: tuple[str, bool, Port], lateral_x: int = lateral_x, lateral_y: int = lateral_y
        ) -> float:
            center = center_of_port_um(item[2])
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
            if cell_in_grid(state, candidate_x, candidate_y):
                lane_offsets[endpoint_key] = (
                    lateral_x * lane_index,
                    lateral_y * lane_index,
                )
            lane_index += 1
    return lane_offsets


def build_crossing_plan_and_port_footprints(
    settings: SessionSettings,
    state: SessionState,
    route_jobs: list[RouteJob],
    endpoint_port_specs_by_instance: dict[str, set[str]],
    dense_port_runway_length_by_spec: dict[str, int],
) -> PlannedJobs:
    """Phase 4: plan the crossings and resolve the footprint of every port.

    Stores the crossing plan (`state.crossing_plan_info`, which later phases
    extend) and one `PortFootprints` (`state.ports`). Returns the jobs,
    unchanged, and the foreign keep-out cells per instance, which phase 5
    needs for the static handoff.
    """
    state.crossing_plan_info = _build_crossing_plan(settings, state, route_jobs)
    access, raw_footprint_cells_by_spec = _build_port_access(
        settings, state, dense_port_runway_length_by_spec
    )
    keepout_cells_by_spec, keepout_cells_by_instance = _build_foreign_keepouts(
        settings, state, raw_footprint_cells_by_spec
    )
    windows, owner_groups = _build_dense_lateral_windows(
        settings, state, endpoint_port_specs_by_instance
    )
    normal_runway_cells: set[tuple[int, int]] = set()
    for cells in access.runway_cells_by_spec.values():
        normal_runway_cells.update(cells)
    state.ports = PortFootprints(
        access_cells_by_spec=access.cells_by_spec,
        access_candidate_cells_by_spec=access.candidate_cells_by_spec,
        access_rule_by_spec=access.rule_by_spec,
        runway_cells_by_spec=access.runway_cells_by_spec,
        normal_runway_cells=normal_runway_cells,
        foreign_keepout_cells_by_spec=keepout_cells_by_spec,
        dense_lateral_windows=windows,
        dense_lateral_owner_groups=owner_groups,
        state_lane_offsets=_assign_lane_offsets(state, route_jobs),
    )
    return PlannedJobs(route_jobs, keepout_cells_by_instance)
