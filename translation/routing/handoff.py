"""Phase 5: the final route jobs and the static handoff to the kernel."""

from __future__ import annotations

import math
import time

from dataclasses import replace
from typing import Any, Iterable, Mapping, cast

from gdsfactory.typings import Port

from photonic_router.config import ALL_NETS
from photonic_router.static_obstacle_builder import grid_cell_center

from translation.route_order import depth_by_node_from_jobs, order_route_jobs
from translation.route_rust_records import RouteBookkeeping
from translation.route_rust_types import RipupRerouteConfig, RouteJob, RouteTimingBucket

from translation.routing.route_jobs import _angle_to_step, _in_bounds, port_to_grid_state
from translation.routing import timing
from translation.routing.settings import SessionSettings
from translation.routing.stages import FinalJobs
from translation.routing.state import SessionState

def _snap_nearly_collinear_states(
    settings,
    state,
    source_state: Any,
    target_state: Any,
    source_port: Port,
    target_port: Port,
) -> tuple[Any, Any, set[tuple[int, int]]]:
    original_cells = {
        (int(source_state.x), int(source_state.y)),
        (int(target_state.x), int(target_state.y)),
    }
    source_angle = int(source_state.angle) % 8
    target_angle = int(target_state.angle) % 8
    if source_angle != target_angle:
        return source_state, target_state, original_cells

    source_center = getattr(source_port, "center", None)
    target_center = getattr(target_port, "center", None)
    if source_center is None or target_center is None:
        return source_state, target_state, original_cells

    source_x_um = float(source_center[0])
    source_y_um = float(source_center[1])
    target_x_um = float(target_center[0])
    target_y_um = float(target_center[1])
    grid_size = float(state.grid.grid_size_um)
    max_snap_um = max(grid_size, 2.0 * grid_size)
    max_snap_cells = max(1, math.ceil(max_snap_um / grid_size))

    if source_angle in {0, 4}:
        direction = 1 if source_angle == 0 else -1
        if (target_x_um - source_x_um) * direction <= 0.0:
            return source_state, target_state, original_cells
        if abs(target_y_um - source_y_um) > max_snap_um:
            return source_state, target_state, original_cells
        if abs(int(target_state.y) - int(source_state.y)) > max_snap_cells:
            return source_state, target_state, original_cells
        snapped_target = state.rust_backend.State(
            int(target_state.x),
            int(source_state.y),
            int(target_state.angle),
        )
        return source_state, snapped_target, original_cells

    if source_angle in {2, 6}:
        direction = 1 if source_angle == 2 else -1
        if (target_y_um - source_y_um) * direction <= 0.0:
            return source_state, target_state, original_cells
        if abs(target_x_um - source_x_um) > max_snap_um:
            return source_state, target_state, original_cells
        if abs(int(target_state.x) - int(source_state.x)) > max_snap_cells:
            return source_state, target_state, original_cells
        snapped_target = state.rust_backend.State(
            int(source_state.x),
            int(target_state.y),
            int(target_state.angle),
        )
        return source_state, snapped_target, original_cells

    return source_state, target_state, original_cells


def _snap_same_heading_minimum_bend_offset(
    settings,
    state,
    source_state: Any,
    target_state: Any,
) -> tuple[Any, Any, set[tuple[int, int]]]:
    """Snap one-cell-short S-bend offsets to the nearest realizable target.

    With cardinal same-heading ports, two opposing 90-degree bend primitives
    impose a minimum perpendicular displacement of 2R. Physical port centers
    often land half a grid cell off that value. Without this snap, exact-cell
    routing can only satisfy the one-cell deficit by introducing a loop.
    """
    extra_cells: set[tuple[int, int]] = set()
    if settings.allow_45_degree_turns:
        return source_state, target_state, extra_cells

    source_angle = int(source_state.angle) % 8
    target_angle = int(target_state.angle) % 8
    if source_angle != target_angle:
        return source_state, target_state, extra_cells

    min_offset_cells = 2 * int(state.bend_radius_cells)
    if min_offset_cells <= 0:
        return source_state, target_state, extra_cells

    sx = int(source_state.x)
    sy = int(source_state.y)
    tx = int(target_state.x)
    ty = int(target_state.y)

    if source_angle in {0, 4}:
        forward_dx = tx - sx if source_angle == 0 else sx - tx
        dy = ty - sy
        if forward_dx < min_offset_cells or dy == 0:
            return source_state, target_state, extra_cells
        missing = min_offset_cells - abs(dy)
        if missing != 1:
            return source_state, target_state, extra_cells
        snapped_offset_cells = min_offset_cells + 1
        snapped_target = state.rust_backend.State(
            tx,
            sy + (snapped_offset_cells if dy > 0 else -snapped_offset_cells),
            target_angle,
        )
        if not _in_bounds(settings, state, int(snapped_target.x), int(snapped_target.y)):
            return source_state, target_state, extra_cells
        extra_cells.add((int(snapped_target.x), int(snapped_target.y)))
        return source_state, snapped_target, extra_cells

    if source_angle in {2, 6}:
        forward_dy = ty - sy if source_angle == 2 else sy - ty
        dx = tx - sx
        if forward_dy < min_offset_cells or dx == 0:
            return source_state, target_state, extra_cells
        missing = min_offset_cells - abs(dx)
        if missing != 1:
            return source_state, target_state, extra_cells
        snapped_offset_cells = min_offset_cells + 1
        snapped_target = state.rust_backend.State(
            sx + (snapped_offset_cells if dx > 0 else -snapped_offset_cells),
            ty,
            target_angle,
        )
        if not _in_bounds(settings, state, int(snapped_target.x), int(snapped_target.y)):
            return source_state, target_state, extra_cells
        extra_cells.add((int(snapped_target.x), int(snapped_target.y)))
        return source_state, snapped_target, extra_cells

    return source_state, target_state, extra_cells


def _filter_dense_port_opening(
    settings,
    state,
    port_spec: str,
    cells: set[tuple[int, int]],
) -> set[tuple[int, int]]:
    owner_group = state.dense_port_lateral_owner_groups.get(port_spec)
    if owner_group is not None and cells:
        lateral_x, lateral_y, owners = owner_group
        grid_size = float(state.grid.grid_size_um)
        filtered: set[tuple[int, int]] = set()
        for cell_x, cell_y in cells:
            center_x, center_y = grid_cell_center(cell_x, cell_y, state.grid)
            lateral_position = center_x * lateral_x + center_y * lateral_y
            nearest_spec = min(
                owners,
                key=lambda item: (abs(lateral_position - item[1]), item[0]),
            )[0]
            if nearest_spec == port_spec:
                filtered.add((cell_x, cell_y))
        return filtered

    window = state.dense_port_lateral_windows.get(port_spec)
    if window is None or not cells:
        return set(cells)
    lateral_x, lateral_y, lower, upper, lane_margin_um = window
    grid_size = float(state.grid.grid_size_um)
    lower -= lane_margin_um
    upper += lane_margin_um
    eps = max(1.0e-9, grid_size * 1.0e-9)
    filtered: set[tuple[int, int]] = set()
    for cell_x, cell_y in cells:
        center_x, center_y = grid_cell_center(cell_x, cell_y, state.grid)
        lateral_position = center_x * lateral_x + center_y * lateral_y
        if lower - eps <= lateral_position <= upper + eps:
            filtered.add((cell_x, cell_y))
    return filtered


def _opened_cells_for_spec(
    settings,
    state,
    cells_by_spec: Mapping[str, set[tuple[int, int]]],
    port_spec: str,
) -> set[tuple[int, int]]:
    return _filter_dense_port_opening(settings, state,
        port_spec,
        set(cells_by_spec.get(port_spec, set())),
    )


def _foreign_keepout_open_cells_for_spec(settings, state, port_spec: str) -> set[tuple[int, int]]:
    cluster_specs = state.dense_source_cluster_specs_by_port_spec.get(port_spec)
    if cluster_specs:
        cells: set[tuple[int, int]] = set()
        for cluster_port_spec in cluster_specs:
            cells.update(state.foreign_port_keepout_cells_by_spec.get(cluster_port_spec, set()))
        return cells - state.normal_port_runway_cells
    return (
        _opened_cells_for_spec(settings, state, state.foreign_port_keepout_cells_by_spec, port_spec)
        - state.normal_port_runway_cells
    )


def _topological_net_route_order(settings, state, jobs: list[RouteJob]) -> list[RouteJob]:
    """Route nets by `self.net_order` (`translation/route_order.py`):
    source-instance depth first, then declaration order / grid span /
    planned crossing count. Depth comes from this batch's own net graph
    (`inst1 -> inst2` edges), not from optional benchmark metadata or the
    crossing plan, so every mode has it. The plan-based orders need the
    topology plan, which only `lidar-guided` builds
    (`crossing_plan_info.expected_crossings_by_net_id`).
    """
    depth_by_node = (
        settings.net_order_depth_by_node
        if settings.net_order_depth_by_node is not None
        else depth_by_node_from_jobs(jobs)
    )
    span_by_net_id: dict[int, int] | None = None
    if settings.net_order == "topological-span":
        span_by_net_id = {int(job.net_id): _route_job_grid_span(
            settings, state, job
        ) for job in jobs}
    stub_by_net_id: dict[int, int] | None = None
    if settings.net_order == "topological-stub":
        stub_by_net_id = {}
        for job in jobs:
            lengths = []
            for spec in (f"{job.inst1},{job.port1}", f"{job.inst2},{job.port2}"):
                anchor = state.fanout_anchor_by_port_spec.get(spec)
                if anchor is not None:
                    lengths.append(len(anchor.stub_center_cells))
            stub_by_net_id[int(job.net_id)] = max(lengths) if lengths else 0
    planned_by_net_id: dict[int, int] | None = None
    if settings.net_order.startswith("plan-crossings"):
        raw_counts = state.crossing_plan_info.expected_crossings_by_net_id
        if not state.crossing_plan_info.event_count:
            raise ValueError(
                f"net_order {settings.net_order!r} needs the topology plan: run with "
                "--crossing-mode lidar-guided"
            )
        planned_by_net_id = {int(key): int(value) for key, value in raw_counts.items()}
    ordered = order_route_jobs(
        jobs,
        net_order=settings.net_order,
        depth_by_node=depth_by_node,
        span_by_net_id=span_by_net_id,
        planned_crossings_by_net_id=planned_by_net_id,
        stub_by_net_id=stub_by_net_id,
    )
    if settings.net_order != "topological":
        print(f"      - net order: {settings.net_order}")
    return _debug_hoist_instance_first(settings, state, ordered)


def _route_job_grid_span(settings, state, route_job: RouteJob) -> int:
    source_state = port_to_grid_state(settings, state,
        route_job.source_port,
        state.origin_x_um,
        state.origin_y_um,
        float(state.grid.grid_size_um),
        as_target=False,
    )
    target_state = port_to_grid_state(settings, state,
        route_job.target_port,
        state.origin_x_um,
        state.origin_y_um,
        float(state.grid.grid_size_um),
        as_target=True,
    )
    return abs(int(source_state.x) - int(target_state.x)) + abs(
        int(source_state.y) - int(target_state.y)
    )


def _debug_hoist_instance_first(settings, state, ordered: list[RouteJob]) -> list[RouteJob]:
    """Debug-only reordering: `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_INSTANCE=<name>`
    hoists every net touching the named instance to the front of the
    routing order (relative order preserved on both sides), so a dense
    group can be routed and debugged on empty dynamics in minutes
    instead of behind a 30-minute full-context prefix. Routing results
    under this knob are NOT comparable to stable runs -- ordering
    changes every downstream commit; diagnosis use only.
    """
    wanted_order = list(settings.config.diagnostics.debug_route_first_nets)
    if wanted_order:
        wanted = set(wanted_order)
        # the list's own order is binding (ordering experiments, 2026-09-04)
        rank = {net_id: position for position, net_id in enumerate(wanted_order)}
        hoisted = sorted(
            (job for job in ordered if int(job.net_id) in wanted),
            key=lambda job: rank[int(job.net_id)],
        )
        rest = [job for job in ordered if int(job.net_id) not in wanted]
        print(
            f"      - DEBUG route order: {len(hoisted)} nets from the explicit "
            f"net-id list hoisted to the front, in list order (diagnosis only)"
        )
        return hoisted + rest
    instance = settings.config.diagnostics.debug_route_first_instance
    if not instance:
        return ordered
    hoisted = [job for job in ordered if instance in (job.inst1, job.inst2)]
    rest = [job for job in ordered if instance not in (job.inst1, job.inst2)]
    print(
        f"      - DEBUG route order: {len(hoisted)} nets touching "
        f"{instance!r} hoisted to the front (diagnosis only)"
    )
    return hoisted + rest


def _endpoint_bump_candidate_open_cells_for_state(
    settings,
    state,
    grid_state: Any,
) -> set[tuple[int, int]]:
    """Cells a local endpoint bump may need opened against its own port pad."""
    base_x = int(grid_state.x)
    base_y = int(grid_state.y)
    step_x, step_y = _angle_to_step(settings, state, int(grid_state.angle) % 8)
    side_steps = ((-step_y, step_x), (step_y, -step_x))
    reach = max(1, int(state.bend_radius_cells))
    axis_reach = 4 * reach
    lateral_reach = 2 * reach
    cells: set[tuple[int, int]] = set()
    for forward in range(-axis_reach, reach + 1):
        if forward == 0:
            continue
        x = base_x + step_x * forward
        y = base_y + step_y * forward
        if 0 <= x < int(state.grid.width) and 0 <= y < int(state.grid.height):
            cells.add((x, y))
    for side_x, side_y in side_steps:
        for lateral in range(1, lateral_reach + 1):
            for forward in range(-axis_reach, reach + 1):
                if forward == 0:
                    continue
                x = base_x + step_x * forward + side_x * lateral
                y = base_y + step_y * forward + side_y * lateral
                if 0 <= x < int(state.grid.width) and 0 <= y < int(state.grid.height):
                    cells.add((x, y))
    return cells


def _states_and_openings(
    settings,
    state,
    job: RouteJob,
) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
    port1_spec = f"{job.inst1},{job.port1}"
    port2_spec = f"{job.inst2},{job.port2}"
    source_fanout_anchor = state.fanout_anchor_by_port_spec.get(port1_spec)
    target_fanout_anchor = state.fanout_anchor_by_port_spec.get(port2_spec)
    if source_fanout_anchor is None:
        source_state = port_to_grid_state(settings, state,
            job.source_port,
            state.origin_x_um,
            state.origin_y_um,
            float(state.grid.grid_size_um),
            as_target=False,
        )
    else:
        source_state = state.rust_backend.State(
            int(source_fanout_anchor.state_x),
            int(source_fanout_anchor.state_y),
            int(source_fanout_anchor.physical_angle) % 8,
        )
    if target_fanout_anchor is None:
        target_state = port_to_grid_state(settings, state,
            job.target_port,
            state.origin_x_um,
            state.origin_y_um,
            float(state.grid.grid_size_um),
            as_target=True,
        )
    else:
        target_state = state.rust_backend.State(
            int(target_fanout_anchor.state_x),
            int(target_fanout_anchor.state_y),
            (int(target_fanout_anchor.physical_angle) + 4) % 8,
        )
    source_lane_offset = state.port_state_lane_offsets.get((f"{job.inst1},{job.port1}", False))
    if source_lane_offset is not None and source_fanout_anchor is None:
        source_state = state.rust_backend.State(
            int(source_state.x) + int(source_lane_offset[0]),
            int(source_state.y) + int(source_lane_offset[1]),
            int(source_state.angle),
        )
    target_lane_offset = state.port_state_lane_offsets.get((f"{job.inst2},{job.port2}", True))
    if target_lane_offset is not None and target_fanout_anchor is None:
        target_state = state.rust_backend.State(
            int(target_state.x) + int(target_lane_offset[0]),
            int(target_state.y) + int(target_lane_offset[1]),
            int(target_state.angle),
        )
    if source_fanout_anchor is None and target_fanout_anchor is None:
        source_state, target_state, original_anchor_cells = _snap_nearly_collinear_states(
            settings, state,
            source_state,
            target_state,
            job.source_port,
            job.target_port,
        )
        source_state, target_state, snapped_anchor_cells = (
            _snap_same_heading_minimum_bend_offset(settings, state, source_state, target_state)
        )
        original_anchor_cells.update(snapped_anchor_cells)
    else:
        original_anchor_cells = {
            (int(source_state.x), int(source_state.y)),
            (int(target_state.x), int(target_state.y)),
        }
    source_anchor_cell = (int(source_state.x), int(source_state.y))
    target_anchor_cell = (int(target_state.x), int(target_state.y))
    endpoint_foreign_keepout_open_cells = set(
        _foreign_keepout_open_cells_for_spec(settings, state, port1_spec)
    )
    endpoint_foreign_keepout_open_cells.update(
        _foreign_keepout_open_cells_for_spec(settings, state, port2_spec)
    )
    opened_candidate_cells = set(
        _opened_cells_for_spec(
            settings, state, state.port_access_candidate_cells_by_spec, port1_spec
        )
    )
    opened_candidate_cells.update(
        _opened_cells_for_spec(
            settings, state, state.port_access_candidate_cells_by_spec, port2_spec
        )
    )
    opened_candidate_cells.update(endpoint_foreign_keepout_open_cells)
    opened_candidate_cells.update(original_anchor_cells)
    opened_candidate_cells.update({source_anchor_cell, target_anchor_cell})

    opened_cells_set = set(
        _opened_cells_for_spec(settings, state, state.port_access_cells_by_spec, port1_spec)
    )
    opened_cells_set.update(
        _opened_cells_for_spec(settings, state, state.port_access_cells_by_spec, port2_spec)
    )
    opened_cells_set.update(endpoint_foreign_keepout_open_cells)
    opened_cells_set.update(original_anchor_cells)
    opened_cells_set.update({source_anchor_cell, target_anchor_cell})
    if state.fanout_stub_static_cells:
        current_fanout_stub_open_cells: set[tuple[int, int]] = set()
        current_fanout_stub_open_cells.update(
            state.fanout_stub_static_cells_by_spec.get(port1_spec, set())
        )
        current_fanout_stub_open_cells.update(
            state.fanout_stub_static_cells_by_spec.get(port2_spec, set())
        )
        allowed_fanout_stub_open_cells = (
            current_fanout_stub_open_cells
            | original_anchor_cells
            | {source_anchor_cell, target_anchor_cell}
        )
        foreign_fanout_stub_static_cells = (
            state.fanout_stub_static_cells - allowed_fanout_stub_open_cells
        )
        opened_candidate_cells.difference_update(foreign_fanout_stub_static_cells)
        opened_cells_set.difference_update(foreign_fanout_stub_static_cells)
    source_endpoint_bump_open_cells = _endpoint_bump_candidate_open_cells_for_state(settings, state,
        source_state
    )
    target_endpoint_bump_open_cells = _endpoint_bump_candidate_open_cells_for_state(settings, state,
        target_state
    )
    opened_candidate_cells.update(source_endpoint_bump_open_cells)
    opened_candidate_cells.update(target_endpoint_bump_open_cells)
    trace_endpoint_bumps = settings.router_config.diagnostics.trace_endpoint_bump_nets
    if trace_endpoint_bumps is not None and (
        trace_endpoint_bumps == ALL_NETS or str(int(job.net_id)) in trace_endpoint_bumps
    ):
        print(
            "endpoint_open_trace "
            f"net_id={int(job.net_id)} "
            f"source_state=({int(source_state.x)},{int(source_state.y)},{int(source_state.angle)}) "
            f"target_state=({int(target_state.x)},{int(target_state.y)},{int(target_state.angle)}) "
            f"source_bump_open={len(source_endpoint_bump_open_cells)} "
            f"target_bump_open={len(target_endpoint_bump_open_cells)} "
            f"opened_candidate={len(opened_candidate_cells)} "
            f"has_112_243={(112, 243) in opened_candidate_cells}"
        )
    return (
        source_state,
        target_state,
        opened_candidate_cells,
        opened_cells_set,
        sorted(opened_cells_set),
    )


def finalize_route_jobs_and_static_handoff(
    settings: SessionSettings,
    state: SessionState,
    route_jobs: list[RouteJob],
    foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]],
    obstacle_map: Any,
) -> FinalJobs:
    """Order route jobs, hand static/keepout geometry off to Rust, apply debug-limit slicing, and prepare repair/timing bookkeeping.

    Applies `self._topological_net_route_order` when repair is enabled
    (reordering only helps a negotiation loop that can recover from a
    bad order; with repair disabled -- e.g. `test_benchmarks_route_with_astar_only`,
    which isolates plain A* on purpose -- a net that ends up later in a
    changed order can collide irrecoverably with an already-committed
    net that plain A* has no way to rip up, so declaration order is left
    untouched in that mode), aggregates static keepout cells
    (port runways, foreign-port keepouts, fanout stubs) and hands them to the Rust
    router via `set_static_rects`/`set_static_cells`/`add_static_cells`, applies
    `debug_stop_after_route_index`/`PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT` slicing to
    produce the final `route_jobs` for this run (also recording the pre-slicing job
    list as `self.full_route_jobs_by_route_index`), builds `self.repair_config`,
    `self.route_jobs_by_id`, `self.route_order`, and `self.route_bookkeeping`,
    initializes the route-search stats/timing-bucket attributes, and precomputes
    `self.batch_clearance_exempt_cells_by_id` via a batched Rust call. Returns the
    final, post-slicing `route_jobs` list used for dispatch, and `t_astar_start`
    (the `time.perf_counter()` reading taken before the precompute steps in this
    method, not after it returns) so the caller's post-dispatch `astar_elapsed_s`
    measurement still covers this method's own precompute time, exactly as it did
    before this method existed as a separate call.
    """
    repair_enabled = (settings.ripup_reroute_config or RipupRerouteConfig()).enabled
    if repair_enabled:
        route_jobs = _topological_net_route_order(settings, state, route_jobs)
        # Renumber so `route_index` is the execution position, layer by
        # layer. Everything a person points at by index -- the
        # `Routing [i/N]` lines, `--debug-svgs <selector>`,
        # `--debug-stop-after-route N`, a partial GDS "up to the failure"
        # -- then follows the order the nets are actually routed in,
        # instead of the schematic's declaration order (which, with
        # pre-placed crossing grids, lists every `__from_grid` stub after
        # every `__to_grid` one, so a stop-after cut used to drop nets
        # that had already been routed). `net_id` is untouched: it is
        # the identity the obstacle map and bookkeeping key on.
        route_jobs = [
            replace(job, route_index=position)
            for position, job in enumerate(route_jobs, start=1)
        ]

    port_runway_static_cells: set[tuple[int, int]] = set()
    for cells in state.port_runway_cells_by_spec.values():
        port_runway_static_cells.update(cells)
    state.foreign_port_keepout_static_cells: set[tuple[int, int]] = set()
    for cells in foreign_port_keepout_cells_by_instance.values():
        state.foreign_port_keepout_static_cells.update(cells)
    state.debug_port_keepout_cells = set(port_runway_static_cells)
    state.debug_port_keepout_cells.update(state.foreign_port_keepout_static_cells)
    state.debug_port_keepout_cells.update(state.fanout_stub_static_cells)
    state.static_blocked_cells_before_port_reservations = set(state.raw_static_cells)
    state.static_blocked_cells_before_port_reservations.update(port_runway_static_cells)
    state.static_blocked_cells_before_port_reservations.update(
        state.foreign_port_keepout_static_cells
    )
    state.static_blocked_cells_before_port_reservations.update(state.fanout_stub_static_cells)

    t_static_handoff_start = timing.pipeline_timer_start(settings, state)
    state.blocked_static_rects_for_diagnostics: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "blocked_static_rects"):
        blocked_static_rects: list[tuple[int, int, int, int]] = []
        raw_blocked_rects = cast(
            Iterable[tuple[int, int, int, int]], getattr(obstacle_map, "blocked_static_rects")
        )
        for rect in raw_blocked_rects:
            if len(rect) != 4:
                continue
            blocked_static_rects.append(
                (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
            )
        state.blocked_static_rects_for_diagnostics = list(blocked_static_rects)
        if blocked_static_rects:
            if not hasattr(state.router, "set_static_rects"):
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.set_static_rects. Rebuild it with "
                    "`maturin develop --release`; otherwise bounding_boxes mode "
                    "cannot use compact static rectangles."
                )
            state.router.set_static_rects(blocked_static_rects)
        else:
            state.router.set_static_cells(sorted(state.raw_static_cells))
    else:
        sorted_static_cells = sorted(state.raw_static_cells)
        state.router.set_static_cells(sorted_static_cells)
    if (
        port_runway_static_cells
        or state.foreign_port_keepout_static_cells
        or state.fanout_stub_static_cells
    ):
        if not hasattr(state.router, "add_static_cells"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.add_static_cells. Rebuild it with "
                "`maturin develop --release`."
            )
        state.router.add_static_cells(
            sorted(
                port_runway_static_cells
                | state.foreign_port_keepout_static_cells
                | state.fanout_stub_static_cells
            )
        )
    # Chip-boundary keepout: the grid padding outside the routable die
    # (`StaticObstacleMapData.routable_bbox`) is static, so no route can
    # leave the chip (e.g. east of the output couplers) and come back.
    chip_boundary_rects = [
        (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
        for rect in getattr(obstacle_map, "chip_boundary_rects", ())
        if len(rect) == 4
    ]
    state.chip_boundary_static_rects: list[tuple[int, int, int, int]] = chip_boundary_rects
    if chip_boundary_rects:
        if not hasattr(state.router, "add_static_rects"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.add_static_rects. Rebuild it with "
                "`maturin develop --release`."
            )
        state.router.add_static_rects(chip_boundary_rects)
        state.blocked_static_rects_for_diagnostics.extend(chip_boundary_rects)
        if settings.verbose_route_diagnostics:
            print(
                "  Chip-boundary keepout: "
                f"routable_bbox={getattr(obstacle_map, 'routable_bbox', None)} "
                f"rects={chip_boundary_rects}"
            )
    timing.record_pipeline_timing(settings, state, "static_map_handoff", t_static_handoff_start)

    full_route_jobs = list(route_jobs)
    state.full_route_jobs_by_route_index = {int(job.route_index): job for job in full_route_jobs}
    full_route_count = len(full_route_jobs)
    if (
        settings.debug_stop_after_route_index is not None
        and int(settings.debug_stop_after_route_index) > full_route_count
    ):
        raise ValueError(
            "debug_stop_after_route_index exceeds route count "
            f"({settings.debug_stop_after_route_index} > {full_route_count})"
        )
    if settings.debug_stop_after_route_index is not None:
        stop_index = int(settings.debug_stop_after_route_index)
        route_jobs = [job for job in full_route_jobs if int(job.route_index) <= stop_index]
        if (
            settings.verbose_route_diagnostics
            or settings.debug_route_indices is not None
        ):
            print(
                f"  Debug stop-after-route active: routing {len(route_jobs)} "
                f"of {full_route_count} full-context routes"
            )
    debug_execution_limit = settings.config.diagnostics.debug_execution_limit
    if debug_execution_limit is not None:
        original_route_job_count = len(route_jobs)
        route_jobs = route_jobs[:debug_execution_limit]
        if (
            settings.verbose_route_diagnostics
            or settings.debug_route_indices is not None
        ):
            print(
                "  Debug execution limit active: routing "
                f"{len(route_jobs)} of {original_route_job_count} selected "
                "routes in actual execution order"
            )

    state.repair_config = settings.ripup_reroute_config or RipupRerouteConfig()
    state.route_jobs_by_id = {job.net_id: job for job in route_jobs}
    state.route_order = [job.net_id for job in route_jobs]
    state.collect_timing = (
        settings.debug_timing
        or settings.collect_route_stats
        or settings.collect_attempt_diagnostics
    )
    state.track_dynamic_cells = state.diagnostics_enabled
    state.route_bookkeeping = RouteBookkeeping(
        route_order=state.route_order,
        diagnostics_enabled=state.track_dynamic_cells,
    )

    t_astar_start = 0.0
    if state.collect_timing:
        t_astar_start = time.perf_counter()
    state.total_expanded_states = 0
    state.simple_route_count = 0
    state.repair_count = 0
    state.deferred_count = 0
    state.route_attempt_records = []
    state.native_repair_trace_records: list[dict[str, object]] = []
    state.route_timing_buckets: dict[str, RouteTimingBucket] = {
        name: RouteTimingBucket()
        for name in (
            "normal_route",
            "probe_route",
            "preemptive_crossing_ripup",
            "guided_collision_crossing",
            "localized_crossing_keepout",
            "repair_failed_net",
            "reroute_victims",
            "lidar_pure_probe_commit",
            "endpoint_correction",
        )
    }

    if not hasattr(state.router, "build_dynamic_clearance_exempt_cells_for_routes"):
        extension_path = getattr(state.rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.build_dynamic_clearance_exempt_cells_for_routes. "
            "Rebuild it with `maturin develop --release`. "
            f"Loaded extension: {extension_path}"
        )

    t_state_opening_precompute_start = timing.pipeline_timer_start(settings, state)
    state.route_state_openings_by_id = {
        int(job.net_id): _states_and_openings(settings, state, job) for job in route_jobs
    }
    timing.record_pipeline_timing(settings, state,
        "state_opening_precompute",
        t_state_opening_precompute_start,
    )
    clearance_exempt_inputs = [
        (int(net_id), state_openings[0], state_openings[1], state_openings[4])
        for net_id, state_openings in state.route_state_openings_by_id.items()
    ]
    t_clearance_exempt_batch_start = timing.pipeline_timer_start(settings, state)
    state.batch_clearance_exempt_cells_by_id = {
        int(net_id): [(int(cell[0]), int(cell[1])) for cell in cells]
        for net_id, cells in state.router.build_dynamic_clearance_exempt_cells_for_routes(
            clearance_exempt_inputs,
            int(state.commit_radius_cells),
            int(state.port_lane_length_cells),
        )
    }
    timing.record_pipeline_timing(settings, state,
        "clearance_exempt_batch",
        t_clearance_exempt_batch_start,
    )

    state.realization_grid_spec = (
        int(state.grid.width),
        int(state.grid.height),
        float(state.grid.grid_size_um),
        float(state.origin_x_um),
        float(state.origin_y_um),
    )

    return FinalJobs(route_jobs, t_astar_start)
