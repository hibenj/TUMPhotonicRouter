"""Detailed centerline routing for topology-derived individual bundles."""

from __future__ import annotations

import math
from heapq import heappop, heappush
from typing import Literal

from .pad_slots import pad_access_cells
from .pitch_grid import bbox_to_grid_cells
from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    EscapeBundle,
    EscapeTopologyRoute,
    GridCell,
    IndividualEscapeTopologyResult,
    PadAssignment,
    PadPlan,
)

Axis = Literal["x", "y"]
RouteSide = Literal["left", "right"]
DirectionIndex = int
SearchState = tuple[GridCell, DirectionIndex]

_NO_DIRECTION = -1
_GRID_DIRECTIONS: tuple[GridCell, ...] = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
)


def route_detailed_bundles(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult,
    topology: IndividualEscapeTopologyResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> DetailedBundleRoutingResult:
    """Assign physical track offsets to topology routes and extend them to pads.

    The current detailed milestone keeps the grid topology as the centerline
    skeleton and adds a physical offset per route inside each bundle. This is
    the routing-level separation that metal realization should later consume;
    it avoids pretending that one grid cell can represent all parallel tracks.
    """

    config.validate()
    track_pitch_um = config.wire_width_um + config.individual_route_spacing_um
    track_pitch_cells = max(1, math.ceil(track_pitch_um / obstacle_map.grid.grid_size_um))
    assignments_by_terminal_id = {
        assignment.terminal.id: assignment
        for assignment in pad_plan.assignments
        if assignment.kind == "individual" and assignment.terminal is not None
    }
    target_cells_by_slot = {
        assignment.slot.index: pad_access_cells(assignment.slot, obstacle_map, config)
        for assignment in pad_plan.assignments
    }
    pad_lane_rank_by_slot, pad_lane_count_by_slot = _pad_lane_rank_maps(
        pad_plan,
        config,
    )
    topology_route_by_terminal_id = {
        route.terminal.id: route for route in topology.routes if route.success
    }
    all_terminal_cells = (
        set().union(*_individual_terminal_open_cells(obstacle_map).values())
        if _individual_terminal_open_cells(obstacle_map)
        else set()
    )
    hard_blocked = set(obstacle_map.blocked_cells)
    hard_blocked.update(common_bus.tree_cells)
    hard_blocked.update(common_bus_escape.path)

    routes: list[DetailedBundleRoute] = []
    failed_routes: list[DetailedBundleRoute] = []
    cell_usage: dict[GridCell, int] = {}
    committed_cells: set[GridCell] = set()
    committed_footprint_cells: set[GridCell] = set()
    assigned_pad_cells_by_slot = {
        assignment.slot.index: bbox_to_grid_cells(assignment.slot.bbox, obstacle_map.grid)
        for assignment in pad_plan.assignments
    }
    all_assigned_pad_cells = (
        set().union(*assigned_pad_cells_by_slot.values())
        if assigned_pad_cells_by_slot
        else set()
    )

    for bundle in topology.bundles:
        bundle_skeleton_path = _bundle_skeleton_path(bundle, config)
        route_side = _bundle_route_side(bundle, obstacle_map)
        ordered_lane_paths = _realize_ordered_bundle_lanes(
            bundle_skeleton_path,
            lane_count=bundle.required_tracks,
            track_pitch_um=track_pitch_um,
            route_side=route_side,
            grid_size_um=obstacle_map.grid.grid_size_um,
        )
        for rank, terminal in _route_order_for_bundle(bundle, route_side):
            assignment = assignments_by_terminal_id.get(terminal.id)
            topology_route = topology_route_by_terminal_id.get(terminal.id)
            target_cells = (
                target_cells_by_slot.get(assignment.slot.index, frozenset())
                if assignment is not None
                else frozenset()
            )
            failure_reason = _detail_failure_reason(
                terminal,
                assignment,
                topology_route,
                target_cells,
                obstacle_map,
                hard_blocked,
                all_terminal_cells,
            )
            if failure_reason is not None:
                failed_routes.append(
                    DetailedBundleRoute(
                        bundle_id=bundle.bundle_id,
                        rank=rank,
                        terminal=terminal,
                        pad_assignment=assignment,
                        path=(),
                        target_cells=frozenset(target_cells),
                        track_cell=None,
                        lane_cell=None,
                        offset_um=0.0,
                        offset_axis=_offset_axis(bundle),
                        offset_path=(),
                        success=False,
                        reason=failure_reason,
                    )
                )
                continue

            assert assignment is not None
            assert topology_route is not None
            offset_um = _rank_offset_um(rank, bundle, obstacle_map, track_pitch_um)
            bundle_track_path = (
                ordered_lane_paths[rank]
                if rank < len(ordered_lane_paths)
                else _centerline_points(bundle_skeleton_path)
            )
            source_point = _cell_center(topology_route.path[0])
            pad_lane = _individual_pad_lane_point(
                target_cells,
                obstacle_map,
                config,
                track_end=bundle_track_path[-1] if bundle_track_path else source_point,
                pad_lane_rank=pad_lane_rank_by_slot.get(assignment.slot.index, rank),
                pad_lane_count=pad_lane_count_by_slot.get(
                    assignment.slot.index,
                    bundle.required_tracks,
                ),
            )
            route_prefix, source_stub_path, bundle_track_tail, pad_stub_start = (
                _route_prefix_to_pad_stub_start(
                    source_point=source_point,
                    bundle_track_path=bundle_track_path,
                    pad_lane=pad_lane,
                    pad_side=config.pad_side,
                )
            )
            pad_stub_blocked = set(hard_blocked)
            pad_stub_blocked.update(all_terminal_cells)
            pad_stub_blocked.update(committed_footprint_cells)
            pad_stub_blocked.update(
                all_assigned_pad_cells.difference(target_cells)
            )
            pad_stub_path = _route_pad_stub_path(
                start=pad_stub_start,
                target_cells=target_cells,
                blocked=pad_stub_blocked,
                obstacle_map=obstacle_map,
                config=config,
            )
            if not pad_stub_path:
                failed_routes.append(
                    DetailedBundleRoute(
                        bundle_id=bundle.bundle_id,
                        rank=rank,
                        terminal=terminal,
                        pad_assignment=assignment,
                        path=(),
                        target_cells=frozenset(target_cells),
                        track_cell=None,
                        lane_cell=None,
                        offset_um=offset_um,
                        offset_axis=_offset_axis(bundle),
                        offset_path=(),
                        success=False,
                        reason="pad stub cannot avoid committed metal footprint",
                    )
                )
                continue
            detailed_path = _dedupe_points((*route_prefix, *pad_stub_path[1:]))
            path = _cells_from_point_path(detailed_path)
            track_cell = _representative_track_cell(topology_route)
            lane_cell = _representative_lane_cell(path, target_cells, config)
            route = DetailedBundleRoute(
                bundle_id=bundle.bundle_id,
                rank=rank,
                terminal=terminal,
                pad_assignment=assignment,
                path=path,
                target_cells=frozenset(target_cells),
                track_cell=track_cell,
                lane_cell=lane_cell,
                offset_um=offset_um,
                offset_axis=_offset_axis(bundle),
                offset_path=detailed_path,
                source_stub_path=source_stub_path,
                bundle_track_path=bundle_track_tail,
                pad_stub_path=pad_stub_path,
                success=True,
            )
            routes.append(route)
            committed_cells.update(path)
            committed_footprint_cells.update(
                _wire_reservation_cells_from_point_path(
                    detailed_path,
                    obstacle_map,
                    config,
                )
            )
            for cell in path:
                cell_usage[cell] = cell_usage.get(cell, 0) + 1

    routed_terminal_ids = {route.terminal.id for route in routes}
    routed_terminal_ids.update(route.terminal.id for route in failed_routes)
    for assignment in sorted(
        (
            assignment
            for assignment in pad_plan.assignments
            if assignment.kind == "individual" and assignment.terminal is not None
        ),
        key=lambda assignment: assignment.slot.index,
    ):
        terminal = assignment.terminal
        if terminal is None or terminal.id in routed_terminal_ids:
            continue
        failed_routes.append(
            DetailedBundleRoute(
                bundle_id=assignment.topology_bundle_id if assignment.topology_bundle_id is not None else -1,
                rank=assignment.topology_rank if assignment.topology_rank is not None else -1,
                terminal=terminal,
                pad_assignment=assignment,
                path=(),
                target_cells=frozenset(target_cells_by_slot.get(assignment.slot.index, frozenset())),
                track_cell=None,
                lane_cell=None,
                offset_um=0.0,
                offset_axis="x",
                offset_path=(),
                success=False,
                reason="terminal was not present in any escape topology bundle",
            )
        )

    return DetailedBundleRoutingResult(
        routes=tuple(sorted(routes, key=_detailed_route_sort_key)),
        failed_routes=tuple(sorted(failed_routes, key=_detailed_route_sort_key)),
        committed_cells=frozenset(committed_cells),
        cell_usage=cell_usage,
        track_pitch_cells=track_pitch_cells,
    )


def _route_order_for_bundle(
    bundle: EscapeBundle,
    route_side: RouteSide,
) -> tuple[tuple[int, ElectricalTerminal], ...]:
    ranked_terminals = tuple(enumerate(bundle.ordered_terminals))
    if route_side == "right":
        return tuple(reversed(ranked_terminals))
    return ranked_terminals


def _detailed_route_sort_key(
    route: DetailedBundleRoute,
) -> tuple[int, int, str]:
    return (route.bundle_id, route.rank, route.terminal.id)


def _pad_lane_rank_maps(
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> tuple[dict[int, int], dict[int, int]]:
    individual_assignments = tuple(
        assignment
        for assignment in pad_plan.assignments
        if assignment.kind == "individual"
    )
    if not individual_assignments:
        return {}, {}
    if config.pad_origin_x_um is not None:
        rank_by_slot = {
            assignment.slot.index: rank
            for rank, assignment in enumerate(
                sorted(individual_assignments, key=lambda assignment: assignment.slot.index)
            )
        }
        count_by_slot = {
            assignment.slot.index: len(individual_assignments)
            for assignment in individual_assignments
        }
        return rank_by_slot, count_by_slot

    grouped_assignments: dict[tuple[str, int], list[PadAssignment]] = {}
    fallback_group_id = 0
    for assignment in individual_assignments:
        if assignment.topology_bundle_id is None:
            group_key = ("fallback", fallback_group_id)
            fallback_group_id += 1
        else:
            group_key = ("bundle", assignment.topology_bundle_id)
        grouped_assignments.setdefault(group_key, []).append(assignment)

    rank_by_slot: dict[int, int] = {}
    count_by_slot: dict[int, int] = {}
    for assignments in grouped_assignments.values():
        ordered = sorted(
            assignments,
            key=lambda assignment: (
                assignment.topology_rank
                if assignment.topology_rank is not None
                else assignment.slot.index,
                assignment.slot.index,
            ),
        )
        group_count = len(ordered)
        for rank, assignment in enumerate(ordered):
            rank_by_slot[assignment.slot.index] = rank
            count_by_slot[assignment.slot.index] = group_count
    return rank_by_slot, count_by_slot


def _detail_failure_reason(
    terminal: ElectricalTerminal,
    assignment: PadAssignment | None,
    topology_route: EscapeTopologyRoute | None,
    target_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    hard_blocked: set[GridCell],
    all_terminal_cells: set[GridCell],
) -> str | None:
    if assignment is None:
        return "terminal has no pad assignment"
    if topology_route is None or not topology_route.path:
        return "terminal has no successful topology route"
    if not target_cells:
        return "assigned pad slot has no access cells"
    source_cells = _terminal_open_cells(obstacle_map, terminal.id)
    if not source_cells:
        return "terminal has no source cells in electrical grid"
    route_cells = set(topology_route.path)
    other_terminal_hits = route_cells.intersection(all_terminal_cells.difference(source_cells))
    if other_terminal_hits:
        return "topology route intersects other terminal openings"
    blocked_hits = route_cells.intersection(hard_blocked)
    if blocked_hits:
        return "topology route intersects hard blockers"
    if topology_route.path[0] not in source_cells:
        return "topology route does not start in source terminal opening"
    return None


def _target_cell(target_cells: frozenset[GridCell], config: ElectricalRoutingConfig) -> GridCell:
    if config.pad_side == "top":
        edge_y = min(y for _, y in target_cells)
    else:
        edge_y = max(y for _, y in target_cells)
    center_x = round((min(x for x, _ in target_cells) + max(x for x, _ in target_cells)) / 2.0)
    return min(target_cells, key=lambda cell: (abs(cell[1] - edge_y), abs(cell[0] - center_x), cell))


def _cell_center(cell: GridCell) -> tuple[float, float]:
    return (cell[0] + 0.5, cell[1] + 0.5)


def _individual_pad_lane_point(
    target_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    track_end: tuple[float, float],
    pad_lane_rank: int,
    pad_lane_count: int,
) -> tuple[float, float]:
    target = _target_cell(target_cells, config)
    direction_x = 1 if target[0] + 0.5 >= track_end[0] else -1
    track_pitch_cells = max(
        1,
        math.ceil(
            (config.wire_width_um + config.individual_route_spacing_um)
            / obstacle_map.grid.grid_size_um
        ),
    )
    if direction_x >= 0:
        shelf_index = pad_lane_rank + 1
    else:
        shelf_index = pad_lane_count - pad_lane_rank
    if config.pad_side == "top":
        target_edge_y = min(y for _, y in target_cells)
        lane_y = target_edge_y - shelf_index * track_pitch_cells
    else:
        target_edge_y = max(y for _, y in target_cells)
        lane_y = target_edge_y + shelf_index * track_pitch_cells
    lane_y = min(max(0, lane_y), obstacle_map.grid.height - 1)
    return (target[0] + 0.5, lane_y + 0.5)


def _route_prefix_to_pad_stub_start(
    *,
    source_point: tuple[float, float],
    bundle_track_path: tuple[tuple[float, float], ...],
    pad_lane: tuple[float, float],
    pad_side: str,
) -> tuple[
    tuple[tuple[float, float], ...],
    tuple[tuple[float, float], ...],
    tuple[tuple[float, float], ...],
    tuple[float, float],
]:
    if not bundle_track_path:
        source_stub = _manhattan_point_path(source_point, pad_lane)
        return source_stub, source_stub, (), source_stub[-1]

    attach_point, track_tail = _attach_point_and_tail(
        source_point,
        bundle_track_path,
        pad_side=pad_side,
    )
    source_stub = _manhattan_point_path(source_point, attach_point)
    track_tail = _truncate_track_tail_at_pad_lane(track_tail, pad_lane)
    pad_stub_start = track_tail[-1] if track_tail else attach_point
    prefix = _dedupe_points((*source_stub, *track_tail[1:]))
    return prefix, source_stub, track_tail, pad_stub_start


def _truncate_track_tail_at_pad_lane(
    track_tail: tuple[tuple[float, float], ...],
    pad_lane: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    if len(track_tail) <= 1:
        return track_tail
    lane_y = pad_lane[1]
    truncated: list[tuple[float, float]] = [track_tail[0]]
    for start, end in zip(track_tail, track_tail[1:]):
        if _segment_crosses_horizontal_line(start, end, lane_y):
            branch = _project_horizontal_line_to_segment(start, end, lane_y)
            if branch != truncated[-1]:
                truncated.append(branch)
            return _dedupe_points(tuple(truncated))
        if end != truncated[-1]:
            truncated.append(end)
    return _dedupe_points(tuple(truncated))


def _segment_crosses_horizontal_line(
    start: tuple[float, float],
    end: tuple[float, float],
    y: float,
) -> bool:
    low_y, high_y = sorted((start[1], end[1]))
    return low_y <= y <= high_y


def _project_horizontal_line_to_segment(
    start: tuple[float, float],
    end: tuple[float, float],
    y: float,
) -> tuple[float, float]:
    if start[0] == end[0]:
        return (start[0], y)
    if start[1] == end[1]:
        low_x, high_x = sorted((start[0], end[0]))
        return (min(max(start[0], low_x), high_x), y)
    return (start[0], y)


def _attach_point_and_tail(
    source_point: tuple[float, float],
    path: tuple[tuple[float, float], ...],
    *,
    pad_side: str,
) -> tuple[tuple[float, float], tuple[tuple[float, float], ...]]:
    if len(path) == 1:
        return path[0], path

    best_index = 0
    best_projection = path[0]
    best_key: tuple[float, float, int] | None = None
    for index, (start, end) in enumerate(zip(path, path[1:])):
        projection = _project_point_to_axis_segment(source_point, start, end)
        distance = abs(source_point[0] - projection[0]) + abs(source_point[1] - projection[1])
        pad_direction_score = -projection[1] if pad_side == "top" else projection[1]
        key = (distance, pad_direction_score, index)
        if best_key is None or key < best_key:
            best_key = key
            best_index = index
            best_projection = projection

    tail = _dedupe_points((best_projection, *path[best_index + 1 :]))
    return best_projection, tail


def _project_point_to_axis_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    if start[0] == end[0]:
        low_y, high_y = sorted((start[1], end[1]))
        return (start[0], min(max(point[1], low_y), high_y))
    if start[1] == end[1]:
        low_x, high_x = sorted((start[0], end[0]))
        return (min(max(point[0], low_x), high_x), start[1])
    # Offset paths should be rectilinear. Fall back to the segment start if a
    # malformed diagonal slips through so the route stays debuggable.
    return start


def _manhattan_point_path(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    if start == end:
        return (start,)
    if start[0] == end[0] or start[1] == end[1]:
        return (start, end)
    via = (end[0], start[1])
    return _dedupe_points((start, via, end))


def _route_pad_stub_path(
    *,
    start: tuple[float, float],
    target_cells: frozenset[GridCell],
    blocked: set[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[tuple[float, float], ...]:
    start_cell = _point_to_cell(start)
    targets = frozenset(
        cell
        for cell in target_cells
        if _in_bounds(cell, obstacle_map.grid.width, obstacle_map.grid.height)
    )
    if not targets:
        return ()

    blocked = set(blocked)
    blocked.difference_update(targets)
    blocked.discard(start_cell)

    start_state: SearchState = (start_cell, _NO_DIRECTION)
    parent: dict[SearchState, SearchState | None] = {start_state: None}
    best_cost: dict[SearchState, int] = {start_state: 0}
    heap: list[tuple[int, int, int, SearchState]] = []
    counter = 0
    heappush(
        heap,
        (
            _pad_stub_heuristic(start_cell, targets),
            0,
            counter,
            start_state,
        ),
    )
    max_expansions = max(1, obstacle_map.grid.width * obstacle_map.grid.height * 4)
    expansions = 0

    while heap and expansions < max_expansions:
        _, cost, _, state = heappop(heap)
        if cost != best_cost.get(state):
            continue
        cell, direction = state
        if cell in targets:
            return _centerline_points(_reconstruct_search_path(parent, state))
        expansions += 1
        for next_direction, neighbor in _pad_stub_neighbors(
            cell,
            targets,
            config,
        ):
            if not _in_bounds(neighbor, obstacle_map.grid.width, obstacle_map.grid.height):
                continue
            if neighbor in blocked:
                continue
            step_cost = 10
            if direction != _NO_DIRECTION and direction != next_direction:
                step_cost += 4
            next_cost = cost + step_cost
            next_state: SearchState = (neighbor, next_direction)
            if next_cost >= best_cost.get(next_state, 1_000_000_000):
                continue
            parent[next_state] = state
            best_cost[next_state] = next_cost
            counter += 1
            heappush(
                heap,
                (
                    next_cost + _pad_stub_heuristic(neighbor, targets),
                    next_cost,
                    counter,
                    next_state,
                ),
            )
    return ()


def _pad_stub_neighbors(
    cell: GridCell,
    targets: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> tuple[tuple[DirectionIndex, GridCell], ...]:
    x, y = cell
    target_x = round(
        (min(tx for tx, _ in targets) + max(tx for tx, _ in targets)) / 2.0
    )
    preferred_x_step = 1 if target_x >= x else -1
    preferred_y_step = 1 if config.pad_side == "top" else -1
    ordered_directions = (
        (preferred_x_step, 0),
        (0, preferred_y_step),
        (-preferred_x_step, 0),
        (0, -preferred_y_step),
    )
    return tuple(
        (
            _GRID_DIRECTIONS.index(direction),
            (x + direction[0], y + direction[1]),
        )
        for direction in ordered_directions
    )


def _pad_stub_heuristic(
    cell: GridCell,
    targets: frozenset[GridCell],
) -> int:
    return 10 * min(_manhattan(cell, target) for target in targets)


def _reconstruct_search_path(
    parent: dict[SearchState, SearchState | None],
    end: SearchState,
) -> tuple[GridCell, ...]:
    path: list[GridCell] = []
    current: SearchState | None = end
    while current is not None:
        path.append(current[0])
        current = parent[current]
    path.reverse()
    return _dedupe_path(tuple(path))


def _wire_reservation_cells_from_point_path(
    path: tuple[tuple[float, float], ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> frozenset[GridCell]:
    centerline = _cells_from_point_path(path)
    radius = _wire_reservation_radius_cells(obstacle_map, config)
    cells: set[GridCell] = set()
    for x, y in centerline:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (x + dx, y + dy)
                if _in_bounds(cell, obstacle_map.grid.width, obstacle_map.grid.height):
                    cells.add(cell)
    return frozenset(cells)


def _wire_reservation_radius_cells(
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    required_center_spacing_um = config.wire_width_um + max(
        config.obstacle_clearance_um,
        0.0,
    )
    return max(
        0,
        math.ceil(required_center_spacing_um / obstacle_map.grid.grid_size_um) - 1,
    )


def _cells_from_point_path(path: tuple[tuple[float, float], ...]) -> tuple[GridCell, ...]:
    if not path:
        return ()
    cells: list[GridCell] = []
    for start, end in zip(path, path[1:]):
        start_cell = _point_to_cell(start)
        end_cell = _point_to_cell(end)
        segment = grid_segment(start_cell, end_cell)
        if cells:
            cells.extend(segment[1:])
        else:
            cells.extend(segment)
    if not cells:
        cells.append(_point_to_cell(path[0]))
    return _dedupe_path(tuple(cells))


def _point_to_cell(point: tuple[float, float]) -> GridCell:
    return (round(point[0] - 0.5), round(point[1] - 0.5))


def _dedupe_points(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    deduped: list[tuple[float, float]] = []
    for point in points:
        if deduped and deduped[-1] == point:
            continue
        deduped.append(point)
    return tuple(deduped)


def grid_segment(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
    sx, sy = start
    ex, ey = end
    cells: list[GridCell] = []
    if sx != ex:
        step_x = 1 if ex > sx else -1
        cells.extend((x, sy) for x in range(sx, ex + step_x, step_x))
    else:
        cells.append((sx, sy))
    if sy != ey:
        step_y = 1 if ey > sy else -1
        start_y = sy + step_y
        cells.extend((ex, y) for y in range(start_y, ey + step_y, step_y))
    return tuple(cells)


def _in_bounds(cell: GridCell, width: int, height: int) -> bool:
    x, y = cell
    return 0 <= x < width and 0 <= y < height


def _rank_offset_um(
    rank: int,
    bundle: EscapeBundle,
    obstacle_map: ElectricalObstacleMap,
    track_pitch_um: float,
) -> float:
    if bundle.required_tracks <= 1:
        return 0.0
    if _bundle_route_side(bundle, obstacle_map) == "left":
        return -(bundle.required_tracks - 1 - rank) * track_pitch_um
    return rank * track_pitch_um


def _realize_ordered_bundle_lanes(
    skeleton_path: tuple[GridCell, ...],
    *,
    lane_count: int,
    track_pitch_um: float,
    route_side: str,
    grid_size_um: float,
) -> tuple[tuple[tuple[float, float], ...], ...]:
    """Return lane-preserving bus polylines for a Manhattan skeleton.

    Lane indices are logical, not recomputed from local segment geometry.  The
    initial segment defines the outward side; at each 90-degree bend the local
    side flips so the same physical lane remains ordered through the corner.
    """

    if lane_count <= 0:
        return ()
    points = _centerline_points(skeleton_path)
    if not points:
        return tuple(() for _ in range(lane_count))
    if len(points) == 1:
        return tuple((points[0],) for _ in range(lane_count))

    segments = _manhattan_segments(points)
    if not segments:
        return tuple((points[0],) for _ in range(lane_count))
    track_pitch_cells = track_pitch_um / grid_size_um

    lane_paths: list[tuple[tuple[float, float], ...]] = []
    for lane_index in range(lane_count):
        lane_paths.append(
            _realize_single_ordered_lane(
                segments,
                lane_index=lane_index,
                lane_count=lane_count,
                route_side=route_side,
                track_pitch_cells=track_pitch_cells,
            )
        )
    return tuple(lane_paths)


def _centerline_points(path: tuple[GridCell, ...]) -> tuple[tuple[float, float], ...]:
    simplified = _simplify_manhattan_path(path)
    return tuple(_cell_center(cell) for cell in simplified)


def _manhattan_segments(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in zip(points, points[1:]):
        if start == end:
            continue
        if start[0] != end[0] and start[1] != end[1]:
            raise ValueError("bundle skeleton must be Manhattan")
        if segments:
            previous_direction = _point_direction(*segments[-1])
            current_direction = _point_direction(start, end)
            if (
                previous_direction != current_direction
                and previous_direction != (-current_direction[0], -current_direction[1])
                and previous_direction[0] != 0
                and current_direction[0] != 0
            ):
                raise ValueError("consecutive horizontal bundle segments are invalid")
            if (
                previous_direction != current_direction
                and previous_direction != (-current_direction[0], -current_direction[1])
                and previous_direction[1] != 0
                and current_direction[1] != 0
            ):
                raise ValueError("consecutive vertical bundle segments are invalid")
        segments.append((start, end))
    return tuple(segments)


def _lane_offset_for_direction(
    direction: tuple[int, int],
    lane_index: int,
    *,
    lane_count: int,
    route_side: str,
    track_pitch_cells: float,
) -> tuple[float, float]:
    if lane_count <= 1:
        return (0.0, 0.0)
    dx, dy = direction
    if route_side == "left":
        vertical_x_offset = -(lane_count - 1 - lane_index) * track_pitch_cells
    else:
        vertical_x_offset = lane_index * track_pitch_cells
    if dy != 0:
        return (vertical_x_offset, 0.0)
    if dx < 0:
        return (0.0, vertical_x_offset)
    if dx > 0:
        return (0.0, -vertical_x_offset)
    return (0.0, 0.0)


def _realize_single_ordered_lane(
    segments: tuple[tuple[tuple[float, float], tuple[float, float]], ...],
    *,
    lane_index: int,
    lane_count: int,
    route_side: str,
    track_pitch_cells: float,
) -> tuple[tuple[float, float], ...]:
    segment_points: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in segments:
        offset = _lane_offset_for_direction(
            _point_direction(start, end),
            lane_index,
            lane_count=lane_count,
            route_side=route_side,
            track_pitch_cells=track_pitch_cells,
        )
        segment_points.append((_translate_point(start, offset), _translate_point(end, offset)))

    points: list[tuple[float, float]] = [segment_points[0][0]]
    for index in range(1, len(segment_points)):
        points.append(
            _lane_bend_intersection(
                segment_points[index - 1],
                segment_points[index],
            )
        )
    points.append(segment_points[-1][1])
    return _dedupe_points(tuple(points))


def _translate_point(
    point: tuple[float, float],
    offset: tuple[float, float],
) -> tuple[float, float]:
    return (
        point[0] + offset[0],
        point[1] + offset[1],
    )


def _lane_bend_intersection(
    previous_segment: tuple[tuple[float, float], tuple[float, float]],
    current_segment: tuple[tuple[float, float], tuple[float, float]],
) -> tuple[float, float]:
    previous_start, previous_end = previous_segment
    current_start, current_end = current_segment
    previous_direction = _point_direction(previous_start, previous_end)
    current_direction = _point_direction(current_start, current_end)
    if previous_direction[0] == 0 and current_direction[1] == 0:
        return (previous_end[0], current_start[1])
    if previous_direction[1] == 0 and current_direction[0] == 0:
        return (current_start[0], previous_end[1])
    return previous_end


def _point_direction(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[int, int]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    return (0, 0)


def _bundle_skeleton_path(
    bundle: EscapeBundle,
    config: ElectricalRoutingConfig,
) -> tuple[GridCell, ...]:
    if bundle.shared_cells:
        cells = bundle.shared_cells
    else:
        cells = bundle.cells
    if not cells:
        return ()
    if len(cells) == 1:
        return tuple(cells)
    start = _farthest_cell(min(cells), cells)[0]
    end, parent = _farthest_cell(start, cells)
    path = _reconstruct_cell_path(parent, end)
    if not path:
        return tuple(sorted(cells))
    if config.pad_side == "top" and path[-1][1] < path[0][1]:
        path = tuple(reversed(path))
    elif config.pad_side == "bottom" and path[-1][1] > path[0][1]:
        path = tuple(reversed(path))
    return path


def _farthest_cell(
    start: GridCell,
    cells: frozenset[GridCell],
) -> tuple[GridCell, dict[GridCell, GridCell | None]]:
    queue = [start]
    parent: dict[GridCell, GridCell | None] = {start: None}
    for current in queue:
        for neighbor in _grid_neighbors(current):
            if neighbor not in cells or neighbor in parent:
                continue
            parent[neighbor] = current
            queue.append(neighbor)
    farthest = max(parent, key=lambda cell: (_manhattan(start, cell), cell))
    return farthest, parent


def _reconstruct_cell_path(
    parent: dict[GridCell, GridCell | None],
    end: GridCell,
) -> tuple[GridCell, ...]:
    if end not in parent:
        return ()
    path: list[GridCell] = []
    current: GridCell | None = end
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return tuple(path)


def _grid_neighbors(cell: GridCell) -> tuple[GridCell, GridCell, GridCell, GridCell]:
    x, y = cell
    return ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1))


def _manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _offset_path_by_local_normals(
    path: tuple[GridCell, ...],
    *,
    offset_um: float,
    side: str,
    grid_size_um: float,
) -> tuple[tuple[float, float], ...]:
    if not path:
        return ()
    offset_cells = abs(offset_um) / grid_size_um
    if offset_cells == 0:
        return tuple((x + 0.5, y + 0.5) for x, y in path)

    simplified = _simplify_manhattan_path(path)
    if len(simplified) <= 1:
        x, y = simplified[0]
        return ((x + 0.5, y + 0.5),)

    normals = [
        _segment_normal(simplified[index], simplified[index + 1], side=side)
        for index in range(len(simplified) - 1)
    ]
    offset_points: list[tuple[float, float]] = []
    for index, cell in enumerate(simplified):
        x, y = cell
        if index == 0:
            normal = normals[0]
            offset_points.append(
                (
                    x + 0.5 + normal[0] * offset_cells,
                    y + 0.5 + normal[1] * offset_cells,
                )
            )
        elif index == len(simplified) - 1:
            normal = normals[-1]
            offset_points.append(
                (
                    x + 0.5 + normal[0] * offset_cells,
                    y + 0.5 + normal[1] * offset_cells,
                )
            )
        else:
            previous = normals[index - 1]
            current = normals[index]
            offset_points.append(
                (
                    x + 0.5 + previous[0] * offset_cells,
                    y + 0.5 + previous[1] * offset_cells,
                )
            )
            offset_points.append(
                (
                    x + 0.5 + current[0] * offset_cells,
                    y + 0.5 + current[1] * offset_cells,
                )
            )
    return _rectilinearize_points(tuple(offset_points))


def _rectilinearize_points(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    if len(points) <= 1:
        return points
    rectilinear: list[tuple[float, float]] = [points[0]]
    for point in points[1:]:
        previous = rectilinear[-1]
        if previous == point:
            continue
        if previous[0] != point[0] and previous[1] != point[1]:
            via = (previous[0], point[1])
            if via != previous and via != point:
                rectilinear.append(via)
        rectilinear.append(point)
    return tuple(rectilinear)


def _simplify_manhattan_path(path: tuple[GridCell, ...]) -> tuple[GridCell, ...]:
    if len(path) <= 2:
        return path
    simplified: list[GridCell] = [path[0]]
    previous_direction = _direction(path[0], path[1])
    for index in range(1, len(path) - 1):
        current_direction = _direction(path[index], path[index + 1])
        if current_direction != previous_direction:
            simplified.append(path[index])
            previous_direction = current_direction
    simplified.append(path[-1])
    return tuple(simplified)


def _direction(start: GridCell, end: GridCell) -> tuple[int, int]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    return (0, 0)


def _segment_normal(start: GridCell, end: GridCell, *, side: str) -> tuple[int, int]:
    dx, dy = _direction(start, end)
    if side == "left":
        return (-dy, dx)
    return (dy, -dx)


def _bundle_route_side(bundle: EscapeBundle, obstacle_map: ElectricalObstacleMap) -> RouteSide:
    terminal_grid_x = _median(
        tuple(
            _physical_x_to_grid(terminal.center[0], obstacle_map)
            for terminal in bundle.ordered_terminals
        )
    )
    exit_mid = (bundle.exit_interval[0] + bundle.exit_interval[1]) / 2.0
    return "left" if exit_mid <= terminal_grid_x else "right"


def _offset_axis(bundle: EscapeBundle) -> Axis:
    return "x" if bundle.order_axis == "y" else "y"


def _representative_track_cell(route: EscapeTopologyRoute) -> GridCell | None:
    if not route.path:
        return None
    return route.path[min(len(route.path) // 2, len(route.path) - 1)]


def _representative_lane_cell(
    path: tuple[GridCell, ...],
    target_cells: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> GridCell | None:
    if not path or not target_cells:
        return None
    target_y = min(y for _, y in target_cells) if config.pad_side == "top" else max(y for _, y in target_cells)
    return min(path, key=lambda cell: (abs(cell[1] - target_y), cell))


def _dedupe_path(cells: tuple[GridCell, ...]) -> tuple[GridCell, ...]:
    deduped: list[GridCell] = []
    for cell in cells:
        if deduped and deduped[-1] == cell:
            continue
        deduped.append(cell)
    return tuple(deduped)


def _physical_x_to_grid(x_um: float, obstacle_map: ElectricalObstacleMap) -> int:
    return int((x_um - obstacle_map.grid.origin[0]) / obstacle_map.grid.grid_size_um)


def _median(values: tuple[int, ...]) -> float:
    if not values:
        return 0.0
    sorted_values = sorted(values)
    mid = len(sorted_values) // 2
    if len(sorted_values) % 2 == 1:
        return float(sorted_values[mid])
    return (sorted_values[mid - 1] + sorted_values[mid]) / 2.0


def _individual_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    return (
        obstacle_map.individual_terminal_open_cells
        or obstacle_map.terminal_open_cells
    )


def _terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> frozenset[GridCell]:
    return _individual_terminal_open_cells(obstacle_map).get(
        terminal_id,
        frozenset(),
    )
