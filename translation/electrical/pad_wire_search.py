"""Grid search for one pad wire: the A* stub search and its full-grid fallback."""

from __future__ import annotations

from collections import deque
from heapq import heappop, heappush

from .types import (
    ElectricalObstacleMap,
    ElectricalPortAccess,
    ElectricalRoutingConfig,
    GridCell,
    GridPoint,
)
from .wire_geometry import (
    cell_center,
    centerline_points,
    dedupe_path,
    dilate_cells,
    in_bounds,
    manhattan,
    point_to_cell,
    terminal_exit_direction_x,
    wire_reservation_radius_cells,
)

DirectionIndex = int
SearchState = tuple[GridCell, DirectionIndex]

_NO_DIRECTION = -1
_GRID_DIRECTIONS: tuple[GridCell, ...] = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
)


def route_full_grid_pad_wire(
    terminal_access: ElectricalPortAccess,
    pad_target_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    raw_blocked_cells: set[GridCell],
    committed_footprint_cells: set[GridCell],
    config: ElectricalRoutingConfig,
) -> tuple[GridPoint, ...]:
    """Full-grid A* fallback from a terminal's port anchor to its pad.

    Used only for a wire ``try_river_route_bundle`` refused (the pad column
    is on the wrong side of the exit direction, or a run is blocked).
    Searches the whole electrical grid, avoiding the obstacle map, every
    other terminal's opening, and every other pad (``raw_blocked_cells``,
    dilated here by the obstacle-clearance radius, matching the clearance a
    river wire requires by dilating its own centerline instead), plus every
    already-committed wire's reservation footprint
    (``committed_footprint_cells``, already dilated by the spacing radius --
    wire width plus the wider of the obstacle clearance and the individual
    route spacing -- when it was committed; not dilated again, or two wires
    would be held twice their required clearance apart). The first step is
    forced into the terminal's own exit direction so the search cannot double
    back through the terminal's own opening from the wrong side.

    Before searching, ``_pad_reachable`` runs a plain breadth-first search to
    rule out an unreachable pad cheaply: the A* below would otherwise exhaust
    every direction state of the grid (width * height * 4) before concluding
    a pad cannot be reached at all.
    """

    exit_dx = terminal_exit_direction_x(terminal_access)
    forced_first_direction = (exit_dx, 0) if exit_dx != 0 else None
    obstacle_radius = wire_reservation_radius_cells(obstacle_map, config)
    blocked = dilate_cells(
        raw_blocked_cells, obstacle_radius, obstacle_map.grid.width, obstacle_map.grid.height
    )
    blocked.update(committed_footprint_cells)
    if not _pad_reachable(
        terminal_access.anchor_cell,
        pad_target_cells,
        blocked,
        obstacle_map.grid.width,
        obstacle_map.grid.height,
        forced_first_direction,
    ):
        return ()
    return search_pad_wire(
        start=cell_center(terminal_access.anchor_cell),
        target_cells=pad_target_cells,
        blocked=blocked,
        obstacle_map=obstacle_map,
        config=config,
        forced_first_direction=forced_first_direction,
    )


def _pad_reachable(
    start_cell: GridCell,
    targets: frozenset[GridCell],
    blocked: set[GridCell],
    width: int,
    height: int,
    forced_first_direction: tuple[int, int] | None,
) -> bool:
    """Plain 4-connected BFS: can ``start_cell`` reach any of ``targets``?

    Mirrors ``search_pad_wire``'s blocked handling (the targets and the
    start cell are never treated as blocking) and its forced first move, but
    with no direction states and no costs, so it settles an unreachable pad
    in one bounded pass over undirected cells instead of the full A* state
    space.
    """

    targets = frozenset(cell for cell in targets if in_bounds(cell, width, height))
    if not targets:
        return False
    if start_cell in targets:
        return True

    local_blocked = set(blocked)
    local_blocked.difference_update(targets)
    local_blocked.discard(start_cell)

    if forced_first_direction is not None:
        first_cell = (
            start_cell[0] + forced_first_direction[0],
            start_cell[1] + forced_first_direction[1],
        )
        if not in_bounds(first_cell, width, height) or first_cell in local_blocked:
            return False
        frontier_start = first_cell
    else:
        frontier_start = start_cell

    if frontier_start in targets:
        return True
    visited = {start_cell, frontier_start}
    queue: deque[GridCell] = deque((frontier_start,))
    while queue:
        cell = queue.popleft()
        for dx, dy in _GRID_DIRECTIONS:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if neighbor in visited or not in_bounds(neighbor, width, height):
                continue
            if neighbor in local_blocked:
                continue
            visited.add(neighbor)
            if neighbor in targets:
                return True
            queue.append(neighbor)
    return False


def search_pad_wire(
    *,
    start: tuple[float, float],
    target_cells: frozenset[GridCell],
    blocked: set[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    forced_first_direction: tuple[int, int] | None = None,
) -> tuple[tuple[float, float], ...]:
    """Grid A* from ``start`` to the nearest cell in ``target_cells``.

    Used both for the short local pad stub and, via
    ``route_full_grid_pad_wire``, as the whole-grid Z fallback. When given,
    ``forced_first_direction`` restricts the very first move out of
    ``start`` to that direction (used to forbid a Z fallback from doubling
    back through its own terminal's opening).
    """

    start_cell = point_to_cell(start)
    targets = frozenset(
        cell
        for cell in target_cells
        if in_bounds(cell, obstacle_map.grid.width, obstacle_map.grid.height)
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
            return centerline_points(_reconstruct_search_path(parent, state))
        expansions += 1
        for next_direction, neighbor in _pad_stub_neighbors(
            cell,
            targets,
            config,
        ):
            if (
                direction == _NO_DIRECTION
                and forced_first_direction is not None
                and _GRID_DIRECTIONS[next_direction] != forced_first_direction
            ):
                continue
            if not in_bounds(neighbor, obstacle_map.grid.width, obstacle_map.grid.height):
                continue
            if neighbor in blocked:
                continue
            step_cost = _pad_stub_step_cost(
                cell,
                neighbor,
                direction,
                next_direction,
                targets,
                config,
            )
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
    target_x = round((min(tx for tx, _ in targets) + max(tx for tx, _ in targets)) / 2.0)
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
    return 10 * min(manhattan(cell, target) for target in targets)


def _pad_stub_step_cost(
    cell: GridCell,
    neighbor: GridCell,
    direction: DirectionIndex,
    next_direction: DirectionIndex,
    targets: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> int:
    cost = 10
    if direction != _NO_DIRECTION and direction != next_direction:
        cost += 10
    if _distance_to_targets(neighbor, targets) > _distance_to_targets(cell, targets):
        cost += 8
    if _moves_away_from_pad_edge(cell, neighbor, targets, config):
        cost += 4
    if _moves_away_from_target_center_x(cell, neighbor, targets):
        cost += 2
    return cost


def _distance_to_targets(cell: GridCell, targets: frozenset[GridCell]) -> int:
    return min(manhattan(cell, target) for target in targets)


def _moves_away_from_pad_edge(
    cell: GridCell,
    neighbor: GridCell,
    targets: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> bool:
    _, y = cell
    _, next_y = neighbor
    if config.pad_side == "top":
        target_y = min(target_y for _, target_y in targets)
        return next_y < y and y <= target_y
    target_y = max(target_y for _, target_y in targets)
    return next_y > y and y >= target_y


def _moves_away_from_target_center_x(
    cell: GridCell,
    neighbor: GridCell,
    targets: frozenset[GridCell],
) -> bool:
    x, _ = cell
    next_x, _ = neighbor
    target_x = _target_center_x(targets)
    return abs(next_x - target_x) > abs(x - target_x)


def _target_center_x(targets: frozenset[GridCell]) -> int:
    return round((min(x for x, _ in targets) + max(x for x, _ in targets)) / 2.0)


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
    return dedupe_path(tuple(path))
