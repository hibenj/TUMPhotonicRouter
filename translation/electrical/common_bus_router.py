"""Greedy rooted common-bus router for heater terminal selection."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Iterable, cast

from photonic_router.static_obstacle_builder import physical_to_grid

from .types import (
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    GridCell,
    TerminalBusRoute,
    TerminalPairGroup,
)


@dataclass(frozen=True)
class _CandidatePath:
    group: TerminalPairGroup
    terminal: ElectricalTerminal
    path: tuple[GridCell, ...]

    @property
    def cost(self) -> int:
        return max(0, len(self.path) - 1)


def route_common_bus(
    terminal_groups: tuple[TerminalPairGroup, ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> CommonBusRoutingResult:
    """Connect exactly one terminal from each heater to the fixed common bus.

    This is a deterministic greedy group-Steiner heuristic. The existing bus
    stripe is the root tree. At each step the router evaluates both candidate
    terminals of every unconnected heater and commits the globally cheapest path
    to the current same-net tree.
    """

    config.validate()
    remaining = {group.heater_id: group for group in terminal_groups}
    selected: dict[str, ElectricalTerminal] = {}
    unselected: dict[str, ElectricalTerminal] = {}
    routes: list[TerminalBusRoute] = []
    tree_cells: set[GridCell] = set(obstacle_map.bus.cells)
    blocked = set(obstacle_map.blocked_cells)
    all_terminal_cells = _all_terminal_cells(obstacle_map)
    median_x = _terminal_median_grid_x(terminal_groups, obstacle_map)
    local_target_x_by_group = _local_pair_target_grid_x_by_group(
        terminal_groups,
        obstacle_map,
        config,
        fallback_x=median_x,
    )

    if config.common_bus_routing_strategy == "local_trunk_then_greedy":
        _route_local_trunks(
            remaining,
            selected,
            unselected,
            routes,
            tree_cells,
            blocked,
            obstacle_map,
            config,
            local_target_x_by_group,
        )

    while remaining:
        candidates: list[_CandidatePath] = []
        for group in remaining.values():
            for terminal in group.terminals:
                forbidden = _forbidden_terminal_cells(
                    obstacle_map,
                    all_terminal_cells,
                    allowed_terminal_ids={terminal.id},
                )
                path = _shortest_path_to_tree(
                    terminal,
                    tree_cells=frozenset(tree_cells),
                    blocked=blocked,
                    forbidden=forbidden,
                    obstacle_map=obstacle_map,
                )
                if path is None:
                    continue
                candidates.append(_CandidatePath(group=group, terminal=terminal, path=path))

        if not candidates:
            break

        best = min(
            candidates,
            key=lambda candidate: (
                _candidate_selection_score(
                    candidate,
                    median_x,
                    local_target_x_by_group,
                    obstacle_map,
                    config,
                ),
                _candidate_target_distance(candidate, median_x, local_target_x_by_group, obstacle_map, config),
                candidate.cost,
                candidate.group.heater_id,
                candidate.terminal.side_key,
                candidate.terminal.id,
                candidate.path,
            ),
        )
        group = best.group
        selected[group.heater_id] = best.terminal
        other_terminal = (
            group.terminal_b if group.terminal_a.id == best.terminal.id else group.terminal_a
        )
        unselected[group.heater_id] = other_terminal
        best_path = cast(tuple[GridCell, ...], best.path)
        routes.append(
            TerminalBusRoute(
                heater_id=group.heater_id,
                terminal=best.terminal,
                path=best_path,
                cost=best.cost,
            )
        )
        tree_cells.update(best_path)
        remaining.pop(group.heater_id)

    return CommonBusRoutingResult(
        bus_side=config.bus_side,
        bus=obstacle_map.bus,
        selected_terminals=selected,
        unselected_terminals=unselected,
        routes=tuple(routes),
        tree_cells=frozenset(tree_cells),
        failed_heaters=tuple(sorted(remaining)),
    )


def _route_local_trunks(
    remaining: dict[str, TerminalPairGroup],
    selected: dict[str, ElectricalTerminal],
    unselected: dict[str, ElectricalTerminal],
    routes: list[TerminalBusRoute],
    tree_cells: set[GridCell],
    blocked: set[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    local_target_x_by_group: dict[str, float],
) -> None:
    pairs = _local_same_row_pairs(tuple(remaining.values()), config)
    used: set[str] = set()
    for left_id, right_id in pairs:
        if left_id in used or right_id in used:
            continue
        left_group = remaining.get(left_id)
        right_group = remaining.get(right_id)
        if left_group is None or right_group is None:
            continue
        left_pair, right_pair = sorted(
            (left_group, right_group),
            key=lambda group: _group_center(group)[0],
        )
        group_pair = (left_pair, right_pair)
        target_grid_x = int(round(
            (
                local_target_x_by_group.get(group_pair[0].heater_id, 0.0)
                + local_target_x_by_group.get(group_pair[1].heater_id, 0.0)
            )
            / 2.0
        ))
        pair_routes = _build_local_trunk_pair_routes(
            group_pair,
            target_grid_x,
            tree_cells=frozenset(tree_cells),
            blocked=blocked,
            obstacle_map=obstacle_map,
            config=config,
        )
        if pair_routes is None:
            continue
        for group, terminal, path in pair_routes:
            selected[group.heater_id] = terminal
            other_terminal = (
                group.terminal_b if group.terminal_a.id == terminal.id else group.terminal_a
            )
            unselected[group.heater_id] = other_terminal
            routes.append(
                TerminalBusRoute(
                    heater_id=group.heater_id,
                    terminal=terminal,
                    path=path,
                    cost=max(0, len(path) - 1),
                )
            )
            tree_cells.update(path)
            remaining.pop(group.heater_id, None)
            used.add(group.heater_id)


def _local_same_row_pairs(
    terminal_groups: tuple[TerminalPairGroup, ...],
    config: ElectricalRoutingConfig,
) -> tuple[tuple[str, str], ...]:
    centers = {group.heater_id: _group_center(group) for group in terminal_groups}
    pairs: list[tuple[float, str, str]] = []
    for group in terminal_groups:
        x, y = centers[group.heater_id]
        candidates: list[tuple[float, str]] = []
        for other in terminal_groups:
            if other.heater_id == group.heater_id:
                continue
            other_x, other_y = centers[other.heater_id]
            gap = abs(other_x - x)
            if gap <= 0 or gap > config.common_bus_local_pair_max_gap_um:
                continue
            if abs(other_y - y) > config.common_bus_local_pair_y_tolerance_um:
                continue
            candidates.append((gap, other.heater_id))
        if not candidates:
            continue
        gap, other_id = min(candidates)
        left_id, right_id = sorted(
            (group.heater_id, other_id),
            key=lambda heater_id: centers[heater_id][0],
        )
        pairs.append((gap, left_id, right_id))
    unique: dict[tuple[str, str], float] = {}
    for gap, left_id, right_id in pairs:
        key = (left_id, right_id)
        unique[key] = min(gap, unique.get(key, gap))
    return tuple(key for key, _ in sorted(unique.items(), key=lambda item: (item[1], item[0])))


def _build_local_trunk_pair_routes(
    group_pair: tuple[TerminalPairGroup, TerminalPairGroup],
    target_grid_x: int,
    *,
    tree_cells: frozenset[GridCell],
    blocked: set[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[tuple[TerminalPairGroup, ElectricalTerminal, tuple[GridCell, ...]], ...] | None:
    terminal_routes: list[tuple[TerminalPairGroup, ElectricalTerminal, tuple[GridCell, ...]]] = []
    arm_endpoints: list[GridCell] = []
    all_terminal_cells = _all_terminal_cells(obstacle_map)
    for group in group_pair:
        terminal = min(
            group.terminals,
            key=lambda candidate: (
                abs(physical_to_grid(candidate.center[0], candidate.center[1], obstacle_map.grid)[0] - target_grid_x),
                candidate.side_key,
                candidate.id,
            ),
        )
        start = _nearest_terminal_cell(terminal, obstacle_map, target_grid_x)
        if start is None:
            return None
        _, start_y = start
        trunk_cell = (target_grid_x, start_y)
        arm = _axis_path(start, trunk_cell)
        allowed_terminal_cells = set(_terminal_open_cells(obstacle_map, terminal.id))
        forbidden = set(all_terminal_cells).difference(allowed_terminal_cells)
        if _path_hits_blockers(
            arm,
            blocked,
            allowed=allowed_terminal_cells,
            forbidden=forbidden,
        ):
            return None
        terminal_routes.append((group, terminal, arm))
        arm_endpoints.append(trunk_cell)

    trunk_y_values = [cell[1] for cell in arm_endpoints]
    tree_targets = tree_cells.difference(all_terminal_cells) or tree_cells
    target_tree_cell = min(tree_targets, key=lambda cell: (abs(cell[0] - target_grid_x), cell[1]))
    trunk_bus_cell = (target_grid_x, target_tree_cell[1])
    trunk_y_values.append(trunk_bus_cell[1])
    trunk_min_y = min(trunk_y_values)
    trunk_max_y = max(trunk_y_values)
    trunk = tuple((target_grid_x, y) for y in range(trunk_min_y, trunk_max_y + 1))
    if _path_hits_blockers(
        trunk,
        blocked,
        allowed=set(arm_endpoints),
        forbidden=set(all_terminal_cells).difference(arm_endpoints),
    ):
        return None
    connector = _axis_path(trunk_bus_cell, target_tree_cell)
    if _path_hits_blockers(
        connector,
        blocked,
        allowed=set(tree_cells),
        forbidden=set(all_terminal_cells),
    ):
        return None

    trunk_and_connector = tuple(dict.fromkeys((*trunk, *connector)))
    result: list[tuple[TerminalPairGroup, ElectricalTerminal, tuple[GridCell, ...]]] = []
    for index, (group, terminal, arm) in enumerate(terminal_routes):
        if index == 0:
            path = tuple(dict.fromkeys((*arm, *trunk_and_connector)))
        else:
            path = tuple(dict.fromkeys((*arm, *trunk)))
        result.append((group, terminal, path))
    return tuple(result)


def _nearest_terminal_cell(
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
    target_grid_x: int,
) -> GridCell | None:
    cells = _terminal_open_cells(obstacle_map, terminal.id)
    if not cells:
        center = physical_to_grid(terminal.center[0], terminal.center[1], obstacle_map.grid)
        return center
    return min(cells, key=lambda cell: (abs(cell[0] - target_grid_x), abs(cell[1]), cell[0], cell[1]))


def _axis_path(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
    x0, y0 = start
    x1, y1 = end
    cells: list[GridCell] = []
    step_x = 1 if x1 >= x0 else -1
    for x in range(x0, x1 + step_x, step_x):
        cells.append((x, y0))
    step_y = 1 if y1 >= y0 else -1
    for y in range(y0 + step_y, y1 + step_y, step_y):
        cells.append((x1, y))
    return tuple(cells)


def _path_hits_blockers(
    path: Iterable[GridCell],
    blocked: set[GridCell],
    *,
    allowed: set[GridCell],
    forbidden: set[GridCell] | frozenset[GridCell] = frozenset(),
) -> bool:
    return any(
        (cell in blocked and cell not in allowed) or cell in forbidden
        for cell in path
    )


def _local_pair_target_grid_x_by_group(
    terminal_groups: tuple[TerminalPairGroup, ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    fallback_x: float,
) -> dict[str, float]:
    if config.common_bus_terminal_selection != "local_pair_median_x_biased":
        return {group.heater_id: fallback_x for group in terminal_groups}

    group_centers = {
        group.heater_id: _group_center(group)
        for group in terminal_groups
    }
    target_by_group: dict[str, float] = {}
    max_gap = config.common_bus_local_pair_max_gap_um
    y_tol = config.common_bus_local_pair_y_tolerance_um
    for group in terminal_groups:
        x, y = group_centers[group.heater_id]
        neighbors: list[tuple[float, str, float]] = []
        for other in terminal_groups:
            if other.heater_id == group.heater_id:
                continue
            other_x, other_y = group_centers[other.heater_id]
            x_gap = abs(other_x - x)
            if x_gap <= 0 or x_gap > max_gap:
                continue
            if abs(other_y - y) > y_tol:
                continue
            neighbors.append((x_gap, other.heater_id, other_x))
        if not neighbors:
            target_by_group[group.heater_id] = fallback_x
            continue
        _, _, neighbor_x = min(neighbors)
        target_x_um = (x + neighbor_x) / 2.0
        target_grid_x, _ = physical_to_grid(target_x_um, y, obstacle_map.grid)
        target_by_group[group.heater_id] = float(target_grid_x)
    return target_by_group


def _group_center(group: TerminalPairGroup) -> tuple[float, float]:
    return (
        (group.terminal_a.center[0] + group.terminal_b.center[0]) / 2.0,
        (group.terminal_a.center[1] + group.terminal_b.center[1]) / 2.0,
    )


def _terminal_median_grid_x(
    terminal_groups: tuple[TerminalPairGroup, ...],
    obstacle_map: ElectricalObstacleMap,
) -> float:
    terminal_grid_xs = [
        physical_to_grid(terminal.center[0], terminal.center[1], obstacle_map.grid)[0]
        for group in terminal_groups
        for terminal in group.terminals
    ]
    if not terminal_grid_xs:
        return 0.0
    return float(median(terminal_grid_xs))


def _candidate_selection_score(
    candidate: _CandidatePath,
    median_x: float,
    local_target_x_by_group: dict[str, float],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> float:
    if config.common_bus_terminal_selection == "path_cost":
        return float(candidate.cost)
    return float(candidate.cost) + (
        config.common_bus_median_bias_weight
        * _candidate_target_distance(
            candidate,
            median_x,
            local_target_x_by_group,
            obstacle_map,
            config,
        )
    )


def _candidate_target_distance(
    candidate: _CandidatePath,
    median_x: float,
    local_target_x_by_group: dict[str, float],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> float:
    if config.common_bus_terminal_selection == "path_cost":
        return 0.0
    if config.common_bus_terminal_selection == "local_pair_median_x_biased":
        target_x = local_target_x_by_group.get(candidate.group.heater_id, median_x)
    else:
        target_x = median_x
    return _candidate_distance_to_target_x(candidate, target_x, obstacle_map)


def _candidate_median_distance(
    candidate: _CandidatePath,
    median_x: float,
    obstacle_map: ElectricalObstacleMap,
) -> float:
    return _candidate_distance_to_target_x(candidate, median_x, obstacle_map)


def _candidate_distance_to_target_x(
    candidate: _CandidatePath,
    target_x: float,
    obstacle_map: ElectricalObstacleMap,
) -> float:
    grid_x, _ = physical_to_grid(
        candidate.terminal.center[0],
        candidate.terminal.center[1],
        obstacle_map.grid,
    )
    return abs(float(grid_x) - target_x)


def _all_terminal_cells(obstacle_map: ElectricalObstacleMap) -> frozenset[GridCell]:
    cells: set[GridCell] = set()
    terminal_open_cells = (
        obstacle_map.common_bus_terminal_open_cells
        or obstacle_map.terminal_open_cells
    )
    for terminal_cells in terminal_open_cells.values():
        cells.update(terminal_cells)
    return frozenset(cells)


def _forbidden_terminal_cells(
    obstacle_map: ElectricalObstacleMap,
    all_terminal_cells: frozenset[GridCell],
    *,
    allowed_terminal_ids: set[str],
) -> frozenset[GridCell]:
    allowed: set[GridCell] = set()
    for terminal_id in allowed_terminal_ids:
        allowed.update(_terminal_open_cells(obstacle_map, terminal_id))
    return frozenset(all_terminal_cells.difference(allowed))


def _shortest_path_to_tree(
    terminal: ElectricalTerminal,
    *,
    tree_cells: frozenset[GridCell],
    blocked: set[GridCell],
    forbidden: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
) -> tuple[GridCell, ...] | None:
    grid = obstacle_map.grid
    terminal_cells = _terminal_open_cells(obstacle_map, terminal.id)
    center_cell = physical_to_grid(terminal.center[0], terminal.center[1], grid)
    start_candidates = {
        cell for cell in terminal_cells if _in_bounds(cell, grid.width, grid.height)
    }
    if not start_candidates and _in_bounds(center_cell, grid.width, grid.height):
        start_candidates.add(center_cell)
    starts = sorted(
        start_candidates,
        key=lambda cell: (_manhattan(cell, center_cell), cell[0], cell[1]),
    )
    if not starts:
        return None

    targets = tree_cells.difference(forbidden)
    if not targets:
        return None
    if any(start in targets for start in starts):
        start = min((cell for cell in starts if cell in targets), key=lambda c: (c[0], c[1]))
        return (start,)

    queue: deque[GridCell] = deque()
    parent: dict[GridCell, GridCell | None] = {}
    for start in starts:
        if start in forbidden:
            continue
        parent[start] = None
        queue.append(start)

    while queue:
        current = queue.popleft()
        for neighbor in _neighbors(current, bus_side=obstacle_map.bus.side):
            if not _in_bounds(neighbor, grid.width, grid.height):
                continue
            if neighbor in parent:
                continue
            if neighbor in forbidden:
                continue
            if neighbor not in targets and neighbor in blocked:
                continue

            parent[neighbor] = current
            if neighbor in targets:
                return _reconstruct_path(parent, neighbor)
            queue.append(neighbor)

    return None


def _neighbors(cell: GridCell, *, bus_side: str) -> tuple[GridCell, GridCell, GridCell, GridCell]:
    x, y = cell
    if bus_side == "bottom":
        return ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1))
    return ((x, y + 1), (x - 1, y), (x + 1, y), (x, y - 1))


def _reconstruct_path(
    parent: dict[GridCell, GridCell | None],
    end: GridCell,
) -> tuple[GridCell, ...]:
    path: list[GridCell] = []
    current: GridCell | None = end
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return tuple(path)


def _in_bounds(cell: GridCell, width: int, height: int) -> bool:
    x, y = cell
    return 0 <= x < width and 0 <= y < height


def _manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> frozenset[GridCell]:
    if obstacle_map.common_bus_terminal_open_cells:
        return obstacle_map.common_bus_terminal_open_cells.get(terminal_id, frozenset())
    return obstacle_map.terminal_open_cells.get(terminal_id, frozenset())
