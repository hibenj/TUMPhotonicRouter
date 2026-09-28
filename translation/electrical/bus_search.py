"""Grid search primitives shared by the common bus's trunk and search phases."""

from __future__ import annotations

from collections import deque
from typing import Iterable

from photonic_router.static_obstacle_builder import physical_to_grid

from .port_access import choose_route_start_cell, ordered_route_start_cells
from .types import ElectricalObstacleMap, ElectricalTerminal, GridCell


def all_terminal_cells(obstacle_map: ElectricalObstacleMap) -> frozenset[GridCell]:
    cells: set[GridCell] = set()
    terminal_open_cells_map = (
        obstacle_map.common_bus_terminal_open_cells or obstacle_map.terminal_open_cells
    )
    for terminal_cells in terminal_open_cells_map.values():
        cells.update(terminal_cells)
    return frozenset(cells)


def forbidden_terminal_cells(
    obstacle_map: ElectricalObstacleMap,
    all_terminal_cells_set: frozenset[GridCell],
    *,
    allowed_terminal_ids: set[str],
) -> frozenset[GridCell]:
    allowed: set[GridCell] = set()
    for terminal_id in allowed_terminal_ids:
        allowed.update(terminal_open_cells(obstacle_map, terminal_id))
    return frozenset(all_terminal_cells_set.difference(allowed))


def terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> frozenset[GridCell]:
    if obstacle_map.common_bus_terminal_open_cells:
        return obstacle_map.common_bus_terminal_open_cells.get(terminal_id, frozenset())
    return obstacle_map.terminal_open_cells.get(terminal_id, frozenset())


def terminal_access_anchor_cell(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> GridCell | None:
    access = _terminal_access(obstacle_map, terminal_id)
    return access.anchor_cell if access is not None else None


def _terminal_access(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
):
    return obstacle_map.common_bus_port_accesses.get(terminal_id)


def route_uses_access_anchor(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
    path: tuple[GridCell, ...],
) -> bool:
    anchor = terminal_access_anchor_cell(obstacle_map, terminal_id)
    return bool(path) and anchor is not None and path[0] == anchor


def nearest_terminal_cell(
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
    target_grid_x: int,
) -> GridCell | None:
    center = physical_to_grid(terminal.center[0], terminal.center[1], obstacle_map.grid)
    choice = choose_route_start_cell(
        access=_terminal_access(obstacle_map, terminal.id),
        opened_cells=terminal_open_cells(obstacle_map, terminal.id),
        grid=obstacle_map.grid,
        fallback_cell=center,
        bias_key=lambda cell: (abs(cell[0] - target_grid_x), abs(cell[1]), cell[0], cell[1]),
        prefer_access_anchor=False,
    )
    return choice.cell if choice is not None else None


def axis_path(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
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


def path_hits_blockers(
    path: Iterable[GridCell],
    blocked: set[GridCell],
    *,
    allowed: set[GridCell],
    forbidden: set[GridCell] | frozenset[GridCell] = frozenset(),
) -> bool:
    return any((cell in blocked and cell not in allowed) or cell in forbidden for cell in path)


def shortest_path_to_tree(
    terminal: ElectricalTerminal,
    *,
    tree_cells: frozenset[GridCell],
    blocked: set[GridCell],
    forbidden: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
) -> tuple[GridCell, ...] | None:
    grid = obstacle_map.grid
    terminal_cells = terminal_open_cells(obstacle_map, terminal.id)
    center_cell = physical_to_grid(terminal.center[0], terminal.center[1], grid)
    starts = ordered_route_start_cells(
        access=_terminal_access(obstacle_map, terminal.id),
        opened_cells=terminal_cells,
        grid=grid,
        fallback_cell=center_cell,
        bias_key=lambda cell: (_manhattan(cell, center_cell), cell[0], cell[1]),
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
