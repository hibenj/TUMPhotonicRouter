"""Escape routing from the common bus tree to its assigned external pad slot."""

from __future__ import annotations

from collections import deque

from .pad_slots import pad_access_cells
from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    GridCell,
    PadPlan,
)


def route_common_bus_escape(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    pad_plan: PadPlan,
    config: ElectricalRoutingConfig,
) -> CommonBusEscapeResult:
    """Route the common bus tree to its assigned abstract pad slot."""

    config.validate()
    assignment = pad_plan.common_bus_assignment
    if assignment is None:
        return CommonBusEscapeResult(
            pad_assignment=None,
            path=(),
            target_cells=frozenset(),
            success=False,
            reason="pad plan has no common-bus assignment",
        )
    if not common_bus.success:
        return CommonBusEscapeResult(
            pad_assignment=assignment,
            path=(),
            target_cells=frozenset(),
            success=False,
            reason=f"common bus has failed heaters: {common_bus.failed_heaters}",
        )

    target_cells = pad_access_cells(
        assignment.slot,
        obstacle_map,
        config,
        width_um=config.bus_width_um,
    )
    if not target_cells:
        return CommonBusEscapeResult(
            pad_assignment=assignment,
            path=(),
            target_cells=frozenset(),
            success=False,
            reason="common-bus pad slot is outside the electrical routing grid",
        )

    blocked = set(obstacle_map.blocked_cells)
    blocked.difference_update(common_bus.tree_cells)
    blocked.difference_update(target_cells)
    path = _endpoint_dogleg_to_targets(
        common_bus,
        targets=frozenset(target_cells),
        blocked=blocked,
        width=obstacle_map.grid.width,
        height=obstacle_map.grid.height,
        pad_side=config.pad_side,
        pad_position=config.common_bus_pad_position,
    )
    if path is None:
        path = _shortest_tree_to_targets(
            starts=common_bus.tree_cells,
            targets=target_cells,
            blocked=blocked,
            width=obstacle_map.grid.width,
            height=obstacle_map.grid.height,
            pad_side=config.pad_side,
        )
    if path is None:
        return CommonBusEscapeResult(
            pad_assignment=assignment,
            path=(),
            target_cells=frozenset(target_cells),
            success=False,
            reason="no legal path from common bus tree to common-bus pad slot",
        )

    return CommonBusEscapeResult(
        pad_assignment=assignment,
        path=path,
        target_cells=frozenset(target_cells),
        success=True,
        reason=None,
    )


def _endpoint_dogleg_to_targets(
    common_bus: CommonBusRoutingResult,
    *,
    targets: frozenset[GridCell],
    blocked: set[GridCell],
    width: int,
    height: int,
    pad_side: str,
    pad_position: str,
) -> tuple[GridCell, ...] | None:
    if not common_bus.bus.cells or not targets:
        return None
    endpoint_x = (
        min(x for x, _ in common_bus.bus.cells)
        if pad_position == "left"
        else max(x for x, _ in common_bus.bus.cells)
    )
    endpoint_cells = tuple(
        cell for cell in common_bus.bus.cells if cell[0] == endpoint_x
    )
    target = _choose_dogleg_target(targets, pad_side)
    bus_center_y = (
        min(y for _, y in common_bus.bus.cells)
        + max(y for _, y in common_bus.bus.cells)
    ) / 2.0
    start = min(
        endpoint_cells,
        key=lambda cell: (abs(cell[1] - bus_center_y), cell[1], cell[0]),
    )
    corner = (target[0], start[1])
    path = _axis_path(start, corner) + _axis_path(corner, target)[1:]
    if not all(_in_bounds(cell, width, height) for cell in path):
        return None
    allowed = set(common_bus.tree_cells) | set(targets)
    if any(cell in blocked and cell not in allowed for cell in path):
        return None
    return path


def _choose_dogleg_target(
    targets: frozenset[GridCell],
    pad_side: str,
) -> GridCell:
    if pad_side == "top":
        target_y = min(y for _, y in targets)
    else:
        target_y = max(y for _, y in targets)
    candidates = tuple(cell for cell in targets if cell[1] == target_y)
    target_x_mid = (min(x for x, _ in targets) + max(x for x, _ in targets)) / 2.0
    return min(
        candidates,
        key=lambda cell: (abs(cell[0] - target_x_mid), cell[0], cell[1]),
    )


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


def _shortest_tree_to_targets(
    *,
    starts: frozenset[GridCell],
    targets: frozenset[GridCell],
    blocked: set[GridCell],
    width: int,
    height: int,
    pad_side: str,
) -> tuple[GridCell, ...] | None:
    if not starts or not targets:
        return None
    in_target = starts.intersection(targets)
    if in_target:
        cell = min(in_target)
        return (cell,)

    queue: deque[GridCell] = deque()
    parent: dict[GridCell, GridCell | None] = {}
    for start in sorted(starts, key=lambda cell: _target_distance_key(cell, targets, pad_side)):
        if not _in_bounds(start, width, height):
            continue
        parent[start] = None
        queue.append(start)

    while queue:
        current = queue.popleft()
        for neighbor in _neighbors(current, pad_side=pad_side):
            if not _in_bounds(neighbor, width, height):
                continue
            if neighbor in parent:
                continue
            if neighbor not in targets and neighbor in blocked:
                continue
            parent[neighbor] = current
            if neighbor in targets:
                return _reconstruct_path(parent, neighbor)
            queue.append(neighbor)

    return None


def _neighbors(cell: GridCell, *, pad_side: str) -> tuple[GridCell, GridCell, GridCell, GridCell]:
    x, y = cell
    if pad_side == "top":
        return ((x, y + 1), (x - 1, y), (x + 1, y), (x, y - 1))
    return ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1))


def _target_distance_key(cell: GridCell, targets: frozenset[GridCell], pad_side: str) -> tuple[int, int, int]:
    x, y = cell
    if pad_side == "top":
        primary = max(target_y for _, target_y in targets) - y
    else:
        primary = y - min(target_y for _, target_y in targets)
    target_x_min = min(target_x for target_x, _ in targets)
    target_x_max = max(target_x for target_x, _ in targets)
    if x < target_x_min:
        x_distance = target_x_min - x
    elif x > target_x_max:
        x_distance = x - target_x_max
    else:
        x_distance = 0
    return (abs(primary), x_distance, x)


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
