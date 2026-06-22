"""Coarse topology extraction for one-sided individual electrical escape."""

from __future__ import annotations

from collections import deque
from typing import Literal

from .types import (
    CommonBusRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    EscapeBundle,
    EscapeTopologyRoute,
    GridCell,
    IndividualEscapeTopologyResult,
)

Axis = Literal["x", "y"]


def compute_individual_escape_topology(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    config: ElectricalRoutingConfig,
) -> IndividualEscapeTopologyResult:
    """Infer coarse escape corridors before assigning individual pad slots.

    Routes are computed to the configured pad-side boundary, not to fixed pads.
    Overlap is allowed here because this stage is only used to infer bundle
    topology and terminal order for later one-sided pad assignment.
    """

    config.validate()
    if not common_bus.success:
        failed = tuple(
            EscapeTopologyRoute(
                terminal=terminal,
                path=(),
                exit_cell=None,
                success=False,
                reason="common bus routing failed",
            )
            for terminal in common_bus.unselected_terminals.values()
        )
        return IndividualEscapeTopologyResult(
            routes=(),
            failed_routes=failed,
            bundles=(),
            terminal_order=(),
            debug_info={"reason": "common bus routing failed"},
        )

    terminal_cell_set: set[GridCell] = set()
    for cells in obstacle_map.terminal_open_cells.values():
        terminal_cell_set.update(cells)
    terminal_cells = frozenset(terminal_cell_set)
    blocked = set(obstacle_map.blocked_cells)
    blocked.update(common_bus.tree_cells)
    blocked.update(terminal_cells)
    target_cells = _pad_side_boundary_cells(obstacle_map, blocked, config)
    next_cell = _distance_field_to_targets(
        target_cells,
        blocked=blocked,
        width=obstacle_map.grid.width,
        height=obstacle_map.grid.height,
        pad_side=config.pad_side,
    )

    routes: list[EscapeTopologyRoute] = []
    failed_routes: list[EscapeTopologyRoute] = []
    cell_usage: dict[GridCell, int] = {}
    for terminal in _individual_terminals(common_bus):
        source_cells = obstacle_map.terminal_open_cells.get(terminal.id, frozenset())
        path = _best_source_path_to_boundary(
            source_cells=source_cells,
            next_cell=next_cell,
            targets=target_cells,
            blocked=blocked.difference(source_cells),
            width=obstacle_map.grid.width,
            height=obstacle_map.grid.height,
            pad_side=config.pad_side,
            terminal=terminal,
            obstacle_map=obstacle_map,
        )
        if not path:
            failed_routes.append(
                EscapeTopologyRoute(
                    terminal=terminal,
                    path=(),
                    exit_cell=None,
                    success=False,
                    reason="terminal cannot reach pad-side escape boundary",
                )
            )
            continue

        route = EscapeTopologyRoute(
            terminal=terminal,
            path=path,
            exit_cell=path[-1],
            success=True,
        )
        routes.append(route)
        for cell in _route_core_cells(route, obstacle_map, terminal_cells, config):
            cell_usage[cell] = cell_usage.get(cell, 0) + 1

    shared_cells = frozenset(cell for cell, usage in cell_usage.items() if usage > 1)
    bundles = _extract_bundles(
        routes,
        cell_usage=cell_usage,
        terminal_cells=terminal_cells,
        obstacle_map=obstacle_map,
        config=config,
    )
    terminal_order = _terminal_order_from_bundles(bundles, common_bus)
    return IndividualEscapeTopologyResult(
        routes=tuple(routes),
        failed_routes=tuple(failed_routes),
        bundles=bundles,
        terminal_order=terminal_order,
        cell_usage=cell_usage,
        shared_cells=shared_cells,
        debug_info={
            "assignment_strategy": "greedy_order_preserving",
            "order_source": "escape_boundary_and_corridor_order",
        },
    )


def _individual_terminals(common_bus: CommonBusRoutingResult) -> tuple[ElectricalTerminal, ...]:
    return tuple(
        sorted(
            common_bus.unselected_terminals.values(),
            key=lambda terminal: (terminal.center[0], terminal.center[1], terminal.id),
        )
    )


def _pad_side_boundary_cells(
    obstacle_map: ElectricalObstacleMap,
    blocked: set[GridCell],
    config: ElectricalRoutingConfig,
) -> frozenset[GridCell]:
    y = obstacle_map.grid.height - 1 if config.pad_side == "top" else 0
    return frozenset(
        (x, y)
        for x in range(obstacle_map.grid.width)
        if (x, y) not in blocked
    )


def _distance_field_to_targets(
    targets: frozenset[GridCell],
    *,
    blocked: set[GridCell],
    width: int,
    height: int,
    pad_side: str,
) -> dict[GridCell, GridCell | None]:
    queue: deque[GridCell] = deque()
    next_cell: dict[GridCell, GridCell | None] = {}
    for target in sorted(targets):
        next_cell[target] = None
        queue.append(target)

    while queue:
        current = queue.popleft()
        for neighbor in _reverse_neighbors(current, pad_side=pad_side):
            if not _in_bounds(neighbor, width, height):
                continue
            if neighbor in next_cell:
                continue
            if neighbor in blocked:
                continue
            next_cell[neighbor] = current
            queue.append(neighbor)

    return next_cell


def _trace_to_boundary(
    start: GridCell,
    next_cell: dict[GridCell, GridCell | None],
    targets: frozenset[GridCell],
) -> tuple[GridCell, ...]:
    path: list[GridCell] = []
    current: GridCell | None = start
    seen: set[GridCell] = set()
    while current is not None:
        if current in seen:
            return ()
        seen.add(current)
        path.append(current)
        if current in targets:
            return tuple(path)
        current = next_cell.get(current)
    return tuple(path) if path and path[-1] in targets else ()


def _best_source_path_to_boundary(
    *,
    source_cells: frozenset[GridCell],
    next_cell: dict[GridCell, GridCell | None],
    targets: frozenset[GridCell],
    blocked: set[GridCell],
    width: int,
    height: int,
    pad_side: str,
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[GridCell, ...]:
    candidates: list[tuple[tuple[int, int, int, GridCell], tuple[GridCell, ...]]] = []
    for source in source_cells:
        if not _in_bounds(source, width, height) or source in blocked:
            continue
        if source in targets:
            candidates.append((
                _source_candidate_key(source, terminal, obstacle_map, pad_side),
                (source,),
            ))
            continue
        for neighbor in _forward_neighbors(source, pad_side=pad_side):
            if not _in_bounds(neighbor, width, height):
                continue
            if neighbor in blocked:
                continue
            if neighbor not in next_cell and neighbor not in targets:
                continue
            tail = _trace_to_boundary(neighbor, next_cell, targets)
            if not tail:
                continue
            candidates.append((
                _source_candidate_key(source, terminal, obstacle_map, pad_side),
                (source, *tail),
            ))
    if not candidates:
        return ()
    return min(candidates, key=lambda item: (len(item[1]), item[0]))[1]


def _extract_bundles(
    routes: list[EscapeTopologyRoute],
    *,
    cell_usage: dict[GridCell, int],
    terminal_cells: frozenset[GridCell],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[EscapeBundle, ...]:
    core_cells_by_route = {
        route: _route_core_cells(route, obstacle_map, terminal_cells, config)
        for route in routes
    }
    route_graph: dict[EscapeTopologyRoute, set[EscapeTopologyRoute]] = {
        route: set() for route in routes
    }
    for index, route in enumerate(routes):
        cells = core_cells_by_route[route]
        for other in routes[index + 1 :]:
            if not cells.intersection(core_cells_by_route[other]):
                continue
            route_graph[route].add(other)
            route_graph[other].add(route)

    bundles: list[EscapeBundle] = []
    remaining = set(routes)
    while remaining:
        start = min(remaining, key=_route_sort_key)
        remaining.remove(start)
        component = {start}
        queue: deque[EscapeTopologyRoute] = deque([start])
        while queue:
            route = queue.popleft()
            for neighbor in sorted(route_graph[route], key=_route_sort_key):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                component.add(neighbor)
                queue.append(neighbor)

        component_routes = tuple(sorted(component, key=_route_sort_key))
        bundle_cells = frozenset(
            cell
            for route in component_routes
            for cell in core_cells_by_route[route]
        )
        shared_cells = frozenset(
            cell for cell in bundle_cells if cell_usage.get(cell, 0) > 1
        )
        exit_xs = tuple(
            route.exit_cell[0]
            for route in component_routes
            if route.exit_cell is not None
        )
        exit_interval = (
            min(exit_xs, default=0),
            max(exit_xs, default=0),
        )
        order_axis = _bundle_order_axis(bundle_cells)
        ordered_terminals = _ordered_terminals_for_bundle(
            component_routes,
            order_axis=order_axis,
            exit_interval=exit_interval,
            obstacle_map=obstacle_map,
        )
        required_tracks = len(component_routes)
        required_width_um = (
            required_tracks * config.wire_width_um
            + max(0, required_tracks - 1) * config.individual_route_spacing_um
        )
        bundles.append(
            EscapeBundle(
                bundle_id=len(bundles),
                routes=component_routes,
                cells=bundle_cells,
                shared_cells=shared_cells,
                exit_interval=exit_interval,
                ordered_terminals=ordered_terminals,
                order_axis=order_axis,
                required_tracks=required_tracks,
                required_width_um=required_width_um,
            )
        )

    sorted_bundles = tuple(sorted(bundles, key=_bundle_sort_key))
    merged_bundles = _merge_nearby_bundles(sorted_bundles, obstacle_map, config)
    return _renumber_bundles(merged_bundles)


def _merge_nearby_bundles(
    bundles: tuple[EscapeBundle, ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[EscapeBundle, ...]:
    if not bundles:
        return ()
    merge_gap_cells = max(
        1,
        round(
            (config.wire_width_um + config.individual_route_spacing_um)
            / obstacle_map.grid.grid_size_um
        ),
    )
    merged: list[EscapeBundle] = []
    current = bundles[0]
    for bundle in bundles[1:]:
        gap = bundle.exit_interval[0] - current.exit_interval[1]
        if gap <= merge_gap_cells:
            current = _merge_bundle_pair(current, bundle, obstacle_map, config)
            continue
        merged.append(current)
        current = bundle
    merged.append(current)
    return tuple(merged)


def _merge_bundle_pair(
    left: EscapeBundle,
    right: EscapeBundle,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> EscapeBundle:
    routes = tuple(sorted((*left.routes, *right.routes), key=_route_sort_key))
    cells = frozenset((*left.cells, *right.cells))
    shared_cells = frozenset((*left.shared_cells, *right.shared_cells))
    exit_interval = (
        min(left.exit_interval[0], right.exit_interval[0]),
        max(left.exit_interval[1], right.exit_interval[1]),
    )
    order_axis = _bundle_order_axis(cells)
    ordered_terminals = _ordered_terminals_for_bundle(
        routes,
        order_axis=order_axis,
        exit_interval=exit_interval,
        obstacle_map=obstacle_map,
    )
    required_tracks = len(routes)
    required_width_um = (
        required_tracks * config.wire_width_um
        + max(0, required_tracks - 1) * config.individual_route_spacing_um
    )
    return EscapeBundle(
        bundle_id=left.bundle_id,
        routes=routes,
        cells=cells,
        shared_cells=shared_cells,
        exit_interval=exit_interval,
        ordered_terminals=ordered_terminals,
        order_axis=order_axis,
        required_tracks=required_tracks,
        required_width_um=required_width_um,
    )


def _renumber_bundles(bundles: tuple[EscapeBundle, ...]) -> tuple[EscapeBundle, ...]:
    return tuple(
        EscapeBundle(
            bundle_id=index,
            routes=bundle.routes,
            cells=bundle.cells,
            shared_cells=bundle.shared_cells,
            exit_interval=bundle.exit_interval,
            ordered_terminals=bundle.ordered_terminals,
            order_axis=bundle.order_axis,
            required_tracks=bundle.required_tracks,
            required_width_um=bundle.required_width_um,
        )
        for index, bundle in enumerate(bundles)
    )


def _terminal_order_from_bundles(
    bundles: tuple[EscapeBundle, ...],
    common_bus: CommonBusRoutingResult,
) -> tuple[ElectricalTerminal, ...]:
    ordered: list[ElectricalTerminal] = []
    seen: set[str] = set()
    for bundle in sorted(bundles, key=_bundle_sort_key):
        for terminal in bundle.ordered_terminals:
            ordered.append(terminal)
            seen.add(terminal.id)
    for terminal in _individual_terminals(common_bus):
        if terminal.id in seen:
            continue
        ordered.append(terminal)
    return tuple(ordered)


def _ordered_terminals_for_bundle(
    routes: tuple[EscapeTopologyRoute, ...],
    *,
    order_axis: Axis,
    exit_interval: tuple[int, int],
    obstacle_map: ElectricalObstacleMap | None,
) -> tuple[ElectricalTerminal, ...]:
    if order_axis == "y" and obstacle_map is not None:
        route_side = _bundle_route_side_from_routes(routes, exit_interval, obstacle_map)
        reverse = route_side == "right"
    else:
        reverse = False
    return tuple(
        route.terminal
        for route in sorted(
            routes,
            key=lambda route: _terminal_order_key(route.terminal, order_axis),
            reverse=reverse,
        )
    )


def _bundle_route_side_from_routes(
    routes: tuple[EscapeTopologyRoute, ...],
    exit_interval: tuple[int, int],
    obstacle_map: ElectricalObstacleMap,
) -> str:
    terminal_grid_x = _median(
        tuple(
            _terminal_grid_x(route.terminal, obstacle_map)
            for route in routes
        )
    )
    exit_mid = (exit_interval[0] + exit_interval[1]) / 2.0
    return "left" if exit_mid <= terminal_grid_x else "right"


def _route_core_cells(
    route: EscapeTopologyRoute,
    obstacle_map: ElectricalObstacleMap,
    terminal_cells: frozenset[GridCell],
    config: ElectricalRoutingConfig,
) -> frozenset[GridCell]:
    if not route.path:
        return frozenset()
    if config.pad_side == "top":
        boundary_y = obstacle_map.grid.height - 1
    else:
        boundary_y = 0
    return frozenset(
        cell
        for cell in route.path
        if cell not in terminal_cells and cell[1] != boundary_y
    )


def _bundle_order_axis(cells: frozenset[GridCell]) -> Axis:
    if not cells:
        return "x"
    xs = [x for x, _ in cells]
    ys = [y for _, y in cells]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    return "y" if height >= width else "x"


def _bundle_sort_key(bundle: EscapeBundle) -> tuple[float, int, int]:
    exit_mid = (bundle.exit_interval[0] + bundle.exit_interval[1]) / 2.0
    return (exit_mid, bundle.exit_interval[0], bundle.bundle_id)


def _route_sort_key(route: EscapeTopologyRoute) -> tuple[float, float, str]:
    return (route.terminal.center[0], route.terminal.center[1], route.terminal.id)


def _terminal_order_key(terminal: ElectricalTerminal, axis: Axis) -> tuple[float, float, str]:
    if axis == "y":
        return (terminal.center[1], terminal.center[0], terminal.id)
    return (terminal.center[0], terminal.center[1], terminal.id)


def _distance_to_boundary(cell: GridCell, height: int, pad_side: str) -> int:
    _, y = cell
    return height - 1 - y if pad_side == "top" else y


def _source_candidate_key(
    cell: GridCell,
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
    pad_side: str,
) -> tuple[int, int, int, GridCell]:
    return (
        _distance_to_boundary(cell, obstacle_map.grid.height, pad_side),
        abs(cell[0] - _terminal_grid_x(terminal, obstacle_map)),
        cell[1],
        cell,
    )


def _terminal_grid_x(terminal: ElectricalTerminal, obstacle_map: ElectricalObstacleMap) -> int:
    return int((terminal.center[0] - obstacle_map.grid.origin[0]) / obstacle_map.grid.grid_size_um)


def _median(values: tuple[int, ...]) -> float:
    if not values:
        return 0.0
    sorted_values = sorted(values)
    mid = len(sorted_values) // 2
    if len(sorted_values) % 2 == 1:
        return float(sorted_values[mid])
    return (sorted_values[mid - 1] + sorted_values[mid]) / 2.0


def _forward_neighbors(
    cell: GridCell,
    *,
    pad_side: str,
) -> tuple[GridCell, GridCell, GridCell, GridCell]:
    x, y = cell
    if pad_side == "top":
        return ((x, y + 1), (x - 1, y), (x + 1, y), (x, y - 1))
    return ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1))


def _reverse_neighbors(
    cell: GridCell,
    *,
    pad_side: str,
) -> tuple[GridCell, GridCell, GridCell, GridCell]:
    x, y = cell
    if pad_side == "top":
        return ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1))
    return ((x, y + 1), (x - 1, y), (x + 1, y), (x, y - 1))


def _in_bounds(cell: GridCell, width: int, height: int) -> bool:
    x, y = cell
    return 0 <= x < width and 0 <= y < height
