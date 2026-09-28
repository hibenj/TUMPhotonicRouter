"""Grid/wire geometry helpers shared by the pad-wire search and river-routing modules."""

from __future__ import annotations

import math

from .types import (
    ElectricalObstacleMap,
    ElectricalPortAccess,
    ElectricalRoutingConfig,
    GridCell,
    terminal_exit_dx,
)


def wire_reservation_radius_cells(
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    """Return the obstacle-clearance reservation radius, in cells.

    Dilating a wire's centerline by this radius and testing it against the
    obstacle map (or another net's opening/pad cells) holds the wire
    ``wire_width_um + obstacle_clearance_um`` away from them.
    """

    required_center_spacing_um = config.wire_width_um + max(
        config.obstacle_clearance_um,
        0.0,
    )
    return max(
        0,
        math.ceil(required_center_spacing_um / obstacle_map.grid.grid_size_um) - 1,
    )


def wire_spacing_radius_cells(
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> int:
    """Return the wire-to-wire reservation radius, in cells.

    Dilating one wire's centerline by this radius and testing another wire's
    bare centerline against it holds the two centerlines
    ``wire_width_um + individual_route_spacing_um`` apart (and at least the
    obstacle clearance too, so a committed wire's footprint never invites an
    obstacle-clearance violation either). Used to build
    ``committed_footprint_cells``, the reservation every later wire -- river
    or fallback -- must stay clear of.
    """

    required_center_spacing_um = config.wire_width_um + max(
        config.obstacle_clearance_um,
        config.individual_route_spacing_um,
        0.0,
    )
    return max(
        0,
        math.ceil(required_center_spacing_um / obstacle_map.grid.grid_size_um) - 1,
    )


def dilate_cells(
    cells: set[GridCell],
    radius: int,
    width: int,
    height: int,
) -> set[GridCell]:
    """Return ``cells`` expanded by ``radius`` in every direction (Chebyshev)."""

    if radius <= 0:
        return set(cells)
    dilated: set[GridCell] = set()
    for x, y in cells:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (x + dx, y + dy)
                if in_bounds(cell, width, height):
                    dilated.add(cell)
    return dilated


def wire_reservation_cells_from_point_path(
    path: tuple[tuple[float, float], ...],
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
    *,
    radius: int,
) -> frozenset[GridCell]:
    """Return ``path``'s centerline dilated by ``radius`` cells (Chebyshev).

    ``radius`` is always one of the two reservation radii above, passed
    explicitly by the caller: ``wire_reservation_radius_cells`` (the
    obstacle-clearance radius) when checking against the obstacle map, or
    ``wire_spacing_radius_cells`` (the wire-to-wire radius) when building a
    footprint other wires must stay clear of.
    """

    centerline = cells_from_point_path(path)
    cells: set[GridCell] = set()
    for x, y in centerline:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (x + dx, y + dy)
                if in_bounds(cell, obstacle_map.grid.width, obstacle_map.grid.height):
                    cells.add(cell)
    return frozenset(cells)


def cells_from_point_path(path: tuple[tuple[float, float], ...]) -> tuple[GridCell, ...]:
    if not path:
        return ()
    cells: list[GridCell] = []
    for start, end in zip(path, path[1:]):
        start_cell = point_to_cell(start)
        end_cell = point_to_cell(end)
        segment = grid_segment(start_cell, end_cell)
        if cells:
            cells.extend(segment[1:])
        else:
            cells.extend(segment)
    if not cells:
        cells.append(point_to_cell(path[0]))
    return dedupe_path(tuple(cells))


def point_to_cell(point: tuple[float, float]) -> GridCell:
    return (round(point[0] - 0.5), round(point[1] - 0.5))


def dedupe_points(
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


def in_bounds(cell: GridCell, width: int, height: int) -> bool:
    x, y = cell
    return 0 <= x < width and 0 <= y < height


def centerline_points(path: tuple[GridCell, ...]) -> tuple[tuple[float, float], ...]:
    simplified = _simplify_manhattan_path(path)
    return tuple(cell_center(cell) for cell in simplified)


def point_direction(
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


def manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _direction(start: GridCell, end: GridCell) -> tuple[int, int]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    return (0, 0)


def dedupe_path(cells: tuple[GridCell, ...]) -> tuple[GridCell, ...]:
    deduped: list[GridCell] = []
    for cell in cells:
        if deduped and deduped[-1] == cell:
            continue
        deduped.append(cell)
    return tuple(deduped)


def cell_center(cell: GridCell) -> tuple[float, float]:
    return (cell[0] + 0.5, cell[1] + 0.5)


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


def terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
    terminal_id: str,
) -> frozenset[GridCell]:
    return individual_terminal_open_cells(obstacle_map).get(
        terminal_id,
        frozenset(),
    )


def individual_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    return obstacle_map.individual_terminal_open_cells or obstacle_map.terminal_open_cells


def terminal_exit_direction_x(terminal_access: ElectricalPortAccess) -> int:
    """Return -1/+1 for a terminal's ``:l``/``:r`` exit side, 0 if unknown."""

    side_key = terminal_access.terminal_id.rsplit(":", 1)[-1]
    return terminal_exit_dx(side_key)
