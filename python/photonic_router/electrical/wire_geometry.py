"""Grid/wire geometry helpers shared by the pad-wire search and river-routing modules."""

from __future__ import annotations

import itertools
import math

from photonic_router.electrical.types import (
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
    """Return the grid cells a centerline polyline passes through, in order.

    ``path`` is in grid coordinates (cell units, cell ``(x, y)`` centered at
    ``(x + 0.5, y + 0.5)``). Each segment is walked with ``grid_segment``, so
    a segment that is not axis-aligned is covered horizontally first, then
    vertically. Consecutive repeated cells are collapsed. An empty path gives
    ``()``; a single point gives its one cell.
    """

    if not path:
        return ()
    cells: list[GridCell] = []
    for start, end in itertools.pairwise(path):
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
    """Return the cell whose center is ``point`` (inverse of ``cell_center``).

    ``point`` is in grid coordinates (cell units). A point that is not a cell
    center is rounded with ``round``, so a point exactly on a cell boundary
    goes to the neighbouring cell with the even index.
    """

    return (round(point[0] - 0.5), round(point[1] - 0.5))


def dedupe_points(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    """Return ``points`` with runs of identical consecutive points collapsed.

    A point that recurs later, after a different point, is kept.
    """

    deduped: list[tuple[float, float]] = []
    for point in points:
        if deduped and deduped[-1] == point:
            continue
        deduped.append(point)
    return tuple(deduped)


def grid_segment(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
    """Return the cells from ``start`` to ``end``, both included.

    Runs along ``start``'s row first, then along ``end``'s column, so cells
    that differ on both axes are joined by an L with one corner cell.
    """

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
    """Return whether ``cell`` lies in a grid of ``width`` by ``height`` cells."""

    x, y = cell
    return 0 <= x < width and 0 <= y < height


def centerline_points(path: tuple[GridCell, ...]) -> tuple[tuple[float, float], ...]:
    """Return a cell path's centerline as its corner points only.

    The points are cell centers in grid coordinates (cell units): the first
    cell, every cell where the step direction changes, and the last cell.
    Cells in the middle of a straight run are dropped.
    """

    simplified = _simplify_manhattan_path(path)
    return tuple(cell_center(cell) for cell in simplified)


def point_direction(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[int, int]:
    """Return the axis direction of the step from ``start`` to ``end``.

    One of ``(-1, 0)``, ``(1, 0)``, ``(0, -1)``, ``(0, 1)``, or ``(0, 0)``
    for identical points. Assumes an axis-aligned segment: when both
    coordinates differ, only the x direction is reported.
    """

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    return (0, 0)


def manhattan(a: GridCell, b: GridCell) -> int:
    """Return the Manhattan distance between two cells, in cells."""

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
    """Return ``cells`` with runs of identical consecutive cells collapsed.

    A cell that recurs later, after a different cell, is kept.
    """

    deduped: list[GridCell] = []
    for cell in cells:
        if deduped and deduped[-1] == cell:
            continue
        deduped.append(cell)
    return tuple(deduped)


def cell_center(cell: GridCell) -> tuple[float, float]:
    """Return a cell's center in grid coordinates (cell units, not um)."""

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
    """Return one terminal's open cells for individual pad-wire routing.

    Looks the terminal up in ``individual_terminal_open_cells(obstacle_map)``;
    an unknown terminal id gives an empty set.
    """

    return individual_terminal_open_cells(obstacle_map).get(
        terminal_id,
        frozenset(),
    )


def individual_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    """Return the per-terminal open cells used for individual pad-wire routing.

    That is the map's ``individual_terminal_open_cells`` when non-empty and
    its shared ``terminal_open_cells`` otherwise, keyed by terminal id. The
    map's own dict is returned, not a copy.
    """

    return obstacle_map.individual_terminal_open_cells or obstacle_map.terminal_open_cells


def terminal_exit_direction_x(terminal_access: ElectricalPortAccess) -> int:
    """Return -1/+1 for a terminal's ``:l``/``:r`` exit side, 0 if unknown."""

    side_key = terminal_access.terminal_id.rsplit(":", 1)[-1]
    return terminal_exit_dx(side_key)
