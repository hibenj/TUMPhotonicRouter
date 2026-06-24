"""Explicit electrical terminal access selection for routing-grid anchors."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from typing import Literal, cast

from photonic_router.static_obstacle_builder import GridSpec, physical_to_grid

from .terminal_contacts import (
    port_contact_bbox,
    select_terminal_port_for_side,
)
from .types import (
    ElectricalPortAccess,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    GridCell,
    Point,
    Side,
    TerminalPairGroup,
)


@dataclass(frozen=True)
class RouteStartChoice:
    """Selected routing start cell plus access-anchor provenance."""

    cell: GridCell
    access_anchor_cell: GridCell | None

    @property
    def used_access_anchor(self) -> bool:
        return self.access_anchor_cell is not None and self.cell == self.access_anchor_cell


def build_electrical_port_accesses(
    terminal_groups: tuple[TerminalPairGroup, ...],
    *,
    common_bus_open_cells: dict[str, frozenset[GridCell]],
    individual_open_cells: dict[str, frozenset[GridCell]],
    blocked_cells: frozenset[GridCell],
    grid: object,
    config: ElectricalRoutingConfig,
) -> tuple[dict[str, ElectricalPortAccess], dict[str, ElectricalPortAccess]]:
    """Build one selected access anchor per terminal and electrical purpose."""

    common_bus_accesses: dict[str, ElectricalPortAccess] = {}
    individual_accesses: dict[str, ElectricalPortAccess] = {}
    for group in terminal_groups:
        for terminal in group.terminals:
            common_bus_accesses[terminal.id] = build_terminal_port_access(
                terminal,
                purpose="common_bus",
                side=config.bus_side,
                opened_cells=common_bus_open_cells.get(terminal.id, frozenset()),
                blocked_cells=blocked_cells,
                grid=grid,
                fallback_width_um=max(
                    config.bus_width_um,
                    config.terminal_contact_width_um,
                ),
            )
            individual_accesses[terminal.id] = build_terminal_port_access(
                terminal,
                purpose="individual",
                side=config.pad_side,
                opened_cells=individual_open_cells.get(terminal.id, frozenset()),
                blocked_cells=blocked_cells,
                grid=grid,
                fallback_width_um=config.terminal_contact_width_um,
            )
    return common_bus_accesses, individual_accesses


def choose_route_start_cell(
    *,
    access: ElectricalPortAccess | None,
    opened_cells: frozenset[GridCell],
    grid: object,
    blocked_cells: frozenset[GridCell] | set[GridCell] = frozenset(),
    fallback_cell: GridCell | None = None,
    bias_key: Callable[[GridCell], tuple[object, ...]] | None = None,
    prefer_access_anchor: bool = True,
) -> RouteStartChoice | None:
    """Choose a legal route start, preferring explicit access metadata."""

    candidates = {
        cell
        for cell in opened_cells
        if _in_bounds(cell, grid) and cell not in blocked_cells
    }
    access_anchor = access.anchor_cell if access is not None else None
    if access_anchor is not None and _in_bounds(access_anchor, grid):
        candidates.add(access_anchor)
    if not candidates and fallback_cell is not None and _in_bounds(fallback_cell, grid):
        candidates.add(fallback_cell)
    if not candidates:
        return None

    def key(cell: GridCell) -> tuple[object, ...]:
        access_rank = 0 if cell == access_anchor else 1
        bias = bias_key(cell) if bias_key is not None else ()
        fallback_rank = (
            _manhattan(cell, fallback_cell)
            if fallback_cell is not None
            else 0
        )
        if prefer_access_anchor:
            return (access_rank, *bias, fallback_rank, cell[0], cell[1])
        return (*bias, access_rank, fallback_rank, cell[0], cell[1])

    return RouteStartChoice(
        cell=min(candidates, key=key),
        access_anchor_cell=access_anchor,
    )


def ordered_route_start_cells(
    *,
    access: ElectricalPortAccess | None,
    opened_cells: frozenset[GridCell],
    grid: object,
    blocked_cells: frozenset[GridCell] | set[GridCell] = frozenset(),
    fallback_cell: GridCell | None = None,
    bias_key: Callable[[GridCell], tuple[object, ...]] | None = None,
    prefer_access_anchor: bool = True,
) -> tuple[GridCell, ...]:
    """Return legal route start candidates in the shared access order."""

    candidates = {
        cell
        for cell in opened_cells
        if _in_bounds(cell, grid) and cell not in blocked_cells
    }
    access_anchor = access.anchor_cell if access is not None else None
    if access_anchor is not None and _in_bounds(access_anchor, grid):
        candidates.add(access_anchor)
    if not candidates and fallback_cell is not None and _in_bounds(fallback_cell, grid):
        candidates.add(fallback_cell)

    def key(cell: GridCell) -> tuple[object, ...]:
        access_rank = 0 if cell == access_anchor else 1
        bias = bias_key(cell) if bias_key is not None else ()
        fallback_rank = (
            _manhattan(cell, fallback_cell)
            if fallback_cell is not None
            else 0
        )
        if prefer_access_anchor:
            return (access_rank, *bias, fallback_rank, cell[0], cell[1])
        return (*bias, access_rank, fallback_rank, cell[0], cell[1])

    return tuple(sorted(candidates, key=key))


def build_terminal_port_access(
    terminal: ElectricalTerminal,
    *,
    purpose: Literal["common_bus", "individual"],
    side: Side,
    opened_cells: frozenset[GridCell],
    blocked_cells: frozenset[GridCell],
    grid: object,
    fallback_width_um: float,
) -> ElectricalPortAccess:
    """Select a deterministic legal electrical-grid anchor for one terminal."""

    port = select_terminal_port_for_side(terminal, side)
    port_point = port.center if port is not None else terminal.center
    port_orientation = port.orientation if port is not None else None
    port_width = port.width if port is not None else None
    port_name = port.name if port is not None else None
    contact_bbox = (
        port_contact_bbox(port, fallback_width_um)
        if port is not None
        else _fallback_contact_bbox(port_point, fallback_width_um)
    )
    anchor_cell = select_anchor_cell(
        port_point,
        side=side,
        opened_cells=opened_cells,
        blocked_cells=blocked_cells,
        grid=grid,
    )
    anchor_point = _grid_cell_center_um(anchor_cell, grid)
    centerline = _dedupe_points(_manhattan_centerline(port_point, anchor_point, side))
    return ElectricalPortAccess(
        terminal_id=terminal.id,
        heater_id=terminal.heater_id,
        purpose=purpose,
        side=side,
        port_name=port_name,
        port_point_um=port_point,
        port_orientation_deg=port_orientation,
        port_width_um=port_width,
        contact_bbox=contact_bbox,
        anchor_cell=anchor_cell,
        anchor_point_um=anchor_point,
        access_centerline_um=centerline,
        opened_cells=opened_cells,
        reserved_cells=frozenset({anchor_cell}),
        access_length_um=_polyline_length(centerline),
    )


def select_anchor_cell(
    port_point_um: Point,
    *,
    side: Side,
    opened_cells: frozenset[GridCell],
    blocked_cells: frozenset[GridCell],
    grid: object,
) -> GridCell:
    """Select the preferred legal anchor from already-open terminal cells."""

    seed_cell = physical_to_grid(
        port_point_um[0],
        port_point_um[1],
        cast(GridSpec, grid),
    )
    legal_cells = tuple(
        cell
        for cell in opened_cells
        if _in_bounds(cell, grid) and cell not in blocked_cells
    )
    if legal_cells:
        return min(legal_cells, key=lambda cell: _anchor_cell_key(cell, seed_cell, side))
    bounded_open_cells = tuple(cell for cell in opened_cells if _in_bounds(cell, grid))
    if bounded_open_cells:
        return min(
            bounded_open_cells,
            key=lambda cell: _anchor_cell_key(cell, seed_cell, side),
        )
    if _in_bounds(seed_cell, grid):
        return seed_cell
    return (
        min(max(seed_cell[0], 0), int(getattr(grid, "width")) - 1),
        min(max(seed_cell[1], 0), int(getattr(grid, "height")) - 1),
    )


def _anchor_cell_key(
    cell: GridCell,
    seed_cell: GridCell,
    side: Side,
) -> tuple[int, int, int, int]:
    side_rank = -cell[1] if side == "top" else cell[1]
    return (
        abs(cell[0] - seed_cell[0]) + abs(cell[1] - seed_cell[1]),
        side_rank,
        cell[0],
        cell[1],
    )


def _manhattan(a: GridCell, b: GridCell | None) -> int:
    if b is None:
        return 0
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _grid_cell_center_um(cell: GridCell, grid: object) -> Point:
    origin_x, origin_y = getattr(grid, "origin")
    grid_size = float(getattr(grid, "grid_size_um"))
    return (
        float(origin_x) + (cell[0] + 0.5) * grid_size,
        float(origin_y) + (cell[1] + 0.5) * grid_size,
    )


def _manhattan_centerline(start: Point, end: Point, side: Side) -> tuple[Point, ...]:
    if start == end:
        return (start,)
    if start[0] == end[0] or start[1] == end[1]:
        return (start, end)
    corner = (start[0], end[1]) if side in {"top", "bottom"} else (end[0], start[1])
    return (start, corner, end)


def _dedupe_points(points: tuple[Point, ...]) -> tuple[Point, ...]:
    deduped: list[Point] = []
    for point in points:
        if not deduped or point != deduped[-1]:
            deduped.append(point)
    return tuple(deduped)


def _polyline_length(points: tuple[Point, ...]) -> float:
    return sum(
        math.hypot(right[0] - left[0], right[1] - left[1])
        for left, right in zip(points, points[1:])
    )


def _fallback_contact_bbox(point: Point, width_um: float) -> tuple[float, float, float, float]:
    half_width = max(float(width_um), 0.0) / 2.0
    return (
        point[0] - half_width,
        point[1] - half_width,
        point[0] + half_width,
        point[1] + half_width,
    )


def _in_bounds(cell: GridCell, grid: object) -> bool:
    return (
        0 <= cell[0] < int(getattr(grid, "width"))
        and 0 <= cell[1] < int(getattr(grid, "height"))
    )
