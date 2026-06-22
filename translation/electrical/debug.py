"""Debug SVG exporters for electrical routing milestones."""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Iterable

from photonic_router.static_obstacle_builder import GridSpec

from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    GridCell,
    IndividualEscapeTopologyResult,
    PadPlan,
    TerminalPairGroup,
)

_INDIVIDUAL_ROUTE_PALETTE = (
    "#006d77",
    "#ef476f",
    "#118ab2",
    "#06d6a0",
    "#f77f00",
    "#8338ec",
    "#2a9d8f",
    "#e76f51",
    "#457b9d",
    "#9b5de5",
)

_DEBUG_ROUTE_OFFSETS = (
    (0.0, 0.0),
    (0.22, 0.0),
    (-0.22, 0.0),
    (0.0, 0.22),
    (0.0, -0.22),
    (0.16, 0.16),
    (-0.16, 0.16),
    (0.16, -0.16),
    (-0.16, -0.16),
)


def export_electrical_debug_svg(
    path: str | Path,
    obstacle_map: ElectricalObstacleMap,
    terminal_groups: tuple[TerminalPairGroup, ...],
    common_bus: CommonBusRoutingResult | None = None,
    common_bus_escape: CommonBusEscapeResult | None = None,
    individual_topology: IndividualEscapeTopologyResult | None = None,
    detailed_bundle_routes: DetailedBundleRoutingResult | None = None,
    pad_plan: PadPlan | None = None,
    *,
    max_cell_px: int = 8,
) -> None:
    """Write an SVG showing obstacles, terminal openings, bus, and bus routes."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        electrical_debug_svg(
            obstacle_map,
            terminal_groups,
            common_bus,
            common_bus_escape,
            individual_topology,
            detailed_bundle_routes,
            pad_plan,
            max_cell_px=max_cell_px,
        ),
        encoding="utf-8",
    )


def electrical_debug_svg(
    obstacle_map: ElectricalObstacleMap,
    terminal_groups: tuple[TerminalPairGroup, ...],
    common_bus: CommonBusRoutingResult | None = None,
    common_bus_escape: CommonBusEscapeResult | None = None,
    individual_topology: IndividualEscapeTopologyResult | None = None,
    detailed_bundle_routes: DetailedBundleRoutingResult | None = None,
    pad_plan: PadPlan | None = None,
    *,
    max_cell_px: int = 8,
) -> str:
    grid = obstacle_map.grid
    if grid.width <= 0 or grid.height <= 0:
        return '<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1" />\n'

    cell_px = max(1, min(max_cell_px, 1600 // max(grid.width, grid.height, 1)))
    width_px = grid.width * cell_px
    height_px = grid.height * cell_px

    parts: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" '
        f'viewBox="0 0 {grid.width} {grid.height}">',
        '<rect width="100%" height="100%" fill="#f7f7f2" />',
    ]

    _append_cells(parts, grid.height, obstacle_map.blocked_cells, fill="#222222", opacity=0.85)
    _append_cells(parts, grid.height, obstacle_map.bus.cells, fill="#4f8fd9", opacity=0.9)

    if pad_plan is not None:
        assignment_order = {
            assignment.terminal.id: index
            for index, assignment in enumerate(
                sorted(
                    (
                        assignment
                        for assignment in pad_plan.assignments
                        if assignment.kind == "individual" and assignment.terminal is not None
                    ),
                    key=lambda assignment: assignment.slot.index,
                )
            )
        }
        for assignment in pad_plan.assignments:
            fill = "#d7a600" if assignment.kind == "common_bus" else "#ffd966"
            _append_physical_rect(
                parts,
                grid,
                assignment.slot.bbox,
                fill=fill,
                opacity=0.22,
                stroke="#8a5a00",
                stroke_width=0.12,
            )
            if assignment.kind == "common_bus":
                label = "bus"
            elif assignment.terminal is not None:
                label = str(assignment_order.get(assignment.terminal.id, "?"))
            else:
                label = "?"
            _append_physical_text(
                parts,
                grid,
                assignment.slot.center,
                label,
                font_size=2.6,
                fill="#3f2d00",
            )

    if common_bus is not None:
        route_cells: set[GridCell] = set()
        for route in common_bus.routes:
            route_cells.update(route.path)
        _append_cells(parts, grid.height, route_cells, fill="#f28c28", opacity=0.95)
        _append_cells(parts, grid.height, common_bus.tree_cells, fill="#33a853", opacity=0.35)

    if common_bus_escape is not None:
        _append_cells(parts, grid.height, common_bus_escape.target_cells, fill="#8ab4f8", opacity=0.7)
        _append_cells(parts, grid.height, common_bus_escape.path, fill="#a142f4", opacity=0.95)

    if detailed_bundle_routes is not None:
        detail_cells: set[GridCell] = set()
        detail_target_cells: set[GridCell] = set()
        sorted_routes = sorted(
            detailed_bundle_routes.routes,
            key=lambda route: (
                route.bundle_id,
                route.rank,
                route.pad_assignment.slot.index if route.pad_assignment is not None else -1,
                route.terminal.id,
            ),
        )
        for route in sorted_routes:
            detail_cells.update(route.path)
            detail_target_cells.update(route.target_cells)
        _append_cells(parts, grid.height, detail_target_cells, fill="#fdd663", opacity=0.75)
        _append_cells(parts, grid.height, detail_cells, fill="#2a9d8f", opacity=0.12)
        for route in sorted_routes:
            color = _INDIVIDUAL_ROUTE_PALETTE[route.bundle_id % len(_INDIVIDUAL_ROUTE_PALETTE)]
            title = (
                f"detailed bundle={route.bundle_id} rank={route.rank} "
                f"lane={route.rank} offset={route.offset_um:.3g}um axis=ordered-bus "
                f"pad={route.pad_assignment.slot.index if route.pad_assignment is not None else 'none'} "
                f"terminal={route.terminal.id}"
            )
            if route.offset_path:
                _append_point_polyline(
                    parts,
                    grid.height,
                    route.offset_path,
                    stroke=color,
                    stroke_width=0.24,
                    opacity=0.95,
                    title=title,
                )
                if route.bundle_track_path:
                    _append_grid_text(
                        parts,
                        grid.height,
                        route.bundle_track_path[0],
                        f"L{route.rank}",
                        font_size=1.35,
                        fill=color,
                        opacity=0.65,
                    )
                    _append_grid_text(
                        parts,
                        grid.height,
                        route.bundle_track_path[-1],
                        f"L{route.rank}",
                        font_size=1.35,
                        fill=color,
                        opacity=0.65,
                    )
            else:
                _append_route_polyline(
                    parts,
                    grid.height,
                    route.path,
                    stroke=color,
                    stroke_width=0.24,
                    opacity=0.95,
                    offset=(0.0, 0.0),
                    title=title,
                )

    elif individual_topology is not None:
        _append_cells(parts, grid.height, individual_topology.shared_cells, fill="#5f0f40", opacity=0.24)
        routes_by_terminal_id = {route.terminal.id: route for route in individual_topology.routes}
        for bundle in individual_topology.bundles:
            color = _INDIVIDUAL_ROUTE_PALETTE[bundle.bundle_id % len(_INDIVIDUAL_ROUTE_PALETTE)]
            for rank, terminal in enumerate(bundle.ordered_terminals):
                route = routes_by_terminal_id.get(terminal.id)
                if route is None:
                    continue
                offset = _DEBUG_ROUTE_OFFSETS[rank % len(_DEBUG_ROUTE_OFFSETS)]
                _append_route_polyline(
                    parts,
                    grid.height,
                    route.path,
                    stroke=color,
                    stroke_width=0.28,
                    opacity=0.45,
                    offset=offset,
                    title=(
                        f"topology bundle={bundle.bundle_id} rank={rank} "
                        f"tracks={bundle.required_tracks} terminal={terminal.id}"
                    ),
                    dasharray="1.2 0.8",
                )

    for terminal_id, cells in sorted(obstacle_map.terminal_open_cells.items()):
        fill = "#d93025"
        if common_bus is not None:
            selected_ids = {terminal.id for terminal in common_bus.selected_terminals.values()}
            unselected_ids = {terminal.id for terminal in common_bus.unselected_terminals.values()}
            if terminal_id in selected_ids:
                fill = "#34a853"
            elif terminal_id in unselected_ids:
                fill = "#ea4335"
        _append_cells(parts, grid.height, cells, fill=fill, opacity=0.55)

    terminal_order_index = (
        {terminal.id: index for index, terminal in enumerate(individual_topology.terminal_order)}
        if individual_topology is not None
        else {}
    )
    for group in terminal_groups:
        for terminal in group.terminals:
            gx = int((terminal.center[0] - grid.origin[0]) / grid.grid_size_um)
            gy = int((terminal.center[1] - grid.origin[1]) / grid.grid_size_um)
            if not (0 <= gx < grid.width and 0 <= gy < grid.height):
                continue
            svg_y = grid.height - gy - 1
            label = terminal.side_key
            if terminal.id in terminal_order_index:
                label = f"{terminal_order_index[terminal.id]}"
            parts.append(
                f'<text x="{gx + 0.5}" y="{svg_y + 0.5}" font-size="2" '
                f'text-anchor="middle" dominant-baseline="middle" fill="#000">'
                f'{escape(label)}</text>'
            )

    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def _append_cells(
    parts: list[str],
    grid_height: int,
    cells: Iterable[GridCell],
    *,
    fill: str,
    opacity: float,
) -> None:
    for gx, gy in sorted(cells):
        svg_y = grid_height - gy - 1
        parts.append(
            f'<rect x="{gx}" y="{svg_y}" width="1" height="1" '
            f'fill="{fill}" opacity="{opacity}" />'
        )


def _append_physical_rect(
    parts: list[str],
    grid: GridSpec,
    bbox: tuple[float, float, float, float],
    *,
    fill: str,
    opacity: float,
    stroke: str,
    stroke_width: float,
) -> None:
    xmin, ymin, xmax, ymax = bbox
    origin_x, origin_y = grid.origin
    grid_size = grid.grid_size_um
    x = (xmin - origin_x) / grid_size
    y_top = grid.height - (ymax - origin_y) / grid_size
    width = (xmax - xmin) / grid_size
    height = (ymax - ymin) / grid_size
    parts.append(
        f'<rect x="{x:.6g}" y="{y_top:.6g}" width="{width:.6g}" height="{height:.6g}" '
        f'fill="{fill}" opacity="{opacity}" stroke="{stroke}" '
        f'stroke-width="{stroke_width}" vector-effect="non-scaling-stroke" />'
    )


def _append_physical_text(
    parts: list[str],
    grid: GridSpec,
    point: tuple[float, float],
    label: str,
    *,
    font_size: float,
    fill: str,
) -> None:
    origin_x, origin_y = grid.origin
    grid_size = grid.grid_size_um
    x = (point[0] - origin_x) / grid_size
    y = grid.height - (point[1] - origin_y) / grid_size
    parts.append(
        f'<text x="{x:.6g}" y="{y:.6g}" font-size="{font_size}" '
        f'text-anchor="middle" dominant-baseline="middle" fill="{fill}">'
        f'{escape(label)}</text>'
    )


def _append_grid_text(
    parts: list[str],
    grid_height: int,
    point: tuple[float, float],
    label: str,
    *,
    font_size: float,
    fill: str,
    opacity: float,
) -> None:
    x, y = point
    parts.append(
        f'<text x="{x:.6g}" y="{grid_height - y:.6g}" font-size="{font_size}" '
        f'text-anchor="middle" dominant-baseline="middle" fill="{fill}" '
        f'opacity="{opacity}">{escape(label)}</text>'
    )


def _append_route_polyline(
    parts: list[str],
    grid_height: int,
    cells: tuple[GridCell, ...],
    *,
    stroke: str,
    stroke_width: float,
    opacity: float,
    offset: tuple[float, float],
    title: str,
    dasharray: str | None = None,
) -> None:
    if not cells:
        return
    offset_x, offset_y = offset
    points = " ".join(
        f"{gx + 0.5 + offset_x:.3f},{grid_height - gy - 0.5 + offset_y:.3f}"
        for gx, gy in cells
    )
    dash_attr = f' stroke-dasharray="{dasharray}"' if dasharray is not None else ""
    parts.append(
        f'<polyline points="{points}" fill="none" stroke="{stroke}" '
        f'stroke-width="{stroke_width}" opacity="{opacity}" '
        f'stroke-linecap="round" stroke-linejoin="round" '
        f'vector-effect="non-scaling-stroke"{dash_attr}>'
        f"<title>{escape(title)}</title></polyline>"
    )


def _append_point_polyline(
    parts: list[str],
    grid_height: int,
    points_grid: tuple[tuple[float, float], ...],
    *,
    stroke: str,
    stroke_width: float,
    opacity: float,
    title: str,
) -> None:
    if not points_grid:
        return
    points = " ".join(
        f"{x:.3f},{grid_height - y:.3f}"
        for x, y in points_grid
    )
    parts.append(
        f'<polyline points="{points}" fill="none" stroke="{stroke}" '
        f'stroke-width="{stroke_width}" opacity="{opacity}" '
        f'stroke-linecap="round" stroke-linejoin="round" '
        f'vector-effect="non-scaling-stroke">'
        f"<title>{escape(title)}</title></polyline>"
    )
