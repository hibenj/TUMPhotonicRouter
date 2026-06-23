"""Realize electrical routing results as simple metal polygons."""

from __future__ import annotations

from gdsfactory.component import Component
import klayout.db as kdb

from .terminal_contacts import terminal_access_path
from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    GridCell,
    GridPoint,
    PadPlan,
)

BBox = tuple[float, float, float, float]
RectList = list[BBox]


def realize_electrical_metal(
    component: Component,
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    detailed_bundle_routes: DetailedBundleRoutingResult | None,
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
) -> Component:
    """Return a copy of ``component`` with routed electrical metal polygons.

    Milestone scope is intentionally simple: routes are drawn as constant-width
    Manhattan wire rectangles plus square joins, and assigned bondpads are drawn
    as rectangles. Empty pad slots remain abstract and are not realized.
    """

    routed = component.copy()
    routed.name = f"{component.name}_electrical"

    rects_by_net: dict[str, RectList] = {"common_bus": [common_bus.bus.bbox]}
    for route in common_bus.routes:
        _append_terminal_grid_route(
            rects_by_net["common_bus"],
            route.terminal,
            route.path,
            obstacle_map,
            width_um=config.bus_width_um,
        )

    if common_bus_escape is not None and common_bus_escape.success:
        _append_grid_wire_path(
            rects_by_net["common_bus"],
            common_bus_escape.path,
            obstacle_map,
            width_um=config.bus_width_um,
        )

    if detailed_bundle_routes is not None:
        for route in detailed_bundle_routes.routes:
            if not route.success:
                continue
            net_id = (
                route.pad_assignment.net_id
                if route.pad_assignment is not None
                else f"individual:{route.terminal.heater_id}"
            )
            _append_terminal_point_route(
                rects_by_net.setdefault(net_id, []),
                route.terminal,
                route.offset_path,
                obstacle_map,
                width_um=config.wire_width_um,
            )

    if pad_plan is not None:
        for assignment in pad_plan.assignments:
            rects_by_net.setdefault(assignment.net_id, []).append(assignment.slot.bbox)

    polygon_count_by_net: dict[str, int] = {}
    for net_id in sorted(rects_by_net):
        polygon_count_by_net[net_id] = _append_merged_rects(
            routed,
            rects_by_net[net_id],
            config.metal_layer,
        )
    routed.info["electrical_metal_realization"] = {
        "net_count": len(rects_by_net),
        "pre_union_rect_count": sum(len(rects) for rects in rects_by_net.values()),
        "output_polygon_count": sum(polygon_count_by_net.values()),
        "rect_count_by_net": {
            net_id: len(rects)
            for net_id, rects in sorted(rects_by_net.items())
        },
        "polygon_count_by_net": dict(sorted(polygon_count_by_net.items())),
    }

    return routed


def _append_terminal_grid_route(
    rects: RectList,
    terminal: ElectricalTerminal,
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    *,
    width_um: float,
) -> None:
    points = tuple(
        _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map)
        for cell in path
    )
    _append_terminal_um_route(rects, terminal, points, width_um=width_um)


def _append_terminal_point_route(
    rects: RectList,
    terminal: ElectricalTerminal,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    *,
    width_um: float,
) -> None:
    points = tuple(_grid_point_to_um(point, obstacle_map) for point in points_grid)
    _append_terminal_um_route(rects, terminal, points, width_um=width_um)


def _append_terminal_um_route(
    rects: RectList,
    terminal: ElectricalTerminal,
    route_points_um: tuple[tuple[float, float], ...],
    *,
    width_um: float,
) -> None:
    """Draw a physical-port adapter and only the usable snapped route tail."""

    access = terminal_access_path(
        terminal,
        route_points_um,
        fallback_width_um=width_um,
    )
    _append_rect(rects, access.contact_bbox)
    _append_um_wire_path(
        rects,
        access.adapter_points,
        width_um=access.access_width_um,
    )
    _append_um_wire_path(rects, access.route_tail_points, width_um=width_um)


def _append_grid_wire_path(
    rects: RectList,
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    *,
    width_um: float,
) -> None:
    points = tuple((cell[0] + 0.5, cell[1] + 0.5) for cell in path)
    _append_point_wire_path(rects, points, obstacle_map, width_um=width_um)


def _append_point_wire_path(
    rects: RectList,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    *,
    width_um: float,
) -> None:
    points_um = tuple(_grid_point_to_um(point, obstacle_map) for point in points_grid)
    _append_um_wire_path(rects, points_um, width_um=width_um)


def _append_um_wire_path(
    rects: RectList,
    points_um: tuple[tuple[float, float], ...],
    *,
    width_um: float,
) -> None:
    points_um = _simplify_manhattan_points(_dedupe_points(points_um))
    if not points_um:
        return
    half_width = width_um / 2.0
    if len(points_um) == 1:
        x, y = points_um[0]
        _append_rect(
            rects,
            (x - half_width, y - half_width, x + half_width, y + half_width),
        )
        return

    for start, end in zip(points_um, points_um[1:]):
        _append_segment_rect(rects, start, end, half_width)
    for point in points_um:
        x, y = point
        _append_rect(
            rects,
            (x - half_width, y - half_width, x + half_width, y + half_width),
        )


def _append_segment_rect(
    rects: RectList,
    start: tuple[float, float],
    end: tuple[float, float],
    half_width: float,
) -> None:
    sx, sy = start
    ex, ey = end
    if sx == ex and sy == ey:
        _append_rect(rects, (sx - half_width, sy - half_width, sx + half_width, sy + half_width))
        return
    if sx == ex:
        _append_rect(
            rects,
            (
                sx - half_width,
                min(sy, ey) - half_width,
                sx + half_width,
                max(sy, ey) + half_width,
            ),
        )
        return
    if sy == ey:
        _append_rect(
            rects,
            (
                min(sx, ex) - half_width,
                sy - half_width,
                max(sx, ex) + half_width,
                sy + half_width,
            ),
        )
        return

    # The current router should generate rectilinear paths. Split a malformed
    # diagonal defensively so realization still produces connected metal.
    via = (ex, sy)
    _append_segment_rect(rects, start, via, half_width)
    _append_segment_rect(rects, via, end, half_width)


def _append_rect(
    rects: RectList,
    bbox: BBox,
) -> None:
    xmin, ymin, xmax, ymax = bbox
    if xmax < xmin:
        xmin, xmax = xmax, xmin
    if ymax < ymin:
        ymin, ymax = ymax, ymin
    if xmax == xmin or ymax == ymin:
        return
    rects.append((xmin, ymin, xmax, ymax))


def _append_merged_rects(
    component: Component,
    rects: list[BBox],
    layer: tuple[int, int],
) -> int:
    polygons = _merge_rects(rects)
    for polygon in polygons:
        component.add_polygon(polygon, layer=layer)
    return len(polygons)


def _merge_rects(rects: list[BBox]) -> tuple[tuple[tuple[float, float], ...], ...]:
    region = kdb.Region()
    for rect in rects:
        xmin, ymin, xmax, ymax = rect
        box = kdb.Box(
            _um_to_dbu(xmin),
            _um_to_dbu(ymin),
            _um_to_dbu(xmax),
            _um_to_dbu(ymax),
        )
        if not box.empty():
            region.insert(box)
    if region.is_empty():
        return ()

    polygons: list[tuple[tuple[float, float], ...]] = []
    for polygon in region.merged().each():
        hull = tuple(
            (_dbu_to_um(point.x), _dbu_to_um(point.y))
            for point in polygon.each_point_hull()
        )
        if len(hull) >= 3:
            polygons.append(hull)
    return tuple(polygons)


def _um_to_dbu(value: float) -> int:
    return int(round(value * 1000.0))


def _dbu_to_um(value: int) -> float:
    return value / 1000.0


def _grid_point_to_um(
    point: GridPoint,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[float, float]:
    origin_x, origin_y = obstacle_map.grid.origin
    grid_size = obstacle_map.grid.grid_size_um
    return (origin_x + point[0] * grid_size, origin_y + point[1] * grid_size)


def _dedupe_points(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    deduped: list[tuple[float, float]] = []
    for point in points:
        if deduped and deduped[-1] == point:
            continue
        deduped.append(point)
    return tuple(deduped)


def _simplify_manhattan_points(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    if len(points) <= 2:
        return points
    simplified: list[tuple[float, float]] = [points[0]]
    previous_direction = _point_direction(points[0], points[1])
    for index in range(1, len(points) - 1):
        current_direction = _point_direction(points[index], points[index + 1])
        if current_direction != previous_direction:
            simplified.append(points[index])
            previous_direction = current_direction
    simplified.append(points[-1])
    return tuple(simplified)


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
