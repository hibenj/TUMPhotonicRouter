"""Realize electrical routing results as simple metal polygons."""

from __future__ import annotations

from gdsfactory.component import Component

from .types import (
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    GridCell,
    GridPoint,
    PadPlan,
)


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

    _append_rect(routed, common_bus.bus.bbox, config.metal_layer)
    for route in common_bus.routes:
        _append_grid_wire_path(
            routed,
            route.path,
            obstacle_map,
            width_um=config.bus_width_um,
            layer=config.metal_layer,
        )

    if common_bus_escape is not None and common_bus_escape.success:
        _append_grid_wire_path(
            routed,
            common_bus_escape.path,
            obstacle_map,
            width_um=config.bus_width_um,
            layer=config.metal_layer,
        )

    if detailed_bundle_routes is not None:
        for route in detailed_bundle_routes.routes:
            if not route.success:
                continue
            _append_point_wire_path(
                routed,
                route.offset_path,
                obstacle_map,
                width_um=config.wire_width_um,
                layer=config.metal_layer,
            )

    if pad_plan is not None:
        for assignment in pad_plan.assignments:
            _append_rect(routed, assignment.slot.bbox, config.metal_layer)

    return routed


def _append_grid_wire_path(
    component: Component,
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    *,
    width_um: float,
    layer: tuple[int, int],
) -> None:
    points = tuple((cell[0] + 0.5, cell[1] + 0.5) for cell in path)
    _append_point_wire_path(component, points, obstacle_map, width_um=width_um, layer=layer)


def _append_point_wire_path(
    component: Component,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    *,
    width_um: float,
    layer: tuple[int, int],
) -> None:
    points_um = tuple(_grid_point_to_um(point, obstacle_map) for point in points_grid)
    points_um = _simplify_manhattan_points(_dedupe_points(points_um))
    if not points_um:
        return
    half_width = width_um / 2.0
    if len(points_um) == 1:
        x, y = points_um[0]
        _append_rect(
            component,
            (x - half_width, y - half_width, x + half_width, y + half_width),
            layer,
        )
        return

    for start, end in zip(points_um, points_um[1:]):
        _append_segment_rect(component, start, end, half_width, layer)
    for point in points_um:
        x, y = point
        _append_rect(
            component,
            (x - half_width, y - half_width, x + half_width, y + half_width),
            layer,
        )


def _append_segment_rect(
    component: Component,
    start: tuple[float, float],
    end: tuple[float, float],
    half_width: float,
    layer: tuple[int, int],
) -> None:
    sx, sy = start
    ex, ey = end
    if sx == ex and sy == ey:
        _append_rect(component, (sx - half_width, sy - half_width, sx + half_width, sy + half_width), layer)
        return
    if sx == ex:
        _append_rect(
            component,
            (
                sx - half_width,
                min(sy, ey) - half_width,
                sx + half_width,
                max(sy, ey) + half_width,
            ),
            layer,
        )
        return
    if sy == ey:
        _append_rect(
            component,
            (
                min(sx, ex) - half_width,
                sy - half_width,
                max(sx, ex) + half_width,
                sy + half_width,
            ),
            layer,
        )
        return

    # The current router should generate rectilinear paths. Split a malformed
    # diagonal defensively so realization still produces connected metal.
    via = (ex, sy)
    _append_segment_rect(component, start, via, half_width, layer)
    _append_segment_rect(component, via, end, half_width, layer)


def _append_rect(
    component: Component,
    bbox: tuple[float, float, float, float],
    layer: tuple[int, int],
) -> None:
    xmin, ymin, xmax, ymax = bbox
    if xmax < xmin:
        xmin, xmax = xmax, xmin
    if ymax < ymin:
        ymin, ymax = ymax, ymin
    component.add_polygon(
        (
            (xmin, ymin),
            (xmax, ymin),
            (xmax, ymax),
            (xmin, ymax),
        ),
        layer=layer,
    )


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
