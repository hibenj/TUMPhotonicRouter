"""Post-route verification for electrical heater metal routing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .pad_slots import pad_access_bbox
from .pitch_grid import bbox_to_grid_cells
from .terminal_contacts import terminal_access_path, terminal_contact_bboxes
from .types import (
    BBox,
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    ElectricalVerificationIssue,
    ElectricalVerificationResult,
    GridCell,
    GridPoint,
    PadPlan,
    TerminalBusRoute,
)


@dataclass(frozen=True)
class _NetGeometry:
    net_id: str
    rects: tuple[BBox, ...]
    allowed_cells: frozenset[GridCell]
    allowed_physical_bboxes: tuple[BBox, ...] = ()


def verify_electrical_routing(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    detailed_bundle_routes: DetailedBundleRoutingResult | None,
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
) -> ElectricalVerificationResult:
    """Verify electrical routing geometry contracts.

    The verifier intentionally checks the geometry that realization should draw,
    not only the coarse topology cells. This catches routes that are topologically
    present but do not physically touch heater terminal pads, assigned bondpads,
    or keepout-safe routing space.
    """

    issues: list[ElectricalVerificationIssue] = []
    net_geometries: list[_NetGeometry] = []

    common_bus_rects = _common_bus_rects(
        common_bus,
        common_bus_escape,
        obstacle_map,
        config,
    )
    common_bus_allowed = _common_bus_allowed_cells(
        common_bus,
        common_bus_escape,
        obstacle_map,
        config,
    )
    net_geometries.append(
        _NetGeometry(
            net_id="common_bus",
            rects=common_bus_rects,
            allowed_cells=frozenset(common_bus_allowed),
            allowed_physical_bboxes=_common_bus_allowed_physical_bboxes(
                common_bus,
                obstacle_map,
                config,
            ),
        )
    )
    _verify_common_bus_terminal_contacts(issues, common_bus, common_bus_rects)
    _verify_common_bus_pad_contact(issues, common_bus_escape, common_bus_rects, config)

    if detailed_bundle_routes is not None:
        for route in detailed_bundle_routes.routes:
            route_rects = _detailed_route_rects(route, obstacle_map, config)
            route_start_um = _route_start_um(route, obstacle_map)
            allowed_cells = set(
                _individual_terminal_open_cells(obstacle_map).get(
                    route.terminal.id,
                    (),
                )
            )
            access = _terminal_route_access(route, obstacle_map, config.wire_width_um)
            allowed_cells.update(
                _terminal_contact_cells(
                    route.terminal,
                    obstacle_map,
                    config.wire_width_um,
                    route_start_um=route_start_um,
                )
            )
            allowed_cells.update(route.target_cells)
            net_geometries.append(
                _NetGeometry(
                    net_id=f"individual:{route.terminal.heater_id}",
                    rects=route_rects,
                    allowed_cells=frozenset(allowed_cells),
                    allowed_physical_bboxes=(access.contact_bbox,),
                )
            )
            _verify_terminal_contact(
                issues,
                route.terminal,
                route_rects,
                route.pad_assignment.net_id if route.pad_assignment else None,
            )
            _verify_individual_pad_contact(issues, route, route_rects, config)

        for failed_route in detailed_bundle_routes.failed_routes:
            issues.append(
                ElectricalVerificationIssue(
                    code="failed_detailed_route",
                    message=(
                        "Detailed individual route failed before realization: "
                        f"{failed_route.reason or 'unknown reason'}"
                    ),
                    net_id=(
                        failed_route.pad_assignment.net_id
                        if failed_route.pad_assignment is not None
                        else f"individual:{failed_route.terminal.heater_id}"
                    ),
                    details={"terminal_id": failed_route.terminal.id},
                )
            )

    _verify_raw_physical_obstacle_overlaps(issues, net_geometries, obstacle_map)
    _verify_blocked_cell_clearance(issues, net_geometries, obstacle_map)
    _verify_cross_net_overlaps(issues, net_geometries)

    return ElectricalVerificationResult(issues=tuple(issues))


def _common_bus_rects(
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[BBox, ...]:
    rects: list[BBox] = [common_bus.bus.bbox]
    for route in common_bus.routes:
        rects.extend(
            _terminal_grid_route_rects(
                route.terminal,
                route.path,
                obstacle_map,
                config.bus_width_um,
            )
        )
    if common_bus_escape is not None and common_bus_escape.success:
        rects.extend(
            _grid_wire_rects(
                common_bus_escape.path,
                obstacle_map,
                config.bus_width_um,
            )
        )
    return tuple(rects)


def _common_bus_allowed_cells(
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> set[GridCell]:
    allowed: set[GridCell] = set(common_bus.bus.cells)
    for route in common_bus.routes:
        allowed.update(
            _common_bus_terminal_open_cells(obstacle_map).get(route.terminal.id, ())
        )
        allowed.update(
            _terminal_contact_cells(
                route.terminal,
                obstacle_map,
                config.bus_width_um,
                route_start_um=(
                    _grid_cell_center_um(route.path[0], obstacle_map)
                    if route.path
                    else None
                ),
            )
        )
    if common_bus_escape is not None:
        allowed.update(common_bus_escape.target_cells)
    return allowed


def _common_bus_allowed_physical_bboxes(
    common_bus: CommonBusRoutingResult,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[BBox, ...]:
    return tuple(
        _terminal_grid_route_access(
            route.terminal,
            route.path,
            obstacle_map,
            config.bus_width_um,
        ).contact_bbox
        for route in common_bus.routes
    )


def _common_bus_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    return (
        obstacle_map.common_bus_terminal_open_cells
        or obstacle_map.terminal_open_cells
    )


def _individual_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    return (
        obstacle_map.individual_terminal_open_cells
        or obstacle_map.terminal_open_cells
    )


def _verify_common_bus_terminal_contacts(
    issues: list[ElectricalVerificationIssue],
    common_bus: CommonBusRoutingResult,
    common_bus_rects: tuple[BBox, ...],
) -> None:
    for route in common_bus.routes:
        _verify_terminal_contact(
            issues,
            route.terminal,
            common_bus_rects,
            net_id="common_bus",
            route=route,
        )


def _verify_common_bus_pad_contact(
    issues: list[ElectricalVerificationIssue],
    common_bus_escape: CommonBusEscapeResult | None,
    common_bus_rects: tuple[BBox, ...],
    config: ElectricalRoutingConfig,
) -> None:
    if common_bus_escape is None:
        issues.append(
            ElectricalVerificationIssue(
                code="missing_common_bus_escape",
                message="Common bus has no route to its assigned pad.",
                net_id="common_bus",
            )
        )
        return
    if not common_bus_escape.success or common_bus_escape.pad_assignment is None:
        issues.append(
            ElectricalVerificationIssue(
                code="failed_common_bus_escape",
                message=(
                    "Common bus did not reach its assigned pad: "
                    f"{common_bus_escape.reason or 'unknown reason'}"
                ),
                net_id="common_bus",
            )
        )
        return
    access = pad_access_bbox(common_bus_escape.pad_assignment.slot, config)
    if not _any_rect_intersects(common_bus_rects, access):
        issues.append(
            ElectricalVerificationIssue(
                code="missing_pad_contact",
                message="Common bus metal does not physically touch its assigned pad access region.",
                net_id="common_bus",
                details={"pad_slot": common_bus_escape.pad_assignment.slot.index},
            )
        )


def _verify_terminal_contact(
    issues: list[ElectricalVerificationIssue],
    terminal: ElectricalTerminal,
    rects: tuple[BBox, ...],
    net_id: str | None,
    route: TerminalBusRoute | None = None,
) -> None:
    terminal_bboxes = terminal_contact_bboxes(terminal, fallback_width_um=0.0)
    contacted_bboxes = tuple(
        bbox for bbox in terminal_bboxes if _any_rect_intersects(rects, bbox)
    )
    if contacted_bboxes:
        return
    details: dict[str, Any] = {
        "terminal_id": terminal.id,
        "heater_id": terminal.heater_id,
        "terminal_bbox": terminal.bbox,
        "contact_bboxes": terminal_bboxes,
    }
    if route is not None:
        details["route_cost"] = route.cost
    issues.append(
        ElectricalVerificationIssue(
            code="missing_terminal_contact",
            message=(
                "Routed metal does not physically touch heater terminal "
                f"{terminal.id}."
            ),
            net_id=net_id,
            details=details,
        )
    )


def _verify_individual_pad_contact(
    issues: list[ElectricalVerificationIssue],
    route: DetailedBundleRoute,
    rects: tuple[BBox, ...],
    config: ElectricalRoutingConfig,
) -> None:
    if route.pad_assignment is None:
        issues.append(
            ElectricalVerificationIssue(
                code="missing_pad_assignment",
                message=f"Detailed route for {route.terminal.id} has no assigned pad.",
                net_id=f"individual:{route.terminal.heater_id}",
                details={"terminal_id": route.terminal.id},
            )
        )
        return
    access = pad_access_bbox(route.pad_assignment.slot, config)
    if _any_rect_intersects(rects, access):
        return
    issues.append(
        ElectricalVerificationIssue(
            code="missing_pad_contact",
            message=(
                "Individual route metal does not physically touch its assigned "
                f"pad access region for {route.terminal.id}."
            ),
            net_id=route.pad_assignment.net_id,
            details={
                "terminal_id": route.terminal.id,
                "pad_slot": route.pad_assignment.slot.index,
            },
        )
    )


def _verify_raw_physical_obstacle_overlaps(
    issues: list[ElectricalVerificationIssue],
    net_geometries: list[_NetGeometry],
    obstacle_map: ElectricalObstacleMap,
) -> None:
    if not obstacle_map.raw_obstacle_bboxes:
        return
    for net in net_geometries:
        illegal_overlaps: set[BBox] = set()
        for rect in net.rects:
            for obstacle_bbox in obstacle_map.raw_obstacle_bboxes:
                overlap = _rect_intersection(rect, obstacle_bbox)
                if overlap is None or _rect_area(overlap) <= 0:
                    continue
                if _rect_is_covered_by_any(overlap, net.allowed_physical_bboxes):
                    continue
                illegal_overlaps.add(overlap)
        if not illegal_overlaps:
            continue
        sorted_overlaps = tuple(sorted(illegal_overlaps))
        issues.append(
            ElectricalVerificationIssue(
                code="metal_overlaps_raw_obstacle",
                message=(
                    f"Metal for {net.net_id} overlaps original obstacle geometry "
                    "outside an allowed terminal contact."
                ),
                net_id=net.net_id,
                details={
                    "overlap_count": len(sorted_overlaps),
                    "sample_overlaps": sorted_overlaps[:10],
                },
            )
        )


def _verify_blocked_cell_clearance(
    issues: list[ElectricalVerificationIssue],
    net_geometries: list[_NetGeometry],
    obstacle_map: ElectricalObstacleMap,
) -> None:
    blocked = set(obstacle_map.blocked_cells)
    for net in net_geometries:
        hits: set[GridCell] = set()
        for rect in net.rects:
            hits.update(bbox_to_grid_cells(rect, obstacle_map.grid))
        hits.difference_update(net.allowed_cells)
        hits.intersection_update(blocked)
        if not hits:
            continue
        sample = tuple(sorted(hits)[:10])
        issues.append(
            ElectricalVerificationIssue(
                code="metal_overlaps_blocked_cells",
                message=f"Metal for {net.net_id} overlaps blocked routing cells.",
                net_id=net.net_id,
                details={
                    "blocked_cell_count": len(hits),
                    "sample_blocked_cells": sample,
                },
            )
        )


def _verify_cross_net_overlaps(
    issues: list[ElectricalVerificationIssue],
    net_geometries: list[_NetGeometry],
) -> None:
    for index, left in enumerate(net_geometries):
        for right in net_geometries[index + 1 :]:
            if left.net_id == right.net_id:
                continue
            overlaps = _rect_overlap_samples(left.rects, right.rects)
            if not overlaps:
                continue
            issues.append(
                ElectricalVerificationIssue(
                    code="cross_net_metal_overlap",
                    message=(
                        f"Metal for {left.net_id} overlaps metal for {right.net_id}."
                    ),
                    net_id=left.net_id,
                    details={
                        "other_net_id": right.net_id,
                        "overlap_count": len(overlaps),
                        "sample_overlaps": overlaps[:5],
                    },
                )
            )


def _detailed_route_rects(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[BBox, ...]:
    return _terminal_point_route_rects(
        route.terminal,
        route.offset_path,
        obstacle_map,
        config.wire_width_um,
    )


def _grid_wire_rects(
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
) -> tuple[BBox, ...]:
    points = tuple(
        _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map)
        for cell in path
    )
    return _point_wire_rects(points, width_um)


def _terminal_grid_route_rects(
    terminal: ElectricalTerminal,
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
) -> tuple[BBox, ...]:
    return _terminal_access_rects(
        _terminal_grid_route_access(terminal, path, obstacle_map, width_um),
        width_um,
    )


def _terminal_point_route_rects(
    terminal: ElectricalTerminal,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
) -> tuple[BBox, ...]:
    return _terminal_access_rects(
        _terminal_point_route_access(terminal, points_grid, obstacle_map, width_um),
        width_um,
    )


def _terminal_grid_route_access(
    terminal: ElectricalTerminal,
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
) -> Any:
    points_um = tuple(
        _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map)
        for cell in path
    )
    return terminal_access_path(
        terminal,
        points_um,
        fallback_width_um=width_um,
    )


def _terminal_route_access(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
) -> Any:
    return _terminal_point_route_access(
        route.terminal,
        route.offset_path,
        obstacle_map,
        width_um,
    )


def _terminal_point_route_access(
    terminal: ElectricalTerminal,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
) -> Any:
    points_um = tuple(_grid_point_to_um(point, obstacle_map) for point in points_grid)
    return terminal_access_path(
        terminal,
        points_um,
        fallback_width_um=width_um,
    )


def _terminal_access_rects(
    access: Any,
    width_um: float,
) -> tuple[BBox, ...]:
    rects: list[BBox] = [access.contact_bbox]
    rects.extend(_point_wire_rects(access.adapter_points, access.access_width_um))
    rects.extend(_point_wire_rects(access.route_tail_points, width_um))
    return tuple(rects)


def _terminal_contact_cells(
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
    *,
    route_start_um: tuple[float, float] | None,
) -> frozenset[GridCell]:
    access = terminal_access_path(
        terminal,
        (route_start_um,) if route_start_um is not None else (),
        fallback_width_um=width_um,
    )
    return bbox_to_grid_cells(
        access.contact_bbox,
        obstacle_map.grid,
    )


def _grid_cell_center_um(
    cell: GridCell,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[float, float]:
    return _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map)


def _point_wire_rects(
    points: tuple[tuple[float, float], ...],
    width_um: float,
) -> tuple[BBox, ...]:
    if not points:
        return ()
    half_width = width_um / 2.0
    rects: list[BBox] = []
    if len(points) == 1:
        rects.append(_point_rect(points[0], half_width))
        return tuple(rects)
    for start, end in zip(points, points[1:]):
        rects.extend(_segment_rects(start, end, half_width))
    for point in points:
        rects.append(_point_rect(point, half_width))
    return tuple(rects)


def _segment_rects(
    start: tuple[float, float],
    end: tuple[float, float],
    half_width: float,
) -> tuple[BBox, ...]:
    sx, sy = start
    ex, ey = end
    if sx == ex and sy == ey:
        return (_point_rect(start, half_width),)
    if sx == ex:
        return (
            (
                sx - half_width,
                min(sy, ey) - half_width,
                sx + half_width,
                max(sy, ey) + half_width,
            ),
        )
    if sy == ey:
        return (
            (
                min(sx, ex) - half_width,
                sy - half_width,
                max(sx, ex) + half_width,
                sy + half_width,
            ),
        )
    via = (ex, sy)
    return (*_segment_rects(start, via, half_width), *_segment_rects(via, end, half_width))


def _point_rect(point: tuple[float, float], half_width: float) -> BBox:
    x, y = point
    return (x - half_width, y - half_width, x + half_width, y + half_width)


def _route_start_um(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[float, float] | None:
    if not route.offset_path:
        return None
    return _grid_point_to_um(route.offset_path[0], obstacle_map)


def _grid_point_to_um(
    point: GridPoint,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[float, float]:
    origin_x, origin_y = obstacle_map.grid.origin
    grid_size = obstacle_map.grid.grid_size_um
    return (origin_x + point[0] * grid_size, origin_y + point[1] * grid_size)


def _any_rect_intersects(rects: tuple[BBox, ...], target: BBox) -> bool:
    return any(_rect_intersects(rect, target) for rect in rects)


def _rect_overlap_samples(
    left_rects: tuple[BBox, ...],
    right_rects: tuple[BBox, ...],
) -> tuple[BBox, ...]:
    overlaps: list[BBox] = []
    for left in left_rects:
        for right in right_rects:
            overlap = _rect_intersection(left, right)
            if overlap is None:
                continue
            overlaps.append(overlap)
    return tuple(overlaps)


def _rect_intersection(left: BBox, right: BBox) -> BBox | None:
    if not _rect_intersects(left, right):
        return None
    return (
        max(left[0], right[0]),
        max(left[1], right[1]),
        min(left[2], right[2]),
        min(left[3], right[3]),
    )


def _rect_is_covered_by_any(rect: BBox, covers: tuple[BBox, ...]) -> bool:
    return any(_rect_contains(cover, rect) for cover in covers)


def _rect_contains(outer: BBox, inner: BBox) -> bool:
    return (
        outer[0] <= inner[0]
        and outer[1] <= inner[1]
        and outer[2] >= inner[2]
        and outer[3] >= inner[3]
    )


def _rect_area(rect: BBox) -> float:
    return max(0.0, rect[2] - rect[0]) * max(0.0, rect[3] - rect[1])


def _rect_intersects(left: BBox, right: BBox) -> bool:
    return not (
        left[2] < right[0]
        or right[2] < left[0]
        or left[3] < right[1]
        or right[3] < left[1]
    )
