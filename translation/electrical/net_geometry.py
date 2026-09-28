"""What the realized metal looks like per net: rectangles with their source
tags, allowed cells and boxes, centerlines."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .pitch_grid import bbox_to_grid_cells, grid_cell_center_um
from .rect_geometry import (
    clip_manhattan_path_at_first_bbox_entry,
    clip_manhattan_path_start_at_bbox,
    disjoint_union_rects,
    normalize_rect,
    union_rect_area,
    wire_rects_for_points,
)
from .terminal_contacts import terminal_access_path
from .types import (
    BBox,
    CommonBusEscapeResult,
    CommonBusRoutingResult,
    DetailedBundleRoute,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalPortAccess,
    ElectricalRoutingConfig,
    ElectricalTerminal,
    GridCell,
    GridPoint,
    PadPlan,
)


@dataclass(frozen=True)
class NetGeometry:
    net_id: str
    rects: tuple[BBox, ...]
    rect_sources: tuple[str, ...]
    allowed_cells: frozenset[GridCell]
    allowed_physical_bboxes: tuple[BBox, ...] = ()
    #: One polyline per routed branch (the bus: one per terminal branch plus
    #: the escape; an individual net: its one wire), so lengths and bend
    #: counts never include jumps between branches.
    centerline_polylines_um: tuple[tuple[tuple[float, float], ...], ...] = ()


@dataclass(frozen=True)
class TaggedRect:
    bbox: BBox
    source: str


def build_net_geometries(
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    detailed_bundle_routes: DetailedBundleRoutingResult | None,
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
) -> list[NetGeometry]:
    """Build the per-net geometry list verification checks and metrics share.

    One entry for the common bus (always first), then one entry per
    successful detailed bundle route, in the same order
    ``detailed_bundle_routes.routes`` lists them.
    """

    net_geometries: list[NetGeometry] = []

    pad_bboxes_by_net = _pad_bboxes_by_net(pad_plan)
    common_bus_tagged_rects = _clean_tagged_rects(
        (
            *_common_bus_tagged_rects(
                common_bus,
                common_bus_escape,
                obstacle_map,
                config,
            ),
            *(TaggedRect(bbox, "pad") for bbox in pad_bboxes_by_net.get("common_bus", ())),
        )
    )
    common_bus_rects = tuple(tagged.bbox for tagged in common_bus_tagged_rects)
    common_bus_route_points = _common_bus_centerline_points(
        common_bus,
        common_bus_escape,
        obstacle_map,
    )
    common_bus_allowed = _common_bus_allowed_cells(
        common_bus,
        common_bus_escape,
        obstacle_map,
        config,
    )
    net_geometries.append(
        NetGeometry(
            net_id="common_bus",
            rects=common_bus_rects,
            rect_sources=tuple(tagged.source for tagged in common_bus_tagged_rects),
            allowed_cells=frozenset(common_bus_allowed),
            allowed_physical_bboxes=_common_bus_allowed_physical_bboxes(
                common_bus,
                obstacle_map,
                config,
            ),
            centerline_polylines_um=common_bus_route_points,
        )
    )

    if detailed_bundle_routes is not None:
        for route in detailed_bundle_routes.routes:
            route_net_id = (
                route.pad_assignment.net_id
                if route.pad_assignment is not None
                else f"individual:{route.terminal.heater_id}"
            )
            route_tagged_rects = _clean_tagged_rects(
                (
                    *_detailed_route_tagged_rects(route, obstacle_map, config),
                    *(TaggedRect(bbox, "pad") for bbox in pad_bboxes_by_net.get(route_net_id, ())),
                )
            )
            route_rects = tuple(tagged.bbox for tagged in route_tagged_rects)
            route_start_um = _route_start_um(route, obstacle_map)
            allowed_cells = set(
                _individual_terminal_open_cells(obstacle_map).get(
                    route.terminal.id,
                    (),
                )
            )
            access = _terminal_route_access(
                route,
                obstacle_map,
                contact_width_um=config.terminal_contact_width_um,
            )
            allowed_cells.update(
                _terminal_contact_cells(
                    route.terminal,
                    obstacle_map,
                    config.terminal_contact_width_um,
                    route_start_um=route_start_um,
                )
            )
            allowed_cells.update(route.target_cells)
            net_geometries.append(
                NetGeometry(
                    net_id=route_net_id,
                    rects=route_rects,
                    rect_sources=tuple(tagged.source for tagged in route_tagged_rects),
                    allowed_cells=frozenset(allowed_cells),
                    allowed_physical_bboxes=(access.contact_bbox,),
                    centerline_polylines_um=(
                        detailed_route_centerline_points(route, obstacle_map),
                    ),
                )
            )

    return net_geometries


def _common_bus_tagged_rects(
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[TaggedRect, ...]:
    rects: list[TaggedRect] = [TaggedRect(common_bus.bus.bbox, "bus_stripe")]
    for route in common_bus.routes:
        rects.extend(
            _terminal_grid_route_tagged_rects(
                route.terminal,
                route.path,
                obstacle_map,
                route_width_um=config.wire_width_um,
                contact_width_um=config.terminal_contact_width_um,
                route_source="bus_route",
                entry_clip_bbox=common_bus.bus.bbox,
                access=_common_bus_access(obstacle_map, route.terminal),
            )
        )
    if (
        common_bus_escape is not None
        and common_bus_escape.success
        and len(common_bus_escape.path) > 1
    ):
        rects.extend(
            _tagged_grid_wire_rects(
                common_bus_escape.path,
                obstacle_map,
                config.bus_width_um,
                "bus_escape",
                start_clip_bbox=common_bus.bus.bbox,
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
        allowed.update(_common_bus_terminal_open_cells(obstacle_map).get(route.terminal.id, ()))
        allowed.update(
            _terminal_contact_cells(
                route.terminal,
                obstacle_map,
                config.terminal_contact_width_um,
                route_start_um=(
                    grid_cell_center_um(route.path[0], obstacle_map.grid) if route.path else None
                ),
                access=_common_bus_access(obstacle_map, route.terminal),
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
            config.terminal_contact_width_um,
            access=_common_bus_access(obstacle_map, route.terminal),
        ).contact_bbox
        for route in common_bus.routes
    )


def _common_bus_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    return obstacle_map.common_bus_terminal_open_cells or obstacle_map.terminal_open_cells


def _individual_terminal_open_cells(
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, frozenset[GridCell]]:
    return obstacle_map.individual_terminal_open_cells or obstacle_map.terminal_open_cells


def _common_bus_access(
    obstacle_map: ElectricalObstacleMap,
    terminal: ElectricalTerminal,
) -> ElectricalPortAccess | None:
    return obstacle_map.common_bus_port_accesses.get(terminal.id)


def _individual_access(
    obstacle_map: ElectricalObstacleMap,
    terminal: ElectricalTerminal,
) -> ElectricalPortAccess | None:
    return obstacle_map.individual_port_accesses.get(terminal.id)


def _pad_bboxes_by_net(pad_plan: PadPlan | None) -> dict[str, tuple[BBox, ...]]:
    if pad_plan is None:
        return {}
    bboxes_by_net: dict[str, list[BBox]] = {}
    for assignment in pad_plan.assignments:
        bboxes_by_net.setdefault(assignment.net_id, []).append(assignment.slot.bbox)
    return {net_id: tuple(bboxes) for net_id, bboxes in bboxes_by_net.items()}


def _common_bus_centerline_points(
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[tuple[tuple[float, float], ...], ...]:
    """One polyline per bus branch, plus the escape's."""

    polylines = [
        tuple(grid_cell_center_um(cell, obstacle_map.grid) for cell in route.path)
        for route in common_bus.routes
    ]
    if common_bus_escape is not None and common_bus_escape.success:
        polylines.append(
            tuple(grid_cell_center_um(cell, obstacle_map.grid) for cell in common_bus_escape.path)
        )
    return tuple(polylines)


def detailed_route_centerline_points(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[tuple[float, float], ...]:
    return tuple(_grid_point_to_um(point, obstacle_map) for point in route.offset_path)


def _detailed_route_tagged_rects(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[TaggedRect, ...]:
    return _terminal_point_route_tagged_rects(
        route.terminal,
        route.offset_path,
        obstacle_map,
        route_width_um=config.wire_width_um,
        contact_width_um=config.terminal_contact_width_um,
        route_source="route_tail",
        access=_individual_access(obstacle_map, route.terminal),
        preferred_port_name=route.contact_port_name,
    )


def _tagged_grid_wire_rects(
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
    source: str,
    *,
    start_clip_bbox: BBox | None = None,
) -> tuple[TaggedRect, ...]:
    points = tuple(_grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map) for cell in path)
    if start_clip_bbox is not None:
        points = clip_manhattan_path_start_at_bbox(points, start_clip_bbox)
    return _tagged_point_wire_rects(
        points,
        width_um,
        source,
        trim_start=start_clip_bbox is not None,
    )


def _terminal_grid_route_tagged_rects(
    terminal: ElectricalTerminal,
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    route_width_um: float,
    contact_width_um: float,
    *,
    route_source: str,
    entry_clip_bbox: BBox | None = None,
    access: ElectricalPortAccess | None = None,
) -> tuple[TaggedRect, ...]:
    return _terminal_access_tagged_rects(
        _terminal_grid_route_access(
            terminal,
            path,
            obstacle_map,
            contact_width_um,
            entry_clip_bbox=entry_clip_bbox,
            access=access,
        ),
        route_width_um,
        route_source=route_source,
    )


def _terminal_point_route_tagged_rects(
    terminal: ElectricalTerminal,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    route_width_um: float,
    contact_width_um: float,
    *,
    route_source: str,
    access: ElectricalPortAccess | None = None,
    preferred_port_name: str | None = None,
) -> tuple[TaggedRect, ...]:
    return _terminal_access_tagged_rects(
        _terminal_point_route_access(
            terminal,
            points_grid,
            obstacle_map,
            contact_width_um,
            access=access,
            preferred_port_name=preferred_port_name,
        ),
        route_width_um,
        route_source=route_source,
    )


def _terminal_grid_route_access(
    terminal: ElectricalTerminal,
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    contact_width_um: float,
    *,
    entry_clip_bbox: BBox | None = None,
    access: ElectricalPortAccess | None = None,
) -> Any:
    points_um = tuple(
        _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map) for cell in path
    )
    if entry_clip_bbox is not None:
        points_um = clip_manhattan_path_at_first_bbox_entry(points_um, entry_clip_bbox)
    return terminal_access_path(
        terminal,
        points_um,
        fallback_width_um=contact_width_um,
        preferred_port_name=access.port_name if access is not None else None,
    )


def _terminal_route_access(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
    *,
    contact_width_um: float,
) -> Any:
    return _terminal_point_route_access(
        route.terminal,
        route.offset_path,
        obstacle_map,
        contact_width_um,
        access=_individual_access(obstacle_map, route.terminal),
        preferred_port_name=route.contact_port_name,
    )


def _terminal_point_route_access(
    terminal: ElectricalTerminal,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
    *,
    access: ElectricalPortAccess | None = None,
    preferred_port_name: str | None = None,
) -> Any:
    points_um = tuple(_grid_point_to_um(point, obstacle_map) for point in points_grid)
    resolved_port_name = (
        preferred_port_name
        if preferred_port_name is not None
        else (access.port_name if access is not None else None)
    )
    return terminal_access_path(
        terminal,
        points_um,
        fallback_width_um=width_um,
        preferred_port_name=resolved_port_name,
    )


def _terminal_access_tagged_rects(
    access: Any,
    width_um: float,
    *,
    route_source: str,
) -> tuple[TaggedRect, ...]:
    rects: list[TaggedRect] = [TaggedRect(access.contact_bbox, "terminal_contact")]
    rects.extend(
        _tagged_point_wire_rects(
            access.adapter_points,
            access.access_width_um,
            "terminal_adapter",
        )
    )
    rects.extend(
        _tagged_point_wire_rects(
            access.route_tail_points,
            width_um,
            route_source,
            trim_bends=route_source != "bus_route",
            trim_start=route_source == "route_tail",
        )
    )
    return tuple(rects)


def _terminal_contact_cells(
    terminal: ElectricalTerminal,
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
    *,
    route_start_um: tuple[float, float] | None,
    access: ElectricalPortAccess | None = None,
) -> frozenset[GridCell]:
    terminal_access = terminal_access_path(
        terminal,
        (route_start_um,) if route_start_um is not None else (),
        fallback_width_um=width_um,
        preferred_port_name=access.port_name if access is not None else None,
    )
    return bbox_to_grid_cells(
        terminal_access.contact_bbox,
        obstacle_map.grid,
    )


def _tagged_point_wire_rects(
    points: tuple[tuple[float, float], ...],
    width_um: float,
    source: str,
    *,
    trim_bends: bool = True,
    trim_start: bool = False,
) -> tuple[TaggedRect, ...]:
    return tuple(
        TaggedRect(rect, source)
        for rect in wire_rects_for_points(
            points,
            width_um,
            trim_bends=trim_bends,
            trim_start=trim_start,
        )
    )


def _clean_tagged_rects(
    tagged_rects: tuple[TaggedRect, ...],
) -> tuple[TaggedRect, ...]:
    bboxes_by_source: dict[str, list[BBox]] = {}
    for bbox, source in _normalized_tagged_bbox_sources(tagged_rects):
        bboxes_by_source.setdefault(source, []).append(bbox)
    source_disjoint_rects = tuple(
        TaggedRect(bbox, source)
        for source, bboxes in sorted(bboxes_by_source.items())
        for bbox in disjoint_union_rects(bboxes)
    )
    return _drop_union_redundant_tagged_rects(source_disjoint_rects)


def _drop_union_redundant_tagged_rects(
    tagged_rects: tuple[TaggedRect, ...],
) -> tuple[TaggedRect, ...]:
    kept = sorted(tagged_rects, key=_tagged_rect_sort_key)
    index = 0
    while index < len(kept):
        without_candidate = tuple(
            tagged.bbox for other_index, tagged in enumerate(kept) if other_index != index
        )
        if _same_area(
            union_rect_area(tagged.bbox for tagged in kept),
            union_rect_area(without_candidate),
        ):
            kept.pop(index)
            index = 0
            continue
        index += 1
    return tuple(sorted(kept, key=_tagged_rect_sort_key))


def _tagged_rect_sort_key(tagged: TaggedRect) -> tuple[int, str, BBox]:
    return (_source_keep_priority(tagged.source), tagged.source, tagged.bbox)


def _source_keep_priority(source: str) -> int:
    priorities = {
        "terminal_contact": 0,
        "terminal_adapter": 1,
        "route_tail": 2,
        "bus_route": 3,
        "bus_escape": 4,
        "pad": 5,
        "bus_stripe": 6,
    }
    return priorities.get(source, 10)


def _same_area(left: float, right: float) -> bool:
    return abs(left - right) <= 1e-9


def _normalized_tagged_bbox_sources(
    tagged_rects: tuple[TaggedRect, ...],
) -> tuple[tuple[BBox, str], ...]:
    bbox_sources: list[tuple[BBox, str]] = []
    for tagged in tagged_rects:
        normalized_bbox = normalize_rect(tagged.bbox)
        if normalized_bbox is None:
            continue
        bbox: BBox = normalized_bbox
        bbox_sources.append((bbox, tagged.source))
    return tuple(bbox_sources)


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
