"""Post-route verification for electrical heater metal routing."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from typing import Any

from .pad_slots import pad_access_bbox
from .pitch_grid import bbox_to_grid_cells
from .rect_geometry import (
    clip_manhattan_path_at_first_bbox_entry,
    clip_manhattan_path_start_at_bbox,
    disjoint_union_rects,
    union_rect_area,
    wire_rects_for_points,
)
from .terminal_contacts import terminal_access_path, terminal_contact_bboxes
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
    rect_sources: tuple[str, ...]
    allowed_cells: frozenset[GridCell]
    allowed_physical_bboxes: tuple[BBox, ...] = ()
    centerline_points_um: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class _TaggedRect:
    bbox: BBox
    source: str


_INTENTIONAL_SAME_NET_OVERLAP_PAIRS = frozenset(
    {
        ("bus_escape", "bus_escape"),
        ("bus_escape", "bus_stripe"),
        ("bus_escape", "pad"),
        ("bus_route", "bus_route"),
        ("bus_route", "bus_stripe"),
        ("bus_route", "terminal_adapter"),
        ("bus_route", "terminal_contact"),
        ("pad", "route_tail"),
        ("route_tail", "route_tail"),
        ("route_tail", "terminal_adapter"),
        ("route_tail", "terminal_contact"),
        ("terminal_adapter", "terminal_adapter"),
        ("terminal_adapter", "terminal_contact"),
    }
)


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

    pad_bboxes_by_net = _pad_bboxes_by_net(pad_plan)
    common_bus_tagged_rects = _clean_tagged_rects(
        (
            *_common_bus_tagged_rects(
                common_bus,
                common_bus_escape,
                obstacle_map,
                config,
            ),
            *(
                _TaggedRect(bbox, "pad")
                for bbox in pad_bboxes_by_net.get("common_bus", ())
            ),
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
        _NetGeometry(
            net_id="common_bus",
            rects=common_bus_rects,
            rect_sources=tuple(tagged.source for tagged in common_bus_tagged_rects),
            allowed_cells=frozenset(common_bus_allowed),
            allowed_physical_bboxes=_common_bus_allowed_physical_bboxes(
                common_bus,
                obstacle_map,
                config,
            ),
            centerline_points_um=common_bus_route_points,
        )
    )
    _verify_common_bus_terminal_contacts(issues, common_bus, common_bus_rects)
    _verify_common_bus_pad_contact(issues, common_bus_escape, common_bus_rects, config)

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
                    *(
                        _TaggedRect(bbox, "pad")
                        for bbox in pad_bboxes_by_net.get(route_net_id, ())
                    ),
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
                _NetGeometry(
                    net_id=route_net_id,
                    rects=route_rects,
                    rect_sources=tuple(tagged.source for tagged in route_tagged_rects),
                    allowed_cells=frozenset(allowed_cells),
                    allowed_physical_bboxes=(access.contact_bbox,),
                    centerline_points_um=_detailed_route_centerline_points(
                        route,
                        obstacle_map,
                    ),
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
    _verify_cross_net_spacing(issues, net_geometries, config)

    return ElectricalVerificationResult(
        issues=tuple(issues),
        metrics=_quality_metrics(
            net_geometries,
            obstacle_map,
            common_bus,
            detailed_bundle_routes,
            pad_plan,
            config,
        ),
    )


def _common_bus_tagged_rects(
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[_TaggedRect, ...]:
    rects: list[_TaggedRect] = [_TaggedRect(common_bus.bus.bbox, "bus_stripe")]
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
        allowed.update(
            _common_bus_terminal_open_cells(obstacle_map).get(route.terminal.id, ())
        )
        allowed.update(
            _terminal_contact_cells(
                route.terminal,
                obstacle_map,
                config.terminal_contact_width_um,
                route_start_um=(
                    _grid_cell_center_um(route.path[0], obstacle_map)
                    if route.path
                    else None
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
    return {
        net_id: tuple(bboxes)
        for net_id, bboxes in bboxes_by_net.items()
    }


def _common_bus_centerline_points(
    common_bus: CommonBusRoutingResult,
    common_bus_escape: CommonBusEscapeResult | None,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[tuple[float, float], ...]:
    points: list[tuple[float, float]] = []
    for route in common_bus.routes:
        points.extend(
            _grid_cell_center_um(cell, obstacle_map)
            for cell in route.path
        )
    if common_bus_escape is not None and common_bus_escape.success:
        points.extend(
            _grid_cell_center_um(cell, obstacle_map)
            for cell in common_bus_escape.path
        )
    return tuple(points)


def _detailed_route_centerline_points(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[tuple[float, float], ...]:
    return tuple(
        _grid_point_to_um(point, obstacle_map)
        for point in route.offset_path
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
    access = pad_access_bbox(
        common_bus_escape.pad_assignment.slot,
        config,
        width_um=config.bus_width_um,
    )
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


def _verify_cross_net_spacing(
    issues: list[ElectricalVerificationIssue],
    net_geometries: list[_NetGeometry],
    config: ElectricalRoutingConfig,
) -> None:
    required_clearance = max(0.0, config.obstacle_clearance_um)
    if required_clearance <= 0.0:
        return
    for index, left in enumerate(net_geometries):
        for right in net_geometries[index + 1 :]:
            if left.net_id == right.net_id:
                continue
            min_spacing = _min_rect_spacing(left.rects, right.rects)
            if min_spacing is None or min_spacing <= 0.0:
                continue
            if min_spacing >= required_clearance:
                continue
            issues.append(
                ElectricalVerificationIssue(
                    code="cross_net_metal_spacing",
                    message=(
                        f"Metal for {left.net_id} is closer than the required "
                        f"{required_clearance:.3f}um clearance to {right.net_id}."
                    ),
                    net_id=left.net_id,
                    details={
                        "other_net_id": right.net_id,
                        "required_clearance_um": required_clearance,
                        "actual_clearance_um": min_spacing,
                    },
                )
            )


def _quality_metrics(
    net_geometries: list[_NetGeometry],
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    detailed_bundle_routes: DetailedBundleRoutingResult | None,
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
) -> dict[str, Any]:
    rects_by_net = {net.net_id: net.rects for net in net_geometries}
    all_rects = tuple(rect for net in net_geometries for rect in net.rects)
    same_net_duplicate_rects = sum(
        _duplicate_rect_count(net.rects)
        for net in net_geometries
    )
    same_net_overlap_pairs = sum(
        _rect_overlap_pair_count(net.rects)
        for net in net_geometries
    )
    same_net_overlap_pairs_by_source = Counter[str]()
    same_net_intentional_overlap_pairs_by_reason = Counter[str]()
    same_net_redundant_overlap_pairs_by_source = Counter[str]()
    for net in net_geometries:
        same_net_overlap_pairs_by_source.update(_rect_overlap_pair_counts_by_source(net))
        classification = _classify_same_net_overlap_pairs(net)
        same_net_intentional_overlap_pairs_by_reason.update(classification["intentional"])
        same_net_redundant_overlap_pairs_by_source.update(classification["redundant"])
    same_net_intentional_overlap_pairs = sum(
        same_net_intentional_overlap_pairs_by_reason.values()
    )
    same_net_redundant_overlap_pairs = sum(
        same_net_redundant_overlap_pairs_by_source.values()
    )
    area_overcount_by_reason: dict[str, float] = {}
    area_overcount_by_source: dict[str, float] = {}
    redundant_area_overcount_by_source: dict[str, float] = {}
    for net in net_geometries:
        area_attribution = _area_overcount_attribution(net)
        _add_float_values(area_overcount_by_reason, area_attribution["by_reason"])
        _add_float_values(area_overcount_by_source, area_attribution["by_source"])
        _add_float_values(
            redundant_area_overcount_by_source,
            area_attribution["redundant_by_source"],
        )
    raw_area_by_net = {
        net.net_id: sum(_rect_area(rect) for rect in net.rects)
        for net in net_geometries
    }
    union_area_by_net = {
        net.net_id: union_rect_area(net.rects)
        for net in net_geometries
    }
    raw_area = sum(raw_area_by_net.values())
    union_area = sum(union_area_by_net.values())
    area_overcount = raw_area - union_area
    min_spacing = _min_cross_net_spacing(net_geometries)
    access_metrics = _port_access_metrics(obstacle_map)
    route_start_metrics = _route_start_metrics(common_bus, detailed_bundle_routes)
    return {
        "net_count": len(net_geometries),
        "rect_count": len(all_rects),
        "rect_count_by_net": {
            net_id: len(rects)
            for net_id, rects in sorted(rects_by_net.items())
        },
        "raw_metal_area_um2": raw_area,
        "raw_metal_area_by_net_um2": dict(sorted(raw_area_by_net.items())),
        "union_metal_area_um2": union_area,
        "union_metal_area_by_net_um2": dict(sorted(union_area_by_net.items())),
        "metal_area_overcount_um2": area_overcount,
        "metal_area_overcount_ratio": (
            area_overcount / raw_area
            if raw_area > 0.0
            else 0.0
        ),
        "metal_area_overcount_by_reason_um2": dict(
            sorted(area_overcount_by_reason.items())
        ),
        "metal_area_overcount_by_source_um2": dict(
            sorted(area_overcount_by_source.items())
        ),
        "metal_redundant_area_overcount_um2": sum(
            redundant_area_overcount_by_source.values()
        ),
        "metal_redundant_area_overcount_by_source_um2": dict(
            sorted(redundant_area_overcount_by_source.items())
        ),
        "same_net_duplicate_rect_count": same_net_duplicate_rects,
        "same_net_overlap_pair_count": same_net_overlap_pairs,
        "same_net_overlap_pair_count_by_source": dict(
            sorted(same_net_overlap_pairs_by_source.items())
        ),
        "same_net_intentional_overlap_pair_count": same_net_intentional_overlap_pairs,
        "same_net_intentional_overlap_pair_count_by_reason": dict(
            sorted(same_net_intentional_overlap_pairs_by_reason.items())
        ),
        "same_net_redundant_overlap_pair_count": same_net_redundant_overlap_pairs,
        "same_net_redundant_overlap_pair_count_by_source": dict(
            sorted(same_net_redundant_overlap_pairs_by_source.items())
        ),
        "cross_net_min_spacing_um": min_spacing,
        "required_cross_net_clearance_um": max(0.0, config.obstacle_clearance_um),
        "centerline_length_um": sum(
            _polyline_length(net.centerline_points_um)
            for net in net_geometries
        ),
        "bend_count": sum(
            _bend_count(net.centerline_points_um)
            for net in net_geometries
        ),
        "pad_channel_height_um": _pad_channel_height_um(
            pad_plan,
            obstacle_map,
        ),
        **access_metrics,
        **route_start_metrics,
    }


def _route_start_metrics(
    common_bus: CommonBusRoutingResult,
    detailed_bundle_routes: DetailedBundleRoutingResult | None,
) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    for route in common_bus.routes:
        records.append(
            {
                "purpose": "common_bus",
                "terminal_id": route.terminal.id,
                "route_start_cell": route.route_start_cell,
                "access_anchor_cell": route.access_anchor_cell,
                "used_access_anchor": route.used_access_anchor,
            }
        )
    if detailed_bundle_routes is not None:
        for route in detailed_bundle_routes.routes:
            records.append(
                {
                    "purpose": "individual",
                    "terminal_id": route.terminal.id,
                    "route_start_cell": route.route_start_cell,
                    "access_anchor_cell": route.access_anchor_cell,
                    "used_access_anchor": route.used_access_anchor,
                }
            )
    route_count_by_purpose = Counter(str(record["purpose"]) for record in records)
    exact_count_by_purpose = Counter(
        str(record["purpose"])
        for record in records
        if bool(record["used_access_anchor"])
    )
    biased_count_by_purpose = Counter(
        str(record["purpose"])
        for record in records
        if record["access_anchor_cell"] is not None
        and not bool(record["used_access_anchor"])
    )
    return {
        "port_access_route_start_count_by_purpose": dict(
            sorted(route_count_by_purpose.items())
        ),
        "port_access_exact_anchor_route_count_by_purpose": dict(
            sorted(exact_count_by_purpose.items())
        ),
        "port_access_biased_route_count_by_purpose": dict(
            sorted(biased_count_by_purpose.items())
        ),
        "port_access_route_start_records": sorted(
            records,
            key=lambda record: (
                str(record["purpose"]),
                str(record["terminal_id"]),
            ),
        ),
    }


def _port_access_metrics(obstacle_map: ElectricalObstacleMap) -> dict[str, Any]:
    accesses = tuple(_all_port_accesses(obstacle_map))
    blocked = set(obstacle_map.blocked_cells)
    blocked_anchors = tuple(
        access for access in accesses if access.anchor_cell in blocked
    )
    missing_contact_accesses = tuple(
        access for access in accesses if not access.contact_bbox
    )
    access_count_by_purpose = Counter(access.purpose for access in accesses)
    return {
        "port_access_count": len(accesses),
        "port_access_count_by_purpose": dict(sorted(access_count_by_purpose.items())),
        "port_access_max_offset_um": max(
            (
                math.hypot(
                    access.anchor_point_um[0] - access.port_point_um[0],
                    access.anchor_point_um[1] - access.port_point_um[1],
                )
                for access in accesses
            ),
            default=0.0,
        ),
        "port_access_max_length_um": max(
            (access.access_length_um for access in accesses),
            default=0.0,
        ),
        "port_access_blocked_anchor_count": len(blocked_anchors),
        "port_access_missing_contact_count": len(missing_contact_accesses),
    }


def _all_port_accesses(
    obstacle_map: ElectricalObstacleMap,
) -> tuple[ElectricalPortAccess, ...]:
    accesses_by_key: dict[tuple[str, str], ElectricalPortAccess] = {}
    for terminal_id, access in obstacle_map.common_bus_port_accesses.items():
        accesses_by_key[("common_bus", terminal_id)] = access
    for terminal_id, access in obstacle_map.individual_port_accesses.items():
        accesses_by_key[("individual", terminal_id)] = access
    return tuple(
        access
        for _, access in sorted(
            accesses_by_key.items(),
            key=lambda item: (item[0][0], item[0][1]),
        )
    )


def _detailed_route_tagged_rects(
    route: DetailedBundleRoute,
    obstacle_map: ElectricalObstacleMap,
    config: ElectricalRoutingConfig,
) -> tuple[_TaggedRect, ...]:
    return _terminal_point_route_tagged_rects(
        route.terminal,
        route.offset_path,
        obstacle_map,
        route_width_um=config.wire_width_um,
        contact_width_um=config.terminal_contact_width_um,
        route_source="route_tail",
        access=_individual_access(obstacle_map, route.terminal),
    )


def _tagged_grid_wire_rects(
    path: tuple[GridCell, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
    source: str,
    *,
    start_clip_bbox: BBox | None = None,
) -> tuple[_TaggedRect, ...]:
    points = tuple(
        _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map)
        for cell in path
    )
    if start_clip_bbox is not None:
        points = clip_manhattan_path_start_at_bbox(points, start_clip_bbox)
    return _tagged_point_wire_rects(points, width_um, source)


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
) -> tuple[_TaggedRect, ...]:
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
) -> tuple[_TaggedRect, ...]:
    return _terminal_access_tagged_rects(
        _terminal_point_route_access(
            terminal,
            points_grid,
            obstacle_map,
            contact_width_um,
            access=access,
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
        _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map)
        for cell in path
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
    )


def _terminal_point_route_access(
    terminal: ElectricalTerminal,
    points_grid: tuple[GridPoint, ...],
    obstacle_map: ElectricalObstacleMap,
    width_um: float,
    *,
    access: ElectricalPortAccess | None = None,
) -> Any:
    points_um = tuple(_grid_point_to_um(point, obstacle_map) for point in points_grid)
    return terminal_access_path(
        terminal,
        points_um,
        fallback_width_um=width_um,
        preferred_port_name=access.port_name if access is not None else None,
    )


def _terminal_access_tagged_rects(
    access: Any,
    width_um: float,
    *,
    route_source: str,
) -> tuple[_TaggedRect, ...]:
    rects: list[_TaggedRect] = [_TaggedRect(access.contact_bbox, "terminal_contact")]
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


def _grid_cell_center_um(
    cell: GridCell,
    obstacle_map: ElectricalObstacleMap,
) -> tuple[float, float]:
    return _grid_point_to_um((cell[0] + 0.5, cell[1] + 0.5), obstacle_map)


def _tagged_point_wire_rects(
    points: tuple[tuple[float, float], ...],
    width_um: float,
    source: str,
    *,
    trim_bends: bool = True,
) -> tuple[_TaggedRect, ...]:
    return tuple(
        _TaggedRect(rect, source)
        for rect in wire_rects_for_points(points, width_um, trim_bends=trim_bends)
    )


def _clean_tagged_rects(
    tagged_rects: tuple[_TaggedRect, ...],
) -> tuple[_TaggedRect, ...]:
    bboxes_by_source: dict[str, list[BBox]] = {}
    for bbox, source in _normalized_tagged_bbox_sources(tagged_rects):
        bboxes_by_source.setdefault(source, []).append(bbox)
    source_disjoint_rects = tuple(
        _TaggedRect(bbox, source)
        for source, bboxes in sorted(bboxes_by_source.items())
        for bbox in disjoint_union_rects(bboxes)
    )
    return _drop_union_redundant_tagged_rects(source_disjoint_rects)


def _drop_union_redundant_tagged_rects(
    tagged_rects: tuple[_TaggedRect, ...],
) -> tuple[_TaggedRect, ...]:
    kept = sorted(tagged_rects, key=_tagged_rect_sort_key)
    index = 0
    while index < len(kept):
        without_candidate = tuple(
            tagged.bbox
            for other_index, tagged in enumerate(kept)
            if other_index != index
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


def _tagged_rect_sort_key(tagged: _TaggedRect) -> tuple[int, str, BBox]:
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
    tagged_rects: tuple[_TaggedRect, ...],
) -> tuple[tuple[BBox, str], ...]:
    bbox_sources: list[tuple[BBox, str]] = []
    for tagged in tagged_rects:
        normalized_bbox = _normalized_bbox(tagged.bbox)
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


def _min_cross_net_spacing(
    net_geometries: list[_NetGeometry],
) -> float | None:
    min_spacing: float | None = None
    for index, left in enumerate(net_geometries):
        for right in net_geometries[index + 1 :]:
            spacing = _min_rect_spacing(left.rects, right.rects)
            if spacing is None:
                continue
            if min_spacing is None or spacing < min_spacing:
                min_spacing = spacing
    return min_spacing


def _min_rect_spacing(
    left_rects: tuple[BBox, ...],
    right_rects: tuple[BBox, ...],
) -> float | None:
    min_spacing: float | None = None
    for left in left_rects:
        for right in right_rects:
            spacing = _rect_spacing(left, right)
            if min_spacing is None or spacing < min_spacing:
                min_spacing = spacing
    return min_spacing


def _rect_spacing(left: BBox, right: BBox) -> float:
    x_gap = max(right[0] - left[2], left[0] - right[2], 0.0)
    y_gap = max(right[1] - left[3], left[1] - right[3], 0.0)
    return math.hypot(x_gap, y_gap)


def _duplicate_rect_count(rects: tuple[BBox, ...]) -> int:
    seen: set[BBox] = set()
    duplicates = 0
    for rect in rects:
        if rect in seen:
            duplicates += 1
        seen.add(rect)
    return duplicates


def _rect_overlap_pair_count(rects: tuple[BBox, ...]) -> int:
    count = 0
    for index, left in enumerate(rects):
        for right in rects[index + 1 :]:
            overlap = _rect_intersection(left, right)
            if overlap is None or _rect_area(overlap) <= 0.0:
                continue
            count += 1
    return count


def _rect_overlap_pair_counts_by_source(net: _NetGeometry) -> Counter[str]:
    counts = Counter[str]()
    for index, left in enumerate(net.rects):
        for right_index in range(index + 1, len(net.rects)):
            right = net.rects[right_index]
            overlap = _rect_intersection(left, right)
            if overlap is None or _rect_area(overlap) <= 0.0:
                continue
            source_pair = "/".join(
                sorted((net.rect_sources[index], net.rect_sources[right_index]))
            )
            counts[source_pair] += 1
    return counts


def _classify_same_net_overlap_pairs(
    net: _NetGeometry,
) -> dict[str, Counter[str]]:
    intentional = Counter[str]()
    redundant = Counter[str]()
    for index, left in enumerate(net.rects):
        for right_index in range(index + 1, len(net.rects)):
            right = net.rects[right_index]
            overlap = _rect_intersection(left, right)
            if overlap is None or _rect_area(overlap) <= 0.0:
                continue
            source_pair = _source_pair(net, index, right_index)
            if source_pair in _INTENTIONAL_SAME_NET_OVERLAP_PAIRS:
                intentional[_intentional_overlap_reason(source_pair)] += 1
                continue
            redundant["/".join(source_pair)] += 1
    return {"intentional": intentional, "redundant": redundant}


def _area_overcount_attribution(
    net: _NetGeometry,
) -> dict[str, dict[str, float]]:
    by_reason: dict[str, float] = {}
    by_source: dict[str, float] = {}
    redundant_by_source: dict[str, float] = {}
    if len(net.rects) < 2:
        return {
            "by_reason": by_reason,
            "by_source": by_source,
            "redundant_by_source": redundant_by_source,
        }
    x_edges = sorted({rect[0] for rect in net.rects} | {rect[2] for rect in net.rects})
    y_edges = sorted({rect[1] for rect in net.rects} | {rect[3] for rect in net.rects})
    for left, right in zip(x_edges, x_edges[1:]):
        if right <= left:
            continue
        for bottom, top in zip(y_edges, y_edges[1:]):
            if top <= bottom:
                continue
            covering = tuple(
                index
                for index, rect in enumerate(net.rects)
                if rect[0] < right
                and rect[2] > left
                and rect[1] < top
                and rect[3] > bottom
            )
            cover_count = len(covering)
            if cover_count < 2:
                continue
            cell_area = (right - left) * (top - bottom)
            pair_count = cover_count * (cover_count - 1) / 2.0
            pair_area = cell_area * (cover_count - 1) / pair_count
            for pair_index, left_index in enumerate(covering):
                for right_index in covering[pair_index + 1 :]:
                    source_pair = _source_pair(net, left_index, right_index)
                    source_key = "/".join(source_pair)
                    _add_float_value(by_source, source_key, pair_area)
                    if source_pair in _INTENTIONAL_SAME_NET_OVERLAP_PAIRS:
                        _add_float_value(
                            by_reason,
                            _intentional_overlap_reason(source_pair),
                            pair_area,
                        )
                        continue
                    _add_float_value(by_reason, "redundant", pair_area)
                    _add_float_value(redundant_by_source, source_key, pair_area)
    return {
        "by_reason": by_reason,
        "by_source": by_source,
        "redundant_by_source": redundant_by_source,
    }


def _add_float_values(target: dict[str, float], values: dict[str, float]) -> None:
    for key, value in values.items():
        _add_float_value(target, key, value)


def _add_float_value(target: dict[str, float], key: str, value: float) -> None:
    target[key] = target.get(key, 0.0) + value


def _source_pair(
    net: _NetGeometry,
    left_index: int,
    right_index: int,
) -> tuple[str, str]:
    sorted_sources = sorted((net.rect_sources[left_index], net.rect_sources[right_index]))
    return (sorted_sources[0], sorted_sources[1])


def _intentional_overlap_reason(source_pair: tuple[str, str]) -> str:
    sources = set(source_pair)
    if "pad" in sources:
        return "pad_contact"
    if "bus_stripe" in sources:
        return "bus_stripe_contact"
    if "terminal_adapter" in sources or "terminal_contact" in sources:
        return "terminal_access_join"
    if source_pair[0] == source_pair[1]:
        return "same_source_wire_join"
    return "same_net_join"


def _polyline_length(points: tuple[tuple[float, float], ...]) -> float:
    return sum(
        abs(end[0] - start[0]) + abs(end[1] - start[1])
        for start, end in zip(points, points[1:])
    )


def _bend_count(points: tuple[tuple[float, float], ...]) -> int:
    if len(points) < 3:
        return 0
    count = 0
    previous = _point_direction(points[0], points[1])
    for start, end in zip(points[1:], points[2:]):
        current = _point_direction(start, end)
        if current != (0, 0) and previous != (0, 0) and current != previous:
            count += 1
        if current != (0, 0):
            previous = current
    return count


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


def _pad_channel_height_um(
    pad_plan: PadPlan | None,
    obstacle_map: ElectricalObstacleMap,
) -> float | None:
    if pad_plan is None or not pad_plan.assigned_slots:
        return None
    channel_slots = tuple(
        assignment.slot
        for assignment in pad_plan.assignments
        if assignment.kind == "individual"
    )
    if not channel_slots:
        channel_slots = pad_plan.assigned_slots
    _, layout_ymin, _, layout_ymax = obstacle_map.layout_bbox
    if pad_plan.side == "top":
        return min(slot.bbox[1] for slot in channel_slots) - layout_ymax
    return layout_ymin - max(slot.bbox[3] for slot in channel_slots)


def _rect_intersection(left: BBox, right: BBox) -> BBox | None:
    if not _rect_intersects(left, right):
        return None
    return (
        max(left[0], right[0]),
        max(left[1], right[1]),
        min(left[2], right[2]),
        min(left[3], right[3]),
    )


def _normalized_bbox(rect: BBox) -> BBox | None:
    xmin, ymin, xmax, ymax = rect
    if xmax < xmin:
        xmin, xmax = xmax, xmin
    if ymax < ymin:
        ymin, ymax = ymax, ymin
    if xmax == xmin or ymax == ymin:
        return None
    return (xmin, ymin, xmax, ymax)


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
