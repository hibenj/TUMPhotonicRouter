"""The contracts: every issue an electrical result can fail on."""

from __future__ import annotations

from typing import Any

from .metrics import min_rect_spacing, quality_metrics
from .net_geometry import NetGeometry, build_net_geometries
from .pad_slots import pad_access_bbox
from .pitch_grid import bbox_to_grid_cells
from .rect_geometry import rect_area, rect_intersection, rect_intersects, rect_is_covered_by_any
from .terminal_contacts import terminal_contact_bboxes
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
    PadPlan,
    TerminalBusRoute,
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
    net_geometries = build_net_geometries(
        obstacle_map,
        common_bus,
        common_bus_escape,
        detailed_bundle_routes,
        pad_plan,
        config,
    )

    common_bus_net = net_geometries[0]
    _verify_common_bus_terminal_contacts(issues, common_bus, common_bus_net.rects)
    _verify_common_bus_connectivity(issues, common_bus)
    _verify_common_bus_pad_contact(issues, common_bus_escape, common_bus_net.rects, config)

    if detailed_bundle_routes is not None:
        for route, net in zip(detailed_bundle_routes.routes, net_geometries[1:], strict=True):
            _verify_terminal_contact(
                issues,
                route.terminal,
                net.rects,
                route.pad_assignment.net_id if route.pad_assignment else None,
            )
            _verify_individual_pad_contact(issues, route, net.rects, config)

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
        metrics=quality_metrics(
            net_geometries,
            obstacle_map,
            common_bus,
            detailed_bundle_routes,
            pad_plan,
            config,
        ),
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


def _verify_common_bus_connectivity(
    issues: list[ElectricalVerificationIssue],
    common_bus: CommonBusRoutingResult,
) -> None:
    """Every bus branch must reach the stripe, directly or through other branches.

    The router's tree cells may include cells of branches that were later
    replaced (the pad-side swap), while realization draws only the final
    routes; a branch that ends on such a cell is metal that reaches nothing.
    Branches are accepted transitively: a branch whose cells touch the stripe
    or an already accepted branch is connected.
    """

    connected = set(common_bus.bus.cells)
    pending = list(common_bus.routes)
    progress = True
    while pending and progress:
        progress = False
        still_pending: list[TerminalBusRoute] = []
        for route in pending:
            if any(cell in connected for cell in route.path):
                connected.update(route.path)
                progress = True
            else:
                still_pending.append(route)
        pending = still_pending
    for route in pending:
        issues.append(
            ElectricalVerificationIssue(
                code="common_bus_route_disconnected",
                message=(
                    f"Common-bus branch of {route.terminal.id} reaches neither the bus "
                    "stripe nor another branch."
                ),
                net_id="common_bus",
                details={
                    "terminal_id": route.terminal.id,
                    "route_end_cell": route.path[-1] if route.path else None,
                },
            )
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
    contacted_bboxes = tuple(bbox for bbox in terminal_bboxes if _any_rect_intersects(rects, bbox))
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
            message=(f"Routed metal does not physically touch heater terminal {terminal.id}."),
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
    net_geometries: list[NetGeometry],
    obstacle_map: ElectricalObstacleMap,
) -> None:
    if not obstacle_map.raw_obstacle_bboxes:
        return
    for net in net_geometries:
        illegal_overlaps: set[BBox] = set()
        for rect in net.rects:
            for obstacle_bbox in obstacle_map.raw_obstacle_bboxes:
                overlap = rect_intersection(rect, obstacle_bbox)
                if overlap is None or rect_area(overlap) <= 0:
                    continue
                if rect_is_covered_by_any(overlap, net.allowed_physical_bboxes):
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
    net_geometries: list[NetGeometry],
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
    net_geometries: list[NetGeometry],
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
                    message=(f"Metal for {left.net_id} overlaps metal for {right.net_id}."),
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
    net_geometries: list[NetGeometry],
    config: ElectricalRoutingConfig,
) -> None:
    required_clearance = max(0.0, config.obstacle_clearance_um)
    if required_clearance <= 0.0:
        return
    for index, left in enumerate(net_geometries):
        for right in net_geometries[index + 1 :]:
            if left.net_id == right.net_id:
                continue
            min_spacing = min_rect_spacing(left.rects, right.rects)
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


def _any_rect_intersects(rects: tuple[BBox, ...], target: BBox) -> bool:
    return any(rect_intersects(rect, target) for rect in rects)


def _rect_overlap_samples(
    left_rects: tuple[BBox, ...],
    right_rects: tuple[BBox, ...],
) -> tuple[BBox, ...]:
    overlaps: list[BBox] = []
    for left in left_rects:
        for right in right_rects:
            overlap = rect_intersection(left, right)
            if overlap is None:
                continue
            overlaps.append(overlap)
    return tuple(overlaps)
