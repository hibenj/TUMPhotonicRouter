"""The quality numbers reported in the benchmark summary, none of which is a
pass/fail contract."""

from __future__ import annotations

from collections import Counter
import math
from typing import Any

from .net_geometry import NetGeometry, detailed_route_centerline_points
from .rect_geometry import rect_area, rect_intersection, union_rect_area
from .types import (
    BBox,
    CommonBusRoutingResult,
    DetailedBundleRoutingResult,
    ElectricalObstacleMap,
    ElectricalPortAccess,
    ElectricalRoutingConfig,
    PadPlan,
)

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


def quality_metrics(
    net_geometries: list[NetGeometry],
    obstacle_map: ElectricalObstacleMap,
    common_bus: CommonBusRoutingResult,
    detailed_bundle_routes: DetailedBundleRoutingResult | None,
    pad_plan: PadPlan | None,
    config: ElectricalRoutingConfig,
) -> dict[str, Any]:
    rects_by_net = {net.net_id: net.rects for net in net_geometries}
    all_rects = tuple(rect for net in net_geometries for rect in net.rects)
    same_net_duplicate_rects = sum(_duplicate_rect_count(net.rects) for net in net_geometries)
    same_net_overlap_pairs = sum(_rect_overlap_pair_count(net.rects) for net in net_geometries)
    same_net_overlap_pairs_by_source = Counter[str]()
    same_net_intentional_overlap_pairs_by_reason = Counter[str]()
    same_net_redundant_overlap_pairs_by_source = Counter[str]()
    for net in net_geometries:
        same_net_overlap_pairs_by_source.update(_rect_overlap_pair_counts_by_source(net))
        classification = _classify_same_net_overlap_pairs(net)
        same_net_intentional_overlap_pairs_by_reason.update(classification["intentional"])
        same_net_redundant_overlap_pairs_by_source.update(classification["redundant"])
    same_net_intentional_overlap_pairs = sum(same_net_intentional_overlap_pairs_by_reason.values())
    same_net_redundant_overlap_pairs = sum(same_net_redundant_overlap_pairs_by_source.values())
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
        net.net_id: sum(rect_area(rect) for rect in net.rects) for net in net_geometries
    }
    union_area_by_net = {net.net_id: union_rect_area(net.rects) for net in net_geometries}
    raw_area = sum(raw_area_by_net.values())
    union_area = sum(union_area_by_net.values())
    area_overcount = raw_area - union_area
    min_spacing = _min_cross_net_spacing(net_geometries)
    access_metrics = _port_access_metrics(obstacle_map)
    route_start_metrics = _route_start_metrics(common_bus, detailed_bundle_routes)
    pad_wire_metrics = _pad_wire_metrics(detailed_bundle_routes, obstacle_map)
    common_bus_net = next(
        (net for net in net_geometries if net.net_id == "common_bus"),
        None,
    )
    bus_length_um = _net_centerline_length(common_bus_net) if common_bus_net is not None else 0.0
    bus_bend_count = _net_bend_count(common_bus_net) if common_bus_net is not None else 0
    wire_metal_area_um2, pad_metal_area_um2 = _wire_and_pad_metal_area(net_geometries)
    return {
        "net_count": len(net_geometries),
        "rect_count": len(all_rects),
        "rect_count_by_net": {net_id: len(rects) for net_id, rects in sorted(rects_by_net.items())},
        "raw_metal_area_um2": raw_area,
        "raw_metal_area_by_net_um2": dict(sorted(raw_area_by_net.items())),
        "union_metal_area_um2": union_area,
        "union_metal_area_by_net_um2": dict(sorted(union_area_by_net.items())),
        "metal_area_overcount_um2": area_overcount,
        "metal_area_overcount_ratio": (area_overcount / raw_area if raw_area > 0.0 else 0.0),
        "metal_area_overcount_by_reason_um2": dict(sorted(area_overcount_by_reason.items())),
        "metal_area_overcount_by_source_um2": dict(sorted(area_overcount_by_source.items())),
        "metal_redundant_area_overcount_um2": sum(redundant_area_overcount_by_source.values()),
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
        "centerline_length_um": sum(_net_centerline_length(net) for net in net_geometries),
        "bend_count": sum(_net_bend_count(net) for net in net_geometries),
        "pad_channel_height_um": _pad_channel_height_um(
            pad_plan,
            obstacle_map,
        ),
        "bus_length_um": bus_length_um,
        "bus_bend_count": bus_bend_count,
        "wire_metal_area_um2": wire_metal_area_um2,
        "pad_metal_area_um2": pad_metal_area_um2,
        **pad_wire_metrics,
        **access_metrics,
        **route_start_metrics,
    }


def _pad_wire_metrics(
    detailed_bundle_routes: DetailedBundleRoutingResult | None,
    obstacle_map: ElectricalObstacleMap,
) -> dict[str, Any]:
    """Per-wire detour/bend table for every successful detailed bundle route.

    ``detour_um`` is the centerline length minus the Manhattan distance
    between its endpoints; it is never negative for a Manhattan centerline.
    """

    pad_wires: list[dict[str, Any]] = []
    if detailed_bundle_routes is not None:
        for route in detailed_bundle_routes.routes:
            centerline = detailed_route_centerline_points(route, obstacle_map)
            length_um = _polyline_length(centerline)
            manhattan_um = _endpoint_manhattan_distance(centerline)
            detour_um = length_um - manhattan_um
            assert detour_um >= -1e-9, (
                f"pad wire detour must not be negative: {route.terminal.id} "
                f"length={length_um} manhattan={manhattan_um}"
            )
            detour_um = max(0.0, detour_um)
            pad_wires.append(
                {
                    "terminal_id": route.terminal.id,
                    "heater_id": route.terminal.heater_id,
                    "net_id": (
                        route.pad_assignment.net_id
                        if route.pad_assignment is not None
                        else f"individual:{route.terminal.heater_id}"
                    ),
                    "length_um": length_um,
                    "bend_count": _bend_count(centerline),
                    "manhattan_um": manhattan_um,
                    "detour_um": detour_um,
                    "shape": route.shape,
                }
            )
    pad_wires.sort(key=lambda entry: str(entry["terminal_id"]))
    detours = [float(entry["detour_um"]) for entry in pad_wires]
    bend_counts = [int(entry["bend_count"]) for entry in pad_wires]
    return {
        "pad_wires": pad_wires,
        "pad_wire_detour_total_um": sum(detours) if detours else 0.0,
        "pad_wire_max_detour_um": max(detours) if detours else 0.0,
        "pad_wire_max_bend_count": max(bend_counts) if bend_counts else 0,
    }


def _endpoint_manhattan_distance(
    points: tuple[tuple[float, float], ...],
) -> float:
    if len(points) < 2:
        return 0.0
    start, end = points[0], points[-1]
    return abs(end[0] - start[0]) + abs(end[1] - start[1])


def _wire_and_pad_metal_area(
    net_geometries: list[NetGeometry],
) -> tuple[float, float]:
    """Union metal area split into wire metal (everything but pads) and pads.

    Summed per net rather than globally: cross-net spacing already keeps
    different nets' rects disjoint, so a per-net union summed over nets
    equals the global union.
    """

    wire_area = 0.0
    pad_area = 0.0
    for net in net_geometries:
        wire_rects = tuple(
            rect
            for rect, source in zip(net.rects, net.rect_sources, strict=True)
            if source != "pad"
        )
        pad_rects = tuple(
            rect
            for rect, source in zip(net.rects, net.rect_sources, strict=True)
            if source == "pad"
        )
        wire_area += union_rect_area(wire_rects)
        pad_area += union_rect_area(pad_rects)
    return wire_area, pad_area


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
        str(record["purpose"]) for record in records if bool(record["used_access_anchor"])
    )
    biased_count_by_purpose = Counter(
        str(record["purpose"])
        for record in records
        if record["access_anchor_cell"] is not None and not bool(record["used_access_anchor"])
    )
    return {
        "port_access_route_start_count_by_purpose": dict(sorted(route_count_by_purpose.items())),
        "port_access_exact_anchor_route_count_by_purpose": dict(
            sorted(exact_count_by_purpose.items())
        ),
        "port_access_biased_route_count_by_purpose": dict(sorted(biased_count_by_purpose.items())),
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
    blocked_anchors = tuple(access for access in accesses if access.anchor_cell in blocked)
    missing_contact_accesses = tuple(access for access in accesses if not access.contact_bbox)
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


def _min_cross_net_spacing(
    net_geometries: list[NetGeometry],
) -> float | None:
    min_spacing: float | None = None
    for index, left in enumerate(net_geometries):
        for right in net_geometries[index + 1 :]:
            spacing = min_rect_spacing(left.rects, right.rects)
            if spacing is None:
                continue
            if min_spacing is None or spacing < min_spacing:
                min_spacing = spacing
    return min_spacing


def min_rect_spacing(
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
            overlap = rect_intersection(left, right)
            if overlap is None or rect_area(overlap) <= 0.0:
                continue
            count += 1
    return count


def _rect_overlap_pair_counts_by_source(net: NetGeometry) -> Counter[str]:
    counts = Counter[str]()
    for index, left in enumerate(net.rects):
        for right_index in range(index + 1, len(net.rects)):
            right = net.rects[right_index]
            overlap = rect_intersection(left, right)
            if overlap is None or rect_area(overlap) <= 0.0:
                continue
            source_pair = "/".join(sorted((net.rect_sources[index], net.rect_sources[right_index])))
            counts[source_pair] += 1
    return counts


def _classify_same_net_overlap_pairs(
    net: NetGeometry,
) -> dict[str, Counter[str]]:
    intentional = Counter[str]()
    redundant = Counter[str]()
    for index, left in enumerate(net.rects):
        for right_index in range(index + 1, len(net.rects)):
            right = net.rects[right_index]
            overlap = rect_intersection(left, right)
            if overlap is None or rect_area(overlap) <= 0.0:
                continue
            source_pair = _source_pair(net, index, right_index)
            if source_pair in _INTENTIONAL_SAME_NET_OVERLAP_PAIRS:
                intentional[_intentional_overlap_reason(source_pair)] += 1
                continue
            redundant["/".join(source_pair)] += 1
    return {"intentional": intentional, "redundant": redundant}


def _area_overcount_attribution(
    net: NetGeometry,
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
                if rect[0] < right and rect[2] > left and rect[1] < top and rect[3] > bottom
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
    net: NetGeometry,
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


def _net_centerline_length(net: NetGeometry) -> float:
    return sum(_polyline_length(polyline) for polyline in net.centerline_polylines_um)


def _net_bend_count(net: NetGeometry) -> int:
    return sum(_bend_count(polyline) for polyline in net.centerline_polylines_um)


def _polyline_length(points: tuple[tuple[float, float], ...]) -> float:
    return sum(
        abs(end[0] - start[0]) + abs(end[1] - start[1]) for start, end in zip(points, points[1:])
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
        assignment.slot for assignment in pad_plan.assignments if assignment.kind == "individual"
    )
    if not channel_slots:
        channel_slots = pad_plan.assigned_slots
    _, layout_ymin, _, layout_ymax = obstacle_map.layout_bbox
    if pad_plan.side == "top":
        return min(slot.bbox[1] for slot in channel_slots) - layout_ymax
    return layout_ymin - max(slot.bbox[3] for slot in channel_slots)
