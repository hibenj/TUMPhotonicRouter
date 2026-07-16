"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import json
import math
import os
import re
import sys
import time
from collections import Counter
from collections.abc import Iterable as IterableABC
from dataclasses import dataclass, is_dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

import gdsfactory as gf
from gdsfactory.component import Component
from gdsfactory.schematic import Schematic
from gdsfactory.typings import Port

from translation.route_gds import get_port_from_instance
from photonic_router.routing_layers import (
    find_component_port_access_rule,
    get_routing_obstacle_layers,
)
from photonic_router.crossing_plan import CrossingPlan, build_crossing_plan
from photonic_router.topology_analysis import analyze_schematic_topology
from translation.route_rust_analysis import (
    analysis_to_info_dict,
    analyze_path_length_matching,
    build_requirement_delay_candidates,
    compute_group_lifted_requirements,
    compute_output_matching_requirements,
    delay_candidate_to_dict,
    format_path_length_acceptance_failure,
    matching_group_diagnostics_to_info,
    merge_missing_length_requirements,
    minimum_four_bend_extra_length_um,
    output_matching_diagnostics_to_info,
    path_length_acceptance_summary,
    requirement_to_dict,
)
from translation import route_rust_meanders as _meander_impl
from translation.photonic_verification import (
    PhotonicVerificationIssue,
    PhotonicVerificationResult,
    verify_photonic_routing,
)
from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_records import (
    EndpointCorrectionRouter,
    RouteBookkeeping,
    apply_port_endpoint_corrections,
    build_port_alignment_diagnostics,
    build_route_debug_artifacts,
    format_port_endpoint_correction_error,
    route_edge_key,
    routed_edge_lengths_from_records,
)
from translation.route_rust_types import (
    DEFAULT_MEANDER_MAX_HEIGHT_UM,
    MeanderInsertionConfig,
    OpticalRouteClearancePolicy,
    RipupRerouteConfig,
    RouteJob,
    RouteRustPipelineResult,
    RouteTimingBucket,
    RoutedNetRecord,
    RustRouteDebugArtifacts,
    _as_float,
    bend_radius_cells_from_um,
    route_attempt_record_from_route,
    summarize_route_search,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend

DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING = 2
DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM = 50.0

_ILLEGAL_REALIZED_CROSSING_RE = re.compile(
    r"Illegal realized crossing:\s*net\s+"
    r"(?P<net_a>\d+)\s+intersects\s+net\s+"
    r"(?P<net_b>\d+)\s+at\s+\("
    r"(?P<x>[-+0-9.eE]+),\s*(?P<y>[-+0-9.eE]+)\)\s+"
    r"\((?P<reason>[^)]+)\)"
)


def _format_route_indices(indices: set[int]) -> str:
    if not indices:
        return "<none>"
    ranges: list[str] = []
    sorted_indices = sorted(indices)
    start = sorted_indices[0]
    previous = start
    for index in sorted_indices[1:]:
        if index == previous + 1:
            previous = index
            continue
        ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
        start = index
        previous = index
    ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
    return ",".join(ranges)


def _illegal_crossing_root_causes_from_texts(
    texts: Iterable[str | None],
) -> list[dict[str, object]]:
    root_causes: list[dict[str, object]] = []
    seen: set[tuple[int, int, float, float, str]] = set()
    for text in texts:
        if not text:
            continue
        for match in _ILLEGAL_REALIZED_CROSSING_RE.finditer(str(text)):
            try:
                net_a = int(match.group("net_a"))
                net_b = int(match.group("net_b"))
                x_um = float(match.group("x"))
                y_um = float(match.group("y"))
            except ValueError:
                continue
            reason = match.group("reason")
            key = (net_a, net_b, round(x_um, 6), round(y_um, 6), reason)
            if key in seen:
                continue
            seen.add(key)
            root_causes.append(
                {
                    "net_a": net_a,
                    "net_b": net_b,
                    "x_um": x_um,
                    "y_um": y_um,
                    "reason": reason,
                }
            )
    return root_causes


def _format_illegal_crossing_root_causes_line(
    texts: Iterable[str | None],
) -> str | None:
    root_causes = _illegal_crossing_root_causes_from_texts(texts)
    if not root_causes:
        return None
    return "root_cause_illegal_crossings=" + json.dumps(root_causes, sort_keys=True)


def _format_native_repair_trace_lines(
    records: Iterable[Mapping[str, object]],
    *,
    tail: int = 16,
) -> list[str]:
    normalized: list[dict[str, object]] = []
    for record in records:
        entry: dict[str, object] = {}
        for key in ("event", "route_order", "action", "error"):
            value = record.get(key)
            if value is not None:
                entry[key] = str(value)
        for key in ("net_id", "repair_round", "repair_set_index"):
            value = record.get(key)
            if value is not None:
                try:
                    entry[key] = int(value)
                except (TypeError, ValueError):
                    continue
        for key in ("candidate_blockers", "ripup_ids", "victim_order"):
            value = record.get(key)
            if value is None:
                continue
            try:
                entry[key] = [int(item) for item in cast(Iterable[object], value)]
            except (TypeError, ValueError):
                continue
        for key in ("victim_first", "reverse_victim_order", "success"):
            value = record.get(key)
            if value is not None:
                entry[key] = bool(value)
        if entry:
            normalized.append(entry)
    if not normalized:
        return []
    trace_tail = normalized[-tail:] if tail > 0 else []
    return [
        f"native_repair_trace_count={len(normalized)}",
        "native_repair_trace_tail=" + json.dumps(trace_tail, sort_keys=True),
    ]


def _centerline_tuple(points: object) -> tuple[tuple[float, float], ...]:
    if not isinstance(points, list):
        return ()
    out: list[tuple[float, float]] = []
    for point in points:
        if not isinstance(point, (tuple, list)) or len(point) != 2:
            return ()
        try:
            out.append((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            return ()
    return tuple(out)


def _route_waypoints_from_obj(route_obj: object | None) -> tuple[tuple[float, float], ...]:
    if route_obj is None:
        return ()

    for attr_name in ("compressed_waypoints", "cells"):
        raw_points = getattr(route_obj, attr_name, None)
        if raw_points is None:
            continue
        points: list[tuple[float, float]] = []
        for point in cast(Iterable[Any], raw_points):
            if not isinstance(point, (tuple, list)) or len(point) != 2:
                points = []
                break
            try:
                points.append((float(point[0]), float(point[1])))
            except (TypeError, ValueError):
                points = []
                break
        if len(points) >= 2:
            return tuple(points)

    raw_states = getattr(route_obj, "states", None)
    if raw_states is None:
        return ()
    points = []
    for state in cast(Iterable[Any], raw_states):
        try:
            points.append((float(getattr(state, "x")), float(getattr(state, "y"))))
        except (TypeError, ValueError):
            return ()
    return tuple(points) if len(points) >= 2 else ()


def _route_segments_from_waypoints(
    waypoints: tuple[tuple[float, float], ...],
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in zip(waypoints, waypoints[1:]):
        if start == end:
            continue
        segments.append((start, end))
    return tuple(segments)


def _points_are_collinear(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> bool:
    ab = (b[0] - a[0], b[1] - a[1])
    bc = (c[0] - b[0], c[1] - b[1])
    cross = ab[0] * bc[1] - ab[1] * bc[0]
    dot = ab[0] * bc[0] + ab[1] * bc[1]
    return abs(cross) < 1e-9 and dot >= -1e-9


def _compress_centerline(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    if len(points) < 3:
        return points
    out: list[tuple[float, float]] = []
    for point in points:
        if len(out) >= 2 and _points_are_collinear(out[-2], out[-1], point):
            out.pop()
        if not out or out[-1] != point:
            out.append(point)
    return tuple(out)


def _segment_intersection_with_params(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> tuple[float, float, float, float] | None:
    ax = a1[0] - a0[0]
    ay = a1[1] - a0[1]
    bx = b1[0] - b0[0]
    by = b1[1] - b0[1]
    denom = ax * by - ay * bx
    if abs(denom) < 1e-9:
        return None

    cx = b0[0] - a0[0]
    cy = b0[1] - a0[1]
    t = (cx * by - cy * bx) / denom
    u = (cx * ay - cy * ax) / denom
    eps = 1e-9
    if -eps <= t <= 1.0 + eps and -eps <= u <= 1.0 + eps:
        return (a0[0] + t * ax, a0[1] + t * ay, t, u)
    return None


def _collinear_segment_overlap_with_params(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    float,
    float,
    float,
    float,
] | None:
    ax = a1[0] - a0[0]
    ay = a1[1] - a0[1]
    bx = b1[0] - b0[0]
    by = b1[1] - b0[1]
    eps = 1e-9
    if abs(ax) < eps and abs(ay) < eps:
        return None
    if abs(bx) < eps and abs(by) < eps:
        return None
    if abs(ax * by - ay * bx) >= eps:
        return None
    offset_x = b0[0] - a0[0]
    offset_y = b0[1] - a0[1]
    if abs(offset_x * ay - offset_y * ax) >= eps:
        return None

    axis = 0 if abs(ax) >= abs(ay) else 1
    a_denom = ax if axis == 0 else ay
    b_denom = bx if axis == 0 else by
    if abs(a_denom) < eps or abs(b_denom) < eps:
        return None

    b_t0 = (b0[axis] - a0[axis]) / a_denom
    b_t1 = (b1[axis] - a0[axis]) / a_denom
    t_start = max(0.0, min(b_t0, b_t1))
    t_end = min(1.0, max(b_t0, b_t1))
    if t_end - t_start <= eps:
        return None

    overlap_start = (a0[0] + t_start * ax, a0[1] + t_start * ay)
    overlap_end = (a0[0] + t_end * ax, a0[1] + t_end * ay)
    u_start = (overlap_start[axis] - b0[axis]) / b_denom
    u_end = (overlap_end[axis] - b0[axis]) / b_denom
    return (overlap_start, overlap_end, t_start, t_end, u_start, u_end)


def _segment_length_cells(
    segment: tuple[tuple[float, float], tuple[float, float]],
) -> float:
    start, end = segment
    return max(abs(end[0] - start[0]), abs(end[1] - start[1]))


def _segment_bbox_um(
    segment: tuple[tuple[float, float], tuple[float, float]],
) -> tuple[float, float, float, float]:
    start, end = segment
    return (
        min(float(start[0]), float(end[0])),
        min(float(start[1]), float(end[1])),
        max(float(start[0]), float(end[0])),
        max(float(start[1]), float(end[1])),
    )


def _um_bboxes_overlap(
    bbox_a: tuple[float, float, float, float],
    bbox_b: tuple[float, float, float, float],
    *,
    eps: float = 1.0e-9,
) -> bool:
    return not (
        bbox_a[2] < bbox_b[0] - eps
        or bbox_b[2] < bbox_a[0] - eps
        or bbox_a[3] < bbox_b[1] - eps
        or bbox_b[3] < bbox_a[1] - eps
    )


def _segments_are_perpendicular(
    segment_a: tuple[tuple[float, float], tuple[float, float]],
    segment_b: tuple[tuple[float, float], tuple[float, float]],
) -> bool:
    a0, a1 = segment_a
    b0, b1 = segment_b
    av = (a1[0] - a0[0], a1[1] - a0[1])
    bv = (b1[0] - b0[0], b1[1] - b0[1])
    return abs(av[0] * bv[0] + av[1] * bv[1]) < 1e-9


def _first_perpendicular_route_intersection(
    route_obj_a: object | None,
    route_obj_b: object | None,
) -> dict[str, object] | None:
    waypoints_a = _route_waypoints_from_obj(route_obj_a)
    waypoints_b = _route_waypoints_from_obj(route_obj_b)
    if not waypoints_a or not waypoints_b:
        return None

    for segment_a in _route_segments_from_waypoints(waypoints_a):
        a0, a1 = segment_a
        for segment_b in _route_segments_from_waypoints(waypoints_b):
            b0, b1 = segment_b
            if not _segments_are_perpendicular(segment_a, segment_b):
                continue
            intersection = _segment_intersection_with_params(a0, a1, b0, b1)
            if intersection is None:
                continue
            point_x, point_y, t, u = intersection
            len_a = _segment_length_cells(segment_a)
            len_b = _segment_length_cells(segment_b)
            margin_a = min(max(t, 0.0) * len_a, max(1.0 - t, 0.0) * len_a)
            margin_b = min(max(u, 0.0) * len_b, max(1.0 - u, 0.0) * len_b)
            return {
                "point": [round(float(point_x), 6), round(float(point_y), 6)],
                "segment_a": [
                    [round(float(a0[0]), 6), round(float(a0[1]), 6)],
                    [round(float(a1[0]), 6), round(float(a1[1]), 6)],
                ],
                "segment_b": [
                    [round(float(b0[0]), 6), round(float(b0[1]), 6)],
                    [round(float(b1[0]), 6), round(float(b1[1]), 6)],
                ],
                "segment_a_margin_cells": round(float(margin_a), 6),
                "segment_b_margin_cells": round(float(margin_b), 6),
            }
    return None


def _record_centerline_um(
    record: RoutedNetRecord,
    *,
    grid_size_um: float,
    origin_x_um: float,
    origin_y_um: float,
) -> tuple[tuple[float, float], ...]:
    if record.corrected_centerline_um:
        return _compress_centerline(record.corrected_centerline_um)
    waypoints = _route_waypoints_from_obj(record.route_obj)
    if not waypoints:
        return ()
    return _compress_centerline(
        tuple(
        (
            float(origin_x_um) + float(x) * float(grid_size_um),
            float(origin_y_um) + float(y) * float(grid_size_um),
        )
        for x, y in waypoints
        )
    )


def _physical_point_to_grid_cell(
    point: tuple[float, float],
    *,
    grid_size_um: float,
    origin_x_um: float,
    origin_y_um: float,
) -> tuple[int, int] | None:
    if grid_size_um <= 0.0:
        return None
    x, y = float(point[0]), float(point[1])
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return (
        int(math.floor((x - float(origin_x_um)) / float(grid_size_um))),
        int(math.floor((y - float(origin_y_um)) / float(grid_size_um))),
    )


def _grid_point_to_physical_um(
    point: tuple[float, float],
    *,
    grid_size_um: float,
    origin_x_um: float,
    origin_y_um: float,
) -> tuple[float, float]:
    return (
        float(origin_x_um) + float(point[0]) * float(grid_size_um),
        float(origin_y_um) + float(point[1]) * float(grid_size_um),
    )


def _grid_cell_neighborhood(
    cell: tuple[int, int],
    *,
    radius: int,
) -> set[tuple[int, int]]:
    cx, cy = int(cell[0]), int(cell[1])
    r = max(0, int(radius))
    return {
        (cx + dx, cy + dy)
        for dx in range(-r, r + 1)
        for dy in range(-r, r + 1)
    }


def _point_distance_um(
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _point_near_route_endpoint(
    point: tuple[float, float],
    centerline: tuple[tuple[float, float], ...],
    *,
    tolerance_um: float,
) -> bool:
    if not centerline:
        return False
    return (
        _point_distance_um(point, centerline[0]) <= float(tolerance_um)
        or _point_distance_um(point, centerline[-1]) <= float(tolerance_um)
    )


def _point_route_endpoint_distance_um(
    point: tuple[float, float],
    centerline: tuple[tuple[float, float], ...],
) -> float | None:
    if not centerline:
        return None
    return min(
        _point_distance_um(point, centerline[0]),
        _point_distance_um(point, centerline[-1]),
    )


def _point_near_record_port_endpoint(
    point: tuple[float, float],
    record: RoutedNetRecord,
    *,
    tolerance_um: float,
) -> bool:
    for endpoint in (record.source_port_center_um, record.target_port_center_um):
        if endpoint is None:
            continue
        if _point_distance_um(point, endpoint) <= float(tolerance_um):
            return True
    return False


def _point_record_port_endpoint_distance_um(
    point: tuple[float, float],
    record: RoutedNetRecord,
) -> float | None:
    distances = [
        _point_distance_um(point, endpoint)
        for endpoint in (record.source_port_center_um, record.target_port_center_um)
        if endpoint is not None
    ]
    if not distances:
        return None
    return min(distances)


def _rounded_point(point: tuple[float, float]) -> list[float]:
    return [round(float(point[0]), 6), round(float(point[1]), 6)]


def _rounded_segment(
    segment: tuple[tuple[float, float], tuple[float, float]],
) -> list[list[float]]:
    return [_rounded_point(segment[0]), _rounded_point(segment[1])]


def _crossing_footprint_half_extent_um(
    *,
    crossing_plan_info: Mapping[str, object],
    grid_size_um: float,
) -> float:
    crossing_device = crossing_plan_info.get("crossing_device", {})
    if isinstance(crossing_device, Mapping):
        component_bbox = crossing_device.get("component_bbox_um")
        if isinstance(component_bbox, (list, tuple)) and len(component_bbox) >= 2:
            try:
                footprint_um = max(float(component_bbox[0]), float(component_bbox[1]))
            except (TypeError, ValueError):
                footprint_um = 0.0
            if math.isfinite(footprint_um) and footprint_um > 0.0:
                return 0.5 * float(footprint_um)

    half_size_cells = max(
        0,
        int(crossing_plan_info.get("crossing_half_size_cells", 0) or 0),
    )
    return float(half_size_cells) * float(grid_size_um)


def _segment_length_um(
    segment: tuple[tuple[float, float], tuple[float, float]],
) -> float:
    start, end = segment
    return math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))


def _segment_unit_vector(
    segment: tuple[tuple[float, float], tuple[float, float]],
) -> tuple[float, float] | None:
    start, end = segment
    dx = float(end[0]) - float(start[0])
    dy = float(end[1]) - float(start[1])
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    return (dx / length, dy / length)


def _crossing_footprint_polygon(
    *,
    center: tuple[float, float],
    axis_u: tuple[float, float],
    axis_v: tuple[float, float],
    half_extent_um: float,
) -> list[tuple[float, float]]:
    cx, cy = center
    ux, uy = axis_u
    vx, vy = axis_v
    half = float(half_extent_um)
    return [
        (cx + sx * half * ux + sy * half * vx, cy + sx * half * uy + sy * half * vy)
        for sx, sy in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    ]


def _polygon_axes(
    polygon: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    axes: list[tuple[float, float]] = []
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        ex = end[0] - start[0]
        ey = end[1] - start[1]
        length = math.hypot(ex, ey)
        if length <= 1e-9:
            continue
        axes.append((-ey / length, ex / length))
    return axes


def _project_polygon(
    polygon: list[tuple[float, float]],
    axis: tuple[float, float],
) -> tuple[float, float]:
    projections = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
    return (min(projections), max(projections))


def _convex_polygons_overlap_with_area(
    polygon_a: list[tuple[float, float]],
    polygon_b: list[tuple[float, float]],
    *,
    eps: float = 1e-9,
) -> bool:
    for axis in _polygon_axes(polygon_a) + _polygon_axes(polygon_b):
        min_a, max_a = _project_polygon(polygon_a, axis)
        min_b, max_b = _project_polygon(polygon_b, axis)
        if max_a <= min_b + eps or max_b <= min_a + eps:
            return False
    return True


def _same_undirected_segment(
    segment_a: tuple[tuple[float, float], tuple[float, float]],
    segment_b: tuple[tuple[float, float], tuple[float, float]],
    *,
    eps: float = 1e-9,
) -> bool:
    def same_point(
        point_a: tuple[float, float],
        point_b: tuple[float, float],
    ) -> bool:
        return (
            abs(float(point_a[0]) - float(point_b[0])) <= eps
            and abs(float(point_a[1]) - float(point_b[1])) <= eps
        )

    return (
        same_point(segment_a[0], segment_b[0])
        and same_point(segment_a[1], segment_b[1])
    ) or (
        same_point(segment_a[0], segment_b[1])
        and same_point(segment_a[1], segment_b[0])
    )


def _segment_intersects_crossing_footprint_interior(
    segment: tuple[tuple[float, float], tuple[float, float]],
    *,
    center: tuple[float, float],
    axis_u: tuple[float, float],
    axis_v: tuple[float, float],
    half_extent_um: float,
    eps: float = 1e-9,
) -> bool:
    def to_local(point: tuple[float, float]) -> tuple[float, float]:
        dx = float(point[0]) - float(center[0])
        dy = float(point[1]) - float(center[1])
        return (dx * axis_u[0] + dy * axis_u[1], dx * axis_v[0] + dy * axis_v[1])

    p0 = to_local(segment[0])
    p1 = to_local(segment[1])
    half = float(half_extent_um)

    def strictly_inside(point: tuple[float, float]) -> bool:
        return abs(point[0]) < half - eps and abs(point[1]) < half - eps

    if strictly_inside(p0) or strictly_inside(p1):
        return True

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    t0 = 0.0
    t1 = 1.0
    for p, q in (
        (-dx, p0[0] + half),
        (dx, half - p0[0]),
        (-dy, p0[1] + half),
        (dy, half - p0[1]),
    ):
        if abs(p) <= eps:
            if q < -eps:
                return False
            continue
        r = q / p
        if p < 0.0:
            if r > t1 + eps:
                return False
            t0 = max(t0, r)
        else:
            if r < t0 - eps:
                return False
            t1 = min(t1, r)
    if t1 <= t0 + eps:
        return False
    mid_t = 0.5 * (max(0.0, t0) + min(1.0, t1))
    midpoint = (p0[0] + mid_t * dx, p0[1] + mid_t * dy)
    return strictly_inside(midpoint)


def _crossing_footprint_blockers(
    *,
    center: tuple[float, float],
    axis_u: tuple[float, float],
    axis_v: tuple[float, float],
    half_extent_um: float,
    segments_by_net_id: Mapping[
        int, tuple[tuple[tuple[float, float], tuple[float, float]], ...]
    ],
    allowed_segments: Mapping[
        int, tuple[tuple[tuple[float, float], tuple[float, float]], ...]
    ],
    net_names: Mapping[int, str],
) -> list[dict[str, object]]:
    blockers: list[dict[str, object]] = []
    for net_id, segments in segments_by_net_id.items():
        allowed_for_net = allowed_segments.get(net_id, ())
        for segment in segments:
            if any(_same_undirected_segment(segment, allowed) for allowed in allowed_for_net):
                continue
            if _segment_intersects_crossing_footprint_interior(
                segment,
                center=center,
                axis_u=axis_u,
                axis_v=axis_v,
                half_extent_um=half_extent_um,
            ):
                blockers.append(
                    {
                        "net_id": int(net_id),
                        "net_name": net_names.get(net_id, str(net_id)),
                        "segment_um": _rounded_segment(segment),
                    }
                )
    return blockers


def _verify_realized_route_intersections(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    realization_grid_spec: tuple[int, int, float, float, float],
) -> list[dict[str, object]]:
    if not crossing_plan_info.get("enabled"):
        crossing_plan_info["realized_intersections"] = []
        crossing_plan_info["illegal_realized_crossings"] = []
        crossing_plan_info["illegal_realized_crossing_count"] = 0
        return []

    _width, _height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    footprint_half_um = _crossing_footprint_half_extent_um(
        crossing_plan_info=crossing_plan_info,
        grid_size_um=float(grid_size_um),
    )
    search_required_margin_um = footprint_half_um + float(grid_size_um) * int(
        crossing_plan_info.get("bend_runout_cells_per_crossing", 0) or 0
    )
    required_margin_um = footprint_half_um
    allowed_pairs: set[frozenset[int]] = set()
    net_names: dict[int, str] = {}
    for raw_event in cast(Iterable[object], crossing_plan_info.get("events", [])):
        event = dict(cast(dict[str, object], raw_event))
        if not event.get("loaded"):
            continue
        net_id_a = int(cast(int, event["net_id_a"]))
        net_id_b = int(cast(int, event["net_id_b"]))
        allowed_pairs.add(frozenset((net_id_a, net_id_b)))
        net_names[net_id_a] = str(event.get("net_name_a", net_id_a))
        net_names[net_id_b] = str(event.get("net_name_b", net_id_b))

    records = sorted(routed_records_by_net_id.items())
    centerlines_by_id: dict[int, tuple[tuple[float, float], ...]] = {}
    missing_centerline_illegal: list[dict[str, object]] = []
    for net_id, record in records:
        centerline = _record_centerline_um(
            record,
            grid_size_um=float(grid_size_um),
            origin_x_um=float(origin_x_um),
            origin_y_um=float(origin_y_um),
        )
        if centerline:
            centerlines_by_id[net_id] = centerline
            continue

        reason = (
            "endpoint_correction_error"
            if record.endpoint_correction_error is not None
            else "missing_corrected_centerline"
        )
        missing_centerline_illegal.append(
            {
                "net_id_a": int(net_id),
                "net_id_b": None,
                "net_name_a": record.net_name,
                "net_name_b": None,
                "point_um": None,
                "grid_cell": None,
                "classification": "illegal_route_geometry",
                "reason": reason,
                "message": (
                    record.endpoint_correction_error
                    if record.endpoint_correction_error is not None
                    else (
                        "Route has no corrected physical centerline; crossing "
                        "verification cannot use compressed route waypoints."
                    )
                ),
                "route_waypoint_fallback_available": bool(
                    _route_waypoints_from_obj(record.route_obj)
                ),
            }
        )
        centerlines_by_id[net_id] = ()
    for net_id, record in records:
        net_names.setdefault(net_id, record.net_name)
    opened_cells_by_id = {
        net_id: {(int(x), int(y)) for x, y in record.opened_cells}
        for net_id, record in records
    }
    segments_by_net_id = {
        net_id: _route_segments_from_waypoints(centerline)
        for net_id, centerline in centerlines_by_id.items()
        if len(centerline) >= 2
    }
    segment_entries_by_net_id = {
        net_id: tuple(
            (segment, _segment_length_um(segment), _segment_bbox_um(segment))
            for segment in segments
            if _segment_length_um(segment) > 0.0
        )
        for net_id, segments in segments_by_net_id.items()
    }

    crossing_mode = str(crossing_plan_info.get("crossing_mode", "") or "").strip().lower()
    allow_unexpected = (
        crossing_mode == "lidar-pure"
        or not bool(crossing_plan_info.get("allow_only_expected_crossings", True))
    )
    realized: list[dict[str, object]] = []
    illegal: list[dict[str, object]] = []
    ignored_endpoint_access: list[dict[str, object]] = []
    seen: set[tuple[object, ...]] = set()
    eps = 1e-9
    for index_a, (net_id_a, record_a) in enumerate(records):
        segment_entries_a = segment_entries_by_net_id.get(net_id_a, ())
        if not segment_entries_a:
            continue
        for net_id_b, record_b in records[index_a + 1 :]:
            segment_entries_b = segment_entries_by_net_id.get(net_id_b, ())
            if not segment_entries_b:
                continue
            pair = frozenset((net_id_a, net_id_b))
            pair_expected = pair in allowed_pairs
            pair_allowed = allow_unexpected or pair_expected
            for segment_a, len_a, bbox_a in segment_entries_a:
                for segment_b, len_b, bbox_b in segment_entries_b:
                    if not _um_bboxes_overlap(bbox_a, bbox_b, eps=eps):
                        continue
                    overlap = _collinear_segment_overlap_with_params(
                        segment_a[0],
                        segment_a[1],
                        segment_b[0],
                        segment_b[1],
                    )
                    if overlap is not None:
                        (
                            overlap_start,
                            overlap_end,
                            t_start,
                            t_end,
                            u_start,
                            u_end,
                        ) = overlap
                        overlap_length_um = _segment_length_um(
                            (overlap_start, overlap_end)
                        )
                        if overlap_length_um <= eps:
                            continue
                        start_key = (
                            round(overlap_start[0] * 1_000_000),
                            round(overlap_start[1] * 1_000_000),
                        )
                        end_key = (
                            round(overlap_end[0] * 1_000_000),
                            round(overlap_end[1] * 1_000_000),
                        )
                        ordered_overlap_key = tuple(sorted((start_key, end_key)))
                        key = (
                            min(net_id_a, net_id_b),
                            max(net_id_a, net_id_b),
                            ordered_overlap_key[0],
                            ordered_overlap_key[1],
                        )
                        if key in seen:
                            continue
                        seen.add(key)
                        midpoint = (
                            0.5 * (overlap_start[0] + overlap_end[0]),
                            0.5 * (overlap_start[1] + overlap_end[1]),
                        )
                        grid_cell = _physical_point_to_grid_cell(
                            midpoint,
                            grid_size_um=float(grid_size_um),
                            origin_x_um=float(origin_x_um),
                            origin_y_um=float(origin_y_um),
                        )
                        margin_a = min(
                            max(t_start, 0.0) * len_a,
                            max(1.0 - t_start, 0.0) * len_a,
                            max(t_end, 0.0) * len_a,
                            max(1.0 - t_end, 0.0) * len_a,
                        )
                        margin_b = min(
                            max(u_start, 0.0) * len_b,
                            max(1.0 - u_start, 0.0) * len_b,
                            max(u_end, 0.0) * len_b,
                            max(1.0 - u_end, 0.0) * len_b,
                        )
                        record = {
                            "net_id_a": int(net_id_a),
                            "net_id_b": int(net_id_b),
                            "net_name_a": record_a.net_name,
                            "net_name_b": record_b.net_name,
                            "point_um": _rounded_point(midpoint),
                            "grid_cell": (
                                [int(grid_cell[0]), int(grid_cell[1])]
                                if grid_cell is not None
                                else None
                            ),
                            "overlap_start_um": _rounded_point(overlap_start),
                            "overlap_end_um": _rounded_point(overlap_end),
                            "overlap_length_um": round(float(overlap_length_um), 6),
                            "segment_a_um": _rounded_segment(segment_a),
                            "segment_b_um": _rounded_segment(segment_b),
                            "segment_a_margin_um": round(float(margin_a), 6),
                            "segment_b_margin_um": round(float(margin_b), 6),
                            "required_margin_um": round(float(required_margin_um), 6),
                            "crossing_footprint_half_um": round(
                                float(footprint_half_um),
                                6,
                            ),
                            "crossing_footprint_um": round(
                                2.0 * float(footprint_half_um),
                                6,
                            ),
                            "crossing_footprint_polygon_um": [],
                            "crossing_footprint_blockers": [],
                            "expected_pair": bool(pair_expected),
                            "perpendicular": False,
                            "classification": "illegal_unexpected_crossing",
                            "reason": "collinear_route_overlap",
                        }
                        realized.append(record)
                        illegal.append(record)
                        continue
                    intersection = _segment_intersection_with_params(
                        segment_a[0],
                        segment_a[1],
                        segment_b[0],
                        segment_b[1],
                    )
                    if intersection is None:
                        continue
                    point_x, point_y, t, u = intersection
                    key = (
                        min(net_id_a, net_id_b),
                        max(net_id_a, net_id_b),
                        round(point_x * 1_000_000),
                        round(point_y * 1_000_000),
                    )
                    if key in seen:
                        continue
                    seen.add(key)
                    grid_cell = _physical_point_to_grid_cell(
                        (point_x, point_y),
                        grid_size_um=float(grid_size_um),
                        origin_x_um=float(origin_x_um),
                        origin_y_um=float(origin_y_um),
                    )
                    margin_a = min(max(t, 0.0) * len_a, max(1.0 - t, 0.0) * len_a)
                    margin_b = min(max(u, 0.0) * len_b, max(1.0 - u, 0.0) * len_b)
                    endpoint_margin_tolerance_um = float(grid_size_um) + eps
                    port_access_tolerance_um = endpoint_margin_tolerance_um
                    if grid_cell is not None and (
                        grid_cell in opened_cells_by_id.get(net_id_a, set())
                        or grid_cell in opened_cells_by_id.get(net_id_b, set())
                    ):
                        point = (point_x, point_y)
                        if _point_near_route_endpoint(
                            point,
                            centerlines_by_id.get(net_id_a, ()),
                            tolerance_um=port_access_tolerance_um,
                        ) or _point_near_route_endpoint(
                            point,
                            centerlines_by_id.get(net_id_b, ()),
                            tolerance_um=port_access_tolerance_um,
                        ) or _point_near_record_port_endpoint(
                            point,
                            record_a,
                            tolerance_um=port_access_tolerance_um,
                        ) or _point_near_record_port_endpoint(
                            point,
                            record_b,
                            tolerance_um=port_access_tolerance_um,
                        ):
                            ignored_endpoint_access.append(
                                {
                                    "net_id_a": int(net_id_a),
                                    "net_id_b": int(net_id_b),
                                    "net_name_a": record_a.net_name,
                                    "net_name_b": record_b.net_name,
                                    "point_um": _rounded_point(point),
                                    "grid_cell": [int(grid_cell[0]), int(grid_cell[1])],
                                    "reason": "route_endpoint_access",
                                }
                            )
                            continue
                    if min(margin_a, margin_b) <= endpoint_margin_tolerance_um:
                        point = (point_x, point_y)
                        if _point_near_route_endpoint(
                            point,
                            centerlines_by_id.get(net_id_a, ()),
                            tolerance_um=endpoint_margin_tolerance_um,
                        ) or _point_near_route_endpoint(
                            point,
                            centerlines_by_id.get(net_id_b, ()),
                            tolerance_um=endpoint_margin_tolerance_um,
                        ) or _point_near_record_port_endpoint(
                            point,
                            record_a,
                            tolerance_um=endpoint_margin_tolerance_um,
                        ) or _point_near_record_port_endpoint(
                            point,
                            record_b,
                            tolerance_um=endpoint_margin_tolerance_um,
                        ):
                            ignored_endpoint_access.append(
                                {
                                    "net_id_a": int(net_id_a),
                                    "net_id_b": int(net_id_b),
                                    "net_name_a": record_a.net_name,
                                    "net_name_b": record_b.net_name,
                                    "point_um": _rounded_point(point),
                                    "grid_cell": (
                                        [int(grid_cell[0]), int(grid_cell[1])]
                                        if grid_cell is not None
                                        else None
                                    ),
                                    "reason": "route_endpoint_access",
                                }
                            )
                            continue
                    if grid_cell is not None and min(margin_a, margin_b) <= (
                        endpoint_margin_tolerance_um
                    ):
                        nearby_cells = _grid_cell_neighborhood(grid_cell, radius=1)
                        if nearby_cells.intersection(
                            opened_cells_by_id.get(net_id_a, set())
                        ) or nearby_cells.intersection(
                            opened_cells_by_id.get(net_id_b, set())
                        ):
                            point = (point_x, point_y)
                            nearby_endpoint_tolerance_um = (
                                endpoint_margin_tolerance_um + float(grid_size_um)
                            )
                            if _point_near_route_endpoint(
                                point,
                                centerlines_by_id.get(net_id_a, ()),
                                tolerance_um=nearby_endpoint_tolerance_um,
                            ) or _point_near_route_endpoint(
                                point,
                                centerlines_by_id.get(net_id_b, ()),
                                tolerance_um=nearby_endpoint_tolerance_um,
                            ) or _point_near_record_port_endpoint(
                                point,
                                record_a,
                                tolerance_um=nearby_endpoint_tolerance_um,
                            ) or _point_near_record_port_endpoint(
                                point,
                                record_b,
                                tolerance_um=nearby_endpoint_tolerance_um,
                            ):
                                ignored_endpoint_access.append(
                                    {
                                        "net_id_a": int(net_id_a),
                                        "net_id_b": int(net_id_b),
                                        "net_name_a": record_a.net_name,
                                        "net_name_b": record_b.net_name,
                                        "point_um": _rounded_point((point_x, point_y)),
                                        "grid_cell": [
                                            int(grid_cell[0]),
                                            int(grid_cell[1]),
                                        ],
                                        "reason": "near_endpoint_access_cell",
                                    }
                                )
                                continue
                    perpendicular = _segments_are_perpendicular(segment_a, segment_b)
                    contact_adjacent = margin_a <= eps and margin_b <= eps
                    axis_u = _segment_unit_vector(segment_a)
                    axis_v = _segment_unit_vector(segment_b)
                    footprint_polygon: list[tuple[float, float]] = []
                    footprint_blockers: list[dict[str, object]] = []
                    footprint_straight = (
                        required_margin_um <= eps
                        or (
                            margin_a + eps >= required_margin_um
                            and margin_b + eps >= required_margin_um
                        )
                    )
                    if (
                        axis_u is not None
                        and axis_v is not None
                        and footprint_half_um > eps
                    ):
                        footprint_axis_u = axis_u
                        footprint_axis_v = axis_v
                        if (
                            crossing_mode == "lidar-pure"
                            and pair_allowed
                            and (not perpendicular or not footprint_straight)
                        ):
                            footprint_axis_u = (1.0, 0.0)
                            footprint_axis_v = (0.0, 1.0)
                        footprint_polygon = _crossing_footprint_polygon(
                            center=(point_x, point_y),
                            axis_u=footprint_axis_u,
                            axis_v=footprint_axis_v,
                            half_extent_um=footprint_half_um,
                        )
                        if pair_allowed and footprint_straight:
                            footprint_blockers = _crossing_footprint_blockers(
                                center=(point_x, point_y),
                                axis_u=footprint_axis_u,
                                axis_v=footprint_axis_v,
                                half_extent_um=footprint_half_um,
                                segments_by_net_id=segments_by_net_id,
                                allowed_segments={
                                    net_id_a: (segment_a,),
                                    net_id_b: (segment_b,),
                                },
                                net_names=net_names,
                            )
                    legal = (
                        pair_allowed
                        and perpendicular
                        and footprint_straight
                        and not footprint_blockers
                    )
                    if legal:
                        classification = (
                            "legal_expected_crossing"
                            if pair_expected
                            else "legal_unexpected_crossing"
                        )
                        reason = None
                        degraded_reason = None
                    elif contact_adjacent:
                        classification = "contact_adjacent_geometry"
                        reason = "shared_segment_endpoint"
                        degraded_reason = None
                    else:
                        classification = "illegal_unexpected_crossing"
                        if not pair_allowed:
                            reason = "unexpected_pair"
                        elif not footprint_straight:
                            reason = "crossing_footprint_contains_bend"
                        elif not perpendicular:
                            reason = "not_perpendicular"
                        elif footprint_blockers:
                            reason = "crossing_footprint_contains_route_geometry"
                        else:
                            reason = "crossing_footprint_invalid"
                        degraded_reason = None
                    record = {
                        "net_id_a": int(net_id_a),
                        "net_id_b": int(net_id_b),
                        "net_name_a": record_a.net_name,
                        "net_name_b": record_b.net_name,
                        "point_um": _rounded_point((point_x, point_y)),
                        "grid_cell": (
                            [int(grid_cell[0]), int(grid_cell[1])]
                            if grid_cell is not None
                            else None
                        ),
                        "route_endpoint_distance_a_um": (
                            round(float(route_endpoint_distance_a), 6)
                            if (
                                route_endpoint_distance_a
                                := _point_route_endpoint_distance_um(
                                    (point_x, point_y),
                                    centerlines_by_id.get(net_id_a, ()),
                                )
                            )
                            is not None
                            else None
                        ),
                        "route_endpoint_distance_b_um": (
                            round(float(route_endpoint_distance_b), 6)
                            if (
                                route_endpoint_distance_b
                                := _point_route_endpoint_distance_um(
                                    (point_x, point_y),
                                    centerlines_by_id.get(net_id_b, ()),
                                )
                            )
                            is not None
                            else None
                        ),
                        "port_endpoint_distance_a_um": (
                            round(float(port_endpoint_distance_a), 6)
                            if (
                                port_endpoint_distance_a
                                := _point_record_port_endpoint_distance_um(
                                    (point_x, point_y),
                                    record_a,
                                )
                            )
                            is not None
                            else None
                        ),
                        "port_endpoint_distance_b_um": (
                            round(float(port_endpoint_distance_b), 6)
                            if (
                                port_endpoint_distance_b
                                := _point_record_port_endpoint_distance_um(
                                    (point_x, point_y),
                                    record_b,
                                )
                            )
                            is not None
                            else None
                        ),
                        "segment_a_um": _rounded_segment(segment_a),
                        "segment_b_um": _rounded_segment(segment_b),
                        "segment_a_margin_um": round(float(margin_a), 6),
                        "segment_b_margin_um": round(float(margin_b), 6),
                        "required_margin_um": round(float(required_margin_um), 6),
                        "crossing_footprint_half_um": round(
                            float(footprint_half_um),
                            6,
                        ),
                        "crossing_footprint_um": round(
                            2.0 * float(footprint_half_um),
                            6,
                        ),
                        "search_required_margin_um": round(
                            float(search_required_margin_um),
                            6,
                        ),
                        "crossing_footprint_polygon_um": [
                            _rounded_point(point) for point in footprint_polygon
                        ],
                        "crossing_footprint_blockers": footprint_blockers,
                        "expected_pair": bool(pair_expected),
                        "perpendicular": bool(perpendicular),
                        "classification": classification,
                    }
                    if reason is not None:
                        record["reason"] = reason
                    if degraded_reason is not None:
                        record["degraded_reason"] = degraded_reason
                    realized.append(record)
                    if classification.startswith("illegal_"):
                        illegal.append(record)

    legal_crossing_indices = [
        index
        for index, item in enumerate(realized)
        if str(item.get("classification", "")).startswith("legal_")
        and item.get("crossing_footprint_polygon_um")
    ]
    def _add_overlap_peer(
        crossing: dict[str, object],
        *,
        peer: dict[str, object],
        peer_index: int,
    ) -> None:
        crossing["overlapping_crossing"] = peer
        crossing["overlapping_crossing_index"] = int(peer_index)
        raw_indices = crossing.get("overlapping_crossing_indices")
        if isinstance(raw_indices, list):
            indices = raw_indices
        else:
            indices = []
        if int(peer_index) not in indices:
            indices.append(int(peer_index))
        crossing["overlapping_crossing_indices"] = indices

    for offset, index_a in enumerate(legal_crossing_indices):
        crossing_a = realized[index_a]
        polygon_a = [
            (float(point[0]), float(point[1]))
            for point in cast(
                list[list[float]],
                crossing_a.get("crossing_footprint_polygon_um", []),
            )
        ]
        for index_b in legal_crossing_indices[offset + 1 :]:
            crossing_b = realized[index_b]
            polygon_b = [
                (float(point[0]), float(point[1]))
                for point in cast(
                    list[list[float]],
                    crossing_b.get("crossing_footprint_polygon_um", []),
                )
            ]
            if not polygon_a or not polygon_b:
                continue
            if not _convex_polygons_overlap_with_area(polygon_a, polygon_b):
                continue
            overlap_peer_a = {
                "net_id_a": crossing_b.get("net_id_a"),
                "net_id_b": crossing_b.get("net_id_b"),
                "net_name_a": crossing_b.get("net_name_a"),
                "net_name_b": crossing_b.get("net_name_b"),
                "point_um": crossing_b.get("point_um"),
                "crossing_footprint_polygon_um": crossing_b.get(
                    "crossing_footprint_polygon_um",
                ),
            }
            overlap_peer_b = {
                "net_id_a": crossing_a.get("net_id_a"),
                "net_id_b": crossing_a.get("net_id_b"),
                "net_name_a": crossing_a.get("net_name_a"),
                "net_name_b": crossing_a.get("net_name_b"),
                "point_um": crossing_a.get("point_um"),
                "crossing_footprint_polygon_um": crossing_a.get(
                    "crossing_footprint_polygon_um",
                ),
            }
            ids_a = {
                int(value)
                for value in (crossing_a.get("net_id_a"), crossing_a.get("net_id_b"))
                if isinstance(value, int)
            }
            ids_b = {
                int(value)
                for value in (crossing_b.get("net_id_a"), crossing_b.get("net_id_b"))
                if isinstance(value, int)
            }
            if (
                crossing_mode == "lidar-pure"
                and crossing_a.get("degraded_reason") is not None
                and crossing_b.get("degraded_reason") is not None
                and ids_a.intersection(ids_b)
            ):
                crossing_a["footprint_overlap_policy"] = (
                    "allowed_lidar_pure_degraded_cluster"
                )
                _add_overlap_peer(
                    crossing_a,
                    peer=overlap_peer_a,
                    peer_index=int(index_b),
                )
                crossing_b["footprint_overlap_policy"] = (
                    "allowed_lidar_pure_degraded_cluster"
                )
                _add_overlap_peer(
                    crossing_b,
                    peer=overlap_peer_b,
                    peer_index=int(index_a),
                )
                continue
            for crossing, peer in (
                (crossing_a, overlap_peer_a),
                (crossing_b, overlap_peer_b),
            ):
                if str(crossing.get("classification", "")).startswith("illegal_"):
                    continue
                crossing["classification"] = "illegal_unexpected_crossing"
                crossing["reason"] = "crossing_footprint_overlap"
                _add_overlap_peer(
                    crossing,
                    peer=peer,
                    peer_index=int(index_b if crossing is crossing_a else index_a),
                )
                illegal.append(crossing)

    crossing_plan_info["realized_intersections"] = realized
    crossing_plan_info["realized_intersection_count"] = len(realized)
    crossing_plan_info["routes_missing_corrected_centerline"] = missing_centerline_illegal
    crossing_plan_info["routes_missing_corrected_centerline_count"] = len(
        missing_centerline_illegal
    )
    crossing_plan_info["ignored_endpoint_access_intersections"] = ignored_endpoint_access
    crossing_plan_info["ignored_endpoint_access_intersection_count"] = len(
        ignored_endpoint_access
    )
    crossing_plan_info["illegal_realized_crossings"] = illegal
    crossing_plan_info["illegal_realized_crossing_count"] = len(illegal)
    return illegal


def _populate_realized_intersections_from_native_crossing_events(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    native_crossing_events: Iterable[object],
    realization_grid_spec: tuple[int, int, float, float, float],
) -> list[dict[str, object]]:
    """Populate realized crossing metadata from A*-accepted native events.

    This is the normal fast path. A* is responsible for accepting only legal
    crossing moves; this function converts those accepted events into the
    physical metadata needed for crossing component placement and for the final
    external Python geometry verifier.
    """
    if not crossing_plan_info.get("enabled"):
        crossing_plan_info["realized_intersections"] = []
        crossing_plan_info["realized_intersection_count"] = 0
        crossing_plan_info["illegal_realized_crossings"] = []
        crossing_plan_info["illegal_realized_crossing_count"] = 0
        return []

    _width, _height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    footprint_half_um = _crossing_footprint_half_extent_um(
        crossing_plan_info=crossing_plan_info,
        grid_size_um=float(grid_size_um),
    )
    required_margin_um = footprint_half_um
    search_required_margin_um = footprint_half_um + float(grid_size_um) * int(
        crossing_plan_info.get("bend_runout_cells_per_crossing", 0) or 0
    )

    realized: list[dict[str, object]] = []
    seen: set[tuple[int, int, int, int]] = set()
    for raw_event in native_crossing_events:
        if not isinstance(raw_event, Mapping):
            try:
                raw_event = dict(cast(Any, raw_event))
            except (TypeError, ValueError):
                continue
        try:
            net_id_a = int(cast(Any, raw_event.get("net_id")))
            net_id_b = int(cast(Any, raw_event.get("partner_net_id")))
            raw_point = cast(Any, raw_event.get("point"))
            raw_route_segment = cast(Any, raw_event.get("route_segment"))
            raw_partner_segment = cast(Any, raw_event.get("partner_segment"))
        except (TypeError, ValueError):
            continue
        if (
            not isinstance(raw_point, (tuple, list))
            or len(raw_point) != 2
            or not isinstance(raw_route_segment, (tuple, list))
            or len(raw_route_segment) != 2
            or not isinstance(raw_partner_segment, (tuple, list))
            or len(raw_partner_segment) != 2
        ):
            continue
        try:
            grid_point = (float(raw_point[0]), float(raw_point[1]))
            route_segment_grid = (
                (float(raw_route_segment[0][0]), float(raw_route_segment[0][1])),
                (float(raw_route_segment[1][0]), float(raw_route_segment[1][1])),
            )
            partner_segment_grid = (
                (float(raw_partner_segment[0][0]), float(raw_partner_segment[0][1])),
                (float(raw_partner_segment[1][0]), float(raw_partner_segment[1][1])),
            )
        except (TypeError, ValueError, IndexError):
            continue

        segment_a = (
            _grid_point_to_physical_um(
                route_segment_grid[0],
                grid_size_um=float(grid_size_um),
                origin_x_um=float(origin_x_um),
                origin_y_um=float(origin_y_um),
            ),
            _grid_point_to_physical_um(
                route_segment_grid[1],
                grid_size_um=float(grid_size_um),
                origin_x_um=float(origin_x_um),
                origin_y_um=float(origin_y_um),
            ),
        )
        segment_b = (
            _grid_point_to_physical_um(
                partner_segment_grid[0],
                grid_size_um=float(grid_size_um),
                origin_x_um=float(origin_x_um),
                origin_y_um=float(origin_y_um),
            ),
            _grid_point_to_physical_um(
                partner_segment_grid[1],
                grid_size_um=float(grid_size_um),
                origin_x_um=float(origin_x_um),
                origin_y_um=float(origin_y_um),
            ),
        )
        raw_point_um = (float(grid_point[0]), float(grid_point[1]))
        raw_point_bbox = _segment_bbox_um((raw_point_um, raw_point_um))
        segment_a_bbox = _segment_bbox_um(segment_a)
        segment_b_bbox = _segment_bbox_um(segment_b)
        if _um_bboxes_overlap(
            raw_point_bbox,
            segment_a_bbox,
            eps=float(grid_size_um),
        ) and _um_bboxes_overlap(
            raw_point_bbox,
            segment_b_bbox,
            eps=float(grid_size_um),
        ):
            point_um = raw_point_um
        else:
            point_um = _grid_point_to_physical_um(
                grid_point,
                grid_size_um=float(grid_size_um),
                origin_x_um=float(origin_x_um),
                origin_y_um=float(origin_y_um),
            )
        key = (
            min(net_id_a, net_id_b),
            max(net_id_a, net_id_b),
            round(point_um[0] * 1_000_000),
            round(point_um[1] * 1_000_000),
        )
        if key in seen:
            continue
        seen.add(key)

        record_a = routed_records_by_net_id.get(net_id_a)
        record_b = routed_records_by_net_id.get(net_id_b)
        len_a = _segment_length_um(segment_a)
        len_b = _segment_length_um(segment_b)
        margin_a = min(
            _point_distance_um(point_um, segment_a[0]),
            _point_distance_um(point_um, segment_a[1]),
        )
        margin_b = min(
            _point_distance_um(point_um, segment_b[0]),
            _point_distance_um(point_um, segment_b[1]),
        )
        axis_u = _segment_unit_vector(segment_a)
        axis_v = _segment_unit_vector(segment_b)
        footprint_polygon: list[tuple[float, float]] = []
        if axis_u is not None and axis_v is not None and footprint_half_um > 0.0:
            footprint_polygon = _crossing_footprint_polygon(
                center=point_um,
                axis_u=axis_u,
                axis_v=axis_v,
                half_extent_um=footprint_half_um,
            )
        grid_cell = _physical_point_to_grid_cell(
            point_um,
            grid_size_um=float(grid_size_um),
            origin_x_um=float(origin_x_um),
            origin_y_um=float(origin_y_um),
        )
        realized.append(
            {
                "net_id_a": int(net_id_a),
                "net_id_b": int(net_id_b),
                "net_name_a": record_a.net_name if record_a is not None else str(net_id_a),
                "net_name_b": record_b.net_name if record_b is not None else str(net_id_b),
                "point_um": _rounded_point(point_um),
                "grid_cell": (
                    [int(grid_cell[0]), int(grid_cell[1])] if grid_cell is not None else None
                ),
                "segment_a_um": _rounded_segment(segment_a),
                "segment_b_um": _rounded_segment(segment_b),
                "segment_a_length_um": round(float(len_a), 6),
                "segment_b_length_um": round(float(len_b), 6),
                "segment_a_margin_um": round(float(margin_a), 6),
                "segment_b_margin_um": round(float(margin_b), 6),
                "required_margin_um": round(float(required_margin_um), 6),
                "search_required_margin_um": round(float(search_required_margin_um), 6),
                "crossing_footprint_half_um": round(float(footprint_half_um), 6),
                "crossing_footprint_um": round(2.0 * float(footprint_half_um), 6),
                "crossing_footprint_polygon_um": [
                    _rounded_point(point) for point in footprint_polygon
                ],
                "crossing_footprint_blockers": [],
                "expected_pair": True,
                "perpendicular": True,
                "classification": "legal_native_crossing",
                "source": "native_crossing_events",
                "reservation_cells": list(raw_event.get("reservation_cells", [])),
            }
        )

    crossing_plan_info["realized_intersections"] = realized
    crossing_plan_info["realized_intersection_count"] = len(realized)
    crossing_plan_info["routes_missing_corrected_centerline"] = []
    crossing_plan_info["routes_missing_corrected_centerline_count"] = 0
    crossing_plan_info["ignored_endpoint_access_intersections"] = []
    crossing_plan_info["ignored_endpoint_access_intersection_count"] = 0
    crossing_plan_info["illegal_realized_crossings"] = []
    crossing_plan_info["illegal_realized_crossing_count"] = 0
    return []


def _bbox_size_um(component: Component) -> tuple[float, float] | None:
    try:
        bbox = component.dbbox() if callable(component.dbbox) else component.dbbox
    except (AttributeError, TypeError):
        try:
            bbox = component.bbox() if callable(component.bbox) else component.bbox
        except (AttributeError, TypeError):
            return None
    try:
        width = float(bbox.right) - float(bbox.left)
        height = float(bbox.top) - float(bbox.bottom)
    except AttributeError:
        try:
            left, bottom, right, top = cast(Any, bbox)
            width = float(right) - float(left)
            height = float(top) - float(bottom)
        except (TypeError, ValueError):
            return None
    if not math.isfinite(width) or not math.isfinite(height) or width <= 0 or height <= 0:
        return None
    return width, height


def _bbox_center_um(ref: Any) -> tuple[float, float] | None:
    try:
        bbox = ref.dbbox() if callable(ref.dbbox) else ref.dbbox
    except (AttributeError, TypeError):
        try:
            bbox = ref.bbox() if callable(ref.bbox) else ref.bbox
        except (AttributeError, TypeError):
            return None
    try:
        left = float(bbox.left)
        right = float(bbox.right)
        bottom = float(bbox.bottom)
        top = float(bbox.top)
    except AttributeError:
        try:
            left, bottom, right, top = cast(Any, bbox)
            left = float(left)
            right = float(right)
            bottom = float(bottom)
            top = float(top)
        except (TypeError, ValueError):
            return None
    if not all(math.isfinite(value) for value in (left, right, bottom, top)):
        return None
    return (left + right) / 2.0, (bottom + top) / 2.0


def _ports_optical_center_um(obj: Any) -> tuple[float, float] | None:
    """Return the optical center implied by port positions, when available."""

    try:
        ports = list(obj.ports)
    except (AttributeError, TypeError, ValueError):
        return None
    points: list[tuple[float, float]] = []
    for port in ports:
        try:
            x = float(port.center[0])
            y = float(port.center[1])
        except (AttributeError, IndexError, TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    if len(points) < 2:
        return None
    return (
        sum(point[0] for point in points) / float(len(points)),
        sum(point[1] for point in points) / float(len(points)),
    )


def _active_crossing_component() -> Component | None:
    try:
        return gf.components.crossing()
    except Exception:
        try:
            from gdsfactory.gpdk import get_generic_pdk

            get_generic_pdk().activate()
            return gf.components.crossing()
        except Exception:
            return None


def _crossing_component_bbox_size_um() -> tuple[str, float, float] | None:
    component = _active_crossing_component()
    if component is None:
        return None
    size = _bbox_size_um(component)
    if size is None:
        return None
    return str(component.name), float(size[0]), float(size[1])


def _point_um_from_mapping(value: object) -> tuple[float, float] | None:
    if not isinstance(value, (tuple, list)) or len(value) < 2:
        return None
    try:
        x = float(value[0])
        y = float(value[1])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return x, y


def _segment_um_from_mapping(
    value: object,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    if not isinstance(value, (tuple, list)) or len(value) < 2:
        return None
    start = _point_um_from_mapping(value[0])
    end = _point_um_from_mapping(value[1])
    if start is None or end is None:
        return None
    if math.hypot(end[0] - start[0], end[1] - start[1]) <= 1.0e-9:
        return None
    return start, end


def _crossing_component_rotation_deg(crossing: Mapping[str, object]) -> float:
    segment = _segment_um_from_mapping(crossing.get("segment_a_um"))
    if segment is None:
        return 0.0
    (x0, y0), (x1, y1) = segment
    angle = math.degrees(math.atan2(y1 - y0, x1 - x0))
    return float(angle % 360.0)


_SHARED_CROSSING_COMPONENT_POLICIES = {
    "allowed_lidar_pure_cluster",
    "allowed_lidar_pure_degraded_cluster",
}


def _shared_crossing_peer_indices(raw_crossing: Mapping[str, object]) -> set[int]:
    indices: set[int] = set()
    raw_peer_indices = raw_crossing.get("overlapping_crossing_indices")
    if isinstance(raw_peer_indices, IterableABC) and not isinstance(
        raw_peer_indices,
        (str, bytes, bytearray),
    ):
        for raw_peer_index in raw_peer_indices:
            try:
                indices.add(int(cast(object, raw_peer_index)))
            except (TypeError, ValueError):
                continue
    try:
        indices.add(int(cast(object, raw_crossing.get("overlapping_crossing_index"))))
    except (TypeError, ValueError):
        pass
    return indices


def _shared_crossing_component_clusters(
    raw_crossings: list[Mapping[str, object]],
) -> dict[int, set[int]]:
    shared_indices = {
        index
        for index, raw_crossing in enumerate(raw_crossings)
        if str(raw_crossing.get("footprint_overlap_policy", "") or "")
        in _SHARED_CROSSING_COMPONENT_POLICIES
    }
    if not shared_indices:
        return {}

    parent = {index: index for index in shared_indices}

    def find(index: int) -> int:
        root = parent[index]
        while root != parent[root]:
            root = parent[root]
        while index != root:
            next_index = parent[index]
            parent[index] = root
            index = next_index
        return root

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        root = min(left_root, right_root)
        other = max(left_root, right_root)
        parent[other] = root

    for index in shared_indices:
        for peer_index in _shared_crossing_peer_indices(raw_crossings[index]):
            if peer_index in shared_indices:
                union(index, peer_index)

    clusters_by_root: dict[int, set[int]] = {}
    for index in shared_indices:
        clusters_by_root.setdefault(find(index), set()).add(index)
    return {
        index: set(cluster)
        for cluster in clusters_by_root.values()
        for index in cluster
    }


def _crossing_footprint_polygon_metadata(
    raw_crossing: Mapping[str, object],
) -> list[list[float]] | None:
    raw_polygon = raw_crossing.get("crossing_footprint_polygon_um")
    if not isinstance(raw_polygon, IterableABC) or isinstance(
        raw_polygon,
        (str, bytes, bytearray),
    ):
        return None
    polygon: list[list[float]] = []
    for raw_point in raw_polygon:
        if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
            return None
        try:
            polygon.append([float(raw_point[0]), float(raw_point[1])])
        except (TypeError, ValueError):
            return None
    return polygon if len(polygon) >= 3 else None


def _place_realized_crossing_components(
    routed_layout: Component,
    crossing_plan_info: dict[str, object],
) -> list[dict[str, object]]:
    """Place active PDK crossing refs for legal realized route crossings."""

    if not crossing_plan_info.get("enabled"):
        crossing_plan_info["realized_crossing_components"] = []
        crossing_plan_info["realized_crossing_component_count"] = 0
        return []

    component = _active_crossing_component()
    if component is None:
        crossing_plan_info["realized_crossing_components"] = []
        crossing_plan_info["realized_crossing_component_count"] = 0
        crossing_plan_info["realized_crossing_component_error"] = (
            "crossing_component_unavailable"
        )
        return []

    component_size = _bbox_size_um(component)
    component_bbox_um = (
        [float(component_size[0]), float(component_size[1])]
        if component_size is not None
        else None
    )
    component_name = str(component.name)
    placements: list[dict[str, object]] = []
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        raw_crossings = ()
    raw_crossing_list = [
        raw_crossing
        for raw_crossing in raw_crossings
        if isinstance(raw_crossing, Mapping)
    ]
    shared_clusters_by_index = _shared_crossing_component_clusters(
        raw_crossing_list
    )

    for index, raw_crossing in enumerate(raw_crossing_list):
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        cluster_indices = shared_clusters_by_index.get(index)
        if cluster_indices:
            representative_index = min(cluster_indices)
            if index != representative_index:
                continue
        point_um = _point_um_from_mapping(raw_crossing.get("point_um"))
        if point_um is None:
            continue
        rotation_deg = _crossing_component_rotation_deg(raw_crossing)
        instance_name = (
            f"crossing_{len(placements):04d}_"
            f"{raw_crossing.get('net_id_a', 'na')}_"
            f"{raw_crossing.get('net_id_b', 'nb')}"
        )
        try:
            ref = routed_layout.add_ref(component, name=instance_name)
        except TypeError:
            ref = routed_layout.add_ref(component)
        if abs(rotation_deg) > 1.0e-9:
            ref.drotate(rotation_deg)
        ref_center_um = _ports_optical_center_um(ref) or _bbox_center_um(ref)
        if ref_center_um is None:
            ref.dmove(point_um)
        else:
            ref.dmove(
                (
                    point_um[0] - ref_center_um[0],
                    point_um[1] - ref_center_um[1],
                )
            )

        placement: dict[str, object] = {
            "component_name": component_name,
            "instance_name": str(getattr(ref, "name", instance_name)),
            "point_um": _rounded_point(point_um),
            "center_um": _rounded_point(point_um),
            "optical_center_um": _rounded_point(point_um),
            "rotation_deg": round(float(rotation_deg), 6),
            "source_crossing_index": int(index),
            "classification": classification,
            "net_id_a": raw_crossing.get("net_id_a"),
            "net_id_b": raw_crossing.get("net_id_b"),
            "net_name_a": raw_crossing.get("net_name_a"),
            "net_name_b": raw_crossing.get("net_name_b"),
        }
        if cluster_indices:
            placement["shared_crossing_indices"] = sorted(int(i) for i in cluster_indices)
            shared_owner_names: set[str] = set()
            shared_owner_ids: set[int] = set()
            for shared_index in cluster_indices:
                if shared_index < 0 or shared_index >= len(raw_crossing_list):
                    continue
                shared_crossing = raw_crossing_list[shared_index]
                for key in ("net_name_a", "net_name_b"):
                    value = shared_crossing.get(key)
                    if isinstance(value, str) and value:
                        shared_owner_names.add(value)
                for key in ("net_id_a", "net_id_b"):
                    try:
                        shared_owner_ids.add(int(cast(object, shared_crossing.get(key))))
                    except (TypeError, ValueError):
                        continue
            if shared_owner_names:
                placement["shared_owner_net_names"] = sorted(shared_owner_names)
            if shared_owner_ids:
                placement["shared_owner_net_ids"] = sorted(shared_owner_ids)
        footprint_polygon = _crossing_footprint_polygon_metadata(raw_crossing)
        if footprint_polygon is not None:
            placement["crossing_footprint_polygon_um"] = footprint_polygon
        if component_bbox_um is not None:
            placement["component_bbox_um"] = list(component_bbox_um)
        placements.append(placement)

    crossing_plan_info["realized_crossing_components"] = placements
    crossing_plan_info["realized_crossing_component_count"] = len(placements)
    crossing_plan_info.pop("realized_crossing_component_error", None)
    try:
        routed_layout.info["realized_crossing_components"] = placements
    except (AttributeError, TypeError, ValueError):
        pass
    return placements


def _legal_crossing_overlap_polygons_for_verification(
    crossing_plan_info: Mapping[str, object] | None,
) -> dict[tuple[int, int], tuple[tuple[tuple[float, float], ...], ...]]:
    if crossing_plan_info is None:
        return {}
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return {}

    polygons_by_pair: dict[tuple[int, int], list[tuple[tuple[float, float], ...]]] = {}
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        try:
            net_id_a = int(cast(object, raw_crossing["net_id_a"]))
            net_id_b = int(cast(object, raw_crossing["net_id_b"]))
        except (KeyError, TypeError, ValueError):
            continue
        raw_polygon = raw_crossing.get("crossing_footprint_polygon_um", ())
        if not isinstance(raw_polygon, IterableABC) or isinstance(
            raw_polygon,
            (str, bytes, bytearray),
        ):
            continue
        points: list[tuple[float, float]] = []
        for raw_point in raw_polygon:
            if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
                points = []
                break
            try:
                point = (float(raw_point[0]), float(raw_point[1]))
            except (TypeError, ValueError):
                points = []
                break
            if not math.isfinite(point[0]) or not math.isfinite(point[1]):
                points = []
                break
            points.append(point)
        if len(points) >= 3:
            pair = (net_id_a, net_id_b) if net_id_a <= net_id_b else (net_id_b, net_id_a)
            polygons_by_pair.setdefault(pair, []).append(tuple(points))
    return {pair: tuple(polygons) for pair, polygons in polygons_by_pair.items()}


def _legal_crossing_component_footprints_for_verification(
    crossing_plan_info: Mapping[str, object] | None,
) -> tuple[dict[str, object], ...]:
    if crossing_plan_info is None:
        return ()
    raw_components = crossing_plan_info.get("realized_crossing_components", ())
    if isinstance(raw_components, IterableABC) and not isinstance(
        raw_components,
        (str, bytes, bytearray),
    ):
        components = [
            dict(component)
            for component in raw_components
            if isinstance(component, Mapping)
        ]
        if components:
            return tuple(components)

    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return ()
    footprints: list[dict[str, object]] = []
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        raw_polygon = raw_crossing.get("crossing_footprint_polygon_um", ())
        if isinstance(raw_polygon, IterableABC) and not isinstance(
            raw_polygon,
            (str, bytes, bytearray),
        ):
            raw_points = list(raw_polygon)
            if len(raw_points) >= 3:
                footprint = dict(raw_crossing)
                footprint["crossing_footprint_polygon_um"] = raw_points
                footprints.append(footprint)
    return tuple(footprints)


def _routed_records_by_net_id(
    records: Iterable[RoutedNetRecord],
) -> dict[int, RoutedNetRecord]:
    records_by_id: dict[int, RoutedNetRecord] = {}
    for record in records:
        if record.net_id is None:
            continue
        try:
            records_by_id[int(record.net_id)] = record
        except (TypeError, ValueError):
            continue
    return records_by_id


def _resolve_crossing_half_size_cells(
    *,
    requested_half_size_cells: int,
    enable_crossings: bool,
    grid_size_um: float,
    clearance_um: float,
) -> tuple[int, dict[str, object]]:
    info: dict[str, object] = {
        "requested_half_size_cells": int(requested_half_size_cells),
        "derived_from_component": False,
    }
    if requested_half_size_cells > 0 or not enable_crossings:
        info["half_size_cells"] = int(requested_half_size_cells)
        return int(requested_half_size_cells), info

    component_size = _crossing_component_bbox_size_um()
    if component_size is None:
        info["reason"] = "crossing_component_unavailable"
        info["half_size_cells"] = 0
        return 0, info

    component_name, width_um, height_um = component_size
    half_extent_um = max(width_um, height_um) / 2.0 + max(0.0, float(clearance_um))
    half_size_cells = int(math.ceil(half_extent_um / float(grid_size_um)))
    info.update(
        {
            "component_name": component_name,
            "component_bbox_um": [width_um, height_um],
            "clearance_um": float(clearance_um),
            "grid_size_um": float(grid_size_um),
            "half_size_cells": half_size_cells,
            "derived_from_component": True,
        }
    )
    return half_size_cells, info


def _port_center_um(port: object) -> tuple[float, float] | None:
    center = getattr(port, "center", None)
    if center is None:
        center = getattr(port, "dcenter", None)
    if center is None:
        return None
    try:
        center_seq = cast(Iterable[Any], center)
        x_um, y_um = tuple(center_seq)[:2]
        return (float(x_um), float(y_um))
    except (TypeError, ValueError, IndexError):
        return None


def _edge_key_to_info(edge_key: object) -> dict[str, object]:
    source = getattr(edge_key, "source", None)
    target = getattr(edge_key, "target", None)
    return {
        "net_name": str(getattr(edge_key, "net_name", "")),
        "source_instance": str(getattr(source, "instance", "")),
        "source_port": str(getattr(source, "port", "")),
        "target_instance": str(getattr(target, "instance", "")),
        "target_port": str(getattr(target, "port", "")),
    }


def _effective_crossing_search_loss(
    *,
    enable_crossings: bool,
    crossing_mode: str,
    crossing_loss: float,
) -> float:
    """Return the search-only crossing penalty passed to Rust A*.

    ``crossing_loss`` is the physical insertion-loss term reported to users.
    Collision-discovered crossing modes also need a non-physical search cost so
    A* tries a clean same-net route before probing route-route collisions.
    """
    physical_loss = float(crossing_loss)
    if not enable_crossings:
        return physical_loss
    if physical_loss > 0.0:
        return physical_loss
    if str(crossing_mode).strip().lower() in {"collision", "lidar-pure"}:
        return DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM
    return physical_loss


def _build_crossing_plan_info(
    *,
    rust_backend: object,
    router: object,
    schematic: Schematic,
    route_jobs: list[RouteJob],
    enable_crossings: bool,
    node_depths: dict[str, int] | None,
    node_ranks: dict[str, int] | None,
    edge_ranks: dict[str, dict[str, int]] | None,
    crossing_loss: float,
    crossing_search_loss: float,
    crossing_half_size_cells: int,
    min_straight_cells_per_crossing: int,
    allow_only_expected_crossings: bool,
) -> dict[str, object]:
    info: dict[str, object] = {
        "enabled": bool(enable_crossings),
        "constraint_count": 0,
        "event_count": 0,
        "missing_event_count": 0,
        "missing_events": [],
        "events": [],
        "expected_crossings_by_net_id": {},
        "expected_crossings_by_net_name": {},
        "crossing_loss": float(crossing_loss),
        "crossing_search_loss": float(crossing_search_loss),
        "crossing_half_size_cells": int(crossing_half_size_cells),
        "min_straight_cells_per_crossing": int(min_straight_cells_per_crossing),
        "allow_only_expected_crossings": bool(allow_only_expected_crossings),
    }
    if not enable_crossings:
        return info

    required_backend = (
        "CrossingConfig",
        "CrossingConstraint",
    )
    missing_backend = [name for name in required_backend if not hasattr(rust_backend, name)]
    required_router = (
        "set_crossing_config",
        "set_crossing_constraints",
        "crossing_expected_count",
    )
    missing_router = [name for name in required_router if not hasattr(router, name)]
    if missing_backend or missing_router:
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose crossing "
            "configuration APIs. Rebuild it with `maturin develop`. "
            f"Missing backend attrs: {missing_backend}; missing router attrs: {missing_router}."
        )

    router.set_crossing_constraints([])
    router.set_crossing_config(
        rust_backend.CrossingConfig(
            enabled=True,
            crossing_loss=float(crossing_search_loss),
            crossing_half_size_cells=int(crossing_half_size_cells),
            min_straight_cells_per_crossing=int(min_straight_cells_per_crossing),
            allow_only_expected_pairs=bool(allow_only_expected_crossings),
        )
    )

    if node_depths is None or node_ranks is None or edge_ranks is None:
        info["reason"] = "missing_topology_metadata"
        return info

    try:
        topology = analyze_schematic_topology(
            schematic,
            node_depths=node_depths,
            node_ranks=node_ranks,
            edge_ranks=edge_ranks,
        )
        crossing_plan: CrossingPlan = build_crossing_plan(topology)
    except (KeyError, ValueError) as exc:
        info["reason"] = "invalid_topology_metadata"
        info["error"] = str(exc)
        return info
    info["event_count"] = len(crossing_plan.events)
    info["stage_count"] = len(crossing_plan.stages)
    info["plan_text"] = crossing_plan.to_text(include_empty_stages=True)

    jobs_by_edge = {route_edge_key(job): job for job in route_jobs}
    constraints = []
    missing_events: list[dict[str, object]] = []
    event_records: list[dict[str, object]] = []
    crossing_counts_by_net_id: Counter[int] = Counter()
    crossing_counts_by_net_name: Counter[str] = Counter()

    for event in crossing_plan.events:
        job_a = jobs_by_edge.get(event.edge_a)
        job_b = jobs_by_edge.get(event.edge_b)
        event_record: dict[str, object] = {
            "edge_a": _edge_key_to_info(event.edge_a),
            "edge_b": _edge_key_to_info(event.edge_b),
            "source_depth": int(event.source_depth),
            "target_depth": int(event.target_depth),
            "level": int(event.level),
            "order_index": int(event.order_index),
            "edge_a_source_rank": int(event.edge_a_source_rank),
            "edge_a_target_rank": int(event.edge_a_target_rank),
            "edge_b_source_rank": int(event.edge_b_source_rank),
            "edge_b_target_rank": int(event.edge_b_target_rank),
        }
        if job_a is None or job_b is None:
            missing_record = {
                **event_record,
                "edge_a_found": job_a is not None,
                "edge_b_found": job_b is not None,
            }
            missing_events.append(missing_record)
            event_records.append({**event_record, "loaded": False})
            continue

        event_record.update(
            {
                "loaded": True,
                "net_id_a": int(job_a.net_id),
                "net_id_b": int(job_b.net_id),
                "net_name_a": str(job_a.net_name),
                "net_name_b": str(job_b.net_name),
            }
        )
        event_records.append(event_record)
        constraints.append(
            rust_backend.CrossingConstraint(
                int(job_a.net_id),
                int(job_b.net_id),
                level=int(event.level),
                source_depth=int(event.source_depth),
                target_depth=int(event.target_depth),
            )
        )
        crossing_counts_by_net_id[int(job_a.net_id)] += 1
        crossing_counts_by_net_id[int(job_b.net_id)] += 1
        crossing_counts_by_net_name[str(job_a.net_name)] += 1
        crossing_counts_by_net_name[str(job_b.net_name)] += 1

    router.set_crossing_constraints(constraints)

    info["constraint_count"] = len(constraints)
    info["missing_event_count"] = len(missing_events)
    info["missing_events"] = missing_events
    info["events"] = event_records
    info["expected_crossings_by_net_id"] = dict(sorted(crossing_counts_by_net_id.items()))
    info["expected_crossings_by_net_name"] = dict(sorted(crossing_counts_by_net_name.items()))
    return info


def _segment_bend_units(route_obj: object) -> float:
    bend_units = 0.0
    for raw_segment in getattr(route_obj, "segments", []) or []:
        try:
            kind = str(raw_segment.get("kind", ""))
        except AttributeError:
            continue
        if kind == "turn45":
            bend_units += 0.5
        elif kind == "turn90":
            bend_units += 1.0
        elif kind and kind != "straight":
            bend_units += 1.0
    return bend_units


def _write_insertion_loss_report(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    crossing_counts_by_net_id: Mapping[int, int],
    propagation_loss_per_um: float,
    bend_loss_per_90deg: float,
    crossing_count_source: str,
) -> None:
    crossing_loss = float(crossing_plan_info.get("crossing_loss", 0.0) or 0.0)
    per_net: list[dict[str, object]] = []
    total_length_um = 0.0
    total_bend_units = 0.0
    total_crossing_count = 0
    total_insertion_loss = 0.0
    for net_id, record in sorted(routed_records_by_net_id.items()):
        length_um = float(record.total_length_um)
        bend_units = _segment_bend_units(record.route_obj)
        crossing_count = int(crossing_counts_by_net_id.get(int(net_id), 0))
        insertion_loss = (
            length_um * propagation_loss_per_um
            + bend_units * bend_loss_per_90deg
            + crossing_count * crossing_loss
        )
        total_length_um += length_um
        total_bend_units += bend_units
        total_crossing_count += crossing_count
        total_insertion_loss += insertion_loss
        per_net.append(
            {
                "net_id": int(net_id),
                "net_name": record.net_name,
                "length_um": length_um,
                "bend_90deg_units": bend_units,
                "crossing_count": crossing_count,
                "propagation_loss": length_um * propagation_loss_per_um,
                "bend_loss": bend_units * bend_loss_per_90deg,
                "crossing_loss": crossing_count * crossing_loss,
                "insertion_loss": insertion_loss,
            }
        )

    crossing_plan_info["insertion_loss_model"] = {
        "propagation_loss_per_um": propagation_loss_per_um,
        "bend_loss_per_90deg": bend_loss_per_90deg,
        "crossing_loss": crossing_loss,
        "crossing_count_source": crossing_count_source,
        "device_loss_included": False,
        "formula": (
            "length_um * propagation_loss_per_um + "
            "bend_90deg_units * bend_loss_per_90deg + "
            "crossing_count * crossing_loss"
        ),
    }
    crossing_plan_info["insertion_loss_summary"] = {
        "net_count": len(per_net),
        "total_length_um": total_length_um,
        "total_bend_90deg_units": total_bend_units,
        "total_crossing_count": total_crossing_count,
        "total_insertion_loss": total_insertion_loss,
    }
    crossing_plan_info["insertion_loss_by_net"] = per_net


def _augment_insertion_loss_report(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    native_crossing_events: Iterable[object],
    propagation_loss_per_um: float = 0.0,
    bend_loss_per_90deg: float = 0.0,
) -> None:
    if not crossing_plan_info.get("enabled"):
        return
    crossing_counts_by_net_id: Counter[int] = Counter()
    for raw_event in native_crossing_events:
        try:
            net_id = int(raw_event["net_id"])
            partner_net_id = int(raw_event["partner_net_id"])
        except (TypeError, KeyError, ValueError):
            continue
        crossing_counts_by_net_id[net_id] += 1
        crossing_counts_by_net_id[partner_net_id] += 1

    _write_insertion_loss_report(
        crossing_plan_info=crossing_plan_info,
        routed_records_by_net_id=routed_records_by_net_id,
        crossing_counts_by_net_id=crossing_counts_by_net_id,
        propagation_loss_per_um=propagation_loss_per_um,
        bend_loss_per_90deg=bend_loss_per_90deg,
        crossing_count_source="native_crossing_events",
    )


def _augment_insertion_loss_report_from_realized_intersections(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    propagation_loss_per_um: float = 0.0,
    bend_loss_per_90deg: float = 0.0,
) -> None:
    if not crossing_plan_info.get("enabled"):
        return
    crossing_counts_by_net_id: Counter[int] = Counter()
    raw_intersections = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_intersections, IterableABC) or isinstance(
        raw_intersections,
        (str, bytes, bytearray),
    ):
        raw_intersections = ()
    for raw_intersection in raw_intersections:
        if not isinstance(raw_intersection, Mapping):
            continue
        classification = str(raw_intersection.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        try:
            net_id_a = int(raw_intersection["net_id_a"])
            net_id_b = int(raw_intersection["net_id_b"])
        except (TypeError, KeyError, ValueError):
            continue
        crossing_counts_by_net_id[net_id_a] += 1
        crossing_counts_by_net_id[net_id_b] += 1

    _write_insertion_loss_report(
        crossing_plan_info=crossing_plan_info,
        routed_records_by_net_id=routed_records_by_net_id,
        crossing_counts_by_net_id=crossing_counts_by_net_id,
        propagation_loss_per_um=propagation_loss_per_um,
        bend_loss_per_90deg=bend_loss_per_90deg,
        crossing_count_source="realized_intersections",
    )


def _augment_crossing_plan_with_realized_overlaps(
    *,
    router: object,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord] | None = None,
) -> None:
    if not crossing_plan_info.get("enabled"):
        return
    if not hasattr(router, "all_net_core_cells"):
        crossing_plan_info["actual_crossing_reason"] = "missing_core_cell_api"
        return

    core_cells_by_net_id: dict[int, set[tuple[int, int]]] = {}
    for raw_net_id, raw_cells in router.all_net_core_cells():
        core_cells_by_net_id[int(raw_net_id)] = {
            (int(cell[0]), int(cell[1])) for cell in raw_cells
        }

    actual_crossings: list[dict[str, object]] = []
    unrealized_expected: list[dict[str, object]] = []
    required_crossing_margin_cells = int(
        crossing_plan_info.get("crossing_half_size_cells", 0) or 0
    ) + int(crossing_plan_info.get("bend_runout_cells_per_crossing", 0) or 0)
    for raw_event in list(crossing_plan_info.get("events", [])):
        event = dict(cast(dict[str, object], raw_event))
        if not event.get("loaded"):
            continue
        net_id_a = int(cast(int, event["net_id_a"]))
        net_id_b = int(cast(int, event["net_id_b"]))
        overlap = sorted(
            core_cells_by_net_id.get(net_id_a, set())
            & core_cells_by_net_id.get(net_id_b, set())
        )
        geometric_crossing = None
        if routed_records_by_net_id is not None:
            record_a = routed_records_by_net_id.get(net_id_a)
            record_b = routed_records_by_net_id.get(net_id_b)
            geometric_crossing = _first_perpendicular_route_intersection(
                record_a.route_obj if record_a is not None else None,
                record_b.route_obj if record_b is not None else None,
            )
        record = {
            "net_id_a": net_id_a,
            "net_id_b": net_id_b,
            "net_name_a": event.get("net_name_a"),
            "net_name_b": event.get("net_name_b"),
            "source_depth": event.get("source_depth"),
            "target_depth": event.get("target_depth"),
            "level": event.get("level"),
            "order_index": event.get("order_index"),
            "cell_count": len(overlap),
            "cells": [[int(x), int(y)] for x, y in overlap[:32]],
        }
        if geometric_crossing is not None:
            record["geometric"] = True
            record["point"] = geometric_crossing["point"]
            record["segment_a"] = geometric_crossing["segment_a"]
            record["segment_b"] = geometric_crossing["segment_b"]
            record["segment_a_margin_cells"] = geometric_crossing[
                "segment_a_margin_cells"
            ]
            record["segment_b_margin_cells"] = geometric_crossing[
                "segment_b_margin_cells"
            ]
            margin_a = float(geometric_crossing["segment_a_margin_cells"])
            margin_b = float(geometric_crossing["segment_b_margin_cells"])
            record["valid_crossing_geometry"] = (
                margin_a + 1e-9 >= required_crossing_margin_cells
                and margin_b + 1e-9 >= required_crossing_margin_cells
            )
            if not record["valid_crossing_geometry"]:
                record["unrealized_reason"] = "insufficient_straight_margin"
                record["required_margin_cells"] = required_crossing_margin_cells
        if geometric_crossing is not None and record.get("valid_crossing_geometry"):
            actual_crossings.append(record)
        elif overlap and routed_records_by_net_id is None:
            actual_crossings.append(record)
        else:
            unrealized_expected.append(record)

    crossing_plan_info["actual_crossing_count"] = len(actual_crossings)
    crossing_plan_info["actual_crossing_cell_count"] = sum(
        int(record["cell_count"]) for record in actual_crossings
    )
    crossing_plan_info["actual_geometric_crossing_count"] = sum(
        1 for record in actual_crossings if record.get("geometric")
    )
    crossing_plan_info["actual_crossings"] = actual_crossings
    crossing_plan_info["unrealized_expected_crossings"] = unrealized_expected
    crossing_plan_info["unrealized_expected_crossing_count"] = len(unrealized_expected)


def _write_crossing_debug_artifacts(
    *,
    debug_path: Path | None,
    debug_prefix: str,
    crossing_plan_info: dict[str, object],
) -> None:
    if debug_path is None or not crossing_plan_info.get("enabled"):
        return
    crossing_dir = debug_path / "crossings"
    _ensure_dir(crossing_dir)
    json_path = crossing_dir / f"{debug_prefix}_crossings.json"
    txt_path = crossing_dir / f"{debug_prefix}_crossings.txt"
    json_path.write_text(
        json.dumps(crossing_plan_info, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    lines = [str(crossing_plan_info.get("plan_text", "CrossingPlan: unavailable"))]
    lines.append("")
    lines.append(
        "loaded_constraints="
        f"{int(crossing_plan_info.get('constraint_count', 0))}/"
        f"{int(crossing_plan_info.get('event_count', 0))}"
    )
    lines.append(
        "realized_crossings="
        f"{int(crossing_plan_info.get('actual_crossing_count', 0))}/"
        f"{int(crossing_plan_info.get('constraint_count', 0))}"
    )
    for crossing in cast(
        list[dict[str, object]],
        crossing_plan_info.get("actual_crossings", []),
    ):
        if crossing.get("geometric"):
            lines.append(
                "  - "
                f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')}: "
                f"segment intersection at {crossing.get('point')}"
            )
            continue
        lines.append(
            "  - "
            f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')}: "
            f"{crossing.get('cell_count')} core-overlap cell(s)"
        )
    if crossing_plan_info.get("unrealized_expected_crossing_count", 0):
        lines.append("unrealized_expected:")
        for crossing in cast(
            list[dict[str, object]],
            crossing_plan_info.get("unrealized_expected_crossings", []),
        ):
            details = [f"level={crossing.get('level')}"]
            if crossing.get("unrealized_reason"):
                details.append(f"reason={crossing.get('unrealized_reason')}")
            if crossing.get("point"):
                details.append(f"point={crossing.get('point')}")
            if crossing.get("required_margin_cells") is not None:
                details.append(f"required_margin={crossing.get('required_margin_cells')}")
            if crossing.get("segment_a_margin_cells") is not None:
                details.append(f"margin_a={crossing.get('segment_a_margin_cells')}")
            if crossing.get("segment_b_margin_cells") is not None:
                details.append(f"margin_b={crossing.get('segment_b_margin_cells')}")
            lines.append(
                "  - "
                f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')} "
                + " ".join(details)
            )
    if crossing_plan_info.get("illegal_realized_crossing_count", 0):
        lines.append("illegal_realized_crossings:")
        for crossing in cast(
            list[dict[str, object]],
            crossing_plan_info.get("illegal_realized_crossings", []),
        ):
            lines.append(
                "  - "
                f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')} "
                f"point={crossing.get('point_um')} "
                f"reason={crossing.get('reason')} "
                f"margin_a={crossing.get('segment_a_margin_um')} "
                f"margin_b={crossing.get('segment_b_margin_um')} "
                f"required={crossing.get('required_margin_um')}"
            )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze_meander_insertion_for_requirements(*args: Any, **kwargs: Any):
    _meander_impl._load_rust_backend = _load_rust_backend
    return _meander_impl.analyze_meander_insertion_for_requirements(*args, **kwargs)


def insert_meanders_for_requirements(*args: Any, **kwargs: Any):
    _meander_impl._load_rust_backend = _load_rust_backend
    return _meander_impl.insert_meanders_for_requirements(*args, **kwargs)


def _build_realization_router(
    *,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
) -> EndpointCorrectionRouter:
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError("Rust router backend unavailable for endpoint correction.")
    width, height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    grid_spec = rust_backend.GridSpec(
        int(width),
        int(height),
        float(grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid_size_um),
        bend_radius_cells=int(bend_radius_cells),
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar_cfg = rust_backend.AStarConfig(max_iterations=1)
    return rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)


def _apply_endpoint_corrections_to_debug_artifacts(
    debug_artifacts: RustRouteDebugArtifacts,
) -> RustRouteDebugArtifacts:
    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")
    router = _build_realization_router(
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
    )
    records = apply_port_endpoint_corrections(
        debug_artifacts.routed_net_records,
        router=router,
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_unchecked_bumps=True,
        log_failures=not debug_artifacts.realization_allow_45_degree_turns,
    )
    return replace(
        debug_artifacts,
        routed_net_records=records,
        routed_edge_lengths_um=routed_edge_lengths_from_records(records),
        port_alignment_diagnostics=build_port_alignment_diagnostics(
            records,
            realization_grid_spec=debug_artifacts.realization_grid_spec,
        ),
    )


def _centerline_length_um(points: tuple[tuple[float, float], ...]) -> float:
    total = 0.0
    for start, end in zip(points, points[1:]):
        total += math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
    return total


def _dedupe_centerline(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    out: list[tuple[float, float]] = []
    for point in points:
        if not out or out[-1] != point:
            out.append(point)
    return tuple(out)


def _closest_centerline_projection(
    centerline: tuple[tuple[float, float], ...],
    point: tuple[float, float],
) -> tuple[int, float, tuple[float, float], float] | None:
    best: tuple[int, float, tuple[float, float], float] | None = None
    px, py = float(point[0]), float(point[1])
    for index, (start, end) in enumerate(zip(centerline, centerline[1:])):
        sx, sy = float(start[0]), float(start[1])
        ex, ey = float(end[0]), float(end[1])
        dx = ex - sx
        dy = ey - sy
        length_sq = dx * dx + dy * dy
        if length_sq <= 1.0e-18:
            continue
        t = ((px - sx) * dx + (py - sy) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        projected = (sx + t * dx, sy + t * dy)
        dist_sq = (projected[0] - px) ** 2 + (projected[1] - py) ** 2
        if best is None or dist_sq < best[3]:
            best = (index, t, projected, dist_sq)
    return best


def _insert_centerline_cut_point(
    centerline: tuple[tuple[float, float], ...],
    point: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    if len(centerline) < 2:
        return centerline
    if any(_point_distance_um(existing, point) <= 1.0e-6 for existing in centerline):
        return centerline
    projection = _closest_centerline_projection(centerline, point)
    if projection is None:
        return centerline
    segment_index, _, _, _ = projection
    out = list(centerline[: segment_index + 1])
    out.append((float(point[0]), float(point[1])))
    out.extend(centerline[segment_index + 1 :])
    return _dedupe_centerline(tuple(out))


def _centerline_index_near_point(
    centerline: tuple[tuple[float, float], ...],
    point: tuple[float, float],
    *,
    tolerance_um: float = 1.0e-6,
) -> int | None:
    best_index: int | None = None
    best_dist = float("inf")
    for index, existing in enumerate(centerline):
        dist = _point_distance_um(existing, point)
        if dist < best_dist:
            best_dist = dist
            best_index = index
    return best_index if best_dist <= float(tolerance_um) else None


def _centerline_between_cut_points(
    centerline: tuple[tuple[float, float], ...],
    start_point: tuple[float, float],
    end_point: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    line = _insert_centerline_cut_point(centerline, start_point)
    line = _insert_centerline_cut_point(line, end_point)
    start_index = _centerline_index_near_point(line, start_point)
    end_index = _centerline_index_near_point(line, end_point)
    if start_index is None or end_index is None:
        return ()
    if end_index < start_index:
        start_index, end_index = end_index, start_index
    return _dedupe_centerline(tuple(line[start_index : end_index + 1]))


def _crossing_endpoint_splice_parts(
    *,
    baseline: tuple[tuple[float, float], ...],
    crossing_points: list[tuple[float, float]],
) -> tuple[
    tuple[tuple[float, float], ...],
    tuple[tuple[float, float], ...],
    tuple[tuple[float, float], ...],
    tuple[float, float],
    tuple[float, float],
] | None:
    if len(baseline) < 2 or not crossing_points:
        return None

    ordered_crossings = sorted(
        [(point, _closest_centerline_projection(baseline, point)) for point in crossing_points],
        key=lambda item: float("inf") if item[1] is None else item[1][0] + item[1][1],
    )
    ordered_projections = [
        projection for _, projection in ordered_crossings if projection is not None
    ]
    if not ordered_projections:
        return None

    first_projection = ordered_projections[0]
    last_projection = ordered_projections[-1]
    first_segment_index = int(first_projection[0])
    last_segment_index = int(last_projection[0])
    if last_segment_index < first_segment_index:
        first_segment_index, last_segment_index = last_segment_index, first_segment_index

    def _guarded_cut(
        projection: tuple[int, float, tuple[float, float], float],
        *,
        before: bool,
    ) -> tuple[float, float]:
        segment_index, _, point, _ = projection
        start = baseline[int(segment_index)]
        end = baseline[int(segment_index) + 1]
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        segment_length = math.hypot(dx, dy)
        if segment_length <= 1.0e-9:
            return start if before else end
        ux = dx / segment_length
        uy = dy / segment_length
        distance_from_start = math.hypot(
            float(point[0]) - float(start[0]),
            float(point[1]) - float(start[1]),
        )
        distance_to_end = math.hypot(
            float(end[0]) - float(point[0]),
            float(end[1]) - float(point[1]),
        )
        available = distance_from_start if before else distance_to_end
        if available <= 1.0e-9:
            return start if before else end
        guard_distance = min(available, 4.0)
        sign = -1.0 if before else 1.0
        return (
            float(point[0]) + sign * ux * guard_distance,
            float(point[1]) + sign * uy * guard_distance,
        )

    first_cut = _guarded_cut(first_projection, before=True)
    last_cut = _guarded_cut(last_projection, before=False)
    middle = _centerline_between_cut_points(baseline, first_cut, last_cut)
    if not middle:
        return None

    baseline_with_first_cut = _insert_centerline_cut_point(baseline, first_cut)
    first_cut_index = _centerline_index_near_point(baseline_with_first_cut, first_cut)
    baseline_with_last_cut = _insert_centerline_cut_point(baseline, last_cut)
    last_cut_index = _centerline_index_near_point(baseline_with_last_cut, last_cut)
    if first_cut_index is None or last_cut_index is None:
        return None

    prefix = _dedupe_centerline(tuple(baseline_with_first_cut[: first_cut_index + 1]))
    suffix = _dedupe_centerline(tuple(baseline_with_last_cut[last_cut_index:]))
    if len(prefix) < 2 or len(suffix) < 2:
        return None
    return prefix, middle, suffix, first_cut, last_cut


def _corrected_prefix_to_crossing(
    corrected_centerline: tuple[tuple[float, float], ...],
    crossing_point: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    if len(corrected_centerline) < 2:
        return ()
    projection = _closest_centerline_projection(corrected_centerline, crossing_point)
    if projection is None:
        return ()
    segment_index, t, _, _ = projection
    end_index = segment_index + (1 if t >= 1.0 - 1.0e-9 else 0)
    prefix = list(corrected_centerline[: end_index + 1])
    prefix.append((float(crossing_point[0]), float(crossing_point[1])))
    return _dedupe_centerline(tuple(prefix))


def _corrected_suffix_from_crossing(
    corrected_centerline: tuple[tuple[float, float], ...],
    crossing_point: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    if len(corrected_centerline) < 2:
        return ()
    projection = _closest_centerline_projection(corrected_centerline, crossing_point)
    if projection is None:
        return ()
    segment_index, t, _, _ = projection
    start_index = segment_index + (2 if t >= 1.0 - 1.0e-9 else 1)
    suffix = [(float(crossing_point[0]), float(crossing_point[1]))]
    suffix.extend(corrected_centerline[start_index:])
    return _dedupe_centerline(tuple(suffix))


def _segment_direction_sequence(
    centerline: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    compressed = _compress_centerline(centerline)
    directions: list[tuple[float, float]] = []
    for start, end in zip(compressed, compressed[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            continue
        directions.append((dx / length, dy / length))
    return tuple(directions)


def _same_segment_direction_sequence(
    candidate: tuple[tuple[float, float], ...],
    baseline: tuple[tuple[float, float], ...],
) -> bool:
    baseline_dirs = _segment_direction_sequence(baseline)
    candidate_dirs = _segment_direction_sequence(candidate)
    if len(candidate_dirs) != len(baseline_dirs):
        return False
    for candidate_dir, baseline_dir in zip(candidate_dirs, baseline_dirs):
        cross = candidate_dir[0] * baseline_dir[1] - candidate_dir[1] * baseline_dir[0]
        dot = candidate_dir[0] * baseline_dir[0] + candidate_dir[1] * baseline_dir[1]
        if abs(cross) > 1.0e-6 or dot <= 0.0:
            return False
    return True


def _centerline_lengths_and_dirs(
    centerline: tuple[tuple[float, float], ...],
) -> tuple[list[float], list[tuple[float, float]]]:
    lengths: list[float] = []
    dirs: list[tuple[float, float]] = []
    for start, end in zip(centerline, centerline[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            continue
        lengths.append(length)
        dirs.append((dx / length, dy / length))
    return lengths, dirs


def _is_axis_or_diagonal_direction(direction: tuple[float, float]) -> bool:
    scale = math.sqrt(0.5)
    allowed = (
        (1.0, 0.0),
        (scale, scale),
        (0.0, 1.0),
        (-scale, scale),
        (-1.0, 0.0),
        (-scale, -scale),
        (0.0, -1.0),
        (scale, -scale),
    )
    return any(_same_direction(direction, allowed_dir) for allowed_dir in allowed)


def _solve_terminal_length_adjustments(
    lengths: list[float],
    dirs: list[tuple[float, float]],
    delta: tuple[float, float],
    *,
    adjustable_indices: tuple[int, ...] | None = None,
    required_positive_indices: tuple[int, ...] = (),
) -> list[float] | None:
    if not lengths or len(lengths) != len(dirs):
        return None
    adjustable = (
        tuple(range(len(dirs)))
        if adjustable_indices is None
        else tuple(index for index in adjustable_indices if 0 <= index < len(dirs))
    )
    if not adjustable:
        return None
    dx, dy = float(delta[0]), float(delta[1])
    if math.hypot(dx, dy) <= 1.0e-9:
        updated = list(lengths)
        if all(updated[index] > 1.0e-6 for index in required_positive_indices):
            return updated
        return None

    def has_required_positive(updated: list[float]) -> bool:
        return all(updated[index] > 1.0e-6 for index in required_positive_indices)

    candidates: list[tuple[float, list[float]]] = []
    for index in adjustable:
        direction = dirs[index]
        cross = dx * direction[1] - dy * direction[0]
        if abs(cross) > 1.0e-6:
            continue
        alpha = dx * direction[0] + dy * direction[1]
        updated = list(lengths)
        updated[index] += alpha
        if updated[index] > 1.0e-6 and has_required_positive(updated):
            candidates.append((abs(alpha), updated))

    for left_pos, left in enumerate(adjustable):
        for right in adjustable[left_pos + 1 :]:
            d0 = dirs[left]
            d1 = dirs[right]
            det = d0[0] * d1[1] - d0[1] * d1[0]
            if abs(det) <= 1.0e-9:
                continue
            alpha = (dx * d1[1] - dy * d1[0]) / det
            beta = (d0[0] * dy - d0[1] * dx) / det
            updated = list(lengths)
            updated[left] += alpha
            updated[right] += beta
            if (
                updated[left] > 1.0e-6
                and updated[right] > 1.0e-6
                and has_required_positive(updated)
            ):
                candidates.append((abs(alpha) + abs(beta), updated))

    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _centerline_from_start_dirs_lengths(
    start: tuple[float, float],
    dirs: list[tuple[float, float]],
    lengths: list[float],
) -> tuple[tuple[float, float], ...]:
    points = [(float(start[0]), float(start[1]))]
    x, y = points[0]
    for direction, length in zip(dirs, lengths):
        x += float(direction[0]) * float(length)
        y += float(direction[1]) * float(length)
        points.append((x, y))
    return _dedupe_centerline(tuple(points))


def _absorbed_terminal_centerline(
    baseline_side: tuple[tuple[float, float], ...],
    *,
    desired_start: tuple[float, float] | None = None,
    desired_end: tuple[float, float] | None = None,
    extra_start_dir: tuple[float, float] | None = None,
    extra_end_dir: tuple[float, float] | None = None,
) -> tuple[tuple[float, float], ...]:
    if len(baseline_side) < 2:
        return baseline_side
    if (desired_start is None) == (desired_end is None):
        return ()

    base_lengths, base_dirs = _centerline_lengths_and_dirs(baseline_side)
    if not base_lengths:
        return ()
    base_adjustable_indices = tuple(
        index
        for index, direction in enumerate(base_dirs)
        if _is_axis_or_diagonal_direction(direction)
    )

    def solve_with(
        lengths: list[float],
        dirs: list[tuple[float, float]],
        start: tuple[float, float],
        end: tuple[float, float],
        *,
        adjustable_indices: tuple[int, ...],
        required_positive_indices: tuple[int, ...] = (),
    ) -> tuple[tuple[float, float], ...]:
        current_vector = (
            sum(length * direction[0] for length, direction in zip(lengths, dirs)),
            sum(length * direction[1] for length, direction in zip(lengths, dirs)),
        )
        desired_vector = (float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
        adjusted_lengths = _solve_terminal_length_adjustments(
            lengths,
            dirs,
            (
                desired_vector[0] - current_vector[0],
                desired_vector[1] - current_vector[1],
            ),
            adjustable_indices=adjustable_indices,
            required_positive_indices=required_positive_indices,
        )
        if adjusted_lengths is None:
            return ()
        candidate = _centerline_from_start_dirs_lengths(start, dirs, adjusted_lengths)
        if _point_distance_um(candidate[-1], end) > 1.0e-6:
            return ()
        return candidate

    if desired_start is not None:
        fixed_end = baseline_side[-1]
        candidate = solve_with(
            base_lengths,
            base_dirs,
            desired_start,
            fixed_end,
            adjustable_indices=base_adjustable_indices,
        )
        if candidate and _terminal_segment_matches_direction(
            candidate,
            extra_start_dir,
            at_start=True,
        ):
            return candidate
        if extra_start_dir is not None:
            return solve_with(
                [0.0] + base_lengths,
                [extra_start_dir] + base_dirs,
                desired_start,
                fixed_end,
                adjustable_indices=(
                    0,
                    *(index + 1 for index in base_adjustable_indices),
                ),
                required_positive_indices=(0,),
            )
        return ()

    fixed_start = baseline_side[0]
    assert desired_end is not None
    candidate = solve_with(
        base_lengths,
        base_dirs,
        fixed_start,
        desired_end,
        adjustable_indices=base_adjustable_indices,
    )
    if candidate and _terminal_segment_matches_direction(
        candidate,
        extra_end_dir,
        at_start=False,
    ):
        return candidate
    if extra_end_dir is not None:
        return solve_with(
            base_lengths + [0.0],
            base_dirs + [extra_end_dir],
            fixed_start,
            desired_end,
            adjustable_indices=(
                *base_adjustable_indices,
                len(base_lengths),
            ),
            required_positive_indices=(len(base_lengths),),
        )
    return ()


def _unit_from_orientation_deg(
    orientation_deg: float | None,
    *,
    as_target: bool,
) -> tuple[float, float] | None:
    if orientation_deg is None:
        return None
    angle_rad = math.radians(float(orientation_deg) + (180.0 if as_target else 0.0))
    return (math.cos(angle_rad), math.sin(angle_rad))


def _angle_index_to_unit(angle: int) -> tuple[float, float]:
    scale = math.sqrt(0.5)
    steps = (
        (1.0, 0.0),
        (scale, scale),
        (0.0, 1.0),
        (-scale, scale),
        (-1.0, 0.0),
        (-scale, -scale),
        (0.0, -1.0),
        (scale, -scale),
    )
    return steps[int(angle) % 8]


def _route_endpoint_unit(
    route_obj: object,
    *,
    target: bool,
) -> tuple[float, float] | None:
    states = getattr(route_obj, "states", None)
    if not states:
        return None
    try:
        state = states[-1] if target else states[0]
        return _angle_index_to_unit(int(getattr(state, "angle")))
    except (AttributeError, TypeError, ValueError, IndexError):
        return None


def _same_direction(
    candidate_dir: tuple[float, float],
    expected_dir: tuple[float, float],
) -> bool:
    cross = candidate_dir[0] * expected_dir[1] - candidate_dir[1] * expected_dir[0]
    dot = candidate_dir[0] * expected_dir[0] + candidate_dir[1] * expected_dir[1]
    return abs(cross) <= 1.0e-6 and dot > 0.0


def _terminal_segment_matches_direction(
    centerline: tuple[tuple[float, float], ...],
    expected_dir: tuple[float, float] | None,
    *,
    at_start: bool,
) -> bool:
    if expected_dir is None:
        return True
    points = _dedupe_centerline(centerline)
    if len(points) < 2:
        return False
    start, end = (points[0], points[1]) if at_start else (points[-2], points[-1])
    dx = float(end[0]) - float(start[0])
    dy = float(end[1]) - float(start[1])
    length = math.hypot(dx, dy)
    if length <= 1.0e-9:
        return False
    return _same_direction((dx / length, dy / length), expected_dir)


def _compatible_terminal_direction_sequence(
    candidate: tuple[tuple[float, float], ...],
    baseline: tuple[tuple[float, float], ...],
    *,
    expected_port_dir: tuple[float, float] | None,
    allow_extra_at_start: bool,
) -> bool:
    baseline_dirs = _segment_direction_sequence(baseline)
    candidate_dirs = _segment_direction_sequence(candidate)
    if len(candidate_dirs) == len(baseline_dirs):
        return _same_segment_direction_sequence(candidate, baseline)
    if len(candidate_dirs) != len(baseline_dirs) + 1:
        return False
    if expected_port_dir is None:
        return False
    if allow_extra_at_start:
        extra_dir = candidate_dirs[0]
        remainder_dirs = candidate_dirs[1:]
    else:
        extra_dir = candidate_dirs[-1]
        remainder_dirs = candidate_dirs[:-1]
    if not _same_direction(extra_dir, expected_port_dir):
        return False
    for candidate_dir, baseline_dir in zip(remainder_dirs, baseline_dirs):
        if not _same_direction(candidate_dir, baseline_dir):
            return False
    return True


def _terminal_anchor_matches(
    centerline: tuple[tuple[float, float], ...],
    anchor: tuple[float, float] | None,
    *,
    at_start: bool,
) -> bool:
    if anchor is None:
        return True
    if not centerline:
        return False
    point = centerline[0] if at_start else centerline[-1]
    return _point_distance_um(point, anchor) <= 1.0e-6


def _spliced_crossing_endpoint_centerline(
    *,
    baseline: tuple[tuple[float, float], ...],
    corrected_centerline: tuple[tuple[float, float], ...],
    crossing_points: list[tuple[float, float]],
    source_port_um: tuple[float, float] | None = None,
    target_port_um: tuple[float, float] | None = None,
    route_obj: object | None = None,
    source_port_orientation_deg: float | None = None,
    target_port_orientation_deg: float | None = None,
) -> tuple[tuple[float, float], ...]:
    if len(baseline) < 2 or not crossing_points:
        return ()

    ordered_crossings = sorted(
        [(point, _closest_centerline_projection(baseline, point)) for point in crossing_points],
        key=lambda item: float("inf") if item[1] is None else item[1][0] + item[1][1],
    )
    ordered_projections = [
        projection for _, projection in ordered_crossings if projection is not None
    ]
    if not ordered_projections:
        return ()

    first_projection = ordered_projections[0]
    last_projection = ordered_projections[-1]
    first_segment_index = int(first_projection[0])
    last_segment_index = int(last_projection[0])
    if last_segment_index < first_segment_index:
        first_segment_index, last_segment_index = last_segment_index, first_segment_index

    def _guarded_cut(
        projection: tuple[int, float, tuple[float, float], float],
        *,
        before: bool,
    ) -> tuple[float, float]:
        segment_index, _, point, _ = projection
        start = baseline[int(segment_index)]
        end = baseline[int(segment_index) + 1]
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        segment_length = math.hypot(dx, dy)
        if segment_length <= 1.0e-9:
            return start if before else end
        ux = dx / segment_length
        uy = dy / segment_length
        distance_from_start = math.hypot(
            float(point[0]) - float(start[0]),
            float(point[1]) - float(start[1]),
        )
        distance_to_end = math.hypot(
            float(end[0]) - float(point[0]),
            float(end[1]) - float(point[1]),
        )
        available = distance_from_start if before else distance_to_end
        if available <= 1.0e-9:
            return start if before else end
        guard_distance = min(available, 4.0)
        sign = -1.0 if before else 1.0
        return (
            float(point[0]) + sign * ux * guard_distance,
            float(point[1]) + sign * uy * guard_distance,
        )

    first_cut = _guarded_cut(first_projection, before=True)
    last_cut = _guarded_cut(last_projection, before=False)
    middle = _centerline_between_cut_points(baseline, first_cut, last_cut)
    if not middle:
        return ()

    if not corrected_centerline:
        return middle

    baseline_with_first_cut = _insert_centerline_cut_point(baseline, first_cut)
    first_cut_index = _centerline_index_near_point(baseline_with_first_cut, first_cut)
    baseline_with_last_cut = _insert_centerline_cut_point(baseline, last_cut)
    last_cut_index = _centerline_index_near_point(baseline_with_last_cut, last_cut)
    baseline_prefix = (
        _dedupe_centerline(tuple(baseline_with_first_cut[: first_cut_index + 1]))
        if first_cut_index is not None
        else ()
    )
    baseline_suffix = (
        _dedupe_centerline(tuple(baseline_with_last_cut[last_cut_index:]))
        if last_cut_index is not None
        else ()
    )
    source_dir = _route_endpoint_unit(
        route_obj,
        target=False,
    ) or _unit_from_orientation_deg(
        source_port_orientation_deg,
        as_target=False,
    )
    target_dir = _route_endpoint_unit(
        route_obj,
        target=True,
    ) or _unit_from_orientation_deg(
        target_port_orientation_deg,
        as_target=True,
    )
    prefix = _corrected_prefix_to_crossing(corrected_centerline, first_cut)
    suffix = _corrected_suffix_from_crossing(corrected_centerline, last_cut)
    if (
        not prefix
        or not _terminal_anchor_matches(
            prefix,
            source_port_um,
            at_start=True,
        )
        or not _compatible_terminal_direction_sequence(
            prefix,
            baseline_prefix,
            expected_port_dir=source_dir,
            allow_extra_at_start=True,
        )
    ):
        prefix = _absorbed_terminal_centerline(
            baseline_prefix,
            desired_start=source_port_um,
            extra_start_dir=source_dir,
        )
        if not prefix:
            prefix = baseline_prefix
    if (
        not suffix
        or not _terminal_anchor_matches(
            suffix,
            target_port_um,
            at_start=False,
        )
        or not _compatible_terminal_direction_sequence(
            suffix,
            baseline_suffix,
            expected_port_dir=target_dir,
            allow_extra_at_start=False,
        )
    ):
        suffix = _absorbed_terminal_centerline(
            baseline_suffix,
            desired_end=target_port_um,
            extra_end_dir=target_dir,
        )
        if not suffix:
            suffix = baseline_suffix

    pieces: list[tuple[float, float]] = []
    for segment in (prefix, middle, suffix):
        for point in segment:
            if not pieces or pieces[-1] != point:
                pieces.append(point)
    return _dedupe_centerline(tuple(pieces))


def _legal_crossing_points_by_net_id(
    crossing_plan_info: Mapping[str, object] | None,
) -> dict[int, list[tuple[float, float]]]:
    if not isinstance(crossing_plan_info, Mapping) or not crossing_plan_info.get(
        "enabled",
    ):
        return {}
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return {}

    points_by_net_id: dict[int, list[tuple[float, float]]] = {}
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        classification = str(raw_crossing.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        point = _point_um_from_mapping(raw_crossing.get("point_um"))
        if point is None:
            continue
        for key in ("net_id_a", "net_id_b"):
            try:
                net_id = int(raw_crossing.get(key))
            except (TypeError, ValueError):
                continue
            points_by_net_id.setdefault(net_id, []).append(point)
    return points_by_net_id


def _foreign_crossing_footprint_specs(
    crossing_plan_info: Mapping[str, object] | None,
    *,
    net_id: int | None,
    route_width_um: float,
) -> tuple[tuple[tuple[float, float], tuple[float, float], tuple[float, float], float], ...]:
    if not isinstance(crossing_plan_info, Mapping) or not crossing_plan_info.get(
        "enabled",
    ):
        return ()
    raw_crossings = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_crossings, IterableABC) or isinstance(
        raw_crossings,
        (str, bytes, bytearray),
    ):
        return ()
    specs: list[
        tuple[tuple[float, float], tuple[float, float], tuple[float, float], float]
    ] = []
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        try:
            net_id_a = int(raw_crossing.get("net_id_a"))
            net_id_b = int(raw_crossing.get("net_id_b"))
        except (TypeError, ValueError):
            net_id_a = net_id_b = -1
        if net_id is not None and int(net_id) in (net_id_a, net_id_b):
            continue
        polygon_raw = _crossing_footprint_polygon_metadata(raw_crossing)
        if polygon_raw is None or len(polygon_raw) < 4:
            continue
        polygon = [(float(point[0]), float(point[1])) for point in polygon_raw[:4]]
        center = (
            sum(point[0] for point in polygon) / float(len(polygon)),
            sum(point[1] for point in polygon) / float(len(polygon)),
        )
        axis_u = _segment_unit_vector((polygon[0], polygon[1]))
        axis_v = _segment_unit_vector((polygon[0], polygon[3]))
        if axis_u is None or axis_v is None:
            continue
        half_extent_um = 0.5 * _point_distance_um(polygon[0], polygon[1])
        if half_extent_um <= 0.0:
            continue
        specs.append(
            (
                center,
                axis_u,
                axis_v,
                float(half_extent_um) + 0.5 * float(route_width_um),
            )
        )
    return tuple(specs)


def _centerline_intersects_crossing_footprint_specs(
    centerline: tuple[tuple[float, float], ...],
    specs: tuple[
        tuple[tuple[float, float], tuple[float, float], tuple[float, float], float],
        ...,
    ],
) -> bool:
    if len(centerline) < 2 or not specs:
        return False
    for start, end in zip(centerline, centerline[1:]):
        segment = ((float(start[0]), float(start[1])), (float(end[0]), float(end[1])))
        if _segment_length_um(segment) <= 1.0e-9:
            continue
        for center, axis_u, axis_v, half_extent_um in specs:
            if _segment_intersects_crossing_footprint_interior(
                segment,
                center=center,
                axis_u=axis_u,
                axis_v=axis_v,
                half_extent_um=half_extent_um,
            ):
                return True
    return False


def _primitive_centerline_for_record(
    record: RoutedNetRecord,
    *,
    router: EndpointCorrectionRouter,
    prefer_corrected_baseline: bool = False,
) -> tuple[tuple[float, float], ...]:
    if prefer_corrected_baseline and record.corrected_centerline_um:
        return _dedupe_centerline(record.corrected_centerline_um)
    route_primitive_centerline = getattr(router, "route_primitive_centerline", None)
    if route_primitive_centerline is not None:
        try:
            centerline = _centerline_tuple(route_primitive_centerline(record.route_obj))
        except Exception:
            centerline = ()
        if centerline:
            return _dedupe_centerline(centerline)
    if record.corrected_centerline_um:
        return _dedupe_centerline(record.corrected_centerline_um)
    return ()


def _merge_terminal_corrected_route_centerline(
    *,
    existing_baseline: tuple[tuple[float, float], ...],
    route_baseline: tuple[tuple[float, float], ...],
    corrected_route_centerline: tuple[tuple[float, float], ...],
    freeze_source: bool,
    freeze_target: bool,
) -> tuple[tuple[float, float], ...]:
    """Merge a route-only terminal correction back into a stubbed centerline."""
    if len(corrected_route_centerline) < 2:
        return ()
    if len(existing_baseline) < 2 or len(route_baseline) < 2:
        return corrected_route_centerline
    if not freeze_source and not freeze_target:
        return corrected_route_centerline

    merged: list[tuple[float, float]] = []
    stitch_tolerance_um = 2.5

    def _centerline_with_stitch_point(
        centerline: tuple[tuple[float, float], ...],
        point: tuple[float, float],
    ) -> tuple[tuple[float, float], ...]:
        if _centerline_index_near_point(
            centerline,
            point,
            tolerance_um=stitch_tolerance_um,
        ) is not None:
            return centerline
        projection = _closest_centerline_projection(centerline, point)
        if projection is None:
            return centerline
        _, _, _, dist_sq = projection
        if math.sqrt(float(dist_sq)) > stitch_tolerance_um:
            return centerline
        return _insert_centerline_cut_point(centerline, point)

    if freeze_source:
        source_anchor = route_baseline[0]
        existing_baseline = _centerline_with_stitch_point(
            existing_baseline,
            source_anchor,
        )
        source_index = _centerline_index_near_point(
            existing_baseline,
            source_anchor,
            tolerance_um=stitch_tolerance_um,
        )
        if source_index is None:
            return ()
        merged.extend(existing_baseline[: source_index + 1])
        corrected_start_index = (
            1
            if _point_distance_um(corrected_route_centerline[0], source_anchor)
            <= stitch_tolerance_um
            else 0
        )
        merged.extend(corrected_route_centerline[corrected_start_index:])
    else:
        merged.extend(corrected_route_centerline)

    if freeze_target:
        target_anchor = route_baseline[-1]
        existing_baseline = _centerline_with_stitch_point(
            existing_baseline,
            target_anchor,
        )
        target_index = _centerline_index_near_point(
            existing_baseline,
            target_anchor,
            tolerance_um=stitch_tolerance_um,
        )
        if target_index is None:
            return ()
        if merged and _point_distance_um(merged[-1], target_anchor) <= 1.0e-6:
            suffix_start = target_index + 1
        else:
            suffix_start = target_index
        merged.extend(existing_baseline[suffix_start:])

    return _dedupe_centerline(tuple(merged))


def _apply_crossing_aware_endpoint_correction_to_record(
    record: RoutedNetRecord,
    *,
    router: EndpointCorrectionRouter,
    crossing_points: list[tuple[float, float]],
    realization_grid_spec: tuple[int, int, float, float, float] | None,
    route_width_um: float,
    allow_unchecked_bumps: bool,
    log_failures: bool,
    crossing_plan_info: Mapping[str, object] | None = None,
    correct_source: bool = True,
    correct_target: bool = True,
    prefer_corrected_baseline: bool = False,
    opened_cells: Iterable[tuple[int, int]] | None = None,
    clearance_exempt_cells: Iterable[tuple[int, int]] | None = None,
    clearance_radius_cells: int = 0,
    core_radius_cells: int = 0,
) -> RoutedNetRecord:
    """Apply terminal-only endpoint correction without moving route crossings."""

    trace_endpoint_nets = {
        item.strip()
        for item in os.environ.get(
            "PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS",
            "",
        ).split(",")
        if item.strip()
    }
    trace_endpoint = (
        record.net_name in trace_endpoint_nets
        or (
            record.net_id is not None
            and str(int(record.net_id)) in trace_endpoint_nets
        )
    )

    if not crossing_points:
        if record.corrected_centerline_um and not correct_source and not correct_target:
            return record
        if record.corrected_centerline_um and (
            not correct_source or not correct_target
        ):
            route_baseline = _primitive_centerline_for_record(
                record,
                router=router,
                prefer_corrected_baseline=False,
            )
            baseline = _primitive_centerline_for_record(
                record,
                router=router,
                prefer_corrected_baseline=True,
            )
            route_port_corrected_centerline = getattr(
                router,
                "route_port_corrected_centerline",
                None,
            )
            if (
                len(route_baseline) >= 2
                and len(baseline) >= 2
                and route_port_corrected_centerline is not None
            ):
                try:
                    corrected_route = _centerline_tuple(
                        route_port_corrected_centerline(
                            record.route_obj,
                            source_port_um=(
                                record.source_port_center_um if correct_source else None
                            ),
                            target_port_um=(
                                record.target_port_center_um if correct_target else None
                            ),
                            allow_unchecked_bumps=allow_unchecked_bumps,
                        )
                    )
                except Exception:
                    corrected_route = ()
                centerline = _merge_terminal_corrected_route_centerline(
                    existing_baseline=baseline,
                    route_baseline=route_baseline,
                    corrected_route_centerline=_dedupe_centerline(corrected_route),
                    freeze_source=not correct_source,
                    freeze_target=not correct_target,
                )
                if len(centerline) >= 2:
                    centerline_length = getattr(router, "centerline_length_um", None)
                    if centerline_length is not None:
                        try:
                            corrected_total_length_um = float(
                                centerline_length(list(centerline))
                            )
                        except Exception:
                            corrected_total_length_um = _centerline_length_um(centerline)
                    else:
                        corrected_total_length_um = _centerline_length_um(centerline)
                    return replace(
                        record,
                        total_length_um=corrected_total_length_um,
                        base_total_length_um=(
                            record.base_total_length_um
                            if record.base_total_length_um is not None
                            else float(record.total_length_um)
                        ),
                        corrected_centerline_um=centerline,
                        endpoint_correction_error=None,
                    )
            return record
        uncorrected_record = replace(
            record,
            corrected_centerline_um=(),
            endpoint_correction_error=None,
        )
        normal_record = apply_port_endpoint_corrections(
            [uncorrected_record],
            router=router,
            realization_grid_spec=realization_grid_spec,
            allow_unchecked_bumps=allow_unchecked_bumps,
            log_failures=log_failures,
        )[0]
        foreign_footprints = _foreign_crossing_footprint_specs(
            crossing_plan_info,
            net_id=int(record.net_id) if record.net_id is not None else None,
            route_width_um=route_width_um,
        )
        if (
            normal_record.corrected_centerline_um
            and normal_record.endpoint_correction_error is None
            and _centerline_intersects_crossing_footprint_specs(
                _dedupe_centerline(normal_record.corrected_centerline_um),
                foreign_footprints,
            )
        ):
            return record
        return normal_record

    baseline = _primitive_centerline_for_record(
        record,
        router=router,
        prefer_corrected_baseline=prefer_corrected_baseline,
    )
    if len(baseline) < 2:
        message = format_port_endpoint_correction_error(
            record,
            "crossing-aware endpoint correction requires a primitive centerline",
            realization_grid_spec=realization_grid_spec,
        )
        if log_failures:
            print("ERROR: " + message)
        return replace(record, endpoint_correction_error=message)

    def _realization_accepts(centerline: tuple[tuple[float, float], ...]) -> bool:
        realize = getattr(router, "realize_centerline_polygon_with_terminal_tangents", None)
        if realize is None:
            return True
        try:
            realize(
                list(centerline),
                float(route_width_um),
                record.route_obj,
                source_enabled=record.source_port_center_um is not None,
                target_enabled=record.target_port_center_um is not None,
            )
        except (TypeError, ValueError):
            return False
        return True

    def _merge_splice_segments(
        prefix: tuple[tuple[float, float], ...],
        middle: tuple[tuple[float, float], ...],
        suffix: tuple[tuple[float, float], ...],
    ) -> tuple[tuple[float, float], ...]:
        pieces: list[tuple[float, float]] = []
        for segment in (prefix, middle, suffix):
            for point in segment:
                if not pieces or pieces[-1] != point:
                    pieces.append(point)
        return _dedupe_centerline(tuple(pieces))

    def _checked_terminal_segment(
        segment: tuple[tuple[float, float], ...],
        *,
        source_port_um: tuple[float, float] | None,
        target_port_um: tuple[float, float] | None,
    ) -> tuple[tuple[float, float], ...]:
        centerline_port_corrected_checked = getattr(
            router,
            "centerline_port_corrected_checked",
            None,
        )
        if centerline_port_corrected_checked is None or len(segment) < 2:
            return ()
        if source_port_um is None or target_port_um is None:
            return ()
        try:
            raw = centerline_port_corrected_checked(
                int(record.net_id) if record.net_id is not None else 0,
                list(segment),
                float(route_width_um),
                int(clearance_radius_cells),
                int(core_radius_cells),
                sorted(set(opened_cells or ())),
                sorted(set(clearance_exempt_cells or ())),
                source_port_um=source_port_um,
                target_port_um=target_port_um,
            )
        except Exception as exc:
            if trace_endpoint:
                print(
                    "endpoint_trace "
                    f"net={record.net_name} id={record.net_id} "
                    "mode=checked_terminal_segment "
                    f"segment_len={len(segment)} "
                    f"segment_start={segment[0] if segment else None} "
                    f"segment_end={segment[-1] if segment else None} "
                    f"source_port={source_port_um} target_port={target_port_um} "
                    f"status=reject error={exc}"
                )
            return ()
        try:
            correction = dict(raw)
        except (TypeError, ValueError):
            if trace_endpoint:
                print(
                    "endpoint_trace "
                    f"net={record.net_name} id={record.net_id} "
                    "mode=checked_terminal_segment "
                    f"segment_len={len(segment)} "
                    f"segment_start={segment[0] if segment else None} "
                    f"segment_end={segment[-1] if segment else None} "
                    f"source_port={source_port_um} target_port={target_port_um} "
                    "status=reject error=invalid_result"
                )
            return ()
        centerline = _dedupe_centerline(_centerline_tuple(correction.get("centerline")))
        if trace_endpoint:
            print(
                "endpoint_trace "
                f"net={record.net_name} id={record.net_id} "
                "mode=checked_terminal_segment "
                f"segment_len={len(segment)} "
                f"segment_start={segment[0] if segment else None} "
                f"segment_end={segment[-1] if segment else None} "
                f"source_port={source_port_um} target_port={target_port_um} "
                f"status=accept candidate_label={correction.get('candidate_label')} "
                f"corrected_len={len(centerline)}"
            )
        return centerline

    splice_parts = _crossing_endpoint_splice_parts(
        baseline=baseline,
        crossing_points=crossing_points,
    )
    if splice_parts is not None:
        baseline_prefix, middle, baseline_suffix, _, _ = splice_parts
        checked_prefix = (
            _checked_terminal_segment(
                baseline_prefix,
                source_port_um=record.source_port_center_um,
                target_port_um=baseline_prefix[-1],
            )
            if correct_source
            else ()
        )
        checked_suffix = (
            _checked_terminal_segment(
                baseline_suffix,
                source_port_um=baseline_suffix[0],
                target_port_um=record.target_port_center_um,
            )
            if correct_target
            else ()
        )
        if checked_prefix or checked_suffix:
            candidate = _merge_splice_segments(
                checked_prefix if checked_prefix else baseline_prefix,
                middle,
                checked_suffix if checked_suffix else baseline_suffix,
            )
            source_ok = (
                not correct_source
                or _terminal_anchor_matches(
                    candidate,
                    record.source_port_center_um,
                    at_start=True,
                )
            )
            target_ok = (
                not correct_target
                or _terminal_anchor_matches(
                    candidate,
                    record.target_port_center_um,
                    at_start=False,
                )
            )
            accepts = (
                len(candidate) >= 2
                and source_ok
                and target_ok
                and _realization_accepts(candidate)
            )
            if trace_endpoint:
                print(
                    "endpoint_trace "
                    f"net={record.net_name} id={record.net_id} "
                    f"crossings={len(crossing_points)} "
                    "mode=checked_terminal_segments "
                    f"prefix_checked={bool(checked_prefix)} "
                    f"suffix_checked={bool(checked_suffix)} "
                    f"len={len(candidate)} "
                    f"start={candidate[0] if candidate else None} "
                    f"end={candidate[-1] if candidate else None} "
                    f"source={record.source_port_center_um} "
                    f"target={record.target_port_center_um} "
                    f"accepts={accepts}"
                )
            if accepts:
                centerline_length = getattr(router, "centerline_length_um", None)
                if centerline_length is not None:
                    try:
                        corrected_total_length_um = float(centerline_length(list(candidate)))
                    except Exception:
                        corrected_total_length_um = _centerline_length_um(candidate)
                else:
                    corrected_total_length_um = _centerline_length_um(candidate)
                return replace(
                    record,
                    total_length_um=corrected_total_length_um,
                    base_total_length_um=(
                        record.base_total_length_um
                        if record.base_total_length_um is not None
                        else float(record.total_length_um)
                    ),
                    corrected_centerline_um=candidate,
                    endpoint_correction_error=None,
                )

    def _candidate(*, use_source: bool, use_target: bool) -> tuple[tuple[float, float], ...]:
        if not use_source and not use_target:
            corrected = baseline
        else:
            route_port_corrected_centerline = getattr(
                router,
                "route_port_corrected_centerline",
                None,
            )
            if route_port_corrected_centerline is None:
                return ()
            try:
                corrected = _centerline_tuple(
                    route_port_corrected_centerline(
                        record.route_obj,
                        source_port_um=(
                            record.source_port_center_um if use_source else None
                        ),
                        target_port_um=(
                            record.target_port_center_um if use_target else None
                        ),
                        allow_unchecked_bumps=allow_unchecked_bumps,
                    )
                )
            except Exception:
                return ()
        return _spliced_crossing_endpoint_centerline(
            baseline=baseline,
            corrected_centerline=_dedupe_centerline(corrected),
            crossing_points=crossing_points,
            source_port_um=record.source_port_center_um if correct_source else None,
            target_port_um=record.target_port_center_um if correct_target else None,
            route_obj=record.route_obj,
            source_port_orientation_deg=(
                record.source_port_orientation_deg if correct_source else None
            ),
            target_port_orientation_deg=(
                record.target_port_orientation_deg if correct_target else None
            ),
        )

    candidate_modes = tuple(
        (use_source, use_target)
        for use_source, use_target in (
            (True, True),
            (True, False),
            (False, True),
            (False, False),
        )
        if (correct_source or not use_source) and (correct_target or not use_target)
    )
    centerline = ()
    for use_source, use_target in candidate_modes:
        candidate = _candidate(use_source=use_source, use_target=use_target)
        accepts = len(candidate) >= 2 and _realization_accepts(candidate)
        if trace_endpoint:
            print(
                "endpoint_trace "
                f"net={record.net_name} id={record.net_id} "
                f"crossings={len(crossing_points)} "
                f"mode=({use_source},{use_target}) "
                f"len={len(candidate)} "
                f"start={candidate[0] if candidate else None} "
                f"end={candidate[-1] if candidate else None} "
                f"source={record.source_port_center_um} "
                f"target={record.target_port_center_um} "
                f"accepts={accepts}"
            )
        if accepts:
            centerline = candidate
            break
    if len(centerline) < 2:
        message = format_port_endpoint_correction_error(
            record,
            "crossing-aware endpoint correction produced no realizable centerline",
            realization_grid_spec=realization_grid_spec,
        )
        if log_failures:
            print("ERROR: " + message)
        return replace(record, endpoint_correction_error=message)

    centerline_length = getattr(router, "centerline_length_um", None)
    if centerline_length is not None:
        try:
            corrected_total_length_um = float(centerline_length(list(centerline)))
        except Exception:
            corrected_total_length_um = _centerline_length_um(centerline)
    else:
        corrected_total_length_um = _centerline_length_um(centerline)
    return replace(
        record,
        total_length_um=corrected_total_length_um,
        base_total_length_um=(
            record.base_total_length_um
            if record.base_total_length_um is not None
            else float(record.total_length_um)
        ),
        corrected_centerline_um=centerline,
        endpoint_correction_error=None,
    )


def _apply_crossing_aware_endpoint_corrections_to_debug_artifacts(
    debug_artifacts: RustRouteDebugArtifacts,
    crossing_plan_info: Mapping[str, object] | None,
    *,
    route_width_um: float,
) -> RustRouteDebugArtifacts:
    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")
    router = _build_realization_router(
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
    )
    crossing_points_by_net_id = _legal_crossing_points_by_net_id(crossing_plan_info)
    fanout_anchor_net_ids = {
        int(net_id)
        for net_id in (
            (crossing_plan_info or {}).get("fanout_anchor_net_ids", [])
            if isinstance(crossing_plan_info, Mapping)
            else []
        )
    }
    fanout_anchor_source_net_ids = {
        int(net_id)
        for net_id in (
            (crossing_plan_info or {}).get("fanout_anchor_source_net_ids", [])
            if isinstance(crossing_plan_info, Mapping)
            else []
        )
    }
    fanout_anchor_target_net_ids = {
        int(net_id)
        for net_id in (
            (crossing_plan_info or {}).get("fanout_anchor_target_net_ids", [])
            if isinstance(crossing_plan_info, Mapping)
            else []
        )
    }
    records: list[RoutedNetRecord] = []
    for record in debug_artifacts.routed_net_records:
        net_id = int(record.net_id) if record.net_id is not None else None
        source_has_fanout_stub = (
            net_id is not None and net_id in fanout_anchor_source_net_ids
        )
        target_has_fanout_stub = (
            net_id is not None and net_id in fanout_anchor_target_net_ids
        )
        record_has_fanout_stub = (
            net_id is not None and net_id in fanout_anchor_net_ids
        )
        crossing_points = (
            crossing_points_by_net_id.get(net_id, [])
            if net_id is not None
            else []
        )
        records.append(
            _apply_crossing_aware_endpoint_correction_to_record(
                record,
                router=router,
                crossing_points=crossing_points,
                realization_grid_spec=debug_artifacts.realization_grid_spec,
                route_width_um=route_width_um,
                allow_unchecked_bumps=True,
                log_failures=not debug_artifacts.realization_allow_45_degree_turns,
                crossing_plan_info=crossing_plan_info,
                correct_source=not source_has_fanout_stub,
                correct_target=not target_has_fanout_stub,
                prefer_corrected_baseline=(
                    record_has_fanout_stub and bool(record.corrected_centerline_um)
                ),
            )
        )
    return replace(
        debug_artifacts,
        routed_net_records=records,
        routed_edge_lengths_um=routed_edge_lengths_from_records(records),
        port_alignment_diagnostics=build_port_alignment_diagnostics(
            records,
            realization_grid_spec=debug_artifacts.realization_grid_spec,
        ),
    )


def route_match_and_realize(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    enable_path_length_matching: bool = False,
    path_length_match_outputs: bool = False,
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
    enable_crossings: bool = False,
    node_depths: dict[str, int] | None = None,
    node_ranks: dict[str, int] | None = None,
    edge_ranks: dict[str, dict[str, int]] | None = None,
    crossing_loss: float = 0.0,
    crossing_mode: str = "window",
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
    fanout_access_mode: str | None = None,
    allow_only_expected_crossings: bool = True,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    debug_route_indices: set[int] | None = None,
    debug_stop_after_route_index: int | None = None,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    bend_radius_um: float | None = None,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    enable_simple_routes: bool = True,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    heap_tie_breaker: str = "smaller_g",
    proactive_congestion_weight: float = 0.0,
    proactive_congestion_radius_cells: int = 0,
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    debug_timing: bool = False,
    verbose_route_diagnostics: bool = False,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    enable_internal_photonic_probe_verification: bool = False,
    include_heater_obstacles: bool = False,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    path_length_meander_height_um: float = DEFAULT_MEANDER_MAX_HEIGHT_UM,
    enable_grid_endpoint_correction: bool = True,
) -> RouteRustPipelineResult:
    """Run Phase A->(optional M1)->B entirely in route_rust."""
    route_obstacle_config = obstacle_config
    if debug_dir is None:
        route_obstacle_config = _with_bbox_cell_materialization(
            obstacle_config,
            materialize_bbox_cells=False,
            populate_obstacle_map=False,
        )

    t_route_nets_start = time.perf_counter()
    routed_layout, debug_artifacts = route_nets_rust(
        unrouted_layout,
        schematic,
        obstacle_config=route_obstacle_config,
        debug_dir=debug_dir,
        debug_prefix=debug_prefix,
        debug_route_indices=debug_route_indices,
        debug_stop_after_route_index=debug_stop_after_route_index,
        route_width_um=route_width_um,
        route_layer=route_layer,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_um=bend_radius_um,
        enable_jps4=enable_jps4,
        use_indexed_heap=use_indexed_heap,
        enable_simple_routes=enable_simple_routes,
        primitive_ordering=primitive_ordering,
        heuristic_mode=heuristic_mode,
        heap_tie_breaker=heap_tie_breaker,
        proactive_congestion_weight=proactive_congestion_weight,
        proactive_congestion_radius_cells=proactive_congestion_radius_cells,
        max_iterations=max_iterations,
        routing_window_scale=routing_window_scale,
        debug_timing=debug_timing,
        verbose_route_diagnostics=verbose_route_diagnostics,
        collect_route_stats=collect_route_stats,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
        enable_internal_photonic_probe_verification=(
            enable_internal_photonic_probe_verification
        ),
        include_heater_obstacles=include_heater_obstacles,
        ripup_reroute_config=ripup_reroute_config,
        enable_crossings=enable_crossings,
        node_depths=node_depths,
        node_ranks=node_ranks,
        edge_ranks=edge_ranks,
        crossing_loss=crossing_loss,
        crossing_mode=crossing_mode,
        crossing_half_size_cells=crossing_half_size_cells,
        min_straight_cells_per_crossing=min_straight_cells_per_crossing,
        foreign_port_keepout_cells=foreign_port_keepout_cells,
        fanout_access_mode=fanout_access_mode,
        allow_only_expected_crossings=allow_only_expected_crossings,
        defer_realization=True,
        enable_checked_endpoint_correction=enable_grid_endpoint_correction,
    )
    pipeline_timings_s: dict[str, float] = {
        "route_nets": time.perf_counter() - t_route_nets_start,
    }
    route_nets_timings = getattr(debug_artifacts, "route_nets_timings_s", {})
    if isinstance(route_nets_timings, dict):
        for name, elapsed_s in route_nets_timings.items():
            try:
                pipeline_timings_s[f"route_nets.{name}"] = float(elapsed_s)
            except (TypeError, ValueError):
                continue
    if debug_timing and verbose_route_diagnostics:
        print(
            "      - Optical net routing phase "
            f"(obstacle map + A* + repairs): {pipeline_timings_s['route_nets']:.4f} s"
        )

    if enable_grid_endpoint_correction:
        t_endpoint_correction_start = time.perf_counter()
        # Endpoint correction is part of the live routing phase above, where the
        # router still owns the committed dynamic obstacle map. Do not run a
        # second correction pass here: the realization-only router built for
        # debug artifacts has no committed nets and cannot safely validate bump
        # candidates against neighboring routes.
        debug_artifacts = replace(
            debug_artifacts,
            port_alignment_diagnostics=build_port_alignment_diagnostics(
                debug_artifacts.routed_net_records,
                realization_grid_spec=debug_artifacts.realization_grid_spec,
            ),
        )
        pipeline_timings_s["route_endpoint_correction"] = (
            time.perf_counter() - t_endpoint_correction_start
        )
    analysis_info = None
    requirements_info = None
    meander_report_info = None
    records_for_realization = debug_artifacts.routed_net_records
    if enable_path_length_matching:
        t_analysis_start = time.perf_counter()
        analysis, requirements = analyze_path_length_matching(
            schematic,
            routed_net_records=records_for_realization,
            node_types=node_types,
            internal_delays_um=internal_delays_um,
        )
        pipeline_timings_s["path_length_analysis"] = time.perf_counter() - t_analysis_start
        analysis_info = analysis_to_info_dict(analysis)
        raw_requirements = list(requirements)
        min_insertable_extra_um = minimum_four_bend_extra_length_um(
            grid_size_um=float(debug_artifacts.realization_grid_spec[2])
            if debug_artifacts.realization_grid_spec is not None
            else 0.0,
            bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
        )
        requirements, lifted_groups = compute_group_lifted_requirements(
            analysis,
            minimum_insertable_extra_um=min_insertable_extra_um,
        )
        lifted_requirements = list(requirements)
        output_requirements: list[Any] = []
        output_matching_info = {
            "enabled": bool(path_length_match_outputs),
            "target_output_arrival_um": 0.0,
            "output_count": 0,
            "requirements": [],
            "outputs": [],
        }
        if path_length_match_outputs:
            output_requirements, output_matching_info = compute_output_matching_requirements(
                analysis,
                existing_requirements=lifted_requirements,
            )
            requirements = merge_missing_length_requirements(
                lifted_requirements,
                output_requirements,
            )
        requirement_delay_candidates = build_requirement_delay_candidates(
            analysis,
            requirements,
        )
        analysis_info["raw_requirements"] = [
            requirement_to_dict(req)
            for req in raw_requirements
        ]
        analysis_info["lifted_requirements"] = [
            requirement_to_dict(req)
            for req in lifted_requirements
        ]
        analysis_info["output_matching"] = output_matching_info
        analysis_info["output_requirements"] = [
            requirement_to_dict(req)
            for req in output_requirements
        ]
        analysis_info["requirements"] = [
            requirement_to_dict(req)
            for req in requirements
        ]
        analysis_info["requirement_delay_candidates"] = [
            {
                "edge": requirement_to_dict(req)["edge"],
                "candidates": [
                    delay_candidate_to_dict(candidate)
                    for candidate in requirement_delay_candidates.get(req.edge_key, [])
                ],
            }
            for req in requirements
        ]
        analysis_info["matching_groups"] = lifted_groups
        analysis_info["minimum_insertable_extra_length_um"] = float(
            min_insertable_extra_um
        )
        requirements_info = [requirement_to_dict(req) for req in requirements]
        if debug_artifacts.realization_grid_spec is None:
            raise RuntimeError("Missing realization grid spec from routing phase.")
        t_meander_obstacle_start = time.perf_counter()
        resolved_user_obstacle_config = _resolve_obstacle_config(
            obstacle_config,
            route_layer=route_layer,
            include_heater_obstacles=include_heater_obstacles,
        )
        meander_route_clearance_um = max(
            0.0,
            _as_float(
                getattr(resolved_user_obstacle_config, "clearance_um", 0.0),
                0.0,
            ),
        )
        meander_clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
            route_width_um=float(route_width_um),
            grid_size_um=float(debug_artifacts.realization_grid_spec[2]),
            route_clearance_um=meander_route_clearance_um,
        )
        analysis_info["clearance_policy"] = meander_clearance_policy.to_debug_dict()
        # Meander box legality should use real routed-layer geometry, not
        # conservative component bboxes. Keep static obstacles strict: source
        # and target access openings are valid for route entry/exit only, not
        # for placing meander boxes.
        meander_obstacle_config = _with_obstacle_mode(
            resolved_user_obstacle_config,
            obstacle_mode="rasterized_polygons",
            clear_port_open_cells_from_static=False,
            populate_obstacle_map=False,
            materialize_cell_sets=False,
        )
        meander_obstacle_map = build_static_obstacle_map(
            unrouted_layout,
            config=meander_obstacle_config,
        )
        meander_static_blocked_cell_handle = getattr(
            meander_obstacle_map,
            "rust_blocked_cell_handle",
            None,
        )
        meander_static_blocked_cells = (
            None
            if meander_static_blocked_cell_handle is not None
            else meander_obstacle_map.blocked_cells
        )
        pipeline_timings_s["meander_obstacle_map"] = (
            time.perf_counter() - t_meander_obstacle_start
        )

        meander_config = MeanderInsertionConfig(
            enabled=True,
            max_meander_height_um=float(path_length_meander_height_um),
        )
        analysis_info["meander_config"] = {
            "enabled": bool(meander_config.enabled),
            "min_candidate_straight_length_um": float(
                meander_config.min_candidate_straight_length_um
            ),
            "max_extra_length_per_region_um": float(
                meander_config.max_extra_length_per_region_um
            ),
            "conservative_legal_check": bool(meander_config.conservative_legal_check),
            "max_meander_height_um": float(meander_config.max_meander_height_um),
            "auto_meander_endpoint_inset_um": (
                None
                if meander_config.auto_meander_endpoint_inset_um is None
                else float(meander_config.auto_meander_endpoint_inset_um)
            ),
            "endpoint_inset_policy": (
                "adaptive"
                if meander_config.auto_meander_endpoint_inset_um is None
                else "fixed"
            ),
        }

        t_meander_planning_start = time.perf_counter()
        records_for_realization, meander_report_info = analyze_meander_insertion_for_requirements(
            records_for_realization,
            requirements,
            config=meander_config,
            realization_grid_spec=debug_artifacts.realization_grid_spec,
            allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
            bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
            static_blocked_cells=meander_static_blocked_cells,
            static_blocked_cell_handle=meander_static_blocked_cell_handle,
            route_occupancy_radius_cells=(
                meander_clearance_policy.plm_registered_route_keepout_radius_cells
            ),
            meander_box_clearance_radius_cells=(
                meander_clearance_policy.plm_candidate_box_clearance_radius_cells
            ),
            requirement_delay_candidates=requirement_delay_candidates,
        )
        pipeline_timings_s["meander_planning"] = (
            time.perf_counter() - t_meander_planning_start
        )
        if analysis_info is not None:
            matching_group_diagnostics = matching_group_diagnostics_to_info(
                analysis,
                meander_report_info,
                adjusted_requirements=requirements,
                lifted_groups=lifted_groups,
            )
            output_matching_diagnostics = output_matching_diagnostics_to_info(
                output_matching_info,
                meander_report_info,
            )
            acceptance_summary = path_length_acceptance_summary(
                matching_group_diagnostics + output_matching_diagnostics
            )
            analysis_info["matching_group_diagnostics"] = matching_group_diagnostics
            analysis_info["output_matching_diagnostics"] = output_matching_diagnostics
            analysis_info["path_length_acceptance"] = acceptance_summary
            if not acceptance_summary["passed"]:
                raise RuntimeError(
                    format_path_length_acceptance_failure(acceptance_summary)
                )

    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")

    crossing_plan_info = debug_artifacts.crossing_plan_info
    if (
        isinstance(crossing_plan_info, dict)
        and enable_internal_photonic_probe_verification
    ):
        final_records_by_net_id = _routed_records_by_net_id(records_for_realization)
        if final_records_by_net_id:
            illegal_realized_crossings = _verify_realized_route_intersections(
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=final_records_by_net_id,
                realization_grid_spec=debug_artifacts.realization_grid_spec,
            )
            _augment_insertion_loss_report_from_realized_intersections(
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=final_records_by_net_id,
            )
            if illegal_realized_crossings:
                _write_crossing_debug_artifacts(
                    debug_path=Path(debug_dir) if debug_dir is not None else Path("build"),
                    debug_prefix=debug_prefix,
                    crossing_plan_info=crossing_plan_info,
                )
                preview = "; ".join(
                    f"{item.get('net_name_a')} x {item.get('net_name_b')} "
                    f"at {item.get('point_um')} ({item.get('reason')}, "
                    f"margins={item.get('segment_a_margin_um')}/"
                    f"{item.get('segment_b_margin_um')}, "
                    f"required={item.get('required_margin_um')})"
                    for item in illegal_realized_crossings[:5]
                )
                raise RuntimeError(
                    "Illegal realized route crossing(s) after endpoint correction: "
                    f"{len(illegal_realized_crossings)} found. {preview}"
                )

    t_realization_start = time.perf_counter()
    realize_routed_net_records(
        routed_layout,
        records_for_realization,
        route_width_um=route_width_um,
        route_layer=route_layer,
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
        crossing_plan_info=crossing_plan_info,
        enable_endpoint_correction=enable_grid_endpoint_correction,
    )
    if isinstance(crossing_plan_info, dict):
        _place_realized_crossing_components(routed_layout, crossing_plan_info)
        _write_crossing_debug_artifacts(
            debug_path=Path(debug_dir) if debug_dir is not None else None,
            debug_prefix=debug_prefix,
            crossing_plan_info=crossing_plan_info,
        )
    t_realization_end = time.perf_counter()
    pipeline_timings_s["route_realization"] = t_realization_end - t_realization_start
    if debug_timing and verbose_route_diagnostics:
        print(
            "      - Optical route realization phase: "
            f"{pipeline_timings_s['route_realization']:.4f} s"
        )

    return RouteRustPipelineResult(
        routed_layout=routed_layout,
        debug_artifacts=debug_artifacts,
        path_length_analysis_info=analysis_info,
        meander_requirements_info=requirements_info,
        meander_insertion_report_info=meander_report_info,
        pipeline_timings_s=pipeline_timings_s,
    )


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _as_point_list(raw_points: Iterable[object]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for raw_point in raw_points:
        if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
            continue
        try:
            x = float(raw_point[0])
            y = float(raw_point[1])
        except (TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    return points


def _centerline_bbox_um(
    centerlines: Iterable[Iterable[tuple[float, float]]],
) -> tuple[float, float, float, float] | None:
    xs: list[float] = []
    ys: list[float] = []
    for centerline in centerlines:
        for x, y in centerline:
            xs.append(float(x))
            ys.append(float(y))
    if not xs or not ys:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def _normalize_um_bbox(raw_bbox: object) -> tuple[float, float, float, float] | None:
    if not isinstance(raw_bbox, (tuple, list)) or len(raw_bbox) != 4:
        return None
    try:
        min_x, min_y, max_x, max_y = (float(value) for value in raw_bbox)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (min_x, min_y, max_x, max_y)):
        return None
    if max_x < min_x:
        min_x, max_x = max_x, min_x
    if max_y < min_y:
        min_y, max_y = max_y, min_y
    return min_x, min_y, max_x, max_y


def _expanded_bbox_um(
    bbox: tuple[float, float, float, float],
    *,
    padding_um: float,
) -> tuple[float, float, float, float]:
    min_x, min_y, max_x, max_y = bbox
    padding = max(0.0, float(padding_um))
    return min_x - padding, min_y - padding, max_x + padding, max_y + padding


def _write_centerline_probe_svg(
    path: Path,
    *,
    centerlines_by_net: Mapping[str, Mapping[str, list[tuple[float, float]]]],
    overlap_bboxes_um: Iterable[tuple[float, float, float, float]],
    view_bbox_um: tuple[float, float, float, float],
) -> None:
    min_x, min_y, max_x, max_y = view_bbox_um
    width = max(1.0, max_x - min_x)
    height = max(1.0, max_y - min_y)
    colors = [
        ("#0b57d0", "#7baaf7"),
        ("#d93025", "#f28b82"),
        ("#188038", "#81c995"),
        ("#b06000", "#fbbc04"),
    ]

    def sx(x: float) -> float:
        return float(x) - min_x

    def sy(y: float) -> float:
        return max_y - float(y)

    def polyline(points: list[tuple[float, float]]) -> str:
        return " ".join(f"{sx(x):.6g},{sy(y):.6g}" for x, y in points)

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="1100" height="800" viewBox="0 0 {width:.6g} {height:.6g}">',
        '<rect width="100%" height="100%" fill="#ffffff" />',
        '<g stroke="#d0d7de" stroke-width="0.15" opacity="0.55">',
    ]
    grid_step = 2.0
    gx = math.floor(min_x / grid_step) * grid_step
    while gx <= max_x:
        parts.append(
            f'<path d="M {sx(gx):.6g} 0 V {height:.6g}" '
            'fill="none" />'
        )
        gx += grid_step
    gy = math.floor(min_y / grid_step) * grid_step
    while gy <= max_y:
        parts.append(
            f'<path d="M 0 {sy(gy):.6g} H {width:.6g}" '
            'fill="none" />'
        )
        gy += grid_step
    parts.append("</g>")

    for bbox in overlap_bboxes_um:
        bx0, by0, bx1, by1 = bbox
        parts.append(
            f'<rect x="{sx(bx0):.6g}" y="{sy(by1):.6g}" '
            f'width="{max(0.1, bx1 - bx0):.6g}" '
            f'height="{max(0.1, by1 - by0):.6g}" '
            'fill="#ff00aa" opacity="0.32" stroke="#9c0069" stroke-width="0.25" />'
        )

    for index, (net_name, net_centerlines) in enumerate(centerlines_by_net.items()):
        solid, dashed = colors[index % len(colors)]
        primitive = net_centerlines.get("primitive_centerline_um", [])
        corrected = net_centerlines.get("corrected_centerline_um", [])
        if primitive:
            parts.append(
                f'<polyline points="{polyline(primitive)}" fill="none" '
                f'stroke="{dashed}" stroke-width="0.55" stroke-dasharray="2 1" '
                'stroke-linecap="round" stroke-linejoin="round" />'
            )
        if corrected:
            parts.append(
                f'<polyline points="{polyline(corrected)}" fill="none" '
                f'stroke="{solid}" stroke-width="0.85" '
                'stroke-linecap="round" stroke-linejoin="round" />'
            )
        label_point = (corrected or primitive or [(min_x, max_y)])[0]
        parts.append(
            f'<text x="{sx(label_point[0]):.6g}" y="{sy(label_point[1]):.6g}" '
            f'font-size="3" fill="{solid}">{net_name}</text>'
        )

    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def _dump_photonic_probe_failure_artifacts(
    *,
    debug_path: Path,
    debug_prefix: str,
    probe_layout: Component | None,
    verification: PhotonicVerificationResult,
    records: Iterable[RoutedNetRecord],
    router: EndpointCorrectionRouter,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_unchecked_bumps: bool,
) -> dict[str, str]:
    probe_dir = debug_path / "photonic_probe_failures"
    _ensure_dir(probe_dir)
    stem = f"{debug_prefix}_photonic_probe_failure"
    artifacts: dict[str, str] = {}

    if probe_layout is not None:
        gds_path = probe_dir / f"{stem}.gds"
        probe_layout.write_gds(str(gds_path))
        artifacts["probe_gds"] = str(gds_path)

    issues_path = probe_dir / f"{stem}.json"
    issues_path.write_text(
        json.dumps(verification.as_dict(), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    artifacts["issues_json"] = str(issues_path)

    issue_net_names: set[str] = set()
    overlap_bboxes: list[tuple[float, float, float, float]] = []
    for issue in verification.issues:
        if issue.net_name:
            issue_net_names.add(str(issue.net_name))
        other = (issue.details or {}).get("other_net_name")
        if isinstance(other, str):
            issue_net_names.add(other)
        bbox = _normalize_um_bbox((issue.details or {}).get("overlap_bbox_um"))
        if bbox is not None:
            overlap_bboxes.append(bbox)

    records_by_name = {record.net_name: record for record in records}
    centerlines_by_net: dict[str, dict[str, object]] = {}
    for net_name in sorted(issue_net_names):
        record = records_by_name.get(net_name)
        if record is None:
            continue
        item: dict[str, object] = {
            "net_id": record.net_id,
            "source": str(record.source),
            "target": str(record.target),
            "source_port_center_um": record.source_port_center_um,
            "target_port_center_um": record.target_port_center_um,
            "endpoint_correction_error": record.endpoint_correction_error,
        }
        try:
            primitive = _as_point_list(router.route_primitive_centerline(record.route_obj))
        except Exception as exc:
            primitive = []
            item["primitive_centerline_error"] = str(exc)
        corrected = _as_point_list(record.corrected_centerline_um)
        if not corrected:
            try:
                corrected = _as_point_list(
                    router.route_port_corrected_centerline(
                        record.route_obj,
                        source_port_um=record.source_port_center_um,
                        target_port_um=record.target_port_center_um,
                        allow_unchecked_bumps=allow_unchecked_bumps,
                    )
                )
            except Exception as exc:
                corrected = []
                item["corrected_centerline_error"] = str(exc)
        item["primitive_centerline_um"] = primitive
        item["corrected_centerline_um"] = corrected
        centerlines_by_net[net_name] = item

    centerline_path = probe_dir / f"{stem}_centerlines.json"
    centerline_path.write_text(
        json.dumps(centerlines_by_net, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    artifacts["centerlines_json"] = str(centerline_path)

    svg_centerlines: dict[str, Mapping[str, list[tuple[float, float]]]] = {}
    for net_name, item in centerlines_by_net.items():
        svg_centerlines[net_name] = {
            "primitive_centerline_um": cast(
                list[tuple[float, float]], item.get("primitive_centerline_um", [])
            ),
            "corrected_centerline_um": cast(
                list[tuple[float, float]], item.get("corrected_centerline_um", [])
            ),
        }
    focus_bbox = overlap_bboxes[0] if overlap_bboxes else _centerline_bbox_um(
        line
        for lines in svg_centerlines.values()
        for line in lines.values()
    )
    if focus_bbox is not None:
        svg_path = probe_dir / f"{stem}_centerlines.svg"
        _write_centerline_probe_svg(
            svg_path,
            centerlines_by_net=svg_centerlines,
            overlap_bboxes_um=overlap_bboxes,
            view_bbox_um=_expanded_bbox_um(focus_bbox, padding_um=40.0),
        )
        artifacts["centerlines_svg"] = str(svg_path)

    summary_path = probe_dir / f"{stem}.txt"
    lines = [
        "Photonic probe failure artifacts",
        f"debug_prefix={debug_prefix}",
        f"realization_grid_spec={realization_grid_spec}",
    ]
    for key, value in sorted(artifacts.items()):
        lines.append(f"{key}={value}")
    for issue in verification.issues[:10]:
        lines.append(
            f"issue={issue.code} net={issue.net_name} message={issue.message} "
            f"details={issue.details}"
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    artifacts["summary_txt"] = str(summary_path)
    return artifacts


def _cells_bbox(cells: set[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    if not cells:
        return None
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return min(xs), max(xs), min(ys), max(ys)


def _rect_cell_count(
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    width = max(0, max_x - min_x + 1)
    height = max(0, max_y - min_y + 1)
    return width * height


def _rect_overlap_cell_count(
    rect: tuple[int, int, int, int],
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    rect_min_x, rect_min_y, rect_max_x, rect_max_y = rect
    overlap_min_x = max(min_x, rect_min_x)
    overlap_max_x = min(max_x, rect_max_x)
    overlap_min_y = max(min_y, rect_min_y)
    overlap_max_y = min(max_y, rect_max_y)
    return _rect_cell_count(
        min_x=overlap_min_x,
        max_x=overlap_max_x,
        min_y=overlap_min_y,
        max_y=overlap_max_y,
    )


def _route_cells_bbox(cells: Iterable[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    normalized = [(int(cell[0]), int(cell[1])) for cell in cells]
    if not normalized:
        return None
    xs = [cell[0] for cell in normalized]
    ys = [cell[1] for cell in normalized]
    return min(xs), max(xs), min(ys), max(ys)


def _format_cells_preview(cells: set[tuple[int, int]], *, limit: int = 120) -> str:
    if not cells:
        return "[]"
    ordered = sorted(cells)
    if len(ordered) <= limit:
        return str(ordered)
    head = ordered[:limit]
    return f"{head} ... (+{len(ordered) - limit} more)"


def _grid_origin_xy(grid: GridSpec) -> tuple[float, float]:
    if hasattr(grid, "origin"):
        origin = getattr(grid, "origin")
        return float(origin[0]), float(origin[1])
    return (
        float(getattr(grid, "origin_x_um", 0.0)),
        float(getattr(grid, "origin_y_um", 0.0)),
    )


def _default_obstacle_layers(
    route_layer: tuple[int, int],
    *,
    include_heater_obstacles: bool = False,
) -> tuple[tuple[int, int], ...]:
    if route_layer == (1, 0):
        return get_routing_obstacle_layers(include_heaters=include_heater_obstacles)
    return ((int(route_layer[0]), int(route_layer[1])),)


def _resolve_obstacle_config(
    obstacle_config: object | None,
    *,
    route_layer: tuple[int, int],
    include_heater_obstacles: bool = False,
) -> object:
    """Default obstacle extraction to the photonic routing layer."""

    default_layers = _default_obstacle_layers(
        route_layer,
        include_heater_obstacles=include_heater_obstacles,
    )
    if obstacle_config is None:
        return StaticObstacleMapConfig(obstacle_layers=default_layers)

    if isinstance(obstacle_config, dict):
        if obstacle_config.get("obstacle_layers") is None:
            config_dict = dict(obstacle_config)
            config_dict["obstacle_layers"] = default_layers
            return StaticObstacleMapConfig(**config_dict)
        return StaticObstacleMapConfig(**obstacle_config)

    obstacle_layers = getattr(obstacle_config, "obstacle_layers", None)
    if obstacle_layers is None:
        if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
            try:
                return replace(cast(Any, obstacle_config), obstacle_layers=default_layers)
            except Exception:
                return obstacle_config
        return obstacle_config

    return obstacle_config


def _with_bbox_cell_materialization(
    obstacle_config: object | None,
    *,
    materialize_bbox_cells: bool,
    populate_obstacle_map: bool = True,
) -> object | None:
    if obstacle_config is None:
        return {
            "materialize_bbox_cells": materialize_bbox_cells,
            "populate_obstacle_map": populate_obstacle_map,
        }

    if isinstance(obstacle_config, dict):
        config_dict = dict(obstacle_config)
        config_dict["materialize_bbox_cells"] = materialize_bbox_cells
        config_dict["populate_obstacle_map"] = populate_obstacle_map
        return config_dict

    if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
        try:
            return replace(
                cast(Any, obstacle_config),
                materialize_bbox_cells=materialize_bbox_cells,
                populate_obstacle_map=populate_obstacle_map,
            )
        except Exception:
            return obstacle_config

    return obstacle_config


def _with_obstacle_mode(
    obstacle_config: object | None,
    *,
    obstacle_mode: str,
    clear_port_open_cells_from_static: bool | None = None,
    populate_obstacle_map: bool | None = None,
    materialize_cell_sets: bool | None = None,
) -> object | None:
    updates: dict[str, object] = {"obstacle_mode": obstacle_mode}
    if clear_port_open_cells_from_static is not None:
        updates["clear_port_open_cells_from_static"] = clear_port_open_cells_from_static
    if populate_obstacle_map is not None:
        updates["populate_obstacle_map"] = populate_obstacle_map
    if materialize_cell_sets is not None:
        updates["materialize_cell_sets"] = materialize_cell_sets

    if obstacle_config is None:
        return updates

    if isinstance(obstacle_config, dict):
        config_dict = dict(obstacle_config)
        config_dict.update(updates)
        return config_dict

    if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
        try:
            return replace(cast(Any, obstacle_config), **updates)
        except Exception:
            return obstacle_config

    return obstacle_config


def _coerce_component_name(instance: object) -> str | None:
    component_obj = getattr(instance, "component", None)
    if component_obj is None:
        return None
    if isinstance(component_obj, str):
        return component_obj
    name = getattr(component_obj, "name", None)
    if isinstance(name, str) and name:
        return name
    return None


def _schematic_instance_component_name(
    schematic: object,
    instance_name: str,
) -> str | None:
    netlist = getattr(schematic, "netlist", None)
    instances = getattr(netlist, "instances", None)
    if not isinstance(instances, dict):
        return None
    instance = instances.get(instance_name)
    if instance is None:
        return None
    return _coerce_component_name(instance)


def _port_type_name(port: object) -> str | None:
    port_type = getattr(port, "port_type", None)
    if port_type is None:
        return None
    return str(port_type)


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    debug_route_indices: set[int] | None = None,
    debug_stop_after_route_index: int | None = None,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    bend_radius_um: float | None = None,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    enable_simple_routes: bool = True,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    heap_tie_breaker: str = "smaller_g",
    proactive_congestion_weight: float = 0.0,
    proactive_congestion_radius_cells: int = 0,
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    debug_timing: bool = False,
    verbose_route_diagnostics: bool = False,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    enable_internal_photonic_probe_verification: bool = False,
    include_heater_obstacles: bool = False,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    enable_crossings: bool = False,
    node_depths: dict[str, int] | None = None,
    node_ranks: dict[str, int] | None = None,
    edge_ranks: dict[str, dict[str, int]] | None = None,
    crossing_loss: float = 0.0,
    crossing_mode: str = "window",
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
    fanout_access_mode: str | None = None,
    allow_only_expected_crossings: bool = True,
    defer_realization: bool = False,
    enable_checked_endpoint_correction: bool = True,
) -> tuple[Component, RustRouteDebugArtifacts]:
    """Route schematic nets using Rust A* and add one polygon per routed net.

    This function routes each net by:
    1. Building a static obstacle map from the unrouted layout.
    2. Calling the Rust A* router for each net.
    3. Realizing one closed polygon in Rust and inserting it with add_polygon.
    4. Updating blocked cells for subsequent nets using width-aware inflation.

    Parameters:
        unrouted_layout: Component with placed instances but no routes.
        schematic: Schematic with net definitions.
        obstacle_config: Optional obstacle-map configuration.
        debug_dir: Directory where debug SVGs are written when provided.
        debug_prefix: Prefix used for debug SVG filenames.
        debug_route_indices: Optional 1-based net indices for per-route SVG
            export. When omitted, every route SVG is exported.
        debug_stop_after_route_index: Optional 1-based route index where routing
            stops after building full-netlist debug context. Port keepouts and
            crossing context are still derived from all schematic routes.
        route_width_um: Realized waveguide width in micrometers.
        route_layer: Target GDS layer/datatype tuple for route polygons.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        bend_radius_um: Minimum bend radius in micrometers. When omitted, the
            module default is used. The value is rounded up to the active grid
            cell size.
        use_indexed_heap: Benchmark-only queue experiment. Pass 8E measured
            this slower than duplicate-entry BinaryHeap queueing, so the
            production default remains False.
        enable_simple_routes: If False, skip straight/L/Z simple-route
            candidates and force routes through A* search.
        primitive_ordering: Benchmark-only dense A* primitive iteration order.
            Supported values: "library", "long_straight_first",
            "target_biased". Pass 8F keeps "library" as the default.
        heuristic_mode: Dense A* heuristic. Supported values: "distance",
            "heading_aware".
        max_iterations: Maximum A* state expansions per route attempt.
        verbose_route_diagnostics: If True, print per-net route progress and
            detailed A* timing buckets. Failures are always printed.
        enable_internal_photonic_probe_verification: If True, run the expensive
            internal realized-layout photonic probe before returning. The
            production default is False because `routing_flow.py` runs the
            authoritative Python geometry verification on the final layout.
        foreign_port_keepout_cells: Additional global keepout distance in front
            of each endpoint port. Active endpoint ports can open this region;
            dense multi-port instances can also open same-instance fanout
            keepouts, while unrelated nets cannot.
        fanout_access_mode: Dense multi-port access strategy. The default
            "legacy-runway" preserves the existing staggered source-port
            runway reservations. "off" disables those dense reservations.
            "static-stubs" pre-routes deterministic same-instance fanout
            stubs as static geometry and routes from virtual anchor ports.
        defer_realization: If True, keep routed RouteResult objects but skip
            polygon realization. This is used for pre-realization transforms
            such as path-length matching/meander insertion.
        enable_checked_endpoint_correction: If False, skip the checked
            grid-to-port correction pass used by PLM-oriented flows.

    Returns:
        A tuple of (routed_layout, debug_artifacts).
        :param debug_timing:
    """
    if route_width_um <= 0:
        raise ValueError("route_width_um must be > 0")
    if max_iterations <= 0:
        raise ValueError("max_iterations must be > 0")
    if (
        not math.isfinite(float(proactive_congestion_weight))
        or proactive_congestion_weight < 0
    ):
        raise ValueError("proactive_congestion_weight must be finite and non-negative")
    if proactive_congestion_radius_cells < 0:
        raise ValueError("proactive_congestion_radius_cells must be non-negative")
    if crossing_loss < 0:
        raise ValueError("crossing_loss must be non-negative")
    crossing_mode = str(crossing_mode).strip().lower()
    if crossing_mode in {"pure", "lidar"}:
        crossing_mode = "lidar-pure"
    if crossing_mode not in {"window", "collision", "lidar-pure"}:
        raise ValueError(
            "crossing_mode must be one of 'window', 'collision', or 'lidar-pure'"
        )
    effective_allow_only_expected_crossings = bool(allow_only_expected_crossings)
    if crossing_mode == "lidar-pure":
        effective_allow_only_expected_crossings = False
    crossing_search_loss = _effective_crossing_search_loss(
        enable_crossings=bool(enable_crossings),
        crossing_mode=crossing_mode,
        crossing_loss=float(crossing_loss),
    )
    if crossing_half_size_cells < 0:
        raise ValueError("crossing_half_size_cells must be non-negative")
    if min_straight_cells_per_crossing < 0:
        raise ValueError("min_straight_cells_per_crossing must be non-negative")
    if foreign_port_keepout_cells < 0:
        raise ValueError("foreign_port_keepout_cells must be non-negative")
    raw_fanout_access_mode = os.environ.get(
        "PHOTONIC_ROUTER_FANOUT_ACCESS_MODE",
        "legacy-runway" if fanout_access_mode is None else str(fanout_access_mode),
    )
    fanout_access_mode_normalized = raw_fanout_access_mode.strip().lower().replace("_", "-")
    fanout_mode_aliases = {
        "": "legacy-runway",
        "legacy": "legacy-runway",
        "legacy-runway": "legacy-runway",
        "staggered": "legacy-runway",
        "staggered-runway": "legacy-runway",
        "runway": "legacy-runway",
        "0": "off",
        "false": "off",
        "none": "off",
        "disabled": "off",
        "disable": "off",
        "off": "off",
        "anchor": "static-stubs",
        "anchors": "static-stubs",
        "anchor-pre-spread": "static-stubs",
        "pre-spread": "static-stubs",
        "spread-stubs": "static-stubs",
        "static-stub": "static-stubs",
        "static-stubs": "static-stubs",
        "virtual-ports": "static-stubs",
    }
    fanout_access_mode_normalized = fanout_mode_aliases.get(
        fanout_access_mode_normalized,
        fanout_access_mode_normalized,
    )
    if fanout_access_mode_normalized not in {"legacy-runway", "off", "static-stubs"}:
        raise ValueError(
            "fanout_access_mode must be one of 'legacy-runway', 'off', or 'static-stubs'"
        )
    if debug_stop_after_route_index is not None and debug_stop_after_route_index < 1:
        raise ValueError("debug_stop_after_route_index must be >= 1")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    routed_layout = unrouted_layout.copy()
    routed_layout.name = "routed_layout_rust"

    collect_pipeline_timing = (
        debug_timing or collect_route_stats or collect_attempt_diagnostics
    )
    route_nets_timings_s: dict[str, float] = {}

    def _pipeline_timer_start() -> float:
        return time.perf_counter() if collect_pipeline_timing else 0.0

    def _record_pipeline_timing(name: str, start_s: float) -> None:
        if collect_pipeline_timing:
            route_nets_timings_s[name] = route_nets_timings_s.get(name, 0.0) + (
                time.perf_counter() - start_s
            )

    t_obstacle_start = _pipeline_timer_start()
    resolved_obstacle_config = _resolve_obstacle_config(
        obstacle_config,
        route_layer=route_layer,
        include_heater_obstacles=include_heater_obstacles,
    )
    obstacle_map = build_static_obstacle_map(unrouted_layout, config=resolved_obstacle_config)
    _record_pipeline_timing("obstacle_map", t_obstacle_start)
    if debug_timing and verbose_route_diagnostics:
        print(
            "      - Obstacle Map time: "
            f"{route_nets_timings_s.get('obstacle_map', 0.0):.4f} s"
        )
    grid = obstacle_map.grid
    resolved_crossing_half_size_cells, crossing_device_info = (
        _resolve_crossing_half_size_cells(
            requested_half_size_cells=int(crossing_half_size_cells),
            enable_crossings=bool(enable_crossings),
            grid_size_um=float(grid.grid_size_um),
            clearance_um=_as_float(
                getattr(resolved_obstacle_config, "clearance_um", 0.0),
                0.0,
            ),
        )
    )

    debug_path = Path(debug_dir) if debug_dir is not None else None
    diagnostics_enabled = debug_path is not None
    obstacle_svg = None
    route_svgs: list[Path] = []

    if debug_path is not None:
        obstacle_dir = debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)
        route_dir = debug_path / "routes"
        if route_dir.exists():
            for old_artifact in route_dir.glob(f"{debug_prefix}_*"):
                if old_artifact.is_file() and old_artifact.suffix.lower() in {".svg", ".txt"}:
                    old_artifact.unlink()

    nets = schematic.netlist.routes
    if debug_route_indices is None:
        print(f"\nRouting {len(nets)} nets using Rust router...")
    else:
        selected = _format_route_indices(debug_route_indices)
        print(
            f"\nRouting {len(nets)} nets using Rust router "
            f"(printing/exporting route SVGs for indices: {selected})..."
        )

    if not hasattr(rust_backend, "PyPhotonicRouter"):
        raise RuntimeError(
            "Rust backend does not expose PyPhotonicRouter. "
            "Rebuild/install the Rust extension with the class-based API."
        )

    t_router_setup_start = _pipeline_timer_start()
    origin_x_um, origin_y_um = _grid_origin_xy(grid)
    grid_spec = rust_backend.GridSpec(
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        origin_x_um,
        origin_y_um,
    )
    bend_radius_cells = bend_radius_cells_from_um(
        bend_radius_um,
        grid_size_um=float(grid.grid_size_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid.grid_size_um),
        bend_radius_cells=bend_radius_cells,
        allow_45_degree_turns=allow_45_degree_turns,
    )
    bend_radius_cells = int(primitive_cfg.bend_radius_cells)
    astar_cfg = rust_backend.AStarConfig(max_iterations=int(max_iterations))
    astar_cfg.enable_simple_routes = bool(enable_simple_routes)
    astar_cfg.enable_jps4 = bool(enable_jps4)
    astar_cfg.use_indexed_heap = bool(use_indexed_heap or allow_45_degree_turns)
    astar_cfg.collect_detailed_timing = bool(
        debug_timing or collect_route_stats or collect_attempt_diagnostics
    )
    astar_cfg.primitive_ordering = str(primitive_ordering)
    effective_heuristic_mode = str(heuristic_mode)
    if allow_45_degree_turns and effective_heuristic_mode == "heading_aware":
        effective_heuristic_mode = "diagonal_aware"
    astar_cfg.heuristic_mode = effective_heuristic_mode
    collision_crossing_mode = bool(enable_crossings) and crossing_mode in {
        "collision",
        "lidar-pure",
    }
    if (
        allow_45_degree_turns
        and not collision_crossing_mode
        and hasattr(astar_cfg, "max_iterations")
    ):
        astar_cfg.max_iterations = min(int(astar_cfg.max_iterations), 50_000)
    if collision_crossing_mode and hasattr(astar_cfg, "heuristic_weight"):
        collision_heuristic_weight = os.environ.get(
            "PHOTONIC_ROUTER_COLLISION_HEURISTIC_WEIGHT"
        )
        astar_cfg.heuristic_weight = (
            float(collision_heuristic_weight)
            if collision_heuristic_weight
            else 1.0
        )
    elif allow_45_degree_turns and hasattr(astar_cfg, "heuristic_weight"):
        astar_cfg.heuristic_weight = max(float(astar_cfg.heuristic_weight), 1.25)
    if collision_crossing_mode and hasattr(astar_cfg, "bend_weight"):
        astar_cfg.bend_weight = float(astar_cfg.bend_weight)
    elif allow_45_degree_turns and hasattr(astar_cfg, "bend_weight"):
        # LiDAR heavily penalizes bends relative to propagation. Matching that
        # scale keeps 45-degree A* from spending work on short zig-zag variants.
        astar_cfg.bend_weight = max(float(astar_cfg.bend_weight), 12.0)
    effective_heap_tie_breaker = str(heap_tie_breaker)
    if allow_45_degree_turns and effective_heap_tie_breaker == "smaller_g":
        effective_heap_tie_breaker = "larger_g"
    astar_cfg.heap_tie_breaker = effective_heap_tie_breaker
    if hasattr(astar_cfg, "proactive_congestion_weight"):
        astar_cfg.proactive_congestion_weight = float(proactive_congestion_weight)
    if hasattr(astar_cfg, "proactive_congestion_radius_cells"):
        astar_cfg.proactive_congestion_radius_cells = int(proactive_congestion_radius_cells)
    if routing_window_scale is not None:
        astar_cfg.routing_window_scale = float(routing_window_scale)

    route_clearance_um = max(
        0.0,
        _as_float(getattr(resolved_obstacle_config, "clearance_um", 0.0), 0.0),
    )
    clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
        route_width_um=float(route_width_um),
        grid_size_um=float(grid.grid_size_um),
        route_clearance_um=route_clearance_um,
    )
    block_radius_cells = (
        clearance_policy.dynamic_obstacle_search_expansion_radius_cells
    )
    commit_radius_cells = clearance_policy.dynamic_route_commit_keepout_radius_cells
    core_commit_radius_cells = clearance_policy.dynamic_route_core_radius_cells
    routing_window_min_margin_cells = max(
        int(getattr(astar_cfg, "routing_window_min_margin_cells", 12)),
        int((2 * bend_radius_cells) + commit_radius_cells + 2),
    )
    astar_cfg.routing_window_min_margin_cells = max(
        int(getattr(astar_cfg, "routing_window_min_margin_cells", 12)),
        routing_window_min_margin_cells,
    )
    astar_cfg.simple_route_max_offset_cells = max(
        int(getattr(astar_cfg, "simple_route_max_offset_cells", 96)),
        int(12 * bend_radius_cells + 2 * commit_radius_cells),
    )
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)
    _record_pipeline_timing("router_setup", t_router_setup_start)

    port_entry_length_cells = max(2, bend_radius_cells + 2)
    port_entry_half_width_cells = max(1, bend_radius_cells + commit_radius_cells + 1)
    port_lane_length_cells = max(3, 2 * bend_radius_cells + 2)
    port_lane_half_width_cells = max(1, commit_radius_cells + 1)
    def _orientation_to_angle(orientation: float | None, *, flip: bool = False) -> int:
        if orientation is None:
            angle = 0
        else:
            angle = int(round((float(orientation) % 360.0) / 45.0)) % 8

        if flip:
            angle = (angle + 4) % 8

        return angle


    def _angle_to_step(angle: int) -> tuple[int, int]:
        steps = [
            (1, 0),  # 0 east
            (1, 1),  # 1 northeast
            (0, 1),  # 2 north
            (-1, 1),  # 3 northwest
            (-1, 0),  # 4 west
            (-1, -1),  # 5 southwest
            (0, -1),  # 6 south
            (1, -1),  # 7 southeast
        ]
        return steps[angle % 8]

    def _direction_reaches_target_ray(
        *,
        source_x: int,
        source_y: int,
        source_angle: int,
        target_x: int,
        target_y: int,
        tolerance: int,
    ) -> bool:
        dx = target_x - source_x
        dy = target_y - source_y
        if abs(dx) <= tolerance and abs(dy) <= tolerance:
            return True
        dir_x, dir_y = _angle_to_step(source_angle)
        if dir_x == 0 and dir_y == 0:
            return False
        if dir_x == 0:
            return abs(dx) <= tolerance and (dy > 0) == (dir_y > 0)
        if dir_y == 0:
            return abs(dy) <= tolerance and (dx > 0) == (dir_x > 0)
        return (
            (dx > 0) == (dir_x > 0)
            and (dy > 0) == (dir_y > 0)
            and abs(abs(dx) - abs(dy)) <= tolerance
        )

    def _source_lower_bounds(
        *,
        source_x: int,
        source_y: int,
        source_angle: int,
        target_x: int,
        target_y: int,
        target_angle: int,
    ) -> tuple[float, float]:
        grid_size_um = float(grid.grid_size_um)
        dx = target_x - source_x
        dy = target_y - source_y
        distance = math.hypot(float(dx), float(dy)) * grid_size_um
        heading_lower_bound = distance
        if str(heuristic_mode) == "heading_aware":
            target_angle_ok = not bool(getattr(astar_cfg, "require_target_angle", True)) or (
                source_angle % 8 == target_angle % 8
            )
            reaches_target_ray = _direction_reaches_target_ray(
                source_x=source_x,
                source_y=source_y,
                source_angle=source_angle,
                target_x=target_x,
                target_y=target_y,
                tolerance=max(0, int(getattr(astar_cfg, "target_tolerance_cells", 0))),
            )
            if not target_angle_ok or not reaches_target_ray:
                minimum_bend_units = 1.0 if allow_45_degree_turns else 2.0
                bend_weight = float(getattr(astar_cfg, "bend_weight", 1.0)) * float(
                    getattr(primitive_cfg, "bend_weight", 1.0)
                )
                heading_lower_bound += minimum_bend_units * bend_weight
        return distance, heading_lower_bound

    def _in_bounds(gx: int, gy: int) -> bool:
        return 0 <= gx < int(grid.width) and 0 <= gy < int(grid.height)

    def port_to_grid_state(
        port: Port,
        grid_origin_x_um: float,
        grid_origin_y_um: float,
        grid_size_um: float,
        *,
        as_target: bool = False,
        outward_cells: int = 1,
    ):
        port_angle = _orientation_to_angle(port.orientation, flip=False)

        # For choosing the grid cell, always move outward from the physical port.
        # This avoids starting inside the real component/port geometry.
        sx, sy = _angle_to_step(port_angle)

        x = float(port.center[0]) + sx * outward_cells * grid_size_um
        y = float(port.center[1]) + sy * outward_cells * grid_size_um

        gx = int((x - grid_origin_x_um) // grid_size_um)
        gy = int((y - grid_origin_y_um) // grid_size_um)

        # For the route state angle:
        # - source: route leaves the port outward
        # - target: route approaches the port, so flip direction
        route_angle = _orientation_to_angle(port.orientation, flip=as_target)

        return rust_backend.State(gx, gy, route_angle)

    def _snap_nearly_collinear_states(
        source_state: Any,
        target_state: Any,
        source_port: Port,
        target_port: Port,
    ) -> tuple[Any, Any, set[tuple[int, int]]]:
        original_cells = {
            (int(source_state.x), int(source_state.y)),
            (int(target_state.x), int(target_state.y)),
        }
        source_angle = int(source_state.angle) % 8
        target_angle = int(target_state.angle) % 8
        if source_angle != target_angle:
            return source_state, target_state, original_cells

        source_center = getattr(source_port, "center", None)
        target_center = getattr(target_port, "center", None)
        if source_center is None or target_center is None:
            return source_state, target_state, original_cells

        source_x_um = float(source_center[0])
        source_y_um = float(source_center[1])
        target_x_um = float(target_center[0])
        target_y_um = float(target_center[1])
        grid_size = float(grid.grid_size_um)
        max_snap_um = max(grid_size, 2.0 * grid_size)
        max_snap_cells = max(1, math.ceil(max_snap_um / grid_size))

        if source_angle in {0, 4}:
            direction = 1 if source_angle == 0 else -1
            if (target_x_um - source_x_um) * direction <= 0.0:
                return source_state, target_state, original_cells
            if abs(target_y_um - source_y_um) > max_snap_um:
                return source_state, target_state, original_cells
            if abs(int(target_state.y) - int(source_state.y)) > max_snap_cells:
                return source_state, target_state, original_cells
            snapped_target = rust_backend.State(
                int(target_state.x),
                int(source_state.y),
                int(target_state.angle),
            )
            return source_state, snapped_target, original_cells

        if source_angle in {2, 6}:
            direction = 1 if source_angle == 2 else -1
            if (target_y_um - source_y_um) * direction <= 0.0:
                return source_state, target_state, original_cells
            if abs(target_x_um - source_x_um) > max_snap_um:
                return source_state, target_state, original_cells
            if abs(int(target_state.x) - int(source_state.x)) > max_snap_cells:
                return source_state, target_state, original_cells
            snapped_target = rust_backend.State(
                int(source_state.x),
                int(target_state.y),
                int(target_state.angle),
            )
            return source_state, snapped_target, original_cells

        return source_state, target_state, original_cells

    def _snap_same_heading_minimum_bend_offset(
        source_state: Any,
        target_state: Any,
    ) -> tuple[Any, Any, set[tuple[int, int]]]:
        """Snap one-cell-short S-bend offsets to the nearest realizable target.

        With cardinal same-heading ports, two opposing 90-degree bend primitives
        impose a minimum perpendicular displacement of 2R. Physical port centers
        often land half a grid cell off that value. Without this snap, exact-cell
        routing can only satisfy the one-cell deficit by introducing a loop.
        """
        extra_cells: set[tuple[int, int]] = set()
        if allow_45_degree_turns:
            return source_state, target_state, extra_cells

        source_angle = int(source_state.angle) % 8
        target_angle = int(target_state.angle) % 8
        if source_angle != target_angle:
            return source_state, target_state, extra_cells

        min_offset_cells = 2 * int(bend_radius_cells)
        if min_offset_cells <= 0:
            return source_state, target_state, extra_cells

        sx = int(source_state.x)
        sy = int(source_state.y)
        tx = int(target_state.x)
        ty = int(target_state.y)

        if source_angle in {0, 4}:
            forward_dx = tx - sx if source_angle == 0 else sx - tx
            dy = ty - sy
            if forward_dx < min_offset_cells or dy == 0:
                return source_state, target_state, extra_cells
            missing = min_offset_cells - abs(dy)
            if missing != 1:
                return source_state, target_state, extra_cells
            snapped_offset_cells = min_offset_cells + 1
            snapped_target = rust_backend.State(
                tx,
                sy + (snapped_offset_cells if dy > 0 else -snapped_offset_cells),
                target_angle,
            )
            if not _in_bounds(int(snapped_target.x), int(snapped_target.y)):
                return source_state, target_state, extra_cells
            extra_cells.add((int(snapped_target.x), int(snapped_target.y)))
            return source_state, snapped_target, extra_cells

        if source_angle in {2, 6}:
            forward_dy = ty - sy if source_angle == 2 else sy - ty
            dx = tx - sx
            if forward_dy < min_offset_cells or dx == 0:
                return source_state, target_state, extra_cells
            missing = min_offset_cells - abs(dx)
            if missing != 1:
                return source_state, target_state, extra_cells
            snapped_offset_cells = min_offset_cells + 1
            snapped_target = rust_backend.State(
                sx + (snapped_offset_cells if dx > 0 else -snapped_offset_cells),
                ty,
                target_angle,
            )
            if not _in_bounds(int(snapped_target.x), int(snapped_target.y)):
                return source_state, target_state, extra_cells
            extra_cells.add((int(snapped_target.x), int(snapped_target.y)))
            return source_state, snapped_target, extra_cells

        return source_state, target_state, extra_cells

    port_open_radius_um = _as_float(
        getattr(resolved_obstacle_config, "port_open_radius_um", 0.5),
        0.5,
    )

    raw_blocked_obj: object
    if hasattr(obstacle_map, "raw_blocked_cells"):
        raw_blocked_obj = getattr(obstacle_map, "raw_blocked_cells")
    else:
        raw_blocked_obj = obstacle_map.blocked_cells
    raw_blocked_cells = cast(Iterable[tuple[int, int]], raw_blocked_obj)
    raw_static_cells = {(int(cell[0]), int(cell[1])) for cell in raw_blocked_cells}
    static_blocked_cells_before_port_reservations = raw_static_cells
    raw_static_rects_for_openings: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "raw_static_rects"):
        for rect in cast(
            Iterable[tuple[int, int, int, int]],
            getattr(obstacle_map, "raw_static_rects"),
        ):
            if len(rect) == 4:
                raw_static_rects_for_openings.append(
                    (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                )
    grid_width = int(grid.width)
    grid_height = int(grid.height)
    raw_static_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None

    def _raw_static_rect_ranges_by_y() -> dict[int, list[tuple[int, int]]]:
        nonlocal raw_static_rect_ranges_by_y
        if raw_static_rect_ranges_by_y is not None:
            return raw_static_rect_ranges_by_y
        ranges_by_y: dict[int, list[tuple[int, int]]] = {}
        for rect_min_x, rect_min_y, rect_max_x, rect_max_y in raw_static_rects_for_openings:
            min_x = max(0, rect_min_x)
            max_x = min(grid_width - 1, rect_max_x)
            min_y = max(0, rect_min_y)
            max_y = min(grid_height - 1, rect_max_y)
            if min_x > max_x or min_y > max_y:
                continue
            for y in range(min_y, max_y + 1):
                ranges_by_y.setdefault(y, []).append((min_x, max_x))
        for y, ranges in list(ranges_by_y.items()):
            ranges.sort()
            merged_ranges: list[tuple[int, int]] = []
            for min_x, max_x in ranges:
                if not merged_ranges or min_x > merged_ranges[-1][1] + 1:
                    merged_ranges.append((min_x, max_x))
                else:
                    prev_min_x, prev_max_x = merged_ranges[-1]
                    merged_ranges[-1] = (prev_min_x, max(prev_max_x, max_x))
            ranges_by_y[y] = merged_ranges
        raw_static_rect_ranges_by_y = ranges_by_y
        return ranges_by_y

    def _cell_in_raw_static(cell: tuple[int, int]) -> bool:
        if cell in raw_static_cells:
            return True
        x, y = cell
        return any(
            rect_min_x <= x <= rect_max_x
            for rect_min_x, rect_max_x in _raw_static_rect_ranges_by_y().get(y, ())
        )

    def _cells_in_raw_static_geometry(
        cells: set[tuple[int, int]],
    ) -> set[tuple[int, int]]:
        return {cell for cell in cells if _cell_in_raw_static(cell)}

    def _keyed_port_access_rule(
        *,
        instance_name: str,
        port_name: str,
        port: Port,
    ) -> tuple[float | None, float | None, str | None]:
        port_type = _port_type_name(port)
        component_name = _schematic_instance_component_name(schematic, instance_name)
        rule = (
            find_component_port_access_rule(
                component_name=component_name,
                port_name=port_name,
                port_type=port_type,
            )
            if include_heater_obstacles
            else None
        )
        if rule is not None:
            return (
                float(rule.access_length_um),
                float(rule.access_width_um),
                rule.component_name_pattern,
            )

        return None, None, None

    route_jobs: list[RouteJob] = []
    endpoint_ports_by_spec: dict[str, tuple[str, str, Port]] = {}
    endpoint_port_specs_by_instance: dict[str, set[str]] = {}
    port_access_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_runway_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_rule_by_spec: dict[str, str | None] = {}
    next_net_id = 1
    t_route_job_build_start = _pipeline_timer_start()
    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")
            source_port = get_port_from_instance(routed_layout, inst1, port1)
            target_port = get_port_from_instance(routed_layout, inst2, port2)
            route_jobs.append(
                RouteJob(
                    net_id=next_net_id,
                    route_index=next_net_id,
                    net_name=net_name,
                    inst1=inst1,
                    port1=port1,
                    inst2=inst2,
                    port2=port2,
                    source_port=source_port,
                    target_port=target_port,
                )
            )
            next_net_id += 1
            endpoint_ports_by_spec.setdefault(port1_spec, (inst1, port1, source_port))
            endpoint_ports_by_spec.setdefault(port2_spec, (inst2, port2, target_port))
            endpoint_port_specs_by_instance.setdefault(inst1, set()).add(port1_spec)
            endpoint_port_specs_by_instance.setdefault(inst2, set()).add(port2_spec)
    _record_pipeline_timing("route_job_build", t_route_job_build_start)

    source_port_specs_by_instance: dict[str, set[str]] = {}
    source_port_specs_by_instance_angle: dict[tuple[str, int], set[str]] = {}
    for run_job in route_jobs:
        port_spec = f"{run_job.inst1},{run_job.port1}"
        source_port_specs_by_instance.setdefault(run_job.inst1, set()).add(port_spec)
        angle = _orientation_to_angle(
            getattr(run_job.source_port, "orientation", None),
            flip=False,
        )
        source_port_specs_by_instance_angle.setdefault((run_job.inst1, int(angle)), set()).add(
            port_spec
        )

    def _is_dense_source_fanout_instance(instance_name: str) -> bool:
        return any(
            len(port_specs) > 2
            for (group_instance, _angle), port_specs in source_port_specs_by_instance_angle.items()
            if group_instance == instance_name
        )

    def _is_dense_source_fanout_group(instance_name: str, angle: int) -> bool:
        return (
            len(source_port_specs_by_instance_angle.get((instance_name, int(angle)), set())) > 2
        )

    @dataclass(frozen=True)
    class _FanoutAnchor:
        port_spec: str
        state_x: int
        state_y: int
        physical_angle: int
        center_um: tuple[float, float]
        stub_center_cells: tuple[tuple[int, int], ...]
        stub_centerline_um: tuple[tuple[float, float], ...]

    def _grid_cell_center_um(cell_x: int, cell_y: int) -> tuple[float, float]:
        return (
            float(origin_x_um) + (float(cell_x) + 0.5) * float(grid.grid_size_um),
            float(origin_y_um) + (float(cell_y) + 0.5) * float(grid.grid_size_um),
        )

    def _centerline_grid_cells(
        centerline_um: Iterable[tuple[float, float]],
    ) -> tuple[tuple[int, int], ...]:
        points = [
            (float(point[0]), float(point[1]))
            for point in centerline_um
            if math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
        ]
        if not points:
            return ()
        cells: list[tuple[int, int]] = []

        def append_point(point: tuple[float, float]) -> None:
            cell = _physical_point_to_grid_cell(
                point,
                grid_size_um=float(grid.grid_size_um),
                origin_x_um=float(origin_x_um),
                origin_y_um=float(origin_y_um),
            )
            if cell is None:
                return
            if not _in_bounds(cell[0], cell[1]):
                return
            if cells and cells[-1] == cell:
                return
            cells.append(cell)

        append_point(points[0])
        sample_step_um = max(float(grid.grid_size_um) / 4.0, 1.0e-6)
        for start, end in zip(points, points[1:]):
            dx = float(end[0]) - float(start[0])
            dy = float(end[1]) - float(start[1])
            length = math.hypot(dx, dy)
            if length <= 1.0e-9:
                append_point(end)
                continue
            steps = max(1, int(math.ceil(length / sample_step_um)))
            for index in range(1, steps + 1):
                t = float(index) / float(steps)
                append_point((start[0] + dx * t, start[1] + dy * t))
        return tuple(dict.fromkeys(cells))

    def _env_nonnegative_int(name: str, default: int) -> int:
        raw_value = os.environ.get(name)
        if raw_value is None or raw_value.strip() == "":
            return int(default)
        try:
            value = int(raw_value)
        except ValueError as exc:
            raise ValueError(f"{name} must be a non-negative integer") from exc
        if value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        return value

    def _env_fanout_stub_bend_steps() -> int:
        raw_value = os.environ.get("PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES", "45")
        normalized = raw_value.strip().lower().replace("_", "-")
        aliases = {
            "45": 1,
            "45deg": 1,
            "45-degree": 1,
            "45-deg": 1,
            "diagonal": 1,
            "diag": 1,
            "90": 2,
            "90deg": 2,
            "90-degree": 2,
            "90-deg": 2,
            "orthogonal": 2,
            "orthogonal-u": 2,
        }
        if normalized not in aliases:
            raise ValueError(
                "PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES must be 45 or 90"
            )
        return aliases[normalized]

    def _append_grid_step(
        path: list[tuple[int, int]],
        step_x: int,
        step_y: int,
        count: int,
    ) -> None:
        if count <= 0:
            return
        cell_x, cell_y = path[-1]
        for _ in range(count):
            cell_x += step_x
            cell_y += step_y
            if _in_bounds(cell_x, cell_y):
                path.append((cell_x, cell_y))

    def _inflated_cells(
        cells: Iterable[tuple[int, int]],
        radius: int,
    ) -> set[tuple[int, int]]:
        radius = max(0, int(radius))
        inflated: set[tuple[int, int]] = set()
        for cell_x, cell_y in cells:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx = int(cell_x) + dx
                    ny = int(cell_y) + dy
                    if _in_bounds(nx, ny):
                        inflated.add((nx, ny))
        return inflated

    def _angle_to_unit_vector(angle: int) -> tuple[float, float]:
        radians = (int(angle) % 8) * (math.pi / 4.0)
        return (math.cos(radians), math.sin(radians))

    def _rotate_left_vector(vector: tuple[float, float]) -> tuple[float, float]:
        return (-vector[1], vector[0])

    def _rotate_right_vector(vector: tuple[float, float]) -> tuple[float, float]:
        return (vector[1], -vector[0])

    def _cross2(
        a: tuple[float, float],
        b: tuple[float, float],
    ) -> float:
        return float(a[0]) * float(b[1]) - float(a[1]) * float(b[0])

    def _append_stub_point(
        out: list[tuple[float, float]],
        point: tuple[float, float],
    ) -> None:
        point = (float(point[0]), float(point[1]))
        if out:
            last_x, last_y = out[-1]
            if math.hypot(point[0] - last_x, point[1] - last_y) <= 1.0e-9:
                return
        out.append(point)

    def _append_circular_stub_bend(
        out: list[tuple[float, float]],
        *,
        start_point: tuple[float, float],
        start_angle: int,
        end_point: tuple[float, float],
        end_angle: int,
        angle_delta: int,
    ) -> None:
        radius_um = float(bend_radius_cells) * float(grid.grid_size_um)
        if radius_um <= 0.0 or not math.isfinite(radius_um):
            _append_stub_point(out, end_point)
            return
        start_dir = _angle_to_unit_vector(start_angle)
        end_dir = _angle_to_unit_vector(end_angle)
        chord = (
            float(end_point[0]) - float(start_point[0]),
            float(end_point[1]) - float(start_point[1]),
        )
        denom = _cross2(start_dir, end_dir)
        if abs(denom) <= 1.0e-9:
            _append_stub_point(out, end_point)
            return
        in_len = _cross2(chord, end_dir) / denom
        out_len = _cross2(start_dir, chord) / denom
        if (
            not math.isfinite(in_len)
            or not math.isfinite(out_len)
            or in_len <= 1.0e-9
            or out_len <= 1.0e-9
        ):
            _append_stub_point(out, end_point)
            return
        corner = (
            float(start_point[0]) + start_dir[0] * in_len,
            float(start_point[1]) + start_dir[1] * in_len,
        )
        turn_abs = abs(int(angle_delta)) * (math.pi / 4.0)
        trim = radius_um * math.tan(turn_abs / 2.0)
        trim_eff = min(trim, in_len, out_len)
        if not math.isfinite(trim_eff) or trim_eff <= 1.0e-9:
            _append_stub_point(out, end_point)
            return
        t_in = (
            corner[0] - start_dir[0] * trim_eff,
            corner[1] - start_dir[1] * trim_eff,
        )
        t_out = (
            corner[0] + end_dir[0] * trim_eff,
            corner[1] + end_dir[1] * trim_eff,
        )
        _append_stub_point(out, t_in)
        left_turn = int(angle_delta) > 0
        n_start = (
            _rotate_left_vector(start_dir)
            if left_turn
            else _rotate_right_vector(start_dir)
        )
        n_end = (
            _rotate_left_vector(end_dir)
            if left_turn
            else _rotate_right_vector(end_dir)
        )
        c0 = (
            t_in[0] + n_start[0] * radius_um,
            t_in[1] + n_start[1] * radius_um,
        )
        c1 = (
            t_out[0] + n_end[0] * radius_um,
            t_out[1] + n_end[1] * radius_um,
        )
        center = ((c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5)
        a0 = math.atan2(t_in[1] - center[1], t_in[0] - center[0])
        a1 = math.atan2(t_out[1] - center[1], t_out[0] - center[0])
        if left_turn:
            while a1 <= a0:
                a1 += math.tau
        else:
            while a1 >= a0:
                a1 -= math.tau
        arc_span = abs(a1 - a0)
        steps = max(2, int(math.ceil((arc_span / (math.pi / 2.0)) * 16.0)))
        for index in range(1, steps):
            t = float(index) / float(steps)
            angle = a0 + (a1 - a0) * t
            _append_stub_point(
                out,
                (
                    center[0] + radius_um * math.cos(angle),
                    center[1] + radius_um * math.sin(angle),
                ),
            )
        _append_stub_point(out, t_out)
        _append_stub_point(out, end_point)

    def _append_arc_from_tangencies(
        out: list[tuple[float, float]],
        *,
        t_in: tuple[float, float],
        t_out: tuple[float, float],
        start_angle: int,
        end_angle: int,
        angle_delta: int,
    ) -> None:
        radius_um = float(bend_radius_cells) * float(grid.grid_size_um)
        if radius_um <= 0.0 or not math.isfinite(radius_um):
            _append_stub_point(out, t_out)
            return
        start_dir = _angle_to_unit_vector(start_angle)
        end_dir = _angle_to_unit_vector(end_angle)
        _append_stub_point(out, t_in)
        left_turn = int(angle_delta) > 0
        n_start = (
            _rotate_left_vector(start_dir)
            if left_turn
            else _rotate_right_vector(start_dir)
        )
        n_end = (
            _rotate_left_vector(end_dir)
            if left_turn
            else _rotate_right_vector(end_dir)
        )
        c0 = (
            float(t_in[0]) + n_start[0] * radius_um,
            float(t_in[1]) + n_start[1] * radius_um,
        )
        c1 = (
            float(t_out[0]) + n_end[0] * radius_um,
            float(t_out[1]) + n_end[1] * radius_um,
        )
        center = ((c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5)
        a0 = math.atan2(float(t_in[1]) - center[1], float(t_in[0]) - center[0])
        a1 = math.atan2(float(t_out[1]) - center[1], float(t_out[0]) - center[0])
        if left_turn:
            while a1 <= a0:
                a1 += math.tau
        else:
            while a1 >= a0:
                a1 -= math.tau
        arc_span = abs(a1 - a0)
        steps = max(2, int(math.ceil((arc_span / (math.pi / 2.0)) * 16.0)))
        for index in range(1, steps):
            t = float(index) / float(steps)
            angle = a0 + (a1 - a0) * t
            _append_stub_point(
                out,
                (
                    center[0] + radius_um * math.cos(angle),
                    center[1] + radius_um * math.sin(angle),
                ),
            )
        _append_stub_point(out, t_out)

    def _append_realized_stub_bend(
        out: list[tuple[float, float]],
        start_point_um: tuple[float, float],
        start_angle: int,
        angle_delta: int,
    ) -> tuple[float, float]:
        arm_um = float(bend_radius_cells) * float(grid.grid_size_um)
        radius_um = arm_um
        turn_abs = abs(int(angle_delta)) * (math.pi / 4.0)
        trim = radius_um * math.tan(turn_abs / 2.0)
        end_angle = (int(start_angle) + int(angle_delta)) % 8
        start_dir = _angle_to_unit_vector(int(start_angle) % 8)
        end_dir = _angle_to_unit_vector(end_angle)
        start_step = _angle_to_step(int(start_angle) % 8)
        end_step = _angle_to_step(end_angle)
        start_point = (float(start_point_um[0]), float(start_point_um[1]))
        corner = (
            start_point[0] + float(start_step[0]) * arm_um,
            start_point[1] + float(start_step[1]) * arm_um,
        )
        end_point = (
            corner[0] + float(end_step[0]) * arm_um,
            corner[1] + float(end_step[1]) * arm_um,
        )
        t_in = (
            corner[0] - start_dir[0] * trim,
            corner[1] - start_dir[1] * trim,
        )
        t_out = (
            corner[0] + end_dir[0] * trim,
            corner[1] + end_dir[1] * trim,
        )
        _append_stub_point(out, t_in)
        _append_arc_from_tangencies(
            out,
            t_in=t_in,
            t_out=t_out,
            start_angle=int(start_angle) % 8,
            end_angle=end_angle,
            angle_delta=int(angle_delta),
        )
        _append_stub_point(out, end_point)
        return end_point

    def _two_bend_static_stub_centerline_um(
        port_center_um: tuple[float, float],
        physical_angle: int,
        lateral_sign: int,
        target_anchor_y_cell: int | None = None,
        min_forward_cells: int = 0,
        initial_forward_cells: int = 0,
        extra_final_forward_cells: int = 0,
    ) -> tuple[tuple[tuple[float, float], ...], tuple[int, int]] | None:
        start_angle = int(physical_angle) % 8
        bend_delta = int(lateral_sign) * _env_fanout_stub_bend_steps()
        intermediate_angle = (start_angle + bend_delta) % 8
        intermediate_step = _angle_to_step(intermediate_angle)
        final_step = _angle_to_step(start_angle)
        trace_fanout_stubs = os.environ.get("PHOTONIC_ROUTER_TRACE_FANOUT_STUBS", "").strip()

        def fail(reason: str, extra: str = "") -> None:
            if trace_fanout_stubs:
                print(
                    "fanout_stub_failed "
                    f"reason={reason} "
                    f"port={port_center_um} "
                    f"angle={start_angle} lateral_sign={lateral_sign} "
                    f"target_anchor_y_cell={target_anchor_y_cell} "
                    f"min_forward_cells={min_forward_cells} "
                    f"initial_forward_cells={initial_forward_cells} "
                    f"extra_final_forward_cells={extra_final_forward_cells}"
                    f"{extra}",
                    file=sys.stderr,
                )

        if abs(final_step[0]) + abs(final_step[1]) != 1:
            fail("non_cardinal_final")
            return None
        if intermediate_step[1] == 0:
            fail("intermediate_has_no_y")
            return None
        port_point = (float(port_center_um[0]), float(port_center_um[1]))

        def _next_grid_axis_value(
            value: float,
            origin: float,
            direction: int,
        ) -> float | None:
            if direction == 0:
                return None
            rel = (float(value) - float(origin)) / float(grid.grid_size_um) - 0.5
            eps = 1.0e-9
            if direction > 0:
                index = math.ceil(rel - eps)
            else:
                index = math.floor(rel + eps)
            return float(origin) + (float(index) + 0.5) * float(grid.grid_size_um)

        points: list[tuple[float, float]] = [port_point]
        bend_start = port_point
        initial_forward_um = (
            float(max(0, int(initial_forward_cells))) * float(grid.grid_size_um)
        )
        if initial_forward_um > 1.0e-9:
            bend_start = (
                port_point[0] + float(final_step[0]) * initial_forward_um,
                port_point[1] + float(final_step[1]) * initial_forward_um,
            )
            _append_stub_point(points, bend_start)
        first_end = _append_realized_stub_bend(
            points,
            bend_start,
            start_angle,
            bend_delta,
        )
        if target_anchor_y_cell is None:
            target_intermediate_y = _next_grid_axis_value(
                first_end[1],
                origin_y_um,
                int(intermediate_step[1]),
            )
        else:
            target_intermediate_y = _grid_cell_center_um(
                0,
                int(target_anchor_y_cell)
                - int(intermediate_step[1]) * int(bend_radius_cells),
            )[1]
        if target_intermediate_y is None:
            fail("no_target_intermediate_y")
            return None
        intermediate_delta_y = float(target_intermediate_y) - float(first_end[1])
        if intermediate_delta_y * float(intermediate_step[1]) < -1.0e-9:
            fail("intermediate_moves_backward")
            return None
        intermediate_delta_x = intermediate_delta_y * (
            float(intermediate_step[0]) / float(intermediate_step[1])
        )
        intermediate_end = (
            first_end[0] + intermediate_delta_x,
            float(target_intermediate_y),
        )
        _append_stub_point(points, intermediate_end)
        second_end = _append_realized_stub_bend(
            points,
            intermediate_end,
            intermediate_angle,
            angle_delta=-bend_delta,
        )
        if final_step[0] != 0:
            target_final_x = _next_grid_axis_value(
                second_end[0],
                origin_x_um,
                int(final_step[0]),
            )
            if target_final_x is None:
                fail("no_target_final_x")
                return None
            min_forward_x = (
                port_point[0]
                + float(final_step[0])
                * float(
                    max(0, int(min_forward_cells))
                    + max(0, int(initial_forward_cells))
                    + max(0, int(extra_final_forward_cells))
                )
                * float(grid.grid_size_um)
            )
            if int(final_step[0]) > 0:
                if float(target_final_x) < float(min_forward_x):
                    snapped_min_forward_x = _next_grid_axis_value(
                        float(min_forward_x),
                        origin_x_um,
                        int(final_step[0]),
                    )
                    if snapped_min_forward_x is None:
                        fail("no_snapped_min_forward_x")
                        return None
                    target_final_x = float(snapped_min_forward_x)
            else:
                if float(target_final_x) > float(min_forward_x):
                    snapped_min_forward_x = _next_grid_axis_value(
                        float(min_forward_x),
                        origin_x_um,
                        int(final_step[0]),
                    )
                    if snapped_min_forward_x is None:
                        fail("no_snapped_min_forward_x")
                        return None
                    target_final_x = float(snapped_min_forward_x)
            final_delta_x = float(target_final_x) - float(second_end[0])
            if final_delta_x * float(final_step[0]) < -1.0e-9:
                fail("final_moves_backward_x")
                return None
            anchor_point = (float(target_final_x), float(second_end[1]))
        else:
            target_final_y = _next_grid_axis_value(
                second_end[1],
                origin_y_um,
                int(final_step[1]),
            )
            if target_final_y is None:
                fail("no_target_final_y")
                return None
            final_delta_y = float(target_final_y) - float(second_end[1])
            if final_delta_y * float(final_step[1]) < -1.0e-9:
                fail("final_moves_backward_y")
                return None
            anchor_point = (float(second_end[0]), float(target_final_y))
        _append_stub_point(points, anchor_point)
        anchor_x = int(
            round((anchor_point[0] - origin_x_um) / float(grid.grid_size_um) - 0.5)
        )
        anchor_y = int(
            round((anchor_point[1] - origin_y_um) / float(grid.grid_size_um) - 0.5)
        )
        snapped_anchor = _grid_cell_center_um(anchor_x, anchor_y)
        snap_error_um = math.hypot(
            float(snapped_anchor[0]) - float(anchor_point[0]),
            float(snapped_anchor[1]) - float(anchor_point[1]),
        )
        if snap_error_um > max(1.0e-6, 0.05 * float(grid.grid_size_um)):
            fail(
                f"snap_error:{snap_error_um:.6g}",
                " "
                f"anchor_point=({anchor_point[0]:.6g},{anchor_point[1]:.6g}) "
                f"anchor_cell=({anchor_x},{anchor_y}) "
                f"snapped=({snapped_anchor[0]:.6g},{snapped_anchor[1]:.6g}) "
                f"origin=({origin_x_um:.6g},{origin_y_um:.6g}) "
                f"grid={float(grid.grid_size_um):.6g} "
                f"bend_radius_cells={bend_radius_cells}",
            )
            return None
        if not _in_bounds(anchor_x, anchor_y):
            fail("anchor_out_of_bounds")
            return None
        return _compress_centerline(tuple(points)), (anchor_x, anchor_y)

    def _fanout_stub_centerline_um(
        port_center_um: tuple[float, float] | None,
        anchor_center_um: tuple[float, float],
        physical_angle: int,
    ) -> tuple[tuple[float, float], ...]:
        if port_center_um is None:
            return (anchor_center_um,)
        forward_x, forward_y = _angle_to_step(int(physical_angle) % 8)
        lateral_x, lateral_y = -forward_y, forward_x
        port_x, port_y = (float(port_center_um[0]), float(port_center_um[1]))
        anchor_x, anchor_y = (float(anchor_center_um[0]), float(anchor_center_um[1]))
        delta_x = anchor_x - port_x
        delta_y = anchor_y - port_y
        forward_delta = delta_x * forward_x + delta_y * forward_y
        lateral_delta = delta_x * lateral_x + delta_y * lateral_y
        if forward_delta <= 1.0e-9:
            return _compress_centerline((port_center_um, anchor_center_um))
        lateral_abs = abs(lateral_delta)
        available_straight = forward_delta - lateral_abs
        if available_straight <= 1.0e-9:
            return _compress_centerline((port_center_um, anchor_center_um))

        preferred_first_straight_um = max(
            float(grid.grid_size_um),
            float(bend_radius_cells) * float(grid.grid_size_um),
        )
        first_straight_um = min(preferred_first_straight_um, available_straight)
        points: list[tuple[float, float]] = [
            (port_x, port_y),
            (
                port_x + forward_x * first_straight_um,
                port_y + forward_y * first_straight_um,
            ),
        ]
        if lateral_abs > 1.0e-9:
            lateral_sign = 1 if lateral_delta > 0.0 else -1
            diagonal_angle = (int(physical_angle) + lateral_sign) % 8
            diagonal_end = (
                points[-1][0]
                + forward_x * lateral_abs
                + lateral_x * lateral_delta,
                points[-1][1]
                + forward_y * lateral_abs
                + lateral_y * lateral_delta,
            )
            smoothed: list[tuple[float, float]] = [points[0]]
            _append_circular_stub_bend(
                smoothed,
                start_point=points[0],
                start_angle=physical_angle,
                end_point=diagonal_end,
                end_angle=diagonal_angle,
                angle_delta=lateral_sign,
            )
            _append_circular_stub_bend(
                smoothed,
                start_point=diagonal_end,
                start_angle=diagonal_angle,
                end_point=(anchor_x, anchor_y),
                end_angle=physical_angle,
                angle_delta=-lateral_sign,
            )
            return _compress_centerline(tuple(smoothed))
        points.append((anchor_x, anchor_y))
        return _compress_centerline(tuple(points))

    def _build_static_fanout_anchors() -> dict[str, _FanoutAnchor]:
        if fanout_access_mode_normalized != "static-stubs":
            return {}
        default_forward_cells = max(3, int(bend_radius_cells) + 3)
        default_lane_spacing_cells = 11
        forward_cells = _env_nonnegative_int(
            "PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS",
            default_forward_cells,
        )
        lane_spacing_cells = _env_nonnegative_int(
            "PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS",
            default_lane_spacing_cells,
        )
        stub_x_offset_cells = _env_nonnegative_int(
            "PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS",
            1,
        )
        if forward_cells <= 0 or lane_spacing_cells <= 0:
            return {}

        anchors: dict[str, _FanoutAnchor] = {}
        for instance_name, port_specs in source_port_specs_by_instance.items():
            if not _is_dense_source_fanout_instance(instance_name):
                continue
            by_angle: dict[int, list[str]] = {}
            for port_spec in port_specs:
                _inst, _port_name, port = endpoint_ports_by_spec[port_spec]
                angle = _orientation_to_angle(getattr(port, "orientation", None), flip=False)
                step_x, step_y = _angle_to_step(angle)
                # The first static-stub implementation intentionally handles
                # cardinal MMI port rows. Diagonal component ports fall back to
                # the normal endpoint behavior until a safe breakout is defined.
                if abs(step_x) + abs(step_y) != 1:
                    continue
                by_angle.setdefault(angle, []).append(port_spec)

            for angle, group_specs in by_angle.items():
                if not _is_dense_source_fanout_group(instance_name, angle):
                    continue
                step_x, step_y = _angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x
                ordered_items: list[tuple[str, int, Any]] = []
                for port_spec in group_specs:
                    _inst, _port_name, port = endpoint_ports_by_spec[port_spec]
                    state = port_to_grid_state(
                        port,
                        origin_x_um,
                        origin_y_um,
                        float(grid.grid_size_um),
                        as_target=False,
                    )
                    lateral_cell = int(state.x) * lateral_x + int(state.y) * lateral_y
                    ordered_items.append((port_spec, lateral_cell, state))
                ordered_items.sort(key=lambda item: (item[1], item[0]))
                count = len(ordered_items)
                if count <= 2 or step_y != 0:
                    continue

                def add_two_bend_anchor(
                    item: tuple[str, int, Any],
                    lateral_sign: int,
                    target_anchor_y_cell: int | None,
                    initial_forward_cells: int = 0,
                    extra_final_forward_cells: int = 0,
                ) -> tuple[int, int] | None:
                    port_spec, _current_lateral, state = item
                    _inst, _port_name, port = endpoint_ports_by_spec[port_spec]
                    real_center = _port_center_um(port)
                    if real_center is None:
                        return None
                    stub_result = _two_bend_static_stub_centerline_um(
                        real_center,
                        angle,
                        lateral_sign,
                        target_anchor_y_cell=target_anchor_y_cell,
                        min_forward_cells=int(forward_cells),
                        initial_forward_cells=max(0, int(initial_forward_cells)),
                        extra_final_forward_cells=max(0, int(extra_final_forward_cells)),
                    )
                    if stub_result is None:
                        return None
                    centerline, (anchor_x, anchor_y) = stub_result
                    anchor_center = _grid_cell_center_um(anchor_x, anchor_y)
                    anchors[port_spec] = _FanoutAnchor(
                        port_spec=port_spec,
                        state_x=anchor_x,
                        state_y=anchor_y,
                        physical_angle=angle,
                        center_um=anchor_center,
                        stub_center_cells=_centerline_grid_cells(centerline),
                        stub_centerline_um=centerline,
                    )
                    return anchor_x, anchor_y

                lower_items = ordered_items[: count // 2]
                upper_items = ordered_items[count // 2 :]
                if not lower_items or not upper_items:
                    continue
                stub_bend_steps = _env_fanout_stub_bend_steps()
                stagger_forward_cells = (
                    int(stub_x_offset_cells) if int(stub_bend_steps) >= 2 else 0
                )

                lower_inner = lower_items[-1]
                lower_count = len(lower_items)
                lower_inner_anchor = add_two_bend_anchor(
                    lower_inner,
                    -1,
                    target_anchor_y_cell=None,
                    initial_forward_cells=(lower_count - 1) * stagger_forward_cells,
                    extra_final_forward_cells=0,
                )
                if lower_inner_anchor is not None:
                    lower_base_y = int(lower_inner_anchor[1])
                    for rank, item in enumerate(reversed(lower_items[:-1]), start=1):
                        initial_rank = lower_count - 1 - int(rank)
                        final_rank = int(rank)
                        _min_anchor = add_two_bend_anchor(
                            item,
                            -1,
                            target_anchor_y_cell=None,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )
                        desired_y = lower_base_y - int(rank) * int(lane_spacing_cells)
                        if _min_anchor is not None:
                            desired_y = min(desired_y, int(_min_anchor[1]))
                        add_two_bend_anchor(
                            item,
                            -1,
                            target_anchor_y_cell=desired_y,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )

                upper_inner = upper_items[0]
                upper_count = len(upper_items)
                upper_inner_anchor = add_two_bend_anchor(
                    upper_inner,
                    1,
                    target_anchor_y_cell=None,
                    initial_forward_cells=(upper_count - 1) * stagger_forward_cells,
                    extra_final_forward_cells=0,
                )
                if upper_inner_anchor is not None:
                    upper_base_y = int(upper_inner_anchor[1])
                    for rank, item in enumerate(upper_items[1:], start=1):
                        initial_rank = upper_count - 1 - int(rank)
                        final_rank = int(rank)
                        _min_anchor = add_two_bend_anchor(
                            item,
                            1,
                            target_anchor_y_cell=None,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )
                        desired_y = upper_base_y + int(rank) * int(lane_spacing_cells)
                        if _min_anchor is not None:
                            desired_y = max(desired_y, int(_min_anchor[1]))
                        add_two_bend_anchor(
                            item,
                            1,
                            target_anchor_y_cell=desired_y,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )
        return anchors

    fanout_anchor_by_port_spec = _build_static_fanout_anchors()
    fanout_stub_static_cells_by_spec: dict[str, set[tuple[int, int]]] = {
        port_spec: _inflated_cells(anchor.stub_center_cells, int(commit_radius_cells))
        for port_spec, anchor in fanout_anchor_by_port_spec.items()
    }
    fanout_stub_center_cells: set[tuple[int, int]] = set()
    for anchor in fanout_anchor_by_port_spec.values():
        fanout_stub_center_cells.update(anchor.stub_center_cells)
    fanout_stub_static_cells: set[tuple[int, int]] = set()
    for cells in fanout_stub_static_cells_by_spec.values():
        fanout_stub_static_cells.update(cells)
    fanout_anchor_net_ids = {
        int(job.net_id)
        for job in route_jobs
        if f"{job.inst1},{job.port1}" in fanout_anchor_by_port_spec
        or f"{job.inst2},{job.port2}" in fanout_anchor_by_port_spec
    }
    fanout_anchor_source_net_ids = {
        int(job.net_id)
        for job in route_jobs
        if f"{job.inst1},{job.port1}" in fanout_anchor_by_port_spec
    }
    fanout_anchor_target_net_ids = {
        int(job.net_id)
        for job in route_jobs
        if f"{job.inst2},{job.port2}" in fanout_anchor_by_port_spec
    }

    def _dense_source_port_runway_lengths(
        jobs: list[RouteJob],
    ) -> dict[str, int]:
        """Reserve staggered source-port access in dense MMI fanout runs."""
        lengths_by_spec: dict[str, int] = {}
        if fanout_access_mode_normalized == "static-stubs":
            grouped_specs: dict[tuple[str, int], set[str]] = {}
            source_specs = {
                f"{run_job.inst1},{run_job.port1}"
                for run_job in jobs
                if f"{run_job.inst1},{run_job.port1}" in fanout_anchor_by_port_spec
            }
            for port_spec in source_specs:
                anchor = fanout_anchor_by_port_spec[port_spec]
                instance_name = port_spec.split(",", 1)[0]
                grouped_specs.setdefault(
                    (instance_name, int(anchor.physical_angle) % 8),
                    set(),
                ).add(port_spec)

            for (_instance_name, angle), specs in grouped_specs.items():
                if len(specs) <= 2:
                    continue
                step_x, step_y = _angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x

                def anchor_lateral_position(port_spec: str) -> int:
                    anchor = fanout_anchor_by_port_spec[port_spec]
                    return int(anchor.state_x) * lateral_x + int(anchor.state_y) * lateral_y

                ordered_specs = sorted(
                    specs,
                    key=lambda port_spec: (
                        anchor_lateral_position(port_spec),
                        port_spec,
                    ),
                )
                count = len(ordered_specs)
                spacing_cells = _env_nonnegative_int(
                    "PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS",
                    _env_nonnegative_int(
                        "PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS",
                        3,
                    ),
                )
                spacing_cells = max(1, int(spacing_cells))
                lower_specs = ordered_specs[: count // 2]
                upper_specs = ordered_specs[count // 2 :]
                for port_index, port_spec in enumerate(lower_specs):
                    runway_rank = int(port_index) + 1
                    lengths_by_spec[port_spec] = spacing_cells * runway_rank
                upper_count = len(upper_specs)
                for port_index, port_spec in enumerate(upper_specs):
                    runway_rank = upper_count - int(port_index)
                    lengths_by_spec[port_spec] = spacing_cells * runway_rank
            return lengths_by_spec

        if fanout_access_mode_normalized != "legacy-runway":
            return {}

        index = 0
        while index < len(jobs):
            job = jobs[index]
            if not _is_dense_source_fanout_instance(job.inst1):
                index += 1
                continue
            run_end = index + 1
            while (
                run_end < len(jobs)
                and jobs[run_end].inst1 == job.inst1
                and _is_dense_source_fanout_instance(jobs[run_end].inst1)
            ):
                run_end += 1

            run = jobs[index:run_end]
            by_angle: dict[int, list[RouteJob]] = {}
            for run_job in run:
                angle = _orientation_to_angle(
                    getattr(run_job.source_port, "orientation", None),
                    flip=False,
                )
                by_angle.setdefault(angle, []).append(run_job)

            for angle, angle_jobs in by_angle.items():
                if not _is_dense_source_fanout_group(job.inst1, angle):
                    continue
                step_x, step_y = _angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x

                def lateral_position(run_job: RouteJob) -> float:
                    center = _port_center_um(run_job.source_port)
                    if center is None:
                        return float(run_job.route_index)
                    return float(center[0]) * lateral_x + float(center[1]) * lateral_y

                ordered = sorted(
                    angle_jobs,
                    key=lambda run_job: (lateral_position(run_job), int(run_job.route_index)),
                )
                count = len(ordered)
                for port_index, run_job in enumerate(ordered):
                    port_spec = f"{run_job.inst1},{run_job.port1}"
                    lengths_by_spec[port_spec] = 3 + 3 * (count - 1 - port_index)

            index = run_end
        return lengths_by_spec

    dense_source_port_runway_length_by_spec = _dense_source_port_runway_lengths(route_jobs)
    dense_source_cluster_specs_by_port_spec: dict[str, set[str]] = {}
    index = 0
    while index < len(route_jobs):
        job = route_jobs[index]
        if f"{job.inst1},{job.port1}" not in dense_source_port_runway_length_by_spec:
            index += 1
            continue
        run_end = index + 1
        while (
            run_end < len(route_jobs)
            and route_jobs[run_end].inst1 == job.inst1
            and f"{route_jobs[run_end].inst1},{route_jobs[run_end].port1}"
            in dense_source_port_runway_length_by_spec
        ):
            run_end += 1
        cluster_specs = {
            f"{run_job.inst1},{run_job.port1}"
            for run_job in route_jobs[index:run_end]
            if f"{run_job.inst1},{run_job.port1}" in dense_source_port_runway_length_by_spec
        }
        if len(cluster_specs) > 1:
            for port_spec in cluster_specs:
                dense_source_cluster_specs_by_port_spec[port_spec] = set(cluster_specs)
        index = run_end
    if fanout_anchor_by_port_spec:
        static_stub_groups: dict[tuple[str, int], set[str]] = {}
        for port_spec, anchor in fanout_anchor_by_port_spec.items():
            instance_name = port_spec.split(",", 1)[0]
            static_stub_groups.setdefault(
                (instance_name, int(anchor.physical_angle) % 8),
                set(),
            ).add(port_spec)
        for cluster_specs in static_stub_groups.values():
            if len(cluster_specs) <= 1:
                continue
            for port_spec in cluster_specs:
                dense_source_cluster_specs_by_port_spec[port_spec] = set(cluster_specs)

    t_crossing_context_start = _pipeline_timer_start()
    crossing_plan_info = _build_crossing_plan_info(
        rust_backend=rust_backend,
        router=router,
        schematic=schematic,
        route_jobs=route_jobs,
        enable_crossings=enable_crossings,
        node_depths=node_depths,
        node_ranks=node_ranks,
        edge_ranks=edge_ranks,
        crossing_loss=float(crossing_loss),
        crossing_search_loss=float(crossing_search_loss),
        crossing_half_size_cells=int(resolved_crossing_half_size_cells),
        min_straight_cells_per_crossing=int(min_straight_cells_per_crossing),
        allow_only_expected_crossings=effective_allow_only_expected_crossings,
    )
    crossing_plan_info["crossing_mode"] = crossing_mode
    crossing_plan_info["requested_allow_only_expected_crossings"] = bool(
        allow_only_expected_crossings
    )
    crossing_plan_info["bend_runout_cells_per_crossing"] = int(bend_radius_cells)
    crossing_plan_info["fanout_stub_bend_degrees"] = 45 * int(
        _env_fanout_stub_bend_steps()
    )
    crossing_plan_info["required_straight_margin_cells_per_crossing"] = int(
        resolved_crossing_half_size_cells
    ) + int(bend_radius_cells)
    crossing_plan_info["fanout_access_mode"] = fanout_access_mode_normalized
    crossing_plan_info["fanout_anchor_port_count"] = len(fanout_anchor_by_port_spec)
    crossing_plan_info["fanout_anchor_net_ids"] = sorted(fanout_anchor_net_ids)
    crossing_plan_info["fanout_anchor_source_net_ids"] = sorted(
        fanout_anchor_source_net_ids
    )
    crossing_plan_info["fanout_anchor_target_net_ids"] = sorted(
        fanout_anchor_target_net_ids
    )
    crossing_plan_info["fanout_stub_center_cell_count"] = len(fanout_stub_center_cells)
    crossing_plan_info["fanout_stub_static_cell_count"] = len(fanout_stub_static_cells)
    crossing_plan_info["fanout_stub_centerlines_um"] = [
        {
            "port_spec": anchor.port_spec,
            "anchor_cell": [int(anchor.state_x), int(anchor.state_y)],
            "physical_angle": int(anchor.physical_angle) % 8,
            "centerline_um": [
                [float(point[0]), float(point[1])]
                for point in anchor.stub_centerline_um
            ],
        }
        for anchor in sorted(
            fanout_anchor_by_port_spec.values(),
            key=lambda item: item.port_spec,
        )
    ]
    crossing_plan_info["crossing_device"] = crossing_device_info
    if bool(enable_crossings) and crossing_mode in {"collision", "lidar-pure"}:
        if not hasattr(router, "set_collision_crossing_routing"):
            extension_path = getattr(rust_backend, "__file__", "<unknown>")
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.set_collision_crossing_routing. Rebuild it with "
                "`maturin develop --release`. "
                f"Loaded extension: {extension_path}"
            )
        router.set_collision_crossing_routing(True)
    elif hasattr(router, "set_collision_crossing_routing"):
        router.set_collision_crossing_routing(False)
    _record_pipeline_timing("crossing_context", t_crossing_context_start)

    if not hasattr(router, "build_route_port_openings"):
        extension_path = getattr(rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.build_route_port_openings. Rebuild it with "
            "`maturin develop --release`. "
            f"Loaded extension: {extension_path}"
        )

    t_port_opening_prep_start = _pipeline_timer_start()
    port_opening_inputs: list[
        tuple[str, float, float, float | None, str | None, float | None, float | None]
    ] = []
    for port_spec, (instance_name, port_name, port) in endpoint_ports_by_spec.items():
        fanout_anchor = fanout_anchor_by_port_spec.get(port_spec)
        center = fanout_anchor.center_um if fanout_anchor is not None else _port_center_um(port)
        if center is None:
            raise ValueError(f"Port {port_spec!r} has no finite center coordinate")
        orientation_value = getattr(port, "orientation", None)
        orientation = None if orientation_value is None else float(orientation_value)
        port_type = _port_type_name(port)
        access_length_um, access_width_um, rule_name = _keyed_port_access_rule(
            instance_name=instance_name,
            port_name=port_name,
            port=port,
        )
        port_access_rule_by_spec[port_spec] = rule_name
        port_opening_inputs.append(
            (
                port_spec,
                float(center[0]),
                float(center[1]),
                orientation,
                port_type,
                access_length_um,
                access_width_um,
            )
        )
    _record_pipeline_timing("port_opening_prep", t_port_opening_prep_start)

    raw_static_cells_for_openings = sorted(raw_static_cells)
    t_port_opening_batch_start = _pipeline_timer_start()
    default_runway_length_cells = int(bend_radius_cells) + 1
    port_opening_groups: dict[
        tuple[int, bool],
        list[tuple[str, float, float, float | None, str | None, float | None, float | None]],
    ] = {}
    for item in port_opening_inputs:
        port_spec = str(item[0])
        custom_runway_length = dense_source_port_runway_length_by_spec.get(port_spec)
        if custom_runway_length is None:
            group_key = (default_runway_length_cells, False)
        else:
            group_key = (max(1, int(custom_runway_length)), True)
        port_opening_groups.setdefault(group_key, []).append(item)

    for (runway_length_cells, custom_dense_runway), grouped_inputs in port_opening_groups.items():
        grouped_port_entry_length_cells = (
            min(int(port_entry_length_cells), int(runway_length_cells))
            if custom_dense_runway
            else int(port_entry_length_cells)
        )
        grouped_port_lane_length_cells = (
            int(runway_length_cells)
            if custom_dense_runway
            else int(port_lane_length_cells)
        )
        for port_spec, cells, candidate_cells, runway_cells in router.build_route_port_openings(
            grouped_inputs,
            raw_static_cells=raw_static_cells_for_openings,
            raw_static_rects=raw_static_rects_for_openings,
            route_clearance_um=float(route_clearance_um),
            port_open_radius_um=float(port_open_radius_um),
            bend_radius_cells=max(0, int(runway_length_cells) - 1),
            commit_radius_cells=int(commit_radius_cells),
            port_entry_length_cells=grouped_port_entry_length_cells,
            port_entry_half_width_cells=int(port_entry_half_width_cells),
            port_lane_length_cells=grouped_port_lane_length_cells,
            port_lane_half_width_cells=int(port_lane_half_width_cells),
        ):
            port_access_cells_by_spec[str(port_spec)] = {
                (int(cell[0]), int(cell[1])) for cell in cells
            }
            port_access_candidate_cells_by_spec[str(port_spec)] = {
                (int(cell[0]), int(cell[1])) for cell in candidate_cells
            }
            port_runway_cells_by_spec[str(port_spec)] = {
                (int(cell[0]), int(cell[1])) for cell in runway_cells
            }
    _record_pipeline_timing("port_opening_batch", t_port_opening_batch_start)

    foreign_port_keepout_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
    foreign_port_keepout_nonstatic_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
    if foreign_port_keepout_cells > 0:
        t_foreign_keepout_start = _pipeline_timer_start()
        foreign_length_cells = int(foreign_port_keepout_cells)
        foreign_half_width_cells = int(foreign_port_keepout_cells)
        for port_spec, _cells, _candidate_cells, runway_cells in (
            router.build_route_port_openings(
                port_opening_inputs,
                raw_static_cells=raw_static_cells_for_openings,
                raw_static_rects=raw_static_rects_for_openings,
                route_clearance_um=float(route_clearance_um),
                port_open_radius_um=float(port_open_radius_um),
                bend_radius_cells=max(0, foreign_length_cells - 1),
                commit_radius_cells=foreign_half_width_cells,
                port_entry_length_cells=int(port_entry_length_cells),
                port_entry_half_width_cells=int(port_entry_half_width_cells),
                port_lane_length_cells=int(port_lane_length_cells),
                port_lane_half_width_cells=int(port_lane_half_width_cells),
            )
        ):
            instance_name = str(port_spec).split(",", 1)[0]
            cells_for_spec = {(int(cell[0]), int(cell[1])) for cell in runway_cells}
            foreign_port_keepout_cells_by_spec[str(port_spec)] = cells_for_spec
            foreign_port_keepout_cells_by_instance.setdefault(instance_name, set()).update(cells_for_spec)
            nonstatic_cells_for_spec = cells_for_spec - _cells_in_raw_static_geometry(cells_for_spec)
            foreign_port_keepout_nonstatic_cells_by_instance.setdefault(
                instance_name,
                set(),
            ).update(nonstatic_cells_for_spec)
        _record_pipeline_timing("foreign_port_keepout_batch", t_foreign_keepout_start)

    dense_port_lateral_windows: dict[str, tuple[float, float, float, float, float]] = {}
    dense_port_lateral_owner_groups: dict[
        str,
        tuple[float, float, tuple[tuple[str, float], ...]],
    ] = {}
    for instance_name, port_specs in endpoint_port_specs_by_instance.items():
        if len(port_specs) <= 2:
            continue
        groups: dict[int, list[tuple[str, float]]] = {}
        for port_spec in port_specs:
            _inst, _port_name, port = endpoint_ports_by_spec[port_spec]
            fanout_anchor = fanout_anchor_by_port_spec.get(port_spec)
            angle = (
                int(fanout_anchor.physical_angle) % 8
                if fanout_anchor is not None
                else _orientation_to_angle(getattr(port, "orientation", None), flip=False)
            )
            step_x, step_y = _angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x
            center = fanout_anchor.center_um if fanout_anchor is not None else _port_center_um(port)
            if center is None or (lateral_x == 0 and lateral_y == 0):
                continue
            lateral_position = float(center[0]) * lateral_x + float(center[1]) * lateral_y
            groups.setdefault(angle, []).append((port_spec, lateral_position))
        for angle, group in groups.items():
            if len(group) <= 1:
                continue
            step_x, step_y = _angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x
            ordered = sorted(group, key=lambda item: item[1])
            owner_group = tuple(ordered)
            for owned_port_spec, _lateral_position in ordered:
                dense_port_lateral_owner_groups[owned_port_spec] = (
                    float(lateral_x),
                    float(lateral_y),
                    owner_group,
                )
            for index, (port_spec, lateral_position) in enumerate(ordered):
                previous_position = ordered[index - 1][1] if index > 0 else None
                next_position = ordered[index + 1][1] if index + 1 < len(ordered) else None
                if previous_position is None and next_position is None:
                    continue
                if previous_position is None:
                    gap = abs(next_position - lateral_position)
                    lower = lateral_position - gap * 0.5
                else:
                    lower = (previous_position + lateral_position) * 0.5
                if next_position is None:
                    gap = abs(lateral_position - previous_position)
                    upper = lateral_position + gap * 0.5
                else:
                    upper = (lateral_position + next_position) * 0.5
                lane_margin_um = 0.0
                dense_port_lateral_windows[port_spec] = (
                    float(lateral_x),
                    float(lateral_y),
                    float(lower),
                    float(upper),
                    lane_margin_um,
                )

    def _filter_dense_port_opening(
        port_spec: str,
        cells: set[tuple[int, int]],
    ) -> set[tuple[int, int]]:
        owner_group = dense_port_lateral_owner_groups.get(port_spec)
        if owner_group is not None and cells:
            lateral_x, lateral_y, owners = owner_group
            grid_size = float(grid.grid_size_um)
            filtered: set[tuple[int, int]] = set()
            for cell_x, cell_y in cells:
                center_x = origin_x_um + (float(cell_x) + 0.5) * grid_size
                center_y = origin_y_um + (float(cell_y) + 0.5) * grid_size
                lateral_position = center_x * lateral_x + center_y * lateral_y
                nearest_spec = min(
                    owners,
                    key=lambda item: (abs(lateral_position - item[1]), item[0]),
                )[0]
                if nearest_spec == port_spec:
                    filtered.add((cell_x, cell_y))
            return filtered

        window = dense_port_lateral_windows.get(port_spec)
        if window is None or not cells:
            return set(cells)
        lateral_x, lateral_y, lower, upper, lane_margin_um = window
        grid_size = float(grid.grid_size_um)
        lower -= lane_margin_um
        upper += lane_margin_um
        eps = max(1.0e-9, grid_size * 1.0e-9)
        filtered: set[tuple[int, int]] = set()
        for cell_x, cell_y in cells:
            center_x = origin_x_um + (float(cell_x) + 0.5) * grid_size
            center_y = origin_y_um + (float(cell_y) + 0.5) * grid_size
            lateral_position = center_x * lateral_x + center_y * lateral_y
            if lower - eps <= lateral_position <= upper + eps:
                filtered.add((cell_x, cell_y))
        return filtered

    def _opened_cells_for_spec(
        cells_by_spec: Mapping[str, set[tuple[int, int]]],
        port_spec: str,
    ) -> set[tuple[int, int]]:
        return _filter_dense_port_opening(
            port_spec,
            set(cells_by_spec.get(port_spec, set())),
        )

    normal_port_runway_cells: set[tuple[int, int]] = set()
    for cells in port_runway_cells_by_spec.values():
        normal_port_runway_cells.update(cells)

    def _foreign_keepout_open_cells_for_spec(port_spec: str) -> set[tuple[int, int]]:
        cluster_specs = dense_source_cluster_specs_by_port_spec.get(port_spec)
        if cluster_specs:
            cells: set[tuple[int, int]] = set()
            for cluster_port_spec in cluster_specs:
                cells.update(foreign_port_keepout_cells_by_spec.get(cluster_port_spec, set()))
            return cells - normal_port_runway_cells
        return (
            _opened_cells_for_spec(foreign_port_keepout_cells_by_spec, port_spec)
            - normal_port_runway_cells
        )

    def _endpoint_state_for_lane_assignment(port: Port, *, as_target: bool):
        return port_to_grid_state(
            port,
            origin_x_um,
            origin_y_um,
            float(grid.grid_size_um),
            as_target=as_target,
        )

    endpoint_ports_by_key: dict[tuple[int, int, int], list[tuple[str, bool, Port]]] = {}
    for job in route_jobs:
        for port_spec, port, as_target in (
            (f"{job.inst1},{job.port1}", job.source_port, False),
            (f"{job.inst2},{job.port2}", job.target_port, True),
        ):
            state = _endpoint_state_for_lane_assignment(port, as_target=as_target)
            key = (int(state.x), int(state.y), int(state.angle) % 8)
            endpoint_ports_by_key.setdefault(key, []).append((port_spec, as_target, port))

    port_state_lane_offsets: dict[tuple[str, bool], tuple[int, int]] = {}
    for (base_x, base_y, angle), endpoints in endpoint_ports_by_key.items():
        unique_endpoints = list(dict.fromkeys((spec, is_target) for spec, is_target, _ in endpoints))
        if len(unique_endpoints) <= 1:
            continue
        step_x, step_y = _angle_to_step(angle)
        lateral_x, lateral_y = -step_y, step_x
        if lateral_x == 0 and lateral_y == 0:
            continue

        def lateral_position(item: tuple[str, bool, Port]) -> float:
            center = _port_center_um(item[2])
            if center is None:
                return 0.0
            return center[0] * lateral_x + center[1] * lateral_y

        sorted_endpoints = sorted(endpoints, key=lateral_position)
        seen_endpoint_keys: set[tuple[str, bool]] = set()
        lane_index = 0
        for port_spec, is_target, _ in sorted_endpoints:
            endpoint_key = (port_spec, is_target)
            if endpoint_key in seen_endpoint_keys:
                continue
            seen_endpoint_keys.add(endpoint_key)
            candidate_x = base_x + lateral_x * lane_index
            candidate_y = base_y + lateral_y * lane_index
            if _in_bounds(candidate_x, candidate_y):
                port_state_lane_offsets[endpoint_key] = (
                    lateral_x * lane_index,
                    lateral_y * lane_index,
                )
            lane_index += 1

    def _dense_source_fanout_route_order(jobs: list[RouteJob]) -> list[RouteJob]:
        """Route consecutive dense source fanouts with inversion-aware extremes."""

        def should_reorder_source(instance_name: str) -> bool:
            return _is_dense_source_fanout_instance(instance_name)

        def order_single_run(run: list[RouteJob]) -> list[RouteJob]:
            if len(run) <= 1:
                return list(run)
            order_override = os.environ.get("PHOTONIC_ROUTER_DENSE_FANOUT_ORDER", "")
            if order_override in ("", "original"):
                return list(run)
            if order_override in ("inversion-aware-extremes", "legacy"):
                pass
            elif order_override not in (
                "target-ascending",
                "target-descending",
                "second-target-lane-first",
            ):
                return list(run)
            if order_override == "target-ascending":
                return sorted(
                    run,
                    key=lambda route_job: (
                        float(route_job.target_port.center[1]),
                        int(route_job.route_index),
                    ),
                )
            if order_override == "target-descending":
                return sorted(
                    run,
                    key=lambda route_job: (
                        -float(route_job.target_port.center[1]),
                        int(route_job.route_index),
                    ),
                )
            if order_override == "second-target-lane-first":
                by_target_lane = sorted(
                    run,
                    key=lambda route_job: (
                        float(route_job.target_port.center[1]),
                        int(route_job.route_index),
                    ),
                )
                first_job = by_target_lane[min(1, len(by_target_lane) - 1)]
                return [first_job, *(route_job for route_job in run if route_job != first_job)]
            target_lanes = [float(route_job.target_port.center[1]) for route_job in run]
            first_lane = float(run[0].target_port.center[1])
            first_lane_rank = sorted(target_lanes).index(first_lane)
            if first_lane_rank >= len(run) // 2:
                median_target_lane = sorted(target_lanes)[len(target_lanes) // 2]
                return sorted(
                    run,
                    key=lambda route_job: (
                        -abs(float(route_job.target_port.center[1]) - median_target_lane),
                        float(route_job.target_port.center[1]),
                        int(route_job.route_index),
                    ),
                )
            return [run[0], run[-1], *run[1:-1]]

        ordered_jobs: list[RouteJob] = []
        index = 0
        while index < len(jobs):
            job = jobs[index]
            if not should_reorder_source(job.inst1):
                ordered_jobs.append(job)
                index += 1
                continue

            run_end = index + 1
            while (
                run_end < len(jobs)
                and jobs[run_end].inst1 == job.inst1
                and should_reorder_source(jobs[run_end].inst1)
            ):
                run_end += 1

            ordered_jobs.extend(order_single_run(jobs[index:run_end]))
            index = run_end
        return ordered_jobs

    route_jobs = _dense_source_fanout_route_order(route_jobs)

    port_runway_static_cells: set[tuple[int, int]] = set()
    for cells in port_runway_cells_by_spec.values():
        port_runway_static_cells.update(cells)
    foreign_port_keepout_static_cells: set[tuple[int, int]] = set()
    for cells in foreign_port_keepout_cells_by_instance.values():
        foreign_port_keepout_static_cells.update(cells)
    debug_port_keepout_cells = set(port_runway_static_cells)
    debug_port_keepout_cells.update(foreign_port_keepout_static_cells)
    debug_port_keepout_cells.update(fanout_stub_static_cells)
    static_blocked_cells_before_port_reservations = set(raw_static_cells)
    static_blocked_cells_before_port_reservations.update(port_runway_static_cells)
    static_blocked_cells_before_port_reservations.update(foreign_port_keepout_static_cells)
    static_blocked_cells_before_port_reservations.update(fanout_stub_static_cells)

    t_static_handoff_start = _pipeline_timer_start()
    blocked_static_rects_for_diagnostics: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "blocked_static_rects"):
        blocked_static_rects: list[tuple[int, int, int, int]] = []
        raw_blocked_rects = cast(
            Iterable[tuple[int, int, int, int]], getattr(obstacle_map, "blocked_static_rects")
        )
        for rect in raw_blocked_rects:
            if len(rect) != 4:
                continue
            blocked_static_rects.append(
                (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
            )
        blocked_static_rects_for_diagnostics = list(blocked_static_rects)
        if blocked_static_rects:
            if not hasattr(router, "set_static_rects"):
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.set_static_rects. Rebuild it with "
                    "`maturin develop --release`; otherwise bounding_boxes mode "
                    "cannot use compact static rectangles."
                )
            router.set_static_rects(blocked_static_rects)
        else:
            router.set_static_cells(sorted(static_blocked_cells_before_port_reservations))
    else:
        static_cells = set(static_blocked_cells_before_port_reservations)
        sorted_static_cells = sorted(static_cells)
        router.set_static_cells(sorted_static_cells)
    if port_runway_static_cells or foreign_port_keepout_static_cells or fanout_stub_static_cells:
        if not hasattr(router, "add_static_cells"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.add_static_cells. Rebuild it with "
                "`maturin develop --release`."
            )
        router.add_static_cells(
            sorted(
                port_runway_static_cells
                | foreign_port_keepout_static_cells
                | fanout_stub_static_cells
            )
        )
    _record_pipeline_timing("static_map_handoff", t_static_handoff_start)

    full_route_jobs = list(route_jobs)
    full_route_jobs_by_route_index = {int(job.route_index): job for job in full_route_jobs}
    full_route_count = len(full_route_jobs)
    if (
        debug_stop_after_route_index is not None
        and int(debug_stop_after_route_index) > full_route_count
    ):
        raise ValueError(
            "debug_stop_after_route_index exceeds route count "
            f"({debug_stop_after_route_index} > {full_route_count})"
        )
    if debug_stop_after_route_index is not None:
        stop_index = int(debug_stop_after_route_index)
        route_jobs = [job for job in full_route_jobs if int(job.route_index) <= stop_index]
        if verbose_route_diagnostics or debug_route_indices is not None:
            print(
                f"  Debug stop-after-route active: routing {len(route_jobs)} "
                f"of {full_route_count} full-context routes"
            )
    debug_execution_limit_raw = os.environ.get("PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT")
    if debug_execution_limit_raw:
        try:
            debug_execution_limit = int(debug_execution_limit_raw)
        except ValueError as exc:
            raise ValueError(
                "PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT must be an integer"
            ) from exc
        if debug_execution_limit < 1:
            raise ValueError("PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT must be >= 1")
        original_route_job_count = len(route_jobs)
        route_jobs = route_jobs[:debug_execution_limit]
        if verbose_route_diagnostics or debug_route_indices is not None:
            print(
                "  Debug execution limit active: routing "
                f"{len(route_jobs)} of {original_route_job_count} selected "
                "routes in actual execution order"
            )

    repair_config = ripup_reroute_config or RipupRerouteConfig()
    route_jobs_by_id = {job.net_id: job for job in route_jobs}
    route_order = [job.net_id for job in route_jobs]
    collect_timing = debug_timing or collect_route_stats or collect_attempt_diagnostics
    track_dynamic_cells = diagnostics_enabled
    route_bookkeeping = RouteBookkeeping(
        route_order=route_order,
        diagnostics_enabled=track_dynamic_cells,
    )

    t_astar_start = 0.0
    if collect_timing:
        t_astar_start = time.perf_counter()
    total_expanded_states = 0
    simple_route_count = 0
    repair_count = 0
    route_attempt_records = []
    native_repair_trace_records: list[dict[str, object]] = []
    route_timing_buckets: dict[str, RouteTimingBucket] = {
        name: RouteTimingBucket()
        for name in (
            "normal_route",
            "probe_route",
            "preemptive_crossing_ripup",
            "guided_collision_crossing",
            "localized_crossing_keepout",
            "repair_failed_net",
            "reroute_victims",
            "lidar_pure_probe_commit",
            "endpoint_correction",
        )
    }

    def _timing_start() -> float:
        return time.perf_counter() if collect_timing else 0.0

    def _record_elapsed(bucket_name: str, start_s: float, *, failed: bool = False) -> None:
        if not collect_timing:
            return
        route_timing_buckets[bucket_name].record_elapsed(
            time.perf_counter() - start_s,
            failed=failed,
        )

    def _record_native_batch_timings(batch_result: dict[str, Any]) -> None:
        if not collect_pipeline_timing:
            return
        raw_timings = batch_result.get("timings_s")
        if raw_timings is None:
            return
        for raw_name, raw_elapsed_s in dict(raw_timings).items():
            try:
                elapsed_s = float(raw_elapsed_s)
            except (TypeError, ValueError):
                continue
            name = f"native_batch_{raw_name}"
            route_nets_timings_s[name] = route_nets_timings_s.get(name, 0.0) + elapsed_s

    def _committed_dynamic_cells(*, exclude_net_id: int | None = None) -> set[tuple[int, int]]:
        return route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)

    def _committed_dynamic_cells_for_attempt(
        *,
        exclude_net_id: int | None = None,
    ) -> set[tuple[int, int]]:
        if route_bookkeeping.diagnostics_enabled:
            return route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)
        merged: set[tuple[int, int]] = set()
        for net_id in route_bookkeeping.records_by_id:
            if exclude_net_id is not None and int(net_id) == int(exclude_net_id):
                continue
            merged.update(_route_cells_from_router(net_id))
        return merged

    def _route_cells_from_router(net_id: int) -> set[tuple[int, int]]:
        return {
            (int(cell[0]), int(cell[1]))
            for cell in router.get_net_cells(int(net_id))
        }

    def _append_centerline_points(
        out: list[tuple[float, float]],
        points: Iterable[tuple[float, float]],
    ) -> None:
        for raw_x, raw_y in points:
            point = (float(raw_x), float(raw_y))
            if out:
                last_x, last_y = out[-1]
                if math.hypot(point[0] - last_x, point[1] - last_y) <= 1.0e-9:
                    continue
            out.append(point)

    def _fanout_stubbed_centerline(
        job: RouteJob,
        route_obj: Any,
    ) -> tuple[tuple[float, float], ...]:
        source_anchor = fanout_anchor_by_port_spec.get(f"{job.inst1},{job.port1}")
        target_anchor = fanout_anchor_by_port_spec.get(f"{job.inst2},{job.port2}")
        if source_anchor is None and target_anchor is None:
            return ()
        route_primitive_centerline = getattr(router, "route_primitive_centerline", None)
        try:
            if route_primitive_centerline is not None:
                route_centerline = _centerline_tuple(
                    route_primitive_centerline(route_obj)
                )
            else:
                route_centerline = ()
        except Exception:
            route_centerline = ()
        if len(route_centerline) < 2:
            return ()
        points: list[tuple[float, float]] = []
        if source_anchor is not None:
            _append_centerline_points(points, source_anchor.stub_centerline_um)
        else:
            _append_centerline_points(points, route_centerline[:1])
        _append_centerline_points(points, route_centerline)
        if target_anchor is not None:
            _append_centerline_points(points, reversed(target_anchor.stub_centerline_um))
        else:
            _append_centerline_points(points, route_centerline[-1:])
        centerline = _compress_centerline(tuple(points))
        return centerline if len(centerline) >= 2 else ()

    def _endpoint_bump_candidate_open_cells_for_state(
        state: Any,
    ) -> set[tuple[int, int]]:
        """Cells a local endpoint bump may need opened against its own port pad."""
        base_x = int(state.x)
        base_y = int(state.y)
        step_x, step_y = _angle_to_step(int(state.angle) % 8)
        side_steps = ((-step_y, step_x), (step_y, -step_x))
        reach = max(1, int(bend_radius_cells))
        axis_reach = 4 * reach
        lateral_reach = 2 * reach
        cells: set[tuple[int, int]] = set()
        for forward in range(-axis_reach, reach + 1):
            if forward == 0:
                continue
            x = base_x + step_x * forward
            y = base_y + step_y * forward
            if 0 <= x < int(grid.width) and 0 <= y < int(grid.height):
                cells.add((x, y))
        for side_x, side_y in side_steps:
            for lateral in range(1, lateral_reach + 1):
                for forward in range(-axis_reach, reach + 1):
                    if forward == 0:
                        continue
                    x = base_x + step_x * forward + side_x * lateral
                    y = base_y + step_y * forward + side_y * lateral
                    if 0 <= x < int(grid.width) and 0 <= y < int(grid.height):
                        cells.add((x, y))
        return cells

    def _states_and_openings(
        job: RouteJob,
    ) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_fanout_anchor = fanout_anchor_by_port_spec.get(port1_spec)
        target_fanout_anchor = fanout_anchor_by_port_spec.get(port2_spec)
        if source_fanout_anchor is None:
            source_state = port_to_grid_state(
                job.source_port,
                origin_x_um,
                origin_y_um,
                float(grid.grid_size_um),
                as_target=False,
            )
        else:
            source_state = rust_backend.State(
                int(source_fanout_anchor.state_x),
                int(source_fanout_anchor.state_y),
                int(source_fanout_anchor.physical_angle) % 8,
            )
        if target_fanout_anchor is None:
            target_state = port_to_grid_state(
                job.target_port,
                origin_x_um,
                origin_y_um,
                float(grid.grid_size_um),
                as_target=True,
            )
        else:
            target_state = rust_backend.State(
                int(target_fanout_anchor.state_x),
                int(target_fanout_anchor.state_y),
                (int(target_fanout_anchor.physical_angle) + 4) % 8,
            )
        source_lane_offset = port_state_lane_offsets.get((f"{job.inst1},{job.port1}", False))
        if source_lane_offset is not None and source_fanout_anchor is None:
            source_state = rust_backend.State(
                int(source_state.x) + int(source_lane_offset[0]),
                int(source_state.y) + int(source_lane_offset[1]),
                int(source_state.angle),
            )
        target_lane_offset = port_state_lane_offsets.get((f"{job.inst2},{job.port2}", True))
        if target_lane_offset is not None and target_fanout_anchor is None:
            target_state = rust_backend.State(
                int(target_state.x) + int(target_lane_offset[0]),
                int(target_state.y) + int(target_lane_offset[1]),
                int(target_state.angle),
            )
        if source_fanout_anchor is None and target_fanout_anchor is None:
            source_state, target_state, original_anchor_cells = _snap_nearly_collinear_states(
                source_state,
                target_state,
                job.source_port,
                job.target_port,
            )
            source_state, target_state, snapped_anchor_cells = (
                _snap_same_heading_minimum_bend_offset(source_state, target_state)
            )
            original_anchor_cells.update(snapped_anchor_cells)
        else:
            original_anchor_cells = {
                (int(source_state.x), int(source_state.y)),
                (int(target_state.x), int(target_state.y)),
            }
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        endpoint_foreign_keepout_open_cells = set(
            _foreign_keepout_open_cells_for_spec(port1_spec)
        )
        endpoint_foreign_keepout_open_cells.update(
            _foreign_keepout_open_cells_for_spec(port2_spec)
        )
        opened_candidate_cells = set(
            _opened_cells_for_spec(port_access_candidate_cells_by_spec, port1_spec)
        )
        opened_candidate_cells.update(
            _opened_cells_for_spec(port_access_candidate_cells_by_spec, port2_spec)
        )
        opened_candidate_cells.update(endpoint_foreign_keepout_open_cells)
        opened_candidate_cells.update(original_anchor_cells)
        opened_candidate_cells.update({source_anchor_cell, target_anchor_cell})

        opened_cells_set = set(_opened_cells_for_spec(port_access_cells_by_spec, port1_spec))
        opened_cells_set.update(_opened_cells_for_spec(port_access_cells_by_spec, port2_spec))
        opened_cells_set.update(endpoint_foreign_keepout_open_cells)
        opened_cells_set.update(original_anchor_cells)
        opened_cells_set.update({source_anchor_cell, target_anchor_cell})
        if fanout_stub_static_cells:
            current_fanout_stub_open_cells: set[tuple[int, int]] = set()
            current_fanout_stub_open_cells.update(
                fanout_stub_static_cells_by_spec.get(port1_spec, set())
            )
            current_fanout_stub_open_cells.update(
                fanout_stub_static_cells_by_spec.get(port2_spec, set())
            )
            allowed_fanout_stub_open_cells = (
                current_fanout_stub_open_cells
                | original_anchor_cells
                | {source_anchor_cell, target_anchor_cell}
            )
            foreign_fanout_stub_static_cells = (
                fanout_stub_static_cells - allowed_fanout_stub_open_cells
            )
            opened_candidate_cells.difference_update(foreign_fanout_stub_static_cells)
            opened_cells_set.difference_update(foreign_fanout_stub_static_cells)
        source_endpoint_bump_open_cells = _endpoint_bump_candidate_open_cells_for_state(
            source_state
        )
        target_endpoint_bump_open_cells = _endpoint_bump_candidate_open_cells_for_state(
            target_state
        )
        opened_candidate_cells.update(source_endpoint_bump_open_cells)
        opened_candidate_cells.update(target_endpoint_bump_open_cells)
        trace_endpoint_bumps = os.environ.get("PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS", "")
        if trace_endpoint_bumps and (
            trace_endpoint_bumps.strip() == "*"
            or str(int(job.net_id))
            in {item.strip() for item in trace_endpoint_bumps.split(",")}
        ):
            print(
                "endpoint_open_trace "
                f"net_id={int(job.net_id)} "
                f"source_state=({int(source_state.x)},{int(source_state.y)},{int(source_state.angle)}) "
                f"target_state=({int(target_state.x)},{int(target_state.y)},{int(target_state.angle)}) "
                f"source_bump_open={len(source_endpoint_bump_open_cells)} "
                f"target_bump_open={len(target_endpoint_bump_open_cells)} "
                f"opened_candidate={len(opened_candidate_cells)} "
                f"has_112_243={(112, 243) in opened_candidate_cells}"
            )
        return (
            source_state,
            target_state,
            opened_candidate_cells,
            opened_cells_set,
            sorted(opened_cells_set),
        )

    def _routing_endpoint_center_um(
        job: RouteJob,
        *,
        source: bool,
    ) -> tuple[float, float] | None:
        if source:
            port_spec = f"{job.inst1},{job.port1}"
            anchor = fanout_anchor_by_port_spec.get(port_spec)
            return anchor.center_um if anchor is not None else _port_center_um(job.source_port)
        port_spec = f"{job.inst2},{job.port2}"
        anchor = fanout_anchor_by_port_spec.get(port_spec)
        return anchor.center_um if anchor is not None else _port_center_um(job.target_port)

    if not hasattr(router, "build_dynamic_clearance_exempt_cells_for_routes"):
        extension_path = getattr(rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.build_dynamic_clearance_exempt_cells_for_routes. "
            "Rebuild it with `maturin develop --release`. "
            f"Loaded extension: {extension_path}"
        )

    t_state_opening_precompute_start = _pipeline_timer_start()
    route_state_openings_by_id = {
        int(job.net_id): _states_and_openings(job)
        for job in route_jobs
    }
    _record_pipeline_timing(
        "state_opening_precompute",
        t_state_opening_precompute_start,
    )
    clearance_exempt_inputs = [
        (int(net_id), state_openings[0], state_openings[1])
        for net_id, state_openings in route_state_openings_by_id.items()
    ]
    t_clearance_exempt_batch_start = _pipeline_timer_start()
    batch_clearance_exempt_cells_by_id = {
        int(net_id): [(int(cell[0]), int(cell[1])) for cell in cells]
        for net_id, cells in router.build_dynamic_clearance_exempt_cells_for_routes(
            clearance_exempt_inputs,
            bool(allow_45_degree_turns),
            int(bend_radius_cells),
            int(commit_radius_cells),
        )
    }
    _record_pipeline_timing(
        "clearance_exempt_batch",
        t_clearance_exempt_batch_start,
    )

    def _state_openings_for_job(
        job: RouteJob,
    ) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
        return route_state_openings_by_id[int(job.net_id)]

    def _clearance_exempt_cells_for_job(job: RouteJob) -> list[tuple[int, int]]:
        return batch_clearance_exempt_cells_by_id.get(int(job.net_id), [])

    def _clearance_exempt_cell_set_for_job(job: RouteJob) -> set[tuple[int, int]]:
        return set(_clearance_exempt_cells_for_job(job))

    realization_grid_spec = (
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )

    def _static_cells_in_rect(min_x: int, max_x: int, min_y: int, max_y: int) -> int:
        if min_x > max_x or min_y > max_y:
            return 0
        if blocked_static_rects_for_diagnostics:
            return sum(
                _rect_overlap_cell_count(
                    rect,
                    min_x=min_x,
                    max_x=max_x,
                    min_y=min_y,
                    max_y=max_y,
                )
                for rect in blocked_static_rects_for_diagnostics
            )
        return sum(
            1
            for x, y in static_blocked_cells_before_port_reservations
            if min_x <= x <= max_x and min_y <= y <= max_y
        )

    def _cells_in_rect(
        cells: set[tuple[int, int]],
        *,
        min_x: int,
        max_x: int,
        min_y: int,
        max_y: int,
    ) -> int:
        if min_x > max_x or min_y > max_y:
            return 0
        return sum(1 for x, y in cells if min_x <= x <= max_x and min_y <= y <= max_y)

    def _route_attempt_diagnostics(
        job: RouteJob,
        route_obj: object | None,
        *,
        candidate_blockers: list[int] | None = None,
        ripup_ids: list[int] | None = None,
    ) -> dict[str, object]:
        dynamic_cells_before = _committed_dynamic_cells_for_attempt(exclude_net_id=job.net_id)
        source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
        source_x = int(source_state.x)
        source_y = int(source_state.y)
        source_angle = int(source_state.angle)
        target_x = int(target_state.x)
        target_y = int(target_state.y)
        target_angle = int(target_state.angle)
        span_x = abs(target_x - source_x)
        span_y = abs(target_y - source_y)
        span_bbox_min_x = min(source_x, target_x)
        span_bbox_max_x = max(source_x, target_x)
        span_bbox_min_y = min(source_y, target_y)
        span_bbox_max_y = max(source_y, target_y)
        span_bbox_area = _rect_cell_count(
            min_x=span_bbox_min_x,
            max_x=span_bbox_max_x,
            min_y=span_bbox_min_y,
            max_y=span_bbox_max_y,
        )
        window_min_x = int(getattr(route_obj, "last_window_min_x", 0)) if route_obj else 0
        window_max_x = int(getattr(route_obj, "last_window_max_x", -1)) if route_obj else -1
        window_min_y = int(getattr(route_obj, "last_window_min_y", 0)) if route_obj else 0
        window_max_y = int(getattr(route_obj, "last_window_max_y", -1)) if route_obj else -1
        window_area = int(getattr(route_obj, "last_window_area_cells", 0)) if route_obj else 0
        if window_area <= 0:
            window_area = _rect_cell_count(
                min_x=window_min_x,
                max_x=window_max_x,
                min_y=window_min_y,
                max_y=window_max_y,
            )
        window_static_cells = _static_cells_in_rect(
            window_min_x,
            window_max_x,
            window_min_y,
            window_max_y,
        )
        window_dynamic_cells = _cells_in_rect(
            dynamic_cells_before,
            min_x=window_min_x,
            max_x=window_max_x,
            min_y=window_min_y,
            max_y=window_max_y,
        )
        span_static_cells = _static_cells_in_rect(
            span_bbox_min_x,
            span_bbox_max_x,
            span_bbox_min_y,
            span_bbox_max_y,
        )
        span_dynamic_cells = _cells_in_rect(
            dynamic_cells_before,
            min_x=span_bbox_min_x,
            max_x=span_bbox_max_x,
            min_y=span_bbox_min_y,
            max_y=span_bbox_max_y,
        )
        route_cells = getattr(route_obj, "cells", None) if route_obj is not None else None
        route_bbox = _route_cells_bbox(route_cells or ())
        if route_bbox is None:
            route_bbox_min_x = 0
            route_bbox_max_x = -1
            route_bbox_min_y = 0
            route_bbox_max_y = -1
        else:
            route_bbox_min_x, route_bbox_max_x, route_bbox_min_y, route_bbox_max_y = route_bbox
        route_bbox_area = _rect_cell_count(
            min_x=route_bbox_min_x,
            max_x=route_bbox_max_x,
            min_y=route_bbox_min_y,
            max_y=route_bbox_max_y,
        )
        total_cost = (
            float(getattr(route_obj, "total_cost", 0.0))
            if route_obj is not None
            else None
        )
        euclidean_lower_bound, heading_lower_bound = _source_lower_bounds(
            source_x=source_x,
            source_y=source_y,
            source_angle=source_angle,
            target_x=target_x,
            target_y=target_y,
            target_angle=target_angle,
        )
        blocker_ids = list(candidate_blockers or [])
        victim_ids = list(ripup_ids or [])
        raw_dynamic_cells: set[tuple[int, int]] = set()
        raw_dynamic_refcount_gt1_cells: set[tuple[int, int]] = set()
        raw_core_cells: set[tuple[int, int]] = set()
        raw_core_refcount_gt1_cells: set[tuple[int, int]] = set()
        raw_net_route_cells: set[tuple[int, int]] = set()
        if hasattr(router, "raw_dynamic_obstacle_cells"):
            raw_dynamic_entries = [
                (int(x), int(y), int(refs))
                for x, y, refs in router.raw_dynamic_obstacle_cells()
            ]
            raw_dynamic_cells = {(x, y) for x, y, _ in raw_dynamic_entries}
            raw_dynamic_refcount_gt1_cells = {
                (x, y) for x, y, refs in raw_dynamic_entries if refs > 1
            }
        if hasattr(router, "raw_dynamic_core_cells"):
            raw_core_entries = [
                (int(x), int(y), int(refs))
                for x, y, refs in router.raw_dynamic_core_cells()
            ]
            raw_core_cells = {(x, y) for x, y, _ in raw_core_entries}
            raw_core_refcount_gt1_cells = {
                (x, y) for x, y, refs in raw_core_entries if refs > 1
            }
        if hasattr(router, "all_net_route_cells"):
            for _, cells in router.all_net_route_cells():
                raw_net_route_cells.update(
                    (int(cell[0]), int(cell[1])) for cell in cells
                )
        raw_dynamic_without_owner = raw_dynamic_cells - raw_net_route_cells
        raw_net_route_without_dynamic = raw_net_route_cells - raw_dynamic_cells
        raw_core_without_dynamic = raw_core_cells - raw_dynamic_cells
        raw_dynamic_span_cells = {
            (x, y)
            for x, y in raw_dynamic_cells
            if span_bbox_min_x <= x <= span_bbox_max_x
            and span_bbox_min_y <= y <= span_bbox_max_y
        }
        raw_dynamic_without_owner_span_cells = {
            (x, y)
            for x, y in raw_dynamic_without_owner
            if span_bbox_min_x <= x <= span_bbox_max_x
            and span_bbox_min_y <= y <= span_bbox_max_y
        }
        return {
            "source_state": [source_x, source_y, source_angle],
            "target_state": [target_x, target_y, target_angle],
            "span_x_cells": span_x,
            "span_y_cells": span_y,
            "span_manhattan_cells": span_x + span_y,
            "span_bbox_area_cells": span_bbox_area,
            "span_static_cells": span_static_cells,
            "span_dynamic_cells": span_dynamic_cells,
            "route_bbox_min_x": route_bbox_min_x,
            "route_bbox_max_x": route_bbox_max_x,
            "route_bbox_min_y": route_bbox_min_y,
            "route_bbox_max_y": route_bbox_max_y,
            "route_bbox_width_cells": max(0, route_bbox_max_x - route_bbox_min_x + 1),
            "route_bbox_height_cells": max(0, route_bbox_max_y - route_bbox_min_y + 1),
            "route_bbox_area_cells": route_bbox_area,
            "route_bbox_to_span_bbox_area": (
                float(route_bbox_area) / float(span_bbox_area)
                if span_bbox_area > 0 and route_bbox_area > 0
                else None
            ),
            "opened_cells_count": len(opened_cells),
            "block_radius_cells": block_radius_cells,
            "dynamic_obstacle_search_expansion_radius_cells": (
                clearance_policy.dynamic_obstacle_search_expansion_radius_cells
            ),
            "dynamic_route_commit_keepout_radius_cells": (
                clearance_policy.dynamic_route_commit_keepout_radius_cells
            ),
            "dynamic_route_core_radius_cells": (
                clearance_policy.dynamic_route_core_radius_cells
            ),
            "bend_radius_cells": bend_radius_cells,
            "window_width_cells": max(0, window_max_x - window_min_x + 1),
            "window_height_cells": max(0, window_max_y - window_min_y + 1),
            "window_area_cells": window_area,
            "window_to_span_bbox_area": (
                float(window_area) / float(span_bbox_area)
                if span_bbox_area > 0 and window_area > 0
                else None
            ),
            "route_bbox_to_window_area": (
                float(route_bbox_area) / float(window_area)
                if window_area > 0 and route_bbox_area > 0
                else None
            ),
            "window_static_cells": window_static_cells,
            "window_dynamic_cells": window_dynamic_cells,
            "window_static_density": (
                float(window_static_cells) / float(window_area)
                if window_area > 0
                else None
            ),
            "window_dynamic_density": (
                float(window_dynamic_cells) / float(window_area)
                if window_area > 0
                else None
            ),
            "total_cost": total_cost,
            "euclidean_lower_bound_cost": euclidean_lower_bound,
            "heading_lower_bound_cost": heading_lower_bound,
            "euclidean_lower_bound_to_cost": (
                euclidean_lower_bound / total_cost
                if total_cost is not None and total_cost > 0.0
                else None
            ),
            "heading_lower_bound_to_cost": (
                heading_lower_bound / total_cost
                if total_cost is not None and total_cost > 0.0
                else None
            ),
            "heading_lower_bound_gap_cost": (
                total_cost - heading_lower_bound
                if total_cost is not None
                else None
            ),
            "committed_dynamic_cells_before": len(dynamic_cells_before),
            "raw_dynamic_obstacle_cells_before": len(raw_dynamic_cells),
            "raw_dynamic_core_cells_before": len(raw_core_cells),
            "raw_net_route_cells_before": len(raw_net_route_cells),
            "raw_dynamic_refcount_gt1_count": len(raw_dynamic_refcount_gt1_cells),
            "raw_dynamic_refcount_gt1_bbox": _cells_bbox(raw_dynamic_refcount_gt1_cells),
            "raw_core_refcount_gt1_count": len(raw_core_refcount_gt1_cells),
            "raw_dynamic_without_owner_count": len(raw_dynamic_without_owner),
            "raw_dynamic_without_owner_bbox": _cells_bbox(raw_dynamic_without_owner),
            "raw_dynamic_without_owner_sample": sorted(raw_dynamic_without_owner)[:12],
            "raw_net_route_without_dynamic_count": len(raw_net_route_without_dynamic),
            "raw_net_route_without_dynamic_bbox": _cells_bbox(raw_net_route_without_dynamic),
            "raw_core_without_dynamic_count": len(raw_core_without_dynamic),
            "span_raw_dynamic_cells": len(raw_dynamic_span_cells),
            "span_raw_dynamic_without_owner_count": len(raw_dynamic_without_owner_span_cells),
            "span_raw_dynamic_without_owner_bbox": _cells_bbox(
                raw_dynamic_without_owner_span_cells
            ),
            "candidate_blocker_count": len(blocker_ids),
            "candidate_blocker_net_ids": blocker_ids,
            "candidate_blocker_route_indices": [
                route_jobs_by_id[net_id].route_index
                for net_id in blocker_ids
                if net_id in route_jobs_by_id
            ],
            "ripup_victim_count": len(victim_ids),
            "ripup_victim_net_ids": victim_ids,
            "ripup_victim_route_indices": [
                route_jobs_by_id[net_id].route_index
                for net_id in victim_ids
                if net_id in route_jobs_by_id
            ],
        }

    def _write_route_diagnostics(
        *,
        job: RouteJob,
        source_state: Any,
        target_state: Any,
        opened_candidate_cells: set[tuple[int, int]],
        dynamic_clearance_exempt_cells: set[tuple[int, int]],
        opened_cells_set: set[tuple[int, int]],
        diag_txt: Path | None,
        status: str,
        error_text: str | None = None,
        route_cells: set[tuple[int, int]] | None = None,
        route_obj: Any | None = None,
        repair_note: str | None = None,
    ) -> None:
        if diag_txt is None:
            return
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        committed_dynamic_cells = _committed_dynamic_cells(exclude_net_id=job.net_id)
        if diagnostics_enabled:
            opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
            opened_candidate_static_overlap = _cells_in_raw_static_geometry(
                opened_candidate_cells
            )
            opened_static_overlap = _cells_in_raw_static_geometry(opened_cells_set)
            opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
            dynamic_exempt_dynamic_overlap = (
                dynamic_clearance_exempt_cells & committed_dynamic_cells
            )
        else:
            opened_candidate_dynamic_overlap = set()
            opened_candidate_static_overlap = set()
            opened_static_overlap = set()
            opened_dynamic_overlap = set()
            dynamic_exempt_dynamic_overlap = set()

        route_cells = route_cells or set()
        route_static_overlap = _cells_in_raw_static_geometry(route_cells)
        route_overlap_with_candidate_opened_static = (
            route_cells & opened_candidate_static_overlap
        )
        route_overlap_with_effective_opened_static = route_cells & opened_static_overlap
        route_dynamic_overlap = route_cells & committed_dynamic_cells
        route_overlap_with_candidate_opened_dynamic = (
            route_cells & opened_candidate_dynamic_overlap
        )
        route_overlap_with_effective_opened_dynamic = route_cells & opened_dynamic_overlap
        route_overlap_with_dynamic_exempt = route_cells & dynamic_clearance_exempt_cells
        current_endpoint_foreign_keepout_cells = set(
            _foreign_keepout_open_cells_for_spec(port1_spec)
        )
        current_endpoint_foreign_keepout_cells.update(
            _foreign_keepout_open_cells_for_spec(port2_spec)
        )
        foreign_keepout_open_cells = current_endpoint_foreign_keepout_cells & opened_cells_set
        current_port_runway_cells = set(port_runway_cells_by_spec.get(port1_spec, set()))
        current_port_runway_cells.update(port_runway_cells_by_spec.get(port2_spec, set()))
        source_sibling_port_runway_cells: set[tuple[int, int]] = set()
        for cluster_port_spec in dense_source_cluster_specs_by_port_spec.get(port1_spec, set()):
            if cluster_port_spec == port1_spec:
                continue
            source_sibling_port_runway_cells.update(
                port_runway_cells_by_spec.get(cluster_port_spec, set())
            )
        target_sibling_port_runway_cells: set[tuple[int, int]] = set()
        for cluster_port_spec in dense_source_cluster_specs_by_port_spec.get(port2_spec, set()):
            if cluster_port_spec == port2_spec:
                continue
            target_sibling_port_runway_cells.update(
                port_runway_cells_by_spec.get(cluster_port_spec, set())
            )
        sibling_port_runway_cells = (
            source_sibling_port_runway_cells | target_sibling_port_runway_cells
        )
        current_port_runway_dynamic_overlap = (
            current_port_runway_cells & committed_dynamic_cells
        )
        sibling_port_runway_dynamic_overlap = (
            sibling_port_runway_cells & committed_dynamic_cells
        )
        route_overlap_current_port_runway = route_cells & current_port_runway_cells
        route_overlap_sibling_port_runway = route_cells & sibling_port_runway_cells
        route_segments: list[str] = []
        if route_obj is not None:
            for segment in cast(list[object], getattr(route_obj, "segments", []) or []):
                try:
                    entry = dict(cast(Any, segment))
                except (TypeError, ValueError):
                    continue
                route_segments.append(
                    "{kind}:{start}->{end}@{start_angle}->{end_angle}".format(
                        kind=entry.get("kind"),
                        start=entry.get("start"),
                        end=entry.get("end"),
                        start_angle=entry.get("start_angle"),
                        end_angle=entry.get("end_angle"),
                    )
                )

        def _relative_line_cells(
            *,
            start: tuple[int, int],
            direction: tuple[int, int],
            cells: int,
        ) -> list[tuple[int, int]]:
            return [
                (int(start[0]) + int(direction[0]) * step,
                 int(start[1]) + int(direction[1]) * step)
                for step in range(max(0, int(cells)) + 1)
            ]

        def _unique_cells(
            cells: Iterable[tuple[int, int]],
        ) -> list[tuple[int, int]]:
            seen: set[tuple[int, int]] = set()
            unique: list[tuple[int, int]] = []
            for cell in cells:
                normalized = (int(cell[0]), int(cell[1]))
                if normalized in seen:
                    continue
                seen.add(normalized)
                unique.append(normalized)
            return unique

        def _first_move_footprint(
            *,
            source: tuple[int, int],
            source_angle: int,
            kind: str,
            cells: int = 0,
            delta: int = 0,
        ) -> tuple[tuple[int, int], int, list[tuple[int, int]]]:
            start_dir = _angle_to_step(source_angle)
            if kind == "straight":
                relative = _relative_line_cells(
                    start=(0, 0),
                    direction=start_dir,
                    cells=cells,
                )
                end = relative[-1]
                end_angle = int(source_angle) % 8
            else:
                end_angle = (int(source_angle) + int(delta)) % 8
                end_dir = _angle_to_step(end_angle)
                radius = max(0, int(bend_radius_cells))
                first_leg = _relative_line_cells(
                    start=(0, 0),
                    direction=start_dir,
                    cells=radius,
                )
                corner = (start_dir[0] * radius, start_dir[1] * radius)
                second_leg = _relative_line_cells(
                    start=corner,
                    direction=end_dir,
                    cells=radius,
                )
                relative = _unique_cells([*first_leg, *second_leg])
                end = relative[-1]
            absolute = [
                (int(source[0]) + int(dx), int(source[1]) + int(dy))
                for dx, dy in relative
            ]
            return (
                (int(source[0]) + int(end[0]), int(source[1]) + int(end[1])),
                end_angle,
                absolute,
            )

        def _dynamic_owners_for_cells(
            cells: set[tuple[int, int]],
        ) -> dict[int, list[tuple[int, int]]]:
            owners: dict[int, list[tuple[int, int]]] = {}
            for net_id in route_bookkeeping.records_by_id:
                if int(net_id) == int(job.net_id):
                    continue
                overlap = cells & _route_cells_from_router(int(net_id))
                if overlap:
                    owners[int(net_id)] = sorted(overlap)
            return owners

        def _format_first_move_debug() -> list[str]:
            if not diagnostics_enabled:
                return []
            source = source_anchor_cell
            source_angle = int(source_state.angle)
            source_key = source_anchor_cell
            target_key = target_anchor_cell
            opened_search_cells = {
                cell
                for cell in opened_cells_set
                if cell == source_key
                or cell == target_key
                or cell not in committed_dynamic_cells
            }
            routing_static_cells = set(static_blocked_cells_before_port_reservations)
            routing_static_cells.update(debug_port_keepout_cells)
            routing_static_cells.update(foreign_port_keepout_static_cells)
            routing_static_cells.update(fanout_stub_static_cells)
            primitive_specs: list[tuple[str, str, int, int]] = [
                ("straight_short", "straight", int(primitive_cfg.straight_short_cells), 0),
                ("straight_long", "straight", int(primitive_cfg.straight_long_cells), 0),
                ("turn45_left", "turn", 0, 1),
                ("turn45_right", "turn", 0, -1),
                ("turn90_left", "turn", 0, 2),
                ("turn90_right", "turn", 0, -2),
            ]
            debug_lines: list[str] = []
            for label, kind, cells, delta in primitive_specs:
                if not allow_45_degree_turns and abs(int(delta)) == 1:
                    continue
                end_cell, end_angle, footprint = _first_move_footprint(
                    source=source,
                    source_angle=source_angle,
                    kind=kind,
                    cells=cells,
                    delta=delta,
                )
                footprint_set = set(footprint)
                static_overlap = _cells_in_raw_static_geometry(footprint_set)
                routing_static_overlap = footprint_set & routing_static_cells
                effective_static_blockers = routing_static_overlap - opened_search_cells
                dynamic_overlap = footprint_set & committed_dynamic_cells
                effective_dynamic_blockers = (
                    dynamic_overlap
                    - dynamic_clearance_exempt_cells
                    - (footprint_set & opened_search_cells)
                )
                owner_cells = _dynamic_owners_for_cells(dynamic_overlap)
                debug_lines.append(
                    "first_move_{label}="
                    "end=({end_x},{end_y},{end_angle}); "
                    "footprint={footprint}; "
                    "static={static}; "
                    "routing_static={routing_static}; "
                    "static_blockers={static_blockers}; "
                    "dynamic={dynamic}; "
                    "dynamic_blockers={dynamic_blockers}; "
                    "dynamic_owners={owners}; "
                    "opened={opened}; "
                    "opened_search={opened_search}; "
                    "dynamic_exempt={dynamic_exempt}".format(
                        label=label,
                        end_x=end_cell[0],
                        end_y=end_cell[1],
                        end_angle=end_angle,
                        footprint=footprint,
                        static=sorted(static_overlap),
                        routing_static=sorted(routing_static_overlap),
                        static_blockers=sorted(effective_static_blockers),
                        dynamic=sorted(dynamic_overlap),
                        dynamic_blockers=sorted(effective_dynamic_blockers),
                        owners=owner_cells,
                        opened=sorted(footprint_set & opened_cells_set),
                        opened_search=sorted(footprint_set & opened_search_cells),
                        dynamic_exempt=sorted(
                            footprint_set & dynamic_clearance_exempt_cells
                        ),
                    )
                )
            return debug_lines

        lines = [
            f"net_name={job.net_name}",
            f"status={status}",
            f"source_spec={port1_spec}",
            f"target_spec={port2_spec}",
            f"source_component={_schematic_instance_component_name(schematic, job.inst1)}",
            f"target_component={_schematic_instance_component_name(schematic, job.inst2)}",
            f"source_access_rule={port_access_rule_by_spec.get(port1_spec)}",
            f"target_access_rule={port_access_rule_by_spec.get(port2_spec)}",
            f"foreign_port_keepout_cells={int(foreign_port_keepout_cells)}",
            f"fanout_access_mode={fanout_access_mode_normalized}",
            f"fanout_stub_bend_degrees={45 * int(_env_fanout_stub_bend_steps())}",
            f"fanout_anchor_port_count={len(fanout_anchor_by_port_spec)}",
            f"fanout_stub_center_cell_count={len(fanout_stub_center_cells)}",
            f"fanout_stub_static_cell_count={len(fanout_stub_static_cells)}",
            "source_fanout_anchor="
            f"{f'{job.inst1},{job.port1}' in fanout_anchor_by_port_spec}",
            "target_fanout_anchor="
            f"{f'{job.inst2},{job.port2}' in fanout_anchor_by_port_spec}",
            "source_dense_port_runway_cells="
            f"{dense_source_port_runway_length_by_spec.get(port1_spec)}",
            "target_dense_port_runway_cells="
            f"{dense_source_port_runway_length_by_spec.get(port2_spec)}",
            "source_dense_source_cluster_size="
            f"{len(dense_source_cluster_specs_by_port_spec.get(port1_spec, set()))}",
            "target_dense_source_cluster_size="
            f"{len(dense_source_cluster_specs_by_port_spec.get(port2_spec, set()))}",
            f"foreign_port_keepout_static_count={len(foreign_port_keepout_static_cells)}",
            f"foreign_port_keepout_open_count={len(foreign_keepout_open_cells)}",
            f"source_state=({source_anchor_cell[0]}, {source_anchor_cell[1]}, {int(source_state.angle)})",
            f"target_state=({target_anchor_cell[0]}, {target_anchor_cell[1]}, {int(target_state.angle)})",
            f"opened_candidate_cells_count={len(opened_candidate_cells)}",
            f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
            f"opened_candidate_static_overlap_bbox={_cells_bbox(opened_candidate_static_overlap)}",
            f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
            f"opened_candidate_dynamic_overlap_bbox={_cells_bbox(opened_candidate_dynamic_overlap)}",
            f"opened_cells_count={len(opened_cells_set)}",
            f"opened_cells={sorted(opened_cells_set)}",
            f"opened_static_overlap_count={len(opened_static_overlap)}",
            f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
            f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
            f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
            "current_port_runway_dynamic_overlap_count="
            f"{len(current_port_runway_dynamic_overlap)}",
            "current_port_runway_dynamic_overlap_bbox="
            f"{_cells_bbox(current_port_runway_dynamic_overlap)}",
            "sibling_port_runway_dynamic_overlap_count="
            f"{len(sibling_port_runway_dynamic_overlap)}",
            "sibling_port_runway_dynamic_overlap_bbox="
            f"{_cells_bbox(sibling_port_runway_dynamic_overlap)}",
            f"dynamic_clearance_exempt_cells_count={len(dynamic_clearance_exempt_cells)}",
            f"dynamic_clearance_exempt_cells_bbox={_cells_bbox(dynamic_clearance_exempt_cells)}",
            f"dynamic_clearance_exempt_dynamic_overlap_count={len(dynamic_exempt_dynamic_overlap)}",
            f"dynamic_clearance_exempt_dynamic_overlap_bbox={_cells_bbox(dynamic_exempt_dynamic_overlap)}",
            f"route_cells_count={len(route_cells)}",
            f"route_static_blocked_overlap_count={len(route_static_overlap)}",
            f"route_static_blocked_overlap_bbox={_cells_bbox(route_static_overlap)}",
            f"route_dynamic_overlap_count={len(route_dynamic_overlap)}",
            f"route_dynamic_overlap_bbox={_cells_bbox(route_dynamic_overlap)}",
            "route_overlap_current_port_runway_count="
            f"{len(route_overlap_current_port_runway)}",
            "route_overlap_current_port_runway_bbox="
            f"{_cells_bbox(route_overlap_current_port_runway)}",
            "route_overlap_sibling_port_runway_count="
            f"{len(route_overlap_sibling_port_runway)}",
            "route_overlap_sibling_port_runway_bbox="
            f"{_cells_bbox(route_overlap_sibling_port_runway)}",
            f"route_overlap_candidate_opened_static_count={len(route_overlap_with_candidate_opened_static)}",
            f"route_overlap_effective_opened_static_count={len(route_overlap_with_effective_opened_static)}",
            f"route_overlap_candidate_opened_dynamic_count={len(route_overlap_with_candidate_opened_dynamic)}",
            f"route_overlap_effective_opened_dynamic_count={len(route_overlap_with_effective_opened_dynamic)}",
            f"route_overlap_dynamic_clearance_exempt_count={len(route_overlap_with_dynamic_exempt)}",
        ]
        lines.extend(_format_first_move_debug())
        if route_segments:
            lines.append("route_segments=" + "; ".join(route_segments))
        if repair_note is not None:
            lines.append(f"repair={repair_note}")
        if error_text is not None:
            lines.append(f"error={error_text}")
        diag_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _record_route(
        job: RouteJob,
        route_obj: Any,
        opened_cells: list[tuple[int, int]],
        *,
        corrected_centerline_um: tuple[tuple[float, float], ...] = (),
        corrected_total_length_um: float | None = None,
    ) -> None:
        if not corrected_centerline_um:
            corrected_centerline_um = _fanout_stubbed_centerline(job, route_obj)
        if (
            not corrected_centerline_um
            and not enable_checked_endpoint_correction
            and hasattr(router, "route_primitive_centerline")
        ):
            try:
                corrected_centerline_um = _centerline_tuple(
                    router.route_primitive_centerline(route_obj)
                )
            except Exception:
                corrected_centerline_um = ()
            if corrected_centerline_um and hasattr(router, "centerline_length_um"):
                try:
                    corrected_total_length_um = float(
                        router.centerline_length_um(list(corrected_centerline_um))
                    )
                except Exception:
                    corrected_total_length_um = None
        route_bookkeeping.record_route(
            job,
            route_obj,
            opened_cells,
            route_cells=_route_cells_from_router(job.net_id) if track_dynamic_cells else None,
            corrected_centerline_um=corrected_centerline_um,
            corrected_total_length_um=corrected_total_length_um,
        )

    def _export_route_svg(
        job: RouteJob,
        route_obj: Any,
        *,
        suffix: str = "",
        obstacle_cells: set[tuple[int, int]] | None = None,
        opened_cells: list[tuple[int, int]] | None = None,
    ) -> None:
        should_export = (
            debug_path is not None
            and (debug_route_indices is None or job.route_index in debug_route_indices)
        )
        if not should_export:
            return
        route_dir = debug_path / "routes"
        _ensure_dir(route_dir)
        route_svg = route_dir / f"{debug_prefix}_{job.net_name}{suffix}.svg"
        if obstacle_cells is not None and hasattr(
            router, "export_debug_svg_with_obstacle_cells"
        ):
            svg_text = router.export_debug_svg_with_obstacle_cells(
                route_obj,
                sorted(obstacle_cells),
            )
        else:
            svg_text = router.export_debug_svg(route_obj)
        if opened_cells is None:
            try:
                _, _, _, _, opened_cells = _state_openings_for_job(job)
            except Exception:
                opened_cells = []
        if (
            debug_stop_after_route_index is not None
            and int(job.route_index) == int(debug_stop_after_route_index)
        ):
            next_job = full_route_jobs_by_route_index.get(
                int(debug_stop_after_route_index) + 1
            )
            if next_job is not None:
                try:
                    _, _, _, _, opened_cells = _states_and_openings(next_job)
                except Exception:
                    pass
        red_keepout_cells = debug_port_keepout_cells - {
            (int(cell[0]), int(cell[1])) for cell in opened_cells
        }
        if red_keepout_cells:
            overlay = ['<g id="port-keepout-cells">']
            for gx, gy in sorted(red_keepout_cells):
                if 0 <= gx < grid_width and 0 <= gy < grid_height:
                    svg_y = grid_height - gy - 1
                    overlay.append(
                        f'<rect class="port-keepout" x="{gx}" y="{svg_y}" '
                        'width="1" height="1" fill="#d93025" opacity="0.38" />'
                    )
            overlay.append("</g>")
            overlay_text = "".join(overlay)
            if "</svg>" in svg_text and 'id="port-keepout-cells"' not in svg_text:
                svg_text = svg_text.replace("</svg>", overlay_text + "</svg>", 1)
        route_svg.write_text(svg_text, encoding="utf-8")
        route_svgs.append(route_svg)

    def _route_engine_summary(route_obj: Any) -> str:
        expanded_states = int(getattr(route_obj, "expanded_states", 0))
        route_kind = "simple" if expanded_states == 0 else "astar"
        length_um = _as_float(getattr(route_obj, "total_length_um", 0.0), 0.0)
        total_cost = _as_float(getattr(route_obj, "total_cost", 0.0), 0.0)
        return (
            f"{route_kind} "
            f"length={length_um:.3f}um "
            f"cost={total_cost:.3f} "
            f"expanded={expanded_states}"
        )

    def _write_failed_log(
        job: RouteJob,
        source_state: Any,
        target_state: Any,
        opened_candidate_cells: set[tuple[int, int]],
        opened_cells: list[tuple[int, int]],
        error_text: str,
    ) -> None:
        if debug_path is None:
            return
        route_dir = debug_path / "routes"
        _ensure_dir(route_dir)
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        fail_txt = route_dir / f"{debug_prefix}_{job.net_name}_FAILED.txt"
        committed_dynamic_cells = _committed_dynamic_cells()
        opened_candidate_static_overlap = (
            opened_candidate_cells & static_blocked_cells_before_port_reservations
        )
        opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
        opened_cells_set = set(opened_cells)
        opened_static_overlap = opened_cells_set & static_blocked_cells_before_port_reservations
        opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
        current_attempts = [
            ("current", record)
            for record in route_attempt_records
            if getattr(record, "net_id", None) == job.net_id
        ]
        recent_attempts = [("recent", record) for record in route_attempt_records[-12:]]
        root_cause_line = _format_illegal_crossing_root_causes_line(
            [error_text]
            + [
                str(attempt.error)
                for _, attempt in (current_attempts[-8:] + recent_attempts)
                if getattr(attempt, "error", None)
            ]
        )
        fail_lines = [
            f"net_name={job.net_name}",
            f"source_spec={port1_spec}",
            f"target_spec={port2_spec}",
            f"source_state=({int(source_state.x)}, {int(source_state.y)}, {int(source_state.angle)})",
            f"target_state=({int(target_state.x)}, {int(target_state.y)}, {int(target_state.angle)})",
            f"allow_45_degree_turns={allow_45_degree_turns}",
            f"block_radius_cells={block_radius_cells}",
            "dynamic_obstacle_search_expansion_radius_cells="
            f"{clearance_policy.dynamic_obstacle_search_expansion_radius_cells}",
            "dynamic_route_commit_keepout_radius_cells="
            f"{clearance_policy.dynamic_route_commit_keepout_radius_cells}",
            "dynamic_route_core_radius_cells="
            f"{clearance_policy.dynamic_route_core_radius_cells}",
            f"bend_radius_cells={bend_radius_cells}",
            f"port_entry_length_cells={port_entry_length_cells}",
            f"port_entry_half_width_cells={port_entry_half_width_cells}",
            f"port_lane_length_cells={port_lane_length_cells}",
            f"port_lane_half_width_cells={port_lane_half_width_cells}",
            f"opened_candidate_cells_count={len(opened_candidate_cells)}",
            f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
            f"opened_candidate_static_overlap_bbox={_cells_bbox(opened_candidate_static_overlap)}",
            f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
            f"opened_candidate_dynamic_overlap_bbox={_cells_bbox(opened_candidate_dynamic_overlap)}",
            f"opened_cells_count={len(opened_cells)}",
            f"opened_static_overlap_count={len(opened_static_overlap)}",
            f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
            f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
            f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
            f"error={error_text}",
        ]
        if root_cause_line is not None:
            fail_lines.append(root_cause_line)
        fail_lines.extend(_format_native_repair_trace_lines(native_repair_trace_records))
        for label, attempt in (current_attempts[-8:] + recent_attempts):
            as_dict = attempt.as_dict()
            diagnostics = as_dict.get("diagnostics")
            fail_lines.append(
                f"attempt_{label}="
                + ", ".join(
                    f"{key}={as_dict.get(key)}"
                    for key in (
                        "attempt_index",
                        "bucket_name",
                        "failed",
                        "error",
                        "elapsed_s",
                        "expanded_states",
                        "generated_neighbors",
                        "window_attempts",
                        "used_full_grid_fallback",
                        "candidate_blocker_count",
                        "candidate_blocker_route_indices",
                        "ripup_victim_count",
                        "ripup_victim_route_indices",
                    )
                    if key in as_dict
                )
            )
            if isinstance(diagnostics, dict) and diagnostics:
                fail_lines.append(
                    f"attempt_{label}_diagnostics="
                    + ", ".join(
                        f"{key}={diagnostics.get(key)}"
                        for key in (
                            "candidate_blocker_count",
                            "candidate_blocker_route_indices",
                            "ripup_victim_count",
                            "ripup_victim_route_indices",
                            "route_bbox_min_x",
                            "route_bbox_max_x",
                            "route_bbox_min_y",
                            "route_bbox_max_y",
                        )
                        if key in diagnostics
                    )
                )
        fail_txt.write_text("\n".join(fail_lines) + "\n", encoding="utf-8")

    def _finalize_committed_route(
        job: RouteJob,
        route_obj: Any,
        opened_cells: list[tuple[int, int]],
        *,
        should_print_route: bool,
        diag_txt: Path | None,
        debug_obstacle_cells: set[tuple[int, int]] | None = None,
    ) -> None:
        nonlocal total_expanded_states, simple_route_count
        expanded_states = int(getattr(route_obj, "expanded_states", 0))
        total_expanded_states += expanded_states
        if expanded_states == 0:
            simple_route_count += 1

        if diagnostics_enabled:
            source_state, target_state, opened_candidate_cells, opened_cells_set, _ = (
                _state_openings_for_job(job)
            )
            route_cells = {
                (int(cell[0]), int(cell[1]))
                for cell in (getattr(route_obj, "cells", None) or [])
            }
            _write_route_diagnostics(
                job=job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(job),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="ok",
                route_cells=route_cells,
                route_obj=route_obj,
            )

        _export_route_svg(
            job,
            route_obj,
            obstacle_cells=debug_obstacle_cells,
            opened_cells=opened_cells,
        )

        if should_print_route:
            print(f"ok {_route_engine_summary(route_obj)}")

    if repair_config.enabled:
        if not hasattr(router, "route_many_with_repair_and_commit"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.route_many_with_repair_and_commit. Rebuild it with "
                "`maturin develop --release`; Python repair fallback has been removed."
            )
        batch_jobs: list[
            tuple[
                int,
                Any,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
        batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
        t_batch_job_pack_start = _pipeline_timer_start()
        for job in route_jobs:
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            route_selected_for_debug = (
                debug_route_indices is None or job.route_index in debug_route_indices
            )
            should_print_route = verbose_route_diagnostics and route_selected_for_debug
            if debug_route_indices is not None and route_selected_for_debug:
                should_print_route = True
            if should_print_route:
                print(
                    f"  Routing [{job.route_index}/{len(route_jobs)}] "
                    f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                    end=" ",
                )
            route_dir = debug_path / "routes" if debug_path is not None else None
            diag_txt: Path | None = None
            if (
                debug_path is not None
                and (route_selected_for_debug or collect_attempt_diagnostics)
                and route_dir is not None
            ):
                _ensure_dir(route_dir)
                diag_txt = route_dir / f"{debug_prefix}_{job.net_name}_diagnostics.txt"
            batch_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    _routing_endpoint_center_um(job, source=True),
                    _routing_endpoint_center_um(job, source=False),
                )
            )
            batch_opened_cells_by_id[int(job.net_id)] = opened_cells
            batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
        _record_pipeline_timing("batch_job_pack", t_batch_job_pack_start)

        batch_start = _timing_start()
        raw_batch_result = router.route_many_with_repair_and_commit(
            batch_jobs,
            block_radius_cells,
            commit_radius_cells,
            core_commit_radius_cells,
            int(repair_config.max_rounds),
            int(repair_config.max_victims_per_failure),
            float(repair_config.history_weight),
            int(repair_config.history_increment),
        )
        batch_elapsed_s = time.perf_counter() - batch_start if collect_timing else 0.0
        _record_pipeline_timing("native_route_batch", batch_start)
        t_batch_result_processing_start = _pipeline_timer_start()
        batch_result = dict(raw_batch_result)
        native_repair_trace_records = [
            dict(record)
            for record in cast(
                Iterable[Mapping[str, object]],
                batch_result.get("repair_trace", []),
            )
        ]
        _record_native_batch_timings(batch_result)
        raw_attempts = list(cast(Iterable[Any], batch_result.get("attempts", [])))
        per_attempt_elapsed_s = batch_elapsed_s / max(1, len(raw_attempts))
        for raw_attempt in raw_attempts:
            attempt = dict(raw_attempt)
            net_id = int(attempt["net_id"])
            job = route_jobs_by_id[net_id]
            route_obj = attempt.get("route")
            if route_obj is None:
                route_obj = None
            failed = bool(attempt.get("failed", False))
            bucket_name = str(attempt.get("bucket_name", "normal_route"))
            error_text = (
                str(attempt.get("error"))
                if attempt.get("error") is not None
                else None
            )
            repair_round_raw = attempt.get("repair_round")
            repair_round = (
                int(repair_round_raw)
                if repair_round_raw is not None
                else None
            )
            attempt_index = len(route_attempt_records) + 1
            candidate_blockers = [
                int(value)
                for value in cast(list[object], attempt.get("candidate_blockers", []))
            ]
            ripup_ids = [
                int(value)
                for value in cast(list[object], attempt.get("ripup_ids", []))
            ]
            if (
                route_obj is not None
                and not failed
                and bucket_name != "normal_route"
                and debug_path is not None
                and (debug_route_indices is None or job.route_index in debug_route_indices)
            ):
                _export_route_svg(
                    job,
                    route_obj,
                    suffix=f"_attempt{attempt_index}_{bucket_name}",
            )
            if collect_timing:
                bucket = route_timing_buckets.setdefault(
                    bucket_name,
                    RouteTimingBucket(),
                )
                if route_obj is not None and not failed:
                    bucket.record_route(
                        per_attempt_elapsed_s,
                        route_obj,
                    )
                else:
                    bucket.record_elapsed(
                        per_attempt_elapsed_s,
                        failed=failed,
                    )
                route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=attempt_index,
                        bucket_name=bucket_name,
                        net_id=job.net_id,
                        route_index=job.route_index,
                        net_name=job.net_name,
                        source=f"{job.inst1},{job.port1}",
                        target=f"{job.inst2},{job.port2}",
                        elapsed_s=per_attempt_elapsed_s,
                        route_obj=route_obj if route_obj is not None and not failed else None,
                        failed=failed,
                        repair_round=repair_round,
                        error=error_text,
                        diagnostics=_route_attempt_diagnostics(
                            job,
                            route_obj if route_obj is not None and not failed else None,
                            candidate_blockers=candidate_blockers,
                            ripup_ids=ripup_ids,
                        )
                        if collect_attempt_diagnostics
                        else None,
                    )
                )

        repair_count += int(batch_result.get("repair_count", 0) or 0)
        raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
        for raw_entry in raw_routes:
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            route_obj = entry["route"]
            job = route_jobs_by_id[net_id]
            opened_cells = batch_opened_cells_by_id[net_id]
            _record_route(job, route_obj, opened_cells)
            should_print_route, diag_txt = batch_debug_by_id[net_id]
            _finalize_committed_route(
                job,
                route_obj,
                opened_cells,
                should_print_route=should_print_route,
                diag_txt=diag_txt,
            )
        _record_pipeline_timing(
            "batch_result_processing",
            t_batch_result_processing_start,
        )

        if str(batch_result.get("status", "")) != "routed":
            failed_net_id = int(batch_result.get("failed_net_id", -1))
            failed_job = route_jobs_by_id.get(failed_net_id)
            error_text = str(batch_result.get("error", "No route found"))
            if failed_job is None:
                raise RuntimeError(error_text)
            source_state, target_state, opened_candidate_cells, opened_cells_set, opened_cells = (
                _state_openings_for_job(failed_job)
            )
            should_print_route, diag_txt = batch_debug_by_id.get(
                failed_net_id,
                (False, None),
            )
            if not should_print_route:
                print(
                    f"  Routing [{failed_job.route_index}/{len(route_jobs)}] "
                    f"{failed_job.net_name}: {failed_job.inst1},{failed_job.port1} -> "
                    f"{failed_job.inst2},{failed_job.port2}... failed"
                )
            _write_route_diagnostics(
                job=failed_job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(failed_job),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="failed",
                error_text=error_text,
            )
            _write_failed_log(
                failed_job,
                source_state,
                target_state,
                opened_candidate_cells,
                opened_cells,
                error_text,
            )
            raise RuntimeError(
                f"No route found for {failed_job.net_name}: "
                f"{failed_job.inst1},{failed_job.port1} -> {failed_job.inst2},{failed_job.port2}. "
                f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                f"allow_45_degree_turns={allow_45_degree_turns}. "
                f"error={error_text}"
            )

    else:
        if not hasattr(router, "route_many_normal_and_commit"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.route_many_normal_and_commit. Rebuild it with "
                "`maturin develop --release`; Python sequential routing fallback has been removed."
            )
        batch_jobs: list[
            tuple[
                int,
                Any,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
        batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
        t_batch_job_pack_start = _pipeline_timer_start()
        for job in route_jobs:
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            route_selected_for_debug = (
                debug_route_indices is None or job.route_index in debug_route_indices
            )
            should_print_route = verbose_route_diagnostics and route_selected_for_debug
            if debug_route_indices is not None and route_selected_for_debug:
                should_print_route = True
            if should_print_route:
                print(
                    f"  Routing [{job.route_index}/{len(route_jobs)}] "
                    f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                    end=" ",
                )
            route_dir = debug_path / "routes" if debug_path is not None else None
            diag_txt: Path | None = None
            if (
                debug_path is not None
                and (route_selected_for_debug or collect_attempt_diagnostics)
                and route_dir is not None
            ):
                _ensure_dir(route_dir)
                diag_txt = route_dir / f"{debug_prefix}_{job.net_name}_diagnostics.txt"
            batch_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    _routing_endpoint_center_um(job, source=True),
                    _routing_endpoint_center_um(job, source=False),
                )
            )
            batch_opened_cells_by_id[int(job.net_id)] = opened_cells
            batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
        _record_pipeline_timing("batch_job_pack", t_batch_job_pack_start)

        batch_start = _timing_start()
        raw_batch_result = router.route_many_normal_and_commit(
            batch_jobs,
            block_radius_cells,
            commit_radius_cells,
            core_commit_radius_cells,
        )
        batch_elapsed_s = time.perf_counter() - batch_start if collect_timing else 0.0
        _record_pipeline_timing("native_route_batch", batch_start)
        t_batch_result_processing_start = _pipeline_timer_start()
        batch_result = dict(raw_batch_result)
        _record_native_batch_timings(batch_result)
        raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
        per_route_elapsed_s = batch_elapsed_s / max(1, len(raw_routes))
        for raw_entry in raw_routes:
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            route_obj = entry["route"]
            job = route_jobs_by_id[net_id]
            opened_cells = batch_opened_cells_by_id[net_id]
            if collect_timing:
                route_timing_buckets["normal_route"].record_route(
                    per_route_elapsed_s,
                    route_obj,
                )
                route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=len(route_attempt_records) + 1,
                        bucket_name="normal_route",
                        net_id=job.net_id,
                        route_index=job.route_index,
                        net_name=job.net_name,
                        source=f"{job.inst1},{job.port1}",
                        target=f"{job.inst2},{job.port2}",
                        elapsed_s=per_route_elapsed_s,
                        route_obj=route_obj,
                    )
                )
            _record_route(job, route_obj, opened_cells)
            should_print_route, diag_txt = batch_debug_by_id[net_id]
            _finalize_committed_route(
                job,
                route_obj,
                opened_cells,
                should_print_route=should_print_route,
                diag_txt=diag_txt,
            )
        _record_pipeline_timing(
            "batch_result_processing",
            t_batch_result_processing_start,
        )

        if str(batch_result.get("status", "")) != "routed":
            failed_net_id = int(batch_result.get("failed_net_id", -1))
            failed_job = route_jobs_by_id.get(failed_net_id)
            error_text = str(batch_result.get("error", "No route found"))
            if failed_job is None:
                raise RuntimeError(error_text)
            source_state, target_state, opened_candidate_cells, opened_cells_set, opened_cells = (
                _state_openings_for_job(failed_job)
            )
            should_print_route, diag_txt = batch_debug_by_id.get(
                failed_net_id,
                (False, None),
            )
            if collect_timing:
                route_timing_buckets["normal_route"].record_elapsed(0.0, failed=True)
                route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=len(route_attempt_records) + 1,
                        bucket_name="normal_route",
                        net_id=failed_job.net_id,
                        route_index=failed_job.route_index,
                        net_name=failed_job.net_name,
                        source=f"{failed_job.inst1},{failed_job.port1}",
                        target=f"{failed_job.inst2},{failed_job.port2}",
                        elapsed_s=0.0,
                        route_obj=None,
                        failed=True,
                        error=error_text,
                    )
                )
            if not should_print_route:
                print(
                    f"  Routing [{failed_job.route_index}/{len(route_jobs)}] "
                    f"{failed_job.net_name}: {failed_job.inst1},{failed_job.port1} -> "
                    f"{failed_job.inst2},{failed_job.port2}... failed"
                )
            _write_route_diagnostics(
                job=failed_job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(failed_job),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="failed",
                error_text=error_text,
            )
            _write_failed_log(
                failed_job,
                source_state,
                target_state,
                opened_candidate_cells,
                opened_cells,
                error_text,
            )
            raise RuntimeError(
                f"No route found for {failed_job.net_name}: "
                f"{failed_job.inst1},{failed_job.port1} -> {failed_job.inst2},{failed_job.port2}. "
                f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                f"allow_45_degree_turns={allow_45_degree_turns}"
            )

    astar_elapsed_s = 0.0
    if collect_timing:
        astar_elapsed_s = time.perf_counter() - t_astar_start

    def _apply_checked_endpoint_corrections_for_net_ids(
        net_ids: Iterable[int],
        *,
        record_pipeline_timing: bool = True,
        print_warnings: bool = False,
    ) -> list[int]:
        if not enable_checked_endpoint_correction:
            return []
        if not hasattr(router, "apply_checked_endpoint_corrections"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.apply_checked_endpoint_corrections. "
                "Rebuild it with `maturin develop --release`."
            )
        correction_jobs: list[
            tuple[
                int,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        requested_net_ids = [int(net_id) for net_id in net_ids]
        crossing_net_ids: set[int] = set()
        if enable_crossings and hasattr(router, "crossing_events"):
            try:
                for raw_event in cast(Iterable[Any], router.crossing_events()):
                    if not isinstance(raw_event, Mapping):
                        try:
                            raw_event = dict(cast(Any, raw_event))
                        except (TypeError, ValueError):
                            continue
                    for key in ("net_id", "partner_net_id"):
                        try:
                            crossing_net_ids.add(int(cast(Any, raw_event.get(key))))
                        except (TypeError, ValueError):
                            continue
            except Exception:
                crossing_net_ids = set()
        t_endpoint_correction_pack_start = _pipeline_timer_start()
        for net_id in requested_net_ids:
            record = route_bookkeeping.records_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if record is None or job is None:
                continue
            if int(net_id) in fanout_anchor_net_ids and record.corrected_centerline_um:
                continue
            if enable_crossings and int(net_id) in crossing_net_ids:
                continue
            source_port = _routing_endpoint_center_um(job, source=True)
            target_port = _routing_endpoint_center_um(job, source=False)
            if source_port is None and target_port is None:
                continue
            source_state, target_state, opened_candidate_cells, _, _ = (
                _state_openings_for_job(job)
            )
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            correction_jobs.append(
                (
                    int(net_id),
                    record.route_obj,
                    sorted(opened_candidate_cells),
                    clearance_exempt_cells,
                    source_port,
                    target_port,
                )
            )
        if record_pipeline_timing:
            _record_pipeline_timing(
                "endpoint_correction_pack",
                t_endpoint_correction_pack_start,
            )
        if not correction_jobs:
            return []

        correction_start = _timing_start()
        raw_corrections = router.apply_checked_endpoint_corrections(
            correction_jobs,
            float(route_width_um),
            int(commit_radius_cells),
            int(core_commit_radius_cells),
            True,
        )
        correction_elapsed_s = (
            time.perf_counter() - correction_start if collect_timing else 0.0
        )
        if record_pipeline_timing:
            _record_pipeline_timing("endpoint_correction_native", correction_start)
        t_endpoint_correction_processing_start = _pipeline_timer_start()
        correction_elapsed_per_job_s = correction_elapsed_s / max(1, len(correction_jobs))
        failed_net_ids: list[int] = []
        for raw_correction in cast(Iterable[Any], raw_corrections):
            correction = dict(raw_correction)
            net_id = int(correction["net_id"])
            record = route_bookkeeping.records_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if record is None or job is None:
                continue
            error = correction.get("error")
            if error is not None:
                if collect_timing:
                    route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked grid-to-port endpoint correction skipped for net "
                    f"{job.net_name!r}: {error}"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                if not enable_crossings:
                    route_bookkeeping.records_by_id[net_id] = replace(
                        record,
                        corrected_centerline_um=(),
                        endpoint_correction_error=message,
                    )
                continue
            centerline = _centerline_tuple(correction.get("centerline"))
            if not centerline:
                if collect_timing:
                    route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked grid-to-port endpoint correction skipped for net "
                    f"{job.net_name!r}: endpoint correction returned an invalid centerline"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                if not enable_crossings:
                    route_bookkeeping.records_by_id[net_id] = replace(
                        record,
                        corrected_centerline_um=(),
                        endpoint_correction_error=message,
                    )
                continue
            if collect_timing:
                route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                )
            corrected_total_length_um = float(correction["total_length_um"])
            route_bookkeeping.records_by_id[net_id] = replace(
                record,
                total_length_um=corrected_total_length_um,
                base_total_length_um=(
                    record.base_total_length_um
                    if record.base_total_length_um is not None
                    else float(record.total_length_um)
                ),
                corrected_centerline_um=centerline,
                endpoint_correction_error=None,
            )
        if record_pipeline_timing:
            _record_pipeline_timing(
                "endpoint_correction_processing",
                t_endpoint_correction_processing_start,
            )
        return failed_net_ids

    def _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
        net_ids: Iterable[int],
        *,
        record_pipeline_timing: bool = True,
        print_warnings: bool = False,
    ) -> list[int]:
        if not enable_checked_endpoint_correction or not fanout_anchor_net_ids:
            return []
        if not hasattr(router, "apply_checked_endpoint_corrections"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.apply_checked_endpoint_corrections. "
                "Rebuild it with `maturin develop --release`."
            )

        correction_jobs: list[
            tuple[
                int,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        job_context_by_id: dict[int, tuple[RoutedNetRecord, bool, bool]] = {}
        requested_net_ids = [int(net_id) for net_id in net_ids]
        crossing_net_ids: set[int] = set()
        if enable_crossings and hasattr(router, "crossing_events"):
            try:
                for raw_event in cast(Iterable[Any], router.crossing_events()):
                    if not isinstance(raw_event, Mapping):
                        try:
                            raw_event = dict(cast(Any, raw_event))
                        except (TypeError, ValueError):
                            continue
                    for key in ("net_id", "partner_net_id"):
                        try:
                            crossing_net_ids.add(int(cast(Any, raw_event.get(key))))
                        except (TypeError, ValueError):
                            continue
            except Exception:
                crossing_net_ids = set()
        t_endpoint_correction_pack_start = _pipeline_timer_start()
        for net_id in requested_net_ids:
            record = route_bookkeeping.records_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if record is None or job is None or not record.corrected_centerline_um:
                continue
            source_has_fanout_stub = int(net_id) in fanout_anchor_source_net_ids
            target_has_fanout_stub = int(net_id) in fanout_anchor_target_net_ids
            if not (source_has_fanout_stub or target_has_fanout_stub):
                continue
            if source_has_fanout_stub and target_has_fanout_stub:
                continue
            # A fanout stub is already a corrected endpoint adapter. When the
            # routed net also contains a crossing, the unrestricted native
            # endpoint corrector may move geometry on the protected side of the
            # crossing before we merge it back into the stubbed centerline. Let
            # the crossing-aware pass splice only the source->first-crossing or
            # last-crossing->target segment instead.
            if enable_crossings and int(net_id) in crossing_net_ids:
                continue

            source_port = (
                None
                if source_has_fanout_stub
                else _routing_endpoint_center_um(job, source=True)
            )
            target_port = (
                None
                if target_has_fanout_stub
                else _routing_endpoint_center_um(job, source=False)
            )
            if source_port is None and target_port is None:
                continue
            _, _, opened_candidate_cells, _, _ = _state_openings_for_job(job)
            if source_has_fanout_stub:
                opened_candidate_cells.update(
                    fanout_stub_static_cells_by_spec.get(f"{job.inst1},{job.port1}", set())
                )
            if target_has_fanout_stub:
                opened_candidate_cells.update(
                    fanout_stub_static_cells_by_spec.get(f"{job.inst2},{job.port2}", set())
                )
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            correction_jobs.append(
                (
                    int(net_id),
                    record.route_obj,
                    sorted(opened_candidate_cells),
                    clearance_exempt_cells,
                    source_port,
                    target_port,
                )
            )
            job_context_by_id[int(net_id)] = (
                record,
                source_has_fanout_stub,
                target_has_fanout_stub,
            )

        if record_pipeline_timing:
            _record_pipeline_timing(
                "fanout_stub_endpoint_correction_pack",
                t_endpoint_correction_pack_start,
            )
        if not correction_jobs:
            return []

        correction_start = _timing_start()
        raw_corrections = router.apply_checked_endpoint_corrections(
            correction_jobs,
            float(route_width_um),
            int(commit_radius_cells),
            int(core_commit_radius_cells),
            True,
        )
        correction_elapsed_s = (
            time.perf_counter() - correction_start if collect_timing else 0.0
        )
        if record_pipeline_timing:
            _record_pipeline_timing(
                "fanout_stub_endpoint_correction_native",
                correction_start,
            )
        t_endpoint_correction_processing_start = _pipeline_timer_start()
        correction_elapsed_per_job_s = correction_elapsed_s / max(1, len(correction_jobs))
        failed_net_ids: list[int] = []
        for raw_correction in cast(Iterable[Any], raw_corrections):
            correction = dict(raw_correction)
            net_id = int(correction["net_id"])
            context = job_context_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if context is None or job is None:
                continue
            record, source_has_fanout_stub, target_has_fanout_stub = context
            error = correction.get("error")
            if error is not None:
                if collect_timing:
                    route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked fanout-stub endpoint correction skipped for net "
                    f"{job.net_name!r}: {error}"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    endpoint_correction_error=message,
                )
                continue

            corrected_route = _dedupe_centerline(
                _centerline_tuple(correction.get("centerline"))
            )
            route_baseline = _primitive_centerline_for_record(
                record,
                router=router,
                prefer_corrected_baseline=False,
            )
            existing_baseline = _dedupe_centerline(record.corrected_centerline_um)
            merged_centerline = _merge_terminal_corrected_route_centerline(
                existing_baseline=existing_baseline,
                route_baseline=route_baseline,
                corrected_route_centerline=corrected_route,
                freeze_source=source_has_fanout_stub,
                freeze_target=target_has_fanout_stub,
            )
            if len(merged_centerline) < 2:
                if collect_timing:
                    route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                route_start = route_baseline[0] if route_baseline else None
                route_end = route_baseline[-1] if route_baseline else None
                existing_start = existing_baseline[0] if existing_baseline else None
                existing_end = existing_baseline[-1] if existing_baseline else None
                corrected_start = (
                    corrected_route[0] if corrected_route else None
                )
                corrected_end = corrected_route[-1] if corrected_route else None
                message = (
                    "Checked fanout-stub endpoint correction skipped for net "
                    f"{job.net_name!r}: could not merge corrected route segment "
                    "back into static stub centerline "
                    f"(existing_len={len(existing_baseline)}, "
                    f"route_len={len(route_baseline)}, "
                    f"corrected_len={len(corrected_route)}, "
                    f"freeze_source={source_has_fanout_stub}, "
                    f"freeze_target={target_has_fanout_stub}, "
                    f"existing_start={existing_start}, existing_end={existing_end}, "
                    f"route_start={route_start}, route_end={route_end}, "
                    f"corrected_start={corrected_start}, corrected_end={corrected_end})"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    endpoint_correction_error=message,
                )
                continue

            if collect_timing:
                route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                )
            centerline_length = getattr(router, "centerline_length_um", None)
            if centerline_length is not None:
                try:
                    corrected_total_length_um = float(
                        centerline_length(list(merged_centerline))
                    )
                except Exception:
                    corrected_total_length_um = _centerline_length_um(merged_centerline)
            else:
                corrected_total_length_um = _centerline_length_um(merged_centerline)
            route_bookkeeping.records_by_id[net_id] = replace(
                record,
                total_length_um=corrected_total_length_um,
                base_total_length_um=(
                    record.base_total_length_um
                    if record.base_total_length_um is not None
                    else float(record.total_length_um)
                ),
                corrected_centerline_um=merged_centerline,
                endpoint_correction_error=None,
            )

        if record_pipeline_timing:
            _record_pipeline_timing(
                "fanout_stub_endpoint_correction_processing",
                t_endpoint_correction_processing_start,
            )
        return failed_net_ids

    def _current_crossing_points_by_net_id() -> dict[int, list[tuple[float, float]]]:
        if not enable_crossings or not hasattr(router, "crossing_events"):
            return {}
        try:
            raw_events = list(cast(Iterable[Any], router.crossing_events()))
        except Exception:
            return {}
        if not raw_events:
            return {}
        _populate_realized_intersections_from_native_crossing_events(
            crossing_plan_info=crossing_plan_info,
            routed_records_by_net_id=route_bookkeeping.records_by_id,
            native_crossing_events=raw_events,
            realization_grid_spec=realization_grid_spec,
        )
        return _legal_crossing_points_by_net_id(crossing_plan_info)

    def _route_target_grid_center_um(
        route_obj: object | None,
    ) -> tuple[float, float] | None:
        if route_obj is None:
            return None
        raw_state = getattr(route_obj, "reached_target", None)
        if raw_state is None:
            states = getattr(route_obj, "states", None)
            try:
                raw_state = cast(Any, states)[-1]
            except (TypeError, IndexError):
                return None
        try:
            return _grid_cell_center_um(int(raw_state.x), int(raw_state.y))
        except (AttributeError, TypeError, ValueError):
            return None

    def _route_target_angle(
        route_obj: object | None,
    ) -> int | None:
        if route_obj is None:
            return None
        raw_state = getattr(route_obj, "reached_target", None)
        if raw_state is None:
            states = getattr(route_obj, "states", None)
            try:
                raw_state = cast(Any, states)[-1]
            except (TypeError, IndexError):
                return None
        try:
            return int(raw_state.angle) % 8
        except (AttributeError, TypeError, ValueError):
            return None

    def _record_terminal_bump_distance_check_candidates(
        crossing_points_by_net_id: Mapping[int, list[tuple[float, float]]],
        net_ids: Iterable[int],
    ) -> None:
        """Record where a terminal bump distance check would become active.

        This is diagnostic-only: the router behavior is unchanged.  The check
        is axis-specific: horizontal target approaches care only about
        physical-port-vs-grid y offset, while vertical target approaches care
        only about physical-port-vs-grid x offset.  If a realized crossing sits
        on that same target axis, a future A* guard must make sure the
        remaining terminal segment is long enough to insert the required bump
        geometry.
        """

        if not isinstance(crossing_plan_info, dict):
            return

        trace_raw = os.environ.get(
            "PHOTONIC_ROUTER_TRACE_TERMINAL_BUMP_DISTANCE_CHECKS", ""
        )
        trace_tokens = {item.strip() for item in trace_raw.split(",") if item.strip()}
        trace_all = "*" in trace_tokens
        grid_size = float(grid.grid_size_um)
        eps = max(1e-6, grid_size * 1e-6)
        axis_eps = max(1e-6, grid_size * 0.25)
        crossing_half_um = (
            float(crossing_plan_info.get("crossing_half_size_cells", 0) or 0)
            * grid_size
        )
        required_bump_um = 4.0 * float(bend_radius_cells) * grid_size

        target_x_offset_nets: list[dict[str, object]] = []
        target_y_offset_nets: list[dict[str, object]] = []
        active_checks: list[dict[str, object]] = []

        for raw_net_id in net_ids:
            net_id = int(raw_net_id)
            record = route_bookkeeping.records_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if record is None or job is None or record.target_port_center_um is None:
                continue
            target_grid_um = _route_target_grid_center_um(record.route_obj)
            if target_grid_um is None:
                continue
            target_port_um = record.target_port_center_um
            target_dx_um = float(target_port_um[0]) - float(target_grid_um[0])
            target_dy_um = float(target_port_um[1]) - float(target_grid_um[1])
            target_angle = _route_target_angle(record.route_obj)
            target_axis = (
                "horizontal"
                if target_angle in (0, 4)
                else "vertical"
                if target_angle in (2, 6)
                else "diagonal"
            )

            net_entry = {
                "net_id": int(net_id),
                "net_name": str(record.net_name),
                "target_port": f"{job.inst2},{job.port2}",
                "target_port_um": [
                    round(float(target_port_um[0]), 6),
                    round(float(target_port_um[1]), 6),
                ],
                "target_grid_um": [
                    round(float(target_grid_um[0]), 6),
                    round(float(target_grid_um[1]), 6),
                ],
                "target_dx_um": round(float(target_dx_um), 6),
                "target_dy_um": round(float(target_dy_um), 6),
                "target_angle": None if target_angle is None else int(target_angle),
                "target_axis": target_axis,
            }
            target_x_active = target_axis == "vertical" and abs(target_dx_um) > eps
            target_y_active = target_axis == "horizontal" and abs(target_dy_um) > eps
            if target_x_active:
                target_x_offset_nets.append(net_entry)
            if target_y_active:
                target_y_offset_nets.append(net_entry)
            if not target_x_active and not target_y_active:
                continue

            for crossing_point in crossing_points_by_net_id.get(net_id, []):
                crossing_x = float(crossing_point[0])
                crossing_y = float(crossing_point[1])
                if target_x_active and abs(crossing_x - float(target_grid_um[0])) <= axis_eps:
                    distance_to_target_um = abs(float(target_grid_um[1]) - crossing_y)
                    available_um = max(0.0, distance_to_target_um - crossing_half_um)
                    active_checks.append(
                        {
                            **net_entry,
                            "axis": "target_x",
                            "crossing_um": [round(crossing_x, 6), round(crossing_y, 6)],
                            "distance_to_target_um": round(
                                float(distance_to_target_um), 6
                            ),
                            "crossing_half_um": round(float(crossing_half_um), 6),
                            "available_um": round(float(available_um), 6),
                            "required_bump_um": round(float(required_bump_um), 6),
                            "satisfies": bool(available_um + eps >= required_bump_um),
                        }
                    )
                if target_y_active and abs(crossing_y - float(target_grid_um[1])) <= axis_eps:
                    distance_to_target_um = abs(float(target_grid_um[0]) - crossing_x)
                    available_um = max(0.0, distance_to_target_um - crossing_half_um)
                    active_checks.append(
                        {
                            **net_entry,
                            "axis": "target_y",
                            "crossing_um": [round(crossing_x, 6), round(crossing_y, 6)],
                            "distance_to_target_um": round(
                                float(distance_to_target_um), 6
                            ),
                            "crossing_half_um": round(float(crossing_half_um), 6),
                            "available_um": round(float(available_um), 6),
                            "required_bump_um": round(float(required_bump_um), 6),
                            "satisfies": bool(available_um + eps >= required_bump_um),
                        }
                    )

            should_trace = trace_all or record.net_name in trace_tokens or str(net_id) in trace_tokens
            if should_trace:
                matching_checks = [
                    item for item in active_checks if int(item["net_id"]) == int(net_id)
                ]
                print(
                    "terminal_bump_distance_check "
                    f"net={record.net_name} id={net_id} "
                    f"target_port={job.inst2},{job.port2} "
                    f"target_port={target_port_um} target_grid={target_grid_um} "
                    f"target_axis={target_axis} "
                    f"target_dx={target_dx_um:.6f} target_dy={target_dy_um:.6f} "
                    f"same_axis_crossings={len(matching_checks)}"
                )
                for item in matching_checks:
                    print(
                        "terminal_bump_distance_check "
                        f"net={record.net_name} id={net_id} "
                        f"crossing={item['crossing_um']} "
                        f"available_um={item['available_um']} "
                        f"required_um={item['required_bump_um']} "
                        f"satisfies={item['satisfies']}"
                    )

        crossing_plan_info["terminal_bump_target_x_offset_nets"] = target_x_offset_nets
        crossing_plan_info["terminal_bump_target_x_offset_net_count"] = len(
            target_x_offset_nets
        )
        crossing_plan_info["terminal_bump_target_y_offset_nets"] = target_y_offset_nets
        crossing_plan_info["terminal_bump_target_y_offset_net_count"] = len(
            target_y_offset_nets
        )
        failed_checks = [
            check for check in active_checks if not bool(check.get("satisfies"))
        ]
        crossing_plan_info["terminal_bump_distance_checks"] = active_checks
        crossing_plan_info["terminal_bump_distance_check_count"] = len(active_checks)
        crossing_plan_info["terminal_bump_distance_failures"] = failed_checks
        crossing_plan_info["terminal_bump_distance_failure_count"] = len(failed_checks)

    def _apply_crossing_aware_endpoint_corrections_for_net_ids(
        net_ids: Iterable[int],
        *,
        record_pipeline_timing: bool = True,
        print_warnings: bool = False,
    ) -> list[int]:
        if not enable_checked_endpoint_correction or not enable_crossings:
            return []
        crossing_points_by_net_id = _current_crossing_points_by_net_id()
        if not crossing_points_by_net_id:
            return []
        requested_net_ids = [int(net_id) for net_id in net_ids]
        _record_terminal_bump_distance_check_candidates(
            crossing_points_by_net_id,
            requested_net_ids,
        )

        t_endpoint_correction_start = _pipeline_timer_start()
        failed_net_ids: list[int] = []
        for raw_net_id in requested_net_ids:
            net_id = int(raw_net_id)
            crossing_points = crossing_points_by_net_id.get(net_id, [])
            if not crossing_points:
                continue
            record = route_bookkeeping.records_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if record is None or job is None:
                continue

            source_has_fanout_stub = net_id in fanout_anchor_source_net_ids
            target_has_fanout_stub = net_id in fanout_anchor_target_net_ids
            record_has_fanout_stub = net_id in fanout_anchor_net_ids
            _, _, opened_candidate_cells, _, _ = _state_openings_for_job(job)
            if source_has_fanout_stub:
                opened_candidate_cells.update(
                    fanout_stub_static_cells_by_spec.get(f"{job.inst1},{job.port1}", set())
                )
            if target_has_fanout_stub:
                opened_candidate_cells.update(
                    fanout_stub_static_cells_by_spec.get(f"{job.inst2},{job.port2}", set())
                )
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            start_s = _timing_start()
            updated = _apply_crossing_aware_endpoint_correction_to_record(
                record,
                router=cast(EndpointCorrectionRouter, router),
                crossing_points=crossing_points,
                realization_grid_spec=realization_grid_spec,
                route_width_um=float(route_width_um),
                allow_unchecked_bumps=False,
                log_failures=print_warnings,
                crossing_plan_info=crossing_plan_info,
                correct_source=not source_has_fanout_stub,
                correct_target=not target_has_fanout_stub,
                prefer_corrected_baseline=(
                    record_has_fanout_stub and bool(record.corrected_centerline_um)
                ),
                opened_cells=opened_candidate_cells,
                clearance_exempt_cells=clearance_exempt_cells,
                clearance_radius_cells=int(commit_radius_cells),
                core_radius_cells=int(core_commit_radius_cells),
            )
            failed = updated.endpoint_correction_error is not None
            if collect_timing:
                route_timing_buckets["endpoint_correction"].record_elapsed(
                    time.perf_counter() - start_s,
                    failed=failed,
                )
            if failed:
                failed_net_ids.append(net_id)
            route_bookkeeping.records_by_id[net_id] = updated

        if record_pipeline_timing:
            _record_pipeline_timing(
                "crossing_endpoint_correction",
                t_endpoint_correction_start,
            )
        return failed_net_ids

    if enable_checked_endpoint_correction:
        _apply_checked_endpoint_corrections_for_net_ids(
            list(route_bookkeeping.route_order),
            print_warnings=(
                collect_attempt_diagnostics
                or diagnostics_enabled
                or verbose_route_diagnostics
            ),
        )
        _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
            list(route_bookkeeping.route_order),
            print_warnings=(
                collect_attempt_diagnostics
                or diagnostics_enabled
                or verbose_route_diagnostics
            ),
        )
        _apply_crossing_aware_endpoint_corrections_for_net_ids(
            list(route_bookkeeping.route_order),
            print_warnings=(
                collect_attempt_diagnostics
                or diagnostics_enabled
                or verbose_route_diagnostics
            ),
        )

    t_record_assembly_start = _pipeline_timer_start()
    routed_net_records = route_bookkeeping.ordered_records()
    routed_record_keys = [
        (record.net_name, record.source.instance, record.source.port, record.target.instance, record.target.port)
        for record in routed_net_records
    ]
    duplicate_record_keys = [
        key for key, count in Counter(routed_record_keys).items() if count > 1
    ]
    if duplicate_record_keys:
        formatted = ", ".join(
            f"{name}:{src_i},{src_p}->{dst_i},{dst_p}"
            for name, src_i, src_p, dst_i, dst_p in duplicate_record_keys[:8]
        )
        raise RuntimeError(f"Duplicate routed records generated: {formatted}")
    _record_pipeline_timing("record_assembly", t_record_assembly_start)

    if debug_timing and verbose_route_diagnostics:
        print(f"      - A* route-search loop time: {astar_elapsed_s:.4f} s")
        print(
            "      - Route search stats: "
            f"simple={simple_route_count}/{len(route_jobs)}, "
            f"expanded_states={total_expanded_states}, "
            f"repairs={repair_count}"
        )
        print("      - A* timing breakdown by operation:")
        for bucket_name in (
            "normal_route",
            "probe_route",
            "preemptive_crossing_ripup",
            "guided_collision_crossing",
            "localized_crossing_keepout",
            "repair_failed_net",
            "reroute_victims",
            "lidar_pure_probe_commit",
            "endpoint_correction",
        ):
            bucket = route_timing_buckets[bucket_name]
            if bucket.calls == 0:
                continue
            line = (
                f"        {bucket_name}: calls={bucket.calls}, "
                f"ok={bucket.successes}, fail={bucket.failures}, "
                f"time={bucket.elapsed_s:.4f}s"
            )
            has_route_stats = (
                bucket.expanded_states
                or bucket.generated_neighbors
                or bucket.heap_pushes
                or bucket.heap_pops
                or bucket.window_attempts
                or bucket.footprint_checks
                or bucket.dense_grid_build_time_us
                or bucket.max_window_area_cells
                or bucket.full_grid_fallbacks
            )
            if has_route_stats:
                line += (
                    f", expanded={bucket.expanded_states}, "
                    f"generated={bucket.generated_neighbors}, "
                    f"heap_pushes={bucket.heap_pushes}, "
                    f"heap_pops={bucket.heap_pops}, "
                    f"duplicate_skips={bucket.skipped_duplicate_heap_entries}, "
                    f"windows={bucket.window_attempts}, "
                    f"max_window={bucket.max_window_area_cells}, "
                    f"legality_checks={bucket.obstacle_clearance_checks}, "
                    f"footprint_checks={bucket.footprint_checks}, "
                    f"rect_checks={bucket.footprint_rect_checks}, "
                    f"dense_cells={bucket.dense_grid_cells}, "
                    f"dense_build={bucket.dense_grid_build_time_us / 1_000_000.0:.4f}s, "
                    f"search_loop={bucket.search_loop_time_us / 1_000_000.0:.4f}s, "
                    f"obstacle_prepare={bucket.obstacle_map_prepare_time_us / 1_000_000.0:.4f}s, "
                    f"simple_probe={bucket.simple_route_time_us / 1_000_000.0:.4f}s, "
                    f"commit_prepare={bucket.commit_prepare_time_us / 1_000_000.0:.4f}s, "
                    f"commit={bucket.commit_time_us / 1_000_000.0:.4f}s, "
                    f"neighbor_time={bucket.neighbor_generation_time_us / 1_000_000.0:.4f}s, "
                    f"heap_time={bucket.heap_operation_time_us / 1_000_000.0:.4f}s, "
                    f"legality_time={bucket.legality_check_time_us / 1_000_000.0:.4f}s, "
                    f"reconstruction_time={bucket.reconstruction_time_us / 1_000_000.0:.4f}s, "
                    f"full_grid_fallbacks={bucket.full_grid_fallbacks}"
                )
            print(line)

    def _grid_cell_from_raw_point(raw_point: object) -> tuple[int, int] | None:
        if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
            return None
        try:
            point_x = float(raw_point[0])
            point_y = float(raw_point[1])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(point_x) or not math.isfinite(point_y):
            return None
        return (
            int(math.floor((point_x - float(origin_x_um)) / float(grid.grid_size_um))),
            int(math.floor((point_y - float(origin_y_um)) / float(grid.grid_size_um))),
        )

    def _illegal_crossing_grid_cell(item: Mapping[str, object]) -> tuple[int, int] | None:
        raw_cell = item.get("grid_cell")
        if isinstance(raw_cell, (tuple, list)) and len(raw_cell) == 2:
            try:
                return (int(raw_cell[0]), int(raw_cell[1]))
            except (TypeError, ValueError):
                return None
        return _grid_cell_from_raw_point(item.get("point_um"))

    def _illegal_crossing_keepout_radius(item: Mapping[str, object]) -> int:
        reason = str(item.get("reason", "") or "")
        if reason == "not_perpendicular":
            return max(1, int(resolved_crossing_half_size_cells) + 1)
        if reason == "collinear_route_overlap":
            return 1
        blockers = item.get("crossing_footprint_blockers")
        if isinstance(blockers, IterableABC) and not isinstance(
            blockers,
            (str, bytes, bytearray),
        ):
            if any(isinstance(blocker, Mapping) for blocker in blockers):
                return max(1, int(resolved_crossing_half_size_cells) + 1)
        if reason in {
            "crossing_footprint_contains_route_geometry",
            "crossing_footprint_overlap",
        }:
            return max(1, int(resolved_crossing_half_size_cells) + 1)
        return max(1, min(4, int(resolved_crossing_half_size_cells) + 1))

    def _add_keepout_square(
        keepout_cells: set[tuple[int, int]],
        *,
        center: tuple[int, int],
        radius: int,
    ) -> None:
        for y in range(center[1] - radius, center[1] + radius + 1):
            for x in range(center[0] - radius, center[0] + radius + 1):
                if 0 <= x < int(grid.width) and 0 <= y < int(grid.height):
                    keepout_cells.add((x, y))

    def _add_segment_keepout_cells(
        keepout_cells: set[tuple[int, int]],
        *,
        start_cell: tuple[int, int],
        end_cell: tuple[int, int],
        radius: int,
    ) -> None:
        dx = int(end_cell[0]) - int(start_cell[0])
        dy = int(end_cell[1]) - int(start_cell[1])
        steps = max(abs(dx), abs(dy), 1)
        for step in range(steps + 1):
            t = float(step) / float(steps)
            cell = (
                int(round(float(start_cell[0]) + float(dx) * t)),
                int(round(float(start_cell[1]) + float(dy) * t)),
            )
            _add_keepout_square(keepout_cells, center=cell, radius=radius)

    def _final_crossing_repair_keepout_cells(
        illegal_crossings: Iterable[Mapping[str, object]],
    ) -> set[tuple[int, int]]:
        keepout_cells: set[tuple[int, int]] = set()
        for item in illegal_crossings:
            radius = _illegal_crossing_keepout_radius(item)
            reason = str(item.get("reason", "") or "")
            if reason == "collinear_route_overlap":
                start_cell = _grid_cell_from_raw_point(item.get("overlap_start_um"))
                end_cell = _grid_cell_from_raw_point(item.get("overlap_end_um"))
                if start_cell is not None and end_cell is not None:
                    _add_segment_keepout_cells(
                        keepout_cells,
                        start_cell=start_cell,
                        end_cell=end_cell,
                        radius=radius,
                    )
                    continue
            center = _illegal_crossing_grid_cell(item)
            if center is not None:
                _add_keepout_square(keepout_cells, center=center, radius=radius)
            if reason == "crossing_footprint_overlap":
                peer = item.get("overlapping_crossing")
                if isinstance(peer, Mapping):
                    peer_center = _grid_cell_from_raw_point(peer.get("point_um"))
                    if peer_center is not None:
                        _add_keepout_square(
                            keepout_cells,
                            center=peer_center,
                            radius=radius,
                        )
        return keepout_cells

    def _final_crossing_repair_net_ids(
        illegal_crossings: Iterable[Mapping[str, object]],
    ) -> list[int]:
        net_ids: set[int] = set()

        def _add_footprint_blocker_net_ids(item: Mapping[str, object]) -> None:
            blockers = item.get("crossing_footprint_blockers")
            if not isinstance(blockers, IterableABC) or isinstance(
                blockers,
                (str, bytes, bytearray),
            ):
                return
            for blocker in blockers:
                if not isinstance(blocker, Mapping):
                    continue
                try:
                    blocker_net_id = int(cast(object, blocker.get("net_id")))
                except (TypeError, ValueError):
                    continue
                if blocker_net_id in route_jobs_by_id:
                    net_ids.add(blocker_net_id)

        for item in illegal_crossings:
            item_pair_ids: list[int] = []
            for key in ("net_id_a", "net_id_b"):
                try:
                    net_id = int(cast(object, item.get(key)))
                except (TypeError, ValueError):
                    continue
                if net_id in route_jobs_by_id:
                    item_pair_ids.append(net_id)
            reason = str(item.get("reason", "") or "")
            if reason == "not_perpendicular":
                net_ids.update(item_pair_ids)
                _add_footprint_blocker_net_ids(item)
                continue
            if reason in {
                "crossing_footprint_contains_bend",
                "insufficient_straight_margin",
            }:
                if item_pair_ids:
                    net_ids.add(max(item_pair_ids))
                _add_footprint_blocker_net_ids(item)
                continue
            if reason == "collinear_route_overlap":
                net_ids.update(item_pair_ids)
                continue
            if reason == "crossing_footprint_overlap":
                peer_pair_ids: set[int] = set()
                peer = item.get("overlapping_crossing")
                if isinstance(peer, Mapping):
                    for key in ("net_id_a", "net_id_b"):
                        try:
                            peer_net_id = int(cast(object, peer.get(key)))
                        except (TypeError, ValueError):
                            continue
                        if peer_net_id in route_jobs_by_id:
                            peer_pair_ids.add(peer_net_id)
                net_ids.update(item_pair_ids)
                net_ids.update(peer_pair_ids)
                continue
            if reason == "crossing_footprint_contains_route_geometry":
                before_blocker_net_ids = set(net_ids)
                _add_footprint_blocker_net_ids(item)
                if net_ids == before_blocker_net_ids:
                    net_ids.update(item_pair_ids)
                continue
            net_ids.update(item_pair_ids)
        return [net_id for net_id in route_order if net_id in net_ids]

    def _final_crossing_repair_batches(
        illegal_crossings: list[dict[str, object]],
        *,
        max_net_ids: int,
    ) -> list[list[dict[str, object]]]:
        if not illegal_crossings or max_net_ids <= 0:
            return []
        components: list[tuple[list[dict[str, object]], set[int]]] = []
        for item in illegal_crossings:
            item_net_ids = set(_final_crossing_repair_net_ids([item]))
            if not item_net_ids:
                components.append(([item], set()))
                continue
            matching_indices = [
                index
                for index, (_items, component_net_ids) in enumerate(components)
                if component_net_ids.intersection(item_net_ids)
            ]
            if not matching_indices:
                components.append(([item], set(item_net_ids)))
                continue
            target_index = matching_indices[0]
            target_items, target_net_ids = components[target_index]
            target_items.append(item)
            target_net_ids.update(item_net_ids)
            for merge_index in reversed(matching_indices[1:]):
                merge_items, merge_net_ids = components.pop(merge_index)
                target_items.extend(merge_items)
                target_net_ids.update(merge_net_ids)

        capped_batches: list[list[dict[str, object]]] = []
        oversized: list[dict[str, object]] = []
        for component_items, _component_net_ids in components:
            component_repair_ids = _final_crossing_repair_net_ids(component_items)
            if component_repair_ids and len(component_repair_ids) <= max_net_ids:
                capped_batches.append(component_items)
            else:
                oversized.extend(component_items)

        if oversized:
            current_batch: list[dict[str, object]] = []
            current_net_ids: set[int] = set()
            for item in oversized:
                item_net_ids = set(_final_crossing_repair_net_ids([item]))
                if (
                    current_batch
                    and item_net_ids
                    and len(current_net_ids.union(item_net_ids)) > max_net_ids
                ):
                    capped_batches.append(current_batch)
                    current_batch = []
                    current_net_ids = set()
                current_batch.append(item)
                current_net_ids.update(item_net_ids)
            if current_batch:
                capped_batches.append(current_batch)

        route_index = {int(net_id): index for index, net_id in enumerate(route_order)}

        def _batch_sort_key(batch: list[dict[str, object]]) -> tuple[int, int, int]:
            repair_ids = _final_crossing_repair_net_ids(batch)
            first_route_index = min(
                (route_index.get(int(net_id), len(route_index)) for net_id in repair_ids),
                default=len(route_index),
            )
            return (first_route_index, -len(batch), len(repair_ids))

        capped_batches.sort(key=_batch_sort_key)
        return capped_batches

    def _repair_final_illegal_crossings(
        illegal_crossings: list[dict[str, object]],
    ) -> bool:
        max_repair_net_ids = 12
        attempts = cast(
            list[dict[str, object]],
            crossing_plan_info.setdefault("final_crossing_repair_attempts", []),
        )
        priority_order = (
            "collinear_route_overlap",
            "crossing_footprint_contains_route_geometry",
            "crossing_footprint_contains_bend",
            "insufficient_straight_margin",
            "not_perpendicular",
            "crossing_footprint_overlap",
        )
        selected_reason = next(
            (
                reason
                for reason in priority_order
                if any(str(item.get("reason", "") or "") == reason for item in illegal_crossings)
            ),
            None,
        )
        selected_illegal_crossings = [
            item
            for item in illegal_crossings
            if selected_reason is None
            or str(item.get("reason", "") or "") == selected_reason
        ]
        total_selected_issue_count = len(selected_illegal_crossings)
        if selected_reason in {
            "crossing_footprint_contains_bend",
            "insufficient_straight_margin",
            "not_perpendicular",
        }:
            repair_batches = _final_crossing_repair_batches(
                selected_illegal_crossings,
                max_net_ids=min(4, max_repair_net_ids),
            )
            if repair_batches:
                selected_illegal_crossings = repair_batches[0]
        if (
            not selected_illegal_crossings
            or not crossing_plan_info.get("enabled")
            or not repair_config.enabled
            or not hasattr(router, "add_static_cells")
            or not hasattr(router, "ripup_route")
            or not hasattr(router, "route_many_with_repair_and_commit")
        ):
            return False
        repair_net_ids = _final_crossing_repair_net_ids(selected_illegal_crossings)
        if selected_reason == "not_perpendicular" and repair_net_ids:
            priority: list[int] = []
            for item in selected_illegal_crossings:
                for key in ("net_id_b", "net_id_a"):
                    try:
                        net_id = int(cast(object, item.get(key)))
                    except (TypeError, ValueError):
                        continue
                    if net_id in route_jobs_by_id and net_id not in priority:
                        priority.append(net_id)
                blockers = item.get("crossing_footprint_blockers")
                if not isinstance(blockers, IterableABC) or isinstance(
                    blockers,
                    (str, bytes, bytearray),
                ):
                    continue
                for blocker in blockers:
                    if not isinstance(blocker, Mapping):
                        continue
                    try:
                        blocker_net_id = int(cast(object, blocker.get("net_id")))
                    except (TypeError, ValueError):
                        continue
                    if blocker_net_id in route_jobs_by_id and blocker_net_id not in priority:
                        priority.append(blocker_net_id)
            repair_net_ids = [
                net_id
                for net_id in priority
                if net_id in repair_net_ids
            ] + [
                net_id
                for net_id in repair_net_ids
                if net_id not in priority
            ]
        attempt: dict[str, object] = {
            "selected_reason": selected_reason,
            "illegal_reason_counts": dict(
                Counter(str(item.get("reason", "") or "unknown") for item in illegal_crossings)
            ),
            "selected_reason_counts": dict(
                Counter(
                    str(item.get("reason", "") or "unknown")
                    for item in selected_illegal_crossings
                )
            ),
            "repair_net_ids": [int(net_id) for net_id in repair_net_ids],
        }
        if total_selected_issue_count != len(selected_illegal_crossings):
            attempt["batched_issue_count"] = len(selected_illegal_crossings)
            attempt["total_selected_issue_count"] = total_selected_issue_count
        if not repair_net_ids or len(repair_net_ids) > max_repair_net_ids:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_repairable_nets_or_too_many"
            attempts.append(attempt)
            return False
        keepout_cells = _final_crossing_repair_keepout_cells(selected_illegal_crossings)
        attempt["keepout_cell_count"] = len(keepout_cells)
        if not keepout_cells:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_keepout_cells"
            attempts.append(attempt)
            return False

        router.add_static_cells(sorted(keepout_cells))
        for net_id in repair_net_ids:
            router.ripup_route(int(net_id))
            route_bookkeeping.clear_route(int(net_id))

        repair_jobs: list[
            tuple[
                int,
                Any,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        opened_by_id: dict[int, list[tuple[int, int]]] = {}
        for net_id in repair_net_ids:
            job = route_jobs_by_id[net_id]
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            repair_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    _routing_endpoint_center_um(job, source=True),
                    _routing_endpoint_center_um(job, source=False),
                )
            )
            opened_by_id[int(job.net_id)] = opened_cells

        raw_repair_result = router.route_many_with_repair_and_commit(
            repair_jobs,
            block_radius_cells,
            commit_radius_cells,
            core_commit_radius_cells,
            int(repair_config.max_rounds),
            int(repair_config.max_victims_per_failure),
            float(repair_config.history_weight),
            int(repair_config.history_increment),
        )
        repair_result = dict(raw_repair_result)
        attempt["router_status"] = str(repair_result.get("status", ""))
        attempt["routed_net_ids"] = [
            int(dict(raw_entry)["net_id"])
            for raw_entry in cast(Iterable[Any], repair_result.get("routes", []))
        ]
        if str(repair_result.get("status", "")) != "routed":
            attempt["status"] = "failed"
            attempts.append(attempt)
            return False

        repaired_records: list[RoutedNetRecord] = []
        for raw_entry in cast(Iterable[Any], repair_result.get("routes", [])):
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            job = route_jobs_by_id[net_id]
            route_obj = entry["route"]
            _record_route(job, route_obj, opened_by_id[net_id])
            repaired_records.append(route_bookkeeping.records_by_id[net_id])

        if enable_checked_endpoint_correction and repaired_records:
            repaired_net_ids = [
                int(record.net_id)
                for record in repaired_records
                if record.net_id is not None
            ]
            _apply_checked_endpoint_corrections_for_net_ids(
                repaired_net_ids,
                record_pipeline_timing=False,
            )
            _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
                repaired_net_ids,
                record_pipeline_timing=False,
            )
        attempt["status"] = "routed"
        attempts.append(attempt)
        return True

    def _net_id_by_name() -> dict[str, int]:
        return {
            record.net_name: int(net_id)
            for net_id, record in route_bookkeeping.records_by_id.items()
        }

    def _grid_rect_from_um_bbox(
        raw_bbox: object,
    ) -> tuple[int, int, int, int] | None:
        if not isinstance(raw_bbox, (tuple, list)) or len(raw_bbox) != 4:
            return None
        try:
            min_x_um = float(raw_bbox[0])
            min_y_um = float(raw_bbox[1])
            max_x_um = float(raw_bbox[2])
            max_y_um = float(raw_bbox[3])
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (min_x_um, min_y_um, max_x_um, max_y_um)):
            return None
        if max_x_um < min_x_um:
            min_x_um, max_x_um = max_x_um, min_x_um
        if max_y_um < min_y_um:
            min_y_um, max_y_um = max_y_um, min_y_um
        grid_size = float(grid.grid_size_um)
        return (
            int(math.floor((min_x_um - float(origin_x_um)) / grid_size)),
            int(math.ceil((max_x_um - float(origin_x_um)) / grid_size)),
            int(math.floor((min_y_um - float(origin_y_um)) / grid_size)),
            int(math.ceil((max_y_um - float(origin_y_um)) / grid_size)),
        )

    def _add_keepout_rect(
        keepout_cells: set[tuple[int, int]],
        *,
        rect: tuple[int, int, int, int],
        radius: int,
    ) -> None:
        min_x, max_x, min_y, max_y = rect
        if min_x > max_x:
            min_x, max_x = max_x, min_x
        if min_y > max_y:
            min_y, max_y = max_y, min_y
        for y in range(min_y - radius, max_y + radius + 1):
            if y < 0 or y >= int(grid.height):
                continue
            for x in range(min_x - radius, max_x + radius + 1):
                if 0 <= x < int(grid.width):
                    keepout_cells.add((x, y))

    def _cells_from_um_bbox(
        raw_bbox: object,
        *,
        radius: int,
    ) -> set[tuple[int, int]]:
        rect = _grid_rect_from_um_bbox(raw_bbox)
        if rect is None:
            return set()
        cells: set[tuple[int, int]] = set()
        _add_keepout_rect(cells, rect=rect, radius=radius)
        return cells

    def _grid_rect_from_grid_bbox_text(text: str, name: str) -> tuple[int, int, int, int] | None:
        match = re.search(rf"{re.escape(name)}=\((-?\d+),(-?\d+),(-?\d+),(-?\d+)\)", text)
        if match is None:
            return None
        try:
            min_x = int(match.group(1))
            max_x = int(match.group(2))
            min_y = int(match.group(3))
            max_y = int(match.group(4))
        except (TypeError, ValueError):
            return None
        return min_x, max_x, min_y, max_y

    def _polygon_bbox_um(raw_polygon: object) -> tuple[float, float, float, float] | None:
        if not isinstance(raw_polygon, IterableABC) or isinstance(
            raw_polygon,
            (str, bytes, bytearray),
        ):
            return None
        points: list[tuple[float, float]] = []
        for raw_point in raw_polygon:
            if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
                return None
            try:
                point = (float(raw_point[0]), float(raw_point[1]))
            except (TypeError, ValueError):
                return None
            if not math.isfinite(point[0]) or not math.isfinite(point[1]):
                return None
            points.append(point)
        if not points:
            return None
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        return min(xs), min(ys), max(xs), max(ys)

    def _photonic_issue_keepout_cells(
        issue: PhotonicVerificationIssue,
    ) -> set[tuple[int, int]]:
        radius = max(1, int(core_commit_radius_cells) + 1)
        if issue.code == "endpoint_correction_error":
            cells: set[tuple[int, int]] = set()
            for bbox_name in ("static_bbox", "core_bbox"):
                rect = _grid_rect_from_grid_bbox_text(issue.message, bbox_name)
                if rect is None:
                    continue
                _add_keepout_rect(cells, rect=rect, radius=radius)
                if cells:
                    return cells
            return cells
        details = issue.details or {}
        if issue.code == "cross_net_waveguide_overlap":
            return _cells_from_um_bbox(
                details.get("overlap_bbox_um"),
                radius=radius,
            )
        if issue.code == "waveguide_obstacle_overlap":
            return _cells_from_um_bbox(
                details.get("overlap_bbox_um"),
                radius=radius,
            )
        if issue.code == "crossing_component_route_overlap":
            crossing = details.get("crossing")
            if isinstance(crossing, Mapping):
                polygon_bbox = _polygon_bbox_um(
                    crossing.get("crossing_footprint_polygon_um")
                )
                if polygon_bbox is not None:
                    return _cells_from_um_bbox(polygon_bbox, radius=radius)
            return _cells_from_um_bbox(
                details.get("overlap_bbox_um"),
                radius=radius,
            )
        return set()

    def _photonic_issue_net_ids(
        issue: PhotonicVerificationIssue,
    ) -> set[int]:
        by_name = _net_id_by_name()
        net_ids: set[int] = set()
        if issue.net_name and issue.net_name in by_name:
            net_ids.add(by_name[issue.net_name])
        details = issue.details or {}
        other_net_name = details.get("other_net_name")
        if isinstance(other_net_name, str) and other_net_name in by_name:
            net_ids.add(by_name[other_net_name])
        return net_ids

    photonic_probe_index = 0
    last_photonic_probe_layout: Component | None = None
    last_photonic_probe_records: list[RoutedNetRecord] = []

    def _make_photonic_verification_probe_layout(
        records: Iterable[RoutedNetRecord],
    ) -> Component:
        nonlocal photonic_probe_index
        t_probe_layout_total_start = _pipeline_timer_start()
        photonic_probe_index += 1
        t_probe_copy_start = _pipeline_timer_start()
        probe_layout = unrouted_layout.copy()
        probe_layout.name = f"photonic_repair_probe_{time.time_ns()}_{photonic_probe_index}"
        _record_pipeline_timing("photonic_probe_copy", t_probe_copy_start)
        t_probe_realize_start = _pipeline_timer_start()
        realize_routed_net_records(
            probe_layout,
            list(records),
            route_width_um=route_width_um,
            route_layer=route_layer,
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=bend_radius_cells,
            crossing_plan_info=crossing_plan_info,
            enable_endpoint_correction=enable_checked_endpoint_correction,
        )
        _record_pipeline_timing("photonic_probe_realize", t_probe_realize_start)
        if crossing_plan_info.get("enabled"):
            t_probe_crossings_start = _pipeline_timer_start()
            _place_realized_crossing_components(probe_layout, crossing_plan_info)
            _record_pipeline_timing(
                "photonic_probe_crossing_place",
                t_probe_crossings_start,
            )
        _record_pipeline_timing(
            "photonic_probe_layout_total",
            t_probe_layout_total_start,
        )
        return probe_layout

    def _refresh_photonic_verification() -> PhotonicVerificationResult:
        nonlocal last_photonic_probe_layout, last_photonic_probe_records
        # This is an internal diagnostic verifier, not the intended default
        # source of truth for production routing success. A* and the grid-level
        # crossing checks must reject illegal moves locally; the final geometry
        # gate is the Python verifier in `routing_flow.py` on the realized
        # layout. Keep this probe available for debugging model mismatches
        # between grid decisions and realized geometry, but do not treat it as
        # a mandatory always-on second full verification pass.
        t_refresh_start = _pipeline_timer_start()
        records = route_bookkeeping.ordered_records()
        probe_layout = _make_photonic_verification_probe_layout(records)
        last_photonic_probe_layout = probe_layout
        last_photonic_probe_records = list(records)
        t_verify_start = _pipeline_timer_start()
        result = verify_photonic_routing(
            probe_layout,
            schematic,
            routed_net_records=records,
            unrouted_layout=unrouted_layout,
            route_width_um=route_width_um,
            route_layer=route_layer,
            obstacle_layers=_default_obstacle_layers(
                route_layer,
                include_heater_obstacles=include_heater_obstacles,
            ),
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=bend_radius_cells,
            legal_overlap_polygons_by_net_id_pair_um=(
                _legal_crossing_overlap_polygons_for_verification(crossing_plan_info)
            ),
            crossing_component_footprints_um=(
                _legal_crossing_component_footprints_for_verification(crossing_plan_info)
            ),
            check_route_coverage=debug_stop_after_route_index is None,
            check_endpoint_connectivity=enable_checked_endpoint_correction,
        )
        _record_pipeline_timing("photonic_probe_verify", t_verify_start)
        _record_pipeline_timing("photonic_refresh_total", t_refresh_start)
        return result

    def _repair_final_photonic_issues(
        issues: tuple[PhotonicVerificationIssue, ...],
    ) -> bool:
        attempts = cast(
            list[dict[str, object]],
            crossing_plan_info.setdefault("final_photonic_repair_attempts", []),
        )
        priority_groups: tuple[tuple[str, set[str]], ...] = (
            (
                "endpoint_connection",
                {
                    "endpoint_correction_error",
                    "missing_corrected_centerline",
                    "source_port_not_connected",
                    "target_port_not_connected",
                    "source_endpoint_mismatch",
                    "target_endpoint_mismatch",
                },
            ),
            ("cross_net_waveguide_overlap", {"cross_net_waveguide_overlap"}),
            ("waveguide_obstacle_overlap", {"waveguide_obstacle_overlap"}),
            ("crossing_component_route_overlap", {"crossing_component_route_overlap"}),
        )
        selected_group = next(
            (
                name
                for name, codes in priority_groups
                if any(issue.code in codes for issue in issues)
            ),
            None,
        )
        if selected_group is None:
            return False
        selected_codes = dict(priority_groups)[selected_group]
        selected_issues = [issue for issue in issues if issue.code in selected_codes]
        if selected_group == "crossing_component_route_overlap":
            selected_issues = selected_issues[:1]
        if (
            not selected_issues
            or not repair_config.enabled
            or not hasattr(router, "add_static_cells")
            or not hasattr(router, "ripup_route")
            or not hasattr(router, "route_many_with_repair_and_commit")
        ):
            return False

        repair_net_ids_set: set[int] = set()
        keepout_cells: set[tuple[int, int]] = set()
        for issue in selected_issues:
            repair_net_ids_set.update(_photonic_issue_net_ids(issue))
            keepout_cells.update(_photonic_issue_keepout_cells(issue))
        repair_net_ids = [net_id for net_id in route_order if net_id in repair_net_ids_set]
        attempt: dict[str, object] = {
            "selected_group": selected_group,
            "issue_counts": dict(Counter(issue.code for issue in issues)),
            "selected_issue_counts": dict(Counter(issue.code for issue in selected_issues)),
            "repair_net_ids": [int(net_id) for net_id in repair_net_ids],
            "keepout_cell_count": len(keepout_cells),
        }
        if not repair_net_ids or len(repair_net_ids) > 12:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_repairable_nets_or_too_many"
            attempts.append(attempt)
            return False
        if not keepout_cells:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_keepout_cells"
            attempts.append(attempt)
            return False

        router.add_static_cells(sorted(keepout_cells))
        for net_id in repair_net_ids:
            router.ripup_route(int(net_id))
            route_bookkeeping.clear_route(int(net_id))

        repair_jobs: list[
            tuple[
                int,
                Any,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        opened_by_id: dict[int, list[tuple[int, int]]] = {}
        for net_id in repair_net_ids:
            job = route_jobs_by_id[net_id]
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            repair_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    _routing_endpoint_center_um(job, source=True),
                    _routing_endpoint_center_um(job, source=False),
                )
            )
            opened_by_id[int(job.net_id)] = opened_cells

        raw_repair_result = router.route_many_with_repair_and_commit(
            repair_jobs,
            block_radius_cells,
            commit_radius_cells,
            core_commit_radius_cells,
            int(repair_config.max_rounds),
            int(repair_config.max_victims_per_failure),
            float(repair_config.history_weight),
            int(repair_config.history_increment),
        )
        repair_result = dict(raw_repair_result)
        attempt["router_status"] = str(repair_result.get("status", ""))
        attempt["routed_net_ids"] = [
            int(dict(raw_entry)["net_id"])
            for raw_entry in cast(Iterable[Any], repair_result.get("routes", []))
        ]
        if str(repair_result.get("status", "")) != "routed":
            attempt["status"] = "failed"
            attempts.append(attempt)
            return False

        repaired_net_ids: list[int] = []
        for raw_entry in cast(Iterable[Any], repair_result.get("routes", [])):
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            job = route_jobs_by_id[net_id]
            route_obj = entry["route"]
            _record_route(job, route_obj, opened_by_id[net_id])
            repaired_net_ids.append(net_id)
        if enable_checked_endpoint_correction and repaired_net_ids:
            failed_corrections = _apply_checked_endpoint_corrections_for_net_ids(
                repaired_net_ids,
                record_pipeline_timing=False,
            )
            failed_corrections.extend(
                _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
                    repaired_net_ids,
                    record_pipeline_timing=False,
                )
            )
            attempt["endpoint_correction_failed_net_ids"] = [
                int(net_id) for net_id in failed_corrections
            ]
        attempt["status"] = "routed"
        attempts.append(attempt)
        return True

    def _photonic_repair_failure_preview(
        verification: PhotonicVerificationResult,
    ) -> str:
        lines: list[str] = []
        for issue in verification.issues[:5]:
            details = issue.details or {}
            suffix_parts: list[str] = []
            if "overlap_area_um2" in details:
                suffix_parts.append(f"area={details['overlap_area_um2']}")
            if "overlap_bbox_um" in details:
                suffix_parts.append(f"bbox={details['overlap_bbox_um']}")
            suffix = f" ({', '.join(suffix_parts)})" if suffix_parts else ""
            lines.append(
                f"{issue.code} {issue.net_name or '<unknown>'}: "
                f"{issue.message}{suffix}"
            )
        if len(verification.issues) > 5:
            lines.append(f"... {len(verification.issues) - 5} more")
        return "; ".join(lines)

    def _refresh_realized_crossing_verification() -> list[dict[str, object]]:
        t_refresh_crossings_start = _pipeline_timer_start()
        t_overlap_start = _pipeline_timer_start()
        if enable_internal_photonic_probe_verification:
            _augment_crossing_plan_with_realized_overlaps(
                router=router,
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=route_bookkeeping.records_by_id,
            )
        _record_pipeline_timing("realized_crossing_overlap_augment", t_overlap_start)
        native_crossing_events: list[Any] = []
        if hasattr(router, "crossing_events"):
            t_native_events_start = _pipeline_timer_start()
            try:
                native_crossing_events = list(cast(Iterable[Any], router.crossing_events()))
            except Exception:
                native_crossing_events = []
            crossing_plan_info["native_crossing_events"] = native_crossing_events
            crossing_plan_info["native_crossing_event_count"] = len(native_crossing_events)
            _record_pipeline_timing(
                "realized_crossing_native_events",
                t_native_events_start,
            )
            t_insertion_loss_start = _pipeline_timer_start()
            _augment_insertion_loss_report(
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=route_bookkeeping.records_by_id,
                native_crossing_events=native_crossing_events,
            )
            _record_pipeline_timing(
                "realized_crossing_insertion_loss",
                t_insertion_loss_start,
            )
        t_illegal_crossing_verify_start = _pipeline_timer_start()
        if enable_internal_photonic_probe_verification:
            illegal = _verify_realized_route_intersections(
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=route_bookkeeping.records_by_id,
                realization_grid_spec=realization_grid_spec,
            )
        else:
            illegal = _populate_realized_intersections_from_native_crossing_events(
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=route_bookkeeping.records_by_id,
                native_crossing_events=native_crossing_events,
                realization_grid_spec=realization_grid_spec,
            )
        _record_pipeline_timing(
            "realized_crossing_verify_intersections",
            t_illegal_crossing_verify_start,
        )
        t_realized_insertion_loss_start = _pipeline_timer_start()
        _augment_insertion_loss_report_from_realized_intersections(
            crossing_plan_info=crossing_plan_info,
            routed_records_by_net_id=route_bookkeeping.records_by_id,
        )
        _record_pipeline_timing(
            "realized_crossing_realized_loss",
            t_realized_insertion_loss_start,
        )
        _record_pipeline_timing(
            "realized_crossing_refresh_total",
            t_refresh_crossings_start,
        )
        return illegal

    final_crossing_repair_round_limit = 12
    t_final_verification_block_start = _pipeline_timer_start()
    # Crossing legality is still checked internally because the router owns the
    # crossing event model and can repair/reroute before final realization.
    #
    # The full photonic probe verification is intentionally diagnostic: it was
    # useful while chasing endpoint-correction and crossing-model mismatches,
    # but the normal flow skips this expensive pass unless debug/failure
    # analysis asks for it. The external Python verifier in `routing_flow.py`
    # remains the final GDS/layout-level gate.
    illegal_realized_crossings = _refresh_realized_crossing_verification()
    for _final_repair_round in range(final_crossing_repair_round_limit):
        if not illegal_realized_crossings:
            break
        if not _repair_final_illegal_crossings(illegal_realized_crossings):
            break
        routed_net_records = route_bookkeeping.ordered_records()
        illegal_realized_crossings = _refresh_realized_crossing_verification()
    if not illegal_realized_crossings and enable_internal_photonic_probe_verification:
        final_photonic_verification = _refresh_photonic_verification()
        for _final_photonic_repair_round in range(8):
            if final_photonic_verification.success:
                break
            if not _repair_final_photonic_issues(final_photonic_verification.issues):
                break
            illegal_realized_crossings = _refresh_realized_crossing_verification()
            for _nested_crossing_repair_round in range(final_crossing_repair_round_limit):
                if not illegal_realized_crossings:
                    break
                if not _repair_final_illegal_crossings(illegal_realized_crossings):
                    break
                illegal_realized_crossings = _refresh_realized_crossing_verification()
            routed_net_records = route_bookkeeping.ordered_records()
            if illegal_realized_crossings:
                break
            final_photonic_verification = _refresh_photonic_verification()
        if not illegal_realized_crossings and not final_photonic_verification.success:
            _write_crossing_debug_artifacts(
                debug_path=debug_path if debug_path is not None else Path("build"),
                debug_prefix=debug_prefix,
                crossing_plan_info=crossing_plan_info,
            )
            probe_failure_artifacts = _dump_photonic_probe_failure_artifacts(
                debug_path=debug_path if debug_path is not None else Path("build"),
                debug_prefix=debug_prefix,
                probe_layout=last_photonic_probe_layout,
                verification=final_photonic_verification,
                records=last_photonic_probe_records or route_bookkeeping.ordered_records(),
                router=router,
                realization_grid_spec=realization_grid_spec,
                allow_unchecked_bumps=True,
            )
            crossing_plan_info["photonic_probe_failure_artifacts"] = probe_failure_artifacts
            _record_pipeline_timing(
                "final_verification_block",
                t_final_verification_block_start,
            )
            raise RuntimeError(
                "Final photonic geometry repair failed before realization: "
                f"{final_photonic_verification.error_count} error(s). "
                f"{_photonic_repair_failure_preview(final_photonic_verification)}"
            )
    _record_pipeline_timing(
        "final_verification_block",
        t_final_verification_block_start,
    )
    if not illegal_realized_crossings and not defer_realization:
        t_direct_realization_start = _pipeline_timer_start()
        realize_routed_net_records(
            routed_layout,
            routed_net_records,
            route_width_um=route_width_um,
            route_layer=route_layer,
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=bend_radius_cells,
            crossing_plan_info=crossing_plan_info,
            enable_endpoint_correction=enable_checked_endpoint_correction,
        )
        _record_pipeline_timing("direct_realization", t_direct_realization_start)
        _place_realized_crossing_components(routed_layout, crossing_plan_info)
    elif crossing_plan_info.get("enabled"):
        crossing_plan_info.setdefault("realized_crossing_components", [])
        crossing_plan_info.setdefault("realized_crossing_component_count", 0)
    _write_crossing_debug_artifacts(
        debug_path=debug_path if debug_path is not None else Path("build"),
        debug_prefix=debug_prefix,
        crossing_plan_info=crossing_plan_info,
    )
    if illegal_realized_crossings:
        preview = "; ".join(
            f"{item.get('net_name_a')} x {item.get('net_name_b')} "
            f"at {item.get('point_um')} ({item.get('reason')}, "
            f"margins={item.get('segment_a_margin_um')}/"
            f"{item.get('segment_b_margin_um')}, "
            f"required={item.get('required_margin_um')}, "
            f"grid={item.get('grid_cell')}, "
            f"route_endpoint_dists={item.get('route_endpoint_distance_a_um')}/"
            f"{item.get('route_endpoint_distance_b_um')}, "
            f"port_endpoint_dists={item.get('port_endpoint_distance_a_um')}/"
            f"{item.get('port_endpoint_distance_b_um')})"
            for item in illegal_realized_crossings[:5]
        )
        raise RuntimeError(
            "Illegal realized route crossing(s) after endpoint correction: "
            f"{len(illegal_realized_crossings)} found. {preview}"
        )

    t_debug_artifact_start = _pipeline_timer_start()
    debug_artifacts = build_route_debug_artifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
        obstacle_map=obstacle_map,
        routed_net_records=routed_net_records,
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_cells=bend_radius_cells,
        route_search_summary=summarize_route_search(
            route_timing_buckets,
            route_count=len(route_jobs),
            simple_route_count=simple_route_count,
            repair_count=repair_count,
            astar_elapsed_s=astar_elapsed_s,
        ),
        route_attempt_records=route_attempt_records,
        route_nets_timings_s=route_nets_timings_s,
    )
    debug_artifacts = replace(
        debug_artifacts,
        crossing_plan_info=crossing_plan_info,
    )
    _record_pipeline_timing("debug_artifact_assembly", t_debug_artifact_start)
    if collect_pipeline_timing:
        debug_artifacts = replace(
            debug_artifacts,
            route_nets_timings_s=dict(route_nets_timings_s),
        )
    return routed_layout, debug_artifacts
