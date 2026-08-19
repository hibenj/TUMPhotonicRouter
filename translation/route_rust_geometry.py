"""Pure geometry and grid-cell math for the Rust-backed photonic router."""

from __future__ import annotations

import json
import math
import re
from typing import Any, Iterable, Mapping, cast

from photonic_router.static_obstacle_builder import floor_snap_to_grid
from translation.route_rust_types import RoutedNetRecord

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
        floor_snap_to_grid(x, float(origin_x_um), float(grid_size_um)),
        floor_snap_to_grid(y, float(origin_y_um), float(grid_size_um)),
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
