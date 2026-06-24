"""Shared rectangle geometry helpers for electrical metal realization."""

from __future__ import annotations

from collections.abc import Iterable

from .types import BBox, Point


def wire_rects_for_points(
    points: tuple[Point, ...],
    width_um: float,
    *,
    trim_bends: bool = True,
) -> tuple[BBox, ...]:
    """Return Manhattan wire rectangles for a centerline path.

    Nonzero segments already cover their endpoints. Avoiding separate vertex
    squares keeps the pre-union rectangle set compact while preserving physical
    connectivity at bends.
    """

    points = _simplify_manhattan_points(_dedupe_points(points))
    if not points:
        return ()
    half_width = width_um / 2.0
    if len(points) == 1:
        return (_point_rect(points[0], half_width),)
    rects: list[BBox] = []
    directions = tuple(
        _point_direction(start, end)
        for start, end in zip(points, points[1:])
    )
    for index, (start, end) in enumerate(zip(points, points[1:])):
        trim_end = (
            trim_bends
            and index + 1 < len(directions)
            and directions[index] != directions[index + 1]
        )
        rects.extend(
            _segment_rects(
                start,
                end,
                half_width,
                trim_end_um=(2.0 * half_width) if trim_end else 0.0,
            )
        )
    return clean_rects(rects)


def clip_manhattan_path_at_first_bbox_entry(
    points: tuple[Point, ...],
    bbox: BBox,
) -> tuple[Point, ...]:
    """Trim a path to stop where it first enters ``bbox``."""

    if len(points) < 2:
        return points
    if _point_in_bbox(points[0], bbox):
        return clip_manhattan_path_start_at_bbox(points, bbox)
    for index, (start, end) in enumerate(zip(points, points[1:])):
        if _point_in_bbox(start, bbox):
            return points[: index + 1]
        if _point_in_bbox(end, bbox):
            boundary = _manhattan_segment_bbox_intersection(start, end, bbox)
            return (*points[: index + 1], boundary)
    return points


def clip_manhattan_path_start_at_bbox(
    points: tuple[Point, ...],
    bbox: BBox,
) -> tuple[Point, ...]:
    """Trim a path starting inside ``bbox`` to the first bbox boundary exit."""

    if len(points) < 2 or not _point_in_bbox(points[0], bbox):
        return points
    index = 1
    while index < len(points) and _point_in_bbox(points[index], bbox):
        index += 1
    if index >= len(points):
        return points[-1:]
    boundary = _manhattan_segment_bbox_intersection(
        points[index],
        points[index - 1],
        bbox,
    )
    return (boundary, *points[index:])


def clean_rects(rects: Iterable[BBox]) -> tuple[BBox, ...]:
    """Normalize and compact an equivalent same-net rectangle set."""

    normalized = sorted(
        {
            normalized_rect
            for rect in rects
            if (normalized_rect := _normalize_rect(rect)) is not None
        }
    )
    without_contained = _drop_contained_rects(tuple(normalized))
    return _drop_union_redundant_rects(without_contained)


def _point_in_bbox(point: Point, bbox: BBox) -> bool:
    x, y = point
    xmin, ymin, xmax, ymax = bbox
    return xmin <= x <= xmax and ymin <= y <= ymax


def _manhattan_segment_bbox_intersection(
    outside: Point,
    inside: Point,
    bbox: BBox,
) -> Point:
    """Return where a Manhattan segment from outside to inside crosses bbox."""

    x0, y0 = outside
    x1, y1 = inside
    xmin, ymin, xmax, ymax = bbox
    if x0 == x1:
        if y0 < ymin <= y1:
            return (x0, ymin)
        if y0 > ymax >= y1:
            return (x0, ymax)
    if y0 == y1:
        if x0 < xmin <= x1:
            return (xmin, y0)
        if x0 > xmax >= x1:
            return (xmax, y0)
    return inside


def disjoint_union_rects(rects: Iterable[BBox]) -> tuple[BBox, ...]:
    """Return a deterministic non-overlapping rectangle cover of the same union."""

    compact = clean_rects(rects)
    if len(compact) < 2:
        return compact
    x_edges = sorted({rect[0] for rect in compact} | {rect[2] for rect in compact})
    strips: list[BBox] = []
    for left, right in zip(x_edges, x_edges[1:]):
        if right <= left:
            continue
        intervals = [
            (rect[1], rect[3])
            for rect in compact
            if rect[0] < right and rect[2] > left
        ]
        for bottom, top in _merged_intervals(intervals):
            strips.append((left, bottom, right, top))
    return _merge_adjacent_x_strips(tuple(strips))


def rect_area(rect: BBox) -> float:
    return max(0.0, rect[2] - rect[0]) * max(0.0, rect[3] - rect[1])


def union_rect_area(rects: Iterable[BBox]) -> float:
    """Return the exact area of the union of axis-aligned rectangles."""

    normalized = tuple(
        normalized_rect
        for rect in rects
        if (normalized_rect := _normalize_rect(rect)) is not None
    )
    if not normalized:
        return 0.0
    x_edges = sorted({rect[0] for rect in normalized} | {rect[2] for rect in normalized})
    area = 0.0
    for left, right in zip(x_edges, x_edges[1:]):
        if right <= left:
            continue
        intervals = [
            (rect[1], rect[3])
            for rect in normalized
            if rect[0] < right and rect[2] > left
        ]
        if not intervals:
            continue
        area += (right - left) * _merged_interval_length(intervals)
    return area


def _drop_contained_rects(rects: tuple[BBox, ...]) -> tuple[BBox, ...]:
    kept: list[BBox] = []
    for index, rect in enumerate(rects):
        if any(
            other_index != index and _rect_contains(other, rect)
            for other_index, other in enumerate(rects)
        ):
            continue
        kept.append(rect)
    return tuple(kept)


def _drop_union_redundant_rects(rects: tuple[BBox, ...]) -> tuple[BBox, ...]:
    kept = list(rects)
    index = 0
    while index < len(kept):
        without_candidate = tuple(
            rect
            for other_index, rect in enumerate(kept)
            if other_index != index
        )
        if _same_area(
            union_rect_area(kept),
            union_rect_area(without_candidate),
        ):
            kept.pop(index)
            continue
        index += 1
    return tuple(kept)


def _same_area(left: float, right: float) -> bool:
    return abs(left - right) <= 1e-9


def _merged_interval_length(intervals: Iterable[tuple[float, float]]) -> float:
    return sum(end - start for start, end in _merged_intervals(intervals))


def _merged_intervals(
    intervals: Iterable[tuple[float, float]],
) -> tuple[tuple[float, float], ...]:
    sorted_intervals = sorted(
        (start, end)
        for start, end in intervals
        if end > start
    )
    if not sorted_intervals:
        return ()
    merged: list[tuple[float, float]] = []
    current_start, current_end = sorted_intervals[0]
    for start, end in sorted_intervals[1:]:
        if start <= current_end:
            current_end = max(current_end, end)
            continue
        merged.append((current_start, current_end))
        current_start, current_end = start, end
    merged.append((current_start, current_end))
    return tuple(merged)


def _merge_adjacent_x_strips(rects: tuple[BBox, ...]) -> tuple[BBox, ...]:
    active: dict[tuple[float, float], BBox] = {}
    merged: list[BBox] = []
    for left, bottom, right, top in sorted(rects):
        key = (bottom, top)
        previous = active.get(key)
        if previous is not None and previous[2] == left:
            active[key] = (previous[0], bottom, right, top)
            continue
        if previous is not None:
            merged.append(previous)
        active[key] = (left, bottom, right, top)
    merged.extend(active.values())
    return tuple(sorted(merged))


def _segment_rects(
    start: Point,
    end: Point,
    half_width: float,
    *,
    trim_end_um: float = 0.0,
) -> tuple[BBox, ...]:
    sx, sy = start
    ex, ey = end
    if sx == ex and sy == ey:
        return (_point_rect(start, half_width),)
    if sx == ex:
        trimmed_end_y = _trim_axis_endpoint(sy, ey, trim_end_um)
        return (
            _normalize_non_degenerate_rect(
                (
                    sx - half_width,
                    min(sy, trimmed_end_y) - half_width,
                    sx + half_width,
                    max(sy, trimmed_end_y) + half_width,
                )
            ),
        )
    if sy == ey:
        trimmed_end_x = _trim_axis_endpoint(sx, ex, trim_end_um)
        return (
            _normalize_non_degenerate_rect(
                (
                    min(sx, trimmed_end_x) - half_width,
                    sy - half_width,
                    max(sx, trimmed_end_x) + half_width,
                    sy + half_width,
                )
            ),
        )
    via = (ex, sy)
    return (
        *_segment_rects(start, via, half_width),
        *_segment_rects(via, end, half_width, trim_end_um=trim_end_um),
    )


def _trim_axis_endpoint(start: float, end: float, trim_end_um: float) -> float:
    length = abs(end - start)
    if trim_end_um <= 0.0 or length <= trim_end_um:
        return end
    if end > start:
        return end - trim_end_um
    return end + trim_end_um


def _point_rect(point: Point, half_width: float) -> BBox:
    x, y = point
    return _normalize_non_degenerate_rect(
        (x - half_width, y - half_width, x + half_width, y + half_width)
    )


def _normalize_rect(rect: BBox) -> BBox | None:
    xmin, ymin, xmax, ymax = rect
    if xmax < xmin:
        xmin, xmax = xmax, xmin
    if ymax < ymin:
        ymin, ymax = ymax, ymin
    if xmax == xmin or ymax == ymin:
        return None
    return (xmin, ymin, xmax, ymax)


def _normalize_non_degenerate_rect(rect: BBox) -> BBox:
    normalized = _normalize_rect(rect)
    if normalized is None:
        raise ValueError("Expected a non-degenerate rectangle")
    return normalized


def _rect_contains(outer: BBox, inner: BBox) -> bool:
    return (
        outer[0] <= inner[0]
        and outer[1] <= inner[1]
        and outer[2] >= inner[2]
        and outer[3] >= inner[3]
    )


def _dedupe_points(
    points: tuple[Point, ...],
) -> tuple[Point, ...]:
    deduped: list[Point] = []
    for point in points:
        if deduped and deduped[-1] == point:
            continue
        deduped.append(point)
    return tuple(deduped)


def _simplify_manhattan_points(
    points: tuple[Point, ...],
) -> tuple[Point, ...]:
    if len(points) <= 2:
        return points
    simplified: list[Point] = [points[0]]
    previous_direction = _point_direction(points[0], points[1])
    for index in range(1, len(points) - 1):
        current_direction = _point_direction(points[index], points[index + 1])
        if current_direction != previous_direction:
            simplified.append(points[index])
            previous_direction = current_direction
    simplified.append(points[-1])
    return tuple(simplified)


def _point_direction(
    start: Point,
    end: Point,
) -> tuple[int, int]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if dx != 0:
        return (1 if dx > 0 else -1, 0)
    if dy != 0:
        return (0, 1 if dy > 0 else -1)
    return (0, 0)
