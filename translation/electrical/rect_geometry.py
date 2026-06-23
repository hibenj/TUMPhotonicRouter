"""Shared rectangle geometry helpers for electrical metal realization."""

from __future__ import annotations

from collections.abc import Iterable

from .types import BBox, Point

def wire_rects_for_points(
    points: tuple[Point, ...],
    width_um: float,
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
    for start, end in zip(points, points[1:]):
        rects.extend(_segment_rects(start, end, half_width))
    return clean_rects(rects)


def clean_rects(rects: Iterable[BBox]) -> tuple[BBox, ...]:
    """Normalize and compact an equivalent same-net rectangle set."""

    normalized = sorted(
        {
            normalized_rect
            for rect in rects
            if (normalized_rect := _normalize_rect(rect)) is not None
        }
    )
    return _drop_contained_rects(tuple(normalized))


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


def _segment_rects(
    start: Point,
    end: Point,
    half_width: float,
) -> tuple[BBox, ...]:
    sx, sy = start
    ex, ey = end
    if sx == ex and sy == ey:
        return (_point_rect(start, half_width),)
    if sx == ex:
        return (
            _normalize_non_degenerate_rect(
                (
                    sx - half_width,
                    min(sy, ey) - half_width,
                    sx + half_width,
                    max(sy, ey) + half_width,
                )
            ),
        )
    if sy == ey:
        return (
            _normalize_non_degenerate_rect(
                (
                    min(sx, ex) - half_width,
                    sy - half_width,
                    max(sx, ex) + half_width,
                    sy + half_width,
                )
            ),
        )
    via = (ex, sy)
    return (*_segment_rects(start, via, half_width), *_segment_rects(via, end, half_width))


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
