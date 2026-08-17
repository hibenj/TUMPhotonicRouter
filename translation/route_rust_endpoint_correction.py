"""Crossing-aware endpoint correction and centerline splicing for the Rust-backed photonic router."""

from __future__ import annotations

import math
import os
from collections.abc import Iterable as IterableABC
from dataclasses import replace
from typing import Iterable, Mapping

from translation.route_rust_crossing_components import (
    _crossing_footprint_polygon_metadata,
    _point_um_from_mapping,
)
from translation.route_rust_geometry import (
    _compress_centerline,
    _point_distance_um,
    _segment_intersects_crossing_footprint_interior,
    _segment_length_um,
    _segment_unit_vector,
)
from translation.route_rust_records import (
    EndpointCorrectionRouter,
    _centerline_tuple,
    apply_port_endpoint_corrections,
    build_port_alignment_diagnostics,
    format_port_endpoint_correction_error,
    routed_edge_lengths_from_records,
)
from translation.route_rust_types import RoutedNetRecord, RustRouteDebugArtifacts


def _load_rust_backend():
    from translation.route_rust import _load_rust_backend as route_rust_load_backend

    return route_rust_load_backend()


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
