"""Crossing-aware endpoint correction and centerline splicing for the Rust-backed photonic router."""

from __future__ import annotations

import itertools
import logging
import math
from collections.abc import Iterable, Mapping, Sequence
from collections.abc import Iterable as IterableABC
from dataclasses import dataclass, replace
from typing import Any

from photonic_router.config import FlowDiagnostics
from photonic_router.crossings.route_rust_crossing_components import (
    crossing_footprint_polygon_metadata,
    point_um_from_mapping,
)
from photonic_router.records import fallbacks
from photonic_router.records.route_rust_geometry import (
    point_distance_um,
    segment_intersects_crossing_footprint_interior,
    segment_length_um,
    segment_unit_vector,
)
from photonic_router.records.route_rust_records import (
    EndpointCorrectionRouter,
    apply_port_endpoint_corrections,
    centerline_tuple,
    format_port_endpoint_correction_error,
)
from photonic_router.records.route_rust_types import (
    RoutedNetRecord,
    int_or_none,
)

_LOGGER = logging.getLogger(__name__)


def polyline_length_um(points: tuple[tuple[float, float], ...]) -> float:
    """Return the summed length of the straight pieces between consecutive ``points``."""
    total = 0.0
    for start, end in itertools.pairwise(points):
        total += math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
    return total


def kernel_centerline_length_um(router: Any, centerline: Sequence[tuple[float, float]]) -> float:
    """The length of a centerline as the kernel measures it (um).

    The kernel sums the straight pieces like `polyline_length_um` and
    refuses a centerline with fewer than two points, a point that is not
    finite or a piece of length zero. That refusal is not caught: until
    2026-11-24 it was, and such a centerline got the plain sum instead.
    """
    return float(router.centerline_length_um(list(centerline)))


def dedupe_centerline(
    points: tuple[tuple[float, float], ...],
) -> tuple[tuple[float, float], ...]:
    """Remove consecutive repeated points from a centerline."""
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
    for index, (start, end) in enumerate(itertools.pairwise(centerline)):
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
    if any(point_distance_um(existing, point) <= 1.0e-6 for existing in centerline):
        return centerline
    projection = _closest_centerline_projection(centerline, point)
    if projection is None:
        return centerline
    segment_index, _, _, _ = projection
    out = list(centerline[: segment_index + 1])
    out.append((float(point[0]), float(point[1])))
    out.extend(centerline[segment_index + 1 :])
    return dedupe_centerline(tuple(out))


def _centerline_index_near_point(
    centerline: tuple[tuple[float, float], ...],
    point: tuple[float, float],
    *,
    tolerance_um: float = 1.0e-6,
) -> int | None:
    best_index: int | None = None
    best_dist = float("inf")
    for index, existing in enumerate(centerline):
        dist = point_distance_um(existing, point)
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
    return dedupe_centerline(tuple(line[start_index : end_index + 1]))


def _crossing_endpoint_splice_parts(
    *,
    baseline: tuple[tuple[float, float], ...],
    crossing_points: list[tuple[float, float]],
) -> (
    tuple[
        tuple[tuple[float, float], ...],
        tuple[tuple[float, float], ...],
        tuple[tuple[float, float], ...],
        tuple[float, float],
        tuple[float, float],
    ]
    | None
):
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

    prefix = dedupe_centerline(tuple(baseline_with_first_cut[: first_cut_index + 1]))
    suffix = dedupe_centerline(tuple(baseline_with_last_cut[last_cut_index:]))
    if len(prefix) < 2 or len(suffix) < 2:
        return None
    return prefix, middle, suffix, first_cut, last_cut


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
    return point_distance_um(point, anchor) <= 1.0e-6


def legal_crossing_points_by_net_id(
    crossing_plan_info: Mapping[str, object] | None,
) -> dict[int, list[tuple[float, float]]]:
    """Collect the points in um of the legal realized crossings per net id.

    Reads ``realized_intersections`` of the crossing plan info given as a mapping; a
    crossing whose classification starts with ``legal_`` is listed under both of its
    nets. Returns an empty dict if the info is missing or not enabled.
    """
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
        point = point_um_from_mapping(raw_crossing.get("point_um"))
        if point is None:
            continue
        for key in ("net_id_a", "net_id_b"):
            net_id = int_or_none(raw_crossing.get(key))
            if net_id is None:
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
    specs: list[tuple[tuple[float, float], tuple[float, float], tuple[float, float], float]] = []
    for raw_crossing in raw_crossings:
        if not isinstance(raw_crossing, Mapping):
            continue
        net_id_a = int_or_none(raw_crossing.get("net_id_a"))
        net_id_b = int_or_none(raw_crossing.get("net_id_b"))
        if net_id_a is None or net_id_b is None:
            # A crossing without both net ids belongs to no net of this record.
            net_id_a = net_id_b = -1
        if net_id is not None and int(net_id) in (net_id_a, net_id_b):
            continue
        polygon_raw = crossing_footprint_polygon_metadata(raw_crossing)
        if polygon_raw is None or len(polygon_raw) < 4:
            continue
        polygon = [(float(point[0]), float(point[1])) for point in polygon_raw[:4]]
        center = (
            sum(point[0] for point in polygon) / float(len(polygon)),
            sum(point[1] for point in polygon) / float(len(polygon)),
        )
        axis_u = segment_unit_vector((polygon[0], polygon[1]))
        axis_v = segment_unit_vector((polygon[0], polygon[3]))
        if axis_u is None or axis_v is None:
            continue
        half_extent_um = 0.5 * point_distance_um(polygon[0], polygon[1])
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
    for start, end in itertools.pairwise(centerline):
        segment = ((float(start[0]), float(start[1])), (float(end[0]), float(end[1])))
        if segment_length_um(segment) <= 1.0e-9:
            continue
        for center, axis_u, axis_v, half_extent_um in specs:
            if segment_intersects_crossing_footprint_interior(
                segment,
                center=center,
                axis_u=axis_u,
                axis_v=axis_v,
                half_extent_um=half_extent_um,
            ):
                return True
    return False


def primitive_centerline_for_record(
    record: RoutedNetRecord,
    *,
    router: EndpointCorrectionRouter,
    prefer_corrected_baseline: bool = False,
) -> tuple[tuple[float, float], ...]:
    """Return the centerline in um of a record before endpoint correction, without repeated points.

    This is the router's primitive centerline of the route object. The record's
    corrected centerline is used if the router gives none, and first of all with
    ``prefer_corrected_baseline``. Returns an empty tuple if there is neither.
    """
    if prefer_corrected_baseline and record.corrected_centerline_um:
        return dedupe_centerline(record.corrected_centerline_um)
    route_primitive_centerline = getattr(router, "route_primitive_centerline", None)
    if route_primitive_centerline is not None:
        try:
            centerline = centerline_tuple(route_primitive_centerline(record.route_obj))
        except (TypeError, ValueError):
            centerline = ()
        if centerline:
            return dedupe_centerline(centerline)
    if record.corrected_centerline_um:
        return dedupe_centerline(record.corrected_centerline_um)
    return ()


def merge_terminal_corrected_route_centerline(
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
        if (
            _centerline_index_near_point(
                centerline,
                point,
                tolerance_um=stitch_tolerance_um,
            )
            is not None
        ):
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
            if point_distance_um(corrected_route_centerline[0], source_anchor)
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
        if merged and point_distance_um(merged[-1], target_anchor) <= 1.0e-6:
            suffix_start = target_index + 1
        else:
            suffix_start = target_index
        merged.extend(existing_baseline[suffix_start:])

    return dedupe_centerline(tuple(merged))


@dataclass(frozen=True)
class EndpointCorrectionSetup:
    """What every endpoint correction of one run shares."""

    realization_grid_spec: tuple[int, int, float, float, float] | None
    """(width, height, cell size in um, origin x, origin y)."""
    route_width_um: float
    allow_unchecked_bumps: bool
    log_failures: bool
    crossing_plan_info: Mapping[str, object] | None = None
    diagnostics: FlowDiagnostics | None = None
    """None is the default `FlowDiagnostics`."""


@dataclass(frozen=True)
class EndpointCorrectionScope:
    """Which ends of one record are corrected, and from which baseline."""

    correct_source: bool = True
    correct_target: bool = True
    prefer_corrected_baseline: bool = False


@dataclass(frozen=True)
class EndpointClearance:
    """The cells and radii the checked correction of one record tests against."""

    opened_cells: Iterable[tuple[int, int]] = ()
    exempt_cells: Iterable[tuple[int, int]] = ()
    radius_cells: int = 0
    core_radius_cells: int = 0


_Centerline = tuple[tuple[float, float], ...]


def apply_crossing_aware_endpoint_correction_to_record(
    record: RoutedNetRecord,
    *,
    router: EndpointCorrectionRouter,
    crossing_points: list[tuple[float, float]],
    setup: EndpointCorrectionSetup,
    scope: EndpointCorrectionScope | None = None,
    clearance: EndpointClearance | None = None,
) -> RoutedNetRecord:
    """Apply terminal-only endpoint correction without moving route crossings.

    A net without crossings is corrected as a whole
    (`_correct_without_crossings`). A net with crossings keeps the piece
    between its first and its last crossing: only the piece before and the
    piece after are corrected, each checked by the kernel
    (`_checked_splice`), and when no checked result is accepted the record
    gets an `endpoint_correction_error`.

    ``scope`` defaults to both ends, ``clearance`` to no opened cells and
    radius zero.
    """
    scope = scope if scope is not None else EndpointCorrectionScope()
    clearance = clearance if clearance is not None else EndpointClearance()
    if not crossing_points:
        return _correct_without_crossings(record, router=router, setup=setup, scope=scope)
    baseline = primitive_centerline_for_record(
        record,
        router=router,
        prefer_corrected_baseline=scope.prefer_corrected_baseline,
    )
    if len(baseline) < 2:
        return _with_correction_error(
            record, "crossing-aware endpoint correction requires a primitive centerline", setup
        )
    candidate = _checked_splice(
        _CheckedCorrection(record, router, setup, clearance),
        baseline,
        crossing_points,
        scope,
    )
    if candidate is not None:
        return _with_corrected_centerline(record, router, candidate)
    # No unchecked fallback strategy: a crossing net's two terminal
    # segments are corrected the same way a crossing-free net's is, and
    # nothing else. If that did not produce an accepted result, this net
    # fails endpoint correction honestly instead of silently accepting a
    # candidate whose geometry was never actually verified against the real
    # port (see .agent/execplans/2026-08-24-endpoint-correction-cascade-soundness.md's
    # Decision Log for why the previous, unchecked fallback tier was
    # removed).
    return _with_correction_error(
        record, "crossing-aware endpoint correction produced no realizable centerline", setup
    )


def _with_corrected_centerline(
    record: RoutedNetRecord, router: EndpointCorrectionRouter, centerline: _Centerline
) -> RoutedNetRecord:
    """The record with `centerline` as its corrected centerline and that centerline's length."""
    return replace(
        record,
        total_length_um=kernel_centerline_length_um(router, centerline),
        base_total_length_um=(
            record.base_total_length_um
            if record.base_total_length_um is not None
            else float(record.total_length_um)
        ),
        corrected_centerline_um=centerline,
        endpoint_correction_error=None,
    )


def _with_correction_error(
    record: RoutedNetRecord, text: str, setup: EndpointCorrectionSetup
) -> RoutedNetRecord:
    message = format_port_endpoint_correction_error(
        record,
        text,
        realization_grid_spec=setup.realization_grid_spec,
    )
    if setup.log_failures:
        _LOGGER.info("ERROR: " + message)
    return replace(record, endpoint_correction_error=message)


def _correct_without_crossings(
    record: RoutedNetRecord,
    *,
    router: EndpointCorrectionRouter,
    setup: EndpointCorrectionSetup,
    scope: EndpointCorrectionScope,
) -> RoutedNetRecord:
    """The correction of a net that crosses no other net.

    A record that already has a corrected centerline is returned as it is
    when no end is to be corrected, and gets one end corrected again when
    the scope names only one (`_correct_one_end_again`). Otherwise both
    ends are corrected from the routed centerline; when the result runs
    through the footprint of another net's crossing, the record is
    returned unchanged.
    """
    both_ends = scope.correct_source and scope.correct_target
    if record.corrected_centerline_um and not both_ends:
        if not scope.correct_source and not scope.correct_target:
            return record
        return _correct_one_end_again(record, router=router, setup=setup, scope=scope)
    uncorrected_record = replace(
        record,
        corrected_centerline_um=(),
        endpoint_correction_error=None,
    )
    normal_record = apply_port_endpoint_corrections(
        [uncorrected_record],
        router=router,
        realization_grid_spec=setup.realization_grid_spec,
        allow_unchecked_bumps=setup.allow_unchecked_bumps,
        log_failures=setup.log_failures,
    )[0]
    foreign_footprints = _foreign_crossing_footprint_specs(
        setup.crossing_plan_info,
        net_id=int(record.net_id) if record.net_id is not None else None,
        route_width_um=setup.route_width_um,
    )
    if (
        normal_record.corrected_centerline_um
        and normal_record.endpoint_correction_error is None
        and _centerline_intersects_crossing_footprint_specs(
            dedupe_centerline(normal_record.corrected_centerline_um),
            foreign_footprints,
        )
    ):
        return record
    return normal_record


def _correct_one_end_again(
    record: RoutedNetRecord,
    *,
    router: EndpointCorrectionRouter,
    setup: EndpointCorrectionSetup,
    scope: EndpointCorrectionScope,
) -> RoutedNetRecord:
    """One end corrected from the routed centerline, the other kept as the record has it.

    Returns the record unchanged when the kernel has no centerline for it
    or the two centerlines cannot be joined.
    """
    route_baseline = primitive_centerline_for_record(
        record,
        router=router,
        prefer_corrected_baseline=False,
    )
    baseline = primitive_centerline_for_record(
        record,
        router=router,
        prefer_corrected_baseline=True,
    )
    route_port_corrected_centerline = getattr(router, "route_port_corrected_centerline", None)
    if len(route_baseline) < 2 or len(baseline) < 2 or route_port_corrected_centerline is None:
        return record
    try:
        corrected_route = centerline_tuple(
            route_port_corrected_centerline(
                record.route_obj,
                source_port_um=(record.source_port_center_um if scope.correct_source else None),
                target_port_um=(record.target_port_center_um if scope.correct_target else None),
                allow_unchecked_bumps=setup.allow_unchecked_bumps,
            )
        )
    except (TypeError, ValueError):
        corrected_route = ()
    centerline = merge_terminal_corrected_route_centerline(
        existing_baseline=baseline,
        route_baseline=route_baseline,
        corrected_route_centerline=dedupe_centerline(corrected_route),
        freeze_source=not scope.correct_source,
        freeze_target=not scope.correct_target,
    )
    if len(centerline) < 2:
        return record
    return _with_corrected_centerline(record, router, centerline)


def _merge_splice_segments(
    prefix: _Centerline, middle: _Centerline, suffix: _Centerline
) -> _Centerline:
    pieces: list[tuple[float, float]] = []
    for segment in (prefix, middle, suffix):
        for point in segment:
            if not pieces or pieces[-1] != point:
                pieces.append(point)
    return dedupe_centerline(tuple(pieces))


@dataclass(frozen=True)
class _CheckedCorrection:
    """One record's checked correction: the kernel calls and their trace lines."""

    record: RoutedNetRecord
    router: EndpointCorrectionRouter
    setup: EndpointCorrectionSetup
    clearance: EndpointClearance

    @property
    def traced(self) -> bool:
        """Whether the run's diagnostics name this net for the endpoint trace."""
        diagnostics = self.setup.diagnostics
        names = (
            diagnostics if diagnostics is not None else FlowDiagnostics()
        ).trace_endpoint_correction_nets
        record = self.record
        return record.net_name in names or (
            record.net_id is not None and str(int(record.net_id)) in names
        )

    def trace(self, text: str) -> None:
        if self.traced:
            _LOGGER.info(
                f"endpoint_trace net={self.record.net_name} id={self.record.net_id} {text}"
            )

    def realization_accepts(self, centerline: _Centerline) -> bool:
        """Whether the kernel can realize `centerline` with the record's terminal tangents."""
        record = self.record
        realize = getattr(self.router, "realize_centerline_polygon_with_terminal_tangents", None)
        if realize is None:
            return True
        try:
            realize(
                list(centerline),
                float(self.setup.route_width_um),
                record.route_obj,
                source_enabled=record.source_port_center_um is not None,
                target_enabled=record.target_port_center_um is not None,
            )
        except (TypeError, ValueError):
            return False
        return True

    def terminal_segment(
        self,
        segment: _Centerline,
        *,
        source_port_um: tuple[float, float] | None,
        target_port_um: tuple[float, float] | None,
    ) -> _Centerline:
        """`segment` with its two ends moved to the given points, checked by the kernel.

        Empty when the kernel rejects the correction (the fallback
        `endpoint.checked_segment_rejected`), when the segment has fewer
        than two points or when an end point is missing.
        """
        record, clearance = self.record, self.clearance
        centerline_port_corrected_checked = getattr(
            self.router, "centerline_port_corrected_checked", None
        )
        if centerline_port_corrected_checked is None or len(segment) < 2:
            return ()
        if source_port_um is None or target_port_um is None:
            return ()
        described = (
            "mode=checked_terminal_segment "
            f"segment_len={len(segment)} "
            f"segment_start={segment[0] if segment else None} "
            f"segment_end={segment[-1] if segment else None} "
            f"source_port={source_port_um} target_port={target_port_um} "
        )
        try:
            raw = centerline_port_corrected_checked(
                int(record.net_id) if record.net_id is not None else 0,
                list(segment),
                float(self.setup.route_width_um),
                int(clearance.radius_cells),
                int(clearance.core_radius_cells),
                sorted(set(clearance.opened_cells or ())),
                sorted(set(clearance.exempt_cells or ())),
                source_port_um=source_port_um,
                target_port_um=target_port_um,
            )
        except (RuntimeError, TypeError, ValueError) as exc:
            fallbacks.take("endpoint.checked_segment_rejected")
            self.trace(f"{described}status=reject error={exc}")
            return ()
        try:
            correction = dict(raw)
        except (TypeError, ValueError):
            self.trace(f"{described}status=reject error=invalid_result")
            return ()
        centerline = dedupe_centerline(centerline_tuple(correction.get("centerline")))
        self.trace(
            f"{described}status=accept candidate_label={correction.get('candidate_label')} "
            f"corrected_len={len(centerline)}"
        )
        return centerline


def _checked_splice(
    checked: _CheckedCorrection,
    baseline: _Centerline,
    crossing_points: list[tuple[float, float]],
    scope: EndpointCorrectionScope,
) -> _Centerline | None:
    """The centerline with its two terminal pieces corrected and checked, or None.

    The baseline is cut at its first and its last crossing; the piece
    before is corrected to the source port and the piece after to the
    target port, each only where `scope` asks for it. None when the
    baseline cannot be cut, no piece was corrected, an end of the result
    is not on its port or the kernel cannot realize the result.
    """
    record = checked.record
    splice_parts = _crossing_endpoint_splice_parts(
        baseline=baseline,
        crossing_points=crossing_points,
    )
    if splice_parts is None:
        return None
    baseline_prefix, middle, baseline_suffix, _, _ = splice_parts
    checked_prefix: _Centerline = ()
    if scope.correct_source:
        checked_prefix = checked.terminal_segment(
            baseline_prefix,
            source_port_um=record.source_port_center_um,
            target_port_um=baseline_prefix[-1],
        )
    checked_suffix: _Centerline = ()
    if scope.correct_target:
        checked_suffix = checked.terminal_segment(
            baseline_suffix,
            source_port_um=baseline_suffix[0],
            target_port_um=record.target_port_center_um,
        )
    if not (checked_prefix or checked_suffix):
        return None
    candidate = _merge_splice_segments(
        checked_prefix or baseline_prefix,
        middle,
        checked_suffix or baseline_suffix,
    )
    source_ok = not scope.correct_source or _terminal_anchor_matches(
        candidate,
        record.source_port_center_um,
        at_start=True,
    )
    target_ok = not scope.correct_target or _terminal_anchor_matches(
        candidate,
        record.target_port_center_um,
        at_start=False,
    )
    accepts = (
        len(candidate) >= 2 and source_ok and target_ok and checked.realization_accepts(candidate)
    )
    checked.trace(
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
    return candidate if accepts else None
