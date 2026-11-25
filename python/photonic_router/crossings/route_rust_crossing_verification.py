"""Realized-route crossing verification for the Rust-backed photonic router."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from typing import TYPE_CHECKING, Any, NamedTuple, cast

from photonic_router.records.route_rust_geometry import (
    crossing_footprint_half_extent_um,
    crossing_footprint_polygon,
    grid_point_to_physical_um,
    physical_point_to_grid_cell,
    point_distance_um,
    rounded_point,
    rounded_segment,
    segment_bbox_um,
    segment_length_um,
    segment_unit_vector,
    um_bboxes_overlap,
)
from photonic_router.records.route_rust_types import RoutedNetRecord

if TYPE_CHECKING:
    from photonic_router.records.crossing_plan_info import CrossingPlanInfo


def _planned_pairs_from_plan_info(
    crossing_plan_info: CrossingPlanInfo,
) -> set[frozenset[int]] | None:
    """Loaded topology-plan pairs, or None when the mode carries no plan.

    Used to label realized crossings `planned` (contribution 1 reporting):
    None in lidar-pure (plan deliberately withheld), a set -- possibly empty
    -- whenever a plan was built (window, collision, lidar-guided).
    """
    if not crossing_plan_info.events and not crossing_plan_info.event_count:
        return None
    pairs: set[frozenset[int]] = set()
    for raw_event in crossing_plan_info.events:
        event = dict(raw_event)
        if not event.get("loaded"):
            continue
        pairs.add(frozenset((int(cast(int, event["net_id_a"])), int(cast(int, event["net_id_b"])))))
    return pairs


def _planned_flag(
    planned_pairs: set[frozenset[int]] | None, net_id_a: int, net_id_b: int
) -> bool | None:
    if planned_pairs is None:
        return None
    return frozenset((net_id_a, net_id_b)) in planned_pairs


#: The verdict reasons of the Rust predicate `crossing_geometry::crossing_legality`
#: (do-over phase 4 slice B), as `PyPhotonicRouter.crossing_verdicts()` reports them.
RUST_CROSSING_VERDICT_REASONS: frozenset[str] = frozenset(
    {
        "legal",
        "not_perpendicular",
        "margin_short_route_before",
        "margin_short_route_after",
        "margin_short_partner",
        "footprint_blocked",
        "missing_event",
    }
)


def _rust_verdicts_by_pair(
    native_crossing_verdicts: Iterable[object] | None,
) -> dict[frozenset[int], list[dict[str, object]]]:
    """Index `crossing_verdicts()` records by unordered net pair."""
    by_pair: dict[frozenset[int], list[dict[str, object]]] = {}
    if native_crossing_verdicts is None:
        return by_pair
    for raw_verdict in native_crossing_verdicts:
        if not isinstance(raw_verdict, Mapping):
            try:
                raw_verdict = dict(cast(Any, raw_verdict))
            except (TypeError, ValueError):
                continue
        try:
            net_id = int(cast(Any, raw_verdict.get("net_id")))
            partner_net_id = int(cast(Any, raw_verdict.get("partner_net_id")))
            raw_point = cast(Any, raw_verdict.get("point"))
            point = (float(raw_point[0]), float(raw_point[1]))
        except (TypeError, ValueError, IndexError):
            continue
        verdict = dict(raw_verdict)
        verdict["net_id"] = net_id
        verdict["partner_net_id"] = partner_net_id
        verdict["point"] = point
        by_pair.setdefault(frozenset((net_id, partner_net_id)), []).append(verdict)
    return by_pair


def _nearest_rust_verdict(
    verdicts_by_pair: Mapping[frozenset[int], list[dict[str, object]]],
    net_id_a: int,
    net_id_b: int,
    point_um: tuple[float, float],
    *,
    tolerance_um: float,
) -> dict[str, object] | None:
    """The pair's verdict nearest `point_um`, within `tolerance_um` (the 1.5-cell
    tolerance Rust itself uses to match an event to an intersection)."""
    best: dict[str, object] | None = None
    best_distance = float("inf")
    for verdict in verdicts_by_pair.get(frozenset((net_id_a, net_id_b)), ()):
        distance = point_distance_um(point_um, cast(tuple[float, float], verdict["point"]))
        if distance <= tolerance_um and distance < best_distance:
            best = verdict
            best_distance = distance
    return best


def _rust_verdict_fields(
    verdict: Mapping[str, object],
    *,
    net_id_a: int,
    grid_size_um: float,
) -> dict[str, object]:
    """The record fields the verifier reports from a Rust verdict instead of
    computing: legality, reason, perpendicularity, the straight runs of net a
    and net b in um (the smaller side of each), the one required margin
    (`required_margin_cells * grid`), and the verdict itself.

    `verdict["net_id"]` is the net Rust assessed as the route; its runs are
    `route_*_cells`, the partner's `partner_*_cells`.
    """

    def _cells(key: str) -> float:
        try:
            value = float(cast(Any, verdict.get(key)))
        except (TypeError, ValueError):
            return float("inf")
        return value

    route_margin_cells = min(_cells("route_before_cells"), _cells("route_after_cells"))
    partner_margin_cells = min(_cells("partner_before_cells"), _cells("partner_after_cells"))
    if int(cast(Any, verdict.get("net_id"))) == int(net_id_a):
        margin_a_cells, margin_b_cells = route_margin_cells, partner_margin_cells
    else:
        margin_a_cells, margin_b_cells = partner_margin_cells, route_margin_cells
    try:
        required_margin_cells = int(cast(Any, verdict.get("required_margin_cells")))
    except (TypeError, ValueError):
        required_margin_cells = 0
    required_margin_um = float(required_margin_cells) * float(grid_size_um)

    def _um(cells: float) -> float:
        return cells * float(grid_size_um) if math.isfinite(cells) else float("inf")

    return {
        "legal": bool(verdict.get("legal")),
        "reason": str(verdict.get("reason") or ""),
        "perpendicular": bool(verdict.get("perpendicular")),
        "segment_a_margin_um": _um(margin_a_cells),
        "segment_b_margin_um": _um(margin_b_cells),
        "required_margin_um": required_margin_um,
        "rust_verdict": {
            "net_id": verdict.get("net_id"),
            "partner_net_id": verdict.get("partner_net_id"),
            "reason": verdict.get("reason"),
            "margin_side": verdict.get("margin_side"),
            "route_before_cells": verdict.get("route_before_cells"),
            "route_after_cells": verdict.get("route_after_cells"),
            "partner_before_cells": verdict.get("partner_before_cells"),
            "partner_after_cells": verdict.get("partner_after_cells"),
            "required_margin_cells": verdict.get("required_margin_cells"),
            "required_after_cells": verdict.get("required_after_cells"),
            "required_spacing_cells": verdict.get("required_spacing_cells"),
            "nearest_same_route_cells": verdict.get("nearest_same_route_cells"),
            "nearest_registered_cells": verdict.get("nearest_registered_cells"),
            "too_close_other": verdict.get("too_close_other"),
            "footprint_blocked": verdict.get("footprint_blocked"),
            "event_registered": verdict.get("event_registered"),
            "margins_from_grid": verdict.get("margins_from_grid"),
        },
    }


def _rounded_margin_um(margin_um: float) -> float | None:
    """A margin rounded for the record; `None` for an unbounded run (a crossing
    in the last segment of a search move owes its straight-after cells to the
    following moves, which Rust reports as an infinite run)."""
    return round(float(margin_um), 6) if math.isfinite(margin_um) else None


def _unified_required_margin_um(
    crossing_plan_info: CrossingPlanInfo,
    grid_size_um: float,
) -> float:
    """The one margin of the unified rule, `(half_size + bend_runout) * grid`,
    for records no Rust verdict covers (the verdict's own
    `required_margin_cells` wins where one exists)."""
    half_size_cells = max(0, int(crossing_plan_info.crossing_half_size_cells or 0))
    runout_cells = max(0, int(crossing_plan_info.bend_runout_cells_per_crossing or 0))
    return float(half_size_cells + runout_cells) * float(grid_size_um)


_PointUm = tuple[float, float]
_SegmentUm = tuple[_PointUm, _PointUm]


class _GridFrame(NamedTuple):
    """The routing grid's cell size and origin (um): where a grid point lies on the chip."""

    grid_size_um: float
    origin_x_um: float
    origin_y_um: float

    def point_um(self, grid_point: tuple[float, float]) -> _PointUm:
        return grid_point_to_physical_um(
            grid_point,
            grid_size_um=self.grid_size_um,
            origin_x_um=self.origin_x_um,
            origin_y_um=self.origin_y_um,
        )


class _NativeEvent(NamedTuple):
    """One crossing the search accepted, as `router.crossing_events()` gives it."""

    net_id_a: int
    net_id_b: int
    #: the crossing point in grid coordinates
    grid_point: tuple[float, float]
    #: the straight pieces of the two routes that cross, in um
    segment_a: _SegmentUm
    segment_b: _SegmentUm
    reservation_cells: list[object]


def _native_event(raw_event: Mapping[str, Any], frame: _GridFrame) -> _NativeEvent:
    """Read one event of the kernel.

    Raises `KeyError`, `TypeError` or `ValueError` for an event that is not
    as the kernel writes it (two net ids, a point, two segments of two
    points each): a crossing must not be dropped silently.
    """
    point_x, point_y = raw_event["point"]
    (ax0, ay0), (ax1, ay1) = raw_event["route_segment"]
    (bx0, by0), (bx1, by1) = raw_event["partner_segment"]
    return _NativeEvent(
        net_id_a=int(raw_event["net_id"]),
        net_id_b=int(raw_event["partner_net_id"]),
        grid_point=(float(point_x), float(point_y)),
        segment_a=(
            frame.point_um((float(ax0), float(ay0))),
            frame.point_um((float(ax1), float(ay1))),
        ),
        segment_b=(
            frame.point_um((float(bx0), float(by0))),
            frame.point_um((float(bx1), float(by1))),
        ),
        reservation_cells=list(raw_event.get("reservation_cells", [])),
    )


def _event_point_um(event: _NativeEvent, frame: _GridFrame) -> _PointUm:
    """The crossing point in um.

    The point is taken as it stands when it already lies within one grid
    cell of both segments' boxes in um, otherwise as a grid point.
    """
    as_given = event.grid_point
    box = segment_bbox_um((as_given, as_given))
    if um_bboxes_overlap(
        box, segment_bbox_um(event.segment_a), eps=frame.grid_size_um
    ) and um_bboxes_overlap(box, segment_bbox_um(event.segment_b), eps=frame.grid_size_um):
        return as_given
    return frame.point_um(event.grid_point)


def _event_record(
    event: _NativeEvent,
    point_um: _PointUm,
    *,
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    planned_pairs: set[frozenset[int]] | None,
    footprint_half_um: float,
    required_margin_um: float,
    frame: _GridFrame,
) -> dict[str, object]:
    """The realized-intersection record of one event, as a legal crossing without a verdict yet."""
    segment_a, segment_b = event.segment_a, event.segment_b
    record_a = routed_records_by_net_id.get(event.net_id_a)
    record_b = routed_records_by_net_id.get(event.net_id_b)
    margin_a = min(
        point_distance_um(point_um, segment_a[0]), point_distance_um(point_um, segment_a[1])
    )
    margin_b = min(
        point_distance_um(point_um, segment_b[0]), point_distance_um(point_um, segment_b[1])
    )
    axis_u = segment_unit_vector(segment_a)
    axis_v = segment_unit_vector(segment_b)
    footprint_polygon: list[tuple[float, float]] = []
    if axis_u is not None and axis_v is not None and footprint_half_um > 0.0:
        footprint_polygon = crossing_footprint_polygon(
            center=point_um,
            axis_u=axis_u,
            axis_v=axis_v,
            half_extent_um=footprint_half_um,
        )
    grid_cell = physical_point_to_grid_cell(
        point_um,
        grid_size_um=frame.grid_size_um,
        origin_x_um=frame.origin_x_um,
        origin_y_um=frame.origin_y_um,
    )
    return {
        "net_id_a": event.net_id_a,
        "net_id_b": event.net_id_b,
        "net_name_a": record_a.net_name if record_a is not None else str(event.net_id_a),
        "net_name_b": record_b.net_name if record_b is not None else str(event.net_id_b),
        "point_um": rounded_point(point_um),
        "grid_cell": ([int(grid_cell[0]), int(grid_cell[1])] if grid_cell is not None else None),
        "segment_a_um": rounded_segment(segment_a),
        "segment_b_um": rounded_segment(segment_b),
        "segment_a_length_um": round(float(segment_length_um(segment_a)), 6),
        "segment_b_length_um": round(float(segment_length_um(segment_b)), 6),
        "segment_a_margin_um": round(float(margin_a), 6),
        "segment_b_margin_um": round(float(margin_b), 6),
        "required_margin_um": round(float(required_margin_um), 6),
        "search_required_margin_um": round(float(required_margin_um), 6),
        "crossing_footprint_half_um": round(float(footprint_half_um), 6),
        "crossing_footprint_um": round(2.0 * float(footprint_half_um), 6),
        "crossing_footprint_polygon_um": [rounded_point(point) for point in footprint_polygon],
        "crossing_footprint_blockers": [],
        "expected_pair": True,
        "planned": _planned_flag(planned_pairs, event.net_id_a, event.net_id_b),
        "perpendicular": True,
        "classification": "legal_native_crossing",
        "source": "native_crossing_events",
        "reservation_cells": event.reservation_cells,
    }


def _is_illegal(record: Mapping[str, object]) -> bool:
    return str(record.get("classification", "")).startswith("illegal_")


def populate_realized_intersections_from_native_crossing_events(
    *,
    crossing_plan_info: CrossingPlanInfo,
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    native_crossing_events: Iterable[object],
    realization_grid_spec: tuple[int, int, float, float, float],
    native_crossing_verdicts: Iterable[object],
) -> list[dict[str, object]]:
    """Populate realized crossing metadata from A*-accepted native events.

    This is the normal fast path. A* is responsible for accepting only legal
    crossing moves; this function converts those accepted events into the
    physical metadata needed for crossing component placement and for the final
    external Python geometry verifier.

    Each event carries the Rust predicate's verdict for its point
    (``native_crossing_verdicts`` = ``router.crossing_verdicts()``):
    classification, reason, perpendicularity and margins come from it, and a
    verdict Rust recorded for a point no event covers is reported as its own
    record. The returned list holds the records the verdicts call illegal.

    Raises `KeyError`, `TypeError` or `ValueError` for an event that is not
    as the kernel writes it (`_native_event`), and `RuntimeError` for an
    event without a verdict: every crossing has to be validated by the
    kernel (owner, 2026-11-25).
    """
    if not crossing_plan_info.enabled:
        crossing_plan_info.realized_intersections = []
        crossing_plan_info.realized_intersection_count = 0
        crossing_plan_info.illegal_realized_crossings = []
        crossing_plan_info.illegal_realized_crossing_count = 0
        return []

    _width, _height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    frame = _GridFrame(float(grid_size_um), float(origin_x_um), float(origin_y_um))
    footprint_half_um = crossing_footprint_half_extent_um(
        crossing_plan_info=crossing_plan_info,
        grid_size_um=frame.grid_size_um,
    )
    required_margin_um = _unified_required_margin_um(crossing_plan_info, frame.grid_size_um)
    verdicts_by_pair = _rust_verdicts_by_pair(native_crossing_verdicts)
    matched_verdict_ids: set[int] = set()
    planned_pairs = _planned_pairs_from_plan_info(crossing_plan_info)
    realized: list[dict[str, object]] = []
    seen: set[tuple[int, int, int, int]] = set()
    for raw_event in native_crossing_events:
        event = _native_event(cast(Mapping[str, Any], raw_event), frame)
        point_um = _event_point_um(event, frame)
        key = (
            min(event.net_id_a, event.net_id_b),
            max(event.net_id_a, event.net_id_b),
            round(point_um[0] * 1_000_000),
            round(point_um[1] * 1_000_000),
        )
        if key in seen:
            continue
        seen.add(key)
        record = _event_record(
            event,
            point_um,
            routed_records_by_net_id=routed_records_by_net_id,
            planned_pairs=planned_pairs,
            footprint_half_um=footprint_half_um,
            required_margin_um=required_margin_um,
            frame=frame,
        )
        rust_verdict = _nearest_rust_verdict(
            verdicts_by_pair,
            event.net_id_a,
            event.net_id_b,
            point_um,
            tolerance_um=1.5 * frame.grid_size_um,
        )
        if rust_verdict is not None:
            matched_verdict_ids.add(id(rust_verdict))
            _apply_rust_verdict_to_native_record(
                record,
                rust_verdict,
                net_id_a=event.net_id_a,
                grid_size_um=frame.grid_size_um,
            )
        else:
            raise RuntimeError(
                f"the crossing of {record['net_name_a']} and {record['net_name_b']} at "
                f"({point_um[0]:.3f}, {point_um[1]:.3f}) um has no verdict of the kernel: "
                "a crossing the search did not validate is not accepted"
            )
        realized.append(record)

    # Verdicts for points no event covers: Rust assessed them, so they are
    # reported as their own records (legal or not) rather than dropped.
    for pair_verdicts in verdicts_by_pair.values():
        for verdict in pair_verdicts:
            if id(verdict) in matched_verdict_ids:
                continue
            verdict_record = _native_record_from_rust_verdict(
                verdict,
                routed_records_by_net_id=routed_records_by_net_id,
                planned_pairs=planned_pairs,
                footprint_half_um=footprint_half_um,
                grid_size_um=frame.grid_size_um,
                origin_x_um=frame.origin_x_um,
                origin_y_um=frame.origin_y_um,
            )
            if verdict_record is not None:
                realized.append(verdict_record)

    illegal = [record for record in realized if _is_illegal(record)]
    crossing_plan_info.realized_intersections = realized
    crossing_plan_info.realized_intersection_count = len(realized)
    crossing_plan_info.routes_missing_corrected_centerline = []
    crossing_plan_info.routes_missing_corrected_centerline_count = 0
    crossing_plan_info.ignored_endpoint_access_intersections = []
    crossing_plan_info.ignored_endpoint_access_intersection_count = 0
    crossing_plan_info.illegal_realized_crossings = illegal
    crossing_plan_info.illegal_realized_crossing_count = len(illegal)
    return illegal


def _apply_rust_verdict_to_native_record(
    record: dict[str, object],
    verdict: Mapping[str, object],
    *,
    net_id_a: int,
    grid_size_um: float,
) -> None:
    """Overwrite a native event record's legality fields with the Rust verdict's."""
    fields = _rust_verdict_fields(verdict, net_id_a=net_id_a, grid_size_um=grid_size_um)
    record["perpendicular"] = fields["perpendicular"]
    record["segment_a_margin_um"] = _rounded_margin_um(cast(float, fields["segment_a_margin_um"]))
    record["segment_b_margin_um"] = _rounded_margin_um(cast(float, fields["segment_b_margin_um"]))
    record["required_margin_um"] = round(float(cast(float, fields["required_margin_um"])), 6)
    record["search_required_margin_um"] = record["required_margin_um"]
    record["legality_source"] = "rust_crossing_verdict"
    record["rust_verdict"] = fields["rust_verdict"]
    if bool(fields["legal"]):
        record["classification"] = "legal_native_crossing"
        record.pop("reason", None)
    else:
        record["classification"] = "illegal_unexpected_crossing"
        record["reason"] = str(fields["reason"]) or "crossing_footprint_invalid"


def _native_record_from_rust_verdict(
    verdict: Mapping[str, object],
    *,
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    planned_pairs: set[frozenset[int]] | None,
    footprint_half_um: float,
    grid_size_um: float,
    origin_x_um: float,
    origin_y_um: float,
) -> dict[str, object] | None:
    """A realized-intersection record for a Rust verdict no native event covers."""
    try:
        net_id_a = int(cast(Any, verdict.get("net_id")))
        net_id_b = int(cast(Any, verdict.get("partner_net_id")))
        point_um = cast(tuple[float, float], verdict["point"])
    except (TypeError, ValueError, KeyError):
        return None

    def _segment_um(
        raw: object,
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        if not isinstance(raw, (tuple, list)) or len(raw) != 2:
            return None
        try:
            return (
                grid_point_to_physical_um(
                    (float(raw[0][0]), float(raw[0][1])),
                    grid_size_um=grid_size_um,
                    origin_x_um=origin_x_um,
                    origin_y_um=origin_y_um,
                ),
                grid_point_to_physical_um(
                    (float(raw[1][0]), float(raw[1][1])),
                    grid_size_um=grid_size_um,
                    origin_x_um=origin_x_um,
                    origin_y_um=origin_y_um,
                ),
            )
        except (TypeError, ValueError, IndexError):
            return None

    segment_a = _segment_um(verdict.get("route_segment"))
    segment_b = _segment_um(verdict.get("partner_segment"))
    record_a = routed_records_by_net_id.get(net_id_a)
    record_b = routed_records_by_net_id.get(net_id_b)
    grid_cell = physical_point_to_grid_cell(
        point_um,
        grid_size_um=grid_size_um,
        origin_x_um=origin_x_um,
        origin_y_um=origin_y_um,
    )
    footprint_polygon: list[tuple[float, float]] = []
    axis_u = segment_unit_vector(segment_a) if segment_a is not None else None
    axis_v = segment_unit_vector(segment_b) if segment_b is not None else None
    if axis_u is not None and axis_v is not None and footprint_half_um > 0.0:
        footprint_polygon = crossing_footprint_polygon(
            center=point_um,
            axis_u=axis_u,
            axis_v=axis_v,
            half_extent_um=footprint_half_um,
        )
    record: dict[str, object] = {
        "net_id_a": net_id_a,
        "net_id_b": net_id_b,
        "net_name_a": record_a.net_name if record_a is not None else str(net_id_a),
        "net_name_b": record_b.net_name if record_b is not None else str(net_id_b),
        "point_um": rounded_point(point_um),
        "grid_cell": ([int(grid_cell[0]), int(grid_cell[1])] if grid_cell is not None else None),
        "segment_a_um": rounded_segment(segment_a) if segment_a is not None else None,
        "segment_b_um": rounded_segment(segment_b) if segment_b is not None else None,
        "crossing_footprint_half_um": round(float(footprint_half_um), 6),
        "crossing_footprint_um": round(2.0 * float(footprint_half_um), 6),
        "crossing_footprint_polygon_um": [rounded_point(point) for point in footprint_polygon],
        "crossing_footprint_blockers": [],
        "expected_pair": True,
        "planned": _planned_flag(planned_pairs, net_id_a, net_id_b),
        "source": "native_crossing_verdicts",
        "reservation_cells": [],
    }
    _apply_rust_verdict_to_native_record(
        record,
        verdict,
        net_id_a=net_id_a,
        grid_size_um=grid_size_um,
    )
    return record
