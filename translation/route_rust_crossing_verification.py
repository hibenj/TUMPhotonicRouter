"""Realized-route crossing verification for the Rust-backed photonic router."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from typing import TYPE_CHECKING, Any, cast

from translation.route_rust_geometry import (
    _collinear_segment_overlap_with_params,
    _convex_polygons_overlap_with_area,
    _crossing_footprint_blockers,
    _crossing_footprint_half_extent_um,
    _crossing_footprint_polygon,
    _grid_cell_neighborhood,
    _grid_point_to_physical_um,
    _physical_point_to_grid_cell,
    _point_distance_um,
    _point_near_record_port_endpoint,
    _point_near_route_endpoint,
    _point_record_port_endpoint_distance_um,
    _point_route_endpoint_distance_um,
    _record_centerline_um,
    _rounded_point,
    _rounded_segment,
    _route_segments_from_waypoints,
    _route_waypoints_from_obj,
    _segment_bbox_um,
    _segment_intersection_with_params,
    _segment_length_um,
    _segment_unit_vector,
    _segments_are_perpendicular,
    _um_bboxes_overlap,
)
from translation.route_rust_types import RoutedNetRecord

if TYPE_CHECKING:
    from translation.routing.crossing_plan_info import CrossingPlanInfo


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
        distance = _point_distance_um(point_um, cast(tuple[float, float], verdict["point"]))
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


def _verify_realized_route_intersections(
    *,
    crossing_plan_info: CrossingPlanInfo,
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    realization_grid_spec: tuple[int, int, float, float, float],
    native_crossing_verdicts: Iterable[object] | None = None,
    unified_legality: bool = False,
) -> list[dict[str, object]]:
    """Classify every realized route intersection.

    Legacy (``unified_legality`` off): the classification is computed here from
    the geometry (perpendicularity, footprint straightness, footprint blockers).
    Unified (on): the classification is the Rust predicate's verdict for the
    intersection (``native_crossing_verdicts`` = ``router.crossing_verdicts()``),
    matched by net pair and point; an intersection Rust never assessed is
    ``missing_event``. The geometric exemptions (endpoint access, shared
    segment endpoints), the collinear-overlap check and the footprint-polygon
    overlap check are geometry, not legality, and run in both modes.
    """
    if not crossing_plan_info.enabled:
        crossing_plan_info.realized_intersections = []
        crossing_plan_info.illegal_realized_crossings = []
        crossing_plan_info.illegal_realized_crossing_count = 0
        return []

    _width, _height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    footprint_half_um = _crossing_footprint_half_extent_um(
        crossing_plan_info=crossing_plan_info,
        grid_size_um=float(grid_size_um),
    )
    search_required_margin_um = footprint_half_um + float(grid_size_um) * int(
        crossing_plan_info.bend_runout_cells_per_crossing or 0
    )
    required_margin_um = footprint_half_um
    if unified_legality:
        # One margin: the Rust rule's `(half_size + bend_runout) * grid`,
        # reported as both the required and the search margin.
        required_margin_um = _unified_required_margin_um(crossing_plan_info, float(grid_size_um))
        search_required_margin_um = required_margin_um
    verdicts_by_pair = _rust_verdicts_by_pair(
        native_crossing_verdicts if unified_legality else None
    )
    verdict_tolerance_um = 1.5 * float(grid_size_um)
    allowed_pairs: set[frozenset[int]] = set()
    net_names: dict[int, str] = {}
    for raw_event in crossing_plan_info.events:
        event = dict(raw_event)
        if not event.get("loaded"):
            continue
        net_id_a = int(cast(int, event["net_id_a"]))
        net_id_b = int(cast(int, event["net_id_b"]))
        allowed_pairs.add(frozenset((net_id_a, net_id_b)))
        net_names[net_id_a] = str(event.get("net_name_a", net_id_a))
        net_names[net_id_b] = str(event.get("net_name_b", net_id_b))

    planned_pairs = _planned_pairs_from_plan_info(crossing_plan_info)

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
        net_id: {(int(x), int(y)) for x, y in record.opened_cells} for net_id, record in records
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
                        overlap_length_um = _segment_length_um((overlap_start, overlap_end))
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
                            "planned": _planned_flag(planned_pairs, net_id_a, net_id_b),
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
                        if (
                            _point_near_route_endpoint(
                                point,
                                centerlines_by_id.get(net_id_a, ()),
                                tolerance_um=port_access_tolerance_um,
                            )
                            or _point_near_route_endpoint(
                                point,
                                centerlines_by_id.get(net_id_b, ()),
                                tolerance_um=port_access_tolerance_um,
                            )
                            or _point_near_record_port_endpoint(
                                point,
                                record_a,
                                tolerance_um=port_access_tolerance_um,
                            )
                            or _point_near_record_port_endpoint(
                                point,
                                record_b,
                                tolerance_um=port_access_tolerance_um,
                            )
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
                        if (
                            _point_near_route_endpoint(
                                point,
                                centerlines_by_id.get(net_id_a, ()),
                                tolerance_um=endpoint_margin_tolerance_um,
                            )
                            or _point_near_route_endpoint(
                                point,
                                centerlines_by_id.get(net_id_b, ()),
                                tolerance_um=endpoint_margin_tolerance_um,
                            )
                            or _point_near_record_port_endpoint(
                                point,
                                record_a,
                                tolerance_um=endpoint_margin_tolerance_um,
                            )
                            or _point_near_record_port_endpoint(
                                point,
                                record_b,
                                tolerance_um=endpoint_margin_tolerance_um,
                            )
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
                        ) or nearby_cells.intersection(opened_cells_by_id.get(net_id_b, set())):
                            point = (point_x, point_y)
                            nearby_endpoint_tolerance_um = endpoint_margin_tolerance_um + float(
                                grid_size_um
                            )
                            if (
                                _point_near_route_endpoint(
                                    point,
                                    centerlines_by_id.get(net_id_a, ()),
                                    tolerance_um=nearby_endpoint_tolerance_um,
                                )
                                or _point_near_route_endpoint(
                                    point,
                                    centerlines_by_id.get(net_id_b, ()),
                                    tolerance_um=nearby_endpoint_tolerance_um,
                                )
                                or _point_near_record_port_endpoint(
                                    point,
                                    record_a,
                                    tolerance_um=nearby_endpoint_tolerance_um,
                                )
                                or _point_near_record_port_endpoint(
                                    point,
                                    record_b,
                                    tolerance_um=nearby_endpoint_tolerance_um,
                                )
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
                    contact_adjacent = margin_a <= eps and margin_b <= eps
                    axis_u = _segment_unit_vector(segment_a)
                    axis_v = _segment_unit_vector(segment_b)
                    footprint_polygon: list[tuple[float, float]] = []
                    footprint_blockers: list[dict[str, object]] = []
                    rust_fields: dict[str, object] | None = None
                    record_required_margin_um = required_margin_um
                    if unified_legality:
                        # The Rust predicate's verdict for this intersection
                        # decides; nothing below re-tests it.
                        rust_verdict = _nearest_rust_verdict(
                            verdicts_by_pair,
                            net_id_a,
                            net_id_b,
                            (point_x, point_y),
                            tolerance_um=verdict_tolerance_um,
                        )
                        if rust_verdict is not None:
                            rust_fields = _rust_verdict_fields(
                                rust_verdict,
                                net_id_a=net_id_a,
                                grid_size_um=float(grid_size_um),
                            )
                            perpendicular = bool(rust_fields["perpendicular"])
                            margin_a = float(cast(float, rust_fields["segment_a_margin_um"]))
                            margin_b = float(cast(float, rust_fields["segment_b_margin_um"]))
                            record_required_margin_um = float(
                                cast(float, rust_fields["required_margin_um"])
                            )
                            footprint_straight = bool(rust_fields["legal"]) or str(
                                rust_fields["reason"]
                            ) in {"footprint_blocked", "missing_event", "not_perpendicular"}
                        else:
                            perpendicular = False
                            footprint_straight = False
                    else:
                        perpendicular = _segments_are_perpendicular(segment_a, segment_b)
                        footprint_straight = required_margin_um <= eps or (
                            margin_a + eps >= required_margin_um
                            and margin_b + eps >= required_margin_um
                        )
                    if axis_u is not None and axis_v is not None and footprint_half_um > eps:
                        footprint_axis_u = axis_u
                        footprint_axis_v = axis_v
                        if not perpendicular or not footprint_straight:
                            footprint_axis_u = (1.0, 0.0)
                            footprint_axis_v = (0.0, 1.0)
                        footprint_polygon = _crossing_footprint_polygon(
                            center=(point_x, point_y),
                            axis_u=footprint_axis_u,
                            axis_v=footprint_axis_v,
                            half_extent_um=footprint_half_um,
                        )
                        if footprint_straight and not unified_legality:
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
                    if rust_fields is not None:
                        legal = bool(rust_fields["legal"])
                    elif unified_legality:
                        legal = False
                    else:
                        legal = perpendicular and footprint_straight and not footprint_blockers
                    if legal:
                        classification = (
                            "legal_expected_crossing"
                            if pair_expected
                            else "legal_unexpected_crossing"
                        )
                        reason = None
                        degraded_reason = None
                    elif rust_fields is not None:
                        classification = "illegal_unexpected_crossing"
                        reason = str(rust_fields["reason"]) or "crossing_footprint_invalid"
                        degraded_reason = None
                    elif unified_legality and not contact_adjacent:
                        # Rust assessed no intersection here: a crossing without
                        # a registered event, as the Rust validator would call it.
                        classification = "illegal_unexpected_crossing"
                        reason = "missing_event"
                        degraded_reason = None
                    elif contact_adjacent:
                        classification = "contact_adjacent_geometry"
                        reason = "shared_segment_endpoint"
                        degraded_reason = None
                    else:
                        classification = "illegal_unexpected_crossing"
                        if not footprint_straight:
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
                                route_endpoint_distance_a := _point_route_endpoint_distance_um(
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
                                route_endpoint_distance_b := _point_route_endpoint_distance_um(
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
                                port_endpoint_distance_a := _point_record_port_endpoint_distance_um(
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
                                port_endpoint_distance_b := _point_record_port_endpoint_distance_um(
                                    (point_x, point_y),
                                    record_b,
                                )
                            )
                            is not None
                            else None
                        ),
                        "segment_a_um": _rounded_segment(segment_a),
                        "segment_b_um": _rounded_segment(segment_b),
                        "segment_a_margin_um": _rounded_margin_um(margin_a),
                        "segment_b_margin_um": _rounded_margin_um(margin_b),
                        "required_margin_um": round(float(record_required_margin_um), 6),
                        "crossing_footprint_half_um": round(
                            float(footprint_half_um),
                            6,
                        ),
                        "crossing_footprint_um": round(
                            2.0 * float(footprint_half_um),
                            6,
                        ),
                        "search_required_margin_um": round(
                            float(
                                record_required_margin_um
                                if unified_legality
                                else search_required_margin_um
                            ),
                            6,
                        ),
                        "crossing_footprint_polygon_um": [
                            _rounded_point(point) for point in footprint_polygon
                        ],
                        "crossing_footprint_blockers": footprint_blockers,
                        "expected_pair": bool(pair_expected),
                        "planned": _planned_flag(planned_pairs, net_id_a, net_id_b),
                        "perpendicular": bool(perpendicular),
                        "classification": classification,
                    }
                    if reason is not None:
                        record["reason"] = reason
                    if degraded_reason is not None:
                        record["degraded_reason"] = degraded_reason
                    if unified_legality:
                        record["legality_source"] = (
                            "rust_crossing_verdict"
                            if rust_fields is not None
                            else "no_rust_crossing_verdict"
                        )
                        if rust_fields is not None:
                            record["rust_verdict"] = rust_fields["rust_verdict"]
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
        indices = raw_indices if isinstance(raw_indices, list) else []
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
                crossing_a.get("degraded_reason") is not None
                and crossing_b.get("degraded_reason") is not None
                and ids_a.intersection(ids_b)
            ):
                crossing_a["footprint_overlap_policy"] = "allowed_lidar_pure_degraded_cluster"
                _add_overlap_peer(
                    crossing_a,
                    peer=overlap_peer_a,
                    peer_index=int(index_b),
                )
                crossing_b["footprint_overlap_policy"] = "allowed_lidar_pure_degraded_cluster"
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

    crossing_plan_info.realized_intersections = realized
    crossing_plan_info.realized_intersection_count = len(realized)
    crossing_plan_info.routes_missing_corrected_centerline = missing_centerline_illegal
    crossing_plan_info.routes_missing_corrected_centerline_count = len(missing_centerline_illegal)
    crossing_plan_info.ignored_endpoint_access_intersections = ignored_endpoint_access
    crossing_plan_info.ignored_endpoint_access_intersection_count = len(ignored_endpoint_access)
    crossing_plan_info.illegal_realized_crossings = illegal
    crossing_plan_info.illegal_realized_crossing_count = len(illegal)
    return illegal


def _populate_realized_intersections_from_native_crossing_events(
    *,
    crossing_plan_info: CrossingPlanInfo,
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    native_crossing_events: Iterable[object],
    realization_grid_spec: tuple[int, int, float, float, float],
    native_crossing_verdicts: Iterable[object] | None = None,
    unified_legality: bool = False,
) -> list[dict[str, object]]:
    """Populate realized crossing metadata from A*-accepted native events.

    This is the normal fast path. A* is responsible for accepting only legal
    crossing moves; this function converts those accepted events into the
    physical metadata needed for crossing component placement and for the final
    external Python geometry verifier.

    With ``unified_legality`` on, each event carries the Rust predicate's
    verdict for its point (``native_crossing_verdicts`` =
    ``router.crossing_verdicts()``): classification, reason, perpendicularity
    and margins come from it, and a verdict Rust recorded for a point no event
    covers is reported as its own record. The returned list holds the records
    the verdicts call illegal (always empty in the legacy mode, which trusts
    the events).
    """
    if not crossing_plan_info.enabled:
        crossing_plan_info.realized_intersections = []
        crossing_plan_info.realized_intersection_count = 0
        crossing_plan_info.illegal_realized_crossings = []
        crossing_plan_info.illegal_realized_crossing_count = 0
        return []

    _width, _height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    footprint_half_um = _crossing_footprint_half_extent_um(
        crossing_plan_info=crossing_plan_info,
        grid_size_um=float(grid_size_um),
    )
    required_margin_um = footprint_half_um
    search_required_margin_um = footprint_half_um + float(grid_size_um) * int(
        crossing_plan_info.bend_runout_cells_per_crossing or 0
    )
    if unified_legality:
        required_margin_um = _unified_required_margin_um(crossing_plan_info, float(grid_size_um))
        search_required_margin_um = required_margin_um
    verdicts_by_pair = _rust_verdicts_by_pair(
        native_crossing_verdicts if unified_legality else None
    )
    verdict_tolerance_um = 1.5 * float(grid_size_um)
    matched_verdict_ids: set[int] = set()

    planned_pairs = _planned_pairs_from_plan_info(crossing_plan_info)
    realized: list[dict[str, object]] = []
    illegal: list[dict[str, object]] = []
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
        record: dict[str, object] = {
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
            "crossing_footprint_polygon_um": [_rounded_point(point) for point in footprint_polygon],
            "crossing_footprint_blockers": [],
            "expected_pair": True,
            "planned": _planned_flag(planned_pairs, net_id_a, net_id_b),
            "perpendicular": True,
            "classification": "legal_native_crossing",
            "source": "native_crossing_events",
            "reservation_cells": list(raw_event.get("reservation_cells", [])),
        }
        if unified_legality:
            rust_verdict = _nearest_rust_verdict(
                verdicts_by_pair,
                net_id_a,
                net_id_b,
                point_um,
                tolerance_um=verdict_tolerance_um,
            )
            if rust_verdict is not None:
                matched_verdict_ids.add(id(rust_verdict))
                _apply_rust_verdict_to_native_record(
                    record,
                    rust_verdict,
                    net_id_a=net_id_a,
                    grid_size_um=float(grid_size_um),
                )
            else:
                record["legality_source"] = "no_rust_crossing_verdict"
        realized.append(record)
        if str(record.get("classification", "")).startswith("illegal_"):
            illegal.append(record)

    if unified_legality:
        # Verdicts for points no event covers: Rust assessed them, so they are
        # reported as their own records (legal or not) rather than dropped.
        for pair_verdicts in verdicts_by_pair.values():
            for verdict in pair_verdicts:
                if id(verdict) in matched_verdict_ids:
                    continue
                record = _native_record_from_rust_verdict(
                    verdict,
                    routed_records_by_net_id=routed_records_by_net_id,
                    planned_pairs=planned_pairs,
                    footprint_half_um=footprint_half_um,
                    grid_size_um=float(grid_size_um),
                    origin_x_um=float(origin_x_um),
                    origin_y_um=float(origin_y_um),
                )
                if record is None:
                    continue
                realized.append(record)
                if str(record.get("classification", "")).startswith("illegal_"):
                    illegal.append(record)

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
                _grid_point_to_physical_um(
                    (float(raw[0][0]), float(raw[0][1])),
                    grid_size_um=grid_size_um,
                    origin_x_um=origin_x_um,
                    origin_y_um=origin_y_um,
                ),
                _grid_point_to_physical_um(
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
    grid_cell = _physical_point_to_grid_cell(
        point_um,
        grid_size_um=grid_size_um,
        origin_x_um=origin_x_um,
        origin_y_um=origin_y_um,
    )
    footprint_polygon: list[tuple[float, float]] = []
    axis_u = _segment_unit_vector(segment_a) if segment_a is not None else None
    axis_v = _segment_unit_vector(segment_b) if segment_b is not None else None
    if axis_u is not None and axis_v is not None and footprint_half_um > 0.0:
        footprint_polygon = _crossing_footprint_polygon(
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
        "point_um": _rounded_point(point_um),
        "grid_cell": ([int(grid_cell[0]), int(grid_cell[1])] if grid_cell is not None else None),
        "segment_a_um": _rounded_segment(segment_a) if segment_a is not None else None,
        "segment_b_um": _rounded_segment(segment_b) if segment_b is not None else None,
        "crossing_footprint_half_um": round(float(footprint_half_um), 6),
        "crossing_footprint_um": round(2.0 * float(footprint_half_um), 6),
        "crossing_footprint_polygon_um": [_rounded_point(point) for point in footprint_polygon],
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
