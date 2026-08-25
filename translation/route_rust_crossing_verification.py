"""Realized-route crossing verification for the Rust-backed photonic router."""

from __future__ import annotations

from typing import Any, Iterable, Mapping, cast

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

    crossing_mode = str(crossing_plan_info.get("crossing_mode", "") or "").strip().lower()
    allow_unexpected = crossing_mode == "lidar-pure" or not bool(
        crossing_plan_info.get("allow_only_expected_crossings", True)
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
                    perpendicular = _segments_are_perpendicular(segment_a, segment_b)
                    contact_adjacent = margin_a <= eps and margin_b <= eps
                    axis_u = _segment_unit_vector(segment_a)
                    axis_v = _segment_unit_vector(segment_b)
                    footprint_polygon: list[tuple[float, float]] = []
                    footprint_blockers: list[dict[str, object]] = []
                    footprint_straight = required_margin_um <= eps or (
                        margin_a + eps >= required_margin_um
                        and margin_b + eps >= required_margin_um
                    )
                    if axis_u is not None and axis_v is not None and footprint_half_um > eps:
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

    crossing_plan_info["realized_intersections"] = realized
    crossing_plan_info["realized_intersection_count"] = len(realized)
    crossing_plan_info["routes_missing_corrected_centerline"] = missing_centerline_illegal
    crossing_plan_info["routes_missing_corrected_centerline_count"] = len(
        missing_centerline_illegal
    )
    crossing_plan_info["ignored_endpoint_access_intersections"] = ignored_endpoint_access
    crossing_plan_info["ignored_endpoint_access_intersection_count"] = len(ignored_endpoint_access)
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
