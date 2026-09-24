"""Phase 8: the final geometry repair and verification passes.

The two mid-pipeline repair passes (final crossing repair, issue-group repair)
still call the kernel's legacy repair loop -- question D8 of the restructure
plan; candidate for removal, see Milestone 8."""

from __future__ import annotations

import math
import re
import time

from collections import Counter
from collections.abc import Iterable as IterableABC
from pathlib import Path
from typing import Any, Iterable, Mapping, cast

from gdsfactory.component import Component

from photonic_router.static_obstacle_builder import physical_to_grid

from translation.photonic_verification import (
    PhotonicVerificationIssue,
    PhotonicVerificationResult,
    verify_photonic_routing,
)
from translation.route_rust_crossing_components import (
    _legal_crossing_component_footprints_for_verification,
    _legal_crossing_overlap_polygons_for_verification,
    _place_realized_crossing_components,
)
from translation.route_rust_crossing_plan import (
    _augment_crossing_plan_with_realized_overlaps,
    _augment_insertion_loss_report,
    _augment_insertion_loss_report_from_realized_intersections,
    _write_crossing_debug_artifacts,
)
from translation.route_rust_crossing_verification import (
    _populate_realized_intersections_from_native_crossing_events,
    _verify_realized_route_intersections,
)
from translation.route_rust_debug_artifacts import _dump_photonic_probe_failure_artifacts
from translation.route_rust_obstacle_config import _default_obstacle_layers
from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_types import RoutedNetRecord

def _grid_cell_from_raw_point(session, raw_point: object) -> tuple[int, int] | None:
    if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
        return None
    try:
        point_x = float(raw_point[0])
        point_y = float(raw_point[1])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(point_x) or not math.isfinite(point_y):
        return None
    return physical_to_grid(point_x, point_y, session.grid)


def _illegal_crossing_grid_cell(session, item: Mapping[str, object]) -> tuple[int, int] | None:
    raw_cell = item.get("grid_cell")
    if isinstance(raw_cell, (tuple, list)) and len(raw_cell) == 2:
        try:
            return (int(raw_cell[0]), int(raw_cell[1]))
        except (TypeError, ValueError):
            return None
    return session._grid_cell_from_raw_point(item.get("point_um"))


def _illegal_crossing_keepout_radius(session, item: Mapping[str, object]) -> int:
    reason = str(item.get("reason", "") or "")
    if reason == "not_perpendicular":
        return max(1, int(session.resolved_crossing_half_size_cells) + 1)
    if reason == "collinear_route_overlap":
        return 1
    blockers = item.get("crossing_footprint_blockers")
    if isinstance(blockers, IterableABC) and not isinstance(
        blockers,
        (str, bytes, bytearray),
    ):
        if any(isinstance(blocker, Mapping) for blocker in blockers):
            return max(1, int(session.resolved_crossing_half_size_cells) + 1)
    if reason in {
        "crossing_footprint_contains_route_geometry",
        "crossing_footprint_overlap",
    }:
        return max(1, int(session.resolved_crossing_half_size_cells) + 1)
    return max(1, min(4, int(session.resolved_crossing_half_size_cells) + 1))


def _add_keepout_square(
    session,
    keepout_cells: set[tuple[int, int]],
    *,
    center: tuple[int, int],
    radius: int,
) -> None:
    for y in range(center[1] - radius, center[1] + radius + 1):
        for x in range(center[0] - radius, center[0] + radius + 1):
            if 0 <= x < int(session.grid.width) and 0 <= y < int(session.grid.height):
                keepout_cells.add((x, y))


def _add_segment_keepout_cells(
    session,
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
        session._add_keepout_square(keepout_cells, center=cell, radius=radius)


def _final_crossing_repair_keepout_cells(
    session,
    illegal_crossings: Iterable[Mapping[str, object]],
) -> set[tuple[int, int]]:
    keepout_cells: set[tuple[int, int]] = set()
    for item in illegal_crossings:
        radius = session._illegal_crossing_keepout_radius(item)
        reason = str(item.get("reason", "") or "")
        if reason == "collinear_route_overlap":
            start_cell = session._grid_cell_from_raw_point(item.get("overlap_start_um"))
            end_cell = session._grid_cell_from_raw_point(item.get("overlap_end_um"))
            if start_cell is not None and end_cell is not None:
                session._add_segment_keepout_cells(
                    keepout_cells,
                    start_cell=start_cell,
                    end_cell=end_cell,
                    radius=radius,
                )
                continue
        center = session._illegal_crossing_grid_cell(item)
        if center is not None:
            session._add_keepout_square(keepout_cells, center=center, radius=radius)
        if reason == "crossing_footprint_overlap":
            peer = item.get("overlapping_crossing")
            if isinstance(peer, Mapping):
                peer_center = session._grid_cell_from_raw_point(peer.get("point_um"))
                if peer_center is not None:
                    session._add_keepout_square(
                        keepout_cells,
                        center=peer_center,
                        radius=radius,
                    )
    return keepout_cells


def _final_crossing_repair_net_ids(
    session,
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
            if blocker_net_id in session.route_jobs_by_id:
                net_ids.add(blocker_net_id)

    for item in illegal_crossings:
        item_pair_ids: list[int] = []
        for key in ("net_id_a", "net_id_b"):
            try:
                net_id = int(cast(object, item.get(key)))
            except (TypeError, ValueError):
                continue
            if net_id in session.route_jobs_by_id:
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
                    if peer_net_id in session.route_jobs_by_id:
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
    return [net_id for net_id in session.route_order if net_id in net_ids]


def _final_crossing_repair_batches(
    session,
    illegal_crossings: list[dict[str, object]],
    *,
    max_net_ids: int,
) -> list[list[dict[str, object]]]:
    if not illegal_crossings or max_net_ids <= 0:
        return []
    components: list[tuple[list[dict[str, object]], set[int]]] = []
    for item in illegal_crossings:
        item_net_ids = set(session._final_crossing_repair_net_ids([item]))
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
        component_repair_ids = session._final_crossing_repair_net_ids(component_items)
        if component_repair_ids and len(component_repair_ids) <= max_net_ids:
            capped_batches.append(component_items)
        else:
            oversized.extend(component_items)

    if oversized:
        current_batch: list[dict[str, object]] = []
        current_net_ids: set[int] = set()
        for item in oversized:
            item_net_ids = set(session._final_crossing_repair_net_ids([item]))
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

    route_index = {int(net_id): index for index, net_id in enumerate(session.route_order)}

    def _batch_sort_key(batch: list[dict[str, object]]) -> tuple[int, int, int]:
        repair_ids = session._final_crossing_repair_net_ids(batch)
        first_route_index = min(
            (route_index.get(int(net_id), len(route_index)) for net_id in repair_ids),
            default=len(route_index),
        )
        return (first_route_index, -len(batch), len(repair_ids))

    capped_batches.sort(key=_batch_sort_key)
    return capped_batches


def _repair_final_illegal_crossings(
    session,
    illegal_crossings: list[dict[str, object]],
) -> bool:
    max_repair_net_ids = 12
    attempts = cast(
        list[dict[str, object]],
        session.crossing_plan_info.setdefault("final_crossing_repair_attempts", []),
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
        if selected_reason is None or str(item.get("reason", "") or "") == selected_reason
    ]
    total_selected_issue_count = len(selected_illegal_crossings)
    if selected_reason in {
        "crossing_footprint_contains_bend",
        "insufficient_straight_margin",
        "not_perpendicular",
    }:
        repair_batches = session._final_crossing_repair_batches(
            selected_illegal_crossings,
            max_net_ids=min(4, max_repair_net_ids),
        )
        if repair_batches:
            selected_illegal_crossings = repair_batches[0]
    if (
        not selected_illegal_crossings
        or not session.crossing_plan_info.get("enabled")
        or not session.repair_config.enabled
        or not hasattr(session.router, "add_static_cells")
        or not hasattr(session.router, "ripup_route")
        or not hasattr(session.router, "route_many_with_repair_and_commit")
    ):
        return False
    repair_net_ids = session._final_crossing_repair_net_ids(selected_illegal_crossings)
    if selected_reason == "not_perpendicular" and repair_net_ids:
        priority: list[int] = []
        for item in selected_illegal_crossings:
            for key in ("net_id_b", "net_id_a"):
                try:
                    net_id = int(cast(object, item.get(key)))
                except (TypeError, ValueError):
                    continue
                if net_id in session.route_jobs_by_id and net_id not in priority:
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
                if blocker_net_id in session.route_jobs_by_id and blocker_net_id not in priority:
                    priority.append(blocker_net_id)
        repair_net_ids = [net_id for net_id in priority if net_id in repair_net_ids] + [
            net_id for net_id in repair_net_ids if net_id not in priority
        ]
    attempt: dict[str, object] = {
        "selected_reason": selected_reason,
        "illegal_reason_counts": dict(
            Counter(str(item.get("reason", "") or "unknown") for item in illegal_crossings)
        ),
        "selected_reason_counts": dict(
            Counter(
                str(item.get("reason", "") or "unknown") for item in selected_illegal_crossings
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
    keepout_cells = session._final_crossing_repair_keepout_cells(selected_illegal_crossings)
    attempt["keepout_cell_count"] = len(keepout_cells)
    if not keepout_cells:
        attempt["status"] = "skipped"
        attempt["reason"] = "no_keepout_cells"
        attempts.append(attempt)
        return False

    session.router.add_static_cells(sorted(keepout_cells))
    for net_id in repair_net_ids:
        session.router.ripup_route(int(net_id))
        session.route_bookkeeping.clear_route(int(net_id))

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
        job = session.route_jobs_by_id[net_id]
        source_state, target_state, _, _, opened_cells = session._state_openings_for_job(job)
        clearance_exempt_cells = session._clearance_exempt_cells_for_job(job)
        repair_jobs.append(
            (
                int(job.net_id),
                source_state,
                target_state,
                opened_cells,
                clearance_exempt_cells,
                session._foreign_keepout_cleanup_cells_for_job(job),
                session._routing_endpoint_center_um(job, source=True),
                session._routing_endpoint_center_um(job, source=False),
            )
        )
        opened_by_id[int(job.net_id)] = opened_cells

    raw_repair_result = session.router.route_many_with_repair_and_commit(
        repair_jobs,
        session.block_radius_cells,
        session.commit_radius_cells,
        session.core_commit_radius_cells,
        int(session.repair_config.max_rounds),
        int(session.repair_config.max_victims_per_failure),
        float(session.repair_config.history_weight),
        int(session.repair_config.history_increment),
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
        job = session.route_jobs_by_id[net_id]
        route_obj = entry["route"]
        session._record_route(job, route_obj, opened_by_id[net_id])
        repaired_records.append(session.route_bookkeeping.records_by_id[net_id])

    if session.enable_checked_endpoint_correction and repaired_records:
        repaired_net_ids = [
            int(record.net_id) for record in repaired_records if record.net_id is not None
        ]
        session._apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
            repaired_net_ids,
            record_pipeline_timing=False,
        )
    attempt["status"] = "routed"
    attempts.append(attempt)
    return True


def _net_id_by_name(session) -> dict[str, int]:
    return {
        record.net_name: int(net_id)
        for net_id, record in session.route_bookkeeping.records_by_id.items()
    }


def _grid_rect_from_um_bbox(
    session,
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
    grid_size = float(session.grid.grid_size_um)
    min_cell = physical_to_grid(min_x_um, min_y_um, session.grid)
    return (
        min_cell[0],
        int(math.ceil((max_x_um - float(session.origin_x_um)) / grid_size)),
        min_cell[1],
        int(math.ceil((max_y_um - float(session.origin_y_um)) / grid_size)),
    )


def _add_keepout_rect(
    session,
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
        if y < 0 or y >= int(session.grid.height):
            continue
        for x in range(min_x - radius, max_x + radius + 1):
            if 0 <= x < int(session.grid.width):
                keepout_cells.add((x, y))


def _cells_from_um_bbox(
    session,
    raw_bbox: object,
    *,
    radius: int,
) -> set[tuple[int, int]]:
    rect = session._grid_rect_from_um_bbox(raw_bbox)
    if rect is None:
        return set()
    cells: set[tuple[int, int]] = set()
    session._add_keepout_rect(cells, rect=rect, radius=radius)
    return cells


def _grid_rect_from_grid_bbox_text(
    session, text: str, name: str
) -> tuple[int, int, int, int] | None:
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


def _polygon_bbox_um(session, raw_polygon: object) -> tuple[float, float, float, float] | None:
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
    session,
    issue: PhotonicVerificationIssue,
) -> set[tuple[int, int]]:
    radius = max(1, int(session.core_commit_radius_cells) + 1)
    if issue.code == "endpoint_correction_error":
        cells: set[tuple[int, int]] = set()
        for bbox_name in ("static_bbox", "core_bbox"):
            rect = session._grid_rect_from_grid_bbox_text(issue.message, bbox_name)
            if rect is None:
                continue
            session._add_keepout_rect(cells, rect=rect, radius=radius)
            if cells:
                return cells
        return cells
    details = issue.details or {}
    if issue.code == "cross_net_waveguide_overlap":
        return session._cells_from_um_bbox(
            details.get("overlap_bbox_um"),
            radius=radius,
        )
    if issue.code == "waveguide_obstacle_overlap":
        return session._cells_from_um_bbox(
            details.get("overlap_bbox_um"),
            radius=radius,
        )
    if issue.code == "crossing_component_route_overlap":
        crossing = details.get("crossing")
        if isinstance(crossing, Mapping):
            polygon_bbox = session._polygon_bbox_um(crossing.get("crossing_footprint_polygon_um"))
            if polygon_bbox is not None:
                return session._cells_from_um_bbox(polygon_bbox, radius=radius)
        return session._cells_from_um_bbox(
            details.get("overlap_bbox_um"),
            radius=radius,
        )
    return set()


def _photonic_issue_net_ids(
    session,
    issue: PhotonicVerificationIssue,
) -> set[int]:
    by_name = session._net_id_by_name()
    net_ids: set[int] = set()
    if issue.net_name and issue.net_name in by_name:
        net_ids.add(by_name[issue.net_name])
    details = issue.details or {}
    other_net_name = details.get("other_net_name")
    if isinstance(other_net_name, str) and other_net_name in by_name:
        net_ids.add(by_name[other_net_name])
    return net_ids


def _make_photonic_verification_probe_layout(
    session,
    records: Iterable[RoutedNetRecord],
) -> Component:
    t_probe_layout_total_start = session._pipeline_timer_start()
    session.photonic_probe_index += 1
    t_probe_copy_start = session._pipeline_timer_start()
    probe_layout = session.unrouted_layout.copy()
    probe_layout.name = f"photonic_repair_probe_{time.time_ns()}_{session.photonic_probe_index}"
    session._record_pipeline_timing("photonic_probe_copy", t_probe_copy_start)
    t_probe_realize_start = session._pipeline_timer_start()
    realize_routed_net_records(
        probe_layout,
        list(records),
        route_width_um=session.route_width_um,
        route_layer=session.route_layer,
        realization_grid_spec=session.realization_grid_spec,
        allow_45_degree_turns=session.allow_45_degree_turns,
        bend_radius_cells=session.bend_radius_cells,
        crossing_plan_info=session.crossing_plan_info,
        enable_endpoint_correction=session.enable_checked_endpoint_correction,
    )
    session._record_pipeline_timing("photonic_probe_realize", t_probe_realize_start)
    if session.crossing_plan_info.get("enabled"):
        t_probe_crossings_start = session._pipeline_timer_start()
        _place_realized_crossing_components(probe_layout, session.crossing_plan_info)
        session._record_pipeline_timing(
            "photonic_probe_crossing_place",
            t_probe_crossings_start,
        )
    session._record_pipeline_timing(
        "photonic_probe_layout_total",
        t_probe_layout_total_start,
    )
    return probe_layout


def _refresh_photonic_verification(session) -> PhotonicVerificationResult:
    # This is an internal diagnostic verifier, not the intended default
    # source of truth for production routing success. A* and the grid-level
    # crossing checks must reject illegal moves locally; the final geometry
    # gate is the Python verifier in `routing_flow.py` on the realized
    # layout. Keep this probe available for debugging model mismatches
    # between grid decisions and realized geometry, but do not treat it as
    # a mandatory always-on second full verification pass.
    t_refresh_start = session._pipeline_timer_start()
    records = session.route_bookkeeping.ordered_records()
    probe_layout = session._make_photonic_verification_probe_layout(records)
    session.last_photonic_probe_layout = probe_layout
    session.last_photonic_probe_records = list(records)
    t_verify_start = session._pipeline_timer_start()
    result = verify_photonic_routing(
        probe_layout,
        session.schematic,
        routed_net_records=records,
        unrouted_layout=session.unrouted_layout,
        route_width_um=session.route_width_um,
        route_layer=session.route_layer,
        obstacle_layers=_default_obstacle_layers(
            session.route_layer,
            include_heater_obstacles=session.include_heater_obstacles,
        ),
        realization_grid_spec=session.realization_grid_spec,
        allow_45_degree_turns=session.allow_45_degree_turns,
        bend_radius_cells=session.bend_radius_cells,
        legal_overlap_polygons_by_net_id_pair_um=(
            _legal_crossing_overlap_polygons_for_verification(session.crossing_plan_info)
        ),
        crossing_component_footprints_um=(
            _legal_crossing_component_footprints_for_verification(session.crossing_plan_info)
        ),
        check_route_coverage=session.debug_stop_after_route_index is None,
        check_endpoint_connectivity=session.enable_checked_endpoint_correction,
    )
    session._record_pipeline_timing("photonic_probe_verify", t_verify_start)
    session._record_pipeline_timing("photonic_refresh_total", t_refresh_start)
    return result


def _repair_final_photonic_issues(
    session,
    issues: tuple[PhotonicVerificationIssue, ...],
) -> bool:
    attempts = cast(
        list[dict[str, object]],
        session.crossing_plan_info.setdefault("final_photonic_repair_attempts", []),
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
        or not session.repair_config.enabled
        or not hasattr(session.router, "add_static_cells")
        or not hasattr(session.router, "ripup_route")
        or not hasattr(session.router, "route_many_with_repair_and_commit")
    ):
        return False

    repair_net_ids_set: set[int] = set()
    keepout_cells: set[tuple[int, int]] = set()
    for issue in selected_issues:
        repair_net_ids_set.update(session._photonic_issue_net_ids(issue))
        keepout_cells.update(session._photonic_issue_keepout_cells(issue))
    repair_net_ids = [net_id for net_id in session.route_order if net_id in repair_net_ids_set]
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

    session.router.add_static_cells(sorted(keepout_cells))
    for net_id in repair_net_ids:
        session.router.ripup_route(int(net_id))
        session.route_bookkeeping.clear_route(int(net_id))

    repair_jobs: list[
        tuple[
            int,
            Any,
            Any,
            list[tuple[int, int]],
            list[tuple[int, int]],
            list[tuple[int, int]],
            tuple[float, float] | None,
            tuple[float, float] | None,
        ]
    ] = []
    opened_by_id: dict[int, list[tuple[int, int]]] = {}
    for net_id in repair_net_ids:
        job = session.route_jobs_by_id[net_id]
        source_state, target_state, _, _, opened_cells = session._state_openings_for_job(job)
        clearance_exempt_cells = session._clearance_exempt_cells_for_job(job)
        repair_jobs.append(
            (
                int(job.net_id),
                source_state,
                target_state,
                opened_cells,
                clearance_exempt_cells,
                session._foreign_keepout_cleanup_cells_for_job(job),
                session._routing_endpoint_center_um(job, source=True),
                session._routing_endpoint_center_um(job, source=False),
            )
        )
        opened_by_id[int(job.net_id)] = opened_cells

    raw_repair_result = session.router.route_many_with_repair_and_commit(
        repair_jobs,
        session.block_radius_cells,
        session.commit_radius_cells,
        session.core_commit_radius_cells,
        int(session.repair_config.max_rounds),
        int(session.repair_config.max_victims_per_failure),
        float(session.repair_config.history_weight),
        int(session.repair_config.history_increment),
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
        job = session.route_jobs_by_id[net_id]
        route_obj = entry["route"]
        session._record_route(job, route_obj, opened_by_id[net_id])
        repaired_net_ids.append(net_id)
    if session.enable_checked_endpoint_correction and repaired_net_ids:
        failed_corrections = (
            session._apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
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
    session,
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
        lines.append(f"{issue.code} {issue.net_name or '<unknown>'}: {issue.message}{suffix}")
    if len(verification.issues) > 5:
        lines.append(f"... {len(verification.issues) - 5} more")
    return "; ".join(lines)


def _refresh_realized_crossing_verification(session) -> list[dict[str, object]]:
    t_refresh_crossings_start = session._pipeline_timer_start()
    t_overlap_start = session._pipeline_timer_start()
    if session.enable_internal_photonic_probe_verification:
        _augment_crossing_plan_with_realized_overlaps(
            router=session.router,
            crossing_plan_info=session.crossing_plan_info,
            routed_records_by_net_id=session.route_bookkeeping.records_by_id,
        )
    session._record_pipeline_timing("realized_crossing_overlap_augment", t_overlap_start)
    native_crossing_events: list[Any] = []
    if hasattr(session.router, "crossing_events"):
        t_native_events_start = session._pipeline_timer_start()
        try:
            native_crossing_events = list(cast(Iterable[Any], session.router.crossing_events()))
        except Exception:
            native_crossing_events = []
        session.crossing_plan_info["native_crossing_events"] = native_crossing_events
        session.crossing_plan_info["native_crossing_event_count"] = len(native_crossing_events)
        session._record_pipeline_timing(
            "realized_crossing_native_events",
            t_native_events_start,
        )
        t_insertion_loss_start = session._pipeline_timer_start()
        _augment_insertion_loss_report(
            crossing_plan_info=session.crossing_plan_info,
            routed_records_by_net_id=session.route_bookkeeping.records_by_id,
            native_crossing_events=native_crossing_events,
        )
        session._record_pipeline_timing(
            "realized_crossing_insertion_loss",
            t_insertion_loss_start,
        )
    t_illegal_crossing_verify_start = session._pipeline_timer_start()
    if session.enable_internal_photonic_probe_verification:
        illegal = _verify_realized_route_intersections(
            crossing_plan_info=session.crossing_plan_info,
            routed_records_by_net_id=session.route_bookkeeping.records_by_id,
            realization_grid_spec=session.realization_grid_spec,
        )
    else:
        illegal = _populate_realized_intersections_from_native_crossing_events(
            crossing_plan_info=session.crossing_plan_info,
            routed_records_by_net_id=session.route_bookkeeping.records_by_id,
            native_crossing_events=native_crossing_events,
            realization_grid_spec=session.realization_grid_spec,
        )
    session._record_pipeline_timing(
        "realized_crossing_verify_intersections",
        t_illegal_crossing_verify_start,
    )
    t_realized_insertion_loss_start = session._pipeline_timer_start()
    _augment_insertion_loss_report_from_realized_intersections(
        crossing_plan_info=session.crossing_plan_info,
        routed_records_by_net_id=session.route_bookkeeping.records_by_id,
    )
    session._record_pipeline_timing(
        "realized_crossing_realized_loss",
        t_realized_insertion_loss_start,
    )
    session._record_pipeline_timing(
        "realized_crossing_refresh_total",
        t_refresh_crossings_start,
    )
    return illegal


def repair_and_verify_final_geometry(
    session, routed_net_records: list[RoutedNetRecord]
) -> tuple[list[RoutedNetRecord], list[dict[str, object]]]:
    """Run the final crossing-legality and photonic-verification repair loops before geometry realization.

    Crossing legality is still checked internally because the router owns the
    crossing event model and can repair/reroute before final realization. The full
    photonic probe verification is intentionally diagnostic: it was useful while
    chasing endpoint-correction and crossing-model mismatches, but the normal flow
    skips this expensive pass unless `self.enable_internal_photonic_probe_verification`
    asks for it. The external Python verifier in `routing_flow.py` remains the final
    GDS/layout-level gate. Repairs up to `final_crossing_repair_round_limit=12` rounds
    of illegal realized crossings; if crossings are clean and internal photonic-probe
    verification is enabled, additionally repairs up to 8 rounds of photonic issues
    (each round itself re-running the crossing-repair loop), dumping failure artifacts
    and raising `RuntimeError` if geometry cannot be made legal within those bounds.
    Returns the possibly-updated `routed_net_records` and the final
    `illegal_realized_crossings` list (empty if geometry is clean) for the realization
    phase that follows.
    """
    session.photonic_probe_index = 0
    session.last_photonic_probe_layout: Component | None = None
    session.last_photonic_probe_records: list[RoutedNetRecord] = []

    final_crossing_repair_round_limit = 12
    t_final_verification_block_start = session._pipeline_timer_start()
    # Crossing legality is still checked internally because the router owns the
    # crossing event model and can repair/reroute before final realization.
    #
    # The full photonic probe verification is intentionally diagnostic: it was
    # useful while chasing endpoint-correction and crossing-model mismatches,
    # but the normal flow skips this expensive pass unless debug/failure
    # analysis asks for it. The external Python verifier in `routing_flow.py`
    # remains the final GDS/layout-level gate.
    illegal_realized_crossings = session._refresh_realized_crossing_verification()
    for _final_repair_round in range(final_crossing_repair_round_limit):
        if not illegal_realized_crossings:
            break
        if not session._repair_final_illegal_crossings(illegal_realized_crossings):
            break
        routed_net_records = session.route_bookkeeping.ordered_records()
        illegal_realized_crossings = session._refresh_realized_crossing_verification()
    if not illegal_realized_crossings and session.enable_internal_photonic_probe_verification:
        final_photonic_verification = session._refresh_photonic_verification()
        for _final_photonic_repair_round in range(8):
            if final_photonic_verification.success:
                break
            if not session._repair_final_photonic_issues(final_photonic_verification.issues):
                break
            illegal_realized_crossings = session._refresh_realized_crossing_verification()
            for _nested_crossing_repair_round in range(final_crossing_repair_round_limit):
                if not illegal_realized_crossings:
                    break
                if not session._repair_final_illegal_crossings(illegal_realized_crossings):
                    break
                illegal_realized_crossings = session._refresh_realized_crossing_verification()
            routed_net_records = session.route_bookkeeping.ordered_records()
            if illegal_realized_crossings:
                break
            final_photonic_verification = session._refresh_photonic_verification()
        if not illegal_realized_crossings and not final_photonic_verification.success:
            _write_crossing_debug_artifacts(
                debug_path=session.debug_path if session.debug_path is not None else Path("build"),
                debug_prefix=session.debug_prefix,
                crossing_plan_info=session.crossing_plan_info,
            )
            probe_failure_artifacts = _dump_photonic_probe_failure_artifacts(
                debug_path=session.debug_path if session.debug_path is not None else Path("build"),
                debug_prefix=session.debug_prefix,
                probe_layout=session.last_photonic_probe_layout,
                verification=final_photonic_verification,
                records=session.last_photonic_probe_records
                or session.route_bookkeeping.ordered_records(),
                router=session.router,
                realization_grid_spec=session.realization_grid_spec,
                allow_unchecked_bumps=True,
            )
            session.crossing_plan_info["photonic_probe_failure_artifacts"] = (
                probe_failure_artifacts
            )
            session._record_pipeline_timing(
                "final_verification_block",
                t_final_verification_block_start,
            )
            raise RuntimeError(
                "Final photonic geometry repair failed before realization: "
                f"{final_photonic_verification.error_count} error(s). "
                f"{session._photonic_repair_failure_preview(final_photonic_verification)}"
            )
    session._record_pipeline_timing(
        "final_verification_block",
        t_final_verification_block_start,
    )
    return routed_net_records, illegal_realized_crossings
