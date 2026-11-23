"""Phase 8: the final geometry repair and verification passes.

The two mid-pipeline repair passes (final crossing repair, issue-group repair)
call the kernel's negotiated repair loop
(`route_many_with_negotiated_repair_and_commit`, `NEGOTIATED_MAX_ROUNDS`
rounds), the repository's only batch routing entry point. They called the
legacy repair loop until Milestone 8 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md`
re-pointed them here (question D8, owner decision 2026-09-24); no paper cell
has ever reached either pass, so the gate cannot see the difference."""

from __future__ import annotations

import math
from collections import Counter
from collections.abc import Iterable, Mapping
from collections.abc import Iterable as IterableABC
from typing import Any, cast

from photonic_router.crossings.route_rust_crossing_plan import (
    augment_insertion_loss_report,
    augment_insertion_loss_report_from_realized_intersections,
)
from photonic_router.crossings.route_rust_crossing_verification import (
    populate_realized_intersections_from_native_crossing_events,
)
from photonic_router.records.route_rust_types import RoutedNetRecord, int_or_none
from photonic_router.routing import timing
from photonic_router.routing.dispatch import (
    NEGOTIATED_MAX_ROUNDS,
    native_route_jobs,
    record_route,
)
from photonic_router.routing.finalize import (
    apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids,
)
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.stages import VerifiedRecords
from photonic_router.routing.state import SessionState
from photonic_router.static_obstacle_builder import physical_to_grid


def _grid_cell_from_raw_point(state: SessionState, raw_point: object) -> tuple[int, int] | None:
    if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
        return None
    try:
        point_x = float(raw_point[0])
        point_y = float(raw_point[1])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(point_x) or not math.isfinite(point_y):
        return None
    return physical_to_grid(point_x, point_y, state.obstacles.grid)


def _illegal_crossing_grid_cell(
    state: SessionState, item: Mapping[str, object]
) -> tuple[int, int] | None:
    raw_cell = item.get("grid_cell")
    if isinstance(raw_cell, (tuple, list)) and len(raw_cell) == 2:
        try:
            return (int(raw_cell[0]), int(raw_cell[1]))
        except (TypeError, ValueError):
            return None
    return _grid_cell_from_raw_point(state, item.get("point_um"))


#: How the repair loop reads the reasons of the Rust predicate: each maps onto
#: the repair reason whose tactic fits -- a short straight run is a bend inside the
#: footprint, a blocked footprint is route geometry inside it, a missing
#: event has no special tactic (both nets of the pair are rerouted).
_UNIFIED_REASON_TO_REPAIR_REASON: Mapping[str, str] = {
    "margin_short_route_before": "insufficient_straight_margin",
    "margin_short_route_after": "insufficient_straight_margin",
    "margin_short_partner": "insufficient_straight_margin",
    "footprint_blocked": "crossing_footprint_contains_route_geometry",
    # Slice B-2: the candidate's reservation window shares a cell with another
    # crossing element's (the route's own or a registered one) -- the repair
    # loop's overlapping-footprints case.
    "too_close_same_route": "crossing_footprint_overlap",
    "too_close_registered": "crossing_footprint_overlap",
    "missing_event": "missing_event",
}


def _repair_reason(item: Mapping[str, object]) -> str:
    """The illegal crossing's reason in the repair loop's vocabulary."""
    reason = str(item.get("reason", "") or "")
    return _UNIFIED_REASON_TO_REPAIR_REASON.get(reason, reason)


def _illegal_crossing_keepout_radius(state: SessionState, item: Mapping[str, object]) -> int:
    reason = _repair_reason(item)
    if reason == "not_perpendicular":
        return max(1, int(state.obstacles.resolved_crossing_half_size_cells) + 1)
    if reason == "collinear_route_overlap":
        return 1
    blockers = item.get("crossing_footprint_blockers")
    if (
        isinstance(blockers, IterableABC)
        and not isinstance(
            blockers,
            (str, bytes, bytearray),
        )
        and any(isinstance(blocker, Mapping) for blocker in blockers)
    ):
        return max(1, int(state.obstacles.resolved_crossing_half_size_cells) + 1)
    if reason in {
        "crossing_footprint_contains_route_geometry",
        "crossing_footprint_overlap",
    }:
        return max(1, int(state.obstacles.resolved_crossing_half_size_cells) + 1)
    return max(1, min(4, int(state.obstacles.resolved_crossing_half_size_cells) + 1))


def _add_keepout_square(
    state: SessionState,
    keepout_cells: set[tuple[int, int]],
    *,
    center: tuple[int, int],
    radius: int,
) -> None:
    for y in range(center[1] - radius, center[1] + radius + 1):
        for x in range(center[0] - radius, center[0] + radius + 1):
            if 0 <= x < int(state.obstacles.grid.width) and 0 <= y < int(
                state.obstacles.grid.height
            ):
                keepout_cells.add((x, y))


def _add_segment_keepout_cells(
    state: SessionState,
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
            round(float(start_cell[0]) + float(dx) * t),
            round(float(start_cell[1]) + float(dy) * t),
        )
        _add_keepout_square(state, keepout_cells, center=cell, radius=radius)


def _final_crossing_repair_keepout_cells(
    state: SessionState,
    illegal_crossings: Iterable[Mapping[str, object]],
) -> set[tuple[int, int]]:
    keepout_cells: set[tuple[int, int]] = set()
    for item in illegal_crossings:
        radius = _illegal_crossing_keepout_radius(state, item)
        reason = str(item.get("reason", "") or "")
        if reason == "collinear_route_overlap":
            start_cell = _grid_cell_from_raw_point(state, item.get("overlap_start_um"))
            end_cell = _grid_cell_from_raw_point(state, item.get("overlap_end_um"))
            if start_cell is not None and end_cell is not None:
                _add_segment_keepout_cells(
                    state,
                    keepout_cells,
                    start_cell=start_cell,
                    end_cell=end_cell,
                    radius=radius,
                )
                continue
        center = _illegal_crossing_grid_cell(state, item)
        if center is not None:
            _add_keepout_square(state, keepout_cells, center=center, radius=radius)
        if reason == "crossing_footprint_overlap":
            peer = item.get("overlapping_crossing")
            if isinstance(peer, Mapping):
                peer_center = _grid_cell_from_raw_point(state, peer.get("point_um"))
                if peer_center is not None:
                    _add_keepout_square(
                        state,
                        keepout_cells,
                        center=peer_center,
                        radius=radius,
                    )
    return keepout_cells


def _final_crossing_repair_net_ids(
    state: SessionState,
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
            blocker_net_id = int_or_none(blocker.get("net_id"))
            if blocker_net_id is None:
                continue
            if blocker_net_id in state.run.route_jobs_by_id:
                net_ids.add(blocker_net_id)

    for item in illegal_crossings:
        item_pair_ids: list[int] = []
        for key in ("net_id_a", "net_id_b"):
            net_id = int_or_none(item.get(key))
            if net_id is None:
                continue
            if net_id in state.run.route_jobs_by_id:
                item_pair_ids.append(net_id)
        reason = _repair_reason(item)
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
                    peer_net_id = int_or_none(peer.get(key))
                    if peer_net_id is None:
                        continue
                    if peer_net_id in state.run.route_jobs_by_id:
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
    return [net_id for net_id in state.run.route_order if net_id in net_ids]


def _final_crossing_repair_batches(
    state: SessionState,
    illegal_crossings: list[dict[str, object]],
    *,
    max_net_ids: int,
) -> list[list[dict[str, object]]]:
    if not illegal_crossings or max_net_ids <= 0:
        return []
    components: list[tuple[list[dict[str, object]], set[int]]] = []
    for item in illegal_crossings:
        item_net_ids = set(_final_crossing_repair_net_ids(state, [item]))
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
        component_repair_ids = _final_crossing_repair_net_ids(state, component_items)
        if component_repair_ids and len(component_repair_ids) <= max_net_ids:
            capped_batches.append(component_items)
        else:
            oversized.extend(component_items)

    if oversized:
        current_batch: list[dict[str, object]] = []
        current_net_ids: set[int] = set()
        for item in oversized:
            item_net_ids = set(_final_crossing_repair_net_ids(state, [item]))
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

    route_index = {int(net_id): index for index, net_id in enumerate(state.run.route_order)}

    def _batch_sort_key(batch: list[dict[str, object]]) -> tuple[int, int, int]:
        repair_ids = _final_crossing_repair_net_ids(state, batch)
        first_route_index = min(
            (route_index.get(int(net_id), len(route_index)) for net_id in repair_ids),
            default=len(route_index),
        )
        return (first_route_index, -len(batch), len(repair_ids))

    capped_batches.sort(key=_batch_sort_key)
    return capped_batches


def _not_perpendicular_repair_priority(
    state: SessionState, illegal_crossings: list[dict[str, object]]
) -> list[int]:
    """The nets to re-route first for crossings that are not perpendicular.

    Per crossing its second net, its first net and then the nets of its
    footprint blockers, each once and only nets that have a route job.
    """

    priority: list[int] = []
    for item in illegal_crossings:
        for key in ("net_id_b", "net_id_a"):
            net_id = int_or_none(item.get(key))
            if net_id is None:
                continue
            if net_id in state.run.route_jobs_by_id and net_id not in priority:
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
            blocker_net_id = int_or_none(blocker.get("net_id"))
            if blocker_net_id is None:
                continue
            if blocker_net_id in state.run.route_jobs_by_id and blocker_net_id not in priority:
                priority.append(blocker_net_id)
    return priority


def _reroute_for_crossing_repair(
    settings: SessionSettings,
    state: SessionState,
    repair_net_ids: list[int],
    keepout_cells: set[tuple[int, int]],
    attempt: dict[str, object],
) -> bool:
    """Close ``keepout_cells``, rip up the nets and route them again; True when all are routed.

    The router's status and the routed net ids are written to ``attempt``.
    On success the new routes are recorded and, with checked endpoint
    correction on, corrected at their endpoints. On failure the nets stay
    ripped up and nothing is recorded.
    """

    state.kernel.router.add_static_cells(sorted(keepout_cells))
    for net_id in repair_net_ids:
        state.kernel.router.ripup_route(int(net_id))
        state.search.bookkeeping.clear_route(int(net_id))

    repair_jobs = native_route_jobs(
        state, (state.run.route_jobs_by_id[net_id] for net_id in repair_net_ids)
    )
    opened_by_id = {int(job.net_id): job.opened_cells for job in repair_jobs}

    raw_repair_result = state.kernel.router.route_many_with_negotiated_repair_and_commit(
        repair_jobs,
        state.kernel.radii.block_radius_cells,
        state.kernel.radii.commit_radius_cells,
        state.kernel.radii.core_commit_radius_cells,
        NEGOTIATED_MAX_ROUNDS,
        float(state.run.repair_config.history_weight),
        int(state.run.repair_config.history_increment),
    )
    repair_result = dict(raw_repair_result)
    attempt["router_status"] = str(repair_result.get("status", ""))
    attempt["routed_net_ids"] = [
        int(dict(raw_entry)["net_id"])
        for raw_entry in cast(Iterable[Any], repair_result.get("routes", []))
    ]
    if str(repair_result.get("status", "")) != "routed":
        return False

    repaired_records: list[RoutedNetRecord] = []
    for raw_entry in cast(Iterable[Any], repair_result.get("routes", [])):
        entry = dict(raw_entry)
        net_id = int(entry["net_id"])
        job = state.run.route_jobs_by_id[net_id]
        route_obj = entry["route"]
        record_route(settings, state, job, route_obj, opened_by_id[net_id])
        repaired_records.append(state.search.bookkeeping.records_by_id[net_id])

    if settings.enable_checked_endpoint_correction and repaired_records:
        repaired_net_ids = [
            int(record.net_id) for record in repaired_records if record.net_id is not None
        ]
        apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
            settings,
            state,
            repaired_net_ids,
            record_pipeline_timing=False,
        )
    return True


def _repair_final_illegal_crossings(
    settings: SessionSettings,
    state: SessionState,
    illegal_crossings: list[dict[str, object]],
) -> bool:
    max_repair_net_ids = 12
    attempts = state.crossing_plan_info.final_crossing_repair_attempts
    if attempts is None:
        attempts = []
        state.crossing_plan_info.final_crossing_repair_attempts = attempts
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
            if any(_repair_reason(item) == reason for item in illegal_crossings)
        ),
        None,
    )
    selected_illegal_crossings = [
        item
        for item in illegal_crossings
        if selected_reason is None or _repair_reason(item) == selected_reason
    ]
    total_selected_issue_count = len(selected_illegal_crossings)
    if selected_reason in {
        "crossing_footprint_contains_bend",
        "insufficient_straight_margin",
        "not_perpendicular",
    }:
        repair_batches = _final_crossing_repair_batches(
            state,
            selected_illegal_crossings,
            max_net_ids=min(4, max_repair_net_ids),
        )
        if repair_batches:
            selected_illegal_crossings = repair_batches[0]
    if (
        not selected_illegal_crossings
        or not state.crossing_plan_info.enabled
        or not state.run.repair_config.enabled
    ):
        return False
    repair_net_ids = _final_crossing_repair_net_ids(state, selected_illegal_crossings)
    if selected_reason == "not_perpendicular" and repair_net_ids:
        priority = _not_perpendicular_repair_priority(state, selected_illegal_crossings)
        repair_net_ids = [net_id for net_id in priority if net_id in repair_net_ids] + [
            net_id for net_id in repair_net_ids if net_id not in priority
        ]
    attempt: dict[str, object] = {
        "selected_reason": selected_reason,
        "illegal_reason_counts": dict(
            Counter(str(item.get("reason", "") or "unknown") for item in illegal_crossings)
        ),
        "selected_reason_counts": dict(
            Counter(str(item.get("reason", "") or "unknown") for item in selected_illegal_crossings)
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
    keepout_cells = _final_crossing_repair_keepout_cells(state, selected_illegal_crossings)
    attempt["keepout_cell_count"] = len(keepout_cells)
    if not keepout_cells:
        attempt["status"] = "skipped"
        attempt["reason"] = "no_keepout_cells"
        attempts.append(attempt)
        return False

    if not _reroute_for_crossing_repair(settings, state, repair_net_ids, keepout_cells, attempt):
        attempt["status"] = "failed"
        attempts.append(attempt)
        return False
    attempt["status"] = "routed"
    attempts.append(attempt)
    return True


def _refresh_realized_crossing_verification(
    settings: SessionSettings, state: SessionState
) -> list[dict[str, object]]:
    t_refresh_crossings_start = timing.pipeline_timer_start(settings)
    t_native_events_start = timing.pipeline_timer_start(settings)
    # Not caught: a failure here means crossing detection is broken, and
    # "no crossings" would let illegal geometry pass the final check.
    native_crossing_events = list(cast(Iterable[Any], state.kernel.router.crossing_events()))
    state.crossing_plan_info.native_crossing_events = native_crossing_events
    state.crossing_plan_info.native_crossing_event_count = len(native_crossing_events)
    # The Rust predicate's verdicts, which the classification below
    # reports instead of re-deciding.
    native_crossing_verdicts = list(cast(Iterable[Any], state.kernel.router.crossing_verdicts()))
    state.crossing_plan_info.native_crossing_verdicts = native_crossing_verdicts
    state.crossing_plan_info.native_crossing_verdict_count = len(native_crossing_verdicts)
    timing.record_pipeline_timing(
        settings,
        state,
        "realized_crossing_native_events",
        t_native_events_start,
    )
    t_insertion_loss_start = timing.pipeline_timer_start(settings)
    augment_insertion_loss_report(
        crossing_plan_info=state.crossing_plan_info,
        routed_records_by_net_id=state.search.bookkeeping.records_by_id,
        native_crossing_events=native_crossing_events,
    )
    timing.record_pipeline_timing(
        settings,
        state,
        "realized_crossing_insertion_loss",
        t_insertion_loss_start,
    )
    t_illegal_crossing_verify_start = timing.pipeline_timer_start(settings)
    illegal = populate_realized_intersections_from_native_crossing_events(
        crossing_plan_info=state.crossing_plan_info,
        routed_records_by_net_id=state.search.bookkeeping.records_by_id,
        native_crossing_events=native_crossing_events,
        realization_grid_spec=state.run.realization_grid_spec,
        native_crossing_verdicts=native_crossing_verdicts,
    )
    timing.record_pipeline_timing(
        settings,
        state,
        "realized_crossing_verify_intersections",
        t_illegal_crossing_verify_start,
    )
    t_realized_insertion_loss_start = timing.pipeline_timer_start(settings)
    augment_insertion_loss_report_from_realized_intersections(
        crossing_plan_info=state.crossing_plan_info,
        routed_records_by_net_id=state.search.bookkeeping.records_by_id,
    )
    timing.record_pipeline_timing(
        settings,
        state,
        "realized_crossing_realized_loss",
        t_realized_insertion_loss_start,
    )
    timing.record_pipeline_timing(
        settings,
        state,
        "realized_crossing_refresh_total",
        t_refresh_crossings_start,
    )
    return illegal


def repair_and_verify_final_geometry(
    settings: SessionSettings, state: SessionState, routed_net_records: list[RoutedNetRecord]
) -> VerifiedRecords:
    """Check the legality of the realized crossings and repair what is illegal, before realization.

    The router owns the crossing event model, so it can rip up and reroute
    here; the Python verifier of the flow stays the final check of the
    layout. Up to `final_crossing_repair_round_limit` rounds; the loop ends
    when a round changes nothing. Returns the records as they are now and
    the crossings that are still illegal (empty if the geometry is clean).
    """
    final_crossing_repair_round_limit = 12
    t_final_verification_block_start = timing.pipeline_timer_start(settings)
    illegal_realized_crossings = _refresh_realized_crossing_verification(settings, state)
    for _final_repair_round in range(final_crossing_repair_round_limit):
        if not illegal_realized_crossings:
            break
        if not _repair_final_illegal_crossings(settings, state, illegal_realized_crossings):
            break
        routed_net_records = state.search.bookkeeping.ordered_records()
        illegal_realized_crossings = _refresh_realized_crossing_verification(settings, state)
    timing.record_pipeline_timing(
        settings,
        state,
        "final_verification_block",
        t_final_verification_block_start,
    )
    return VerifiedRecords(routed_net_records, illegal_realized_crossings)
