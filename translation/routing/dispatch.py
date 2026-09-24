"""Phase 6: the native kernel dispatch and its per-route bookkeeping."""

from __future__ import annotations

import math
import time

from collections import deque
from pathlib import Path
from typing import Any, Iterable, Mapping, cast

from photonic_router.config import EngineSelection

from translation.route_rust_crossing_plan import _port_center_um
from translation.route_rust_debug_artifacts import (
    _cells_bbox,
    _ensure_dir,
    _rect_cell_count,
    _rect_overlap_cell_count,
    _route_cells_bbox,
)
from translation.route_rust_geometry import (
    _compress_centerline,
    _format_illegal_crossing_root_causes_line,
    _format_native_repair_trace_lines,
)
from translation.route_rust_obstacle_config import _schematic_instance_component_name
from translation.route_rust_records import _centerline_tuple
from translation.route_rust_types import (
    RouteJob,
    RouteTimingBucket,
    _as_float,
    route_attempt_record_from_route,
)

from translation.routing.crossing_plan_stage import _cells_in_raw_static_geometry
from translation.routing.handoff import _foreign_keepout_open_cells_for_spec, _states_and_openings
from translation.routing.route_jobs import _angle_to_step, _fanout_stub_bend_steps
from translation.routing import timing
from translation.routing.settings import SessionSettings
from translation.routing.state import SessionState

# `route_many_with_negotiated_repair_and_commit`'s own round budget --
# LiDAR's `runNRR` default (`.agent/execplans/2026-09-14-lidar-style-
# negotiated-ripup-endgame.md`, Milestone 3). Kept separate from
# `RipupRerouteConfig.max_rounds` (default 4, the chain's per-net
# `--ripup-max-rounds`), which is a different knob for a different engine:
# the negotiated engine's "round" is a whole-queue pass with a global
# rip-up at the end (LiDAR's `ripupfailedNets`), not a per-net repair-set
# retry count, so the chain's smaller default does not carry over.
NEGOTIATED_MAX_ROUNDS = 10


def _direction_reaches_target_ray(
    settings,
    state,
    *,
    source_x: int,
    source_y: int,
    source_angle: int,
    target_x: int,
    target_y: int,
    tolerance: int,
) -> bool:
    dx = target_x - source_x
    dy = target_y - source_y
    if abs(dx) <= tolerance and abs(dy) <= tolerance:
        return True
    dir_x, dir_y = _angle_to_step(settings, state, source_angle)
    if dir_x == 0 and dir_y == 0:
        return False
    if dir_x == 0:
        return abs(dx) <= tolerance and (dy > 0) == (dir_y > 0)
    if dir_y == 0:
        return abs(dy) <= tolerance and (dx > 0) == (dir_x > 0)
    return (
        (dx > 0) == (dir_x > 0)
        and (dy > 0) == (dir_y > 0)
        and abs(abs(dx) - abs(dy)) <= tolerance
    )


def _source_lower_bounds(
    settings,
    state,
    *,
    source_x: int,
    source_y: int,
    source_angle: int,
    target_x: int,
    target_y: int,
    target_angle: int,
) -> tuple[float, float]:
    grid_size_um = float(state.grid.grid_size_um)
    dx = target_x - source_x
    dy = target_y - source_y
    distance = math.hypot(float(dx), float(dy)) * grid_size_um
    heading_lower_bound = distance
    if str(settings.heuristic_mode) == "heading_aware":
        target_angle_ok = not bool(getattr(state.astar_cfg, "require_target_angle", True)) or (
            source_angle % 8 == target_angle % 8
        )
        reaches_target_ray = _direction_reaches_target_ray(settings, state,
            source_x=source_x,
            source_y=source_y,
            source_angle=source_angle,
            target_x=target_x,
            target_y=target_y,
            tolerance=max(0, int(getattr(state.astar_cfg, "target_tolerance_cells", 0))),
        )
        if not target_angle_ok or not reaches_target_ray:
            minimum_bend_units = 1.0 if settings.allow_45_degree_turns else 2.0
            bend_weight = float(getattr(state.astar_cfg, "bend_weight", 1.0)) * float(
                getattr(state.primitive_cfg, "bend_weight", 1.0)
            )
            heading_lower_bound += minimum_bend_units * bend_weight
    return distance, heading_lower_bound


def _foreign_keepout_cleanup_cells_for_spec(
    settings, state, port_spec: str
) -> set[tuple[int, int]]:
    cells = set(state.foreign_port_keepout_cells_by_spec.get(port_spec, set()))
    if not cells:
        return set()
    cells.difference_update(state.normal_port_runway_cells)
    cells.difference_update(state.fanout_stub_static_cells)
    cells.difference_update(_cells_in_raw_static_geometry(settings, state, cells))
    return cells


def _foreign_keepout_cleanup_cells_for_job(settings, state, job: RouteJob) -> list[tuple[int, int]]:
    cells = _foreign_keepout_cleanup_cells_for_spec(settings, state, f"{job.inst1},{job.port1}")
    cells.update(_foreign_keepout_cleanup_cells_for_spec(
        settings, state, f"{job.inst2},{job.port2}"
    ))
    return sorted(cells)


def _timing_start(settings, state) -> float:
    return time.perf_counter() if state.collect_timing else 0.0


def _record_native_batch_timings(settings, state, batch_result: dict[str, Any]) -> None:
    if not settings.collect_pipeline_timing:
        return
    raw_timings = batch_result.get("timings_s")
    if raw_timings is None:
        return
    for raw_name, raw_elapsed_s in dict(raw_timings).items():
        try:
            elapsed_s = float(raw_elapsed_s)
        except (TypeError, ValueError):
            continue
        name = f"native_batch_{raw_name}"
        state.route_nets_timings_s[name] = state.route_nets_timings_s.get(name, 0.0) + elapsed_s


def _report_long_straight_congestion(settings, state, batch_result: dict[str, Any]) -> None:
    records = [
        dict(record)
        for record in cast(
            Iterable[Mapping[str, object]],
            batch_result.get("long_straight_congestion", []),
        )
    ]
    if not records or not settings.verbose_route_diagnostics:
        return
    grouped: dict[int, list[dict[str, object]]] = {}
    for record in records:
        try:
            net_id = int(record.get("net_id", -1))
        except (TypeError, ValueError):
            continue
        grouped.setdefault(net_id, []).append(record)
    print("      - Long-straight congestion contributors:")
    for net_id in sorted(grouped):
        job = state.route_jobs_by_id.get(net_id)
        label = job.net_name if job is not None else f"net_id={net_id}"
        route_index = job.route_index if job is not None else "?"
        segments = grouped[net_id]
        marked_cells = sum(int(segment.get("marked_cells", 0) or 0) for segment in segments)
        lengths = ", ".join(
            f"{float(segment.get('length_um', 0.0) or 0.0):.1f}um" for segment in segments
        )
        print(
            "        "
            f"route[{route_index}] {label}: "
            f"{len(segments)} long straight(s), "
            f"marked_cells={marked_cells}, lengths=[{lengths}]"
        )


def _committed_dynamic_cells(
    settings, state, *, exclude_net_id: int | None = None
) -> set[tuple[int, int]]:
    return state.route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)


def _committed_dynamic_cells_for_attempt(
    settings,
    state,
    *,
    exclude_net_id: int | None = None,
) -> set[tuple[int, int]]:
    if state.route_bookkeeping.diagnostics_enabled:
        return state.route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)
    merged: set[tuple[int, int]] = set()
    for net_id in state.route_bookkeeping.records_by_id:
        if exclude_net_id is not None and int(net_id) == int(exclude_net_id):
            continue
        merged.update(_route_cells_from_router(settings, state, net_id))
    return merged


def _route_cells_from_router(settings, state, net_id: int) -> set[tuple[int, int]]:
    return {(int(cell[0]), int(cell[1])) for cell in state.router.get_net_cells(int(net_id))}


def _append_centerline_points(
    settings,
    state,
    out: list[tuple[float, float]],
    points: Iterable[tuple[float, float]],
) -> None:
    for raw_x, raw_y in points:
        point = (float(raw_x), float(raw_y))
        if out:
            last_x, last_y = out[-1]
            if math.hypot(point[0] - last_x, point[1] - last_y) <= 1.0e-9:
                continue
        out.append(point)


def _fanout_stubbed_centerline(
    settings,
    state,
    job: RouteJob,
    route_obj: Any,
) -> tuple[tuple[float, float], ...]:
    """Eagerly stitch a SOURCE fanout stub onto a route's own centerline.

    Deliberately does NOT do the same for a TARGET fanout stub: a
    target stub's anchor is not guaranteed to land exactly on a grid
    cell (a plain straight stub can only correct the forward axis, not
    the lateral one -- see `_straight_static_stub_centerline_um`), so
    stitching it in eagerly here would mark the record "already fully
    corrected" and skip the real endpoint-correction pass needed to
    close that gap. Instead, `_record_route` points this net's own
    effective target port at the anchor's exact position
    (`target_port_center_um_override`), letting ordinary endpoint
    correction resolve it the same way it would for any ordinary port,
    and `_append_target_fanout_stubs_after_correction` appends the
    fixed, pre-built anchor-to-true-port segment once that correction
    has completed. See
    `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
    """
    source_anchor = state.fanout_anchor_by_port_spec.get(f"{job.inst1},{job.port1}")
    if source_anchor is None:
        return ()
    route_primitive_centerline = getattr(state.router, "route_primitive_centerline", None)
    try:
        if route_primitive_centerline is not None:
            route_centerline = _centerline_tuple(route_primitive_centerline(route_obj))
        else:
            route_centerline = ()
    except Exception:
        route_centerline = ()
    if len(route_centerline) < 2:
        return ()
    points: list[tuple[float, float]] = []
    _append_centerline_points(settings, state, points, source_anchor.stub_centerline_um)
    _append_centerline_points(settings, state, points, route_centerline)
    centerline = _compress_centerline(tuple(points))
    return centerline if len(centerline) >= 2 else ()


def _routing_endpoint_center_um(
    settings,
    state,
    job: RouteJob,
    *,
    source: bool,
) -> tuple[float, float] | None:
    if source:
        port_spec = f"{job.inst1},{job.port1}"
        anchor = state.fanout_anchor_by_port_spec.get(port_spec)
        return anchor.center_um if anchor is not None else _port_center_um(job.source_port)
    port_spec = f"{job.inst2},{job.port2}"
    anchor = state.fanout_anchor_by_port_spec.get(port_spec)
    return anchor.center_um if anchor is not None else _port_center_um(job.target_port)


def _state_openings_for_job(
    settings,
    state,
    job: RouteJob,
) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
    return state.route_state_openings_by_id[int(job.net_id)]


def _clearance_exempt_cells_for_job(settings, state, job: RouteJob) -> list[tuple[int, int]]:
    return state.batch_clearance_exempt_cells_by_id.get(int(job.net_id), [])


def _clearance_exempt_cell_set_for_job(settings, state, job: RouteJob) -> set[tuple[int, int]]:
    return set(_clearance_exempt_cells_for_job(settings, state, job))


def _static_cells_in_rect(settings, state, min_x: int, max_x: int, min_y: int, max_y: int) -> int:
    if min_x > max_x or min_y > max_y:
        return 0
    if state.blocked_static_rects_for_diagnostics:
        return sum(
            _rect_overlap_cell_count(
                rect,
                min_x=min_x,
                max_x=max_x,
                min_y=min_y,
                max_y=max_y,
            )
            for rect in state.blocked_static_rects_for_diagnostics
        )
    return sum(
        1
        for x, y in state.static_blocked_cells_before_port_reservations
        if min_x <= x <= max_x and min_y <= y <= max_y
    )


def _cells_in_rect(
    settings,
    state,
    cells: set[tuple[int, int]],
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    if min_x > max_x or min_y > max_y:
        return 0
    return sum(1 for x, y in cells if min_x <= x <= max_x and min_y <= y <= max_y)


def _route_attempt_diagnostics(
    settings,
    state,
    job: RouteJob,
    route_obj: object | None,
    *,
    candidate_blockers: list[int] | None = None,
    ripup_ids: list[int] | None = None,
) -> dict[str, object]:
    dynamic_cells_before = _committed_dynamic_cells_for_attempt(
        settings, state, exclude_net_id=job.net_id
    )
    source_state, target_state, _, _, opened_cells = _state_openings_for_job(settings, state, job)
    source_x = int(source_state.x)
    source_y = int(source_state.y)
    source_angle = int(source_state.angle)
    target_x = int(target_state.x)
    target_y = int(target_state.y)
    target_angle = int(target_state.angle)
    span_x = abs(target_x - source_x)
    span_y = abs(target_y - source_y)
    span_bbox_min_x = min(source_x, target_x)
    span_bbox_max_x = max(source_x, target_x)
    span_bbox_min_y = min(source_y, target_y)
    span_bbox_max_y = max(source_y, target_y)
    span_bbox_area = _rect_cell_count(
        min_x=span_bbox_min_x,
        max_x=span_bbox_max_x,
        min_y=span_bbox_min_y,
        max_y=span_bbox_max_y,
    )
    window_min_x = int(getattr(route_obj, "last_window_min_x", 0)) if route_obj else 0
    window_max_x = int(getattr(route_obj, "last_window_max_x", -1)) if route_obj else -1
    window_min_y = int(getattr(route_obj, "last_window_min_y", 0)) if route_obj else 0
    window_max_y = int(getattr(route_obj, "last_window_max_y", -1)) if route_obj else -1
    window_area = int(getattr(route_obj, "last_window_area_cells", 0)) if route_obj else 0
    if window_area <= 0:
        window_area = _rect_cell_count(
            min_x=window_min_x,
            max_x=window_max_x,
            min_y=window_min_y,
            max_y=window_max_y,
        )
    window_static_cells = _static_cells_in_rect(settings, state,
        window_min_x,
        window_max_x,
        window_min_y,
        window_max_y,
    )
    window_dynamic_cells = _cells_in_rect(settings, state,
        dynamic_cells_before,
        min_x=window_min_x,
        max_x=window_max_x,
        min_y=window_min_y,
        max_y=window_max_y,
    )
    span_static_cells = _static_cells_in_rect(settings, state,
        span_bbox_min_x,
        span_bbox_max_x,
        span_bbox_min_y,
        span_bbox_max_y,
    )
    span_dynamic_cells = _cells_in_rect(settings, state,
        dynamic_cells_before,
        min_x=span_bbox_min_x,
        max_x=span_bbox_max_x,
        min_y=span_bbox_min_y,
        max_y=span_bbox_max_y,
    )
    route_cells = getattr(route_obj, "cells", None) if route_obj is not None else None
    route_bbox = _route_cells_bbox(route_cells or ())
    if route_bbox is None:
        route_bbox_min_x = 0
        route_bbox_max_x = -1
        route_bbox_min_y = 0
        route_bbox_max_y = -1
    else:
        route_bbox_min_x, route_bbox_max_x, route_bbox_min_y, route_bbox_max_y = route_bbox
    route_bbox_area = _rect_cell_count(
        min_x=route_bbox_min_x,
        max_x=route_bbox_max_x,
        min_y=route_bbox_min_y,
        max_y=route_bbox_max_y,
    )
    total_cost = float(getattr(route_obj, "total_cost", 0.0)) if route_obj is not None else None
    euclidean_lower_bound, heading_lower_bound = _source_lower_bounds(settings, state,
        source_x=source_x,
        source_y=source_y,
        source_angle=source_angle,
        target_x=target_x,
        target_y=target_y,
        target_angle=target_angle,
    )
    blocker_ids = list(candidate_blockers or [])
    victim_ids = list(ripup_ids or [])
    raw_dynamic_cells: set[tuple[int, int]] = set()
    raw_dynamic_refcount_gt1_cells: set[tuple[int, int]] = set()
    raw_core_cells: set[tuple[int, int]] = set()
    raw_core_refcount_gt1_cells: set[tuple[int, int]] = set()
    raw_net_route_cells: set[tuple[int, int]] = set()
    if hasattr(state.router, "raw_dynamic_obstacle_cells"):
        raw_dynamic_entries = [
            (int(x), int(y), int(refs))
            for x, y, refs in state.router.raw_dynamic_obstacle_cells()
        ]
        raw_dynamic_cells = {(x, y) for x, y, _ in raw_dynamic_entries}
        raw_dynamic_refcount_gt1_cells = {
            (x, y) for x, y, refs in raw_dynamic_entries if refs > 1
        }
    if hasattr(state.router, "raw_dynamic_core_cells"):
        raw_core_entries = [
            (int(x), int(y), int(refs)) for x, y, refs in state.router.raw_dynamic_core_cells()
        ]
        raw_core_cells = {(x, y) for x, y, _ in raw_core_entries}
        raw_core_refcount_gt1_cells = {(x, y) for x, y, refs in raw_core_entries if refs > 1}
    if hasattr(state.router, "all_net_route_cells"):
        for _, cells in state.router.all_net_route_cells():
            raw_net_route_cells.update((int(cell[0]), int(cell[1])) for cell in cells)
    raw_dynamic_without_owner = raw_dynamic_cells - raw_net_route_cells
    raw_net_route_without_dynamic = raw_net_route_cells - raw_dynamic_cells
    raw_core_without_dynamic = raw_core_cells - raw_dynamic_cells
    raw_dynamic_span_cells = {
        (x, y)
        for x, y in raw_dynamic_cells
        if span_bbox_min_x <= x <= span_bbox_max_x and span_bbox_min_y <= y <= span_bbox_max_y
    }
    raw_dynamic_without_owner_span_cells = {
        (x, y)
        for x, y in raw_dynamic_without_owner
        if span_bbox_min_x <= x <= span_bbox_max_x and span_bbox_min_y <= y <= span_bbox_max_y
    }
    return {
        "source_state": [source_x, source_y, source_angle],
        "target_state": [target_x, target_y, target_angle],
        "span_x_cells": span_x,
        "span_y_cells": span_y,
        "span_manhattan_cells": span_x + span_y,
        "span_bbox_area_cells": span_bbox_area,
        "span_static_cells": span_static_cells,
        "span_dynamic_cells": span_dynamic_cells,
        "route_bbox_min_x": route_bbox_min_x,
        "route_bbox_max_x": route_bbox_max_x,
        "route_bbox_min_y": route_bbox_min_y,
        "route_bbox_max_y": route_bbox_max_y,
        "route_bbox_width_cells": max(0, route_bbox_max_x - route_bbox_min_x + 1),
        "route_bbox_height_cells": max(0, route_bbox_max_y - route_bbox_min_y + 1),
        "route_bbox_area_cells": route_bbox_area,
        "route_bbox_to_span_bbox_area": (
            float(route_bbox_area) / float(span_bbox_area)
            if span_bbox_area > 0 and route_bbox_area > 0
            else None
        ),
        "opened_cells_count": len(opened_cells),
        "block_radius_cells": state.block_radius_cells,
        "dynamic_obstacle_search_expansion_radius_cells": (
            state.clearance_policy.dynamic_obstacle_search_expansion_radius_cells
        ),
        "dynamic_route_commit_keepout_radius_cells": (
            state.clearance_policy.dynamic_route_commit_keepout_radius_cells
        ),
        "dynamic_route_core_radius_cells": (
            state.clearance_policy.dynamic_route_core_radius_cells
        ),
        "bend_radius_cells": state.bend_radius_cells,
        "window_width_cells": max(0, window_max_x - window_min_x + 1),
        "window_height_cells": max(0, window_max_y - window_min_y + 1),
        "window_area_cells": window_area,
        "window_to_span_bbox_area": (
            float(window_area) / float(span_bbox_area)
            if span_bbox_area > 0 and window_area > 0
            else None
        ),
        "route_bbox_to_window_area": (
            float(route_bbox_area) / float(window_area)
            if window_area > 0 and route_bbox_area > 0
            else None
        ),
        "window_static_cells": window_static_cells,
        "window_dynamic_cells": window_dynamic_cells,
        "window_static_density": (
            float(window_static_cells) / float(window_area) if window_area > 0 else None
        ),
        "window_dynamic_density": (
            float(window_dynamic_cells) / float(window_area) if window_area > 0 else None
        ),
        "total_cost": total_cost,
        "euclidean_lower_bound_cost": euclidean_lower_bound,
        "heading_lower_bound_cost": heading_lower_bound,
        "euclidean_lower_bound_to_cost": (
            euclidean_lower_bound / total_cost
            if total_cost is not None and total_cost > 0.0
            else None
        ),
        "heading_lower_bound_to_cost": (
            heading_lower_bound / total_cost
            if total_cost is not None and total_cost > 0.0
            else None
        ),
        "heading_lower_bound_gap_cost": (
            total_cost - heading_lower_bound if total_cost is not None else None
        ),
        "committed_dynamic_cells_before": len(dynamic_cells_before),
        "raw_dynamic_obstacle_cells_before": len(raw_dynamic_cells),
        "raw_dynamic_core_cells_before": len(raw_core_cells),
        "raw_net_route_cells_before": len(raw_net_route_cells),
        "raw_dynamic_refcount_gt1_count": len(raw_dynamic_refcount_gt1_cells),
        "raw_dynamic_refcount_gt1_bbox": _cells_bbox(raw_dynamic_refcount_gt1_cells),
        "raw_core_refcount_gt1_count": len(raw_core_refcount_gt1_cells),
        "raw_dynamic_without_owner_count": len(raw_dynamic_without_owner),
        "raw_dynamic_without_owner_bbox": _cells_bbox(raw_dynamic_without_owner),
        "raw_dynamic_without_owner_sample": sorted(raw_dynamic_without_owner)[:12],
        "raw_net_route_without_dynamic_count": len(raw_net_route_without_dynamic),
        "raw_net_route_without_dynamic_bbox": _cells_bbox(raw_net_route_without_dynamic),
        "raw_core_without_dynamic_count": len(raw_core_without_dynamic),
        "span_raw_dynamic_cells": len(raw_dynamic_span_cells),
        "span_raw_dynamic_without_owner_count": len(raw_dynamic_without_owner_span_cells),
        "span_raw_dynamic_without_owner_bbox": _cells_bbox(
            raw_dynamic_without_owner_span_cells
        ),
        "candidate_blocker_count": len(blocker_ids),
        "candidate_blocker_net_ids": blocker_ids,
        "candidate_blocker_route_indices": [
            state.route_jobs_by_id[net_id].route_index
            for net_id in blocker_ids
            if net_id in state.route_jobs_by_id
        ],
        "ripup_victim_count": len(victim_ids),
        "ripup_victim_net_ids": victim_ids,
        "ripup_victim_route_indices": [
            state.route_jobs_by_id[net_id].route_index
            for net_id in victim_ids
            if net_id in state.route_jobs_by_id
        ],
    }


def _write_route_diagnostics(
    settings,
    state,
    *,
    job: RouteJob,
    source_state: Any,
    target_state: Any,
    opened_candidate_cells: set[tuple[int, int]],
    dynamic_clearance_exempt_cells: set[tuple[int, int]],
    opened_cells_set: set[tuple[int, int]],
    diag_txt: Path | None,
    status: str,
    error_text: str | None = None,
    route_cells: set[tuple[int, int]] | None = None,
    route_obj: Any | None = None,
    repair_note: str | None = None,
) -> None:
    if diag_txt is None:
        return
    port1_spec = f"{job.inst1},{job.port1}"
    port2_spec = f"{job.inst2},{job.port2}"
    source_anchor_cell = (int(source_state.x), int(source_state.y))
    target_anchor_cell = (int(target_state.x), int(target_state.y))
    committed_dynamic_cells = _committed_dynamic_cells(settings, state, exclude_net_id=job.net_id)
    if state.diagnostics_enabled:
        opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
        opened_candidate_static_overlap = _cells_in_raw_static_geometry(settings, state,
            opened_candidate_cells
        )
        opened_static_overlap = _cells_in_raw_static_geometry(settings, state, opened_cells_set)
        opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
        dynamic_exempt_dynamic_overlap = (
            dynamic_clearance_exempt_cells & committed_dynamic_cells
        )
    else:
        opened_candidate_dynamic_overlap = set()
        opened_candidate_static_overlap = set()
        opened_static_overlap = set()
        opened_dynamic_overlap = set()
        dynamic_exempt_dynamic_overlap = set()

    route_cells = route_cells or set()
    route_static_overlap = _cells_in_raw_static_geometry(settings, state, route_cells)
    route_overlap_with_candidate_opened_static = route_cells & opened_candidate_static_overlap
    route_overlap_with_effective_opened_static = route_cells & opened_static_overlap
    route_dynamic_overlap = route_cells & committed_dynamic_cells
    route_overlap_with_candidate_opened_dynamic = route_cells & opened_candidate_dynamic_overlap
    route_overlap_with_effective_opened_dynamic = route_cells & opened_dynamic_overlap
    route_overlap_with_dynamic_exempt = route_cells & dynamic_clearance_exempt_cells
    current_endpoint_foreign_keepout_cells = set(
        _foreign_keepout_open_cells_for_spec(settings, state, port1_spec)
    )
    current_endpoint_foreign_keepout_cells.update(
        _foreign_keepout_open_cells_for_spec(settings, state, port2_spec)
    )
    foreign_keepout_open_cells = current_endpoint_foreign_keepout_cells & opened_cells_set
    current_port_runway_cells = set(state.port_runway_cells_by_spec.get(port1_spec, set()))
    current_port_runway_cells.update(state.port_runway_cells_by_spec.get(port2_spec, set()))
    source_sibling_port_runway_cells: set[tuple[int, int]] = set()
    for cluster_port_spec in state.dense_source_cluster_specs_by_port_spec.get(
        port1_spec, set()
    ):
        if cluster_port_spec == port1_spec:
            continue
        source_sibling_port_runway_cells.update(
            state.port_runway_cells_by_spec.get(cluster_port_spec, set())
        )
    target_sibling_port_runway_cells: set[tuple[int, int]] = set()
    for cluster_port_spec in state.dense_source_cluster_specs_by_port_spec.get(
        port2_spec, set()
    ):
        if cluster_port_spec == port2_spec:
            continue
        target_sibling_port_runway_cells.update(
            state.port_runway_cells_by_spec.get(cluster_port_spec, set())
        )
    sibling_port_runway_cells = (
        source_sibling_port_runway_cells | target_sibling_port_runway_cells
    )
    current_port_runway_dynamic_overlap = current_port_runway_cells & committed_dynamic_cells
    sibling_port_runway_dynamic_overlap = sibling_port_runway_cells & committed_dynamic_cells
    route_overlap_current_port_runway = route_cells & current_port_runway_cells
    route_overlap_sibling_port_runway = route_cells & sibling_port_runway_cells
    route_segments: list[str] = []
    if route_obj is not None:
        for segment in cast(list[object], getattr(route_obj, "segments", []) or []):
            try:
                entry = dict(cast(Any, segment))
            except (TypeError, ValueError):
                continue
            route_segments.append(
                "{kind}:{start}->{end}@{start_angle}->{end_angle}".format(
                    kind=entry.get("kind"),
                    start=entry.get("start"),
                    end=entry.get("end"),
                    start_angle=entry.get("start_angle"),
                    end_angle=entry.get("end_angle"),
                )
            )

    def _relative_line_cells(
        *,
        start: tuple[int, int],
        direction: tuple[int, int],
        cells: int,
    ) -> list[tuple[int, int]]:
        return [
            (int(start[0]) + int(direction[0]) * step, int(start[1]) + int(direction[1]) * step)
            for step in range(max(0, int(cells)) + 1)
        ]

    def _unique_cells(
        cells: Iterable[tuple[int, int]],
    ) -> list[tuple[int, int]]:
        seen: set[tuple[int, int]] = set()
        unique: list[tuple[int, int]] = []
        for cell in cells:
            normalized = (int(cell[0]), int(cell[1]))
            if normalized in seen:
                continue
            seen.add(normalized)
            unique.append(normalized)
        return unique

    def _first_move_footprint(
        *,
        source: tuple[int, int],
        source_angle: int,
        kind: str,
        cells: int = 0,
        delta: int = 0,
    ) -> tuple[tuple[int, int], int, list[tuple[int, int]]]:
        start_dir = _angle_to_step(settings, state, source_angle)
        if kind == "straight":
            relative = _relative_line_cells(
                start=(0, 0),
                direction=start_dir,
                cells=cells,
            )
            end = relative[-1]
            end_angle = int(source_angle) % 8
        else:
            end_angle = (int(source_angle) + int(delta)) % 8
            end_dir = _angle_to_step(settings, state, end_angle)
            radius = max(0, int(state.bend_radius_cells))
            first_leg = _relative_line_cells(
                start=(0, 0),
                direction=start_dir,
                cells=radius,
            )
            corner = (start_dir[0] * radius, start_dir[1] * radius)
            second_leg = _relative_line_cells(
                start=corner,
                direction=end_dir,
                cells=radius,
            )
            relative = _unique_cells([*first_leg, *second_leg])
            end = relative[-1]
        absolute = [(int(source[0]) + int(dx), int(source[1]) + int(dy)) for dx, dy in relative]
        return (
            (int(source[0]) + int(end[0]), int(source[1]) + int(end[1])),
            end_angle,
            absolute,
        )

    def _dynamic_owners_for_cells(
        cells: set[tuple[int, int]],
    ) -> dict[int, list[tuple[int, int]]]:
        owners: dict[int, list[tuple[int, int]]] = {}
        for net_id in state.route_bookkeeping.records_by_id:
            if int(net_id) == int(job.net_id):
                continue
            overlap = cells & _route_cells_from_router(settings, state, int(net_id))
            if overlap:
                owners[int(net_id)] = sorted(overlap)
        return owners

    def _format_post_crossing_orthogonal_candidates() -> list[str]:
        if not state.diagnostics_enabled or not route_dynamic_overlap:
            return []
        if int(source_state.angle) != 0 or int(target_state.angle) != 0:
            return []
        if source_anchor_cell[1] == target_anchor_cell[1]:
            return []
        crossing_cells = sorted(route_dynamic_overlap)
        if len(crossing_cells) != 1:
            return []
        cross_x, cross_y = crossing_cells[0]
        dy_sign = 1 if target_anchor_cell[1] > cross_y else -1
        dx_sign = 1 if target_anchor_cell[0] > cross_x else -1
        if dx_sign != 1:
            return []
        radius = max(1, int(state.bend_radius_cells))
        routing_static_cells = set(state.static_blocked_cells_before_port_reservations)
        routing_static_cells.update(state.debug_port_keepout_cells)
        routing_static_cells.update(state.foreign_port_keepout_static_cells)
        routing_static_cells.update(state.fanout_stub_static_cells)
        opened_search_cells = set(opened_cells_set)
        allowed_dynamic_cells = set(route_dynamic_overlap)
        lines: list[str] = []
        min_start = cross_x + radius
        max_start = target_anchor_cell[0] - 2 * radius
        for bend_start_x in range(min_start, max_start + 1):
            if target_anchor_cell[1] - dy_sign * radius == cross_y:
                continue
            cells: set[tuple[int, int]] = set()
            for x in range(source_anchor_cell[0], bend_start_x + radius + 1):
                cells.add((x, cross_y))
            first_corner_x = bend_start_x + radius
            first_corner_y = cross_y
            first_end_y = cross_y + dy_sign * radius
            for step in range(0, radius + 1):
                cells.add((first_corner_x, first_corner_y + dy_sign * step))
            vertical_start_y = first_end_y
            second_start_y = target_anchor_cell[1] - dy_sign * radius
            y0, y1 = sorted((vertical_start_y, second_start_y))
            for y in range(y0, y1 + 1):
                cells.add((first_corner_x, y))
            second_end_x = first_corner_x + radius
            for step in range(0, radius + 1):
                cells.add((first_corner_x + step, second_start_y + dy_sign * step))
            for x in range(second_end_x, target_anchor_cell[0] + 1):
                cells.add((x, target_anchor_cell[1]))
            static_blockers = (cells & routing_static_cells) - opened_search_cells
            dynamic_blockers = (
                (cells & committed_dynamic_cells)
                - allowed_dynamic_cells
                - (cells & opened_search_cells)
            )
            lines.append(
                "post_crossing_90_candidate="
                f"bend_start_x={bend_start_x}; "
                f"cells={len(cells)}; "
                f"static_blockers={sorted(static_blockers)[:24]}; "
                f"static_blocker_count={len(static_blockers)}; "
                f"dynamic_blockers={sorted(dynamic_blockers)[:24]}; "
                f"dynamic_blocker_count={len(dynamic_blockers)}; "
                f"dynamic_blocker_owners={_dynamic_owners_for_cells(dynamic_blockers)}"
            )
        return lines

    def _format_target_bend_footprints() -> list[str]:
        if not state.diagnostics_enabled:
            return []
        radius = max(1, int(state.bend_radius_cells))
        routing_static_cells = set(state.static_blocked_cells_before_port_reservations)
        routing_static_cells.update(state.debug_port_keepout_cells)
        routing_static_cells.update(state.foreign_port_keepout_static_cells)
        routing_static_cells.update(state.fanout_stub_static_cells)
        opened_search_cells = set(opened_cells_set)

        def turn_cells(
            start: tuple[int, int],
            start_angle: int,
            delta: int,
        ) -> set[tuple[int, int]]:
            start_dir = _angle_to_step(settings, state, start_angle)
            end_angle = (int(start_angle) + int(delta)) % 8
            end_dir = _angle_to_step(settings, state, end_angle)
            cells: set[tuple[int, int]] = set()
            for step in range(radius + 1):
                cells.add(
                    (
                        int(start[0]) + start_dir[0] * step,
                        int(start[1]) + start_dir[1] * step,
                    )
                )
            corner = (
                int(start[0]) + start_dir[0] * radius,
                int(start[1]) + start_dir[1] * radius,
            )
            for step in range(radius + 1):
                cells.add(
                    (
                        corner[0] + end_dir[0] * step,
                        corner[1] + end_dir[1] * step,
                    )
                )
            return cells

        target = target_anchor_cell
        specs = [
            (
                "end_from_below_to_port",
                (target[0] - radius, target[1] - radius),
                2,
                -2,
            ),
            (
                "end_from_above_to_port",
                (target[0] - radius, target[1] + radius),
                6,
                2,
            ),
            ("target_out_down", target, 0, -2),
            ("target_out_up", target, 0, 2),
        ]
        lines: list[str] = []
        for label, start, angle, delta in specs:
            cells = turn_cells(start, angle, delta)
            static_blockers = (cells & routing_static_cells) - opened_search_cells
            dynamic_blockers = (
                (cells & committed_dynamic_cells)
                - (cells & opened_search_cells)
                - route_dynamic_overlap
            )
            lines.append(
                "target_bend_footprint="
                f"label={label}; start={start}; angle={angle}; delta={delta}; "
                f"cells={sorted(cells)}; "
                f"static_blockers={sorted(static_blockers)}; "
                f"static_blocker_count={len(static_blockers)}; "
                f"dynamic_blockers={sorted(dynamic_blockers)}; "
                f"dynamic_blocker_count={len(dynamic_blockers)}; "
                f"dynamic_blocker_owners={_dynamic_owners_for_cells(dynamic_blockers)}"
            )
        return lines

    def _format_first_move_debug() -> list[str]:
        if not state.diagnostics_enabled:
            return []
        source = source_anchor_cell
        source_angle = int(source_state.angle)
        source_key = source_anchor_cell
        target_key = target_anchor_cell
        opened_search_cells = {
            cell
            for cell in opened_cells_set
            if cell == source_key or cell == target_key or cell not in committed_dynamic_cells
        }
        routing_static_cells = set(state.static_blocked_cells_before_port_reservations)
        routing_static_cells.update(state.debug_port_keepout_cells)
        routing_static_cells.update(state.foreign_port_keepout_static_cells)
        routing_static_cells.update(state.fanout_stub_static_cells)
        primitive_specs: list[tuple[str, str, int, int]] = [
            ("straight_short", "straight", int(state.primitive_cfg.straight_short_cells), 0),
            ("straight_long", "straight", int(state.primitive_cfg.straight_long_cells), 0),
            ("turn45_left", "turn", 0, 1),
            ("turn45_right", "turn", 0, -1),
            ("turn90_left", "turn", 0, 2),
            ("turn90_right", "turn", 0, -2),
        ]
        debug_lines: list[str] = []
        for label, kind, cells, delta in primitive_specs:
            if not settings.allow_45_degree_turns and abs(int(delta)) == 1:
                continue
            end_cell, end_angle, footprint = _first_move_footprint(
                source=source,
                source_angle=source_angle,
                kind=kind,
                cells=cells,
                delta=delta,
            )
            footprint_set = set(footprint)
            static_overlap = _cells_in_raw_static_geometry(settings, state, footprint_set)
            routing_static_overlap = footprint_set & routing_static_cells
            effective_static_blockers = routing_static_overlap - opened_search_cells
            dynamic_overlap = footprint_set & committed_dynamic_cells
            effective_dynamic_blockers = (
                dynamic_overlap
                - dynamic_clearance_exempt_cells
                - (footprint_set & opened_search_cells)
            )
            owner_cells = _dynamic_owners_for_cells(dynamic_overlap)
            debug_lines.append(
                "first_move_{label}="
                "end=({end_x},{end_y},{end_angle}); "
                "footprint={footprint}; "
                "static={static}; "
                "routing_static={routing_static}; "
                "static_blockers={static_blockers}; "
                "dynamic={dynamic}; "
                "dynamic_blockers={dynamic_blockers}; "
                "dynamic_owners={owners}; "
                "opened={opened}; "
                "opened_search={opened_search}; "
                "dynamic_exempt={dynamic_exempt}".format(
                    label=label,
                    end_x=end_cell[0],
                    end_y=end_cell[1],
                    end_angle=end_angle,
                    footprint=footprint,
                    static=sorted(static_overlap),
                    routing_static=sorted(routing_static_overlap),
                    static_blockers=sorted(effective_static_blockers),
                    dynamic=sorted(dynamic_overlap),
                    dynamic_blockers=sorted(effective_dynamic_blockers),
                    owners=owner_cells,
                    opened=sorted(footprint_set & opened_cells_set),
                    opened_search=sorted(footprint_set & opened_search_cells),
                    dynamic_exempt=sorted(footprint_set & dynamic_clearance_exempt_cells),
                )
            )
        return debug_lines

    lines = [
        f"net_name={job.net_name}",
        f"status={status}",
        f"source_spec={port1_spec}",
        f"target_spec={port2_spec}",
        f"source_component={_schematic_instance_component_name(settings.schematic, job.inst1)}",
        f"target_component={_schematic_instance_component_name(settings.schematic, job.inst2)}",
        f"source_access_rule={state.port_access_rule_by_spec.get(port1_spec)}",
        f"target_access_rule={state.port_access_rule_by_spec.get(port2_spec)}",
        f"foreign_port_keepout_cells={int(settings.foreign_port_keepout_cells)}",
        f"fanout_access_mode={settings.fanout_access_mode_normalized}",
        f"fanout_stub_bend_degrees={45 * int(_fanout_stub_bend_steps(settings, state))}",
        f"fanout_anchor_port_count={len(state.fanout_anchor_by_port_spec)}",
        f"fanout_stub_center_cell_count={len(state.fanout_stub_center_cells)}",
        f"fanout_stub_static_cell_count={len(state.fanout_stub_static_cells)}",
        f"source_fanout_anchor={f'{job.inst1},{job.port1}' in state.fanout_anchor_by_port_spec}",
        f"target_fanout_anchor={f'{job.inst2},{job.port2}' in state.fanout_anchor_by_port_spec}",
        "source_dense_port_runway_cells="
        f"{state.dense_source_port_runway_length_by_spec.get(port1_spec)}",
        "target_dense_port_runway_cells="
        f"{state.dense_target_port_runway_length_by_spec.get(port2_spec)}",
        "source_dense_source_cluster_size="
        f"{len(state.dense_source_cluster_specs_by_port_spec.get(port1_spec, set()))}",
        "target_dense_source_cluster_size="
        f"{len(state.dense_source_cluster_specs_by_port_spec.get(port2_spec, set()))}",
        f"foreign_port_keepout_static_count={len(state.foreign_port_keepout_static_cells)}",
        f"foreign_port_keepout_open_count={len(foreign_keepout_open_cells)}",
        f"source_state=({source_anchor_cell[0]}, {source_anchor_cell[1]}, {int(source_state.angle)})",
        f"target_state=({target_anchor_cell[0]}, {target_anchor_cell[1]}, {int(target_state.angle)})",
        f"opened_candidate_cells_count={len(opened_candidate_cells)}",
        f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
        f"opened_candidate_static_overlap_bbox={_cells_bbox(opened_candidate_static_overlap)}",
        f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
        f"opened_candidate_dynamic_overlap_bbox={_cells_bbox(opened_candidate_dynamic_overlap)}",
        f"opened_cells_count={len(opened_cells_set)}",
        f"opened_cells={sorted(opened_cells_set)}",
        f"opened_static_overlap_count={len(opened_static_overlap)}",
        f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
        f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
        f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
        f"opened_dynamic_overlap_owners={_dynamic_owners_for_cells(opened_dynamic_overlap)}",
        f"current_port_runway_dynamic_overlap_count={len(current_port_runway_dynamic_overlap)}",
        "current_port_runway_dynamic_overlap_bbox="
        f"{_cells_bbox(current_port_runway_dynamic_overlap)}",
        f"sibling_port_runway_dynamic_overlap_count={len(sibling_port_runway_dynamic_overlap)}",
        "sibling_port_runway_dynamic_overlap_bbox="
        f"{_cells_bbox(sibling_port_runway_dynamic_overlap)}",
        f"dynamic_clearance_exempt_cells_count={len(dynamic_clearance_exempt_cells)}",
        f"dynamic_clearance_exempt_cells_bbox={_cells_bbox(dynamic_clearance_exempt_cells)}",
        f"dynamic_clearance_exempt_dynamic_overlap_count={len(dynamic_exempt_dynamic_overlap)}",
        f"dynamic_clearance_exempt_dynamic_overlap_bbox={_cells_bbox(dynamic_exempt_dynamic_overlap)}",
        f"route_cells_count={len(route_cells)}",
        f"route_static_blocked_overlap_count={len(route_static_overlap)}",
        f"route_static_blocked_overlap_bbox={_cells_bbox(route_static_overlap)}",
        f"route_dynamic_overlap_count={len(route_dynamic_overlap)}",
        f"route_dynamic_overlap_bbox={_cells_bbox(route_dynamic_overlap)}",
        f"route_dynamic_overlap_owners={_dynamic_owners_for_cells(route_dynamic_overlap)}",
        f"route_overlap_current_port_runway_count={len(route_overlap_current_port_runway)}",
        "route_overlap_current_port_runway_bbox="
        f"{_cells_bbox(route_overlap_current_port_runway)}",
        f"route_overlap_sibling_port_runway_count={len(route_overlap_sibling_port_runway)}",
        "route_overlap_sibling_port_runway_bbox="
        f"{_cells_bbox(route_overlap_sibling_port_runway)}",
        f"route_overlap_candidate_opened_static_count={len(route_overlap_with_candidate_opened_static)}",
        f"route_overlap_effective_opened_static_count={len(route_overlap_with_effective_opened_static)}",
        f"route_overlap_candidate_opened_dynamic_count={len(route_overlap_with_candidate_opened_dynamic)}",
        f"route_overlap_effective_opened_dynamic_count={len(route_overlap_with_effective_opened_dynamic)}",
        f"route_overlap_dynamic_clearance_exempt_count={len(route_overlap_with_dynamic_exempt)}",
    ]
    lines.extend(_format_post_crossing_orthogonal_candidates())
    lines.extend(_format_target_bend_footprints())
    lines.extend(_format_first_move_debug())
    if route_segments:
        lines.append("route_segments=" + "; ".join(route_segments))
    if repair_note is not None:
        lines.append(f"repair={repair_note}")
    if error_text is not None:
        lines.append(f"error={error_text}")
    diag_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _record_route(
    settings,
    state,
    job: RouteJob,
    route_obj: Any,
    opened_cells: list[tuple[int, int]],
    *,
    corrected_centerline_um: tuple[tuple[float, float], ...] = (),
    corrected_total_length_um: float | None = None,
) -> None:
    if not corrected_centerline_um:
        corrected_centerline_um = _fanout_stubbed_centerline(settings, state, job, route_obj)
    if (
        not corrected_centerline_um
        and not settings.enable_checked_endpoint_correction
        and hasattr(state.router, "route_primitive_centerline")
    ):
        try:
            corrected_centerline_um = _centerline_tuple(
                state.router.route_primitive_centerline(route_obj)
            )
        except Exception:
            corrected_centerline_um = ()
        if corrected_centerline_um and hasattr(state.router, "centerline_length_um"):
            try:
                corrected_total_length_um = float(
                    state.router.centerline_length_um(list(corrected_centerline_um))
                )
            except Exception:
                corrected_total_length_um = None
    target_anchor = state.fanout_anchor_by_port_spec.get(f"{job.inst2},{job.port2}")
    target_port_center_um_override = (
        target_anchor.center_um if target_anchor is not None else None
    )
    state.route_bookkeeping.record_route(
        job,
        route_obj,
        opened_cells,
        route_cells=_route_cells_from_router(settings, state, job.net_id)
        if state.track_dynamic_cells
        else None,
        corrected_centerline_um=corrected_centerline_um,
        corrected_total_length_um=corrected_total_length_um,
        target_port_center_um_override=target_port_center_um_override,
    )


def _export_route_svg(
    settings,
    state,
    job: RouteJob,
    route_obj: Any,
    *,
    suffix: str = "",
    obstacle_cells: set[tuple[int, int]] | None = None,
    opened_cells: list[tuple[int, int]] | None = None,
) -> None:
    should_export = state.debug_path is not None and (
        settings.debug_route_indices is None
        or job.route_index in settings.debug_route_indices
    )
    if not should_export:
        return
    route_dir = state.debug_path / "routes"
    _ensure_dir(route_dir)
    route_svg = route_dir / f"{settings.debug_prefix}_{job.net_name}{suffix}.svg"
    if obstacle_cells is not None and hasattr(
        state.router, "export_debug_svg_with_obstacle_cells"
    ):
        svg_text = state.router.export_debug_svg_with_obstacle_cells(
            route_obj,
            sorted(obstacle_cells),
        )
    else:
        svg_text = state.router.export_debug_svg(route_obj)
    if opened_cells is None:
        try:
            _, _, _, _, opened_cells = _state_openings_for_job(settings, state, job)
        except Exception:
            opened_cells = []
    if settings.debug_stop_after_route_index is not None and int(job.route_index) == int(
        settings.debug_stop_after_route_index
    ):
        next_job = state.full_route_jobs_by_route_index.get(
            int(settings.debug_stop_after_route_index) + 1
        )
        if next_job is not None:
            try:
                _, _, _, _, opened_cells = _states_and_openings(settings, state, next_job)
            except Exception:
                pass
    red_keepout_cells = state.debug_port_keepout_cells - {
        (int(cell[0]), int(cell[1])) for cell in opened_cells
    }
    if red_keepout_cells:
        overlay = ['<g id="port-keepout-cells">']
        for gx, gy in sorted(red_keepout_cells):
            if 0 <= gx < state.grid_width and 0 <= gy < state.grid_height:
                svg_y = state.grid_height - gy - 1
                overlay.append(
                    f'<rect class="port-keepout" x="{gx}" y="{svg_y}" '
                    'width="1" height="1" fill="#d93025" opacity="0.38" />'
                )
        overlay.append("</g>")
        overlay_text = "".join(overlay)
        if "</svg>" in svg_text and 'id="port-keepout-cells"' not in svg_text:
            svg_text = svg_text.replace("</svg>", overlay_text + "</svg>", 1)
    route_svg.write_text(svg_text, encoding="utf-8")
    state.route_svgs.append(route_svg)


def _route_engine_summary(settings, state, route_obj: Any) -> str:
    expanded_states = int(getattr(route_obj, "expanded_states", 0))
    route_kind = "simple" if expanded_states == 0 else "astar"
    length_um = _as_float(getattr(route_obj, "total_length_um", 0.0), 0.0)
    total_cost = _as_float(getattr(route_obj, "total_cost", 0.0), 0.0)
    return (
        f"{route_kind} "
        f"length={length_um:.3f}um "
        f"cost={total_cost:.3f} "
        f"expanded={expanded_states}"
    )


def _corridor_clearance_diagnostic(
    settings,
    state,
    source_state: Any,
    target_state: Any,
    blocked_cells: set[tuple[int, int]],
    *,
    max_radius: int | None = None,
) -> dict[str, Any]:
    if max_radius is None:
        max_radius = max(4, int(state.bend_radius_cells) + 2)
    else:
        max_radius = int(max_radius)

    source = (int(source_state.x), int(source_state.y))
    target = (int(target_state.x), int(target_state.y))
    neighbors = [
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    ]

    def inflate(radius: int) -> set[tuple[int, int]]:
        if radius <= 0:
            inflated = set(blocked_cells)
        else:
            inflated = set()
            for x, y in blocked_cells:
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        inflated.add((x + dx, y + dy))
        inflated.discard(source)
        inflated.discard(target)
        return inflated

    def reachable_from(
        start: tuple[int, int],
        blocked: set[tuple[int, int]],
    ) -> set[tuple[int, int]]:
        sx, sy = start
        if not (0 <= sx < state.grid_width and 0 <= sy < state.grid_height):
            return set()
        if start in blocked:
            return set()
        reached = {start}
        queue: deque[tuple[int, int]] = deque([start])
        while queue:
            x, y = queue.popleft()
            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy
                neighbor = (nx, ny)
                if (
                    0 <= nx < state.grid_width
                    and 0 <= ny < state.grid_height
                    and neighbor not in blocked
                    and neighbor not in reached
                ):
                    reached.add(neighbor)
                    queue.append(neighbor)
        return reached

    last_connected_radius: int | None = None
    first_disconnected_radius: int | None = None
    disconnected_blocked: set[tuple[int, int]] | None = None
    for radius in range(max_radius + 1):
        inflated = inflate(radius)
        reachable = reachable_from(source, inflated)
        if target in reachable:
            last_connected_radius = radius
            continue
        first_disconnected_radius = radius
        disconnected_blocked = inflated
        break

    source_region_size: int | None = None
    target_region_size: int | None = None
    source_region_min_distance_to_target: int | None = None
    target_region_min_distance_to_source: int | None = None
    if first_disconnected_radius is not None and disconnected_blocked is not None:
        source_region = reachable_from(source, disconnected_blocked)
        target_region = reachable_from(target, disconnected_blocked)
        source_region_size = len(source_region)
        target_region_size = len(target_region)
        if target in source_region:
            source_region_min_distance_to_target = 0
        elif source_region:
            source_region_min_distance_to_target = min(
                abs(x - target[0]) + abs(y - target[1]) for x, y in source_region
            )
        if source in target_region:
            target_region_min_distance_to_source = 0
        elif target_region:
            target_region_min_distance_to_source = min(
                abs(x - source[0]) + abs(y - source[1]) for x, y in target_region
            )

    return {
        "max_radius_checked": max_radius,
        "last_connected_radius": last_connected_radius,
        "first_disconnected_radius": first_disconnected_radius,
        "source_region_size": source_region_size,
        "target_region_size": target_region_size,
        "source_region_min_distance_to_target": source_region_min_distance_to_target,
        "target_region_min_distance_to_source": target_region_min_distance_to_source,
    }


def _write_failed_log(
    settings,
    state,
    job: RouteJob,
    source_state: Any,
    target_state: Any,
    opened_candidate_cells: set[tuple[int, int]],
    opened_cells: list[tuple[int, int]],
    error_text: str,
) -> None:
    if state.debug_path is None:
        return
    route_dir = state.debug_path / "routes"
    _ensure_dir(route_dir)
    port1_spec = f"{job.inst1},{job.port1}"
    port2_spec = f"{job.inst2},{job.port2}"
    fail_txt = route_dir / f"{settings.debug_prefix}_{job.net_name}_FAILED.txt"
    committed_dynamic_cells = _committed_dynamic_cells(settings, state)
    opened_candidate_static_overlap = (
        opened_candidate_cells & state.static_blocked_cells_before_port_reservations
    )
    opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
    opened_cells_set = set(opened_cells)
    corridor_diagnostic = _corridor_clearance_diagnostic(settings, state,
        source_state,
        target_state,
        (state.static_blocked_cells_before_port_reservations - opened_cells_set)
        | committed_dynamic_cells,
    )
    opened_static_overlap = (
        opened_cells_set & state.static_blocked_cells_before_port_reservations
    )
    opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
    current_attempts = [
        ("current", record)
        for record in state.route_attempt_records
        if getattr(record, "net_id", None) == job.net_id
    ]
    recent_attempts = [("recent", record) for record in state.route_attempt_records[-12:]]
    root_cause_line = _format_illegal_crossing_root_causes_line(
        [error_text]
        + [
            str(attempt.error)
            for _, attempt in (current_attempts[-8:] + recent_attempts)
            if getattr(attempt, "error", None)
        ]
    )
    fail_lines = [
        f"net_name={job.net_name}",
        f"source_spec={port1_spec}",
        f"target_spec={port2_spec}",
        f"source_state=({int(source_state.x)}, {int(source_state.y)}, {int(source_state.angle)})",
        f"target_state=({int(target_state.x)}, {int(target_state.y)}, {int(target_state.angle)})",
        f"allow_45_degree_turns={settings.allow_45_degree_turns}",
        f"block_radius_cells={state.block_radius_cells}",
        "dynamic_obstacle_search_expansion_radius_cells="
        f"{state.clearance_policy.dynamic_obstacle_search_expansion_radius_cells}",
        "dynamic_route_commit_keepout_radius_cells="
        f"{state.clearance_policy.dynamic_route_commit_keepout_radius_cells}",
        "dynamic_route_core_radius_cells="
        f"{state.clearance_policy.dynamic_route_core_radius_cells}",
        f"bend_radius_cells={state.bend_radius_cells}",
        f"port_lane_length_cells={state.port_lane_length_cells}",
        f"port_lane_half_width_cells={state.port_lane_half_width_cells}",
        f"opened_candidate_cells_count={len(opened_candidate_cells)}",
        f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
        f"opened_candidate_static_overlap_bbox={_cells_bbox(opened_candidate_static_overlap)}",
        f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
        f"opened_candidate_dynamic_overlap_bbox={_cells_bbox(opened_candidate_dynamic_overlap)}",
        f"opened_cells_count={len(opened_cells)}",
        f"opened_static_overlap_count={len(opened_static_overlap)}",
        f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
        f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
        f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
        f"corridor_clearance_max_radius_checked={corridor_diagnostic['max_radius_checked']}",
        f"corridor_clearance_last_connected_radius={corridor_diagnostic['last_connected_radius']}",
        f"corridor_clearance_first_disconnected_radius={corridor_diagnostic['first_disconnected_radius']}",
        f"corridor_clearance_source_region_size={corridor_diagnostic['source_region_size']}",
        f"corridor_clearance_target_region_size={corridor_diagnostic['target_region_size']}",
        "corridor_clearance_source_region_min_distance_to_target="
        f"{corridor_diagnostic['source_region_min_distance_to_target']}",
        "corridor_clearance_target_region_min_distance_to_source="
        f"{corridor_diagnostic['target_region_min_distance_to_source']}",
        f"error={error_text}",
    ]
    if root_cause_line is not None:
        fail_lines.append(root_cause_line)
    fail_lines.extend(_format_native_repair_trace_lines(state.native_repair_trace_records))
    for label, attempt in current_attempts[-8:] + recent_attempts:
        as_dict = attempt.as_dict()
        diagnostics = as_dict.get("diagnostics")
        fail_lines.append(
            f"attempt_{label}="
            + ", ".join(
                f"{key}={as_dict.get(key)}"
                for key in (
                    "attempt_index",
                    "bucket_name",
                    "failed",
                    "error",
                    "elapsed_s",
                    "expanded_states",
                    "generated_neighbors",
                    "window_attempts",
                    "used_full_grid_fallback",
                    "candidate_blocker_count",
                    "candidate_blocker_route_indices",
                    "ripup_victim_count",
                    "ripup_victim_route_indices",
                )
                if key in as_dict
            )
        )
        if isinstance(diagnostics, dict) and diagnostics:
            fail_lines.append(
                f"attempt_{label}_diagnostics="
                + ", ".join(
                    f"{key}={diagnostics.get(key)}"
                    for key in (
                        "candidate_blocker_count",
                        "candidate_blocker_route_indices",
                        "ripup_victim_count",
                        "ripup_victim_route_indices",
                        "route_bbox_min_x",
                        "route_bbox_max_x",
                        "route_bbox_min_y",
                        "route_bbox_max_y",
                    )
                    if key in diagnostics
                )
            )
    fail_txt.write_text("\n".join(fail_lines) + "\n", encoding="utf-8")


def _finalize_committed_route(
    settings,
    state,
    job: RouteJob,
    route_obj: Any,
    opened_cells: list[tuple[int, int]],
    *,
    should_print_route: bool,
    diag_txt: Path | None,
    debug_obstacle_cells: set[tuple[int, int]] | None = None,
) -> None:
    expanded_states = int(getattr(route_obj, "expanded_states", 0))
    state.total_expanded_states += expanded_states
    if expanded_states == 0:
        state.simple_route_count += 1

    if state.diagnostics_enabled:
        source_state, target_state, opened_candidate_cells, opened_cells_set, _ = (
            _state_openings_for_job(settings, state, job)
        )
        route_cells = {
            (int(cell[0]), int(cell[1])) for cell in (getattr(route_obj, "cells", None) or [])
        }
        _write_route_diagnostics(settings, state,
            job=job,
            source_state=source_state,
            target_state=target_state,
            opened_candidate_cells=opened_candidate_cells,
            dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(settings, state, job),
            opened_cells_set=opened_cells_set,
            diag_txt=diag_txt,
            status="ok",
            route_cells=route_cells,
            route_obj=route_obj,
        )

    _export_route_svg(settings, state,
        job,
        route_obj,
        obstacle_cells=debug_obstacle_cells,
        opened_cells=opened_cells,
    )

    if should_print_route:
        print(f"ok {_route_engine_summary(settings, state, route_obj)}")


def dispatch_native_routing(
    settings: SessionSettings, state: SessionState, route_jobs: list[RouteJob]
) -> None:
    if state.repair_config.enabled:
        if not hasattr(state.router, "route_many_with_repair_and_commit"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.route_many_with_repair_and_commit. Rebuild it with "
                "`maturin develop --release`; Python repair fallback has been removed."
            )
        batch_jobs: list[
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
        batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
        batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
        t_batch_job_pack_start = timing.pipeline_timer_start(settings, state)
        for job in route_jobs:
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(
                settings, state, job
            )
            clearance_exempt_cells = _clearance_exempt_cells_for_job(settings, state, job)
            route_selected_for_debug = (
                settings.debug_route_indices is None
                or job.route_index in settings.debug_route_indices
            )
            should_print_route = (
                settings.verbose_route_diagnostics and route_selected_for_debug
            )
            if settings.debug_route_indices is not None and route_selected_for_debug:
                should_print_route = True
            if should_print_route:
                print(
                    f"  Routing [{job.route_index}/{len(route_jobs)}] "
                    f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                    end=" ",
                )
            route_dir = state.debug_path / "routes" if state.debug_path is not None else None
            diag_txt: Path | None = None
            if (
                state.debug_path is not None
                and (route_selected_for_debug or settings.collect_attempt_diagnostics)
                and route_dir is not None
            ):
                _ensure_dir(route_dir)
                diag_txt = (
                    route_dir / f"{settings.debug_prefix}_{job.net_name}_diagnostics.txt"
                )
            batch_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    _foreign_keepout_cleanup_cells_for_job(settings, state, job),
                    _routing_endpoint_center_um(settings, state, job, source=True),
                    _routing_endpoint_center_um(settings, state, job, source=False),
                )
            )
            batch_opened_cells_by_id[int(job.net_id)] = opened_cells
            batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
        timing.record_pipeline_timing(settings, state, "batch_job_pack", t_batch_job_pack_start)

        batch_start = _timing_start(settings, state)
        if negotiated_repair_engine_enabled(settings.config.engine):
            # Default since 2026-09-16 (owner decision, baseline
            # freeze): the LiDAR-style negotiated rip-up loop of
            # .agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md
            # (the only engine that routes the 64x64 mesh). The older
            # 17-strategy chain `route_many_with_repair_and_commit`
            # stays available for A/B runs via
            # PHOTONIC_ROUTER_LEGACY_REPAIR_CHAIN=1 (or
            # PHOTONIC_ROUTER_NEGOTIATED_REPAIR=0).
            if not hasattr(state.router, "route_many_with_negotiated_repair_and_commit"):
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.route_many_with_negotiated_repair_and_commit. "
                    "Rebuild it with `maturin develop --release`."
                )
            raw_batch_result = state.router.route_many_with_negotiated_repair_and_commit(
                batch_jobs,
                state.block_radius_cells,
                state.commit_radius_cells,
                state.core_commit_radius_cells,
                NEGOTIATED_MAX_ROUNDS,
                float(state.repair_config.history_weight),
                int(state.repair_config.history_increment),
            )
        else:
            raw_batch_result = state.router.route_many_with_repair_and_commit(
                batch_jobs,
                state.block_radius_cells,
                state.commit_radius_cells,
                state.core_commit_radius_cells,
                int(state.repair_config.max_rounds),
                int(state.repair_config.max_victims_per_failure),
                float(state.repair_config.history_weight),
                int(state.repair_config.history_increment),
            )
        batch_elapsed_s = time.perf_counter() - batch_start if state.collect_timing else 0.0
        timing.record_pipeline_timing(settings, state, "native_route_batch", batch_start)
        t_batch_result_processing_start = timing.pipeline_timer_start(settings, state)
        batch_result = dict(raw_batch_result)
        state.native_repair_trace_records = [
            dict(record)
            for record in cast(
                Iterable[Mapping[str, object]],
                batch_result.get("repair_trace", []),
            )
        ]
        _record_native_batch_timings(settings, state, batch_result)
        _report_long_straight_congestion(settings, state, batch_result)
        raw_attempts = list(cast(Iterable[Any], batch_result.get("attempts", [])))
        per_attempt_elapsed_s = batch_elapsed_s / max(1, len(raw_attempts))
        for raw_attempt in raw_attempts:
            attempt = dict(raw_attempt)
            net_id = int(attempt["net_id"])
            job = state.route_jobs_by_id[net_id]
            route_obj = attempt.get("route")
            if route_obj is None:
                route_obj = None
            failed = bool(attempt.get("failed", False))
            bucket_name = str(attempt.get("bucket_name", "normal_route"))
            error_text = str(attempt.get("error")) if attempt.get("error") is not None else None
            repair_round_raw = attempt.get("repair_round")
            repair_round = int(repair_round_raw) if repair_round_raw is not None else None
            attempt_index = len(state.route_attempt_records) + 1
            candidate_blockers = [
                int(value)
                for value in cast(list[object], attempt.get("candidate_blockers", []))
            ]
            ripup_ids = [
                int(value) for value in cast(list[object], attempt.get("ripup_ids", []))
            ]
            if (
                route_obj is not None
                and not failed
                and bucket_name != "normal_route"
                and state.debug_path is not None
                and (
                    settings.debug_route_indices is None
                    or job.route_index in settings.debug_route_indices
                )
            ):
                _export_route_svg(settings, state,
                    job,
                    route_obj,
                    suffix=f"_attempt{attempt_index}_{bucket_name}",
                )
            if state.collect_timing:
                bucket = state.route_timing_buckets.setdefault(
                    bucket_name,
                    RouteTimingBucket(),
                )
                if route_obj is not None and not failed:
                    bucket.record_route(
                        per_attempt_elapsed_s,
                        route_obj,
                    )
                else:
                    bucket.record_elapsed(
                        per_attempt_elapsed_s,
                        failed=failed,
                    )
                state.route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=attempt_index,
                        bucket_name=bucket_name,
                        net_id=job.net_id,
                        route_index=job.route_index,
                        net_name=job.net_name,
                        source=f"{job.inst1},{job.port1}",
                        target=f"{job.inst2},{job.port2}",
                        elapsed_s=per_attempt_elapsed_s,
                        route_obj=route_obj if route_obj is not None and not failed else None,
                        failed=failed,
                        repair_round=repair_round,
                        error=error_text,
                        diagnostics=_route_attempt_diagnostics(settings, state,
                            job,
                            route_obj if route_obj is not None and not failed else None,
                            candidate_blockers=candidate_blockers,
                            ripup_ids=ripup_ids,
                        )
                        if settings.collect_attempt_diagnostics
                        else None,
                    )
                )

        state.repair_count += int(batch_result.get("repair_count", 0) or 0)
        state.deferred_count += int(batch_result.get("deferred_count", 0) or 0)
        raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
        for raw_entry in raw_routes:
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            route_obj = entry["route"]
            job = state.route_jobs_by_id[net_id]
            opened_cells = batch_opened_cells_by_id[net_id]
            _record_route(settings, state, job, route_obj, opened_cells)
            should_print_route, diag_txt = batch_debug_by_id[net_id]
            _finalize_committed_route(settings, state,
                job,
                route_obj,
                opened_cells,
                should_print_route=should_print_route,
                diag_txt=diag_txt,
            )
        timing.record_pipeline_timing(settings, state,
            "batch_result_processing",
            t_batch_result_processing_start,
        )

        if str(batch_result.get("status", "")) != "routed":
            failed_net_id = int(batch_result.get("failed_net_id", -1))
            failed_job = state.route_jobs_by_id.get(failed_net_id)
            error_text = str(batch_result.get("error", "No route found"))
            if failed_job is None:
                raise RuntimeError(error_text)
            (
                source_state,
                target_state,
                opened_candidate_cells,
                opened_cells_set,
                opened_cells,
            ) = _state_openings_for_job(settings, state, failed_job)
            should_print_route, diag_txt = batch_debug_by_id.get(
                failed_net_id,
                (False, None),
            )
            if not should_print_route:
                print(
                    f"  Routing [{failed_job.route_index}/{len(route_jobs)}] "
                    f"{failed_job.net_name}: {failed_job.inst1},{failed_job.port1} -> "
                    f"{failed_job.inst2},{failed_job.port2}... failed"
                )
            _write_route_diagnostics(settings, state,
                job=failed_job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(settings, state,
                    failed_job
                ),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="failed",
                error_text=error_text,
            )
            _write_failed_log(settings, state,
                failed_job,
                source_state,
                target_state,
                opened_candidate_cells,
                opened_cells,
                error_text,
            )
            raise RuntimeError(
                f"No route found for {failed_job.net_name}: "
                f"{failed_job.inst1},{failed_job.port1} -> {failed_job.inst2},{failed_job.port2}. "
                f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                f"allow_45_degree_turns={settings.allow_45_degree_turns}. "
                f"error={error_text}"
            )

    else:
        if not hasattr(state.router, "route_many_normal_and_commit"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.route_many_normal_and_commit. Rebuild it with "
                "`maturin develop --release`; Python sequential routing fallback has been removed."
            )
        batch_jobs: list[
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
        batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
        batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
        t_batch_job_pack_start = timing.pipeline_timer_start(settings, state)
        for job in route_jobs:
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(
                settings, state, job
            )
            clearance_exempt_cells = _clearance_exempt_cells_for_job(settings, state, job)
            route_selected_for_debug = (
                settings.debug_route_indices is None
                or job.route_index in settings.debug_route_indices
            )
            should_print_route = (
                settings.verbose_route_diagnostics and route_selected_for_debug
            )
            if settings.debug_route_indices is not None and route_selected_for_debug:
                should_print_route = True
            if should_print_route:
                print(
                    f"  Routing [{job.route_index}/{len(route_jobs)}] "
                    f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                    end=" ",
                )
            route_dir = state.debug_path / "routes" if state.debug_path is not None else None
            diag_txt: Path | None = None
            if (
                state.debug_path is not None
                and (route_selected_for_debug or settings.collect_attempt_diagnostics)
                and route_dir is not None
            ):
                _ensure_dir(route_dir)
                diag_txt = (
                    route_dir / f"{settings.debug_prefix}_{job.net_name}_diagnostics.txt"
                )
            batch_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    _foreign_keepout_cleanup_cells_for_job(settings, state, job),
                    _routing_endpoint_center_um(settings, state, job, source=True),
                    _routing_endpoint_center_um(settings, state, job, source=False),
                )
            )
            batch_opened_cells_by_id[int(job.net_id)] = opened_cells
            batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
        timing.record_pipeline_timing(settings, state, "batch_job_pack", t_batch_job_pack_start)

        batch_start = _timing_start(settings, state)
        raw_batch_result = state.router.route_many_normal_and_commit(
            batch_jobs,
            state.block_radius_cells,
            state.commit_radius_cells,
            state.core_commit_radius_cells,
        )
        batch_elapsed_s = time.perf_counter() - batch_start if state.collect_timing else 0.0
        timing.record_pipeline_timing(settings, state, "native_route_batch", batch_start)
        t_batch_result_processing_start = timing.pipeline_timer_start(settings, state)
        batch_result = dict(raw_batch_result)
        _record_native_batch_timings(settings, state, batch_result)
        _report_long_straight_congestion(settings, state, batch_result)
        raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
        per_route_elapsed_s = batch_elapsed_s / max(1, len(raw_routes))
        for raw_entry in raw_routes:
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            route_obj = entry["route"]
            job = state.route_jobs_by_id[net_id]
            opened_cells = batch_opened_cells_by_id[net_id]
            if state.collect_timing:
                state.route_timing_buckets["normal_route"].record_route(
                    per_route_elapsed_s,
                    route_obj,
                )
                state.route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=len(state.route_attempt_records) + 1,
                        bucket_name="normal_route",
                        net_id=job.net_id,
                        route_index=job.route_index,
                        net_name=job.net_name,
                        source=f"{job.inst1},{job.port1}",
                        target=f"{job.inst2},{job.port2}",
                        elapsed_s=per_route_elapsed_s,
                        route_obj=route_obj,
                    )
                )
            _record_route(settings, state, job, route_obj, opened_cells)
            should_print_route, diag_txt = batch_debug_by_id[net_id]
            _finalize_committed_route(settings, state,
                job,
                route_obj,
                opened_cells,
                should_print_route=should_print_route,
                diag_txt=diag_txt,
            )
        timing.record_pipeline_timing(settings, state,
            "batch_result_processing",
            t_batch_result_processing_start,
        )

        if str(batch_result.get("status", "")) != "routed":
            failed_net_id = int(batch_result.get("failed_net_id", -1))
            failed_job = state.route_jobs_by_id.get(failed_net_id)
            error_text = str(batch_result.get("error", "No route found"))
            if failed_job is None:
                raise RuntimeError(error_text)
            (
                source_state,
                target_state,
                opened_candidate_cells,
                opened_cells_set,
                opened_cells,
            ) = _state_openings_for_job(settings, state, failed_job)
            should_print_route, diag_txt = batch_debug_by_id.get(
                failed_net_id,
                (False, None),
            )
            if state.collect_timing:
                state.route_timing_buckets["normal_route"].record_elapsed(0.0, failed=True)
                state.route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=len(state.route_attempt_records) + 1,
                        bucket_name="normal_route",
                        net_id=failed_job.net_id,
                        route_index=failed_job.route_index,
                        net_name=failed_job.net_name,
                        source=f"{failed_job.inst1},{failed_job.port1}",
                        target=f"{failed_job.inst2},{failed_job.port2}",
                        elapsed_s=0.0,
                        route_obj=None,
                        failed=True,
                        error=error_text,
                    )
                )
            if not should_print_route:
                print(
                    f"  Routing [{failed_job.route_index}/{len(route_jobs)}] "
                    f"{failed_job.net_name}: {failed_job.inst1},{failed_job.port1} -> "
                    f"{failed_job.inst2},{failed_job.port2}... failed"
                )
            _write_route_diagnostics(settings, state,
                job=failed_job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(settings, state,
                    failed_job
                ),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="failed",
                error_text=error_text,
            )
            _write_failed_log(settings, state,
                failed_job,
                source_state,
                target_state,
                opened_candidate_cells,
                opened_cells,
                error_text,
            )
            raise RuntimeError(
                f"No route found for {failed_job.net_name}: "
                f"{failed_job.inst1},{failed_job.port1} -> {failed_job.inst2},{failed_job.port2}. "
                f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                f"allow_45_degree_turns={settings.allow_45_degree_turns}"
            )


def negotiated_repair_engine_enabled(engine: EngineSelection | None = None) -> bool:
    """The negotiated rip-up engine is the default repair engine since the
    2026-09-16 baseline freeze; `PHOTONIC_ROUTER_LEGACY_REPAIR_CHAIN=1` or
    `PHOTONIC_ROUTER_NEGOTIATED_REPAIR=0` selects the older repair chain."""
    engine = engine if engine is not None else EngineSelection()
    if engine.legacy_repair_chain:
        return False
    return engine.negotiated_repair
