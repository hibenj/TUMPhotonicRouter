"""Phase 6: the native kernel dispatch and its per-route bookkeeping."""

from __future__ import annotations

import math
import time
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, cast

from translation.route_rust_crossing_plan import _port_center_um
from translation.route_rust_debug_artifacts import (
    _ensure_dir,
)
from translation.route_rust_geometry import (
    _compress_centerline,
)
from translation.route_rust_records import _centerline_tuple
from translation.route_rust_types import (
    RouteJob,
    RouteTimingBucket,
    route_attempt_record_from_route,
)
from translation.routing import timing
from translation.routing.crossing_plan_stage import _cells_in_raw_static_geometry
from translation.routing.diagnostics import (
    _export_route_svg,
    _route_attempt_diagnostics,
    _route_cells_from_router,
    _route_engine_summary,
    _state_openings_for_job,
    _write_failed_log,
    _write_route_diagnostics,
)
from translation.routing.settings import SessionSettings
from translation.routing.state import SessionState

# `route_many_with_negotiated_repair_and_commit`'s own round budget --
# LiDAR's `runNRR` default (`.agent/execplans/2026-09-14-lidar-style-
# negotiated-ripup-endgame.md`, Milestone 3). This is the only round budget
# left: the older repair chain's own two budgets (a per-net repair-round cap
# on `RipupRerouteConfig`, default 4, and a per-round victim cap) counted a
# per-net repair-set retry, a different thing from the negotiated engine's
# "round" (a whole-queue pass
# with a global rip-up at the end, LiDAR's `ripupfailedNets`), which is why
# the chain's smaller default never carried over; Milestone 8 of
# .agent/execplans/2026-09-22-modular-readable-router-restructure.md deleted
# that chain in Slice A and both knobs with it in Slice C.
NEGOTIATED_MAX_ROUNDS = 10


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
    cells.update(
        _foreign_keepout_cleanup_cells_for_spec(settings, state, f"{job.inst2},{job.port2}")
    )
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


def _clearance_exempt_cells_for_job(settings, state, job: RouteJob) -> list[tuple[int, int]]:
    return state.batch_clearance_exempt_cells_by_id.get(int(job.net_id), [])


def _clearance_exempt_cell_set_for_job(settings, state, job: RouteJob) -> set[tuple[int, int]]:
    return set(_clearance_exempt_cells_for_job(settings, state, job))


def native_route_jobs(settings, state, jobs: Iterable[RouteJob]) -> list[Any]:
    """The kernel's `RouteJob` objects for `route_many_with_negotiated_repair_and_commit`.

    One job per Python `RouteJob`, in the order given: the net id, the two grid
    states and the opened port cells of the static hand-off
    (`state.route_state_openings_by_id`), the clearance-exempt cells of the
    batch, the foreign keep-out cells the net may clean up, and the two routing
    endpoint centers (the fan-out anchor where one exists, the port center
    otherwise). The first routing, the detour pass and the two verify-repair
    passes all pack their batches here; `job.opened_cells` on the result is the
    opened-cell list the callers record with the route.
    """
    rust_route_job = state.rust_backend.RouteJob
    native_jobs: list[Any] = []
    for job in jobs:
        source_state, target_state, _, _, opened_cells = _state_openings_for_job(
            settings, state, job
        )
        native_jobs.append(
            rust_route_job(
                net_id=int(job.net_id),
                source=source_state,
                target=target_state,
                opened_cells=opened_cells,
                clearance_exempt_cells=_clearance_exempt_cells_for_job(settings, state, job),
                foreign_keepout_cleanup_cells=_foreign_keepout_cleanup_cells_for_job(
                    settings, state, job
                ),
                source_port_um=_routing_endpoint_center_um(settings, state, job, source=True),
                target_port_um=_routing_endpoint_center_um(settings, state, job, source=False),
            )
        )
    return native_jobs


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
    target_port_center_um_override = target_anchor.center_um if target_anchor is not None else None
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
        _write_route_diagnostics(
            settings,
            state,
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

    _export_route_svg(
        settings,
        state,
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
    # Repair on (the default, every benchmark needs it): the negotiated
    # rip-up loop over NEGOTIATED_MAX_ROUNDS rounds. Repair off (a
    # diagnostic/test baseline): the same loop with one round and
    # `no_ripup`, which is plain in-order routing with no local rip-up and no
    # requeue; with one round every net has `failed_count == 0`, so the
    # loop's history weight stays 0.0 and its searches are unbounded.
    repair_enabled = bool(state.repair_config.enabled)
    max_rounds = NEGOTIATED_MAX_ROUNDS if repair_enabled else 1
    no_ripup = not repair_enabled
    if not hasattr(state.router, "route_many_with_negotiated_repair_and_commit"):
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.route_many_with_negotiated_repair_and_commit. Rebuild it "
            "with `maturin develop --release`; the Python routing fallbacks have been removed."
        )
    batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
    batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
    t_batch_job_pack_start = timing.pipeline_timer_start(settings, state)
    batch_jobs = native_route_jobs(settings, state, route_jobs)
    for job, native_job in zip(route_jobs, batch_jobs, strict=True):
        route_selected_for_debug = (
            settings.debug_route_indices is None or job.route_index in settings.debug_route_indices
        )
        should_print_route = settings.verbose_route_diagnostics and route_selected_for_debug
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
            diag_txt = route_dir / f"{settings.debug_prefix}_{job.net_name}_diagnostics.txt"
        batch_opened_cells_by_id[int(job.net_id)] = native_job.opened_cells
        batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
    timing.record_pipeline_timing(settings, state, "batch_job_pack", t_batch_job_pack_start)

    batch_start = _timing_start(settings, state)
    # The LiDAR-style negotiated rip-up loop of
    # .agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md
    # (the only engine that routes the 64x64 mesh), the default since
    # 2026-09-16 (owner decision, baseline freeze) and the only repair
    # engine since Milestone 8 of
    # .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    # deleted the older 17-strategy chain and its engine switch.
    raw_batch_result = state.router.route_many_with_negotiated_repair_and_commit(
        batch_jobs,
        state.block_radius_cells,
        state.commit_radius_cells,
        state.core_commit_radius_cells,
        max_rounds,
        float(state.repair_config.history_weight),
        int(state.repair_config.history_increment),
        no_ripup=no_ripup,
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
            int(value) for value in cast(list[object], attempt.get("candidate_blockers", []))
        ]
        ripup_ids = [int(value) for value in cast(list[object], attempt.get("ripup_ids", []))]
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
            _export_route_svg(
                settings,
                state,
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
                    diagnostics=_route_attempt_diagnostics(
                        settings,
                        state,
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
    raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
    for raw_entry in raw_routes:
        entry = dict(raw_entry)
        net_id = int(entry["net_id"])
        route_obj = entry["route"]
        job = state.route_jobs_by_id[net_id]
        opened_cells = batch_opened_cells_by_id[net_id]
        _record_route(settings, state, job, route_obj, opened_cells)
        should_print_route, diag_txt = batch_debug_by_id[net_id]
        _finalize_committed_route(
            settings,
            state,
            job,
            route_obj,
            opened_cells,
            should_print_route=should_print_route,
            diag_txt=diag_txt,
        )
    timing.record_pipeline_timing(
        settings,
        state,
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
        _write_route_diagnostics(
            settings,
            state,
            job=failed_job,
            source_state=source_state,
            target_state=target_state,
            opened_candidate_cells=opened_candidate_cells,
            dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(
                settings, state, failed_job
            ),
            opened_cells_set=opened_cells_set,
            diag_txt=diag_txt,
            status="failed",
            error_text=error_text,
        )
        _write_failed_log(
            settings,
            state,
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
