"""Phase 7: the routed-net records and the endpoint-correction application passes."""

from __future__ import annotations

import time

from collections import Counter
from dataclasses import replace
from typing import Any, Iterable, Mapping, cast

from translation.route_rust_crossing_plan import _port_center_um
from translation.route_rust_crossing_verification import (
    _populate_realized_intersections_from_native_crossing_events,
)
from translation.route_rust_endpoint_correction import (
    _apply_crossing_aware_endpoint_correction_to_record,
    _centerline_length_um,
    _dedupe_centerline,
    _legal_crossing_points_by_net_id,
    _merge_terminal_corrected_route_centerline,
    _primitive_centerline_for_record,
)
from translation.route_rust_geometry import _compress_centerline
from translation.route_rust_records import EndpointCorrectionRouter, _centerline_tuple
from translation.route_rust_types import (
    EndpointCorrectionCategory,
    NetEndpointCorrectionClassification,
    RouteJob,
    RoutedNetRecord,
)

from translation.routing.dispatch import (
    _append_centerline_points,
    _clearance_exempt_cells_for_job,
    _routing_endpoint_center_um,
    _state_openings_for_job,
    _timing_start,
)
from translation.routing.route_jobs import _grid_cell_center_um
from translation.routing import timing
from translation.routing.settings import SessionSettings
from translation.routing.stages import RoutedRecords
from translation.routing.state import SessionState

def _endpoint_correction_crossing_net_ids(settings, state) -> set[int]:
    """Net ids involved in any crossing, per `router.crossing_events()`.

    Shared by `_apply_checked_endpoint_corrections_for_net_ids` and
    `_apply_checked_fanout_stub_endpoint_corrections_for_net_ids`,
    which previously each re-derived this identical set inline
    (Milestone 1 of
    .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md).
    Not used by `_apply_crossing_aware_endpoint_corrections_for_net_ids`,
    which uses a separate, independently-derived notion of "has a
    crossing" (`_current_crossing_points_by_net_id()`) that carries
    actual crossing points, not just event membership; unifying the
    two is out of this milestone's additive scope.
    """
    crossing_net_ids: set[int] = set()
    if settings.enable_crossings and hasattr(state.router, "crossing_events"):
        # No broad `except Exception` here: Milestone 0 of the
        # restructuring ExecPlan confirmed via git history that this
        # used to swallow any error from `router.crossing_events()`
        # (introduced verbatim by commit 9925249, never touched since,
        # no test/log/diagnostic anywhere shows it ever legitimately
        # firing). A real failure here means crossing detection is
        # broken, which should stop the run loudly, not be silently
        # treated as "no crossings" and let every net fall through to
        # the unrestricted corrector as if crossings were disabled.
        for raw_event in cast(Iterable[Any], state.router.crossing_events()):
            if not isinstance(raw_event, Mapping):
                try:
                    raw_event = dict(cast(Any, raw_event))
                except (TypeError, ValueError):
                    continue
            for key in ("net_id", "partner_net_id"):
                try:
                    crossing_net_ids.add(int(cast(Any, raw_event.get(key))))
                except (TypeError, ValueError):
                    continue
    return crossing_net_ids


def _classify_net_for_endpoint_correction(
    settings,
    state,
    net_id: int,
    *,
    crossing_net_ids: set[int],
) -> NetEndpointCorrectionClassification | None:
    """Classify one net for endpoint-correction dispatch.

    Mirrors, exactly, the per-net guard conditions that were
    previously inline and duplicated across
    `_apply_checked_endpoint_corrections_for_net_ids` and
    `_apply_checked_fanout_stub_endpoint_corrections_for_net_ids`
    (Milestone 1). Returns `None` only when the net has no record or
    job at all in this session's bookkeeping, which every caller
    already treats as "skip" today.

    This only decides which of pass 1 (unrestricted), pass 2
    (fanout-stub partial), pass 3 (crossing-aware), or no pass at all
    applies. It deliberately does not also decide whether the
    resolved source/target ports are usable (`None`/`None`): that
    check differs between pass 1 (resolves both sides unconditionally)
    and pass 2 (resolves only the non-stub side, since the stub side
    is deliberately left as `None`), so each pass still performs its
    own port-resolution check after using this classification to
    decide whether it owns the net at all.
    """
    net_id = int(net_id)
    record = state.route_bookkeeping.records_by_id.get(net_id)
    job = state.route_jobs_by_id.get(net_id)
    if record is None or job is None:
        return None

    source_has_fanout_stub = net_id in state.fanout_anchor_source_net_ids
    target_has_fanout_stub = net_id in state.fanout_anchor_target_net_ids
    has_crossing = settings.enable_crossings and net_id in crossing_net_ids

    if has_crossing:
        return NetEndpointCorrectionClassification(
            category=EndpointCorrectionCategory.CROSSING_AWARE,
            has_crossing=True,
            source_has_fanout_stub=source_has_fanout_stub,
            target_has_fanout_stub=target_has_fanout_stub,
        )

    # Pass 1 skips any fanout-anchor net that has already been given a
    # corrected centerline by the earlier fanout-stub pre-correction
    # stage (upstream of all three passes), deferring it to pass 2. A
    # fanout-anchor net that has *not* yet been given one -- an edge
    # case, since pre-correction is expected to have already run by
    # this point -- falls through to UNRESTRICTED below instead, the
    # same as it does in the real pass 1 today; that is preserved
    # exactly, not treated as a bug, since Milestone 1 is additive.
    already_fanout_precorrected = net_id in state.fanout_anchor_net_ids and bool(
        record.corrected_centerline_um
    )
    if already_fanout_precorrected:
        if source_has_fanout_stub and target_has_fanout_stub:
            category = EndpointCorrectionCategory.ALREADY_CORRECTED_NO_OP
        elif source_has_fanout_stub:
            category = EndpointCorrectionCategory.FANOUT_STUB_SOURCE_ONLY
        elif target_has_fanout_stub:
            category = EndpointCorrectionCategory.FANOUT_STUB_TARGET_ONLY
        else:
            # net_id in fanout_anchor_net_ids is constructed as exactly
            # the union of the source/target subsets today, so this is
            # unreachable; if that invariant ever changes, treat it the
            # same as the no-op category rather than guess silently.
            category = EndpointCorrectionCategory.ALREADY_CORRECTED_NO_OP
        return NetEndpointCorrectionClassification(
            category=category,
            has_crossing=False,
            source_has_fanout_stub=source_has_fanout_stub,
            target_has_fanout_stub=target_has_fanout_stub,
        )

    return NetEndpointCorrectionClassification(
        category=EndpointCorrectionCategory.UNRESTRICTED,
        has_crossing=False,
        source_has_fanout_stub=source_has_fanout_stub,
        target_has_fanout_stub=target_has_fanout_stub,
    )


def _apply_checked_endpoint_corrections_for_net_ids(
    settings,
    state,
    net_ids: Iterable[int],
    *,
    record_pipeline_timing: bool = True,
    print_warnings: bool = False,
) -> list[int]:
    if not settings.enable_checked_endpoint_correction:
        return []
    if not hasattr(state.router, "apply_checked_endpoint_corrections"):
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.apply_checked_endpoint_corrections. "
            "Rebuild it with `maturin develop --release`."
        )
    correction_jobs: list[
        tuple[
            int,
            Any,
            list[tuple[int, int]],
            list[tuple[int, int]],
            tuple[float, float] | None,
            tuple[float, float] | None,
        ]
    ] = []
    requested_net_ids = [int(net_id) for net_id in net_ids]
    crossing_net_ids = _endpoint_correction_crossing_net_ids(settings, state)
    t_endpoint_correction_pack_start = timing.pipeline_timer_start(settings, state)
    for net_id in requested_net_ids:
        classification = _classify_net_for_endpoint_correction(settings, state,
            net_id, crossing_net_ids=crossing_net_ids
        )
        if (
            classification is None
            or classification.category != EndpointCorrectionCategory.UNRESTRICTED
        ):
            continue
        record = state.route_bookkeeping.records_by_id[net_id]
        job = state.route_jobs_by_id[net_id]
        source_port = _routing_endpoint_center_um(settings, state, job, source=True)
        target_port = _routing_endpoint_center_um(settings, state, job, source=False)
        if source_port is None and target_port is None:
            continue
        source_state, target_state, opened_candidate_cells, _, _ = _state_openings_for_job(
            settings, state,
            job
        )
        clearance_exempt_cells = _clearance_exempt_cells_for_job(settings, state, job)
        correction_jobs.append(
            (
                int(net_id),
                record.route_obj,
                sorted(opened_candidate_cells),
                clearance_exempt_cells,
                source_port,
                target_port,
            )
        )
    if record_pipeline_timing:
        timing.record_pipeline_timing(settings, state,
            "endpoint_correction_pack",
            t_endpoint_correction_pack_start,
        )
    if not correction_jobs:
        return []

    correction_start = _timing_start(settings, state)
    raw_corrections = state.router.apply_checked_endpoint_corrections(
        correction_jobs,
        float(settings.route_width_um),
        int(state.commit_radius_cells),
        int(state.core_commit_radius_cells),
        True,
    )
    correction_elapsed_s = (
        time.perf_counter() - correction_start if state.collect_timing else 0.0
    )
    if record_pipeline_timing:
        timing.record_pipeline_timing(
            settings, state, "endpoint_correction_native", correction_start
        )
    t_endpoint_correction_processing_start = timing.pipeline_timer_start(settings, state)
    correction_elapsed_per_job_s = correction_elapsed_s / max(1, len(correction_jobs))
    failed_net_ids: list[int] = []
    for raw_correction in cast(Iterable[Any], raw_corrections):
        correction = dict(raw_correction)
        net_id = int(correction["net_id"])
        record = state.route_bookkeeping.records_by_id.get(net_id)
        job = state.route_jobs_by_id.get(net_id)
        if record is None or job is None:
            continue
        error = correction.get("error")
        if error is not None:
            if state.collect_timing:
                state.route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                    failed=True,
                )
            message = (
                "Checked grid-to-port endpoint correction skipped for net "
                f"{job.net_name!r}: {error}"
            )
            if print_warnings:
                print("WARNING: " + message)
            failed_net_ids.append(net_id)
            # Every net reaching this point was already excluded from
            # crossing handling above (crossing_net_ids), regardless of
            # self.enable_crossings, so there is no later pass that will
            # still process it and no reason to withhold recording this
            # failure. Withholding it here used to leave
            # corrected_centerline_um empty with endpoint_correction_error
            # still None, which made geometry realization's own fallback
            # (_physical_port_centerline, translation/route_rust_realization.py)
            # silently re-derive uncorrected, collision-unchecked geometry
            # instead of surfacing the rejection -- found while confirming
            # the Milestone 0.5 fix in
            # .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md
            # actually took effect end to end.
            state.route_bookkeeping.records_by_id[net_id] = replace(
                record,
                corrected_centerline_um=(),
                endpoint_correction_error=message,
            )
            continue
        centerline = _centerline_tuple(correction.get("centerline"))
        if not centerline:
            if state.collect_timing:
                state.route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                    failed=True,
                )
            message = (
                "Checked grid-to-port endpoint correction skipped for net "
                f"{job.net_name!r}: endpoint correction returned an invalid centerline"
            )
            if print_warnings:
                print("WARNING: " + message)
            failed_net_ids.append(net_id)
            state.route_bookkeeping.records_by_id[net_id] = replace(
                record,
                corrected_centerline_um=(),
                endpoint_correction_error=message,
            )
            continue
        if state.collect_timing:
            state.route_timing_buckets["endpoint_correction"].record_elapsed(
                correction_elapsed_per_job_s,
            )
        corrected_total_length_um = float(correction["total_length_um"])
        state.route_bookkeeping.records_by_id[net_id] = replace(
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
    if record_pipeline_timing:
        timing.record_pipeline_timing(settings, state,
            "endpoint_correction_processing",
            t_endpoint_correction_processing_start,
        )
    return failed_net_ids


def _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
    settings,
    state,
    net_ids: Iterable[int],
    *,
    record_pipeline_timing: bool = True,
    print_warnings: bool = False,
) -> list[int]:
    if not settings.enable_checked_endpoint_correction or not state.fanout_anchor_net_ids:
        return []
    if not hasattr(state.router, "apply_checked_endpoint_corrections"):
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.apply_checked_endpoint_corrections. "
            "Rebuild it with `maturin develop --release`."
        )

    correction_jobs: list[
        tuple[
            int,
            Any,
            list[tuple[int, int]],
            list[tuple[int, int]],
            tuple[float, float] | None,
            tuple[float, float] | None,
        ]
    ] = []
    job_context_by_id: dict[int, tuple[RoutedNetRecord, bool, bool]] = {}
    requested_net_ids = [int(net_id) for net_id in net_ids]
    crossing_net_ids = _endpoint_correction_crossing_net_ids(settings, state)
    t_endpoint_correction_pack_start = timing.pipeline_timer_start(settings, state)
    for net_id in requested_net_ids:
        # A fanout stub is already a corrected endpoint adapter. When the
        # routed net also contains a crossing, the unrestricted native
        # endpoint corrector may move geometry on the protected side of the
        # crossing before we merge it back into the stubbed centerline. Let
        # the crossing-aware pass splice only the source->first-crossing or
        # last-crossing->target segment instead -- this pass's
        # classification puts any crossing net (and any both-sides-stub
        # net) into a category other than the two this pass owns, so
        # both exclusions fall out of the single category check below.
        classification = _classify_net_for_endpoint_correction(settings, state,
            net_id, crossing_net_ids=crossing_net_ids
        )
        # ALREADY_CORRECTED_NO_OP (source *and* target stubbed) dates from
        # the eager-stitch design in which both sides were pre-stitched;
        # a target stub is no longer, so such a net still needs its
        # target corrected to the anchor exactly like TARGET_ONLY.
        if classification is None or classification.category not in (
            EndpointCorrectionCategory.FANOUT_STUB_SOURCE_ONLY,
            EndpointCorrectionCategory.FANOUT_STUB_TARGET_ONLY,
            EndpointCorrectionCategory.ALREADY_CORRECTED_NO_OP,
        ):
            continue
        record = state.route_bookkeeping.records_by_id[net_id]
        job = state.route_jobs_by_id[net_id]
        source_has_fanout_stub = classification.source_has_fanout_stub
        target_has_fanout_stub = classification.target_has_fanout_stub

        source_port = (
            None
            if source_has_fanout_stub
            else _routing_endpoint_center_um(settings, state, job, source=True)
        )
        # A TARGET fanout stub is not eagerly stitched: the record's
        # target is the anchor's exact point and the search's grid state
        # still has to be corrected to it, otherwise the fixed stub gets
        # spliced onto the raw cell center and the last segment is
        # slanted (realization rejects it as an unsupported terminal
        # stub). Same reasoning as the crossing-aware pass's
        # `correct_target=True`; only SOURCE stubs are pre-stitched.
        target_port = _routing_endpoint_center_um(settings, state, job, source=False)
        if source_port is None and target_port is None:
            continue
        _, _, opened_candidate_cells, _, _ = _state_openings_for_job(settings, state, job)
        if source_has_fanout_stub:
            opened_candidate_cells.update(
                state.fanout_stub_static_cells_by_spec.get(f"{job.inst1},{job.port1}", set())
            )
        if target_has_fanout_stub:
            opened_candidate_cells.update(
                state.fanout_stub_static_cells_by_spec.get(f"{job.inst2},{job.port2}", set())
            )
        clearance_exempt_cells = _clearance_exempt_cells_for_job(settings, state, job)
        correction_jobs.append(
            (
                int(net_id),
                record.route_obj,
                sorted(opened_candidate_cells),
                clearance_exempt_cells,
                source_port,
                target_port,
            )
        )
        job_context_by_id[int(net_id)] = (
            record,
            source_has_fanout_stub,
            target_has_fanout_stub,
        )

    if record_pipeline_timing:
        timing.record_pipeline_timing(settings, state,
            "fanout_stub_endpoint_correction_pack",
            t_endpoint_correction_pack_start,
        )
    if not correction_jobs:
        return []

    correction_start = _timing_start(settings, state)
    raw_corrections = state.router.apply_checked_endpoint_corrections(
        correction_jobs,
        float(settings.route_width_um),
        int(state.commit_radius_cells),
        int(state.core_commit_radius_cells),
        True,
    )
    correction_elapsed_s = (
        time.perf_counter() - correction_start if state.collect_timing else 0.0
    )
    if record_pipeline_timing:
        timing.record_pipeline_timing(settings, state,
            "fanout_stub_endpoint_correction_native",
            correction_start,
        )
    t_endpoint_correction_processing_start = timing.pipeline_timer_start(settings, state)
    correction_elapsed_per_job_s = correction_elapsed_s / max(1, len(correction_jobs))
    failed_net_ids: list[int] = []
    for raw_correction in cast(Iterable[Any], raw_corrections):
        correction = dict(raw_correction)
        net_id = int(correction["net_id"])
        context = job_context_by_id.get(net_id)
        job = state.route_jobs_by_id.get(net_id)
        if context is None or job is None:
            continue
        record, source_has_fanout_stub, target_has_fanout_stub = context
        error = correction.get("error")
        if error is not None:
            if state.collect_timing:
                state.route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                    failed=True,
                )
            message = (
                "Checked fanout-stub endpoint correction skipped for net "
                f"{job.net_name!r}: {error}"
            )
            if print_warnings:
                print("WARNING: " + message)
            failed_net_ids.append(net_id)
            state.route_bookkeeping.records_by_id[net_id] = replace(
                record,
                endpoint_correction_error=message,
            )
            continue

        corrected_route = _dedupe_centerline(_centerline_tuple(correction.get("centerline")))
        route_baseline = _primitive_centerline_for_record(
            record,
            router=state.router,
            prefer_corrected_baseline=False,
        )
        existing_baseline = _dedupe_centerline(record.corrected_centerline_um)
        merged_centerline = _merge_terminal_corrected_route_centerline(
            existing_baseline=existing_baseline,
            route_baseline=route_baseline,
            corrected_route_centerline=corrected_route,
            freeze_source=source_has_fanout_stub,
            freeze_target=target_has_fanout_stub,
        )
        if len(merged_centerline) < 2:
            if state.collect_timing:
                state.route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                    failed=True,
                )
            route_start = route_baseline[0] if route_baseline else None
            route_end = route_baseline[-1] if route_baseline else None
            existing_start = existing_baseline[0] if existing_baseline else None
            existing_end = existing_baseline[-1] if existing_baseline else None
            corrected_start = corrected_route[0] if corrected_route else None
            corrected_end = corrected_route[-1] if corrected_route else None
            message = (
                "Checked fanout-stub endpoint correction skipped for net "
                f"{job.net_name!r}: could not merge corrected route segment "
                "back into static stub centerline "
                f"(existing_len={len(existing_baseline)}, "
                f"route_len={len(route_baseline)}, "
                f"corrected_len={len(corrected_route)}, "
                f"freeze_source={source_has_fanout_stub}, "
                f"freeze_target={target_has_fanout_stub}, "
                f"existing_start={existing_start}, existing_end={existing_end}, "
                f"route_start={route_start}, route_end={route_end}, "
                f"corrected_start={corrected_start}, corrected_end={corrected_end})"
            )
            if print_warnings:
                print("WARNING: " + message)
            failed_net_ids.append(net_id)
            state.route_bookkeeping.records_by_id[net_id] = replace(
                record,
                endpoint_correction_error=message,
            )
            continue

        if state.collect_timing:
            state.route_timing_buckets["endpoint_correction"].record_elapsed(
                correction_elapsed_per_job_s,
            )
        centerline_length = getattr(state.router, "centerline_length_um", None)
        if centerline_length is not None:
            try:
                corrected_total_length_um = float(centerline_length(list(merged_centerline)))
            except Exception:
                corrected_total_length_um = _centerline_length_um(merged_centerline)
        else:
            corrected_total_length_um = _centerline_length_um(merged_centerline)
        state.route_bookkeeping.records_by_id[net_id] = replace(
            record,
            total_length_um=corrected_total_length_um,
            base_total_length_um=(
                record.base_total_length_um
                if record.base_total_length_um is not None
                else float(record.total_length_um)
            ),
            corrected_centerline_um=merged_centerline,
            endpoint_correction_error=None,
        )

    if record_pipeline_timing:
        timing.record_pipeline_timing(settings, state,
            "fanout_stub_endpoint_correction_processing",
            t_endpoint_correction_processing_start,
        )
    return failed_net_ids


def _current_crossing_points_by_net_id(settings, state) -> dict[int, list[tuple[float, float]]]:
    if not settings.enable_crossings or not hasattr(state.router, "crossing_events"):
        return {}
    try:
        raw_events = list(cast(Iterable[Any], state.router.crossing_events()))
    except Exception:
        return {}
    if not raw_events:
        return {}
    _populate_realized_intersections_from_native_crossing_events(
        crossing_plan_info=state.crossing_plan_info,
        routed_records_by_net_id=state.route_bookkeeping.records_by_id,
        native_crossing_events=raw_events,
        realization_grid_spec=state.realization_grid_spec,
    )
    return _legal_crossing_points_by_net_id(state.crossing_plan_info.to_dict())


def _route_target_grid_center_um(
    settings,
    state,
    route_obj: object | None,
) -> tuple[float, float] | None:
    if route_obj is None:
        return None
    raw_state = getattr(route_obj, "reached_target", None)
    if raw_state is None:
        states = getattr(route_obj, "states", None)
        try:
            raw_state = cast(Any, states)[-1]
        except (TypeError, IndexError):
            return None
    try:
        return _grid_cell_center_um(settings, state, int(raw_state.x), int(raw_state.y))
    except (AttributeError, TypeError, ValueError):
        return None


def _route_target_angle(
    settings,
    state,
    route_obj: object | None,
) -> int | None:
    if route_obj is None:
        return None
    raw_state = getattr(route_obj, "reached_target", None)
    if raw_state is None:
        states = getattr(route_obj, "states", None)
        try:
            raw_state = cast(Any, states)[-1]
        except (TypeError, IndexError):
            return None
    try:
        return int(raw_state.angle) % 8
    except (AttributeError, TypeError, ValueError):
        return None


def _record_terminal_bump_distance_check_candidates(
    settings,
    state,
    crossing_points_by_net_id: Mapping[int, list[tuple[float, float]]],
    net_ids: Iterable[int],
) -> None:
    """Record where a terminal bump distance check would become active.

    This is diagnostic-only: the router behavior is unchanged.  The check
    is axis-specific: horizontal target approaches care only about
    physical-port-vs-grid y offset, while vertical target approaches care
    only about physical-port-vs-grid x offset.  If a realized crossing sits
    on that same target axis, a future A* guard must make sure the
    remaining terminal segment is long enough to insert the required bump
    geometry.
    """

    trace_tokens = settings.config.diagnostics.trace_terminal_bump_distance_checks
    trace_all = "*" in trace_tokens
    grid_size = float(state.grid.grid_size_um)
    eps = max(1e-6, grid_size * 1e-6)
    axis_eps = max(1e-6, grid_size * 0.25)
    crossing_half_um = (
        float(state.crossing_plan_info.crossing_half_size_cells or 0) * grid_size
    )
    required_bump_um = 4.0 * float(state.bend_radius_cells) * grid_size

    target_x_offset_nets: list[dict[str, object]] = []
    target_y_offset_nets: list[dict[str, object]] = []
    active_checks: list[dict[str, object]] = []

    for raw_net_id in net_ids:
        net_id = int(raw_net_id)
        record = state.route_bookkeeping.records_by_id.get(net_id)
        job = state.route_jobs_by_id.get(net_id)
        if record is None or job is None or record.target_port_center_um is None:
            continue
        target_grid_um = _route_target_grid_center_um(settings, state, record.route_obj)
        if target_grid_um is None:
            continue
        target_port_um = record.target_port_center_um
        target_dx_um = float(target_port_um[0]) - float(target_grid_um[0])
        target_dy_um = float(target_port_um[1]) - float(target_grid_um[1])
        target_angle = _route_target_angle(settings, state, record.route_obj)
        target_axis = (
            "horizontal"
            if target_angle in (0, 4)
            else "vertical"
            if target_angle in (2, 6)
            else "diagonal"
        )

        net_entry = {
            "net_id": int(net_id),
            "net_name": str(record.net_name),
            "target_port": f"{job.inst2},{job.port2}",
            "target_port_um": [
                round(float(target_port_um[0]), 6),
                round(float(target_port_um[1]), 6),
            ],
            "target_grid_um": [
                round(float(target_grid_um[0]), 6),
                round(float(target_grid_um[1]), 6),
            ],
            "target_dx_um": round(float(target_dx_um), 6),
            "target_dy_um": round(float(target_dy_um), 6),
            "target_angle": None if target_angle is None else int(target_angle),
            "target_axis": target_axis,
        }
        target_x_active = target_axis == "vertical" and abs(target_dx_um) > eps
        target_y_active = target_axis == "horizontal" and abs(target_dy_um) > eps
        if target_x_active:
            target_x_offset_nets.append(net_entry)
        if target_y_active:
            target_y_offset_nets.append(net_entry)
        if not target_x_active and not target_y_active:
            continue

        for crossing_point in crossing_points_by_net_id.get(net_id, []):
            crossing_x = float(crossing_point[0])
            crossing_y = float(crossing_point[1])
            if target_x_active and abs(crossing_x - float(target_grid_um[0])) <= axis_eps:
                distance_to_target_um = abs(float(target_grid_um[1]) - crossing_y)
                available_um = max(0.0, distance_to_target_um - crossing_half_um)
                active_checks.append(
                    {
                        **net_entry,
                        "axis": "target_x",
                        "crossing_um": [round(crossing_x, 6), round(crossing_y, 6)],
                        "distance_to_target_um": round(float(distance_to_target_um), 6),
                        "crossing_half_um": round(float(crossing_half_um), 6),
                        "available_um": round(float(available_um), 6),
                        "required_bump_um": round(float(required_bump_um), 6),
                        "satisfies": bool(available_um + eps >= required_bump_um),
                    }
                )
            if target_y_active and abs(crossing_y - float(target_grid_um[1])) <= axis_eps:
                distance_to_target_um = abs(float(target_grid_um[0]) - crossing_x)
                available_um = max(0.0, distance_to_target_um - crossing_half_um)
                active_checks.append(
                    {
                        **net_entry,
                        "axis": "target_y",
                        "crossing_um": [round(crossing_x, 6), round(crossing_y, 6)],
                        "distance_to_target_um": round(float(distance_to_target_um), 6),
                        "crossing_half_um": round(float(crossing_half_um), 6),
                        "available_um": round(float(available_um), 6),
                        "required_bump_um": round(float(required_bump_um), 6),
                        "satisfies": bool(available_um + eps >= required_bump_um),
                    }
                )

        should_trace = (
            trace_all or record.net_name in trace_tokens or str(net_id) in trace_tokens
        )
        if should_trace:
            matching_checks = [
                item for item in active_checks if int(item["net_id"]) == int(net_id)
            ]
            print(
                "terminal_bump_distance_check "
                f"net={record.net_name} id={net_id} "
                f"target_port={job.inst2},{job.port2} "
                f"target_port={target_port_um} target_grid={target_grid_um} "
                f"target_axis={target_axis} "
                f"target_dx={target_dx_um:.6f} target_dy={target_dy_um:.6f} "
                f"same_axis_crossings={len(matching_checks)}"
            )
            for item in matching_checks:
                print(
                    "terminal_bump_distance_check "
                    f"net={record.net_name} id={net_id} "
                    f"crossing={item['crossing_um']} "
                    f"available_um={item['available_um']} "
                    f"required_um={item['required_bump_um']} "
                    f"satisfies={item['satisfies']}"
                )

    state.crossing_plan_info.terminal_bump_target_x_offset_nets = target_x_offset_nets
    state.crossing_plan_info.terminal_bump_target_x_offset_net_count = len(target_x_offset_nets)
    state.crossing_plan_info.terminal_bump_target_y_offset_nets = target_y_offset_nets
    state.crossing_plan_info.terminal_bump_target_y_offset_net_count = len(target_y_offset_nets)
    failed_checks = [check for check in active_checks if not bool(check.get("satisfies"))]
    state.crossing_plan_info.terminal_bump_distance_checks = active_checks
    state.crossing_plan_info.terminal_bump_distance_check_count = len(active_checks)
    state.crossing_plan_info.terminal_bump_distance_failures = failed_checks
    state.crossing_plan_info.terminal_bump_distance_failure_count = len(failed_checks)


def _apply_crossing_aware_endpoint_corrections_for_net_ids(
    settings,
    state,
    net_ids: Iterable[int],
    *,
    record_pipeline_timing: bool = True,
    print_warnings: bool = False,
) -> list[int]:
    if (
        not settings.enable_checked_endpoint_correction
        or not settings.enable_crossings
    ):
        return []
    crossing_points_by_net_id = _current_crossing_points_by_net_id(settings, state)
    if not crossing_points_by_net_id:
        return []
    requested_net_ids = [int(net_id) for net_id in net_ids]
    _record_terminal_bump_distance_check_candidates(settings, state,
        crossing_points_by_net_id,
        requested_net_ids,
    )

    t_endpoint_correction_start = timing.pipeline_timer_start(settings, state)
    failed_net_ids: list[int] = []
    for raw_net_id in requested_net_ids:
        net_id = int(raw_net_id)
        crossing_points = crossing_points_by_net_id.get(net_id, [])
        if not crossing_points:
            # A net classified crossing-aware (the crossing plan expected
            # it to cross something) but with zero REALIZED crossing
            # points normally just gets skipped here entirely, on the
            # assumption it will be corrected some other way. That
            # assumption silently breaks for a net whose target has a
            # dense-fanout static stub (`target_has_fanout_stub`):
            # `record.target_port_center_um` was deliberately pointed at
            # the stub anchor's own exact position (not the true port)
            # specifically so THIS correction machinery would reconcile
            # it, but skipping here means nothing ever does -- see
            # `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`
            # Surprises & Discoveries for the concrete trace that found
            # this. `_apply_crossing_aware_endpoint_correction_to_record`
            # already has its own correct, generic handling for empty
            # `crossing_points` (falls back to plain
            # `apply_port_endpoint_corrections`), so only nets that
            # actually need it are let through here, keeping every other
            # net's existing behavior (and this function's own
            # early-return-on-nothing-to-do intent) unchanged.
            if net_id not in state.fanout_anchor_net_ids:
                continue
        record = state.route_bookkeeping.records_by_id.get(net_id)
        job = state.route_jobs_by_id.get(net_id)
        if record is None or job is None:
            continue

        source_has_fanout_stub = net_id in state.fanout_anchor_source_net_ids
        target_has_fanout_stub = net_id in state.fanout_anchor_target_net_ids
        record_has_fanout_stub = net_id in state.fanout_anchor_net_ids
        _, _, opened_candidate_cells, _, _ = _state_openings_for_job(settings, state, job)
        if source_has_fanout_stub:
            opened_candidate_cells.update(
                state.fanout_stub_static_cells_by_spec.get(f"{job.inst1},{job.port1}", set())
            )
        if target_has_fanout_stub:
            opened_candidate_cells.update(
                state.fanout_stub_static_cells_by_spec.get(f"{job.inst2},{job.port2}", set())
            )
        clearance_exempt_cells = _clearance_exempt_cells_for_job(settings, state, job)
        start_s = _timing_start(settings, state)
        updated = _apply_crossing_aware_endpoint_correction_to_record(
            record,
            router=cast(EndpointCorrectionRouter, state.router),
            crossing_points=crossing_points,
            realization_grid_spec=state.realization_grid_spec,
            route_width_um=float(settings.route_width_um),
            allow_unchecked_bumps=False,
            log_failures=print_warnings,
            crossing_plan_info=state.crossing_plan_info.to_dict(),
            correct_source=not source_has_fanout_stub,
            # Unlike a source fanout stub (still always eagerly,
            # fully pre-stitched to the true port by
            # `_fanout_stubbed_centerline`, so its side never needs
            # further correction), a TARGET fanout stub's own
            # `record.target_port_center_um` is deliberately pointed
            # at the anchor's own exact position, not the true port
            # (see `RouteBookkeeping.record_route`'s
            # `target_port_center_um_override`) -- so the target side
            # of a crossing-aware net always still needs correction
            # too, regardless of whether it has a fanout stub. Always
            # correcting here (instead of `not target_has_fanout_stub`,
            # which assumed the old eager-stitch design where the
            # target was already fully corrected) is what actually
            # closes that gap; see
            # `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
            correct_target=True,
            prefer_corrected_baseline=(
                record_has_fanout_stub and bool(record.corrected_centerline_um)
            ),
            opened_cells=opened_candidate_cells,
            clearance_exempt_cells=clearance_exempt_cells,
            clearance_radius_cells=int(state.commit_radius_cells),
            core_radius_cells=int(state.core_commit_radius_cells),
            config=settings.config.diagnostics,
        )
        failed = updated.endpoint_correction_error is not None
        if state.collect_timing:
            state.route_timing_buckets["endpoint_correction"].record_elapsed(
                time.perf_counter() - start_s,
                failed=failed,
            )
        if failed:
            failed_net_ids.append(net_id)
        state.route_bookkeeping.records_by_id[net_id] = updated

    if record_pipeline_timing:
        timing.record_pipeline_timing(settings, state,
            "crossing_endpoint_correction",
            t_endpoint_correction_start,
        )
    return failed_net_ids


def _apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
    settings,
    state,
    net_ids: Iterable[int],
    *,
    record_pipeline_timing: bool = True,
    print_warnings: bool = False,
) -> list[int]:
    """Pass 1 and pass 2 together: every net in `net_ids` not involved
    in a crossing gets either the unrestricted corrector or the
    fanout-stub-partial corrector, whichever `_classify_net_for_endpoint_correction`
    calls for; a net involved in a crossing is left untouched here (see
    `_apply_all_endpoint_corrections_for_net_ids` below for where that
    net gets handled). Both existing passes already self-filter safely
    by classification, so calling both with the same, unfiltered
    `net_ids` list is exactly equivalent to today's behavior, just
    expressed as one call instead of two.

    This is also, deliberately, as far as consolidation goes for the
    two mid-repair call sites in the batch-repair methods (around what were
    lines 5638-5645 and 6019-6028 before this milestone): repair runs
    before the run's final crossing plan is
    settled, so those call sites only ever need this pair, never the
    crossing-aware pass, and always did -- this method exists to give
    that pre-existing two-call pattern one name instead of leaving it
    duplicated verbatim in two places (Milestone 3 of
    .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md).
    """
    net_id_list = [int(net_id) for net_id in net_ids]
    failed_net_ids = list(
        _apply_checked_endpoint_corrections_for_net_ids(settings, state,
            net_id_list,
            record_pipeline_timing=record_pipeline_timing,
            print_warnings=print_warnings,
        )
    )
    failed_net_ids.extend(
        _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(settings, state,
            net_id_list,
            record_pipeline_timing=record_pipeline_timing,
            print_warnings=print_warnings,
        )
    )
    return failed_net_ids


def _apply_all_endpoint_corrections_for_net_ids(
    settings,
    state,
    net_ids: Iterable[int],
    *,
    print_warnings: bool = False,
) -> list[int]:
    """The one entry point for endpoint correction, telling the whole
    story top to bottom: every net not involved in a crossing gets the
    unrestricted or fanout-stub-partial corrector (whichever its
    classification calls for -- see `_classify_net_for_endpoint_correction`
    and `_apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids`
    above); every net involved in a crossing instead gets the separate,
    checked, per-segment crossing-aware corrector, which either succeeds
    or fails honestly with `RoutedNetRecord.endpoint_correction_error`
    set -- there is no unchecked fallback strategy (see
    .agent/execplans/2026-08-24-endpoint-correction-cascade-soundness.md).
    Replaces `run()`'s three separate, independently-ordered
    calls with one (Milestone 3 of
    .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md).

    This function is not used by the two mid-repair call sites, which
    only ever need the first two passes (see
    `_apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids`'s
    own docstring for why) -- calling this one there would run the
    crossing-aware pass mid-repair, before the run's final crossing
    plan is settled, which is not equivalent to today's behavior and
    was deliberately not attempted as part of this additive milestone.
    """
    net_id_list = [int(net_id) for net_id in net_ids]
    failed_net_ids = _apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
        settings, state,
        net_id_list,
        print_warnings=print_warnings,
    )
    failed_net_ids.extend(
        _apply_crossing_aware_endpoint_corrections_for_net_ids(settings, state,
            net_id_list,
            print_warnings=print_warnings,
        )
    )
    return failed_net_ids


def _append_target_fanout_stubs_after_correction(settings, state) -> None:
    """Splice each target-anchored net's fixed stub onto its corrected route.

    `_record_route`/`RouteBookkeeping.record_route` pointed any
    target-anchored net's own `target_port_center_um` at the anchor's
    exact position (not the true physical port) specifically so that
    the endpoint-correction pass just run in `_finalize_routing_results`
    would resolve the search's grid state to that exact point using
    the same machinery every ordinary port already relies on. Now that
    correction is done, append the fixed, pre-built stub segment
    (already known exactly -- `anchor.stub_centerline_um`, reversed --
    no further correction needed for it) to reach the true port, and
    restore `target_port_center_um` to the true port so downstream
    verification and reporting see the net's real, declared endpoint.
    See `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
    """
    for net_id, record in list(state.route_bookkeeping.records_by_id.items()):
        target_spec = f"{record.target.instance},{record.target.port}"
        anchor = state.fanout_anchor_by_port_spec.get(target_spec)
        if anchor is None:
            continue
        centerline = list(record.corrected_centerline_um)
        if len(centerline) < 2:
            continue
        endpoint_entry = state.endpoint_ports_by_spec.get(target_spec)
        if endpoint_entry is None:
            continue
        _inst, _port_name, port = endpoint_entry
        true_port_um = _port_center_um(port)
        if true_port_um is None:
            continue
        points = list(centerline)
        _append_centerline_points(settings, state, points, [true_port_um])
        new_centerline = _compress_centerline(tuple(points))
        if len(new_centerline) < 2:
            continue
        try:
            new_total_length_um = float(_centerline_length_um(new_centerline))
        except Exception:
            continue
        state.route_bookkeeping.records_by_id[net_id] = replace(
            record,
            corrected_centerline_um=new_centerline,
            total_length_um=new_total_length_um,
            target_port_center_um=true_port_um,
        )


def finalize_routing_results(
    settings: SessionSettings,
    state: SessionState,
    route_jobs: list[RouteJob],
    t_astar_start: float,
) -> RoutedRecords:
    """Apply checked endpoint corrections, assemble routed-net records, and print the debug timing breakdown.

    Computes `astar_elapsed_s` from `t_astar_start` (see
    `_finalize_route_jobs_and_static_handoff`'s docstring for why that reading was
    taken before this method's caller's own precompute work, not at native-dispatch
    time). Applies checked endpoint corrections if `self.enable_checked_endpoint_correction`,
    assembles `routed_net_records` from `self.route_bookkeeping` and raises
    `RuntimeError` if any duplicate net record was produced, then — only if
    `self.debug_timing and self.verbose_route_diagnostics` — prints the per-bucket A*
    timing breakdown. Returns the assembled records and `astar_elapsed_s` for the
    final verification/realization phases.
    """
    astar_elapsed_s = 0.0
    if state.collect_timing:
        astar_elapsed_s = time.perf_counter() - t_astar_start

    if settings.enable_checked_endpoint_correction:
        _apply_all_endpoint_corrections_for_net_ids(settings, state,
            list(state.route_bookkeeping.route_order),
            print_warnings=(
                settings.collect_attempt_diagnostics
                or state.diagnostics_enabled
                or settings.verbose_route_diagnostics
            ),
        )
    _append_target_fanout_stubs_after_correction(settings, state)

    t_record_assembly_start = timing.pipeline_timer_start(settings, state)
    routed_net_records = state.route_bookkeeping.ordered_records()
    routed_record_keys = [
        (
            record.net_name,
            record.source.instance,
            record.source.port,
            record.target.instance,
            record.target.port,
        )
        for record in routed_net_records
    ]
    duplicate_record_keys = [
        key for key, count in Counter(routed_record_keys).items() if count > 1
    ]
    if duplicate_record_keys:
        formatted = ", ".join(
            f"{name}:{src_i},{src_p}->{dst_i},{dst_p}"
            for name, src_i, src_p, dst_i, dst_p in duplicate_record_keys[:8]
        )
        raise RuntimeError(f"Duplicate routed records generated: {formatted}")
    timing.record_pipeline_timing(settings, state, "record_assembly", t_record_assembly_start)

    if settings.debug_timing and settings.verbose_route_diagnostics:
        print(f"      - A* route-search loop time: {astar_elapsed_s:.4f} s")
        print(
            "      - Route search stats: "
            f"simple={state.simple_route_count}/{len(route_jobs)}, "
            f"expanded_states={state.total_expanded_states}, "
            f"repairs={state.repair_count}, "
            f"deferred={state.deferred_count}"
        )
        print("      - A* timing breakdown by operation:")
        for bucket_name in (
            "normal_route",
            "probe_route",
            "preemptive_crossing_ripup",
            "guided_collision_crossing",
            "localized_crossing_keepout",
            "repair_failed_net",
            "reroute_victims",
            "lidar_pure_probe_commit",
            "endpoint_correction",
        ):
            bucket = state.route_timing_buckets[bucket_name]
            if bucket.calls == 0:
                continue
            line = (
                f"        {bucket_name}: calls={bucket.calls}, "
                f"ok={bucket.successes}, fail={bucket.failures}, "
                f"time={bucket.elapsed_s:.4f}s"
            )
            has_route_stats = (
                bucket.expanded_states
                or bucket.generated_neighbors
                or bucket.heap_pushes
                or bucket.heap_pops
                or bucket.window_attempts
                or bucket.footprint_checks
                or bucket.dense_grid_build_time_us
                or bucket.max_window_area_cells
                or bucket.full_grid_fallbacks
            )
            if has_route_stats:
                line += (
                    f", expanded={bucket.expanded_states}, "
                    f"generated={bucket.generated_neighbors}, "
                    f"heap_pushes={bucket.heap_pushes}, "
                    f"heap_pops={bucket.heap_pops}, "
                    f"duplicate_skips={bucket.skipped_duplicate_heap_entries}, "
                    f"windows={bucket.window_attempts}, "
                    f"max_window={bucket.max_window_area_cells}, "
                    f"legality_checks={bucket.obstacle_clearance_checks}, "
                    f"footprint_checks={bucket.footprint_checks}, "
                    f"rect_checks={bucket.footprint_rect_checks}, "
                    f"dense_cells={bucket.dense_grid_cells}, "
                    f"dense_build={bucket.dense_grid_build_time_us / 1_000_000.0:.4f}s, "
                    f"search_loop={bucket.search_loop_time_us / 1_000_000.0:.4f}s, "
                    f"obstacle_prepare={bucket.obstacle_map_prepare_time_us / 1_000_000.0:.4f}s, "
                    f"simple_probe={bucket.simple_route_time_us / 1_000_000.0:.4f}s, "
                    f"commit_prepare={bucket.commit_prepare_time_us / 1_000_000.0:.4f}s, "
                    f"commit={bucket.commit_time_us / 1_000_000.0:.4f}s, "
                    f"neighbor_time={bucket.neighbor_generation_time_us / 1_000_000.0:.4f}s, "
                    f"heap_time={bucket.heap_operation_time_us / 1_000_000.0:.4f}s, "
                    f"legality_time={bucket.legality_check_time_us / 1_000_000.0:.4f}s, "
                    f"reconstruction_time={bucket.reconstruction_time_us / 1_000_000.0:.4f}s, "
                    f"full_grid_fallbacks={bucket.full_grid_fallbacks}"
                )
            print(line)

    return RoutedRecords(routed_net_records, astar_elapsed_s)
