"""The routing session's pipeline timers.

Milestone 5 Slice 2b turned the session's timer methods into module functions so
that the stage modules call them directly instead of through a compatibility
forwarder on the session class. `pipeline_timer_start` / `record_pipeline_timing`
feed the `route_nets_timings_s` bucket the flow reports as the per-phase pipeline
breakdown.

Milestone 8 Slice C removed `record_elapsed`, which no phase of the pipeline
reached; the per-net `route_timing_buckets` are fed by
`RouteTimingBucket.record_elapsed` at the call sites directly.
"""

from __future__ import annotations

import time


def pipeline_timer_start(settings, state) -> float:
    return time.perf_counter() if settings.collect_pipeline_timing else 0.0


def record_pipeline_timing(settings, state, name: str, start_s: float) -> None:
    if settings.collect_pipeline_timing:
        state.route_nets_timings_s[name] = state.route_nets_timings_s.get(name, 0.0) + (
            time.perf_counter() - start_s
        )
