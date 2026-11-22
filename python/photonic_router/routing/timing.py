"""The routing session's pipeline timers.

Milestone 5 Slice 2b turned the session's timer methods into module functions so
that the stage modules call them directly instead of through a compatibility
forwarder on the session class. `pipeline_timer_start` / `record_pipeline_timing`
feed the `route_nets_timings_s` bucket the flow reports as the per-phase pipeline
breakdown.

`record_elapsed` is the same for a caller that holds the bucket itself:
phases 1 and 2 run before the session state exists. The per-net
`route_timing_buckets` are fed by `RouteTimingBucket.record_elapsed` at the
call sites directly.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # annotation-only: the stage modules import this one
    from photonic_router.routing.settings import SessionSettings
    from photonic_router.routing.state import SessionState


def pipeline_timer_start(settings: SessionSettings) -> float:
    """Return the clock when pipeline timing is collected, 0.0 otherwise."""
    return time.perf_counter() if settings.collect_pipeline_timing else 0.0


def record_elapsed(
    settings: SessionSettings, timings_s: dict[str, float], name: str, start_s: float
) -> None:
    """Add the time since `start_s` to the bucket `name` of `timings_s`."""
    if settings.collect_pipeline_timing:
        timings_s[name] = timings_s.get(name, 0.0) + (time.perf_counter() - start_s)


def record_pipeline_timing(
    settings: SessionSettings, state: SessionState, name: str, start_s: float
) -> None:
    """Add the time since `start_s` to the bucket `name` of the session state."""
    record_elapsed(settings, state.route_nets_timings_s, name, start_s)
