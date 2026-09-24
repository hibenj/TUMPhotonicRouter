"""Unit tests for `translation/routing/timing.py`.

The three functions are the routing session's only timers: two feed the
per-phase pipeline breakdown the flow reports (`route_nets_timings_s`) and one
feeds the per-net `route_timing_buckets`. Each is gated by its own flag, so the
tests below cover both the on and the off state of every gate -- when the gate
is off the function must not read the clock or touch the state at all.

Added by Milestone 6, Slice 3 of
`.agent/execplans/2026-09-22-modular-readable-router-restructure.md` (the
module had no direct test).
"""

from __future__ import annotations

from types import SimpleNamespace

from translation.route_rust_types import RouteTimingBucket
from translation.routing.timing import (
    pipeline_timer_start,
    record_elapsed,
    record_pipeline_timing,
)


def _pipeline_state() -> SimpleNamespace:
    return SimpleNamespace(route_nets_timings_s={})


def _bucket_state(*, collect_timing: bool, bucket_names: tuple[str, ...]) -> SimpleNamespace:
    return SimpleNamespace(
        collect_timing=collect_timing,
        route_timing_buckets={name: RouteTimingBucket() for name in bucket_names},
    )


def test_pipeline_timer_start_returns_a_perf_counter_reading_when_timing_is_on():
    settings = SimpleNamespace(collect_pipeline_timing=True)
    first = pipeline_timer_start(settings, _pipeline_state())
    second = pipeline_timer_start(settings, _pipeline_state())
    assert first > 0.0
    # `time.perf_counter` is monotonic, so a later call never reads earlier.
    assert second >= first


def test_pipeline_timer_start_returns_zero_when_timing_is_off():
    settings = SimpleNamespace(collect_pipeline_timing=False)
    assert pipeline_timer_start(settings, _pipeline_state()) == 0.0


def test_record_pipeline_timing_adds_the_elapsed_time_under_the_phase_name():
    settings = SimpleNamespace(collect_pipeline_timing=True)
    state = _pipeline_state()
    start = pipeline_timer_start(settings, state)
    record_pipeline_timing(settings, state, "obstacle_context", start)
    assert list(state.route_nets_timings_s) == ["obstacle_context"]
    assert state.route_nets_timings_s["obstacle_context"] >= 0.0


def test_record_pipeline_timing_accumulates_repeated_phases_instead_of_replacing_them():
    settings = SimpleNamespace(collect_pipeline_timing=True)
    state = _pipeline_state()
    # Two synthetic intervals of 1.0 and 0.5 seconds: the bucket must hold
    # their sum, not the last one. `start_s` is a `perf_counter` reading, so
    # `now - start_s` is the interval; passing `now - 1.0` makes it 1.0.
    now = pipeline_timer_start(settings, state)
    record_pipeline_timing(settings, state, "router_setup", now - 1.0)
    after_first = state.route_nets_timings_s["router_setup"]
    assert after_first >= 1.0
    record_pipeline_timing(settings, state, "router_setup", pipeline_timer_start(settings, state) - 0.5)
    assert state.route_nets_timings_s["router_setup"] >= after_first + 0.5


def test_record_pipeline_timing_writes_nothing_when_timing_is_off():
    settings = SimpleNamespace(collect_pipeline_timing=False)
    state = _pipeline_state()
    record_pipeline_timing(settings, state, "router_setup", 0.0)
    assert state.route_nets_timings_s == {}


def test_record_elapsed_counts_one_successful_call_on_the_named_bucket():
    state = _bucket_state(collect_timing=True, bucket_names=("normal_route",))
    record_elapsed(None, state, "normal_route", 0.0)
    bucket = state.route_timing_buckets["normal_route"]
    assert bucket.calls == 1
    assert bucket.failures == 0
    assert bucket.successes == 1
    assert bucket.elapsed_s > 0.0


def test_record_elapsed_counts_a_failed_call_as_both_a_call_and_a_failure():
    state = _bucket_state(collect_timing=True, bucket_names=("normal_route",))
    record_elapsed(None, state, "normal_route", 0.0, failed=True)
    bucket = state.route_timing_buckets["normal_route"]
    assert bucket.calls == 1
    assert bucket.failures == 1
    assert bucket.successes == 0


def test_record_elapsed_accumulates_over_calls_and_leaves_other_buckets_alone():
    state = _bucket_state(collect_timing=True, bucket_names=("normal_route", "probe_route"))
    record_elapsed(None, state, "normal_route", 0.0)
    record_elapsed(None, state, "normal_route", 0.0, failed=True)
    assert state.route_timing_buckets["normal_route"].calls == 2
    assert state.route_timing_buckets["normal_route"].failures == 1
    assert state.route_timing_buckets["probe_route"].calls == 0


def test_record_elapsed_writes_nothing_when_per_net_timing_is_off():
    state = _bucket_state(collect_timing=False, bucket_names=("normal_route",))
    record_elapsed(None, state, "normal_route", 0.0)
    assert state.route_timing_buckets["normal_route"].calls == 0
    assert state.route_timing_buckets["normal_route"].elapsed_s == 0.0
