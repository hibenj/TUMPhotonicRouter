"""`RouteTimingBucket.record_route` adds the search statistics of a route by name."""

from __future__ import annotations

import dataclasses
from types import SimpleNamespace

from photonic_router.records import route_rust_types as types
from photonic_router.records.route_rust_types import (
    PRIMITIVE_TRANSITION_CLASSES,
    RouteTimingBucket,
)


def _route(offset: int) -> SimpleNamespace:
    """A route whose every listed statistic has its own value, shifted by ``offset``."""

    stats = {stat: offset + index for index, (_, stat) in enumerate(types._BUCKET_SUMMED_STATS, 1)}
    stats.update(
        {name: 1000 * offset + index for index, name in enumerate(types._BUCKET_MAX_STATS)}
    )
    stats.update(
        {
            name: [offset + index] * len(PRIMITIVE_TRANSITION_CLASSES)
            for index, name in enumerate(types._BUCKET_COUNTER_DICTS)
        }
    )
    return SimpleNamespace(**stats)


def test_every_listed_statistic_is_a_field_of_the_bucket():
    fields = {field.name for field in dataclasses.fields(RouteTimingBucket)}
    summed = [field for field, _ in types._BUCKET_SUMMED_STATS]
    assert len(summed) == 61 and len(set(summed)) == 61
    assert len({stat for _, stat in types._BUCKET_SUMMED_STATS}) == 61
    assert set(summed) <= fields
    assert types._BUCKET_MAX_STATS == ("max_heap_size", "max_window_area_cells")
    assert set(types._BUCKET_MAX_STATS) <= fields
    assert len(types._BUCKET_COUNTER_DICTS) == 7
    assert set(types._BUCKET_COUNTER_DICTS) <= fields
    # the four statistics the kernel names differently from the bucket
    assert {(f, s) for f, s in types._BUCKET_SUMMED_STATS if f != s} == {
        ("footprint_checks", "primitive_footprint_checks"),
        ("footprint_cells_tested", "primitive_footprint_cells_tested"),
        ("footprint_rect_checks", "primitive_footprint_rect_checks"),
        ("footprint_rect_rejects", "primitive_footprint_rect_rejects"),
    }


def test_two_routes_are_summed_and_the_maxima_kept():
    bucket = RouteTimingBucket()
    bucket.record_route(0.5, _route(10))
    bucket.record_route(0.25, _route(3))

    assert bucket.calls == 2 and bucket.failures == 0 and bucket.elapsed_s == 0.75
    for index, (field, _) in enumerate(types._BUCKET_SUMMED_STATS, 1):
        assert getattr(bucket, field) == (10 + index) + (3 + index), field
    for index, name in enumerate(types._BUCKET_MAX_STATS):
        assert getattr(bucket, name) == 10_000 + index, name
    for index, name in enumerate(types._BUCKET_COUNTER_DICTS):
        assert getattr(bucket, name) == dict.fromkeys(
            PRIMITIVE_TRANSITION_CLASSES, (10 + index) + (3 + index)
        ), name
    assert bucket.full_grid_fallbacks == 0


def test_a_failed_attempt_counts_the_time_only_and_a_fallback_is_counted():
    bucket = RouteTimingBucket()
    bucket.record_route(1.0, _route(5), failed=True)
    assert (bucket.calls, bucket.failures, bucket.expanded_states) == (1, 1, 0)

    route = _route(5)
    route.used_full_grid_fallback = True
    route.heap_pushes = -4  # a negative statistic counts as 0
    bucket.record_route(1.0, route)
    assert bucket.full_grid_fallbacks == 1
    assert bucket.heap_pushes == 0
