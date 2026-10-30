"""The fixed SUMMARY line every flow run ends with (agents grep it)."""

from __future__ import annotations

import contextlib

import pytest
from photonic_router import flow as routing_flow
from photonic_router.flow_stats import RoutingFlowStats, format_flow_summary_line


def test_summary_line_lists_every_field_and_marks_skipped_stages():
    stats = RoutingFlowStats()
    stats.benchmark_name = "cascade_8_folded"
    stats.net_count = 81
    stats.route_failures = 0
    stats.verification_error_count = 0
    stats.verification_warning_count = 0
    stats.path_length_groups = 26
    stats.path_length_groups_over_tolerance = 0
    stats.meander_inserted_um = 23536.447
    stats.electrical_terminal_groups = 21
    stats.electrical_pad_assignments = 22
    stats.electrical_failed_detailed_routes = 0
    stats.electrical_verification_success = True
    stats.total_time_s = 4.94

    line = format_flow_summary_line(stats)

    assert line == (
        "SUMMARY benchmark=cascade_8_folded nets=81 route_failures=0 detours=- "
        "verifier_errors=0 verifier_warnings=0 plm_groups=26 plm_over_tolerance=0 "
        "meander_inserted_um=23536.4 electrical_heaters=21 electrical_pads=22 "
        "electrical_failed_wires=0 electrical_ok=yes total_s=4.9"
    )


def test_route_failures_print_a_dash_without_per_route_collection():
    """``populate_route_stats`` copies the failure count from the timing
    buckets, which are empty unless per-route collection ran; then the line
    says ``-``, not a guessed 0 (merge review 2026-10-05)."""

    from types import SimpleNamespace

    from photonic_router.flow_stats import populate_route_stats

    def summary(attempts: int, failures: int) -> SimpleNamespace:
        return SimpleNamespace(
            astar_elapsed_s=0.0,
            route_attempts=attempts,
            route_failures=failures,
            simple_route_count=0,
            repair_count=0,
            full_grid_fallbacks=0,
            endpoint_correction_time_s=0.0,
            endpoint_correction_calls=0,
            endpoint_correction_failures=0,
            expanded_states=0,
            generated_neighbors=0,
            heap_pushes=0,
            heap_pops=0,
            skipped_duplicate_heap_entries=0,
            stale_generation_heap_entries=0,
            closed_heap_entries=0,
            max_heap_size=0,
            dense_search_states=0,
            dense_search_storage_bytes=0,
            best_cost_updates=0,
            parent_updates=0,
            obstacle_clearance_checks=0,
            footprint_checks=0,
            footprint_rect_checks=0,
            crossing_candidate_checks=0,
            crossing_accepted=0,
            crossing_reject_non_straight=0,
            crossing_reject_not_perpendicular=0,
            crossing_reject_margin=0,
            crossing_reject_wrong_order=0,
            crossing_reject_unexpected_owner=0,
            crossing_reject_unmatched_owner=0,
            crossing_reject_unmatched_centerline=0,
            crossing_reject_unmatched_footprint=0,
            crossing_reject_unmatched_route_centerline=0,
            crossing_reject_unmatched_route_footprint=0,
            crossing_reject_pending_straight=0,
            search_loop_time_s=0.0,
            obstacle_map_prepare_time_s=0.0,
            simple_route_time_s=0.0,
            commit_prepare_time_s=0.0,
            commit_time_s=0.0,
            neighbor_generation_time_s=0.0,
            heap_operation_time_s=0.0,
            primitive_candidate_counts={},
            primitive_accepted_counts={},
            primitive_rejected_blocked_counts={},
            primitive_rejected_bounds_counts={},
            primitive_rejected_turn_counts={},
            primitive_rejected_worse_cost_counts={},
            primitive_rejected_other_counts={},
        )

    def populated(attempts: int, failures: int) -> RoutingFlowStats:
        stats = RoutingFlowStats()
        stats.benchmark_name = "x"
        stats.net_count = 128
        # fields this stub does not model are not what the test is about
        with contextlib.suppress(AttributeError):
            populate_route_stats(
                stats,
                route_result=None,
                debug_artifacts=None,
                route_summary=summary(attempts, failures),
                route_attempt_records=[],
                route_time_s=0.0,
            )
        return stats

    assert "route_failures=- " in format_flow_summary_line(populated(0, 0))
    assert "route_failures=4 " in format_flow_summary_line(populated(132, 4))


def test_summary_line_tells_zero_heaters_from_a_skipped_electrical_stage():
    stats = RoutingFlowStats()
    stats.benchmark_name = "plain"
    stats.net_count = 8
    assert "electrical_heaters=- electrical_pads=- electrical_failed_wires=- " in (
        format_flow_summary_line(stats)
    )
    stats.electrical_terminal_groups = 0
    stats.electrical_pad_assignments = 0
    stats.electrical_failed_detailed_routes = 0
    assert "electrical_heaters=0 electrical_pads=0 electrical_failed_wires=0 " in (
        format_flow_summary_line(stats)
    )


def test_a_failing_run_still_ends_with_the_summary_line(monkeypatch, capsys):
    """``route_benchmark`` prints the footer in a ``finally``: a stage that
    raises leaves the SUMMARY line on stdout with ``-`` for what did not run."""

    monkeypatch.setattr(routing_flow, "_load_benchmark_stage", lambda **kwargs: object())

    def explode(schematic, config, options):
        raise RuntimeError("stage failed")

    monkeypatch.setattr(routing_flow, "route_schematic", explode)
    with pytest.raises(RuntimeError, match="stage failed"):
        routing_flow.run_routing_flow("FAKE_FAILING")
    out = capsys.readouterr().out
    assert "SUMMARY benchmark=FAKE_FAILING" in out
    assert "route_failures=- " in out


def test_summary_line_prints_dashes_for_stages_that_did_not_run():
    stats = RoutingFlowStats()
    stats.benchmark_name = "benes_4x4_flat"
    stats.net_count = 8

    line = format_flow_summary_line(stats)

    assert "verifier_errors=- " in line
    assert "plm_groups=- " in line
    assert "electrical_heaters=- " in line
    assert "electrical_ok=-" in line
    assert format_flow_summary_line(None) == "SUMMARY stats=off"
