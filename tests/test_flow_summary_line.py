"""The fixed SUMMARY line every flow run ends with (agents grep it)."""

from __future__ import annotations

import pytest
from photonic_router import flow as routing_flow
from photonic_router.flow_stats import RoutingFlowStats, format_flow_summary_line

from tests.fixtures.flows import run_flow


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

    import gdsfactory as gf
    from photonic_router.flow_stats import populate_route_stats
    from photonic_router.records.route_rust_types import (
        RouteRustPipelineResult,
        RouteSearchSummary,
        RustRouteDebugArtifacts,
    )

    def populated(attempts: int, failures: int) -> RoutingFlowStats:
        stats = RoutingFlowStats()
        stats.benchmark_name = "x"
        stats.net_count = 128
        artifacts = RustRouteDebugArtifacts(
            obstacle_svg=None,
            route_svgs=[],
            routed_edge_lengths_um={},
            route_search_summary=RouteSearchSummary(
                route_attempts=attempts, route_failures=failures
            ),
        )
        populate_route_stats(
            stats,
            route_result=RouteRustPipelineResult(
                routed_layout=gf.Component(), debug_artifacts=artifacts
            ),
            debug_artifacts=artifacts,
            route_summary=artifacts.route_search_summary,
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
        run_flow("FAKE_FAILING")
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
