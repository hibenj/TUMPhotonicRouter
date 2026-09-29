"""The fixed SUMMARY line every flow run ends with (agents grep it)."""

from routing_flow_stats import RoutingFlowStats, format_flow_summary_line


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
        "SUMMARY benchmark=cascade_8_folded nets=81 route_failures=0 "
        "verifier_errors=0 verifier_warnings=0 plm_groups=26 plm_over_tolerance=0 "
        "meander_inserted_um=23536.4 electrical_heaters=21 electrical_pads=22 "
        "electrical_failed_wires=0 electrical_ok=yes total_s=4.9"
    )


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
