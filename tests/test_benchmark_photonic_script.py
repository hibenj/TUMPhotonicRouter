import csv
import importlib.util
import json
from pathlib import Path
import sys

import pytest


def _load_benchmark_photonic_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "benchmark_photonic.py"
    spec = importlib.util.spec_from_file_location("benchmark_photonic", path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row_with_attempts() -> dict[str, object]:
    return {
        "benchmark": "case",
        "instances": 2,
        "nets": 1,
        "grid": "10x10",
        "total_s": 1.0,
        "route_s": 0.9,
        "route_nets_s": 0.7,
        "path_length_analysis_s": None,
        "meander_obstacle_map_s": None,
        "meander_planning_s": None,
        "route_realization_s": 0.2,
        "astar_s": 0.8,
        "meander_requirements": 0,
        "meander_planner_calls": None,
        "meander_requested_um": None,
        "meander_inserted_um": None,
        "meander_disregarded_um": None,
        "meander_unmatched_um": None,
        "meander_status_counts": {},
        "meander_planner_elapsed_s": None,
        "meander_candidate_runs": 0,
        "meander_candidate_intervals": 0,
        "meander_rejected_box_blocked": 0,
        "meander_rejected_planning_failed": 0,
        "meander_rejected_exact_length_mismatch": 0,
        "meander_rejected_too_short": 0,
        "meander_max_candidate_runs": 0,
        "meander_max_candidate_intervals": 0,
        "slowest_meander_planning_s": None,
        "slowest_meander_status": None,
        "slowest_meander_requested_um": None,
        "slowest_meander_candidate_runs": None,
        "slowest_meander_candidate_intervals": None,
        "slowest_meander_rejected_box_blocked": None,
        "slowest_meander_rejected_planning_failed": None,
        "slowest_meander_rejected_exact_length_mismatch": None,
        "route_attempts": 2,
        "route_failures": 0,
        "simple_routes": 1,
        "repairs": 1,
        "expanded_states": 123,
        "generated_neighbors": 456,
        "heap_pushes": 300,
        "heap_pops": 250,
        "duplicate_heap_skips": 7,
        "stale_generation_heap_entries": 4,
        "closed_heap_entries": 3,
        "max_heap_size": 80,
        "dense_search_states": 2000,
        "dense_search_storage_bytes": 36_250,
        "best_cost_updates": 301,
        "parent_updates": 300,
        "obstacle_clearance_checks": 99,
        "footprint_rect_checks": 77,
        "full_grid_fallbacks": 0,
        "route_attempt_records": [
            {
                "attempt_index": 1,
                "bucket_name": "normal_route",
                "net_id": 1,
                "route_index": 1,
                "net_name": "n0",
                "source": "a,o1",
                "target": "b,o1",
                "elapsed_s": 0.01,
                "failed": False,
                "used_simple_route": True,
                "expanded_states": 0,
                "generated_neighbors": 0,
                "heap_pushes": 0,
                "heap_pops": 0,
                "skipped_duplicate_heap_entries": 0,
                "stale_generation_heap_entries": 0,
                "closed_heap_entries": 0,
                "max_heap_size": 0,
                "dense_search_states": 0,
                "dense_search_storage_bytes": 0,
                "best_cost_updates": 0,
                "parent_updates": 0,
                "obstacle_clearance_checks": 0,
                "footprint_rect_checks": 0,
                "dense_grid_build_time_s": 0.0,
            },
            {
                "attempt_index": 2,
                "bucket_name": "reroute_victims",
                "net_id": 1,
                "route_index": 1,
                "net_name": "n0",
                "source": "a,o1",
                "target": "b,o1",
                "elapsed_s": 0.75,
                "failed": False,
                "repair_round": 1,
                "used_simple_route": False,
                "expanded_states": 123,
                "generated_neighbors": 456,
                "heap_pushes": 300,
                "heap_pops": 250,
                "skipped_duplicate_heap_entries": 7,
                "stale_generation_heap_entries": 4,
                "closed_heap_entries": 3,
                "max_heap_size": 80,
                "dense_search_states": 2000,
                "dense_search_storage_bytes": 36_250,
                "best_cost_updates": 301,
                "parent_updates": 300,
                "obstacle_clearance_checks": 99,
                "footprint_rect_checks": 77,
                "dense_grid_build_time_s": 0.2,
            },
        ],
    }


def test_markdown_report_includes_slowest_route_attempts():
    module = _load_benchmark_photonic_module()
    args = module.argparse.Namespace(
        path_length_matching=False,
        allow_45_degree_turns=False,
        include_heater_obstacles=True,
        obstacle_mode="bounding_boxes",
        max_iterations=5_000_000,
        routing_window_scale=0.05,
        use_indexed_heap=False,
        primitive_ordering="library",
        heuristic_mode="distance",
        heap_tie_breaker="smaller_g",
    )

    report = module._markdown_report([_row_with_attempts()], args)

    assert "Slowest Route Attempts" in report
    assert "reroute_victims" in report
    assert "n0" in report
    assert "0.7500" in report
    assert "Dominant Route Diagnostics" not in report


def test_repeat_aggregation_uses_median_timing_fields():
    module = _load_benchmark_photonic_module()
    samples = []
    for route_nets_s in (0.9, 0.5, 0.7):
        row = _row_with_attempts()
        row["route_nets_s"] = route_nets_s
        row["route_s"] = route_nets_s + 0.1
        samples.append(row)

    aggregated = module._aggregate_repeat_rows(samples, "route_nets_s")

    assert aggregated["repeat_runs"] == 3
    assert aggregated["route_nets_s"] == 0.7
    assert aggregated["route_s"] == pytest.approx(0.8)
    assert aggregated["perf_metric_min"] == 0.5
    assert aggregated["perf_metric_max"] == 0.9


def test_perf_baseline_accepts_rows_within_tolerance():
    module = _load_benchmark_photonic_module()
    baseline_row = _row_with_attempts()
    baseline = module._perf_baseline_payload(
        [baseline_row],
        metric="route_nets_s",
        relative_tolerance=0.1,
        absolute_tolerance_s=0.05,
        counter_relative_tolerance=0.1,
    )
    current = json.loads(json.dumps(baseline_row))
    current["route_nets_s"] = 0.82

    violations = module._perf_baseline_violations(
        [current],
        baseline,
        metric="route_nets_s",
        relative_tolerance=0.1,
        absolute_tolerance_s=0.05,
        counter_relative_tolerance=0.1,
    )

    assert violations == []


def test_perf_baseline_reports_timing_and_counter_regression():
    module = _load_benchmark_photonic_module()
    baseline_row = _row_with_attempts()
    baseline = module._perf_baseline_payload(
        [baseline_row],
        metric="route_nets_s",
        relative_tolerance=0.1,
        absolute_tolerance_s=0.05,
        counter_relative_tolerance=0.1,
    )
    current = json.loads(json.dumps(baseline_row))
    current["route_nets_s"] = 0.83
    current["expanded_states"] = 200

    violations = module._perf_baseline_violations(
        [current],
        baseline,
        metric="route_nets_s",
        relative_tolerance=0.1,
        absolute_tolerance_s=0.05,
        counter_relative_tolerance=0.1,
    )

    assert {violation["name"] for violation in violations} == {
        "route_nets_s",
        "expanded_states",
    }


def test_main_fails_on_perf_baseline_violation(monkeypatch, tmp_path):
    module = _load_benchmark_photonic_module()
    baseline_row = _row_with_attempts()
    baseline_path = tmp_path / "baseline.json"
    baseline_path.write_text(
        json.dumps(
            module._perf_baseline_payload(
                [baseline_row],
                metric="route_nets_s",
                relative_tolerance=0.1,
                absolute_tolerance_s=0.05,
                counter_relative_tolerance=0.1,
            )
        ),
        encoding="utf-8",
    )
    current = json.loads(json.dumps(baseline_row))
    current["route_nets_s"] = 0.83

    monkeypatch.setattr(module, "_benchmark_rows", lambda args: [current])
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "benchmark_photonic.py",
            "case",
            "--compare-perf-baseline",
            str(baseline_path),
            "--perf-relative-tolerance",
            "0.1",
            "--perf-absolute-tolerance-s",
            "0.05",
        ],
    )

    assert module.main() == 1
    assert current["perf_baseline_violations"][0]["name"] == "route_nets_s"


def test_markdown_report_includes_diagnostics_when_available():
    module = _load_benchmark_photonic_module()
    args = module.argparse.Namespace(
        path_length_matching=False,
        allow_45_degree_turns=False,
        include_heater_obstacles=True,
        obstacle_mode="bounding_boxes",
        max_iterations=5_000_000,
        routing_window_scale=0.05,
        use_indexed_heap=False,
        primitive_ordering="library",
        heuristic_mode="distance",
        heap_tie_breaker="smaller_g",
        attempt_diagnostics=True,
    )
    row = _row_with_attempts()
    attempts = row["route_attempt_records"]
    assert isinstance(attempts, list)
    attempts[1]["diagnostics"] = {
        "span_x_cells": 12,
        "span_y_cells": 3,
        "window_width_cells": 20,
        "window_height_cells": 8,
        "route_bbox_width_cells": 10,
        "route_bbox_height_cells": 4,
        "window_to_span_bbox_area": 3.333,
        "route_bbox_to_window_area": 0.25,
        "heading_lower_bound_to_cost": 0.875,
        "window_static_density": 0.125,
        "window_dynamic_density": 0.25,
        "committed_dynamic_cells_before": 42,
        "candidate_blocker_count": 2,
        "ripup_victim_count": 1,
    }

    report = module._markdown_report([row], args)

    assert "Dominant Route Diagnostics" in report
    assert "12x3" in report
    assert "20x8" in report
    assert "10x4" in report
    assert "87.50%" in report
    assert "12.50%" in report
    assert "25.00%" in report


def test_markdown_report_includes_path_length_matching_section():
    module = _load_benchmark_photonic_module()
    args = module.argparse.Namespace(
        path_length_matching=True,
        allow_45_degree_turns=False,
        include_heater_obstacles=True,
        obstacle_mode="bounding_boxes",
        max_iterations=5_000_000,
        routing_window_scale=0.05,
        use_indexed_heap=False,
        primitive_ordering="library",
        heuristic_mode="distance",
        heap_tie_breaker="smaller_g",
    )
    row = _row_with_attempts()
    row.update(
        {
            "path_length_analysis_s": 0.01,
            "meander_obstacle_map_s": 0.02,
            "meander_planning_s": 0.03,
            "route_realization_s": 0.04,
            "path_length_group_count": 2,
            "path_length_groups_with_requirements": 2,
            "path_length_groups_over_tolerance": 0,
            "path_length_lifted_group_count": 1,
            "path_length_max_target_lift_um": 25.0,
            "path_length_min_insertable_extra_um": 25.0,
            "path_length_raw_requirements": 1,
            "meander_requirements": 2,
            "meander_planner_calls": 1,
            "meander_requested_um": 30.0,
            "meander_inserted_um": 20.0,
            "meander_disregarded_um": 5.0,
            "meander_unmatched_um": 10.0,
            "meander_status_counts": {"planned": 1, "below_minimum_bump": 1},
            "meander_planner_elapsed_s": 0.025,
            "meander_candidate_runs": 8,
            "meander_candidate_intervals": 13,
            "meander_rejected_box_blocked": 2,
            "meander_rejected_planning_failed": 3,
            "meander_rejected_exact_length_mismatch": 4,
            "meander_rejected_too_short": 5,
            "meander_max_candidate_runs": 6,
            "meander_max_candidate_intervals": 7,
            "slowest_meander_planning_s": 0.02,
            "slowest_meander_status": "planned",
            "slowest_meander_requested_um": 12.0,
            "slowest_meander_candidate_runs": 6,
            "slowest_meander_candidate_intervals": 7,
            "slowest_meander_rejected_box_blocked": 1,
            "slowest_meander_rejected_planning_failed": 2,
            "slowest_meander_rejected_exact_length_mismatch": 3,
        }
    )

    report = module._markdown_report([row], args)

    assert "Path-Length Matching" in report
    assert "Lifted groups" in report
    assert "| case | 2 | 2 | 1 | 25.0000 | 25.0000 | 1 | 0 |" in report
    assert "planned:1" in report
    assert "below_minimum_bump:1" in report
    assert "Meander Planner Diagnostics" in report
    assert "runs=6 intervals=7 blocked=1 plan_fail=2 exact_mismatch=3" in report


def test_path_length_group_diagnostics_accepts_get_only_info_objects():
    module = _load_benchmark_photonic_module()

    class _Info:
        def __init__(self, values: dict[str, object]) -> None:
            self._values = values

        def get(self, key: str, default: object = None) -> object:
            return self._values.get(key, default)

    layout_info = _Info(
        {
            "path_length_analysis": _Info(
                {
                    "matching_group_diagnostics": [
                        {
                            "node_name": "gate0",
                            "target_lift_um": 25.0,
                            "max_physical_residual_um": 0.0,
                        }
                    ]
                }
            )
        }
    )

    groups = module._path_length_group_diagnostics(layout_info)

    assert len(groups) == 1
    assert groups[0]["node_name"] == "gate0"
    assert module._count_lifted_groups(groups) == 1
    assert module._max_group_float(groups, "target_lift_um") == 25.0


def test_parser_defaults_keep_losing_experiments_gated(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(sys, "argv", ["benchmark_photonic.py"])

    args = module._parse_args()

    assert args.use_indexed_heap is False
    assert args.primitive_ordering == "library"
    assert args.heap_tie_breaker == "smaller_g"


def test_perf_smoke_preset_applies_release_benchmark_defaults(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        sys, "argv", ["benchmark_photonic.py", "--preset", "perf-smoke"]
    )

    args = module._parse_args()

    assert args.benchmarks == list(module.PERF_SMOKE_BENCHMARKS)
    assert args.output == module.PROJECT_ROOT / "build" / "perf_smoke.md"
    assert args.write_perf_baseline == module.PROJECT_ROOT / "build" / "perf_smoke.json"
    assert (
        args.attempt_output
        == module.PROJECT_ROOT / "build" / "perf_smoke_attempts.json"
    )
    assert args.require_release is True
    assert args.allow_45_degree_turns is False
    assert args.crossings is False
    assert args.include_heater_obstacles is False
    assert args.attempt_diagnostics is True


def test_perf_smoke_preset_keeps_explicit_benchmarks(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        sys,
        "argv",
        ["benchmark_photonic.py", "--preset", "perf-smoke", "benes_4x4"],
    )

    args = module._parse_args()

    assert args.benchmarks == ["benes_4x4"]
    assert args.require_release is True


def test_crossing_smoke_preset_applies_benes_stress_defaults(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        sys, "argv", ["benchmark_photonic.py", "--preset", "crossing-smoke"]
    )

    args = module._parse_args()

    assert args.benchmarks == list(module.CROSSING_SMOKE_BENCHMARKS)
    assert args.output == module.PROJECT_ROOT / "build" / "crossing_smoke.md"
    assert (
        args.write_perf_baseline
        == module.PROJECT_ROOT / "build" / "crossing_smoke.json"
    )
    assert (
        args.attempt_output
        == module.PROJECT_ROOT / "build" / "crossing_smoke_attempts.json"
    )
    assert args.require_release is True
    assert args.attempt_diagnostics is True
    assert args.continue_on_failure is True


def test_crossing_smoke_preset_keeps_explicit_benchmarks(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        sys,
        "argv",
        ["benchmark_photonic.py", "--preset", "crossing-smoke", "benes_16x16"],
    )

    args = module._parse_args()

    assert args.benchmarks == ["benes_16x16"]
    assert args.require_release is True
    assert args.continue_on_failure is True


def test_perf_smoke_worker_command_enables_crossings_only_for_benes(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(sys, "argv", ["benchmark_photonic.py", "--preset", "perf-smoke"])
    args = module._parse_args()

    benes_command = module._worker_command("benes_4x4", args)
    clements_command = module._worker_command("clements_8x8", args)

    assert "--crossings" in benes_command
    assert "--allow-45-degree-turns" in benes_command
    assert "--include-heater-obstacles" in benes_command
    assert benes_command[
        benes_command.index("--waveguide-clearance-um") + 1
    ] == "0.0"
    assert "--crossings" not in clements_command
    assert "--allow-45-degree-turns" not in clements_command
    assert "--include-heater-obstacles" not in clements_command


def test_crossing_smoke_worker_command_enables_crossings_for_large_benes(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        sys, "argv", ["benchmark_photonic.py", "--preset", "crossing-smoke"]
    )
    args = module._parse_args()

    command = module._worker_command("benes_16x16", args)

    assert "--crossings" in command
    assert "--allow-45-degree-turns" in command
    assert "--include-heater-obstacles" in command
    assert "--continue-on-failure" in command
    assert command[command.index("--grid-size-um") + 1] == "2.0"
    assert command[command.index("--waveguide-clearance-um") + 1] == "0.0"
    assert command[command.index("--heater-clearance-um") + 1] == "10.0"


def test_continue_on_failure_records_failed_worker_row(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        sys, "argv", ["benchmark_photonic.py", "--preset", "crossing-smoke"]
    )
    args = module._parse_args()

    def fail_route(*_args, **_kwargs):
        raise RuntimeError("synthetic route failure")

    monkeypatch.setattr(module, "run_routing_flow", fail_route)

    row = module._run_single_benchmark("benes_16x16", args)

    assert row["failed"] is True
    assert row["route_failures"] == 1
    assert "synthetic route failure" in row["error"]


def test_perf_smoke_worker_command_enables_plm_for_heater(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(sys, "argv", ["benchmark_photonic.py", "--preset", "perf-smoke"])
    args = module._parse_args()

    heater_command = module._worker_command("heater_s_mod", args)

    assert "--path-length-matching" in heater_command
    assert "--path-length-match-outputs" in heater_command
    assert "--include-heater-obstacles" in heater_command
    assert "--crossings" not in heater_command
    assert "--allow-45-degree-turns" not in heater_command


def test_require_release_rejects_debug_like_extension(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        module,
        "_rust_extension_info",
        lambda: {
            "path": "/tmp/photonic_router/_rust.abi3.so",
            "size_bytes": 50_000_000,
            "release_like": False,
            "reason": "extension looks like a debug build",
        },
    )

    with pytest.raises(SystemExit, match="release Rust extension"):
        module._ensure_release_extension()


def test_require_release_accepts_release_like_extension(monkeypatch):
    module = _load_benchmark_photonic_module()
    monkeypatch.setattr(
        module,
        "_rust_extension_info",
        lambda: {
            "path": "/tmp/photonic_router/_rust.abi3.so",
            "size_bytes": 2_000_000,
            "release_like": True,
            "reason": "",
        },
    )

    module._ensure_release_extension()


def test_write_attempt_output_supports_json_and_csv(tmp_path):
    module = _load_benchmark_photonic_module()
    rows = [_row_with_attempts()]
    json_path = tmp_path / "attempts.json"
    csv_path = tmp_path / "attempts.csv"

    module._write_attempt_output(rows, json_path)
    module._write_attempt_output(rows, csv_path)

    json_records = json.loads(json_path.read_text(encoding="utf-8"))
    assert json_records[1]["benchmark"] == "case"
    assert json_records[1]["bucket_name"] == "reroute_victims"

    with csv_path.open(newline="", encoding="utf-8") as stream:
        csv_records = list(csv.DictReader(stream))
    assert csv_records[1]["benchmark"] == "case"
    assert csv_records[1]["bucket_name"] == "reroute_victims"
