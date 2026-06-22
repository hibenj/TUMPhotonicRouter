import csv
import importlib.util
import json
from pathlib import Path
import sys


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
        "astar_s": 0.8,
        "route_attempts": 2,
        "simple_routes": 1,
        "repairs": 1,
        "expanded_states": 123,
        "generated_neighbors": 456,
        "heap_pushes": 300,
        "heap_pops": 250,
        "duplicate_heap_skips": 7,
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
    )

    report = module._markdown_report([_row_with_attempts()], args)

    assert "Slowest Route Attempts" in report
    assert "reroute_victims" in report
    assert "n0" in report
    assert "0.7500" in report


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
