import importlib.util
from pathlib import Path
import sys


def _load_profile_astar_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "profile_astar.py"
    spec = importlib.util.spec_from_file_location("profile_astar", path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_scenario_catalog_contains_expected_smoke_cases():
    module = _load_profile_astar_module()
    catalog = module._scenario_catalog()

    assert {"straight_simple", "straight_astar", "wall_gap_astar"} <= set(catalog)
    assert "object_ports_n_n" in catalog
    assert "object_ports_w_s" in catalog
    assert "jps4_empty_grid" in catalog
    assert "jps4_corridor" in catalog
    assert "jps4_forced_detour" in catalog
    assert catalog["straight_simple"].enable_simple_routes is True
    assert catalog["straight_astar"].enable_simple_routes is False
    assert catalog["jps4_empty_grid"].primitive_mode == "jps4_unit"
    assert len(catalog["wall_gap_astar"].static_cells) > 0


def test_default_scenario_names_exclude_jps4_experiments():
    module = _load_profile_astar_module()
    catalog = module._scenario_catalog()
    defaults = module._default_scenario_names(catalog)

    assert "straight_astar" in defaults
    assert "object_ports_n_n" in defaults
    assert "jps4_empty_grid" not in defaults


def test_two_object_port_scenarios_cover_all_cardinal_pairs():
    module = _load_profile_astar_module()
    scenarios = module._two_object_port_scenarios()

    assert len(scenarios) == 16
    expected_names = {
        f"object_ports_{source}_{target}"
        for source in ("n", "e", "s", "w")
        for target in ("n", "e", "s", "w")
    }
    assert set(scenarios) == expected_names

    for name, scenario in scenarios.items():
        _prefix, _ports, source_side, target_side = name.split("_")
        assert scenario.enable_simple_routes is False
        assert scenario.require_target_angle is True
        assert scenario.source[2] == module.PORT_ANGLES[source_side]
        assert scenario.target[2] == (module.PORT_ANGLES[target_side] + 4) % 8
        assert (scenario.source[0], scenario.source[1]) not in scenario.static_cells
        assert (scenario.target[0], scenario.target[1]) not in scenario.static_cells


def test_markdown_report_contains_route_stats_columns():
    module = _load_profile_astar_module()
    args = module.argparse.Namespace(iterations=3, warmup=1)
    report = module._markdown_report(
        [
            {
                "scenario": "case",
                "grid": "10x10",
                "static_cells": 2,
                "median_s": 0.001,
                "p95_s": 0.002,
                "expanded_states": 3,
                "generated_neighbors": 8,
                "heap_pushes": 6,
                "heap_pops": 5,
                "duplicate_heap_skips": 1,
                "obstacle_clearance_checks": 4,
                "window_attempts": 1,
                "footprint_checks": 4,
                "dense_build_s": 0.0,
                "neighbor_generation_s": 0.0001,
                "heap_operation_s": 0.0002,
                "legality_check_s": 0.0003,
                "reconstruction_s": 0.0004,
                "jps4_requested": True,
                "jps4_eligible": False,
                "jps4_used": False,
                "jps4_fallbacks": 1,
                "jps4_fallback_reason": "primitive library is not plain 4-connected unit grid",
                "full_grid_fallback": False,
                "target_state_ok": True,
                "route_cells": 5,
                "route_length_um": 6.0,
            }
        ],
        args,
    )

    assert "Isolated Rust A* Profile" in report
    assert "Footprint checks" in report
    assert "Heap push/pop" in report
    assert "Legality checks" in report
    assert "JPS4 fallback" in report
    assert "primitive library is not plain 4-connected unit grid" in report
    assert "| case | 10x10 | 2 |" in report


def test_paired_report_contains_acceleration_columns():
    module = _load_profile_astar_module()
    args = module.argparse.Namespace(iterations=3, warmup=1)
    report = module._markdown_paired_report(
        [
            {
                "scenario": "jps4_empty_grid",
                "primitive_mode": "jps4_unit",
                "baseline": {
                    "median_s": 0.002,
                    "expanded_states": 100,
                    "generated_neighbors": 400,
                    "heap_pushes": 150,
                    "heap_pops": 100,
                    "route_length_um": 20.0,
                },
                "accelerated": {
                    "median_s": 0.001,
                    "expanded_states": 10,
                    "generated_neighbors": 40,
                    "heap_pushes": 20,
                    "heap_pops": 10,
                    "route_length_um": 20.0,
                    "jps4_requested": True,
                    "jps4_eligible": True,
                    "jps4_used": True,
                    "jps4_fallback_reason": "eligible",
                },
                "length_delta_um": 0.0,
                "target_cell_match": True,
                "target_state_match": False,
            }
        ],
        args,
    )

    assert "Paired Rust A* Accelerator Comparison" in report
    assert "Expanded ratio" in report
    assert "jps4_empty_grid" in report
    assert "| jps4_empty_grid | jps4_unit |" in report
    assert "| used | eligible |" in report


def test_baseline_check_accepts_matching_route_quality():
    module = _load_profile_astar_module()
    failures = module._check_baseline(
        [
            {
                "scenario": "case",
                "route_length_um": 10.0,
                "target": [5, 6, 0],
                "reached_target": [5, 6, 2],
                "target_state_ok": False,
            }
        ],
        {
            "length_tolerance_um": 0.1,
            "scenarios": {
                "case": {
                    "route_length_um": 10.0,
                    "target": [5, 6, 0],
                    "reached_target": [5, 6, 2],
                    "target_state_ok": False,
                }
            },
        },
        length_tolerance_um=None,
    )

    assert failures == []


def test_baseline_check_rejects_longer_routes_and_wrong_target_state():
    module = _load_profile_astar_module()
    failures = module._check_baseline(
        [
            {
                "scenario": "case",
                "route_length_um": 11.0,
                "target": [5, 6, 0],
                "reached_target": [5, 6, 2],
                "target_state_ok": False,
            }
        ],
        {
            "length_tolerance_um": 0.1,
            "scenarios": {
                "case": {
                    "route_length_um": 10.0,
                    "target": [5, 6, 0],
                    "reached_target": [5, 6, 0],
                    "target_state_ok": True,
                }
            },
        },
        length_tolerance_um=None,
    )

    assert any("route length" in failure for failure in failures)
    assert any("reached target" in failure for failure in failures)
    assert any("required target state" in failure for failure in failures)
