from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from translation.electrical.types import ElectricalRoutingConfig


def _load_benchmark_electrical_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "benchmark_electrical.py"
    spec = importlib.util.spec_from_file_location("benchmark_electrical", path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _Info:
    def __init__(self, values: dict[str, object]) -> None:
        self._values = values

    def get(self, key: str, default: object = None) -> object:
        return self._values.get(key, default)


class _RoutedComponent:
    def __init__(self) -> None:
        self.gds_path: Path | None = None

    def write_gds(self, path: Path) -> None:
        self.gds_path = path
        path.write_text("gds", encoding="utf-8")


def _result_stub() -> SimpleNamespace:
    verification = SimpleNamespace(
        success=True,
        error_count=0,
        warning_count=0,
        issues=(),
        metrics={
            "net_count": 12,
            "rect_count": 102,
            "raw_metal_area_um2": 556_778.772,
            "union_metal_area_um2": 549_599.152,
            "metal_area_overcount_um2": 7_179.619999999995,
            "metal_area_overcount_ratio": 0.0128949240902453,
            "metal_area_overcount_by_reason_um2": {
                "bus_stripe_contact": 1_255.659999999998,
                "pad_contact": 2_172.9600000000073,
                "terminal_access_join": 3_751.0,
            },
            "metal_area_overcount_by_source_um2": {
                "bus_escape/bus_stripe": 255.65999999999804,
                "bus_escape/pad": 181.0800000000006,
                "bus_route/bus_stripe": 1_000.0,
                "bus_route/terminal_adapter": 1_875.5,
                "pad/route_tail": 1_991.8800000000067,
                "route_tail/terminal_adapter": 1_875.5,
            },
            "metal_redundant_area_overcount_um2": 0,
            "metal_redundant_area_overcount_by_source_um2": {},
            "same_net_duplicate_rect_count": 0,
            "same_net_overlap_pair_count": 41,
            "same_net_overlap_pair_count_by_source": {
                "bus_escape/bus_stripe": 1,
                "bus_escape/pad": 1,
                "bus_route/bus_stripe": 3,
                "bus_route/terminal_adapter": 12,
                "pad/route_tail": 12,
                "route_tail/terminal_adapter": 12,
            },
            "same_net_intentional_overlap_pair_count": 41,
            "same_net_intentional_overlap_pair_count_by_reason": {
                "bus_stripe_contact": 4,
                "pad_contact": 13,
                "terminal_access_join": 24,
            },
            "same_net_redundant_overlap_pair_count": 0,
            "same_net_redundant_overlap_pair_count_by_source": {},
            "cross_net_min_spacing_um": 11.0,
            "required_cross_net_clearance_um": 10.0,
            "centerline_length_um": 19_390.0,
            "bend_count": 65,
            "pad_channel_height_um": 180.0,
            "internal_debug_metric": 123,
        },
    )
    routed_component = SimpleNamespace(
        info=_Info(
            {
                "electrical_metal_realization": {
                    "net_count": 12,
                    "pre_union_rect_count": 128,
                    "output_polygon_count": 12,
                    "raw_metal_area_um2": 549_599.152,
                    "union_metal_area_um2": 549_599.152,
                    "metal_area_overcount_um2": 0.0,
                    "rect_count_by_net": {"common_bus": 140},
                }
            }
        )
    )
    return SimpleNamespace(
        verification=verification,
        terminal_groups=tuple(range(11)),
        pad_plan=SimpleNamespace(assignments=tuple(range(12))),
        common_bus=SimpleNamespace(success=True),
        common_bus_escape=SimpleNamespace(success=True),
        individual_topology=SimpleNamespace(success=True),
        detailed_bundle_routes=SimpleNamespace(routes=tuple(range(11)), failed_routes=()),
        routed_component=routed_component,
    )


def test_electrical_benchmark_summary_is_compact_and_json_ready():
    module = _load_benchmark_electrical_module()

    summary = module.electrical_benchmark_summary(
        "case",
        ElectricalRoutingConfig(pad_side="top"),
        _result_stub(),
    )

    assert summary["benchmark"] == "case"
    assert summary["verification_success"] is True
    assert summary["verification_error_count"] == 0
    assert summary["detailed_route_count"] == 11
    assert summary["failed_detailed_route_count"] == 0
    assert summary["metrics"]["centerline_length_um"] == 19_390.0
    assert "internal_debug_metric" not in summary["metrics"]
    assert summary["realization_metrics"] == {
        "net_count": 12,
        "pre_union_rect_count": 128,
        "output_polygon_count": 12,
        "raw_metal_area_um2": 549_599.152,
        "union_metal_area_um2": 549_599.152,
        "metal_area_overcount_um2": 0.0,
    }
    json.dumps(summary, sort_keys=True)


def test_electrical_benchmark_guardrails_accept_current_shape():
    module = _load_benchmark_electrical_module()
    summary = module.electrical_benchmark_summary(
        "case",
        ElectricalRoutingConfig(pad_side="top"),
        _result_stub(),
    )

    assert module.guardrail_violations(summary) == []


def test_electrical_benchmark_uses_case_specific_guardrails():
    module = _load_benchmark_electrical_module()
    small_summary = module.electrical_benchmark_summary(
        "mmi_heater",
        ElectricalRoutingConfig(pad_side="top"),
        _result_stub(),
    )
    small_summary["detailed_route_count"] = 1
    small_summary["pad_assignment_count"] = 2
    small_summary["metrics"]["centerline_length_um"] = 1_090.0
    small_summary["metrics"]["bend_count"] = 3
    small_summary["metrics"]["pad_channel_height_um"] = 60.0
    small_summary["metrics"]["raw_metal_area_um2"] = 78_711.88
    small_summary["metrics"]["union_metal_area_um2"] = 77_692.42
    small_summary["metrics"]["metal_area_overcount_um2"] = 1_019.4600000000064
    small_summary["metrics"]["metal_area_overcount_ratio"] = 0.012951793299817084
    small_summary["metrics"]["metal_area_overcount_by_reason_um2"] = {
        "bus_stripe_contact": 135.2200000000003,
        "pad_contact": 543.2399999999984,
        "terminal_access_join": 341.0,
    }
    small_summary["metrics"]["metal_area_overcount_by_source_um2"] = {
        "bus_escape/bus_stripe": 35.22000000000001,
        "bus_escape/pad": 362.15999999999894,
        "bus_route/bus_stripe": 100.00000000000028,
        "bus_route/terminal_adapter": 170.5,
        "pad/route_tail": 181.07999999999947,
        "route_tail/terminal_adapter": 170.5,
    }
    small_summary["metrics"]["metal_redundant_area_overcount_um2"] = 0
    small_summary["metrics"]["metal_redundant_area_overcount_by_source_um2"] = {}
    small_summary["metrics"]["rect_count"] = 9
    small_summary["metrics"]["same_net_duplicate_rect_count"] = 0
    small_summary["metrics"]["same_net_overlap_pair_count"] = 6
    small_summary["metrics"]["same_net_overlap_pair_count_by_source"] = {
        "bus_escape/bus_stripe": 1,
        "bus_escape/pad": 1,
        "bus_route/bus_stripe": 1,
        "bus_route/terminal_adapter": 1,
        "pad/route_tail": 1,
        "route_tail/terminal_adapter": 1,
    }
    small_summary["metrics"]["same_net_intentional_overlap_pair_count"] = 6
    small_summary["metrics"]["same_net_intentional_overlap_pair_count_by_reason"] = {
        "bus_stripe_contact": 2,
        "pad_contact": 2,
        "terminal_access_join": 2,
    }
    small_summary["metrics"]["same_net_redundant_overlap_pair_count"] = 0
    small_summary["metrics"]["same_net_redundant_overlap_pair_count_by_source"] = {}
    small_summary["realization_metrics"]["output_polygon_count"] = 2
    small_summary["realization_metrics"]["pre_union_rect_count"] = 15
    small_summary["realization_metrics"]["raw_metal_area_um2"] = 77_692.42
    small_summary["realization_metrics"]["union_metal_area_um2"] = 77_692.42
    small_summary["realization_metrics"]["metal_area_overcount_um2"] = (
        0.0
    )

    assert module.guardrail_violations(
        small_summary,
        module.guardrails_for_benchmark("mmi_heater"),
    ) == []
    assert module.guardrail_violations(
        small_summary,
        module.guardrails_for_benchmark("mmi_heater_8x4_ripup_reroute"),
    )


def test_electrical_benchmark_guardrails_report_metric_regressions():
    module = _load_benchmark_electrical_module()
    summary = module.electrical_benchmark_summary(
        "case",
        ElectricalRoutingConfig(pad_side="top"),
        _result_stub(),
    )
    summary["verification_success"] = False
    summary["failed_detailed_route_count"] = 1
    summary["metrics"]["centerline_length_um"] = 25_000.0
    summary["metrics"]["metal_area_overcount_um2"] = 40_000.0
    summary["metrics"]["metal_area_overcount_ratio"] = 0.2
    summary["metrics"]["metal_redundant_area_overcount_um2"] = 1.0
    summary["metrics"]["same_net_overlap_pair_count"] = 3_000
    summary["metrics"]["same_net_redundant_overlap_pair_count"] = 1
    summary["metrics"]["cross_net_min_spacing_um"] = 5.0

    violations = module.guardrail_violations(summary)

    assert {
        violation["name"]
        for violation in violations
    } >= {
        "verification_success",
        "failed_detailed_route_count",
        "metrics.centerline_length_um",
        "metrics.metal_area_overcount_um2",
        "metrics.metal_area_overcount_ratio",
        "metrics.metal_redundant_area_overcount_um2",
        "metrics.same_net_overlap_pair_count",
        "metrics.same_net_redundant_overlap_pair_count",
        "metrics.cross_net_min_spacing_um",
    }


def test_electrical_benchmark_baseline_accepts_matching_summary():
    module = _load_benchmark_electrical_module()
    summary = module.electrical_benchmark_summary(
        "case",
        ElectricalRoutingConfig(pad_side="top"),
        _result_stub(),
    )

    assert module.baseline_violations(summary, summary) == []


def test_electrical_benchmark_baseline_reports_metric_drift():
    module = _load_benchmark_electrical_module()
    baseline = module.electrical_benchmark_summary(
        "case",
        ElectricalRoutingConfig(pad_side="top"),
        _result_stub(),
    )
    current = json.loads(json.dumps(baseline))
    current["metrics"]["centerline_length_um"] += 1.0
    current["realization_metrics"]["output_polygon_count"] += 1

    violations = module.baseline_violations(current, baseline)

    assert {
        violation["name"]
        for violation in violations
    } >= {
        "metrics.centerline_length_um",
        "realization_metrics.output_polygon_count",
    }
    assert all(violation["benchmark"] == "case" for violation in violations)


def test_electrical_benchmark_attaches_baseline_violations_to_rows():
    module = _load_benchmark_electrical_module()
    baseline = module.electrical_benchmark_summary(
        "case",
        ElectricalRoutingConfig(pad_side="top"),
        _result_stub(),
    )
    current = json.loads(json.dumps(baseline))
    current["detailed_route_count"] -= 1

    violations = module.attach_baseline_violations(current, baseline)

    assert violations
    assert current["baseline_violations"] == violations


def test_electrical_benchmark_main_writes_json(monkeypatch, tmp_path):
    module = _load_benchmark_electrical_module()
    output_path = tmp_path / "summary.json"
    expected = {
        "benchmark": "case",
        "guardrail_violations": [],
        "metrics": {"centerline_length_um": 1.0},
    }

    monkeypatch.setattr(module, "run_electrical_benchmark", lambda *_args, **_kwargs: expected)

    exit_code = module.main(["case", "--output", str(output_path), "--check"])

    assert exit_code == 0
    assert json.loads(output_path.read_text(encoding="utf-8")) == expected


def test_electrical_benchmark_suite_writes_json_list(monkeypatch, tmp_path):
    module = _load_benchmark_electrical_module()
    output_path = tmp_path / "suite.json"
    captured = {}
    rows = [
        {"benchmark": "mmi_heater", "guardrail_violations": []},
        {"benchmark": "mmi_heater_8x4", "guardrail_violations": []},
    ]

    def fake_run_electrical_benchmarks(benchmark_names, **kwargs):
        captured["benchmark_names"] = benchmark_names
        captured["artifacts_dir"] = kwargs["artifacts_dir"]
        return rows

    monkeypatch.setattr(module, "run_electrical_benchmarks", fake_run_electrical_benchmarks)

    exit_code = module.main(
        [
            "mmi_heater",
            "mmi_heater_8x4",
            "--output",
            str(output_path),
            "--check",
        ]
    )

    assert exit_code == 0
    assert captured["benchmark_names"] == ("mmi_heater", "mmi_heater_8x4")
    assert captured["artifacts_dir"] is None
    assert json.loads(output_path.read_text(encoding="utf-8")) == rows


def test_electrical_benchmark_main_fails_on_baseline_drift(monkeypatch, tmp_path):
    module = _load_benchmark_electrical_module()
    output_path = tmp_path / "summary.json"
    baseline_path = tmp_path / "baseline.json"
    baseline = {
        "benchmark": "case",
        "pad_side": "top",
        "detailed_route_count": 11,
        "guardrail_violations": [],
        "metrics": {"centerline_length_um": 10.0},
        "realization_metrics": {"output_polygon_count": 1},
    }
    current = json.loads(json.dumps(baseline))
    current["metrics"]["centerline_length_um"] = 11.0
    baseline_path.write_text(json.dumps(baseline), encoding="utf-8")

    monkeypatch.setattr(module, "run_electrical_benchmark", lambda *_args, **_kwargs: current)

    exit_code = module.main(
        [
            "case",
            "--compare-baseline",
            str(baseline_path),
            "--output",
            str(output_path),
            "--check",
        ]
    )

    assert exit_code == 1
    summary = json.loads(output_path.read_text(encoding="utf-8"))
    assert summary["baseline_violations"][0]["name"] == "metrics.centerline_length_um"


def test_electrical_benchmark_suite_check_fails_on_any_violation(monkeypatch, tmp_path):
    module = _load_benchmark_electrical_module()
    output_path = tmp_path / "suite.json"
    rows = [
        {"benchmark": "mmi_heater", "guardrail_violations": []},
        {
            "benchmark": "mmi_heater_8x4",
            "guardrail_violations": [{"name": "verification_success"}],
        },
    ]

    monkeypatch.setattr(
        module,
        "run_electrical_benchmarks",
        lambda *_args, **_kwargs: rows,
    )

    exit_code = module.main(["--suite", "--output", str(output_path), "--check"])

    assert exit_code == 1
    assert json.loads(output_path.read_text(encoding="utf-8")) == rows


def test_run_electrical_benchmarks_uses_artifact_subdirectories(monkeypatch, tmp_path):
    module = _load_benchmark_electrical_module()
    captured = []

    def fake_run_electrical_benchmark(benchmark_name, **kwargs):
        captured.append((benchmark_name, kwargs["artifacts_dir"]))
        return {"benchmark": benchmark_name, "guardrail_violations": []}

    monkeypatch.setattr(module, "run_electrical_benchmark", fake_run_electrical_benchmark)

    rows = module.run_electrical_benchmarks(
        ("a", "b"),
        config=ElectricalRoutingConfig(pad_side="top"),
        artifacts_dir=tmp_path,
    )

    assert rows == [
        {"benchmark": "a", "guardrail_violations": []},
        {"benchmark": "b", "guardrail_violations": []},
    ]
    assert captured == [
        ("a", tmp_path / "a"),
        ("b", tmp_path / "b"),
    ]
    assert json.loads((tmp_path / "electrical_suite_summary.json").read_text()) == rows


def test_electrical_benchmark_writes_artifact_bundle(tmp_path):
    module = _load_benchmark_electrical_module()
    artifacts_dir = tmp_path / "artifacts"
    svg_path = artifacts_dir / "electrical" / "case_common_bus.svg"
    metal_snapshot_path = artifacts_dir / "electrical" / "case_metal_snapshot.svg"
    svg_path.parent.mkdir(parents=True)
    svg_path.write_text("<svg />", encoding="utf-8")
    metal_snapshot_path.write_text("<svg />", encoding="utf-8")
    routed_component = _RoutedComponent()
    result = SimpleNamespace(
        debug_artifacts={
            "common_bus_svg": str(svg_path),
            "metal_snapshot_svg": str(metal_snapshot_path),
        },
        routed_component=routed_component,
    )
    summary = {
        "benchmark": "case",
        "pad_side": "top",
        "verification_success": True,
        "verification_error_count": 0,
        "verification_warning_count": 0,
        "detailed_route_count": 11,
        "failed_detailed_route_count": 0,
        "pad_assignment_count": 12,
        "guardrail_violations": [],
        "metrics": {
            "centerline_length_um": 19_390.0,
            "bend_count": 65,
            "pad_channel_height_um": 180.0,
            "raw_metal_area_um2": 556_778.772,
            "union_metal_area_um2": 549_599.152,
            "metal_area_overcount_um2": 7_179.619999999995,
            "metal_area_overcount_ratio": 0.0128949240902453,
            "rect_count": 102,
            "same_net_overlap_pair_count": 41,
            "same_net_redundant_overlap_pair_count": 0,
            "cross_net_min_spacing_um": 11.0,
        },
        "realization_metrics": {
            "output_polygon_count": 12,
            "pre_union_rect_count": 128,
            "raw_metal_area_um2": 549_599.152,
            "union_metal_area_um2": 549_599.152,
            "metal_area_overcount_um2": 0.0,
        },
    }

    artifacts = module.write_artifact_bundle(summary, result, artifacts_dir, "case")

    assert artifacts == {
        "common_bus_svg": str(svg_path),
        "gds": str(artifacts_dir / "case_electrical.gds"),
        "metal_snapshot_svg": str(metal_snapshot_path),
        "summary_json": str(artifacts_dir / "case_summary.json"),
        "summary_md": str(artifacts_dir / "case_summary.md"),
    }
    assert routed_component.gds_path == artifacts_dir / "case_electrical.gds"
    summary_json = json.loads((artifacts_dir / "case_summary.json").read_text(encoding="utf-8"))
    assert summary_json["artifacts"] == artifacts
    report = (artifacts_dir / "case_summary.md").read_text(encoding="utf-8")
    assert "Electrical Benchmark: case" in report
    assert "`centerline_length_um`" in report


def test_electrical_benchmark_main_passes_artifacts_dir(monkeypatch, tmp_path):
    module = _load_benchmark_electrical_module()
    captured = {}

    def fake_run_electrical_benchmark(*_args, **kwargs):
        captured["artifacts_dir"] = kwargs["artifacts_dir"]
        return {"benchmark": "case", "guardrail_violations": []}

    monkeypatch.setattr(module, "run_electrical_benchmark", fake_run_electrical_benchmark)

    exit_code = module.main(["case", "--artifacts-dir", str(tmp_path), "--check"])

    assert exit_code == 0
    assert captured["artifacts_dir"] == tmp_path


def test_electrical_benchmark_main_fails_when_checked_regression(monkeypatch, tmp_path):
    module = _load_benchmark_electrical_module()
    output_path = tmp_path / "summary.json"
    expected = {
        "benchmark": "case",
        "guardrail_violations": [{"name": "verification_success"}],
        "metrics": {},
    }

    monkeypatch.setattr(module, "run_electrical_benchmark", lambda *_args, **_kwargs: expected)

    exit_code = module.main(["case", "--output", str(output_path), "--check"])

    assert exit_code == 1
    assert json.loads(output_path.read_text(encoding="utf-8")) == expected
