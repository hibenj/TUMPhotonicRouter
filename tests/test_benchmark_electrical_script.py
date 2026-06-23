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
            "rect_count": 312,
            "raw_metal_area_um2": 734_180.942,
            "same_net_duplicate_rect_count": 45,
            "same_net_overlap_pair_count": 1_109,
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
                    "pre_union_rect_count": 312,
                    "output_polygon_count": 12,
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
        "pre_union_rect_count": 312,
        "output_polygon_count": 12,
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
    summary["metrics"]["same_net_overlap_pair_count"] = 3_000
    summary["metrics"]["cross_net_min_spacing_um"] = 5.0

    violations = module.guardrail_violations(summary)

    assert {
        violation["name"]
        for violation in violations
    } >= {
        "verification_success",
        "failed_detailed_route_count",
        "metrics.centerline_length_um",
        "metrics.same_net_overlap_pair_count",
        "metrics.cross_net_min_spacing_um",
    }


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


def test_electrical_benchmark_writes_artifact_bundle(tmp_path):
    module = _load_benchmark_electrical_module()
    artifacts_dir = tmp_path / "artifacts"
    svg_path = artifacts_dir / "electrical" / "case_common_bus.svg"
    svg_path.parent.mkdir(parents=True)
    svg_path.write_text("<svg />", encoding="utf-8")
    routed_component = _RoutedComponent()
    result = SimpleNamespace(
        debug_artifacts={"common_bus_svg": str(svg_path)},
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
            "raw_metal_area_um2": 734_180.942,
            "rect_count": 312,
            "same_net_overlap_pair_count": 1_109,
            "cross_net_min_spacing_um": 11.0,
        },
        "realization_metrics": {
            "output_polygon_count": 12,
            "pre_union_rect_count": 312,
        },
    }

    artifacts = module.write_artifact_bundle(summary, result, artifacts_dir, "case")

    assert artifacts == {
        "common_bus_svg": str(svg_path),
        "gds": str(artifacts_dir / "case_electrical.gds"),
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
