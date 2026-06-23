#!/usr/bin/env python3
"""Run deterministic electrical-routing benchmarks and emit compact JSON."""

from __future__ import annotations

import argparse
from collections import Counter
from collections.abc import Mapping
import json
from pathlib import Path
import sys
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from routing_flow import load_benchmark
from translation.electrical import ElectricalRoutingConfig, route_electrical_heaters
from translation.layout_from_schematic import layout_from_schematic


DEFAULT_BENCHMARK = "mmi_heater_8x4_ripup_reroute"
DEFAULT_GUARDRAILS: Mapping[str, float | int | bool] = {
    "verification_success": True,
    "verification_error_count_max": 0,
    "verification_warning_count_max": 0,
    "failed_detailed_route_count_max": 0,
    "detailed_route_count_min": 11,
    "pad_assignment_count_min": 12,
    "pad_channel_height_um_max": 220.0,
    "centerline_length_um_max": 21_000.0,
    "bend_count_max": 75,
    "raw_metal_area_um2_max": 800_000.0,
    "rect_count_max": 400,
    "same_net_duplicate_rect_count_max": 80,
    "same_net_overlap_pair_count_max": 2_000,
    "output_polygon_count_max": 20,
    "pre_union_rect_count_max": 450,
}


def run_electrical_benchmark(
    benchmark_name: str = DEFAULT_BENCHMARK,
    *,
    config: ElectricalRoutingConfig | None = None,
    artifacts_dir: Path | None = None,
) -> dict[str, Any]:
    """Run one electrical benchmark and return a stable JSON-compatible row."""

    config = config or ElectricalRoutingConfig()
    schematic = load_benchmark(benchmark_name)
    component = layout_from_schematic(schematic)
    result = route_electrical_heaters(
        component,
        schematic,
        config,
        debug_dir=artifacts_dir,
        debug_prefix=benchmark_name,
    )
    summary = electrical_benchmark_summary(benchmark_name, config, result)
    summary["guardrail_violations"] = guardrail_violations(summary)
    if artifacts_dir is not None:
        summary["artifacts"] = write_artifact_bundle(
            summary,
            result,
            artifacts_dir,
            benchmark_name,
        )
    return summary


def electrical_benchmark_summary(
    benchmark_name: str,
    config: ElectricalRoutingConfig,
    result: object,
) -> dict[str, Any]:
    verification = getattr(result, "verification", None)
    detailed_routes = getattr(result, "detailed_bundle_routes", None)
    pad_plan = getattr(result, "pad_plan", None)
    routed_component = getattr(result, "routed_component", None)
    metrics = _as_dict(getattr(verification, "metrics", {}))
    realization_metrics = _realization_metrics(routed_component)
    issues = tuple(getattr(verification, "issues", ()) or ())
    issue_counts = Counter(getattr(issue, "code", "unknown") for issue in issues)

    return {
        "benchmark": benchmark_name,
        "pad_side": config.pad_side,
        "verification_success": bool(getattr(verification, "success", False)),
        "verification_error_count": int(getattr(verification, "error_count", 0)),
        "verification_warning_count": int(getattr(verification, "warning_count", 0)),
        "verification_issue_counts": dict(sorted(issue_counts.items())),
        "terminal_group_count": len(getattr(result, "terminal_groups", ()) or ()),
        "pad_assignment_count": len(getattr(pad_plan, "assignments", ()) or ()),
        "common_bus_success": bool(getattr(getattr(result, "common_bus", None), "success", False)),
        "common_bus_escape_success": bool(
            getattr(getattr(result, "common_bus_escape", None), "success", False)
        ),
        "individual_topology_success": bool(
            getattr(getattr(result, "individual_topology", None), "success", False)
        ),
        "detailed_route_count": len(getattr(detailed_routes, "routes", ()) or ()),
        "failed_detailed_route_count": len(
            getattr(detailed_routes, "failed_routes", ()) or ()
        ),
        "metrics": _selected_metrics(metrics),
        "realization_metrics": _selected_realization_metrics(realization_metrics),
    }


def guardrail_violations(
    summary: Mapping[str, Any],
    guardrails: Mapping[str, float | int | bool] = DEFAULT_GUARDRAILS,
) -> list[dict[str, Any]]:
    violations: list[dict[str, Any]] = []
    _check_equal(violations, summary, "verification_success", guardrails)
    _check_max(violations, summary, "verification_error_count", guardrails)
    _check_max(violations, summary, "verification_warning_count", guardrails)
    _check_max(violations, summary, "failed_detailed_route_count", guardrails)
    _check_min(violations, summary, "detailed_route_count", guardrails)
    _check_min(violations, summary, "pad_assignment_count", guardrails)
    _check_metric_max(violations, summary, "pad_channel_height_um", guardrails)
    _check_metric_max(violations, summary, "centerline_length_um", guardrails)
    _check_metric_max(violations, summary, "bend_count", guardrails)
    _check_metric_max(violations, summary, "raw_metal_area_um2", guardrails)
    _check_metric_max(violations, summary, "rect_count", guardrails)
    _check_metric_max(violations, summary, "same_net_duplicate_rect_count", guardrails)
    _check_metric_max(violations, summary, "same_net_overlap_pair_count", guardrails)
    _check_realization_max(violations, summary, "output_polygon_count", guardrails)
    _check_realization_max(violations, summary, "pre_union_rect_count", guardrails)
    _check_cross_net_spacing(violations, summary)
    return violations


def _check_equal(
    violations: list[dict[str, Any]],
    summary: Mapping[str, Any],
    key: str,
    guardrails: Mapping[str, float | int | bool],
) -> None:
    expected = guardrails.get(key)
    actual = summary.get(key)
    if expected is not None and actual != expected:
        violations.append(
            {"name": key, "actual": actual, "expected": expected, "operator": "=="}
        )


def _check_min(
    violations: list[dict[str, Any]],
    summary: Mapping[str, Any],
    key: str,
    guardrails: Mapping[str, float | int | bool],
) -> None:
    limit = guardrails.get(f"{key}_min")
    actual = summary.get(key)
    if isinstance(limit, (int, float)) and _number(actual) < float(limit):
        violations.append(
            {"name": key, "actual": actual, "expected": limit, "operator": ">="}
        )


def _check_max(
    violations: list[dict[str, Any]],
    summary: Mapping[str, Any],
    key: str,
    guardrails: Mapping[str, float | int | bool],
) -> None:
    limit = guardrails.get(f"{key}_max")
    actual = summary.get(key)
    if isinstance(limit, (int, float)) and _number(actual) > float(limit):
        violations.append(
            {"name": key, "actual": actual, "expected": limit, "operator": "<="}
        )


def _check_metric_max(
    violations: list[dict[str, Any]],
    summary: Mapping[str, Any],
    key: str,
    guardrails: Mapping[str, float | int | bool],
) -> None:
    _check_nested_max(violations, summary, "metrics", key, guardrails)


def _check_realization_max(
    violations: list[dict[str, Any]],
    summary: Mapping[str, Any],
    key: str,
    guardrails: Mapping[str, float | int | bool],
) -> None:
    _check_nested_max(violations, summary, "realization_metrics", key, guardrails)


def _check_nested_max(
    violations: list[dict[str, Any]],
    summary: Mapping[str, Any],
    section: str,
    key: str,
    guardrails: Mapping[str, float | int | bool],
) -> None:
    limit = guardrails.get(f"{key}_max")
    values = summary.get(section, {})
    actual = values.get(key) if isinstance(values, Mapping) else None
    if isinstance(limit, (int, float)) and _number(actual) > float(limit):
        violations.append(
            {
                "name": f"{section}.{key}",
                "actual": actual,
                "expected": limit,
                "operator": "<=",
            }
        )


def _check_cross_net_spacing(
    violations: list[dict[str, Any]],
    summary: Mapping[str, Any],
) -> None:
    metrics = summary.get("metrics", {})
    if not isinstance(metrics, Mapping):
        return
    actual = _number(metrics.get("cross_net_min_spacing_um"))
    required = _number(metrics.get("required_cross_net_clearance_um"))
    if actual < required:
        violations.append(
            {
                "name": "metrics.cross_net_min_spacing_um",
                "actual": actual,
                "expected": required,
                "operator": ">=",
            }
        )


def _selected_metrics(metrics: Mapping[str, Any]) -> dict[str, Any]:
    keys = (
        "net_count",
        "rect_count",
        "raw_metal_area_um2",
        "same_net_duplicate_rect_count",
        "same_net_overlap_pair_count",
        "cross_net_min_spacing_um",
        "required_cross_net_clearance_um",
        "centerline_length_um",
        "bend_count",
        "pad_channel_height_um",
    )
    return {key: metrics.get(key) for key in keys if key in metrics}


def _selected_realization_metrics(metrics: Mapping[str, Any]) -> dict[str, Any]:
    keys = (
        "net_count",
        "pre_union_rect_count",
        "output_polygon_count",
    )
    return {key: metrics.get(key) for key in keys if key in metrics}


def _realization_metrics(routed_component: object) -> dict[str, Any]:
    info = getattr(routed_component, "info", {})
    metrics = _get_value(info, "electrical_metal_realization", {})
    return _as_dict(metrics)


def _as_dict(value: object) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        return {}
    return dict(value)


def _get_value(value: object, key: str, default: object) -> object:
    getter = getattr(value, "get", None)
    if callable(getter):
        return getter(key, default)
    return default


def _number(value: object) -> float:
    if isinstance(value, bool):
        return 0.0
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return 0.0
    return 0.0


def _write_json(summary: Mapping[str, Any], output_path: Path | None) -> None:
    text = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if output_path is None:
        print(text, end="")
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(text, encoding="utf-8")


def write_artifact_bundle(
    summary: Mapping[str, Any],
    result: object,
    artifacts_dir: Path,
    benchmark_name: str,
) -> dict[str, str]:
    artifacts_dir.mkdir(parents=True, exist_ok=True)
    summary_path = artifacts_dir / f"{benchmark_name}_summary.json"
    report_path = artifacts_dir / f"{benchmark_name}_summary.md"
    artifact_paths: dict[str, str] = {
        "summary_json": str(summary_path),
        "summary_md": str(report_path),
    }

    debug_artifacts = _as_dict(getattr(result, "debug_artifacts", {}))
    common_bus_svg = debug_artifacts.get("common_bus_svg")
    if isinstance(common_bus_svg, str):
        artifact_paths["common_bus_svg"] = common_bus_svg

    routed_component = getattr(result, "routed_component", None)
    if routed_component is not None:
        gds_path = artifacts_dir / f"{benchmark_name}_electrical.gds"
        writer = getattr(routed_component, "write_gds", None)
        if callable(writer):
            writer(gds_path)
            artifact_paths["gds"] = str(gds_path)

    summary_with_artifacts = dict(summary)
    summary_with_artifacts["artifacts"] = dict(sorted(artifact_paths.items()))
    _write_json(summary_with_artifacts, summary_path)
    report_path.write_text(_markdown_report(summary_with_artifacts), encoding="utf-8")
    return artifact_paths


def _markdown_report(summary: Mapping[str, Any]) -> str:
    metrics = summary.get("metrics", {})
    realization = summary.get("realization_metrics", {})
    violations = summary.get("guardrail_violations", [])
    metric_rows = [
        ("verification_success", summary.get("verification_success")),
        ("verification_error_count", summary.get("verification_error_count")),
        ("verification_warning_count", summary.get("verification_warning_count")),
        ("detailed_route_count", summary.get("detailed_route_count")),
        ("failed_detailed_route_count", summary.get("failed_detailed_route_count")),
        ("pad_assignment_count", summary.get("pad_assignment_count")),
        ("centerline_length_um", _mapping_get(metrics, "centerline_length_um")),
        ("bend_count", _mapping_get(metrics, "bend_count")),
        ("pad_channel_height_um", _mapping_get(metrics, "pad_channel_height_um")),
        ("raw_metal_area_um2", _mapping_get(metrics, "raw_metal_area_um2")),
        ("rect_count", _mapping_get(metrics, "rect_count")),
        ("same_net_overlap_pair_count", _mapping_get(metrics, "same_net_overlap_pair_count")),
        ("cross_net_min_spacing_um", _mapping_get(metrics, "cross_net_min_spacing_um")),
        ("output_polygon_count", _mapping_get(realization, "output_polygon_count")),
        ("pre_union_rect_count", _mapping_get(realization, "pre_union_rect_count")),
    ]
    lines = [
        f"# Electrical Benchmark: {summary.get('benchmark', '')}",
        "",
        f"- Pad side: `{summary.get('pad_side', '')}`",
        f"- Guardrail violations: `{len(violations) if isinstance(violations, list) else 0}`",
        "",
        "| Metric | Value |",
        "|---|---:|",
    ]
    lines.extend(f"| `{name}` | `{value}` |" for name, value in metric_rows)
    if isinstance(violations, list) and violations:
        lines.extend(["", "## Guardrail Violations", ""])
        for violation in violations:
            if not isinstance(violation, Mapping):
                continue
            lines.append(
                "- "
                f"`{violation.get('name')}` "
                f"{violation.get('operator')} "
                f"`{violation.get('expected')}` "
                f"(actual `{violation.get('actual')}`)"
            )
    return "\n".join(lines) + "\n"


def _mapping_get(value: object, key: str) -> object:
    if isinstance(value, Mapping):
        return value.get(key)
    return None


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an electrical-routing benchmark and emit JSON metrics."
    )
    parser.add_argument(
        "benchmark",
        nargs="?",
        default=DEFAULT_BENCHMARK,
        help=f"Benchmark module name from benchmarks/ (default: {DEFAULT_BENCHMARK}).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional JSON output path. Defaults to stdout.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit non-zero if built-in guardrails are violated.",
    )
    parser.add_argument(
        "--pad-side",
        choices=("top", "bottom"),
        default="top",
        help="Electrical pad side for the benchmark.",
    )
    parser.add_argument(
        "--artifacts-dir",
        type=Path,
        default=None,
        help="Optional directory for summary JSON, Markdown, debug SVG, and GDS artifacts.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    summary = run_electrical_benchmark(
        args.benchmark,
        config=ElectricalRoutingConfig(pad_side=args.pad_side),
        artifacts_dir=args.artifacts_dir,
    )
    _write_json(summary, args.output)
    if args.check and summary["guardrail_violations"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
