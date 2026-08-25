"""Electrical heater-routing stage for routing_flow."""

from collections import Counter
from pathlib import Path
import time
from typing import Any, cast

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from routing_flow_component_info import component_info, copy_component_info
from translation.electrical import (
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
    ElectricalVerificationIssue,
    ElectricalVerificationResult,
    route_electrical_heaters,
)


def run_electrical_routing_step(
    *,
    benchmark_name: str,
    schematic: Schematic,
    routed_layout: Component,
    electrical_config: ElectricalRoutingConfig | None,
    debug_dir: Path | None,
    total_steps: int,
    stats: object | None,
    debug_timing: bool,
    debug_svgs_enabled: bool,
) -> tuple[Component, ElectricalRoutingResult]:
    """Run heater metal routing and return the electrically routed layout."""
    print(f"\n[4/{total_steps}] Routing heater electrical metal...")
    t_electrical_start = time.perf_counter()
    current_electrical_result = route_electrical_heaters(
        routed_layout,
        schematic,
        electrical_config,
        debug_dir=debug_dir,
        debug_prefix=benchmark_name.lower(),
    )
    t_electrical_end = time.perf_counter()
    if stats is not None:
        stats.step_times_s["electrical_routing"] = t_electrical_end - t_electrical_start
    if current_electrical_result.routed_component is None:
        raise RuntimeError(_electrical_failure_summary(current_electrical_result))
    electrical_summary = electrical_summary_from_result(
        current_electrical_result,
        electrical_config,
    )
    if stats is not None:
        stats.electrical_terminal_groups = int(electrical_summary["terminal_group_count"])
        stats.electrical_pad_assignments = int(electrical_summary["pad_assignment_count"])
        stats.electrical_detailed_routes = int(electrical_summary["detailed_route_count"])
        stats.electrical_failed_detailed_routes = int(
            electrical_summary["failed_detailed_route_count"]
        )
    electrical_layout = current_electrical_result.routed_component
    copy_component_info(routed_layout, electrical_layout)
    component_info(electrical_layout)["electrical_routing"] = electrical_summary
    electrical_pad_count = (
        len(current_electrical_result.pad_plan.assignments)
        if current_electrical_result.pad_plan
        else 0
    )
    if current_electrical_result.terminal_groups:
        print(f"      \u2713 Electrical layout generated: {electrical_layout.name}")
    else:
        print("      \u2713 No heater electrical terminals found; electrical routing skipped")
    print(
        "      - Electrical routes: "
        f"heaters={len(current_electrical_result.terminal_groups)}, "
        f"pads={electrical_pad_count}"
    )
    if debug_timing:
        print(f"      - Electrical routing time: {t_electrical_end - t_electrical_start:.4f} s")
    if debug_svgs_enabled:
        for name, path in current_electrical_result.debug_artifacts.items():
            print(f"      - Electrical {name}: {path}")
    return electrical_layout, current_electrical_result


def electrical_config_summary(
    config: ElectricalRoutingConfig | None,
) -> dict[str, Any]:
    if config is None:
        config = ElectricalRoutingConfig()
    keys = (
        "pad_side",
        "bus_side",
        "routing_grid_pitch_um",
        "obstacle_clearance_um",
        "wire_width_um",
        "bus_width_um",
        "terminal_contact_width_um",
        "pad_pitch_um",
        "bondpad_width_um",
        "common_bus_bondpad_width_um",
        "common_bus_bondpad_length_um",
        "bondpad_length_um",
        "pad_offset_um",
        "pad_access_depth_um",
        "common_bus_pad_position",
        "individual_route_spacing_um",
        "obstacle_mode",
        "clearance_metric",
        "metal_layer",
        "pad_marker_layer",
        "heater_layers",
        "metal_obstacle_layers",
    )
    summary: dict[str, Any] = {}
    for key in keys:
        value = getattr(config, key, None)
        if value is None:
            continue
        if isinstance(value, tuple):
            summary[key] = tuple(value)
            continue
        summary[key] = value
    return summary


def electrical_summary_from_result(
    result: ElectricalRoutingResult,
    config: ElectricalRoutingConfig | None = None,
) -> dict[str, Any]:
    detailed_routes = result.detailed_bundle_routes
    failed_detailed_routes = (
        tuple(detailed_routes.failed_routes) if detailed_routes is not None else ()
    )
    verification = cast(
        ElectricalVerificationResult | None,
        getattr(result, "verification", None),
    )
    verification_issues: tuple[ElectricalVerificationIssue, ...] = (
        verification.issues if verification is not None else ()
    )
    issue_counts = Counter(issue.code for issue in verification_issues)
    realization_metrics = (
        dict(result.routed_component.info.get("electrical_metal_realization", {}))
        if result.routed_component is not None
        else {}
    )
    debug_artifacts = dict(result.debug_artifacts)
    return {
        "config": electrical_config_summary(config),
        "terminal_group_count": len(result.terminal_groups),
        "common_bus_success": result.common_bus.success,
        "failed_heaters": tuple(result.common_bus.failed_heaters),
        "pad_assignment_count": (
            len(result.pad_plan.assignments) if result.pad_plan is not None else 0
        ),
        "common_bus_escape_success": (
            result.common_bus_escape.success if result.common_bus_escape is not None else None
        ),
        "detailed_route_count": (len(detailed_routes.routes) if detailed_routes is not None else 0),
        "failed_detailed_route_count": len(failed_detailed_routes),
        "failed_detailed_routes": tuple(
            {
                "terminal_id": route.terminal.id,
                "reason": route.reason,
            }
            for route in failed_detailed_routes
        ),
        "verification_success": verification.success if verification is not None else None,
        "verification_error_count": verification.error_count if verification is not None else 0,
        "verification_warning_count": (
            verification.warning_count if verification is not None else 0
        ),
        "verification_issue_counts": dict(sorted(issue_counts.items())),
        "verification_metrics": (dict(verification.metrics) if verification is not None else {}),
        "realization_metrics": realization_metrics,
        "verification_issues": (
            tuple(
                {
                    "code": issue.code,
                    "message": issue.message,
                    "severity": issue.severity,
                    "net_id": issue.net_id,
                    "details": dict(issue.details),
                }
                for issue in verification_issues
            )
            if verification is not None
            else ()
        ),
        "debug_artifacts": debug_artifacts,
        "debug_artifact_count": len(debug_artifacts),
    }


def _electrical_failure_summary(result: ElectricalRoutingResult) -> str:
    summary = electrical_summary_from_result(result)
    return (
        "Electrical routing failed to produce a routed component: "
        f"failed_heaters={summary['failed_heaters']}, "
        f"failed_detailed_route_count={summary['failed_detailed_route_count']}"
    )
