#!/usr/bin/env python3
"""Run repeatable photonic routing benchmarks and print a compact table."""

from __future__ import annotations

import argparse
import csv
from collections.abc import Mapping
from datetime import datetime
import json
import statistics
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any, Iterable, Literal, Protocol, TypeAlias, TypeGuard, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow import (
    SCRIPT_BEND_RADIUS_UM,
    SCRIPT_CHIP_ADD_X_UM,
    SCRIPT_CHIP_ADD_Y_UM,
    SCRIPT_GRID_SIZE_UM,
    SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    SCRIPT_RIPUP_HISTORY_INCREMENT,
    SCRIPT_RIPUP_HISTORY_WEIGHT,
    SCRIPT_RIPUP_MAX_ROUNDS,
    SCRIPT_RIPUP_MAX_VICTIMS,
    RipupRerouteConfig,
    RoutingFlowStats,
    run_routing_flow,
)


DEFAULT_BENCHMARKS = ("TOY", "mmi_heater", "mmi_heater_8x4_ripup_reroute")
DEFAULT_PERF_BASELINE_PATH = (
    PROJECT_ROOT / "tests" / "baselines" / "photonic_perf_baseline.json"
)
DEFAULT_PERF_METRIC = "route_nets_s"
DEFAULT_PERF_RELATIVE_TOLERANCE = 0.10
DEFAULT_PERF_ABSOLUTE_TOLERANCE_S = 0.05
DEFAULT_PERF_COUNTER_RELATIVE_TOLERANCE = 0.10
WORKER_MARKER = "PHOTONIC_BENCHMARK_JSON:"
ROUTE_NETS_TIMING_KEYS = (
    "obstacle_map",
    "router_setup",
    "route_job_build",
    "port_opening_prep",
    "port_opening_batch",
    "static_map_handoff",
    "state_opening_precompute",
    "clearance_exempt_batch",
    "batch_job_pack",
    "native_route_batch",
    "native_batch_route_job_unpack",
    "native_batch_obstacle_map_prepare",
    "native_batch_route_search_total",
    "native_batch_simple_route_candidate",
    "native_batch_dense_astar",
    "native_batch_commit_cell_build",
    "native_batch_commit_update_dynamic_map",
    "native_batch_normal_route_wall",
    "native_batch_probe_route_wall",
    "native_batch_repair_failed_net_wall",
    "native_batch_reroute_victims_wall",
    "native_batch_normal_route_failed_wall",
    "native_batch_probe_route_failed_wall",
    "native_batch_repair_failed_net_failed_wall",
    "native_batch_reroute_victims_failed_wall",
    "native_batch_repair_probe_victim_selection",
    "native_batch_repair_state_reset",
    "native_batch_ripup",
    "native_batch_history_update",
    "native_batch_route_result_construction",
    "native_batch_python_return_dict",
    "batch_result_processing",
    "endpoint_correction_pack",
    "endpoint_correction_native",
    "endpoint_correction_processing",
    "record_assembly",
    "direct_realization",
    "debug_artifact_assembly",
)
AttemptColumn: TypeAlias = Literal[
    "benchmark",
    "attempt_index",
    "bucket_name",
    "net_id",
    "route_index",
    "net_name",
    "source",
    "target",
    "elapsed_s",
    "failed",
    "repair_round",
    "used_simple_route",
    "expanded_states",
    "generated_neighbors",
    "heap_pushes",
    "heap_pops",
    "skipped_duplicate_heap_entries",
    "stale_generation_heap_entries",
    "closed_heap_entries",
    "max_heap_size",
    "dense_search_states",
    "dense_search_storage_bytes",
    "best_cost_updates",
    "parent_updates",
    "obstacle_clearance_checks",
    "window_attempts",
    "last_window_min_x",
    "last_window_max_x",
    "last_window_min_y",
    "last_window_max_y",
    "last_window_area_cells",
    "primitive_generated_by_class",
    "primitive_bounds_rejects_by_class",
    "primitive_closed_rejects_by_class",
    "primitive_cost_pruned_by_class",
    "primitive_footprint_checks_by_class",
    "primitive_footprint_rejects_by_class",
    "primitive_accepted_by_class",
    "footprint_rect_checks",
    "dense_grid_build_time_s",
    "dense_grid_cells",
    "used_full_grid_fallback",
    "diagnostics",
    "error",
]
ATTEMPT_COLUMNS: tuple[AttemptColumn, ...] = (
    "benchmark",
    "attempt_index",
    "bucket_name",
    "net_id",
    "route_index",
    "net_name",
    "source",
    "target",
    "elapsed_s",
    "failed",
    "repair_round",
    "used_simple_route",
    "expanded_states",
    "generated_neighbors",
    "heap_pushes",
    "heap_pops",
    "skipped_duplicate_heap_entries",
    "stale_generation_heap_entries",
    "closed_heap_entries",
    "max_heap_size",
    "dense_search_states",
    "dense_search_storage_bytes",
    "best_cost_updates",
    "parent_updates",
    "obstacle_clearance_checks",
    "window_attempts",
    "last_window_min_x",
    "last_window_max_x",
    "last_window_min_y",
    "last_window_max_y",
    "last_window_area_cells",
    "primitive_generated_by_class",
    "primitive_bounds_rejects_by_class",
    "primitive_closed_rejects_by_class",
    "primitive_cost_pruned_by_class",
    "primitive_footprint_checks_by_class",
    "primitive_footprint_rejects_by_class",
    "primitive_accepted_by_class",
    "footprint_rect_checks",
    "dense_grid_build_time_s",
    "dense_grid_cells",
    "used_full_grid_fallback",
    "diagnostics",
    "error",
)


class SupportsGet(Protocol):
    def get(self, key: str, default: object = None) -> object: ...


class EmptyInfo:
    def get(self, key: str, default: object = None) -> object:
        return default


def _supports_get(value: object) -> TypeGuard[SupportsGet]:
    return hasattr(value, "get")


EMPTY_INFO: SupportsGet = EmptyInfo()


def _git_rev() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=PROJECT_ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return "unknown"


def _format_seconds(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.4f}"


def _format_int(value: object) -> str:
    if value is None:
        return ""
    if not isinstance(value, (str, bytes, bytearray, int, float)):
        return ""
    return f"{int(value):,}"


def _format_mib(value: object) -> str:
    if value is None:
        return ""
    if not isinstance(value, (str, bytes, bytearray, int, float)):
        return ""
    return f"{float(value) / (1024.0 * 1024.0):.2f}"


def _format_primitive_counter(value: object) -> str:
    if not isinstance(value, Mapping):
        return ""
    labels = (
        ("straight_short", "s"),
        ("straight_long", "l"),
        ("bend45", "b45"),
        ("bend90", "b90"),
    )
    parts = []
    for key, label in labels:
        raw = value.get(key, 0)
        count = int(raw) if isinstance(raw, (str, bytes, bytearray, int, float)) else 0
        parts.append(f"{label}:{count:,}")
    return " ".join(parts)


def _row_seconds(row: dict[str, object], key: str) -> float | None:
    return cast(float | None, row[key])


def _route_attempt_records(row: dict[str, object]) -> list[dict[str, object]]:
    records = row.get("route_attempt_records", [])
    if not isinstance(records, list):
        return []
    return [record for record in records if isinstance(record, dict)]


def _attempt_seconds(record: dict[str, object]) -> float:
    value = record.get("elapsed_s", 0.0)
    return float(value) if isinstance(value, (int, float, str)) else 0.0


def _record_seconds(record: dict[str, object], key: str) -> float:
    value = record.get(key, 0.0)
    return float(value) if isinstance(value, (int, float, str)) else 0.0


def _record_diagnostics(record: dict[str, object]) -> dict[str, object]:
    diagnostics = record.get("diagnostics", {})
    return diagnostics if isinstance(diagnostics, dict) else {}


def _diagnostic_float(
    diagnostics: dict[str, object],
    key: str,
) -> float | None:
    value = diagnostics.get(key)
    if isinstance(value, (int, float, str)) and not isinstance(value, bool):
        return float(value)
    return None


def _format_ratio(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.3f}"


def _format_percent(value: float | None) -> str:
    if value is None:
        return ""
    return f"{100.0 * value:.2f}%"


def _format_status_counts(value: object) -> str:
    if not isinstance(value, Mapping):
        return ""
    parts = []
    for key in sorted(value):
        count = value.get(key, 0)
        if isinstance(count, (str, bytes, bytearray, int, float)):
            parts.append(f"{key}:{int(count):,}")
    return " ".join(parts)


def _format_candidate_profile(value: object) -> str:
    if not isinstance(value, Mapping):
        return ""
    parts = []
    for reason, raw_profile in sorted(
        value.items(),
        key=lambda item: (
            -_numeric_float(item[1].get("elapsed_s"))
            if isinstance(item[1], Mapping)
            else 0.0
        ),
    ):
        if not isinstance(raw_profile, Mapping):
            continue
        parts.append(
            "{reason}:calls={calls},elapsed={elapsed:.4f}s,planned={planned},miss={miss}".format(
                reason=reason,
                calls=_numeric_int(raw_profile.get("edge_calls")),
                elapsed=_numeric_float(raw_profile.get("elapsed_s")),
                planned=_numeric_int(raw_profile.get("planned")),
                miss=_numeric_int(raw_profile.get("no_candidate")),
            )
        )
    return "; ".join(parts)


def _numeric_float(value: object, default: float = 0.0) -> float:
    if isinstance(value, bool):
        return default
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return default
    return default


def _numeric_int(value: object, default: int = 0) -> int:
    if isinstance(value, bool):
        return default
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    if isinstance(value, str):
        try:
            return int(float(value))
        except ValueError:
            return default
    return default


def _sum_meander_result_int(results: Iterable[object], key: str) -> int:
    total = 0
    for result in results:
        if isinstance(result, Mapping):
            total += _numeric_int(result.get(key))
    return total


def _max_meander_result_int(results: Iterable[object], key: str) -> int:
    maximum = 0
    for result in results:
        if isinstance(result, Mapping):
            maximum = max(maximum, _numeric_int(result.get(key)))
    return maximum


def _slowest_meander_result(results: Iterable[object]) -> dict[str, object]:
    slowest: dict[str, object] = {}
    slowest_elapsed_s = 0.0
    for result in results:
        if not isinstance(result, Mapping):
            continue
        elapsed_s = _numeric_float(result.get("planning_elapsed_s"))
        if not slowest or elapsed_s > slowest_elapsed_s:
            slowest = dict(result)
            slowest_elapsed_s = elapsed_s
    return slowest


def _path_length_group_diagnostics(layout_info: object) -> list[Mapping[str, object]]:
    if not _supports_get(layout_info):
        return []
    analysis = layout_info.get("path_length_analysis", {})
    if not _supports_get(analysis):
        return []
    groups = analysis.get("matching_group_diagnostics")
    if not isinstance(groups, list):
        groups = analysis.get("matching_groups")
    if not isinstance(groups, list):
        return []
    return [group for group in groups if isinstance(group, Mapping)]


def _path_length_analysis_info(layout_info: object) -> SupportsGet:
    if not _supports_get(layout_info):
        return EMPTY_INFO
    analysis = layout_info.get("path_length_analysis", {})
    if not _supports_get(analysis):
        return EMPTY_INFO
    return analysis


def _max_group_float(groups: Iterable[Mapping[str, object]], key: str) -> float:
    maximum = 0.0
    for group in groups:
        maximum = max(maximum, _numeric_float(group.get(key)))
    return maximum


def _count_groups_with_requirements(groups: Iterable[Mapping[str, object]]) -> int:
    count = 0
    for group in groups:
        if _numeric_int(group.get("edges_requiring_meander")) > 0:
            count += 1
    return count


def _count_groups_over_tolerance(groups: Iterable[Mapping[str, object]]) -> int:
    count = 0
    for group in groups:
        within = group.get("within_tolerance")
        if within is False:
            count += 1
    return count


def _count_lifted_groups(groups: Iterable[Mapping[str, object]]) -> int:
    count = 0
    for group in groups:
        if _numeric_float(group.get("target_lift_um")) > 0.0:
            count += 1
    return count


def _list_length(value: object) -> int:
    return len(value) if isinstance(value, list) else 0


def _flatten_attempt_records(rows: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    flattened: list[dict[str, object]] = []
    for row in rows:
        benchmark = str(row.get("benchmark", ""))
        for record in _route_attempt_records(row):
            flattened.append({"benchmark": benchmark, **record})
    return flattened


def _write_attempt_output(rows: Iterable[dict[str, object]], output_path: Path) -> None:
    records = _flatten_attempt_records(rows)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.suffix.lower() == ".csv":
        with output_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=list(ATTEMPT_COLUMNS),
                extrasaction="ignore",
            )
            writer.writeheader()
            writer.writerows(cast(Iterable[Mapping[AttemptColumn, Any]], cast(object, records)))
        return
    output_path.write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _run_single_benchmark(benchmark: str, args: argparse.Namespace) -> dict[str, object]:
    stats = RoutingFlowStats()
    routed_layout = run_routing_flow(
        benchmark,
        debug_svgs=False,
        debug_timing=False,
        show_klayout=False,
        enable_path_length_matching=args.path_length_matching,
        path_length_match_outputs=args.path_length_match_outputs,
        allow_45_degree_turns=args.allow_45_degree_turns,
        bend_radius_um=args.bend_radius_um,
        use_indexed_heap=args.use_indexed_heap,
        enable_simple_routes=args.enable_simple_routes,
        primitive_ordering=args.primitive_ordering,
        heuristic_mode=args.heuristic_mode,
        heap_tie_breaker=args.heap_tie_breaker,
        max_iterations=args.max_iterations,
        routing_window_scale=args.routing_window_scale,
        include_heater_obstacles=args.include_heater_obstacles,
        grid_size_um=args.grid_size_um,
        chip_add_x_um=args.chip_add_x_um,
        chip_add_y_um=args.chip_add_y_um,
        path_length_meander_height_um=args.path_length_meander_height_um,
        ripup_reroute_config=RipupRerouteConfig(
            enabled=args.ripup_reroute,
            max_rounds=args.ripup_max_rounds,
            max_victims_per_failure=args.ripup_max_victims,
            history_weight=args.ripup_history_weight,
            history_increment=args.ripup_history_increment,
        ),
        collect_attempt_diagnostics=getattr(args, "attempt_diagnostics", False),
        static_obstacle_config=StaticObstacleMapConfig(
            grid_size_um=args.grid_size_um,
            obstacle_mode=args.obstacle_mode,
            clearance_um=args.waveguide_clearance_um,
            heater_clearance_um=args.heater_clearance_um,
            chip_add_x_um=args.chip_add_x_um,
            chip_add_y_um=args.chip_add_y_um,
            clear_port_open_cells_from_static=args.clear_port_open_cells_from_static,
        ),
        stats=stats,
    )
    layout_info_raw = getattr(routed_layout, "info", {})
    layout_info: SupportsGet = (
        layout_info_raw if _supports_get(layout_info_raw) else EMPTY_INFO
    )
    meander_report = layout_info.get("meander_insertion_report", {})
    if not isinstance(meander_report, Mapping):
        meander_report = {}
    meander_results = meander_report.get("results", [])
    if not isinstance(meander_results, list):
        meander_results = []
    meander_status_counts: dict[str, int] = {}
    for result in meander_results:
        if not isinstance(result, Mapping):
            continue
        status = str(result.get("status", "unknown"))
        meander_status_counts[status] = meander_status_counts.get(status, 0) + 1
    slowest_meander = _slowest_meander_result(meander_results)
    matching_group_diagnostics = _path_length_group_diagnostics(layout_info)
    path_length_analysis = _path_length_analysis_info(layout_info)
    route_nets_timing_fields = {
        f"route_nets_{key}_s": stats.step_times_s.get(f"route_nets.{key}")
        for key in ROUTE_NETS_TIMING_KEYS
    }
    return {
        "benchmark": benchmark,
        "instances": stats.instance_count,
        "nets": stats.net_count,
        "grid": (
            f"{stats.static_grid_width}x{stats.static_grid_height}"
            if stats.static_grid_width is not None
            and stats.static_grid_height is not None
            else ""
        ),
        "total_s": stats.total_time_s,
        "load_s": stats.step_times_s.get("load_benchmark"),
        "layout_s": stats.step_times_s.get("layout_from_schematic"),
        "route_s": stats.step_times_s.get("baseline_gdsfactory_routing"),
        "route_nets_s": stats.step_times_s.get("route_nets"),
        "path_length_analysis_s": stats.step_times_s.get("path_length_analysis"),
        "meander_obstacle_map_s": stats.step_times_s.get("meander_obstacle_map"),
        "meander_planning_s": stats.step_times_s.get("meander_planning"),
        "route_realization_s": stats.step_times_s.get("route_realization"),
        "route_endpoint_correction_s": stats.step_times_s.get(
            "route_endpoint_correction"
        ),
        **route_nets_timing_fields,
        "astar_s": stats.astar_time_s,
        "meander_requirements": _list_length(layout_info.get("meander_requirements", [])),
        "path_length_group_count": len(matching_group_diagnostics),
        "path_length_groups_with_requirements": _count_groups_with_requirements(
            matching_group_diagnostics
        ),
        "path_length_groups_over_tolerance": _count_groups_over_tolerance(
            matching_group_diagnostics
        ),
        "path_length_lifted_group_count": _count_lifted_groups(
            matching_group_diagnostics
        ),
        "path_length_max_target_lift_um": _max_group_float(
            matching_group_diagnostics,
            "target_lift_um",
        ),
        "path_length_raw_requirements": _list_length(
            path_length_analysis.get("raw_requirements", [])
        ),
        "path_length_min_insertable_extra_um": path_length_analysis.get(
            "minimum_insertable_extra_length_um"
        ),
        "path_length_max_accepted_unmatched_um": _max_group_float(
            matching_group_diagnostics,
            "max_accepted_unmatched_um",
        ),
        "path_length_max_physical_residual_um": _max_group_float(
            matching_group_diagnostics,
            "max_physical_residual_um",
        ),
        "path_length_max_disregarded_residual_um": _max_group_float(
            matching_group_diagnostics,
            "max_disregarded_residual_um",
        ),
        "meander_planner_calls": meander_report.get("planner_calls"),
        "meander_requested_um": meander_report.get("total_requested_extra_length_um"),
        "meander_inserted_um": meander_report.get("total_inserted_extra_length_um"),
        "meander_disregarded_um": meander_report.get("total_disregarded_extra_length_um"),
        "meander_unmatched_um": meander_report.get("unmatched_length_um"),
        "meander_status_counts": meander_status_counts,
        "meander_planner_elapsed_s": meander_report.get("planner_elapsed_s"),
        "meander_candidate_profile": meander_report.get("candidate_profile", {}),
        "meander_candidate_runs": _sum_meander_result_int(
            meander_results,
            "candidate_runs",
        ),
        "meander_candidate_intervals": _sum_meander_result_int(
            meander_results,
            "candidate_intervals",
        ),
        "meander_rejected_box_blocked": _sum_meander_result_int(
            meander_results,
            "rejected_box_blocked",
        ),
        "meander_rejected_planning_failed": _sum_meander_result_int(
            meander_results,
            "rejected_planning_failed",
        ),
        "meander_rejected_exact_length_mismatch": _sum_meander_result_int(
            meander_results,
            "rejected_exact_length_mismatch",
        ),
        "meander_rejected_too_short": _sum_meander_result_int(
            meander_results,
            "rejected_too_short",
        ),
        "meander_max_candidate_runs": _max_meander_result_int(
            meander_results,
            "candidate_runs",
        ),
        "meander_max_candidate_intervals": _max_meander_result_int(
            meander_results,
            "candidate_intervals",
        ),
        "slowest_meander_planning_s": slowest_meander.get("planning_elapsed_s"),
        "slowest_meander_status": slowest_meander.get("status"),
        "slowest_meander_requested_um": slowest_meander.get("requested_extra_length_um"),
        "slowest_meander_candidate_runs": slowest_meander.get("candidate_runs"),
        "slowest_meander_candidate_intervals": slowest_meander.get("candidate_intervals"),
        "slowest_meander_rejected_box_blocked": slowest_meander.get("rejected_box_blocked"),
        "slowest_meander_rejected_planning_failed": slowest_meander.get(
            "rejected_planning_failed"
        ),
        "slowest_meander_rejected_exact_length_mismatch": slowest_meander.get(
            "rejected_exact_length_mismatch"
        ),
        "route_attempts": stats.route_attempts,
        "route_failures": stats.route_failures,
        "simple_routes": stats.simple_route_count,
        "repairs": stats.repair_count,
        "expanded_states": stats.expanded_states,
        "generated_neighbors": stats.generated_neighbors,
        "heap_pushes": stats.heap_pushes,
        "heap_pops": stats.heap_pops,
        "duplicate_heap_skips": stats.skipped_duplicate_heap_entries,
        "stale_generation_heap_entries": stats.stale_generation_heap_entries,
        "closed_heap_entries": stats.closed_heap_entries,
        "max_heap_size": stats.max_heap_size,
        "dense_search_states": stats.dense_search_states,
        "dense_search_storage_bytes": stats.dense_search_storage_bytes,
        "best_cost_updates": stats.best_cost_updates,
        "parent_updates": stats.parent_updates,
        "obstacle_clearance_checks": stats.obstacle_clearance_checks,
        "footprint_checks": stats.footprint_checks,
        "footprint_rect_checks": stats.footprint_rect_checks,
        "full_grid_fallbacks": stats.full_grid_fallbacks,
        "neighbor_generation_s": stats.neighbor_generation_time_s,
        "heap_operation_s": stats.heap_operation_time_s,
        "legality_check_s": stats.legality_check_time_s,
        "reconstruction_s": stats.reconstruction_time_s,
        "route_attempt_records": stats.route_attempt_records,
    }


def _worker_command(benchmark: str, args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--_worker-benchmark",
        benchmark,
        "--max-iterations",
        str(args.max_iterations),
        "--routing-window-scale",
        str(args.routing_window_scale),
        "--bend-radius-um",
        str(args.bend_radius_um),
        "--grid-size-um",
        str(args.grid_size_um),
        "--chip-add-x-um",
        str(args.chip_add_x_um),
        "--chip-add-y-um",
        str(args.chip_add_y_um),
        "--path-length-meander-height-um",
        str(args.path_length_meander_height_um),
        "--ripup-max-rounds",
        str(args.ripup_max_rounds),
        "--ripup-max-victims",
        str(args.ripup_max_victims),
        "--ripup-history-weight",
        str(args.ripup_history_weight),
        "--ripup-history-increment",
        str(args.ripup_history_increment),
        "--obstacle-mode",
        args.obstacle_mode,
        "--waveguide-clearance-um",
        str(args.waveguide_clearance_um),
        "--heater-clearance-um",
        str(args.heater_clearance_um),
        "--primitive-ordering",
        args.primitive_ordering,
        "--heuristic-mode",
        args.heuristic_mode,
        "--heap-tie-breaker",
        args.heap_tie_breaker,
    ]
    if args.use_indexed_heap:
        command.append("--use-indexed-heap")
    if not args.enable_simple_routes:
        command.append("--no-enable-simple-routes")
    if args.path_length_matching:
        command.append("--path-length-matching")
    if args.path_length_match_outputs:
        command.append("--path-length-match-outputs")
    if args.allow_45_degree_turns:
        command.append("--allow-45-degree-turns")
    if args.include_heater_obstacles:
        command.append("--include-heater-obstacles")
    if args.ripup_reroute:
        command.append("--ripup-reroute")
    if args.clear_port_open_cells_from_static:
        command.append("--clear-port-open-cells-from-static")
    if getattr(args, "attempt_diagnostics", False):
        command.append("--attempt-diagnostics")
    return command


def _worker_row(benchmark: str, args: argparse.Namespace) -> dict[str, object]:
    result = subprocess.run(
        _worker_command(benchmark, args),
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    for line in result.stdout.splitlines():
        if not line.startswith(WORKER_MARKER):
            sys.stderr.write(f"{line}\n")
    sys.stderr.write(result.stderr)
    result.check_returncode()
    marker_lines = [
        line[len(WORKER_MARKER) :]
        for line in result.stdout.splitlines()
        if line.startswith(WORKER_MARKER)
    ]
    if not marker_lines:
        raise RuntimeError(f"benchmark worker did not report stats for {benchmark}")
    loaded = json.loads(marker_lines[-1])
    if not isinstance(loaded, dict):
        raise RuntimeError(f"benchmark worker reported invalid stats for {benchmark}")
    return loaded


_REPEAT_MEDIAN_FIELDS = (
    "total_s",
    "load_s",
    "layout_s",
    "route_s",
    "route_nets_s",
    "route_realization_s",
    "route_endpoint_correction_s",
    "astar_s",
    "path_length_analysis_s",
    "meander_obstacle_map_s",
    "meander_planning_s",
    "meander_planner_elapsed_s",
) + tuple(f"route_nets_{key}_s" for key in ROUTE_NETS_TIMING_KEYS)


def _as_float(value: object) -> float | None:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _median_float(values: Iterable[object]) -> float | None:
    numbers = [_as_float(value) for value in values]
    filtered = [value for value in numbers if value is not None]
    if not filtered:
        return None
    return float(statistics.median(filtered))


def _aggregate_repeat_rows(
    samples: list[dict[str, object]], metric: str
) -> dict[str, object]:
    if not samples:
        raise ValueError("expected at least one benchmark sample")
    if len(samples) == 1:
        row = dict(samples[0])
        row["repeat_runs"] = 1
        return row

    sortable_samples = [
        (metric_value, sample)
        for sample in samples
        if (metric_value := _as_float(sample.get(metric))) is not None
    ]
    representative = (
        sorted(sortable_samples, key=lambda item: item[0])[len(sortable_samples) // 2][1]
        if sortable_samples
        else samples[0]
    )
    row = dict(representative)
    for field in _REPEAT_MEDIAN_FIELDS:
        median_value = _median_float(sample.get(field) for sample in samples)
        if median_value is not None:
            row[field] = median_value

    metric_samples = [
        value
        for sample in samples
        if (value := _as_float(sample.get(metric))) is not None
    ]
    row["repeat_runs"] = len(samples)
    row["perf_metric"] = metric
    row["perf_metric_samples"] = metric_samples
    if metric_samples:
        row["perf_metric_min"] = min(metric_samples)
        row["perf_metric_max"] = max(metric_samples)
    return row


def _benchmark_rows(args: argparse.Namespace) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    repeat_runs = max(1, int(getattr(args, "repeat_runs", 1)))
    metric = str(getattr(args, "perf_metric", DEFAULT_PERF_METRIC))
    for benchmark in args.benchmarks:
        samples = [_worker_row(benchmark, args) for _ in range(repeat_runs)]
        rows.append(_aggregate_repeat_rows(samples, metric))
    return rows


def _rows_by_benchmark(payload: object) -> dict[str, Mapping[str, object]]:
    if isinstance(payload, Mapping):
        benchmarks = payload.get("benchmarks")
        if isinstance(benchmarks, Mapping):
            return {
                str(name): row
                for name, row in benchmarks.items()
                if isinstance(row, Mapping)
            }
    if isinstance(payload, list):
        return {
            str(row["benchmark"]): row
            for row in payload
            if isinstance(row, Mapping) and "benchmark" in row
        }
    return {}


def _perf_baseline_payload(
    rows: Iterable[Mapping[str, object]],
    *,
    metric: str,
    relative_tolerance: float,
    absolute_tolerance_s: float,
    counter_relative_tolerance: float,
) -> dict[str, object]:
    benchmarks: dict[str, dict[str, object]] = {}
    for row in rows:
        name = str(row.get("benchmark", ""))
        if not name:
            continue
        compact: dict[str, object] = {
            metric: row.get(metric),
            "grid": row.get("grid"),
            "nets": row.get("nets"),
            "route_attempts": row.get("route_attempts"),
            "route_failures": row.get("route_failures"),
            "simple_routes": row.get("simple_routes"),
            "repairs": row.get("repairs"),
            "expanded_states": row.get("expanded_states"),
            "generated_neighbors": row.get("generated_neighbors"),
            "repeat_runs": row.get("repeat_runs", 1),
        }
        benchmarks[name] = compact
    return {
        "metric": metric,
        "relative_tolerance": relative_tolerance,
        "absolute_tolerance_s": absolute_tolerance_s,
        "counter_relative_tolerance": counter_relative_tolerance,
        "benchmarks": benchmarks,
    }


def _load_json(path: Path) -> object:
    load_path = path if path.is_absolute() else PROJECT_ROOT / path
    return json.loads(load_path.read_text(encoding="utf-8"))


def _write_json(path: Path, payload: object) -> None:
    output_path = path if path.is_absolute() else PROJECT_ROOT / path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _perf_baseline_violations(
    current_rows: Iterable[Mapping[str, object]],
    baseline_payload: object,
    *,
    metric: str,
    relative_tolerance: float,
    absolute_tolerance_s: float,
    counter_relative_tolerance: float,
) -> list[dict[str, object]]:
    current_by_benchmark = {
        str(row.get("benchmark", "")): row
        for row in current_rows
        if str(row.get("benchmark", ""))
    }
    baseline_by_benchmark = _rows_by_benchmark(baseline_payload)
    violations: list[dict[str, object]] = []
    for benchmark, baseline in baseline_by_benchmark.items():
        current = current_by_benchmark.get(benchmark)
        if current is None:
            violations.append(
                {
                    "benchmark": benchmark,
                    "name": "benchmark",
                    "reason": "missing_current_row",
                }
            )
            continue

        expected = _as_float(baseline.get(metric))
        actual = _as_float(current.get(metric))
        if expected is None:
            violations.append(
                {
                    "benchmark": benchmark,
                    "name": metric,
                    "reason": "missing_baseline_metric",
                }
            )
        elif actual is None:
            violations.append(
                {
                    "benchmark": benchmark,
                    "name": metric,
                    "reason": "missing_current_metric",
                }
            )
        else:
            allowed = expected * (1.0 + relative_tolerance) + absolute_tolerance_s
            if actual > allowed:
                violations.append(
                    {
                        "benchmark": benchmark,
                        "name": metric,
                        "actual": actual,
                        "expected": expected,
                        "allowed": allowed,
                        "relative_tolerance": relative_tolerance,
                        "absolute_tolerance_s": absolute_tolerance_s,
                    }
                )

        for key in (
            "grid",
            "nets",
            "route_attempts",
            "route_failures",
            "simple_routes",
            "repairs",
        ):
            if baseline.get(key) != current.get(key):
                violations.append(
                    {
                        "benchmark": benchmark,
                        "name": key,
                        "actual": current.get(key),
                        "expected": baseline.get(key),
                    }
                )

        for key in ("expanded_states", "generated_neighbors"):
            expected_counter = _as_float(baseline.get(key))
            actual_counter = _as_float(current.get(key))
            if expected_counter is None or actual_counter is None:
                continue
            allowed_counter = expected_counter * (1.0 + counter_relative_tolerance)
            if actual_counter > allowed_counter:
                violations.append(
                    {
                        "benchmark": benchmark,
                        "name": key,
                        "actual": actual_counter,
                        "expected": expected_counter,
                        "allowed": allowed_counter,
                        "relative_tolerance": counter_relative_tolerance,
                    }
                )
    for benchmark in sorted(set(current_by_benchmark) - set(baseline_by_benchmark)):
        violations.append(
            {
                "benchmark": benchmark,
                "name": "benchmark",
                "reason": "missing_baseline_row",
            }
        )
    return violations


def _attach_perf_baseline_violations(
    rows: list[dict[str, object]],
    violations: list[dict[str, object]],
) -> None:
    by_benchmark: dict[str, list[dict[str, object]]] = {}
    for violation in violations:
        by_benchmark.setdefault(str(violation.get("benchmark", "")), []).append(
            violation
        )
    for row in rows:
        row_violations = by_benchmark.get(str(row.get("benchmark", "")), [])
        if row_violations:
            row["perf_baseline_violations"] = row_violations


def _has_perf_baseline_violations(rows: Iterable[Mapping[str, object]]) -> bool:
    return any(bool(row.get("perf_baseline_violations")) for row in rows)


def _markdown_report(rows: Iterable[dict[str, object]], args: argparse.Namespace) -> str:
    lines = [
        "# Photonic Routing Baseline",
        "",
        f"- Captured: {datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"- Git revision: `{_git_rev()}`",
        f"- Python: `{platform.python_version()}`",
        f"- Path-length matching: `{args.path_length_matching}`",
        f"- Path-length match outputs: `{getattr(args, 'path_length_match_outputs', False)}`",
        f"- 45-degree turns: `{args.allow_45_degree_turns}`",
        f"- Heater obstacles: `{args.include_heater_obstacles}`",
        f"- Obstacle mode: `{args.obstacle_mode}`",
        f"- Max iterations: `{args.max_iterations}`",
        f"- Routing window scale: `{args.routing_window_scale}`",
        f"- Indexed heap: `{args.use_indexed_heap}`",
        f"- Primitive ordering: `{args.primitive_ordering}`",
        f"- Heuristic mode: `{args.heuristic_mode}`",
        f"- Heap tie-breaker: `{getattr(args, 'heap_tie_breaker', 'smaller_g')}`",
        f"- Attempt diagnostics: `{getattr(args, 'attempt_diagnostics', False)}`",
        f"- Repeat runs: `{getattr(args, 'repeat_runs', 1)}`",
        f"- Perf metric: `{getattr(args, 'perf_metric', DEFAULT_PERF_METRIC)}`",
        "",
        "| Benchmark | Instances | Nets | Grid | Total s | Route s | A* s | Attempts | Simple | Repairs | Expanded | Generated | Heap push/pop | Dup skips | Stale gen/closed | Max heap | Dense MiB | Obstacle checks | Footprint rect checks | Full fallback |",
        "| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| {benchmark} | {instances} | {nets} | {grid} | {total_s} | {route_s} | {astar_s} | {attempts} | {simple} | {repairs} | {expanded} | {generated} | {heap_pushes}/{heap_pops} | {dup_skips} | {stale_generation_heap_entries}/{closed_heap_entries} | {max_heap_size} | {dense_search_storage_mib} | {obstacle_checks} | {footprint_rect_checks} | {fallbacks} |".format(
                benchmark=row["benchmark"],
                instances=row["instances"],
                nets=row["nets"],
                grid=row["grid"],
                total_s=_format_seconds(_row_seconds(row, "total_s")),
                route_s=_format_seconds(_row_seconds(row, "route_s")),
                astar_s=_format_seconds(_row_seconds(row, "astar_s")),
                attempts=_format_int(row["route_attempts"]),
                simple=_format_int(row["simple_routes"]),
                repairs=_format_int(row["repairs"]),
                expanded=_format_int(row["expanded_states"]),
                generated=_format_int(row["generated_neighbors"]),
                heap_pushes=_format_int(row["heap_pushes"]),
                heap_pops=_format_int(row["heap_pops"]),
                dup_skips=_format_int(row["duplicate_heap_skips"]),
                stale_generation_heap_entries=_format_int(
                    row["stale_generation_heap_entries"]
                ),
                closed_heap_entries=_format_int(row["closed_heap_entries"]),
                max_heap_size=_format_int(row["max_heap_size"]),
                dense_search_storage_mib=_format_mib(
                    row["dense_search_storage_bytes"]
                ),
                obstacle_checks=_format_int(row["obstacle_clearance_checks"]),
                footprint_rect_checks=_format_int(row["footprint_rect_checks"]),
                fallbacks=_format_int(row["full_grid_fallbacks"]),
            )
        )
    route_split_rows = [
        row
        for row in rows
        if any(
            _as_float(row.get(f"route_nets_{key}_s")) is not None
            for key in ROUTE_NETS_TIMING_KEYS
        )
    ]
    if route_split_rows:
        lines.extend(
            [
                "",
                "## Route Nets Timing Split",
                "",
                "| Benchmark | Obstacle map s | Prep s | Native batch s | Batch post s | Endpoint corr s | Debug/artifact s | Other route_nets s |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        prep_keys = (
            "router_setup",
            "route_job_build",
            "port_opening_prep",
            "port_opening_batch",
            "static_map_handoff",
            "state_opening_precompute",
            "clearance_exempt_batch",
            "batch_job_pack",
        )
        endpoint_keys = (
            "endpoint_correction_pack",
            "endpoint_correction_native",
            "endpoint_correction_processing",
        )
        artifact_keys = ("record_assembly", "debug_artifact_assembly")

        def route_nets_time(row: Mapping[str, object], key: str) -> float:
            return _numeric_float(row.get(f"route_nets_{key}_s"))

        for row in route_split_rows:
            obstacle_map_s = route_nets_time(row, "obstacle_map")
            prep_s = sum(route_nets_time(row, key) for key in prep_keys)
            native_batch_s = route_nets_time(row, "native_route_batch")
            batch_post_s = route_nets_time(row, "batch_result_processing")
            endpoint_s = sum(route_nets_time(row, key) for key in endpoint_keys)
            artifact_s = sum(route_nets_time(row, key) for key in artifact_keys)
            known_s = (
                obstacle_map_s
                + prep_s
                + native_batch_s
                + batch_post_s
                + endpoint_s
                + artifact_s
            )
            route_nets_s = _numeric_float(row.get("route_nets_s"))
            other_s = max(0.0, route_nets_s - known_s)
            lines.append(
                "| {benchmark} | {obstacle_map_s} | {prep_s} | {native_batch_s} | {batch_post_s} | {endpoint_s} | {artifact_s} | {other_s} |".format(
                    benchmark=row.get("benchmark", ""),
                    obstacle_map_s=_format_seconds(obstacle_map_s),
                    prep_s=_format_seconds(prep_s),
                    native_batch_s=_format_seconds(native_batch_s),
                    batch_post_s=_format_seconds(batch_post_s),
                    endpoint_s=_format_seconds(endpoint_s),
                    artifact_s=_format_seconds(artifact_s),
                    other_s=_format_seconds(other_s),
                )
            )
    native_split_rows = [
        row
        for row in rows
        if any(
            _as_float(row.get(f"route_nets_native_batch_{key}_s")) is not None
            for key in (
                "route_job_unpack",
                "route_search_total",
                "simple_route_candidate",
                "dense_astar",
                "commit_update_dynamic_map",
                "route_result_construction",
                "python_return_dict",
                "normal_route_failed_wall",
                "probe_route_failed_wall",
                "repair_failed_net_failed_wall",
                "reroute_victims_failed_wall",
            )
        )
    ]
    if native_split_rows:
        lines.extend(
            [
                "",
                "## Native Batch Timing Split",
                "",
                "| Benchmark | Unpack s | Route wall s | Failed route wall s | Search total s | Simple in route s | Dense A* in route s | Commit build in route s | Commit/map in route s | Repair book s | Result obj s | Py return s | Other native s |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )

        def native_batch_time(row: Mapping[str, object], key: str) -> float:
            return _numeric_float(row.get(f"route_nets_native_batch_{key}_s"))

        repair_bookkeeping_keys = (
            "repair_probe_victim_selection",
            "repair_state_reset",
            "ripup",
            "history_update",
        )
        route_wall_keys = (
            "normal_route_wall",
            "probe_route_wall",
            "repair_failed_net_wall",
            "reroute_victims_wall",
        )
        failed_route_wall_keys = (
            "normal_route_failed_wall",
            "probe_route_failed_wall",
            "repair_failed_net_failed_wall",
            "reroute_victims_failed_wall",
        )
        for row in native_split_rows:
            unpack_s = native_batch_time(row, "route_job_unpack")
            route_wall_s = sum(native_batch_time(row, key) for key in route_wall_keys)
            failed_route_wall_s = sum(
                native_batch_time(row, key) for key in failed_route_wall_keys
            )
            search_total_s = native_batch_time(row, "route_search_total")
            simple_s = native_batch_time(row, "simple_route_candidate")
            dense_s = native_batch_time(row, "dense_astar")
            commit_build_s = native_batch_time(row, "commit_cell_build")
            commit_s = native_batch_time(row, "commit_update_dynamic_map")
            repair_book_s = sum(
                native_batch_time(row, key) for key in repair_bookkeeping_keys
            )
            result_obj_s = native_batch_time(row, "route_result_construction")
            py_return_s = native_batch_time(row, "python_return_dict")
            known_s = (
                unpack_s
                + route_wall_s
                + repair_book_s
                + result_obj_s
                + py_return_s
            )
            native_total_s = _numeric_float(row.get("route_nets_native_route_batch_s"))
            other_native_s = max(0.0, native_total_s - known_s)
            lines.append(
                "| {benchmark} | {unpack_s} | {route_wall_s} | {failed_route_wall_s} | {search_total_s} | {simple_s} | {dense_s} | {commit_build_s} | {commit_s} | {repair_book_s} | {result_obj_s} | {py_return_s} | {other_native_s} |".format(
                    benchmark=row.get("benchmark", ""),
                    unpack_s=_format_seconds(unpack_s),
                    route_wall_s=_format_seconds(route_wall_s),
                    failed_route_wall_s=_format_seconds(failed_route_wall_s),
                    search_total_s=_format_seconds(search_total_s),
                    simple_s=_format_seconds(simple_s),
                    dense_s=_format_seconds(dense_s),
                    commit_build_s=_format_seconds(commit_build_s),
                    commit_s=_format_seconds(commit_s),
                    repair_book_s=_format_seconds(repair_book_s),
                    result_obj_s=_format_seconds(result_obj_s),
                    py_return_s=_format_seconds(py_return_s),
                    other_native_s=_format_seconds(other_native_s),
                )
            )
    repeated_rows = [
        row for row in rows if int(row.get("repeat_runs", 1) or 1) > 1
    ]
    if repeated_rows:
        metric = str(getattr(args, "perf_metric", DEFAULT_PERF_METRIC))
        lines.extend(
            [
                "",
                "## Repeat Samples",
                "",
                "| Benchmark | Runs | Metric | Median s | Min s | Max s | Samples s |",
                "| --- | ---: | --- | ---: | ---: | ---: | --- |",
            ]
        )
        for row in repeated_rows:
            samples = row.get("perf_metric_samples", [])
            sample_values = samples if isinstance(samples, list) else []
            lines.append(
                "| {benchmark} | {runs} | {metric} | {median} | {minimum} | {maximum} | {samples} |".format(
                    benchmark=row.get("benchmark", ""),
                    runs=_format_int(row.get("repeat_runs")),
                    metric=metric,
                    median=_format_seconds(_record_seconds(row, metric)),
                    minimum=_format_seconds(_record_seconds(row, "perf_metric_min")),
                    maximum=_format_seconds(_record_seconds(row, "perf_metric_max")),
                    samples=", ".join(
                        _format_seconds(_as_float(value)) for value in sample_values
                    ),
                )
            )
    violation_rows = [
        row for row in rows if row.get("perf_baseline_violations")
    ]
    if violation_rows:
        lines.extend(
            [
                "",
                "## Performance Baseline Violations",
                "",
                "| Benchmark | Metric | Actual | Expected | Allowed | Reason |",
                "| --- | --- | ---: | ---: | ---: | --- |",
            ]
        )
        for row in violation_rows:
            violations = row.get("perf_baseline_violations", [])
            if not isinstance(violations, list):
                continue
            for violation in violations:
                if not isinstance(violation, Mapping):
                    continue
                lines.append(
                    "| {benchmark} | {name} | {actual} | {expected} | {allowed} | {reason} |".format(
                        benchmark=violation.get("benchmark", ""),
                        name=violation.get("name", ""),
                        actual=violation.get("actual", ""),
                        expected=violation.get("expected", ""),
                        allowed=violation.get("allowed", ""),
                        reason=violation.get("reason", ""),
                    )
                )
    plm_rows = [
        row
        for row in rows
        if bool(args.path_length_matching)
        or row.get("path_length_analysis_s") is not None
        or row.get("meander_requirements") not in (None, 0)
    ]
    if plm_rows:
        lines.extend(
            [
                "",
                "## Path-Length Matching",
                "",
                "| Benchmark | Groups | Groups needing PLM | Lifted groups | Max lift um | Min bump um | Raw reqs | Groups over tol | Max accepted residual um | Max physical residual um | Max disregarded residual um | Requirements | Planner calls | Requested um | Inserted um | Disregarded um | Unmatched um | Analysis s | Obstacle s | Planning s | Realization s | Statuses |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
            ]
        )
        for row in plm_rows:
            lines.append(
                "| {benchmark} | {groups} | {groups_needing} | {lifted_groups} | {max_lift} | {min_bump} | {raw_requirements} | {groups_over} | {max_accepted} | {max_physical} | {max_disregarded} | {requirements} | {planner_calls} | {requested} | {inserted} | {disregarded} | {unmatched} | {analysis_s} | {obstacle_s} | {planning_s} | {realization_s} | {statuses} |".format(
                    benchmark=row["benchmark"],
                    groups=_format_int(row.get("path_length_group_count")),
                    groups_needing=_format_int(
                        row.get("path_length_groups_with_requirements")
                    ),
                    lifted_groups=_format_int(
                        row.get("path_length_lifted_group_count")
                    ),
                    max_lift=_format_seconds(
                        _record_seconds(row, "path_length_max_target_lift_um")
                    ),
                    min_bump=_format_seconds(
                        _record_seconds(row, "path_length_min_insertable_extra_um")
                    ),
                    raw_requirements=_format_int(
                        row.get("path_length_raw_requirements")
                    ),
                    groups_over=_format_int(row.get("path_length_groups_over_tolerance")),
                    max_accepted=_format_seconds(
                        _record_seconds(row, "path_length_max_accepted_unmatched_um")
                    ),
                    max_physical=_format_seconds(
                        _record_seconds(row, "path_length_max_physical_residual_um")
                    ),
                    max_disregarded=_format_seconds(
                        _record_seconds(row, "path_length_max_disregarded_residual_um")
                    ),
                    requirements=_format_int(row.get("meander_requirements")),
                    planner_calls=_format_int(row.get("meander_planner_calls")),
                    requested=_format_seconds(
                        _record_seconds(row, "meander_requested_um")
                    ),
                    inserted=_format_seconds(
                        _record_seconds(row, "meander_inserted_um")
                    ),
                    disregarded=_format_seconds(
                        _record_seconds(row, "meander_disregarded_um")
                    ),
                    unmatched=_format_seconds(
                        _record_seconds(row, "meander_unmatched_um")
                    ),
                    analysis_s=_format_seconds(
                        _record_seconds(row, "path_length_analysis_s")
                    ),
                    obstacle_s=_format_seconds(
                        _record_seconds(row, "meander_obstacle_map_s")
                    ),
                    planning_s=_format_seconds(
                        _record_seconds(row, "meander_planning_s")
                    ),
                    realization_s=_format_seconds(
                        _record_seconds(row, "route_realization_s")
                    ),
                    statuses=_format_status_counts(row.get("meander_status_counts")),
                )
            )
        diagnostic_rows = [
            row
            for row in plm_rows
            if row.get("meander_planner_calls") not in (None, 0)
            or row.get("meander_candidate_runs") not in (None, 0)
            or row.get("slowest_meander_planning_s") is not None
        ]
        if diagnostic_rows:
            lines.extend(
                [
                    "",
                    "## Meander Planner Diagnostics",
                    "",
                    "| Benchmark | Planner elapsed s | Candidate runs | Intervals | Blocked | Plan fail | Exact mismatch | Too short | Max runs | Max intervals | Slowest s | Slowest requested um | Slowest status | Slowest counters | Candidate profile |",
                    "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |",
                ]
            )
            for row in diagnostic_rows:
                slowest_counters = (
                    "runs={runs} intervals={intervals} blocked={blocked} "
                    "plan_fail={plan_fail} exact_mismatch={exact_mismatch}"
                ).format(
                    runs=_format_int(row.get("slowest_meander_candidate_runs")),
                    intervals=_format_int(
                        row.get("slowest_meander_candidate_intervals")
                    ),
                    blocked=_format_int(
                        row.get("slowest_meander_rejected_box_blocked")
                    ),
                    plan_fail=_format_int(
                        row.get("slowest_meander_rejected_planning_failed")
                    ),
                    exact_mismatch=_format_int(
                        row.get("slowest_meander_rejected_exact_length_mismatch")
                    ),
                )
                lines.append(
                    "| {benchmark} | {planner_elapsed_s} | {candidate_runs} | {candidate_intervals} | {blocked} | {plan_fail} | {exact_mismatch} | {too_short} | {max_runs} | {max_intervals} | {slowest_s} | {slowest_requested} | {slowest_status} | {slowest_counters} | {candidate_profile} |".format(
                        benchmark=row["benchmark"],
                        planner_elapsed_s=_format_seconds(
                            _record_seconds(row, "meander_planner_elapsed_s")
                        ),
                        candidate_runs=_format_int(row.get("meander_candidate_runs")),
                        candidate_intervals=_format_int(
                            row.get("meander_candidate_intervals")
                        ),
                        blocked=_format_int(row.get("meander_rejected_box_blocked")),
                        plan_fail=_format_int(
                            row.get("meander_rejected_planning_failed")
                        ),
                        exact_mismatch=_format_int(
                            row.get("meander_rejected_exact_length_mismatch")
                        ),
                        too_short=_format_int(row.get("meander_rejected_too_short")),
                        max_runs=_format_int(row.get("meander_max_candidate_runs")),
                        max_intervals=_format_int(
                            row.get("meander_max_candidate_intervals")
                        ),
                        slowest_s=_format_seconds(
                            _record_seconds(row, "slowest_meander_planning_s")
                        ),
                        slowest_requested=_format_seconds(
                            _record_seconds(row, "slowest_meander_requested_um")
                        ),
                        slowest_status=row.get("slowest_meander_status", ""),
                        slowest_counters=slowest_counters,
                        candidate_profile=_format_candidate_profile(
                            row.get("meander_candidate_profile")
                        ),
                    )
                )
    all_attempts = _flatten_attempt_records(rows)
    slow_attempts = sorted(
        (
            record
            for record in all_attempts
            if not bool(record.get("used_simple_route", False))
            or bool(record.get("failed", False))
        ),
        key=_attempt_seconds,
        reverse=True,
    )[:8]
    if slow_attempts:
        lines.extend(
            [
                "",
                "## Slowest Route Attempts",
                "",
                "| Benchmark | Attempt | Bucket | Route | Net | Time s | Window cells | Expanded | Generated | Primitive gen | Primitive accepted | Primitive footprint rejects | Heap push/pop | Stale gen/closed | Max heap | Dense MiB | Rect checks | Dense build s | Failed |",
                "| --- | ---: | --- | ---: | --- | ---: | ---: | ---: | ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
            ]
        )
        for record in slow_attempts:
            lines.append(
                "| {benchmark} | {attempt} | {bucket} | {route} | {net} | {time_s} | {window_area} | {expanded} | {generated} | {primitive_generated} | {primitive_accepted} | {primitive_footprint_rejects} | {heap_pushes}/{heap_pops} | {stale_generation_heap_entries}/{closed_heap_entries} | {max_heap_size} | {dense_search_storage_mib} | {rect_checks} | {dense_build_s} | {failed} |".format(
                    benchmark=record.get("benchmark", ""),
                    attempt=record.get("attempt_index", ""),
                    bucket=record.get("bucket_name", ""),
                    route=record.get("route_index", ""),
                    net=record.get("net_name", ""),
                    time_s=_format_seconds(_attempt_seconds(record)),
                    window_area=_format_int(record.get("last_window_area_cells")),
                    expanded=_format_int(record.get("expanded_states")),
                    generated=_format_int(record.get("generated_neighbors")),
                    primitive_generated=_format_primitive_counter(
                        record.get("primitive_generated_by_class")
                    ),
                    primitive_accepted=_format_primitive_counter(
                        record.get("primitive_accepted_by_class")
                    ),
                    primitive_footprint_rejects=_format_primitive_counter(
                        record.get("primitive_footprint_rejects_by_class")
                    ),
                    heap_pushes=_format_int(record.get("heap_pushes")),
                    heap_pops=_format_int(record.get("heap_pops")),
                    stale_generation_heap_entries=_format_int(
                        record.get("stale_generation_heap_entries")
                    ),
                    closed_heap_entries=_format_int(record.get("closed_heap_entries")),
                    max_heap_size=_format_int(record.get("max_heap_size")),
                    dense_search_storage_mib=_format_mib(
                        record.get("dense_search_storage_bytes")
                    ),
                    rect_checks=_format_int(record.get("footprint_rect_checks")),
                    dense_build_s=_format_seconds(
                        _record_seconds(record, "dense_grid_build_time_s")
                    ),
                    failed=record.get("failed", ""),
                )
            )
        diagnostic_attempts = [
            record for record in slow_attempts if _record_diagnostics(record)
        ]
        if diagnostic_attempts:
            lines.extend(
                [
                    "",
                    "## Dominant Route Diagnostics",
                    "",
                    "| Benchmark | Attempt | Bucket | Route | Net | Span | Window | Route bbox | Window/span | Route/window | LB/cost | Static dens | Dynamic dens | Dynamic before | Blockers | Victims |",
                    "| --- | ---: | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
                ]
            )
        for record in diagnostic_attempts[:6]:
            diagnostics = _record_diagnostics(record)
            span = (
                f"{_format_int(diagnostics.get('span_x_cells'))}x"
                f"{_format_int(diagnostics.get('span_y_cells'))}"
            )
            window = (
                f"{_format_int(diagnostics.get('window_width_cells'))}x"
                f"{_format_int(diagnostics.get('window_height_cells'))}"
            )
            route_bbox = (
                f"{_format_int(diagnostics.get('route_bbox_width_cells'))}x"
                f"{_format_int(diagnostics.get('route_bbox_height_cells'))}"
            )
            lines.append(
                "| {benchmark} | {attempt} | {bucket} | {route} | {net} | {span} | {window} | {route_bbox} | {window_span} | {route_window} | {lower_bound_cost} | {static_density} | {dynamic_density} | {dynamic_before} | {blockers} | {victims} |".format(
                    benchmark=record.get("benchmark", ""),
                    attempt=record.get("attempt_index", ""),
                    bucket=record.get("bucket_name", ""),
                    route=record.get("route_index", ""),
                    net=record.get("net_name", ""),
                    span=span,
                    window=window,
                    route_bbox=route_bbox,
                    window_span=_format_ratio(
                        _diagnostic_float(
                            diagnostics,
                            "window_to_span_bbox_area",
                        )
                    ),
                    route_window=_format_ratio(
                        _diagnostic_float(
                            diagnostics,
                            "route_bbox_to_window_area",
                        )
                    ),
                    lower_bound_cost=_format_percent(
                        _diagnostic_float(
                            diagnostics,
                            "heading_lower_bound_to_cost",
                        )
                    ),
                    static_density=_format_percent(
                        _diagnostic_float(diagnostics, "window_static_density")
                    ),
                    dynamic_density=_format_percent(
                        _diagnostic_float(diagnostics, "window_dynamic_density")
                    ),
                    dynamic_before=_format_int(
                        diagnostics.get("committed_dynamic_cells_before")
                    ),
                    blockers=_format_int(diagnostics.get("candidate_blocker_count")),
                    victims=_format_int(diagnostics.get("ripup_victim_count")),
                )
            )
    lines.extend(
        [
            "",
            "Detailed JSON rows also include load/layout time, per-attempt route records, and neighbor-generation, heap-operation, legality-check, and reconstruction timing buckets.",
        ]
    )
    lines.append("")
    return "\n".join(lines)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmarks", nargs="*", default=list(DEFAULT_BENCHMARKS))
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--repeat-runs",
        type=int,
        default=1,
        help=(
            "Run each benchmark this many times in fresh worker processes and "
            "report median timing fields."
        ),
    )
    parser.add_argument(
        "--perf-metric",
        default=DEFAULT_PERF_METRIC,
        help="Numeric row field to compare for performance regressions.",
    )
    parser.add_argument(
        "--compare-perf-baseline",
        type=Path,
        nargs="?",
        const=DEFAULT_PERF_BASELINE_PATH,
        default=None,
        help=(
            "Compare compact benchmark rows against a JSON performance baseline "
            "and exit non-zero on violations. Defaults to "
            f"{DEFAULT_PERF_BASELINE_PATH.relative_to(PROJECT_ROOT)} when no "
            "path is supplied."
        ),
    )
    parser.add_argument(
        "--write-perf-baseline",
        type=Path,
        default=None,
        help="Write a compact JSON performance baseline from the current rows.",
    )
    parser.add_argument(
        "--perf-relative-tolerance",
        type=float,
        default=DEFAULT_PERF_RELATIVE_TOLERANCE,
        help="Allowed relative slowdown against the baseline metric.",
    )
    parser.add_argument(
        "--perf-absolute-tolerance-s",
        type=float,
        default=DEFAULT_PERF_ABSOLUTE_TOLERANCE_S,
        help="Additional absolute slowdown allowance in seconds.",
    )
    parser.add_argument(
        "--perf-counter-relative-tolerance",
        type=float,
        default=DEFAULT_PERF_COUNTER_RELATIVE_TOLERANCE,
        help="Allowed relative increase for expanded/generated route counters.",
    )
    parser.add_argument(
        "--attempt-output",
        type=Path,
        default=None,
        help="Write per-route-attempt records as JSON, or CSV when the suffix is .csv.",
    )
    parser.add_argument("--path-length-matching", action="store_true")
    parser.add_argument("--path-length-match-outputs", action="store_true")
    parser.add_argument(
        "--path-length-meander-height-um",
        type=float,
        default=SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    )
    parser.add_argument("--allow-45-degree-turns", action="store_true")
    parser.add_argument("--bend-radius-um", type=float, default=SCRIPT_BEND_RADIUS_UM)
    parser.add_argument(
        "--enable-simple-routes",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--max-iterations", type=int, default=5_000_000)
    parser.add_argument("--routing-window-scale", type=float, default=0.05)
    parser.add_argument("--include-heater-obstacles", action="store_true")
    parser.add_argument("--ripup-reroute", action="store_true")
    parser.add_argument("--ripup-max-rounds", type=int, default=SCRIPT_RIPUP_MAX_ROUNDS)
    parser.add_argument(
        "--ripup-max-victims", type=int, default=SCRIPT_RIPUP_MAX_VICTIMS
    )
    parser.add_argument(
        "--ripup-history-weight", type=float, default=SCRIPT_RIPUP_HISTORY_WEIGHT
    )
    parser.add_argument(
        "--ripup-history-increment", type=int, default=SCRIPT_RIPUP_HISTORY_INCREMENT
    )
    parser.add_argument(
        "--attempt-diagnostics",
        action="store_true",
        help=(
            "Collect extra per-attempt window, obstacle-density, and rip-up "
            "diagnostics for slow or failed route attempts."
        ),
    )
    parser.add_argument(
        "--use-indexed-heap",
        action="store_true",
        help=(
            "Benchmark-only: use the experimental decrease-key indexed heap "
            "for dense A* (slower in Pass 8E; default stays off)."
        ),
    )
    parser.add_argument(
        "--primitive-ordering",
        choices=("library", "long_straight_first", "target_biased"),
        default="library",
        help=(
            "Benchmark-only dense A* primitive ordering experiment "
            "(Pass 8F keeps library as the default)."
        ),
    )
    parser.add_argument(
        "--heuristic-mode",
        choices=("distance", "heading_aware"),
        default="heading_aware",
        help="Dense A* heuristic mode.",
    )
    parser.add_argument(
        "--heap-tie-breaker",
        choices=("smaller_g", "larger_g"),
        default="smaller_g",
        help=(
            "Benchmark-only dense A* heap tie-breaker experiment. "
            "Default preserves historical smaller-g behavior."
        ),
    )
    parser.add_argument("--obstacle-mode", default="bounding_boxes")
    parser.add_argument("--grid-size-um", type=float, default=SCRIPT_GRID_SIZE_UM)
    parser.add_argument("--waveguide-clearance-um", type=float, default=0.5)
    parser.add_argument("--heater-clearance-um", type=float, default=5.0)
    parser.add_argument("--chip-add-x-um", type=float, default=SCRIPT_CHIP_ADD_X_UM)
    parser.add_argument("--chip-add-y-um", type=float, default=SCRIPT_CHIP_ADD_Y_UM)
    parser.add_argument("--clear-port-open-cells-from-static", action="store_true")
    parser.add_argument("--_worker-benchmark", default=None, help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args._worker_benchmark is not None:
        row = _run_single_benchmark(args._worker_benchmark, args)
        print(f"{WORKER_MARKER}{json.dumps(row, sort_keys=True)}")
        return 0

    rows = _benchmark_rows(args)
    if args.compare_perf_baseline is not None:
        baseline_payload = _load_json(args.compare_perf_baseline)
        violations = _perf_baseline_violations(
            rows,
            baseline_payload,
            metric=args.perf_metric,
            relative_tolerance=args.perf_relative_tolerance,
            absolute_tolerance_s=args.perf_absolute_tolerance_s,
            counter_relative_tolerance=args.perf_counter_relative_tolerance,
        )
        _attach_perf_baseline_violations(rows, violations)
    report = _markdown_report(rows, args)
    print(report)
    if args.output is not None:
        output_path = args.output
        if not output_path.is_absolute():
            output_path = PROJECT_ROOT / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(report, encoding="utf-8")
    if args.attempt_output is not None:
        attempt_output_path = args.attempt_output
        if not attempt_output_path.is_absolute():
            attempt_output_path = PROJECT_ROOT / attempt_output_path
        _write_attempt_output(rows, attempt_output_path)
    if args.write_perf_baseline is not None:
        _write_json(
            args.write_perf_baseline,
            _perf_baseline_payload(
                rows,
                metric=args.perf_metric,
                relative_tolerance=args.perf_relative_tolerance,
                absolute_tolerance_s=args.perf_absolute_tolerance_s,
                counter_relative_tolerance=args.perf_counter_relative_tolerance,
            ),
        )
    return 1 if _has_perf_baseline_violations(rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
