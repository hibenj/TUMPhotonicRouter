#!/usr/bin/env python3
"""Run repeatable photonic routing benchmarks and print a compact table."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any, Iterable, Literal, Mapping, TypeAlias, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow import RipupRerouteConfig, RoutingFlowStats, run_routing_flow


DEFAULT_BENCHMARKS = ("TOY", "mmi_heater", "mmi_heater_8x4_ripup_reroute")
WORKER_MARKER = "PHOTONIC_BENCHMARK_JSON:"
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
    if not isinstance(layout_info, Mapping):
        return []
    analysis = layout_info.get("path_length_analysis", {})
    if not isinstance(analysis, Mapping):
        return []
    groups = analysis.get("matching_group_diagnostics")
    if not isinstance(groups, list):
        groups = analysis.get("matching_groups")
    if not isinstance(groups, list):
        return []
    return [group for group in groups if isinstance(group, Mapping)]


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
        allow_45_degree_turns=args.allow_45_degree_turns,
        use_indexed_heap=args.use_indexed_heap,
        primitive_ordering=args.primitive_ordering,
        heuristic_mode=args.heuristic_mode,
        heap_tie_breaker=args.heap_tie_breaker,
        max_iterations=args.max_iterations,
        routing_window_scale=args.routing_window_scale,
        include_heater_obstacles=args.include_heater_obstacles,
        ripup_reroute_config=RipupRerouteConfig(enabled=args.ripup_reroute),
        collect_attempt_diagnostics=getattr(args, "attempt_diagnostics", False),
        static_obstacle_config=StaticObstacleMapConfig(
            obstacle_mode=args.obstacle_mode,
            clearance_um=args.waveguide_clearance_um,
            heater_clearance_um=args.heater_clearance_um,
            clear_port_open_cells_from_static=False,
        ),
        stats=stats,
    )
    layout_info = getattr(routed_layout, "info", {})
    if not hasattr(layout_info, "get"):
        layout_info = {}
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
        "astar_s": stats.astar_time_s,
        "meander_requirements": len(
            layout_info.get("meander_requirements", [])
            if isinstance(layout_info.get("meander_requirements", []), list)
            else []
        ),
        "path_length_group_count": len(matching_group_diagnostics),
        "path_length_groups_with_requirements": _count_groups_with_requirements(
            matching_group_diagnostics
        ),
        "path_length_groups_over_tolerance": _count_groups_over_tolerance(
            matching_group_diagnostics
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
    if args.path_length_matching:
        command.append("--path-length-matching")
    if args.allow_45_degree_turns:
        command.append("--allow-45-degree-turns")
    if args.include_heater_obstacles:
        command.append("--include-heater-obstacles")
    if args.ripup_reroute:
        command.append("--ripup-reroute")
    if getattr(args, "attempt_diagnostics", False):
        command.append("--attempt-diagnostics")
    return command


def _benchmark_rows(args: argparse.Namespace) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for benchmark in args.benchmarks:
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
        rows.append(json.loads(marker_lines[-1]))
    return rows


def _markdown_report(rows: Iterable[dict[str, object]], args: argparse.Namespace) -> str:
    lines = [
        "# Photonic Routing Baseline",
        "",
        f"- Captured: {datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"- Git revision: `{_git_rev()}`",
        f"- Python: `{platform.python_version()}`",
        f"- Path-length matching: `{args.path_length_matching}`",
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
                "| Benchmark | Groups | Groups needing PLM | Groups over tol | Max accepted residual um | Max physical residual um | Max disregarded residual um | Requirements | Planner calls | Requested um | Inserted um | Disregarded um | Unmatched um | Analysis s | Obstacle s | Planning s | Realization s | Statuses |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
            ]
        )
        for row in plm_rows:
            lines.append(
                "| {benchmark} | {groups} | {groups_needing} | {groups_over} | {max_accepted} | {max_physical} | {max_disregarded} | {requirements} | {planner_calls} | {requested} | {inserted} | {disregarded} | {unmatched} | {analysis_s} | {obstacle_s} | {planning_s} | {realization_s} | {statuses} |".format(
                    benchmark=row["benchmark"],
                    groups=_format_int(row.get("path_length_group_count")),
                    groups_needing=_format_int(
                        row.get("path_length_groups_with_requirements")
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
                    "| Benchmark | Planner elapsed s | Candidate runs | Intervals | Blocked | Plan fail | Exact mismatch | Too short | Max runs | Max intervals | Slowest s | Slowest requested um | Slowest status | Slowest counters |",
                    "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
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
                    "| {benchmark} | {planner_elapsed_s} | {candidate_runs} | {candidate_intervals} | {blocked} | {plan_fail} | {exact_mismatch} | {too_short} | {max_runs} | {max_intervals} | {slowest_s} | {slowest_requested} | {slowest_status} | {slowest_counters} |".format(
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
        "--attempt-output",
        type=Path,
        default=None,
        help="Write per-route-attempt records as JSON, or CSV when the suffix is .csv.",
    )
    parser.add_argument("--path-length-matching", action="store_true")
    parser.add_argument("--allow-45-degree-turns", action="store_true")
    parser.add_argument("--max-iterations", type=int, default=5_000_000)
    parser.add_argument("--routing-window-scale", type=float, default=0.05)
    parser.add_argument("--include-heater-obstacles", action="store_true")
    parser.add_argument("--ripup-reroute", action="store_true")
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
    parser.add_argument("--waveguide-clearance-um", type=float, default=0.5)
    parser.add_argument("--heater-clearance-um", type=float, default=5.0)
    parser.add_argument("--_worker-benchmark", default=None, help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args._worker_benchmark is not None:
        row = _run_single_benchmark(args._worker_benchmark, args)
        print(f"{WORKER_MARKER}{json.dumps(row, sort_keys=True)}")
        return 0

    rows = _benchmark_rows(args)
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
