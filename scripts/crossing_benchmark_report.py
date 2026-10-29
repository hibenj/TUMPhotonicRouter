"""Run crossing-aware routing benchmarks and summarize routing/crossing stats."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from routing_flow import (
    RoutingFlowStats,
    load_benchmark,
    run_routing_flow,
)

DEFAULT_BENCHMARKS = ("benes_4x4", "benes_8x8", "benes_16x16", "benes_32x32")


def _crossing_json_path(benchmark: str) -> Path:
    return PROJECT_ROOT / "build" / "crossings" / f"{benchmark.lower()}_crossings.json"


def _load_crossing_summary(benchmark: str) -> dict[str, Any]:
    path = _crossing_json_path(benchmark)
    if not path.exists():
        return {
            "planned_crossings": 0,
            "actual_crossings": 0,
            "crossing_report": None,
        }
    data = json.loads(path.read_text(encoding="utf-8"))
    # `actual_crossings` counts the crossing events the kernel registered
    # (`native_crossing_event_count`; the plan-versus-realized reconciliation
    # that produced `actual_crossing_count` went with phase 4 slice A of the
    # 2026-10 do-over). `planned_crossings` is the topology plan's event
    # count: 0 under lidar-pure, which ignores the plan.
    return {
        "planned_crossings": int(data.get("event_count", 0) or 0),
        "actual_crossings": int(data.get("native_crossing_event_count", 0) or 0),
        "crossing_report": str(path.relative_to(PROJECT_ROOT)),
        "crossing_device": data.get("crossing_device"),
    }


def run_benchmark(
    benchmark: str,
    *,
    max_iterations: int,
    crossing_half_size_cells: int,
) -> dict[str, Any]:
    schematic = load_benchmark(benchmark)
    net_count = len(schematic.netlist.routes)
    stats = RoutingFlowStats()
    start = time.perf_counter()
    status = "ok"
    error = ""
    crossing_path = _crossing_json_path(benchmark)
    crossing_path.unlink(missing_ok=True)
    crossing_path.with_suffix(".txt").unlink(missing_ok=True)
    try:
        run_routing_flow(
            benchmark,
            debug_svgs=str(net_count),
            show_klayout=False,
            enable_crossings=True,
            crossing_half_size_cells=int(crossing_half_size_cells),
            allow_45_degree_turns=True,
            collect_route_stats=True,
            max_iterations=max_iterations,
            stats=stats,
        )
    except Exception as exc:  # pragma: no cover - exercised manually for failures.
        status = "failed"
        error = f"{type(exc).__name__}: {exc}"
    elapsed_s = time.perf_counter() - start
    crossing_summary = _load_crossing_summary(benchmark)
    return {
        "benchmark": benchmark,
        "status": status,
        "error": error,
        "instance_count": len(schematic.netlist.instances),
        "net_count": net_count,
        "crossing_half_size_cells": int(crossing_half_size_cells),
        "route_failures": max(int(stats.route_failures or 0), int(status != "ok")),
        "simple_routes": int(stats.simple_route_count),
        "route_attempts": int(stats.route_attempts),
        "expanded_states": int(stats.expanded_states),
        "max_heap_size": int(stats.max_heap_size),
        "runtime_s": round(elapsed_s, 3),
        **crossing_summary,
    }


def _print_table(rows: list[dict[str, Any]]) -> None:
    columns = (
        "benchmark",
        "status",
        "net_count",
        "planned_crossings",
        "actual_crossings",
        "route_failures",
        "expanded_states",
        "runtime_s",
    )
    widths = {
        column: max(len(column), *(len(str(row.get(column, ""))) for row in rows))
        for column in columns
    }
    print("  ".join(column.ljust(widths[column]) for column in columns))
    print("  ".join("-" * widths[column] for column in columns))
    for row in rows:
        print("  ".join(str(row.get(column, "")).ljust(widths[column]) for column in columns))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmarks", nargs="*", default=list(DEFAULT_BENCHMARKS))
    parser.add_argument("--max-iterations", type=int, default=500_000)
    parser.add_argument(
        "--crossing-half-size-cells",
        type=int,
        default=0,
        help="Crossing keepout half-size in grid cells; 0 derives from the component bbox.",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=PROJECT_ROOT / "build" / "crossing_benchmark_report.json",
        help="Path to write the JSON report.",
    )
    args = parser.parse_args(argv)

    rows = [
        run_benchmark(
            str(benchmark),
            max_iterations=int(args.max_iterations),
            crossing_half_size_cells=int(args.crossing_half_size_cells),
        )
        for benchmark in args.benchmarks
    ]
    _print_table(rows)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(rows, indent=2, sort_keys=True), encoding="utf-8")
    print(f"\nWrote {args.json}")
    return 1 if any(row["status"] != "ok" for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
