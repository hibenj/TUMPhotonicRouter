#!/usr/bin/env python3
"""Run repeatable photonic routing benchmarks and print a compact table."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Iterable, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow import RipupRerouteConfig, RoutingFlowStats, run_routing_flow


DEFAULT_BENCHMARKS = ("TOY", "mmi_heater", "mmi_heater_8x4")
WORKER_MARKER = "PHOTONIC_BENCHMARK_JSON:"


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


def _row_seconds(row: dict[str, object], key: str) -> float | None:
    return cast(float | None, row[key])


def _run_single_benchmark(benchmark: str, args: argparse.Namespace) -> dict[str, object]:
    stats = RoutingFlowStats()
    run_routing_flow(
        benchmark,
        debug_svgs=False,
        debug_timing=False,
        show_klayout=False,
        enable_path_length_matching=args.path_length_matching,
        allow_45_degree_turns=args.allow_45_degree_turns,
        max_iterations=args.max_iterations,
        include_heater_obstacles=args.include_heater_obstacles,
        ripup_reroute_config=RipupRerouteConfig(enabled=args.ripup_reroute),
        static_obstacle_config=StaticObstacleMapConfig(
            obstacle_mode=args.obstacle_mode,
            clearance_um=args.waveguide_clearance_um,
            heater_clearance_um=args.heater_clearance_um,
            clear_port_open_cells_from_static=False,
        ),
        stats=stats,
    )
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
    }


def _worker_command(benchmark: str, args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--_worker-benchmark",
        benchmark,
        "--max-iterations",
        str(args.max_iterations),
        "--obstacle-mode",
        args.obstacle_mode,
        "--waveguide-clearance-um",
        str(args.waveguide_clearance_um),
        "--heater-clearance-um",
        str(args.heater_clearance_um),
    ]
    if args.path_length_matching:
        command.append("--path-length-matching")
    if args.allow_45_degree_turns:
        command.append("--allow-45-degree-turns")
    if args.include_heater_obstacles:
        command.append("--include-heater-obstacles")
    if args.ripup_reroute:
        command.append("--ripup-reroute")
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
        "",
        "| Benchmark | Instances | Nets | Grid | Total s | Load s | Layout s | Route s |",
        "| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| {benchmark} | {instances} | {nets} | {grid} | {total_s} | {load_s} | {layout_s} | {route_s} |".format(
                benchmark=row["benchmark"],
                instances=row["instances"],
                nets=row["nets"],
                grid=row["grid"],
                total_s=_format_seconds(_row_seconds(row, "total_s")),
                load_s=_format_seconds(_row_seconds(row, "load_s")),
                layout_s=_format_seconds(_row_seconds(row, "layout_s")),
                route_s=_format_seconds(_row_seconds(row, "route_s")),
            )
        )
    lines.append("")
    return "\n".join(lines)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmarks", nargs="*", default=list(DEFAULT_BENCHMARKS))
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--path-length-matching", action="store_true")
    parser.add_argument("--allow-45-degree-turns", action="store_true")
    parser.add_argument("--max-iterations", type=int, default=5_000_000)
    parser.add_argument("--include-heater-obstacles", action="store_true")
    parser.add_argument("--ripup-reroute", action="store_true")
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
