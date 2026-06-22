#!/usr/bin/env python3
"""Profile isolated Rust A* routing scenarios.

This intentionally avoids gdsfactory and the schematic pipeline. It measures
the PyO3 router call plus Rust search/route result conversion for small,
repeatable grids, and reports the route stats returned by Rust.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable as IterableABC
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time
from typing import Any, Iterable, Protocol, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
for path in (PROJECT_ROOT, PYTHON_SOURCE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from photonic_router.static_obstacle_builder import _load_rust_backend

DEFAULT_BASELINE_PATH = PROJECT_ROOT / "docs" / "astar_quality_baseline.json"


@dataclass(frozen=True)
class AStarScenario:
    name: str
    width: int
    height: int
    source: tuple[int, int, int]
    target: tuple[int, int, int]
    static_cells: tuple[tuple[int, int], ...] = ()
    opened_cells: tuple[tuple[int, int], ...] = ()
    allow_45_degree_turns: bool = False
    enable_simple_routes: bool = False
    require_target_angle: bool = False
    use_routing_window: bool = True
    routing_window_fallback_full_grid: bool = True
    max_iterations: int = 500_000


class RustBackend(Protocol):
    def GridSpec(
        self,
        width: int,
        height: int,
        grid_size_um: float,
        origin_x_um: float,
        origin_y_um: float,
    ) -> object: ...

    def PrimitiveLibraryConfig(
        self,
        *,
        grid_size_um: float,
        bend_radius_cells: int,
        allow_45_degree_turns: bool,
    ) -> object: ...

    def AStarConfig(
        self,
        *,
        max_iterations: int,
        require_target_angle: bool,
        use_routing_window: bool,
        routing_window_fallback_full_grid: bool,
        collect_detailed_timing: bool,
    ) -> Any: ...

    def PyPhotonicRouter(self, grid: object, primitive: object, astar: Any) -> Any: ...

    def State(self, x: int, y: int, angle: int) -> object: ...


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
    return f"{value:.6f}"


def _vertical_wall_with_gap(
    *,
    x: int,
    y_min: int,
    y_max: int,
    gap_min: int,
    gap_max: int,
) -> tuple[tuple[int, int], ...]:
    return tuple(
        (x, y)
        for y in range(y_min, y_max + 1)
        if not gap_min <= y <= gap_max
    )


def _slalom_walls() -> tuple[tuple[int, int], ...]:
    cells: list[tuple[int, int]] = []
    for x in (45, 75, 105, 135):
        gap_min, gap_max = (12, 20) if x in (45, 105) else (50, 58)
        cells.extend(
            (x, y)
            for y in range(4, 68)
            if not gap_min <= y <= gap_max
        )
    return tuple(cells)


PORT_ANGLES = {
    "e": 0,
    "n": 2,
    "w": 4,
    "s": 6,
}


def _rect_cells(
    x_min: int,
    y_min: int,
    x_max: int,
    y_max: int,
) -> tuple[tuple[int, int], ...]:
    return tuple(
        (x, y)
        for x in range(x_min, x_max + 1)
        for y in range(y_min, y_max + 1)
    )


def _object_port_state(
    rect: tuple[int, int, int, int],
    side: str,
    *,
    as_target: bool = False,
    clearance_cells: int = 2,
) -> tuple[int, int, int]:
    x_min, y_min, x_max, y_max = rect
    center_x = (x_min + x_max) // 2
    center_y = (y_min + y_max) // 2
    if side == "n":
        x, y = center_x, y_max + clearance_cells
    elif side == "e":
        x, y = x_max + clearance_cells, center_y
    elif side == "s":
        x, y = center_x, y_min - clearance_cells
    elif side == "w":
        x, y = x_min - clearance_cells, center_y
    else:
        raise ValueError(f"Unsupported port side: {side}")

    outward_angle = PORT_ANGLES[side]
    route_angle = (outward_angle + 4) % 8 if as_target else outward_angle
    return x, y, route_angle


def _two_object_port_scenarios() -> dict[str, AStarScenario]:
    left_rect = (28, 30, 44, 50)
    right_rect = (76, 30, 92, 50)
    static_cells = _rect_cells(*left_rect) + _rect_cells(*right_rect)
    scenarios: dict[str, AStarScenario] = {}
    for source_side in ("n", "e", "s", "w"):
        for target_side in ("n", "e", "s", "w"):
            name = f"object_ports_{source_side}_{target_side}"
            scenarios[name] = AStarScenario(
                name=name,
                width=124,
                height=84,
                source=_object_port_state(left_rect, source_side, as_target=False),
                target=_object_port_state(right_rect, target_side, as_target=True),
                static_cells=static_cells,
                allow_45_degree_turns=False,
                enable_simple_routes=False,
                require_target_angle=True,
                use_routing_window=True,
                routing_window_fallback_full_grid=True,
                max_iterations=750_000,
            )
    return scenarios


def _scenario_catalog() -> dict[str, AStarScenario]:
    scenarios = {
        "straight_simple": AStarScenario(
            name="straight_simple",
            width=160,
            height=48,
            source=(10, 24, 0),
            target=(140, 24, 0),
            enable_simple_routes=True,
            require_target_angle=True,
        ),
        "straight_astar": AStarScenario(
            name="straight_astar",
            width=160,
            height=48,
            source=(10, 24, 0),
            target=(140, 24, 0),
            enable_simple_routes=False,
            require_target_angle=True,
        ),
        "wall_gap_astar": AStarScenario(
            name="wall_gap_astar",
            width=180,
            height=80,
            source=(12, 20, 0),
            target=(160, 20, 0),
            static_cells=_vertical_wall_with_gap(
                x=85,
                y_min=4,
                y_max=72,
                gap_min=42,
                gap_max=50,
            ),
        ),
        "slalom_astar": AStarScenario(
            name="slalom_astar",
            width=180,
            height=80,
            source=(12, 36, 0),
            target=(165, 36, 0),
            static_cells=_slalom_walls(),
        ),
        "full_grid_fallback": AStarScenario(
            name="full_grid_fallback",
            width=220,
            height=120,
            source=(10, 18, 0),
            target=(205, 102, 0),
            static_cells=_vertical_wall_with_gap(
                x=110,
                y_min=0,
                y_max=118,
                gap_min=104,
                gap_max=110,
            ),
            use_routing_window=True,
            routing_window_fallback_full_grid=True,
        ),
    }
    scenarios.update(_two_object_port_scenarios())
    return scenarios


def _build_router(rust_backend: RustBackend, scenario: AStarScenario) -> Any:
    grid = rust_backend.GridSpec(
        scenario.width,
        scenario.height,
        1.0,
        0.0,
        0.0,
    )
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=2,
        allow_45_degree_turns=scenario.allow_45_degree_turns,
    )
    astar = rust_backend.AStarConfig(
        max_iterations=scenario.max_iterations,
        require_target_angle=scenario.require_target_angle,
        use_routing_window=scenario.use_routing_window,
        routing_window_fallback_full_grid=scenario.routing_window_fallback_full_grid,
        collect_detailed_timing=True,
    )
    astar.enable_simple_routes = scenario.enable_simple_routes
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    if scenario.static_cells:
        router.set_static_cells(list(scenario.static_cells))
    return router


def _route_once(rust_backend: RustBackend, scenario: AStarScenario) -> tuple[float, object]:
    router = _build_router(rust_backend, scenario)
    source = rust_backend.State(*scenario.source)
    target = rust_backend.State(*scenario.target)
    start = time.perf_counter()
    route = router.route_single_net(
        source,
        target,
        opened_cells=list(scenario.opened_cells) if scenario.opened_cells else None,
    )
    elapsed_s = time.perf_counter() - start
    return elapsed_s, route


def _quantile(values: list[float], ratio: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * ratio)))
    return ordered[index]


def _route_stat(route: object, attr: str) -> int | float | bool:
    value = getattr(route, attr)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value
    return 0


def _state_tuple(state: object) -> tuple[int, int, int]:
    return (
        int(getattr(state, "x")),
        int(getattr(state, "y")),
        int(getattr(state, "angle")),
    )


def run_scenario(
    rust_backend: RustBackend,
    scenario: AStarScenario,
    *,
    iterations: int,
    warmup: int,
) -> dict[str, object]:
    if iterations <= 0:
        raise ValueError("iterations must be > 0")
    if warmup < 0:
        raise ValueError("warmup must be >= 0")

    for _ in range(warmup):
        _route_once(rust_backend, scenario)

    elapsed_values: list[float] = []
    last_route: object | None = None
    for _ in range(iterations):
        elapsed_s, route = _route_once(rust_backend, scenario)
        elapsed_values.append(elapsed_s)
        last_route = route

    assert last_route is not None
    reached_target = _state_tuple(getattr(last_route, "reached_target"))
    return {
        "scenario": scenario.name,
        "grid": f"{scenario.width}x{scenario.height}",
        "source": list(scenario.source),
        "target": list(scenario.target),
        "reached_target": list(reached_target),
        "target_state_ok": reached_target == scenario.target,
        "static_cells": len(scenario.static_cells),
        "iterations": iterations,
        "median_s": statistics.median(elapsed_values),
        "mean_s": statistics.fmean(elapsed_values),
        "min_s": min(elapsed_values),
        "p95_s": _quantile(elapsed_values, 0.95),
        "expanded_states": _route_stat(last_route, "expanded_states"),
        "generated_neighbors": _route_stat(last_route, "generated_neighbors"),
        "heap_pushes": _route_stat(last_route, "heap_pushes"),
        "heap_pops": _route_stat(last_route, "heap_pops"),
        "duplicate_heap_skips": _route_stat(last_route, "skipped_duplicate_heap_entries"),
        "obstacle_clearance_checks": _route_stat(last_route, "obstacle_clearance_checks"),
        "window_attempts": _route_stat(last_route, "window_attempts"),
        "window_rejects": _route_stat(last_route, "window_rejects"),
        "footprint_checks": _route_stat(last_route, "primitive_footprint_checks"),
        "footprint_rejects": _route_stat(last_route, "footprint_rejects"),
        "rect_checks": _route_stat(last_route, "primitive_footprint_rect_checks"),
        "rect_rejects": _route_stat(last_route, "primitive_footprint_rect_rejects"),
        "dense_cells": _route_stat(last_route, "dense_grid_cells"),
        "dense_build_s": float(_route_stat(last_route, "dense_grid_build_time_us")) / 1_000_000.0,
        "neighbor_generation_s": float(_route_stat(last_route, "neighbor_generation_time_us")) / 1_000_000.0,
        "heap_operation_s": float(_route_stat(last_route, "heap_operation_time_us")) / 1_000_000.0,
        "legality_check_s": float(_route_stat(last_route, "legality_check_time_us")) / 1_000_000.0,
        "reconstruction_s": float(_route_stat(last_route, "reconstruction_time_us")) / 1_000_000.0,
        "full_grid_fallback": bool(_route_stat(last_route, "used_full_grid_fallback")),
        "route_cells": len(getattr(last_route, "cells", [])),
        "route_length_um": float(getattr(last_route, "total_length_um", 0.0)),
    }


def _load_baseline(path: Path) -> dict[str, object]:
    baseline_path = path if path.is_absolute() else PROJECT_ROOT / path
    return json.loads(baseline_path.read_text(encoding="utf-8"))


def _object_to_float(value: object) -> float:
    if isinstance(value, (str, bytes, bytearray, int, float)):
        return float(value)
    return 0.0


def _object_to_list(value: object) -> list[object]:
    if isinstance(value, IterableABC) and not isinstance(value, (str, bytes, bytearray)):
        return list(value)
    return []


def _check_baseline(
    rows: Iterable[dict[str, object]],
    baseline: dict[str, object],
    *,
    length_tolerance_um: float | None,
) -> list[str]:
    scenarios = baseline.get("scenarios", {})
    if not isinstance(scenarios, dict):
        return ["baseline is missing object field: scenarios"]

    tolerance = (
        float(length_tolerance_um)
        if length_tolerance_um is not None
        else _object_to_float(baseline.get("length_tolerance_um", 0.0))
    )
    failures: list[str] = []
    for row in rows:
        name = str(row["scenario"])
        expected = scenarios.get(name)
        if not isinstance(expected, dict):
            failures.append(f"{name}: missing from baseline")
            continue

        expected_length = _object_to_float(expected["route_length_um"])
        actual_length = _object_to_float(row["route_length_um"])
        if actual_length > expected_length + tolerance:
            failures.append(
                f"{name}: route length {actual_length:.6g} um exceeds baseline "
                f"{expected_length:.6g} um + tolerance {tolerance:.6g} um"
            )

        expected_target = expected.get("target")
        if expected_target is not None and _object_to_list(row["target"]) != _object_to_list(expected_target):
            failures.append(
                f"{name}: target changed from {expected_target} to {row['target']}"
            )

        expected_reached = expected.get("reached_target", expected_target)
        if expected_reached is not None and _object_to_list(row["reached_target"]) != _object_to_list(expected_reached):
            failures.append(
                f"{name}: reached target {row['reached_target']} != expected {expected_reached}"
            )

        if expected.get("target_state_ok", True) and not bool(row["target_state_ok"]):
            failures.append(f"{name}: route did not reach the required target state")

    return failures


def _markdown_report(rows: Iterable[dict[str, object]], args: argparse.Namespace) -> str:
    lines = [
        "# Isolated Rust A* Profile",
        "",
        f"- Captured: {datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"- Git revision: `{_git_rev()}`",
        f"- Python: `{platform.python_version()}`",
        f"- Iterations: `{args.iterations}`",
        f"- Warmup: `{args.warmup}`",
        "",
        "| Scenario | Grid | Obstacles | Median s | P95 s | Expanded | Generated | Heap push/pop | Dup skips | Legality checks | Footprint checks | Dense build s | Neighbor s | Heap s | Legality s | Reconstruct s | Full grid | Target ok | Route cells | Length um |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| {scenario} | {grid} | {static_cells} | {median_s} | {p95_s} | {expanded_states} | {generated_neighbors} | {heap_pushes}/{heap_pops} | {duplicate_heap_skips} | {obstacle_clearance_checks} | {footprint_checks} | {dense_build_s} | {neighbor_generation_s} | {heap_operation_s} | {legality_check_s} | {reconstruction_s} | {full_grid_fallback} | {target_state_ok} | {route_cells} | {route_length_um:.3f} |".format(
                scenario=row["scenario"],
                grid=row["grid"],
                static_cells=row["static_cells"],
                median_s=_format_seconds(_object_to_float(row["median_s"])),
                p95_s=_format_seconds(_object_to_float(row["p95_s"])),
                expanded_states=row["expanded_states"],
                generated_neighbors=row["generated_neighbors"],
                heap_pushes=row["heap_pushes"],
                heap_pops=row["heap_pops"],
                duplicate_heap_skips=row["duplicate_heap_skips"],
                obstacle_clearance_checks=row["obstacle_clearance_checks"],
                footprint_checks=row["footprint_checks"],
                dense_build_s=_format_seconds(_object_to_float(row["dense_build_s"])),
                neighbor_generation_s=_format_seconds(_object_to_float(row["neighbor_generation_s"])),
                heap_operation_s=_format_seconds(_object_to_float(row["heap_operation_s"])),
                legality_check_s=_format_seconds(_object_to_float(row["legality_check_s"])),
                reconstruction_s=_format_seconds(_object_to_float(row["reconstruction_s"])),
                full_grid_fallback=row["full_grid_fallback"],
                target_state_ok=row.get("target_state_ok", ""),
                route_cells=row["route_cells"],
                route_length_um=row["route_length_um"],
            )
        )
    lines.append("")
    return "\n".join(lines)


def _parse_args() -> argparse.Namespace:
    catalog = _scenario_catalog()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenarios",
        nargs="*",
        default=list(catalog),
        metavar="scenario",
        help=f"Scenario names. Available: {', '.join(sorted(catalog))}",
    )
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--json-output", type=Path, default=None)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE_PATH)
    parser.add_argument("--check-baseline", action="store_true")
    parser.add_argument("--length-tolerance-um", type=float, default=None)
    args = parser.parse_args()
    unknown = sorted(set(args.scenarios) - set(catalog))
    if unknown:
        parser.error(
            "unknown scenario(s): "
            + ", ".join(unknown)
            + f". Available: {', '.join(sorted(catalog))}"
        )
    return args


def _write_text(path: Path, text: str) -> None:
    output_path = path if path.is_absolute() else PROJECT_ROOT / path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(text, encoding="utf-8")


def main() -> int:
    args = _parse_args()
    rust_backend_obj = _load_rust_backend()
    if rust_backend_obj is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `maturin develop`."
        )
    rust_backend = cast(RustBackend, rust_backend_obj)

    catalog = _scenario_catalog()
    rows = [
        run_scenario(
            rust_backend,
            catalog[name],
            iterations=args.iterations,
            warmup=args.warmup,
        )
        for name in args.scenarios
    ]
    report = _markdown_report(rows, args)
    print(report)
    if args.check_baseline:
        baseline = _load_baseline(args.baseline)
        failures = _check_baseline(
            rows,
            baseline,
            length_tolerance_um=args.length_tolerance_um,
        )
        if failures:
            for failure in failures:
                print(f"BASELINE CHECK FAILED: {failure}", file=sys.stderr)
            return 1
        print("Baseline quality check passed.")
    if args.output is not None:
        _write_text(args.output, report)
    if args.json_output is not None:
        _write_text(args.json_output, json.dumps(rows, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
