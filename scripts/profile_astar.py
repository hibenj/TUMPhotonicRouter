#!/usr/bin/env python3
"""Profile isolated Rust A* routing scenarios.

This intentionally avoids gdsfactory and the schematic pipeline. It measures
the PyO3 router call plus Rust search/route result conversion for small,
repeatable grids, and reports the route stats returned by Rust.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable as IterableABC
from dataclasses import dataclass, replace
from datetime import datetime
import json
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time
from typing import Any, Iterable, Literal, Protocol, TypeAlias, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
for path in (PROJECT_ROOT, PYTHON_SOURCE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from photonic_router.static_obstacle_builder import _load_rust_backend

DEFAULT_BASELINE_PATH = PROJECT_ROOT / "docs" / "astar_quality_baseline.json"
PrimitiveClass: TypeAlias = Literal["straight_short", "straight_long", "bend45", "bend90"]


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
    enable_jps4: bool = False
    use_indexed_heap: bool = False
    primitive_ordering: str = "library"
    heuristic_mode: str = "heading_aware"
    primitive_mode: str = "photonic"


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
        use_indexed_heap: bool,
        primitive_ordering: str,
        heuristic_mode: str,
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


def _format_mib(value: object) -> str:
    if not isinstance(value, (str, bytes, bytearray, int, float)):
        return ""
    return f"{float(value) / (1024.0 * 1024.0):.2f}"


def _route_counter_dict(route_obj: object, attr: str) -> dict[str, int]:
    values = getattr(route_obj, attr, None)
    labels: tuple[PrimitiveClass, ...] = ("straight_short", "straight_long", "bend45", "bend90")
    counters: dict[str, int] = {label: 0 for label in labels}
    if not isinstance(values, IterableABC):
        return counters
    sequence = list(values)
    for label, value in zip(labels, sequence):
        if isinstance(value, (str, bytes, bytearray, int, float)):
            counters[label] = int(value)
    return counters


def _format_primitive_counter(value: object) -> str:
    if not isinstance(value, dict):
        return ""
    labels = (
        ("straight_short", "s"),
        ("straight_long", "l"),
        ("bend45", "b45"),
        ("bend90", "b90"),
    )
    return " ".join(f"{label}:{int(value.get(key, 0)):,}" for key, label in labels)


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


def _jps4_detour_wall() -> tuple[tuple[int, int], ...]:
    return tuple((90, y) for y in range(4, 76) if y != 62)


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
        "jps4_empty_grid": AStarScenario(
            name="jps4_empty_grid",
            width=180,
            height=90,
            source=(10, 12, 0),
            target=(160, 72, 0),
            enable_simple_routes=False,
            require_target_angle=False,
            use_routing_window=False,
            primitive_mode="jps4_unit",
        ),
        "jps4_corridor": AStarScenario(
            name="jps4_corridor",
            width=180,
            height=48,
            source=(8, 24, 0),
            target=(170, 24, 0),
            static_cells=tuple(
                (x, y)
                for y in range(48)
                if y != 24
                for x in range(180)
            ),
            enable_simple_routes=False,
            require_target_angle=False,
            use_routing_window=False,
            primitive_mode="jps4_unit",
        ),
        "jps4_forced_detour": AStarScenario(
            name="jps4_forced_detour",
            width=180,
            height=90,
            source=(12, 18, 0),
            target=(165, 18, 0),
            static_cells=_jps4_detour_wall(),
            enable_simple_routes=False,
            require_target_angle=False,
            use_routing_window=False,
            primitive_mode="jps4_unit",
        ),
    }
    scenarios.update(_two_object_port_scenarios())
    return scenarios


def _default_scenario_names(catalog: dict[str, AStarScenario]) -> list[str]:
    return [name for name in catalog if not name.startswith("jps4_")]


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
    if scenario.primitive_mode == "jps4_unit":
        primitive.jps4_unit_grid = True
    elif scenario.primitive_mode == "grid4_unit":
        primitive.grid4_unit_grid = True
    elif scenario.primitive_mode != "photonic":
        raise ValueError(f"unknown primitive mode: {scenario.primitive_mode}")
    astar = rust_backend.AStarConfig(
        max_iterations=scenario.max_iterations,
        require_target_angle=scenario.require_target_angle,
        use_routing_window=scenario.use_routing_window,
        routing_window_fallback_full_grid=scenario.routing_window_fallback_full_grid,
        collect_detailed_timing=True,
        use_indexed_heap=scenario.use_indexed_heap,
        primitive_ordering=scenario.primitive_ordering,
        heuristic_mode=scenario.heuristic_mode,
    )
    astar.enable_simple_routes = scenario.enable_simple_routes
    astar.enable_jps4 = scenario.enable_jps4
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
        "stale_generation_heap_entries": _route_stat(
            last_route,
            "stale_generation_heap_entries",
        ),
        "closed_heap_entries": _route_stat(last_route, "closed_heap_entries"),
        "max_heap_size": _route_stat(last_route, "max_heap_size"),
        "dense_search_states": _route_stat(last_route, "dense_search_states"),
        "dense_search_storage_bytes": _route_stat(
            last_route,
            "dense_search_storage_bytes",
        ),
        "best_cost_updates": _route_stat(last_route, "best_cost_updates"),
        "parent_updates": _route_stat(last_route, "parent_updates"),
        "obstacle_clearance_checks": _route_stat(last_route, "obstacle_clearance_checks"),
        "window_attempts": _route_stat(last_route, "window_attempts"),
        "window_rejects": _route_stat(last_route, "window_rejects"),
        "primitive_generated_by_class": _route_counter_dict(
            last_route,
            "primitive_generated_by_class",
        ),
        "primitive_accepted_by_class": _route_counter_dict(
            last_route,
            "primitive_accepted_by_class",
        ),
        "primitive_footprint_rejects_by_class": _route_counter_dict(
            last_route,
            "primitive_footprint_rejects_by_class",
        ),
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
        "jps4_requested": bool(_route_stat(last_route, "jps4_requested")),
        "jps4_eligible": bool(_route_stat(last_route, "jps4_eligible")),
        "jps4_used": bool(_route_stat(last_route, "jps4_used")),
        "jps4_fallbacks": _route_stat(last_route, "jps4_fallbacks"),
        "jps4_fallback_reason": str(getattr(last_route, "jps4_fallback_reason", "")),
        "indexed_heap": scenario.use_indexed_heap,
        "primitive_ordering": scenario.primitive_ordering,
        "heuristic_mode": scenario.heuristic_mode,
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
        f"- Indexed heap: `{getattr(args, 'use_indexed_heap', False)}`",
        f"- Primitive ordering: `{getattr(args, 'primitive_ordering', 'library')}`",
        f"- Heuristic mode: `{getattr(args, 'heuristic_mode', 'heading_aware')}`",
        "",
        "| Scenario | Grid | Obstacles | Median s | P95 s | Expanded | Generated | Primitive gen | Primitive accepted | Heap push/pop | Dup skips | Stale gen/closed | Max heap | Dense states | Dense MiB | Cost/parent updates | Legality checks | Footprint checks | Dense build s | Neighbor s | Heap s | Legality s | Reconstruct s | JPS4 | JPS4 fallback | Full grid | Target ok | Route cells | Length um |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- | --- | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| {scenario} | {grid} | {static_cells} | {median_s} | {p95_s} | {expanded_states} | {generated_neighbors} | {primitive_generated} | {primitive_accepted} | {heap_pushes}/{heap_pops} | {duplicate_heap_skips} | {stale_generation_heap_entries}/{closed_heap_entries} | {max_heap_size} | {dense_search_states} | {dense_search_storage_mib} | {best_cost_updates}/{parent_updates} | {obstacle_clearance_checks} | {footprint_checks} | {dense_build_s} | {neighbor_generation_s} | {heap_operation_s} | {legality_check_s} | {reconstruction_s} | {jps4_status} | {jps4_fallback_reason} | {full_grid_fallback} | {target_state_ok} | {route_cells} | {route_length_um:.3f} |".format(
                scenario=row["scenario"],
                grid=row["grid"],
                static_cells=row["static_cells"],
                median_s=_format_seconds(_object_to_float(row["median_s"])),
                p95_s=_format_seconds(_object_to_float(row["p95_s"])),
                expanded_states=row["expanded_states"],
                generated_neighbors=row["generated_neighbors"],
                primitive_generated=_format_primitive_counter(
                    row.get("primitive_generated_by_class")
                ),
                primitive_accepted=_format_primitive_counter(
                    row.get("primitive_accepted_by_class")
                ),
                heap_pushes=row["heap_pushes"],
                heap_pops=row["heap_pops"],
                duplicate_heap_skips=row["duplicate_heap_skips"],
                stale_generation_heap_entries=row.get("stale_generation_heap_entries", 0),
                closed_heap_entries=row.get("closed_heap_entries", 0),
                max_heap_size=row.get("max_heap_size", 0),
                dense_search_states=row.get("dense_search_states", 0),
                dense_search_storage_mib=_format_mib(
                    row.get("dense_search_storage_bytes", 0)
                ),
                best_cost_updates=row.get("best_cost_updates", 0),
                parent_updates=row.get("parent_updates", 0),
                obstacle_clearance_checks=row["obstacle_clearance_checks"],
                footprint_checks=row["footprint_checks"],
                dense_build_s=_format_seconds(_object_to_float(row["dense_build_s"])),
                neighbor_generation_s=_format_seconds(_object_to_float(row["neighbor_generation_s"])),
                heap_operation_s=_format_seconds(_object_to_float(row["heap_operation_s"])),
                legality_check_s=_format_seconds(_object_to_float(row["legality_check_s"])),
                reconstruction_s=_format_seconds(_object_to_float(row["reconstruction_s"])),
                jps4_status=(
                    "used"
                    if row.get("jps4_used")
                    else "eligible"
                    if row.get("jps4_eligible")
                    else ("requested" if row.get("jps4_requested") else "off")
                ),
                jps4_fallback_reason=row.get("jps4_fallback_reason", ""),
                full_grid_fallback=row["full_grid_fallback"],
                target_state_ok=row.get("target_state_ok", ""),
                route_cells=row["route_cells"],
                route_length_um=row["route_length_um"],
            )
        )
    lines.append("")
    return "\n".join(lines)


def _percent_delta(new: float, old: float) -> str:
    if old == 0.0:
        return ""
    return f"{((new - old) / old) * 100.0:+.1f}%"


def _ratio_delta(new: object, old: object) -> str:
    old_value = _object_to_float(old)
    new_value = _object_to_float(new)
    if old_value == 0.0:
        return ""
    return f"{new_value / old_value:.3f}x"


def _paired_comparison_rows(
    rust_backend: RustBackend,
    scenarios: Iterable[AStarScenario],
    *,
    iterations: int,
    warmup: int,
    accelerator: str = "jps4",
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for scenario in scenarios:
        if accelerator == "heading_aware":
            baseline_scenario = replace(
                scenario,
                enable_jps4=False,
                heuristic_mode="distance",
            )
            accelerated_scenario = replace(
                scenario,
                enable_jps4=False,
                heuristic_mode="heading_aware",
            )
        else:
            baseline_primitive_mode = (
                "grid4_unit" if scenario.primitive_mode == "jps4_unit" else scenario.primitive_mode
            )
            baseline_scenario = replace(
                scenario,
                enable_jps4=False,
                primitive_mode=baseline_primitive_mode,
            )
            accelerated_scenario = replace(scenario, enable_jps4=True)
        baseline = run_scenario(
            rust_backend,
            baseline_scenario,
            iterations=iterations,
            warmup=warmup,
        )
        accelerated = run_scenario(
            rust_backend,
            accelerated_scenario,
            iterations=iterations,
            warmup=warmup,
        )
        baseline_reached = _object_to_list(baseline["reached_target"])
        accelerated_reached = _object_to_list(accelerated["reached_target"])
        rows.append(
            {
                "scenario": scenario.name,
                "accelerator": accelerator,
                "primitive_mode": scenario.primitive_mode,
                "baseline": baseline,
                "accelerated": accelerated,
                "length_delta_um": _object_to_float(accelerated["route_length_um"])
                - _object_to_float(baseline["route_length_um"]),
                "target_cell_match": accelerated_reached[:2] == baseline_reached[:2],
                "target_state_match": accelerated_reached == baseline_reached,
            }
        )
    return rows


def _markdown_paired_report(rows: Iterable[dict[str, object]], args: argparse.Namespace) -> str:
    lines = [
        "# Paired Rust A* Accelerator Comparison",
        "",
        f"- Captured: {datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"- Git revision: `{_git_rev()}`",
        f"- Python: `{platform.python_version()}`",
        f"- Iterations: `{args.iterations}`",
        f"- Warmup: `{args.warmup}`",
        f"- Paired accelerator: `{getattr(args, 'paired_accelerator', 'jps4')}`",
        "",
        "| Scenario | Primitive mode | Base mode | Accel mode | Base median s | Accel median s | Time delta | Base expanded | Accel expanded | Expanded ratio | Base generated | Accel generated | Generated ratio | Base heap | Accel heap | JPS4 | Fallback | Length delta um | Target cell |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | ---: | --- |",
    ]
    for row in rows:
        baseline = row["baseline"]
        accelerated = row["accelerated"]
        assert isinstance(baseline, dict)
        assert isinstance(accelerated, dict)
        base_heap = f"{baseline['heap_pushes']}/{baseline['heap_pops']}"
        accel_heap = f"{accelerated['heap_pushes']}/{accelerated['heap_pops']}"
        jps4_status = (
            "used"
            if accelerated.get("jps4_used")
            else "eligible"
            if accelerated.get("jps4_eligible")
            else ("requested" if accelerated.get("jps4_requested") else "off")
        )
        lines.append(
            "| {scenario} | {primitive_mode} | {base_mode} | {accel_mode} | {base_median} | {accel_median} | {time_delta} | {base_expanded} | {accel_expanded} | {expanded_ratio} | {base_generated} | {accel_generated} | {generated_ratio} | {base_heap} | {accel_heap} | {jps4_status} | {fallback} | {length_delta:.3f} | {target_cell_match} |".format(
                scenario=row["scenario"],
                primitive_mode=row["primitive_mode"],
                base_mode=str(baseline.get("heuristic_mode", "")),
                accel_mode=str(accelerated.get("heuristic_mode", "")),
                base_median=_format_seconds(_object_to_float(baseline["median_s"])),
                accel_median=_format_seconds(_object_to_float(accelerated["median_s"])),
                time_delta=_percent_delta(
                    _object_to_float(accelerated["median_s"]),
                    _object_to_float(baseline["median_s"]),
                ),
                base_expanded=baseline["expanded_states"],
                accel_expanded=accelerated["expanded_states"],
                expanded_ratio=_ratio_delta(
                    accelerated["expanded_states"],
                    baseline["expanded_states"],
                ),
                base_generated=baseline["generated_neighbors"],
                accel_generated=accelerated["generated_neighbors"],
                generated_ratio=_ratio_delta(
                    accelerated["generated_neighbors"],
                    baseline["generated_neighbors"],
                ),
                base_heap=base_heap,
                accel_heap=accel_heap,
                jps4_status=jps4_status,
                fallback=accelerated.get("jps4_fallback_reason", ""),
                length_delta=_object_to_float(row["length_delta_um"]),
                target_cell_match=row["target_cell_match"],
            )
        )
    lines.append("")
    return "\n".join(lines)


def _parse_args() -> argparse.Namespace:
    catalog = _scenario_catalog()
    default_scenarios = _default_scenario_names(catalog)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenarios",
        nargs="*",
        default=default_scenarios,
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
    parser.add_argument(
        "--enable-jps4",
        action="store_true",
        help=(
            "Request the experimental Manhattan JPS4 accelerator for every "
            "scenario. Pass 3A reports eligibility and falls back to baseline A*."
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
        "--paired-comparison",
        action="store_true",
        help=(
            "Run each selected scenario twice, baseline and accelerator-requested, "
            "and report side-by-side deltas."
        ),
    )
    parser.add_argument(
        "--paired-accelerator",
        choices=("jps4", "heading_aware"),
        default="jps4",
        help="Accelerator to compare when --paired-comparison is set.",
    )
    args = parser.parse_args()
    if args.paired_comparison and args.scenarios == default_scenarios:
        if args.paired_accelerator == "heading_aware":
            args.scenarios = ["wall_gap_astar", "slalom_astar", "full_grid_fallback"]
        else:
            args.scenarios = sorted(name for name in catalog if name.startswith("jps4_"))
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
    if args.paired_comparison:
        paired_rows = _paired_comparison_rows(
            rust_backend,
            [catalog[name] for name in args.scenarios],
            iterations=args.iterations,
            warmup=args.warmup,
            accelerator=args.paired_accelerator,
        )
        report = _markdown_paired_report(paired_rows, args)
        print(report)
        if args.check_baseline:
            print(
                "Baseline quality check is not applied to paired comparison reports.",
                file=sys.stderr,
            )
        if args.output is not None:
            _write_text(args.output, report)
        if args.json_output is not None:
            _write_text(args.json_output, json.dumps(paired_rows, indent=2, sort_keys=True) + "\n")
        return 0

    if args.enable_jps4:
        catalog = {
            name: replace(scenario, enable_jps4=True)
            for name, scenario in catalog.items()
        }
    if args.use_indexed_heap:
        catalog = {
            name: replace(scenario, use_indexed_heap=True)
            for name, scenario in catalog.items()
        }
    if args.primitive_ordering != "library":
        catalog = {
            name: replace(scenario, primitive_ordering=args.primitive_ordering)
            for name, scenario in catalog.items()
        }
    if args.heuristic_mode != "heading_aware":
        catalog = {
            name: replace(scenario, heuristic_mode=args.heuristic_mode)
            for name, scenario in catalog.items()
        }
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
