"""Photonic routing flow orchestrator.

This module orchestrates the photonic routing flow:
1. Load benchmark (schematic)
2. Translate schematic to unrouted layout
3. Route connections using the Rust router backend
4. [Future] Generate final routed layout
"""

import importlib
import time
from pathlib import Path
import webbrowser
from typing import Any

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import (
    route_match_and_realize,
)


def load_benchmark(benchmark_name: str) -> Schematic:
    """Load a benchmark schematic from the benchmarks directory.

    Parameters:
        benchmark_name: The name of the benchmark module (e.g., 'TOY').
                       The module must have a `build_schematic()` function.

    Returns:
        A gdsfactory Schematic object.

    Raises:
        ModuleNotFoundError: If the benchmark module is not found.
        AttributeError: If the benchmark doesn't have a `build_schematic()` function.
    """
    try:
        benchmark_module = importlib.import_module(f"benchmarks.{benchmark_name}")
        schematic = benchmark_module.build_schematic()

        if not isinstance(schematic, Schematic):
            raise TypeError(
                f"Expected Schematic from {benchmark_name}.build_schematic(), "
                f"got {type(schematic)}"
            )

        return schematic
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            f"Benchmark '{benchmark_name}' not found in benchmarks/ directory"
        ) from e
    except AttributeError as e:
        raise AttributeError(
            f"Benchmark '{benchmark_name}' must have a 'build_schematic()' function"
        ) from e


def load_benchmark_metadata(benchmark_name: str) -> dict[str, Any]:
    """Load optional benchmark metadata used by path-length analysis."""
    benchmark_module = importlib.import_module(f"benchmarks.{benchmark_name}")
    return {
        "node_types": getattr(benchmark_module, "NODE_TYPES", {}),
        "internal_delays_um": getattr(benchmark_module, "INTERNAL_DELAYS_UM", {}),
    }


def run_routing_flow(
    benchmark_name: str,
    *,
    debug_svgs: bool = False,
    debug_timing: bool = False,
    show_klayout: bool = False,
    enable_path_length_matching: bool = False,
    allow_45_degree_turns: bool = True,
    max_iterations: int = 500_000,
) -> Component:
    """Execute the routing flow for a given benchmark.

    Parameters:
        benchmark_name: Name of the benchmark to run (e.g., 'TOY').
        debug_svgs: If True, generate debug SVGs into the build/ directory.
        debug_timing: If True, print timing information for each stage.
        show_klayout: If True, open the final routed layout in KLayout via
                      `Component.show()`.
        enable_path_length_matching: If True, run post-route path-length
                      analysis and compute per-edge missing lengths.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        max_iterations: Maximum A* state expansions per route attempt.

    Returns:
        The routed layout component.
    """
    print(f"\n{'='*60}")
    print(f"Routing Flow: {benchmark_name}")
    print(f"{'='*60}")

    if debug_svgs:
        prefix = benchmark_name.lower()
        for pattern in (
            f"build/static_obstacles/{prefix}_*.svg",
            f"build/routes/{prefix}_*.svg",
            f"build/routes/{prefix}_*_FAILED.txt",
        ):
            for path in Path(".").glob(pattern):
                try:
                    path.unlink()
                except OSError:
                    pass

    def _report_partial_debug_artifacts() -> None:
        if not debug_svgs:
            return
        prefix = benchmark_name.lower()
        build_dir = Path("build")
        obstacle_dir = build_dir / "static_obstacles"
        routes_dir = build_dir / "routes"
        obstacle_svgs = sorted(obstacle_dir.glob(f"{prefix}_*.svg")) if obstacle_dir.exists() else []
        route_svgs = sorted(routes_dir.glob(f"{prefix}_*.svg")) if routes_dir.exists() else []
        failed_logs = sorted(routes_dir.glob(f"{prefix}_*_FAILED.txt")) if routes_dir.exists() else []

        print("      - Partial debug artifacts:")
        print(f"        static obstacle SVGs: {len(obstacle_svgs)}")
        print(f"        route SVGs: {len(route_svgs)}")
        print(f"        failure logs: {len(failed_logs)}")
        for failed_log in failed_logs:
            print(f"        failure log: {failed_log}")

        try:
            for svg_path in obstacle_svgs:
                webbrowser.open_new_tab(svg_path.resolve().as_uri())
            for svg_path in route_svgs:
                webbrowser.open_new_tab(svg_path.resolve().as_uri())
        except Exception as e:
            print(f"      - Warning: failed to open partial SVGs automatically: {e}")

    # Step 1: Load benchmark
    t0 = 0.0
    if debug_timing:
        t0 = time.perf_counter()
    print(f"\n[1/3] Loading benchmark: {benchmark_name}...")
    schematic = load_benchmark(benchmark_name)
    print("      ✓ Schematic loaded")
    print(f"      - Instances: {list(schematic.netlist.instances.keys())}")
    print(f"      - Placements: {list(schematic.placements.keys())}")

    # Step 2: Translate schematic to layout
    print("\n[2/3] Translating schematic to layout...")
    unrouted_layout = layout_from_schematic(schematic)
    print(f"      ✓ Layout generated: {unrouted_layout.name}")
    print(f"      - Bounding box: {unrouted_layout.bbox}")
    if debug_timing:
        t1 = time.perf_counter()
        print(f"      - Translation time: {t1 - t0:.4f} s")

    # Step 3: Route nets with Rust backend
    print("\n[3/3] Routing nets with Rust backend...")
    debug_dir = Path("build") if debug_svgs else None
    metadata = load_benchmark_metadata(benchmark_name)
    t_route_start = 0.0
    if debug_timing:
        t_route_start = time.perf_counter()
    try:
        route_result = route_match_and_realize(
            unrouted_layout,
            schematic,
            enable_path_length_matching=enable_path_length_matching,
            node_types=metadata.get("node_types"),
            internal_delays_um=metadata.get("internal_delays_um"),
            debug_dir=debug_dir,
            debug_prefix=benchmark_name.lower(),
            debug_timing=debug_timing,
            allow_45_degree_turns=allow_45_degree_turns,
            max_iterations=max_iterations,
        )
    except Exception:
        print("      ✗ Routing failed.")
        _report_partial_debug_artifacts()
        raise
    routed_layout = route_result.routed_layout
    debug_artifacts = route_result.debug_artifacts
    if debug_timing:
        t_route_end = time.perf_counter()
        print(f"      - Routing time: {t_route_end - t_route_start:.4f} s")
    print(f"      ✓ Routed layout generated: {routed_layout.name}")

    if route_result.path_length_analysis_info is not None:
        meander_report_info = getattr(route_result, "meander_insertion_report_info", None)
        routed_layout.info["path_length_analysis"] = route_result.path_length_analysis_info
        routed_layout.info["meander_requirements"] = (
            route_result.meander_requirements_info or []
        )
        if meander_report_info is not None:
            routed_layout.info["meander_insertion_report"] = (
                meander_report_info
            )
        print(
            "      - Path-length matching: "
            f"{len(routed_layout.info['meander_requirements'])} edge(s) require extra length"
        )
        if meander_report_info is not None:
            report = meander_report_info
            total_requested = float(report.get("total_requested_extra_length_um", 0.0))
            total_inserted = float(report.get("total_inserted_extra_length_um", 0.0))
            unmatched = float(report.get("unmatched_length_um", 0.0))
            print(
                "      - Meander insertion: "
                f"requested={total_requested:.3f}um, "
                f"inserted={total_inserted:.3f}um, "
                f"unmatched={unmatched:.3f}um"
            )
            for entry in report.get("results", []):
                edge = entry.get("edge", {})
                net_name = edge.get("net_name", "<unknown>")
                status = entry.get("status", "<unknown>")
                reason = entry.get("reason", "")
                req = float(entry.get("requested_extra_length_um", 0.0))
                ins = float(entry.get("inserted_extra_length_um", 0.0))
                unmatched = float(entry.get("unmatched_length_um", max(0.0, req - ins)))
                planning_mode = entry.get("planning_mode", None)
                effective_radius = entry.get("effective_bend_radius_um", None)
                primitive_radius = entry.get("primitive_bend_radius_um", None)
                selected_box = entry.get("selected_box", None)
                selected_grid_rect = entry.get("selected_grid_rect", None)
                bumps = entry.get("bumps", None)
                side = entry.get("side", None)
                reserved_cells_count = entry.get("reserved_cells_count", None)
                print(
                    f"        • {net_name}: status={status}, requested={req:.3f}um, "
                    f"inserted={ins:.3f}um, unmatched={unmatched:.3f}um, "
                    f"planning_mode={planning_mode}, side={side}, bumps={bumps}, "
                    f"effective_bend_radius_um={effective_radius}, "
                    f"primitive_bend_radius_um={primitive_radius}, "
                    f"selected_box={selected_box}, selected_grid_rect={selected_grid_rect}, "
                    f"reserved_cells_count={reserved_cells_count}, reason={reason}"
                )

    if debug_svgs:
        if debug_artifacts.obstacle_svg is not None:
            print(f"      - Obstacle SVG: {debug_artifacts.obstacle_svg}")
        if debug_artifacts.route_svgs:
            print(f"      - Route SVGs: {len(debug_artifacts.route_svgs)} files")

        # Open generated SVGs in the default browser/viewer so the user can inspect them.
        try:
            if debug_artifacts.obstacle_svg is not None:
                obs_path = Path(debug_artifacts.obstacle_svg)
                if obs_path.exists():
                    webbrowser.open_new_tab(obs_path.resolve().as_uri())
            for svg in debug_artifacts.route_svgs or []:
                svg_path = Path(svg)
                if svg_path.exists():
                    webbrowser.open_new_tab(svg_path.resolve().as_uri())
        except Exception as e:
            print(f"      - Warning: failed to open SVGs automatically: {e}")

    # Optionally show the final routed layout in KLayout
    if show_klayout:
        try:
            print("      - Opening routed layout in KLayout...")
            routed_layout.show()
        except Exception as e:
            print(f"      - Warning: failed to open layout in KLayout: {e}")

    if debug_timing:
        t_end = time.perf_counter()
        total = t_end - t0
        print(f"\nTiming summary for {benchmark_name}:\n  total: {total:.4f} s")

    print(f"\n{'='*60}\n")

    return routed_layout


if __name__ == "__main__":
    run_routing_flow("TOY",
                     debug_svgs=False,
                     debug_timing=True,
                     show_klayout=True,
                     allow_45_degree_turns=False,
                     enable_path_length_matching=True)
