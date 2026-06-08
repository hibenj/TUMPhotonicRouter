"""Photonic routing flow orchestrator.

This module orchestrates the photonic routing flow:
1. Load benchmark (schematic)
2. Translate schematic to unrouted layout
3. Route connections using the Rust router backend
4. [Future] Generate final routed layout
"""

from dataclasses import dataclass, field
import importlib
import time
from pathlib import Path
import webbrowser
from typing import Any

from benchmark_metadata import load_benchmark_metadata
from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import (
    route_match_and_realize,
)
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig


@dataclass
class RoutingFlowStats:
    """Compatibility container for legacy routing-flow timing/stat collection."""

    benchmark_name: str | None = None
    total_time_s: float = 0.0
    instance_count: int = 0
    net_count: int = 0
    static_grid_width: int | None = None
    static_grid_height: int | None = None
    raw_blocked_cells: int | None = None
    blocked_cells: int | None = None
    port_open_cells: int = 0
    step_times_s: dict[str, float] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "benchmark_name": self.benchmark_name,
            "total_time_s": self.total_time_s,
            "instance_count": self.instance_count,
            "net_count": self.net_count,
            "static_grid_width": self.static_grid_width,
            "static_grid_height": self.static_grid_height,
            "raw_blocked_cells": self.raw_blocked_cells,
            "blocked_cells": self.blocked_cells,
            "port_open_cells": self.port_open_cells,
            "step_times_s": dict(self.step_times_s),
        }


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


def run_routing_flow(
    benchmark_name: str,
    *,
    debug_svgs: bool = False,
    show_unrouted: bool | None = None,
    show_routed: bool | None = None,
    show_debug_svgs: bool | None = None,
    show_static_obstacles_svg: bool | None = None,
    debug_timing: bool = False,
    debug_meanders: bool = False,
    show_klayout: bool = False,
    enable_path_length_matching: bool = False,
    allow_45_degree_turns: bool = True,
    max_iterations: int = 500_000,
    static_obstacle_config: StaticObstacleMapConfig | None = None,
    stats: RoutingFlowStats | None = None,
) -> Component:
    """Execute the routing flow for a given benchmark.

    Parameters:
        benchmark_name: Name of the benchmark to run (e.g., 'TOY').
        debug_svgs: If True, generate debug SVGs into the build/ directory.
        debug_timing: If True, print timing information for each stage.
        debug_meanders: If True, print verbose path-length and meander
                      insertion details when path-length matching is enabled.
        show_klayout: If True, open the final routed layout in KLayout via
                      `Component.show()`.
        show_unrouted: Legacy alias. Currently unused (kept for compatibility).
        show_routed: Legacy alias for `show_klayout`.
        show_debug_svgs: Legacy alias for `debug_svgs`.
        show_static_obstacles_svg: Legacy alias for enabling debug SVG output.
        stats: Optional legacy stats collector. If provided, step metrics are
               populated in-place.
        enable_path_length_matching: If True, run post-route path-length
                      analysis and compute per-edge missing lengths.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        max_iterations: Maximum A* state expansions per route attempt.
        static_obstacle_config: Optional obstacle builder config. If omitted,
            strict bounding-box static obstacles are used.

    Returns:
        The routed layout component.
    """
    if show_routed is not None:
        show_klayout = bool(show_routed)
    if show_debug_svgs is not None:
        debug_svgs = bool(show_debug_svgs)
    if show_static_obstacles_svg is not None:
        debug_svgs = bool(show_static_obstacles_svg)
    if show_unrouted is not None:
        # Historical argument kept for compatibility.
        pass

    print(f"\n{'='*60}")
    print(f"Routing Flow: {benchmark_name}")
    print(f"{'='*60}")

    if stats is not None:
        stats.benchmark_name = benchmark_name

    route_static_obstacle_config = static_obstacle_config or StaticObstacleMapConfig(
        obstacle_mode="bounding_boxes",
        clear_port_open_cells_from_static=False,
    )

    t_flow_start = time.perf_counter()

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
    step_load_start = time.perf_counter()
    print(f"\n[1/3] Loading benchmark: {benchmark_name}...")
    schematic = load_benchmark(benchmark_name)
    step_load_end = time.perf_counter()
    if stats is not None:
        stats.instance_count = len(schematic.netlist.instances)
        stats.net_count = len(schematic.netlist.routes)
        stats.step_times_s["load_benchmark"] = step_load_end - step_load_start
    print("      ✓ Schematic loaded")
    print(f"      - Instances: {list(schematic.netlist.instances.keys())}")
    print(f"      - Placements: {list(schematic.placements.keys())}")

    # Step 2: Translate schematic to layout
    step_layout_start = time.perf_counter()
    print("\n[2/3] Translating schematic to layout...")
    unrouted_layout = layout_from_schematic(schematic)
    step_layout_end = time.perf_counter()
    if stats is not None:
        stats.step_times_s["layout_from_schematic"] = step_layout_end - step_layout_start
    print(f"      ✓ Layout generated: {unrouted_layout.name}")
    bbox = unrouted_layout.bbox
    if callable(bbox):
        bbox = bbox()
    print(f"      - Bounding box: {bbox}")
    if debug_timing:
        print(f"      - Translation time: {step_layout_end - step_layout_start:.4f} s")

    # Step 3: Route nets with Rust backend
    print("\n[3/3] Routing nets with Rust backend...")
    if stats is not None:
        stats.step_times_s["build_static_obstacle_map"] = 0.0
        stats.step_times_s["baseline_gdsfactory_routing"] = 0.0
    debug_dir = Path("build") if debug_svgs else None
    metadata = load_benchmark_metadata(benchmark_name, schematic=schematic)
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
            obstacle_config=route_static_obstacle_config,
        )
    except Exception:
        print("      ✗ Routing failed.")
        _report_partial_debug_artifacts()
        raise
    routed_layout = route_result.routed_layout
    debug_artifacts = route_result.debug_artifacts
    t_route_end = time.perf_counter()
    if stats is not None:
        route_time = t_route_end - t_route_start
        stats.step_times_s["baseline_gdsfactory_routing"] = route_time
        if "build_static_obstacle_map" not in stats.step_times_s:
            stats.step_times_s["build_static_obstacle_map"] = 0.0
        if debug_artifacts.realization_grid_spec is not None:
            width, height, *_ = debug_artifacts.realization_grid_spec
            stats.static_grid_width = int(width)
            stats.static_grid_height = int(height)
        blocked_count = len(debug_artifacts.static_blocked_cells)
        if blocked_count == 0:
            blocked_count = int(getattr(debug_artifacts, "static_obstacle_count", 0) or 0)
        if blocked_count > 0:
            stats.blocked_cells = blocked_count
            stats.raw_blocked_cells = blocked_count
            if isinstance(stats.static_grid_width, int) and isinstance(stats.static_grid_height, int):
                grid_area = max(1, stats.static_grid_width * stats.static_grid_height)
                stats.port_open_cells = max(1, grid_area - blocked_count)
            else:
                stats.port_open_cells = max(1, blocked_count)
    if debug_timing:
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
        if debug_meanders and route_result.path_length_analysis_info is not None:
            node_timings = route_result.path_length_analysis_info.get("node_timings_um", {})
            if isinstance(node_timings, dict):
                for node_name, node_info in node_timings.items():
                    if not isinstance(node_info, dict):
                        continue
                    incoming = node_info.get("incoming_edges")
                    if incoming is None:
                        incoming = []
                    print(
                        f"        • node={node_name}, "
                        f"type={node_info.get('node_type')}, "
                        f"internal={float(node_info.get('internal_delay_um', 0.0)):.3f}um, "
                        f"input={float(node_info.get('input_arrival_um', 0.0)):.3f}um, "
                        f"output={float(node_info.get('output_arrival_um', 0.0)):.3f}um"
                    )
                    for incoming_entry in incoming:
                        if not isinstance(incoming_entry, dict):
                            continue
                        edge = incoming_entry.get("edge", {})
                        edge_name = (
                            f"{edge.get('source', {}).get('instance', '?')}->"
                            f"{edge.get('target', {}).get('instance', '?')} "
                            f"({edge.get('net_name', '?')})"
                        )
                        print(
                            "          - "
                            f"{edge_name}: edge_len={float(incoming_entry.get('routed_length_um', 0.0)):.3f}um, "
                            f"edge_arrival={float(incoming_entry.get('edge_arrival_um', 0.0)):.3f}um, "
                            f"missing={float(incoming_entry.get('missing_length_um', 0.0)):.3f}um"
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
            if debug_meanders:
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
    else:
        print("      - Write GDS...")
        routed_layout.write_gds(f"build/routed_{benchmark_name}.gds")

    if debug_timing:
        t_end = time.perf_counter()
        total = t_end - t_flow_start
        print(f"\nTiming summary for {benchmark_name}:\n  total: {total:.4f} s")
    if stats is not None:
        total = time.perf_counter() - t_flow_start
        stats.total_time_s = float(total)

    print(f"\n{'='*60}\n")

    return routed_layout


if __name__ == "__main__":
    run_routing_flow("mmi_heater_8x4",
                     debug_svgs=False,
                     debug_timing=True,
                     debug_meanders=True,
                     show_klayout=False,
                     allow_45_degree_turns=False,
                     enable_path_length_matching=True,
                     static_obstacle_config=StaticObstacleMapConfig(
                         obstacle_mode="bounding_boxes",
                         clear_port_open_cells_from_static=False,  # strict net-local openings
                     ),)
