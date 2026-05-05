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

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import route_nets_rust


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
    debug_timing: bool = False,
    show_klayout: bool = False,
) -> Component:
    """Execute the routing flow for a given benchmark.

    Parameters:
        benchmark_name: Name of the benchmark to run (e.g., 'TOY').
        debug_svgs: If True, generate debug SVGs into the build/ directory.
        debug_timing: If True, print timing information for each stage.
        show_klayout: If True, open the final routed layout in KLayout via
                      `Component.show()`.

    Returns:
        The routed layout component.
    """
    print(f"\n{'='*60}")
    print(f"Routing Flow: {benchmark_name}")
    print(f"{'='*60}")

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
    t_route_start = 0.0
    if debug_timing:
        t_route_start = time.perf_counter()
    routed_layout, debug_artifacts = route_nets_rust(
        unrouted_layout,
        schematic,
        debug_dir=debug_dir,
        debug_prefix=benchmark_name.lower(),
    )
    if debug_timing:
        t_route_end = time.perf_counter()
        print(f"      - Routing time: {t_route_end - t_route_start:.4f} s")
    print(f"      ✓ Routed layout generated: {routed_layout.name}")

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
    run_routing_flow("TOY", debug_svgs=False, debug_timing=True, show_klayout=False)
