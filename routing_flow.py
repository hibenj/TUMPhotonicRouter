"""Photonic routing flow orchestrator.

This module orchestrates the photonic routing flow:
1. Load benchmark (schematic)
2. Translate schematic to unrouted layout
3. Route connections using the Rust router backend
4. Optionally route heater electrical metal
5. Generate final routed layout
"""

import argparse
import importlib
import os
import sys
import time
from pathlib import Path

if "MPLCONFIGDIR" not in os.environ:
    _default_mpl_config_dir = Path(__file__).resolve().parent / "build" / "mpl"
    try:
        _default_mpl_config_dir.mkdir(parents=True, exist_ok=True)
    except OSError:
        pass
    os.environ["MPLCONFIGDIR"] = str(_default_mpl_config_dir)

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from translation.crossing_modes import CROSSING_MODES, is_guided_mode
from translation.route_order import NET_ORDERS
from translation.electrical import ElectricalRoutingConfig, ElectricalRoutingResult
from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import RipupRerouteConfig
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow_config import (
    DebugSvgSelector,
    SCRIPT_ALLOW_45_DEGREE_TURNS,
    SCRIPT_ATTEMPT_DIAGNOSTICS,
    SCRIPT_BENCHMARK,
    SCRIPT_BEND_RADIUS_UM,
    SCRIPT_CHIP_ADD_X_UM,
    SCRIPT_CHIP_ADD_Y_UM,
    SCRIPT_CLEAR_PORT_OPEN_CELLS_FROM_STATIC,
    SCRIPT_CROSSING_MODE,
    SCRIPT_DEBUG_MEANDERS,
    SCRIPT_DEBUG_SVGS,
    SCRIPT_DEBUG_TIMING,
    SCRIPT_ELECTRICAL_BUS_WIDTH_UM,
    SCRIPT_ELECTRICAL_GRID_PITCH_UM,
    SCRIPT_ELECTRICAL_OBSTACLE_CLEARANCE_UM,
    SCRIPT_ELECTRICAL_PAD_PITCH_UM,
    SCRIPT_ELECTRICAL_PAD_SIDE,
    SCRIPT_ELECTRICAL_TERMINAL_CONTACT_WIDTH_UM,
    SCRIPT_ELECTRICAL_WIRE_WIDTH_UM,
    SCRIPT_ENABLE_CROSSINGS,
    SCRIPT_ENABLE_ELECTRICAL_ROUTING,
    SCRIPT_ENABLE_PATH_LENGTH_MATCHING,
    SCRIPT_ENABLE_RIPUP_REROUTE,
    SCRIPT_FANOUT_ACCESS_MODE,
    SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
    SCRIPT_GRID_SIZE_UM,
    SCRIPT_HEATER_CLEARANCE_UM,
    SCRIPT_INCLUDE_HEATER_OBSTACLES,
    SCRIPT_MAX_ITERATIONS,
    SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    SCRIPT_OBSTACLE_CLEARANCE_UM,
    SCRIPT_OBSTACLE_MODE,
    SCRIPT_PATH_LENGTH_MATCH_OUTPUTS,
    SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
    SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
    SCRIPT_RIPUP_HISTORY_INCREMENT,
    SCRIPT_RIPUP_HISTORY_WEIGHT,
    SCRIPT_RIPUP_MAX_ROUNDS,
    SCRIPT_RIPUP_MAX_VICTIMS,
    SCRIPT_ROUTING_WINDOW_SCALE,
    SCRIPT_SHOW_KLAYOUT,
    SCRIPT_VERBOSE_ROUTES,
    SCRIPT_WAVEGUIDE_CLEARANCE_UM,
    build_optical_routing_stage_config,
    build_static_obstacle_config,
    debug_artifact_routing_options,
    parse_debug_svg_selector,
    resolve_legacy_display_options,
)
from routing_flow_electrical import run_electrical_routing_step
from routing_flow_optical import run_photonic_routing_stage
from routing_flow_plm import attach_and_report_path_length_matching
from routing_flow_reporting import (
    _format_debug_route_indices,
    cleanup_debug_artifacts,
    report_and_open_debug_svgs,
    write_or_show_routed_layout,
)
from routing_flow_stats import (
    RoutingFlowStats,
    populate_route_stats,
    record_initial_route_stats,
)
from routing_flow_verification import (
    _verification_status_metadata,
    verify_and_attach_photonic_reports,
)

# Compatibility alias: this was a private module-level function before the
# 2026-08-11 routing-flow readability refactor moved it (and made it public)
# in routing_flow_config.py; kept here under its old name for existing
# imports (see tests/test_routing_flow_stats.py).
_parse_debug_svg_selector = parse_debug_svg_selector


def _parse_bool_flag(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"expected boolean value, got {value!r}")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the photonic routing flow for a benchmark.")
    parser.add_argument(
        "benchmark",
        nargs="?",
        default=SCRIPT_BENCHMARK,
        help=f"Benchmark module name from benchmarks/ (default: {SCRIPT_BENCHMARK}).",
    )
    parser.add_argument(
        "--debug-svgs",
        default=SCRIPT_DEBUG_SVGS,
        metavar="SELECTOR",
        help=(
            "Generate debug SVGs for the selected routes only: 1-based route "
            "selectors like '5', '5-10', or '2,5-10'. Pass 'all' explicitly for "
            "every route (large benchmarks then write hundreds of multi-MB "
            "files; a bare --debug-svgs is deliberately not accepted)."
        ),
    )
    parser.add_argument(
        "--debug-timing",
        type=_parse_bool_flag,
        default=SCRIPT_DEBUG_TIMING,
        metavar="BOOL",
        help=f"Print timing details (default: {str(SCRIPT_DEBUG_TIMING).lower()}).",
    )
    parser.add_argument(
        "--debug-stop-after-route",
        type=int,
        default=None,
        metavar="N",
        help=(
            "Debug helper: build full-netlist obstacle/crossing context, but "
            "route only through 1-based route index N."
        ),
    )
    parser.add_argument(
        "--debug-meanders",
        action="store_true",
        default=SCRIPT_DEBUG_MEANDERS,
        help="Print verbose path-length and meander details.",
    )
    parser.add_argument(
        "--verbose-routes",
        action="store_true",
        default=SCRIPT_VERBOSE_ROUTES,
        help="Print per-net routing details, including simple vs A* route type.",
    )
    parser.add_argument(
        "--show-klayout",
        action="store_true",
        default=SCRIPT_SHOW_KLAYOUT,
        help="Open the routed layout in KLayout instead of only writing GDS.",
    )
    parser.add_argument(
        "--allow-45-degree-turns",
        type=_parse_bool_flag,
        default=SCRIPT_ALLOW_45_DEGREE_TURNS,
        metavar="BOOL",
        help=(
            "Allow 45-degree routing primitives "
            f"(default: {str(SCRIPT_ALLOW_45_DEGREE_TURNS).lower()})."
        ),
    )
    parser.add_argument(
        "--bend-radius-um",
        type=float,
        default=SCRIPT_BEND_RADIUS_UM,
        metavar="UM",
        help=(
            "Minimum optical waveguide bend radius in micrometers. The router "
            "rounds this up to an integer number of grid cells "
            f"(default: {SCRIPT_BEND_RADIUS_UM})."
        ),
    )
    parser.add_argument(
        "--path-length-matching",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_PATH_LENGTH_MATCHING,
        metavar="BOOL",
        help=(
            "Enable path-length matching analysis and realization "
            f"(default: {str(SCRIPT_ENABLE_PATH_LENGTH_MATCHING).lower()})."
        ),
    )
    parser.add_argument(
        "--path-length-match-outputs",
        type=_parse_bool_flag,
        default=SCRIPT_PATH_LENGTH_MATCH_OUTPUTS,
        metavar="BOOL",
        help=(
            "Also require all output nodes to have equal arrival delay when "
            "path-length matching is enabled "
            f"(default: {str(SCRIPT_PATH_LENGTH_MATCH_OUTPUTS).lower()})."
        ),
    )
    parser.add_argument(
        "--path-length-meander-height-um",
        type=float,
        default=SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
        metavar="UM",
        help=(
            "Maximum meander height used for path-length matching "
            f"(default: {SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM})."
        ),
    )
    parser.add_argument(
        "--crossings",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_CROSSINGS,
        metavar="BOOL",
        help=(
            "Build and pass topology-derived crossing constraints to the Rust "
            "router (default: "
            f"{str(SCRIPT_ENABLE_CROSSINGS).lower()})."
        ),
    )
    parser.add_argument(
        "--preplaced-crossing-grids",
        type=_parse_bool_flag,
        default=False,
        metavar="BOOL",
        help=(
            "Place topology-derived, pre-wired crossing grids into every "
            "interstage layer before routing and route only crossing-free stubs "
            "to/from them; forces crossings off in the router. Benes benchmarks "
            "only (default: false)."
        ),
    )
    parser.add_argument(
        "--crossing-mode",
        choices=CROSSING_MODES,
        default=SCRIPT_CROSSING_MODE,
        help=(
            "Crossing search mode. 'window' preserves the existing expected-partner "
            "window search; 'collision' enables LiDAR-style collision-driven "
            "crossing legalization constrained by topology; 'lidar-pure' enables "
            "collision-driven crossing legalization without topology pair permissions "
            "(the baseline); 'lidar-guided' is contribution 1: lidar-pure mechanics "
            "plus the precomputed topology crossings as soft search guidance "
            "(planned crossings cost PHOTONIC_ROUTER_PLANNED_CROSSING_SEARCH_LOSS_UM, "
            "default 0, in the search; unplanned ones keep the collision price) "
            f"(default: {SCRIPT_CROSSING_MODE})."
        ),
    )
    parser.add_argument(
        "--min-straight-cells-per-crossing",
        type=int,
        default=SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
        metavar="N",
        help=(
            "Minimum straight access cells before and after each crossing "
            f"(default: {SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING})."
        ),
    )
    parser.add_argument(
        "--foreign-port-keepout-cells",
        type=int,
        default=SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
        metavar="N",
        help=(
            "Additional larger port keepout in grid cells. Active endpoint "
            "ports and dense multi-port fanout instances can open it; "
            "unrelated nets are kept out "
            f"(default: {SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS})."
        ),
    )
    parser.add_argument(
        "--fanout-access-mode",
        choices=(
            "legacy-runway",
            "staggered",
            "off",
            "static-stubs",
            "anchor-pre-spread",
            "virtual-ports",
        ),
        default=SCRIPT_FANOUT_ACCESS_MODE,
        help=(
            "Dense multi-port access strategy. 'legacy-runway' keeps the "
            "existing staggered source-port reservations; 'off' disables "
            "dense fanout reservations; 'static-stubs' pre-routes fixed "
            "static breakout stubs and routes from virtual anchor ports "
            f"(default: {SCRIPT_FANOUT_ACCESS_MODE})."
        ),
    )
    parser.add_argument(
        "--proactive-congestion-weight",
        type=float,
        default=SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
        metavar="COST",
        help=(
            "Soft A* cost per nearby blocked cell beside straight moves "
            f"(default: {SCRIPT_PROACTIVE_CONGESTION_WEIGHT})."
        ),
    )
    parser.add_argument(
        "--proactive-congestion-radius-cells",
        type=int,
        default=SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
        metavar="N",
        help=(
            "Sideways grid radius used for proactive congestion counting "
            f"(default: {SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS})."
        ),
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=SCRIPT_MAX_ITERATIONS,
        metavar="N",
        help=f"Maximum A* state expansions per route attempt (default: {SCRIPT_MAX_ITERATIONS}).",
    )
    parser.add_argument(
        "--routing-window-scale",
        type=float,
        default=SCRIPT_ROUTING_WINDOW_SCALE,
        metavar="SCALE",
        help=(
            "Routing-window margin scale relative to source-target span "
            f"(default: {SCRIPT_ROUTING_WINDOW_SCALE})."
        ),
    )
    parser.add_argument(
        "--include-heater-obstacles",
        type=_parse_bool_flag,
        default=SCRIPT_INCLUDE_HEATER_OBSTACLES,
        metavar="BOOL",
        help=(
            "Include heater/metal layers as routing obstacles "
            f"(default: {str(SCRIPT_INCLUDE_HEATER_OBSTACLES).lower()})."
        ),
    )
    parser.add_argument(
        "--obstacle-mode",
        default=SCRIPT_OBSTACLE_MODE,
        help="Static obstacle mode passed to StaticObstacleMapConfig.",
    )
    parser.add_argument(
        "--grid-size-um",
        type=float,
        default=SCRIPT_GRID_SIZE_UM,
        metavar="UM",
        help=f"Optical routing grid resolution in micrometers (default: {SCRIPT_GRID_SIZE_UM}).",
    )
    parser.add_argument(
        "--waveguide-clearance-um",
        type=float,
        default=SCRIPT_WAVEGUIDE_CLEARANCE_UM,
        metavar="UM",
        help=(
            "Clearance in micrometers for existing optical/waveguide obstacles "
            f"(default: {SCRIPT_WAVEGUIDE_CLEARANCE_UM})."
        ),
    )
    parser.add_argument(
        "--obstacle-clearance-um",
        dest="waveguide_clearance_um",
        type=float,
        default=argparse.SUPPRESS,
        metavar="UM",
        help="Deprecated alias for --waveguide-clearance-um.",
    )
    parser.add_argument(
        "--heater-clearance-um",
        type=float,
        default=SCRIPT_HEATER_CLEARANCE_UM,
        metavar="UM",
        help=(
            "Clearance in micrometers for heater/metal obstacles when "
            "--include-heater-obstacles is true "
            f"(default: {SCRIPT_HEATER_CLEARANCE_UM})."
        ),
    )
    parser.add_argument(
        "--chip-add-x-um",
        type=float,
        default=SCRIPT_CHIP_ADD_X_UM,
        metavar="UM",
        help=(
            "Extra horizontal chip margin added to both left and right when "
            "the die bbox is computed automatically "
            f"(default: {SCRIPT_CHIP_ADD_X_UM})."
        ),
    )
    parser.add_argument(
        "--chip-add-y-um",
        type=float,
        default=SCRIPT_CHIP_ADD_Y_UM,
        metavar="UM",
        help=(
            "Extra vertical chip margin added to both bottom and top when "
            "the die bbox is computed automatically "
            f"(default: {SCRIPT_CHIP_ADD_Y_UM})."
        ),
    )
    parser.add_argument(
        "--clear-port-open-cells-from-static",
        type=_parse_bool_flag,
        default=SCRIPT_CLEAR_PORT_OPEN_CELLS_FROM_STATIC,
        metavar="BOOL",
        help=(
            "Clear port-open cells from static obstacles "
            f"(default: {str(SCRIPT_CLEAR_PORT_OPEN_CELLS_FROM_STATIC).lower()})."
        ),
    )
    parser.add_argument(
        "--ripup-reroute",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_RIPUP_REROUTE,
        metavar="BOOL",
        help=(
            "Enable conflict-probe rip-up and reroute "
            f"(default: {str(SCRIPT_ENABLE_RIPUP_REROUTE).lower()})."
        ),
    )
    parser.add_argument(
        "--ripup-max-rounds",
        type=int,
        default=SCRIPT_RIPUP_MAX_ROUNDS,
        metavar="N",
        help=f"Maximum repair rounds per failed net (default: {SCRIPT_RIPUP_MAX_ROUNDS}).",
    )
    parser.add_argument(
        "--ripup-max-victims",
        type=int,
        default=SCRIPT_RIPUP_MAX_VICTIMS,
        metavar="N",
        help=(
            "Maximum blocker routes to rip up per repair round "
            f"(default: {SCRIPT_RIPUP_MAX_VICTIMS})."
        ),
    )
    parser.add_argument(
        "--ripup-history-weight",
        type=float,
        default=SCRIPT_RIPUP_HISTORY_WEIGHT,
        metavar="W",
        help=f"A* history penalty weight during repair (default: {SCRIPT_RIPUP_HISTORY_WEIGHT}).",
    )
    parser.add_argument(
        "--ripup-history-increment",
        type=int,
        default=SCRIPT_RIPUP_HISTORY_INCREMENT,
        metavar="N",
        help=(
            "History penalty increment for probed/ripped route cells "
            f"(default: {SCRIPT_RIPUP_HISTORY_INCREMENT})."
        ),
    )
    parser.add_argument(
        "--attempt-diagnostics",
        action="store_true",
        default=SCRIPT_ATTEMPT_DIAGNOSTICS,
        help=(
            "Collect extra per-attempt window, obstacle-density, and rip-up "
            "diagnostics for slow or failed route attempts."
        ),
    )
    parser.add_argument(
        "--electrical-routing",
        type=_parse_bool_flag,
        default=SCRIPT_ENABLE_ELECTRICAL_ROUTING,
        metavar="BOOL",
        help=(
            "Route heater electrical metal after optical routing "
            f"(default: {str(SCRIPT_ENABLE_ELECTRICAL_ROUTING).lower()})."
        ),
    )
    parser.add_argument(
        "--electrical-pad-side",
        choices=("top", "bottom"),
        default=SCRIPT_ELECTRICAL_PAD_SIDE,
        help=(
            f"Side used for electrical bondpad placement (default: {SCRIPT_ELECTRICAL_PAD_SIDE})."
        ),
    )
    parser.add_argument(
        "--electrical-grid-pitch-um",
        type=float,
        default=SCRIPT_ELECTRICAL_GRID_PITCH_UM,
        metavar="UM",
        help=(f"Electrical routing grid pitch (default: {SCRIPT_ELECTRICAL_GRID_PITCH_UM})."),
    )
    parser.add_argument(
        "--electrical-obstacle-clearance-um",
        type=float,
        default=SCRIPT_ELECTRICAL_OBSTACLE_CLEARANCE_UM,
        metavar="UM",
        help=(
            "Electrical routing obstacle clearance "
            f"(default: {SCRIPT_ELECTRICAL_OBSTACLE_CLEARANCE_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-wire-width-um",
        type=float,
        default=SCRIPT_ELECTRICAL_WIRE_WIDTH_UM,
        metavar="UM",
        help=(
            f"Electrical individual route wire width (default: {SCRIPT_ELECTRICAL_WIRE_WIDTH_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-bus-width-um",
        type=float,
        default=SCRIPT_ELECTRICAL_BUS_WIDTH_UM,
        metavar="UM",
        help=(f"Electrical common bus route width (default: {SCRIPT_ELECTRICAL_BUS_WIDTH_UM})."),
    )
    parser.add_argument(
        "--electrical-terminal-contact-width-um",
        type=float,
        default=SCRIPT_ELECTRICAL_TERMINAL_CONTACT_WIDTH_UM,
        metavar="UM",
        help=(
            "Minimum electrical terminal contact width "
            f"(default: {SCRIPT_ELECTRICAL_TERMINAL_CONTACT_WIDTH_UM})."
        ),
    )
    parser.add_argument(
        "--electrical-pad-pitch-um",
        type=float,
        default=SCRIPT_ELECTRICAL_PAD_PITCH_UM,
        metavar="UM",
        help=(f"Electrical bondpad pitch (default: {SCRIPT_ELECTRICAL_PAD_PITCH_UM})."),
    )
    parser.add_argument(
        "--enable-jps4",
        type=_parse_bool_flag,
        default=False,
        metavar="BOOL",
        help=(
            "Request the experimental Manhattan JPS4 accelerator when eligible "
            "(default: false). Pass 3A still falls back to baseline A*."
        ),
    )
    parser.add_argument(
        "--use-indexed-heap",
        type=_parse_bool_flag,
        default=False,
        metavar="BOOL",
        help=(
            "Use the experimental decrease-key indexed heap for dense A* "
            "instead of duplicate-entry BinaryHeap queueing (default: false)."
        ),
    )
    parser.add_argument(
        "--enable-simple-routes",
        type=_parse_bool_flag,
        default=True,
        metavar="BOOL",
        help=(
            "Enable straight/L/Z simple-route candidates before dense A* "
            "(default: true). Pass false for A*-only regression runs."
        ),
    )
    parser.add_argument(
        "--primitive-ordering",
        choices=("library", "long_straight_first", "target_biased"),
        default="library",
        help="Dense A* primitive iteration order experiment (default: library).",
    )
    parser.add_argument(
        "--heuristic-mode",
        choices=("distance", "heading_aware"),
        default="heading_aware",
        help="Dense A* heuristic mode (default: heading_aware).",
    )
    parser.add_argument(
        "--net-order",
        choices=NET_ORDERS,
        default="topological",
        help=(
            "Net routing order within the topological layers: 'topological' = "
            "declaration order (default, the baseline); 'topological-span' = "
            "shortest grid span first (LiDAR-like); 'plan-crossings-desc' / "
            "'plan-crossings-asc' = most / fewest planned crossings first "
            "(contribution 1 S3, needs --crossing-mode lidar-guided)."
        ),
    )
    parser.add_argument(
        "--heap-tie-breaker",
        choices=("smaller_g", "larger_g"),
        default="smaller_g",
        help=(
            "Dense A* heap tie-breaker experiment. Default preserves the "
            "historical smaller-g behavior."
        ),
    )
    return parser


def _benchmark_stable_defaults(argv: list[str] | None) -> tuple[list[str], dict[str, str]]:
    """The selected benchmark's `STABLE_ROUTING_FLAGS` / `STABLE_ROUTING_ENV`.

    A benchmark module may declare the configuration it is known to route
    cleanly with. Those flags become the CLI defaults for that benchmark:
    they are parsed first, so anything given explicitly on the command line
    still wins (argparse keeps the last occurrence). `benchmark_photonic.py`
    and the tests pass their flags explicitly and are unaffected."""
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument("benchmark", nargs="?")
    known, _ = probe.parse_known_args(sys.argv[1:] if argv is None else argv)
    if not known.benchmark:
        return [], {}
    try:
        module = importlib.import_module(f"benchmarks.{known.benchmark}")
    except ImportError:
        return [], {}
    flags = [str(flag) for flag in getattr(module, "STABLE_ROUTING_FLAGS", ())]
    env = {str(k): str(v) for k, v in dict(getattr(module, "STABLE_ROUTING_ENV", {})).items()}
    return flags, env


def main(argv: list[str] | None = None) -> Component:
    stable_flags, stable_env = _benchmark_stable_defaults(argv)
    for key, value in stable_env.items():
        os.environ.setdefault(key, value)
    args = _build_arg_parser().parse_args(
        stable_flags + (sys.argv[1:] if argv is None else list(argv))
    )
    if stable_flags:
        print(f"      - Benchmark stable defaults applied: {' '.join(stable_flags)}")
    return run_routing_flow(
        args.benchmark,
        debug_svgs=args.debug_svgs,
        debug_timing=args.debug_timing,
        debug_stop_after_route_index=args.debug_stop_after_route,
        debug_meanders=args.debug_meanders,
        verbose_routes=args.verbose_routes,
        show_klayout=args.show_klayout,
        allow_45_degree_turns=args.allow_45_degree_turns,
        enable_jps4=args.enable_jps4,
        use_indexed_heap=args.use_indexed_heap,
        primitive_ordering=args.primitive_ordering,
        heuristic_mode=args.heuristic_mode,
        net_order=args.net_order,
        heap_tie_breaker=args.heap_tie_breaker,
        bend_radius_um=args.bend_radius_um,
        enable_path_length_matching=args.path_length_matching,
        path_length_match_outputs=args.path_length_match_outputs,
        path_length_meander_height_um=args.path_length_meander_height_um,
        enable_crossings=args.crossings,
        crossing_mode=args.crossing_mode,
        preplaced_crossing_grids=args.preplaced_crossing_grids,
        min_straight_cells_per_crossing=args.min_straight_cells_per_crossing,
        foreign_port_keepout_cells=args.foreign_port_keepout_cells,
        fanout_access_mode=args.fanout_access_mode,
        proactive_congestion_weight=args.proactive_congestion_weight,
        proactive_congestion_radius_cells=args.proactive_congestion_radius_cells,
        max_iterations=args.max_iterations,
        enable_simple_routes=args.enable_simple_routes,
        routing_window_scale=args.routing_window_scale,
        include_heater_obstacles=args.include_heater_obstacles,
        ripup_reroute_config=RipupRerouteConfig(
            enabled=args.ripup_reroute,
            max_rounds=args.ripup_max_rounds,
            max_victims_per_failure=args.ripup_max_victims,
            history_weight=args.ripup_history_weight,
            history_increment=args.ripup_history_increment,
        ),
        collect_attempt_diagnostics=args.attempt_diagnostics,
        enable_electrical_routing=args.electrical_routing,
        electrical_config=ElectricalRoutingConfig(
            pad_side=args.electrical_pad_side,
            routing_grid_pitch_um=args.electrical_grid_pitch_um,
            obstacle_clearance_um=args.electrical_obstacle_clearance_um,
            wire_width_um=args.electrical_wire_width_um,
            bus_width_um=args.electrical_bus_width_um,
            terminal_contact_width_um=args.electrical_terminal_contact_width_um,
            pad_pitch_um=args.electrical_pad_pitch_um,
        ),
        static_obstacle_config=StaticObstacleMapConfig(
            grid_size_um=args.grid_size_um,
            obstacle_mode=args.obstacle_mode,
            clearance_um=args.waveguide_clearance_um,
            heater_clearance_um=args.heater_clearance_um,
            chip_add_x_um=args.chip_add_x_um,
            chip_add_y_um=args.chip_add_y_um,
            clear_port_open_cells_from_static=args.clear_port_open_cells_from_static,
        ),
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
                f"Expected Schematic from {benchmark_name}.build_schematic(), got {type(schematic)}"
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


def _load_benchmark_stage(
    *,
    benchmark_name: str,
    total_steps: int,
    stats: RoutingFlowStats | None,
    debug_meanders: bool,
) -> Schematic:
    """Load the schematic and record the load-stage stats/output."""
    step_load_start = time.perf_counter()
    print(f"\n[1/{total_steps}] Loading benchmark: {benchmark_name}...")
    schematic = load_benchmark(benchmark_name)
    step_load_end = time.perf_counter()
    if stats is not None:
        stats.instance_count = len(schematic.netlist.instances)
        stats.net_count = len(schematic.netlist.routes)
        stats.step_times_s["load_benchmark"] = step_load_end - step_load_start
    print("      \u2713 Schematic loaded")
    print(f"      - Load time: {step_load_end - step_load_start:.4f} s")
    if debug_meanders:
        print(f"      - Instances: {list(schematic.netlist.instances.keys())}")
        print(f"      - Placements: {list(schematic.placements.keys())}")
    else:
        print(f"      - Instances: {len(schematic.netlist.instances)}")
        print(f"      - Placements: {len(schematic.placements)}")
    return schematic


def _layout_from_schematic_stage(
    *,
    schematic: Schematic,
    total_steps: int,
    stats: RoutingFlowStats | None,
    debug_timing: bool,
) -> Component:
    """Translate the schematic to an unrouted layout and report layout metadata."""
    step_layout_start = time.perf_counter()
    print(f"\n[2/{total_steps}] Translating schematic to layout...")
    unrouted_layout = layout_from_schematic(schematic)
    step_layout_end = time.perf_counter()
    if stats is not None:
        stats.step_times_s["layout_from_schematic"] = step_layout_end - step_layout_start
    print(f"      \u2713 Layout generated: {unrouted_layout.name}")
    bbox = unrouted_layout.bbox
    if callable(bbox):
        bbox = bbox()
    print(f"      - Bounding box: {bbox}")
    if debug_timing:
        print(f"      - Translation time: {step_layout_end - step_layout_start:.4f} s")
    return unrouted_layout


def _print_flow_header(benchmark_name: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"Routing Flow: {benchmark_name}")
    print(f"{'=' * 60}")


def _print_flow_footer() -> None:
    print(f"\n{'=' * 60}\n")


def _preplaced_crossing_grids_stage(
    *,
    benchmark_name: str,
    schematic: Schematic,
    unrouted_layout: Component,
    total_steps: int,
    stats: RoutingFlowStats | None,
) -> tuple[Schematic, Component, dict[str, object]]:
    """Replace the schematic/layout with the crossing-grid-derived pair.

    Every interstage crossing of the benchmark is computed from its topology
    metadata, realized as a pre-wired crossing grid instance placed in the
    layer's free band, and the interstage nets are split into crossing-free
    stubs. Downstream stages see an ordinary schematic and layout.
    """
    from benchmark_metadata import load_benchmark_metadata
    from translation.preplaced_crossing_grids import (
        build_crossing_plan_for_benchmark,
        derive_preplaced_crossing_layout,
        preplaced_crossing_grid_metrics,
    )

    print(f"\n[2b/{total_steps}] Placing topology-derived crossing grids...")
    t_start = time.perf_counter()
    metadata = load_benchmark_metadata(benchmark_name, schematic=schematic)
    crossing_plan = build_crossing_plan_for_benchmark(schematic, metadata)
    derived = derive_preplaced_crossing_layout(schematic, unrouted_layout, crossing_plan)
    metrics = preplaced_crossing_grid_metrics(derived)
    if derived.placed_crossing_count != derived.expected_crossing_count:
        raise RuntimeError(
            "pre-placed crossing grids realized "
            f"{derived.placed_crossing_count} crossing component(s) but the topology "
            f"expects {derived.expected_crossing_count}"
        )
    elapsed = time.perf_counter() - t_start
    if stats is not None:
        stats.step_times_s["preplaced_crossing_grids"] = elapsed
    print(
        f"      - Grids: {metrics['grid_count']} placed, "
        f"{metrics['crossing_component_count']} crossing component(s) "
        f"(topology expects {metrics['expected_crossing_count']}), "
        f"{metrics['split_net_count']} interstage net(s) split into stubs; "
        f"{len(derived.schematic.netlist.routes)} route(s) to route ({elapsed:.2f}s)"
    )
    return derived.schematic, derived.unrouted_layout, {"preplaced_crossing_grids": metrics}


def run_routing_flow(
    benchmark_name: str,
    *,
    debug_svgs: DebugSvgSelector = False,
    show_unrouted: bool | None = None,
    show_routed: bool | None = None,
    show_debug_svgs: DebugSvgSelector | None = None,
    show_static_obstacles_svg: bool | None = None,
    debug_timing: bool = False,
    debug_stop_after_route_index: int | None = None,
    debug_meanders: bool = False,
    verbose_routes: bool = False,
    show_klayout: bool = False,
    enable_path_length_matching: bool = False,
    path_length_match_outputs: bool = False,
    path_length_meander_height_um: float = SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    enable_crossings: bool = False,
    crossing_mode: str = "window",
    preplaced_crossing_grids: bool = False,
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
    fanout_access_mode: str | None = SCRIPT_FANOUT_ACCESS_MODE,
    proactive_congestion_weight: float = SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
    proactive_congestion_radius_cells: int = SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
    allow_45_degree_turns: bool = SCRIPT_ALLOW_45_DEGREE_TURNS,
    bend_radius_um: float = SCRIPT_BEND_RADIUS_UM,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    enable_simple_routes: bool = True,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    net_order: str = "topological",
    heap_tie_breaker: str = "smaller_g",
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    include_heater_obstacles: bool = False,
    grid_size_um: float = SCRIPT_GRID_SIZE_UM,
    waveguide_clearance_um: float | None = None,
    heater_clearance_um: float | None = None,
    obstacle_clearance_um: float | None = None,
    chip_add_x_um: float = SCRIPT_CHIP_ADD_X_UM,
    chip_add_y_um: float = SCRIPT_CHIP_ADD_Y_UM,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    static_obstacle_config: StaticObstacleMapConfig | None = None,
    enable_electrical_routing: bool = False,
    electrical_config: ElectricalRoutingConfig | None = None,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    stats: RoutingFlowStats | None = None,
) -> Component:
    """Execute the routing flow for a given benchmark.

    Parameters:
        benchmark_name: Name of the benchmark to run (e.g., 'TOY').
        debug_svgs: If True or "all", generate all debug SVGs into build/.
                    If a selector such as "5-10", "5", or "2,5-10" is
                    provided, generate only matching per-route SVGs by
                    1-based net order. Static obstacle SVGs are still
                    generated when debug SVGs are enabled.
        debug_timing: If True, print timing information for each stage.
        debug_meanders: If True, print verbose path-length and meander
                      insertion details when path-length matching is enabled.
        verbose_routes: If True, print per-net routing progress and whether
                      each route used the simple router or A*.
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
        path_length_match_outputs: If True, add output-arrival equalization
                      requirements after local path-length matching.
        path_length_meander_height_um: Maximum meander height used when
                      inserting path-length matching meanders.
        crossing_half_size_cells: Crossing keepout half-size in grid cells.
                      The default 0 derives it from the crossing component bbox.
        crossing_mode: Crossing routing mode. "window" uses the existing
                      expected-partner crossing search; "collision" legalizes
                      crossings after A* collides with topology-allowed route
                      geometry; "lidar-pure" uses dynamic DRC-style crossing
                      permission against any committed route (the baseline);
                      "lidar-guided" is contribution 1: lidar-pure mechanics plus
                      the precomputed topology crossings as soft search guidance.
                      Exactly one of lidar-pure / lidar-guided /
                      preplaced_crossing_grids (contribution 2) runs at a time.
        preplaced_crossing_grids: If True, derive every interstage layer's
                      crossings from the benchmark topology, place one
                      pre-wired crossing grid per layer into the layout before
                      routing, split each interstage net into two crossing-free
                      stubs to/from the grid, and route with crossings disabled.
                      Mutually exclusive with enable_crossings. See
                      translation/preplaced_crossing_grids.py.
        min_straight_cells_per_crossing: Minimum straight access length on each
                      side of a crossing in grid cells.
        fanout_access_mode: Dense multi-port access strategy passed to the Rust
                      routing bridge. Use "static-stubs" to pre-spread
                      same-element fanout ports as fixed static breakout
                      geometry and route from virtual anchor ports.
        proactive_congestion_weight: Soft A* cost per blocked side-neighbor
                      cell beside straight moves.
        proactive_congestion_radius_cells: Sideways grid radius used for
                      proactive congestion counting.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        bend_radius_um: Minimum optical waveguide bend radius. Rounded up to
                      the active routing grid before primitive generation.
        use_indexed_heap: Benchmark-only indexed-heap experiment. Pass 8E
            measured it slower than duplicate-entry BinaryHeap queueing, so
            the default remains False.
        enable_simple_routes: If False, force optical nets through A* by
            disabling straight/L/Z simple-route candidates.
        primitive_ordering: Benchmark-only dense A* primitive ordering
            experiment. Pass 8F keeps "library" as the default.
        heuristic_mode: Dense A* heuristic experiment.
        heap_tie_breaker: Benchmark-only dense A* heap tie-breaker experiment.
            "smaller_g" preserves historical behavior; "larger_g" favors
            deeper states on equal f-score plateaus.
        max_iterations: Maximum A* state expansions per route attempt.
        routing_window_scale: Optional A* routing-window margin scale. If None,
                      the Rust AStarConfig default is used.
        include_heater_obstacles: If True, include configured heater/metal
                      layers as static optical-routing obstacles. Component-
                      specific optical port openings still apply when their
                      component rules match.
        grid_size_um: Optical routing grid resolution in micrometers.
        waveguide_clearance_um: Static clearance in micrometers for existing
                      optical/waveguide obstacles.
        heater_clearance_um: Static clearance in micrometers for heater/metal
                      obstacles. Defaults to the waveguide clearance.
        obstacle_clearance_um: Deprecated alias for waveguide_clearance_um.
        chip_add_x_um: Extra horizontal chip margin added to both left and
                      right when the die bbox is computed automatically.
        chip_add_y_um: Extra vertical chip margin added to both bottom and top
                      when the die bbox is computed automatically.
        static_obstacle_config: Optional obstacle builder config. If omitted,
            strict bounding-box static obstacles are used.
        enable_electrical_routing: If True, run the electrical heater-metal
            routing stage after optical routing and return/write/show the
            electrically routed component.
        electrical_config: Optional electrical routing configuration. If
            omitted, `ElectricalRoutingConfig()` defaults are used.
        collect_route_stats: If True, collect route-search counters without
            printing debug timing. This is enabled automatically when stats is
            provided.
        collect_attempt_diagnostics: If True, collect extra per-attempt window,
            obstacle-density, and ripup diagnostics for slow/failed attempts.

    Returns:
        The routed layout component.
    """
    debug_svgs, show_klayout = resolve_legacy_display_options(
        debug_svgs=debug_svgs,
        show_klayout=show_klayout,
        show_unrouted=show_unrouted,
        show_routed=show_routed,
        show_debug_svgs=show_debug_svgs,
        show_static_obstacles_svg=show_static_obstacles_svg,
    )
    debug_svgs_enabled, debug_route_indices = parse_debug_svg_selector(debug_svgs)
    total_steps = 4 if enable_electrical_routing else 3

    _print_flow_header(benchmark_name)

    if stats is not None:
        stats.benchmark_name = benchmark_name

    route_static_obstacle_config, waveguide_clearance_um, heater_clearance_um = (
        build_static_obstacle_config(
            static_obstacle_config=static_obstacle_config,
            grid_size_um=grid_size_um,
            waveguide_clearance_um=waveguide_clearance_um,
            heater_clearance_um=heater_clearance_um,
            obstacle_clearance_um=obstacle_clearance_um,
            chip_add_x_um=chip_add_x_um,
            chip_add_y_um=chip_add_y_um,
        )
    )
    debug_dir, route_debug_indices = debug_artifact_routing_options(
        debug_svgs_enabled=debug_svgs_enabled,
        debug_route_indices=debug_route_indices,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
    )

    t_flow_start = time.perf_counter()

    if debug_svgs_enabled:
        cleanup_debug_artifacts(benchmark_name)

    # Keep the body below as the stage outline; detailed behavior belongs in
    # the stage modules and small helpers above.
    schematic = _load_benchmark_stage(
        benchmark_name=benchmark_name,
        total_steps=total_steps,
        stats=stats,
        debug_meanders=debug_meanders,
    )
    unrouted_layout = _layout_from_schematic_stage(
        schematic=schematic,
        total_steps=total_steps,
        stats=stats,
        debug_timing=debug_timing,
    )
    preplaced_report_metadata: dict[str, object] | None = None
    if preplaced_crossing_grids:
        if is_guided_mode(crossing_mode):
            raise ValueError(
                "preplaced_crossing_grids (contribution 2) and crossing_mode "
                "'lidar-guided' (contribution 1) are alternative contributions; "
                "run exactly one of lidar-pure / lidar-guided / preplaced grids."
            )
        if enable_crossings:
            raise ValueError(
                "preplaced_crossing_grids and enable_crossings are mutually exclusive: "
                "pre-placed grids resolve every crossing before routing, so the router "
                "must run with crossings disabled."
            )
        schematic, unrouted_layout, preplaced_report_metadata = _preplaced_crossing_grids_stage(
            benchmark_name=benchmark_name,
            schematic=schematic,
            unrouted_layout=unrouted_layout,
            total_steps=total_steps,
            stats=stats,
        )
    record_initial_route_stats(stats)
    optical_config = build_optical_routing_stage_config(
        enable_path_length_matching=enable_path_length_matching,
        path_length_match_outputs=path_length_match_outputs,
        path_length_meander_height_um=path_length_meander_height_um,
        enable_crossings=enable_crossings,
        crossing_mode=crossing_mode,
        crossing_half_size_cells=crossing_half_size_cells,
        min_straight_cells_per_crossing=min_straight_cells_per_crossing,
        foreign_port_keepout_cells=foreign_port_keepout_cells,
        fanout_access_mode=fanout_access_mode,
        proactive_congestion_weight=proactive_congestion_weight,
        proactive_congestion_radius_cells=proactive_congestion_radius_cells,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_um=bend_radius_um,
        enable_jps4=enable_jps4,
        use_indexed_heap=use_indexed_heap,
        enable_simple_routes=enable_simple_routes,
        primitive_ordering=primitive_ordering,
        heuristic_mode=heuristic_mode,
        net_order=net_order,
        heap_tie_breaker=heap_tie_breaker,
        max_iterations=max_iterations,
        routing_window_scale=routing_window_scale,
        include_heater_obstacles=include_heater_obstacles,
        ripup_reroute_config=ripup_reroute_config,
        route_static_obstacle_config=route_static_obstacle_config,
        debug_dir=debug_dir,
        route_debug_indices=route_debug_indices,
        debug_stop_after_route_index=debug_stop_after_route_index,
        debug_timing=debug_timing,
        debug_meanders=debug_meanders,
        verbose_routes=verbose_routes,
        debug_svgs_enabled=debug_svgs_enabled,
        collect_route_stats=collect_route_stats,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
        stats=stats,
    )
    optical_result = run_photonic_routing_stage(
        benchmark_name=benchmark_name,
        schematic=schematic,
        unrouted_layout=unrouted_layout,
        total_steps=total_steps,
        config=optical_config,
    )
    route_result = optical_result.route_result
    routed_layout = optical_result.routed_layout
    debug_artifacts = optical_result.debug_artifacts
    if stats is not None:
        populate_route_stats(
            stats,
            route_result=route_result,
            debug_artifacts=debug_artifacts,
            route_summary=optical_result.route_summary,
            route_attempt_records=optical_result.route_attempt_records,
            route_time_s=optical_result.route_time_s,
        )
    electrical_result: ElectricalRoutingResult | None = None

    t_verify_start = time.perf_counter()
    verify_and_attach_photonic_reports(
        benchmark_name=benchmark_name,
        schematic=schematic,
        unrouted_layout=unrouted_layout,
        routed_layout=routed_layout,
        debug_artifacts=debug_artifacts,
        include_heater_obstacles=include_heater_obstacles,
        debug_stop_after_route_index=debug_stop_after_route_index,
        extra_report_metadata=preplaced_report_metadata,
    )
    if debug_timing:
        # Performance pass 2026-09-03 (P5): the non-routing phases were an
        # un-itemised 16 % of a multiportmmi_32x32 run.
        print(f"      - Verification time: {time.perf_counter() - t_verify_start:.4f} s")

    t_plm_start = time.perf_counter()
    attach_and_report_path_length_matching(
        routed_layout=routed_layout,
        route_result=route_result,
        debug_meanders=debug_meanders,
    )
    if debug_timing:
        print(
            f"      - Path-length-matching report time: {time.perf_counter() - t_plm_start:.4f} s"
        )

    if enable_electrical_routing:
        routed_layout, electrical_result = run_electrical_routing_step(
            benchmark_name=benchmark_name,
            schematic=schematic,
            routed_layout=routed_layout,
            electrical_config=electrical_config,
            debug_dir=debug_dir,
            total_steps=total_steps,
            stats=stats,
            debug_timing=debug_timing,
            debug_svgs_enabled=debug_svgs_enabled,
        )

    if debug_svgs_enabled:
        report_and_open_debug_svgs(
            debug_artifacts=debug_artifacts,
            electrical_result=electrical_result,
            debug_route_indices=debug_route_indices,
        )

    t_write_start = time.perf_counter()
    write_or_show_routed_layout(
        benchmark_name=benchmark_name,
        routed_layout=routed_layout,
        show_klayout=show_klayout,
    )
    if debug_timing:
        print(f"      - GDS write time: {time.perf_counter() - t_write_start:.4f} s")

    if debug_timing:
        t_end = time.perf_counter()
        total = t_end - t_flow_start
        print(f"\nTiming summary for {benchmark_name}:\n  total: {total:.4f} s")
    if stats is not None:
        total = time.perf_counter() - t_flow_start
        stats.total_time_s = float(total)

    _print_flow_footer()

    return routed_layout


if __name__ == "__main__":
    main()
