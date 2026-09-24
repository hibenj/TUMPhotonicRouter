"""The router's command line (Milestone 5, Slice 3; D6).

    python -m photonic_router route <benchmark> [flags]

`main` is the whole entry point: it parses the arguments, merges them with the
benchmark's own stable block, builds the two objects the flow is configured
with - a `RoutingConfig` (through `photonic_router.config_loading.build_config`)
and a `FlowOptions` - and calls `photonic_router.flow.route_benchmark`. Nothing
else in the flow sees a command line.

`--configuration {baseline,contribution1,contribution2}` names the three
configurations the paper's rows were produced with (owner rule 2026-09-04:
exactly one of them runs at a time). It expands to exactly the flags
`scripts/results/reproduce_date2027.sh` passes, inserted before the user's own
flags so that an explicit flag always wins.

`routing_flow.py` in the repository root is a shim over this module, so
`python routing_flow.py <benchmark> [flags]` keeps working unchanged (the
reproduction scripts and the benchmark stable blocks use it; D4).
"""

import argparse
import importlib
import sys

from gdsfactory.component import Component

from photonic_router.config_loading import build_config
from photonic_router.flow import route_benchmark
from photonic_router.flow_options import (
    DebugArtifactOptions,
    ElectricalOptions,
    FlowOptions,
    LayoutOptions,
    LoadingOptions,
    ObstacleOptions,
    OpticalStageOptions,
    OutputOptions,
    PathLengthMatchingOptions,
    PreplacedCrossingGridOptions,
)
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from translation.crossing_modes import CROSSING_MODES
from translation.electrical import ElectricalRoutingConfig
from translation.route_order import NET_ORDERS
from translation.route_rust import RipupRerouteConfig

from routing_flow_config import (
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
)


_CONFIGURATION_FLAGS: dict[str, tuple[str, ...]] = {
    # Exactly the flags scripts/results/reproduce_date2027.sh passes for each
    # row of the paper's table. Changing one of these changes what the paper's
    # numbers mean, so they are pinned here and nowhere else.
    "baseline": ("--crossing-mode", "lidar-pure"),
    "contribution1": ("--crossing-mode", "lidar-guided"),
    "contribution2": ("--preplaced-crossing-grids", "true", "--verbose-routes"),
}


def configuration_flags(argv: list[str]) -> list[str]:
    """The flags `--configuration <name>` in `argv` expands to (empty if absent).

    They are parsed before the caller's own flags, which therefore win: a
    `--configuration contribution1 --crossing-mode lidar-pure` run is
    contribution 1's configuration with the crossing mode overridden.
    """
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument("--configuration", choices=tuple(_CONFIGURATION_FLAGS), default=None)
    known, _ = probe.parse_known_args(argv)
    if known.configuration is None:
        return []
    return list(_CONFIGURATION_FLAGS[known.configuration])


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
        "--configuration",
        choices=tuple(_CONFIGURATION_FLAGS),
        default=None,
        help=(
            "One of the three named configurations the paper's rows use "
            "(owner rule 2026-09-04: exactly one of them runs at a time): "
            "'baseline' = --crossing-mode lidar-pure; 'contribution1' = "
            "--crossing-mode lidar-guided (crossing-guided A*); "
            "'contribution2' = --preplaced-crossing-grids true "
            "--verbose-routes (pre-placed crossing structures). The flags are "
            "inserted before the ones given here, so an explicit flag wins."
        ),
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
        default=None,
        metavar="BOOL",
        help=(
            "Build and pass topology-derived crossing constraints to the Rust "
            "router (default: "
            f"{str(SCRIPT_ENABLE_CROSSINGS).lower()}; false when "
            "--preplaced-crossing-grids is on, whose grids resolve every crossing)."
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
            "Crossing search mode. 'lidar-pure' enables collision-driven crossing "
            "legalization against any committed route, with no topology pair "
            "permissions (the baseline); 'lidar-guided' is contribution 1: "
            "lidar-pure mechanics plus the precomputed topology crossings as soft "
            "search guidance (planned crossings cost "
            "PHOTONIC_ROUTER_PLANNED_CROSSING_SEARCH_LOSS_UM, default 0, in the "
            "search; unplanned ones keep the collision price) "
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
        default=None,
        help=(
            "Net routing order within the topological layers: 'topological' = "
            "declaration order (default for the baseline and contribution 1); "
            "'topological-span' = shortest grid span first (LiDAR-like; default "
            "for --preplaced-crossing-grids, whose planar stubs need it); "
            "'plan-crossings-desc' / 'plan-crossings-asc' = most / fewest "
            "planned crossings first (contribution 1 S3, needs --crossing-mode "
            "lidar-guided)."
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


_CROSSING_DISCOVERY_FLAGS: frozenset[str] = frozenset({"--crossings", "--crossing-mode"})


def stable_flags_for_configuration(
    stable_flags: list[str], *, preplaced_crossing_grids: bool
) -> list[str]:
    """A benchmark's stable block describes its lidar-pure baseline; under
    contribution 2 (pre-placed crossing grids) the crossing-discovery flags
    of that block (`--crossings`, `--crossing-mode`) are dropped and every
    other stable flag (fan-out access, congestion, iteration caps) stays.
    Without this, `--preplaced-crossing-grids true` alone would inherit
    `--crossings true` and be rejected as a contradictory configuration."""
    if not preplaced_crossing_grids:
        return list(stable_flags)
    kept: list[str] = []
    skip_value = False
    for flag in stable_flags:
        if skip_value:
            skip_value = False
            continue
        if flag in _CROSSING_DISCOVERY_FLAGS:
            skip_value = True
            continue
        kept.append(flag)
    return kept


def _requests_preplaced_crossing_grids(argv: list[str]) -> bool:
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument("--preplaced-crossing-grids", type=_parse_bool_flag, default=False)
    known, _ = probe.parse_known_args(argv)
    return bool(known.preplaced_crossing_grids)


def main(argv: list[str] | None = None) -> Component:
    user_argv = sys.argv[1:] if argv is None else list(argv)
    configuration_argv = configuration_flags(user_argv)
    stable_flags, stable_env = _benchmark_stable_defaults(argv)
    stable_flags = stable_flags_for_configuration(
        stable_flags,
        preplaced_crossing_grids=_requests_preplaced_crossing_grids(
            configuration_argv + user_argv
        ),
    )
    # The benchmark's STABLE_ROUTING_ENV block and the process environment go
    # through the overlay table into one configuration object (environment
    # wins, as os.environ.setdefault did before); nothing is written to the
    # environment any more.
    config = build_config(stable_env)
    args = _build_arg_parser().parse_args(stable_flags + configuration_argv + user_argv)
    if stable_flags:
        print(f"      - Benchmark stable defaults applied: {' '.join(stable_flags)}")
    if stable_env:
        print(f"      - Benchmark stable environment applied: {' '.join(sorted(stable_env))}")
    if configuration_argv:
        print(
            f"      - Configuration '{args.configuration}' flags applied: "
            f"{' '.join(configuration_argv)}"
        )
    return route_benchmark(config, _flow_options(args))


def _flow_options(args: argparse.Namespace) -> FlowOptions:
    """The parsed command line as the flow's option tree."""
    return FlowOptions(
        loading=LoadingOptions(benchmark_name=args.benchmark),
        layout=LayoutOptions(
            allow_45_degree_turns=args.allow_45_degree_turns,
            bend_radius_um=args.bend_radius_um,
        ),
        obstacles=ObstacleOptions(
            static_obstacle_config=StaticObstacleMapConfig(
                grid_size_um=args.grid_size_um,
                obstacle_mode=args.obstacle_mode,
                clearance_um=args.waveguide_clearance_um,
                heater_clearance_um=args.heater_clearance_um,
                chip_add_x_um=args.chip_add_x_um,
                chip_add_y_um=args.chip_add_y_um,
                clear_port_open_cells_from_static=args.clear_port_open_cells_from_static,
            ),
            grid_size_um=args.grid_size_um,
            waveguide_clearance_um=args.waveguide_clearance_um,
            heater_clearance_um=args.heater_clearance_um,
            chip_add_x_um=args.chip_add_x_um,
            chip_add_y_um=args.chip_add_y_um,
            include_heater_obstacles=args.include_heater_obstacles,
        ),
        preplaced_grids=PreplacedCrossingGridOptions(
            preplaced_crossing_grids=args.preplaced_crossing_grids,
        ),
        optical=OpticalStageOptions(
            # `--crossings` unset means "the script default, unless the
            # pre-placed grids resolve every crossing already".
            enable_crossings=(
                args.crossings
                if args.crossings is not None
                else (False if args.preplaced_crossing_grids else SCRIPT_ENABLE_CROSSINGS)
            ),
            crossing_mode=args.crossing_mode,
            min_straight_cells_per_crossing=args.min_straight_cells_per_crossing,
            foreign_port_keepout_cells=args.foreign_port_keepout_cells,
            fanout_access_mode=args.fanout_access_mode,
            proactive_congestion_weight=args.proactive_congestion_weight,
            proactive_congestion_radius_cells=args.proactive_congestion_radius_cells,
            enable_jps4=args.enable_jps4,
            use_indexed_heap=args.use_indexed_heap,
            enable_simple_routes=args.enable_simple_routes,
            primitive_ordering=args.primitive_ordering,
            heuristic_mode=args.heuristic_mode,
            net_order=args.net_order,
            heap_tie_breaker=args.heap_tie_breaker,
            max_iterations=args.max_iterations,
            routing_window_scale=args.routing_window_scale,
            ripup_reroute_config=RipupRerouteConfig(
                enabled=args.ripup_reroute,
                max_rounds=args.ripup_max_rounds,
                max_victims_per_failure=args.ripup_max_victims,
                history_weight=args.ripup_history_weight,
                history_increment=args.ripup_history_increment,
            ),
        ),
        path_length=PathLengthMatchingOptions(
            enable_path_length_matching=args.path_length_matching,
            path_length_match_outputs=args.path_length_match_outputs,
            path_length_meander_height_um=args.path_length_meander_height_um,
        ),
        electrical=ElectricalOptions(
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
        ),
        debug=DebugArtifactOptions(
            debug_svgs=args.debug_svgs,
            debug_timing=args.debug_timing,
            debug_stop_after_route_index=args.debug_stop_after_route,
            debug_meanders=args.debug_meanders,
            verbose_routes=args.verbose_routes,
            collect_attempt_diagnostics=args.attempt_diagnostics,
        ),
        output=OutputOptions(show_klayout=args.show_klayout),
    )


if __name__ == "__main__":
    main()
