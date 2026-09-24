"""Compatibility shim over the routing entry points (Milestone 5, Slice 3).

The flow itself now lives in `photonic_router.flow` (`route_benchmark`,
`route_schematic`, and the `run_routing_flow` keyword wrapper over them) and the
command line in `photonic_router.cli`, so the human entry point is

    python -m photonic_router route <benchmark> --configuration contribution2

This module stays because the reproduction scripts
(`scripts/results/run_and_archive.sh`), the benchmark stable blocks, the
benchmark drivers in `scripts/` and the tests all call `python routing_flow.py
<benchmark> [flags]` or import names from here (D4, D6). Everything below is a
re-export of the module that now owns it - no behaviour lives in this file.
Milestone 7 retires it.
"""

from photonic_router.cli import (  # noqa: F401
    _benchmark_stable_defaults,
    _build_arg_parser,
    _CONFIGURATION_FLAGS,
    _CROSSING_DISCOVERY_FLAGS,
    _parse_bool_flag,
    _requests_preplaced_crossing_grids,
    configuration_flags,
    main,
    stable_flags_for_configuration,
)
from photonic_router.config import RoutingConfig  # noqa: F401
from photonic_router.config_loading import build_config  # noqa: F401
from photonic_router.flow import (  # noqa: F401
    _depth_by_node_from_schematic,
    _layout_from_schematic_stage,
    _load_benchmark_stage,
    _preplaced_crossing_grids_stage,
    _print_flow_footer,
    _print_flow_header,
    layout_from_schematic,
    load_benchmark,
    route_benchmark,
    route_schematic,
    run_photonic_routing_stage,
    run_routing_flow,
)
from photonic_router.flow_options import FlowOptions  # noqa: F401
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig  # noqa: F401
from routing_flow_config import (  # noqa: F401
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
from routing_flow_electrical import run_electrical_routing_step  # noqa: F401
from routing_flow_plm import attach_and_report_path_length_matching  # noqa: F401
from routing_flow_reporting import (  # noqa: F401
    _format_debug_route_indices,
    cleanup_debug_artifacts,
    report_and_open_debug_svgs,
    write_or_show_routed_layout,
)
from routing_flow_stats import (  # noqa: F401
    RoutingFlowStats,
    populate_route_stats,
    record_initial_route_stats,
)
from routing_flow_verification import (  # noqa: F401
    _verification_status_metadata,
    verify_and_attach_photonic_reports,
)
from translation.crossing_modes import CROSSING_MODES, is_guided_mode  # noqa: F401
from translation.electrical import (  # noqa: F401
    ElectricalRoutingConfig,
    ElectricalRoutingResult,
)
from translation.route_order import NET_ORDERS, default_net_order  # noqa: F401
from translation.route_rust import RipupRerouteConfig  # noqa: F401

# Compatibility alias: this was a private module-level function before the
# 2026-08-11 routing-flow readability refactor moved it (and made it public)
# in routing_flow_config.py; kept here under its old name for existing
# imports (see tests/test_routing_flow_stats.py).
_parse_debug_svg_selector = parse_debug_svg_selector


if __name__ == "__main__":
    main()
