"""Configuration defaults and option resolution for routing_flow."""

from pathlib import Path

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow_optical import OpticalRoutingStageConfig
from translation.electrical import (
    DEFAULT_BUS_WIDTH_UM,
    DEFAULT_PAD_PITCH_UM,
    DEFAULT_WIRE_WIDTH_UM,
)
from translation.route_rust import (
    DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    RipupRerouteConfig,
)
from translation.route_rust_types import DEFAULT_MEANDER_MAX_HEIGHT_UM

DebugSvgSelector = bool | int | str | range | set[int] | list[int] | tuple[int, ...]

# Edit these values when running `routing_flow.py` directly from an IDE or file.
# Command-line arguments override these defaults.
SCRIPT_BENCHMARK = "benes_16x16"
SCRIPT_DEBUG_SVGS: DebugSvgSelector = False  # Examples: True, "all", "5-10"
SCRIPT_DEBUG_TIMING = True
SCRIPT_DEBUG_MEANDERS = False
SCRIPT_VERBOSE_ROUTES = False
SCRIPT_SHOW_KLAYOUT = False
SCRIPT_ALLOW_45_DEGREE_TURNS = True
SCRIPT_BEND_RADIUS_UM = 5.0
SCRIPT_ENABLE_PATH_LENGTH_MATCHING = False
SCRIPT_PATH_LENGTH_MATCH_OUTPUTS = False
SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM = DEFAULT_MEANDER_MAX_HEIGHT_UM
SCRIPT_ENABLE_CROSSINGS = True
SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING
SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS = 6
SCRIPT_FANOUT_ACCESS_MODE = "legacy-runway"
SCRIPT_PROACTIVE_CONGESTION_WEIGHT = 0.0
SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS = 0
SCRIPT_MAX_ITERATIONS = 5_000_000
SCRIPT_ROUTING_WINDOW_SCALE = 0.05
SCRIPT_INCLUDE_HEATER_OBSTACLES = True
SCRIPT_OBSTACLE_MODE = "bounding_boxes"
SCRIPT_GRID_SIZE_UM = 2.0
SCRIPT_WAVEGUIDE_CLEARANCE_UM = 0.0
SCRIPT_HEATER_CLEARANCE_UM = 10.0
SCRIPT_CHIP_ADD_X_UM = 0.0
SCRIPT_CROSSING_MODE = "lidar-pure"
SCRIPT_CHIP_ADD_Y_UM = 0.0
SCRIPT_OBSTACLE_CLEARANCE_UM = SCRIPT_WAVEGUIDE_CLEARANCE_UM
SCRIPT_CLEAR_PORT_OPEN_CELLS_FROM_STATIC = False
SCRIPT_ENABLE_RIPUP_REROUTE = True
SCRIPT_RIPUP_MAX_ROUNDS = 4
SCRIPT_RIPUP_MAX_VICTIMS = 8
SCRIPT_RIPUP_HISTORY_WEIGHT = 2.0
SCRIPT_RIPUP_HISTORY_INCREMENT = 1
SCRIPT_ATTEMPT_DIAGNOSTICS = False
SCRIPT_ENABLE_ELECTRICAL_ROUTING = False
SCRIPT_ELECTRICAL_PAD_SIDE = "top"
SCRIPT_ELECTRICAL_GRID_PITCH_UM = 10.0
SCRIPT_ELECTRICAL_OBSTACLE_CLEARANCE_UM = 10.0
SCRIPT_ELECTRICAL_WIRE_WIDTH_UM = DEFAULT_WIRE_WIDTH_UM
SCRIPT_ELECTRICAL_BUS_WIDTH_UM = DEFAULT_BUS_WIDTH_UM
SCRIPT_ELECTRICAL_TERMINAL_CONTACT_WIDTH_UM = 10.0
SCRIPT_ELECTRICAL_PAD_PITCH_UM = DEFAULT_PAD_PITCH_UM


def parse_debug_svg_selector(
    debug_svgs: DebugSvgSelector,
) -> tuple[bool, set[int] | None]:
    """Parse debug SVG selection into an enabled flag and route-index set."""
    if isinstance(debug_svgs, bool):
        return debug_svgs, None
    if isinstance(debug_svgs, int):
        if debug_svgs < 1:
            raise ValueError("debug_svgs integer selectors must be >= 1")
        return True, {debug_svgs}
    if isinstance(debug_svgs, range):
        indices = set(debug_svgs)
        if any(index < 1 for index in indices):
            raise ValueError("debug_svgs range selectors must contain only indices >= 1")
        return True, indices
    if isinstance(debug_svgs, (set, list, tuple)):
        indices = {int(index) for index in debug_svgs}
        if any(index < 1 for index in indices):
            raise ValueError("debug_svgs sequence selectors must contain only indices >= 1")
        return bool(indices), indices
    if isinstance(debug_svgs, str):
        selector = debug_svgs.strip().lower()
        if selector in {"", "false", "off", "none", "no"}:
            return False, None
        if selector in {"true", "on", "yes", "all", "*"}:
            return True, None

        indices: set[int] = set()
        for part in selector.split(","):
            token = part.strip()
            if not token:
                continue
            if "-" in token:
                start_text, end_text = token.split("-", 1)
                start = int(start_text.strip())
                end = int(end_text.strip())
                if start < 1 or end < 1:
                    raise ValueError("debug_svgs range selectors must use indices >= 1")
                if start > end:
                    raise ValueError(f"debug_svgs range start must be <= end: {token!r}")
                indices.update(range(start, end + 1))
            else:
                index = int(token)
                if index < 1:
                    raise ValueError("debug_svgs route selectors must be >= 1")
                indices.add(index)
        if not indices:
            return False, None
        return True, indices

    raise TypeError("debug_svgs must be a bool, int, range, sequence of ints, or selector string")


def resolve_legacy_display_options(
    *,
    debug_svgs: DebugSvgSelector,
    show_klayout: bool,
    show_unrouted: bool | None,
    show_routed: bool | None,
    show_debug_svgs: DebugSvgSelector | None,
    show_static_obstacles_svg: bool | None,
) -> tuple[DebugSvgSelector, bool]:
    """Normalize legacy display aliases while preserving the public API."""
    if show_routed is not None:
        show_klayout = bool(show_routed)
    if show_debug_svgs is not None:
        debug_svgs = show_debug_svgs
    if show_static_obstacles_svg is not None:
        debug_svgs = bool(show_static_obstacles_svg)
    if show_unrouted is not None:
        # Historical argument kept for compatibility.
        pass
    return debug_svgs, show_klayout


def build_static_obstacle_config(
    *,
    static_obstacle_config: StaticObstacleMapConfig | None,
    grid_size_um: float,
    waveguide_clearance_um: float | None,
    heater_clearance_um: float | None,
    obstacle_clearance_um: float | None,
    chip_add_x_um: float,
    chip_add_y_um: float,
) -> tuple[StaticObstacleMapConfig, float, float]:
    """Resolve clearance aliases and construct the optical obstacle config."""
    if waveguide_clearance_um is None:
        waveguide_clearance_um = (
            float(obstacle_clearance_um)
            if obstacle_clearance_um is not None
            else SCRIPT_WAVEGUIDE_CLEARANCE_UM
        )
    if heater_clearance_um is None:
        heater_clearance_um = float(waveguide_clearance_um)

    config = static_obstacle_config or StaticObstacleMapConfig(
        grid_size_um=float(grid_size_um),
        obstacle_mode="bounding_boxes",
        clearance_um=float(waveguide_clearance_um),
        heater_clearance_um=float(heater_clearance_um),
        chip_add_x_um=float(chip_add_x_um),
        chip_add_y_um=float(chip_add_y_um),
        clear_port_open_cells_from_static=False,
    )
    return config, float(waveguide_clearance_um), float(heater_clearance_um)


def debug_artifact_routing_options(
    *,
    debug_svgs_enabled: bool,
    debug_route_indices: set[int] | None,
    collect_attempt_diagnostics: bool,
) -> tuple[Path | None, set[int] | None]:
    """Return the debug directory and route selector passed to optical routing."""
    debug_dir = Path("build") if debug_svgs_enabled or collect_attempt_diagnostics else None
    route_debug_indices = debug_route_indices
    if not debug_svgs_enabled and collect_attempt_diagnostics:
        route_debug_indices = set()
    return debug_dir, route_debug_indices


def build_optical_routing_stage_config(
    *,
    enable_path_length_matching: bool,
    path_length_match_outputs: bool,
    path_length_meander_height_um: float,
    enable_crossings: bool,
    crossing_mode: str,
    crossing_half_size_cells: int,
    min_straight_cells_per_crossing: int,
    foreign_port_keepout_cells: int,
    fanout_access_mode: str | None,
    proactive_congestion_weight: float,
    proactive_congestion_radius_cells: int,
    allow_45_degree_turns: bool,
    bend_radius_um: float,
    enable_jps4: bool,
    use_indexed_heap: bool,
    enable_simple_routes: bool,
    primitive_ordering: str,
    heuristic_mode: str,
    net_order: str,
    heap_tie_breaker: str,
    max_iterations: int,
    routing_window_scale: float | None,
    include_heater_obstacles: bool,
    ripup_reroute_config: RipupRerouteConfig | None,
    route_static_obstacle_config: StaticObstacleMapConfig,
    debug_dir: Path | None,
    route_debug_indices: set[int] | None,
    debug_stop_after_route_index: int | None,
    debug_timing: bool,
    debug_meanders: bool,
    verbose_routes: bool,
    debug_svgs_enabled: bool,
    collect_route_stats: bool,
    collect_attempt_diagnostics: bool,
    stats: object | None,
    crossing_guidance_net_names: frozenset[str] | None = None,
) -> OpticalRoutingStageConfig:
    """Collect public flow arguments into the optical-stage config object."""
    return OpticalRoutingStageConfig(
        enable_path_length_matching=enable_path_length_matching,
        path_length_match_outputs=path_length_match_outputs,
        path_length_meander_height_um=path_length_meander_height_um,
        enable_crossings=enable_crossings,
        crossing_mode=crossing_mode,
        crossing_guidance_net_names=crossing_guidance_net_names,
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
        obstacle_config=route_static_obstacle_config,
        debug_dir=debug_dir,
        debug_route_indices=route_debug_indices,
        debug_stop_after_route_index=debug_stop_after_route_index,
        debug_timing=debug_timing,
        debug_meanders=debug_meanders,
        verbose_routes=verbose_routes,
        debug_svgs_enabled=debug_svgs_enabled,
        collect_route_stats=collect_route_stats or stats is not None,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
    )
