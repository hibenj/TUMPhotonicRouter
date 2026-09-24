"""Flow options: the 48 keyword arguments of `run_routing_flow` as one frozen
tree, grouped by the stage that consumes them (Milestone 5, Slice 3).

`FlowOptions` is the second argument of `photonic_router.flow.route_benchmark`
and `route_schematic`; the first is the `RoutingConfig` built by
`photonic_router.config_loading.build_config`. The split is deliberate:
`RoutingConfig` carries the router's own settings (the former
`PHOTONIC_ROUTER_*` overlay, Milestone 1), `FlowOptions` carries the flow-level
choices that used to be `run_routing_flow`'s keywords - which benchmark, which
stages run, which artifacts are written.

Every default below is copied from `run_routing_flow`'s signature, so
`FlowOptions()` is exactly today's default flow (see
`tests/test_flow_entry_points.py`, which introspects the wrapper's signature so
the two cannot drift apart). The single exception is `loading.benchmark_name`:
`run_routing_flow` takes it positionally without a default, so the group uses
the command line's own default (`SCRIPT_BENCHMARK`).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow_config import (
    DebugSvgSelector,
    SCRIPT_ALLOW_45_DEGREE_TURNS,
    SCRIPT_BENCHMARK,
    SCRIPT_BEND_RADIUS_UM,
    SCRIPT_CHIP_ADD_X_UM,
    SCRIPT_CHIP_ADD_Y_UM,
    SCRIPT_FANOUT_ACCESS_MODE,
    SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS,
    SCRIPT_GRID_SIZE_UM,
    SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM,
    SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS,
    SCRIPT_PROACTIVE_CONGESTION_WEIGHT,
)
from routing_flow_stats import RoutingFlowStats
from translation.electrical import ElectricalRoutingConfig
from translation.route_rust import RipupRerouteConfig


@dataclass(frozen=True)
class LoadingOptions:
    """Stage 1, `load_benchmark`: which benchmark module the flow routes."""

    benchmark_name: str = SCRIPT_BENCHMARK


@dataclass(frozen=True)
class LayoutOptions:
    """Waveguide geometry the layout and every routing stage must agree on.

    Read by the optical stage's primitive library, and `bend_radius_um` also by
    the pre-placed crossing grid stage's static-fan-out probe, which has to see
    the routing run's own geometry.
    """

    allow_45_degree_turns: bool = SCRIPT_ALLOW_45_DEGREE_TURNS
    bend_radius_um: float = SCRIPT_BEND_RADIUS_UM


@dataclass(frozen=True)
class ObstacleOptions:
    """`build_static_obstacle_config`: the static optical obstacle map.

    `include_heater_obstacles` decides whether the heater/metal layers are
    obstacles at all, so the optical stage, the verification stage and the
    pre-placed grid probe all read it from here.
    """

    static_obstacle_config: StaticObstacleMapConfig | None = None
    grid_size_um: float = SCRIPT_GRID_SIZE_UM
    waveguide_clearance_um: float | None = None
    heater_clearance_um: float | None = None
    obstacle_clearance_um: float | None = None
    chip_add_x_um: float = SCRIPT_CHIP_ADD_X_UM
    chip_add_y_um: float = SCRIPT_CHIP_ADD_Y_UM
    include_heater_obstacles: bool = False


@dataclass(frozen=True)
class PreplacedCrossingGridOptions:
    """Contribution 2: the pre-placed crossing grid stage (stage 2b)."""

    preplaced_crossing_grids: bool = False


@dataclass(frozen=True)
class OpticalStageOptions:
    """`build_optical_routing_stage_config`: the optical routing stage."""

    enable_crossings: bool = False
    crossing_mode: str = "lidar-pure"
    crossing_half_size_cells: int = 0
    min_straight_cells_per_crossing: int = SCRIPT_MIN_STRAIGHT_CELLS_PER_CROSSING
    foreign_port_keepout_cells: int = SCRIPT_FOREIGN_PORT_KEEPOUT_CELLS
    fanout_access_mode: str | None = SCRIPT_FANOUT_ACCESS_MODE
    proactive_congestion_weight: float = SCRIPT_PROACTIVE_CONGESTION_WEIGHT
    proactive_congestion_radius_cells: int = SCRIPT_PROACTIVE_CONGESTION_RADIUS_CELLS
    enable_jps4: bool = False
    use_indexed_heap: bool = False
    enable_simple_routes: bool = True
    primitive_ordering: str = "library"
    heuristic_mode: str = "heading_aware"
    net_order: str | None = None
    heap_tie_breaker: str = "smaller_g"
    max_iterations: int = 500_000
    routing_window_scale: float | None = None
    ripup_reroute_config: RipupRerouteConfig | None = None


@dataclass(frozen=True)
class VerificationOptions:
    """`verify_and_attach_photonic_reports`: the post-route verification stage.

    Deliberately empty: the stage always runs, and its three inputs come from
    elsewhere - `obstacles.include_heater_obstacles`,
    `debug.debug_stop_after_route_index` and
    `RoutingConfig.write_gds_on_photonic_verification_failure`. The group is
    named so the boundary is visible in the option tree.
    """


@dataclass(frozen=True)
class PathLengthMatchingOptions:
    """`attach_and_report_path_length_matching` and the optical stage's
    path-length requirements."""

    enable_path_length_matching: bool = False
    path_length_match_outputs: bool = False
    path_length_meander_height_um: float = SCRIPT_PATH_LENGTH_MEANDER_HEIGHT_UM


@dataclass(frozen=True)
class ElectricalOptions:
    """`run_electrical_routing_step`: the heater-metal routing stage."""

    enable_electrical_routing: bool = False
    electrical_config: ElectricalRoutingConfig | None = None


@dataclass(frozen=True)
class DebugArtifactOptions:
    """Debug artifacts and verbosity: SVGs, timings, per-net prints.

    `show_debug_svgs` and `show_static_obstacles_svg` are the legacy aliases
    `resolve_legacy_display_options` folds into `debug_svgs`.
    """

    debug_svgs: DebugSvgSelector = False
    show_debug_svgs: DebugSvgSelector | None = None
    show_static_obstacles_svg: bool | None = None
    debug_timing: bool = False
    debug_stop_after_route_index: int | None = None
    debug_meanders: bool = False
    verbose_routes: bool = False
    collect_attempt_diagnostics: bool = False


@dataclass(frozen=True)
class OutputOptions:
    """`write_or_show_routed_layout`: where the routed layout goes.

    `show_routed` and `show_unrouted` are the legacy aliases
    `resolve_legacy_display_options` folds into `show_klayout`.
    """

    show_klayout: bool = False
    show_routed: bool | None = None
    show_unrouted: bool | None = None


@dataclass(frozen=True)
class StatsOptions:
    """The optional legacy stats collector filled in place by the stages."""

    collect_route_stats: bool = False
    stats: RoutingFlowStats | None = None


@dataclass(frozen=True)
class FlowOptions:
    """Every flow-level choice, grouped by consuming stage."""

    loading: LoadingOptions = field(default_factory=LoadingOptions)
    layout: LayoutOptions = field(default_factory=LayoutOptions)
    obstacles: ObstacleOptions = field(default_factory=ObstacleOptions)
    preplaced_grids: PreplacedCrossingGridOptions = field(
        default_factory=PreplacedCrossingGridOptions
    )
    optical: OpticalStageOptions = field(default_factory=OpticalStageOptions)
    verification: VerificationOptions = field(default_factory=VerificationOptions)
    path_length: PathLengthMatchingOptions = field(default_factory=PathLengthMatchingOptions)
    electrical: ElectricalOptions = field(default_factory=ElectricalOptions)
    debug: DebugArtifactOptions = field(default_factory=DebugArtifactOptions)
    output: OutputOptions = field(default_factory=OutputOptions)
    stats: StatsOptions = field(default_factory=StatsOptions)
