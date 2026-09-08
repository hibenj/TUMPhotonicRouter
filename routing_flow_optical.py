"""Optical routing stage for routing_flow.

This module owns the Python-side call into `route_match_and_realize()`. Keeping
the large router option set here lets `run_routing_flow()` read as orchestration
instead of as a full argument map for the Rust bridge.
"""

from dataclasses import dataclass
from pathlib import Path
import time

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from benchmark_metadata import load_benchmark_metadata
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig
from routing_flow_reporting import (
    report_optical_timing,
    report_partial_debug_artifacts,
    route_attempt_as_dict,
)
from translation.route_rust import RipupRerouteConfig, route_match_and_realize


@dataclass(frozen=True)
class OpticalRoutingStageConfig:
    """Resolved options needed by the optical routing stage."""

    enable_path_length_matching: bool
    path_length_match_outputs: bool
    path_length_meander_height_um: float
    enable_crossings: bool
    crossing_mode: str
    crossing_half_size_cells: int
    min_straight_cells_per_crossing: int
    foreign_port_keepout_cells: int
    fanout_access_mode: str | None
    proactive_congestion_weight: float
    proactive_congestion_radius_cells: int
    allow_45_degree_turns: bool
    bend_radius_um: float
    enable_jps4: bool
    use_indexed_heap: bool
    enable_simple_routes: bool
    primitive_ordering: str
    heuristic_mode: str
    net_order: str
    heap_tie_breaker: str
    max_iterations: int
    routing_window_scale: float | None
    include_heater_obstacles: bool
    ripup_reroute_config: RipupRerouteConfig | None
    obstacle_config: StaticObstacleMapConfig
    debug_dir: Path | None
    debug_route_indices: set[int] | None
    debug_stop_after_route_index: int | None
    debug_timing: bool
    debug_meanders: bool
    verbose_routes: bool
    debug_svgs_enabled: bool
    collect_route_stats: bool
    collect_attempt_diagnostics: bool
    # contribution 2 router fallback: nets of layers without a pre-placed
    # structure; the guided search only sees plan events between them
    crossing_guidance_net_names: frozenset[str] | None = None


@dataclass(frozen=True)
class OpticalRoutingStageResult:
    route_result: object
    routed_layout: Component
    debug_artifacts: object
    route_summary: object
    route_attempt_records: list[dict[str, object]]
    route_time_s: float


def run_photonic_routing_stage(
    *,
    benchmark_name: str,
    schematic: Schematic,
    unrouted_layout: Component,
    total_steps: int,
    config: OpticalRoutingStageConfig,
) -> OpticalRoutingStageResult:
    """Route optical nets, collect route records, and emit optional timing."""
    print(f"\n[3/{total_steps}] Routing nets with Rust backend...")
    metadata = load_benchmark_metadata(benchmark_name, schematic=schematic)
    t_route_start = time.perf_counter()
    try:
        route_result = route_match_and_realize(
            unrouted_layout,
            schematic,
            enable_path_length_matching=config.enable_path_length_matching,
            path_length_match_outputs=config.path_length_match_outputs,
            node_types=metadata.get("node_types"),
            internal_delays_um=metadata.get("internal_delays_um"),
            enable_crossings=config.enable_crossings,
            crossing_mode=config.crossing_mode,
            crossing_guidance_net_names=config.crossing_guidance_net_names,
            crossing_half_size_cells=int(config.crossing_half_size_cells),
            min_straight_cells_per_crossing=int(config.min_straight_cells_per_crossing),
            foreign_port_keepout_cells=int(config.foreign_port_keepout_cells),
            fanout_access_mode=config.fanout_access_mode,
            node_depths=metadata.get("node_depths"),
            node_ranks=metadata.get("node_ranks"),
            edge_ranks=metadata.get("edge_ranks"),
            debug_dir=config.debug_dir,
            debug_prefix=benchmark_name.lower(),
            debug_route_indices=config.debug_route_indices,
            debug_stop_after_route_index=config.debug_stop_after_route_index,
            debug_timing=config.debug_timing,
            verbose_route_diagnostics=config.verbose_routes or config.debug_meanders,
            allow_45_degree_turns=config.allow_45_degree_turns,
            bend_radius_um=config.bend_radius_um,
            enable_jps4=config.enable_jps4,
            use_indexed_heap=config.use_indexed_heap,
            enable_simple_routes=config.enable_simple_routes,
            primitive_ordering=config.primitive_ordering,
            heuristic_mode=config.heuristic_mode,
            net_order=config.net_order,
            heap_tie_breaker=config.heap_tie_breaker,
            proactive_congestion_weight=float(config.proactive_congestion_weight),
            proactive_congestion_radius_cells=int(config.proactive_congestion_radius_cells),
            max_iterations=config.max_iterations,
            routing_window_scale=config.routing_window_scale,
            collect_route_stats=config.collect_route_stats,
            collect_attempt_diagnostics=config.collect_attempt_diagnostics,
            include_heater_obstacles=config.include_heater_obstacles,
            ripup_reroute_config=config.ripup_reroute_config,
            path_length_meander_height_um=config.path_length_meander_height_um,
            enable_grid_endpoint_correction=True,
            obstacle_config=config.obstacle_config,
        )
    except Exception:
        print("      \u2717 Routing failed.")
        report_partial_debug_artifacts(
            benchmark_name,
            debug_svgs_enabled=config.debug_svgs_enabled,
        )
        raise

    t_route_end = time.perf_counter()
    debug_artifacts = route_result.debug_artifacts
    route_summary = debug_artifacts.route_search_summary
    route_attempt_records = [
        record_dict
        for record in getattr(debug_artifacts, "route_attempt_records", ())
        if (record_dict := route_attempt_as_dict(record))
    ]
    route_time_s = t_route_end - t_route_start
    if config.debug_timing:
        report_optical_timing(
            route_result=route_result,
            route_summary=route_summary,
            route_attempt_records=route_attempt_records,
            route_time_s=route_time_s,
        )
    print(f"      \u2713 Routed layout generated: {route_result.routed_layout.name}")
    return OpticalRoutingStageResult(
        route_result=route_result,
        routed_layout=route_result.routed_layout,
        debug_artifacts=debug_artifacts,
        route_summary=route_summary,
        route_attempt_records=route_attempt_records,
        route_time_s=route_time_s,
    )
