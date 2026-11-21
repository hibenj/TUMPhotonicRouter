"""Optical routing stage for routing_flow.

This module owns the call into `route_match_and_realize()`:
`build_optical_routing_stage_config` turns the caller's `FlowOptions` and what
the earlier stages decided (`OpticalStageInputs`) into the session's options,
and `run_photonic_routing_stage` runs the session and collects its records.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.benchmark_metadata import load_benchmark_metadata
from photonic_router.config import RoutingConfig
from photonic_router.flow_options import FlowOptions
from photonic_router.flow_reporting import (
    report_optical_timing,
    report_partial_debug_artifacts,
    route_attempt_as_dict,
)
from photonic_router.records.route_rust_types import (
    RouteRustPipelineResult,
    RouteSearchSummary,
    RustRouteDebugArtifacts,
)
from photonic_router.routing.api import route_match_and_realize
from photonic_router.routing.options import (
    CircuitMetadata,
    CrossingOptions,
    DebugOptions,
    MatchingOptions,
    PortAccessOptions,
    RealizationOptions,
    SearchOptions,
    SessionOptions,
)
from photonic_router.static_obstacle_builder import StaticObstacleMapConfig


@dataclass(frozen=True)
class OpticalStageInputs:
    """What the stages before the optical one decided for it.

    The flow negotiates the crossing configuration between the pre-placed
    grids and the router, resolves the net order and builds the obstacle
    configuration; these are the results, as opposed to the caller's
    `FlowOptions`.
    """

    #: the crossing setup after contribution 2's fallback was decided, with
    #: the placement analysis when the search is guided
    crossings: CrossingOptions
    net_order: str
    #: contribution 2: instance depths of the original netlist for the net
    #: order (the derived netlist's tile stubs would scramble the layers)
    net_order_depth_by_node: dict[str, int] | None
    obstacle_config: StaticObstacleMapConfig
    debug_dir: Path | None
    debug_route_indices: set[int] | None
    debug_svgs_enabled: bool


@dataclass(frozen=True)
class OpticalRoutingStageConfig:
    """The resolved inputs of the optical routing stage."""

    session: SessionOptions
    matching: MatchingOptions
    debug_svgs_enabled: bool


def build_optical_routing_stage_config(
    options: FlowOptions, config: RoutingConfig, inputs: OpticalStageInputs
) -> OpticalRoutingStageConfig:
    """Combine the caller's options with what the earlier stages decided."""
    optical = options.optical
    debug = options.debug
    path_length = options.path_length
    search = SearchOptions(
        allow_45_degree_turns=options.layout.allow_45_degree_turns,
        use_indexed_heap=optical.use_indexed_heap,
        enable_simple_routes=optical.enable_simple_routes,
        primitive_ordering=optical.primitive_ordering,
        heuristic_mode=optical.heuristic_mode,
        heap_tie_breaker=optical.heap_tie_breaker,
        proactive_congestion_weight=float(optical.proactive_congestion_weight),
        proactive_congestion_radius_cells=int(optical.proactive_congestion_radius_cells),
        max_iterations=optical.max_iterations,
        routing_window_scale=optical.routing_window_scale,
        net_order=inputs.net_order,
        net_order_depth_by_node=inputs.net_order_depth_by_node,
        detour_pass_factor=optical.detour_pass_factor,
    )
    session = SessionOptions(
        search=search,
        crossings=inputs.crossings,
        port_access=PortAccessOptions(
            foreign_port_keepout_cells=int(optical.foreign_port_keepout_cells),
            fanout_access_mode=optical.fanout_access_mode,
        ),
        debug=DebugOptions(
            debug_dir=inputs.debug_dir,
            debug_prefix=options.loading.benchmark_name.lower(),
            debug_route_indices=inputs.debug_route_indices,
            debug_stop_after_route_index=debug.debug_stop_after_route_index,
            debug_timing=debug.debug_timing,
            verbose_route_diagnostics=debug.verbose_routes or debug.debug_meanders,
            collect_route_stats=options.stats.collect_route_stats,
            collect_attempt_diagnostics=debug.collect_attempt_diagnostics,
        ),
        realization=RealizationOptions(bend_radius_um=options.layout.bend_radius_um),
        obstacle_config=inputs.obstacle_config,
        include_heater_obstacles=options.obstacles.include_heater_obstacles,
        ripup_reroute_config=optical.ripup_reroute_config,
        config=config,
    )
    matching = MatchingOptions(
        enable_path_length_matching=path_length.enable_path_length_matching,
        path_length_match_outputs=path_length.path_length_match_outputs,
        path_length_declared_unmatched_groups=path_length.path_length_declared_unmatched_groups,
        path_length_output_scope=path_length.path_length_output_scope,
        path_length_meander_height_um=path_length.path_length_meander_height_um,
        debug_meanders=debug.debug_meanders,
    )
    return OpticalRoutingStageConfig(
        session=session, matching=matching, debug_svgs_enabled=inputs.debug_svgs_enabled
    )


@dataclass(frozen=True)
class OpticalRoutingStageResult:
    """What run_photonic_routing_stage returns to the flow.

    The pipeline result, plus what the later stages read out of it:
    routed_layout, debug_artifacts and route_summary are the pipeline
    result's own layout, debug artifacts and route search summary.
    route_attempt_records are the attempt records as dicts (records that
    convert to an empty dict are dropped). route_time_s is the wall time of
    the routing call in seconds, measured whether or not timing is printed.
    """

    route_result: RouteRustPipelineResult
    routed_layout: Component
    debug_artifacts: RustRouteDebugArtifacts
    route_summary: RouteSearchSummary
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
            config.session,
            matching=config.matching,
            metadata=CircuitMetadata(
                node_types=metadata.get("node_types"),
                internal_delays_um=metadata.get("internal_delays_um"),
                input_ports_by_component=metadata.get("input_ports_by_component"),
            ),
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
    if config.session.debug.debug_timing:
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
