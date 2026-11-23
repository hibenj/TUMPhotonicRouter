"""Phase 9: realization and the assembly of the debug artifacts."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from photonic_router.crossings.route_rust_crossing_components import (
    place_realized_crossing_components,
)
from photonic_router.crossings.route_rust_crossing_plan import write_crossing_debug_artifacts
from photonic_router.records.route_rust_realization import (
    RealizationGeometry,
    realize_routed_net_records,
)
from photonic_router.records.route_rust_records import build_route_debug_artifacts
from photonic_router.records.route_rust_types import (
    RouteJob,
    RustRouteDebugArtifacts,
    summarize_route_search,
)
from photonic_router.routing import timing
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.stages import RoutedRecords, VerifiedRecords
from photonic_router.routing.state import SessionState


def realize_and_assemble_debug_artifacts(
    settings: SessionSettings,
    state: SessionState,
    route_jobs: list[RouteJob],
    routed: RoutedRecords,
    verified: VerifiedRecords,
) -> RustRouteDebugArtifacts:
    """Phase 9: realize the routes into the layout and assemble the debug artifacts.

    Adds one polygon per net and the crossing components unless
    `settings.defer_realization` is set, writes the crossing debug
    artifacts, and builds the artifacts record from the verified records,
    the search's log and the timings.

    Args:
        settings: the session's resolved inputs.
        state: the session state.
        route_jobs: the jobs of this run.
        routed: phase 7's product (after the detour pass, when it ran), for
            the search time and the detour count.
        verified: phase 8's product.

    Raises:
        RuntimeError: a realized crossing is still illegal after phase 8's
            repair loops; a second guard behind phase 8's own.
    """
    routed_net_records = verified.routed_net_records
    illegal_realized_crossings = verified.illegal_realized_crossings
    if not illegal_realized_crossings and not settings.defer_realization:
        t_direct_realization_start = timing.pipeline_timer_start(settings)
        realize_routed_net_records(
            state.routed_layout,
            routed_net_records,
            crossing_plan_info=state.crossing_plan_info.to_dict(),
            geometry=RealizationGeometry(
                route_width_um=settings.route_width_um,
                route_layer=settings.route_layer,
                realization_grid_spec=state.run.realization_grid_spec,
                allow_45_degree_turns=settings.allow_45_degree_turns,
                bend_radius_cells=state.kernel.radii.bend_radius_cells,
            ),
        )
        timing.record_pipeline_timing(
            settings, state, "direct_realization", t_direct_realization_start
        )
        place_realized_crossing_components(state.routed_layout, state.crossing_plan_info)
    elif state.crossing_plan_info.enabled:
        if state.crossing_plan_info.realized_crossing_components is None:
            state.crossing_plan_info.realized_crossing_components = []
        if state.crossing_plan_info.realized_crossing_component_count is None:
            state.crossing_plan_info.realized_crossing_component_count = 0
    write_crossing_debug_artifacts(
        debug_path=state.obstacles.debug_path
        if state.obstacles.debug_path is not None
        else Path("build"),
        debug_prefix=settings.debug_prefix,
        crossing_plan_info=state.crossing_plan_info,
    )
    if illegal_realized_crossings:
        preview = "; ".join(
            f"{item.get('net_name_a')} x {item.get('net_name_b')} "
            f"at {item.get('point_um')} ({item.get('reason')}, "
            f"margins={item.get('segment_a_margin_um')}/"
            f"{item.get('segment_b_margin_um')}, "
            f"required={item.get('required_margin_um')}, "
            f"grid={item.get('grid_cell')}, "
            f"route_endpoint_dists={item.get('route_endpoint_distance_a_um')}/"
            f"{item.get('route_endpoint_distance_b_um')}, "
            f"port_endpoint_dists={item.get('port_endpoint_distance_a_um')}/"
            f"{item.get('port_endpoint_distance_b_um')})"
            for item in illegal_realized_crossings[:5]
        )
        raise RuntimeError(
            "Illegal realized route crossing(s) after endpoint correction: "
            f"{len(illegal_realized_crossings)} found. {preview}"
        )

    t_debug_artifact_start = timing.pipeline_timer_start(settings)
    debug_artifacts = build_route_debug_artifacts(
        obstacle_svg=state.obstacles.obstacle_svg,
        route_svgs=state.search.route_svgs,
        obstacle_map=state.obstacles.obstacle_map,
        routed_net_records=routed_net_records,
        realization_grid_spec=state.run.realization_grid_spec,
        allow_45_degree_turns=settings.allow_45_degree_turns,
        bend_radius_cells=state.kernel.radii.bend_radius_cells,
        route_search_summary=summarize_route_search(
            state.search.timing_buckets,
            route_count=len(route_jobs),
            simple_route_count=state.search.simple_route_count,
            repair_count=state.search.repair_count,
            astar_elapsed_s=routed.astar_elapsed_s,
        ),
        route_attempt_records=state.search.attempt_records,
        route_nets_timings_s=state.route_nets_timings_s,
    )
    debug_artifacts = replace(
        debug_artifacts,
        crossing_plan_info=state.crossing_plan_info.to_dict(),
    )
    timing.record_pipeline_timing(
        settings, state, "debug_artifact_assembly", t_debug_artifact_start
    )
    if settings.collect_pipeline_timing:
        debug_artifacts = replace(
            debug_artifacts,
            route_nets_timings_s=dict(state.route_nets_timings_s),
        )
    debug_artifacts = replace(debug_artifacts, detour_pass_reroutes=routed.detour_pass_reroutes)
    return debug_artifacts
