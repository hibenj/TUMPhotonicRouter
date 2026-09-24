"""Phase 9: realization and the assembly of the debug artifacts."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any

from translation.route_rust_crossing_components import _place_realized_crossing_components
from translation.route_rust_crossing_plan import _write_crossing_debug_artifacts
from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_records import build_route_debug_artifacts
from translation.route_rust_types import (
    RouteJob,
    RoutedNetRecord,
    RustRouteDebugArtifacts,
    summarize_route_search,
)

from translation.routing import timing
from translation.routing.settings import SessionSettings
from translation.routing.state import SessionState

def realize_and_assemble_debug_artifacts(
    settings: SessionSettings,
    state: SessionState,
    route_jobs: list[RouteJob],
    routed_net_records: list[RoutedNetRecord],
    illegal_realized_crossings: list[dict[str, object]],
    obstacle_map: Any,
    obstacle_svg: Path | None,
    astar_elapsed_s: float,
) -> RustRouteDebugArtifacts:
    """Realize final geometry (unless deferred), guard against any still-illegal crossing, and assemble debug artifacts.

    Performs direct geometry realization (`realize_routed_net_records`,
    `_place_realized_crossing_components`) unless `self.defer_realization` is set,
    writes crossing debug artifacts, raises a final `RuntimeError` if
    `illegal_realized_crossings` is still non-empty after the repair loops in
    `_repair_and_verify_final_geometry` (a second, separate guard from that method's
    own mid-loop failure raise — this one covers the case where realization proceeds
    with the same list unexpectedly still populated), then assembles and returns the
    debug artifacts bundle.
    """
    if not illegal_realized_crossings and not settings.defer_realization:
        t_direct_realization_start = timing.pipeline_timer_start(settings, state)
        realize_routed_net_records(
            state.routed_layout,
            routed_net_records,
            route_width_um=settings.route_width_um,
            route_layer=settings.route_layer,
            realization_grid_spec=state.realization_grid_spec,
            allow_45_degree_turns=settings.allow_45_degree_turns,
            bend_radius_cells=state.bend_radius_cells,
            crossing_plan_info=state.crossing_plan_info.to_dict(),
            enable_endpoint_correction=settings.enable_checked_endpoint_correction,
        )
        timing.record_pipeline_timing(
            settings, state, "direct_realization", t_direct_realization_start
        )
        _place_realized_crossing_components(state.routed_layout, state.crossing_plan_info)
    elif state.crossing_plan_info.enabled:
        if state.crossing_plan_info.realized_crossing_components is None:
            state.crossing_plan_info.realized_crossing_components = []
        if state.crossing_plan_info.realized_crossing_component_count is None:
            state.crossing_plan_info.realized_crossing_component_count = 0
    _write_crossing_debug_artifacts(
        debug_path=state.debug_path if state.debug_path is not None else Path("build"),
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

    t_debug_artifact_start = timing.pipeline_timer_start(settings, state)
    debug_artifacts = build_route_debug_artifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=state.route_svgs,
        obstacle_map=obstacle_map,
        routed_net_records=routed_net_records,
        realization_grid_spec=state.realization_grid_spec,
        allow_45_degree_turns=settings.allow_45_degree_turns,
        bend_radius_cells=state.bend_radius_cells,
        route_search_summary=summarize_route_search(
            state.route_timing_buckets,
            route_count=len(route_jobs),
            simple_route_count=state.simple_route_count,
            repair_count=state.repair_count,
            deferred_count=state.deferred_count,
            astar_elapsed_s=astar_elapsed_s,
        ),
        route_attempt_records=state.route_attempt_records,
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
    return debug_artifacts
