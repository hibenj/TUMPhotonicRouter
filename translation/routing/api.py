"""The module-level routing API: the full match-and-realize pipeline, the static
fan-out anchor query and the path-length-matching re-export wrappers."""

from __future__ import annotations

import time

from dataclasses import replace
from pathlib import Path
from typing import Any

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.config import RoutingConfig
from photonic_router.static_obstacle_builder import (
    _load_rust_backend,
    build_static_obstacle_map,
)

from translation import route_rust_meanders as _meander_impl
from translation.route_rust_analysis import (
    analysis_to_info_dict,
    analyze_path_length_matching,
    build_requirement_delay_candidates,
    compute_group_lifted_requirements,
    compute_output_matching_requirements,
    delay_candidate_to_dict,
    format_path_length_acceptance_failure,
    matching_group_diagnostics_to_info,
    merge_missing_length_requirements,
    minimum_four_bend_extra_length_um,
    output_matching_diagnostics_to_info,
    path_length_acceptance_summary,
    requirement_to_dict,
)
from translation.route_rust_crossing_components import _place_realized_crossing_components
from translation.route_rust_crossing_plan import (
    _augment_insertion_loss_report_from_realized_intersections,
    _routed_records_by_net_id,
    _write_crossing_debug_artifacts,
)
from translation.route_rust_crossing_verification import _verify_realized_route_intersections
from translation.route_rust_obstacle_config import (
    _resolve_obstacle_config,
    _with_bbox_cell_materialization,
    _with_obstacle_mode,
)
from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_records import build_port_alignment_diagnostics
from translation.route_rust_types import (
    DEFAULT_MEANDER_MAX_HEIGHT_UM,
    MeanderInsertionConfig,
    OpticalRouteClearancePolicy,
    RipupRerouteConfig,
    RouteRustPipelineResult,
    _as_float,
)

from translation.routing.crossing_plan_info import CrossingPlanInfo
from translation.routing.session import (
    DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    _RouteNetsRustSession,
    route_nets_rust,
)

# re-exported here as part of the module-level routing API
from translation.routing.router_setup import (  # noqa: F401
    DENSE_OBSTACLE_CELL_CAP_MARGIN,
    dense_obstacle_cell_cap,
)

from translation.routing.obstacle_context import build_static_obstacle_context
from translation.routing.route_jobs import build_route_jobs_and_fanout_clustering
from translation.routing.router_setup import configure_router_and_grid

def analyze_meander_insertion_for_requirements(*args: Any, **kwargs: Any):
    _meander_impl._load_rust_backend = _load_rust_backend
    return _meander_impl.analyze_meander_insertion_for_requirements(*args, **kwargs)


def insert_meanders_for_requirements(*args: Any, **kwargs: Any):
    _meander_impl._load_rust_backend = _load_rust_backend
    return _meander_impl.insert_meanders_for_requirements(*args, **kwargs)


def route_match_and_realize(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    enable_path_length_matching: bool = False,
    path_length_match_outputs: bool = False,
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
    enable_crossings: bool = False,
    node_depths: dict[str, int] | None = None,
    node_ranks: dict[str, int] | None = None,
    edge_ranks: dict[str, dict[str, int]] | None = None,
    crossing_loss: float = 0.0,
    crossing_mode: str = "lidar-pure",
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
    fanout_access_mode: str | None = None,
    allow_only_expected_crossings: bool = True,
    crossing_guidance_net_names: frozenset[str] | None = None,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    debug_route_indices: set[int] | None = None,
    debug_stop_after_route_index: int | None = None,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    bend_radius_um: float | None = None,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    enable_simple_routes: bool = True,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    net_order: str = "topological",
    net_order_depth_by_node: dict[str, int] | None = None,
    heap_tie_breaker: str = "smaller_g",
    proactive_congestion_weight: float = 0.0,
    proactive_congestion_radius_cells: int = 0,
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    debug_timing: bool = False,
    verbose_route_diagnostics: bool = False,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    enable_internal_photonic_probe_verification: bool = False,
    include_heater_obstacles: bool = False,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    path_length_meander_height_um: float = DEFAULT_MEANDER_MAX_HEIGHT_UM,
    enable_grid_endpoint_correction: bool = True,
    config: RoutingConfig | None = None,
) -> RouteRustPipelineResult:
    """Run Phase A->(optional M1)->B entirely in route_rust."""
    route_obstacle_config = obstacle_config
    if debug_dir is None:
        route_obstacle_config = _with_bbox_cell_materialization(
            obstacle_config,
            materialize_bbox_cells=False,
            populate_obstacle_map=False,
        )

    t_route_nets_start = time.perf_counter()
    routed_layout, debug_artifacts = route_nets_rust(
        unrouted_layout,
        schematic,
        obstacle_config=route_obstacle_config,
        debug_dir=debug_dir,
        debug_prefix=debug_prefix,
        debug_route_indices=debug_route_indices,
        debug_stop_after_route_index=debug_stop_after_route_index,
        route_width_um=route_width_um,
        route_layer=route_layer,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_um=bend_radius_um,
        enable_jps4=enable_jps4,
        use_indexed_heap=use_indexed_heap,
        enable_simple_routes=enable_simple_routes,
        primitive_ordering=primitive_ordering,
        heuristic_mode=heuristic_mode,
        net_order=net_order,
        net_order_depth_by_node=net_order_depth_by_node,
        heap_tie_breaker=heap_tie_breaker,
        proactive_congestion_weight=proactive_congestion_weight,
        proactive_congestion_radius_cells=proactive_congestion_radius_cells,
        max_iterations=max_iterations,
        routing_window_scale=routing_window_scale,
        debug_timing=debug_timing,
        verbose_route_diagnostics=verbose_route_diagnostics,
        collect_route_stats=collect_route_stats,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
        enable_internal_photonic_probe_verification=(enable_internal_photonic_probe_verification),
        include_heater_obstacles=include_heater_obstacles,
        ripup_reroute_config=ripup_reroute_config,
        enable_crossings=enable_crossings,
        node_depths=node_depths,
        node_ranks=node_ranks,
        edge_ranks=edge_ranks,
        crossing_loss=crossing_loss,
        crossing_mode=crossing_mode,
        crossing_half_size_cells=crossing_half_size_cells,
        min_straight_cells_per_crossing=min_straight_cells_per_crossing,
        foreign_port_keepout_cells=foreign_port_keepout_cells,
        fanout_access_mode=fanout_access_mode,
        allow_only_expected_crossings=allow_only_expected_crossings,
        crossing_guidance_net_names=crossing_guidance_net_names,
        defer_realization=True,
        enable_checked_endpoint_correction=enable_grid_endpoint_correction,
        config=config,
    )
    pipeline_timings_s: dict[str, float] = {
        "route_nets": time.perf_counter() - t_route_nets_start,
    }
    route_nets_timings = getattr(debug_artifacts, "route_nets_timings_s", {})
    if isinstance(route_nets_timings, dict):
        for name, elapsed_s in route_nets_timings.items():
            try:
                pipeline_timings_s[f"route_nets.{name}"] = float(elapsed_s)
            except (TypeError, ValueError):
                continue
    if debug_timing and verbose_route_diagnostics:
        print(
            "      - Optical net routing phase "
            f"(obstacle map + A* + repairs): {pipeline_timings_s['route_nets']:.4f} s"
        )

    if enable_grid_endpoint_correction:
        t_endpoint_correction_start = time.perf_counter()
        # Endpoint correction is part of the live routing phase above, where the
        # router still owns the committed dynamic obstacle map. Do not run a
        # second correction pass here: the realization-only router built for
        # debug artifacts has no committed nets and cannot safely validate bump
        # candidates against neighboring routes.
        debug_artifacts = replace(
            debug_artifacts,
            port_alignment_diagnostics=build_port_alignment_diagnostics(
                debug_artifacts.routed_net_records,
                realization_grid_spec=debug_artifacts.realization_grid_spec,
            ),
        )
        pipeline_timings_s["route_endpoint_correction"] = (
            time.perf_counter() - t_endpoint_correction_start
        )
    analysis_info = None
    requirements_info = None
    meander_report_info = None
    records_for_realization = debug_artifacts.routed_net_records
    if enable_path_length_matching:
        t_analysis_start = time.perf_counter()
        analysis, requirements = analyze_path_length_matching(
            schematic,
            routed_net_records=records_for_realization,
            node_types=node_types,
            internal_delays_um=internal_delays_um,
        )
        pipeline_timings_s["path_length_analysis"] = time.perf_counter() - t_analysis_start
        analysis_info = analysis_to_info_dict(analysis)
        raw_requirements = list(requirements)
        min_insertable_extra_um = minimum_four_bend_extra_length_um(
            grid_size_um=float(debug_artifacts.realization_grid_spec[2])
            if debug_artifacts.realization_grid_spec is not None
            else 0.0,
            bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
        )
        requirements, lifted_groups = compute_group_lifted_requirements(
            analysis,
            minimum_insertable_extra_um=min_insertable_extra_um,
        )
        lifted_requirements = list(requirements)
        output_requirements: list[Any] = []
        output_matching_info = {
            "enabled": bool(path_length_match_outputs),
            "target_output_arrival_um": 0.0,
            "output_count": 0,
            "requirements": [],
            "outputs": [],
        }
        if path_length_match_outputs:
            output_requirements, output_matching_info = compute_output_matching_requirements(
                analysis,
                existing_requirements=lifted_requirements,
            )
            requirements = merge_missing_length_requirements(
                lifted_requirements,
                output_requirements,
            )
        requirement_delay_candidates = build_requirement_delay_candidates(
            analysis,
            requirements,
        )
        analysis_info["raw_requirements"] = [requirement_to_dict(req) for req in raw_requirements]
        analysis_info["lifted_requirements"] = [
            requirement_to_dict(req) for req in lifted_requirements
        ]
        analysis_info["output_matching"] = output_matching_info
        analysis_info["output_requirements"] = [
            requirement_to_dict(req) for req in output_requirements
        ]
        analysis_info["requirements"] = [requirement_to_dict(req) for req in requirements]
        analysis_info["requirement_delay_candidates"] = [
            {
                "edge": requirement_to_dict(req)["edge"],
                "candidates": [
                    delay_candidate_to_dict(candidate)
                    for candidate in requirement_delay_candidates.get(req.edge_key, [])
                ],
            }
            for req in requirements
        ]
        analysis_info["matching_groups"] = lifted_groups
        analysis_info["minimum_insertable_extra_length_um"] = float(min_insertable_extra_um)
        requirements_info = [requirement_to_dict(req) for req in requirements]
        if debug_artifacts.realization_grid_spec is None:
            raise RuntimeError("Missing realization grid spec from routing phase.")
        t_meander_obstacle_start = time.perf_counter()
        resolved_user_obstacle_config = _resolve_obstacle_config(
            obstacle_config,
            route_layer=route_layer,
            include_heater_obstacles=include_heater_obstacles,
        )
        meander_route_clearance_um = max(
            0.0,
            _as_float(
                getattr(resolved_user_obstacle_config, "clearance_um", 0.0),
                0.0,
            ),
        )
        meander_clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
            route_width_um=float(route_width_um),
            grid_size_um=float(debug_artifacts.realization_grid_spec[2]),
            route_clearance_um=meander_route_clearance_um,
        )
        analysis_info["clearance_policy"] = meander_clearance_policy.to_debug_dict()
        # Meander box legality should use real routed-layer geometry, not
        # conservative component bboxes. Keep static obstacles strict: source
        # and target access openings are valid for route entry/exit only, not
        # for placing meander boxes.
        meander_obstacle_config = _with_obstacle_mode(
            resolved_user_obstacle_config,
            obstacle_mode="rasterized_polygons",
            clear_port_open_cells_from_static=False,
            populate_obstacle_map=False,
            materialize_cell_sets=False,
        )
        meander_obstacle_map = build_static_obstacle_map(
            unrouted_layout,
            config=meander_obstacle_config,
        )
        meander_static_blocked_cell_handle = getattr(
            meander_obstacle_map,
            "rust_blocked_cell_handle",
            None,
        )
        meander_static_blocked_cells = (
            None
            if meander_static_blocked_cell_handle is not None
            else meander_obstacle_map.blocked_cells
        )
        pipeline_timings_s["meander_obstacle_map"] = time.perf_counter() - t_meander_obstacle_start

        meander_config = MeanderInsertionConfig(
            enabled=True,
            max_meander_height_um=float(path_length_meander_height_um),
        )
        analysis_info["meander_config"] = {
            "enabled": bool(meander_config.enabled),
            "min_candidate_straight_length_um": float(
                meander_config.min_candidate_straight_length_um
            ),
            "max_extra_length_per_region_um": float(meander_config.max_extra_length_per_region_um),
            "conservative_legal_check": bool(meander_config.conservative_legal_check),
            "max_meander_height_um": float(meander_config.max_meander_height_um),
            "auto_meander_endpoint_inset_um": (
                None
                if meander_config.auto_meander_endpoint_inset_um is None
                else float(meander_config.auto_meander_endpoint_inset_um)
            ),
            "endpoint_inset_policy": (
                "adaptive" if meander_config.auto_meander_endpoint_inset_um is None else "fixed"
            ),
        }

        t_meander_planning_start = time.perf_counter()
        records_for_realization, meander_report_info = analyze_meander_insertion_for_requirements(
            records_for_realization,
            requirements,
            config=meander_config,
            realization_grid_spec=debug_artifacts.realization_grid_spec,
            allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
            bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
            static_blocked_cells=meander_static_blocked_cells,
            static_blocked_cell_handle=meander_static_blocked_cell_handle,
            route_occupancy_radius_cells=(
                meander_clearance_policy.plm_registered_route_keepout_radius_cells
            ),
            meander_box_clearance_radius_cells=(
                meander_clearance_policy.plm_candidate_box_clearance_radius_cells
            ),
            requirement_delay_candidates=requirement_delay_candidates,
        )
        pipeline_timings_s["meander_planning"] = time.perf_counter() - t_meander_planning_start
        if analysis_info is not None:
            matching_group_diagnostics = matching_group_diagnostics_to_info(
                analysis,
                meander_report_info,
                adjusted_requirements=requirements,
                lifted_groups=lifted_groups,
            )
            output_matching_diagnostics = output_matching_diagnostics_to_info(
                output_matching_info,
                meander_report_info,
            )
            acceptance_summary = path_length_acceptance_summary(
                matching_group_diagnostics + output_matching_diagnostics
            )
            analysis_info["matching_group_diagnostics"] = matching_group_diagnostics
            analysis_info["output_matching_diagnostics"] = output_matching_diagnostics
            analysis_info["path_length_acceptance"] = acceptance_summary
            if not acceptance_summary["passed"]:
                raise RuntimeError(format_path_length_acceptance_failure(acceptance_summary))

    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")

    crossing_plan_info = debug_artifacts.crossing_plan_info
    plan_info = (
        CrossingPlanInfo.from_dict(crossing_plan_info)
        if isinstance(crossing_plan_info, dict)
        else None
    )
    if plan_info is not None and enable_internal_photonic_probe_verification:
        final_records_by_net_id = _routed_records_by_net_id(records_for_realization)
        if final_records_by_net_id:
            illegal_realized_crossings = _verify_realized_route_intersections(
                crossing_plan_info=plan_info,
                routed_records_by_net_id=final_records_by_net_id,
                realization_grid_spec=debug_artifacts.realization_grid_spec,
            )
            _augment_insertion_loss_report_from_realized_intersections(
                crossing_plan_info=plan_info,
                routed_records_by_net_id=final_records_by_net_id,
            )
            if illegal_realized_crossings:
                _write_crossing_debug_artifacts(
                    debug_path=Path(debug_dir) if debug_dir is not None else Path("build"),
                    debug_prefix=debug_prefix,
                    crossing_plan_info=plan_info,
                )
                preview = "; ".join(
                    f"{item.get('net_name_a')} x {item.get('net_name_b')} "
                    f"at {item.get('point_um')} ({item.get('reason')}, "
                    f"margins={item.get('segment_a_margin_um')}/"
                    f"{item.get('segment_b_margin_um')}, "
                    f"required={item.get('required_margin_um')})"
                    for item in illegal_realized_crossings[:5]
                )
                raise RuntimeError(
                    "Illegal realized route crossing(s) after endpoint correction: "
                    f"{len(illegal_realized_crossings)} found. {preview}"
                )

    t_realization_start = time.perf_counter()
    realize_routed_net_records(
        routed_layout,
        records_for_realization,
        route_width_um=route_width_um,
        route_layer=route_layer,
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
        crossing_plan_info=plan_info.to_dict() if plan_info is not None else None,
        enable_endpoint_correction=enable_grid_endpoint_correction,
    )
    if plan_info is not None:
        _place_realized_crossing_components(routed_layout, plan_info)
        _write_crossing_debug_artifacts(
            debug_path=Path(debug_dir) if debug_dir is not None else None,
            debug_prefix=debug_prefix,
            crossing_plan_info=plan_info,
        )
        # The phases above wrote into `plan_info`, not into the mapping the
        # artifacts still carry: hand the updated plan back to them.
        debug_artifacts = replace(
            debug_artifacts,
            crossing_plan_info=plan_info.to_dict(),
        )
    t_realization_end = time.perf_counter()
    pipeline_timings_s["route_realization"] = t_realization_end - t_realization_start
    if debug_timing and verbose_route_diagnostics:
        print(
            "      - Optical route realization phase: "
            f"{pipeline_timings_s['route_realization']:.4f} s"
        )

    return RouteRustPipelineResult(
        routed_layout=routed_layout,
        debug_artifacts=debug_artifacts,
        path_length_analysis_info=analysis_info,
        meander_requirements_info=requirements_info,
        meander_insertion_report_info=meander_report_info,
        pipeline_timings_s=pipeline_timings_s,
    )


def static_fanout_anchors_um(
    unrouted_layout: Component,
    schematic: Schematic,
    **session_kwargs: Any,
) -> dict[str, tuple[float, float]]:
    """The router's static fan-out anchors for ``unrouted_layout`` WITHOUT routing.

    Runs the same setup a routing session performs before its first search
    (static obstacle map, grid, route jobs, dense-port clustering) and returns
    ``{port_spec: (x_um, y_um)}`` for every port that gets a static stub -- the
    point a route to/from that port actually starts from. Pre-placed crossing
    structures (contribution 2) put their entry tiles on these rows so the
    stub and the tile line up without a jog. ``session_kwargs`` must match the
    routing run's configuration (fanout_access_mode, bend radius, obstacle
    config), otherwise the anchors differ.
    """
    session = _RouteNetsRustSession(
        unrouted_layout=unrouted_layout, schematic=schematic, **session_kwargs
    )
    settings, state = session.settings, session.state
    context = build_static_obstacle_context(settings, state)
    configure_router_and_grid(settings, state, context.obstacle_map)
    build_route_jobs_and_fanout_clustering(settings, state, schematic.netlist.routes)
    return {
        spec: (float(anchor.center_um[0]), float(anchor.center_um[1]))
        for spec, anchor in state.fanout_anchor_by_port_spec.items()
    }
