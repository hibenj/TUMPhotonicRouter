"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import json
import math
import os
import re
import sys
import time
from collections import Counter, deque
from collections.abc import Iterable as IterableABC
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

import gdsfactory as gf
from gdsfactory.component import Component
from gdsfactory.schematic import Schematic
from gdsfactory.typings import Port

from translation.route_gds import get_port_from_instance
from photonic_router.routing_layers import (
    ComponentPortAccessRule,
    find_component_port_access_rule,
)
from photonic_router.static_obstacle_builder import grid_cell_center, physical_to_grid
from photonic_router.crossing_plan import CrossingPlan, build_crossing_plan
from photonic_router.topology_analysis import analyze_schematic_topology
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
from translation import route_rust_meanders as _meander_impl
from translation.route_rust_geometry import (
    _ILLEGAL_REALIZED_CROSSING_RE,
    _collinear_segment_overlap_with_params,
    _compress_centerline,
    _convex_polygons_overlap_with_area,
    _crossing_footprint_blockers,
    _crossing_footprint_half_extent_um,
    _crossing_footprint_polygon,
    _first_perpendicular_route_intersection,
    _format_illegal_crossing_root_causes_line,
    _format_native_repair_trace_lines,
    _format_route_indices,
    _grid_cell_neighborhood,
    _grid_point_to_physical_um,
    _illegal_crossing_root_causes_from_texts,
    _physical_point_to_grid_cell,
    _point_distance_um,
    _point_near_record_port_endpoint,
    _point_near_route_endpoint,
    _point_record_port_endpoint_distance_um,
    _point_route_endpoint_distance_um,
    _points_are_collinear,
    _polygon_axes,
    _project_polygon,
    _record_centerline_um,
    _rounded_point,
    _rounded_segment,
    _route_segments_from_waypoints,
    _route_waypoints_from_obj,
    _same_undirected_segment,
    _segment_bbox_um,
    _segment_intersection_with_params,
    _segment_intersects_crossing_footprint_interior,
    _segment_length_cells,
    _segment_length_um,
    _segment_unit_vector,
    _segments_are_perpendicular,
    _um_bboxes_overlap,
)
from translation.route_rust_crossing_verification import (
    _populate_realized_intersections_from_native_crossing_events,
    _verify_realized_route_intersections,
)
from translation.route_rust_crossing_components import (
    _active_crossing_component,
    _bbox_bounds_um,
    _bbox_center_um,
    _bbox_size_um,
    _crossing_component_bbox_size_um,
    _crossing_component_rotation_deg,
    _crossing_footprint_polygon_metadata,
    _legal_crossing_component_footprints_for_verification,
    _legal_crossing_overlap_polygons_for_verification,
    _place_realized_crossing_components,
    _point_um_from_mapping,
    _ports_optical_center_um,
    _segment_um_from_mapping,
    _shared_crossing_component_clusters,
    _shared_crossing_peer_indices,
)
from translation.route_rust_crossing_plan import (
    COLLISION_CROSSING_SEARCH_LOSS_ENV,
    DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM,
    _augment_crossing_plan_with_realized_overlaps,
    _augment_insertion_loss_report,
    _augment_insertion_loss_report_from_realized_intersections,
    _build_crossing_plan_info,
    _edge_key_to_info,
    _effective_crossing_search_loss,
    _port_center_um,
    _resolve_crossing_half_size_cells,
    _routed_records_by_net_id,
    _segment_bend_units,
    _write_crossing_debug_artifacts,
    _write_insertion_loss_report,
)
from translation.route_rust_endpoint_correction import (
    _apply_crossing_aware_endpoint_correction_to_record,
    _apply_crossing_aware_endpoint_corrections_to_debug_artifacts,
    _apply_endpoint_corrections_to_debug_artifacts,
    _build_realization_router,
    _centerline_between_cut_points,
    _centerline_index_near_point,
    _centerline_intersects_crossing_footprint_specs,
    _centerline_length_um,
    _closest_centerline_projection,
    _crossing_endpoint_splice_parts,
    _dedupe_centerline,
    _foreign_crossing_footprint_specs,
    _insert_centerline_cut_point,
    _legal_crossing_points_by_net_id,
    _merge_terminal_corrected_route_centerline,
    _primitive_centerline_for_record,
)
from translation.route_rust_debug_artifacts import (
    _as_point_list,
    _cells_bbox,
    _centerline_bbox_um,
    _dump_photonic_probe_failure_artifacts,
    _ensure_dir,
    _expanded_bbox_um,
    _format_cells_preview,
    _normalize_um_bbox,
    _rect_cell_count,
    _rect_overlap_cell_count,
    _route_cells_bbox,
    _write_centerline_probe_svg,
)
from translation.route_rust_obstacle_config import (
    _coerce_component_name,
    _default_obstacle_layers,
    _grid_origin_xy,
    _port_type_name,
    _resolve_obstacle_config,
    _schematic_instance_component_name,
    _with_bbox_cell_materialization,
    _with_obstacle_mode,
)
from translation.photonic_verification import (
    PhotonicVerificationIssue,
    PhotonicVerificationResult,
    verify_photonic_routing,
)
from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_records import (
    EndpointCorrectionRouter,
    _centerline_tuple,
    RouteBookkeeping,
    apply_port_endpoint_corrections,
    build_port_alignment_diagnostics,
    build_route_debug_artifacts,
    format_port_endpoint_correction_error,
    route_edge_key,
    routed_edge_lengths_from_records,
)
from translation.route_rust_types import (
    DEFAULT_MEANDER_MAX_HEIGHT_UM,
    EndpointCorrectionCategory,
    MeanderInsertionConfig,
    NetEndpointCorrectionClassification,
    OpticalRouteClearancePolicy,
    RipupRerouteConfig,
    RouteJob,
    RouteRustPipelineResult,
    RouteTimingBucket,
    RoutedNetRecord,
    RustRouteDebugArtifacts,
    _as_float,
    bend_radius_cells_from_um,
    route_attempt_record_from_route,
    summarize_route_search,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend

DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING = 2


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
    crossing_mode: str = "window",
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
    fanout_access_mode: str | None = None,
    allow_only_expected_crossings: bool = True,
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
        defer_realization=True,
        enable_checked_endpoint_correction=enable_grid_endpoint_correction,
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
    if isinstance(crossing_plan_info, dict) and enable_internal_photonic_probe_verification:
        final_records_by_net_id = _routed_records_by_net_id(records_for_realization)
        if final_records_by_net_id:
            illegal_realized_crossings = _verify_realized_route_intersections(
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=final_records_by_net_id,
                realization_grid_spec=debug_artifacts.realization_grid_spec,
            )
            _augment_insertion_loss_report_from_realized_intersections(
                crossing_plan_info=crossing_plan_info,
                routed_records_by_net_id=final_records_by_net_id,
            )
            if illegal_realized_crossings:
                _write_crossing_debug_artifacts(
                    debug_path=Path(debug_dir) if debug_dir is not None else Path("build"),
                    debug_prefix=debug_prefix,
                    crossing_plan_info=crossing_plan_info,
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
        crossing_plan_info=crossing_plan_info,
        enable_endpoint_correction=enable_grid_endpoint_correction,
    )
    if isinstance(crossing_plan_info, dict):
        _place_realized_crossing_components(routed_layout, crossing_plan_info)
        _write_crossing_debug_artifacts(
            debug_path=Path(debug_dir) if debug_dir is not None else None,
            debug_prefix=debug_prefix,
            crossing_plan_info=crossing_plan_info,
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


class _RouteNetsRustSession:
    def __init__(
        self,
        unrouted_layout: Component,
        schematic: Schematic,
        *,
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
        enable_crossings: bool = False,
        node_depths: dict[str, int] | None = None,
        node_ranks: dict[str, int] | None = None,
        edge_ranks: dict[str, dict[str, int]] | None = None,
        crossing_loss: float = 0.0,
        crossing_mode: str = "window",
        crossing_half_size_cells: int = 0,
        min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
        foreign_port_keepout_cells: int = 0,
        fanout_access_mode: str | None = None,
        allow_only_expected_crossings: bool = True,
        defer_realization: bool = False,
        enable_checked_endpoint_correction: bool = True,
    ):
        """Route schematic nets using Rust A* and add one polygon per routed net.

        This function routes each net by:
        1. Building a static obstacle map from the unrouted layout.
        2. Calling the Rust A* router for each net.
        3. Realizing one closed polygon in Rust and inserting it with add_polygon.
        4. Updating blocked cells for subsequent nets using width-aware inflation.

        Parameters:
            unrouted_layout: Component with placed instances but no routes.
            schematic: Schematic with net definitions.
            obstacle_config: Optional obstacle-map configuration.
            debug_dir: Directory where debug SVGs are written when provided.
            debug_prefix: Prefix used for debug SVG filenames.
            debug_route_indices: Optional 1-based net indices for per-route SVG
                export. When omitted, every route SVG is exported.
            debug_stop_after_route_index: Optional 1-based route index where routing
                stops after building full-netlist debug context. Port keepouts and
                crossing context are still derived from all schematic routes.
            route_width_um: Realized waveguide width in micrometers.
            route_layer: Target GDS layer/datatype tuple for route polygons.
            allow_45_degree_turns: If False, omit ±45-degree turn primitives.
            bend_radius_um: Minimum bend radius in micrometers. When omitted, the
                module default is used. The value is rounded up to the active grid
                cell size.
            use_indexed_heap: Benchmark-only queue experiment. Pass 8E measured
                this slower than duplicate-entry BinaryHeap queueing, so the
                production default remains False.
            enable_simple_routes: If False, skip straight/L/Z simple-route
                candidates and force routes through A* search.
            primitive_ordering: Benchmark-only dense A* primitive iteration order.
                Supported values: "library", "long_straight_first",
                "target_biased". Pass 8F keeps "library" as the default.
            heuristic_mode: Dense A* heuristic. Supported values: "distance",
                "heading_aware".
            max_iterations: Maximum A* state expansions per route attempt.
            verbose_route_diagnostics: If True, print per-net route progress and
                detailed A* timing buckets. Failures are always printed.
            enable_internal_photonic_probe_verification: If True, run the expensive
                internal realized-layout photonic probe before returning. The
                production default is False because `routing_flow.py` runs the
                authoritative Python geometry verification on the final layout.
            foreign_port_keepout_cells: Additional global keepout distance in front
                of each endpoint port. Active endpoint ports can open this region;
                dense multi-port instances can also open same-instance fanout
                keepouts, while unrelated nets cannot.
            fanout_access_mode: Dense multi-port access strategy. The default
                "legacy-runway" preserves the existing staggered source-port
                runway reservations. "off" disables those dense reservations.
                "static-stubs" pre-routes deterministic same-instance fanout
                stubs as static geometry and routes from virtual anchor ports.
            defer_realization: If True, keep routed RouteResult objects but skip
                polygon realization. This is used for pre-realization transforms
                such as path-length matching/meander insertion.
            enable_checked_endpoint_correction: If False, skip the checked
                grid-to-port correction pass used by PLM-oriented flows.

        Returns:
            A tuple of (routed_layout, debug_artifacts).
            :param debug_timing:
        """
        if route_width_um <= 0:
            raise ValueError("route_width_um must be > 0")
        if max_iterations <= 0:
            raise ValueError("max_iterations must be > 0")
        if not math.isfinite(float(proactive_congestion_weight)) or proactive_congestion_weight < 0:
            raise ValueError("proactive_congestion_weight must be finite and non-negative")
        if proactive_congestion_radius_cells < 0:
            raise ValueError("proactive_congestion_radius_cells must be non-negative")
        if crossing_loss < 0:
            raise ValueError("crossing_loss must be non-negative")
        crossing_mode = str(crossing_mode).strip().lower()
        if crossing_mode in {"pure", "lidar"}:
            crossing_mode = "lidar-pure"
        if crossing_mode not in {"window", "collision", "lidar-pure"}:
            raise ValueError("crossing_mode must be one of 'window', 'collision', or 'lidar-pure'")
        effective_allow_only_expected_crossings = bool(allow_only_expected_crossings)
        if crossing_mode == "lidar-pure":
            effective_allow_only_expected_crossings = False
        crossing_search_loss = _effective_crossing_search_loss(
            enable_crossings=bool(enable_crossings),
            crossing_mode=crossing_mode,
            crossing_loss=float(crossing_loss),
        )
        if crossing_half_size_cells < 0:
            raise ValueError("crossing_half_size_cells must be non-negative")
        if min_straight_cells_per_crossing < 0:
            raise ValueError("min_straight_cells_per_crossing must be non-negative")
        if foreign_port_keepout_cells < 0:
            raise ValueError("foreign_port_keepout_cells must be non-negative")
        raw_fanout_access_mode = os.environ.get(
            "PHOTONIC_ROUTER_FANOUT_ACCESS_MODE",
            "legacy-runway" if fanout_access_mode is None else str(fanout_access_mode),
        )
        fanout_access_mode_normalized = raw_fanout_access_mode.strip().lower().replace("_", "-")
        fanout_mode_aliases = {
            "": "legacy-runway",
            "legacy": "legacy-runway",
            "legacy-runway": "legacy-runway",
            "staggered": "legacy-runway",
            "staggered-runway": "legacy-runway",
            "runway": "legacy-runway",
            "0": "off",
            "false": "off",
            "none": "off",
            "disabled": "off",
            "disable": "off",
            "off": "off",
            "anchor": "static-stubs",
            "anchors": "static-stubs",
            "anchor-pre-spread": "static-stubs",
            "pre-spread": "static-stubs",
            "spread-stubs": "static-stubs",
            "static-stub": "static-stubs",
            "static-stubs": "static-stubs",
            "virtual-ports": "static-stubs",
        }
        fanout_access_mode_normalized = fanout_mode_aliases.get(
            fanout_access_mode_normalized,
            fanout_access_mode_normalized,
        )
        if fanout_access_mode_normalized not in {"legacy-runway", "off", "static-stubs"}:
            raise ValueError(
                "fanout_access_mode must be one of 'legacy-runway', 'off', or 'static-stubs'"
            )
        if debug_stop_after_route_index is not None and debug_stop_after_route_index < 1:
            raise ValueError("debug_stop_after_route_index must be >= 1")

        self.unrouted_layout = unrouted_layout
        self.schematic = schematic
        self.obstacle_config = obstacle_config
        self.debug_dir = debug_dir
        self.debug_prefix = debug_prefix
        self.debug_route_indices = debug_route_indices
        self.debug_stop_after_route_index = debug_stop_after_route_index
        self.route_width_um = route_width_um
        self.route_layer = route_layer
        self.allow_45_degree_turns = allow_45_degree_turns
        self.bend_radius_um = bend_radius_um
        self.enable_jps4 = enable_jps4
        self.use_indexed_heap = use_indexed_heap
        self.enable_simple_routes = enable_simple_routes
        self.primitive_ordering = primitive_ordering
        self.heuristic_mode = heuristic_mode
        self.heap_tie_breaker = heap_tie_breaker
        self.proactive_congestion_weight = proactive_congestion_weight
        self.proactive_congestion_radius_cells = proactive_congestion_radius_cells
        self.max_iterations = max_iterations
        self.routing_window_scale = routing_window_scale
        self.debug_timing = debug_timing
        self.verbose_route_diagnostics = verbose_route_diagnostics
        self.collect_route_stats = collect_route_stats
        self.collect_attempt_diagnostics = collect_attempt_diagnostics
        self.enable_internal_photonic_probe_verification = (
            enable_internal_photonic_probe_verification
        )
        self.include_heater_obstacles = include_heater_obstacles
        self.ripup_reroute_config = ripup_reroute_config
        self.enable_crossings = enable_crossings
        self.node_depths = node_depths
        self.node_ranks = node_ranks
        self.edge_ranks = edge_ranks
        self.crossing_loss = crossing_loss
        self.crossing_mode = crossing_mode
        self.crossing_half_size_cells = crossing_half_size_cells
        self.min_straight_cells_per_crossing = min_straight_cells_per_crossing
        self.foreign_port_keepout_cells = foreign_port_keepout_cells
        self.allow_only_expected_crossings = allow_only_expected_crossings
        self.defer_realization = defer_realization
        self.enable_checked_endpoint_correction = enable_checked_endpoint_correction
        self.effective_allow_only_expected_crossings = effective_allow_only_expected_crossings
        self.crossing_search_loss = crossing_search_loss
        self.fanout_access_mode_normalized = fanout_access_mode_normalized

        self.rust_backend = _load_rust_backend()
        if self.rust_backend is None:
            raise RuntimeError(
                "Rust router backend is not available. Build it with `cargo build` "
                "or `maturin develop` so photonic_router._rust can be imported."
            )

        self.routed_layout = self.unrouted_layout.copy()
        self.routed_layout.name = "routed_layout_rust"

        self.collect_pipeline_timing = (
            self.debug_timing or self.collect_route_stats or self.collect_attempt_diagnostics
        )
        self.route_nets_timings_s: dict[str, float] = {}

    def _pipeline_timer_start(self) -> float:
        return time.perf_counter() if self.collect_pipeline_timing else 0.0

    def _record_pipeline_timing(self, name: str, start_s: float) -> None:
        if self.collect_pipeline_timing:
            self.route_nets_timings_s[name] = self.route_nets_timings_s.get(name, 0.0) + (
                time.perf_counter() - start_s
            )

    def _orientation_to_angle(self, orientation: float | None, *, flip: bool = False) -> int:
        if orientation is None:
            angle = 0
        else:
            angle = int(round((float(orientation) % 360.0) / 45.0)) % 8

        if flip:
            angle = (angle + 4) % 8

        return angle

    def _angle_to_step(self, angle: int) -> tuple[int, int]:
        steps = [
            (1, 0),  # 0 east
            (1, 1),  # 1 northeast
            (0, 1),  # 2 north
            (-1, 1),  # 3 northwest
            (-1, 0),  # 4 west
            (-1, -1),  # 5 southwest
            (0, -1),  # 6 south
            (1, -1),  # 7 southeast
        ]
        return steps[angle % 8]

    def _direction_reaches_target_ray(
        self,
        *,
        source_x: int,
        source_y: int,
        source_angle: int,
        target_x: int,
        target_y: int,
        tolerance: int,
    ) -> bool:
        dx = target_x - source_x
        dy = target_y - source_y
        if abs(dx) <= tolerance and abs(dy) <= tolerance:
            return True
        dir_x, dir_y = self._angle_to_step(source_angle)
        if dir_x == 0 and dir_y == 0:
            return False
        if dir_x == 0:
            return abs(dx) <= tolerance and (dy > 0) == (dir_y > 0)
        if dir_y == 0:
            return abs(dy) <= tolerance and (dx > 0) == (dir_x > 0)
        return (
            (dx > 0) == (dir_x > 0)
            and (dy > 0) == (dir_y > 0)
            and abs(abs(dx) - abs(dy)) <= tolerance
        )

    def _source_lower_bounds(
        self,
        *,
        source_x: int,
        source_y: int,
        source_angle: int,
        target_x: int,
        target_y: int,
        target_angle: int,
    ) -> tuple[float, float]:
        grid_size_um = float(self.grid.grid_size_um)
        dx = target_x - source_x
        dy = target_y - source_y
        distance = math.hypot(float(dx), float(dy)) * grid_size_um
        heading_lower_bound = distance
        if str(self.heuristic_mode) == "heading_aware":
            target_angle_ok = not bool(getattr(self.astar_cfg, "require_target_angle", True)) or (
                source_angle % 8 == target_angle % 8
            )
            reaches_target_ray = self._direction_reaches_target_ray(
                source_x=source_x,
                source_y=source_y,
                source_angle=source_angle,
                target_x=target_x,
                target_y=target_y,
                tolerance=max(0, int(getattr(self.astar_cfg, "target_tolerance_cells", 0))),
            )
            if not target_angle_ok or not reaches_target_ray:
                minimum_bend_units = 1.0 if self.allow_45_degree_turns else 2.0
                bend_weight = float(getattr(self.astar_cfg, "bend_weight", 1.0)) * float(
                    getattr(self.primitive_cfg, "bend_weight", 1.0)
                )
                heading_lower_bound += minimum_bend_units * bend_weight
        return distance, heading_lower_bound

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < int(self.grid.width) and 0 <= gy < int(self.grid.height)

    def port_to_grid_state(
        self,
        port: Port,
        grid_origin_x_um: float,
        grid_origin_y_um: float,
        grid_size_um: float,
        *,
        as_target: bool = False,
        outward_cells: int = 1,
    ):
        port_angle = self._orientation_to_angle(port.orientation, flip=False)

        # For choosing the grid cell, always move outward from the physical port.
        # This avoids starting inside the real component/port geometry.
        sx, sy = self._angle_to_step(port_angle)

        x = float(port.center[0]) + sx * outward_cells * grid_size_um
        y = float(port.center[1]) + sy * outward_cells * grid_size_um

        gx = int((x - grid_origin_x_um) // grid_size_um)
        gy = int((y - grid_origin_y_um) // grid_size_um)

        # For the route state angle:
        # - source: route leaves the port outward
        # - target: route approaches the port, so flip direction
        route_angle = self._orientation_to_angle(port.orientation, flip=as_target)

        return self.rust_backend.State(gx, gy, route_angle)

    def _snap_nearly_collinear_states(
        self,
        source_state: Any,
        target_state: Any,
        source_port: Port,
        target_port: Port,
    ) -> tuple[Any, Any, set[tuple[int, int]]]:
        original_cells = {
            (int(source_state.x), int(source_state.y)),
            (int(target_state.x), int(target_state.y)),
        }
        source_angle = int(source_state.angle) % 8
        target_angle = int(target_state.angle) % 8
        if source_angle != target_angle:
            return source_state, target_state, original_cells

        source_center = getattr(source_port, "center", None)
        target_center = getattr(target_port, "center", None)
        if source_center is None or target_center is None:
            return source_state, target_state, original_cells

        source_x_um = float(source_center[0])
        source_y_um = float(source_center[1])
        target_x_um = float(target_center[0])
        target_y_um = float(target_center[1])
        grid_size = float(self.grid.grid_size_um)
        max_snap_um = max(grid_size, 2.0 * grid_size)
        max_snap_cells = max(1, math.ceil(max_snap_um / grid_size))

        if source_angle in {0, 4}:
            direction = 1 if source_angle == 0 else -1
            if (target_x_um - source_x_um) * direction <= 0.0:
                return source_state, target_state, original_cells
            if abs(target_y_um - source_y_um) > max_snap_um:
                return source_state, target_state, original_cells
            if abs(int(target_state.y) - int(source_state.y)) > max_snap_cells:
                return source_state, target_state, original_cells
            snapped_target = self.rust_backend.State(
                int(target_state.x),
                int(source_state.y),
                int(target_state.angle),
            )
            return source_state, snapped_target, original_cells

        if source_angle in {2, 6}:
            direction = 1 if source_angle == 2 else -1
            if (target_y_um - source_y_um) * direction <= 0.0:
                return source_state, target_state, original_cells
            if abs(target_x_um - source_x_um) > max_snap_um:
                return source_state, target_state, original_cells
            if abs(int(target_state.x) - int(source_state.x)) > max_snap_cells:
                return source_state, target_state, original_cells
            snapped_target = self.rust_backend.State(
                int(source_state.x),
                int(target_state.y),
                int(target_state.angle),
            )
            return source_state, snapped_target, original_cells

        return source_state, target_state, original_cells

    def _snap_same_heading_minimum_bend_offset(
        self,
        source_state: Any,
        target_state: Any,
    ) -> tuple[Any, Any, set[tuple[int, int]]]:
        """Snap one-cell-short S-bend offsets to the nearest realizable target.

        With cardinal same-heading ports, two opposing 90-degree bend primitives
        impose a minimum perpendicular displacement of 2R. Physical port centers
        often land half a grid cell off that value. Without this snap, exact-cell
        routing can only satisfy the one-cell deficit by introducing a loop.
        """
        extra_cells: set[tuple[int, int]] = set()
        if self.allow_45_degree_turns:
            return source_state, target_state, extra_cells

        source_angle = int(source_state.angle) % 8
        target_angle = int(target_state.angle) % 8
        if source_angle != target_angle:
            return source_state, target_state, extra_cells

        min_offset_cells = 2 * int(self.bend_radius_cells)
        if min_offset_cells <= 0:
            return source_state, target_state, extra_cells

        sx = int(source_state.x)
        sy = int(source_state.y)
        tx = int(target_state.x)
        ty = int(target_state.y)

        if source_angle in {0, 4}:
            forward_dx = tx - sx if source_angle == 0 else sx - tx
            dy = ty - sy
            if forward_dx < min_offset_cells or dy == 0:
                return source_state, target_state, extra_cells
            missing = min_offset_cells - abs(dy)
            if missing != 1:
                return source_state, target_state, extra_cells
            snapped_offset_cells = min_offset_cells + 1
            snapped_target = self.rust_backend.State(
                tx,
                sy + (snapped_offset_cells if dy > 0 else -snapped_offset_cells),
                target_angle,
            )
            if not self._in_bounds(int(snapped_target.x), int(snapped_target.y)):
                return source_state, target_state, extra_cells
            extra_cells.add((int(snapped_target.x), int(snapped_target.y)))
            return source_state, snapped_target, extra_cells

        if source_angle in {2, 6}:
            forward_dy = ty - sy if source_angle == 2 else sy - ty
            dx = tx - sx
            if forward_dy < min_offset_cells or dx == 0:
                return source_state, target_state, extra_cells
            missing = min_offset_cells - abs(dx)
            if missing != 1:
                return source_state, target_state, extra_cells
            snapped_offset_cells = min_offset_cells + 1
            snapped_target = self.rust_backend.State(
                sx + (snapped_offset_cells if dx > 0 else -snapped_offset_cells),
                ty,
                target_angle,
            )
            if not self._in_bounds(int(snapped_target.x), int(snapped_target.y)):
                return source_state, target_state, extra_cells
            extra_cells.add((int(snapped_target.x), int(snapped_target.y)))
            return source_state, snapped_target, extra_cells

        return source_state, target_state, extra_cells

    def _rect_ranges_by_y(
        self,
        rects: Iterable[tuple[int, int, int, int]],
    ) -> dict[int, list[tuple[int, int]]]:
        ranges_by_y: dict[int, list[tuple[int, int]]] = {}
        for rect_min_x, rect_min_y, rect_max_x, rect_max_y in rects:
            min_x = max(0, rect_min_x)
            max_x = min(self.grid_width - 1, rect_max_x)
            min_y = max(0, rect_min_y)
            max_y = min(self.grid_height - 1, rect_max_y)
            if min_x > max_x or min_y > max_y:
                continue
            for y in range(min_y, max_y + 1):
                ranges_by_y.setdefault(y, []).append((min_x, max_x))
        for y, ranges in list(ranges_by_y.items()):
            ranges.sort()
            merged_ranges: list[tuple[int, int]] = []
            for min_x, max_x in ranges:
                if not merged_ranges or min_x > merged_ranges[-1][1] + 1:
                    merged_ranges.append((min_x, max_x))
                else:
                    prev_min_x, prev_max_x = merged_ranges[-1]
                    merged_ranges[-1] = (prev_min_x, max(prev_max_x, max_x))
            ranges_by_y[y] = merged_ranges
        return ranges_by_y

    def _raw_static_rect_ranges_by_y(self) -> dict[int, list[tuple[int, int]]]:
        if self.raw_static_rect_ranges_by_y is not None:
            return self.raw_static_rect_ranges_by_y
        ranges_by_y = self._rect_ranges_by_y(self.raw_static_rects_for_openings)
        self.raw_static_rect_ranges_by_y = ranges_by_y
        return ranges_by_y

    def _heater_opening_rect_ranges_by_y(self) -> dict[int, list[tuple[int, int]]]:
        if self.heater_opening_rect_ranges_by_y is not None:
            return self.heater_opening_rect_ranges_by_y
        ranges_by_y = self._rect_ranges_by_y(self.heater_opening_rects_for_openings)
        self.heater_opening_rect_ranges_by_y = ranges_by_y
        return ranges_by_y

    def _raw_static_cells_by_y(self) -> dict[int, set[int]]:
        if self.raw_static_cells_by_y is not None:
            return self.raw_static_cells_by_y
        cells_by_y: dict[int, set[int]] = {}
        for cell_x, cell_y in self.raw_static_cells:
            cells_by_y.setdefault(int(cell_y), set()).add(int(cell_x))
        self.raw_static_cells_by_y = cells_by_y
        return cells_by_y

    def _cell_in_raw_static(self, cell: tuple[int, int]) -> bool:
        if cell in self.raw_static_cells:
            return True
        x, y = cell
        return any(
            rect_min_x <= x <= rect_max_x
            for rect_min_x, rect_max_x in self._raw_static_rect_ranges_by_y().get(y, ())
        )

    def _cells_in_raw_static_geometry(
        self,
        cells: set[tuple[int, int]],
    ) -> set[tuple[int, int]]:
        return {cell for cell in cells if self._cell_in_raw_static(cell)}

    def _resolve_port_footprint_cells(
        self,
        *,
        instance_name: str,
        port_name: str,
        port: object,
    ) -> tuple[int, int]:
        """Return (length_cells, half_width_cells) sizing this port's access/keepout region."""
        rule = self._port_access_rule_for(
            instance_name=instance_name,
            port_name=port_name,
            port=port,
        )
        if rule is not None:
            grid_size = float(self.grid.grid_size_um)
            length_cells = max(
                1,
                int(math.ceil(max(0.0, float(rule.access_length_um)) / grid_size)),
            )
            half_width_cells = max(
                0,
                int(math.ceil((max(0.0, float(rule.access_width_um)) / 2.0) / grid_size)),
            )
            return length_cells, half_width_cells

        if self._is_dense_source_fanout_instance(instance_name):
            return int(self.stub_port_lane_length_cells), int(self.stub_port_lane_half_width_cells)
        # A dense TARGET port with a real, pre-committed static stub
        # (`_build_static_fanout_target_anchors`) is in exactly the same
        # position as a dense source port with one: the stub's own committed
        # waveguide already protects the approach, so the generic port-lane
        # reservation below would only be redundant. Reuse the same
        # stub-scoped knobs (still real, still overridable via the same
        # `PHOTONIC_ROUTER_STUB_PORT_LANE_*` environment variables) rather
        # than introducing separate target-only ones -- see
        # `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
        if f"{instance_name},{port_name}" in getattr(self, "fanout_anchor_by_port_spec", {}):
            return int(self.stub_port_lane_length_cells), int(self.stub_port_lane_half_width_cells)
        return int(self.port_lane_length_cells), int(self.port_lane_half_width_cells)

    def _port_access_rule_for(
        self,
        *,
        instance_name: str,
        port_name: str,
        port: Port,
    ) -> ComponentPortAccessRule | None:
        """The component access rule governing this port, if any.

        Returns the rule itself (not a projection of it) so every consumer --
        runway sizing, dense-group exclusion, the instance-geometry opening --
        reads the same declaration.
        """
        return find_component_port_access_rule(
            component_name=_schematic_instance_component_name(self.schematic, instance_name),
            port_name=port_name,
            port_type=_port_type_name(port),
        )

    def _instance_ref_by_name(self, instance_name: str) -> Any | None:
        try:
            instances = self.routed_layout.insts
        except (AttributeError, TypeError):
            return None
        for instance in instances:
            if getattr(instance, "name", None) == instance_name:
                return instance
        return None

    def _instance_static_geometry_open_cells(
        self,
        *,
        instance_name: str,
        center_um: tuple[float, float],
        orientation: float | None,
    ) -> set[tuple[int, int]]:
        """Static cells of the port's own instance to open on the port-facing side.

        Only called for ports whose access rule declares
        ``opens_instance_static_geometry`` (today: heater optical ports, whose
        metal pad sits on the port-facing side and would otherwise block the
        access runway). The margin covers the clearance-expanded blocked
        rectangles as well, so the opening matches what the obstacle map
        actually blocks. Never call this on the strength of a rule merely
        existing: the pre-placed crossing grids register a zero-size runway
        rule, and opening their interior let a net hook through the grid
        (``benes_16x16`` grid mode, route 74, 2026-08-27).
        """
        if orientation is None:
            return set()
        ref = self._instance_ref_by_name(instance_name)
        if ref is None:
            return set()
        bounds = _bbox_bounds_um(ref)
        if bounds is None:
            return set()
        left, bottom, right, top = bounds
        grid_size = float(self.grid.grid_size_um)
        if grid_size <= 0.0:
            return set()

        angle_rad = math.radians(float(orientation))
        dir_x = math.cos(angle_rad)
        dir_y = math.sin(angle_rad)
        if not math.isfinite(dir_x) or not math.isfinite(dir_y):
            return set()

        bbox_margin = 0.5 * grid_size
        min_bbox_cell = physical_to_grid(
            float(left) - bbox_margin,
            float(bottom) - bbox_margin,
            self.grid,
        )
        max_bbox_cell = physical_to_grid(
            float(right) + bbox_margin,
            float(top) + bbox_margin,
            self.grid,
        )
        min_x = max(
            0,
            min_bbox_cell[0],
        )
        max_x = min(
            self.grid_width - 1,
            max_bbox_cell[0],
        )
        min_y = max(
            0,
            min_bbox_cell[1],
        )
        max_y = min(
            self.grid_height - 1,
            max_bbox_cell[1],
        )
        if min_x > max_x or min_y > max_y:
            return set()

        heater_clearance_um = getattr(self.resolved_obstacle_config, "heater_clearance_um", None)
        opening_margin_um = max(
            float(self.route_clearance_um),
            0.0 if heater_clearance_um is None else float(heater_clearance_um),
        )
        opening_margin_cells = int(math.ceil(opening_margin_um / grid_size)) + 1
        search_min_x = max(0, min_x - opening_margin_cells)
        search_max_x = min(self.grid_width - 1, max_x + opening_margin_cells)
        search_min_y = max(0, min_y - opening_margin_cells)
        search_max_y = min(self.grid_height - 1, max_y + opening_margin_cells)
        opening_margin_distance = float(opening_margin_cells) * grid_size + bbox_margin
        opening_left = float(left) - opening_margin_distance
        opening_right = float(right) + opening_margin_distance
        opening_bottom = float(bottom) - opening_margin_distance
        opening_top = float(top) + opening_margin_distance

        candidate_cells: set[tuple[int, int]] = set()
        explicit_cells_by_y = self._raw_static_cells_by_y()
        heater_rect_ranges_by_y = self._heater_opening_rect_ranges_by_y()
        for cell_y in range(search_min_y, search_max_y + 1):
            xs: set[int] = set()
            xs.update(
                cell_x
                for cell_x in explicit_cells_by_y.get(cell_y, set())
                if search_min_x <= int(cell_x) <= search_max_x
            )
            for rect_min_x, rect_max_x in heater_rect_ranges_by_y.get(cell_y, ()):
                start_x = max(search_min_x, int(rect_min_x))
                end_x = min(search_max_x, int(rect_max_x))
                if start_x <= end_x:
                    xs.update(range(start_x, end_x + 1))
            for cell_x in xs:
                inside_instance_bbox = (
                    min_x <= int(cell_x) <= max_x and min_y <= int(cell_y) <= max_y
                )
                cell_center = self._grid_cell_center_um(int(cell_x), int(cell_y))
                if (
                    opening_left <= cell_center[0] <= opening_right
                    and opening_bottom <= cell_center[1] <= opening_top
                ):
                    candidate_cells.add((int(cell_x), int(cell_y)))
                    continue
                if inside_instance_bbox:
                    if cell_center[0] < left - bbox_margin or cell_center[0] > right + bbox_margin:
                        continue
                    if cell_center[1] < bottom - bbox_margin or cell_center[1] > top + bbox_margin:
                        continue
                outward_distance = (cell_center[0] - float(center_um[0])) * dir_x + (
                    cell_center[1] - float(center_um[1])
                ) * dir_y
                if outward_distance >= -bbox_margin:
                    candidate_cells.add((int(cell_x), int(cell_y)))

        return candidate_cells

    def _dense_fanout_min_ports(self) -> int:
        """Smallest same-instance, same-angle port group treated as a dense fanout.

        Default 3 (the historical `> 2`). `PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS`
        overrides it; the pre-placed crossing-grid flow uses 2 so a 2x2
        switch's port pair (1.25 um apart, inside one routing cell) gets the
        same staggered static stubs a multiport MMI's port row gets.
        """
        raw = os.environ.get("PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS", "").strip()
        if not raw:
            return 3
        value = int(raw)
        if value < 2:
            raise ValueError("PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS must be >= 2")
        return value

    def _dense_fanout_group_size(self, port_specs: set[str]) -> int:
        """Ports of a group that qualify for automatic dense-fanout handling.

        A port with an explicit component access rule (see
        `_port_access_rule_for`) has had its access geometry decided
        deliberately and is left out, so components that manage their own
        approach (e.g. pre-placed crossing grids) never get automatic stubs
        or runways. Looked up directly rather than through
        `port_access_rule_by_spec`, which is only populated after the anchor
        passes that call this.
        """
        count = 0
        for spec in port_specs:
            endpoint = self.endpoint_ports_by_spec.get(spec)
            if endpoint is None:
                count += 1
                continue
            instance_name, port_name, port = endpoint
            rule = self._port_access_rule_for(
                instance_name=instance_name, port_name=port_name, port=port
            )
            if rule is None:
                count += 1
        return count

    def _is_dense_source_fanout_instance(self, instance_name: str) -> bool:
        return any(
            self._dense_fanout_group_size(port_specs) >= self._dense_fanout_min_ports()
            for (
                group_instance,
                _angle,
            ), port_specs in self.source_port_specs_by_instance_angle.items()
            if group_instance == instance_name
        )

    def _is_dense_source_fanout_group(self, instance_name: str, angle: int) -> bool:
        return (
            self._dense_fanout_group_size(
                self.source_port_specs_by_instance_angle.get((instance_name, int(angle)), set())
            )
            >= self._dense_fanout_min_ports()
        )

    def _is_dense_target_fanout_instance(self, instance_name: str) -> bool:
        return any(
            self._dense_fanout_group_size(port_specs) >= self._dense_fanout_min_ports()
            for (
                group_instance,
                _angle,
            ), port_specs in self.target_port_specs_by_instance_angle.items()
            if group_instance == instance_name
        )

    def _is_dense_target_fanout_group(self, instance_name: str, angle: int) -> bool:
        return (
            self._dense_fanout_group_size(
                self.target_port_specs_by_instance_angle.get((instance_name, int(angle)), set())
            )
            >= self._dense_fanout_min_ports()
        )

    @dataclass(frozen=True)
    class _FanoutAnchor:
        port_spec: str
        state_x: int
        state_y: int
        physical_angle: int
        center_um: tuple[float, float]
        stub_center_cells: tuple[tuple[int, int], ...]
        stub_centerline_um: tuple[tuple[float, float], ...]

    def _grid_cell_center_um(self, cell_x: int, cell_y: int) -> tuple[float, float]:
        return grid_cell_center(cell_x, cell_y, self.grid)

    def _centerline_grid_cells(
        self,
        centerline_um: Iterable[tuple[float, float]],
    ) -> tuple[tuple[int, int], ...]:
        points = [
            (float(point[0]), float(point[1]))
            for point in centerline_um
            if math.isfinite(float(point[0])) and math.isfinite(float(point[1]))
        ]
        if not points:
            return ()
        cells: list[tuple[int, int]] = []

        def append_point(point: tuple[float, float]) -> None:
            cell = _physical_point_to_grid_cell(
                point,
                grid_size_um=float(self.grid.grid_size_um),
                origin_x_um=float(self.origin_x_um),
                origin_y_um=float(self.origin_y_um),
            )
            if cell is None:
                return
            if not self._in_bounds(cell[0], cell[1]):
                return
            if cells and cells[-1] == cell:
                return
            cells.append(cell)

        append_point(points[0])
        sample_step_um = max(float(self.grid.grid_size_um) / 4.0, 1.0e-6)
        for start, end in zip(points, points[1:]):
            dx = float(end[0]) - float(start[0])
            dy = float(end[1]) - float(start[1])
            length = math.hypot(dx, dy)
            if length <= 1.0e-9:
                append_point(end)
                continue
            steps = max(1, int(math.ceil(length / sample_step_um)))
            for index in range(1, steps + 1):
                t = float(index) / float(steps)
                append_point((start[0] + dx * t, start[1] + dy * t))
        return tuple(dict.fromkeys(cells))

    def _env_nonnegative_int(self, name: str, default: int) -> int:
        raw_value = os.environ.get(name)
        if raw_value is None or raw_value.strip() == "":
            return int(default)
        try:
            value = int(raw_value)
        except ValueError as exc:
            raise ValueError(f"{name} must be a non-negative integer") from exc
        if value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        return value

    def _env_fanout_stub_bend_steps(self) -> int:
        raw_value = os.environ.get("PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES", "90")
        normalized = raw_value.strip().lower().replace("_", "-")
        aliases = {
            "45": 1,
            "45deg": 1,
            "45-degree": 1,
            "45-deg": 1,
            "diagonal": 1,
            "diag": 1,
            "90": 2,
            "90deg": 2,
            "90-degree": 2,
            "90-deg": 2,
            "orthogonal": 2,
            "orthogonal-u": 2,
        }
        if normalized not in aliases:
            raise ValueError("PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES must be 45 or 90")
        return aliases[normalized]

    def _append_grid_step(
        self,
        path: list[tuple[int, int]],
        step_x: int,
        step_y: int,
        count: int,
    ) -> None:
        if count <= 0:
            return
        cell_x, cell_y = path[-1]
        for _ in range(count):
            cell_x += step_x
            cell_y += step_y
            if self._in_bounds(cell_x, cell_y):
                path.append((cell_x, cell_y))

    def _inflated_cells(
        self,
        cells: Iterable[tuple[int, int]],
        radius: int,
    ) -> set[tuple[int, int]]:
        radius = max(0, int(radius))
        inflated: set[tuple[int, int]] = set()
        for cell_x, cell_y in cells:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx = int(cell_x) + dx
                    ny = int(cell_y) + dy
                    if self._in_bounds(nx, ny):
                        inflated.add((nx, ny))
        return inflated

    def _angle_to_unit_vector(self, angle: int) -> tuple[float, float]:
        radians = (int(angle) % 8) * (math.pi / 4.0)
        return (math.cos(radians), math.sin(radians))

    def _rotate_left_vector(self, vector: tuple[float, float]) -> tuple[float, float]:
        return (-vector[1], vector[0])

    def _rotate_right_vector(self, vector: tuple[float, float]) -> tuple[float, float]:
        return (vector[1], -vector[0])

    def _cross2(
        self,
        a: tuple[float, float],
        b: tuple[float, float],
    ) -> float:
        return float(a[0]) * float(b[1]) - float(a[1]) * float(b[0])

    def _append_stub_point(
        self,
        out: list[tuple[float, float]],
        point: tuple[float, float],
    ) -> None:
        point = (float(point[0]), float(point[1]))
        if out:
            last_x, last_y = out[-1]
            if math.hypot(point[0] - last_x, point[1] - last_y) <= 1.0e-9:
                return
        out.append(point)

    def _append_circular_stub_bend(
        self,
        out: list[tuple[float, float]],
        *,
        start_point: tuple[float, float],
        start_angle: int,
        end_point: tuple[float, float],
        end_angle: int,
        angle_delta: int,
    ) -> None:
        radius_um = float(self.bend_radius_cells) * float(self.grid.grid_size_um)
        if radius_um <= 0.0 or not math.isfinite(radius_um):
            self._append_stub_point(out, end_point)
            return
        start_dir = self._angle_to_unit_vector(start_angle)
        end_dir = self._angle_to_unit_vector(end_angle)
        chord = (
            float(end_point[0]) - float(start_point[0]),
            float(end_point[1]) - float(start_point[1]),
        )
        denom = self._cross2(start_dir, end_dir)
        if abs(denom) <= 1.0e-9:
            self._append_stub_point(out, end_point)
            return
        in_len = self._cross2(chord, end_dir) / denom
        out_len = self._cross2(start_dir, chord) / denom
        if (
            not math.isfinite(in_len)
            or not math.isfinite(out_len)
            or in_len <= 1.0e-9
            or out_len <= 1.0e-9
        ):
            self._append_stub_point(out, end_point)
            return
        corner = (
            float(start_point[0]) + start_dir[0] * in_len,
            float(start_point[1]) + start_dir[1] * in_len,
        )
        turn_abs = abs(int(angle_delta)) * (math.pi / 4.0)
        trim = radius_um * math.tan(turn_abs / 2.0)
        trim_eff = min(trim, in_len, out_len)
        if not math.isfinite(trim_eff) or trim_eff <= 1.0e-9:
            self._append_stub_point(out, end_point)
            return
        t_in = (
            corner[0] - start_dir[0] * trim_eff,
            corner[1] - start_dir[1] * trim_eff,
        )
        t_out = (
            corner[0] + end_dir[0] * trim_eff,
            corner[1] + end_dir[1] * trim_eff,
        )
        self._append_stub_point(out, t_in)
        left_turn = int(angle_delta) > 0
        n_start = (
            self._rotate_left_vector(start_dir)
            if left_turn
            else self._rotate_right_vector(start_dir)
        )
        n_end = (
            self._rotate_left_vector(end_dir) if left_turn else self._rotate_right_vector(end_dir)
        )
        c0 = (
            t_in[0] + n_start[0] * radius_um,
            t_in[1] + n_start[1] * radius_um,
        )
        c1 = (
            t_out[0] + n_end[0] * radius_um,
            t_out[1] + n_end[1] * radius_um,
        )
        center = ((c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5)
        a0 = math.atan2(t_in[1] - center[1], t_in[0] - center[0])
        a1 = math.atan2(t_out[1] - center[1], t_out[0] - center[0])
        if left_turn:
            while a1 <= a0:
                a1 += math.tau
        else:
            while a1 >= a0:
                a1 -= math.tau
        arc_span = abs(a1 - a0)
        steps = max(2, int(math.ceil((arc_span / (math.pi / 2.0)) * 16.0)))
        for index in range(1, steps):
            t = float(index) / float(steps)
            angle = a0 + (a1 - a0) * t
            self._append_stub_point(
                out,
                (
                    center[0] + radius_um * math.cos(angle),
                    center[1] + radius_um * math.sin(angle),
                ),
            )
        self._append_stub_point(out, t_out)
        self._append_stub_point(out, end_point)

    def _append_arc_from_tangencies(
        self,
        out: list[tuple[float, float]],
        *,
        t_in: tuple[float, float],
        t_out: tuple[float, float],
        start_angle: int,
        end_angle: int,
        angle_delta: int,
    ) -> None:
        radius_um = float(self.bend_radius_cells) * float(self.grid.grid_size_um)
        if radius_um <= 0.0 or not math.isfinite(radius_um):
            self._append_stub_point(out, t_out)
            return
        start_dir = self._angle_to_unit_vector(start_angle)
        end_dir = self._angle_to_unit_vector(end_angle)
        self._append_stub_point(out, t_in)
        left_turn = int(angle_delta) > 0
        n_start = (
            self._rotate_left_vector(start_dir)
            if left_turn
            else self._rotate_right_vector(start_dir)
        )
        n_end = (
            self._rotate_left_vector(end_dir) if left_turn else self._rotate_right_vector(end_dir)
        )
        c0 = (
            float(t_in[0]) + n_start[0] * radius_um,
            float(t_in[1]) + n_start[1] * radius_um,
        )
        c1 = (
            float(t_out[0]) + n_end[0] * radius_um,
            float(t_out[1]) + n_end[1] * radius_um,
        )
        center = ((c0[0] + c1[0]) * 0.5, (c0[1] + c1[1]) * 0.5)
        a0 = math.atan2(float(t_in[1]) - center[1], float(t_in[0]) - center[0])
        a1 = math.atan2(float(t_out[1]) - center[1], float(t_out[0]) - center[0])
        if left_turn:
            while a1 <= a0:
                a1 += math.tau
        else:
            while a1 >= a0:
                a1 -= math.tau
        arc_span = abs(a1 - a0)
        steps = max(2, int(math.ceil((arc_span / (math.pi / 2.0)) * 16.0)))
        for index in range(1, steps):
            t = float(index) / float(steps)
            angle = a0 + (a1 - a0) * t
            self._append_stub_point(
                out,
                (
                    center[0] + radius_um * math.cos(angle),
                    center[1] + radius_um * math.sin(angle),
                ),
            )
        self._append_stub_point(out, t_out)

    def _append_realized_stub_bend(
        self,
        out: list[tuple[float, float]],
        start_point_um: tuple[float, float],
        start_angle: int,
        angle_delta: int,
    ) -> tuple[float, float]:
        arm_um = float(self.bend_radius_cells) * float(self.grid.grid_size_um)
        radius_um = arm_um
        turn_abs = abs(int(angle_delta)) * (math.pi / 4.0)
        trim = radius_um * math.tan(turn_abs / 2.0)
        end_angle = (int(start_angle) + int(angle_delta)) % 8
        start_dir = self._angle_to_unit_vector(int(start_angle) % 8)
        end_dir = self._angle_to_unit_vector(end_angle)
        start_step = self._angle_to_step(int(start_angle) % 8)
        end_step = self._angle_to_step(end_angle)
        start_point = (float(start_point_um[0]), float(start_point_um[1]))
        corner = (
            start_point[0] + float(start_step[0]) * arm_um,
            start_point[1] + float(start_step[1]) * arm_um,
        )
        end_point = (
            corner[0] + float(end_step[0]) * arm_um,
            corner[1] + float(end_step[1]) * arm_um,
        )
        t_in = (
            corner[0] - start_dir[0] * trim,
            corner[1] - start_dir[1] * trim,
        )
        t_out = (
            corner[0] + end_dir[0] * trim,
            corner[1] + end_dir[1] * trim,
        )
        self._append_stub_point(out, t_in)
        self._append_arc_from_tangencies(
            out,
            t_in=t_in,
            t_out=t_out,
            start_angle=int(start_angle) % 8,
            end_angle=end_angle,
            angle_delta=int(angle_delta),
        )
        self._append_stub_point(out, end_point)
        return end_point

    def _two_bend_static_stub_centerline_um(
        self,
        port_center_um: tuple[float, float],
        physical_angle: int,
        lateral_sign: int,
        target_anchor_y_cell: int | None = None,
        min_forward_cells: int = 0,
        initial_forward_cells: int = 0,
        extra_final_forward_cells: int = 0,
    ) -> tuple[tuple[tuple[float, float], ...], tuple[int, int]] | None:
        start_angle = int(physical_angle) % 8
        bend_delta = int(lateral_sign) * self._env_fanout_stub_bend_steps()
        intermediate_angle = (start_angle + bend_delta) % 8
        intermediate_step = self._angle_to_step(intermediate_angle)
        final_step = self._angle_to_step(start_angle)
        trace_fanout_stubs = os.environ.get("PHOTONIC_ROUTER_TRACE_FANOUT_STUBS", "").strip()

        def fail(reason: str, extra: str = "") -> None:
            if trace_fanout_stubs:
                print(
                    "fanout_stub_failed "
                    f"reason={reason} "
                    f"port={port_center_um} "
                    f"angle={start_angle} lateral_sign={lateral_sign} "
                    f"target_anchor_y_cell={target_anchor_y_cell} "
                    f"min_forward_cells={min_forward_cells} "
                    f"initial_forward_cells={initial_forward_cells} "
                    f"extra_final_forward_cells={extra_final_forward_cells}"
                    f"{extra}",
                    file=sys.stderr,
                )

        if abs(final_step[0]) + abs(final_step[1]) != 1:
            fail("non_cardinal_final")
            return None
        if intermediate_step[1] == 0:
            fail("intermediate_has_no_y")
            return None
        port_point = (float(port_center_um[0]), float(port_center_um[1]))

        def _next_grid_axis_value(
            value: float,
            axis: str,
            direction: int,
        ) -> float | None:
            if direction == 0:
                return None
            origin = self.origin_x_um if axis == "x" else self.origin_y_um
            rel = (float(value) - float(origin)) / float(self.grid.grid_size_um) - 0.5
            eps = 1.0e-9
            if direction > 0:
                index = math.ceil(rel - eps)
            else:
                index = math.floor(rel + eps)
            center = grid_cell_center(
                int(index) if axis == "x" else 0,
                int(index) if axis == "y" else 0,
                self.grid,
            )
            return center[0] if axis == "x" else center[1]

        points: list[tuple[float, float]] = [port_point]
        bend_start = port_point
        initial_forward_um = float(max(0, int(initial_forward_cells))) * float(
            self.grid.grid_size_um
        )
        if initial_forward_um > 1.0e-9:
            bend_start = (
                port_point[0] + float(final_step[0]) * initial_forward_um,
                port_point[1] + float(final_step[1]) * initial_forward_um,
            )
            self._append_stub_point(points, bend_start)
        first_end = self._append_realized_stub_bend(
            points,
            bend_start,
            start_angle,
            bend_delta,
        )
        if target_anchor_y_cell is None:
            target_intermediate_y = _next_grid_axis_value(
                first_end[1],
                "y",
                int(intermediate_step[1]),
            )
        else:
            target_intermediate_y = self._grid_cell_center_um(
                0,
                int(target_anchor_y_cell) - int(intermediate_step[1]) * int(self.bend_radius_cells),
            )[1]
        if target_intermediate_y is None:
            fail("no_target_intermediate_y")
            return None
        intermediate_delta_y = float(target_intermediate_y) - float(first_end[1])
        if intermediate_delta_y * float(intermediate_step[1]) < -1.0e-9:
            fail("intermediate_moves_backward")
            return None
        intermediate_delta_x = intermediate_delta_y * (
            float(intermediate_step[0]) / float(intermediate_step[1])
        )
        intermediate_end = (
            first_end[0] + intermediate_delta_x,
            float(target_intermediate_y),
        )
        self._append_stub_point(points, intermediate_end)
        second_end = self._append_realized_stub_bend(
            points,
            intermediate_end,
            intermediate_angle,
            angle_delta=-bend_delta,
        )
        if final_step[0] != 0:
            target_final_x = _next_grid_axis_value(
                second_end[0],
                "x",
                int(final_step[0]),
            )
            if target_final_x is None:
                fail("no_target_final_x")
                return None
            min_forward_x = port_point[0] + float(final_step[0]) * float(
                max(0, int(min_forward_cells))
                + max(0, int(initial_forward_cells))
                + max(0, int(extra_final_forward_cells))
            ) * float(self.grid.grid_size_um)
            if int(final_step[0]) > 0:
                if float(target_final_x) < float(min_forward_x):
                    snapped_min_forward_x = _next_grid_axis_value(
                        float(min_forward_x),
                        "x",
                        int(final_step[0]),
                    )
                    if snapped_min_forward_x is None:
                        fail("no_snapped_min_forward_x")
                        return None
                    target_final_x = float(snapped_min_forward_x)
            else:
                if float(target_final_x) > float(min_forward_x):
                    snapped_min_forward_x = _next_grid_axis_value(
                        float(min_forward_x),
                        "x",
                        int(final_step[0]),
                    )
                    if snapped_min_forward_x is None:
                        fail("no_snapped_min_forward_x")
                        return None
                    target_final_x = float(snapped_min_forward_x)
            final_delta_x = float(target_final_x) - float(second_end[0])
            if final_delta_x * float(final_step[0]) < -1.0e-9:
                fail("final_moves_backward_x")
                return None
            anchor_point = (float(target_final_x), float(second_end[1]))
        else:
            target_final_y = _next_grid_axis_value(
                second_end[1],
                "y",
                int(final_step[1]),
            )
            if target_final_y is None:
                fail("no_target_final_y")
                return None
            final_delta_y = float(target_final_y) - float(second_end[1])
            if final_delta_y * float(final_step[1]) < -1.0e-9:
                fail("final_moves_backward_y")
                return None
            anchor_point = (float(second_end[0]), float(target_final_y))
        self._append_stub_point(points, anchor_point)
        anchor_x = int(
            round((anchor_point[0] - self.origin_x_um) / float(self.grid.grid_size_um) - 0.5)
        )
        anchor_y = int(
            round((anchor_point[1] - self.origin_y_um) / float(self.grid.grid_size_um) - 0.5)
        )
        snapped_anchor = self._grid_cell_center_um(anchor_x, anchor_y)
        snap_error_um = math.hypot(
            float(snapped_anchor[0]) - float(anchor_point[0]),
            float(snapped_anchor[1]) - float(anchor_point[1]),
        )
        if snap_error_um > max(1.0e-6, 0.05 * float(self.grid.grid_size_um)):
            fail(
                f"snap_error:{snap_error_um:.6g}",
                " "
                f"anchor_point=({anchor_point[0]:.6g},{anchor_point[1]:.6g}) "
                f"anchor_cell=({anchor_x},{anchor_y}) "
                f"snapped=({snapped_anchor[0]:.6g},{snapped_anchor[1]:.6g}) "
                f"origin=({self.origin_x_um:.6g},{self.origin_y_um:.6g}) "
                f"grid={float(self.grid.grid_size_um):.6g} "
                f"bend_radius_cells={self.bend_radius_cells}",
            )
            return None
        if not self._in_bounds(anchor_x, anchor_y):
            fail("anchor_out_of_bounds")
            return None
        return _compress_centerline(tuple(points)), (anchor_x, anchor_y)

    def _straight_static_stub_centerline_um(
        self,
        port_center_um: tuple[float, float],
        physical_angle: int,
        forward_cells: int,
    ) -> tuple[tuple[tuple[float, float], ...], tuple[int, int]] | None:
        """Build a straight stub extending `forward_cells` grid cells forward
        from a port along its own physical orientation, absorbing any
        sub-cell port-to-grid misalignment with a small real-bend-radius
        curve rather than a lateral offset.

        Used for dense TARGET port stubs. Unlike source stubs (which need
        to laterally redistribute a tight component pitch out to a wider
        lane spacing via `_two_bend_static_stub_centerline_um`, since many
        source nets fan out from adjacent ports toward widely separated
        destinations), a target port just needs a short, protected,
        straight approach directly in front of itself -- any lateral
        movement a route still needs happens in the main A* search after
        it reaches this anchor, not baked into the stub geometry.

        Two things were tried and rejected before this one, both recorded
        in `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`
        Surprises & Discoveries: (1) reusing the two-bend, laterally
        offsetting source-stub geometry for target ports too caused a real
        regression (nearby target stubs ended up on different lateral
        lanes, producing near-duplicate diagonal candidate paths for
        adjacent nets that then illegally collided); (2) keeping the
        anchor's geometric Y exactly at the port's own Y (a perfectly
        straight, unbent line) and deferring the resulting sub-cell offset
        to the ordinary checked-endpoint-correction pass did not work --
        that pass left the route's raw grid endpoint essentially untouched
        rather than reconciling it, for reasons not yet root-caused. This
        version absorbs the offset locally instead, using
        `_fanout_stub_centerline_um`, which is a plain straight line when
        the port and the grid-snapped anchor are already aligned, and a
        small circular arc (built by `_append_circular_stub_bend`, using
        `self.bend_radius_cells` -- the exact same physical bend radius
        every other primitive in this router uses, not an approximation)
        only when they are not.
        """
        step_x, step_y = self._angle_to_step(int(physical_angle) % 8)
        if abs(step_x) + abs(step_y) != 1:
            return None
        forward_cells = max(0, int(forward_cells))
        if forward_cells <= 0:
            return None
        port_cell = _physical_point_to_grid_cell(
            port_center_um,
            grid_size_um=float(self.grid.grid_size_um),
            origin_x_um=self.origin_x_um,
            origin_y_um=self.origin_y_um,
        )
        if port_cell is None:
            return None
        anchor_x = int(port_cell[0]) + step_x * forward_cells
        anchor_y = int(port_cell[1]) + step_y * forward_cells
        if not self._in_bounds(anchor_x, anchor_y):
            return None
        # Deliberately NOT grid-snapped: a port's exact physical position
        # generally does not fall exactly on a grid cell center, and a
        # pure straight run along one axis cannot itself correct the other
        # axis. Keep the port's exact Y throughout (a truly straight line,
        # X-only) and let this exact point be what standard, ordinary
        # checked-endpoint-correction resolves against -- see
        # `RouteBookkeeping.record_route`'s `target_port_center_um_override`
        # and `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
        anchor_point_um = (
            float(port_center_um[0])
            + float(step_x) * float(forward_cells) * float(self.grid.grid_size_um),
            float(port_center_um[1])
            + float(step_y) * float(forward_cells) * float(self.grid.grid_size_um),
        )
        centerline = _compress_centerline((tuple(port_center_um), anchor_point_um))
        if len(centerline) < 2:
            return None
        return centerline, (anchor_x, anchor_y)

    def _fanout_stub_centerline_um(
        self,
        port_center_um: tuple[float, float] | None,
        anchor_center_um: tuple[float, float],
        physical_angle: int,
    ) -> tuple[tuple[float, float], ...]:
        if port_center_um is None:
            return (anchor_center_um,)
        forward_x, forward_y = self._angle_to_step(int(physical_angle) % 8)
        lateral_x, lateral_y = -forward_y, forward_x
        port_x, port_y = (float(port_center_um[0]), float(port_center_um[1]))
        anchor_x, anchor_y = (float(anchor_center_um[0]), float(anchor_center_um[1]))
        delta_x = anchor_x - port_x
        delta_y = anchor_y - port_y
        forward_delta = delta_x * forward_x + delta_y * forward_y
        lateral_delta = delta_x * lateral_x + delta_y * lateral_y
        if forward_delta <= 1.0e-9:
            return _compress_centerline((port_center_um, anchor_center_um))
        lateral_abs = abs(lateral_delta)
        available_straight = forward_delta - lateral_abs
        if available_straight <= 1.0e-9:
            return _compress_centerline((port_center_um, anchor_center_um))

        preferred_first_straight_um = max(
            float(self.grid.grid_size_um),
            float(self.bend_radius_cells) * float(self.grid.grid_size_um),
        )
        first_straight_um = min(preferred_first_straight_um, available_straight)
        points: list[tuple[float, float]] = [
            (port_x, port_y),
            (
                port_x + forward_x * first_straight_um,
                port_y + forward_y * first_straight_um,
            ),
        ]
        if lateral_abs > 1.0e-9:
            lateral_sign = 1 if lateral_delta > 0.0 else -1
            diagonal_angle = (int(physical_angle) + lateral_sign) % 8
            diagonal_end = (
                points[-1][0] + forward_x * lateral_abs + lateral_x * lateral_delta,
                points[-1][1] + forward_y * lateral_abs + lateral_y * lateral_delta,
            )
            smoothed: list[tuple[float, float]] = [points[0]]
            self._append_circular_stub_bend(
                smoothed,
                start_point=points[0],
                start_angle=physical_angle,
                end_point=diagonal_end,
                end_angle=diagonal_angle,
                angle_delta=lateral_sign,
            )
            self._append_circular_stub_bend(
                smoothed,
                start_point=diagonal_end,
                start_angle=diagonal_angle,
                end_point=(anchor_x, anchor_y),
                end_angle=physical_angle,
                angle_delta=-lateral_sign,
            )
            return _compress_centerline(tuple(smoothed))
        points.append((anchor_x, anchor_y))
        return _compress_centerline(tuple(points))

    def _build_static_fanout_anchors(self) -> dict[str, _FanoutAnchor]:
        if self.fanout_access_mode_normalized != "static-stubs":
            return {}
        default_forward_cells = max(3, int(self.bend_radius_cells) + 3)
        default_lane_spacing_cells = 11
        forward_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS",
            default_forward_cells,
        )
        lane_spacing_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS",
            default_lane_spacing_cells,
        )
        stub_x_offset_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS",
            1,
        )
        if forward_cells <= 0 or lane_spacing_cells <= 0:
            return {}

        anchors: dict[str, _FanoutAnchor] = {}
        for instance_name, port_specs in self.source_port_specs_by_instance.items():
            if not self._is_dense_source_fanout_instance(instance_name):
                continue
            by_angle: dict[int, list[str]] = {}
            for port_spec in port_specs:
                _inst, _port_name, port = self.endpoint_ports_by_spec[port_spec]
                angle = self._orientation_to_angle(getattr(port, "orientation", None), flip=False)
                step_x, step_y = self._angle_to_step(angle)
                # The first static-stub implementation intentionally handles
                # cardinal MMI port rows. Diagonal component ports fall back to
                # the normal endpoint behavior until a safe breakout is defined.
                if abs(step_x) + abs(step_y) != 1:
                    continue
                by_angle.setdefault(angle, []).append(port_spec)

            for angle, group_specs in by_angle.items():
                if not self._is_dense_source_fanout_group(instance_name, angle):
                    continue
                step_x, step_y = self._angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x
                ordered_items: list[tuple[str, int, Any]] = []
                for port_spec in group_specs:
                    _inst, _port_name, port = self.endpoint_ports_by_spec[port_spec]
                    state = self.port_to_grid_state(
                        port,
                        self.origin_x_um,
                        self.origin_y_um,
                        float(self.grid.grid_size_um),
                        as_target=False,
                    )
                    lateral_cell = int(state.x) * lateral_x + int(state.y) * lateral_y
                    ordered_items.append((port_spec, lateral_cell, state))
                ordered_items.sort(key=lambda item: (item[1], item[0]))
                count = len(ordered_items)
                if count < self._dense_fanout_min_ports() or step_y != 0:
                    continue

                def add_two_bend_anchor(
                    item: tuple[str, int, Any],
                    lateral_sign: int,
                    target_anchor_y_cell: int | None,
                    initial_forward_cells: int = 0,
                    extra_final_forward_cells: int = 0,
                ) -> tuple[int, int] | None:
                    port_spec, _current_lateral, state = item
                    _inst, _port_name, port = self.endpoint_ports_by_spec[port_spec]
                    real_center = _port_center_um(port)
                    if real_center is None:
                        return None
                    stub_result = self._two_bend_static_stub_centerline_um(
                        real_center,
                        angle,
                        lateral_sign,
                        target_anchor_y_cell=target_anchor_y_cell,
                        min_forward_cells=int(forward_cells),
                        initial_forward_cells=max(0, int(initial_forward_cells)),
                        extra_final_forward_cells=max(0, int(extra_final_forward_cells)),
                    )
                    if stub_result is None:
                        return None
                    centerline, (anchor_x, anchor_y) = stub_result
                    anchor_center = self._grid_cell_center_um(anchor_x, anchor_y)
                    anchors[port_spec] = self._FanoutAnchor(
                        port_spec=port_spec,
                        state_x=anchor_x,
                        state_y=anchor_y,
                        physical_angle=angle,
                        center_um=anchor_center,
                        stub_center_cells=self._centerline_grid_cells(centerline),
                        stub_centerline_um=centerline,
                    )
                    return anchor_x, anchor_y

                lower_items = ordered_items[: count // 2]
                upper_items = ordered_items[count // 2 :]
                if not lower_items or not upper_items:
                    continue
                stub_bend_steps = self._env_fanout_stub_bend_steps()
                stagger_forward_cells = int(stub_x_offset_cells) if int(stub_bend_steps) >= 2 else 0

                lower_inner = lower_items[-1]
                lower_count = len(lower_items)
                lower_inner_anchor = add_two_bend_anchor(
                    lower_inner,
                    -1,
                    target_anchor_y_cell=None,
                    initial_forward_cells=(lower_count - 1) * stagger_forward_cells,
                    extra_final_forward_cells=0,
                )
                if lower_inner_anchor is not None:
                    lower_base_y = int(lower_inner_anchor[1])
                    for rank, item in enumerate(reversed(lower_items[:-1]), start=1):
                        initial_rank = lower_count - 1 - int(rank)
                        final_rank = int(rank)
                        _min_anchor = add_two_bend_anchor(
                            item,
                            -1,
                            target_anchor_y_cell=None,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )
                        desired_y = lower_base_y - int(rank) * int(lane_spacing_cells)
                        if _min_anchor is not None:
                            desired_y = min(desired_y, int(_min_anchor[1]))
                        add_two_bend_anchor(
                            item,
                            -1,
                            target_anchor_y_cell=desired_y,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )

                upper_inner = upper_items[0]
                upper_count = len(upper_items)
                upper_inner_anchor = add_two_bend_anchor(
                    upper_inner,
                    1,
                    target_anchor_y_cell=None,
                    initial_forward_cells=(upper_count - 1) * stagger_forward_cells,
                    extra_final_forward_cells=0,
                )
                if upper_inner_anchor is not None:
                    upper_base_y = int(upper_inner_anchor[1])
                    for rank, item in enumerate(upper_items[1:], start=1):
                        initial_rank = upper_count - 1 - int(rank)
                        final_rank = int(rank)
                        _min_anchor = add_two_bend_anchor(
                            item,
                            1,
                            target_anchor_y_cell=None,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )
                        desired_y = upper_base_y + int(rank) * int(lane_spacing_cells)
                        if _min_anchor is not None:
                            desired_y = max(desired_y, int(_min_anchor[1]))
                        add_two_bend_anchor(
                            item,
                            1,
                            target_anchor_y_cell=desired_y,
                            initial_forward_cells=initial_rank * stagger_forward_cells,
                            extra_final_forward_cells=final_rank * stagger_forward_cells,
                        )
        return anchors

    def _build_static_fanout_target_anchors(self) -> dict[str, _FanoutAnchor]:
        """Build real, pre-committed straight stubs for dense TARGET ports.

        Unlike `_build_static_fanout_anchors` (the source-side equivalent),
        this deliberately does NOT reuse the two-bend, laterally-offsetting
        stub geometry: a first implementation attempt did reuse it, and that
        turned out to actively cause a regression on `multiportmmi_8x8`
        (`n_24` newly failed to route) -- see
        `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`
        Surprises & Discoveries for the full trace evidence. Laterally
        offsetting each target port onto a different lane produced
        near-duplicate diagonal candidate paths for adjacent nets, which
        then illegally collided (non-perpendicular / collinear-overlap
        crossings) with each other. A target port does not need lateral
        redistribution the way a dense source fanout does (source nets fan
        out from adjacent ports toward widely separated destinations;
        target nets converge from widely separated sources onto adjacent
        ports, which is not the same problem) -- it only needs a short,
        protected, straight approach directly in front of itself, staggered
        in LENGTH only (middle ports reaching further, matching this
        benchmark's own physical layout intent), never in lateral position.
        Any lateral movement a route still needs happens in the main A*
        search after it reaches this anchor, exactly as it already does for
        every non-stubbed port.

        `_states_and_openings` and `_fanout_stubbed_centerline` already
        consume `fanout_anchor_by_port_spec` symmetrically for source and
        target anchors (confirmed by reading both before writing this
        function), so populating target entries in that same dict is all
        that is required for the rest of the routing pipeline to pick them
        up correctly -- no other call site needs to change.
        """
        if self.fanout_access_mode_normalized != "static-stubs":
            return {}
        default_forward_cells = max(3, int(self.bend_radius_cells) + 3)
        forward_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS",
            default_forward_cells,
        )
        spacing_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_TARGET_PROTECTED_LANE_SPACING_CELLS",
            self._env_nonnegative_int(
                "PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS",
                self._env_nonnegative_int(
                    "PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS",
                    3,
                ),
            ),
        )
        if forward_cells <= 0:
            return {}

        anchors: dict[str, _FanoutAnchor] = {}
        for instance_name, port_specs in self.target_port_specs_by_instance.items():
            if not self._is_dense_target_fanout_instance(instance_name):
                continue
            by_angle: dict[int, list[str]] = {}
            for port_spec in port_specs:
                _inst, _port_name, port = self.endpoint_ports_by_spec[port_spec]
                angle = self._orientation_to_angle(getattr(port, "orientation", None), flip=False)
                step_x, step_y = self._angle_to_step(angle)
                # Mirrors the source-side restriction: static stub geometry
                # is only defined for cardinal (axis-aligned) port rows
                # today. Diagonal target ports fall back to the normal
                # (non-stub) endpoint behavior, exactly like diagonal
                # source ports already do.
                if abs(step_x) + abs(step_y) != 1:
                    continue
                by_angle.setdefault(angle, []).append(port_spec)

            for angle, group_specs in by_angle.items():
                if not self._is_dense_target_fanout_group(instance_name, angle):
                    continue
                step_x, step_y = self._angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x

                def _lateral_position(port_spec: str) -> float:
                    _inst, _port_name, port = self.endpoint_ports_by_spec[port_spec]
                    center = _port_center_um(port)
                    if center is None:
                        return 0.0
                    return float(center[0]) * lateral_x + float(center[1]) * lateral_y

                ordered = sorted(group_specs, key=lambda spec: (_lateral_position(spec), spec))
                count = len(ordered)
                if count < self._dense_fanout_min_ports():
                    continue
                lower_specs = ordered[: count // 2]
                upper_specs = ordered[count // 2 :]
                # Rank 1 = shortest (edge of the group), highest rank =
                # longest (middle of the group) -- the "middle ones go out
                # the furthest" staggering, in length only.
                ranked: list[tuple[str, int]] = [
                    (port_spec, port_index + 1) for port_index, port_spec in enumerate(lower_specs)
                ]
                upper_count = len(upper_specs)
                ranked.extend(
                    (port_spec, upper_count - port_index)
                    for port_index, port_spec in enumerate(upper_specs)
                )
                if count == 2:
                    # A bare pair (e.g. a 2x2 switch's inputs, 1.25 um apart
                    # inside one routing cell) has no "middle": the symmetric
                    # ranking gives both the same length and the two
                    # approaches would still share the port cell. Ranks 1
                    # and 2 put their anchors at different x instead.
                    ranked = [(ordered[0], 1), (ordered[1], 2)]

                for port_spec, rank in ranked:
                    _inst, _port_name, port = self.endpoint_ports_by_spec[port_spec]
                    real_center = _port_center_um(port)
                    if real_center is None:
                        continue
                    length_cells = int(forward_cells) + int(spacing_cells) * (int(rank) - 1)
                    stub_result = self._straight_static_stub_centerline_um(
                        real_center,
                        angle,
                        length_cells,
                    )
                    if stub_result is None:
                        continue
                    centerline, (anchor_x, anchor_y) = stub_result
                    # The anchor's own exact geometric position (the
                    # straight stub's far end, keeping the port's true Y),
                    # NOT the grid cell's snapped center -- see
                    # `_straight_static_stub_centerline_um`.
                    anchor_center = centerline[-1]
                    anchors[port_spec] = self._FanoutAnchor(
                        port_spec=port_spec,
                        state_x=anchor_x,
                        state_y=anchor_y,
                        physical_angle=angle,
                        center_um=anchor_center,
                        stub_center_cells=self._centerline_grid_cells(centerline),
                        stub_centerline_um=centerline,
                    )
        return anchors

    def _dense_source_port_runway_lengths(
        self,
        jobs: list[RouteJob],
    ) -> dict[str, int]:
        """Reserve staggered source-port access in dense MMI fanout runs."""
        lengths_by_spec: dict[str, int] = {}
        if self.fanout_access_mode_normalized == "static-stubs":
            grouped_specs: dict[tuple[str, int], set[str]] = {}
            source_specs = {
                f"{run_job.inst1},{run_job.port1}"
                for run_job in jobs
                if f"{run_job.inst1},{run_job.port1}" in self.fanout_anchor_by_port_spec
            }
            for port_spec in source_specs:
                anchor = self.fanout_anchor_by_port_spec[port_spec]
                instance_name = port_spec.split(",", 1)[0]
                grouped_specs.setdefault(
                    (instance_name, int(anchor.physical_angle) % 8),
                    set(),
                ).add(port_spec)

            for (_instance_name, angle), specs in grouped_specs.items():
                if len(specs) < self._dense_fanout_min_ports():
                    continue
                step_x, step_y = self._angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x

                def anchor_lateral_position(port_spec: str) -> int:
                    anchor = self.fanout_anchor_by_port_spec[port_spec]
                    return int(anchor.state_x) * lateral_x + int(anchor.state_y) * lateral_y

                ordered_specs = sorted(
                    specs,
                    key=lambda port_spec: (
                        anchor_lateral_position(port_spec),
                        port_spec,
                    ),
                )
                count = len(ordered_specs)
                spacing_cells = self._env_nonnegative_int(
                    "PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS",
                    self._env_nonnegative_int(
                        "PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS",
                        3,
                    ),
                )
                spacing_cells = max(1, int(spacing_cells))
                lower_specs = ordered_specs[: count // 2]
                upper_specs = ordered_specs[count // 2 :]
                for port_index, port_spec in enumerate(lower_specs):
                    runway_rank = int(port_index) + 1
                    lengths_by_spec[port_spec] = spacing_cells * runway_rank
                upper_count = len(upper_specs)
                for port_index, port_spec in enumerate(upper_specs):
                    runway_rank = upper_count - int(port_index)
                    lengths_by_spec[port_spec] = spacing_cells * runway_rank
                self._equalize_dense_runway_reach(lengths_by_spec, list(ordered_specs))
            return lengths_by_spec

        if self.fanout_access_mode_normalized != "legacy-runway":
            return {}

        index = 0
        while index < len(jobs):
            job = jobs[index]
            if not self._is_dense_source_fanout_instance(job.inst1):
                index += 1
                continue
            run_end = index + 1
            while (
                run_end < len(jobs)
                and jobs[run_end].inst1 == job.inst1
                and self._is_dense_source_fanout_instance(jobs[run_end].inst1)
            ):
                run_end += 1

            run = jobs[index:run_end]
            by_angle: dict[int, list[RouteJob]] = {}
            for run_job in run:
                angle = self._orientation_to_angle(
                    getattr(run_job.source_port, "orientation", None),
                    flip=False,
                )
                by_angle.setdefault(angle, []).append(run_job)

            for angle, angle_jobs in by_angle.items():
                if not self._is_dense_source_fanout_group(job.inst1, angle):
                    continue
                step_x, step_y = self._angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x

                def _lateral_position_for_dense_source_runway(run_job: RouteJob) -> float:
                    center = _port_center_um(run_job.source_port)
                    if center is None:
                        return float(run_job.route_index)
                    return float(center[0]) * lateral_x + float(center[1]) * lateral_y

                ordered = sorted(
                    angle_jobs,
                    key=lambda run_job: (
                        _lateral_position_for_dense_source_runway(run_job),
                        int(run_job.route_index),
                    ),
                )
                count = len(ordered)
                group_port_specs: list[str] = []
                for port_index, run_job in enumerate(ordered):
                    port_spec = f"{run_job.inst1},{run_job.port1}"
                    lengths_by_spec[port_spec] = 3 + 3 * (count - 1 - port_index)
                    group_port_specs.append(port_spec)
                self._equalize_dense_runway_reach(lengths_by_spec, group_port_specs)

            index = run_end
        return lengths_by_spec

    def _equalize_dense_runway_reach(
        self,
        lengths_by_spec: dict[str, int],
        port_specs: list[str],
    ) -> None:
        """Extend every port in one dense group to match the group's own
        furthest forward reach, in place.

        A port's staggered length only controls how far its own raw
        footprint extends. That raw footprint (before any per-port lateral
        narrowing) is reserved globally, in `port_runway_cells_by_spec`,
        which becomes part of `static_blocked_cells_before_port_reservations`
        for every OTHER net's routing (see `run()`, where
        `port_runway_static_cells` is unioned into it). If ports in the same
        dense group are given different lengths, the longest-reaching one's
        raw footprint -- which is a wide box, not just that port's own
        narrow lane, since half_width_cells is not reduced by staggering --
        can end up blocking a shorter neighbor's own narrower lane in the
        gap between them, even though nothing physically stands in that gap.
        Equalizing every port in the group to the same forward reach as its
        longest sibling removes this self-inflicted gap: by definition, no
        sibling's own raw footprint reserves anything beyond the group's
        current maximum reach, so no sibling can block another sibling
        beyond that point once every port shares the same reach.
        """
        if not port_specs:
            return
        half_width_cells = int(self.port_lane_half_width_cells)
        max_reach = max(int(lengths_by_spec[spec]) + half_width_cells - 1 for spec in port_specs)
        equalized_length = max_reach - half_width_cells + 1
        for spec in port_specs:
            lengths_by_spec[spec] = max(int(lengths_by_spec[spec]), equalized_length)

    def _dense_target_port_runway_lengths(
        self,
        jobs: list[RouteJob],
    ) -> dict[str, int]:
        """Reserve staggered target-port access for dense same-instance sinks."""
        lengths_by_spec: dict[str, int] = {}
        grouped: dict[tuple[str, int], list[RouteJob]] = {}
        for run_job in jobs:
            angle = self._orientation_to_angle(
                getattr(run_job.target_port, "orientation", None),
                flip=False,
            )
            grouped.setdefault((run_job.inst2, int(angle)), []).append(run_job)

        base_cells = max(1, int(self.bend_radius_cells) + 1)
        spacing_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_TARGET_PROTECTED_LANE_SPACING_CELLS",
            self._env_nonnegative_int(
                "PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS",
                self._env_nonnegative_int(
                    "PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS",
                    3,
                ),
            ),
        )
        spacing_cells = max(1, int(spacing_cells))

        for (_instance_name, angle), angle_jobs in grouped.items():
            if len(angle_jobs) < 4:
                continue
            step_x, step_y = self._angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x

            def _lateral_position_for_dense_target_runway(run_job: RouteJob) -> float:
                center = _port_center_um(run_job.target_port)
                if center is None:
                    return float(run_job.route_index)
                return float(center[0]) * lateral_x + float(center[1]) * lateral_y

            ordered = sorted(
                angle_jobs,
                key=lambda run_job: (
                    _lateral_position_for_dense_target_runway(run_job),
                    int(run_job.route_index),
                ),
            )
            count = len(ordered)
            lower_jobs = ordered[: count // 2]
            upper_jobs = ordered[count // 2 :]
            group_port_specs: list[str] = []
            for port_index, run_job in enumerate(lower_jobs):
                port_spec = f"{run_job.inst2},{run_job.port2}"
                if port_spec in self.fanout_anchor_by_port_spec:
                    # A real, pre-committed static stub already exists for
                    # this port (see `_build_static_fanout_target_anchors`):
                    # the abstract reservation this function computes is
                    # redundant for it and would just reintroduce the
                    # flattened-staggering behavior this mechanism exists to
                    # avoid, so skip it entirely (0 reservation, excluded
                    # from equalization) rather than reserving anything.
                    lengths_by_spec[port_spec] = 0
                    continue
                lengths_by_spec[port_spec] = base_cells + spacing_cells * int(port_index)
                group_port_specs.append(port_spec)
            upper_count = len(upper_jobs)
            for port_index, run_job in enumerate(upper_jobs):
                port_spec = f"{run_job.inst2},{run_job.port2}"
                if port_spec in self.fanout_anchor_by_port_spec:
                    lengths_by_spec[port_spec] = 0
                    continue
                lengths_by_spec[port_spec] = base_cells + spacing_cells * (
                    upper_count - 1 - int(port_index)
                )
                group_port_specs.append(port_spec)
            self._equalize_dense_runway_reach(lengths_by_spec, group_port_specs)
        return lengths_by_spec

    def _filter_dense_port_opening(
        self,
        port_spec: str,
        cells: set[tuple[int, int]],
    ) -> set[tuple[int, int]]:
        owner_group = self.dense_port_lateral_owner_groups.get(port_spec)
        if owner_group is not None and cells:
            lateral_x, lateral_y, owners = owner_group
            grid_size = float(self.grid.grid_size_um)
            filtered: set[tuple[int, int]] = set()
            for cell_x, cell_y in cells:
                center_x, center_y = grid_cell_center(cell_x, cell_y, self.grid)
                lateral_position = center_x * lateral_x + center_y * lateral_y
                nearest_spec = min(
                    owners,
                    key=lambda item: (abs(lateral_position - item[1]), item[0]),
                )[0]
                if nearest_spec == port_spec:
                    filtered.add((cell_x, cell_y))
            return filtered

        window = self.dense_port_lateral_windows.get(port_spec)
        if window is None or not cells:
            return set(cells)
        lateral_x, lateral_y, lower, upper, lane_margin_um = window
        grid_size = float(self.grid.grid_size_um)
        lower -= lane_margin_um
        upper += lane_margin_um
        eps = max(1.0e-9, grid_size * 1.0e-9)
        filtered: set[tuple[int, int]] = set()
        for cell_x, cell_y in cells:
            center_x, center_y = grid_cell_center(cell_x, cell_y, self.grid)
            lateral_position = center_x * lateral_x + center_y * lateral_y
            if lower - eps <= lateral_position <= upper + eps:
                filtered.add((cell_x, cell_y))
        return filtered

    def _opened_cells_for_spec(
        self,
        cells_by_spec: Mapping[str, set[tuple[int, int]]],
        port_spec: str,
    ) -> set[tuple[int, int]]:
        return self._filter_dense_port_opening(
            port_spec,
            set(cells_by_spec.get(port_spec, set())),
        )

    def _foreign_keepout_open_cells_for_spec(self, port_spec: str) -> set[tuple[int, int]]:
        cluster_specs = self.dense_source_cluster_specs_by_port_spec.get(port_spec)
        if cluster_specs:
            cells: set[tuple[int, int]] = set()
            for cluster_port_spec in cluster_specs:
                cells.update(self.foreign_port_keepout_cells_by_spec.get(cluster_port_spec, set()))
            return cells - self.normal_port_runway_cells
        return (
            self._opened_cells_for_spec(self.foreign_port_keepout_cells_by_spec, port_spec)
            - self.normal_port_runway_cells
        )

    def _foreign_keepout_cleanup_cells_for_spec(self, port_spec: str) -> set[tuple[int, int]]:
        cells = set(self.foreign_port_keepout_cells_by_spec.get(port_spec, set()))
        if not cells:
            return set()
        cells.difference_update(self.normal_port_runway_cells)
        cells.difference_update(self.fanout_stub_static_cells)
        cells.difference_update(self._cells_in_raw_static_geometry(cells))
        return cells

    def _foreign_keepout_cleanup_cells_for_job(self, job: RouteJob) -> list[tuple[int, int]]:
        cells = self._foreign_keepout_cleanup_cells_for_spec(f"{job.inst1},{job.port1}")
        cells.update(self._foreign_keepout_cleanup_cells_for_spec(f"{job.inst2},{job.port2}"))
        return sorted(cells)

    def _endpoint_state_for_lane_assignment(self, port: Port, *, as_target: bool):
        return self.port_to_grid_state(
            port,
            self.origin_x_um,
            self.origin_y_um,
            float(self.grid.grid_size_um),
            as_target=as_target,
        )

    def _topological_net_route_order(self, jobs: list[RouteJob]) -> list[RouteJob]:
        """Route nets in topological order (source-instance depth), tiebroken by declaration order.

        Depth is derived directly from this batch's own net graph (`inst1 ->
        inst2` edges across `jobs`), not from any benchmark's optional
        `NODE_DEPTHS` metadata (`self.node_depths`) or the crossing-plan
        topology analysis in `route_rust_crossing_plan.py`. Net ordering
        needs a depth signal unconditionally, for every benchmark and every
        crossing mode: `self.node_depths` is `{}` (not derived) for every
        benchmark that does not hardcode `NODE_DEPTHS`, and the crossing-plan
        topology metadata is deliberately withheld entirely in `lidar-pure`
        mode (`_build_crossing_plan_info`'s own early return, protecting the
        router-discovered crossing path from topology-precomputed hints) --
        neither is a usable ordering signal in general. A net's depth is its
        source instance's depth (how many hops from a true source, an
        instance with no incoming net in this batch); nets sharing a source
        instance share a depth and so keep their relative declaration order,
        matching LiDAR's own `Nets.__lt__` (`comp_dist` primary key,
        declaration order tiebreak).
        """
        incoming: dict[str, set[str]] = {}
        all_nodes: set[str] = set()
        for job in jobs:
            all_nodes.add(job.inst1)
            all_nodes.add(job.inst2)
            incoming.setdefault(job.inst2, set()).add(job.inst1)

        depth_by_node: dict[str, int] = {}

        def resolve_depth(node: str, visiting: set[str]) -> int:
            if node in depth_by_node:
                return depth_by_node[node]
            sources = incoming.get(node)
            if not sources or node in visiting:
                # No incoming edges (a true source), or a cycle in the net
                # graph -- a real photonic netlist's signal flow is a DAG,
                # so a cycle should not happen, but treat a cycle member as
                # depth 0 rather than recursing forever.
                depth_by_node[node] = 0
                return 0
            visiting.add(node)
            depth = 1 + max(resolve_depth(source, visiting) for source in sources)
            visiting.discard(node)
            depth_by_node[node] = depth
            return depth

        for node in all_nodes:
            resolve_depth(node, set())

        if os.environ.get("PHOTONIC_ROUTER_LAYER_ORDER", "").strip().lower() == "span":
            # EXPERIMENT (2026-08-27, off by default; decision pending in
            # `.agent/execplans/2026-08-27-router-fixes-for-crossing-grid-stubs.md`
            # Decision Log): within a layer, shortest nets first, so that a
            # planar fan-out (pre-placed crossing grids, no crossings outside
            # the grid) nests the widest-span stubs outermost. Completes
            # `benes_16x16` grid mode (196/196) but breaks the normal
            # router's Benes crossing discovery, so it must become a mode
            # property (option plumbing), not stay an env var.

            def span(route_job: RouteJob) -> int:
                source_state = self.port_to_grid_state(
                    route_job.source_port,
                    self.origin_x_um,
                    self.origin_y_um,
                    float(self.grid.grid_size_um),
                    as_target=False,
                )
                target_state = self.port_to_grid_state(
                    route_job.target_port,
                    self.origin_x_um,
                    self.origin_y_um,
                    float(self.grid.grid_size_um),
                    as_target=True,
                )
                return abs(int(source_state.x) - int(target_state.x)) + abs(
                    int(source_state.y) - int(target_state.y)
                )

            ordered = sorted(
                jobs,
                key=lambda route_job: (
                    depth_by_node[route_job.inst1],
                    span(route_job),
                    int(route_job.route_index),
                ),
            )
        else:
            ordered = sorted(
                jobs,
                key=lambda route_job: (depth_by_node[route_job.inst1], int(route_job.route_index)),
            )
        return self._debug_hoist_instance_first(ordered)

    @staticmethod
    def _debug_hoist_instance_first(ordered: list[RouteJob]) -> list[RouteJob]:
        """Debug-only reordering: `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_INSTANCE=<name>`
        hoists every net touching the named instance to the front of the
        routing order (relative order preserved on both sides), so a dense
        group can be routed and debugged on empty dynamics in minutes
        instead of behind a 30-minute full-context prefix. Routing results
        under this knob are NOT comparable to stable runs -- ordering
        changes every downstream commit; diagnosis use only.
        """
        raw_net_ids = os.environ.get("PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS", "").strip()
        if raw_net_ids:
            wanted = {
                int(token)
                for token in raw_net_ids.split(",")
                if token.strip().isdigit()
            }
            hoisted = [job for job in ordered if int(job.net_id) in wanted]
            rest = [job for job in ordered if int(job.net_id) not in wanted]
            print(
                f"      - DEBUG route order: {len(hoisted)} nets from the explicit "
                f"net-id list hoisted to the front (diagnosis only)"
            )
            return hoisted + rest
        instance = os.environ.get("PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_INSTANCE", "").strip()
        if not instance:
            return ordered
        hoisted = [job for job in ordered if instance in (job.inst1, job.inst2)]
        rest = [job for job in ordered if instance not in (job.inst1, job.inst2)]
        print(
            f"      - DEBUG route order: {len(hoisted)} nets touching "
            f"{instance!r} hoisted to the front (diagnosis only)"
        )
        return hoisted + rest

    def _timing_start(self) -> float:
        return time.perf_counter() if self.collect_timing else 0.0

    def _record_elapsed(self, bucket_name: str, start_s: float, *, failed: bool = False) -> None:
        if not self.collect_timing:
            return
        self.route_timing_buckets[bucket_name].record_elapsed(
            time.perf_counter() - start_s,
            failed=failed,
        )

    def _record_native_batch_timings(self, batch_result: dict[str, Any]) -> None:
        if not self.collect_pipeline_timing:
            return
        raw_timings = batch_result.get("timings_s")
        if raw_timings is None:
            return
        for raw_name, raw_elapsed_s in dict(raw_timings).items():
            try:
                elapsed_s = float(raw_elapsed_s)
            except (TypeError, ValueError):
                continue
            name = f"native_batch_{raw_name}"
            self.route_nets_timings_s[name] = self.route_nets_timings_s.get(name, 0.0) + elapsed_s

    def _report_long_straight_congestion(self, batch_result: dict[str, Any]) -> None:
        records = [
            dict(record)
            for record in cast(
                Iterable[Mapping[str, object]],
                batch_result.get("long_straight_congestion", []),
            )
        ]
        if not records or not self.verbose_route_diagnostics:
            return
        grouped: dict[int, list[dict[str, object]]] = {}
        for record in records:
            try:
                net_id = int(record.get("net_id", -1))
            except (TypeError, ValueError):
                continue
            grouped.setdefault(net_id, []).append(record)
        print("      - Long-straight congestion contributors:")
        for net_id in sorted(grouped):
            job = self.route_jobs_by_id.get(net_id)
            label = job.net_name if job is not None else f"net_id={net_id}"
            route_index = job.route_index if job is not None else "?"
            segments = grouped[net_id]
            marked_cells = sum(int(segment.get("marked_cells", 0) or 0) for segment in segments)
            lengths = ", ".join(
                f"{float(segment.get('length_um', 0.0) or 0.0):.1f}um" for segment in segments
            )
            print(
                "        "
                f"route[{route_index}] {label}: "
                f"{len(segments)} long straight(s), "
                f"marked_cells={marked_cells}, lengths=[{lengths}]"
            )

    def _committed_dynamic_cells(
        self, *, exclude_net_id: int | None = None
    ) -> set[tuple[int, int]]:
        return self.route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)

    def _committed_dynamic_cells_for_attempt(
        self,
        *,
        exclude_net_id: int | None = None,
    ) -> set[tuple[int, int]]:
        if self.route_bookkeeping.diagnostics_enabled:
            return self.route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)
        merged: set[tuple[int, int]] = set()
        for net_id in self.route_bookkeeping.records_by_id:
            if exclude_net_id is not None and int(net_id) == int(exclude_net_id):
                continue
            merged.update(self._route_cells_from_router(net_id))
        return merged

    def _route_cells_from_router(self, net_id: int) -> set[tuple[int, int]]:
        return {(int(cell[0]), int(cell[1])) for cell in self.router.get_net_cells(int(net_id))}

    def _append_centerline_points(
        self,
        out: list[tuple[float, float]],
        points: Iterable[tuple[float, float]],
    ) -> None:
        for raw_x, raw_y in points:
            point = (float(raw_x), float(raw_y))
            if out:
                last_x, last_y = out[-1]
                if math.hypot(point[0] - last_x, point[1] - last_y) <= 1.0e-9:
                    continue
            out.append(point)

    def _fanout_stubbed_centerline(
        self,
        job: RouteJob,
        route_obj: Any,
    ) -> tuple[tuple[float, float], ...]:
        """Eagerly stitch a SOURCE fanout stub onto a route's own centerline.

        Deliberately does NOT do the same for a TARGET fanout stub: a
        target stub's anchor is not guaranteed to land exactly on a grid
        cell (a plain straight stub can only correct the forward axis, not
        the lateral one -- see `_straight_static_stub_centerline_um`), so
        stitching it in eagerly here would mark the record "already fully
        corrected" and skip the real endpoint-correction pass needed to
        close that gap. Instead, `_record_route` points this net's own
        effective target port at the anchor's exact position
        (`target_port_center_um_override`), letting ordinary endpoint
        correction resolve it the same way it would for any ordinary port,
        and `_append_target_fanout_stubs_after_correction` appends the
        fixed, pre-built anchor-to-true-port segment once that correction
        has completed. See
        `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
        """
        source_anchor = self.fanout_anchor_by_port_spec.get(f"{job.inst1},{job.port1}")
        if source_anchor is None:
            return ()
        route_primitive_centerline = getattr(self.router, "route_primitive_centerline", None)
        try:
            if route_primitive_centerline is not None:
                route_centerline = _centerline_tuple(route_primitive_centerline(route_obj))
            else:
                route_centerline = ()
        except Exception:
            route_centerline = ()
        if len(route_centerline) < 2:
            return ()
        points: list[tuple[float, float]] = []
        self._append_centerline_points(points, source_anchor.stub_centerline_um)
        self._append_centerline_points(points, route_centerline)
        centerline = _compress_centerline(tuple(points))
        return centerline if len(centerline) >= 2 else ()

    def _endpoint_bump_candidate_open_cells_for_state(
        self,
        state: Any,
    ) -> set[tuple[int, int]]:
        """Cells a local endpoint bump may need opened against its own port pad."""
        base_x = int(state.x)
        base_y = int(state.y)
        step_x, step_y = self._angle_to_step(int(state.angle) % 8)
        side_steps = ((-step_y, step_x), (step_y, -step_x))
        reach = max(1, int(self.bend_radius_cells))
        axis_reach = 4 * reach
        lateral_reach = 2 * reach
        cells: set[tuple[int, int]] = set()
        for forward in range(-axis_reach, reach + 1):
            if forward == 0:
                continue
            x = base_x + step_x * forward
            y = base_y + step_y * forward
            if 0 <= x < int(self.grid.width) and 0 <= y < int(self.grid.height):
                cells.add((x, y))
        for side_x, side_y in side_steps:
            for lateral in range(1, lateral_reach + 1):
                for forward in range(-axis_reach, reach + 1):
                    if forward == 0:
                        continue
                    x = base_x + step_x * forward + side_x * lateral
                    y = base_y + step_y * forward + side_y * lateral
                    if 0 <= x < int(self.grid.width) and 0 <= y < int(self.grid.height):
                        cells.add((x, y))
        return cells

    def _states_and_openings(
        self,
        job: RouteJob,
    ) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_fanout_anchor = self.fanout_anchor_by_port_spec.get(port1_spec)
        target_fanout_anchor = self.fanout_anchor_by_port_spec.get(port2_spec)
        if source_fanout_anchor is None:
            source_state = self.port_to_grid_state(
                job.source_port,
                self.origin_x_um,
                self.origin_y_um,
                float(self.grid.grid_size_um),
                as_target=False,
            )
        else:
            source_state = self.rust_backend.State(
                int(source_fanout_anchor.state_x),
                int(source_fanout_anchor.state_y),
                int(source_fanout_anchor.physical_angle) % 8,
            )
        if target_fanout_anchor is None:
            target_state = self.port_to_grid_state(
                job.target_port,
                self.origin_x_um,
                self.origin_y_um,
                float(self.grid.grid_size_um),
                as_target=True,
            )
        else:
            target_state = self.rust_backend.State(
                int(target_fanout_anchor.state_x),
                int(target_fanout_anchor.state_y),
                (int(target_fanout_anchor.physical_angle) + 4) % 8,
            )
        source_lane_offset = self.port_state_lane_offsets.get((f"{job.inst1},{job.port1}", False))
        if source_lane_offset is not None and source_fanout_anchor is None:
            source_state = self.rust_backend.State(
                int(source_state.x) + int(source_lane_offset[0]),
                int(source_state.y) + int(source_lane_offset[1]),
                int(source_state.angle),
            )
        target_lane_offset = self.port_state_lane_offsets.get((f"{job.inst2},{job.port2}", True))
        if target_lane_offset is not None and target_fanout_anchor is None:
            target_state = self.rust_backend.State(
                int(target_state.x) + int(target_lane_offset[0]),
                int(target_state.y) + int(target_lane_offset[1]),
                int(target_state.angle),
            )
        if source_fanout_anchor is None and target_fanout_anchor is None:
            source_state, target_state, original_anchor_cells = self._snap_nearly_collinear_states(
                source_state,
                target_state,
                job.source_port,
                job.target_port,
            )
            source_state, target_state, snapped_anchor_cells = (
                self._snap_same_heading_minimum_bend_offset(source_state, target_state)
            )
            original_anchor_cells.update(snapped_anchor_cells)
        else:
            original_anchor_cells = {
                (int(source_state.x), int(source_state.y)),
                (int(target_state.x), int(target_state.y)),
            }
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        endpoint_foreign_keepout_open_cells = set(
            self._foreign_keepout_open_cells_for_spec(port1_spec)
        )
        endpoint_foreign_keepout_open_cells.update(
            self._foreign_keepout_open_cells_for_spec(port2_spec)
        )
        opened_candidate_cells = set(
            self._opened_cells_for_spec(self.port_access_candidate_cells_by_spec, port1_spec)
        )
        opened_candidate_cells.update(
            self._opened_cells_for_spec(self.port_access_candidate_cells_by_spec, port2_spec)
        )
        opened_candidate_cells.update(endpoint_foreign_keepout_open_cells)
        opened_candidate_cells.update(original_anchor_cells)
        opened_candidate_cells.update({source_anchor_cell, target_anchor_cell})

        opened_cells_set = set(
            self._opened_cells_for_spec(self.port_access_cells_by_spec, port1_spec)
        )
        opened_cells_set.update(
            self._opened_cells_for_spec(self.port_access_cells_by_spec, port2_spec)
        )
        opened_cells_set.update(endpoint_foreign_keepout_open_cells)
        opened_cells_set.update(original_anchor_cells)
        opened_cells_set.update({source_anchor_cell, target_anchor_cell})
        if self.fanout_stub_static_cells:
            current_fanout_stub_open_cells: set[tuple[int, int]] = set()
            current_fanout_stub_open_cells.update(
                self.fanout_stub_static_cells_by_spec.get(port1_spec, set())
            )
            current_fanout_stub_open_cells.update(
                self.fanout_stub_static_cells_by_spec.get(port2_spec, set())
            )
            allowed_fanout_stub_open_cells = (
                current_fanout_stub_open_cells
                | original_anchor_cells
                | {source_anchor_cell, target_anchor_cell}
            )
            foreign_fanout_stub_static_cells = (
                self.fanout_stub_static_cells - allowed_fanout_stub_open_cells
            )
            opened_candidate_cells.difference_update(foreign_fanout_stub_static_cells)
            opened_cells_set.difference_update(foreign_fanout_stub_static_cells)
        source_endpoint_bump_open_cells = self._endpoint_bump_candidate_open_cells_for_state(
            source_state
        )
        target_endpoint_bump_open_cells = self._endpoint_bump_candidate_open_cells_for_state(
            target_state
        )
        opened_candidate_cells.update(source_endpoint_bump_open_cells)
        opened_candidate_cells.update(target_endpoint_bump_open_cells)
        trace_endpoint_bumps = os.environ.get("PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS", "")
        if trace_endpoint_bumps and (
            trace_endpoint_bumps.strip() == "*"
            or str(int(job.net_id)) in {item.strip() for item in trace_endpoint_bumps.split(",")}
        ):
            print(
                "endpoint_open_trace "
                f"net_id={int(job.net_id)} "
                f"source_state=({int(source_state.x)},{int(source_state.y)},{int(source_state.angle)}) "
                f"target_state=({int(target_state.x)},{int(target_state.y)},{int(target_state.angle)}) "
                f"source_bump_open={len(source_endpoint_bump_open_cells)} "
                f"target_bump_open={len(target_endpoint_bump_open_cells)} "
                f"opened_candidate={len(opened_candidate_cells)} "
                f"has_112_243={(112, 243) in opened_candidate_cells}"
            )
        return (
            source_state,
            target_state,
            opened_candidate_cells,
            opened_cells_set,
            sorted(opened_cells_set),
        )

    def _routing_endpoint_center_um(
        self,
        job: RouteJob,
        *,
        source: bool,
    ) -> tuple[float, float] | None:
        if source:
            port_spec = f"{job.inst1},{job.port1}"
            anchor = self.fanout_anchor_by_port_spec.get(port_spec)
            return anchor.center_um if anchor is not None else _port_center_um(job.source_port)
        port_spec = f"{job.inst2},{job.port2}"
        anchor = self.fanout_anchor_by_port_spec.get(port_spec)
        return anchor.center_um if anchor is not None else _port_center_um(job.target_port)

    def _state_openings_for_job(
        self,
        job: RouteJob,
    ) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
        return self.route_state_openings_by_id[int(job.net_id)]

    def _clearance_exempt_cells_for_job(self, job: RouteJob) -> list[tuple[int, int]]:
        return self.batch_clearance_exempt_cells_by_id.get(int(job.net_id), [])

    def _clearance_exempt_cell_set_for_job(self, job: RouteJob) -> set[tuple[int, int]]:
        return set(self._clearance_exempt_cells_for_job(job))

    def _static_cells_in_rect(self, min_x: int, max_x: int, min_y: int, max_y: int) -> int:
        if min_x > max_x or min_y > max_y:
            return 0
        if self.blocked_static_rects_for_diagnostics:
            return sum(
                _rect_overlap_cell_count(
                    rect,
                    min_x=min_x,
                    max_x=max_x,
                    min_y=min_y,
                    max_y=max_y,
                )
                for rect in self.blocked_static_rects_for_diagnostics
            )
        return sum(
            1
            for x, y in self.static_blocked_cells_before_port_reservations
            if min_x <= x <= max_x and min_y <= y <= max_y
        )

    def _cells_in_rect(
        self,
        cells: set[tuple[int, int]],
        *,
        min_x: int,
        max_x: int,
        min_y: int,
        max_y: int,
    ) -> int:
        if min_x > max_x or min_y > max_y:
            return 0
        return sum(1 for x, y in cells if min_x <= x <= max_x and min_y <= y <= max_y)

    def _route_attempt_diagnostics(
        self,
        job: RouteJob,
        route_obj: object | None,
        *,
        candidate_blockers: list[int] | None = None,
        ripup_ids: list[int] | None = None,
    ) -> dict[str, object]:
        dynamic_cells_before = self._committed_dynamic_cells_for_attempt(exclude_net_id=job.net_id)
        source_state, target_state, _, _, opened_cells = self._state_openings_for_job(job)
        source_x = int(source_state.x)
        source_y = int(source_state.y)
        source_angle = int(source_state.angle)
        target_x = int(target_state.x)
        target_y = int(target_state.y)
        target_angle = int(target_state.angle)
        span_x = abs(target_x - source_x)
        span_y = abs(target_y - source_y)
        span_bbox_min_x = min(source_x, target_x)
        span_bbox_max_x = max(source_x, target_x)
        span_bbox_min_y = min(source_y, target_y)
        span_bbox_max_y = max(source_y, target_y)
        span_bbox_area = _rect_cell_count(
            min_x=span_bbox_min_x,
            max_x=span_bbox_max_x,
            min_y=span_bbox_min_y,
            max_y=span_bbox_max_y,
        )
        window_min_x = int(getattr(route_obj, "last_window_min_x", 0)) if route_obj else 0
        window_max_x = int(getattr(route_obj, "last_window_max_x", -1)) if route_obj else -1
        window_min_y = int(getattr(route_obj, "last_window_min_y", 0)) if route_obj else 0
        window_max_y = int(getattr(route_obj, "last_window_max_y", -1)) if route_obj else -1
        window_area = int(getattr(route_obj, "last_window_area_cells", 0)) if route_obj else 0
        if window_area <= 0:
            window_area = _rect_cell_count(
                min_x=window_min_x,
                max_x=window_max_x,
                min_y=window_min_y,
                max_y=window_max_y,
            )
        window_static_cells = self._static_cells_in_rect(
            window_min_x,
            window_max_x,
            window_min_y,
            window_max_y,
        )
        window_dynamic_cells = self._cells_in_rect(
            dynamic_cells_before,
            min_x=window_min_x,
            max_x=window_max_x,
            min_y=window_min_y,
            max_y=window_max_y,
        )
        span_static_cells = self._static_cells_in_rect(
            span_bbox_min_x,
            span_bbox_max_x,
            span_bbox_min_y,
            span_bbox_max_y,
        )
        span_dynamic_cells = self._cells_in_rect(
            dynamic_cells_before,
            min_x=span_bbox_min_x,
            max_x=span_bbox_max_x,
            min_y=span_bbox_min_y,
            max_y=span_bbox_max_y,
        )
        route_cells = getattr(route_obj, "cells", None) if route_obj is not None else None
        route_bbox = _route_cells_bbox(route_cells or ())
        if route_bbox is None:
            route_bbox_min_x = 0
            route_bbox_max_x = -1
            route_bbox_min_y = 0
            route_bbox_max_y = -1
        else:
            route_bbox_min_x, route_bbox_max_x, route_bbox_min_y, route_bbox_max_y = route_bbox
        route_bbox_area = _rect_cell_count(
            min_x=route_bbox_min_x,
            max_x=route_bbox_max_x,
            min_y=route_bbox_min_y,
            max_y=route_bbox_max_y,
        )
        total_cost = float(getattr(route_obj, "total_cost", 0.0)) if route_obj is not None else None
        euclidean_lower_bound, heading_lower_bound = self._source_lower_bounds(
            source_x=source_x,
            source_y=source_y,
            source_angle=source_angle,
            target_x=target_x,
            target_y=target_y,
            target_angle=target_angle,
        )
        blocker_ids = list(candidate_blockers or [])
        victim_ids = list(ripup_ids or [])
        raw_dynamic_cells: set[tuple[int, int]] = set()
        raw_dynamic_refcount_gt1_cells: set[tuple[int, int]] = set()
        raw_core_cells: set[tuple[int, int]] = set()
        raw_core_refcount_gt1_cells: set[tuple[int, int]] = set()
        raw_net_route_cells: set[tuple[int, int]] = set()
        if hasattr(self.router, "raw_dynamic_obstacle_cells"):
            raw_dynamic_entries = [
                (int(x), int(y), int(refs))
                for x, y, refs in self.router.raw_dynamic_obstacle_cells()
            ]
            raw_dynamic_cells = {(x, y) for x, y, _ in raw_dynamic_entries}
            raw_dynamic_refcount_gt1_cells = {
                (x, y) for x, y, refs in raw_dynamic_entries if refs > 1
            }
        if hasattr(self.router, "raw_dynamic_core_cells"):
            raw_core_entries = [
                (int(x), int(y), int(refs)) for x, y, refs in self.router.raw_dynamic_core_cells()
            ]
            raw_core_cells = {(x, y) for x, y, _ in raw_core_entries}
            raw_core_refcount_gt1_cells = {(x, y) for x, y, refs in raw_core_entries if refs > 1}
        if hasattr(self.router, "all_net_route_cells"):
            for _, cells in self.router.all_net_route_cells():
                raw_net_route_cells.update((int(cell[0]), int(cell[1])) for cell in cells)
        raw_dynamic_without_owner = raw_dynamic_cells - raw_net_route_cells
        raw_net_route_without_dynamic = raw_net_route_cells - raw_dynamic_cells
        raw_core_without_dynamic = raw_core_cells - raw_dynamic_cells
        raw_dynamic_span_cells = {
            (x, y)
            for x, y in raw_dynamic_cells
            if span_bbox_min_x <= x <= span_bbox_max_x and span_bbox_min_y <= y <= span_bbox_max_y
        }
        raw_dynamic_without_owner_span_cells = {
            (x, y)
            for x, y in raw_dynamic_without_owner
            if span_bbox_min_x <= x <= span_bbox_max_x and span_bbox_min_y <= y <= span_bbox_max_y
        }
        return {
            "source_state": [source_x, source_y, source_angle],
            "target_state": [target_x, target_y, target_angle],
            "span_x_cells": span_x,
            "span_y_cells": span_y,
            "span_manhattan_cells": span_x + span_y,
            "span_bbox_area_cells": span_bbox_area,
            "span_static_cells": span_static_cells,
            "span_dynamic_cells": span_dynamic_cells,
            "route_bbox_min_x": route_bbox_min_x,
            "route_bbox_max_x": route_bbox_max_x,
            "route_bbox_min_y": route_bbox_min_y,
            "route_bbox_max_y": route_bbox_max_y,
            "route_bbox_width_cells": max(0, route_bbox_max_x - route_bbox_min_x + 1),
            "route_bbox_height_cells": max(0, route_bbox_max_y - route_bbox_min_y + 1),
            "route_bbox_area_cells": route_bbox_area,
            "route_bbox_to_span_bbox_area": (
                float(route_bbox_area) / float(span_bbox_area)
                if span_bbox_area > 0 and route_bbox_area > 0
                else None
            ),
            "opened_cells_count": len(opened_cells),
            "block_radius_cells": self.block_radius_cells,
            "dynamic_obstacle_search_expansion_radius_cells": (
                self.clearance_policy.dynamic_obstacle_search_expansion_radius_cells
            ),
            "dynamic_route_commit_keepout_radius_cells": (
                self.clearance_policy.dynamic_route_commit_keepout_radius_cells
            ),
            "dynamic_route_core_radius_cells": (
                self.clearance_policy.dynamic_route_core_radius_cells
            ),
            "bend_radius_cells": self.bend_radius_cells,
            "window_width_cells": max(0, window_max_x - window_min_x + 1),
            "window_height_cells": max(0, window_max_y - window_min_y + 1),
            "window_area_cells": window_area,
            "window_to_span_bbox_area": (
                float(window_area) / float(span_bbox_area)
                if span_bbox_area > 0 and window_area > 0
                else None
            ),
            "route_bbox_to_window_area": (
                float(route_bbox_area) / float(window_area)
                if window_area > 0 and route_bbox_area > 0
                else None
            ),
            "window_static_cells": window_static_cells,
            "window_dynamic_cells": window_dynamic_cells,
            "window_static_density": (
                float(window_static_cells) / float(window_area) if window_area > 0 else None
            ),
            "window_dynamic_density": (
                float(window_dynamic_cells) / float(window_area) if window_area > 0 else None
            ),
            "total_cost": total_cost,
            "euclidean_lower_bound_cost": euclidean_lower_bound,
            "heading_lower_bound_cost": heading_lower_bound,
            "euclidean_lower_bound_to_cost": (
                euclidean_lower_bound / total_cost
                if total_cost is not None and total_cost > 0.0
                else None
            ),
            "heading_lower_bound_to_cost": (
                heading_lower_bound / total_cost
                if total_cost is not None and total_cost > 0.0
                else None
            ),
            "heading_lower_bound_gap_cost": (
                total_cost - heading_lower_bound if total_cost is not None else None
            ),
            "committed_dynamic_cells_before": len(dynamic_cells_before),
            "raw_dynamic_obstacle_cells_before": len(raw_dynamic_cells),
            "raw_dynamic_core_cells_before": len(raw_core_cells),
            "raw_net_route_cells_before": len(raw_net_route_cells),
            "raw_dynamic_refcount_gt1_count": len(raw_dynamic_refcount_gt1_cells),
            "raw_dynamic_refcount_gt1_bbox": _cells_bbox(raw_dynamic_refcount_gt1_cells),
            "raw_core_refcount_gt1_count": len(raw_core_refcount_gt1_cells),
            "raw_dynamic_without_owner_count": len(raw_dynamic_without_owner),
            "raw_dynamic_without_owner_bbox": _cells_bbox(raw_dynamic_without_owner),
            "raw_dynamic_without_owner_sample": sorted(raw_dynamic_without_owner)[:12],
            "raw_net_route_without_dynamic_count": len(raw_net_route_without_dynamic),
            "raw_net_route_without_dynamic_bbox": _cells_bbox(raw_net_route_without_dynamic),
            "raw_core_without_dynamic_count": len(raw_core_without_dynamic),
            "span_raw_dynamic_cells": len(raw_dynamic_span_cells),
            "span_raw_dynamic_without_owner_count": len(raw_dynamic_without_owner_span_cells),
            "span_raw_dynamic_without_owner_bbox": _cells_bbox(
                raw_dynamic_without_owner_span_cells
            ),
            "candidate_blocker_count": len(blocker_ids),
            "candidate_blocker_net_ids": blocker_ids,
            "candidate_blocker_route_indices": [
                self.route_jobs_by_id[net_id].route_index
                for net_id in blocker_ids
                if net_id in self.route_jobs_by_id
            ],
            "ripup_victim_count": len(victim_ids),
            "ripup_victim_net_ids": victim_ids,
            "ripup_victim_route_indices": [
                self.route_jobs_by_id[net_id].route_index
                for net_id in victim_ids
                if net_id in self.route_jobs_by_id
            ],
        }

    def _write_route_diagnostics(
        self,
        *,
        job: RouteJob,
        source_state: Any,
        target_state: Any,
        opened_candidate_cells: set[tuple[int, int]],
        dynamic_clearance_exempt_cells: set[tuple[int, int]],
        opened_cells_set: set[tuple[int, int]],
        diag_txt: Path | None,
        status: str,
        error_text: str | None = None,
        route_cells: set[tuple[int, int]] | None = None,
        route_obj: Any | None = None,
        repair_note: str | None = None,
    ) -> None:
        if diag_txt is None:
            return
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        committed_dynamic_cells = self._committed_dynamic_cells(exclude_net_id=job.net_id)
        if self.diagnostics_enabled:
            opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
            opened_candidate_static_overlap = self._cells_in_raw_static_geometry(
                opened_candidate_cells
            )
            opened_static_overlap = self._cells_in_raw_static_geometry(opened_cells_set)
            opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
            dynamic_exempt_dynamic_overlap = (
                dynamic_clearance_exempt_cells & committed_dynamic_cells
            )
        else:
            opened_candidate_dynamic_overlap = set()
            opened_candidate_static_overlap = set()
            opened_static_overlap = set()
            opened_dynamic_overlap = set()
            dynamic_exempt_dynamic_overlap = set()

        route_cells = route_cells or set()
        route_static_overlap = self._cells_in_raw_static_geometry(route_cells)
        route_overlap_with_candidate_opened_static = route_cells & opened_candidate_static_overlap
        route_overlap_with_effective_opened_static = route_cells & opened_static_overlap
        route_dynamic_overlap = route_cells & committed_dynamic_cells
        route_overlap_with_candidate_opened_dynamic = route_cells & opened_candidate_dynamic_overlap
        route_overlap_with_effective_opened_dynamic = route_cells & opened_dynamic_overlap
        route_overlap_with_dynamic_exempt = route_cells & dynamic_clearance_exempt_cells
        current_endpoint_foreign_keepout_cells = set(
            self._foreign_keepout_open_cells_for_spec(port1_spec)
        )
        current_endpoint_foreign_keepout_cells.update(
            self._foreign_keepout_open_cells_for_spec(port2_spec)
        )
        foreign_keepout_open_cells = current_endpoint_foreign_keepout_cells & opened_cells_set
        current_port_runway_cells = set(self.port_runway_cells_by_spec.get(port1_spec, set()))
        current_port_runway_cells.update(self.port_runway_cells_by_spec.get(port2_spec, set()))
        source_sibling_port_runway_cells: set[tuple[int, int]] = set()
        for cluster_port_spec in self.dense_source_cluster_specs_by_port_spec.get(
            port1_spec, set()
        ):
            if cluster_port_spec == port1_spec:
                continue
            source_sibling_port_runway_cells.update(
                self.port_runway_cells_by_spec.get(cluster_port_spec, set())
            )
        target_sibling_port_runway_cells: set[tuple[int, int]] = set()
        for cluster_port_spec in self.dense_source_cluster_specs_by_port_spec.get(
            port2_spec, set()
        ):
            if cluster_port_spec == port2_spec:
                continue
            target_sibling_port_runway_cells.update(
                self.port_runway_cells_by_spec.get(cluster_port_spec, set())
            )
        sibling_port_runway_cells = (
            source_sibling_port_runway_cells | target_sibling_port_runway_cells
        )
        current_port_runway_dynamic_overlap = current_port_runway_cells & committed_dynamic_cells
        sibling_port_runway_dynamic_overlap = sibling_port_runway_cells & committed_dynamic_cells
        route_overlap_current_port_runway = route_cells & current_port_runway_cells
        route_overlap_sibling_port_runway = route_cells & sibling_port_runway_cells
        route_segments: list[str] = []
        if route_obj is not None:
            for segment in cast(list[object], getattr(route_obj, "segments", []) or []):
                try:
                    entry = dict(cast(Any, segment))
                except (TypeError, ValueError):
                    continue
                route_segments.append(
                    "{kind}:{start}->{end}@{start_angle}->{end_angle}".format(
                        kind=entry.get("kind"),
                        start=entry.get("start"),
                        end=entry.get("end"),
                        start_angle=entry.get("start_angle"),
                        end_angle=entry.get("end_angle"),
                    )
                )

        def _relative_line_cells(
            *,
            start: tuple[int, int],
            direction: tuple[int, int],
            cells: int,
        ) -> list[tuple[int, int]]:
            return [
                (int(start[0]) + int(direction[0]) * step, int(start[1]) + int(direction[1]) * step)
                for step in range(max(0, int(cells)) + 1)
            ]

        def _unique_cells(
            cells: Iterable[tuple[int, int]],
        ) -> list[tuple[int, int]]:
            seen: set[tuple[int, int]] = set()
            unique: list[tuple[int, int]] = []
            for cell in cells:
                normalized = (int(cell[0]), int(cell[1]))
                if normalized in seen:
                    continue
                seen.add(normalized)
                unique.append(normalized)
            return unique

        def _first_move_footprint(
            *,
            source: tuple[int, int],
            source_angle: int,
            kind: str,
            cells: int = 0,
            delta: int = 0,
        ) -> tuple[tuple[int, int], int, list[tuple[int, int]]]:
            start_dir = self._angle_to_step(source_angle)
            if kind == "straight":
                relative = _relative_line_cells(
                    start=(0, 0),
                    direction=start_dir,
                    cells=cells,
                )
                end = relative[-1]
                end_angle = int(source_angle) % 8
            else:
                end_angle = (int(source_angle) + int(delta)) % 8
                end_dir = self._angle_to_step(end_angle)
                radius = max(0, int(self.bend_radius_cells))
                first_leg = _relative_line_cells(
                    start=(0, 0),
                    direction=start_dir,
                    cells=radius,
                )
                corner = (start_dir[0] * radius, start_dir[1] * radius)
                second_leg = _relative_line_cells(
                    start=corner,
                    direction=end_dir,
                    cells=radius,
                )
                relative = _unique_cells([*first_leg, *second_leg])
                end = relative[-1]
            absolute = [(int(source[0]) + int(dx), int(source[1]) + int(dy)) for dx, dy in relative]
            return (
                (int(source[0]) + int(end[0]), int(source[1]) + int(end[1])),
                end_angle,
                absolute,
            )

        def _dynamic_owners_for_cells(
            cells: set[tuple[int, int]],
        ) -> dict[int, list[tuple[int, int]]]:
            owners: dict[int, list[tuple[int, int]]] = {}
            for net_id in self.route_bookkeeping.records_by_id:
                if int(net_id) == int(job.net_id):
                    continue
                overlap = cells & self._route_cells_from_router(int(net_id))
                if overlap:
                    owners[int(net_id)] = sorted(overlap)
            return owners

        def _format_post_crossing_orthogonal_candidates() -> list[str]:
            if not self.diagnostics_enabled or not route_dynamic_overlap:
                return []
            if int(source_state.angle) != 0 or int(target_state.angle) != 0:
                return []
            if source_anchor_cell[1] == target_anchor_cell[1]:
                return []
            crossing_cells = sorted(route_dynamic_overlap)
            if len(crossing_cells) != 1:
                return []
            cross_x, cross_y = crossing_cells[0]
            dy_sign = 1 if target_anchor_cell[1] > cross_y else -1
            dx_sign = 1 if target_anchor_cell[0] > cross_x else -1
            if dx_sign != 1:
                return []
            radius = max(1, int(self.bend_radius_cells))
            routing_static_cells = set(self.static_blocked_cells_before_port_reservations)
            routing_static_cells.update(self.debug_port_keepout_cells)
            routing_static_cells.update(self.foreign_port_keepout_static_cells)
            routing_static_cells.update(self.fanout_stub_static_cells)
            opened_search_cells = set(opened_cells_set)
            allowed_dynamic_cells = set(route_dynamic_overlap)
            lines: list[str] = []
            min_start = cross_x + radius
            max_start = target_anchor_cell[0] - 2 * radius
            for bend_start_x in range(min_start, max_start + 1):
                if target_anchor_cell[1] - dy_sign * radius == cross_y:
                    continue
                cells: set[tuple[int, int]] = set()
                for x in range(source_anchor_cell[0], bend_start_x + radius + 1):
                    cells.add((x, cross_y))
                first_corner_x = bend_start_x + radius
                first_corner_y = cross_y
                first_end_y = cross_y + dy_sign * radius
                for step in range(0, radius + 1):
                    cells.add((first_corner_x, first_corner_y + dy_sign * step))
                vertical_start_y = first_end_y
                second_start_y = target_anchor_cell[1] - dy_sign * radius
                y0, y1 = sorted((vertical_start_y, second_start_y))
                for y in range(y0, y1 + 1):
                    cells.add((first_corner_x, y))
                second_end_x = first_corner_x + radius
                for step in range(0, radius + 1):
                    cells.add((first_corner_x + step, second_start_y + dy_sign * step))
                for x in range(second_end_x, target_anchor_cell[0] + 1):
                    cells.add((x, target_anchor_cell[1]))
                static_blockers = (cells & routing_static_cells) - opened_search_cells
                dynamic_blockers = (
                    (cells & committed_dynamic_cells)
                    - allowed_dynamic_cells
                    - (cells & opened_search_cells)
                )
                lines.append(
                    "post_crossing_90_candidate="
                    f"bend_start_x={bend_start_x}; "
                    f"cells={len(cells)}; "
                    f"static_blockers={sorted(static_blockers)[:24]}; "
                    f"static_blocker_count={len(static_blockers)}; "
                    f"dynamic_blockers={sorted(dynamic_blockers)[:24]}; "
                    f"dynamic_blocker_count={len(dynamic_blockers)}; "
                    f"dynamic_blocker_owners={_dynamic_owners_for_cells(dynamic_blockers)}"
                )
            return lines

        def _format_target_bend_footprints() -> list[str]:
            if not self.diagnostics_enabled:
                return []
            radius = max(1, int(self.bend_radius_cells))
            routing_static_cells = set(self.static_blocked_cells_before_port_reservations)
            routing_static_cells.update(self.debug_port_keepout_cells)
            routing_static_cells.update(self.foreign_port_keepout_static_cells)
            routing_static_cells.update(self.fanout_stub_static_cells)
            opened_search_cells = set(opened_cells_set)

            def turn_cells(
                start: tuple[int, int],
                start_angle: int,
                delta: int,
            ) -> set[tuple[int, int]]:
                start_dir = self._angle_to_step(start_angle)
                end_angle = (int(start_angle) + int(delta)) % 8
                end_dir = self._angle_to_step(end_angle)
                cells: set[tuple[int, int]] = set()
                for step in range(radius + 1):
                    cells.add(
                        (
                            int(start[0]) + start_dir[0] * step,
                            int(start[1]) + start_dir[1] * step,
                        )
                    )
                corner = (
                    int(start[0]) + start_dir[0] * radius,
                    int(start[1]) + start_dir[1] * radius,
                )
                for step in range(radius + 1):
                    cells.add(
                        (
                            corner[0] + end_dir[0] * step,
                            corner[1] + end_dir[1] * step,
                        )
                    )
                return cells

            target = target_anchor_cell
            specs = [
                (
                    "end_from_below_to_port",
                    (target[0] - radius, target[1] - radius),
                    2,
                    -2,
                ),
                (
                    "end_from_above_to_port",
                    (target[0] - radius, target[1] + radius),
                    6,
                    2,
                ),
                ("target_out_down", target, 0, -2),
                ("target_out_up", target, 0, 2),
            ]
            lines: list[str] = []
            for label, start, angle, delta in specs:
                cells = turn_cells(start, angle, delta)
                static_blockers = (cells & routing_static_cells) - opened_search_cells
                dynamic_blockers = (
                    (cells & committed_dynamic_cells)
                    - (cells & opened_search_cells)
                    - route_dynamic_overlap
                )
                lines.append(
                    "target_bend_footprint="
                    f"label={label}; start={start}; angle={angle}; delta={delta}; "
                    f"cells={sorted(cells)}; "
                    f"static_blockers={sorted(static_blockers)}; "
                    f"static_blocker_count={len(static_blockers)}; "
                    f"dynamic_blockers={sorted(dynamic_blockers)}; "
                    f"dynamic_blocker_count={len(dynamic_blockers)}; "
                    f"dynamic_blocker_owners={_dynamic_owners_for_cells(dynamic_blockers)}"
                )
            return lines

        def _format_first_move_debug() -> list[str]:
            if not self.diagnostics_enabled:
                return []
            source = source_anchor_cell
            source_angle = int(source_state.angle)
            source_key = source_anchor_cell
            target_key = target_anchor_cell
            opened_search_cells = {
                cell
                for cell in opened_cells_set
                if cell == source_key or cell == target_key or cell not in committed_dynamic_cells
            }
            routing_static_cells = set(self.static_blocked_cells_before_port_reservations)
            routing_static_cells.update(self.debug_port_keepout_cells)
            routing_static_cells.update(self.foreign_port_keepout_static_cells)
            routing_static_cells.update(self.fanout_stub_static_cells)
            primitive_specs: list[tuple[str, str, int, int]] = [
                ("straight_short", "straight", int(self.primitive_cfg.straight_short_cells), 0),
                ("straight_long", "straight", int(self.primitive_cfg.straight_long_cells), 0),
                ("turn45_left", "turn", 0, 1),
                ("turn45_right", "turn", 0, -1),
                ("turn90_left", "turn", 0, 2),
                ("turn90_right", "turn", 0, -2),
            ]
            debug_lines: list[str] = []
            for label, kind, cells, delta in primitive_specs:
                if not self.allow_45_degree_turns and abs(int(delta)) == 1:
                    continue
                end_cell, end_angle, footprint = _first_move_footprint(
                    source=source,
                    source_angle=source_angle,
                    kind=kind,
                    cells=cells,
                    delta=delta,
                )
                footprint_set = set(footprint)
                static_overlap = self._cells_in_raw_static_geometry(footprint_set)
                routing_static_overlap = footprint_set & routing_static_cells
                effective_static_blockers = routing_static_overlap - opened_search_cells
                dynamic_overlap = footprint_set & committed_dynamic_cells
                effective_dynamic_blockers = (
                    dynamic_overlap
                    - dynamic_clearance_exempt_cells
                    - (footprint_set & opened_search_cells)
                )
                owner_cells = _dynamic_owners_for_cells(dynamic_overlap)
                debug_lines.append(
                    "first_move_{label}="
                    "end=({end_x},{end_y},{end_angle}); "
                    "footprint={footprint}; "
                    "static={static}; "
                    "routing_static={routing_static}; "
                    "static_blockers={static_blockers}; "
                    "dynamic={dynamic}; "
                    "dynamic_blockers={dynamic_blockers}; "
                    "dynamic_owners={owners}; "
                    "opened={opened}; "
                    "opened_search={opened_search}; "
                    "dynamic_exempt={dynamic_exempt}".format(
                        label=label,
                        end_x=end_cell[0],
                        end_y=end_cell[1],
                        end_angle=end_angle,
                        footprint=footprint,
                        static=sorted(static_overlap),
                        routing_static=sorted(routing_static_overlap),
                        static_blockers=sorted(effective_static_blockers),
                        dynamic=sorted(dynamic_overlap),
                        dynamic_blockers=sorted(effective_dynamic_blockers),
                        owners=owner_cells,
                        opened=sorted(footprint_set & opened_cells_set),
                        opened_search=sorted(footprint_set & opened_search_cells),
                        dynamic_exempt=sorted(footprint_set & dynamic_clearance_exempt_cells),
                    )
                )
            return debug_lines

        lines = [
            f"net_name={job.net_name}",
            f"status={status}",
            f"source_spec={port1_spec}",
            f"target_spec={port2_spec}",
            f"source_component={_schematic_instance_component_name(self.schematic, job.inst1)}",
            f"target_component={_schematic_instance_component_name(self.schematic, job.inst2)}",
            f"source_access_rule={self.port_access_rule_by_spec.get(port1_spec)}",
            f"target_access_rule={self.port_access_rule_by_spec.get(port2_spec)}",
            f"foreign_port_keepout_cells={int(self.foreign_port_keepout_cells)}",
            f"fanout_access_mode={self.fanout_access_mode_normalized}",
            f"fanout_stub_bend_degrees={45 * int(self._env_fanout_stub_bend_steps())}",
            f"fanout_anchor_port_count={len(self.fanout_anchor_by_port_spec)}",
            f"fanout_stub_center_cell_count={len(self.fanout_stub_center_cells)}",
            f"fanout_stub_static_cell_count={len(self.fanout_stub_static_cells)}",
            f"source_fanout_anchor={f'{job.inst1},{job.port1}' in self.fanout_anchor_by_port_spec}",
            f"target_fanout_anchor={f'{job.inst2},{job.port2}' in self.fanout_anchor_by_port_spec}",
            "source_dense_port_runway_cells="
            f"{self.dense_source_port_runway_length_by_spec.get(port1_spec)}",
            "target_dense_port_runway_cells="
            f"{self.dense_target_port_runway_length_by_spec.get(port2_spec)}",
            "source_dense_source_cluster_size="
            f"{len(self.dense_source_cluster_specs_by_port_spec.get(port1_spec, set()))}",
            "target_dense_source_cluster_size="
            f"{len(self.dense_source_cluster_specs_by_port_spec.get(port2_spec, set()))}",
            f"foreign_port_keepout_static_count={len(self.foreign_port_keepout_static_cells)}",
            f"foreign_port_keepout_open_count={len(foreign_keepout_open_cells)}",
            f"source_state=({source_anchor_cell[0]}, {source_anchor_cell[1]}, {int(source_state.angle)})",
            f"target_state=({target_anchor_cell[0]}, {target_anchor_cell[1]}, {int(target_state.angle)})",
            f"opened_candidate_cells_count={len(opened_candidate_cells)}",
            f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
            f"opened_candidate_static_overlap_bbox={_cells_bbox(opened_candidate_static_overlap)}",
            f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
            f"opened_candidate_dynamic_overlap_bbox={_cells_bbox(opened_candidate_dynamic_overlap)}",
            f"opened_cells_count={len(opened_cells_set)}",
            f"opened_cells={sorted(opened_cells_set)}",
            f"opened_static_overlap_count={len(opened_static_overlap)}",
            f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
            f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
            f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
            f"opened_dynamic_overlap_owners={_dynamic_owners_for_cells(opened_dynamic_overlap)}",
            f"current_port_runway_dynamic_overlap_count={len(current_port_runway_dynamic_overlap)}",
            "current_port_runway_dynamic_overlap_bbox="
            f"{_cells_bbox(current_port_runway_dynamic_overlap)}",
            f"sibling_port_runway_dynamic_overlap_count={len(sibling_port_runway_dynamic_overlap)}",
            "sibling_port_runway_dynamic_overlap_bbox="
            f"{_cells_bbox(sibling_port_runway_dynamic_overlap)}",
            f"dynamic_clearance_exempt_cells_count={len(dynamic_clearance_exempt_cells)}",
            f"dynamic_clearance_exempt_cells_bbox={_cells_bbox(dynamic_clearance_exempt_cells)}",
            f"dynamic_clearance_exempt_dynamic_overlap_count={len(dynamic_exempt_dynamic_overlap)}",
            f"dynamic_clearance_exempt_dynamic_overlap_bbox={_cells_bbox(dynamic_exempt_dynamic_overlap)}",
            f"route_cells_count={len(route_cells)}",
            f"route_static_blocked_overlap_count={len(route_static_overlap)}",
            f"route_static_blocked_overlap_bbox={_cells_bbox(route_static_overlap)}",
            f"route_dynamic_overlap_count={len(route_dynamic_overlap)}",
            f"route_dynamic_overlap_bbox={_cells_bbox(route_dynamic_overlap)}",
            f"route_dynamic_overlap_owners={_dynamic_owners_for_cells(route_dynamic_overlap)}",
            f"route_overlap_current_port_runway_count={len(route_overlap_current_port_runway)}",
            "route_overlap_current_port_runway_bbox="
            f"{_cells_bbox(route_overlap_current_port_runway)}",
            f"route_overlap_sibling_port_runway_count={len(route_overlap_sibling_port_runway)}",
            "route_overlap_sibling_port_runway_bbox="
            f"{_cells_bbox(route_overlap_sibling_port_runway)}",
            f"route_overlap_candidate_opened_static_count={len(route_overlap_with_candidate_opened_static)}",
            f"route_overlap_effective_opened_static_count={len(route_overlap_with_effective_opened_static)}",
            f"route_overlap_candidate_opened_dynamic_count={len(route_overlap_with_candidate_opened_dynamic)}",
            f"route_overlap_effective_opened_dynamic_count={len(route_overlap_with_effective_opened_dynamic)}",
            f"route_overlap_dynamic_clearance_exempt_count={len(route_overlap_with_dynamic_exempt)}",
        ]
        lines.extend(_format_post_crossing_orthogonal_candidates())
        lines.extend(_format_target_bend_footprints())
        lines.extend(_format_first_move_debug())
        if route_segments:
            lines.append("route_segments=" + "; ".join(route_segments))
        if repair_note is not None:
            lines.append(f"repair={repair_note}")
        if error_text is not None:
            lines.append(f"error={error_text}")
        diag_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _record_route(
        self,
        job: RouteJob,
        route_obj: Any,
        opened_cells: list[tuple[int, int]],
        *,
        corrected_centerline_um: tuple[tuple[float, float], ...] = (),
        corrected_total_length_um: float | None = None,
    ) -> None:
        if not corrected_centerline_um:
            corrected_centerline_um = self._fanout_stubbed_centerline(job, route_obj)
        if (
            not corrected_centerline_um
            and not self.enable_checked_endpoint_correction
            and hasattr(self.router, "route_primitive_centerline")
        ):
            try:
                corrected_centerline_um = _centerline_tuple(
                    self.router.route_primitive_centerline(route_obj)
                )
            except Exception:
                corrected_centerline_um = ()
            if corrected_centerline_um and hasattr(self.router, "centerline_length_um"):
                try:
                    corrected_total_length_um = float(
                        self.router.centerline_length_um(list(corrected_centerline_um))
                    )
                except Exception:
                    corrected_total_length_um = None
        target_anchor = self.fanout_anchor_by_port_spec.get(f"{job.inst2},{job.port2}")
        target_port_center_um_override = (
            target_anchor.center_um if target_anchor is not None else None
        )
        self.route_bookkeeping.record_route(
            job,
            route_obj,
            opened_cells,
            route_cells=self._route_cells_from_router(job.net_id)
            if self.track_dynamic_cells
            else None,
            corrected_centerline_um=corrected_centerline_um,
            corrected_total_length_um=corrected_total_length_um,
            target_port_center_um_override=target_port_center_um_override,
        )

    def _export_route_svg(
        self,
        job: RouteJob,
        route_obj: Any,
        *,
        suffix: str = "",
        obstacle_cells: set[tuple[int, int]] | None = None,
        opened_cells: list[tuple[int, int]] | None = None,
    ) -> None:
        should_export = self.debug_path is not None and (
            self.debug_route_indices is None or job.route_index in self.debug_route_indices
        )
        if not should_export:
            return
        route_dir = self.debug_path / "routes"
        _ensure_dir(route_dir)
        route_svg = route_dir / f"{self.debug_prefix}_{job.net_name}{suffix}.svg"
        if obstacle_cells is not None and hasattr(
            self.router, "export_debug_svg_with_obstacle_cells"
        ):
            svg_text = self.router.export_debug_svg_with_obstacle_cells(
                route_obj,
                sorted(obstacle_cells),
            )
        else:
            svg_text = self.router.export_debug_svg(route_obj)
        if opened_cells is None:
            try:
                _, _, _, _, opened_cells = self._state_openings_for_job(job)
            except Exception:
                opened_cells = []
        if self.debug_stop_after_route_index is not None and int(job.route_index) == int(
            self.debug_stop_after_route_index
        ):
            next_job = self.full_route_jobs_by_route_index.get(
                int(self.debug_stop_after_route_index) + 1
            )
            if next_job is not None:
                try:
                    _, _, _, _, opened_cells = self._states_and_openings(next_job)
                except Exception:
                    pass
        red_keepout_cells = self.debug_port_keepout_cells - {
            (int(cell[0]), int(cell[1])) for cell in opened_cells
        }
        if red_keepout_cells:
            overlay = ['<g id="port-keepout-cells">']
            for gx, gy in sorted(red_keepout_cells):
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    svg_y = self.grid_height - gy - 1
                    overlay.append(
                        f'<rect class="port-keepout" x="{gx}" y="{svg_y}" '
                        'width="1" height="1" fill="#d93025" opacity="0.38" />'
                    )
            overlay.append("</g>")
            overlay_text = "".join(overlay)
            if "</svg>" in svg_text and 'id="port-keepout-cells"' not in svg_text:
                svg_text = svg_text.replace("</svg>", overlay_text + "</svg>", 1)
        route_svg.write_text(svg_text, encoding="utf-8")
        self.route_svgs.append(route_svg)

    def _route_engine_summary(self, route_obj: Any) -> str:
        expanded_states = int(getattr(route_obj, "expanded_states", 0))
        route_kind = "simple" if expanded_states == 0 else "astar"
        length_um = _as_float(getattr(route_obj, "total_length_um", 0.0), 0.0)
        total_cost = _as_float(getattr(route_obj, "total_cost", 0.0), 0.0)
        return (
            f"{route_kind} "
            f"length={length_um:.3f}um "
            f"cost={total_cost:.3f} "
            f"expanded={expanded_states}"
        )

    def _corridor_clearance_diagnostic(
        self,
        source_state: Any,
        target_state: Any,
        blocked_cells: set[tuple[int, int]],
        *,
        max_radius: int | None = None,
    ) -> dict[str, Any]:
        if max_radius is None:
            max_radius = max(4, int(self.bend_radius_cells) + 2)
        else:
            max_radius = int(max_radius)

        source = (int(source_state.x), int(source_state.y))
        target = (int(target_state.x), int(target_state.y))
        neighbors = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        def inflate(radius: int) -> set[tuple[int, int]]:
            if radius <= 0:
                inflated = set(blocked_cells)
            else:
                inflated = set()
                for x, y in blocked_cells:
                    for dx in range(-radius, radius + 1):
                        for dy in range(-radius, radius + 1):
                            inflated.add((x + dx, y + dy))
            inflated.discard(source)
            inflated.discard(target)
            return inflated

        def reachable_from(
            start: tuple[int, int],
            blocked: set[tuple[int, int]],
        ) -> set[tuple[int, int]]:
            sx, sy = start
            if not (0 <= sx < self.grid_width and 0 <= sy < self.grid_height):
                return set()
            if start in blocked:
                return set()
            reached = {start}
            queue: deque[tuple[int, int]] = deque([start])
            while queue:
                x, y = queue.popleft()
                for dx, dy in neighbors:
                    nx = x + dx
                    ny = y + dy
                    neighbor = (nx, ny)
                    if (
                        0 <= nx < self.grid_width
                        and 0 <= ny < self.grid_height
                        and neighbor not in blocked
                        and neighbor not in reached
                    ):
                        reached.add(neighbor)
                        queue.append(neighbor)
            return reached

        last_connected_radius: int | None = None
        first_disconnected_radius: int | None = None
        disconnected_blocked: set[tuple[int, int]] | None = None
        for radius in range(max_radius + 1):
            inflated = inflate(radius)
            reachable = reachable_from(source, inflated)
            if target in reachable:
                last_connected_radius = radius
                continue
            first_disconnected_radius = radius
            disconnected_blocked = inflated
            break

        source_region_size: int | None = None
        target_region_size: int | None = None
        source_region_min_distance_to_target: int | None = None
        target_region_min_distance_to_source: int | None = None
        if first_disconnected_radius is not None and disconnected_blocked is not None:
            source_region = reachable_from(source, disconnected_blocked)
            target_region = reachable_from(target, disconnected_blocked)
            source_region_size = len(source_region)
            target_region_size = len(target_region)
            if target in source_region:
                source_region_min_distance_to_target = 0
            elif source_region:
                source_region_min_distance_to_target = min(
                    abs(x - target[0]) + abs(y - target[1]) for x, y in source_region
                )
            if source in target_region:
                target_region_min_distance_to_source = 0
            elif target_region:
                target_region_min_distance_to_source = min(
                    abs(x - source[0]) + abs(y - source[1]) for x, y in target_region
                )

        return {
            "max_radius_checked": max_radius,
            "last_connected_radius": last_connected_radius,
            "first_disconnected_radius": first_disconnected_radius,
            "source_region_size": source_region_size,
            "target_region_size": target_region_size,
            "source_region_min_distance_to_target": source_region_min_distance_to_target,
            "target_region_min_distance_to_source": target_region_min_distance_to_source,
        }

    def _write_failed_log(
        self,
        job: RouteJob,
        source_state: Any,
        target_state: Any,
        opened_candidate_cells: set[tuple[int, int]],
        opened_cells: list[tuple[int, int]],
        error_text: str,
    ) -> None:
        if self.debug_path is None:
            return
        route_dir = self.debug_path / "routes"
        _ensure_dir(route_dir)
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        fail_txt = route_dir / f"{self.debug_prefix}_{job.net_name}_FAILED.txt"
        committed_dynamic_cells = self._committed_dynamic_cells()
        opened_candidate_static_overlap = (
            opened_candidate_cells & self.static_blocked_cells_before_port_reservations
        )
        opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
        opened_cells_set = set(opened_cells)
        corridor_diagnostic = self._corridor_clearance_diagnostic(
            source_state,
            target_state,
            (self.static_blocked_cells_before_port_reservations - opened_cells_set)
            | committed_dynamic_cells,
        )
        opened_static_overlap = (
            opened_cells_set & self.static_blocked_cells_before_port_reservations
        )
        opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
        current_attempts = [
            ("current", record)
            for record in self.route_attempt_records
            if getattr(record, "net_id", None) == job.net_id
        ]
        recent_attempts = [("recent", record) for record in self.route_attempt_records[-12:]]
        root_cause_line = _format_illegal_crossing_root_causes_line(
            [error_text]
            + [
                str(attempt.error)
                for _, attempt in (current_attempts[-8:] + recent_attempts)
                if getattr(attempt, "error", None)
            ]
        )
        fail_lines = [
            f"net_name={job.net_name}",
            f"source_spec={port1_spec}",
            f"target_spec={port2_spec}",
            f"source_state=({int(source_state.x)}, {int(source_state.y)}, {int(source_state.angle)})",
            f"target_state=({int(target_state.x)}, {int(target_state.y)}, {int(target_state.angle)})",
            f"allow_45_degree_turns={self.allow_45_degree_turns}",
            f"block_radius_cells={self.block_radius_cells}",
            "dynamic_obstacle_search_expansion_radius_cells="
            f"{self.clearance_policy.dynamic_obstacle_search_expansion_radius_cells}",
            "dynamic_route_commit_keepout_radius_cells="
            f"{self.clearance_policy.dynamic_route_commit_keepout_radius_cells}",
            "dynamic_route_core_radius_cells="
            f"{self.clearance_policy.dynamic_route_core_radius_cells}",
            f"bend_radius_cells={self.bend_radius_cells}",
            f"port_lane_length_cells={self.port_lane_length_cells}",
            f"port_lane_half_width_cells={self.port_lane_half_width_cells}",
            f"opened_candidate_cells_count={len(opened_candidate_cells)}",
            f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
            f"opened_candidate_static_overlap_bbox={_cells_bbox(opened_candidate_static_overlap)}",
            f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
            f"opened_candidate_dynamic_overlap_bbox={_cells_bbox(opened_candidate_dynamic_overlap)}",
            f"opened_cells_count={len(opened_cells)}",
            f"opened_static_overlap_count={len(opened_static_overlap)}",
            f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
            f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
            f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
            f"corridor_clearance_max_radius_checked={corridor_diagnostic['max_radius_checked']}",
            f"corridor_clearance_last_connected_radius={corridor_diagnostic['last_connected_radius']}",
            f"corridor_clearance_first_disconnected_radius={corridor_diagnostic['first_disconnected_radius']}",
            f"corridor_clearance_source_region_size={corridor_diagnostic['source_region_size']}",
            f"corridor_clearance_target_region_size={corridor_diagnostic['target_region_size']}",
            "corridor_clearance_source_region_min_distance_to_target="
            f"{corridor_diagnostic['source_region_min_distance_to_target']}",
            "corridor_clearance_target_region_min_distance_to_source="
            f"{corridor_diagnostic['target_region_min_distance_to_source']}",
            f"error={error_text}",
        ]
        if root_cause_line is not None:
            fail_lines.append(root_cause_line)
        fail_lines.extend(_format_native_repair_trace_lines(self.native_repair_trace_records))
        for label, attempt in current_attempts[-8:] + recent_attempts:
            as_dict = attempt.as_dict()
            diagnostics = as_dict.get("diagnostics")
            fail_lines.append(
                f"attempt_{label}="
                + ", ".join(
                    f"{key}={as_dict.get(key)}"
                    for key in (
                        "attempt_index",
                        "bucket_name",
                        "failed",
                        "error",
                        "elapsed_s",
                        "expanded_states",
                        "generated_neighbors",
                        "window_attempts",
                        "used_full_grid_fallback",
                        "candidate_blocker_count",
                        "candidate_blocker_route_indices",
                        "ripup_victim_count",
                        "ripup_victim_route_indices",
                    )
                    if key in as_dict
                )
            )
            if isinstance(diagnostics, dict) and diagnostics:
                fail_lines.append(
                    f"attempt_{label}_diagnostics="
                    + ", ".join(
                        f"{key}={diagnostics.get(key)}"
                        for key in (
                            "candidate_blocker_count",
                            "candidate_blocker_route_indices",
                            "ripup_victim_count",
                            "ripup_victim_route_indices",
                            "route_bbox_min_x",
                            "route_bbox_max_x",
                            "route_bbox_min_y",
                            "route_bbox_max_y",
                        )
                        if key in diagnostics
                    )
                )
        fail_txt.write_text("\n".join(fail_lines) + "\n", encoding="utf-8")

    def _finalize_committed_route(
        self,
        job: RouteJob,
        route_obj: Any,
        opened_cells: list[tuple[int, int]],
        *,
        should_print_route: bool,
        diag_txt: Path | None,
        debug_obstacle_cells: set[tuple[int, int]] | None = None,
    ) -> None:
        expanded_states = int(getattr(route_obj, "expanded_states", 0))
        self.total_expanded_states += expanded_states
        if expanded_states == 0:
            self.simple_route_count += 1

        if self.diagnostics_enabled:
            source_state, target_state, opened_candidate_cells, opened_cells_set, _ = (
                self._state_openings_for_job(job)
            )
            route_cells = {
                (int(cell[0]), int(cell[1])) for cell in (getattr(route_obj, "cells", None) or [])
            }
            self._write_route_diagnostics(
                job=job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=self._clearance_exempt_cell_set_for_job(job),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="ok",
                route_cells=route_cells,
                route_obj=route_obj,
            )

        self._export_route_svg(
            job,
            route_obj,
            obstacle_cells=debug_obstacle_cells,
            opened_cells=opened_cells,
        )

        if should_print_route:
            print(f"ok {self._route_engine_summary(route_obj)}")

    def _dispatch_native_routing(self, route_jobs: list[RouteJob]) -> None:
        if self.repair_config.enabled:
            if not hasattr(self.router, "route_many_with_repair_and_commit"):
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.route_many_with_repair_and_commit. Rebuild it with "
                    "`maturin develop --release`; Python repair fallback has been removed."
                )
            batch_jobs: list[
                tuple[
                    int,
                    Any,
                    Any,
                    list[tuple[int, int]],
                    list[tuple[int, int]],
                    list[tuple[int, int]],
                    tuple[float, float] | None,
                    tuple[float, float] | None,
                ]
            ] = []
            batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
            batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
            t_batch_job_pack_start = self._pipeline_timer_start()
            for job in route_jobs:
                source_state, target_state, _, _, opened_cells = self._state_openings_for_job(job)
                clearance_exempt_cells = self._clearance_exempt_cells_for_job(job)
                route_selected_for_debug = (
                    self.debug_route_indices is None or job.route_index in self.debug_route_indices
                )
                should_print_route = self.verbose_route_diagnostics and route_selected_for_debug
                if self.debug_route_indices is not None and route_selected_for_debug:
                    should_print_route = True
                if should_print_route:
                    print(
                        f"  Routing [{job.route_index}/{len(route_jobs)}] "
                        f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                        end=" ",
                    )
                route_dir = self.debug_path / "routes" if self.debug_path is not None else None
                diag_txt: Path | None = None
                if (
                    self.debug_path is not None
                    and (route_selected_for_debug or self.collect_attempt_diagnostics)
                    and route_dir is not None
                ):
                    _ensure_dir(route_dir)
                    diag_txt = route_dir / f"{self.debug_prefix}_{job.net_name}_diagnostics.txt"
                batch_jobs.append(
                    (
                        int(job.net_id),
                        source_state,
                        target_state,
                        opened_cells,
                        clearance_exempt_cells,
                        self._foreign_keepout_cleanup_cells_for_job(job),
                        self._routing_endpoint_center_um(job, source=True),
                        self._routing_endpoint_center_um(job, source=False),
                    )
                )
                batch_opened_cells_by_id[int(job.net_id)] = opened_cells
                batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
            self._record_pipeline_timing("batch_job_pack", t_batch_job_pack_start)

            batch_start = self._timing_start()
            if os.environ.get("PHOTONIC_ROUTER_NEGOTIATED_REPAIR", "") == "1":
                # A/B comparison path for
                # .agent/execplans/2026-08-25-negotiated-repair-engine.md
                # Milestone 5's new negotiated-congestion loop, kept
                # opt-in behind this env var specifically so the
                # existing, validated `route_many_with_repair_and_commit`
                # stays the default until the new loop's own coverage
                # (crossing-specific repair strategies, dense-source-
                # fanout static cleanup) closes the gap documented in
                # that milestone's Surprises & Discoveries.
                if not hasattr(self.router, "route_many_with_negotiated_repair_and_commit"):
                    raise RuntimeError(
                        "The loaded photonic_router._rust extension does not expose "
                        "PyPhotonicRouter.route_many_with_negotiated_repair_and_commit. "
                        "Rebuild it with `maturin develop --release`."
                    )
                raw_batch_result = self.router.route_many_with_negotiated_repair_and_commit(
                    batch_jobs,
                    self.block_radius_cells,
                    self.commit_radius_cells,
                    self.core_commit_radius_cells,
                    int(self.repair_config.max_rounds),
                    float(self.repair_config.history_weight),
                    int(self.repair_config.history_increment),
                )
            else:
                raw_batch_result = self.router.route_many_with_repair_and_commit(
                    batch_jobs,
                    self.block_radius_cells,
                    self.commit_radius_cells,
                    self.core_commit_radius_cells,
                    int(self.repair_config.max_rounds),
                    int(self.repair_config.max_victims_per_failure),
                    float(self.repair_config.history_weight),
                    int(self.repair_config.history_increment),
                )
            batch_elapsed_s = time.perf_counter() - batch_start if self.collect_timing else 0.0
            self._record_pipeline_timing("native_route_batch", batch_start)
            t_batch_result_processing_start = self._pipeline_timer_start()
            batch_result = dict(raw_batch_result)
            self.native_repair_trace_records = [
                dict(record)
                for record in cast(
                    Iterable[Mapping[str, object]],
                    batch_result.get("repair_trace", []),
                )
            ]
            self._record_native_batch_timings(batch_result)
            self._report_long_straight_congestion(batch_result)
            raw_attempts = list(cast(Iterable[Any], batch_result.get("attempts", [])))
            per_attempt_elapsed_s = batch_elapsed_s / max(1, len(raw_attempts))
            for raw_attempt in raw_attempts:
                attempt = dict(raw_attempt)
                net_id = int(attempt["net_id"])
                job = self.route_jobs_by_id[net_id]
                route_obj = attempt.get("route")
                if route_obj is None:
                    route_obj = None
                failed = bool(attempt.get("failed", False))
                bucket_name = str(attempt.get("bucket_name", "normal_route"))
                error_text = str(attempt.get("error")) if attempt.get("error") is not None else None
                repair_round_raw = attempt.get("repair_round")
                repair_round = int(repair_round_raw) if repair_round_raw is not None else None
                attempt_index = len(self.route_attempt_records) + 1
                candidate_blockers = [
                    int(value)
                    for value in cast(list[object], attempt.get("candidate_blockers", []))
                ]
                ripup_ids = [
                    int(value) for value in cast(list[object], attempt.get("ripup_ids", []))
                ]
                if (
                    route_obj is not None
                    and not failed
                    and bucket_name != "normal_route"
                    and self.debug_path is not None
                    and (
                        self.debug_route_indices is None
                        or job.route_index in self.debug_route_indices
                    )
                ):
                    self._export_route_svg(
                        job,
                        route_obj,
                        suffix=f"_attempt{attempt_index}_{bucket_name}",
                    )
                if self.collect_timing:
                    bucket = self.route_timing_buckets.setdefault(
                        bucket_name,
                        RouteTimingBucket(),
                    )
                    if route_obj is not None and not failed:
                        bucket.record_route(
                            per_attempt_elapsed_s,
                            route_obj,
                        )
                    else:
                        bucket.record_elapsed(
                            per_attempt_elapsed_s,
                            failed=failed,
                        )
                    self.route_attempt_records.append(
                        route_attempt_record_from_route(
                            attempt_index=attempt_index,
                            bucket_name=bucket_name,
                            net_id=job.net_id,
                            route_index=job.route_index,
                            net_name=job.net_name,
                            source=f"{job.inst1},{job.port1}",
                            target=f"{job.inst2},{job.port2}",
                            elapsed_s=per_attempt_elapsed_s,
                            route_obj=route_obj if route_obj is not None and not failed else None,
                            failed=failed,
                            repair_round=repair_round,
                            error=error_text,
                            diagnostics=self._route_attempt_diagnostics(
                                job,
                                route_obj if route_obj is not None and not failed else None,
                                candidate_blockers=candidate_blockers,
                                ripup_ids=ripup_ids,
                            )
                            if self.collect_attempt_diagnostics
                            else None,
                        )
                    )

            self.repair_count += int(batch_result.get("repair_count", 0) or 0)
            raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
            for raw_entry in raw_routes:
                entry = dict(raw_entry)
                net_id = int(entry["net_id"])
                route_obj = entry["route"]
                job = self.route_jobs_by_id[net_id]
                opened_cells = batch_opened_cells_by_id[net_id]
                self._record_route(job, route_obj, opened_cells)
                should_print_route, diag_txt = batch_debug_by_id[net_id]
                self._finalize_committed_route(
                    job,
                    route_obj,
                    opened_cells,
                    should_print_route=should_print_route,
                    diag_txt=diag_txt,
                )
            self._record_pipeline_timing(
                "batch_result_processing",
                t_batch_result_processing_start,
            )

            if str(batch_result.get("status", "")) != "routed":
                failed_net_id = int(batch_result.get("failed_net_id", -1))
                failed_job = self.route_jobs_by_id.get(failed_net_id)
                error_text = str(batch_result.get("error", "No route found"))
                if failed_job is None:
                    raise RuntimeError(error_text)
                (
                    source_state,
                    target_state,
                    opened_candidate_cells,
                    opened_cells_set,
                    opened_cells,
                ) = self._state_openings_for_job(failed_job)
                should_print_route, diag_txt = batch_debug_by_id.get(
                    failed_net_id,
                    (False, None),
                )
                if not should_print_route:
                    print(
                        f"  Routing [{failed_job.route_index}/{len(route_jobs)}] "
                        f"{failed_job.net_name}: {failed_job.inst1},{failed_job.port1} -> "
                        f"{failed_job.inst2},{failed_job.port2}... failed"
                    )
                self._write_route_diagnostics(
                    job=failed_job,
                    source_state=source_state,
                    target_state=target_state,
                    opened_candidate_cells=opened_candidate_cells,
                    dynamic_clearance_exempt_cells=self._clearance_exempt_cell_set_for_job(
                        failed_job
                    ),
                    opened_cells_set=opened_cells_set,
                    diag_txt=diag_txt,
                    status="failed",
                    error_text=error_text,
                )
                self._write_failed_log(
                    failed_job,
                    source_state,
                    target_state,
                    opened_candidate_cells,
                    opened_cells,
                    error_text,
                )
                raise RuntimeError(
                    f"No route found for {failed_job.net_name}: "
                    f"{failed_job.inst1},{failed_job.port1} -> {failed_job.inst2},{failed_job.port2}. "
                    f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                    f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                    f"allow_45_degree_turns={self.allow_45_degree_turns}. "
                    f"error={error_text}"
                )

        else:
            if not hasattr(self.router, "route_many_normal_and_commit"):
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.route_many_normal_and_commit. Rebuild it with "
                    "`maturin develop --release`; Python sequential routing fallback has been removed."
                )
            batch_jobs: list[
                tuple[
                    int,
                    Any,
                    Any,
                    list[tuple[int, int]],
                    list[tuple[int, int]],
                    list[tuple[int, int]],
                    tuple[float, float] | None,
                    tuple[float, float] | None,
                ]
            ] = []
            batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
            batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
            t_batch_job_pack_start = self._pipeline_timer_start()
            for job in route_jobs:
                source_state, target_state, _, _, opened_cells = self._state_openings_for_job(job)
                clearance_exempt_cells = self._clearance_exempt_cells_for_job(job)
                route_selected_for_debug = (
                    self.debug_route_indices is None or job.route_index in self.debug_route_indices
                )
                should_print_route = self.verbose_route_diagnostics and route_selected_for_debug
                if self.debug_route_indices is not None and route_selected_for_debug:
                    should_print_route = True
                if should_print_route:
                    print(
                        f"  Routing [{job.route_index}/{len(route_jobs)}] "
                        f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                        end=" ",
                    )
                route_dir = self.debug_path / "routes" if self.debug_path is not None else None
                diag_txt: Path | None = None
                if (
                    self.debug_path is not None
                    and (route_selected_for_debug or self.collect_attempt_diagnostics)
                    and route_dir is not None
                ):
                    _ensure_dir(route_dir)
                    diag_txt = route_dir / f"{self.debug_prefix}_{job.net_name}_diagnostics.txt"
                batch_jobs.append(
                    (
                        int(job.net_id),
                        source_state,
                        target_state,
                        opened_cells,
                        clearance_exempt_cells,
                        self._foreign_keepout_cleanup_cells_for_job(job),
                        self._routing_endpoint_center_um(job, source=True),
                        self._routing_endpoint_center_um(job, source=False),
                    )
                )
                batch_opened_cells_by_id[int(job.net_id)] = opened_cells
                batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
            self._record_pipeline_timing("batch_job_pack", t_batch_job_pack_start)

            batch_start = self._timing_start()
            raw_batch_result = self.router.route_many_normal_and_commit(
                batch_jobs,
                self.block_radius_cells,
                self.commit_radius_cells,
                self.core_commit_radius_cells,
            )
            batch_elapsed_s = time.perf_counter() - batch_start if self.collect_timing else 0.0
            self._record_pipeline_timing("native_route_batch", batch_start)
            t_batch_result_processing_start = self._pipeline_timer_start()
            batch_result = dict(raw_batch_result)
            self._record_native_batch_timings(batch_result)
            self._report_long_straight_congestion(batch_result)
            raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
            per_route_elapsed_s = batch_elapsed_s / max(1, len(raw_routes))
            for raw_entry in raw_routes:
                entry = dict(raw_entry)
                net_id = int(entry["net_id"])
                route_obj = entry["route"]
                job = self.route_jobs_by_id[net_id]
                opened_cells = batch_opened_cells_by_id[net_id]
                if self.collect_timing:
                    self.route_timing_buckets["normal_route"].record_route(
                        per_route_elapsed_s,
                        route_obj,
                    )
                    self.route_attempt_records.append(
                        route_attempt_record_from_route(
                            attempt_index=len(self.route_attempt_records) + 1,
                            bucket_name="normal_route",
                            net_id=job.net_id,
                            route_index=job.route_index,
                            net_name=job.net_name,
                            source=f"{job.inst1},{job.port1}",
                            target=f"{job.inst2},{job.port2}",
                            elapsed_s=per_route_elapsed_s,
                            route_obj=route_obj,
                        )
                    )
                self._record_route(job, route_obj, opened_cells)
                should_print_route, diag_txt = batch_debug_by_id[net_id]
                self._finalize_committed_route(
                    job,
                    route_obj,
                    opened_cells,
                    should_print_route=should_print_route,
                    diag_txt=diag_txt,
                )
            self._record_pipeline_timing(
                "batch_result_processing",
                t_batch_result_processing_start,
            )

            if str(batch_result.get("status", "")) != "routed":
                failed_net_id = int(batch_result.get("failed_net_id", -1))
                failed_job = self.route_jobs_by_id.get(failed_net_id)
                error_text = str(batch_result.get("error", "No route found"))
                if failed_job is None:
                    raise RuntimeError(error_text)
                (
                    source_state,
                    target_state,
                    opened_candidate_cells,
                    opened_cells_set,
                    opened_cells,
                ) = self._state_openings_for_job(failed_job)
                should_print_route, diag_txt = batch_debug_by_id.get(
                    failed_net_id,
                    (False, None),
                )
                if self.collect_timing:
                    self.route_timing_buckets["normal_route"].record_elapsed(0.0, failed=True)
                    self.route_attempt_records.append(
                        route_attempt_record_from_route(
                            attempt_index=len(self.route_attempt_records) + 1,
                            bucket_name="normal_route",
                            net_id=failed_job.net_id,
                            route_index=failed_job.route_index,
                            net_name=failed_job.net_name,
                            source=f"{failed_job.inst1},{failed_job.port1}",
                            target=f"{failed_job.inst2},{failed_job.port2}",
                            elapsed_s=0.0,
                            route_obj=None,
                            failed=True,
                            error=error_text,
                        )
                    )
                if not should_print_route:
                    print(
                        f"  Routing [{failed_job.route_index}/{len(route_jobs)}] "
                        f"{failed_job.net_name}: {failed_job.inst1},{failed_job.port1} -> "
                        f"{failed_job.inst2},{failed_job.port2}... failed"
                    )
                self._write_route_diagnostics(
                    job=failed_job,
                    source_state=source_state,
                    target_state=target_state,
                    opened_candidate_cells=opened_candidate_cells,
                    dynamic_clearance_exempt_cells=self._clearance_exempt_cell_set_for_job(
                        failed_job
                    ),
                    opened_cells_set=opened_cells_set,
                    diag_txt=diag_txt,
                    status="failed",
                    error_text=error_text,
                )
                self._write_failed_log(
                    failed_job,
                    source_state,
                    target_state,
                    opened_candidate_cells,
                    opened_cells,
                    error_text,
                )
                raise RuntimeError(
                    f"No route found for {failed_job.net_name}: "
                    f"{failed_job.inst1},{failed_job.port1} -> {failed_job.inst2},{failed_job.port2}. "
                    f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                    f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                    f"allow_45_degree_turns={self.allow_45_degree_turns}"
                )

    def _endpoint_correction_crossing_net_ids(self) -> set[int]:
        """Net ids involved in any crossing, per `router.crossing_events()`.

        Shared by `_apply_checked_endpoint_corrections_for_net_ids` and
        `_apply_checked_fanout_stub_endpoint_corrections_for_net_ids`,
        which previously each re-derived this identical set inline
        (Milestone 1 of
        .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md).
        Not used by `_apply_crossing_aware_endpoint_corrections_for_net_ids`,
        which uses a separate, independently-derived notion of "has a
        crossing" (`_current_crossing_points_by_net_id()`) that carries
        actual crossing points, not just event membership; unifying the
        two is out of this milestone's additive scope.
        """
        crossing_net_ids: set[int] = set()
        if self.enable_crossings and hasattr(self.router, "crossing_events"):
            # No broad `except Exception` here: Milestone 0 of the
            # restructuring ExecPlan confirmed via git history that this
            # used to swallow any error from `router.crossing_events()`
            # (introduced verbatim by commit 9925249, never touched since,
            # no test/log/diagnostic anywhere shows it ever legitimately
            # firing). A real failure here means crossing detection is
            # broken, which should stop the run loudly, not be silently
            # treated as "no crossings" and let every net fall through to
            # the unrestricted corrector as if crossings were disabled.
            for raw_event in cast(Iterable[Any], self.router.crossing_events()):
                if not isinstance(raw_event, Mapping):
                    try:
                        raw_event = dict(cast(Any, raw_event))
                    except (TypeError, ValueError):
                        continue
                for key in ("net_id", "partner_net_id"):
                    try:
                        crossing_net_ids.add(int(cast(Any, raw_event.get(key))))
                    except (TypeError, ValueError):
                        continue
        return crossing_net_ids

    def _classify_net_for_endpoint_correction(
        self,
        net_id: int,
        *,
        crossing_net_ids: set[int],
    ) -> NetEndpointCorrectionClassification | None:
        """Classify one net for endpoint-correction dispatch.

        Mirrors, exactly, the per-net guard conditions that were
        previously inline and duplicated across
        `_apply_checked_endpoint_corrections_for_net_ids` and
        `_apply_checked_fanout_stub_endpoint_corrections_for_net_ids`
        (Milestone 1). Returns `None` only when the net has no record or
        job at all in this session's bookkeeping, which every caller
        already treats as "skip" today.

        This only decides which of pass 1 (unrestricted), pass 2
        (fanout-stub partial), pass 3 (crossing-aware), or no pass at all
        applies. It deliberately does not also decide whether the
        resolved source/target ports are usable (`None`/`None`): that
        check differs between pass 1 (resolves both sides unconditionally)
        and pass 2 (resolves only the non-stub side, since the stub side
        is deliberately left as `None`), so each pass still performs its
        own port-resolution check after using this classification to
        decide whether it owns the net at all.
        """
        net_id = int(net_id)
        record = self.route_bookkeeping.records_by_id.get(net_id)
        job = self.route_jobs_by_id.get(net_id)
        if record is None or job is None:
            return None

        source_has_fanout_stub = net_id in self.fanout_anchor_source_net_ids
        target_has_fanout_stub = net_id in self.fanout_anchor_target_net_ids
        has_crossing = self.enable_crossings and net_id in crossing_net_ids

        if has_crossing:
            return NetEndpointCorrectionClassification(
                category=EndpointCorrectionCategory.CROSSING_AWARE,
                has_crossing=True,
                source_has_fanout_stub=source_has_fanout_stub,
                target_has_fanout_stub=target_has_fanout_stub,
            )

        # Pass 1 skips any fanout-anchor net that has already been given a
        # corrected centerline by the earlier fanout-stub pre-correction
        # stage (upstream of all three passes), deferring it to pass 2. A
        # fanout-anchor net that has *not* yet been given one -- an edge
        # case, since pre-correction is expected to have already run by
        # this point -- falls through to UNRESTRICTED below instead, the
        # same as it does in the real pass 1 today; that is preserved
        # exactly, not treated as a bug, since Milestone 1 is additive.
        already_fanout_precorrected = net_id in self.fanout_anchor_net_ids and bool(
            record.corrected_centerline_um
        )
        if already_fanout_precorrected:
            if source_has_fanout_stub and target_has_fanout_stub:
                category = EndpointCorrectionCategory.ALREADY_CORRECTED_NO_OP
            elif source_has_fanout_stub:
                category = EndpointCorrectionCategory.FANOUT_STUB_SOURCE_ONLY
            elif target_has_fanout_stub:
                category = EndpointCorrectionCategory.FANOUT_STUB_TARGET_ONLY
            else:
                # net_id in fanout_anchor_net_ids is constructed as exactly
                # the union of the source/target subsets today, so this is
                # unreachable; if that invariant ever changes, treat it the
                # same as the no-op category rather than guess silently.
                category = EndpointCorrectionCategory.ALREADY_CORRECTED_NO_OP
            return NetEndpointCorrectionClassification(
                category=category,
                has_crossing=False,
                source_has_fanout_stub=source_has_fanout_stub,
                target_has_fanout_stub=target_has_fanout_stub,
            )

        return NetEndpointCorrectionClassification(
            category=EndpointCorrectionCategory.UNRESTRICTED,
            has_crossing=False,
            source_has_fanout_stub=source_has_fanout_stub,
            target_has_fanout_stub=target_has_fanout_stub,
        )

    def _apply_checked_endpoint_corrections_for_net_ids(
        self,
        net_ids: Iterable[int],
        *,
        record_pipeline_timing: bool = True,
        print_warnings: bool = False,
    ) -> list[int]:
        if not self.enable_checked_endpoint_correction:
            return []
        if not hasattr(self.router, "apply_checked_endpoint_corrections"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.apply_checked_endpoint_corrections. "
                "Rebuild it with `maturin develop --release`."
            )
        correction_jobs: list[
            tuple[
                int,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        requested_net_ids = [int(net_id) for net_id in net_ids]
        crossing_net_ids = self._endpoint_correction_crossing_net_ids()
        t_endpoint_correction_pack_start = self._pipeline_timer_start()
        for net_id in requested_net_ids:
            classification = self._classify_net_for_endpoint_correction(
                net_id, crossing_net_ids=crossing_net_ids
            )
            if (
                classification is None
                or classification.category != EndpointCorrectionCategory.UNRESTRICTED
            ):
                continue
            record = self.route_bookkeeping.records_by_id[net_id]
            job = self.route_jobs_by_id[net_id]
            source_port = self._routing_endpoint_center_um(job, source=True)
            target_port = self._routing_endpoint_center_um(job, source=False)
            if source_port is None and target_port is None:
                continue
            source_state, target_state, opened_candidate_cells, _, _ = self._state_openings_for_job(
                job
            )
            clearance_exempt_cells = self._clearance_exempt_cells_for_job(job)
            correction_jobs.append(
                (
                    int(net_id),
                    record.route_obj,
                    sorted(opened_candidate_cells),
                    clearance_exempt_cells,
                    source_port,
                    target_port,
                )
            )
        if record_pipeline_timing:
            self._record_pipeline_timing(
                "endpoint_correction_pack",
                t_endpoint_correction_pack_start,
            )
        if not correction_jobs:
            return []

        correction_start = self._timing_start()
        raw_corrections = self.router.apply_checked_endpoint_corrections(
            correction_jobs,
            float(self.route_width_um),
            int(self.commit_radius_cells),
            int(self.core_commit_radius_cells),
            True,
        )
        correction_elapsed_s = (
            time.perf_counter() - correction_start if self.collect_timing else 0.0
        )
        if record_pipeline_timing:
            self._record_pipeline_timing("endpoint_correction_native", correction_start)
        t_endpoint_correction_processing_start = self._pipeline_timer_start()
        correction_elapsed_per_job_s = correction_elapsed_s / max(1, len(correction_jobs))
        failed_net_ids: list[int] = []
        for raw_correction in cast(Iterable[Any], raw_corrections):
            correction = dict(raw_correction)
            net_id = int(correction["net_id"])
            record = self.route_bookkeeping.records_by_id.get(net_id)
            job = self.route_jobs_by_id.get(net_id)
            if record is None or job is None:
                continue
            error = correction.get("error")
            if error is not None:
                if self.collect_timing:
                    self.route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked grid-to-port endpoint correction skipped for net "
                    f"{job.net_name!r}: {error}"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                # Every net reaching this point was already excluded from
                # crossing handling above (crossing_net_ids), regardless of
                # self.enable_crossings, so there is no later pass that will
                # still process it and no reason to withhold recording this
                # failure. Withholding it here used to leave
                # corrected_centerline_um empty with endpoint_correction_error
                # still None, which made geometry realization's own fallback
                # (_physical_port_centerline, translation/route_rust_realization.py)
                # silently re-derive uncorrected, collision-unchecked geometry
                # instead of surfacing the rejection -- found while confirming
                # the Milestone 0.5 fix in
                # .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md
                # actually took effect end to end.
                self.route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    corrected_centerline_um=(),
                    endpoint_correction_error=message,
                )
                continue
            centerline = _centerline_tuple(correction.get("centerline"))
            if not centerline:
                if self.collect_timing:
                    self.route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked grid-to-port endpoint correction skipped for net "
                    f"{job.net_name!r}: endpoint correction returned an invalid centerline"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                self.route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    corrected_centerline_um=(),
                    endpoint_correction_error=message,
                )
                continue
            if self.collect_timing:
                self.route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                )
            corrected_total_length_um = float(correction["total_length_um"])
            self.route_bookkeeping.records_by_id[net_id] = replace(
                record,
                total_length_um=corrected_total_length_um,
                base_total_length_um=(
                    record.base_total_length_um
                    if record.base_total_length_um is not None
                    else float(record.total_length_um)
                ),
                corrected_centerline_um=centerline,
                endpoint_correction_error=None,
            )
        if record_pipeline_timing:
            self._record_pipeline_timing(
                "endpoint_correction_processing",
                t_endpoint_correction_processing_start,
            )
        return failed_net_ids

    def _apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
        self,
        net_ids: Iterable[int],
        *,
        record_pipeline_timing: bool = True,
        print_warnings: bool = False,
    ) -> list[int]:
        if not self.enable_checked_endpoint_correction or not self.fanout_anchor_net_ids:
            return []
        if not hasattr(self.router, "apply_checked_endpoint_corrections"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.apply_checked_endpoint_corrections. "
                "Rebuild it with `maturin develop --release`."
            )

        correction_jobs: list[
            tuple[
                int,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        job_context_by_id: dict[int, tuple[RoutedNetRecord, bool, bool]] = {}
        requested_net_ids = [int(net_id) for net_id in net_ids]
        crossing_net_ids = self._endpoint_correction_crossing_net_ids()
        t_endpoint_correction_pack_start = self._pipeline_timer_start()
        for net_id in requested_net_ids:
            # A fanout stub is already a corrected endpoint adapter. When the
            # routed net also contains a crossing, the unrestricted native
            # endpoint corrector may move geometry on the protected side of the
            # crossing before we merge it back into the stubbed centerline. Let
            # the crossing-aware pass splice only the source->first-crossing or
            # last-crossing->target segment instead -- this pass's
            # classification puts any crossing net (and any both-sides-stub
            # net) into a category other than the two this pass owns, so
            # both exclusions fall out of the single category check below.
            classification = self._classify_net_for_endpoint_correction(
                net_id, crossing_net_ids=crossing_net_ids
            )
            # ALREADY_CORRECTED_NO_OP (source *and* target stubbed) dates from
            # the eager-stitch design in which both sides were pre-stitched;
            # a target stub is no longer, so such a net still needs its
            # target corrected to the anchor exactly like TARGET_ONLY.
            if classification is None or classification.category not in (
                EndpointCorrectionCategory.FANOUT_STUB_SOURCE_ONLY,
                EndpointCorrectionCategory.FANOUT_STUB_TARGET_ONLY,
                EndpointCorrectionCategory.ALREADY_CORRECTED_NO_OP,
            ):
                continue
            record = self.route_bookkeeping.records_by_id[net_id]
            job = self.route_jobs_by_id[net_id]
            source_has_fanout_stub = classification.source_has_fanout_stub
            target_has_fanout_stub = classification.target_has_fanout_stub

            source_port = (
                None
                if source_has_fanout_stub
                else self._routing_endpoint_center_um(job, source=True)
            )
            # A TARGET fanout stub is not eagerly stitched: the record's
            # target is the anchor's exact point and the search's grid state
            # still has to be corrected to it, otherwise the fixed stub gets
            # spliced onto the raw cell center and the last segment is
            # slanted (realization rejects it as an unsupported terminal
            # stub). Same reasoning as the crossing-aware pass's
            # `correct_target=True`; only SOURCE stubs are pre-stitched.
            target_port = self._routing_endpoint_center_um(job, source=False)
            if source_port is None and target_port is None:
                continue
            _, _, opened_candidate_cells, _, _ = self._state_openings_for_job(job)
            if source_has_fanout_stub:
                opened_candidate_cells.update(
                    self.fanout_stub_static_cells_by_spec.get(f"{job.inst1},{job.port1}", set())
                )
            if target_has_fanout_stub:
                opened_candidate_cells.update(
                    self.fanout_stub_static_cells_by_spec.get(f"{job.inst2},{job.port2}", set())
                )
            clearance_exempt_cells = self._clearance_exempt_cells_for_job(job)
            correction_jobs.append(
                (
                    int(net_id),
                    record.route_obj,
                    sorted(opened_candidate_cells),
                    clearance_exempt_cells,
                    source_port,
                    target_port,
                )
            )
            job_context_by_id[int(net_id)] = (
                record,
                source_has_fanout_stub,
                target_has_fanout_stub,
            )

        if record_pipeline_timing:
            self._record_pipeline_timing(
                "fanout_stub_endpoint_correction_pack",
                t_endpoint_correction_pack_start,
            )
        if not correction_jobs:
            return []

        correction_start = self._timing_start()
        raw_corrections = self.router.apply_checked_endpoint_corrections(
            correction_jobs,
            float(self.route_width_um),
            int(self.commit_radius_cells),
            int(self.core_commit_radius_cells),
            True,
        )
        correction_elapsed_s = (
            time.perf_counter() - correction_start if self.collect_timing else 0.0
        )
        if record_pipeline_timing:
            self._record_pipeline_timing(
                "fanout_stub_endpoint_correction_native",
                correction_start,
            )
        t_endpoint_correction_processing_start = self._pipeline_timer_start()
        correction_elapsed_per_job_s = correction_elapsed_s / max(1, len(correction_jobs))
        failed_net_ids: list[int] = []
        for raw_correction in cast(Iterable[Any], raw_corrections):
            correction = dict(raw_correction)
            net_id = int(correction["net_id"])
            context = job_context_by_id.get(net_id)
            job = self.route_jobs_by_id.get(net_id)
            if context is None or job is None:
                continue
            record, source_has_fanout_stub, target_has_fanout_stub = context
            error = correction.get("error")
            if error is not None:
                if self.collect_timing:
                    self.route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked fanout-stub endpoint correction skipped for net "
                    f"{job.net_name!r}: {error}"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                self.route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    endpoint_correction_error=message,
                )
                continue

            corrected_route = _dedupe_centerline(_centerline_tuple(correction.get("centerline")))
            route_baseline = _primitive_centerline_for_record(
                record,
                router=self.router,
                prefer_corrected_baseline=False,
            )
            existing_baseline = _dedupe_centerline(record.corrected_centerline_um)
            merged_centerline = _merge_terminal_corrected_route_centerline(
                existing_baseline=existing_baseline,
                route_baseline=route_baseline,
                corrected_route_centerline=corrected_route,
                freeze_source=source_has_fanout_stub,
                freeze_target=target_has_fanout_stub,
            )
            if len(merged_centerline) < 2:
                if self.collect_timing:
                    self.route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                route_start = route_baseline[0] if route_baseline else None
                route_end = route_baseline[-1] if route_baseline else None
                existing_start = existing_baseline[0] if existing_baseline else None
                existing_end = existing_baseline[-1] if existing_baseline else None
                corrected_start = corrected_route[0] if corrected_route else None
                corrected_end = corrected_route[-1] if corrected_route else None
                message = (
                    "Checked fanout-stub endpoint correction skipped for net "
                    f"{job.net_name!r}: could not merge corrected route segment "
                    "back into static stub centerline "
                    f"(existing_len={len(existing_baseline)}, "
                    f"route_len={len(route_baseline)}, "
                    f"corrected_len={len(corrected_route)}, "
                    f"freeze_source={source_has_fanout_stub}, "
                    f"freeze_target={target_has_fanout_stub}, "
                    f"existing_start={existing_start}, existing_end={existing_end}, "
                    f"route_start={route_start}, route_end={route_end}, "
                    f"corrected_start={corrected_start}, corrected_end={corrected_end})"
                )
                if print_warnings:
                    print("WARNING: " + message)
                failed_net_ids.append(net_id)
                self.route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    endpoint_correction_error=message,
                )
                continue

            if self.collect_timing:
                self.route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                )
            centerline_length = getattr(self.router, "centerline_length_um", None)
            if centerline_length is not None:
                try:
                    corrected_total_length_um = float(centerline_length(list(merged_centerline)))
                except Exception:
                    corrected_total_length_um = _centerline_length_um(merged_centerline)
            else:
                corrected_total_length_um = _centerline_length_um(merged_centerline)
            self.route_bookkeeping.records_by_id[net_id] = replace(
                record,
                total_length_um=corrected_total_length_um,
                base_total_length_um=(
                    record.base_total_length_um
                    if record.base_total_length_um is not None
                    else float(record.total_length_um)
                ),
                corrected_centerline_um=merged_centerline,
                endpoint_correction_error=None,
            )

        if record_pipeline_timing:
            self._record_pipeline_timing(
                "fanout_stub_endpoint_correction_processing",
                t_endpoint_correction_processing_start,
            )
        return failed_net_ids

    def _current_crossing_points_by_net_id(self) -> dict[int, list[tuple[float, float]]]:
        if not self.enable_crossings or not hasattr(self.router, "crossing_events"):
            return {}
        try:
            raw_events = list(cast(Iterable[Any], self.router.crossing_events()))
        except Exception:
            return {}
        if not raw_events:
            return {}
        _populate_realized_intersections_from_native_crossing_events(
            crossing_plan_info=self.crossing_plan_info,
            routed_records_by_net_id=self.route_bookkeeping.records_by_id,
            native_crossing_events=raw_events,
            realization_grid_spec=self.realization_grid_spec,
        )
        return _legal_crossing_points_by_net_id(self.crossing_plan_info)

    def _route_target_grid_center_um(
        self,
        route_obj: object | None,
    ) -> tuple[float, float] | None:
        if route_obj is None:
            return None
        raw_state = getattr(route_obj, "reached_target", None)
        if raw_state is None:
            states = getattr(route_obj, "states", None)
            try:
                raw_state = cast(Any, states)[-1]
            except (TypeError, IndexError):
                return None
        try:
            return self._grid_cell_center_um(int(raw_state.x), int(raw_state.y))
        except (AttributeError, TypeError, ValueError):
            return None

    def _route_target_angle(
        self,
        route_obj: object | None,
    ) -> int | None:
        if route_obj is None:
            return None
        raw_state = getattr(route_obj, "reached_target", None)
        if raw_state is None:
            states = getattr(route_obj, "states", None)
            try:
                raw_state = cast(Any, states)[-1]
            except (TypeError, IndexError):
                return None
        try:
            return int(raw_state.angle) % 8
        except (AttributeError, TypeError, ValueError):
            return None

    def _record_terminal_bump_distance_check_candidates(
        self,
        crossing_points_by_net_id: Mapping[int, list[tuple[float, float]]],
        net_ids: Iterable[int],
    ) -> None:
        """Record where a terminal bump distance check would become active.

        This is diagnostic-only: the router behavior is unchanged.  The check
        is axis-specific: horizontal target approaches care only about
        physical-port-vs-grid y offset, while vertical target approaches care
        only about physical-port-vs-grid x offset.  If a realized crossing sits
        on that same target axis, a future A* guard must make sure the
        remaining terminal segment is long enough to insert the required bump
        geometry.
        """

        if not isinstance(self.crossing_plan_info, dict):
            return

        trace_raw = os.environ.get("PHOTONIC_ROUTER_TRACE_TERMINAL_BUMP_DISTANCE_CHECKS", "")
        trace_tokens = {item.strip() for item in trace_raw.split(",") if item.strip()}
        trace_all = "*" in trace_tokens
        grid_size = float(self.grid.grid_size_um)
        eps = max(1e-6, grid_size * 1e-6)
        axis_eps = max(1e-6, grid_size * 0.25)
        crossing_half_um = (
            float(self.crossing_plan_info.get("crossing_half_size_cells", 0) or 0) * grid_size
        )
        required_bump_um = 4.0 * float(self.bend_radius_cells) * grid_size

        target_x_offset_nets: list[dict[str, object]] = []
        target_y_offset_nets: list[dict[str, object]] = []
        active_checks: list[dict[str, object]] = []

        for raw_net_id in net_ids:
            net_id = int(raw_net_id)
            record = self.route_bookkeeping.records_by_id.get(net_id)
            job = self.route_jobs_by_id.get(net_id)
            if record is None or job is None or record.target_port_center_um is None:
                continue
            target_grid_um = self._route_target_grid_center_um(record.route_obj)
            if target_grid_um is None:
                continue
            target_port_um = record.target_port_center_um
            target_dx_um = float(target_port_um[0]) - float(target_grid_um[0])
            target_dy_um = float(target_port_um[1]) - float(target_grid_um[1])
            target_angle = self._route_target_angle(record.route_obj)
            target_axis = (
                "horizontal"
                if target_angle in (0, 4)
                else "vertical"
                if target_angle in (2, 6)
                else "diagonal"
            )

            net_entry = {
                "net_id": int(net_id),
                "net_name": str(record.net_name),
                "target_port": f"{job.inst2},{job.port2}",
                "target_port_um": [
                    round(float(target_port_um[0]), 6),
                    round(float(target_port_um[1]), 6),
                ],
                "target_grid_um": [
                    round(float(target_grid_um[0]), 6),
                    round(float(target_grid_um[1]), 6),
                ],
                "target_dx_um": round(float(target_dx_um), 6),
                "target_dy_um": round(float(target_dy_um), 6),
                "target_angle": None if target_angle is None else int(target_angle),
                "target_axis": target_axis,
            }
            target_x_active = target_axis == "vertical" and abs(target_dx_um) > eps
            target_y_active = target_axis == "horizontal" and abs(target_dy_um) > eps
            if target_x_active:
                target_x_offset_nets.append(net_entry)
            if target_y_active:
                target_y_offset_nets.append(net_entry)
            if not target_x_active and not target_y_active:
                continue

            for crossing_point in crossing_points_by_net_id.get(net_id, []):
                crossing_x = float(crossing_point[0])
                crossing_y = float(crossing_point[1])
                if target_x_active and abs(crossing_x - float(target_grid_um[0])) <= axis_eps:
                    distance_to_target_um = abs(float(target_grid_um[1]) - crossing_y)
                    available_um = max(0.0, distance_to_target_um - crossing_half_um)
                    active_checks.append(
                        {
                            **net_entry,
                            "axis": "target_x",
                            "crossing_um": [round(crossing_x, 6), round(crossing_y, 6)],
                            "distance_to_target_um": round(float(distance_to_target_um), 6),
                            "crossing_half_um": round(float(crossing_half_um), 6),
                            "available_um": round(float(available_um), 6),
                            "required_bump_um": round(float(required_bump_um), 6),
                            "satisfies": bool(available_um + eps >= required_bump_um),
                        }
                    )
                if target_y_active and abs(crossing_y - float(target_grid_um[1])) <= axis_eps:
                    distance_to_target_um = abs(float(target_grid_um[0]) - crossing_x)
                    available_um = max(0.0, distance_to_target_um - crossing_half_um)
                    active_checks.append(
                        {
                            **net_entry,
                            "axis": "target_y",
                            "crossing_um": [round(crossing_x, 6), round(crossing_y, 6)],
                            "distance_to_target_um": round(float(distance_to_target_um), 6),
                            "crossing_half_um": round(float(crossing_half_um), 6),
                            "available_um": round(float(available_um), 6),
                            "required_bump_um": round(float(required_bump_um), 6),
                            "satisfies": bool(available_um + eps >= required_bump_um),
                        }
                    )

            should_trace = (
                trace_all or record.net_name in trace_tokens or str(net_id) in trace_tokens
            )
            if should_trace:
                matching_checks = [
                    item for item in active_checks if int(item["net_id"]) == int(net_id)
                ]
                print(
                    "terminal_bump_distance_check "
                    f"net={record.net_name} id={net_id} "
                    f"target_port={job.inst2},{job.port2} "
                    f"target_port={target_port_um} target_grid={target_grid_um} "
                    f"target_axis={target_axis} "
                    f"target_dx={target_dx_um:.6f} target_dy={target_dy_um:.6f} "
                    f"same_axis_crossings={len(matching_checks)}"
                )
                for item in matching_checks:
                    print(
                        "terminal_bump_distance_check "
                        f"net={record.net_name} id={net_id} "
                        f"crossing={item['crossing_um']} "
                        f"available_um={item['available_um']} "
                        f"required_um={item['required_bump_um']} "
                        f"satisfies={item['satisfies']}"
                    )

        self.crossing_plan_info["terminal_bump_target_x_offset_nets"] = target_x_offset_nets
        self.crossing_plan_info["terminal_bump_target_x_offset_net_count"] = len(
            target_x_offset_nets
        )
        self.crossing_plan_info["terminal_bump_target_y_offset_nets"] = target_y_offset_nets
        self.crossing_plan_info["terminal_bump_target_y_offset_net_count"] = len(
            target_y_offset_nets
        )
        failed_checks = [check for check in active_checks if not bool(check.get("satisfies"))]
        self.crossing_plan_info["terminal_bump_distance_checks"] = active_checks
        self.crossing_plan_info["terminal_bump_distance_check_count"] = len(active_checks)
        self.crossing_plan_info["terminal_bump_distance_failures"] = failed_checks
        self.crossing_plan_info["terminal_bump_distance_failure_count"] = len(failed_checks)

    def _apply_crossing_aware_endpoint_corrections_for_net_ids(
        self,
        net_ids: Iterable[int],
        *,
        record_pipeline_timing: bool = True,
        print_warnings: bool = False,
    ) -> list[int]:
        if not self.enable_checked_endpoint_correction or not self.enable_crossings:
            return []
        crossing_points_by_net_id = self._current_crossing_points_by_net_id()
        if not crossing_points_by_net_id:
            return []
        requested_net_ids = [int(net_id) for net_id in net_ids]
        self._record_terminal_bump_distance_check_candidates(
            crossing_points_by_net_id,
            requested_net_ids,
        )

        t_endpoint_correction_start = self._pipeline_timer_start()
        failed_net_ids: list[int] = []
        for raw_net_id in requested_net_ids:
            net_id = int(raw_net_id)
            crossing_points = crossing_points_by_net_id.get(net_id, [])
            if not crossing_points:
                # A net classified crossing-aware (the crossing plan expected
                # it to cross something) but with zero REALIZED crossing
                # points normally just gets skipped here entirely, on the
                # assumption it will be corrected some other way. That
                # assumption silently breaks for a net whose target has a
                # dense-fanout static stub (`target_has_fanout_stub`):
                # `record.target_port_center_um` was deliberately pointed at
                # the stub anchor's own exact position (not the true port)
                # specifically so THIS correction machinery would reconcile
                # it, but skipping here means nothing ever does -- see
                # `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`
                # Surprises & Discoveries for the concrete trace that found
                # this. `_apply_crossing_aware_endpoint_correction_to_record`
                # already has its own correct, generic handling for empty
                # `crossing_points` (falls back to plain
                # `apply_port_endpoint_corrections`), so only nets that
                # actually need it are let through here, keeping every other
                # net's existing behavior (and this function's own
                # early-return-on-nothing-to-do intent) unchanged.
                if net_id not in self.fanout_anchor_net_ids:
                    continue
            record = self.route_bookkeeping.records_by_id.get(net_id)
            job = self.route_jobs_by_id.get(net_id)
            if record is None or job is None:
                continue

            source_has_fanout_stub = net_id in self.fanout_anchor_source_net_ids
            target_has_fanout_stub = net_id in self.fanout_anchor_target_net_ids
            record_has_fanout_stub = net_id in self.fanout_anchor_net_ids
            _, _, opened_candidate_cells, _, _ = self._state_openings_for_job(job)
            if source_has_fanout_stub:
                opened_candidate_cells.update(
                    self.fanout_stub_static_cells_by_spec.get(f"{job.inst1},{job.port1}", set())
                )
            if target_has_fanout_stub:
                opened_candidate_cells.update(
                    self.fanout_stub_static_cells_by_spec.get(f"{job.inst2},{job.port2}", set())
                )
            clearance_exempt_cells = self._clearance_exempt_cells_for_job(job)
            start_s = self._timing_start()
            updated = _apply_crossing_aware_endpoint_correction_to_record(
                record,
                router=cast(EndpointCorrectionRouter, self.router),
                crossing_points=crossing_points,
                realization_grid_spec=self.realization_grid_spec,
                route_width_um=float(self.route_width_um),
                allow_unchecked_bumps=False,
                log_failures=print_warnings,
                crossing_plan_info=self.crossing_plan_info,
                correct_source=not source_has_fanout_stub,
                # Unlike a source fanout stub (still always eagerly,
                # fully pre-stitched to the true port by
                # `_fanout_stubbed_centerline`, so its side never needs
                # further correction), a TARGET fanout stub's own
                # `record.target_port_center_um` is deliberately pointed
                # at the anchor's own exact position, not the true port
                # (see `RouteBookkeeping.record_route`'s
                # `target_port_center_um_override`) -- so the target side
                # of a crossing-aware net always still needs correction
                # too, regardless of whether it has a fanout stub. Always
                # correcting here (instead of `not target_has_fanout_stub`,
                # which assumed the old eager-stitch design where the
                # target was already fully corrected) is what actually
                # closes that gap; see
                # `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
                correct_target=True,
                prefer_corrected_baseline=(
                    record_has_fanout_stub and bool(record.corrected_centerline_um)
                ),
                opened_cells=opened_candidate_cells,
                clearance_exempt_cells=clearance_exempt_cells,
                clearance_radius_cells=int(self.commit_radius_cells),
                core_radius_cells=int(self.core_commit_radius_cells),
            )
            failed = updated.endpoint_correction_error is not None
            if self.collect_timing:
                self.route_timing_buckets["endpoint_correction"].record_elapsed(
                    time.perf_counter() - start_s,
                    failed=failed,
                )
            if failed:
                failed_net_ids.append(net_id)
            self.route_bookkeeping.records_by_id[net_id] = updated

        if record_pipeline_timing:
            self._record_pipeline_timing(
                "crossing_endpoint_correction",
                t_endpoint_correction_start,
            )
        return failed_net_ids

    def _apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
        self,
        net_ids: Iterable[int],
        *,
        record_pipeline_timing: bool = True,
        print_warnings: bool = False,
    ) -> list[int]:
        """Pass 1 and pass 2 together: every net in `net_ids` not involved
        in a crossing gets either the unrestricted corrector or the
        fanout-stub-partial corrector, whichever `_classify_net_for_endpoint_correction`
        calls for; a net involved in a crossing is left untouched here (see
        `_apply_all_endpoint_corrections_for_net_ids` below for where that
        net gets handled). Both existing passes already self-filter safely
        by classification, so calling both with the same, unfiltered
        `net_ids` list is exactly equivalent to today's behavior, just
        expressed as one call instead of two.

        This is also, deliberately, as far as consolidation goes for the
        two mid-repair call sites in `_route_many_with_repair_and_commit`-style
        methods (around what were lines 5638-5645 and 6019-6028 before this
        milestone): repair runs before the run's final crossing plan is
        settled, so those call sites only ever need this pair, never the
        crossing-aware pass, and always did -- this method exists to give
        that pre-existing two-call pattern one name instead of leaving it
        duplicated verbatim in two places (Milestone 3 of
        .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md).
        """
        net_id_list = [int(net_id) for net_id in net_ids]
        failed_net_ids = list(
            self._apply_checked_endpoint_corrections_for_net_ids(
                net_id_list,
                record_pipeline_timing=record_pipeline_timing,
                print_warnings=print_warnings,
            )
        )
        failed_net_ids.extend(
            self._apply_checked_fanout_stub_endpoint_corrections_for_net_ids(
                net_id_list,
                record_pipeline_timing=record_pipeline_timing,
                print_warnings=print_warnings,
            )
        )
        return failed_net_ids

    def _apply_all_endpoint_corrections_for_net_ids(
        self,
        net_ids: Iterable[int],
        *,
        print_warnings: bool = False,
    ) -> list[int]:
        """The one entry point for endpoint correction, telling the whole
        story top to bottom: every net not involved in a crossing gets the
        unrestricted or fanout-stub-partial corrector (whichever its
        classification calls for -- see `_classify_net_for_endpoint_correction`
        and `_apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids`
        above); every net involved in a crossing instead gets the separate,
        checked, per-segment crossing-aware corrector, which either succeeds
        or fails honestly with `RoutedNetRecord.endpoint_correction_error`
        set -- there is no unchecked fallback strategy (see
        .agent/execplans/2026-08-24-endpoint-correction-cascade-soundness.md).
        Replaces `run()`'s three separate, independently-ordered
        calls with one (Milestone 3 of
        .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md).

        This function is not used by the two mid-repair call sites, which
        only ever need the first two passes (see
        `_apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids`'s
        own docstring for why) -- calling this one there would run the
        crossing-aware pass mid-repair, before the run's final crossing
        plan is settled, which is not equivalent to today's behavior and
        was deliberately not attempted as part of this additive milestone.
        """
        net_id_list = [int(net_id) for net_id in net_ids]
        failed_net_ids = self._apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
            net_id_list,
            print_warnings=print_warnings,
        )
        failed_net_ids.extend(
            self._apply_crossing_aware_endpoint_corrections_for_net_ids(
                net_id_list,
                print_warnings=print_warnings,
            )
        )
        return failed_net_ids

    def _grid_cell_from_raw_point(self, raw_point: object) -> tuple[int, int] | None:
        if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
            return None
        try:
            point_x = float(raw_point[0])
            point_y = float(raw_point[1])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(point_x) or not math.isfinite(point_y):
            return None
        return physical_to_grid(point_x, point_y, self.grid)

    def _illegal_crossing_grid_cell(self, item: Mapping[str, object]) -> tuple[int, int] | None:
        raw_cell = item.get("grid_cell")
        if isinstance(raw_cell, (tuple, list)) and len(raw_cell) == 2:
            try:
                return (int(raw_cell[0]), int(raw_cell[1]))
            except (TypeError, ValueError):
                return None
        return self._grid_cell_from_raw_point(item.get("point_um"))

    def _illegal_crossing_keepout_radius(self, item: Mapping[str, object]) -> int:
        reason = str(item.get("reason", "") or "")
        if reason == "not_perpendicular":
            return max(1, int(self.resolved_crossing_half_size_cells) + 1)
        if reason == "collinear_route_overlap":
            return 1
        blockers = item.get("crossing_footprint_blockers")
        if isinstance(blockers, IterableABC) and not isinstance(
            blockers,
            (str, bytes, bytearray),
        ):
            if any(isinstance(blocker, Mapping) for blocker in blockers):
                return max(1, int(self.resolved_crossing_half_size_cells) + 1)
        if reason in {
            "crossing_footprint_contains_route_geometry",
            "crossing_footprint_overlap",
        }:
            return max(1, int(self.resolved_crossing_half_size_cells) + 1)
        return max(1, min(4, int(self.resolved_crossing_half_size_cells) + 1))

    def _add_keepout_square(
        self,
        keepout_cells: set[tuple[int, int]],
        *,
        center: tuple[int, int],
        radius: int,
    ) -> None:
        for y in range(center[1] - radius, center[1] + radius + 1):
            for x in range(center[0] - radius, center[0] + radius + 1):
                if 0 <= x < int(self.grid.width) and 0 <= y < int(self.grid.height):
                    keepout_cells.add((x, y))

    def _add_segment_keepout_cells(
        self,
        keepout_cells: set[tuple[int, int]],
        *,
        start_cell: tuple[int, int],
        end_cell: tuple[int, int],
        radius: int,
    ) -> None:
        dx = int(end_cell[0]) - int(start_cell[0])
        dy = int(end_cell[1]) - int(start_cell[1])
        steps = max(abs(dx), abs(dy), 1)
        for step in range(steps + 1):
            t = float(step) / float(steps)
            cell = (
                int(round(float(start_cell[0]) + float(dx) * t)),
                int(round(float(start_cell[1]) + float(dy) * t)),
            )
            self._add_keepout_square(keepout_cells, center=cell, radius=radius)

    def _final_crossing_repair_keepout_cells(
        self,
        illegal_crossings: Iterable[Mapping[str, object]],
    ) -> set[tuple[int, int]]:
        keepout_cells: set[tuple[int, int]] = set()
        for item in illegal_crossings:
            radius = self._illegal_crossing_keepout_radius(item)
            reason = str(item.get("reason", "") or "")
            if reason == "collinear_route_overlap":
                start_cell = self._grid_cell_from_raw_point(item.get("overlap_start_um"))
                end_cell = self._grid_cell_from_raw_point(item.get("overlap_end_um"))
                if start_cell is not None and end_cell is not None:
                    self._add_segment_keepout_cells(
                        keepout_cells,
                        start_cell=start_cell,
                        end_cell=end_cell,
                        radius=radius,
                    )
                    continue
            center = self._illegal_crossing_grid_cell(item)
            if center is not None:
                self._add_keepout_square(keepout_cells, center=center, radius=radius)
            if reason == "crossing_footprint_overlap":
                peer = item.get("overlapping_crossing")
                if isinstance(peer, Mapping):
                    peer_center = self._grid_cell_from_raw_point(peer.get("point_um"))
                    if peer_center is not None:
                        self._add_keepout_square(
                            keepout_cells,
                            center=peer_center,
                            radius=radius,
                        )
        return keepout_cells

    def _final_crossing_repair_net_ids(
        self,
        illegal_crossings: Iterable[Mapping[str, object]],
    ) -> list[int]:
        net_ids: set[int] = set()

        def _add_footprint_blocker_net_ids(item: Mapping[str, object]) -> None:
            blockers = item.get("crossing_footprint_blockers")
            if not isinstance(blockers, IterableABC) or isinstance(
                blockers,
                (str, bytes, bytearray),
            ):
                return
            for blocker in blockers:
                if not isinstance(blocker, Mapping):
                    continue
                try:
                    blocker_net_id = int(cast(object, blocker.get("net_id")))
                except (TypeError, ValueError):
                    continue
                if blocker_net_id in self.route_jobs_by_id:
                    net_ids.add(blocker_net_id)

        for item in illegal_crossings:
            item_pair_ids: list[int] = []
            for key in ("net_id_a", "net_id_b"):
                try:
                    net_id = int(cast(object, item.get(key)))
                except (TypeError, ValueError):
                    continue
                if net_id in self.route_jobs_by_id:
                    item_pair_ids.append(net_id)
            reason = str(item.get("reason", "") or "")
            if reason == "not_perpendicular":
                net_ids.update(item_pair_ids)
                _add_footprint_blocker_net_ids(item)
                continue
            if reason in {
                "crossing_footprint_contains_bend",
                "insufficient_straight_margin",
            }:
                if item_pair_ids:
                    net_ids.add(max(item_pair_ids))
                _add_footprint_blocker_net_ids(item)
                continue
            if reason == "collinear_route_overlap":
                net_ids.update(item_pair_ids)
                continue
            if reason == "crossing_footprint_overlap":
                peer_pair_ids: set[int] = set()
                peer = item.get("overlapping_crossing")
                if isinstance(peer, Mapping):
                    for key in ("net_id_a", "net_id_b"):
                        try:
                            peer_net_id = int(cast(object, peer.get(key)))
                        except (TypeError, ValueError):
                            continue
                        if peer_net_id in self.route_jobs_by_id:
                            peer_pair_ids.add(peer_net_id)
                net_ids.update(item_pair_ids)
                net_ids.update(peer_pair_ids)
                continue
            if reason == "crossing_footprint_contains_route_geometry":
                before_blocker_net_ids = set(net_ids)
                _add_footprint_blocker_net_ids(item)
                if net_ids == before_blocker_net_ids:
                    net_ids.update(item_pair_ids)
                continue
            net_ids.update(item_pair_ids)
        return [net_id for net_id in self.route_order if net_id in net_ids]

    def _final_crossing_repair_batches(
        self,
        illegal_crossings: list[dict[str, object]],
        *,
        max_net_ids: int,
    ) -> list[list[dict[str, object]]]:
        if not illegal_crossings or max_net_ids <= 0:
            return []
        components: list[tuple[list[dict[str, object]], set[int]]] = []
        for item in illegal_crossings:
            item_net_ids = set(self._final_crossing_repair_net_ids([item]))
            if not item_net_ids:
                components.append(([item], set()))
                continue
            matching_indices = [
                index
                for index, (_items, component_net_ids) in enumerate(components)
                if component_net_ids.intersection(item_net_ids)
            ]
            if not matching_indices:
                components.append(([item], set(item_net_ids)))
                continue
            target_index = matching_indices[0]
            target_items, target_net_ids = components[target_index]
            target_items.append(item)
            target_net_ids.update(item_net_ids)
            for merge_index in reversed(matching_indices[1:]):
                merge_items, merge_net_ids = components.pop(merge_index)
                target_items.extend(merge_items)
                target_net_ids.update(merge_net_ids)

        capped_batches: list[list[dict[str, object]]] = []
        oversized: list[dict[str, object]] = []
        for component_items, _component_net_ids in components:
            component_repair_ids = self._final_crossing_repair_net_ids(component_items)
            if component_repair_ids and len(component_repair_ids) <= max_net_ids:
                capped_batches.append(component_items)
            else:
                oversized.extend(component_items)

        if oversized:
            current_batch: list[dict[str, object]] = []
            current_net_ids: set[int] = set()
            for item in oversized:
                item_net_ids = set(self._final_crossing_repair_net_ids([item]))
                if (
                    current_batch
                    and item_net_ids
                    and len(current_net_ids.union(item_net_ids)) > max_net_ids
                ):
                    capped_batches.append(current_batch)
                    current_batch = []
                    current_net_ids = set()
                current_batch.append(item)
                current_net_ids.update(item_net_ids)
            if current_batch:
                capped_batches.append(current_batch)

        route_index = {int(net_id): index for index, net_id in enumerate(self.route_order)}

        def _batch_sort_key(batch: list[dict[str, object]]) -> tuple[int, int, int]:
            repair_ids = self._final_crossing_repair_net_ids(batch)
            first_route_index = min(
                (route_index.get(int(net_id), len(route_index)) for net_id in repair_ids),
                default=len(route_index),
            )
            return (first_route_index, -len(batch), len(repair_ids))

        capped_batches.sort(key=_batch_sort_key)
        return capped_batches

    def _repair_final_illegal_crossings(
        self,
        illegal_crossings: list[dict[str, object]],
    ) -> bool:
        max_repair_net_ids = 12
        attempts = cast(
            list[dict[str, object]],
            self.crossing_plan_info.setdefault("final_crossing_repair_attempts", []),
        )
        priority_order = (
            "collinear_route_overlap",
            "crossing_footprint_contains_route_geometry",
            "crossing_footprint_contains_bend",
            "insufficient_straight_margin",
            "not_perpendicular",
            "crossing_footprint_overlap",
        )
        selected_reason = next(
            (
                reason
                for reason in priority_order
                if any(str(item.get("reason", "") or "") == reason for item in illegal_crossings)
            ),
            None,
        )
        selected_illegal_crossings = [
            item
            for item in illegal_crossings
            if selected_reason is None or str(item.get("reason", "") or "") == selected_reason
        ]
        total_selected_issue_count = len(selected_illegal_crossings)
        if selected_reason in {
            "crossing_footprint_contains_bend",
            "insufficient_straight_margin",
            "not_perpendicular",
        }:
            repair_batches = self._final_crossing_repair_batches(
                selected_illegal_crossings,
                max_net_ids=min(4, max_repair_net_ids),
            )
            if repair_batches:
                selected_illegal_crossings = repair_batches[0]
        if (
            not selected_illegal_crossings
            or not self.crossing_plan_info.get("enabled")
            or not self.repair_config.enabled
            or not hasattr(self.router, "add_static_cells")
            or not hasattr(self.router, "ripup_route")
            or not hasattr(self.router, "route_many_with_repair_and_commit")
        ):
            return False
        repair_net_ids = self._final_crossing_repair_net_ids(selected_illegal_crossings)
        if selected_reason == "not_perpendicular" and repair_net_ids:
            priority: list[int] = []
            for item in selected_illegal_crossings:
                for key in ("net_id_b", "net_id_a"):
                    try:
                        net_id = int(cast(object, item.get(key)))
                    except (TypeError, ValueError):
                        continue
                    if net_id in self.route_jobs_by_id and net_id not in priority:
                        priority.append(net_id)
                blockers = item.get("crossing_footprint_blockers")
                if not isinstance(blockers, IterableABC) or isinstance(
                    blockers,
                    (str, bytes, bytearray),
                ):
                    continue
                for blocker in blockers:
                    if not isinstance(blocker, Mapping):
                        continue
                    try:
                        blocker_net_id = int(cast(object, blocker.get("net_id")))
                    except (TypeError, ValueError):
                        continue
                    if blocker_net_id in self.route_jobs_by_id and blocker_net_id not in priority:
                        priority.append(blocker_net_id)
            repair_net_ids = [net_id for net_id in priority if net_id in repair_net_ids] + [
                net_id for net_id in repair_net_ids if net_id not in priority
            ]
        attempt: dict[str, object] = {
            "selected_reason": selected_reason,
            "illegal_reason_counts": dict(
                Counter(str(item.get("reason", "") or "unknown") for item in illegal_crossings)
            ),
            "selected_reason_counts": dict(
                Counter(
                    str(item.get("reason", "") or "unknown") for item in selected_illegal_crossings
                )
            ),
            "repair_net_ids": [int(net_id) for net_id in repair_net_ids],
        }
        if total_selected_issue_count != len(selected_illegal_crossings):
            attempt["batched_issue_count"] = len(selected_illegal_crossings)
            attempt["total_selected_issue_count"] = total_selected_issue_count
        if not repair_net_ids or len(repair_net_ids) > max_repair_net_ids:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_repairable_nets_or_too_many"
            attempts.append(attempt)
            return False
        keepout_cells = self._final_crossing_repair_keepout_cells(selected_illegal_crossings)
        attempt["keepout_cell_count"] = len(keepout_cells)
        if not keepout_cells:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_keepout_cells"
            attempts.append(attempt)
            return False

        self.router.add_static_cells(sorted(keepout_cells))
        for net_id in repair_net_ids:
            self.router.ripup_route(int(net_id))
            self.route_bookkeeping.clear_route(int(net_id))

        repair_jobs: list[
            tuple[
                int,
                Any,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        opened_by_id: dict[int, list[tuple[int, int]]] = {}
        for net_id in repair_net_ids:
            job = self.route_jobs_by_id[net_id]
            source_state, target_state, _, _, opened_cells = self._state_openings_for_job(job)
            clearance_exempt_cells = self._clearance_exempt_cells_for_job(job)
            repair_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    self._foreign_keepout_cleanup_cells_for_job(job),
                    self._routing_endpoint_center_um(job, source=True),
                    self._routing_endpoint_center_um(job, source=False),
                )
            )
            opened_by_id[int(job.net_id)] = opened_cells

        raw_repair_result = self.router.route_many_with_repair_and_commit(
            repair_jobs,
            self.block_radius_cells,
            self.commit_radius_cells,
            self.core_commit_radius_cells,
            int(self.repair_config.max_rounds),
            int(self.repair_config.max_victims_per_failure),
            float(self.repair_config.history_weight),
            int(self.repair_config.history_increment),
        )
        repair_result = dict(raw_repair_result)
        attempt["router_status"] = str(repair_result.get("status", ""))
        attempt["routed_net_ids"] = [
            int(dict(raw_entry)["net_id"])
            for raw_entry in cast(Iterable[Any], repair_result.get("routes", []))
        ]
        if str(repair_result.get("status", "")) != "routed":
            attempt["status"] = "failed"
            attempts.append(attempt)
            return False

        repaired_records: list[RoutedNetRecord] = []
        for raw_entry in cast(Iterable[Any], repair_result.get("routes", [])):
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            job = self.route_jobs_by_id[net_id]
            route_obj = entry["route"]
            self._record_route(job, route_obj, opened_by_id[net_id])
            repaired_records.append(self.route_bookkeeping.records_by_id[net_id])

        if self.enable_checked_endpoint_correction and repaired_records:
            repaired_net_ids = [
                int(record.net_id) for record in repaired_records if record.net_id is not None
            ]
            self._apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
                repaired_net_ids,
                record_pipeline_timing=False,
            )
        attempt["status"] = "routed"
        attempts.append(attempt)
        return True

    def _net_id_by_name(self) -> dict[str, int]:
        return {
            record.net_name: int(net_id)
            for net_id, record in self.route_bookkeeping.records_by_id.items()
        }

    def _grid_rect_from_um_bbox(
        self,
        raw_bbox: object,
    ) -> tuple[int, int, int, int] | None:
        if not isinstance(raw_bbox, (tuple, list)) or len(raw_bbox) != 4:
            return None
        try:
            min_x_um = float(raw_bbox[0])
            min_y_um = float(raw_bbox[1])
            max_x_um = float(raw_bbox[2])
            max_y_um = float(raw_bbox[3])
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (min_x_um, min_y_um, max_x_um, max_y_um)):
            return None
        if max_x_um < min_x_um:
            min_x_um, max_x_um = max_x_um, min_x_um
        if max_y_um < min_y_um:
            min_y_um, max_y_um = max_y_um, min_y_um
        grid_size = float(self.grid.grid_size_um)
        min_cell = physical_to_grid(min_x_um, min_y_um, self.grid)
        return (
            min_cell[0],
            int(math.ceil((max_x_um - float(self.origin_x_um)) / grid_size)),
            min_cell[1],
            int(math.ceil((max_y_um - float(self.origin_y_um)) / grid_size)),
        )

    def _add_keepout_rect(
        self,
        keepout_cells: set[tuple[int, int]],
        *,
        rect: tuple[int, int, int, int],
        radius: int,
    ) -> None:
        min_x, max_x, min_y, max_y = rect
        if min_x > max_x:
            min_x, max_x = max_x, min_x
        if min_y > max_y:
            min_y, max_y = max_y, min_y
        for y in range(min_y - radius, max_y + radius + 1):
            if y < 0 or y >= int(self.grid.height):
                continue
            for x in range(min_x - radius, max_x + radius + 1):
                if 0 <= x < int(self.grid.width):
                    keepout_cells.add((x, y))

    def _cells_from_um_bbox(
        self,
        raw_bbox: object,
        *,
        radius: int,
    ) -> set[tuple[int, int]]:
        rect = self._grid_rect_from_um_bbox(raw_bbox)
        if rect is None:
            return set()
        cells: set[tuple[int, int]] = set()
        self._add_keepout_rect(cells, rect=rect, radius=radius)
        return cells

    def _grid_rect_from_grid_bbox_text(
        self, text: str, name: str
    ) -> tuple[int, int, int, int] | None:
        match = re.search(rf"{re.escape(name)}=\((-?\d+),(-?\d+),(-?\d+),(-?\d+)\)", text)
        if match is None:
            return None
        try:
            min_x = int(match.group(1))
            max_x = int(match.group(2))
            min_y = int(match.group(3))
            max_y = int(match.group(4))
        except (TypeError, ValueError):
            return None
        return min_x, max_x, min_y, max_y

    def _polygon_bbox_um(self, raw_polygon: object) -> tuple[float, float, float, float] | None:
        if not isinstance(raw_polygon, IterableABC) or isinstance(
            raw_polygon,
            (str, bytes, bytearray),
        ):
            return None
        points: list[tuple[float, float]] = []
        for raw_point in raw_polygon:
            if not isinstance(raw_point, (tuple, list)) or len(raw_point) != 2:
                return None
            try:
                point = (float(raw_point[0]), float(raw_point[1]))
            except (TypeError, ValueError):
                return None
            if not math.isfinite(point[0]) or not math.isfinite(point[1]):
                return None
            points.append(point)
        if not points:
            return None
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        return min(xs), min(ys), max(xs), max(ys)

    def _photonic_issue_keepout_cells(
        self,
        issue: PhotonicVerificationIssue,
    ) -> set[tuple[int, int]]:
        radius = max(1, int(self.core_commit_radius_cells) + 1)
        if issue.code == "endpoint_correction_error":
            cells: set[tuple[int, int]] = set()
            for bbox_name in ("static_bbox", "core_bbox"):
                rect = self._grid_rect_from_grid_bbox_text(issue.message, bbox_name)
                if rect is None:
                    continue
                self._add_keepout_rect(cells, rect=rect, radius=radius)
                if cells:
                    return cells
            return cells
        details = issue.details or {}
        if issue.code == "cross_net_waveguide_overlap":
            return self._cells_from_um_bbox(
                details.get("overlap_bbox_um"),
                radius=radius,
            )
        if issue.code == "waveguide_obstacle_overlap":
            return self._cells_from_um_bbox(
                details.get("overlap_bbox_um"),
                radius=radius,
            )
        if issue.code == "crossing_component_route_overlap":
            crossing = details.get("crossing")
            if isinstance(crossing, Mapping):
                polygon_bbox = self._polygon_bbox_um(crossing.get("crossing_footprint_polygon_um"))
                if polygon_bbox is not None:
                    return self._cells_from_um_bbox(polygon_bbox, radius=radius)
            return self._cells_from_um_bbox(
                details.get("overlap_bbox_um"),
                radius=radius,
            )
        return set()

    def _photonic_issue_net_ids(
        self,
        issue: PhotonicVerificationIssue,
    ) -> set[int]:
        by_name = self._net_id_by_name()
        net_ids: set[int] = set()
        if issue.net_name and issue.net_name in by_name:
            net_ids.add(by_name[issue.net_name])
        details = issue.details or {}
        other_net_name = details.get("other_net_name")
        if isinstance(other_net_name, str) and other_net_name in by_name:
            net_ids.add(by_name[other_net_name])
        return net_ids

    def _make_photonic_verification_probe_layout(
        self,
        records: Iterable[RoutedNetRecord],
    ) -> Component:
        t_probe_layout_total_start = self._pipeline_timer_start()
        self.photonic_probe_index += 1
        t_probe_copy_start = self._pipeline_timer_start()
        probe_layout = self.unrouted_layout.copy()
        probe_layout.name = f"photonic_repair_probe_{time.time_ns()}_{self.photonic_probe_index}"
        self._record_pipeline_timing("photonic_probe_copy", t_probe_copy_start)
        t_probe_realize_start = self._pipeline_timer_start()
        realize_routed_net_records(
            probe_layout,
            list(records),
            route_width_um=self.route_width_um,
            route_layer=self.route_layer,
            realization_grid_spec=self.realization_grid_spec,
            allow_45_degree_turns=self.allow_45_degree_turns,
            bend_radius_cells=self.bend_radius_cells,
            crossing_plan_info=self.crossing_plan_info,
            enable_endpoint_correction=self.enable_checked_endpoint_correction,
        )
        self._record_pipeline_timing("photonic_probe_realize", t_probe_realize_start)
        if self.crossing_plan_info.get("enabled"):
            t_probe_crossings_start = self._pipeline_timer_start()
            _place_realized_crossing_components(probe_layout, self.crossing_plan_info)
            self._record_pipeline_timing(
                "photonic_probe_crossing_place",
                t_probe_crossings_start,
            )
        self._record_pipeline_timing(
            "photonic_probe_layout_total",
            t_probe_layout_total_start,
        )
        return probe_layout

    def _refresh_photonic_verification(self) -> PhotonicVerificationResult:
        # This is an internal diagnostic verifier, not the intended default
        # source of truth for production routing success. A* and the grid-level
        # crossing checks must reject illegal moves locally; the final geometry
        # gate is the Python verifier in `routing_flow.py` on the realized
        # layout. Keep this probe available for debugging model mismatches
        # between grid decisions and realized geometry, but do not treat it as
        # a mandatory always-on second full verification pass.
        t_refresh_start = self._pipeline_timer_start()
        records = self.route_bookkeeping.ordered_records()
        probe_layout = self._make_photonic_verification_probe_layout(records)
        self.last_photonic_probe_layout = probe_layout
        self.last_photonic_probe_records = list(records)
        t_verify_start = self._pipeline_timer_start()
        result = verify_photonic_routing(
            probe_layout,
            self.schematic,
            routed_net_records=records,
            unrouted_layout=self.unrouted_layout,
            route_width_um=self.route_width_um,
            route_layer=self.route_layer,
            obstacle_layers=_default_obstacle_layers(
                self.route_layer,
                include_heater_obstacles=self.include_heater_obstacles,
            ),
            realization_grid_spec=self.realization_grid_spec,
            allow_45_degree_turns=self.allow_45_degree_turns,
            bend_radius_cells=self.bend_radius_cells,
            legal_overlap_polygons_by_net_id_pair_um=(
                _legal_crossing_overlap_polygons_for_verification(self.crossing_plan_info)
            ),
            crossing_component_footprints_um=(
                _legal_crossing_component_footprints_for_verification(self.crossing_plan_info)
            ),
            check_route_coverage=self.debug_stop_after_route_index is None,
            check_endpoint_connectivity=self.enable_checked_endpoint_correction,
        )
        self._record_pipeline_timing("photonic_probe_verify", t_verify_start)
        self._record_pipeline_timing("photonic_refresh_total", t_refresh_start)
        return result

    def _repair_final_photonic_issues(
        self,
        issues: tuple[PhotonicVerificationIssue, ...],
    ) -> bool:
        attempts = cast(
            list[dict[str, object]],
            self.crossing_plan_info.setdefault("final_photonic_repair_attempts", []),
        )
        priority_groups: tuple[tuple[str, set[str]], ...] = (
            (
                "endpoint_connection",
                {
                    "endpoint_correction_error",
                    "missing_corrected_centerline",
                    "source_port_not_connected",
                    "target_port_not_connected",
                    "source_endpoint_mismatch",
                    "target_endpoint_mismatch",
                },
            ),
            ("cross_net_waveguide_overlap", {"cross_net_waveguide_overlap"}),
            ("waveguide_obstacle_overlap", {"waveguide_obstacle_overlap"}),
            ("crossing_component_route_overlap", {"crossing_component_route_overlap"}),
        )
        selected_group = next(
            (
                name
                for name, codes in priority_groups
                if any(issue.code in codes for issue in issues)
            ),
            None,
        )
        if selected_group is None:
            return False
        selected_codes = dict(priority_groups)[selected_group]
        selected_issues = [issue for issue in issues if issue.code in selected_codes]
        if selected_group == "crossing_component_route_overlap":
            selected_issues = selected_issues[:1]
        if (
            not selected_issues
            or not self.repair_config.enabled
            or not hasattr(self.router, "add_static_cells")
            or not hasattr(self.router, "ripup_route")
            or not hasattr(self.router, "route_many_with_repair_and_commit")
        ):
            return False

        repair_net_ids_set: set[int] = set()
        keepout_cells: set[tuple[int, int]] = set()
        for issue in selected_issues:
            repair_net_ids_set.update(self._photonic_issue_net_ids(issue))
            keepout_cells.update(self._photonic_issue_keepout_cells(issue))
        repair_net_ids = [net_id for net_id in self.route_order if net_id in repair_net_ids_set]
        attempt: dict[str, object] = {
            "selected_group": selected_group,
            "issue_counts": dict(Counter(issue.code for issue in issues)),
            "selected_issue_counts": dict(Counter(issue.code for issue in selected_issues)),
            "repair_net_ids": [int(net_id) for net_id in repair_net_ids],
            "keepout_cell_count": len(keepout_cells),
        }
        if not repair_net_ids or len(repair_net_ids) > 12:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_repairable_nets_or_too_many"
            attempts.append(attempt)
            return False
        if not keepout_cells:
            attempt["status"] = "skipped"
            attempt["reason"] = "no_keepout_cells"
            attempts.append(attempt)
            return False

        self.router.add_static_cells(sorted(keepout_cells))
        for net_id in repair_net_ids:
            self.router.ripup_route(int(net_id))
            self.route_bookkeeping.clear_route(int(net_id))

        repair_jobs: list[
            tuple[
                int,
                Any,
                Any,
                list[tuple[int, int]],
                list[tuple[int, int]],
                list[tuple[int, int]],
                tuple[float, float] | None,
                tuple[float, float] | None,
            ]
        ] = []
        opened_by_id: dict[int, list[tuple[int, int]]] = {}
        for net_id in repair_net_ids:
            job = self.route_jobs_by_id[net_id]
            source_state, target_state, _, _, opened_cells = self._state_openings_for_job(job)
            clearance_exempt_cells = self._clearance_exempt_cells_for_job(job)
            repair_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                    self._foreign_keepout_cleanup_cells_for_job(job),
                    self._routing_endpoint_center_um(job, source=True),
                    self._routing_endpoint_center_um(job, source=False),
                )
            )
            opened_by_id[int(job.net_id)] = opened_cells

        raw_repair_result = self.router.route_many_with_repair_and_commit(
            repair_jobs,
            self.block_radius_cells,
            self.commit_radius_cells,
            self.core_commit_radius_cells,
            int(self.repair_config.max_rounds),
            int(self.repair_config.max_victims_per_failure),
            float(self.repair_config.history_weight),
            int(self.repair_config.history_increment),
        )
        repair_result = dict(raw_repair_result)
        attempt["router_status"] = str(repair_result.get("status", ""))
        attempt["routed_net_ids"] = [
            int(dict(raw_entry)["net_id"])
            for raw_entry in cast(Iterable[Any], repair_result.get("routes", []))
        ]
        if str(repair_result.get("status", "")) != "routed":
            attempt["status"] = "failed"
            attempts.append(attempt)
            return False

        repaired_net_ids: list[int] = []
        for raw_entry in cast(Iterable[Any], repair_result.get("routes", [])):
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            job = self.route_jobs_by_id[net_id]
            route_obj = entry["route"]
            self._record_route(job, route_obj, opened_by_id[net_id])
            repaired_net_ids.append(net_id)
        if self.enable_checked_endpoint_correction and repaired_net_ids:
            failed_corrections = (
                self._apply_unrestricted_and_fanout_stub_endpoint_corrections_for_net_ids(
                    repaired_net_ids,
                    record_pipeline_timing=False,
                )
            )
            attempt["endpoint_correction_failed_net_ids"] = [
                int(net_id) for net_id in failed_corrections
            ]
        attempt["status"] = "routed"
        attempts.append(attempt)
        return True

    def _photonic_repair_failure_preview(
        self,
        verification: PhotonicVerificationResult,
    ) -> str:
        lines: list[str] = []
        for issue in verification.issues[:5]:
            details = issue.details or {}
            suffix_parts: list[str] = []
            if "overlap_area_um2" in details:
                suffix_parts.append(f"area={details['overlap_area_um2']}")
            if "overlap_bbox_um" in details:
                suffix_parts.append(f"bbox={details['overlap_bbox_um']}")
            suffix = f" ({', '.join(suffix_parts)})" if suffix_parts else ""
            lines.append(f"{issue.code} {issue.net_name or '<unknown>'}: {issue.message}{suffix}")
        if len(verification.issues) > 5:
            lines.append(f"... {len(verification.issues) - 5} more")
        return "; ".join(lines)

    def _refresh_realized_crossing_verification(self) -> list[dict[str, object]]:
        t_refresh_crossings_start = self._pipeline_timer_start()
        t_overlap_start = self._pipeline_timer_start()
        if self.enable_internal_photonic_probe_verification:
            _augment_crossing_plan_with_realized_overlaps(
                router=self.router,
                crossing_plan_info=self.crossing_plan_info,
                routed_records_by_net_id=self.route_bookkeeping.records_by_id,
            )
        self._record_pipeline_timing("realized_crossing_overlap_augment", t_overlap_start)
        native_crossing_events: list[Any] = []
        if hasattr(self.router, "crossing_events"):
            t_native_events_start = self._pipeline_timer_start()
            try:
                native_crossing_events = list(cast(Iterable[Any], self.router.crossing_events()))
            except Exception:
                native_crossing_events = []
            self.crossing_plan_info["native_crossing_events"] = native_crossing_events
            self.crossing_plan_info["native_crossing_event_count"] = len(native_crossing_events)
            self._record_pipeline_timing(
                "realized_crossing_native_events",
                t_native_events_start,
            )
            t_insertion_loss_start = self._pipeline_timer_start()
            _augment_insertion_loss_report(
                crossing_plan_info=self.crossing_plan_info,
                routed_records_by_net_id=self.route_bookkeeping.records_by_id,
                native_crossing_events=native_crossing_events,
            )
            self._record_pipeline_timing(
                "realized_crossing_insertion_loss",
                t_insertion_loss_start,
            )
        t_illegal_crossing_verify_start = self._pipeline_timer_start()
        if self.enable_internal_photonic_probe_verification:
            illegal = _verify_realized_route_intersections(
                crossing_plan_info=self.crossing_plan_info,
                routed_records_by_net_id=self.route_bookkeeping.records_by_id,
                realization_grid_spec=self.realization_grid_spec,
            )
        else:
            illegal = _populate_realized_intersections_from_native_crossing_events(
                crossing_plan_info=self.crossing_plan_info,
                routed_records_by_net_id=self.route_bookkeeping.records_by_id,
                native_crossing_events=native_crossing_events,
                realization_grid_spec=self.realization_grid_spec,
            )
        self._record_pipeline_timing(
            "realized_crossing_verify_intersections",
            t_illegal_crossing_verify_start,
        )
        t_realized_insertion_loss_start = self._pipeline_timer_start()
        _augment_insertion_loss_report_from_realized_intersections(
            crossing_plan_info=self.crossing_plan_info,
            routed_records_by_net_id=self.route_bookkeeping.records_by_id,
        )
        self._record_pipeline_timing(
            "realized_crossing_realized_loss",
            t_realized_insertion_loss_start,
        )
        self._record_pipeline_timing(
            "realized_crossing_refresh_total",
            t_refresh_crossings_start,
        )
        return illegal

    def _build_static_obstacle_context(self) -> tuple[Any, dict[str, Any], Path | None]:
        """Build the static obstacle map, resolve grid/crossing sizing, export debug artifacts if enabled, and validate the Rust backend.

        Sets `self.grid`, `self.resolved_crossing_half_size_cells`, `self.debug_path`,
        `self.diagnostics_enabled`, and `self.route_svgs`. Prints the "Routing N nets..."
        header. Returns the obstacle map, the crossing-device info dict
        `_resolve_crossing_half_size_cells` produces (consumed later when building
        `self.crossing_plan_info`), and the debug obstacle SVG path (or `None` if debug
        output is disabled).
        """
        t_obstacle_start = self._pipeline_timer_start()
        self.resolved_obstacle_config = _resolve_obstacle_config(
            self.obstacle_config,
            route_layer=self.route_layer,
            include_heater_obstacles=self.include_heater_obstacles,
        )
        obstacle_map = build_static_obstacle_map(
            self.unrouted_layout, config=self.resolved_obstacle_config
        )
        self._record_pipeline_timing("obstacle_map", t_obstacle_start)
        if self.debug_timing and self.verbose_route_diagnostics:
            print(
                "      - Obstacle Map time: "
                f"{self.route_nets_timings_s.get('obstacle_map', 0.0):.4f} s"
            )
        self.grid = obstacle_map.grid
        self.resolved_crossing_half_size_cells, crossing_device_info = (
            _resolve_crossing_half_size_cells(
                requested_half_size_cells=int(self.crossing_half_size_cells),
                enable_crossings=bool(self.enable_crossings),
                grid_size_um=float(self.grid.grid_size_um),
                clearance_um=_as_float(
                    getattr(self.resolved_obstacle_config, "clearance_um", 0.0),
                    0.0,
                ),
            )
        )

        self.debug_path = Path(self.debug_dir) if self.debug_dir is not None else None
        self.diagnostics_enabled = self.debug_path is not None
        obstacle_svg = None
        self.route_svgs: list[Path] = []

        if self.debug_path is not None:
            obstacle_dir = self.debug_path / "static_obstacles"
            _ensure_dir(obstacle_dir)
            obstacle_svg = obstacle_dir / f"{self.debug_prefix}_obstacles.svg"
            obstacle_map.export_debug_svg(obstacle_svg)
            route_dir = self.debug_path / "routes"
            if route_dir.exists():
                for old_artifact in route_dir.glob(f"{self.debug_prefix}_*"):
                    if old_artifact.is_file() and old_artifact.suffix.lower() in {".svg", ".txt"}:
                        old_artifact.unlink()

        nets = self.schematic.netlist.routes
        if self.debug_route_indices is None:
            print(f"\nRouting {len(nets)} nets using Rust router...")
        else:
            selected = _format_route_indices(self.debug_route_indices)
            print(
                f"\nRouting {len(nets)} nets using Rust router "
                f"(printing/exporting route SVGs for indices: {selected})..."
            )

        if not hasattr(self.rust_backend, "PyPhotonicRouter"):
            raise RuntimeError(
                "Rust backend does not expose PyPhotonicRouter. "
                "Rebuild/install the Rust extension with the class-based API."
            )

        return obstacle_map, crossing_device_info, obstacle_svg

    def _configure_router_and_grid(self, obstacle_map: Any) -> None:
        """Build the Rust grid/primitive/A* configs, construct `self.router`, and extract raw static geometry.

        Tunes `self.astar_cfg` (JPS4, heuristic mode/weight, bend weight, heap tie-breaker,
        proactive congestion, routing-window sizing) based on `self.allow_45_degree_turns`,
        `self.crossing_mode`, and related settings, resolves `self.clearance_policy` and the
        block/commit/core radii, constructs `self.router` (`PyPhotonicRouter`), and extracts
        `self.raw_static_cells`/`self.raw_static_rects_for_openings`/
        `self.heater_opening_rects_for_openings` from `obstacle_map` for later phases to
        consume when computing port openings and keepouts.
        """
        t_router_setup_start = self._pipeline_timer_start()
        self.origin_x_um, self.origin_y_um = _grid_origin_xy(self.grid)
        grid_spec = self.rust_backend.GridSpec(
            int(self.grid.width),
            int(self.grid.height),
            float(self.grid.grid_size_um),
            self.origin_x_um,
            self.origin_y_um,
        )
        self.bend_radius_cells = bend_radius_cells_from_um(
            self.bend_radius_um,
            grid_size_um=float(self.grid.grid_size_um),
        )
        self.primitive_cfg = self.rust_backend.PrimitiveLibraryConfig(
            grid_size_um=float(self.grid.grid_size_um),
            bend_radius_cells=self.bend_radius_cells,
            allow_45_degree_turns=self.allow_45_degree_turns,
        )
        self.bend_radius_cells = int(self.primitive_cfg.bend_radius_cells)
        self.astar_cfg = self.rust_backend.AStarConfig(max_iterations=int(self.max_iterations))
        self.astar_cfg.enable_simple_routes = bool(self.enable_simple_routes)
        self.astar_cfg.enable_jps4 = bool(self.enable_jps4)
        self.astar_cfg.use_indexed_heap = bool(self.use_indexed_heap or self.allow_45_degree_turns)
        self.astar_cfg.collect_detailed_timing = bool(
            self.debug_timing or self.collect_route_stats or self.collect_attempt_diagnostics
        )
        self.astar_cfg.primitive_ordering = str(self.primitive_ordering)
        effective_heuristic_mode = str(self.heuristic_mode)
        if self.allow_45_degree_turns and effective_heuristic_mode == "heading_aware":
            effective_heuristic_mode = "diagonal_aware"
        self.astar_cfg.heuristic_mode = effective_heuristic_mode
        collision_crossing_mode = bool(self.enable_crossings) and self.crossing_mode in {
            "collision",
            "lidar-pure",
        }
        min_heuristic_weight = float(
            os.environ.get("PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT", "1.0")
        )
        if (
            self.allow_45_degree_turns
            and not collision_crossing_mode
            and min_heuristic_weight > 1.0
            and hasattr(self.astar_cfg, "max_iterations")
        ):
            # This cap was sized for Weighted A* (min weight > 1.0), which
            # reaches targets in few expansions; admissible search (1.0)
            # legitimately needs the caller's full iteration budget
            # (heater_s_mod's mmi_extra_3->mmi_extra_4 net exhausts 50k).
            self.astar_cfg.max_iterations = min(int(self.astar_cfg.max_iterations), 50_000)
        if self.allow_45_degree_turns and hasattr(self.astar_cfg, "heuristic_weight"):
            # 1.0 is admissible A*: cheapest paths, no weighted-search
            # geometry artifacts (chicanes, overshoot detours), and the only
            # weight at which multiportmmi_16x16 routes completely. 1.25 is
            # the old Weighted A* behavior (fewer expanded states, up to 25%
            # suboptimal paths); benes_16x16 still pins it in its
            # STABLE_ROUTING_ENV until its no-candidate endpoint-correction
            # hole is fixed -- see
            # .agent/execplans/2026-08-31-admissible-astar-heuristic-45-degree.md.
            self.astar_cfg.heuristic_weight = max(
                float(self.astar_cfg.heuristic_weight), min_heuristic_weight
            )
        if self.allow_45_degree_turns and hasattr(self.astar_cfg, "bend_weight"):
            # A bend costs bend_weight per angle-eighth of turn (see
            # src/primitives.rs's bend_cost field); boosting it keeps 45-degree
            # A* from spending work on short zig-zag variants. This applies
            # uniformly regardless of crossing_mode -- crossing-awareness only
            # changes whether/what a *crossing* costs (see crossing_loss,
            # translation/route_rust_crossing_plan.py), not how bends are priced.
            # Experiment gate for the heuristics audit
            # (.agent/execplans/2026-08-31-heuristics-inventory.md): the
            # magnitude 12.0 has no recorded derivation; the env var lets the
            # neutral-value ladder run without a code edit. Default unchanged.
            min_bend_weight = float(os.environ.get("PHOTONIC_ROUTER_MIN_BEND_WEIGHT", "12.0"))
            self.astar_cfg.bend_weight = max(
                float(self.astar_cfg.bend_weight), min_bend_weight
            )
        effective_heap_tie_breaker = str(self.heap_tie_breaker)
        if self.allow_45_degree_turns and effective_heap_tie_breaker == "smaller_g":
            effective_heap_tie_breaker = "larger_g"
        # Experiment gate (same audit): the 45-degree-only tie-break flip is
        # unjustified in-repo; an explicit env value overrides the outcome.
        env_tie_breaker = os.environ.get("PHOTONIC_ROUTER_HEAP_TIE_BREAKER", "").strip()
        if env_tie_breaker in {"smaller_g", "larger_g"}:
            effective_heap_tie_breaker = env_tie_breaker
        self.astar_cfg.heap_tie_breaker = effective_heap_tie_breaker
        if hasattr(self.astar_cfg, "proactive_congestion_weight"):
            self.astar_cfg.proactive_congestion_weight = float(self.proactive_congestion_weight)
        if hasattr(self.astar_cfg, "proactive_congestion_radius_cells"):
            self.astar_cfg.proactive_congestion_radius_cells = int(
                self.proactive_congestion_radius_cells
            )
        if self.routing_window_scale is not None:
            self.astar_cfg.routing_window_scale = float(self.routing_window_scale)

        self.route_clearance_um = max(
            0.0,
            _as_float(getattr(self.resolved_obstacle_config, "clearance_um", 0.0), 0.0),
        )
        self.clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
            route_width_um=float(self.route_width_um),
            grid_size_um=float(self.grid.grid_size_um),
            route_clearance_um=self.route_clearance_um,
        )
        self.block_radius_cells = (
            self.clearance_policy.dynamic_obstacle_search_expansion_radius_cells
        )
        self.commit_radius_cells = self.clearance_policy.dynamic_route_commit_keepout_radius_cells
        self.core_commit_radius_cells = self.clearance_policy.dynamic_route_core_radius_cells
        routing_window_min_margin_cells = max(
            int(getattr(self.astar_cfg, "routing_window_min_margin_cells", 12)),
            int((2 * self.bend_radius_cells) + self.commit_radius_cells + 2),
        )
        self.astar_cfg.routing_window_min_margin_cells = max(
            int(getattr(self.astar_cfg, "routing_window_min_margin_cells", 12)),
            routing_window_min_margin_cells,
        )
        self.astar_cfg.simple_route_max_offset_cells = max(
            int(getattr(self.astar_cfg, "simple_route_max_offset_cells", 96)),
            int(12 * self.bend_radius_cells + 2 * self.commit_radius_cells),
        )
        self.router = self.rust_backend.PyPhotonicRouter(
            grid_spec, self.primitive_cfg, self.astar_cfg
        )
        if hasattr(self.router, "set_route_width_um"):
            # The commit validation's parallel-overlap check needs the
            # physical waveguide width (see py_router.rs `route_width_um`).
            self.router.set_route_width_um(float(self.route_width_um))
        self._record_pipeline_timing("router_setup", t_router_setup_start)

        self.port_lane_length_cells = max(3, 2 * self.bend_radius_cells + 2)
        self.port_lane_half_width_cells = max(
            1, self.bend_radius_cells + self.commit_radius_cells + 1
        )
        # Separate override scoped to dense-source-fanout instances only
        # (`_is_dense_source_fanout_instance`) -- e.g. a multi-port MMI
        # splitter with many stubbed ports stacked close together, where the
        # flat per-port reservation above overlaps heavily and merges into
        # one large blocked region. Every *other* port (e.g. a heater port
        # on the same benchmark, confirmed via a real regression to need
        # the full default margin for its own endpoint correction) is
        # unaffected regardless of these values, so this stays scoped
        # rather than becoming a second general default.
        #
        # Both default to `0` (no reservation at all): at a stub, the
        # runway/keepout exists to protect a port whose own waveguide isn't
        # committed yet, but a dense-source-fanout stub's waveguide *is*
        # already committed static geometry by the time this reservation
        # would matter. A crossing (or anything else) placed directly at a
        # stub's exit cell is governed by the crossing-legality rules
        # (`crossing_half_size_cells`, `min_straight_cells_per_crossing`),
        # not by this reservation, so there is nothing left for it to
        # protect at stubs specifically. Validated on `multiportmmi_8x8`'s
        # stable baseline with both knobs at `0` (previously `half_width=2`,
        # `length=port_lane_length_cells`): still routes 111/111, 0 errors,
        # 0 self-intersections, 0 cross-net overlaps.
        self.stub_port_lane_length_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH_CELLS",
            0,
        )
        self.stub_port_lane_half_width_cells = self._env_nonnegative_int(
            "PHOTONIC_ROUTER_STUB_PORT_LANE_HALF_WIDTH_CELLS",
            0,
        )

        port_open_radius_um = _as_float(
            getattr(self.resolved_obstacle_config, "port_open_radius_um", 0.5),
            0.5,
        )

        raw_blocked_obj: object
        if hasattr(obstacle_map, "raw_blocked_cells"):
            raw_blocked_obj = getattr(obstacle_map, "raw_blocked_cells")
        else:
            raw_blocked_obj = obstacle_map.blocked_cells
        raw_blocked_cells = cast(Iterable[tuple[int, int]], raw_blocked_obj)
        self.raw_static_cells = {(int(cell[0]), int(cell[1])) for cell in raw_blocked_cells}
        self.static_blocked_cells_before_port_reservations = self.raw_static_cells
        self.raw_static_rects_for_openings: list[tuple[int, int, int, int]] = []
        if hasattr(obstacle_map, "raw_static_rects"):
            for rect in cast(
                Iterable[tuple[int, int, int, int]],
                getattr(obstacle_map, "raw_static_rects"),
            ):
                if len(rect) == 4:
                    self.raw_static_rects_for_openings.append(
                        (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                    )
        blocked_static_rects_for_openings: list[tuple[int, int, int, int]] = []
        if hasattr(obstacle_map, "blocked_static_rects"):
            for rect in cast(
                Iterable[tuple[int, int, int, int]],
                getattr(obstacle_map, "blocked_static_rects"),
            ):
                if len(rect) == 4:
                    blocked_static_rects_for_openings.append(
                        (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                    )
        self.heater_opening_rects_for_openings = (
            self.raw_static_rects_for_openings + blocked_static_rects_for_openings
        )
        self.grid_width = int(self.grid.width)
        self.grid_height = int(self.grid.height)
        self.raw_static_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
        self.heater_opening_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
        self.raw_static_cells_by_y: dict[int, set[int]] | None = None

    def _build_route_jobs_and_fanout_clustering(
        self, nets: Mapping[str, Any]
    ) -> tuple[
        list[RouteJob],
        dict[str, set[str]],
        dict[str, int],
    ]:
        """Build the per-net `RouteJob` list from the schematic netlist and compute fanout/dense-port clustering.

        Populates `self.endpoint_ports_by_spec`, `self.source_port_specs_by_instance`
        (and its angle-keyed variant), `self.fanout_anchor_by_port_spec` and the related
        fanout-stub cell sets, and `self.dense_source_cluster_specs_by_port_spec` (which
        source ports on a dense multi-port instance share a lateral cluster). Returns the
        route jobs plus two locals the next phase (port-opening/footprint computation)
        still needs: `endpoint_port_specs_by_instance` and
        `dense_port_runway_length_by_spec` (the merged source+target runway-length map).
        """
        route_jobs: list[RouteJob] = []
        self.endpoint_ports_by_spec: dict[str, tuple[str, str, Port]] = {}
        endpoint_port_specs_by_instance: dict[str, set[str]] = {}
        self.port_access_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
        self.port_access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
        self.port_runway_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
        self.port_access_rule_by_spec: dict[str, str | None] = {}
        next_net_id = 1
        t_route_job_build_start = self._pipeline_timer_start()
        for net_name, bundle in nets.items():
            links = bundle.links
            for port1_spec, port2_spec in links.items():
                inst1, port1 = port1_spec.split(",")
                inst2, port2 = port2_spec.split(",")
                source_port = get_port_from_instance(self.routed_layout, inst1, port1)
                target_port = get_port_from_instance(self.routed_layout, inst2, port2)
                route_jobs.append(
                    RouteJob(
                        net_id=next_net_id,
                        route_index=next_net_id,
                        net_name=net_name,
                        inst1=inst1,
                        port1=port1,
                        inst2=inst2,
                        port2=port2,
                        source_port=source_port,
                        target_port=target_port,
                    )
                )
                next_net_id += 1
                self.endpoint_ports_by_spec.setdefault(port1_spec, (inst1, port1, source_port))
                self.endpoint_ports_by_spec.setdefault(port2_spec, (inst2, port2, target_port))
                endpoint_port_specs_by_instance.setdefault(inst1, set()).add(port1_spec)
                endpoint_port_specs_by_instance.setdefault(inst2, set()).add(port2_spec)
        self._record_pipeline_timing("route_job_build", t_route_job_build_start)

        self.source_port_specs_by_instance: dict[str, set[str]] = {}
        self.source_port_specs_by_instance_angle: dict[tuple[str, int], set[str]] = {}
        self.target_port_specs_by_instance: dict[str, set[str]] = {}
        self.target_port_specs_by_instance_angle: dict[tuple[str, int], set[str]] = {}
        for run_job in route_jobs:
            port_spec = f"{run_job.inst1},{run_job.port1}"
            self.source_port_specs_by_instance.setdefault(run_job.inst1, set()).add(port_spec)
            angle = self._orientation_to_angle(
                getattr(run_job.source_port, "orientation", None),
                flip=False,
            )
            self.source_port_specs_by_instance_angle.setdefault(
                (run_job.inst1, int(angle)), set()
            ).add(port_spec)
            target_port_spec = f"{run_job.inst2},{run_job.port2}"
            self.target_port_specs_by_instance.setdefault(run_job.inst2, set()).add(
                target_port_spec
            )
            target_angle = self._orientation_to_angle(
                getattr(run_job.target_port, "orientation", None),
                flip=False,
            )
            self.target_port_specs_by_instance_angle.setdefault(
                (run_job.inst2, int(target_angle)), set()
            ).add(target_port_spec)

        self.fanout_anchor_by_port_spec = {
            **self._build_static_fanout_anchors(),
            **self._build_static_fanout_target_anchors(),
        }
        if os.environ.get("PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE"):
            traced_instance = os.environ["PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE"]
            for port_spec, anchor in sorted(self.fanout_anchor_by_port_spec.items()):
                if port_spec.startswith(f"{traced_instance},"):
                    print(
                        f"anchor_trace {port_spec} state=({anchor.state_x},{anchor.state_y}) "
                        f"angle={anchor.physical_angle} "
                        f"centerline={anchor.stub_centerline_um}"
                    )
        self.fanout_stub_static_cells_by_spec: dict[str, set[tuple[int, int]]] = {
            port_spec: self._inflated_cells(anchor.stub_center_cells, int(self.commit_radius_cells))
            for port_spec, anchor in self.fanout_anchor_by_port_spec.items()
        }
        self.fanout_stub_center_cells: set[tuple[int, int]] = set()
        for anchor in self.fanout_anchor_by_port_spec.values():
            self.fanout_stub_center_cells.update(anchor.stub_center_cells)
        self.fanout_stub_static_cells: set[tuple[int, int]] = set()
        for cells in self.fanout_stub_static_cells_by_spec.values():
            self.fanout_stub_static_cells.update(cells)
        self.fanout_anchor_net_ids = {
            int(job.net_id)
            for job in route_jobs
            if f"{job.inst1},{job.port1}" in self.fanout_anchor_by_port_spec
            or f"{job.inst2},{job.port2}" in self.fanout_anchor_by_port_spec
        }
        self.fanout_anchor_source_net_ids = {
            int(job.net_id)
            for job in route_jobs
            if f"{job.inst1},{job.port1}" in self.fanout_anchor_by_port_spec
        }
        self.fanout_anchor_target_net_ids = {
            int(job.net_id)
            for job in route_jobs
            if f"{job.inst2},{job.port2}" in self.fanout_anchor_by_port_spec
        }

        self.dense_source_port_runway_length_by_spec = self._dense_source_port_runway_lengths(
            route_jobs
        )
        self.dense_target_port_runway_length_by_spec = self._dense_target_port_runway_lengths(
            route_jobs
        )
        if os.environ.get("PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE"):
            traced_instance = os.environ["PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE"]
            for port_spec, runway_length in sorted(
                self.dense_target_port_runway_length_by_spec.items()
            ):
                if port_spec.startswith(f"{traced_instance},"):
                    print(f"runway_trace target {port_spec} length={runway_length}")
            for port_spec, runway_length in sorted(
                self.dense_source_port_runway_length_by_spec.items()
            ):
                if port_spec.startswith(f"{traced_instance},"):
                    print(f"runway_trace source {port_spec} length={runway_length}")
        dense_port_runway_length_by_spec: dict[str, int] = dict(
            self.dense_source_port_runway_length_by_spec
        )
        for port_spec, runway_length in self.dense_target_port_runway_length_by_spec.items():
            existing_length = dense_port_runway_length_by_spec.get(port_spec)
            dense_port_runway_length_by_spec[port_spec] = max(
                int(existing_length) if existing_length is not None else 0,
                int(runway_length),
            )
        self.dense_source_cluster_specs_by_port_spec: dict[str, set[str]] = {}
        index = 0
        while index < len(route_jobs):
            job = route_jobs[index]
            if f"{job.inst1},{job.port1}" not in self.dense_source_port_runway_length_by_spec:
                index += 1
                continue
            run_end = index + 1
            while (
                run_end < len(route_jobs)
                and route_jobs[run_end].inst1 == job.inst1
                and f"{route_jobs[run_end].inst1},{route_jobs[run_end].port1}"
                in self.dense_source_port_runway_length_by_spec
            ):
                run_end += 1
            cluster_specs = {
                f"{run_job.inst1},{run_job.port1}"
                for run_job in route_jobs[index:run_end]
                if f"{run_job.inst1},{run_job.port1}"
                in self.dense_source_port_runway_length_by_spec
            }
            if len(cluster_specs) > 1:
                for port_spec in cluster_specs:
                    self.dense_source_cluster_specs_by_port_spec[port_spec] = set(cluster_specs)
            index = run_end
        if self.fanout_anchor_by_port_spec:
            static_stub_groups: dict[tuple[str, int], set[str]] = {}
            for port_spec, anchor in self.fanout_anchor_by_port_spec.items():
                instance_name = port_spec.split(",", 1)[0]
                static_stub_groups.setdefault(
                    (instance_name, int(anchor.physical_angle) % 8),
                    set(),
                ).add(port_spec)
            for cluster_specs in static_stub_groups.values():
                if len(cluster_specs) <= 1:
                    continue
                for port_spec in cluster_specs:
                    self.dense_source_cluster_specs_by_port_spec[port_spec] = set(cluster_specs)

        return route_jobs, endpoint_port_specs_by_instance, dense_port_runway_length_by_spec

    def _build_crossing_plan_and_port_footprints(
        self,
        route_jobs: list[RouteJob],
        endpoint_port_specs_by_instance: dict[str, set[str]],
        dense_port_runway_length_by_spec: dict[str, int],
        obstacle_map: Any,
        crossing_device_info: dict[str, Any],
    ) -> tuple[list[RouteJob], dict[str, set[tuple[int, int]]]]:
        """Build `self.crossing_plan_info`, batch port-opening/footprint and foreign-keepout cells, and assign dense-port lane geometry.

        Enables collision-crossing routing on `self.router` if configured. Populates
        `self.port_access_cells_by_spec`/`self.port_access_candidate_cells_by_spec`/
        `self.port_runway_cells_by_spec` (one raw footprint per port, from the single
        unified sizing computation), `self.foreign_port_keepout_cells_by_spec`,
        `self.dense_port_lateral_windows`/`self.dense_port_lateral_owner_groups` (lateral
        room allocated to each port in a dense multi-port group sharing a facing angle),
        and `self.port_state_lane_offsets` (grid-lane spreading for endpoints that would
        otherwise share the exact same source/target cell and angle). Returns `route_jobs`
        (unchanged by this phase, passed straight through) and
        `foreign_port_keepout_cells_by_instance`, which the next phase needs to build the
        static-cell handoff to Rust.
        """
        port_rule_extra_open_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
        t_crossing_context_start = self._pipeline_timer_start()
        self.crossing_plan_info = _build_crossing_plan_info(
            rust_backend=self.rust_backend,
            router=self.router,
            schematic=self.schematic,
            route_jobs=route_jobs,
            enable_crossings=self.enable_crossings,
            crossing_mode=self.crossing_mode,
            node_depths=self.node_depths,
            node_ranks=self.node_ranks,
            edge_ranks=self.edge_ranks,
            crossing_loss=float(self.crossing_loss),
            crossing_search_loss=float(self.crossing_search_loss),
            crossing_half_size_cells=int(self.resolved_crossing_half_size_cells),
            min_straight_cells_per_crossing=int(self.min_straight_cells_per_crossing),
            allow_only_expected_crossings=self.effective_allow_only_expected_crossings,
        )
        self.crossing_plan_info["crossing_mode"] = self.crossing_mode
        self.crossing_plan_info["requested_allow_only_expected_crossings"] = bool(
            self.allow_only_expected_crossings
        )
        self.crossing_plan_info["bend_runout_cells_per_crossing"] = int(self.bend_radius_cells)
        self.crossing_plan_info["fanout_stub_bend_degrees"] = 45 * int(
            self._env_fanout_stub_bend_steps()
        )
        self.crossing_plan_info["required_straight_margin_cells_per_crossing"] = int(
            self.resolved_crossing_half_size_cells
        ) + int(self.bend_radius_cells)
        self.crossing_plan_info["fanout_access_mode"] = self.fanout_access_mode_normalized
        self.crossing_plan_info["fanout_anchor_port_count"] = len(self.fanout_anchor_by_port_spec)
        self.crossing_plan_info["fanout_anchor_net_ids"] = sorted(self.fanout_anchor_net_ids)
        self.crossing_plan_info["fanout_anchor_source_net_ids"] = sorted(
            self.fanout_anchor_source_net_ids
        )
        self.crossing_plan_info["fanout_anchor_target_net_ids"] = sorted(
            self.fanout_anchor_target_net_ids
        )
        self.crossing_plan_info["fanout_stub_center_cell_count"] = len(
            self.fanout_stub_center_cells
        )
        self.crossing_plan_info["fanout_stub_static_cell_count"] = len(
            self.fanout_stub_static_cells
        )
        self.crossing_plan_info["fanout_stub_centerlines_um"] = [
            {
                "port_spec": anchor.port_spec,
                "anchor_cell": [int(anchor.state_x), int(anchor.state_y)],
                "physical_angle": int(anchor.physical_angle) % 8,
                "centerline_um": [
                    [float(point[0]), float(point[1])] for point in anchor.stub_centerline_um
                ],
            }
            for anchor in sorted(
                self.fanout_anchor_by_port_spec.values(),
                key=lambda item: item.port_spec,
            )
        ]
        self.crossing_plan_info["crossing_device"] = crossing_device_info
        if bool(self.enable_crossings) and self.crossing_mode in {"collision", "lidar-pure"}:
            if not hasattr(self.router, "set_collision_crossing_routing"):
                extension_path = getattr(self.rust_backend, "__file__", "<unknown>")
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.set_collision_crossing_routing. Rebuild it with "
                    "`maturin develop --release`. "
                    f"Loaded extension: {extension_path}"
                )
            self.router.set_collision_crossing_routing(True)
        elif hasattr(self.router, "set_collision_crossing_routing"):
            self.router.set_collision_crossing_routing(False)
        self._record_pipeline_timing("crossing_context", t_crossing_context_start)

        if not hasattr(self.router, "build_port_footprint_cells"):
            extension_path = getattr(self.rust_backend, "__file__", "<unknown>")
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.build_port_footprint_cells. Rebuild it with "
                "`maturin develop --release`. "
                f"Loaded extension: {extension_path}"
            )

        t_port_opening_prep_start = self._pipeline_timer_start()
        port_opening_inputs: list[
            tuple[str, float, float, float | None, str | None, float | None, float | None]
        ] = []
        for port_spec, (instance_name, port_name, port) in self.endpoint_ports_by_spec.items():
            fanout_anchor = self.fanout_anchor_by_port_spec.get(port_spec)
            center = fanout_anchor.center_um if fanout_anchor is not None else _port_center_um(port)
            if center is None:
                raise ValueError(f"Port {port_spec!r} has no finite center coordinate")
            orientation_value = getattr(port, "orientation", None)
            orientation = None if orientation_value is None else float(orientation_value)
            port_type = _port_type_name(port)
            rule = self._port_access_rule_for(
                instance_name=instance_name,
                port_name=port_name,
                port=port,
            )
            access_length_um = None if rule is None else float(rule.access_length_um)
            access_width_um = None if rule is None else float(rule.access_width_um)
            self.port_access_rule_by_spec[port_spec] = (
                None if rule is None else rule.component_name_pattern
            )
            port_rule_extra_open_cells_by_spec[port_spec] = (
                self._instance_static_geometry_open_cells(
                    instance_name=instance_name,
                    center_um=(float(center[0]), float(center[1])),
                    orientation=orientation,
                )
                if rule is not None and rule.opens_instance_static_geometry
                else set()
            )
            port_opening_inputs.append(
                (
                    port_spec,
                    float(center[0]),
                    float(center[1]),
                    orientation,
                    port_type,
                    access_length_um,
                    access_width_um,
                )
            )
        self._record_pipeline_timing("port_opening_prep", t_port_opening_prep_start)

        t_port_opening_batch_start = self._pipeline_timer_start()

        # One raw footprint per port, from the single unified sizing computation
        # (Milestone 1's _resolve_port_footprint_cells plus the dense-runway
        # length override, matching the precedence the old code already used:
        # an explicit custom access rule wins over a dense-runway override).
        # Self-opening and foreign-keepout both derive from this same dict
        # below, so they can never drift out of sync with each other again --
        # see .agent/execplans/2026-08-18-unify-port-access-region-computation.md.
        raw_footprint_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
        port_footprint_inputs: list[tuple[str, float, float, float | None, int, int]] = []
        for item in port_opening_inputs:
            port_spec = str(item[0])
            raw_footprint_cells_by_spec[port_spec] = set()
            _spec, x_um, y_um, orientation, port_type, access_length_um, access_width_um = item
            instance_name, port_name, port = self.endpoint_ports_by_spec[port_spec]
            custom_access = access_length_um is not None or access_width_um is not None
            if not custom_access and port_type is not None and str(port_type) != "optical":
                continue
            length_cells, half_width_cells = self._resolve_port_footprint_cells(
                instance_name=instance_name,
                port_name=port_name,
                port=port,
            )
            custom_runway_length = dense_port_runway_length_by_spec.get(port_spec)
            if custom_runway_length is not None and not custom_access:
                length_cells = max(1, int(custom_runway_length))
            port_footprint_inputs.append(
                (
                    port_spec,
                    float(x_um),
                    float(y_um),
                    orientation,
                    int(length_cells),
                    int(half_width_cells),
                )
            )

        for port_spec, raw_cells in self.router.build_port_footprint_cells(port_footprint_inputs):
            raw_footprint_cells_by_spec[str(port_spec)] = {
                (int(cell[0]), int(cell[1])) for cell in raw_cells
            }

        for port_spec, raw_footprint_cells in raw_footprint_cells_by_spec.items():
            open_cells = raw_footprint_cells - self._cells_in_raw_static_geometry(
                raw_footprint_cells
            )
            self.port_access_cells_by_spec[port_spec] = set(open_cells)
            self.port_access_candidate_cells_by_spec[port_spec] = set(raw_footprint_cells)
            extra_open_cells = port_rule_extra_open_cells_by_spec.get(port_spec, set())
            if extra_open_cells:
                self.port_access_cells_by_spec[port_spec].update(extra_open_cells)
                self.port_access_candidate_cells_by_spec[port_spec].update(extra_open_cells)
            self.port_runway_cells_by_spec[port_spec] = set(raw_footprint_cells)
        self._record_pipeline_timing("port_opening_batch", t_port_opening_batch_start)

        # Foreign-keepout reuses the exact same raw footprint each port already
        # got for its own opening above -- a keepout region is the same shape,
        # just unfiltered (no must-stay-blocked subtraction), since it exists to
        # say "nothing else may enter here," not to describe what this port's
        # own net may route through. foreign_port_keepout_cells now only gates
        # whether keepout logic runs at all; it no longer controls size.
        self.foreign_port_keepout_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
        foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
        foreign_port_keepout_nonstatic_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
        if self.foreign_port_keepout_cells > 0:
            t_foreign_keepout_start = self._pipeline_timer_start()
            for port_spec, raw_footprint_cells in raw_footprint_cells_by_spec.items():
                instance_name = port_spec.split(",", 1)[0]
                cells_for_spec = set(raw_footprint_cells)
                self.foreign_port_keepout_cells_by_spec[port_spec] = cells_for_spec
                foreign_port_keepout_cells_by_instance.setdefault(instance_name, set()).update(
                    cells_for_spec
                )
                nonstatic_cells_for_spec = cells_for_spec - self._cells_in_raw_static_geometry(
                    cells_for_spec
                )
                foreign_port_keepout_nonstatic_cells_by_instance.setdefault(
                    instance_name,
                    set(),
                ).update(nonstatic_cells_for_spec)
            self._record_pipeline_timing("foreign_port_keepout_batch", t_foreign_keepout_start)

        self.dense_port_lateral_windows: dict[str, tuple[float, float, float, float, float]] = {}
        self.dense_port_lateral_owner_groups: dict[
            str,
            tuple[float, float, tuple[tuple[str, float], ...]],
        ] = {}
        for instance_name, port_specs in endpoint_port_specs_by_instance.items():
            if len(port_specs) < self._dense_fanout_min_ports():
                continue
            groups: dict[int, list[tuple[str, float]]] = {}
            for port_spec in port_specs:
                _inst, _port_name, port = self.endpoint_ports_by_spec[port_spec]
                fanout_anchor = self.fanout_anchor_by_port_spec.get(port_spec)
                angle = (
                    int(fanout_anchor.physical_angle) % 8
                    if fanout_anchor is not None
                    else self._orientation_to_angle(getattr(port, "orientation", None), flip=False)
                )
                step_x, step_y = self._angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x
                center = (
                    fanout_anchor.center_um if fanout_anchor is not None else _port_center_um(port)
                )
                if center is None or (lateral_x == 0 and lateral_y == 0):
                    continue
                lateral_position = float(center[0]) * lateral_x + float(center[1]) * lateral_y
                groups.setdefault(angle, []).append((port_spec, lateral_position))
            for angle, group in groups.items():
                if len(group) <= 1:
                    continue
                step_x, step_y = self._angle_to_step(angle)
                lateral_x, lateral_y = -step_y, step_x
                ordered = sorted(group, key=lambda item: item[1])
                owner_group = tuple(ordered)
                for owned_port_spec, _lateral_position in ordered:
                    self.dense_port_lateral_owner_groups[owned_port_spec] = (
                        float(lateral_x),
                        float(lateral_y),
                        owner_group,
                    )
                for index, (port_spec, lateral_position) in enumerate(ordered):
                    previous_position = ordered[index - 1][1] if index > 0 else None
                    next_position = ordered[index + 1][1] if index + 1 < len(ordered) else None
                    if previous_position is None and next_position is None:
                        continue
                    if previous_position is None:
                        gap = abs(next_position - lateral_position)
                        lower = lateral_position - gap * 0.5
                    else:
                        lower = (previous_position + lateral_position) * 0.5
                    if next_position is None:
                        gap = abs(lateral_position - previous_position)
                        upper = lateral_position + gap * 0.5
                    else:
                        upper = (lateral_position + next_position) * 0.5
                    lane_margin_um = 0.0
                    self.dense_port_lateral_windows[port_spec] = (
                        float(lateral_x),
                        float(lateral_y),
                        float(lower),
                        float(upper),
                        lane_margin_um,
                    )

        self.normal_port_runway_cells: set[tuple[int, int]] = set()
        for cells in self.port_runway_cells_by_spec.values():
            self.normal_port_runway_cells.update(cells)

        endpoint_ports_by_key: dict[tuple[int, int, int], list[tuple[str, bool, Port]]] = {}
        for job in route_jobs:
            for port_spec, port, as_target in (
                (f"{job.inst1},{job.port1}", job.source_port, False),
                (f"{job.inst2},{job.port2}", job.target_port, True),
            ):
                state = self._endpoint_state_for_lane_assignment(port, as_target=as_target)
                key = (int(state.x), int(state.y), int(state.angle) % 8)
                endpoint_ports_by_key.setdefault(key, []).append((port_spec, as_target, port))

        self.port_state_lane_offsets: dict[tuple[str, bool], tuple[int, int]] = {}
        for (base_x, base_y, angle), endpoints in endpoint_ports_by_key.items():
            unique_endpoints = list(
                dict.fromkeys((spec, is_target) for spec, is_target, _ in endpoints)
            )
            if len(unique_endpoints) <= 1:
                continue
            step_x, step_y = self._angle_to_step(angle)
            lateral_x, lateral_y = -step_y, step_x
            if lateral_x == 0 and lateral_y == 0:
                continue

            def _lateral_position_for_lane_assignment(item: tuple[str, bool, Port]) -> float:
                center = _port_center_um(item[2])
                if center is None:
                    return 0.0
                return center[0] * lateral_x + center[1] * lateral_y

            sorted_endpoints = sorted(endpoints, key=_lateral_position_for_lane_assignment)
            seen_endpoint_keys: set[tuple[str, bool]] = set()
            lane_index = 0
            for port_spec, is_target, _ in sorted_endpoints:
                endpoint_key = (port_spec, is_target)
                if endpoint_key in seen_endpoint_keys:
                    continue
                seen_endpoint_keys.add(endpoint_key)
                candidate_x = base_x + lateral_x * lane_index
                candidate_y = base_y + lateral_y * lane_index
                if self._in_bounds(candidate_x, candidate_y):
                    self.port_state_lane_offsets[endpoint_key] = (
                        lateral_x * lane_index,
                        lateral_y * lane_index,
                    )
                lane_index += 1

        return route_jobs, foreign_port_keepout_cells_by_instance

    def _finalize_route_jobs_and_static_handoff(
        self,
        route_jobs: list[RouteJob],
        foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]],
        obstacle_map: Any,
    ) -> tuple[list[RouteJob], float]:
        """Order route jobs, hand static/keepout geometry off to Rust, apply debug-limit slicing, and prepare repair/timing bookkeeping.

        Applies `self._topological_net_route_order` when repair is enabled
        (reordering only helps a negotiation loop that can recover from a
        bad order; with repair disabled -- e.g. `test_benchmarks_route_with_astar_only`,
        which isolates plain A* on purpose -- a net that ends up later in a
        changed order can collide irrecoverably with an already-committed
        net that plain A* has no way to rip up, so declaration order is left
        untouched in that mode), aggregates static keepout cells
        (port runways, foreign-port keepouts, fanout stubs) and hands them to the Rust
        router via `set_static_rects`/`set_static_cells`/`add_static_cells`, applies
        `debug_stop_after_route_index`/`PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT` slicing to
        produce the final `route_jobs` for this run (also recording the pre-slicing job
        list as `self.full_route_jobs_by_route_index`), builds `self.repair_config`,
        `self.route_jobs_by_id`, `self.route_order`, and `self.route_bookkeeping`,
        initializes the route-search stats/timing-bucket attributes, and precomputes
        `self.batch_clearance_exempt_cells_by_id` via a batched Rust call. Returns the
        final, post-slicing `route_jobs` list used for dispatch, and `t_astar_start`
        (the `time.perf_counter()` reading taken before the precompute steps in this
        method, not after it returns) so the caller's post-dispatch `astar_elapsed_s`
        measurement still covers this method's own precompute time, exactly as it did
        before this method existed as a separate call.
        """
        repair_enabled = (self.ripup_reroute_config or RipupRerouteConfig()).enabled
        if repair_enabled:
            route_jobs = self._topological_net_route_order(route_jobs)
            # Renumber so `route_index` is the execution position, layer by
            # layer. Everything a person points at by index -- the
            # `Routing [i/N]` lines, `--debug-svgs <selector>`,
            # `--debug-stop-after-route N`, a partial GDS "up to the failure"
            # -- then follows the order the nets are actually routed in,
            # instead of the schematic's declaration order (which, with
            # pre-placed crossing grids, lists every `__from_grid` stub after
            # every `__to_grid` one, so a stop-after cut used to drop nets
            # that had already been routed). `net_id` is untouched: it is
            # the identity the obstacle map and bookkeeping key on.
            route_jobs = [
                replace(job, route_index=position)
                for position, job in enumerate(route_jobs, start=1)
            ]

        port_runway_static_cells: set[tuple[int, int]] = set()
        for cells in self.port_runway_cells_by_spec.values():
            port_runway_static_cells.update(cells)
        self.foreign_port_keepout_static_cells: set[tuple[int, int]] = set()
        for cells in foreign_port_keepout_cells_by_instance.values():
            self.foreign_port_keepout_static_cells.update(cells)
        self.debug_port_keepout_cells = set(port_runway_static_cells)
        self.debug_port_keepout_cells.update(self.foreign_port_keepout_static_cells)
        self.debug_port_keepout_cells.update(self.fanout_stub_static_cells)
        self.static_blocked_cells_before_port_reservations = set(self.raw_static_cells)
        self.static_blocked_cells_before_port_reservations.update(port_runway_static_cells)
        self.static_blocked_cells_before_port_reservations.update(
            self.foreign_port_keepout_static_cells
        )
        self.static_blocked_cells_before_port_reservations.update(self.fanout_stub_static_cells)

        t_static_handoff_start = self._pipeline_timer_start()
        self.blocked_static_rects_for_diagnostics: list[tuple[int, int, int, int]] = []
        if hasattr(obstacle_map, "blocked_static_rects"):
            blocked_static_rects: list[tuple[int, int, int, int]] = []
            raw_blocked_rects = cast(
                Iterable[tuple[int, int, int, int]], getattr(obstacle_map, "blocked_static_rects")
            )
            for rect in raw_blocked_rects:
                if len(rect) != 4:
                    continue
                blocked_static_rects.append(
                    (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                )
            self.blocked_static_rects_for_diagnostics = list(blocked_static_rects)
            if blocked_static_rects:
                if not hasattr(self.router, "set_static_rects"):
                    raise RuntimeError(
                        "The loaded photonic_router._rust extension does not expose "
                        "PyPhotonicRouter.set_static_rects. Rebuild it with "
                        "`maturin develop --release`; otherwise bounding_boxes mode "
                        "cannot use compact static rectangles."
                    )
                self.router.set_static_rects(blocked_static_rects)
            else:
                self.router.set_static_cells(sorted(self.raw_static_cells))
        else:
            sorted_static_cells = sorted(self.raw_static_cells)
            self.router.set_static_cells(sorted_static_cells)
        if (
            port_runway_static_cells
            or self.foreign_port_keepout_static_cells
            or self.fanout_stub_static_cells
        ):
            if not hasattr(self.router, "add_static_cells"):
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.add_static_cells. Rebuild it with "
                    "`maturin develop --release`."
                )
            self.router.add_static_cells(
                sorted(
                    port_runway_static_cells
                    | self.foreign_port_keepout_static_cells
                    | self.fanout_stub_static_cells
                )
            )
        self._record_pipeline_timing("static_map_handoff", t_static_handoff_start)

        full_route_jobs = list(route_jobs)
        self.full_route_jobs_by_route_index = {int(job.route_index): job for job in full_route_jobs}
        full_route_count = len(full_route_jobs)
        if (
            self.debug_stop_after_route_index is not None
            and int(self.debug_stop_after_route_index) > full_route_count
        ):
            raise ValueError(
                "debug_stop_after_route_index exceeds route count "
                f"({self.debug_stop_after_route_index} > {full_route_count})"
            )
        if self.debug_stop_after_route_index is not None:
            stop_index = int(self.debug_stop_after_route_index)
            route_jobs = [job for job in full_route_jobs if int(job.route_index) <= stop_index]
            if self.verbose_route_diagnostics or self.debug_route_indices is not None:
                print(
                    f"  Debug stop-after-route active: routing {len(route_jobs)} "
                    f"of {full_route_count} full-context routes"
                )
        debug_execution_limit_raw = os.environ.get("PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT")
        if debug_execution_limit_raw:
            try:
                debug_execution_limit = int(debug_execution_limit_raw)
            except ValueError as exc:
                raise ValueError(
                    "PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT must be an integer"
                ) from exc
            if debug_execution_limit < 1:
                raise ValueError("PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT must be >= 1")
            original_route_job_count = len(route_jobs)
            route_jobs = route_jobs[:debug_execution_limit]
            if self.verbose_route_diagnostics or self.debug_route_indices is not None:
                print(
                    "  Debug execution limit active: routing "
                    f"{len(route_jobs)} of {original_route_job_count} selected "
                    "routes in actual execution order"
                )

        self.repair_config = self.ripup_reroute_config or RipupRerouteConfig()
        self.route_jobs_by_id = {job.net_id: job for job in route_jobs}
        self.route_order = [job.net_id for job in route_jobs]
        self.collect_timing = (
            self.debug_timing or self.collect_route_stats or self.collect_attempt_diagnostics
        )
        self.track_dynamic_cells = self.diagnostics_enabled
        self.route_bookkeeping = RouteBookkeeping(
            route_order=self.route_order,
            diagnostics_enabled=self.track_dynamic_cells,
        )

        t_astar_start = 0.0
        if self.collect_timing:
            t_astar_start = time.perf_counter()
        self.total_expanded_states = 0
        self.simple_route_count = 0
        self.repair_count = 0
        self.route_attempt_records = []
        self.native_repair_trace_records: list[dict[str, object]] = []
        self.route_timing_buckets: dict[str, RouteTimingBucket] = {
            name: RouteTimingBucket()
            for name in (
                "normal_route",
                "probe_route",
                "preemptive_crossing_ripup",
                "guided_collision_crossing",
                "localized_crossing_keepout",
                "repair_failed_net",
                "reroute_victims",
                "lidar_pure_probe_commit",
                "endpoint_correction",
            )
        }

        if not hasattr(self.router, "build_dynamic_clearance_exempt_cells_for_routes"):
            extension_path = getattr(self.rust_backend, "__file__", "<unknown>")
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.build_dynamic_clearance_exempt_cells_for_routes. "
                "Rebuild it with `maturin develop --release`. "
                f"Loaded extension: {extension_path}"
            )

        t_state_opening_precompute_start = self._pipeline_timer_start()
        self.route_state_openings_by_id = {
            int(job.net_id): self._states_and_openings(job) for job in route_jobs
        }
        self._record_pipeline_timing(
            "state_opening_precompute",
            t_state_opening_precompute_start,
        )
        clearance_exempt_inputs = [
            (int(net_id), state_openings[0], state_openings[1], state_openings[4])
            for net_id, state_openings in self.route_state_openings_by_id.items()
        ]
        t_clearance_exempt_batch_start = self._pipeline_timer_start()
        self.batch_clearance_exempt_cells_by_id = {
            int(net_id): [(int(cell[0]), int(cell[1])) for cell in cells]
            for net_id, cells in self.router.build_dynamic_clearance_exempt_cells_for_routes(
                clearance_exempt_inputs,
                int(self.commit_radius_cells),
                int(self.port_lane_length_cells),
            )
        }
        self._record_pipeline_timing(
            "clearance_exempt_batch",
            t_clearance_exempt_batch_start,
        )

        self.realization_grid_spec = (
            int(self.grid.width),
            int(self.grid.height),
            float(self.grid.grid_size_um),
            float(self.origin_x_um),
            float(self.origin_y_um),
        )

        return route_jobs, t_astar_start

    def _append_target_fanout_stubs_after_correction(self) -> None:
        """Splice each target-anchored net's fixed stub onto its corrected route.

        `_record_route`/`RouteBookkeeping.record_route` pointed any
        target-anchored net's own `target_port_center_um` at the anchor's
        exact position (not the true physical port) specifically so that
        the endpoint-correction pass just run in `_finalize_routing_results`
        would resolve the search's grid state to that exact point using
        the same machinery every ordinary port already relies on. Now that
        correction is done, append the fixed, pre-built stub segment
        (already known exactly -- `anchor.stub_centerline_um`, reversed --
        no further correction needed for it) to reach the true port, and
        restore `target_port_center_um` to the true port so downstream
        verification and reporting see the net's real, declared endpoint.
        See `.agent/execplans/2026-08-26-target-side-static-stubs-for-dense-mmi-ports.md`.
        """
        for net_id, record in list(self.route_bookkeeping.records_by_id.items()):
            target_spec = f"{record.target.instance},{record.target.port}"
            anchor = self.fanout_anchor_by_port_spec.get(target_spec)
            if anchor is None:
                continue
            centerline = list(record.corrected_centerline_um)
            if len(centerline) < 2:
                continue
            endpoint_entry = self.endpoint_ports_by_spec.get(target_spec)
            if endpoint_entry is None:
                continue
            _inst, _port_name, port = endpoint_entry
            true_port_um = _port_center_um(port)
            if true_port_um is None:
                continue
            points = list(centerline)
            self._append_centerline_points(points, [true_port_um])
            new_centerline = _compress_centerline(tuple(points))
            if len(new_centerline) < 2:
                continue
            try:
                new_total_length_um = float(_centerline_length_um(new_centerline))
            except Exception:
                continue
            self.route_bookkeeping.records_by_id[net_id] = replace(
                record,
                corrected_centerline_um=new_centerline,
                total_length_um=new_total_length_um,
                target_port_center_um=true_port_um,
            )

    def _finalize_routing_results(
        self, route_jobs: list[RouteJob], t_astar_start: float
    ) -> tuple[list[RoutedNetRecord], float]:
        """Apply checked endpoint corrections, assemble routed-net records, and print the debug timing breakdown.

        Computes `astar_elapsed_s` from `t_astar_start` (see
        `_finalize_route_jobs_and_static_handoff`'s docstring for why that reading was
        taken before this method's caller's own precompute work, not at native-dispatch
        time). Applies checked endpoint corrections if `self.enable_checked_endpoint_correction`,
        assembles `routed_net_records` from `self.route_bookkeeping` and raises
        `RuntimeError` if any duplicate net record was produced, then — only if
        `self.debug_timing and self.verbose_route_diagnostics` — prints the per-bucket A*
        timing breakdown. Returns the assembled records and `astar_elapsed_s` for the
        final verification/realization phases.
        """
        astar_elapsed_s = 0.0
        if self.collect_timing:
            astar_elapsed_s = time.perf_counter() - t_astar_start

        if self.enable_checked_endpoint_correction:
            self._apply_all_endpoint_corrections_for_net_ids(
                list(self.route_bookkeeping.route_order),
                print_warnings=(
                    self.collect_attempt_diagnostics
                    or self.diagnostics_enabled
                    or self.verbose_route_diagnostics
                ),
            )
        self._append_target_fanout_stubs_after_correction()

        t_record_assembly_start = self._pipeline_timer_start()
        routed_net_records = self.route_bookkeeping.ordered_records()
        routed_record_keys = [
            (
                record.net_name,
                record.source.instance,
                record.source.port,
                record.target.instance,
                record.target.port,
            )
            for record in routed_net_records
        ]
        duplicate_record_keys = [
            key for key, count in Counter(routed_record_keys).items() if count > 1
        ]
        if duplicate_record_keys:
            formatted = ", ".join(
                f"{name}:{src_i},{src_p}->{dst_i},{dst_p}"
                for name, src_i, src_p, dst_i, dst_p in duplicate_record_keys[:8]
            )
            raise RuntimeError(f"Duplicate routed records generated: {formatted}")
        self._record_pipeline_timing("record_assembly", t_record_assembly_start)

        if self.debug_timing and self.verbose_route_diagnostics:
            print(f"      - A* route-search loop time: {astar_elapsed_s:.4f} s")
            print(
                "      - Route search stats: "
                f"simple={self.simple_route_count}/{len(route_jobs)}, "
                f"expanded_states={self.total_expanded_states}, "
                f"repairs={self.repair_count}"
            )
            print("      - A* timing breakdown by operation:")
            for bucket_name in (
                "normal_route",
                "probe_route",
                "preemptive_crossing_ripup",
                "guided_collision_crossing",
                "localized_crossing_keepout",
                "repair_failed_net",
                "reroute_victims",
                "lidar_pure_probe_commit",
                "endpoint_correction",
            ):
                bucket = self.route_timing_buckets[bucket_name]
                if bucket.calls == 0:
                    continue
                line = (
                    f"        {bucket_name}: calls={bucket.calls}, "
                    f"ok={bucket.successes}, fail={bucket.failures}, "
                    f"time={bucket.elapsed_s:.4f}s"
                )
                has_route_stats = (
                    bucket.expanded_states
                    or bucket.generated_neighbors
                    or bucket.heap_pushes
                    or bucket.heap_pops
                    or bucket.window_attempts
                    or bucket.footprint_checks
                    or bucket.dense_grid_build_time_us
                    or bucket.max_window_area_cells
                    or bucket.full_grid_fallbacks
                )
                if has_route_stats:
                    line += (
                        f", expanded={bucket.expanded_states}, "
                        f"generated={bucket.generated_neighbors}, "
                        f"heap_pushes={bucket.heap_pushes}, "
                        f"heap_pops={bucket.heap_pops}, "
                        f"duplicate_skips={bucket.skipped_duplicate_heap_entries}, "
                        f"windows={bucket.window_attempts}, "
                        f"max_window={bucket.max_window_area_cells}, "
                        f"legality_checks={bucket.obstacle_clearance_checks}, "
                        f"footprint_checks={bucket.footprint_checks}, "
                        f"rect_checks={bucket.footprint_rect_checks}, "
                        f"dense_cells={bucket.dense_grid_cells}, "
                        f"dense_build={bucket.dense_grid_build_time_us / 1_000_000.0:.4f}s, "
                        f"search_loop={bucket.search_loop_time_us / 1_000_000.0:.4f}s, "
                        f"obstacle_prepare={bucket.obstacle_map_prepare_time_us / 1_000_000.0:.4f}s, "
                        f"simple_probe={bucket.simple_route_time_us / 1_000_000.0:.4f}s, "
                        f"commit_prepare={bucket.commit_prepare_time_us / 1_000_000.0:.4f}s, "
                        f"commit={bucket.commit_time_us / 1_000_000.0:.4f}s, "
                        f"neighbor_time={bucket.neighbor_generation_time_us / 1_000_000.0:.4f}s, "
                        f"heap_time={bucket.heap_operation_time_us / 1_000_000.0:.4f}s, "
                        f"legality_time={bucket.legality_check_time_us / 1_000_000.0:.4f}s, "
                        f"reconstruction_time={bucket.reconstruction_time_us / 1_000_000.0:.4f}s, "
                        f"full_grid_fallbacks={bucket.full_grid_fallbacks}"
                    )
                print(line)

        return routed_net_records, astar_elapsed_s

    def run(self) -> tuple[Component, RustRouteDebugArtifacts]:
        obstacle_map, crossing_device_info, obstacle_svg = self._build_static_obstacle_context()
        nets = self.schematic.netlist.routes
        self._configure_router_and_grid(obstacle_map)
        route_jobs, endpoint_port_specs_by_instance, dense_port_runway_length_by_spec = (
            self._build_route_jobs_and_fanout_clustering(nets)
        )
        route_jobs, foreign_port_keepout_cells_by_instance = (
            self._build_crossing_plan_and_port_footprints(
                route_jobs,
                endpoint_port_specs_by_instance,
                dense_port_runway_length_by_spec,
                obstacle_map,
                crossing_device_info,
            )
        )
        route_jobs, t_astar_start = self._finalize_route_jobs_and_static_handoff(
            route_jobs,
            foreign_port_keepout_cells_by_instance,
            obstacle_map,
        )

        self._dispatch_native_routing(route_jobs)

        routed_net_records, astar_elapsed_s = self._finalize_routing_results(
            route_jobs, t_astar_start
        )

        routed_net_records, illegal_realized_crossings = self._repair_and_verify_final_geometry(
            routed_net_records
        )

        debug_artifacts = self._realize_and_assemble_debug_artifacts(
            route_jobs,
            routed_net_records,
            illegal_realized_crossings,
            obstacle_map,
            obstacle_svg,
            astar_elapsed_s,
        )
        return self.routed_layout, debug_artifacts

    def _repair_and_verify_final_geometry(
        self, routed_net_records: list[RoutedNetRecord]
    ) -> tuple[list[RoutedNetRecord], list[dict[str, object]]]:
        """Run the final crossing-legality and photonic-verification repair loops before geometry realization.

        Crossing legality is still checked internally because the router owns the
        crossing event model and can repair/reroute before final realization. The full
        photonic probe verification is intentionally diagnostic: it was useful while
        chasing endpoint-correction and crossing-model mismatches, but the normal flow
        skips this expensive pass unless `self.enable_internal_photonic_probe_verification`
        asks for it. The external Python verifier in `routing_flow.py` remains the final
        GDS/layout-level gate. Repairs up to `final_crossing_repair_round_limit=12` rounds
        of illegal realized crossings; if crossings are clean and internal photonic-probe
        verification is enabled, additionally repairs up to 8 rounds of photonic issues
        (each round itself re-running the crossing-repair loop), dumping failure artifacts
        and raising `RuntimeError` if geometry cannot be made legal within those bounds.
        Returns the possibly-updated `routed_net_records` and the final
        `illegal_realized_crossings` list (empty if geometry is clean) for the realization
        phase that follows.
        """
        self.photonic_probe_index = 0
        self.last_photonic_probe_layout: Component | None = None
        self.last_photonic_probe_records: list[RoutedNetRecord] = []

        final_crossing_repair_round_limit = 12
        t_final_verification_block_start = self._pipeline_timer_start()
        # Crossing legality is still checked internally because the router owns the
        # crossing event model and can repair/reroute before final realization.
        #
        # The full photonic probe verification is intentionally diagnostic: it was
        # useful while chasing endpoint-correction and crossing-model mismatches,
        # but the normal flow skips this expensive pass unless debug/failure
        # analysis asks for it. The external Python verifier in `routing_flow.py`
        # remains the final GDS/layout-level gate.
        illegal_realized_crossings = self._refresh_realized_crossing_verification()
        for _final_repair_round in range(final_crossing_repair_round_limit):
            if not illegal_realized_crossings:
                break
            if not self._repair_final_illegal_crossings(illegal_realized_crossings):
                break
            routed_net_records = self.route_bookkeeping.ordered_records()
            illegal_realized_crossings = self._refresh_realized_crossing_verification()
        if not illegal_realized_crossings and self.enable_internal_photonic_probe_verification:
            final_photonic_verification = self._refresh_photonic_verification()
            for _final_photonic_repair_round in range(8):
                if final_photonic_verification.success:
                    break
                if not self._repair_final_photonic_issues(final_photonic_verification.issues):
                    break
                illegal_realized_crossings = self._refresh_realized_crossing_verification()
                for _nested_crossing_repair_round in range(final_crossing_repair_round_limit):
                    if not illegal_realized_crossings:
                        break
                    if not self._repair_final_illegal_crossings(illegal_realized_crossings):
                        break
                    illegal_realized_crossings = self._refresh_realized_crossing_verification()
                routed_net_records = self.route_bookkeeping.ordered_records()
                if illegal_realized_crossings:
                    break
                final_photonic_verification = self._refresh_photonic_verification()
            if not illegal_realized_crossings and not final_photonic_verification.success:
                _write_crossing_debug_artifacts(
                    debug_path=self.debug_path if self.debug_path is not None else Path("build"),
                    debug_prefix=self.debug_prefix,
                    crossing_plan_info=self.crossing_plan_info,
                )
                probe_failure_artifacts = _dump_photonic_probe_failure_artifacts(
                    debug_path=self.debug_path if self.debug_path is not None else Path("build"),
                    debug_prefix=self.debug_prefix,
                    probe_layout=self.last_photonic_probe_layout,
                    verification=final_photonic_verification,
                    records=self.last_photonic_probe_records
                    or self.route_bookkeeping.ordered_records(),
                    router=self.router,
                    realization_grid_spec=self.realization_grid_spec,
                    allow_unchecked_bumps=True,
                )
                self.crossing_plan_info["photonic_probe_failure_artifacts"] = (
                    probe_failure_artifacts
                )
                self._record_pipeline_timing(
                    "final_verification_block",
                    t_final_verification_block_start,
                )
                raise RuntimeError(
                    "Final photonic geometry repair failed before realization: "
                    f"{final_photonic_verification.error_count} error(s). "
                    f"{self._photonic_repair_failure_preview(final_photonic_verification)}"
                )
        self._record_pipeline_timing(
            "final_verification_block",
            t_final_verification_block_start,
        )
        return routed_net_records, illegal_realized_crossings

    def _realize_and_assemble_debug_artifacts(
        self,
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
        if not illegal_realized_crossings and not self.defer_realization:
            t_direct_realization_start = self._pipeline_timer_start()
            realize_routed_net_records(
                self.routed_layout,
                routed_net_records,
                route_width_um=self.route_width_um,
                route_layer=self.route_layer,
                realization_grid_spec=self.realization_grid_spec,
                allow_45_degree_turns=self.allow_45_degree_turns,
                bend_radius_cells=self.bend_radius_cells,
                crossing_plan_info=self.crossing_plan_info,
                enable_endpoint_correction=self.enable_checked_endpoint_correction,
            )
            self._record_pipeline_timing("direct_realization", t_direct_realization_start)
            _place_realized_crossing_components(self.routed_layout, self.crossing_plan_info)
        elif self.crossing_plan_info.get("enabled"):
            self.crossing_plan_info.setdefault("realized_crossing_components", [])
            self.crossing_plan_info.setdefault("realized_crossing_component_count", 0)
        _write_crossing_debug_artifacts(
            debug_path=self.debug_path if self.debug_path is not None else Path("build"),
            debug_prefix=self.debug_prefix,
            crossing_plan_info=self.crossing_plan_info,
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

        t_debug_artifact_start = self._pipeline_timer_start()
        debug_artifacts = build_route_debug_artifacts(
            obstacle_svg=obstacle_svg,
            route_svgs=self.route_svgs,
            obstacle_map=obstacle_map,
            routed_net_records=routed_net_records,
            realization_grid_spec=self.realization_grid_spec,
            allow_45_degree_turns=self.allow_45_degree_turns,
            bend_radius_cells=self.bend_radius_cells,
            route_search_summary=summarize_route_search(
                self.route_timing_buckets,
                route_count=len(route_jobs),
                simple_route_count=self.simple_route_count,
                repair_count=self.repair_count,
                astar_elapsed_s=astar_elapsed_s,
            ),
            route_attempt_records=self.route_attempt_records,
            route_nets_timings_s=self.route_nets_timings_s,
        )
        debug_artifacts = replace(
            debug_artifacts,
            crossing_plan_info=self.crossing_plan_info,
        )
        self._record_pipeline_timing("debug_artifact_assembly", t_debug_artifact_start)
        if self.collect_pipeline_timing:
            debug_artifacts = replace(
                debug_artifacts,
                route_nets_timings_s=dict(self.route_nets_timings_s),
            )
        return debug_artifacts


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
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
    enable_crossings: bool = False,
    node_depths: dict[str, int] | None = None,
    node_ranks: dict[str, int] | None = None,
    edge_ranks: dict[str, dict[str, int]] | None = None,
    crossing_loss: float = 0.0,
    crossing_mode: str = "window",
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
    fanout_access_mode: str | None = None,
    allow_only_expected_crossings: bool = True,
    defer_realization: bool = False,
    enable_checked_endpoint_correction: bool = True,
) -> tuple[Component, RustRouteDebugArtifacts]:
    """Route schematic nets through a temporary routing session."""
    session = _RouteNetsRustSession(
        unrouted_layout=unrouted_layout,
        schematic=schematic,
        obstacle_config=obstacle_config,
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
        heap_tie_breaker=heap_tie_breaker,
        proactive_congestion_weight=proactive_congestion_weight,
        proactive_congestion_radius_cells=proactive_congestion_radius_cells,
        max_iterations=max_iterations,
        routing_window_scale=routing_window_scale,
        debug_timing=debug_timing,
        verbose_route_diagnostics=verbose_route_diagnostics,
        collect_route_stats=collect_route_stats,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
        enable_internal_photonic_probe_verification=enable_internal_photonic_probe_verification,
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
        defer_realization=defer_realization,
        enable_checked_endpoint_correction=enable_checked_endpoint_correction,
    )
    return session.run()
