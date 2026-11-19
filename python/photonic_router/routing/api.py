"""The module-level routing API: the full match-and-realize pipeline, the static
fan-out anchor query and the path-length-matching re-export wrappers."""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from typing import Any

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.crossings.route_rust_crossing_components import (
    place_realized_crossing_components,
)
from photonic_router.crossings.route_rust_crossing_plan import (
    _routed_records_by_net_id,
    augment_insertion_loss_report_from_realized_intersections,
    write_crossing_debug_artifacts,
)
from photonic_router.crossings.route_rust_crossing_verification import (
    verify_realized_route_intersections,
)
from photonic_router.plm import route_rust_meanders as _meander_impl
from photonic_router.plm.path_length_diagnostics import (
    analysis_to_info_dict,
    format_path_length_acceptance_failure,
    matching_group_diagnostics_to_info,
    output_matching_diagnostics_to_info,
    path_length_acceptance_summary,
)
from photonic_router.plm.path_length_requirements import (
    analyze_path_length_matching,
    build_requirement_delay_candidates,
    compute_group_lifted_requirements,
    compute_output_matching_requirements,
    delay_candidate_to_dict,
    merge_missing_length_requirements,
    minimum_four_bend_extra_length_um,
    requirement_to_dict,
)
from photonic_router.records.route_rust_obstacle_config import (
    resolve_obstacle_config,
    with_bbox_cell_materialization,
    with_obstacle_mode,
)
from photonic_router.records.route_rust_realization import realize_routed_net_records
from photonic_router.records.route_rust_records import build_port_alignment_diagnostics
from photonic_router.records.route_rust_types import (
    MeanderInsertionConfig,
    OpticalRouteClearancePolicy,
    RouteRustPipelineResult,
    _as_float,
)
from photonic_router.routing.crossing_plan_info import CrossingPlanInfo
from photonic_router.routing.options import CircuitMetadata, MatchingOptions, SessionOptions
from photonic_router.routing.route_jobs import build_route_jobs_and_fanout_clustering

# re-exported here as part of the module-level routing API
from photonic_router.routing.router_setup import (  # noqa: F401
    DENSE_OBSTACLE_CELL_CAP_MARGIN,
    configure_router_and_grid,
    dense_obstacle_cell_cap,
)
from photonic_router.routing.session import (
    RouteNetsRustSession,
    route_nets_rust,
)
from photonic_router.static_obstacle_builder import (
    _load_rust_backend,
    build_static_obstacle_map,
)


def analyze_meander_insertion_for_requirements(*args: Any, **kwargs: Any):
    _meander_impl._load_rust_backend = _load_rust_backend
    return _meander_impl.analyze_meander_insertion_for_requirements(*args, **kwargs)


def route_match_and_realize(
    unrouted_layout: Component,
    schematic: Schematic,
    options: SessionOptions | None = None,
    *,
    matching: MatchingOptions | None = None,
    metadata: CircuitMetadata | None = None,
) -> RouteRustPipelineResult:
    """Route the nets, match the path lengths when asked to, and realize.

    The three steps are: the routing session (`route_nets_rust`, with the
    realization deferred), the path-length analysis and the meander
    insertion when `matching.enable_path_length_matching` is set, and the
    realization of the routes into polygons.

    Args:
        unrouted_layout: the component with placed instances and no routes.
        schematic: the schematic whose nets are routed.
        options: the session's options; None is the default session.
            `realization.defer_realization` is set by this function.
        matching: what path-length matching does; None leaves it off.
        metadata: what the benchmark declares about its circuit, for the
            path-length analysis.
    """
    options = options if options is not None else SessionOptions()
    matching = matching if matching is not None else MatchingOptions()
    metadata = metadata if metadata is not None else CircuitMetadata()
    debug = options.debug
    realization = options.realization
    route_obstacle_config = options.obstacle_config
    if debug.debug_dir is None:
        route_obstacle_config = with_bbox_cell_materialization(
            options.obstacle_config,
            materialize_bbox_cells=False,
            populate_obstacle_map=False,
        )

    t_route_nets_start = time.perf_counter()
    routed_layout, debug_artifacts = route_nets_rust(
        unrouted_layout,
        schematic,
        replace(
            options,
            obstacle_config=route_obstacle_config,
            realization=replace(realization, defer_realization=True),
        ),
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
    if debug.debug_timing and debug.verbose_route_diagnostics:
        print(
            "      - Optical net routing phase "
            f"(obstacle map + A* + repairs): {pipeline_timings_s['route_nets']:.4f} s"
        )

    if realization.enable_checked_endpoint_correction:
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
    meander_report = None
    meander_report_info = None
    records_for_realization = debug_artifacts.routed_net_records
    if matching.enable_path_length_matching:
        t_analysis_start = time.perf_counter()
        analysis, requirements = analyze_path_length_matching(
            schematic,
            routed_net_records=records_for_realization,
            node_types=metadata.node_types,
            internal_delays_um=metadata.internal_delays_um,
            input_ports_by_component=metadata.input_ports_by_component,
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
            "enabled": bool(matching.path_length_match_outputs),
            "target_output_arrival_um": 0.0,
            "output_count": 0,
            "requirements": [],
            "outputs": [],
        }
        if matching.path_length_match_outputs:
            output_requirements, output_matching_info = compute_output_matching_requirements(
                analysis,
                existing_requirements=lifted_requirements,
                scope=matching.path_length_output_scope,
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
        resolved_user_obstacle_config = resolve_obstacle_config(
            options.obstacle_config,
            route_layer=realization.route_layer,
            include_heater_obstacles=options.include_heater_obstacles,
        )
        meander_route_clearance_um = max(
            0.0,
            _as_float(
                getattr(resolved_user_obstacle_config, "clearance_um", 0.0),
                0.0,
            ),
        )
        meander_clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
            route_width_um=float(realization.route_width_um),
            grid_size_um=float(debug_artifacts.realization_grid_spec[2]),
            route_clearance_um=meander_route_clearance_um,
        )
        analysis_info["clearance_policy"] = meander_clearance_policy.to_debug_dict()
        # Meander box legality should use real routed-layer geometry, not
        # conservative component bboxes. Keep static obstacles strict: source
        # and target access openings are valid for route entry/exit only, not
        # for placing meander boxes.
        meander_obstacle_config = with_obstacle_mode(
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
            max_meander_height_um=float(matching.path_length_meander_height_um),
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
        records_for_realization, meander_report = analyze_meander_insertion_for_requirements(
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
            debug_meanders=matching.debug_meanders,
        )
        pipeline_timings_s["meander_planning"] = time.perf_counter() - t_meander_planning_start
        meander_report_info = meander_report.to_info_dict()
        if analysis_info is not None:
            matching_group_diagnostics = matching_group_diagnostics_to_info(
                analysis,
                meander_report,
                adjusted_requirements=requirements,
                lifted_groups=lifted_groups,
            )
            output_matching_diagnostics = output_matching_diagnostics_to_info(
                output_matching_info,
                meander_report,
                analysis=analysis,
            )
            acceptance_summary = path_length_acceptance_summary(
                matching_group_diagnostics + output_matching_diagnostics
            )
            analysis_info["matching_group_diagnostics"] = matching_group_diagnostics
            analysis_info["output_matching_diagnostics"] = output_matching_diagnostics
            analysis_info["path_length_acceptance"] = acceptance_summary
            if not acceptance_summary["passed"]:
                failure = format_path_length_acceptance_failure(acceptance_summary)
                declared = frozenset(matching.path_length_declared_unmatched_groups)
                undeclared = _undeclared_failed_groups(acceptance_summary, declared)
                analysis_info["path_length_declared_unmatched_groups"] = sorted(declared)
                if undeclared:
                    raise RuntimeError(f"{failure} Undeclared group(s): {', '.join(undeclared)}.")
                # Decision 11 (2026-11-05): the benchmark declares the groups it
                # leaves unmatched on purpose; the run finishes and reports them
                # (SUMMARY plm_over_tolerance).
                print(f"      - WARNING: {failure} (declared unmatched groups)")

    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")

    crossing_plan_info = debug_artifacts.crossing_plan_info
    plan_info = (
        CrossingPlanInfo.from_dict(crossing_plan_info)
        if isinstance(crossing_plan_info, dict)
        else None
    )
    if plan_info is not None and options.enable_internal_photonic_probe_verification:
        final_records_by_net_id = _routed_records_by_net_id(records_for_realization)
        if final_records_by_net_id:
            illegal_realized_crossings = verify_realized_route_intersections(
                crossing_plan_info=plan_info,
                routed_records_by_net_id=final_records_by_net_id,
                realization_grid_spec=debug_artifacts.realization_grid_spec,
                native_crossing_verdicts=plan_info.native_crossing_verdicts,
            )
            augment_insertion_loss_report_from_realized_intersections(
                crossing_plan_info=plan_info,
                routed_records_by_net_id=final_records_by_net_id,
            )
            if illegal_realized_crossings:
                write_crossing_debug_artifacts(
                    debug_path=Path(debug.debug_dir)
                    if debug.debug_dir is not None
                    else Path("build"),
                    debug_prefix=debug.debug_prefix,
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
        route_width_um=realization.route_width_um,
        route_layer=realization.route_layer,
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
        crossing_plan_info=plan_info.to_dict() if plan_info is not None else None,
        enable_endpoint_correction=realization.enable_checked_endpoint_correction,
    )
    if plan_info is not None:
        place_realized_crossing_components(routed_layout, plan_info)
        write_crossing_debug_artifacts(
            debug_path=Path(debug.debug_dir) if debug.debug_dir is not None else None,
            debug_prefix=debug.debug_prefix,
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
    if debug.debug_timing and debug.verbose_route_diagnostics:
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
        meander_insertion_report=meander_report,
        pipeline_timings_s=pipeline_timings_s,
    )


def static_fanout_anchors_um(
    unrouted_layout: Component,
    schematic: Schematic,
    options: SessionOptions | None = None,
) -> dict[str, tuple[float, float]]:
    """The router's static fan-out anchors for ``unrouted_layout`` WITHOUT routing.

    Runs the same setup a routing session performs before its first search
    (static obstacle map, grid, route jobs, dense-port clustering) and returns
    ``{port_spec: (x_um, y_um)}`` for every port that gets a static stub -- the
    point a route to/from that port actually starts from. Pre-placed crossing
    structures (contribution 2) put their entry tiles on these rows so the
    stub and the tile line up without a jog. ``options`` must match the
    routing run's (fan-out access mode, bend radius, obstacle configuration),
    otherwise the anchors differ.
    """
    session = RouteNetsRustSession(unrouted_layout, schematic, options)
    state = session.prepare()
    build_route_jobs_and_fanout_clustering(session.settings, state, schematic.netlist.routes)
    return {
        spec: (float(anchor.center_um[0]), float(anchor.center_um[1]))
        for spec, anchor in state.fanout.anchor_by_port_spec.items()
    }


def _undeclared_failed_groups(
    acceptance_summary: dict[str, object], declared: frozenset[str]
) -> list[str]:
    """Node names of the groups the acceptance failed that the benchmark did not
    declare unmatched (decision 11): the realized-spread failures and the
    residual failures alike."""
    names: list[str] = []
    for key in ("failed_groups", "realized_failed_groups"):
        groups = acceptance_summary.get(key, [])
        if not isinstance(groups, list):
            continue
        for group in groups:
            if not isinstance(group, dict):
                continue
            name = str(group.get("node_name", ""))
            if name not in declared and name not in names:
                names.append(name)
    return names
