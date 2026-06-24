"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import math
import sys
import time
from dataclasses import is_dataclass, replace
from pathlib import Path
from typing import Any, Iterable, cast

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_SOURCE = PROJECT_ROOT / "python"
if str(PYTHON_SOURCE) not in sys.path:
    sys.path.insert(0, str(PYTHON_SOURCE))

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic
from gdsfactory.typings import Port

from translation.route_gds import get_port_from_instance
from photonic_router.routing_layers import (
    find_component_port_access_rule,
    get_routing_obstacle_layers,
)
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
from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_records import (
    EndpointCorrectionRouter,
    RouteBookkeeping,
    apply_port_endpoint_corrections,
    build_port_alignment_diagnostics,
    build_route_debug_artifacts,
    routed_edge_lengths_from_records,
)
from translation.route_rust_types import (
    MeanderInsertionConfig,
    RipupRerouteConfig,
    RouteJob,
    RouteRustPipelineResult,
    RouteTimingBucket,
    RoutedNetRecord,
    RustRouteDebugArtifacts,
    _as_float,
    route_attempt_record_from_route,
    summarize_route_search,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend


def _format_route_indices(indices: set[int]) -> str:
    if not indices:
        return "<none>"
    ranges: list[str] = []
    sorted_indices = sorted(indices)
    start = sorted_indices[0]
    previous = start
    for index in sorted_indices[1:]:
        if index == previous + 1:
            previous = index
            continue
        ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
        start = index
        previous = index
    ranges.append(f"{start}" if start == previous else f"{start}-{previous}")
    return ",".join(ranges)


def analyze_meander_insertion_for_requirements(*args: Any, **kwargs: Any):
    _meander_impl._load_rust_backend = _load_rust_backend
    return _meander_impl.analyze_meander_insertion_for_requirements(*args, **kwargs)


def insert_meanders_for_requirements(*args: Any, **kwargs: Any):
    _meander_impl._load_rust_backend = _load_rust_backend
    return _meander_impl.insert_meanders_for_requirements(*args, **kwargs)


def _build_realization_router(
    *,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
) -> EndpointCorrectionRouter:
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError("Rust router backend unavailable for endpoint correction.")
    width, height, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    grid_spec = rust_backend.GridSpec(
        int(width),
        int(height),
        float(grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid_size_um),
        bend_radius_cells=int(bend_radius_cells),
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar_cfg = rust_backend.AStarConfig(max_iterations=1)
    return rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)


def _apply_endpoint_corrections_to_debug_artifacts(
    debug_artifacts: RustRouteDebugArtifacts,
) -> RustRouteDebugArtifacts:
    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")
    router = _build_realization_router(
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
    )
    records = apply_port_endpoint_corrections(
        debug_artifacts.routed_net_records,
        router=router,
    )
    return replace(
        debug_artifacts,
        routed_net_records=records,
        routed_edge_lengths_um=routed_edge_lengths_from_records(records),
        port_alignment_diagnostics=build_port_alignment_diagnostics(
            records,
            realization_grid_spec=debug_artifacts.realization_grid_spec,
        ),
    )


def route_match_and_realize(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    enable_path_length_matching: bool = False,
    path_length_match_outputs: bool = False,
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    debug_route_indices: set[int] | None = None,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    heap_tie_breaker: str = "smaller_g",
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    debug_timing: bool = False,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    include_heater_obstacles: bool = False,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    path_length_meander_height_um: float = 20.0,
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
        route_width_um=route_width_um,
        route_layer=route_layer,
        allow_45_degree_turns=allow_45_degree_turns,
        enable_jps4=enable_jps4,
        use_indexed_heap=use_indexed_heap,
        primitive_ordering=primitive_ordering,
        heuristic_mode=heuristic_mode,
        heap_tie_breaker=heap_tie_breaker,
        max_iterations=max_iterations,
        routing_window_scale=routing_window_scale,
        debug_timing=debug_timing,
        collect_route_stats=collect_route_stats,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
        include_heater_obstacles=include_heater_obstacles,
        ripup_reroute_config=ripup_reroute_config,
        defer_realization=True,
    )
    pipeline_timings_s: dict[str, float] = {
        "route_nets": time.perf_counter() - t_route_nets_start,
    }
    if debug_timing:
        print(f"      - route_nets_rust phase: {pipeline_timings_s['route_nets']:.4f} s")

    debug_artifacts = _apply_endpoint_corrections_to_debug_artifacts(debug_artifacts)
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
        analysis_info["raw_requirements"] = [
            requirement_to_dict(req)
            for req in raw_requirements
        ]
        analysis_info["lifted_requirements"] = [
            requirement_to_dict(req)
            for req in lifted_requirements
        ]
        analysis_info["output_matching"] = output_matching_info
        analysis_info["output_requirements"] = [
            requirement_to_dict(req)
            for req in output_requirements
        ]
        analysis_info["requirements"] = [
            requirement_to_dict(req)
            for req in requirements
        ]
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
        analysis_info["minimum_insertable_extra_length_um"] = float(
            min_insertable_extra_um
        )
        requirements_info = [requirement_to_dict(req) for req in requirements]
        if debug_artifacts.realization_grid_spec is None:
            raise RuntimeError("Missing realization grid spec from routing phase.")
        t_meander_obstacle_start = time.perf_counter()
        resolved_user_obstacle_config = _resolve_obstacle_config(
            obstacle_config,
            route_layer=route_layer,
            include_heater_obstacles=include_heater_obstacles,
        )
        # Meander box legality should use real routed-layer geometry, not
        # conservative component bboxes. Keep static obstacles strict: source
        # and target access openings are valid for route entry/exit only, not
        # for placing meander boxes.
        meander_obstacle_config = _with_obstacle_mode(
            resolved_user_obstacle_config,
            obstacle_mode="rasterized_polygons",
            clear_port_open_cells_from_static=False,
            populate_obstacle_map=False,
        )
        meander_obstacle_map = build_static_obstacle_map(
            unrouted_layout,
            config=meander_obstacle_config,
        )
        meander_static_blocked_cells = meander_obstacle_map.blocked_cells
        pipeline_timings_s["meander_obstacle_map"] = (
            time.perf_counter() - t_meander_obstacle_start
        )

        t_meander_planning_start = time.perf_counter()
        records_for_realization, meander_report_info = analyze_meander_insertion_for_requirements(
            records_for_realization,
            requirements,
            config=MeanderInsertionConfig(
                enabled=True,
                max_meander_height_um=float(path_length_meander_height_um),
            ),
            realization_grid_spec=debug_artifacts.realization_grid_spec,
            allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
            bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
            static_blocked_cells=meander_static_blocked_cells,
            requirement_delay_candidates=requirement_delay_candidates,
        )
        pipeline_timings_s["meander_planning"] = (
            time.perf_counter() - t_meander_planning_start
        )
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
                raise RuntimeError(
                    format_path_length_acceptance_failure(acceptance_summary)
                )

    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")
    t_realization_start = time.perf_counter()
    realize_routed_net_records(
        routed_layout,
        records_for_realization,
        route_width_um=route_width_um,
        route_layer=route_layer,
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
    )
    t_realization_end = time.perf_counter()
    pipeline_timings_s["route_realization"] = t_realization_end - t_realization_start
    if debug_timing:
        print(f"      - route realization phase: {pipeline_timings_s['route_realization']:.4f} s")

    return RouteRustPipelineResult(
        routed_layout=routed_layout,
        debug_artifacts=debug_artifacts,
        path_length_analysis_info=analysis_info,
        meander_requirements_info=requirements_info,
        meander_insertion_report_info=meander_report_info,
        pipeline_timings_s=pipeline_timings_s,
    )


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _cells_bbox(cells: set[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    if not cells:
        return None
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return min(xs), max(xs), min(ys), max(ys)


def _rect_cell_count(
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    width = max(0, max_x - min_x + 1)
    height = max(0, max_y - min_y + 1)
    return width * height


def _rect_overlap_cell_count(
    rect: tuple[int, int, int, int],
    *,
    min_x: int,
    max_x: int,
    min_y: int,
    max_y: int,
) -> int:
    rect_min_x, rect_min_y, rect_max_x, rect_max_y = rect
    overlap_min_x = max(min_x, rect_min_x)
    overlap_max_x = min(max_x, rect_max_x)
    overlap_min_y = max(min_y, rect_min_y)
    overlap_max_y = min(max_y, rect_max_y)
    return _rect_cell_count(
        min_x=overlap_min_x,
        max_x=overlap_max_x,
        min_y=overlap_min_y,
        max_y=overlap_max_y,
    )


def _route_cells_bbox(cells: Iterable[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    normalized = [(int(cell[0]), int(cell[1])) for cell in cells]
    if not normalized:
        return None
    xs = [cell[0] for cell in normalized]
    ys = [cell[1] for cell in normalized]
    return min(xs), max(xs), min(ys), max(ys)


def _format_cells_preview(cells: set[tuple[int, int]], *, limit: int = 120) -> str:
    if not cells:
        return "[]"
    ordered = sorted(cells)
    if len(ordered) <= limit:
        return str(ordered)
    head = ordered[:limit]
    return f"{head} ... (+{len(ordered) - limit} more)"


def _grid_origin_xy(grid: GridSpec) -> tuple[float, float]:
    if hasattr(grid, "origin"):
        origin = getattr(grid, "origin")
        return float(origin[0]), float(origin[1])
    return (
        float(getattr(grid, "origin_x_um", 0.0)),
        float(getattr(grid, "origin_y_um", 0.0)),
    )


def _default_obstacle_layers(
    route_layer: tuple[int, int],
    *,
    include_heater_obstacles: bool = False,
) -> tuple[tuple[int, int], ...]:
    if route_layer == (1, 0):
        return get_routing_obstacle_layers(include_heaters=include_heater_obstacles)
    return ((int(route_layer[0]), int(route_layer[1])),)


def _resolve_obstacle_config(
    obstacle_config: object | None,
    *,
    route_layer: tuple[int, int],
    include_heater_obstacles: bool = False,
) -> object:
    """Default obstacle extraction to the photonic routing layer."""

    default_layers = _default_obstacle_layers(
        route_layer,
        include_heater_obstacles=include_heater_obstacles,
    )
    if obstacle_config is None:
        return StaticObstacleMapConfig(obstacle_layers=default_layers)

    if isinstance(obstacle_config, dict):
        if obstacle_config.get("obstacle_layers") is None:
            config_dict = dict(obstacle_config)
            config_dict["obstacle_layers"] = default_layers
            return StaticObstacleMapConfig(**config_dict)
        return StaticObstacleMapConfig(**obstacle_config)

    obstacle_layers = getattr(obstacle_config, "obstacle_layers", None)
    if obstacle_layers is None:
        if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
            try:
                return replace(cast(Any, obstacle_config), obstacle_layers=default_layers)
            except Exception:
                return obstacle_config
        return obstacle_config

    return obstacle_config


def _with_bbox_cell_materialization(
    obstacle_config: object | None,
    *,
    materialize_bbox_cells: bool,
    populate_obstacle_map: bool = True,
) -> object | None:
    if obstacle_config is None:
        return {
            "materialize_bbox_cells": materialize_bbox_cells,
            "populate_obstacle_map": populate_obstacle_map,
        }

    if isinstance(obstacle_config, dict):
        config_dict = dict(obstacle_config)
        config_dict["materialize_bbox_cells"] = materialize_bbox_cells
        config_dict["populate_obstacle_map"] = populate_obstacle_map
        return config_dict

    if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
        try:
            return replace(
                cast(Any, obstacle_config),
                materialize_bbox_cells=materialize_bbox_cells,
                populate_obstacle_map=populate_obstacle_map,
            )
        except Exception:
            return obstacle_config

    return obstacle_config


def _with_obstacle_mode(
    obstacle_config: object | None,
    *,
    obstacle_mode: str,
    clear_port_open_cells_from_static: bool | None = None,
    populate_obstacle_map: bool | None = None,
) -> object | None:
    updates: dict[str, object] = {"obstacle_mode": obstacle_mode}
    if clear_port_open_cells_from_static is not None:
        updates["clear_port_open_cells_from_static"] = clear_port_open_cells_from_static
    if populate_obstacle_map is not None:
        updates["populate_obstacle_map"] = populate_obstacle_map

    if obstacle_config is None:
        return updates

    if isinstance(obstacle_config, dict):
        config_dict = dict(obstacle_config)
        config_dict.update(updates)
        return config_dict

    if is_dataclass(obstacle_config) and not isinstance(obstacle_config, type):
        try:
            return replace(cast(Any, obstacle_config), **updates)
        except Exception:
            return obstacle_config

    return obstacle_config


def _coerce_component_name(instance: object) -> str | None:
    component_obj = getattr(instance, "component", None)
    if component_obj is None:
        return None
    if isinstance(component_obj, str):
        return component_obj
    name = getattr(component_obj, "name", None)
    if isinstance(name, str) and name:
        return name
    return None


def _schematic_instance_component_name(
    schematic: object,
    instance_name: str,
) -> str | None:
    netlist = getattr(schematic, "netlist", None)
    instances = getattr(netlist, "instances", None)
    if not isinstance(instances, dict):
        return None
    instance = instances.get(instance_name)
    if instance is None:
        return None
    return _coerce_component_name(instance)


def _port_type_name(port: object) -> str | None:
    port_type = getattr(port, "port_type", None)
    if port_type is None:
        return None
    return str(port_type)


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    debug_route_indices: set[int] | None = None,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    heap_tie_breaker: str = "smaller_g",
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    debug_timing: bool = False,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    include_heater_obstacles: bool = False,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    defer_realization: bool = False,
) -> tuple[Component, RustRouteDebugArtifacts]:
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
        route_width_um: Realized waveguide width in micrometers.
        route_layer: Target GDS layer/datatype tuple for route polygons.
        allow_45_degree_turns: If False, omit ±45-degree turn primitives.
        use_indexed_heap: Benchmark-only queue experiment. Pass 8E measured
            this slower than duplicate-entry BinaryHeap queueing, so the
            production default remains False.
        primitive_ordering: Benchmark-only dense A* primitive iteration order.
            Supported values: "library", "long_straight_first",
            "target_biased". Pass 8F keeps "library" as the default.
        heuristic_mode: Dense A* heuristic. Supported values: "distance",
            "heading_aware".
        max_iterations: Maximum A* state expansions per route attempt.
        defer_realization: If True, keep routed RouteResult objects but skip
            polygon realization. This is used for pre-realization transforms
            such as path-length matching/meander insertion.

    Returns:
        A tuple of (routed_layout, debug_artifacts).
        :param debug_timing:
    """
    if route_width_um <= 0:
        raise ValueError("route_width_um must be > 0")
    if max_iterations <= 0:
        raise ValueError("max_iterations must be > 0")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    routed_layout = unrouted_layout.copy()
    routed_layout.name = "routed_layout_rust"

    t_obstacle_start = 0.0
    if debug_timing:
        t_obstacle_start = time.perf_counter()
    resolved_obstacle_config = _resolve_obstacle_config(
        obstacle_config,
        route_layer=route_layer,
        include_heater_obstacles=include_heater_obstacles,
    )
    obstacle_map = build_static_obstacle_map(unrouted_layout, config=resolved_obstacle_config)
    if debug_timing:
        t_obstacle_end = time.perf_counter()
        print(f"      - Obstacle Map time: {t_obstacle_end - t_obstacle_start:.4f} s")
    grid = obstacle_map.grid

    debug_path = Path(debug_dir) if debug_dir is not None else None
    diagnostics_enabled = debug_path is not None
    obstacle_svg = None
    route_svgs: list[Path] = []

    if debug_path is not None:
        obstacle_dir = debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)

    nets = schematic.netlist.routes
    if debug_route_indices is None:
        print(f"\nRouting {len(nets)} nets using Rust router...")
    else:
        selected = _format_route_indices(debug_route_indices)
        print(
            f"\nRouting {len(nets)} nets using Rust router "
            f"(printing/exporting route SVGs for indices: {selected})..."
        )

    if not hasattr(rust_backend, "PyPhotonicRouter"):
        raise RuntimeError(
            "Rust backend does not expose PyPhotonicRouter. "
            "Rebuild/install the Rust extension with the class-based API."
        )

    origin_x_um, origin_y_um = _grid_origin_xy(grid)
    grid_spec = rust_backend.GridSpec(
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        origin_x_um,
        origin_y_um,
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid.grid_size_um),
        bend_radius_cells=4,
        allow_45_degree_turns=allow_45_degree_turns,
    )
    bend_radius_cells = int(primitive_cfg.bend_radius_cells)
    astar_cfg = rust_backend.AStarConfig(max_iterations=int(max_iterations))
    astar_cfg.enable_jps4 = bool(enable_jps4)
    astar_cfg.use_indexed_heap = bool(use_indexed_heap)
    astar_cfg.primitive_ordering = str(primitive_ordering)
    astar_cfg.heuristic_mode = str(heuristic_mode)
    astar_cfg.heap_tie_breaker = str(heap_tie_breaker)
    if routing_window_scale is not None:
        astar_cfg.routing_window_scale = float(routing_window_scale)
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    block_radius_cells = max(
        0, math.ceil((float(route_width_um) / 2.0) / float(grid.grid_size_um))
    )
    port_entry_length_cells = max(2, bend_radius_cells + 2)
    port_entry_half_width_cells = max(1, bend_radius_cells + block_radius_cells + 1)
    port_lane_length_cells = max(3, 2 * bend_radius_cells + 2)
    port_lane_half_width_cells = max(1, block_radius_cells + 1)
    def _orientation_to_angle(orientation: float | None, *, flip: bool = False) -> int:
        if orientation is None:
            angle = 0
        else:
            angle = int(round((float(orientation) % 360.0) / 45.0)) % 8

        if flip:
            angle = (angle + 4) % 8

        return angle


    def _angle_to_step(angle: int) -> tuple[int, int]:
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
        dir_x, dir_y = _angle_to_step(source_angle)
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
        *,
        source_x: int,
        source_y: int,
        source_angle: int,
        target_x: int,
        target_y: int,
        target_angle: int,
    ) -> tuple[float, float]:
        grid_size_um = float(grid.grid_size_um)
        dx = target_x - source_x
        dy = target_y - source_y
        distance = math.hypot(float(dx), float(dy)) * grid_size_um
        heading_lower_bound = distance
        if str(heuristic_mode) == "heading_aware":
            target_angle_ok = not bool(getattr(astar_cfg, "require_target_angle", True)) or (
                source_angle % 8 == target_angle % 8
            )
            reaches_target_ray = _direction_reaches_target_ray(
                source_x=source_x,
                source_y=source_y,
                source_angle=source_angle,
                target_x=target_x,
                target_y=target_y,
                tolerance=max(0, int(getattr(astar_cfg, "target_tolerance_cells", 0))),
            )
            if not target_angle_ok or not reaches_target_ray:
                minimum_bend_units = 1.0 if allow_45_degree_turns else 2.0
                bend_weight = float(getattr(astar_cfg, "bend_weight", 1.0)) * float(
                    getattr(primitive_cfg, "bend_weight", 1.0)
                )
                heading_lower_bound += minimum_bend_units * bend_weight
        return distance, heading_lower_bound

    def _in_bounds(gx: int, gy: int) -> bool:
        return 0 <= gx < int(grid.width) and 0 <= gy < int(grid.height)

    def port_to_grid_state(
        port: Port,
        grid_origin_x_um: float,
        grid_origin_y_um: float,
        grid_size_um: float,
        *,
        as_target: bool = False,
        outward_cells: int = 1,
    ):
        port_angle = _orientation_to_angle(port.orientation, flip=False)

        # For choosing the grid cell, always move outward from the physical port.
        # This avoids starting inside the real component/port geometry.
        sx, sy = _angle_to_step(port_angle)

        x = float(port.center[0]) + sx * outward_cells * grid_size_um
        y = float(port.center[1]) + sy * outward_cells * grid_size_um

        gx = int((x - grid_origin_x_um) // grid_size_um)
        gy = int((y - grid_origin_y_um) // grid_size_um)

        # For the route state angle:
        # - source: route leaves the port outward
        # - target: route approaches the port, so flip direction
        route_angle = _orientation_to_angle(port.orientation, flip=as_target)

        return rust_backend.State(gx, gy, route_angle)

    def _collect_inflated_step_cells(
        base_x: int,
        base_y: int,
        *,
        step_x: int,
        step_y: int,
        length_cells: int,
        half_width_cells: int,
    ) -> set[tuple[int, int]]:
        cells: set[tuple[int, int]] = set()
        for step_idx in range(length_cells):
            cx = base_x + step_x * step_idx
            cy = base_y + step_y * step_idx
            if not _in_bounds(cx, cy):
                continue
            for dx in range(-half_width_cells, half_width_cells + 1):
                for dy in range(-half_width_cells, half_width_cells + 1):
                    nx = cx + dx
                    ny = cy + dy
                    if _in_bounds(nx, ny):
                        cells.add((nx, ny))
        return cells

    port_open_radius_um = _as_float(
        getattr(resolved_obstacle_config, "port_open_radius_um", 0.5),
        0.5,
    )
    port_open_radius_cells = max(
        0,
        math.ceil(port_open_radius_um / float(grid.grid_size_um)),
    )

    def build_port_access_cells(
        port: Port,
        *,
        access_length_um: float | None = None,
        access_width_um: float | None = None,
    ) -> set[tuple[int, int]]:
        state = port_to_grid_state(
            port,
            origin_x_um,
            origin_y_um,
            float(grid.grid_size_um),
            as_target=False,
        )
        port_angle = _orientation_to_angle(port.orientation, flip=False)
        sx, sy = _angle_to_step(port_angle)
        base_x = int(state.x)
        base_y = int(state.y)
        if access_length_um is not None or access_width_um is not None:
            length_cells = max(
                1,
                math.ceil(
                    max(0.0, _as_float(access_length_um, 0.0))
                    / float(grid.grid_size_um),
                ),
            )
            half_width_cells = max(
                0,
                math.ceil(
                    (max(0.0, _as_float(access_width_um, 0.0)) / 2.0)
                    / float(grid.grid_size_um),
                ),
            )
            cells = _collect_inflated_step_cells(
                base_x,
                base_y,
                step_x=sx,
                step_y=sy,
                length_cells=length_cells,
                half_width_cells=half_width_cells,
            )
            cells.add((int(state.x), int(state.y)))
            return cells

        entry_zone = _collect_inflated_step_cells(
            base_x,
            base_y,
            step_x=sx,
            step_y=sy,
            length_cells=port_entry_length_cells,
            half_width_cells=port_entry_half_width_cells,
        )
        lane_zone = _collect_inflated_step_cells(
            base_x,
            base_y,
            step_x=sx,
            step_y=sy,
            length_cells=port_lane_length_cells,
            half_width_cells=port_lane_half_width_cells,
        )
        cells = entry_zone | lane_zone
        cells.add((int(state.x), int(state.y)))
        return cells

    def build_base_port_open_cells(port: Port) -> set[tuple[int, int]]:
        center = getattr(port, "center", None)
        if center is None:
            center = getattr(port, "dcenter", None)
        if center is None:
            return set()
        base_x = int((float(center[0]) - origin_x_um) // float(grid.grid_size_um))
        base_y = int((float(center[1]) - origin_y_um) // float(grid.grid_size_um))
        return _collect_inflated_step_cells(
            base_x,
            base_y,
            step_x=0,
            step_y=0,
            length_cells=1,
            half_width_cells=port_open_radius_cells,
        )

    def build_keyed_port_access_cells(
        *,
        instance_name: str,
        port_name: str,
        port: Port,
    ) -> tuple[set[tuple[int, int]], set[tuple[int, int]], str | None]:
        port_type = _port_type_name(port)
        component_name = _schematic_instance_component_name(schematic, instance_name)
        rule = (
            find_component_port_access_rule(
                component_name=component_name,
                port_name=port_name,
                port_type=port_type,
            )
            if include_heater_obstacles
            else None
        )
        if rule is not None:
            cells = build_port_access_cells(
                port,
                access_length_um=rule.access_length_um,
                access_width_um=rule.access_width_um,
            )
            return cells, cells, rule.component_name_pattern

        if port_type is not None and port_type != "optical":
            return set(), set(), None

        candidate_cells = build_port_access_cells(port)
        effective_cells = candidate_cells & build_base_port_open_cells(port)
        return effective_cells, candidate_cells, None

    route_jobs: list[RouteJob] = []
    port_access_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_rule_by_spec: dict[str, str | None] = {}
    next_net_id = 1
    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")
            source_port = get_port_from_instance(routed_layout, inst1, port1)
            target_port = get_port_from_instance(routed_layout, inst2, port2)
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
            if port1_spec not in port_access_cells_by_spec:
                cells, candidate_cells, rule_name = build_keyed_port_access_cells(
                    instance_name=inst1,
                    port_name=port1,
                    port=source_port,
                )
                port_access_cells_by_spec[port1_spec] = cells
                port_access_candidate_cells_by_spec[port1_spec] = candidate_cells
                port_access_rule_by_spec[port1_spec] = rule_name
            if port2_spec not in port_access_cells_by_spec:
                cells, candidate_cells, rule_name = build_keyed_port_access_cells(
                    instance_name=inst2,
                    port_name=port2,
                    port=target_port,
                )
                port_access_cells_by_spec[port2_spec] = cells
                port_access_candidate_cells_by_spec[port2_spec] = candidate_cells
                port_access_rule_by_spec[port2_spec] = rule_name

    # Use raw static geometry as the baseline truth for "real component body"
    # checks. `blocked_cells` may exclude port-open carve-outs.
    raw_blocked_obj: object
    if hasattr(obstacle_map, "raw_blocked_cells"):
        raw_blocked_obj = getattr(obstacle_map, "raw_blocked_cells")
    else:
        raw_blocked_obj = obstacle_map.blocked_cells
    raw_blocked_cells = cast(Iterable[tuple[int, int]], raw_blocked_obj)
    raw_static_cells = {(int(cell[0]), int(cell[1])) for cell in raw_blocked_cells}
    static_blocked_cells_before_port_reservations = raw_static_cells
    blocked_static_rects_for_diagnostics: list[tuple[int, int, int, int]] = []
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
        blocked_static_rects_for_diagnostics = list(blocked_static_rects)
        if blocked_static_rects:
            if not hasattr(router, "set_static_rects"):
                raise RuntimeError(
                    "The loaded photonic_router._rust extension does not expose "
                    "PyPhotonicRouter.set_static_rects. Rebuild it with "
                    "`maturin develop --release`; otherwise bounding_boxes mode "
                    "cannot use compact static rectangles."
                )
            router.set_static_rects(blocked_static_rects)
        else:
            router.set_static_cells(sorted(static_blocked_cells_before_port_reservations))
    else:
        static_cells = set(static_blocked_cells_before_port_reservations)
        sorted_static_cells = sorted(static_cells)
        router.set_static_cells(sorted_static_cells)
    repair_config = ripup_reroute_config or RipupRerouteConfig()
    route_jobs_by_id = {job.net_id: job for job in route_jobs}
    route_order = [job.net_id for job in route_jobs]
    collect_timing = debug_timing or collect_route_stats or collect_attempt_diagnostics
    track_dynamic_cells = diagnostics_enabled
    route_bookkeeping = RouteBookkeeping(
        route_order=route_order,
        diagnostics_enabled=track_dynamic_cells,
    )

    t_astar_start = 0.0
    if collect_timing:
        t_astar_start = time.perf_counter()
    total_expanded_states = 0
    simple_route_count = 0
    repair_count = 0
    route_attempt_records = []
    route_timing_buckets: dict[str, RouteTimingBucket] = {
        name: RouteTimingBucket()
        for name in (
            "normal_route",
            "probe_route",
            "repair_failed_net",
            "reroute_victims",
            "snapshot_cells",
            "history_update",
            "owner_lookup",
            "ripup",
            "rollback",
        )
    }

    def _timing_start() -> float:
        return time.perf_counter() if collect_timing else 0.0

    def _record_elapsed(bucket_name: str, start_s: float, *, failed: bool = False) -> None:
        if not collect_timing:
            return
        route_timing_buckets[bucket_name].record_elapsed(
            time.perf_counter() - start_s,
            failed=failed,
        )

    def _record_route_attempt(
        job: RouteJob,
        bucket_name: str,
        start_s: float,
        route_obj: object | None = None,
        *,
        failed: bool = False,
        repair_round: int | None = None,
        error_text: str | None = None,
        diagnostics: dict[str, object] | None = None,
        candidate_blockers: list[int] | None = None,
        ripup_ids: list[int] | None = None,
    ) -> None:
        if not collect_timing:
            return
        elapsed_s = time.perf_counter() - start_s
        if route_obj is None:
            route_timing_buckets[bucket_name].record_elapsed(
                elapsed_s,
                failed=failed,
            )
        else:
            route_timing_buckets[bucket_name].record_route(
                elapsed_s,
                route_obj,
                failed=failed,
            )
        if diagnostics is None:
            generated_neighbors = (
                int(getattr(route_obj, "generated_neighbors", 0))
                if route_obj is not None
                else 0
            )
            if collect_attempt_diagnostics and (
                failed or elapsed_s >= 0.01 or generated_neighbors >= 100_000
            ):
                diagnostics = _route_attempt_diagnostics(
                    job,
                    route_obj,
                    candidate_blockers=candidate_blockers,
                    ripup_ids=ripup_ids,
                )
        route_attempt_records.append(
            route_attempt_record_from_route(
                attempt_index=len(route_attempt_records) + 1,
                bucket_name=bucket_name,
                net_id=job.net_id,
                route_index=job.route_index,
                net_name=job.net_name,
                source=f"{job.inst1},{job.port1}",
                target=f"{job.inst2},{job.port2}",
                elapsed_s=elapsed_s,
                route_obj=route_obj,
                failed=failed,
                repair_round=repair_round,
                error=error_text,
                diagnostics=diagnostics,
            )
        )

    def _committed_dynamic_cells(*, exclude_net_id: int | None = None) -> set[tuple[int, int]]:
        return route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)

    def _committed_dynamic_cells_for_attempt(
        *,
        exclude_net_id: int | None = None,
    ) -> set[tuple[int, int]]:
        if route_bookkeeping.diagnostics_enabled:
            return route_bookkeeping.committed_dynamic_cells(exclude_net_id=exclude_net_id)
        merged: set[tuple[int, int]] = set()
        for net_id in route_bookkeeping.records_by_id:
            if exclude_net_id is not None and int(net_id) == int(exclude_net_id):
                continue
            merged.update(_route_cells_from_router(net_id))
        return merged

    def _route_cells_from_router(net_id: int) -> set[tuple[int, int]]:
        return {
            (int(cell[0]), int(cell[1]))
            for cell in router.get_net_cells(int(net_id))
        }

    def _states_and_openings(
        job: RouteJob,
    ) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
        source_state = port_to_grid_state(
            job.source_port,
            origin_x_um,
            origin_y_um,
            float(grid.grid_size_um),
            as_target=False,
        )
        target_state = port_to_grid_state(
            job.target_port,
            origin_x_um,
            origin_y_um,
            float(grid.grid_size_um),
            as_target=True,
        )
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        opened_candidate_cells = set(port_access_candidate_cells_by_spec.get(port1_spec, set()))
        opened_candidate_cells.update(port_access_candidate_cells_by_spec.get(port2_spec, set()))
        opened_candidate_cells.update({source_anchor_cell, target_anchor_cell})

        opened_cells_set = set(port_access_cells_by_spec.get(port1_spec, set()))
        opened_cells_set.update(port_access_cells_by_spec.get(port2_spec, set()))
        opened_cells_set.update({source_anchor_cell, target_anchor_cell})
        return (
            source_state,
            target_state,
            opened_candidate_cells,
            opened_cells_set,
            sorted(opened_cells_set),
        )

    def _static_cells_in_rect(min_x: int, max_x: int, min_y: int, max_y: int) -> int:
        if min_x > max_x or min_y > max_y:
            return 0
        if blocked_static_rects_for_diagnostics:
            return sum(
                _rect_overlap_cell_count(
                    rect,
                    min_x=min_x,
                    max_x=max_x,
                    min_y=min_y,
                    max_y=max_y,
                )
                for rect in blocked_static_rects_for_diagnostics
            )
        return sum(
            1
            for x, y in static_blocked_cells_before_port_reservations
            if min_x <= x <= max_x and min_y <= y <= max_y
        )

    def _cells_in_rect(
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
        job: RouteJob,
        route_obj: object | None,
        *,
        candidate_blockers: list[int] | None = None,
        ripup_ids: list[int] | None = None,
    ) -> dict[str, object]:
        dynamic_cells_before = _committed_dynamic_cells_for_attempt(exclude_net_id=job.net_id)
        source_state, target_state, _, _, opened_cells = _states_and_openings(job)
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
        window_static_cells = _static_cells_in_rect(
            window_min_x,
            window_max_x,
            window_min_y,
            window_max_y,
        )
        window_dynamic_cells = _cells_in_rect(
            dynamic_cells_before,
            min_x=window_min_x,
            max_x=window_max_x,
            min_y=window_min_y,
            max_y=window_max_y,
        )
        span_static_cells = _static_cells_in_rect(
            span_bbox_min_x,
            span_bbox_max_x,
            span_bbox_min_y,
            span_bbox_max_y,
        )
        span_dynamic_cells = _cells_in_rect(
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
        total_cost = (
            float(getattr(route_obj, "total_cost", 0.0))
            if route_obj is not None
            else None
        )
        euclidean_lower_bound, heading_lower_bound = _source_lower_bounds(
            source_x=source_x,
            source_y=source_y,
            source_angle=source_angle,
            target_x=target_x,
            target_y=target_y,
            target_angle=target_angle,
        )
        blocker_ids = list(candidate_blockers or [])
        victim_ids = list(ripup_ids or [])
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
            "block_radius_cells": block_radius_cells,
            "bend_radius_cells": bend_radius_cells,
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
                float(window_static_cells) / float(window_area)
                if window_area > 0
                else None
            ),
            "window_dynamic_density": (
                float(window_dynamic_cells) / float(window_area)
                if window_area > 0
                else None
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
                total_cost - heading_lower_bound
                if total_cost is not None
                else None
            ),
            "committed_dynamic_cells_before": len(dynamic_cells_before),
            "candidate_blocker_count": len(blocker_ids),
            "candidate_blocker_net_ids": blocker_ids,
            "candidate_blocker_route_indices": [
                route_jobs_by_id[net_id].route_index
                for net_id in blocker_ids
                if net_id in route_jobs_by_id
            ],
            "ripup_victim_count": len(victim_ids),
            "ripup_victim_net_ids": victim_ids,
            "ripup_victim_route_indices": [
                route_jobs_by_id[net_id].route_index
                for net_id in victim_ids
                if net_id in route_jobs_by_id
            ],
        }

    def _write_route_diagnostics(
        *,
        job: RouteJob,
        source_state: Any,
        target_state: Any,
        opened_candidate_cells: set[tuple[int, int]],
        opened_cells_set: set[tuple[int, int]],
        diag_txt: Path | None,
        status: str,
        error_text: str | None = None,
        route_cells: set[tuple[int, int]] | None = None,
        repair_note: str | None = None,
    ) -> None:
        if diag_txt is None:
            return
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        committed_dynamic_cells = _committed_dynamic_cells(exclude_net_id=job.net_id)
        if diagnostics_enabled:
            opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
            opened_candidate_static_overlap = (
                opened_candidate_cells & static_blocked_cells_before_port_reservations
            )
            opened_static_overlap = (
                opened_cells_set & static_blocked_cells_before_port_reservations
            )
            opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
        else:
            opened_candidate_dynamic_overlap = set()
            opened_candidate_static_overlap = set()
            opened_static_overlap = set()
            opened_dynamic_overlap = set()

        route_cells = route_cells or set()
        route_static_overlap = route_cells & static_blocked_cells_before_port_reservations
        route_overlap_with_candidate_opened_static = (
            route_cells & opened_candidate_static_overlap
        )
        route_overlap_with_effective_opened_static = route_cells & opened_static_overlap
        route_dynamic_overlap = route_cells & committed_dynamic_cells
        route_overlap_with_candidate_opened_dynamic = (
            route_cells & opened_candidate_dynamic_overlap
        )
        route_overlap_with_effective_opened_dynamic = route_cells & opened_dynamic_overlap
        lines = [
            f"net_name={job.net_name}",
            f"status={status}",
            f"source_spec={port1_spec}",
            f"target_spec={port2_spec}",
            f"source_component={_schematic_instance_component_name(schematic, job.inst1)}",
            f"target_component={_schematic_instance_component_name(schematic, job.inst2)}",
            f"source_access_rule={port_access_rule_by_spec.get(port1_spec)}",
            f"target_access_rule={port_access_rule_by_spec.get(port2_spec)}",
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
            f"route_cells_count={len(route_cells)}",
            f"route_static_blocked_overlap_count={len(route_static_overlap)}",
            f"route_static_blocked_overlap_bbox={_cells_bbox(route_static_overlap)}",
            f"route_dynamic_overlap_count={len(route_dynamic_overlap)}",
            f"route_dynamic_overlap_bbox={_cells_bbox(route_dynamic_overlap)}",
            f"route_overlap_candidate_opened_static_count={len(route_overlap_with_candidate_opened_static)}",
            f"route_overlap_effective_opened_static_count={len(route_overlap_with_effective_opened_static)}",
            f"route_overlap_candidate_opened_dynamic_count={len(route_overlap_with_candidate_opened_dynamic)}",
            f"route_overlap_effective_opened_dynamic_count={len(route_overlap_with_effective_opened_dynamic)}",
        ]
        if repair_note is not None:
            lines.append(f"repair={repair_note}")
        if error_text is not None:
            lines.append(f"error={error_text}")
        diag_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _record_route(job: RouteJob, route_obj: Any, opened_cells: list[tuple[int, int]]) -> None:
        route_bookkeeping.record_route(
            job,
            route_obj,
            opened_cells,
            route_cells=_route_cells_from_router(job.net_id) if track_dynamic_cells else None,
        )

    def _route_and_commit(
        job: RouteJob,
        *,
        repair: bool,
        timing_bucket: str,
        repair_round: int | None = None,
        candidate_blockers: list[int] | None = None,
        ripup_ids: list[int] | None = None,
    ) -> tuple[Any, list[tuple[int, int]]]:
        source_state, target_state, _, _, opened_cells = _states_and_openings(job)
        route_start = _timing_start()
        if repair:
            try:
                route_obj = router.route_single_net_and_commit_repair(
                    job.net_id,
                    source_state,
                    target_state,
                    block_radius_cells,
                    opened_cells,
                    float(repair_config.history_weight),
                )
            except RuntimeError as exc:
                _record_route_attempt(
                    job,
                    timing_bucket,
                    route_start,
                    failed=True,
                    repair_round=repair_round,
                    error_text=str(exc),
                    candidate_blockers=candidate_blockers,
                    ripup_ids=ripup_ids,
                )
                raise
        else:
            try:
                route_obj = router.route_single_net_and_commit(
                    job.net_id,
                    source_state,
                    target_state,
                    block_radius_cells,
                    opened_cells,
                )
            except RuntimeError as exc:
                _record_route_attempt(
                    job,
                    timing_bucket,
                    route_start,
                    failed=True,
                    repair_round=repair_round,
                    error_text=str(exc),
                    candidate_blockers=candidate_blockers,
                    ripup_ids=ripup_ids,
                )
                raise
        _record_route_attempt(
            job,
            timing_bucket,
            route_start,
            route_obj,
            repair_round=repair_round,
            candidate_blockers=candidate_blockers,
            ripup_ids=ripup_ids,
        )
        _record_route(job, route_obj, opened_cells)
        return route_obj, opened_cells

    def _export_route_svg(job: RouteJob, route_obj: Any, *, suffix: str = "") -> None:
        should_export = (
            debug_path is not None
            and (debug_route_indices is None or job.route_index in debug_route_indices)
        )
        if not should_export:
            return
        route_dir = debug_path / "routes"
        _ensure_dir(route_dir)
        route_svg = route_dir / f"{debug_prefix}_{job.net_name}{suffix}.svg"
        route_svg.write_text(router.export_debug_svg(route_obj), encoding="utf-8")
        route_svgs.append(route_svg)

    def _write_failed_log(
        job: RouteJob,
        source_state: Any,
        target_state: Any,
        opened_candidate_cells: set[tuple[int, int]],
        opened_cells: list[tuple[int, int]],
        error_text: str,
    ) -> None:
        if debug_path is None:
            return
        route_dir = debug_path / "routes"
        _ensure_dir(route_dir)
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        fail_txt = route_dir / f"{debug_prefix}_{job.net_name}_FAILED.txt"
        committed_dynamic_cells = _committed_dynamic_cells()
        opened_candidate_static_overlap = (
            opened_candidate_cells & static_blocked_cells_before_port_reservations
        )
        opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
        opened_cells_set = set(opened_cells)
        opened_static_overlap = opened_cells_set & static_blocked_cells_before_port_reservations
        opened_dynamic_overlap = opened_cells_set & committed_dynamic_cells
        fail_lines = [
            f"net_name={job.net_name}",
            f"source_spec={port1_spec}",
            f"target_spec={port2_spec}",
            f"source_state=({int(source_state.x)}, {int(source_state.y)}, {int(source_state.angle)})",
            f"target_state=({int(target_state.x)}, {int(target_state.y)}, {int(target_state.angle)})",
            f"allow_45_degree_turns={allow_45_degree_turns}",
            f"block_radius_cells={block_radius_cells}",
            f"bend_radius_cells={bend_radius_cells}",
            f"port_entry_length_cells={port_entry_length_cells}",
            f"port_entry_half_width_cells={port_entry_half_width_cells}",
            f"port_lane_length_cells={port_lane_length_cells}",
            f"port_lane_half_width_cells={port_lane_half_width_cells}",
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
            f"error={error_text}",
        ]
        fail_txt.write_text("\n".join(fail_lines) + "\n", encoding="utf-8")

    def _restore_snapshot(
        snapshot_records: dict[int, RoutedNetRecord],
        snapshot_lengths: dict[int, float],
        snapshot_cells: dict[int, set[tuple[int, int]]],
        touched_ids: set[int],
    ) -> None:
        for net_id_to_clear in touched_ids:
            router.ripup_route(net_id_to_clear)
            route_bookkeeping.clear_route(net_id_to_clear)
        for old_id, cells in snapshot_cells.items():
            if not router.commit_route_cells(old_id, sorted(cells)):
                raise RuntimeError(f"Failed to rollback route cells for net id {old_id}")
            route_bookkeeping.set_committed_cells(old_id, cells)
        route_bookkeeping.restore_records(snapshot_records, snapshot_lengths)

    def _attempt_repair(job: RouteJob, failed_error: Exception) -> bool:
        nonlocal repair_count, total_expanded_states, simple_route_count
        if not repair_config.enabled:
            return False
        source_state, target_state, _, _, opened_cells = _states_and_openings(job)
        probe_start = _timing_start()
        try:
            probe_route = router.route_single_net_ignore_dynamic(
                source_state,
                target_state,
                block_radius_cells,
                opened_cells,
            )
        except RuntimeError as exc:
            _record_route_attempt(
                job,
                "probe_route",
                probe_start,
                failed=True,
                error_text=str(exc),
            )
            return False
        _record_route_attempt(
            job,
            "probe_route",
            probe_start,
            probe_route,
        )

        history_start = _timing_start()
        router.add_history_for_route(
            probe_route,
            block_radius_cells,
            int(repair_config.history_increment),
        )
        _record_elapsed("history_update", history_start)
        owner_lookup_start = _timing_start()
        candidate_blockers = [
            int(owner)
            for owner in router.dynamic_owners_for_route(probe_route, block_radius_cells)
            if int(owner) != job.net_id and int(owner) in route_bookkeeping.records_by_id
        ]
        _record_elapsed("owner_lookup", owner_lookup_start)
        if not candidate_blockers:
            return False

        candidate_blockers = sorted(
            dict.fromkeys(candidate_blockers),
            key=lambda owner: route_jobs_by_id[owner].route_index,
        )
        max_victims = max(1, int(repair_config.max_victims_per_failure))
        max_rounds = max(1, int(repair_config.max_rounds))

        for round_idx in range(1, max_rounds + 1):
            ripup_ids = candidate_blockers[: min(len(candidate_blockers), max_victims * round_idx)]
            if not ripup_ids:
                return False
            snapshot_records = {
                old_id: route_bookkeeping.records_by_id[old_id]
                for old_id in ripup_ids
                if old_id in route_bookkeeping.records_by_id
            }
            snapshot_lengths = {
                old_id: route_bookkeeping.lengths_by_id[old_id]
                for old_id in ripup_ids
                if old_id in route_bookkeeping.lengths_by_id
            }
            snapshot_start = _timing_start()
            snapshot_cells = {
                old_id: _route_cells_from_router(old_id)
                for old_id in ripup_ids
            }
            _record_elapsed("snapshot_cells", snapshot_start)
            touched_ids = set(ripup_ids)
            touched_ids.add(job.net_id)
            try:
                for old_id in ripup_ids:
                    record = route_bookkeeping.records_by_id.get(old_id)
                    if record is not None:
                        history_start = _timing_start()
                        router.add_history_for_route(
                            record.route_obj,
                            block_radius_cells,
                            int(repair_config.history_increment),
                        )
                        _record_elapsed("history_update", history_start)
                    ripup_start = _timing_start()
                    router.ripup_route(old_id)
                    _record_elapsed("ripup", ripup_start)
                    route_bookkeeping.clear_route(old_id)

                repaired_route, _ = _route_and_commit(
                    job,
                    # Route the originally failed net first with simple L/Z
                    # candidates enabled. The blocker victims below still use
                    # repair A* with history so they avoid recreating the
                    # conflict that caused this rip-up.
                    repair=False,
                    timing_bucket="repair_failed_net",
                    repair_round=round_idx,
                    candidate_blockers=candidate_blockers,
                    ripup_ids=ripup_ids,
                )
                total_expanded_states += int(getattr(repaired_route, "expanded_states", 0))
                if int(getattr(repaired_route, "expanded_states", 0)) == 0:
                    simple_route_count += 1

                for old_id in ripup_ids:
                    reroute_job = route_jobs_by_id[old_id]
                    rerouted_obj, _ = _route_and_commit(
                        reroute_job,
                        repair=True,
                        timing_bucket="reroute_victims",
                        repair_round=round_idx,
                        candidate_blockers=candidate_blockers,
                        ripup_ids=ripup_ids,
                    )
                    total_expanded_states += int(getattr(rerouted_obj, "expanded_states", 0))
                    if int(getattr(rerouted_obj, "expanded_states", 0)) == 0:
                        simple_route_count += 1
                    _export_route_svg(
                        reroute_job,
                        rerouted_obj,
                        suffix=f"_repair{round_idx}",
                    )

                _export_route_svg(job, repaired_route, suffix=f"_repair{round_idx}")
                repair_count += 1
                return True
            except RuntimeError:
                rollback_start = _timing_start()
                _restore_snapshot(snapshot_records, snapshot_lengths, snapshot_cells, touched_ids)
                _record_elapsed("rollback", rollback_start)
                continue

        _ = failed_error
        return False

    for job in route_jobs:
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_state, target_state, opened_candidate_cells, opened_cells_set, opened_cells = (
            _states_and_openings(job)
        )
        should_print_route = (
            debug_route_indices is None or job.route_index in debug_route_indices
        )
        should_export_route_debug = (
            debug_path is not None
            and (debug_route_indices is None or job.route_index in debug_route_indices)
        )
        route_progress_text = (
            f"  Routing [{job.route_index}/{len(route_jobs)}] "
            f"{job.net_name}: {port1_spec} -> {port2_spec}..."
        )
        if should_print_route:
            print(route_progress_text, end=" ")

        route_dir = debug_path / "routes" if debug_path is not None else None
        diag_txt: Path | None = None
        if should_export_route_debug and route_dir is not None:
            _ensure_dir(route_dir)
            diag_txt = route_dir / f"{debug_prefix}_{job.net_name}_diagnostics.txt"

        try:
            route_obj, opened_cells = _route_and_commit(
                job,
                repair=False,
                timing_bucket="normal_route",
            )
        except RuntimeError as exc:
            if _attempt_repair(job, exc):
                if should_print_route:
                    print("repaired")
                continue
            if not should_print_route:
                print(f"{route_progress_text} failed")
            _write_route_diagnostics(
                job=job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="failed",
                error_text=str(exc),
            )
            _write_failed_log(
                job,
                source_state,
                target_state,
                opened_candidate_cells,
                opened_cells,
                str(exc),
            )
            raise RuntimeError(
                f"No route found for {job.net_name}: {port1_spec} -> {port2_spec}. "
                f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                f"allow_45_degree_turns={allow_45_degree_turns}"
            ) from exc

        expanded_states = int(getattr(route_obj, "expanded_states", 0))
        total_expanded_states += expanded_states
        if expanded_states == 0:
            simple_route_count += 1

        if diagnostics_enabled:
            route_cells = {
                (int(cell[0]), int(cell[1]))
                for cell in (getattr(route_obj, "cells", None) or [])
            }
            _write_route_diagnostics(
                job=job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="ok",
                route_cells=route_cells,
            )

        _export_route_svg(job, route_obj)

        if should_print_route:
            print("ok")

    routed_net_records = route_bookkeeping.ordered_records()

    astar_elapsed_s = 0.0
    if collect_timing:
        astar_elapsed_s = time.perf_counter() - t_astar_start

    if debug_timing:
        print(f"      - Astar time: {astar_elapsed_s:.4f} s")
        print(
            "      - Route search stats: "
            f"simple={simple_route_count}/{len(route_jobs)}, "
            f"expanded_states={total_expanded_states}, "
            f"repairs={repair_count}"
        )
        print("      - Route timing breakdown:")
        for bucket_name in (
            "normal_route",
            "probe_route",
            "repair_failed_net",
            "reroute_victims",
            "snapshot_cells",
            "history_update",
            "owner_lookup",
            "ripup",
            "rollback",
        ):
            bucket = route_timing_buckets[bucket_name]
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
                    f"neighbor_time={bucket.neighbor_generation_time_us / 1_000_000.0:.4f}s, "
                    f"heap_time={bucket.heap_operation_time_us / 1_000_000.0:.4f}s, "
                    f"legality_time={bucket.legality_check_time_us / 1_000_000.0:.4f}s, "
                    f"reconstruction_time={bucket.reconstruction_time_us / 1_000_000.0:.4f}s, "
                    f"full_grid_fallbacks={bucket.full_grid_fallbacks}"
                )
            print(line)

    realization_grid_spec = (
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )
    if not defer_realization:
        realize_routed_net_records(
            routed_layout,
            routed_net_records,
            route_width_um=route_width_um,
            route_layer=route_layer,
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=bend_radius_cells,
        )

    return routed_layout, build_route_debug_artifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
        obstacle_map=obstacle_map,
        routed_net_records=routed_net_records,
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_cells=bend_radius_cells,
        route_search_summary=summarize_route_search(
            route_timing_buckets,
            route_count=len(route_jobs),
            simple_route_count=simple_route_count,
            repair_count=repair_count,
            astar_elapsed_s=astar_elapsed_s,
        ),
        route_attempt_records=route_attempt_records,
    )
