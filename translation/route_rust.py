"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass, field, is_dataclass, replace
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
from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    PathLengthAnalysisResult,
    NodeTiming,
    PortRef,
    RoutedEdgeKey,
    SchematicLike,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)
from photonic_router.routing_layers import (
    find_component_port_access_rule,
    get_routing_obstacle_layers,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend

MEANDER_DEPTH_CANDIDATES_UM = (
    40.0,
    30.0,
    24.0,
    20.0,
    16.0,
    12.0,
    10.0,
    8.0,
    6.0,
    4.0,
    3.0,
    2.0,
)
EXACT_MEANDER_EPS_UM = 1.0e-6


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


@dataclass(frozen=True)
class RustRouteDebugArtifacts:
    obstacle_svg: Path | None
    route_svgs: list[Path]
    routed_edge_lengths_um: dict[RoutedEdgeKey, float]
    routed_net_records: list["RoutedNetRecord"] = field(default_factory=list)
    static_blocked_cells: tuple[tuple[int, int], ...] = ()
    static_obstacle_count: int = 0
    realization_grid_spec: tuple[int, int, float, float, float] | None = None
    realization_allow_45_degree_turns: bool = True
    realization_bend_radius_cells: int = 4


@dataclass(frozen=True)
class RoutedNetRecord:
    net_name: str
    source: PortRef
    target: PortRef
    route_obj: object
    total_length_um: float
    meander_auto_plan: dict[str, object] | None = None
    opened_cells: tuple[tuple[int, int], ...] = ()


@dataclass(frozen=True)
class MeanderInsertionConfig:
    enabled: bool = True
    min_candidate_straight_length_um: float = 2.0
    max_extra_length_per_region_um: float = 200.0
    conservative_legal_check: bool = True
    max_meander_height_um: float = 20.0
    auto_meander_endpoint_inset_um: float | None = None


@dataclass(frozen=True)
class MeanderInsertionResult:
    edge: RoutedEdgeKey
    requested_extra_length_um: float
    inserted_extra_length_um: float
    status: str
    reason: str


@dataclass(frozen=True)
class MeanderInsertionReport:
    results: list[MeanderInsertionResult]
    total_requested_extra_length_um: float
    total_inserted_extra_length_um: float
    unmatched_length_um: float


@dataclass(frozen=True)
class RouteRustPipelineResult:
    routed_layout: Component
    debug_artifacts: RustRouteDebugArtifacts
    path_length_analysis_info: dict[str, object] | None = None
    meander_requirements_info: list[dict[str, object]] | None = None
    meander_insertion_report_info: dict[str, object] | None = None


def routed_net_records_to_edge_lengths(
    records: list[RoutedNetRecord],
) -> dict[RoutedEdgeKey, float]:
    """Convert routed net records into edge-length annotations."""
    return {
        RoutedEdgeKey(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
        ): float(record.total_length_um)
        for record in records
    }


def analyze_path_length_matching(
    schematic: SchematicLike,
    *,
    routed_net_records: list[RoutedNetRecord],
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
) -> tuple[PathLengthAnalysisResult, list]:
    """Phase M1: compute per-edge missing lengths before polygon realization."""
    graph = build_graph_from_schematic(
        schematic,
        node_types=node_types,
        internal_delays_um=internal_delays_um,
    )
    annotate_edge_lengths(graph, routed_net_records_to_edge_lengths(routed_net_records))
    analysis = graph.analyze_missing_lengths()
    return analysis, list(list_edges_requiring_meander(analysis))


def edge_key_to_dict(edge_key: RoutedEdgeKey) -> dict[str, object]:
    return {
        "net_name": edge_key.net_name,
        "source": {"instance": edge_key.source.instance, "port": edge_key.source.port},
        "target": {"instance": edge_key.target.instance, "port": edge_key.target.port},
    }


def requirement_to_dict(req: MissingLengthRequirement) -> dict[str, object]:
    return {
        "edge": edge_key_to_dict(req.edge_key),
        "missing_length_um": float(req.missing_length_um),
    }


def node_timing_to_dict(timing: NodeTiming) -> dict[str, object]:
    return {
        "node_name": timing.node_name,
        "node_type": timing.node_type.value,
        "internal_delay_um": float(timing.internal_delay_um),
        "input_arrival_um": float(timing.input_arrival_um),
        "output_arrival_um": float(timing.output_arrival_um),
        "incoming_edges": [
            {
                "edge": edge_key_to_dict(edge_timing.edge_key),
                "routed_length_um": float(edge_timing.routed_length_um),
                "edge_arrival_um": float(edge_timing.edge_arrival_um),
                "missing_length_um": float(edge_timing.missing_length_um),
            }
            for edge_timing in timing.incoming_edges
        ],
    }


def analysis_to_info_dict(analysis: PathLengthAnalysisResult) -> dict[str, object]:
    return {
        "topological_order": list(analysis.topological_order),
        "node_arrival_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_um.items()
        },
        "node_arrival_input_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_input_um.items()
        },
        "node_arrival_output_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_output_um.items()
        },
        "node_timings_um": {
            str(node): node_timing_to_dict(timing)
            for node, timing in analysis.node_timings.items()
        },
        "edge_missing_lengths_um": [
            {
                "edge": edge_key_to_dict(edge_key),
                "missing_length_um": float(missing),
            }
            for edge_key, missing in analysis.edge_missing_lengths_um.items()
        ],
        "requirements": [requirement_to_dict(req) for req in analysis.requirements],
    }


def _record_edge_key(record: RoutedNetRecord) -> RoutedEdgeKey:
    return RoutedEdgeKey(
        net_name=record.net_name,
        source=record.source,
        target=record.target,
    )


def _state_xy(state: object) -> tuple[int, int]:
    if hasattr(state, "x") and hasattr(state, "y"):
        return int(getattr(state, "x")), int(getattr(state, "y"))
    if isinstance(state, (tuple, list)) and len(state) >= 2:
        return int(state[0]), int(state[1])
    raise TypeError(f"Unsupported state representation: {type(state)}")


def _grid_to_um(
    x: int,
    y: int,
    realization_grid_spec: tuple[int, int, float, float, float],
) -> tuple[float, float]:
    _, _, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    return (
        float(origin_x_um) + (float(x) + 0.5) * float(grid_size_um),
        float(origin_y_um) + (float(y) + 0.5) * float(grid_size_um),
    )


def _as_float(value: object, default: float = 0.0) -> float:
    try:
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return float(value)
        return default
    except (TypeError, ValueError):
        return default


def _as_int(value: object, default: int = 0) -> int:
    try:
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return int(value)
        return default
    except (TypeError, ValueError):
        return default


def _minimum_four_bend_extra_length_um(
    *,
    grid_size_um: float,
    bend_radius_cells: int,
) -> float:
    """Minimum practical matching request: one bump needs four 90-degree bends."""
    bend_radius_um = max(0.0, float(grid_size_um) * float(bend_radius_cells))
    return 2.0 * math.pi * bend_radius_um


def analyze_meander_insertion_for_requirements(
    routed_net_records: list[RoutedNetRecord],
    requirements: list[MissingLengthRequirement],
    *,
    config: MeanderInsertionConfig,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    static_blocked_cells: list[tuple[int, int]] | tuple[tuple[int, int], ...] | None = None,
) -> tuple[list[RoutedNetRecord], dict[str, object]]:
    """Plan meander insertion using auto analytic multi-bump planning."""
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError("Rust router backend unavailable for meander analysis.")
    width, height, grid_size_um_cfg, origin_x_um, origin_y_um = realization_grid_spec
    grid_spec = rust_backend.GridSpec(
        int(width),
        int(height),
        float(grid_size_um_cfg),
        float(origin_x_um),
        float(origin_y_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid_size_um_cfg),
        bend_radius_cells=int(bend_radius_cells),
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar_cfg = rust_backend.AStarConfig(max_iterations=1)
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)
    base_static_cells: set[tuple[int, int]] = set(static_blocked_cells or [])

    def _record_route_cells(record: RoutedNetRecord) -> set[tuple[int, int]]:
        cells = getattr(record.route_obj, "cells", None) or []
        return {(int(x), int(y)) for x, y in cells}

    def _grid_rect_cells(grid_rect: object) -> set[tuple[int, int]]:
        if (
            not isinstance(grid_rect, (tuple, list))
            or len(grid_rect) != 4
        ):
            return set()
        min_x = _as_int(grid_rect[0], 0)
        max_x = _as_int(grid_rect[1], -1)
        min_y = _as_int(grid_rect[2], 0)
        max_y = _as_int(grid_rect[3], -1)
        if max_x < min_x or max_y < min_y:
            return set()
        return {
            (x, y)
            for x in range(min_x, max_x + 1)
            for y in range(min_y, max_y + 1)
        }

    reserved_meander_cells: set[tuple[int, int]] = set()

    by_edge = {_record_edge_key(r): r for r in routed_net_records}
    updated = dict(by_edge)
    route_cells_by_edge = {
        edge: _record_route_cells(record)
        for edge, record in by_edge.items()
    }
    route_cell_refcounts: Counter[tuple[int, int]] = Counter()
    for cells in route_cells_by_edge.values():
        route_cell_refcounts.update(cells)
    base_static_and_route_cells = set(base_static_cells)
    base_static_and_route_cells.update(route_cell_refcounts.keys())
    router.set_static_cells(list(base_static_and_route_cells))
    results: list[dict[str, object]] = []
    total_requested = 0.0
    total_inserted = 0.0
    total_disregarded = 0.0
    planner_calls = 0
    min_insertable_extra_um = _minimum_four_bend_extra_length_um(
        grid_size_um=float(grid_size_um_cfg),
        bend_radius_cells=int(bend_radius_cells),
    )

    for req in requirements:
        requested = float(req.missing_length_um)
        edge_key = req.edge_key
        record = by_edge.get(edge_key)
        entry = {
            "edge": edge_key_to_dict(edge_key),
            "requested_extra_length_um": requested,
            "status": "no_candidate",
            "reason": "no_matching_routed_record",
            "planning_mode": "fill_box_multi_bump",
            "inserted_extra_length_um": 0.0,
            "unmatched_length_um": requested,
            "effective_bend_radius_um": None,
            "primitive_bend_radius_um": None,
            "selected_box": None,
            "selected_grid_rect": None,
            "bumps": 0,
            "side": None,
            "using_legacy_meander_path": False,
            "minimum_insertable_extra_length_um": min_insertable_extra_um,
        }
        if requested < min_insertable_extra_um:
            total_disregarded += requested
            entry["status"] = "below_minimum_bump"
            entry["reason"] = (
                "requested extra length is below the four-90-degree-bend "
                f"minimum ({min_insertable_extra_um:.6g} um)"
            )
            entry["unmatched_length_um"] = 0.0
            results.append(entry)
            continue
        total_requested += requested
        if record is None:
            results.append(entry)
            continue
        current_route_open_cells = {
            cell
            for cell in route_cells_by_edge.get(edge_key, set())
            if route_cell_refcounts.get(cell, 0) == 1
            and cell not in base_static_cells
            and cell not in reserved_meander_cells
        }

        # Sweep box depth for legality, but keep requested length fixed.
        min_straight_um = max(0.0, float(config.min_candidate_straight_length_um))
        min_seg_um = max(0.5, float(config.min_candidate_straight_length_um))
        max_height_um = max(0.0, float(config.max_meander_height_um))
        box_depths_um = [
            depth
            for depth in MEANDER_DEPTH_CANDIDATES_UM
            if depth <= max_height_um + 1.0e-9
        ]
        if not box_depths_um:
            box_depths_um = [max_height_um]
        bend_radius_um = float(grid_size_um_cfg) * float(bend_radius_cells)
        endpoint_inset_um = (
            max(2.0 * bend_radius_um, min_seg_um)
            if config.auto_meander_endpoint_inset_um is None
            else max(0.0, float(config.auto_meander_endpoint_inset_um))
        )
        best_rr: dict[str, object] | None = None
        last_exc: Exception | None = None
        try:
            planner_calls += 1
            best_rr = cast(
                dict[str, object],
                router.plan_auto_analytic_meander_for_route_depth_sweep(
                    record.route_obj,
                    requested_extra_length_um=float(requested),
                    box_depths_um=box_depths_um,
                    min_bend_radius_um=None,
                    min_straight_um=min_straight_um,
                    max_bumps=32,
                    max_meander_height_um=max_height_um,
                    min_segment_length_um=min_seg_um,
                    endpoint_inset_um=endpoint_inset_um,
                    clearance_radius_cells=0,
                    side_policy="both",
                    # Only open this route's own non-static cells while
                    # planning its replacement. Port-access and fixed
                    # component/heater cells remain blocked.
                    opened_cells=sorted(current_route_open_cells),
                    planning_mode="fill_box_multi_bump",
                ),
            )
        except Exception as exc:
            last_exc = exc

        if best_rr is None:
            entry["status"] = "no_candidate"
            entry["reason"] = (
                str(last_exc)
                if last_exc is not None
                else f"no exact meander candidate found (|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)"
            )
            results.append(entry)
            continue
        rr = best_rr
        inserted = _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0)
        if abs(inserted - requested) > EXACT_MEANDER_EPS_UM:
            entry["status"] = "no_candidate"
            entry["reason"] = f"candidate residual {abs(inserted - requested):.6g} um exceeds hard limit {EXACT_MEANDER_EPS_UM} um"
            results.append(entry)
            continue
        unmatched = max(0.0, requested - inserted)
        entry["status"] = "planned"
        entry["reason"] = ""
        entry["inserted_extra_length_um"] = inserted
        entry["unmatched_length_um"] = unmatched
        entry["effective_bend_radius_um"] = rr.get("effective_bend_radius_um")
        entry["primitive_bend_radius_um"] = rr.get("primitive_bend_radius_um")
        entry["selected_box"] = rr.get("selected_box")
        entry["selected_grid_rect"] = rr.get("selected_grid_rect")
        entry["bumps"] = rr.get("bumps")
        entry["side"] = rr.get("side")
        entry["planning_mode"] = rr.get("planning_mode", "fill_box_multi_bump")
        entry["candidate_runs"] = rr.get("candidate_runs")
        entry["candidate_intervals"] = rr.get("candidate_intervals")
        entry["rejected_box_blocked"] = rr.get("rejected_box_blocked")
        entry["rejected_planning_failed"] = rr.get("rejected_planning_failed")
        entry["rejected_exact_length_mismatch"] = rr.get("rejected_exact_length_mismatch")
        entry["rejected_too_short"] = rr.get("rejected_too_short")
        entry["selected_interval_length_um"] = rr.get("selected_interval_length_um")
        entry["endpoint_inset_um"] = endpoint_inset_um
        entry["requested_probe_length_um"] = requested
        if unmatched > 1.0e-9:
            entry["status"] = "planned_partial"
        total_inserted += inserted
        selected_grid_rect = rr.get("selected_grid_rect")
        new_reserved_cells = _grid_rect_cells(selected_grid_rect)
        if new_reserved_cells:
            reserved_meander_cells.update(new_reserved_cells)
            router.add_static_cells(list(new_reserved_cells))
        updated[edge_key] = RoutedNetRecord(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
            route_obj=record.route_obj,
            total_length_um=record.total_length_um,
            meander_auto_plan={
                "requested_extra_length_um": requested,
                "min_bend_radius_um": None,
                "min_straight_um": min_straight_um,
                "max_bumps": 32,
                "max_meander_height_um": max_height_um,
                "box_depth_um": _as_float(rr.get("box_depth_um", 20.0), 20.0),
                "min_segment_length_um": min_seg_um,
                "endpoint_inset_um": endpoint_inset_um,
                "clearance_radius_cells": 0,
                "side_policy": "both",
                "selected_side": rr.get("side"),
                "selected_box": rr.get("selected_box"),
                "selected_grid_rect": rr.get("selected_grid_rect"),
                "selected_run_start_index": rr.get("selected_run_start_index"),
                "selected_run_end_index": rr.get("selected_run_end_index"),
                "selected_meander_centerline": rr.get("centerline"),
                "planning_mode": "fill_box_multi_bump",
            },
            opened_cells=record.opened_cells,
        )
        results.append(entry)

    total_unmatched = max(0.0, total_requested - total_inserted)
    return (
        [updated.get(_record_edge_key(r), r) for r in routed_net_records],
        {
            "results": results,
            "total_requested_extra_length_um": float(total_requested),
            "total_inserted_extra_length_um": float(total_inserted),
            "total_disregarded_extra_length_um": float(total_disregarded),
            "unmatched_length_um": float(total_unmatched),
            "planner_calls": int(planner_calls),
            "minimum_insertable_extra_length_um": float(min_insertable_extra_um),
            "using_legacy_meander_path": False,
        },
    )


def insert_meanders_for_requirements(
    routed_net_records: list[RoutedNetRecord],
    requirements: list[MissingLengthRequirement],
    *,
    config: MeanderInsertionConfig,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
) -> tuple[list[RoutedNetRecord], MeanderInsertionReport]:
    """Compatibility API used by tests for M2 skeleton behavior."""
    if not config.enabled:
        return (
            routed_net_records,
            MeanderInsertionReport(
                results=[],
                total_requested_extra_length_um=0.0,
                total_inserted_extra_length_um=0.0,
                unmatched_length_um=0.0,
            ),
        )

    updated, raw_report = analyze_meander_insertion_for_requirements(
        routed_net_records,
        requirements,
        config=config,
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_cells=bend_radius_cells,
    )
    results: list[MeanderInsertionResult] = []
    raw_results = cast(list[dict[str, object]], raw_report.get("results", []))
    for item in raw_results:
        edge_info = item.get("edge", {})
        if not isinstance(edge_info, dict):
            edge_info = {}
        source = edge_info.get("source", {})
        if not isinstance(source, dict):
            source = {}
        target = edge_info.get("target", {})
        if not isinstance(target, dict):
            target = {}
        edge = RoutedEdgeKey(
            net_name=str(edge_info.get("net_name", "")),
            source=PortRef(instance=str(source.get("instance", "")), port=str(source.get("port", ""))),
            target=PortRef(instance=str(target.get("instance", "")), port=str(target.get("port", ""))),
        )
        status = str(item.get("status", "unknown"))
        reason = str(item.get("reason", ""))
        results.append(
            MeanderInsertionResult(
                edge=edge,
                requested_extra_length_um=_as_float(item.get("requested_extra_length_um", 0.0), 0.0),
                inserted_extra_length_um=_as_float(item.get("inserted_extra_length_um", 0.0), 0.0),
                status=status,
                reason=reason,
            )
        )
    report = MeanderInsertionReport(
        results=results,
        total_requested_extra_length_um=float(cast(float, raw_report.get("total_requested_extra_length_um", 0.0))),
        total_inserted_extra_length_um=float(cast(float, raw_report.get("total_inserted_extra_length_um", 0.0))),
        unmatched_length_um=float(cast(float, raw_report.get("unmatched_length_um", 0.0))),
    )
    return updated, report


def route_match_and_realize(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    enable_path_length_matching: bool = False,
    node_types: dict[str, str] | None = None,
    internal_delays_um: dict[str, float] | None = None,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    debug_route_indices: set[int] | None = None,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    max_iterations: int = 500_000,
    debug_timing: bool = False,
    include_heater_obstacles: bool = False,
) -> RouteRustPipelineResult:
    """Run Phase A->(optional M1)->B entirely in route_rust."""
    route_obstacle_config = obstacle_config
    if debug_dir is None:
        route_obstacle_config = _with_bbox_cell_materialization(
            obstacle_config,
            materialize_bbox_cells=False,
            populate_obstacle_map=False,
        )

    t_route_nets_start = 0.0
    if debug_timing:
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
        max_iterations=max_iterations,
        debug_timing=debug_timing,
        include_heater_obstacles=include_heater_obstacles,
        defer_realization=True,
    )
    if debug_timing:
        t_route_nets_end = time.perf_counter()
        print(f"      - route_nets_rust phase: {t_route_nets_end - t_route_nets_start:.4f} s")

    analysis_info = None
    requirements_info = None
    meander_report_info = None
    records_for_realization = debug_artifacts.routed_net_records
    if enable_path_length_matching:
        analysis, requirements = analyze_path_length_matching(
            schematic,
            routed_net_records=debug_artifacts.routed_net_records,
            node_types=node_types,
            internal_delays_um=internal_delays_um,
        )
        analysis_info = analysis_to_info_dict(analysis)
        requirements_info = [requirement_to_dict(req) for req in requirements]
        if debug_artifacts.realization_grid_spec is None:
            raise RuntimeError("Missing realization grid spec from routing phase.")
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
        meander_static_blocked_cells = tuple(sorted(set(meander_obstacle_map.blocked_cells)))

        records_for_realization, meander_report_info = analyze_meander_insertion_for_requirements(
            debug_artifacts.routed_net_records,
            requirements,
            config=MeanderInsertionConfig(enabled=True),
            realization_grid_spec=debug_artifacts.realization_grid_spec,
            allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
            bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
            static_blocked_cells=meander_static_blocked_cells,
        )

    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")
    t_realization_start = 0.0
    if debug_timing:
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
    if debug_timing:
        t_realization_end = time.perf_counter()
        print(f"      - route realization phase: {t_realization_end - t_realization_start:.4f} s")

    return RouteRustPipelineResult(
        routed_layout=routed_layout,
        debug_artifacts=debug_artifacts,
        path_length_analysis_info=analysis_info,
        meander_requirements_info=requirements_info,
        meander_insertion_report_info=meander_report_info,
    )


def realize_routed_net_records(
    routed_layout: Component,
    routed_net_records: list[RoutedNetRecord],
    *,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool = True,
    bend_radius_cells: int = 4,
) -> None:
    """Phase B: realize routed records into polygons on the target layout."""
    if route_width_um <= 0:
        raise ValueError("route_width_um must be > 0")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

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
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    for record in routed_net_records:
        if record.meander_auto_plan is not None:
            plan = record.meander_auto_plan
            selected_side = plan.get("selected_side")
            selected_box = plan.get("selected_box")
            selected_run_start_index = plan.get("selected_run_start_index")
            selected_run_end_index = plan.get("selected_run_end_index")
            selected_meander_centerline = plan.get("selected_meander_centerline")
            if (
                isinstance(selected_run_start_index, (int, float))
                and isinstance(selected_run_end_index, (int, float))
                and isinstance(selected_meander_centerline, list)
                and len(selected_meander_centerline) >= 2
            ):
                meander_centerline = [
                    (_as_float(p[0], 0.0), _as_float(p[1], 0.0))
                    for p in selected_meander_centerline
                    if isinstance(p, (tuple, list)) and len(p) == 2
                ]
                polygon = router.realize_route_polygon_from_planned_auto_meander(
                    record.route_obj,
                    float(route_width_um),
                    selected_run_start_index=_as_int(selected_run_start_index, 0),
                    selected_run_end_index=_as_int(selected_run_end_index, 0),
                    meander_centerline=meander_centerline,
                )
            elif (
                isinstance(selected_side, str)
                and selected_side in {"left", "right"}
                and isinstance(selected_box, (tuple, list))
                and len(selected_box) == 4
            ):
                box_tuple = (
                    _as_float(selected_box[0], 0.0),
                    _as_float(selected_box[1], 0.0),
                    _as_float(selected_box[2], 0.0),
                    _as_float(selected_box[3], 0.0),
                )
                polygon = router.realize_route_polygon_with_analytic_meander(
                    record.route_obj,
                    float(route_width_um),
                    requested_extra_length_um=_as_float(plan["requested_extra_length_um"], 0.0),
                    min_bend_radius_um=plan["min_bend_radius_um"],
                    min_straight_um=_as_float(plan["min_straight_um"], 0.0),
                    max_bumps=_as_int(plan["max_bumps"], 8),
                    side=selected_side,
                    available_box=box_tuple,
                    planning_mode=str(plan["planning_mode"]),
                )
            else:
                # Backward-compatibility fallback for records created before
                # selected_side/selected_box were persisted.
                polygon = router.realize_route_polygon_with_auto_checked_analytic_meander(
                    record.route_obj,
                    float(route_width_um),
                    requested_extra_length_um=_as_float(plan["requested_extra_length_um"], 0.0),
                    min_bend_radius_um=plan["min_bend_radius_um"],
                    min_straight_um=_as_float(plan["min_straight_um"], 0.0),
                    max_bumps=_as_int(plan["max_bumps"], 8),
                    max_meander_height_um=_as_float(plan.get("max_meander_height_um", 20.0), 20.0),
                    box_depth_um=_as_float(plan["box_depth_um"], 20.0),
                    min_segment_length_um=_as_float(plan["min_segment_length_um"], 1.0),
                    clearance_radius_cells=_as_int(plan["clearance_radius_cells"], 0),
                    side_policy=str(plan["side_policy"]),
                    opened_cells=[],
                    planning_mode=str(plan["planning_mode"]),
                )
            routed_layout.add_polygon(polygon, layer=route_layer)
            continue
        polygon = router.realize_route_polygon(record.route_obj, float(route_width_um))
        routed_layout.add_polygon(polygon, layer=route_layer)


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _cells_bbox(cells: set[tuple[int, int]]) -> tuple[int, int, int, int] | None:
    if not cells:
        return None
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
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
    max_iterations: int = 500_000,
    debug_timing: bool = False,
    include_heater_obstacles: bool = False,
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
    routed_edge_lengths_um: dict[RoutedEdgeKey, float] = {}
    routed_net_records: list[RoutedNetRecord] = []

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
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    block_radius_cells = max(
        0, math.ceil((float(route_width_um) / 2.0) / float(grid.grid_size_um))
    )
    port_entry_length_cells = max(2, bend_radius_cells + 2)
    port_entry_half_width_cells = max(1, bend_radius_cells + block_radius_cells + 1)
    port_lane_length_cells = max(3, 2 * bend_radius_cells + 2)
    port_lane_half_width_cells = max(1, block_radius_cells + 1)
    net_id = 0

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
            (1, 0),    # 0 east
            (1, 1),    # 1 northeast
            (0, 1),    # 2 north
            (-1, 1),   # 3 northwest
            (-1, 0),   # 4 west
            (-1, -1),  # 5 southwest
            (0, -1),   # 6 south
            (1, -1),   # 7 southeast
        ]
        return steps[angle % 8]


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
                    / float(grid.grid_size_um)
                ),
            )
            half_width_cells = max(
                0,
                math.ceil(
                    (
                        max(0.0, _as_float(access_width_um, 0.0))
                        / 2.0
                    )
                    / float(grid.grid_size_um)
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

    def _inflate_cells_square(
        cells: set[tuple[int, int]],
        *,
        radius_cells: int,
    ) -> set[tuple[int, int]]:
        if radius_cells <= 0:
            return {(x, y) for (x, y) in cells if _in_bounds(x, y)}
        inflated: set[tuple[int, int]] = set()
        for x, y in cells:
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    nx = x + dx
                    ny = y + dy
                    if _in_bounds(nx, ny):
                        inflated.add((nx, ny))
        return inflated

    route_jobs: list[tuple[str, str, str, str, str, Port, Port]] = []
    port_access_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_rule_by_spec: dict[str, str | None] = {}
    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")
            source_port = get_port_from_instance(routed_layout, inst1, port1)
            target_port = get_port_from_instance(routed_layout, inst2, port2)
            route_jobs.append((net_name, inst1, port1, inst2, port2, source_port, target_port))
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
    committed_dynamic_cells: set[tuple[int, int]] = set()

    t_astar_start = 0.0
    if debug_timing:
        t_astar_start = time.perf_counter()
    total_expanded_states = 0
    simple_route_count = 0

    for route_index, (
        net_name,
        inst1,
        port1,
        inst2,
        port2,
        source_port,
        target_port,
    ) in enumerate(route_jobs, start=1):
        port1_spec = f"{inst1},{port1}"
        port2_spec = f"{inst2},{port2}"
        source_state = port_to_grid_state(
            source_port,
            origin_x_um,
            origin_y_um,
            float(grid.grid_size_um),
            as_target=False,
        )
        target_state = port_to_grid_state(
            target_port,
            origin_x_um,
            origin_y_um,
            float(grid.grid_size_um),
            as_target=True,
        )

        should_print_route = (
            debug_route_indices is None or route_index in debug_route_indices
        )
        should_export_route_debug = (
            debug_path is not None
            and (debug_route_indices is None or route_index in debug_route_indices)
        )
        route_progress_text = (
            f"  Routing [{route_index}/{len(route_jobs)}] "
            f"{net_name}: {port1_spec} -> {port2_spec}..."
        )
        if should_print_route:
            print(route_progress_text, end=" ")

        net_id += 1
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        opened_candidate_cells = set(port_access_candidate_cells_by_spec.get(port1_spec, set()))
        opened_candidate_cells.update(port_access_candidate_cells_by_spec.get(port2_spec, set()))
        opened_candidate_cells.update({source_anchor_cell, target_anchor_cell})

        opened_cells_set = set(port_access_cells_by_spec.get(port1_spec, set()))
        opened_cells_set.update(port_access_cells_by_spec.get(port2_spec, set()))
        opened_cells_set.update({source_anchor_cell, target_anchor_cell})
        opened_cells = sorted(opened_cells_set)

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

        route_dir = debug_path / "routes" if debug_path is not None else None
        diag_txt: Path | None = None
        if should_export_route_debug and route_dir is not None:
            _ensure_dir(route_dir)
            diag_txt = route_dir / f"{debug_prefix}_{net_name}_diagnostics.txt"

        def _write_diagnostics(
            *,
            status: str,
            error_text: str | None = None,
            route_cells: set[tuple[int, int]] | None = None,
        ) -> None:
            if diag_txt is None:
                return
            route_cells = route_cells or set()
            route_static_overlap = (
                route_cells & static_blocked_cells_before_port_reservations
            )
            route_overlap_with_candidate_opened_static = (
                route_cells & opened_candidate_static_overlap
            )
            route_overlap_with_effective_opened_static = (
                route_cells & opened_static_overlap
            )
            route_dynamic_overlap = route_cells & committed_dynamic_cells
            route_overlap_with_candidate_opened_dynamic = (
                route_cells & opened_candidate_dynamic_overlap
            )
            route_overlap_with_effective_opened_dynamic = (
                route_cells & opened_dynamic_overlap
            )
            lines = [
                f"net_name={net_name}",
                f"status={status}",
                f"source_spec={port1_spec}",
                f"target_spec={port2_spec}",
                f"source_component={_schematic_instance_component_name(schematic, inst1)}",
                f"target_component={_schematic_instance_component_name(schematic, inst2)}",
                f"source_access_rule={port_access_rule_by_spec.get(port1_spec)}",
                f"target_access_rule={port_access_rule_by_spec.get(port2_spec)}",
                f"source_state=({source_anchor_cell[0]}, {source_anchor_cell[1]}, {int(source_state.angle)})",
                f"target_state=({target_anchor_cell[0]}, {target_anchor_cell[1]}, {int(target_state.angle)})",
                f"opened_candidate_cells_count={len(opened_candidate_cells)}",
                f"opened_candidate_static_overlap_count={len(opened_candidate_static_overlap)}",
                f"opened_candidate_static_overlap_bbox={_cells_bbox(opened_candidate_static_overlap)}",
                f"opened_candidate_dynamic_overlap_count={len(opened_candidate_dynamic_overlap)}",
                f"opened_candidate_dynamic_overlap_bbox={_cells_bbox(opened_candidate_dynamic_overlap)}",
                (
                    "opened_candidate_static_overlap_cells="
                    f"{_format_cells_preview(opened_candidate_static_overlap)}"
                ),
                f"opened_cells_count={len(opened_cells_set)}",
                f"opened_cells={sorted(opened_cells_set)}",
                f"opened_static_overlap_count={len(opened_static_overlap)}",
                f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
                f"opened_static_overlap_cells={_format_cells_preview(opened_static_overlap)}",
                f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
                f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
                f"opened_dynamic_overlap_cells={_format_cells_preview(opened_dynamic_overlap)}",
                f"route_cells_count={len(route_cells)}",
                f"route_static_blocked_overlap_count={len(route_static_overlap)}",
                f"route_static_blocked_overlap_bbox={_cells_bbox(route_static_overlap)}",
                f"route_static_blocked_overlap_cells={_format_cells_preview(route_static_overlap)}",
                f"route_dynamic_overlap_count={len(route_dynamic_overlap)}",
                f"route_dynamic_overlap_bbox={_cells_bbox(route_dynamic_overlap)}",
                f"route_dynamic_overlap_cells={_format_cells_preview(route_dynamic_overlap)}",
                (
                    "route_overlap_candidate_opened_static_count="
                    f"{len(route_overlap_with_candidate_opened_static)}"
                ),
                (
                    "route_overlap_candidate_opened_static_bbox="
                    f"{_cells_bbox(route_overlap_with_candidate_opened_static)}"
                ),
                (
                    "route_overlap_effective_opened_static_count="
                    f"{len(route_overlap_with_effective_opened_static)}"
                ),
                (
                    "route_overlap_effective_opened_static_bbox="
                    f"{_cells_bbox(route_overlap_with_effective_opened_static)}"
                ),
                (
                    "route_overlap_candidate_opened_dynamic_count="
                    f"{len(route_overlap_with_candidate_opened_dynamic)}"
                ),
                (
                    "route_overlap_candidate_opened_dynamic_bbox="
                    f"{_cells_bbox(route_overlap_with_candidate_opened_dynamic)}"
                ),
                (
                    "route_overlap_effective_opened_dynamic_count="
                    f"{len(route_overlap_with_effective_opened_dynamic)}"
                ),
                (
                    "route_overlap_effective_opened_dynamic_bbox="
                    f"{_cells_bbox(route_overlap_with_effective_opened_dynamic)}"
                ),
            ]
            if error_text is not None:
                lines.append(f"error={error_text}")
            diag_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")

        try:
            route_obj = router.route_single_net_and_commit(
                net_id,
                source_state,
                target_state,
                block_radius_cells,
                opened_cells,
            )
        except RuntimeError as exc:
            if not should_print_route:
                print(f"{route_progress_text} failed")
            _write_diagnostics(status="failed", error_text=str(exc))
            if debug_path is not None:
                assert route_dir is not None
                _ensure_dir(route_dir)
                fail_txt = route_dir / f"{debug_prefix}_{net_name}_FAILED.txt"
                fail_lines = [
                    f"net_name={net_name}",
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
                    f"opened_cells={opened_cells}",
                    f"opened_static_overlap_count={len(opened_static_overlap)}",
                    f"opened_static_overlap_bbox={_cells_bbox(opened_static_overlap)}",
                    f"opened_dynamic_overlap_count={len(opened_dynamic_overlap)}",
                    f"opened_dynamic_overlap_bbox={_cells_bbox(opened_dynamic_overlap)}",
                    f"error={exc}",
                ]
                fail_txt.write_text("\n".join(fail_lines) + "\n", encoding="utf-8")
            raise RuntimeError(
                f"No route found for {net_name}: {port1_spec} -> {port2_spec}. "
                f"source=({source_state.x}, {source_state.y}, {source_state.angle}), "
                f"target=({target_state.x}, {target_state.y}, {target_state.angle}), "
                f"allow_45_degree_turns={allow_45_degree_turns}"
            ) from exc

        edge_key = RoutedEdgeKey(
            net_name=net_name,
            source=PortRef(instance=inst1, port=port1),
            target=PortRef(instance=inst2, port=port2),
        )
        routed_net_records.append(
            RoutedNetRecord(
                net_name=net_name,
                source=edge_key.source,
                target=edge_key.target,
                route_obj=route_obj,
                total_length_um=float(route_obj.total_length_um),
                opened_cells=tuple(opened_cells),
            )
        )
        routed_edge_lengths_um[edge_key] = float(route_obj.total_length_um)
        expanded_states = int(getattr(route_obj, "expanded_states", 0))
        total_expanded_states += expanded_states
        if expanded_states == 0:
            simple_route_count += 1

        if diagnostics_enabled:
            route_cells = {
                (int(cell[0]), int(cell[1]))
                for cell in (getattr(route_obj, "cells", None) or [])
            }
            _write_diagnostics(status="ok", route_cells=route_cells)
            committed_dynamic_cells.update(
                _inflate_cells_square(route_cells, radius_cells=block_radius_cells)
            )

        if should_export_route_debug:
            assert route_dir is not None
            route_svg = route_dir / f"{debug_prefix}_{net_name}.svg"
            route_svg.write_text(router.export_debug_svg(route_obj), encoding="utf-8")
            route_svgs.append(route_svg)

        if should_print_route:
            print("ok")

    if debug_timing:
        t_astar_end = time.perf_counter()
        print(f"      - Astar time: {t_astar_end - t_astar_start:.4f} s")
        print(
            "      - Route search stats: "
            f"simple={simple_route_count}/{len(route_jobs)}, "
            f"expanded_states={total_expanded_states}"
        )

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

    build_stats = getattr(obstacle_map, "build_stats", None)
    static_obstacle_count = len(obstacle_map.blocked_cells)
    if isinstance(build_stats, dict):
        raw_count = build_stats.get("blocked_cell_count")
        if isinstance(raw_count, int):
            static_obstacle_count = raw_count

    return routed_layout, RustRouteDebugArtifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
        routed_edge_lengths_um=routed_edge_lengths_um,
        routed_net_records=routed_net_records,
        # Keep meander planning base obstacles limited to layout-static geometry.
        # Port-access reservation lanes are routing-time guards and are added to
        # `static_cells` above for net-to-net A* ordering, but they should not
        # globally block post-route meander box checks.
        static_blocked_cells=tuple(sorted(set(obstacle_map.blocked_cells))),
        static_obstacle_count=int(static_obstacle_count),
        realization_grid_spec=realization_grid_spec,
        realization_allow_45_degree_turns=allow_45_degree_turns,
        realization_bend_radius_cells=bend_radius_cells,
    )
