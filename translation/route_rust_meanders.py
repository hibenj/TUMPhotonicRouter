"""Meander planning helpers for path-length matching."""

from __future__ import annotations

import importlib
import math
from collections import Counter
from typing import cast

from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    PortRef,
    RoutedEdgeKey,
)

from translation.route_rust_analysis import edge_key_to_dict
from translation.route_rust_types import (
    MeanderInsertionConfig,
    MeanderInsertionReport,
    MeanderInsertionResult,
    RoutedNetRecord,
    _as_float,
    _as_int,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
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


def _record_edge_key(record: RoutedNetRecord) -> RoutedEdgeKey:
    return RoutedEdgeKey(
        net_name=record.net_name,
        source=record.source,
        target=record.target,
    )


def _minimum_four_bend_extra_length_um(
    *,
    grid_size_um: float,
    bend_radius_cells: int,
) -> float:
    """Minimum practical matching request: one bump needs four 90-degree bends."""
    bend_radius_um = max(0.0, float(grid_size_um) * float(bend_radius_cells))
    return 2.0 * math.pi * bend_radius_um


def _route_geometry_max_meander_bumps(
    *,
    route_obj: object,
    grid_size_um: float,
    bend_radius_um: float,
) -> int:
    """Derive the odd internal bump cap from visible lobe width.

    One visible lobe consumes four 90-degree bend radii along the selected
    straight run. Rust's comb planner reports odd internal bump counts where
    visible_lobes = (bumps + 1) / 2.
    """
    radius = float(bend_radius_um)
    grid_size = float(grid_size_um)
    if (
        not math.isfinite(radius)
        or radius <= 0.0
        or not math.isfinite(grid_size)
        or grid_size <= 0.0
    ):
        return 1

    waypoints = getattr(route_obj, "compressed_waypoints", None) or []
    longest_straight_um = 0.0
    for p0, p1 in zip(waypoints, waypoints[1:]):
        if (
            not isinstance(p0, (tuple, list))
            or not isinstance(p1, (tuple, list))
            or len(p0) != 2
            or len(p1) != 2
        ):
            continue
        x0 = _as_int(p0[0], 0)
        y0 = _as_int(p0[1], 0)
        x1 = _as_int(p1[0], 0)
        y1 = _as_int(p1[1], 0)
        if x0 == x1:
            longest_straight_um = max(longest_straight_um, abs(y1 - y0) * grid_size)
        elif y0 == y1:
            longest_straight_um = max(longest_straight_um, abs(x1 - x0) * grid_size)

    visible_lobes = int(math.floor(longest_straight_um / (4.0 * radius)))
    return max(1, 2 * visible_lobes - 1)


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
        if not isinstance(grid_rect, (tuple, list)) or len(grid_rect) != 4:
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
        max_bumps = _route_geometry_max_meander_bumps(
            route_obj=record.route_obj,
            grid_size_um=float(grid_size_um_cfg),
            bend_radius_um=bend_radius_um,
        )
        entry["max_bumps"] = max_bumps
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
                    max_bumps=max_bumps,
                    max_meander_height_um=max_height_um,
                    min_segment_length_um=min_seg_um,
                    endpoint_inset_um=endpoint_inset_um,
                    clearance_radius_cells=0,
                    side_policy="both",
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
            entry["reason"] = (
                f"candidate residual {abs(inserted - requested):.6g} um "
                f"exceeds hard limit {EXACT_MEANDER_EPS_UM} um"
            )
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
                "max_bumps": max_bumps,
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
            source=PortRef(
                instance=str(source.get("instance", "")),
                port=str(source.get("port", "")),
            ),
            target=PortRef(
                instance=str(target.get("instance", "")),
                port=str(target.get("port", "")),
            ),
        )
        status = str(item.get("status", "unknown"))
        reason = str(item.get("reason", ""))
        results.append(
            MeanderInsertionResult(
                edge=edge,
                requested_extra_length_um=_as_float(
                    item.get("requested_extra_length_um", 0.0),
                    0.0,
                ),
                inserted_extra_length_um=_as_float(
                    item.get("inserted_extra_length_um", 0.0),
                    0.0,
                ),
                status=status,
                reason=reason,
            )
        )
    report = MeanderInsertionReport(
        results=results,
        total_requested_extra_length_um=float(
            cast(float, raw_report.get("total_requested_extra_length_um", 0.0))
        ),
        total_inserted_extra_length_um=float(
            cast(float, raw_report.get("total_inserted_extra_length_um", 0.0))
        ),
        unmatched_length_um=float(
            cast(float, raw_report.get("unmatched_length_um", 0.0))
        ),
    )
    return updated, report
