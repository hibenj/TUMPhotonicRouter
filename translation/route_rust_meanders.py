"""Meander planning helpers for path-length matching."""

from __future__ import annotations

import importlib
import math
import time
from collections import Counter
from collections.abc import Iterable, Mapping
from typing import cast

from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
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
GridCell = tuple[int, int]
PlannedEdgeInsertion = tuple[
    RoutedEdgeKey,
    RoutedNetRecord,
    dict[str, object],
    bool,
    int,
    set[GridCell],
]
EdgePlanAttempt = tuple[
    dict[str, object],
    dict[str, object] | None,
    RoutedNetRecord | None,
    bool,
    int,
    set[GridCell],
    float,
    Exception | None,
]


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
    record: RoutedNetRecord,
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

    longest_straight_um = _longest_axis_aligned_centerline_run_um(
        record.corrected_centerline_um
    )
    if longest_straight_um <= 0.0:
        waypoints = getattr(record.route_obj, "compressed_waypoints", None) or []
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


def _longest_axis_aligned_centerline_run_um(
    centerline: tuple[tuple[float, float], ...],
) -> float:
    if len(centerline) < 2:
        return 0.0
    eps = 1.0e-9
    longest = 0.0
    current = 0.0
    current_axis: str | None = None
    current_line_coord = 0.0
    current_dir = 0
    for p0, p1 in zip(centerline, centerline[1:]):
        dx = float(p1[0]) - float(p0[0])
        dy = float(p1[1]) - float(p0[1])
        if abs(dy) <= eps and abs(dx) > eps:
            axis = "x"
            line_coord = float(p0[1])
            direction = 1 if dx > 0 else -1
            length = abs(dx)
        elif abs(dx) <= eps and abs(dy) > eps:
            axis = "y"
            line_coord = float(p0[0])
            direction = 1 if dy > 0 else -1
            length = abs(dy)
        else:
            longest = max(longest, current)
            current = 0.0
            current_axis = None
            continue
        if (
            axis == current_axis
            and direction == current_dir
            and abs(line_coord - current_line_coord) <= eps
        ):
            current += length
        else:
            longest = max(longest, current)
            current = length
            current_axis = axis
            current_line_coord = line_coord
            current_dir = direction
    return max(longest, current)


def analyze_meander_insertion_for_requirements(
    routed_net_records: list[RoutedNetRecord],
    requirements: list[MissingLengthRequirement],
    *,
    config: MeanderInsertionConfig,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    static_blocked_cells: Iterable[tuple[int, int]] | None = None,
    requirement_edge_alternatives: Mapping[
        RoutedEdgeKey,
        Iterable[RoutedEdgeKey],
    ]
    | None = None,
    requirement_delay_candidates: Mapping[
        RoutedEdgeKey,
        Iterable[DelayInsertionCandidate],
    ]
    | None = None,
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
    if isinstance(static_blocked_cells, set):
        base_static_cells = static_blocked_cells
    else:
        base_static_cells = {
            (int(x), int(y))
            for x, y in (static_blocked_cells or ())
        }

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

    reserved_meander_cells: set[GridCell] = set()

    by_edge = {_record_edge_key(r): r for r in routed_net_records}
    updated = dict(by_edge)
    route_cells_by_edge = {
        edge: _record_route_cells(record)
        for edge, record in by_edge.items()
    }
    route_cell_refcounts: Counter[tuple[int, int]] = Counter()
    for cells in route_cells_by_edge.values():
        route_cell_refcounts.update(cells)
    router.set_static_cells(list(base_static_cells))
    if route_cell_refcounts:
        router.add_static_cells(list(route_cell_refcounts.keys()))
    results: list[dict[str, object]] = []
    total_requested = 0.0
    total_inserted = 0.0
    total_disregarded = 0.0
    planner_calls = 0
    planner_elapsed_s = 0.0
    min_insertable_extra_um = _minimum_four_bend_extra_length_um(
        grid_size_um=float(grid_size_um_cfg),
        bend_radius_cells=int(bend_radius_cells),
    )
    total_requested = sum(
        float(req.missing_length_um)
        for req in requirements
        if float(req.missing_length_um) >= min_insertable_extra_um
    )

    def _candidates_for_requirement(
        req: MissingLengthRequirement,
    ) -> list[DelayInsertionCandidate]:
        if requirement_delay_candidates is not None:
            explicit_candidates = list(
                requirement_delay_candidates.get(req.edge_key, ())
            )
            if explicit_candidates:
                return explicit_candidates

        edge_keys = [req.edge_key]
        if requirement_edge_alternatives is not None:
            edge_keys.extend(requirement_edge_alternatives.get(req.edge_key, ()))

        candidates: list[DelayInsertionCandidate] = []
        seen_edges: set[RoutedEdgeKey] = set()
        for index, edge_key in enumerate(edge_keys):
            if edge_key in seen_edges:
                continue
            seen_edges.add(edge_key)
            candidates.append(
                DelayInsertionCandidate(
                    requirement_edge_key=req.edge_key,
                    edge_keys=(edge_key,),
                    extra_length_um=float(req.missing_length_um),
                    reason="direct_edge" if index == 0 else "legacy_single_edge_alternative",
                    affected_requirement_edge_keys=(req.edge_key,),
                )
            )
        return candidates

    def _planned_record(
        *,
        record: RoutedNetRecord,
        requested: float,
        rr: dict[str, object],
        min_straight_um: float,
        max_bumps: int,
        max_height_um: float,
        min_seg_um: float,
        endpoint_inset_um: float,
    ) -> RoutedNetRecord:
        return RoutedNetRecord(
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
            source_port_center_um=record.source_port_center_um,
            target_port_center_um=record.target_port_center_um,
            source_port_orientation_deg=record.source_port_orientation_deg,
            target_port_orientation_deg=record.target_port_orientation_deg,
            base_total_length_um=record.base_total_length_um,
            corrected_centerline_um=record.corrected_centerline_um,
        )

    def _plan_edge_candidate(
        *,
        candidate_edge_key: RoutedEdgeKey,
        requested: float,
        extra_reserved_cells: set[GridCell],
        box_depths_um: list[float],
        min_straight_um: float,
        min_seg_um: float,
        max_height_um: float,
        bend_radius_um: float,
        endpoint_inset_um: float,
    ) -> EdgePlanAttempt:
        record = by_edge.get(candidate_edge_key)
        attempt_info: dict[str, object] = {
            "edge": edge_key_to_dict(candidate_edge_key),
            "status": "no_candidate",
            "reason": "",
        }
        if record is None:
            attempt_info["reason"] = "no_matching_routed_record"
            return attempt_info, None, None, True, 1, set(), 0.0, None

        blocked_by_planned = reserved_meander_cells | extra_reserved_cells
        candidate_route_open_cells = {
            cell
            for cell in route_cells_by_edge.get(candidate_edge_key, set())
            if route_cell_refcounts.get(cell, 0) == 1
            and cell not in base_static_cells
            and cell not in blocked_by_planned
        }
        candidate_max_bumps = _route_geometry_max_meander_bumps(
            record=record,
            grid_size_um=float(grid_size_um_cfg),
            bend_radius_um=bend_radius_um,
        )
        candidate_best_rr: dict[str, object] | None = None
        candidate_last_exc: Exception | None = None
        candidate_used_reserved_overlay = True
        t_plan_start = time.perf_counter()
        try:
            planner_kwargs = dict(
                requested_extra_length_um=float(requested),
                box_depths_um=box_depths_um,
                min_bend_radius_um=None,
                min_straight_um=min_straight_um,
                max_bumps=candidate_max_bumps,
                max_meander_height_um=max_height_um,
                min_segment_length_um=min_seg_um,
                endpoint_inset_um=endpoint_inset_um,
                clearance_radius_cells=0,
                side_policy="both",
                opened_cells=sorted(candidate_route_open_cells),
                planning_mode="fill_box_multi_bump",
                extra_blocked_cells=sorted(blocked_by_planned),
            )
            try:
                if record.corrected_centerline_um:
                    candidate_best_rr = cast(
                        dict[str, object],
                        router.plan_auto_analytic_meander_for_centerline_depth_sweep(
                            list(record.corrected_centerline_um),
                            **planner_kwargs,
                        ),
                    )
                else:
                    candidate_best_rr = cast(
                        dict[str, object],
                        router.plan_auto_analytic_meander_for_route_depth_sweep(
                            record.route_obj,
                            **planner_kwargs,
                        ),
                    )
            except TypeError:
                candidate_used_reserved_overlay = False
                planner_kwargs.pop("extra_blocked_cells", None)
                if record.corrected_centerline_um:
                    candidate_best_rr = cast(
                        dict[str, object],
                        router.plan_auto_analytic_meander_for_centerline_depth_sweep(
                            list(record.corrected_centerline_um),
                            **planner_kwargs,
                        ),
                    )
                else:
                    candidate_best_rr = cast(
                        dict[str, object],
                        router.plan_auto_analytic_meander_for_route_depth_sweep(
                            record.route_obj,
                            **planner_kwargs,
                        ),
                    )
        except Exception as exc:
            candidate_last_exc = exc
        elapsed_s = time.perf_counter() - t_plan_start

        if candidate_best_rr is None:
            attempt_info["reason"] = (
                str(candidate_last_exc)
                if candidate_last_exc is not None
                else f"no exact meander candidate found (|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)"
            )
            return (
                attempt_info,
                None,
                record,
                candidate_used_reserved_overlay,
                candidate_max_bumps,
                candidate_route_open_cells,
                elapsed_s,
                candidate_last_exc,
            )

        candidate_inserted = _as_float(
            candidate_best_rr.get("inserted_extra_length_um", 0.0),
            0.0,
        )
        if abs(candidate_inserted - requested) > EXACT_MEANDER_EPS_UM:
            attempt_info["reason"] = (
                f"candidate residual {abs(candidate_inserted - requested):.6g} um "
                f"exceeds hard limit {EXACT_MEANDER_EPS_UM} um"
            )
            attempt_info["inserted_extra_length_um"] = candidate_inserted
            return (
                attempt_info,
                None,
                record,
                candidate_used_reserved_overlay,
                candidate_max_bumps,
                candidate_route_open_cells,
                elapsed_s,
                None,
            )

        attempt_info["status"] = "planned"
        attempt_info["reason"] = ""
        attempt_info["inserted_extra_length_um"] = candidate_inserted
        return (
            attempt_info,
            candidate_best_rr,
            record,
            candidate_used_reserved_overlay,
            candidate_max_bumps,
            candidate_route_open_cells,
            elapsed_s,
            None,
        )

    requirement_missing_by_edge: dict[RoutedEdgeKey, float] = {}
    for req in requirements:
        requirement_missing_by_edge[req.edge_key] = (
            requirement_missing_by_edge.get(req.edge_key, 0.0)
            + float(req.missing_length_um)
        )
    effective_inserted_by_requirement_edge: dict[RoutedEdgeKey, float] = {}
    total_physical_inserted = 0.0

    for req in requirements:
        original_requested = float(req.missing_length_um)
        edge_key = req.edge_key
        already_inserted = effective_inserted_by_requirement_edge.get(edge_key, 0.0)
        requested = max(0.0, original_requested - already_inserted)
        if requested <= EXACT_MEANDER_EPS_UM:
            continue
        candidate_insertions = _candidates_for_requirement(req)
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
            "planning_elapsed_s": 0.0,
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
        entry["box_depth_candidates_um"] = list(box_depths_um)
        bend_radius_um = float(grid_size_um_cfg) * float(bend_radius_cells)
        endpoint_inset_um = (
            max(2.0 * bend_radius_um, min_seg_um)
            if config.auto_meander_endpoint_inset_um is None
            else max(0.0, float(config.auto_meander_endpoint_inset_um))
        )
        last_exc: Exception | None = None
        selected_plans: list[PlannedEdgeInsertion] = []
        selected_candidate: DelayInsertionCandidate | None = None
        selected_candidate_requested = requested
        selected_affected_edges: tuple[RoutedEdgeKey, ...] = (edge_key,)
        candidate_attempts: list[dict[str, object]] = []
        attempted_edges: list[dict[str, object]] = []
        planning_elapsed_for_entry_s = 0.0
        max_bumps = 1
        current_route_open_cells: set[tuple[int, int]] = set()

        for candidate in candidate_insertions:
            affected_edges = (
                candidate.affected_requirement_edge_keys
                if candidate.affected_requirement_edge_keys
                else (edge_key,)
            )
            remaining_for_affected = [
                max(
                    0.0,
                    requirement_missing_by_edge.get(affected_edge, requested)
                    - effective_inserted_by_requirement_edge.get(affected_edge, 0.0),
                )
                for affected_edge in affected_edges
            ]
            if not remaining_for_affected:
                continue
            candidate_requested = min(
                float(candidate.extra_length_um),
                min(remaining_for_affected),
            )
            edge_attempts: list[dict[str, object]] = []
            candidate_info: dict[str, object] = {
                "candidate_reason": candidate.reason,
                "requested_extra_length_um": candidate_requested,
                "edges": [edge_key_to_dict(edge) for edge in candidate.edge_keys],
                "affected_requirement_edges": [
                    edge_key_to_dict(edge)
                    for edge in affected_edges
                ],
                "edge_count": len(candidate.edge_keys),
                "status": "no_candidate",
                "failure_reason": "",
                "edge_attempts": edge_attempts,
            }
            if candidate_requested <= EXACT_MEANDER_EPS_UM:
                candidate_info["status"] = "already_satisfied"
                candidate_attempts.append(candidate_info)
                continue
            candidate_plans: list[PlannedEdgeInsertion] = []
            candidate_reserved_cells: set[GridCell] = set()
            candidate_open_cells: set[GridCell] = set()
            candidate_max_bumps_values: list[int] = []
            candidate_failed = False
            for candidate_edge_key in candidate.edge_keys:
                planner_calls += 1
                (
                    attempt_info,
                    rr,
                    record,
                    used_reserved_overlay,
                    candidate_max_bumps,
                    candidate_route_open_cells,
                    elapsed_s,
                    candidate_last_exc,
                ) = _plan_edge_candidate(
                    candidate_edge_key=candidate_edge_key,
                    requested=candidate_requested,
                    extra_reserved_cells=candidate_reserved_cells,
                    box_depths_um=box_depths_um,
                    min_straight_um=min_straight_um,
                    min_seg_um=min_seg_um,
                    max_height_um=max_height_um,
                    bend_radius_um=bend_radius_um,
                    endpoint_inset_um=endpoint_inset_um,
                )
                planning_elapsed_for_entry_s += elapsed_s
                planner_elapsed_s += elapsed_s
                edge_attempts.append(attempt_info)
                attempted_edges.append(attempt_info)
                last_exc = candidate_last_exc
                if rr is None or record is None:
                    candidate_failed = True
                    candidate_info["failure_reason"] = attempt_info.get("reason", "")
                    break
                selected_grid_rect = rr.get("selected_grid_rect")
                new_reserved_cells = _grid_rect_cells(selected_grid_rect)
                candidate_reserved_cells.update(new_reserved_cells)
                candidate_open_cells.update(candidate_route_open_cells)
                candidate_max_bumps_values.append(candidate_max_bumps)
                candidate_plans.append(
                    (
                        candidate_edge_key,
                        record,
                        rr,
                        used_reserved_overlay,
                        candidate_max_bumps,
                        candidate_route_open_cells,
                    )
                )
            if candidate_failed:
                candidate_info["status"] = "no_candidate"
                candidate_attempts.append(candidate_info)
                continue
            candidate_info["status"] = "planned"
            candidate_info["failure_reason"] = ""
            candidate_attempts.append(candidate_info)
            selected_candidate = candidate
            selected_candidate_requested = candidate_requested
            selected_affected_edges = tuple(affected_edges)
            selected_plans = candidate_plans
            max_bumps = max(candidate_max_bumps_values, default=1)
            current_route_open_cells = candidate_open_cells
            break

        entry["planning_elapsed_s"] = planning_elapsed_for_entry_s
        entry["candidate_edges"] = attempted_edges
        entry["candidate_attempts"] = candidate_attempts
        entry["max_bumps"] = max_bumps
        entry["opened_route_cell_count"] = len(current_route_open_cells)

        if selected_candidate is None or not selected_plans:
            entry["status"] = "no_candidate"
            entry["reason"] = (
                str(last_exc)
                if last_exc is not None
                else f"no exact meander candidate found (|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)"
            )
            results.append(entry)
            continue
        inserted = selected_candidate_requested
        physical_inserted = sum(
            _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0)
            for _, _, rr, _, _, _ in selected_plans
        )
        unmatched = 0.0
        representative_rr = selected_plans[0][2]
        entry["status"] = "planned"
        entry["reason"] = ""
        entry["inserted_extra_length_um"] = inserted
        entry["physical_inserted_extra_length_um"] = physical_inserted
        entry["unmatched_length_um"] = unmatched
        entry["effective_bend_radius_um"] = representative_rr.get("effective_bend_radius_um")
        entry["primitive_bend_radius_um"] = representative_rr.get("primitive_bend_radius_um")
        entry["selected_box"] = representative_rr.get("selected_box")
        entry["selected_grid_rect"] = representative_rr.get("selected_grid_rect")
        entry["bumps"] = representative_rr.get("bumps")
        entry["side"] = representative_rr.get("side")
        entry["planning_mode"] = representative_rr.get("planning_mode", "fill_box_multi_bump")
        entry["candidate_runs"] = representative_rr.get("candidate_runs")
        entry["candidate_intervals"] = representative_rr.get("candidate_intervals")
        entry["rejected_box_blocked"] = representative_rr.get("rejected_box_blocked")
        entry["rejected_planning_failed"] = representative_rr.get("rejected_planning_failed")
        entry["rejected_exact_length_mismatch"] = representative_rr.get("rejected_exact_length_mismatch")
        entry["rejected_too_short"] = representative_rr.get("rejected_too_short")
        entry["selected_interval_length_um"] = representative_rr.get("selected_interval_length_um")
        entry["endpoint_inset_um"] = endpoint_inset_um
        entry["requested_probe_length_um"] = requested
        entry["used_reserved_overlay"] = all(plan[3] for plan in selected_plans)
        entry["selected_candidate_reason"] = selected_candidate.reason
        entry["selected_candidate_edge_count"] = len(selected_candidate.edge_keys)
        entry["affected_requirement_edges"] = [
            edge_key_to_dict(affected_edge)
            for affected_edge in selected_affected_edges
        ]
        entry["planned_edge"] = edge_key_to_dict(selected_plans[0][0])
        entry["planned_edges"] = [
            edge_key_to_dict(candidate_edge_key)
            for candidate_edge_key, *_ in selected_plans
        ]
        if unmatched > 1.0e-9:
            entry["status"] = "planned_partial"
        for affected_edge in selected_affected_edges:
            effective_inserted_by_requirement_edge[affected_edge] = (
                effective_inserted_by_requirement_edge.get(affected_edge, 0.0)
                + inserted
            )
        total_inserted += inserted * len(selected_affected_edges)
        total_physical_inserted += physical_inserted
        for (
            selected_edge_key,
            record,
            rr,
            used_reserved_overlay,
            candidate_max_bumps,
            _candidate_route_open_cells,
        ) in selected_plans:
            new_reserved_cells = _grid_rect_cells(rr.get("selected_grid_rect"))
            if new_reserved_cells:
                reserved_meander_cells.update(new_reserved_cells)
                if not used_reserved_overlay:
                    router.add_static_cells(list(new_reserved_cells))
            updated[selected_edge_key] = _planned_record(
                record=record,
                requested=float(selected_candidate_requested),
                rr=rr,
                min_straight_um=min_straight_um,
                max_bumps=candidate_max_bumps,
                max_height_um=max_height_um,
                min_seg_um=min_seg_um,
                endpoint_inset_um=endpoint_inset_um,
            )
        results.append(entry)
        for affected_edge in selected_affected_edges:
            if affected_edge == edge_key:
                continue
            virtual_entry = {
                **entry,
                "edge": edge_key_to_dict(affected_edge),
                "satisfied_by_requirement_edge": edge_key_to_dict(edge_key),
            }
            results.append(virtual_entry)

    total_unmatched = max(0.0, total_requested - total_inserted)
    return (
        [updated.get(_record_edge_key(r), r) for r in routed_net_records],
        {
            "results": results,
            "total_requested_extra_length_um": float(total_requested),
            "total_inserted_extra_length_um": float(total_inserted),
            "total_physical_inserted_extra_length_um": float(total_physical_inserted),
            "total_disregarded_extra_length_um": float(total_disregarded),
            "unmatched_length_um": float(total_unmatched),
            "planner_calls": int(planner_calls),
            "planner_elapsed_s": float(planner_elapsed_s),
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
