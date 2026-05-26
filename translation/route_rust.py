"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence, TypedDict, cast

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
    PortRef,
    RoutedEdgeKey,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend


@dataclass(frozen=True)
class RustRouteDebugArtifacts:
    obstacle_svg: Path | None
    route_svgs: list[Path]
    routed_edge_lengths_um: dict[RoutedEdgeKey, float]
    routed_net_records: list["RoutedNetRecord"] = field(default_factory=list)
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
    meander_gap_plan: "MeanderGapPlan | None" = None


class MeanderGapPlan(TypedDict):
    gap_start_um: tuple[float, float]
    gap_end_um: tuple[float, float]
    meander_height_um: float
    side: int


@dataclass(frozen=True)
class MeanderInsertionConfig:
    enabled: bool = True
    min_candidate_straight_length_um: float = 2.0
    max_extra_length_per_region_um: float = 200.0
    conservative_legal_check: bool = True


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
    schematic: Schematic,
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


def analysis_to_info_dict(analysis: PathLengthAnalysisResult) -> dict[str, object]:
    return {
        "topological_order": list(analysis.topological_order),
        "node_arrival_um": {
            str(node): float(arrival)
            for node, arrival in analysis.node_arrival_um.items()
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


def _plan_gap_on_candidate(
    start_um: tuple[float, float],
    end_um: tuple[float, float],
    *,
    gap_width_um: float,
    anchor: str,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    dx = float(end_um[0]) - float(start_um[0])
    dy = float(end_um[1]) - float(start_um[1])
    length = math.hypot(dx, dy)
    if length <= 1.0e-12 or gap_width_um <= 0.0 or gap_width_um > length + 1.0e-12:
        return None
    ux, uy = dx / length, dy / length
    if anchor == "begin":
        start_offset = 0.0
    elif anchor == "end":
        start_offset = length - gap_width_um
    else:
        start_offset = (length - gap_width_um) / 2.0
    start_offset = max(0.0, min(start_offset, length - gap_width_um))
    gs = (start_um[0] + ux * start_offset, start_um[1] + uy * start_offset)
    ge = (gs[0] + ux * gap_width_um, gs[1] + uy * gap_width_um)
    return gs, ge


def analyze_meander_insertion_for_requirements(
    routed_net_records: list[RoutedNetRecord],
    requirements: list[MissingLengthRequirement],
    *,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    gap_anchor: str = "middle",
) -> tuple[list[RoutedNetRecord], dict[str, object]]:
    """Gap-only plan (no meander insertion)."""
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

    by_edge = {_record_edge_key(r): r for r in routed_net_records}
    updated = dict(by_edge)
    results: list[dict[str, object]] = []
    total_requested = 0.0
    min_bend_radius_um = 10.0
    spacing_um = 2.0
    max_height_um = 80.0
    min_height_um = max(min_bend_radius_um + spacing_um, spacing_um)
    width_um = 4.0 * min_bend_radius_um

    for req in requirements:
        requested = float(req.missing_length_um)
        total_requested += requested
        edge_key = req.edge_key
        record = by_edge.get(edge_key)
        entry = {
            "edge": edge_key_to_dict(edge_key),
            "requested_extra_length_um": requested,
            "inserted_extra_length_um": 0.0,
            "status": "no_candidate",
            "reason": "no_matching_routed_record",
            "candidate_count": 0,
            "candidate_lengths_um": [],
            "analysis_found_solution": False,
            "analysis_reason": "no_matching_routed_record",
            "side": 1,
            "number_of_bumps": 1,
            "meander_height_um": None,
            "meander_width_um": width_um,
            "planned_extra_length_um": 0.0,
            "residual_um": requested,
            "gap_anchor": gap_anchor,
            "gap_start_um": None,
            "gap_end_um": None,
        }
        if record is None:
            results.append(entry)
            continue
        rr = router.analyze_meander_insertion_candidate(
            record.route_obj,
            requested,
            min_endpoint_margin_cells=1,
            min_candidate_straight_length_um=2.0,
            max_extra_length_per_region_um=200.0,
            conservative_legal_check=True,
        )
        candidates = list(rr.get("candidates", []))
        lengths = [float(c.get("length_um", 0.0)) for c in candidates]
        entry["status"] = str(rr.get("status", "unknown"))
        entry["reason"] = str(rr.get("reason", ""))
        entry["candidate_count"] = len(candidates)
        entry["candidate_lengths_um"] = lengths
        if not candidates:
            results.append(entry)
            continue
        h = (requested - (2.0 * math.pi * min_bend_radius_um) + width_um) / 2.0
        entry["meander_height_um"] = h
        if h < min_height_um:
            entry["analysis_reason"] = "required_height_below_minimum"
            results.append(entry)
            continue
        if h > max_height_um:
            entry["analysis_reason"] = "required_height_above_maximum"
            results.append(entry)
            continue
        cand = candidates[0]
        states_obj = getattr(record.route_obj, "states", None)
        if states_obj is None:
            entry["analysis_reason"] = "route_states_unavailable"
            results.append(entry)
            continue
        states = cast(Sequence[object], states_obj)
        sidx = int(cand.get("start_index", -1))
        eidx = int(cand.get("end_index", -1))
        if sidx < 0 or eidx <= sidx or eidx >= len(states):
            entry["analysis_reason"] = "candidate_indices_invalid"
            results.append(entry)
            continue
        sx, sy = _state_xy(states[sidx])
        ex, ey = _state_xy(states[eidx])
        cand_start = _grid_to_um(sx, sy, realization_grid_spec)
        cand_end = _grid_to_um(ex, ey, realization_grid_spec)
        gap = _plan_gap_on_candidate(cand_start, cand_end, gap_width_um=width_um, anchor=gap_anchor)
        if gap is None:
            entry["analysis_reason"] = "failed_to_place_gap_on_candidate"
            results.append(entry)
            continue
        gs, ge = gap
        entry["analysis_found_solution"] = True
        entry["analysis_reason"] = "continuous_single_bump_solution_found"
        entry["planned_extra_length_um"] = requested
        entry["residual_um"] = 0.0
        entry["gap_start_um"] = gs
        entry["gap_end_um"] = ge
        gap_plan: MeanderGapPlan = {
            "gap_start_um": gs,
            "gap_end_um": ge,
            "meander_height_um": float(h),
            "side": 1,
        }
        updated[edge_key] = RoutedNetRecord(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
            route_obj=record.route_obj,
            total_length_um=record.total_length_um,
            meander_gap_plan=gap_plan,
        )
        results.append(entry)

    return (
        [updated.get(_record_edge_key(r), r) for r in routed_net_records],
        {
            "results": results,
            "total_requested_extra_length_um": float(total_requested),
            "total_inserted_extra_length_um": 0.0,
            "unmatched_length_um": float(total_requested),
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
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_cells=bend_radius_cells,
        gap_anchor="middle",
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
        if status not in {"no_candidate", "insufficient_space"}:
            status = "unsupported_representation"
            reason = "meander polygon realization not implemented for this candidate shape"
        results.append(
            MeanderInsertionResult(
                edge=edge,
                requested_extra_length_um=_as_float(item.get("requested_extra_length_um", 0.0), 0.0),
                inserted_extra_length_um=0.0,
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
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    max_iterations: int = 500_000,
    debug_timing: bool = False,
) -> RouteRustPipelineResult:
    """Run Phase A->(optional M1)->B entirely in route_rust."""
    routed_layout, debug_artifacts = route_nets_rust(
        unrouted_layout,
        schematic,
        obstacle_config=obstacle_config,
        debug_dir=debug_dir,
        debug_prefix=debug_prefix,
        route_width_um=route_width_um,
        route_layer=route_layer,
        allow_45_degree_turns=allow_45_degree_turns,
        max_iterations=max_iterations,
        debug_timing=debug_timing,
        defer_realization=True,
    )

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
        records_for_realization, meander_report_info = analyze_meander_insertion_for_requirements(
            debug_artifacts.routed_net_records,
            requirements,
            realization_grid_spec=debug_artifacts.realization_grid_spec,
            allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
            bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
            gap_anchor="middle",
        )

    if debug_artifacts.realization_grid_spec is None:
        raise RuntimeError("Missing realization grid spec from routing phase.")
    realize_routed_net_records(
        routed_layout,
        records_for_realization,
        route_width_um=route_width_um,
        route_layer=route_layer,
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_45_degree_turns=debug_artifacts.realization_allow_45_degree_turns,
        bend_radius_cells=debug_artifacts.realization_bend_radius_cells,
    )

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

    def _route_centerline_points(route_obj: object) -> list[tuple[float, float]]:
        segments = getattr(route_obj, "segments", None) or []
        pts: list[tuple[float, float]] = []
        for seg in segments:
            kind = str(seg.get("kind", ""))
            start = seg.get("start", None)
            end = seg.get("end", None)
            if start is None or end is None:
                continue
            s_um = _grid_to_um(int(start[0]), int(start[1]), realization_grid_spec)
            e_um = _grid_to_um(int(end[0]), int(end[1]), realization_grid_spec)
            if not pts:
                pts.append(s_um)
            if kind == "straight":
                pts.append(e_um)
                continue

            # Arc-aware bend reconstruction from primitive metadata.
            start_angle_idx = int(seg.get("start_angle", 0)) % 8
            end_angle_idx = int(seg.get("end_angle", start_angle_idx)) % 8
            delta = (end_angle_idx - start_angle_idx) % 8
            if delta == 0:
                pts.append(e_um)
                continue
            if delta in (1, 2):
                turn_sign = 1.0  # CCW
                theta = (math.pi / 4.0) * float(delta)
            elif delta in (6, 7):
                turn_sign = -1.0  # CW
                theta = (math.pi / 4.0) * float(8 - delta)
            else:
                pts.append(e_um)
                continue
            if theta <= 1.0e-12:
                pts.append(e_um)
                continue

            # Build arc from endpoints + known sweep, so endpoints are consistent.
            vx = e_um[0] - s_um[0]
            vy = e_um[1] - s_um[1]
            chord = math.hypot(vx, vy)
            if chord <= 1.0e-12:
                pts.append(e_um)
                continue
            sin_half = math.sin(theta / 2.0)
            if abs(sin_half) <= 1.0e-12:
                pts.append(e_um)
                continue
            radius = chord / (2.0 * sin_half)
            mid = ((s_um[0] + e_um[0]) / 2.0, (s_um[1] + e_um[1]) / 2.0)
            nx = -vy / chord
            ny = vx / chord
            center_off = math.sqrt(max(0.0, (radius * radius) - ((chord * 0.5) ** 2)))
            c1 = (mid[0] + nx * center_off, mid[1] + ny * center_off)
            c2 = (mid[0] - nx * center_off, mid[1] - ny * center_off)

            def _norm_angle(a: float) -> float:
                return (a + 2.0 * math.pi) % (2.0 * math.pi)

            def _candidate(center: tuple[float, float]) -> tuple[float, float, float]:
                a0c = math.atan2(s_um[1] - center[1], s_um[0] - center[0])
                a1c = math.atan2(e_um[1] - center[1], e_um[0] - center[0])
                if turn_sign > 0:
                    sw = _norm_angle(a1c - a0c)
                else:
                    sw = -_norm_angle(a0c - a1c)
                return a0c, a1c, sw

            a0_1, _, sw1 = _candidate(c1)
            a0_2, _, sw2 = _candidate(c2)
            err1 = abs(abs(sw1) - theta) + (0.0 if sw1 * turn_sign > 0 else 10.0)
            err2 = abs(abs(sw2) - theta) + (0.0 if sw2 * turn_sign > 0 else 10.0)
            if err1 <= err2:
                center = c1
                a0 = a0_1
                sweep = sw1
            else:
                center = c2
                a0 = a0_2
                sweep = sw2

            n = max(6, int(24 * (abs(sweep) / (math.pi / 2.0))))
            for i in range(1, n + 1):
                t = i / n
                a = a0 + sweep * t
                p = (center[0] + radius * math.cos(a), center[1] + radius * math.sin(a))
                if i == n:
                    # Preserve exact segment chaining endpoint.
                    p = e_um
                if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1.0e-9:
                    pts.append(p)
        return pts

    def _poly_from_centerline(centerline: list[tuple[float, float]], width_um_local: float) -> list[tuple[float, float]]:
        if len(centerline) < 2:
            return []
        hw = float(width_um_local) / 2.0
        left: list[tuple[float, float]] = []
        right: list[tuple[float, float]] = []
        for i, p in enumerate(centerline):
            if i == 0:
                p_next = centerline[i + 1]
                dx, dy = p_next[0] - p[0], p_next[1] - p[1]
            elif i == len(centerline) - 1:
                p_prev = centerline[i - 1]
                dx, dy = p[0] - p_prev[0], p[1] - p_prev[1]
            else:
                p_prev = centerline[i - 1]
                p_next = centerline[i + 1]
                dx, dy = p_next[0] - p_prev[0], p_next[1] - p_prev[1]
            seg = math.hypot(dx, dy)
            if seg <= 1.0e-9:
                continue
            nx, ny = -dy / seg, dx / seg
            left.append((p[0] + nx * hw, p[1] + ny * hw))
            right.append((p[0] - nx * hw, p[1] - ny * hw))
        return left + list(reversed(right))

    def _insert_gap_in_centerline(
        centerline: list[tuple[float, float]],
        gap_start: tuple[float, float],
        gap_end: tuple[float, float],
    ) -> tuple[list[tuple[float, float]], list[tuple[float, float]]] | None:
        if len(centerline) < 2:
            return None
        tol = 1.0e-9
        seg_lens: list[float] = []
        cum = [0.0]
        for i in range(len(centerline) - 1):
            a = centerline[i]
            b = centerline[i + 1]
            l = math.hypot(b[0] - a[0], b[1] - a[1])
            seg_lens.append(l)
            cum.append(cum[-1] + l)
        total = cum[-1]
        if total <= tol:
            return None

        def proj_s(p: tuple[float, float]) -> float:
            best_s = 0.0
            best_d2 = float("inf")
            for i, l in enumerate(seg_lens):
                if l <= tol:
                    continue
                a = centerline[i]
                b = centerline[i + 1]
                dx = b[0] - a[0]
                dy = b[1] - a[1]
                t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (l * l)
                t = max(0.0, min(1.0, t))
                q = (a[0] + dx * t, a[1] + dy * t)
                d2 = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_s = cum[i] + t * l
            return best_s

        s0 = proj_s(gap_start)
        s1 = proj_s(gap_end)
        if s1 < s0:
            s0, s1 = s1, s0
        if s1 - s0 <= 1.0e-6:
            return None

        def split_at_s(s: float) -> tuple[int, tuple[float, float]]:
            for i, l in enumerate(seg_lens):
                if cum[i + 1] + tol < s:
                    continue
                if l <= tol:
                    return i, centerline[i]
                a = centerline[i]
                b = centerline[i + 1]
                t = (s - cum[i]) / l
                t = max(0.0, min(1.0, t))
                return i, (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            return len(seg_lens) - 1, centerline[-1]

        i0, p0 = split_at_s(s0)
        i1, p1 = split_at_s(s1)
        first = list(centerline[: i0 + 1])
        if math.hypot(first[-1][0] - p0[0], first[-1][1] - p0[1]) > 1.0e-6:
            first.append(p0)
        second = [p1]
        second.extend(centerline[i1 + 1 :])
        if len(second) == 1:
            second.append(centerline[-1])
        if len(first) < 2 or len(second) < 2:
            return None
        return first, second

    def _build_single_bump_meander_centerline(
        gap_start: tuple[float, float],
        gap_end: tuple[float, float],
        *,
        meander_height_um: float,
        side: int,
        samples_per_90: int = 24,
    ) -> list[tuple[float, float]] | None:
        # Exact topology: 90 -> straight -> 180 -> straight -> 90
        dx = float(gap_end[0]) - float(gap_start[0])
        dy = float(gap_end[1]) - float(gap_start[1])
        width = math.hypot(dx, dy)
        if width <= 1.0e-9:
            return None
        r = width / 4.0
        h = float(meander_height_um)
        if r <= 1.0e-9 or h < 0.0:
            return None
        ux, uy = dx / width, dy / width
        nx, ny = -uy, ux
        sgn = 1.0 if int(side) >= 0 else -1.0
        nx *= sgn
        ny *= sgn

        def world(xl: float, yl: float) -> tuple[float, float]:
            return (
                gap_start[0] + xl * ux + yl * nx,
                gap_start[1] + xl * uy + yl * ny,
            )

        pts: list[tuple[float, float]] = [world(0.0, 0.0)]

        def append_arc(
            center_local: tuple[float, float],
            start_local: tuple[float, float],
            end_local: tuple[float, float],
            sweep_sign: float,
            sweep_mag: float,
            n90_scale: float,
        ) -> None:
            c = world(center_local[0], center_local[1])
            s = world(start_local[0], start_local[1])
            e = world(end_local[0], end_local[1])
            a0 = math.atan2(s[1] - c[1], s[0] - c[0])
            sweep = sweep_sign * sweep_mag
            n = max(8, int(samples_per_90 * n90_scale))
            for i in range(1, n + 1):
                t = i / n
                a = a0 + sweep * t
                p = (c[0] + r * math.cos(a), c[1] + r * math.sin(a))
                if i == n:
                    p = e
                if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1.0e-9:
                    pts.append(p)

        # Arc 1: from (0,0) to (r,r), CCW 90 in local frame.
        append_arc(
            center_local=(0.0, r),
            start_local=(0.0, 0.0),
            end_local=(r, r),
            sweep_sign=1.0,
            sweep_mag=math.pi / 2.0,
            n90_scale=1.0,
        )
        # Straight up.
        p = world(r, r + h)
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1.0e-9:
            pts.append(p)
        # Arc 2: from (r,r+h) to (3r,r+h), CW 180 in local frame.
        append_arc(
            center_local=(2.0 * r, r + h),
            start_local=(r, r + h),
            end_local=(3.0 * r, r + h),
            sweep_sign=-1.0,
            sweep_mag=math.pi,
            n90_scale=2.0,
        )
        # Straight down.
        p = world(3.0 * r, r)
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1.0e-9:
            pts.append(p)
        # Arc 3: from (3r,r) to (4r,0), CW 90 in local frame.
        append_arc(
            center_local=(4.0 * r, r),
            start_local=(3.0 * r, r),
            end_local=(4.0 * r, 0.0),
            sweep_sign=1.0,
            sweep_mag=math.pi / 2.0,
            n90_scale=1.0,
        )
        if math.hypot(gap_end[0] - pts[-1][0], gap_end[1] - pts[-1][1]) > 1.0e-9:
            pts.append((float(gap_end[0]), float(gap_end[1])))
        return pts

    for record in routed_net_records:
        if record.meander_gap_plan is not None:
            cl = _route_centerline_points(record.route_obj)
            gs = record.meander_gap_plan["gap_start_um"]
            ge = record.meander_gap_plan["gap_end_um"]
            split = _insert_gap_in_centerline(cl, gs, ge)
            if split is not None:
                first_cl, second_cl = split
                h = float(record.meander_gap_plan["meander_height_um"])
                side = int(record.meander_gap_plan["side"])
                meander_cl = _build_single_bump_meander_centerline(
                    gs,
                    ge,
                    meander_height_um=h,
                    side=side,
                )
                if meander_cl is not None and len(meander_cl) >= 2:
                    combined = list(first_cl)
                    combined.extend(meander_cl[1:])
                    combined.extend(second_cl[1:])
                    poly = _poly_from_centerline(combined, route_width_um)
                    if poly:
                        routed_layout.add_polygon(poly, layer=route_layer)
                        continue
                # Fallback to gap-only if meander construction fails.
                poly1 = _poly_from_centerline(first_cl, route_width_um)
                poly2 = _poly_from_centerline(second_cl, route_width_um)
                if poly1:
                    routed_layout.add_polygon(poly1, layer=route_layer)
                if poly2:
                    routed_layout.add_polygon(poly2, layer=route_layer)
                continue
        polygon = router.realize_route_polygon(record.route_obj, float(route_width_um))
        routed_layout.add_polygon(polygon, layer=route_layer)


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _grid_origin_xy(grid: GridSpec) -> tuple[float, float]:
    if hasattr(grid, "origin"):
        origin = getattr(grid, "origin")
        return float(origin[0]), float(origin[1])
    return (
        float(getattr(grid, "origin_x_um", 0.0)),
        float(getattr(grid, "origin_y_um", 0.0)),
    )


def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    max_iterations: int = 500_000,
    debug_timing: bool = False,
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
    obstacle_map = build_static_obstacle_map(unrouted_layout, config=obstacle_config)
    if debug_timing:
        t_obstacle_end = time.perf_counter()
        print(f"      - Obstacle Map time: {t_obstacle_end - t_obstacle_start:.4f} s")
    grid = obstacle_map.grid

    debug_path = Path(debug_dir) if debug_dir is not None else None
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
    print(f"\nRouting {len(nets)} nets using Rust router...")

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
    astar_cfg = rust_backend.AStarConfig(max_iterations=int(max_iterations))
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)

    block_radius_cells = max(
        0, math.ceil((float(route_width_um) / 2.0) / float(grid.grid_size_um))
    )
    router.set_static_cells(sorted(obstacle_map.blocked_cells))
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

    t_astar_start = 0.0
    if debug_timing:
        t_astar_start = time.perf_counter()

    for net_name, bundle in nets.items():
        links = bundle.links
        for port1_spec, port2_spec in links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")

            source_port = get_port_from_instance(routed_layout, inst1, port1)
            target_port = get_port_from_instance(routed_layout, inst2, port2)

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

            print(f"  Routing {net_name}: {port1_spec} -> {port2_spec}...", end=" ")

            net_id += 1
            opened_cells = sorted(
                {
                    (int(source_state.x), int(source_state.y)),
                    (int(target_state.x), int(target_state.y)),
                }
            )
            try:
                route_obj = router.route_single_net_and_commit(
                    net_id,
                    source_state,
                    target_state,
                    block_radius_cells,
                    opened_cells,
                )
            except RuntimeError as exc:
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
                )
            )
            routed_edge_lengths_um[edge_key] = float(route_obj.total_length_um)

            if debug_path is not None:
                route_dir = debug_path / "routes"
                _ensure_dir(route_dir)
                route_svg = route_dir / f"{debug_prefix}_{net_name}.svg"
                route_svg.write_text(router.export_debug_svg(route_obj), encoding="utf-8")
                route_svgs.append(route_svg)

            print("ok")
        # break

    if debug_timing:
        t_astar_end = time.perf_counter()
        print(f"      - Astar time: {t_astar_end - t_astar_start:.4f} s")

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
            bend_radius_cells=4,
        )

    return routed_layout, RustRouteDebugArtifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
        routed_edge_lengths_um=routed_edge_lengths_um,
        routed_net_records=routed_net_records,
        realization_grid_spec=realization_grid_spec,
        realization_allow_45_degree_turns=allow_45_degree_turns,
        realization_bend_radius_cells=4,
    )
