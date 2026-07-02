"""Route an unrouted GDS layout using the Rust router backend."""

from __future__ import annotations

import importlib
import json
import math
import sys
import time
from collections import Counter
from dataclasses import is_dataclass, replace
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
    find_component_port_access_rule,
    get_routing_obstacle_layers,
)
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
from translation.route_rust_realization import realize_routed_net_records
from translation.route_rust_records import (
    EndpointCorrectionRouter,
    RouteBookkeeping,
    apply_port_endpoint_corrections,
    build_port_alignment_diagnostics,
    build_route_debug_artifacts,
    route_edge_key,
    routed_edge_lengths_from_records,
)
from translation.route_rust_types import (
    DEFAULT_MEANDER_MAX_HEIGHT_UM,
    MeanderInsertionConfig,
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


def _centerline_tuple(points: object) -> tuple[tuple[float, float], ...]:
    if not isinstance(points, list):
        return ()
    out: list[tuple[float, float]] = []
    for point in points:
        if not isinstance(point, (tuple, list)) or len(point) != 2:
            return ()
        try:
            out.append((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            return ()
    return tuple(out)


def _route_waypoints_from_obj(route_obj: object | None) -> tuple[tuple[float, float], ...]:
    if route_obj is None:
        return ()

    for attr_name in ("compressed_waypoints", "cells"):
        raw_points = getattr(route_obj, attr_name, None)
        if raw_points is None:
            continue
        points: list[tuple[float, float]] = []
        for point in cast(Iterable[Any], raw_points):
            if not isinstance(point, (tuple, list)) or len(point) != 2:
                points = []
                break
            try:
                points.append((float(point[0]), float(point[1])))
            except (TypeError, ValueError):
                points = []
                break
        if len(points) >= 2:
            return tuple(points)

    raw_states = getattr(route_obj, "states", None)
    if raw_states is None:
        return ()
    points = []
    for state in cast(Iterable[Any], raw_states):
        try:
            points.append((float(getattr(state, "x")), float(getattr(state, "y"))))
        except (TypeError, ValueError):
            return ()
    return tuple(points) if len(points) >= 2 else ()


def _route_segments_from_waypoints(
    waypoints: tuple[tuple[float, float], ...],
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for start, end in zip(waypoints, waypoints[1:]):
        if start == end:
            continue
        segments.append((start, end))
    return tuple(segments)


def _segment_intersection_with_params(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> tuple[float, float, float, float] | None:
    ax = a1[0] - a0[0]
    ay = a1[1] - a0[1]
    bx = b1[0] - b0[0]
    by = b1[1] - b0[1]
    denom = ax * by - ay * bx
    if abs(denom) < 1e-9:
        return None

    cx = b0[0] - a0[0]
    cy = b0[1] - a0[1]
    t = (cx * by - cy * bx) / denom
    u = (cx * ay - cy * ax) / denom
    eps = 1e-9
    if -eps <= t <= 1.0 + eps and -eps <= u <= 1.0 + eps:
        return (a0[0] + t * ax, a0[1] + t * ay, t, u)
    return None


def _segment_length_cells(
    segment: tuple[tuple[float, float], tuple[float, float]],
) -> float:
    start, end = segment
    return max(abs(end[0] - start[0]), abs(end[1] - start[1]))


def _segments_are_perpendicular(
    segment_a: tuple[tuple[float, float], tuple[float, float]],
    segment_b: tuple[tuple[float, float], tuple[float, float]],
) -> bool:
    a0, a1 = segment_a
    b0, b1 = segment_b
    av = (a1[0] - a0[0], a1[1] - a0[1])
    bv = (b1[0] - b0[0], b1[1] - b0[1])
    return abs(av[0] * bv[0] + av[1] * bv[1]) < 1e-9


def _first_perpendicular_route_intersection(
    route_obj_a: object | None,
    route_obj_b: object | None,
) -> dict[str, object] | None:
    waypoints_a = _route_waypoints_from_obj(route_obj_a)
    waypoints_b = _route_waypoints_from_obj(route_obj_b)
    if not waypoints_a or not waypoints_b:
        return None

    for segment_a in _route_segments_from_waypoints(waypoints_a):
        a0, a1 = segment_a
        for segment_b in _route_segments_from_waypoints(waypoints_b):
            b0, b1 = segment_b
            if not _segments_are_perpendicular(segment_a, segment_b):
                continue
            intersection = _segment_intersection_with_params(a0, a1, b0, b1)
            if intersection is None:
                continue
            point_x, point_y, t, u = intersection
            len_a = _segment_length_cells(segment_a)
            len_b = _segment_length_cells(segment_b)
            margin_a = min(max(t, 0.0) * len_a, max(1.0 - t, 0.0) * len_a)
            margin_b = min(max(u, 0.0) * len_b, max(1.0 - u, 0.0) * len_b)
            return {
                "point": [round(float(point_x), 6), round(float(point_y), 6)],
                "segment_a": [
                    [round(float(a0[0]), 6), round(float(a0[1]), 6)],
                    [round(float(a1[0]), 6), round(float(a1[1]), 6)],
                ],
                "segment_b": [
                    [round(float(b0[0]), 6), round(float(b0[1]), 6)],
                    [round(float(b1[0]), 6), round(float(b1[1]), 6)],
                ],
                "segment_a_margin_cells": round(float(margin_a), 6),
                "segment_b_margin_cells": round(float(margin_b), 6),
            }
    return None


def _bbox_size_um(component: Component) -> tuple[float, float] | None:
    try:
        bbox = component.dbbox() if callable(component.dbbox) else component.dbbox
    except (AttributeError, TypeError):
        try:
            bbox = component.bbox() if callable(component.bbox) else component.bbox
        except (AttributeError, TypeError):
            return None
    try:
        width = float(bbox.right) - float(bbox.left)
        height = float(bbox.top) - float(bbox.bottom)
    except AttributeError:
        try:
            left, bottom, right, top = cast(Any, bbox)
            width = float(right) - float(left)
            height = float(top) - float(bottom)
        except (TypeError, ValueError):
            return None
    if not math.isfinite(width) or not math.isfinite(height) or width <= 0 or height <= 0:
        return None
    return width, height


def _crossing_component_bbox_size_um() -> tuple[str, float, float] | None:
    try:
        component = gf.components.crossing()
    except Exception:
        try:
            from gdsfactory.gpdk import get_generic_pdk

            get_generic_pdk().activate()
            component = gf.components.crossing()
        except Exception:
            return None
    size = _bbox_size_um(component)
    if size is None:
        return None
    return str(component.name), float(size[0]), float(size[1])


def _resolve_crossing_half_size_cells(
    *,
    requested_half_size_cells: int,
    enable_crossings: bool,
    grid_size_um: float,
    clearance_um: float,
) -> tuple[int, dict[str, object]]:
    info: dict[str, object] = {
        "requested_half_size_cells": int(requested_half_size_cells),
        "derived_from_component": False,
    }
    if requested_half_size_cells > 0 or not enable_crossings:
        info["half_size_cells"] = int(requested_half_size_cells)
        return int(requested_half_size_cells), info

    component_size = _crossing_component_bbox_size_um()
    if component_size is None:
        info["reason"] = "crossing_component_unavailable"
        info["half_size_cells"] = 0
        return 0, info

    component_name, width_um, height_um = component_size
    half_extent_um = max(width_um, height_um) / 2.0 + max(0.0, float(clearance_um))
    half_size_cells = int(math.ceil(half_extent_um / float(grid_size_um)))
    info.update(
        {
            "component_name": component_name,
            "component_bbox_um": [width_um, height_um],
            "clearance_um": float(clearance_um),
            "grid_size_um": float(grid_size_um),
            "half_size_cells": half_size_cells,
            "derived_from_component": True,
        }
    )
    return half_size_cells, info


def _port_center_um(port: object) -> tuple[float, float] | None:
    center = getattr(port, "center", None)
    if center is None:
        center = getattr(port, "dcenter", None)
    if center is None:
        return None
    try:
        center_seq = cast(Iterable[Any], center)
        x_um, y_um = tuple(center_seq)[:2]
        return (float(x_um), float(y_um))
    except (TypeError, ValueError, IndexError):
        return None


def _edge_key_to_info(edge_key: object) -> dict[str, object]:
    source = getattr(edge_key, "source", None)
    target = getattr(edge_key, "target", None)
    return {
        "net_name": str(getattr(edge_key, "net_name", "")),
        "source_instance": str(getattr(source, "instance", "")),
        "source_port": str(getattr(source, "port", "")),
        "target_instance": str(getattr(target, "instance", "")),
        "target_port": str(getattr(target, "port", "")),
    }


def _build_crossing_plan_info(
    *,
    rust_backend: object,
    router: object,
    schematic: Schematic,
    route_jobs: list[RouteJob],
    enable_crossings: bool,
    node_depths: dict[str, int] | None,
    node_ranks: dict[str, int] | None,
    edge_ranks: dict[str, dict[str, int]] | None,
    crossing_loss: float,
    crossing_half_size_cells: int,
    min_straight_cells_per_crossing: int,
    allow_only_expected_crossings: bool,
) -> dict[str, object]:
    info: dict[str, object] = {
        "enabled": bool(enable_crossings),
        "constraint_count": 0,
        "event_count": 0,
        "missing_event_count": 0,
        "missing_events": [],
        "events": [],
        "expected_crossings_by_net_id": {},
        "expected_crossings_by_net_name": {},
    }
    if not enable_crossings:
        return info

    required_backend = (
        "CrossingConfig",
        "CrossingConstraint",
    )
    missing_backend = [name for name in required_backend if not hasattr(rust_backend, name)]
    required_router = (
        "set_crossing_config",
        "set_crossing_constraints",
        "crossing_expected_count",
    )
    missing_router = [name for name in required_router if not hasattr(router, name)]
    if missing_backend or missing_router:
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose crossing "
            "configuration APIs. Rebuild it with `maturin develop`. "
            f"Missing backend attrs: {missing_backend}; missing router attrs: {missing_router}."
        )

    if node_depths is None or node_ranks is None or edge_ranks is None:
        info["reason"] = "missing_topology_metadata"
        return info

    try:
        topology = analyze_schematic_topology(
            schematic,
            node_depths=node_depths,
            node_ranks=node_ranks,
            edge_ranks=edge_ranks,
        )
        crossing_plan: CrossingPlan = build_crossing_plan(topology)
    except (KeyError, ValueError) as exc:
        info["reason"] = "invalid_topology_metadata"
        info["error"] = str(exc)
        return info
    info["event_count"] = len(crossing_plan.events)
    info["stage_count"] = len(crossing_plan.stages)
    info["plan_text"] = crossing_plan.to_text(include_empty_stages=True)

    jobs_by_edge = {route_edge_key(job): job for job in route_jobs}
    constraints = []
    missing_events: list[dict[str, object]] = []
    event_records: list[dict[str, object]] = []
    crossing_counts_by_net_id: Counter[int] = Counter()
    crossing_counts_by_net_name: Counter[str] = Counter()

    for event in crossing_plan.events:
        job_a = jobs_by_edge.get(event.edge_a)
        job_b = jobs_by_edge.get(event.edge_b)
        event_record: dict[str, object] = {
            "edge_a": _edge_key_to_info(event.edge_a),
            "edge_b": _edge_key_to_info(event.edge_b),
            "source_depth": int(event.source_depth),
            "target_depth": int(event.target_depth),
            "level": int(event.level),
            "order_index": int(event.order_index),
            "edge_a_source_rank": int(event.edge_a_source_rank),
            "edge_a_target_rank": int(event.edge_a_target_rank),
            "edge_b_source_rank": int(event.edge_b_source_rank),
            "edge_b_target_rank": int(event.edge_b_target_rank),
        }
        if job_a is None or job_b is None:
            missing_record = {
                **event_record,
                "edge_a_found": job_a is not None,
                "edge_b_found": job_b is not None,
            }
            missing_events.append(missing_record)
            event_records.append({**event_record, "loaded": False})
            continue

        event_record.update(
            {
                "loaded": True,
                "net_id_a": int(job_a.net_id),
                "net_id_b": int(job_b.net_id),
                "net_name_a": str(job_a.net_name),
                "net_name_b": str(job_b.net_name),
            }
        )
        event_records.append(event_record)
        constraints.append(
            rust_backend.CrossingConstraint(
                int(job_a.net_id),
                int(job_b.net_id),
                level=int(event.level),
                source_depth=int(event.source_depth),
                target_depth=int(event.target_depth),
            )
        )
        crossing_counts_by_net_id[int(job_a.net_id)] += 1
        crossing_counts_by_net_id[int(job_b.net_id)] += 1
        crossing_counts_by_net_name[str(job_a.net_name)] += 1
        crossing_counts_by_net_name[str(job_b.net_name)] += 1

    router.set_crossing_constraints(constraints)
    router.set_crossing_config(
        rust_backend.CrossingConfig(
            enabled=True,
            crossing_loss=float(crossing_loss),
            crossing_half_size_cells=int(crossing_half_size_cells),
            min_straight_cells_per_crossing=int(min_straight_cells_per_crossing),
            allow_only_expected_pairs=bool(allow_only_expected_crossings),
        )
    )

    info["constraint_count"] = len(constraints)
    info["missing_event_count"] = len(missing_events)
    info["missing_events"] = missing_events
    info["events"] = event_records
    info["expected_crossings_by_net_id"] = dict(sorted(crossing_counts_by_net_id.items()))
    info["expected_crossings_by_net_name"] = dict(sorted(crossing_counts_by_net_name.items()))
    info["crossing_loss"] = float(crossing_loss)
    info["crossing_half_size_cells"] = int(crossing_half_size_cells)
    info["min_straight_cells_per_crossing"] = int(min_straight_cells_per_crossing)
    info["allow_only_expected_crossings"] = bool(allow_only_expected_crossings)
    return info


def _augment_crossing_plan_with_realized_overlaps(
    *,
    router: object,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord] | None = None,
) -> None:
    if not crossing_plan_info.get("enabled"):
        return
    if not hasattr(router, "all_net_core_cells"):
        crossing_plan_info["actual_crossing_reason"] = "missing_core_cell_api"
        return

    core_cells_by_net_id: dict[int, set[tuple[int, int]]] = {}
    for raw_net_id, raw_cells in router.all_net_core_cells():
        core_cells_by_net_id[int(raw_net_id)] = {
            (int(cell[0]), int(cell[1])) for cell in raw_cells
        }

    actual_crossings: list[dict[str, object]] = []
    unrealized_expected: list[dict[str, object]] = []
    required_crossing_margin_cells = max(
        int(crossing_plan_info.get("min_straight_cells_per_crossing", 0) or 0),
        int(crossing_plan_info.get("crossing_half_size_cells", 0) or 0),
    )
    for raw_event in list(crossing_plan_info.get("events", [])):
        event = dict(cast(dict[str, object], raw_event))
        if not event.get("loaded"):
            continue
        net_id_a = int(cast(int, event["net_id_a"]))
        net_id_b = int(cast(int, event["net_id_b"]))
        overlap = sorted(
            core_cells_by_net_id.get(net_id_a, set())
            & core_cells_by_net_id.get(net_id_b, set())
        )
        geometric_crossing = None
        if routed_records_by_net_id is not None:
            record_a = routed_records_by_net_id.get(net_id_a)
            record_b = routed_records_by_net_id.get(net_id_b)
            geometric_crossing = _first_perpendicular_route_intersection(
                record_a.route_obj if record_a is not None else None,
                record_b.route_obj if record_b is not None else None,
            )
        record = {
            "net_id_a": net_id_a,
            "net_id_b": net_id_b,
            "net_name_a": event.get("net_name_a"),
            "net_name_b": event.get("net_name_b"),
            "source_depth": event.get("source_depth"),
            "target_depth": event.get("target_depth"),
            "level": event.get("level"),
            "order_index": event.get("order_index"),
            "cell_count": len(overlap),
            "cells": [[int(x), int(y)] for x, y in overlap[:32]],
        }
        if geometric_crossing is not None:
            record["geometric"] = True
            record["point"] = geometric_crossing["point"]
            record["segment_a"] = geometric_crossing["segment_a"]
            record["segment_b"] = geometric_crossing["segment_b"]
            record["segment_a_margin_cells"] = geometric_crossing[
                "segment_a_margin_cells"
            ]
            record["segment_b_margin_cells"] = geometric_crossing[
                "segment_b_margin_cells"
            ]
            margin_a = float(geometric_crossing["segment_a_margin_cells"])
            margin_b = float(geometric_crossing["segment_b_margin_cells"])
            record["valid_crossing_geometry"] = (
                margin_a + 1e-9 >= required_crossing_margin_cells
                and margin_b + 1e-9 >= required_crossing_margin_cells
            )
            if not record["valid_crossing_geometry"]:
                record["unrealized_reason"] = "insufficient_straight_margin"
                record["required_margin_cells"] = required_crossing_margin_cells
        if geometric_crossing is not None and record.get("valid_crossing_geometry"):
            actual_crossings.append(record)
        elif overlap and routed_records_by_net_id is None:
            actual_crossings.append(record)
        else:
            unrealized_expected.append(record)

    crossing_plan_info["actual_crossing_count"] = len(actual_crossings)
    crossing_plan_info["actual_crossing_cell_count"] = sum(
        int(record["cell_count"]) for record in actual_crossings
    )
    crossing_plan_info["actual_geometric_crossing_count"] = sum(
        1 for record in actual_crossings if record.get("geometric")
    )
    crossing_plan_info["actual_crossings"] = actual_crossings
    crossing_plan_info["unrealized_expected_crossings"] = unrealized_expected
    crossing_plan_info["unrealized_expected_crossing_count"] = len(unrealized_expected)


def _write_crossing_debug_artifacts(
    *,
    debug_path: Path | None,
    debug_prefix: str,
    crossing_plan_info: dict[str, object],
) -> None:
    if debug_path is None or not crossing_plan_info.get("enabled"):
        return
    crossing_dir = debug_path / "crossings"
    _ensure_dir(crossing_dir)
    json_path = crossing_dir / f"{debug_prefix}_crossings.json"
    txt_path = crossing_dir / f"{debug_prefix}_crossings.txt"
    json_path.write_text(
        json.dumps(crossing_plan_info, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    lines = [str(crossing_plan_info.get("plan_text", "CrossingPlan: unavailable"))]
    lines.append("")
    lines.append(
        "loaded_constraints="
        f"{int(crossing_plan_info.get('constraint_count', 0))}/"
        f"{int(crossing_plan_info.get('event_count', 0))}"
    )
    lines.append(
        "realized_crossings="
        f"{int(crossing_plan_info.get('actual_crossing_count', 0))}/"
        f"{int(crossing_plan_info.get('constraint_count', 0))}"
    )
    for crossing in cast(
        list[dict[str, object]],
        crossing_plan_info.get("actual_crossings", []),
    ):
        if crossing.get("geometric"):
            lines.append(
                "  - "
                f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')}: "
                f"segment intersection at {crossing.get('point')}"
            )
            continue
        lines.append(
            "  - "
            f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')}: "
            f"{crossing.get('cell_count')} core-overlap cell(s)"
        )
    if crossing_plan_info.get("unrealized_expected_crossing_count", 0):
        lines.append("unrealized_expected:")
        for crossing in cast(
            list[dict[str, object]],
            crossing_plan_info.get("unrealized_expected_crossings", []),
        ):
            details = [f"level={crossing.get('level')}"]
            if crossing.get("unrealized_reason"):
                details.append(f"reason={crossing.get('unrealized_reason')}")
            if crossing.get("point"):
                details.append(f"point={crossing.get('point')}")
            if crossing.get("required_margin_cells") is not None:
                details.append(f"required_margin={crossing.get('required_margin_cells')}")
            if crossing.get("segment_a_margin_cells") is not None:
                details.append(f"margin_a={crossing.get('segment_a_margin_cells')}")
            if crossing.get("segment_b_margin_cells") is not None:
                details.append(f"margin_b={crossing.get('segment_b_margin_cells')}")
            lines.append(
                "  - "
                f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')} "
                + " ".join(details)
            )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


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
        realization_grid_spec=debug_artifacts.realization_grid_spec,
        allow_unchecked_bumps=not debug_artifacts.realization_allow_45_degree_turns,
        log_failures=not debug_artifacts.realization_allow_45_degree_turns,
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
    enable_crossings: bool = False,
    node_depths: dict[str, int] | None = None,
    node_ranks: dict[str, int] | None = None,
    edge_ranks: dict[str, dict[str, int]] | None = None,
    crossing_loss: float = 0.0,
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
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
        include_heater_obstacles=include_heater_obstacles,
        ripup_reroute_config=ripup_reroute_config,
        enable_crossings=enable_crossings,
        node_depths=node_depths,
        node_ranks=node_ranks,
        edge_ranks=edge_ranks,
        crossing_loss=crossing_loss,
        crossing_half_size_cells=crossing_half_size_cells,
        min_straight_cells_per_crossing=min_straight_cells_per_crossing,
        foreign_port_keepout_cells=foreign_port_keepout_cells,
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
        debug_artifacts = _apply_endpoint_corrections_to_debug_artifacts(debug_artifacts)
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
        pipeline_timings_s["meander_obstacle_map"] = (
            time.perf_counter() - t_meander_obstacle_start
        )

        meander_config = MeanderInsertionConfig(
            enabled=True,
            max_meander_height_um=float(path_length_meander_height_um),
        )
        analysis_info["meander_config"] = {
            "enabled": bool(meander_config.enabled),
            "min_candidate_straight_length_um": float(
                meander_config.min_candidate_straight_length_um
            ),
            "max_extra_length_per_region_um": float(
                meander_config.max_extra_length_per_region_um
            ),
            "conservative_legal_check": bool(meander_config.conservative_legal_check),
            "max_meander_height_um": float(meander_config.max_meander_height_um),
            "auto_meander_endpoint_inset_um": (
                None
                if meander_config.auto_meander_endpoint_inset_um is None
                else float(meander_config.auto_meander_endpoint_inset_um)
            ),
            "endpoint_inset_policy": (
                "adaptive"
                if meander_config.auto_meander_endpoint_inset_um is None
                else "fixed"
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
    materialize_cell_sets: bool | None = None,
) -> object | None:
    updates: dict[str, object] = {"obstacle_mode": obstacle_mode}
    if clear_port_open_cells_from_static is not None:
        updates["clear_port_open_cells_from_static"] = clear_port_open_cells_from_static
    if populate_obstacle_map is not None:
        updates["populate_obstacle_map"] = populate_obstacle_map
    if materialize_cell_sets is not None:
        updates["materialize_cell_sets"] = materialize_cell_sets

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
    include_heater_obstacles: bool = False,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    enable_crossings: bool = False,
    node_depths: dict[str, int] | None = None,
    node_ranks: dict[str, int] | None = None,
    edge_ranks: dict[str, dict[str, int]] | None = None,
    crossing_loss: float = 0.0,
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
    allow_only_expected_crossings: bool = True,
    defer_realization: bool = False,
    enable_checked_endpoint_correction: bool = True,
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
        foreign_port_keepout_cells: Additional global keepout distance in front
            of each endpoint port. Nets connected to the same instance can open
            that instance's keepout; unrelated nets cannot.
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
    if (
        not math.isfinite(float(proactive_congestion_weight))
        or proactive_congestion_weight < 0
    ):
        raise ValueError("proactive_congestion_weight must be finite and non-negative")
    if proactive_congestion_radius_cells < 0:
        raise ValueError("proactive_congestion_radius_cells must be non-negative")
    if crossing_loss < 0:
        raise ValueError("crossing_loss must be non-negative")
    if crossing_half_size_cells < 0:
        raise ValueError("crossing_half_size_cells must be non-negative")
    if min_straight_cells_per_crossing < 0:
        raise ValueError("min_straight_cells_per_crossing must be non-negative")
    if foreign_port_keepout_cells < 0:
        raise ValueError("foreign_port_keepout_cells must be non-negative")
    if debug_stop_after_route_index is not None and debug_stop_after_route_index < 1:
        raise ValueError("debug_stop_after_route_index must be >= 1")

    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError(
            "Rust router backend is not available. Build it with `cargo build` "
            "or `maturin develop` so photonic_router._rust can be imported."
        )

    routed_layout = unrouted_layout.copy()
    routed_layout.name = "routed_layout_rust"

    collect_pipeline_timing = (
        debug_timing or collect_route_stats or collect_attempt_diagnostics
    )
    route_nets_timings_s: dict[str, float] = {}

    def _pipeline_timer_start() -> float:
        return time.perf_counter() if collect_pipeline_timing else 0.0

    def _record_pipeline_timing(name: str, start_s: float) -> None:
        if collect_pipeline_timing:
            route_nets_timings_s[name] = route_nets_timings_s.get(name, 0.0) + (
                time.perf_counter() - start_s
            )

    t_obstacle_start = _pipeline_timer_start()
    resolved_obstacle_config = _resolve_obstacle_config(
        obstacle_config,
        route_layer=route_layer,
        include_heater_obstacles=include_heater_obstacles,
    )
    obstacle_map = build_static_obstacle_map(unrouted_layout, config=resolved_obstacle_config)
    _record_pipeline_timing("obstacle_map", t_obstacle_start)
    if debug_timing and verbose_route_diagnostics:
        print(
            "      - Obstacle Map time: "
            f"{route_nets_timings_s.get('obstacle_map', 0.0):.4f} s"
        )
    grid = obstacle_map.grid
    resolved_crossing_half_size_cells, crossing_device_info = (
        _resolve_crossing_half_size_cells(
            requested_half_size_cells=int(crossing_half_size_cells),
            enable_crossings=bool(enable_crossings),
            grid_size_um=float(grid.grid_size_um),
            clearance_um=_as_float(
                getattr(resolved_obstacle_config, "clearance_um", 0.0),
                0.0,
            ),
        )
    )

    debug_path = Path(debug_dir) if debug_dir is not None else None
    diagnostics_enabled = debug_path is not None
    obstacle_svg = None
    route_svgs: list[Path] = []

    if debug_path is not None:
        obstacle_dir = debug_path / "static_obstacles"
        _ensure_dir(obstacle_dir)
        obstacle_svg = obstacle_dir / f"{debug_prefix}_obstacles.svg"
        obstacle_map.export_debug_svg(obstacle_svg)
        route_dir = debug_path / "routes"
        if route_dir.exists():
            for old_artifact in route_dir.glob(f"{debug_prefix}_*"):
                if old_artifact.is_file() and old_artifact.suffix.lower() in {".svg", ".txt"}:
                    old_artifact.unlink()

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

    t_router_setup_start = _pipeline_timer_start()
    origin_x_um, origin_y_um = _grid_origin_xy(grid)
    grid_spec = rust_backend.GridSpec(
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        origin_x_um,
        origin_y_um,
    )
    bend_radius_cells = bend_radius_cells_from_um(
        bend_radius_um,
        grid_size_um=float(grid.grid_size_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid.grid_size_um),
        bend_radius_cells=bend_radius_cells,
        allow_45_degree_turns=allow_45_degree_turns,
    )
    bend_radius_cells = int(primitive_cfg.bend_radius_cells)
    astar_cfg = rust_backend.AStarConfig(max_iterations=int(max_iterations))
    astar_cfg.enable_simple_routes = bool(enable_simple_routes)
    astar_cfg.enable_jps4 = bool(enable_jps4)
    astar_cfg.use_indexed_heap = bool(use_indexed_heap or allow_45_degree_turns)
    astar_cfg.collect_detailed_timing = bool(
        debug_timing or collect_route_stats or collect_attempt_diagnostics
    )
    astar_cfg.primitive_ordering = str(primitive_ordering)
    effective_heuristic_mode = str(heuristic_mode)
    if allow_45_degree_turns and effective_heuristic_mode == "heading_aware":
        effective_heuristic_mode = "diagonal_aware"
    astar_cfg.heuristic_mode = effective_heuristic_mode
    if allow_45_degree_turns and hasattr(astar_cfg, "max_iterations"):
        astar_cfg.max_iterations = min(int(astar_cfg.max_iterations), 50_000)
    if allow_45_degree_turns and hasattr(astar_cfg, "heuristic_weight"):
        astar_cfg.heuristic_weight = max(float(astar_cfg.heuristic_weight), 1.25)
    if allow_45_degree_turns and hasattr(astar_cfg, "bend_weight"):
        # LiDAR heavily penalizes bends relative to propagation. Matching that
        # scale keeps 45-degree A* from spending work on short zig-zag variants.
        astar_cfg.bend_weight = max(float(astar_cfg.bend_weight), 12.0)
    effective_heap_tie_breaker = str(heap_tie_breaker)
    if allow_45_degree_turns and effective_heap_tie_breaker == "smaller_g":
        effective_heap_tie_breaker = "larger_g"
    astar_cfg.heap_tie_breaker = effective_heap_tie_breaker
    if hasattr(astar_cfg, "proactive_congestion_weight"):
        astar_cfg.proactive_congestion_weight = float(proactive_congestion_weight)
    if hasattr(astar_cfg, "proactive_congestion_radius_cells"):
        astar_cfg.proactive_congestion_radius_cells = int(proactive_congestion_radius_cells)
    if routing_window_scale is not None:
        astar_cfg.routing_window_scale = float(routing_window_scale)

    route_clearance_um = max(
        0.0,
        _as_float(getattr(resolved_obstacle_config, "clearance_um", 0.0), 0.0),
    )
    clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
        route_width_um=float(route_width_um),
        grid_size_um=float(grid.grid_size_um),
        route_clearance_um=route_clearance_um,
    )
    block_radius_cells = (
        clearance_policy.dynamic_obstacle_search_expansion_radius_cells
    )
    commit_radius_cells = clearance_policy.dynamic_route_commit_keepout_radius_cells
    core_commit_radius_cells = clearance_policy.dynamic_route_core_radius_cells
    routing_window_min_margin_cells = max(
        int(getattr(astar_cfg, "routing_window_min_margin_cells", 12)),
        int((2 * bend_radius_cells) + commit_radius_cells + 2),
    )
    astar_cfg.routing_window_min_margin_cells = max(
        int(getattr(astar_cfg, "routing_window_min_margin_cells", 12)),
        routing_window_min_margin_cells,
    )
    astar_cfg.simple_route_max_offset_cells = max(
        int(getattr(astar_cfg, "simple_route_max_offset_cells", 96)),
        int(12 * bend_radius_cells + 2 * commit_radius_cells),
    )
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)
    _record_pipeline_timing("router_setup", t_router_setup_start)

    port_entry_length_cells = max(2, bend_radius_cells + 2)
    port_entry_half_width_cells = max(1, bend_radius_cells + commit_radius_cells + 1)
    port_lane_length_cells = max(3, 2 * bend_radius_cells + 2)
    port_lane_half_width_cells = max(1, commit_radius_cells + 1)
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

    def _snap_nearly_collinear_states(
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
        grid_size = float(grid.grid_size_um)
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
            snapped_target = rust_backend.State(
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
            snapped_target = rust_backend.State(
                int(source_state.x),
                int(target_state.y),
                int(target_state.angle),
            )
            return source_state, snapped_target, original_cells

        return source_state, target_state, original_cells

    def _snap_same_heading_minimum_bend_offset(
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
        if allow_45_degree_turns:
            return source_state, target_state, extra_cells

        source_angle = int(source_state.angle) % 8
        target_angle = int(target_state.angle) % 8
        if source_angle != target_angle:
            return source_state, target_state, extra_cells

        min_offset_cells = 2 * int(bend_radius_cells)
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
            snapped_target = rust_backend.State(
                tx,
                sy + (snapped_offset_cells if dy > 0 else -snapped_offset_cells),
                target_angle,
            )
            if not _in_bounds(int(snapped_target.x), int(snapped_target.y)):
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
            snapped_target = rust_backend.State(
                sx + (snapped_offset_cells if dx > 0 else -snapped_offset_cells),
                ty,
                target_angle,
            )
            if not _in_bounds(int(snapped_target.x), int(snapped_target.y)):
                return source_state, target_state, extra_cells
            extra_cells.add((int(snapped_target.x), int(snapped_target.y)))
            return source_state, snapped_target, extra_cells

        return source_state, target_state, extra_cells

    port_open_radius_um = _as_float(
        getattr(resolved_obstacle_config, "port_open_radius_um", 0.5),
        0.5,
    )

    raw_blocked_obj: object
    if hasattr(obstacle_map, "raw_blocked_cells"):
        raw_blocked_obj = getattr(obstacle_map, "raw_blocked_cells")
    else:
        raw_blocked_obj = obstacle_map.blocked_cells
    raw_blocked_cells = cast(Iterable[tuple[int, int]], raw_blocked_obj)
    raw_static_cells = {(int(cell[0]), int(cell[1])) for cell in raw_blocked_cells}
    static_blocked_cells_before_port_reservations = raw_static_cells
    raw_static_rects_for_openings: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "raw_static_rects"):
        for rect in cast(
            Iterable[tuple[int, int, int, int]],
            getattr(obstacle_map, "raw_static_rects"),
        ):
            if len(rect) == 4:
                raw_static_rects_for_openings.append(
                    (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                )
    grid_width = int(grid.width)
    grid_height = int(grid.height)
    raw_static_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None

    def _raw_static_rect_ranges_by_y() -> dict[int, list[tuple[int, int]]]:
        nonlocal raw_static_rect_ranges_by_y
        if raw_static_rect_ranges_by_y is not None:
            return raw_static_rect_ranges_by_y
        ranges_by_y: dict[int, list[tuple[int, int]]] = {}
        for rect_min_x, rect_min_y, rect_max_x, rect_max_y in raw_static_rects_for_openings:
            min_x = max(0, rect_min_x)
            max_x = min(grid_width - 1, rect_max_x)
            min_y = max(0, rect_min_y)
            max_y = min(grid_height - 1, rect_max_y)
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
        raw_static_rect_ranges_by_y = ranges_by_y
        return ranges_by_y

    def _cell_in_raw_static(cell: tuple[int, int]) -> bool:
        if cell in raw_static_cells:
            return True
        x, y = cell
        return any(
            rect_min_x <= x <= rect_max_x
            for rect_min_x, rect_max_x in _raw_static_rect_ranges_by_y().get(y, ())
        )

    def _cells_in_raw_static_geometry(
        cells: set[tuple[int, int]],
    ) -> set[tuple[int, int]]:
        return {cell for cell in cells if _cell_in_raw_static(cell)}

    def _keyed_port_access_rule(
        *,
        instance_name: str,
        port_name: str,
        port: Port,
    ) -> tuple[float | None, float | None, str | None]:
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
            return (
                float(rule.access_length_um),
                float(rule.access_width_um),
                rule.component_name_pattern,
            )

        return None, None, None

    route_jobs: list[RouteJob] = []
    endpoint_ports_by_spec: dict[str, tuple[str, str, Port]] = {}
    port_access_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_runway_cells_by_spec: dict[str, set[tuple[int, int]]] = {}
    port_access_rule_by_spec: dict[str, str | None] = {}
    next_net_id = 1
    t_route_job_build_start = _pipeline_timer_start()
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
            endpoint_ports_by_spec.setdefault(port1_spec, (inst1, port1, source_port))
            endpoint_ports_by_spec.setdefault(port2_spec, (inst2, port2, target_port))
    _record_pipeline_timing("route_job_build", t_route_job_build_start)

    t_crossing_context_start = _pipeline_timer_start()
    crossing_plan_info = _build_crossing_plan_info(
        rust_backend=rust_backend,
        router=router,
        schematic=schematic,
        route_jobs=route_jobs,
        enable_crossings=enable_crossings,
        node_depths=node_depths,
        node_ranks=node_ranks,
        edge_ranks=edge_ranks,
        crossing_loss=float(crossing_loss),
        crossing_half_size_cells=int(resolved_crossing_half_size_cells),
        min_straight_cells_per_crossing=int(min_straight_cells_per_crossing),
        allow_only_expected_crossings=bool(allow_only_expected_crossings),
    )
    crossing_plan_info["crossing_device"] = crossing_device_info
    _record_pipeline_timing("crossing_context", t_crossing_context_start)

    if not hasattr(router, "build_route_port_openings"):
        extension_path = getattr(rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.build_route_port_openings. Rebuild it with "
            "`maturin develop --release`. "
            f"Loaded extension: {extension_path}"
        )

    t_port_opening_prep_start = _pipeline_timer_start()
    port_opening_inputs: list[
        tuple[str, float, float, float | None, str | None, float | None, float | None]
    ] = []
    for port_spec, (instance_name, port_name, port) in endpoint_ports_by_spec.items():
        center = _port_center_um(port)
        if center is None:
            raise ValueError(f"Port {port_spec!r} has no finite center coordinate")
        orientation_value = getattr(port, "orientation", None)
        orientation = None if orientation_value is None else float(orientation_value)
        port_type = _port_type_name(port)
        access_length_um, access_width_um, rule_name = _keyed_port_access_rule(
            instance_name=instance_name,
            port_name=port_name,
            port=port,
        )
        port_access_rule_by_spec[port_spec] = rule_name
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
    _record_pipeline_timing("port_opening_prep", t_port_opening_prep_start)

    raw_static_cells_for_openings = sorted(raw_static_cells)
    t_port_opening_batch_start = _pipeline_timer_start()
    for port_spec, cells, candidate_cells, runway_cells in router.build_route_port_openings(
        port_opening_inputs,
        raw_static_cells=raw_static_cells_for_openings,
        raw_static_rects=raw_static_rects_for_openings,
        route_clearance_um=float(route_clearance_um),
        port_open_radius_um=float(port_open_radius_um),
        bend_radius_cells=int(bend_radius_cells),
        commit_radius_cells=int(commit_radius_cells),
        port_entry_length_cells=int(port_entry_length_cells),
        port_entry_half_width_cells=int(port_entry_half_width_cells),
        port_lane_length_cells=int(port_lane_length_cells),
        port_lane_half_width_cells=int(port_lane_half_width_cells),
    ):
        port_access_cells_by_spec[str(port_spec)] = {
            (int(cell[0]), int(cell[1])) for cell in cells
        }
        port_access_candidate_cells_by_spec[str(port_spec)] = {
            (int(cell[0]), int(cell[1])) for cell in candidate_cells
        }
        port_runway_cells_by_spec[str(port_spec)] = {
            (int(cell[0]), int(cell[1])) for cell in runway_cells
        }
    _record_pipeline_timing("port_opening_batch", t_port_opening_batch_start)

    foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]] = {}
    if foreign_port_keepout_cells > 0:
        t_foreign_keepout_start = _pipeline_timer_start()
        foreign_length_cells = int(foreign_port_keepout_cells)
        foreign_half_width_cells = int(foreign_port_keepout_cells)
        for port_spec, _cells, _candidate_cells, runway_cells in router.build_route_port_openings(
            port_opening_inputs,
            raw_static_cells=raw_static_cells_for_openings,
            raw_static_rects=raw_static_rects_for_openings,
            route_clearance_um=float(route_clearance_um),
            port_open_radius_um=float(port_open_radius_um),
            bend_radius_cells=max(0, foreign_length_cells - 1),
            commit_radius_cells=foreign_half_width_cells,
            port_entry_length_cells=int(port_entry_length_cells),
            port_entry_half_width_cells=int(port_entry_half_width_cells),
            port_lane_length_cells=int(port_lane_length_cells),
            port_lane_half_width_cells=int(port_lane_half_width_cells),
        ):
            instance_name = str(port_spec).split(",", 1)[0]
            foreign_port_keepout_cells_by_instance.setdefault(instance_name, set()).update(
                (int(cell[0]), int(cell[1])) for cell in runway_cells
            )
        _record_pipeline_timing("foreign_port_keepout_batch", t_foreign_keepout_start)

    def _endpoint_state_for_lane_assignment(port: Port, *, as_target: bool):
        return port_to_grid_state(
            port,
            origin_x_um,
            origin_y_um,
            float(grid.grid_size_um),
            as_target=as_target,
        )

    endpoint_ports_by_key: dict[tuple[int, int, int], list[tuple[str, bool, Port]]] = {}
    for job in route_jobs:
        for port_spec, port, as_target in (
            (f"{job.inst1},{job.port1}", job.source_port, False),
            (f"{job.inst2},{job.port2}", job.target_port, True),
        ):
            state = _endpoint_state_for_lane_assignment(port, as_target=as_target)
            key = (int(state.x), int(state.y), int(state.angle) % 8)
            endpoint_ports_by_key.setdefault(key, []).append((port_spec, as_target, port))

    port_state_lane_offsets: dict[tuple[str, bool], tuple[int, int]] = {}
    for (base_x, base_y, angle), endpoints in endpoint_ports_by_key.items():
        unique_endpoints = list(dict.fromkeys((spec, is_target) for spec, is_target, _ in endpoints))
        if len(unique_endpoints) <= 1:
            continue
        step_x, step_y = _angle_to_step(angle)
        lateral_x, lateral_y = -step_y, step_x
        if lateral_x == 0 and lateral_y == 0:
            continue

        def lateral_position(item: tuple[str, bool, Port]) -> float:
            center = _port_center_um(item[2])
            if center is None:
                return 0.0
            return center[0] * lateral_x + center[1] * lateral_y

        sorted_endpoints = sorted(endpoints, key=lateral_position)
        seen_endpoint_keys: set[tuple[str, bool]] = set()
        lane_index = 0
        for port_spec, is_target, _ in sorted_endpoints:
            endpoint_key = (port_spec, is_target)
            if endpoint_key in seen_endpoint_keys:
                continue
            seen_endpoint_keys.add(endpoint_key)
            candidate_x = base_x + lateral_x * lane_index
            candidate_y = base_y + lateral_y * lane_index
            if _in_bounds(candidate_x, candidate_y):
                port_state_lane_offsets[endpoint_key] = (
                    lateral_x * lane_index,
                    lateral_y * lane_index,
                )
            lane_index += 1

    port_runway_static_cells: set[tuple[int, int]] = set()
    for cells in port_runway_cells_by_spec.values():
        port_runway_static_cells.update(cells)
    foreign_port_keepout_static_cells: set[tuple[int, int]] = set()
    for cells in foreign_port_keepout_cells_by_instance.values():
        foreign_port_keepout_static_cells.update(cells)
    debug_port_keepout_cells = set(port_runway_static_cells)
    debug_port_keepout_cells.update(foreign_port_keepout_static_cells)
    static_blocked_cells_before_port_reservations = set(raw_static_cells)
    static_blocked_cells_before_port_reservations.update(port_runway_static_cells)
    static_blocked_cells_before_port_reservations.update(foreign_port_keepout_static_cells)

    t_static_handoff_start = _pipeline_timer_start()
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
    if port_runway_static_cells or foreign_port_keepout_static_cells:
        if not hasattr(router, "add_static_cells"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.add_static_cells. Rebuild it with "
                "`maturin develop --release`."
            )
        router.add_static_cells(
            sorted(port_runway_static_cells | foreign_port_keepout_static_cells)
        )
    _record_pipeline_timing("static_map_handoff", t_static_handoff_start)

    full_route_jobs = list(route_jobs)
    full_route_jobs_by_route_index = {int(job.route_index): job for job in full_route_jobs}
    full_route_count = len(full_route_jobs)
    if (
        debug_stop_after_route_index is not None
        and int(debug_stop_after_route_index) > full_route_count
    ):
        raise ValueError(
            "debug_stop_after_route_index exceeds route count "
            f"({debug_stop_after_route_index} > {full_route_count})"
        )
    if debug_stop_after_route_index is not None:
        stop_index = int(debug_stop_after_route_index)
        route_jobs = [job for job in full_route_jobs if int(job.route_index) <= stop_index]
        if verbose_route_diagnostics or debug_route_indices is not None:
            print(
                f"  Debug stop-after-route active: routing {len(route_jobs)} "
                f"of {full_route_count} full-context routes"
            )

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
            "preemptive_crossing_ripup",
            "repair_failed_net",
            "reroute_victims",
            "endpoint_correction",
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

    def _record_native_batch_timings(batch_result: dict[str, Any]) -> None:
        if not collect_pipeline_timing:
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
            route_nets_timings_s[name] = route_nets_timings_s.get(name, 0.0) + elapsed_s

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
        source_lane_offset = port_state_lane_offsets.get((f"{job.inst1},{job.port1}", False))
        if source_lane_offset is not None:
            source_state = rust_backend.State(
                int(source_state.x) + int(source_lane_offset[0]),
                int(source_state.y) + int(source_lane_offset[1]),
                int(source_state.angle),
            )
        target_lane_offset = port_state_lane_offsets.get((f"{job.inst2},{job.port2}", True))
        if target_lane_offset is not None:
            target_state = rust_backend.State(
                int(target_state.x) + int(target_lane_offset[0]),
                int(target_state.y) + int(target_lane_offset[1]),
                int(target_state.angle),
            )
        source_state, target_state, original_anchor_cells = _snap_nearly_collinear_states(
            source_state,
            target_state,
            job.source_port,
            job.target_port,
        )
        source_state, target_state, snapped_anchor_cells = (
            _snap_same_heading_minimum_bend_offset(source_state, target_state)
        )
        original_anchor_cells.update(snapped_anchor_cells)
        port1_spec = f"{job.inst1},{job.port1}"
        port2_spec = f"{job.inst2},{job.port2}"
        source_anchor_cell = (int(source_state.x), int(source_state.y))
        target_anchor_cell = (int(target_state.x), int(target_state.y))
        instance_keepout_open_cells = set(
            foreign_port_keepout_cells_by_instance.get(job.inst1, set())
        )
        instance_keepout_open_cells.update(
            foreign_port_keepout_cells_by_instance.get(job.inst2, set())
        )
        opened_candidate_cells = set(port_access_candidate_cells_by_spec.get(port1_spec, set()))
        opened_candidate_cells.update(port_access_candidate_cells_by_spec.get(port2_spec, set()))
        opened_candidate_cells.update(instance_keepout_open_cells)
        opened_candidate_cells.update(original_anchor_cells)
        opened_candidate_cells.update({source_anchor_cell, target_anchor_cell})

        opened_cells_set = set(port_access_cells_by_spec.get(port1_spec, set()))
        opened_cells_set.update(port_access_cells_by_spec.get(port2_spec, set()))
        opened_cells_set.update(instance_keepout_open_cells)
        opened_cells_set.update(original_anchor_cells)
        opened_cells_set.update({source_anchor_cell, target_anchor_cell})
        return (
            source_state,
            target_state,
            opened_candidate_cells,
            opened_cells_set,
            sorted(opened_cells_set),
        )

    if not hasattr(router, "build_dynamic_clearance_exempt_cells_for_routes"):
        extension_path = getattr(rust_backend, "__file__", "<unknown>")
        raise RuntimeError(
            "The loaded photonic_router._rust extension does not expose "
            "PyPhotonicRouter.build_dynamic_clearance_exempt_cells_for_routes. "
            "Rebuild it with `maturin develop --release`. "
            f"Loaded extension: {extension_path}"
        )

    t_state_opening_precompute_start = _pipeline_timer_start()
    route_state_openings_by_id = {
        int(job.net_id): _states_and_openings(job)
        for job in route_jobs
    }
    _record_pipeline_timing(
        "state_opening_precompute",
        t_state_opening_precompute_start,
    )
    clearance_exempt_inputs = [
        (int(net_id), state_openings[0], state_openings[1])
        for net_id, state_openings in route_state_openings_by_id.items()
    ]
    t_clearance_exempt_batch_start = _pipeline_timer_start()
    batch_clearance_exempt_cells_by_id = {
        int(net_id): [(int(cell[0]), int(cell[1])) for cell in cells]
        for net_id, cells in router.build_dynamic_clearance_exempt_cells_for_routes(
            clearance_exempt_inputs,
            bool(allow_45_degree_turns),
            int(bend_radius_cells),
            int(commit_radius_cells),
        )
    }
    _record_pipeline_timing(
        "clearance_exempt_batch",
        t_clearance_exempt_batch_start,
    )

    def _state_openings_for_job(
        job: RouteJob,
    ) -> tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]:
        return route_state_openings_by_id[int(job.net_id)]

    def _clearance_exempt_cells_for_job(job: RouteJob) -> list[tuple[int, int]]:
        return batch_clearance_exempt_cells_by_id.get(int(job.net_id), [])

    def _clearance_exempt_cell_set_for_job(job: RouteJob) -> set[tuple[int, int]]:
        return set(_clearance_exempt_cells_for_job(job))

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
        source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
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
        raw_dynamic_cells: set[tuple[int, int]] = set()
        raw_dynamic_refcount_gt1_cells: set[tuple[int, int]] = set()
        raw_core_cells: set[tuple[int, int]] = set()
        raw_core_refcount_gt1_cells: set[tuple[int, int]] = set()
        raw_net_route_cells: set[tuple[int, int]] = set()
        if hasattr(router, "raw_dynamic_obstacle_cells"):
            raw_dynamic_entries = [
                (int(x), int(y), int(refs))
                for x, y, refs in router.raw_dynamic_obstacle_cells()
            ]
            raw_dynamic_cells = {(x, y) for x, y, _ in raw_dynamic_entries}
            raw_dynamic_refcount_gt1_cells = {
                (x, y) for x, y, refs in raw_dynamic_entries if refs > 1
            }
        if hasattr(router, "raw_dynamic_core_cells"):
            raw_core_entries = [
                (int(x), int(y), int(refs))
                for x, y, refs in router.raw_dynamic_core_cells()
            ]
            raw_core_cells = {(x, y) for x, y, _ in raw_core_entries}
            raw_core_refcount_gt1_cells = {
                (x, y) for x, y, refs in raw_core_entries if refs > 1
            }
        if hasattr(router, "all_net_route_cells"):
            for _, cells in router.all_net_route_cells():
                raw_net_route_cells.update(
                    (int(cell[0]), int(cell[1])) for cell in cells
                )
        raw_dynamic_without_owner = raw_dynamic_cells - raw_net_route_cells
        raw_net_route_without_dynamic = raw_net_route_cells - raw_dynamic_cells
        raw_core_without_dynamic = raw_core_cells - raw_dynamic_cells
        raw_dynamic_span_cells = {
            (x, y)
            for x, y in raw_dynamic_cells
            if span_bbox_min_x <= x <= span_bbox_max_x
            and span_bbox_min_y <= y <= span_bbox_max_y
        }
        raw_dynamic_without_owner_span_cells = {
            (x, y)
            for x, y in raw_dynamic_without_owner
            if span_bbox_min_x <= x <= span_bbox_max_x
            and span_bbox_min_y <= y <= span_bbox_max_y
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
            "block_radius_cells": block_radius_cells,
            "dynamic_obstacle_search_expansion_radius_cells": (
                clearance_policy.dynamic_obstacle_search_expansion_radius_cells
            ),
            "dynamic_route_commit_keepout_radius_cells": (
                clearance_policy.dynamic_route_commit_keepout_radius_cells
            ),
            "dynamic_route_core_radius_cells": (
                clearance_policy.dynamic_route_core_radius_cells
            ),
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
        committed_dynamic_cells = _committed_dynamic_cells(exclude_net_id=job.net_id)
        if diagnostics_enabled:
            opened_candidate_dynamic_overlap = opened_candidate_cells & committed_dynamic_cells
            opened_candidate_static_overlap = _cells_in_raw_static_geometry(
                opened_candidate_cells
            )
            opened_static_overlap = _cells_in_raw_static_geometry(opened_cells_set)
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
        route_static_overlap = _cells_in_raw_static_geometry(route_cells)
        route_overlap_with_candidate_opened_static = (
            route_cells & opened_candidate_static_overlap
        )
        route_overlap_with_effective_opened_static = route_cells & opened_static_overlap
        route_dynamic_overlap = route_cells & committed_dynamic_cells
        route_overlap_with_candidate_opened_dynamic = (
            route_cells & opened_candidate_dynamic_overlap
        )
        route_overlap_with_effective_opened_dynamic = route_cells & opened_dynamic_overlap
        route_overlap_with_dynamic_exempt = route_cells & dynamic_clearance_exempt_cells
        foreign_keepout_open_cells = set(
            foreign_port_keepout_cells_by_instance.get(job.inst1, set())
        )
        foreign_keepout_open_cells.update(
            foreign_port_keepout_cells_by_instance.get(job.inst2, set())
        )
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
        lines = [
            f"net_name={job.net_name}",
            f"status={status}",
            f"source_spec={port1_spec}",
            f"target_spec={port2_spec}",
            f"source_component={_schematic_instance_component_name(schematic, job.inst1)}",
            f"target_component={_schematic_instance_component_name(schematic, job.inst2)}",
            f"source_access_rule={port_access_rule_by_spec.get(port1_spec)}",
            f"target_access_rule={port_access_rule_by_spec.get(port2_spec)}",
            f"foreign_port_keepout_cells={int(foreign_port_keepout_cells)}",
            f"foreign_port_keepout_static_count={len(foreign_port_keepout_static_cells)}",
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
            f"dynamic_clearance_exempt_cells_count={len(dynamic_clearance_exempt_cells)}",
            f"dynamic_clearance_exempt_cells_bbox={_cells_bbox(dynamic_clearance_exempt_cells)}",
            f"dynamic_clearance_exempt_dynamic_overlap_count={len(dynamic_exempt_dynamic_overlap)}",
            f"dynamic_clearance_exempt_dynamic_overlap_bbox={_cells_bbox(dynamic_exempt_dynamic_overlap)}",
            f"route_cells_count={len(route_cells)}",
            f"route_static_blocked_overlap_count={len(route_static_overlap)}",
            f"route_static_blocked_overlap_bbox={_cells_bbox(route_static_overlap)}",
            f"route_dynamic_overlap_count={len(route_dynamic_overlap)}",
            f"route_dynamic_overlap_bbox={_cells_bbox(route_dynamic_overlap)}",
            f"route_overlap_candidate_opened_static_count={len(route_overlap_with_candidate_opened_static)}",
            f"route_overlap_effective_opened_static_count={len(route_overlap_with_effective_opened_static)}",
            f"route_overlap_candidate_opened_dynamic_count={len(route_overlap_with_candidate_opened_dynamic)}",
            f"route_overlap_effective_opened_dynamic_count={len(route_overlap_with_effective_opened_dynamic)}",
            f"route_overlap_dynamic_clearance_exempt_count={len(route_overlap_with_dynamic_exempt)}",
        ]
        if route_segments:
            lines.append("route_segments=" + "; ".join(route_segments))
        if repair_note is not None:
            lines.append(f"repair={repair_note}")
        if error_text is not None:
            lines.append(f"error={error_text}")
        diag_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _record_route(
        job: RouteJob,
        route_obj: Any,
        opened_cells: list[tuple[int, int]],
        *,
        corrected_centerline_um: tuple[tuple[float, float], ...] = (),
        corrected_total_length_um: float | None = None,
    ) -> None:
        route_bookkeeping.record_route(
            job,
            route_obj,
            opened_cells,
            route_cells=_route_cells_from_router(job.net_id) if track_dynamic_cells else None,
            corrected_centerline_um=corrected_centerline_um,
            corrected_total_length_um=corrected_total_length_um,
        )

    def _export_route_svg(
        job: RouteJob,
        route_obj: Any,
        *,
        suffix: str = "",
        obstacle_cells: set[tuple[int, int]] | None = None,
        opened_cells: list[tuple[int, int]] | None = None,
    ) -> None:
        should_export = (
            debug_path is not None
            and (debug_route_indices is None or job.route_index in debug_route_indices)
        )
        if not should_export:
            return
        route_dir = debug_path / "routes"
        _ensure_dir(route_dir)
        route_svg = route_dir / f"{debug_prefix}_{job.net_name}{suffix}.svg"
        if obstacle_cells is not None and hasattr(
            router, "export_debug_svg_with_obstacle_cells"
        ):
            svg_text = router.export_debug_svg_with_obstacle_cells(
                route_obj,
                sorted(obstacle_cells),
            )
        else:
            svg_text = router.export_debug_svg(route_obj)
        if opened_cells is None:
            try:
                _, _, _, _, opened_cells = _state_openings_for_job(job)
            except Exception:
                opened_cells = []
        if (
            debug_stop_after_route_index is not None
            and int(job.route_index) == int(debug_stop_after_route_index)
        ):
            next_job = full_route_jobs_by_route_index.get(
                int(debug_stop_after_route_index) + 1
            )
            if next_job is not None:
                try:
                    _, _, _, _, opened_cells = _states_and_openings(next_job)
                except Exception:
                    pass
        red_keepout_cells = debug_port_keepout_cells - {
            (int(cell[0]), int(cell[1])) for cell in opened_cells
        }
        if red_keepout_cells:
            overlay = ['<g id="port-keepout-cells">']
            for gx, gy in sorted(red_keepout_cells):
                if 0 <= gx < grid_width and 0 <= gy < grid_height:
                    svg_y = grid_height - gy - 1
                    overlay.append(
                        f'<rect class="port-keepout" x="{gx}" y="{svg_y}" '
                        'width="1" height="1" fill="#d93025" opacity="0.38" />'
                    )
            overlay.append("</g>")
            overlay_text = "".join(overlay)
            if "</svg>" in svg_text and 'id="port-keepout-cells"' not in svg_text:
                svg_text = svg_text.replace("</svg>", overlay_text + "</svg>", 1)
        route_svg.write_text(svg_text, encoding="utf-8")
        route_svgs.append(route_svg)

    def _route_engine_summary(route_obj: Any) -> str:
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
            "dynamic_obstacle_search_expansion_radius_cells="
            f"{clearance_policy.dynamic_obstacle_search_expansion_radius_cells}",
            "dynamic_route_commit_keepout_radius_cells="
            f"{clearance_policy.dynamic_route_commit_keepout_radius_cells}",
            "dynamic_route_core_radius_cells="
            f"{clearance_policy.dynamic_route_core_radius_cells}",
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
        current_attempts = [
            ("current", record)
            for record in route_attempt_records
            if getattr(record, "net_id", None) == job.net_id
        ]
        recent_attempts = [("recent", record) for record in route_attempt_records[-12:]]
        for label, attempt in (current_attempts[-8:] + recent_attempts):
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
        job: RouteJob,
        route_obj: Any,
        opened_cells: list[tuple[int, int]],
        *,
        should_print_route: bool,
        diag_txt: Path | None,
        debug_obstacle_cells: set[tuple[int, int]] | None = None,
    ) -> None:
        nonlocal total_expanded_states, simple_route_count
        expanded_states = int(getattr(route_obj, "expanded_states", 0))
        total_expanded_states += expanded_states
        if expanded_states == 0:
            simple_route_count += 1

        if diagnostics_enabled:
            source_state, target_state, opened_candidate_cells, opened_cells_set, _ = (
                _state_openings_for_job(job)
            )
            route_cells = {
                (int(cell[0]), int(cell[1]))
                for cell in (getattr(route_obj, "cells", None) or [])
            }
            _write_route_diagnostics(
                job=job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(job),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="ok",
                route_cells=route_cells,
                route_obj=route_obj,
            )

        _export_route_svg(
            job,
            route_obj,
            obstacle_cells=debug_obstacle_cells,
            opened_cells=opened_cells,
        )

        if should_print_route:
            print(f"ok {_route_engine_summary(route_obj)}")

    if repair_config.enabled:
        if not hasattr(router, "route_many_with_repair_and_commit"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.route_many_with_repair_and_commit. Rebuild it with "
                "`maturin develop --release`; Python repair fallback has been removed."
            )
        batch_jobs: list[
            tuple[int, Any, Any, list[tuple[int, int]], list[tuple[int, int]]]
        ] = []
        batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
        batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
        t_batch_job_pack_start = _pipeline_timer_start()
        for job in route_jobs:
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            route_selected_for_debug = (
                debug_route_indices is None or job.route_index in debug_route_indices
            )
            should_print_route = verbose_route_diagnostics and route_selected_for_debug
            if debug_route_indices is not None and route_selected_for_debug:
                should_print_route = True
            if should_print_route:
                print(
                    f"  Routing [{job.route_index}/{len(route_jobs)}] "
                    f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                    end=" ",
                )
            route_dir = debug_path / "routes" if debug_path is not None else None
            diag_txt: Path | None = None
            if debug_path is not None and route_selected_for_debug and route_dir is not None:
                _ensure_dir(route_dir)
                diag_txt = route_dir / f"{debug_prefix}_{job.net_name}_diagnostics.txt"
            batch_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                )
            )
            batch_opened_cells_by_id[int(job.net_id)] = opened_cells
            batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
        _record_pipeline_timing("batch_job_pack", t_batch_job_pack_start)

        batch_start = _timing_start()
        raw_batch_result = router.route_many_with_repair_and_commit(
            batch_jobs,
            block_radius_cells,
            commit_radius_cells,
            core_commit_radius_cells,
            int(repair_config.max_rounds),
            int(repair_config.max_victims_per_failure),
            float(repair_config.history_weight),
            int(repair_config.history_increment),
        )
        batch_elapsed_s = time.perf_counter() - batch_start if collect_timing else 0.0
        _record_pipeline_timing("native_route_batch", batch_start)
        t_batch_result_processing_start = _pipeline_timer_start()
        batch_result = dict(raw_batch_result)
        _record_native_batch_timings(batch_result)
        raw_attempts = list(cast(Iterable[Any], batch_result.get("attempts", [])))
        per_attempt_elapsed_s = batch_elapsed_s / max(1, len(raw_attempts))
        for raw_attempt in raw_attempts:
            attempt = dict(raw_attempt)
            net_id = int(attempt["net_id"])
            job = route_jobs_by_id[net_id]
            route_obj = attempt.get("route")
            if route_obj is None:
                route_obj = None
            failed = bool(attempt.get("failed", False))
            bucket_name = str(attempt.get("bucket_name", "normal_route"))
            error_text = (
                str(attempt.get("error"))
                if attempt.get("error") is not None
                else None
            )
            repair_round_raw = attempt.get("repair_round")
            repair_round = (
                int(repair_round_raw)
                if repair_round_raw is not None
                else None
            )
            attempt_index = len(route_attempt_records) + 1
            candidate_blockers = [
                int(value)
                for value in cast(list[object], attempt.get("candidate_blockers", []))
            ]
            ripup_ids = [
                int(value)
                for value in cast(list[object], attempt.get("ripup_ids", []))
            ]
            if (
                route_obj is not None
                and not failed
                and bucket_name != "normal_route"
                and debug_path is not None
                and (debug_route_indices is None or job.route_index in debug_route_indices)
            ):
                _export_route_svg(
                    job,
                    route_obj,
                    suffix=f"_attempt{attempt_index}_{bucket_name}",
                )
            if collect_timing:
                if route_obj is not None and not failed:
                    route_timing_buckets[bucket_name].record_route(
                        per_attempt_elapsed_s,
                        route_obj,
                    )
                else:
                    route_timing_buckets[bucket_name].record_elapsed(
                        per_attempt_elapsed_s,
                        failed=failed,
                    )
                route_attempt_records.append(
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
                        diagnostics=_route_attempt_diagnostics(
                            job,
                            route_obj if route_obj is not None and not failed else None,
                            candidate_blockers=candidate_blockers,
                            ripup_ids=ripup_ids,
                        )
                        if collect_attempt_diagnostics
                        else None,
                    )
                )

        repair_count += int(batch_result.get("repair_count", 0) or 0)
        raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
        for raw_entry in raw_routes:
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            route_obj = entry["route"]
            job = route_jobs_by_id[net_id]
            opened_cells = batch_opened_cells_by_id[net_id]
            _record_route(job, route_obj, opened_cells)
            should_print_route, diag_txt = batch_debug_by_id[net_id]
            _finalize_committed_route(
                job,
                route_obj,
                opened_cells,
                should_print_route=should_print_route,
                diag_txt=diag_txt,
            )
        _record_pipeline_timing(
            "batch_result_processing",
            t_batch_result_processing_start,
        )

        if str(batch_result.get("status", "")) != "routed":
            failed_net_id = int(batch_result.get("failed_net_id", -1))
            failed_job = route_jobs_by_id.get(failed_net_id)
            error_text = str(batch_result.get("error", "No route found"))
            if failed_job is None:
                raise RuntimeError(error_text)
            source_state, target_state, opened_candidate_cells, opened_cells_set, opened_cells = (
                _state_openings_for_job(failed_job)
            )
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
            _write_route_diagnostics(
                job=failed_job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(failed_job),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="failed",
                error_text=error_text,
            )
            _write_failed_log(
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
                f"allow_45_degree_turns={allow_45_degree_turns}"
            )

    else:
        if not hasattr(router, "route_many_normal_and_commit"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.route_many_normal_and_commit. Rebuild it with "
                "`maturin develop --release`; Python sequential routing fallback has been removed."
            )
        batch_jobs: list[
            tuple[int, Any, Any, list[tuple[int, int]], list[tuple[int, int]]]
        ] = []
        batch_opened_cells_by_id: dict[int, list[tuple[int, int]]] = {}
        batch_debug_by_id: dict[int, tuple[bool, Path | None]] = {}
        t_batch_job_pack_start = _pipeline_timer_start()
        for job in route_jobs:
            source_state, target_state, _, _, opened_cells = _state_openings_for_job(job)
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
            route_selected_for_debug = (
                debug_route_indices is None or job.route_index in debug_route_indices
            )
            should_print_route = verbose_route_diagnostics and route_selected_for_debug
            if debug_route_indices is not None and route_selected_for_debug:
                should_print_route = True
            if should_print_route:
                print(
                    f"  Routing [{job.route_index}/{len(route_jobs)}] "
                    f"{job.net_name}: {job.inst1},{job.port1} -> {job.inst2},{job.port2}...",
                    end=" ",
                )
            route_dir = debug_path / "routes" if debug_path is not None else None
            diag_txt: Path | None = None
            if debug_path is not None and route_selected_for_debug and route_dir is not None:
                _ensure_dir(route_dir)
                diag_txt = route_dir / f"{debug_prefix}_{job.net_name}_diagnostics.txt"
            batch_jobs.append(
                (
                    int(job.net_id),
                    source_state,
                    target_state,
                    opened_cells,
                    clearance_exempt_cells,
                )
            )
            batch_opened_cells_by_id[int(job.net_id)] = opened_cells
            batch_debug_by_id[int(job.net_id)] = (should_print_route, diag_txt)
        _record_pipeline_timing("batch_job_pack", t_batch_job_pack_start)

        batch_start = _timing_start()
        raw_batch_result = router.route_many_normal_and_commit(
            batch_jobs,
            block_radius_cells,
            commit_radius_cells,
            core_commit_radius_cells,
        )
        batch_elapsed_s = time.perf_counter() - batch_start if collect_timing else 0.0
        _record_pipeline_timing("native_route_batch", batch_start)
        t_batch_result_processing_start = _pipeline_timer_start()
        batch_result = dict(raw_batch_result)
        _record_native_batch_timings(batch_result)
        raw_routes = list(cast(Iterable[Any], batch_result.get("routes", [])))
        per_route_elapsed_s = batch_elapsed_s / max(1, len(raw_routes))
        for raw_entry in raw_routes:
            entry = dict(raw_entry)
            net_id = int(entry["net_id"])
            route_obj = entry["route"]
            job = route_jobs_by_id[net_id]
            opened_cells = batch_opened_cells_by_id[net_id]
            if collect_timing:
                route_timing_buckets["normal_route"].record_route(
                    per_route_elapsed_s,
                    route_obj,
                )
                route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=len(route_attempt_records) + 1,
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
            _record_route(job, route_obj, opened_cells)
            should_print_route, diag_txt = batch_debug_by_id[net_id]
            _finalize_committed_route(
                job,
                route_obj,
                opened_cells,
                should_print_route=should_print_route,
                diag_txt=diag_txt,
            )
        _record_pipeline_timing(
            "batch_result_processing",
            t_batch_result_processing_start,
        )

        if str(batch_result.get("status", "")) != "routed":
            failed_net_id = int(batch_result.get("failed_net_id", -1))
            failed_job = route_jobs_by_id.get(failed_net_id)
            error_text = str(batch_result.get("error", "No route found"))
            if failed_job is None:
                raise RuntimeError(error_text)
            source_state, target_state, opened_candidate_cells, opened_cells_set, opened_cells = (
                _state_openings_for_job(failed_job)
            )
            should_print_route, diag_txt = batch_debug_by_id.get(
                failed_net_id,
                (False, None),
            )
            if collect_timing:
                route_timing_buckets["normal_route"].record_elapsed(0.0, failed=True)
                route_attempt_records.append(
                    route_attempt_record_from_route(
                        attempt_index=len(route_attempt_records) + 1,
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
            _write_route_diagnostics(
                job=failed_job,
                source_state=source_state,
                target_state=target_state,
                opened_candidate_cells=opened_candidate_cells,
                dynamic_clearance_exempt_cells=_clearance_exempt_cell_set_for_job(failed_job),
                opened_cells_set=opened_cells_set,
                diag_txt=diag_txt,
                status="failed",
                error_text=error_text,
            )
            _write_failed_log(
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
                f"allow_45_degree_turns={allow_45_degree_turns}"
            )

    astar_elapsed_s = 0.0
    if collect_timing:
        astar_elapsed_s = time.perf_counter() - t_astar_start

    if enable_checked_endpoint_correction:
        if not hasattr(router, "apply_checked_endpoint_corrections_and_commit"):
            raise RuntimeError(
                "The loaded photonic_router._rust extension does not expose "
                "PyPhotonicRouter.apply_checked_endpoint_corrections_and_commit. "
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
        t_endpoint_correction_pack_start = _pipeline_timer_start()
        for net_id in list(route_bookkeeping.route_order):
            record = route_bookkeeping.records_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if record is None or job is None:
                continue
            source_port = _port_center_um(job.source_port)
            target_port = _port_center_um(job.target_port)
            if source_port is None and target_port is None:
                continue
            source_state, target_state, opened_candidate_cells, _, _ = (
                _state_openings_for_job(job)
            )
            clearance_exempt_cells = _clearance_exempt_cells_for_job(job)
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
        _record_pipeline_timing(
            "endpoint_correction_pack",
            t_endpoint_correction_pack_start,
        )

        correction_start = _timing_start()
        raw_corrections = router.apply_checked_endpoint_corrections_and_commit(
            correction_jobs,
            float(route_width_um),
            int(commit_radius_cells),
            int(core_commit_radius_cells),
            not allow_45_degree_turns,
        )
        correction_elapsed_s = (
            time.perf_counter() - correction_start if collect_timing else 0.0
        )
        _record_pipeline_timing("endpoint_correction_native", correction_start)
        t_endpoint_correction_processing_start = _pipeline_timer_start()
        correction_elapsed_per_job_s = correction_elapsed_s / max(1, len(correction_jobs))
        for raw_correction in cast(Iterable[Any], raw_corrections):
            correction = dict(raw_correction)
            net_id = int(correction["net_id"])
            record = route_bookkeeping.records_by_id.get(net_id)
            job = route_jobs_by_id.get(net_id)
            if record is None or job is None:
                continue
            error = correction.get("error")
            if error is not None:
                if collect_timing:
                    route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked grid-to-port endpoint correction skipped for net "
                    f"{job.net_name!r}: {error}"
                )
                print("WARNING: " + message)
                route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    endpoint_correction_error=message,
                )
                continue
            centerline = _centerline_tuple(correction.get("centerline"))
            if not centerline:
                if collect_timing:
                    route_timing_buckets["endpoint_correction"].record_elapsed(
                        correction_elapsed_per_job_s,
                        failed=True,
                    )
                message = (
                    "Checked grid-to-port endpoint correction skipped for net "
                    f"{job.net_name!r}: endpoint correction returned an invalid centerline"
                )
                print("WARNING: " + message)
                route_bookkeeping.records_by_id[net_id] = replace(
                    record,
                    endpoint_correction_error=message,
                )
                continue
            if collect_timing:
                route_timing_buckets["endpoint_correction"].record_elapsed(
                    correction_elapsed_per_job_s,
                )
            corrected_total_length_um = float(correction["total_length_um"])
            route_bookkeeping.records_by_id[net_id] = replace(
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
        _record_pipeline_timing(
            "endpoint_correction_processing",
            t_endpoint_correction_processing_start,
        )

    t_record_assembly_start = _pipeline_timer_start()
    routed_net_records = route_bookkeeping.ordered_records()
    routed_record_keys = [
        (record.net_name, record.source.instance, record.source.port, record.target.instance, record.target.port)
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
    _record_pipeline_timing("record_assembly", t_record_assembly_start)

    if debug_timing and verbose_route_diagnostics:
        print(f"      - A* route-search loop time: {astar_elapsed_s:.4f} s")
        print(
            "      - Route search stats: "
            f"simple={simple_route_count}/{len(route_jobs)}, "
            f"expanded_states={total_expanded_states}, "
            f"repairs={repair_count}"
        )
        print("      - A* timing breakdown by operation:")
        for bucket_name in (
            "normal_route",
            "probe_route",
            "preemptive_crossing_ripup",
            "repair_failed_net",
            "reroute_victims",
            "endpoint_correction",
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

    realization_grid_spec = (
        int(grid.width),
        int(grid.height),
        float(grid.grid_size_um),
        float(origin_x_um),
        float(origin_y_um),
    )
    if not defer_realization:
        t_direct_realization_start = _pipeline_timer_start()
        realize_routed_net_records(
            routed_layout,
            routed_net_records,
            route_width_um=route_width_um,
            route_layer=route_layer,
            realization_grid_spec=realization_grid_spec,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_cells=bend_radius_cells,
        )
        _record_pipeline_timing("direct_realization", t_direct_realization_start)

    _augment_crossing_plan_with_realized_overlaps(
        router=router,
        crossing_plan_info=crossing_plan_info,
        routed_records_by_net_id=route_bookkeeping.records_by_id,
    )
    if hasattr(router, "crossing_events"):
        try:
            native_crossing_events = list(cast(Iterable[Any], router.crossing_events()))
        except Exception:
            native_crossing_events = []
        crossing_plan_info["native_crossing_events"] = native_crossing_events
        crossing_plan_info["native_crossing_event_count"] = len(native_crossing_events)
    _write_crossing_debug_artifacts(
        debug_path=debug_path,
        debug_prefix=debug_prefix,
        crossing_plan_info=crossing_plan_info,
    )

    t_debug_artifact_start = _pipeline_timer_start()
    debug_artifacts = build_route_debug_artifacts(
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
        route_nets_timings_s=route_nets_timings_s,
    )
    debug_artifacts = replace(
        debug_artifacts,
        crossing_plan_info=crossing_plan_info,
    )
    _record_pipeline_timing("debug_artifact_assembly", t_debug_artifact_start)
    if collect_pipeline_timing:
        debug_artifacts = replace(
            debug_artifacts,
            route_nets_timings_s=dict(route_nets_timings_s),
        )
    return routed_layout, debug_artifacts
