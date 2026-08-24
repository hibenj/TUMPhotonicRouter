"""Crossing plan construction and insertion-loss reporting for the Rust-backed photonic router."""

from __future__ import annotations

import json
import math
import os
from collections import Counter
from collections.abc import Iterable as IterableABC
from pathlib import Path
from typing import Any, Iterable, Mapping, cast

from gdsfactory.schematic import Schematic

from photonic_router.crossing_plan import CrossingPlan, build_crossing_plan
from photonic_router.topology_analysis import analyze_schematic_topology
from translation.route_rust_crossing_components import _crossing_component_bbox_size_um
from translation.route_rust_geometry import _first_perpendicular_route_intersection
from translation.route_rust_records import route_edge_key
from translation.route_rust_types import RouteJob, RoutedNetRecord


def _ensure_dir(path: Path) -> None:
    from translation.route_rust import _ensure_dir as route_rust_ensure_dir

    route_rust_ensure_dir(path)


DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM = 200.0
COLLISION_CROSSING_SEARCH_LOSS_ENV = "PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM"

def _routed_records_by_net_id(
    records: Iterable[RoutedNetRecord],
) -> dict[int, RoutedNetRecord]:
    records_by_id: dict[int, RoutedNetRecord] = {}
    for record in records:
        if record.net_id is None:
            continue
        try:
            records_by_id[int(record.net_id)] = record
        except (TypeError, ValueError):
            continue
    return records_by_id


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


def _effective_crossing_search_loss(
    *,
    enable_crossings: bool,
    crossing_mode: str,
    crossing_loss: float,
) -> float:
    """Return the search-only crossing penalty passed to Rust A*.

    ``crossing_loss`` is the physical insertion-loss term reported to users.
    Collision-discovered crossing modes also need a non-physical search cost so
    A* tries a clean same-net route before probing route-route collisions.
    """
    physical_loss = float(crossing_loss)
    if not enable_crossings:
        return physical_loss
    if physical_loss > 0.0:
        return physical_loss
    if str(crossing_mode).strip().lower() in {"collision", "lidar-pure"}:
        override = os.environ.get(COLLISION_CROSSING_SEARCH_LOSS_ENV)
        if override is not None:
            override_value = float(override)
            if not math.isfinite(override_value) or override_value < 0.0:
                raise ValueError(
                    f"{COLLISION_CROSSING_SEARCH_LOSS_ENV} must be finite and non-negative"
                )
            return override_value
        return DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM
    return physical_loss


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
    crossing_search_loss: float,
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
        "crossing_loss": float(crossing_loss),
        "crossing_search_loss": float(crossing_search_loss),
        "crossing_half_size_cells": int(crossing_half_size_cells),
        "min_straight_cells_per_crossing": int(min_straight_cells_per_crossing),
        "allow_only_expected_crossings": bool(allow_only_expected_crossings),
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

    router.set_crossing_constraints([])
    router.set_crossing_config(
        rust_backend.CrossingConfig(
            enabled=True,
            crossing_loss=float(crossing_search_loss),
            crossing_half_size_cells=int(crossing_half_size_cells),
            min_straight_cells_per_crossing=int(min_straight_cells_per_crossing),
            allow_only_expected_pairs=bool(allow_only_expected_crossings),
        )
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

    info["constraint_count"] = len(constraints)
    info["missing_event_count"] = len(missing_events)
    info["missing_events"] = missing_events
    info["events"] = event_records
    info["expected_crossings_by_net_id"] = dict(sorted(crossing_counts_by_net_id.items()))
    info["expected_crossings_by_net_name"] = dict(sorted(crossing_counts_by_net_name.items()))
    return info


def _segment_bend_units(route_obj: object) -> float:
    bend_units = 0.0
    for raw_segment in getattr(route_obj, "segments", []) or []:
        try:
            kind = str(raw_segment.get("kind", ""))
        except AttributeError:
            continue
        if kind == "turn45":
            bend_units += 0.5
        elif kind == "turn90":
            bend_units += 1.0
        elif kind and kind != "straight":
            bend_units += 1.0
    return bend_units


def _write_insertion_loss_report(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    crossing_counts_by_net_id: Mapping[int, int],
    propagation_loss_per_um: float,
    bend_loss_per_90deg: float,
    crossing_count_source: str,
) -> None:
    crossing_loss = float(crossing_plan_info.get("crossing_loss", 0.0) or 0.0)
    per_net: list[dict[str, object]] = []
    total_length_um = 0.0
    total_bend_units = 0.0
    total_crossing_count = 0
    total_insertion_loss = 0.0
    for net_id, record in sorted(routed_records_by_net_id.items()):
        length_um = float(record.total_length_um)
        bend_units = _segment_bend_units(record.route_obj)
        crossing_count = int(crossing_counts_by_net_id.get(int(net_id), 0))
        insertion_loss = (
            length_um * propagation_loss_per_um
            + bend_units * bend_loss_per_90deg
            + crossing_count * crossing_loss
        )
        total_length_um += length_um
        total_bend_units += bend_units
        total_crossing_count += crossing_count
        total_insertion_loss += insertion_loss
        per_net.append(
            {
                "net_id": int(net_id),
                "net_name": record.net_name,
                "length_um": length_um,
                "bend_90deg_units": bend_units,
                "crossing_count": crossing_count,
                "propagation_loss": length_um * propagation_loss_per_um,
                "bend_loss": bend_units * bend_loss_per_90deg,
                "crossing_loss": crossing_count * crossing_loss,
                "insertion_loss": insertion_loss,
            }
        )

    crossing_plan_info["insertion_loss_model"] = {
        "propagation_loss_per_um": propagation_loss_per_um,
        "bend_loss_per_90deg": bend_loss_per_90deg,
        "crossing_loss": crossing_loss,
        "crossing_count_source": crossing_count_source,
        "device_loss_included": False,
        "formula": (
            "length_um * propagation_loss_per_um + "
            "bend_90deg_units * bend_loss_per_90deg + "
            "crossing_count * crossing_loss"
        ),
    }
    crossing_plan_info["insertion_loss_summary"] = {
        "net_count": len(per_net),
        "total_length_um": total_length_um,
        "total_bend_90deg_units": total_bend_units,
        "total_crossing_count": total_crossing_count,
        "total_insertion_loss": total_insertion_loss,
    }
    crossing_plan_info["insertion_loss_by_net"] = per_net


def _augment_insertion_loss_report(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    native_crossing_events: Iterable[object],
    propagation_loss_per_um: float = 0.0,
    bend_loss_per_90deg: float = 0.0,
) -> None:
    if not crossing_plan_info.get("enabled"):
        return
    crossing_counts_by_net_id: Counter[int] = Counter()
    for raw_event in native_crossing_events:
        try:
            net_id = int(raw_event["net_id"])
            partner_net_id = int(raw_event["partner_net_id"])
        except (TypeError, KeyError, ValueError):
            continue
        crossing_counts_by_net_id[net_id] += 1
        crossing_counts_by_net_id[partner_net_id] += 1

    _write_insertion_loss_report(
        crossing_plan_info=crossing_plan_info,
        routed_records_by_net_id=routed_records_by_net_id,
        crossing_counts_by_net_id=crossing_counts_by_net_id,
        propagation_loss_per_um=propagation_loss_per_um,
        bend_loss_per_90deg=bend_loss_per_90deg,
        crossing_count_source="native_crossing_events",
    )


def _augment_insertion_loss_report_from_realized_intersections(
    *,
    crossing_plan_info: dict[str, object],
    routed_records_by_net_id: Mapping[int, RoutedNetRecord],
    propagation_loss_per_um: float = 0.0,
    bend_loss_per_90deg: float = 0.0,
) -> None:
    if not crossing_plan_info.get("enabled"):
        return
    crossing_counts_by_net_id: Counter[int] = Counter()
    raw_intersections = crossing_plan_info.get("realized_intersections", ())
    if not isinstance(raw_intersections, IterableABC) or isinstance(
        raw_intersections,
        (str, bytes, bytearray),
    ):
        raw_intersections = ()
    for raw_intersection in raw_intersections:
        if not isinstance(raw_intersection, Mapping):
            continue
        classification = str(raw_intersection.get("classification", "") or "")
        if not classification.startswith("legal_"):
            continue
        try:
            net_id_a = int(raw_intersection["net_id_a"])
            net_id_b = int(raw_intersection["net_id_b"])
        except (TypeError, KeyError, ValueError):
            continue
        crossing_counts_by_net_id[net_id_a] += 1
        crossing_counts_by_net_id[net_id_b] += 1

    _write_insertion_loss_report(
        crossing_plan_info=crossing_plan_info,
        routed_records_by_net_id=routed_records_by_net_id,
        crossing_counts_by_net_id=crossing_counts_by_net_id,
        propagation_loss_per_um=propagation_loss_per_um,
        bend_loss_per_90deg=bend_loss_per_90deg,
        crossing_count_source="realized_intersections",
    )


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
    required_crossing_margin_cells = int(
        crossing_plan_info.get("crossing_half_size_cells", 0) or 0
    ) + int(crossing_plan_info.get("bend_runout_cells_per_crossing", 0) or 0)
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
    if crossing_plan_info.get("illegal_realized_crossing_count", 0):
        lines.append("illegal_realized_crossings:")
        for crossing in cast(
            list[dict[str, object]],
            crossing_plan_info.get("illegal_realized_crossings", []),
        ):
            lines.append(
                "  - "
                f"{crossing.get('net_name_a')} x {crossing.get('net_name_b')} "
                f"point={crossing.get('point_um')} "
                f"reason={crossing.get('reason')} "
                f"margin_a={crossing.get('segment_a_margin_um')} "
                f"margin_b={crossing.get('segment_b_margin_um')} "
                f"required={crossing.get('required_margin_um')}"
            )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

