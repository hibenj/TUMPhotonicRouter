"""Path-length analysis helpers for Rust-routed photonic nets."""

from __future__ import annotations

from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    NodeIncomingEdgeTiming,
    NodeTiming,
    PathLengthAnalysisResult,
    RoutedEdgeKey,
    SchematicLike,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)

from translation.route_rust_types import RoutedNetRecord

PATH_LENGTH_MATCH_TOLERANCE_UM = 1.0e-6


def _object_to_float(value: object, default: float = 0.0) -> float:
    if isinstance(value, (str, bytes, bytearray, int, float)):
        return float(value)
    return default


def _object_to_int(value: object, default: int = 0) -> int:
    if isinstance(value, (str, bytes, bytearray, int, float)):
        return int(value)
    return default


def _list_length(value: object) -> int:
    return len(value) if isinstance(value, list) else 0


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


def minimum_four_bend_extra_length_um(
    *,
    grid_size_um: float,
    bend_radius_cells: int,
) -> float:
    """Minimum practical matching request: one bump needs four 90-degree bends."""
    bend_radius_um = max(0.0, float(grid_size_um) * float(bend_radius_cells))
    return 2.0 * 3.141592653589793 * bend_radius_um


def compute_group_lifted_requirements(
    analysis: PathLengthAnalysisResult,
    *,
    minimum_insertable_extra_um: float,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> tuple[list[MissingLengthRequirement], list[dict[str, object]]]:
    """Raise convergence targets when an edge deficit is smaller than one bump.

    The topological pass propagates lifted output arrivals downstream. Without
    this, adding one bump at an upstream node would be invisible to later
    convergence groups.
    """
    min_insertable = max(0.0, float(minimum_insertable_extra_um))
    adjusted_output_arrivals: dict[str, float] = {}
    requirements: list[MissingLengthRequirement] = []
    groups: list[dict[str, object]] = []

    for node_name in analysis.topological_order:
        timing = analysis.node_timings[node_name]
        if not timing.incoming_edges:
            adjusted_input_arrival = 0.0
            adjusted_output_arrivals[node_name] = adjusted_input_arrival + float(
                timing.internal_delay_um
            )
            continue

        edge_arrivals: list[tuple[NodeIncomingEdgeTiming, float]] = []
        for edge_timing in timing.incoming_edges:
            source_output = adjusted_output_arrivals.get(
                edge_timing.edge_key.source.instance,
                0.0,
            )
            edge_arrivals.append(
                (edge_timing, source_output + float(edge_timing.routed_length_um))
            )
        base_target = max(arrival for _, arrival in edge_arrivals)
        base_missing = [
            max(0.0, base_target - arrival)
            for _, arrival in edge_arrivals
        ]
        has_sub_bump_deficit = any(
            tolerance_um < missing < min_insertable - tolerance_um
            for missing in base_missing
        )
        lift_um = min_insertable if has_sub_bump_deficit else 0.0
        adjusted_target = base_target + lift_um
        adjusted_output_arrivals[node_name] = adjusted_target + float(
            timing.internal_delay_um
        )

        if len(edge_arrivals) < 2:
            continue

        incoming_edges: list[dict[str, object]] = []
        edges_requiring_meander = 0
        adjusted_missing_values: list[float] = []
        for (edge_timing, arrival), raw_missing in zip(edge_arrivals, base_missing):
            adjusted_missing = max(0.0, adjusted_target - arrival)
            if adjusted_missing <= tolerance_um:
                adjusted_missing = 0.0
            else:
                edges_requiring_meander += 1
                requirements.append(
                    MissingLengthRequirement(
                        edge_key=edge_timing.edge_key,
                        missing_length_um=adjusted_missing,
                    )
                )
            adjusted_missing_values.append(adjusted_missing)
            incoming_edges.append(
                {
                    **_incoming_edge_timing_to_dict(edge_timing),
                    "adjusted_edge_arrival_um": float(arrival),
                    "raw_missing_length_um": float(raw_missing),
                    "target_lift_um": float(lift_um),
                    "adjusted_missing_length_um": float(adjusted_missing),
                }
            )

        groups.append(
            {
                "node_name": timing.node_name,
                "node_type": timing.node_type.value,
                "incoming_count": len(edge_arrivals),
                "base_target_input_arrival_um": float(base_target),
                "target_lift_um": float(lift_um),
                "target_input_arrival_um": float(adjusted_target),
                "output_arrival_um": float(adjusted_output_arrivals[node_name]),
                "minimum_insertable_extra_um": float(min_insertable),
                "max_missing_length_um": max(adjusted_missing_values, default=0.0),
                "total_missing_length_um": float(sum(adjusted_missing_values)),
                "edges_requiring_meander": edges_requiring_meander,
                "incoming_edges": incoming_edges,
            }
        )

    return requirements, groups


def transparent_node_requirement_alternatives(
    analysis: PathLengthAnalysisResult,
    requirements: list[MissingLengthRequirement],
    *,
    transparent_prefixes: tuple[str, ...] = ("heater",),
) -> dict[RoutedEdgeKey, list[RoutedEdgeKey]]:
    """Return upstream insertion alternatives through one-in/one-out devices."""
    incoming_by_node: dict[str, list[RoutedEdgeKey]] = {}
    outgoing_by_node: dict[str, list[RoutedEdgeKey]] = {}
    for timing in analysis.node_timings.values():
        for edge_timing in timing.incoming_edges:
            edge_key = edge_timing.edge_key
            incoming_by_node.setdefault(edge_key.target.instance, []).append(edge_key)
            outgoing_by_node.setdefault(edge_key.source.instance, []).append(edge_key)

    alternatives: dict[RoutedEdgeKey, list[RoutedEdgeKey]] = {}
    for req in requirements:
        edge_key = req.edge_key
        source_instance = edge_key.source.instance
        if not source_instance.startswith(transparent_prefixes):
            continue

        incoming = incoming_by_node.get(source_instance, [])
        outgoing = outgoing_by_node.get(source_instance, [])
        if len(incoming) != 1 or len(outgoing) != 1 or outgoing[0] != edge_key:
            continue

        alternatives[edge_key] = [incoming[0]]

    return alternatives


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


def _edge_identity(edge_key: RoutedEdgeKey) -> tuple[str, str, str, str, str]:
    return (
        edge_key.net_name,
        edge_key.source.instance,
        edge_key.source.port,
        edge_key.target.instance,
        edge_key.target.port,
    )


def _edge_identity_from_info(edge_info: object) -> tuple[str, str, str, str, str] | None:
    if not isinstance(edge_info, dict):
        return None
    source = edge_info.get("source", {})
    target = edge_info.get("target", {})
    if not isinstance(source, dict) or not isinstance(target, dict):
        return None
    return (
        str(edge_info.get("net_name", "")),
        str(source.get("instance", "")),
        str(source.get("port", "")),
        str(target.get("instance", "")),
        str(target.get("port", "")),
    )


def _incoming_edge_timing_to_dict(edge_timing: NodeIncomingEdgeTiming) -> dict[str, object]:
    return {
        "edge": edge_key_to_dict(edge_timing.edge_key),
        "routed_length_um": float(edge_timing.routed_length_um),
        "edge_arrival_um": float(edge_timing.edge_arrival_um),
        "missing_length_um": float(edge_timing.missing_length_um),
    }


def node_timing_to_dict(timing: NodeTiming) -> dict[str, object]:
    return {
        "node_name": timing.node_name,
        "node_type": timing.node_type.value,
        "internal_delay_um": float(timing.internal_delay_um),
        "input_arrival_um": float(timing.input_arrival_um),
        "output_arrival_um": float(timing.output_arrival_um),
        "incoming_edges": [
            _incoming_edge_timing_to_dict(edge_timing)
            for edge_timing in timing.incoming_edges
        ],
    }


def matching_groups_to_info(analysis: PathLengthAnalysisResult) -> list[dict[str, object]]:
    groups: list[dict[str, object]] = []
    for node_name in analysis.topological_order:
        timing = analysis.node_timings.get(node_name)
        if timing is None or len(timing.incoming_edges) < 2:
            continue
        missing_values = [float(edge.missing_length_um) for edge in timing.incoming_edges]
        groups.append(
            {
                "node_name": timing.node_name,
                "node_type": timing.node_type.value,
                "incoming_count": len(timing.incoming_edges),
                "target_input_arrival_um": float(timing.input_arrival_um),
                "max_missing_length_um": max(missing_values, default=0.0),
                "total_missing_length_um": float(sum(missing_values)),
                "edges_requiring_meander": sum(1 for value in missing_values if value > 0.0),
                "incoming_edges": [
                    _incoming_edge_timing_to_dict(edge_timing)
                    for edge_timing in timing.incoming_edges
                ],
            }
        )
    return groups


def matching_group_diagnostics_to_info(
    analysis: PathLengthAnalysisResult,
    meander_report: dict[str, object] | None,
    *,
    adjusted_requirements: list[MissingLengthRequirement] | None = None,
    lifted_groups: list[dict[str, object]] | None = None,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> list[dict[str, object]]:
    results = []
    if isinstance(meander_report, dict):
        raw_results = meander_report.get("results", [])
        if isinstance(raw_results, list):
            results = [item for item in raw_results if isinstance(item, dict)]

    inserted_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    status_by_edge: dict[tuple[str, str, str, str, str], str] = {}
    unmatched_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    for item in results:
        edge_id = _edge_identity_from_info(item.get("edge"))
        if edge_id is None:
            continue
        inserted_by_edge[edge_id] = float(item.get("inserted_extra_length_um", 0.0))
        status_by_edge[edge_id] = str(item.get("status", "unknown"))
        unmatched_by_edge[edge_id] = float(item.get("unmatched_length_um", 0.0))

    adjusted_missing_by_edge: dict[tuple[str, str, str, str, str], float] = {}
    if adjusted_requirements is not None:
        adjusted_missing_by_edge = {
            _edge_identity(req.edge_key): float(req.missing_length_um)
            for req in adjusted_requirements
        }

    base_groups = lifted_groups if lifted_groups is not None else matching_groups_to_info(analysis)
    diagnostics: list[dict[str, object]] = []
    for group in base_groups:
        edges: list[dict[str, object]] = []
        max_unmatched = 0.0
        max_physical_residual = 0.0
        max_disregarded = 0.0
        raw_edges = group.get("incoming_edges", [])
        if not isinstance(raw_edges, list):
            raw_edges = []
        timing_edges_by_id = {
            _edge_identity(edge_timing.edge_key): edge_timing
            for edge_timing in analysis.node_timings[str(group["node_name"])].incoming_edges
        }
        for raw_edge in raw_edges:
            if not isinstance(raw_edge, dict):
                continue
            edge_id = _edge_identity_from_info(raw_edge.get("edge"))
            if edge_id is None:
                continue
            edge_timing = timing_edges_by_id.get(edge_id)
            if edge_timing is None:
                continue
            edge_id = _edge_identity(edge_timing.edge_key)
            missing = float(
                raw_edge.get(
                    "adjusted_missing_length_um",
                    adjusted_missing_by_edge.get(edge_id, edge_timing.missing_length_um),
                )
            )
            inserted = float(inserted_by_edge.get(edge_id, 0.0))
            status = status_by_edge.get(edge_id, "not_required" if missing == 0.0 else "missing")
            physical_residual = max(0.0, missing - inserted)
            accepted_unmatched = max(0.0, unmatched_by_edge.get(edge_id, physical_residual))
            disregarded = physical_residual if status == "below_minimum_bump" else 0.0
            if status == "below_minimum_bump":
                accepted_unmatched = 0.0
            max_unmatched = max(max_unmatched, accepted_unmatched)
            max_physical_residual = max(max_physical_residual, physical_residual)
            max_disregarded = max(max_disregarded, disregarded)
            edges.append(
                {
                    **raw_edge,
                    "inserted_extra_length_um": inserted,
                    "physical_residual_um": physical_residual,
                    "accepted_unmatched_um": accepted_unmatched,
                    "disregarded_residual_um": disregarded,
                    "meander_status": status,
                }
            )
        diagnostics.append(
            {
                **{key: value for key, value in group.items() if key != "incoming_edges"},
                "tolerance_um": float(tolerance_um),
                "max_accepted_unmatched_um": float(max_unmatched),
                "max_physical_residual_um": float(max_physical_residual),
                "max_disregarded_residual_um": float(max_disregarded),
                "within_tolerance": bool(max_unmatched <= tolerance_um),
                "has_disregarded_residual": bool(max_disregarded > tolerance_um),
                "incoming_edges": edges,
            }
        )
    return diagnostics


def path_length_acceptance_summary(
    diagnostics: list[dict[str, object]],
    *,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> dict[str, object]:
    """Summarize whether realized PLM exactly satisfies every group target."""
    failed_groups: list[dict[str, object]] = []
    max_physical_residual = 0.0
    max_accepted_unmatched = 0.0
    max_disregarded_residual = 0.0

    for group in diagnostics:
        if not isinstance(group, dict):
            continue
        group_failures: list[dict[str, object]] = []
        raw_edges = group.get("incoming_edges", [])
        if not isinstance(raw_edges, list):
            raw_edges = []
        for edge in raw_edges:
            if not isinstance(edge, dict):
                continue
            physical_residual = _object_to_float(edge.get("physical_residual_um", 0.0))
            accepted_unmatched = _object_to_float(edge.get("accepted_unmatched_um", 0.0))
            disregarded_residual = _object_to_float(edge.get("disregarded_residual_um", 0.0))
            max_physical_residual = max(max_physical_residual, physical_residual)
            max_accepted_unmatched = max(max_accepted_unmatched, accepted_unmatched)
            max_disregarded_residual = max(max_disregarded_residual, disregarded_residual)
            if physical_residual <= tolerance_um:
                continue
            group_failures.append(
                {
                    "edge": edge.get("edge", {}),
                    "meander_status": edge.get("meander_status", "unknown"),
                    "requested_extra_length_um": _object_to_float(
                        edge.get(
                            "adjusted_missing_length_um",
                            edge.get("missing_length_um", 0.0),
                        )
                    ),
                    "inserted_extra_length_um": _object_to_float(
                        edge.get("inserted_extra_length_um", 0.0)
                    ),
                    "physical_residual_um": physical_residual,
                    "accepted_unmatched_um": accepted_unmatched,
                    "disregarded_residual_um": disregarded_residual,
                }
            )
        if group_failures:
            failed_groups.append(
                {
                    "node_name": group.get("node_name", ""),
                    "node_type": group.get("node_type", ""),
                    "target_input_arrival_um": _object_to_float(
                        group.get("target_input_arrival_um", 0.0)
                    ),
                    "target_lift_um": _object_to_float(group.get("target_lift_um", 0.0)),
                    "max_physical_residual_um": _object_to_float(
                        group.get("max_physical_residual_um", 0.0)
                    ),
                    "failures": group_failures,
                }
            )

    return {
        "passed": not failed_groups,
        "tolerance_um": float(tolerance_um),
        "failed_group_count": len(failed_groups),
        "failed_edge_count": sum(_list_length(group.get("failures", [])) for group in failed_groups),
        "max_physical_residual_um": float(max_physical_residual),
        "max_accepted_unmatched_um": float(max_accepted_unmatched),
        "max_disregarded_residual_um": float(max_disregarded_residual),
        "failed_groups": failed_groups,
    }


def format_path_length_acceptance_failure(summary: dict[str, object]) -> str:
    """Build a compact user-facing PLM failure message."""
    failed_groups = summary.get("failed_groups", [])
    if not isinstance(failed_groups, list):
        failed_groups = []
    lines = [
        "Path-length matching failed: "
        f"{_object_to_int(summary.get('failed_edge_count', 0))} edge(s) in "
        f"{_object_to_int(summary.get('failed_group_count', 0))} group(s) retain physical "
        f"residual above {_object_to_float(summary.get('tolerance_um', 0.0)):.6g} um "
        f"(max {_object_to_float(summary.get('max_physical_residual_um', 0.0)):.6g} um)."
    ]
    for group in failed_groups[:5]:
        if not isinstance(group, dict):
            continue
        node_name = str(group.get("node_name", "<unknown>"))
        failures = group.get("failures", [])
        if not isinstance(failures, list):
            failures = []
        for failure in failures[:3]:
            if not isinstance(failure, dict):
                continue
            edge_info = failure.get("edge", {})
            lines.append(
                "  - "
                f"{node_name} {_edge_label_from_info(edge_info)}: "
                f"requested={float(failure.get('requested_extra_length_um', 0.0)):.6g} um, "
                f"inserted={float(failure.get('inserted_extra_length_um', 0.0)):.6g} um, "
                f"residual={float(failure.get('physical_residual_um', 0.0)):.6g} um, "
                f"status={failure.get('meander_status', 'unknown')}"
            )
    if len(failed_groups) > 5:
        lines.append(f"  - ... {len(failed_groups) - 5} more failed group(s)")
    return "\n".join(lines)


def _edge_label_from_info(edge_info: object) -> str:
    if not isinstance(edge_info, dict):
        return "<unknown edge>"
    source = edge_info.get("source", {})
    target = edge_info.get("target", {})
    if not isinstance(source, dict):
        source = {}
    if not isinstance(target, dict):
        target = {}
    return (
        f"{edge_info.get('net_name', '<unknown>')} "
        f"{source.get('instance', '?')},{source.get('port', '?')} -> "
        f"{target.get('instance', '?')},{target.get('port', '?')}"
    )


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
        "matching_groups": matching_groups_to_info(analysis),
    }
