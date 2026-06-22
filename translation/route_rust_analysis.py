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

    diagnostics: list[dict[str, object]] = []
    for group in matching_groups_to_info(analysis):
        edges: list[dict[str, object]] = []
        max_unmatched = 0.0
        max_physical_residual = 0.0
        max_disregarded = 0.0
        for edge_timing in analysis.node_timings[str(group["node_name"])].incoming_edges:
            edge_id = _edge_identity(edge_timing.edge_key)
            missing = float(edge_timing.missing_length_um)
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
                    **_incoming_edge_timing_to_dict(edge_timing),
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
