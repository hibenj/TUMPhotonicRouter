"""Path-length analysis helpers for Rust-routed photonic nets."""

from __future__ import annotations

from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    NodeTiming,
    PathLengthAnalysisResult,
    RoutedEdgeKey,
    SchematicLike,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)

from translation.route_rust_types import RoutedNetRecord


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
