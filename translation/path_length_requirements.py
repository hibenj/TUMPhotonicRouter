"""Requirement construction for path-length matching."""

from __future__ import annotations

from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    NodeIncomingEdgeTiming,
    NodeType,
    PathLengthAnalysisResult,
    RoutedEdgeKey,
    SchematicLike,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)

from translation.path_length_candidates import edge_key_to_dict, requirement_to_dict
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
    """Compute per-edge missing lengths before polygon realization."""
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
    """Raise convergence targets when an edge deficit is smaller than one bump."""
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


def adjusted_output_arrivals_for_requirements(
    analysis: PathLengthAnalysisResult,
    requirements: list[MissingLengthRequirement],
) -> dict[str, float]:
    """Propagate expected output arrivals after planned edge delays are inserted."""
    extra_by_edge = _requirement_delays_by_edge(requirements)
    adjusted_output_arrivals: dict[str, float] = {}
    for node_name in analysis.topological_order:
        timing = analysis.node_timings[node_name]
        if not timing.incoming_edges:
            adjusted_input_arrival = 0.0
        else:
            adjusted_input_arrival = max(
                adjusted_output_arrivals.get(
                    edge_timing.edge_key.source.instance,
                    0.0,
                )
                + float(edge_timing.routed_length_um)
                + extra_by_edge.get(edge_timing.edge_key, 0.0)
                for edge_timing in timing.incoming_edges
            )
        adjusted_output_arrivals[node_name] = adjusted_input_arrival + float(
            timing.internal_delay_um
        )
    return adjusted_output_arrivals


def compute_output_matching_requirements(
    analysis: PathLengthAnalysisResult,
    *,
    existing_requirements: list[MissingLengthRequirement] | None = None,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> tuple[list[MissingLengthRequirement], dict[str, object]]:
    """Return additional requirements that align all one-input output nodes."""
    existing_requirements = existing_requirements or []
    adjusted_output_arrivals = adjusted_output_arrivals_for_requirements(
        analysis,
        existing_requirements,
    )
    output_timings = [
        timing
        for timing in analysis.node_timings.values()
        if timing.node_type == NodeType.OUTPUT
    ]
    output_arrivals = {
        timing.node_name: float(adjusted_output_arrivals.get(timing.node_name, 0.0))
        for timing in output_timings
    }
    target_arrival = max(output_arrivals.values(), default=0.0)
    requirements: list[MissingLengthRequirement] = []
    outputs: list[dict[str, object]] = []

    for timing in output_timings:
        arrival = output_arrivals[timing.node_name]
        missing = max(0.0, target_arrival - arrival)
        if missing <= tolerance_um:
            missing = 0.0
        output_info: dict[str, object] = {
            "node_name": timing.node_name,
            "arrival_um": float(arrival),
            "missing_length_um": float(missing),
            "incoming_count": len(timing.incoming_edges),
            "status": "not_required" if missing == 0.0 else "requires_delay",
        }
        if missing > 0.0:
            if len(timing.incoming_edges) == 1:
                edge_key = timing.incoming_edges[0].edge_key
                requirements.append(
                    MissingLengthRequirement(
                        edge_key=edge_key,
                        missing_length_um=missing,
                    )
                )
                output_info["edge"] = edge_key_to_dict(edge_key)
            else:
                output_info["status"] = "unsupported_output_fan_in"
        outputs.append(output_info)

    return requirements, {
        "enabled": True,
        "target_output_arrival_um": float(target_arrival),
        "output_count": len(output_timings),
        "requirements": [requirement_to_dict(req) for req in requirements],
        "outputs": outputs,
    }


def merge_missing_length_requirements(
    *requirement_groups: list[MissingLengthRequirement],
) -> list[MissingLengthRequirement]:
    """Merge additive requirement groups by edge while preserving first-seen order."""
    merged: dict[RoutedEdgeKey, float] = {}
    order: list[RoutedEdgeKey] = []
    for requirements in requirement_groups:
        for req in requirements:
            if req.edge_key not in merged:
                order.append(req.edge_key)
                merged[req.edge_key] = 0.0
            merged[req.edge_key] += float(req.missing_length_um)
    return [
        MissingLengthRequirement(edge_key=edge_key, missing_length_um=merged[edge_key])
        for edge_key in order
        if merged[edge_key] > PATH_LENGTH_MATCH_TOLERANCE_UM
    ]


def _requirement_delays_by_edge(
    requirements: list[MissingLengthRequirement],
) -> dict[RoutedEdgeKey, float]:
    delays: dict[RoutedEdgeKey, float] = {}
    for req in requirements:
        delays[req.edge_key] = delays.get(req.edge_key, 0.0) + float(
            req.missing_length_um
        )
    return delays


def _incoming_edge_timing_to_dict(
    edge_timing: NodeIncomingEdgeTiming,
) -> dict[str, object]:
    return {
        "edge": edge_key_to_dict(edge_timing.edge_key),
        "routed_length_um": float(edge_timing.routed_length_um),
        "edge_arrival_um": float(edge_timing.edge_arrival_um),
        "missing_length_um": float(edge_timing.missing_length_um),
    }
