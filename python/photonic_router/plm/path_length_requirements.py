"""Requirement construction and delay-insertion candidates for path-length matching.

The timing requirements (how much extra length each edge needs, the group
lifts, the optional output matching) and the candidates that say where that
length may go (the edge itself, or upstream where every group stays matched,
told by structure). Until phase 6 slice B the candidates lived in their own
module behind the ``route_rust_analysis`` facade.
"""

from __future__ import annotations

import math

from photonic_router.plm.path_length_graph import (
    DelayInsertionCandidate,
    MissingLengthRequirement,
    NodeIncomingEdgeTiming,
    NodeType,
    PathLengthAnalysisResult,
    PathLengthGraphAnnotations,
    PhotonicRoutingGraph,
    RoutedEdgeKey,
    SchematicLike,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)
from photonic_router.records.route_rust_types import (
    DEFAULT_MEANDER_MIN_STRAIGHT_UM,
    RoutedNetRecord,
    edge_key_to_dict,
)

PATH_LENGTH_MATCH_TOLERANCE_UM = 1.0e-6

__all__ = [
    "PATH_LENGTH_MATCH_TOLERANCE_UM",
    "adjusted_output_arrivals_for_requirements",
    "analysis_edge_maps",
    "analyze_path_length_matching",
    "analyze_path_length_matching_for_graph",
    "build_requirement_delay_candidates",
    "compute_group_lifted_requirements",
    "compute_output_matching_requirements",
    "delay_candidate_to_dict",
    "edge_identity",
    "edge_identity_from_info",
    "edge_key_to_dict",
    "merge_missing_length_requirements",
    "minimum_four_bend_extra_length_um",
    "node_depths",
    "requirement_to_dict",
    "routed_net_records_to_edge_lengths",
]


def requirement_to_dict(req: MissingLengthRequirement) -> dict[str, object]:
    return {
        "edge": edge_key_to_dict(req.edge_key),
        "missing_length_um": float(req.missing_length_um),
    }


def delay_candidate_to_dict(candidate: DelayInsertionCandidate) -> dict[str, object]:
    return {
        "requirement_edge": edge_key_to_dict(candidate.requirement_edge_key),
        "edges": [edge_key_to_dict(edge_key) for edge_key in candidate.edge_keys],
        "extra_length_um": float(candidate.extra_length_um),
        "reason": candidate.reason,
        "affected_requirement_edges": [
            edge_key_to_dict(edge_key) for edge_key in candidate.affected_requirement_edge_keys
        ],
    }


def edge_identity(edge_key: RoutedEdgeKey) -> tuple[str, str, str, str, str]:
    return (
        edge_key.net_name,
        edge_key.source.instance,
        edge_key.source.port,
        edge_key.target.instance,
        edge_key.target.port,
    )


def edge_identity_from_info(edge_info: object) -> tuple[str, str, str, str, str] | None:
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


def analysis_edge_maps(
    analysis: PathLengthAnalysisResult,
) -> tuple[dict[str, list[RoutedEdgeKey]], dict[str, list[RoutedEdgeKey]]]:
    incoming_by_node: dict[str, list[RoutedEdgeKey]] = {
        node_name: [] for node_name in analysis.node_timings
    }
    outgoing_by_node: dict[str, list[RoutedEdgeKey]] = {
        node_name: [] for node_name in analysis.node_timings
    }
    for timing in analysis.node_timings.values():
        for edge_timing in timing.incoming_edges:
            edge_key = edge_timing.edge_key
            # along the light: the key names the net as routed, which may be
            # written against the light
            incoming_by_node.setdefault(analysis.target_instance(edge_key), []).append(edge_key)
            outgoing_by_node.setdefault(analysis.source_instance(edge_key), []).append(edge_key)
    return incoming_by_node, outgoing_by_node


def build_requirement_delay_candidates(
    analysis: PathLengthAnalysisResult,
    requirements: list[MissingLengthRequirement],
    *,
    max_recursive_pushback_depth: int = 8,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> dict[RoutedEdgeKey, list[DelayInsertionCandidate]]:
    """Build conservative insertion candidates for each timing requirement.

    Direct candidates always insert on the required edge. Upstream candidates
    are found by recursively moving equal-delay requirements through
    transparent serial elements and complete common-mode output bundles,
    both told by structure, not by name (owner's rule, 2026-10-07: upstream
    only where every group stays matched): a node with exactly one incoming
    and one outgoing edge passes a delay unchanged, and a node whose whole
    output bundle needs the same delay can take it on its inputs instead,
    because every path through it gets the delay.
    """
    incoming_by_node, outgoing_by_node = analysis_edge_maps(analysis)
    missing_by_edge = {req.edge_key: float(req.missing_length_um) for req in requirements}
    candidates: dict[RoutedEdgeKey, list[DelayInsertionCandidate]] = {}

    def _edge_tuple_identity(
        edge_keys: tuple[RoutedEdgeKey, ...],
    ) -> tuple[tuple[str, str, str, str, str], ...]:
        return tuple(edge_identity(edge_key) for edge_key in edge_keys)

    def _dedupe_edge_tuple(
        edge_keys: tuple[RoutedEdgeKey, ...],
    ) -> tuple[RoutedEdgeKey, ...]:
        deduped: list[RoutedEdgeKey] = []
        seen: set[RoutedEdgeKey] = set()
        for edge_key in edge_keys:
            if edge_key in seen:
                continue
            seen.add(edge_key)
            deduped.append(edge_key)
        return tuple(deduped)

    def _candidate_sort_key(candidate: DelayInsertionCandidate) -> tuple[int, int, int]:
        shared_rank = 0 if len(candidate.affected_requirement_edge_keys) > 1 else 1
        edge_count_rank = -len(candidate.edge_keys)
        recursive_rank = 0 if candidate.reason.startswith("recursive") else 1
        return shared_rank, recursive_rank, edge_count_rank

    def _complete_common_mode_source(
        edge_keys: tuple[RoutedEdgeKey, ...],
    ) -> str | None:
        if len(edge_keys) < 2:
            return None
        source_instances = {analysis.source_instance(edge_key) for edge_key in edge_keys}
        if len(source_instances) != 1:
            return None
        source_instance = next(iter(source_instances))
        outgoing = tuple(outgoing_by_node.get(source_instance, ()))
        incoming = tuple(incoming_by_node.get(source_instance, ()))
        if len(outgoing) < 2 or len(incoming) < 1:
            return None
        if set(edge_keys) != set(outgoing):
            return None
        return source_instance

    def _append_candidate(
        edge_candidates: list[DelayInsertionCandidate],
        seen_candidates: set[
            tuple[
                tuple[tuple[str, str, str, str, str], ...],
                tuple[tuple[str, str, str, str, str], ...],
                int,
            ]
        ],
        *,
        requirement_edge_key: RoutedEdgeKey,
        edge_keys: tuple[RoutedEdgeKey, ...],
        extra_length_um: float,
        reason: str,
        affected_requirement_edge_keys: tuple[RoutedEdgeKey, ...],
    ) -> None:
        edge_keys = _dedupe_edge_tuple(edge_keys)
        if not edge_keys or len(edge_keys) != len(set(edge_keys)):
            return
        affected_requirement_edge_keys = tuple(
            sorted(
                _dedupe_edge_tuple(affected_requirement_edge_keys),
                key=edge_identity,
            )
        )
        key = (
            _edge_tuple_identity(edge_keys),
            _edge_tuple_identity(affected_requirement_edge_keys),
            round(float(extra_length_um) / max(tolerance_um, 1.0e-12)),
        )
        if key in seen_candidates:
            return
        seen_candidates.add(key)
        edge_candidates.append(
            DelayInsertionCandidate(
                requirement_edge_key=requirement_edge_key,
                edge_keys=edge_keys,
                extra_length_um=float(extra_length_um),
                reason=reason,
                affected_requirement_edge_keys=affected_requirement_edge_keys,
            )
        )

    for req in requirements:
        if req.missing_length_um <= tolerance_um:
            continue
        edge_key = req.edge_key
        requested = float(req.missing_length_um)
        edge_candidates: list[DelayInsertionCandidate] = []
        seen_candidates: set[
            tuple[
                tuple[tuple[str, str, str, str, str], ...],
                tuple[tuple[str, str, str, str, str], ...],
                int,
            ]
        ] = set()
        queue: list[
            tuple[
                tuple[RoutedEdgeKey, ...],
                tuple[RoutedEdgeKey, ...],
                float,
                str,
                int,
            ]
        ] = [((edge_key,), (edge_key,), requested, "direct_edge", 0)]

        source_instance = analysis.source_instance(edge_key)
        source_outgoing = outgoing_by_node.get(source_instance, [])
        source_incoming = incoming_by_node.get(source_instance, [])
        if len(source_outgoing) >= 2 and len(source_incoming) >= 1:
            common_delay = min(
                missing_by_edge.get(outgoing_edge, 0.0) for outgoing_edge in source_outgoing
            )
            if common_delay > tolerance_um and requested <= common_delay + tolerance_um:
                affected_edges = tuple(
                    outgoing_edge
                    for outgoing_edge in source_outgoing
                    if missing_by_edge.get(outgoing_edge, 0.0) >= common_delay - tolerance_um
                )
                if len(affected_edges) >= 2 and set(affected_edges) == set(source_outgoing):
                    queue.append(
                        (
                            tuple(source_incoming),
                            affected_edges,
                            common_delay,
                            "common_mode_upstream_bundle",
                            0,
                        )
                    )

        seen_states: set[
            tuple[
                tuple[tuple[str, str, str, str, str], ...],
                tuple[tuple[str, str, str, str, str], ...],
                int,
            ]
        ] = set()
        while queue:
            current_edges, affected_edges, extra_length_um, reason, depth = queue.pop(0)
            current_edges = _dedupe_edge_tuple(current_edges)
            affected_edges = _dedupe_edge_tuple(affected_edges)
            state_key = (
                _edge_tuple_identity(current_edges),
                _edge_tuple_identity(affected_edges),
                round(float(extra_length_um) / max(tolerance_um, 1.0e-12)),
            )
            if state_key in seen_states:
                continue
            seen_states.add(state_key)
            _append_candidate(
                edge_candidates,
                seen_candidates,
                requirement_edge_key=edge_key,
                edge_keys=current_edges,
                extra_length_um=extra_length_um,
                reason=reason,
                affected_requirement_edge_keys=affected_edges,
            )

            if depth >= max_recursive_pushback_depth:
                continue

            for index, current_edge in enumerate(current_edges):
                source = analysis.source_instance(current_edge)
                incoming = incoming_by_node.get(source, [])
                outgoing = outgoing_by_node.get(source, [])
                if len(incoming) != 1 or len(outgoing) != 1 or outgoing[0] != current_edge:
                    continue
                next_edges = tuple(
                    incoming[0] if i == index else edge for i, edge in enumerate(current_edges)
                )
                queue.append(
                    (
                        next_edges,
                        affected_edges,
                        extra_length_um,
                        "transparent_serial_upstream"
                        if len(current_edges) == 1 and depth == 0
                        else "recursive_transparent_serial_upstream",
                        depth + 1,
                    )
                )

            common_source = _complete_common_mode_source(current_edges)
            if common_source is not None:
                queue.append(
                    (
                        tuple(incoming_by_node.get(common_source, ())),
                        affected_edges,
                        extra_length_um,
                        "recursive_common_mode_upstream_bundle",
                        depth + 1,
                    )
                )

        candidates[edge_key] = sorted(
            edge_candidates,
            key=_candidate_sort_key,
        )

    return candidates


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
    input_ports_by_component: dict[str, tuple[str, ...]] | None = None,
) -> tuple[PathLengthAnalysisResult, list]:
    """Compute per-edge missing lengths before polygon realization."""
    graph = build_graph_from_schematic(
        schematic,
        node_types=node_types,
        internal_delays_um=internal_delays_um,
        input_ports_by_component=input_ports_by_component,
    )
    return analyze_path_length_matching_for_graph(
        graph,
        routed_net_records=routed_net_records,
    )


def analyze_path_length_matching_for_graph(
    graph: PhotonicRoutingGraph,
    *,
    routed_net_records: list[RoutedNetRecord],
) -> tuple[PathLengthAnalysisResult, list]:
    """Compute per-edge missing lengths on an existing Python routing graph."""
    annotations = PathLengthGraphAnnotations.from_edge_lengths(
        graph,
        routed_net_records_to_edge_lengths(routed_net_records),
    )
    analysis = annotations.analyze_missing_lengths(graph)
    return analysis, list(list_edges_requiring_meander(analysis))


def minimum_four_bend_extra_length_um(
    *,
    grid_size_um: float,
    bend_radius_cells: int,
    min_straight_um: float = DEFAULT_MEANDER_MIN_STRAIGHT_UM,
) -> float:
    """Minimum practical matching request for the analytic fill-box meander.

    The historical function name is kept for API compatibility. The active
    multi-bump planner's one-bump lower bound is smaller than a full
    four-bend circumference delta because the inserted path replaces a straight
    segment.
    """
    bend_radius_um = max(0.0, float(grid_size_um) * float(bend_radius_cells))
    min_straight = max(0.0, float(min_straight_um))
    # One U-turn at the minimum amplitude (2r + min_straight), from the
    # fill-box model `extra = legs * (A - 4r + pi*r)` with legs = 2:
    return max(0.0, 2.0 * (min_straight + (math.pi - 2.0) * bend_radius_um))


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
            source_output = adjusted_output_arrivals.get(edge_timing.source_instance, 0.0)
            edge_arrivals.append((edge_timing, source_output + float(edge_timing.routed_length_um)))
        base_target = max(arrival for _, arrival in edge_arrivals)
        base_missing = [max(0.0, base_target - arrival) for _, arrival in edge_arrivals]
        has_sub_bump_deficit = any(
            tolerance_um < missing < min_insertable - tolerance_um for missing in base_missing
        )
        lift_um = min_insertable if has_sub_bump_deficit else 0.0
        adjusted_target = base_target + lift_um
        adjusted_output_arrivals[node_name] = adjusted_target + float(timing.internal_delay_um)

        if len(edge_arrivals) < 2:
            continue

        incoming_edges: list[dict[str, object]] = []
        edges_requiring_meander = 0
        adjusted_missing_values: list[float] = []
        for (edge_timing, arrival), raw_missing in zip(edge_arrivals, base_missing, strict=False):
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
                adjusted_output_arrivals.get(edge_timing.source_instance, 0.0)
                + float(edge_timing.routed_length_um)
                + extra_by_edge.get(edge_timing.edge_key, 0.0)
                for edge_timing in timing.incoming_edges
            )
        adjusted_output_arrivals[node_name] = adjusted_input_arrival + float(
            timing.internal_delay_um
        )
    return adjusted_output_arrivals


OUTPUT_SCOPES = ("all", "depth")


def node_depths(analysis: PathLengthAnalysisResult) -> dict[str, int]:
    """Each node's depth: the most components on any path from a source to it."""

    depths: dict[str, int] = {}
    for node_name in analysis.topological_order:
        timing = analysis.node_timings[node_name]
        if not timing.incoming_edges:
            depths[node_name] = 0
            continue
        depths[node_name] = 1 + max(
            depths.get(edge_timing.source_instance, 0) for edge_timing in timing.incoming_edges
        )
    return depths


def compute_output_matching_requirements(
    analysis: PathLengthAnalysisResult,
    *,
    existing_requirements: list[MissingLengthRequirement] | None = None,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
    scope: str = "all",
) -> tuple[list[MissingLengthRequirement], dict[str, object]]:
    """Return additional requirements that align the one-input output nodes.

    ``scope`` ``all`` aligns every output to the latest one; ``depth`` aligns
    only outputs of the same depth with each other (the owner's beam
    splitter, 2026-10-07: four outputs tapped after the first MZI stage are
    equalized among themselves, the four behind the last stage among
    themselves, the two sets 1.3 mm apart by design).
    """
    if scope not in OUTPUT_SCOPES:
        raise ValueError(f"output scope {scope!r} is not one of {OUTPUT_SCOPES}")
    existing_requirements = existing_requirements or []
    adjusted_output_arrivals = adjusted_output_arrivals_for_requirements(
        analysis,
        existing_requirements,
    )
    output_timings = [
        timing for timing in analysis.node_timings.values() if timing.node_type == NodeType.OUTPUT
    ]
    output_arrivals = {
        timing.node_name: float(adjusted_output_arrivals.get(timing.node_name, 0.0))
        for timing in output_timings
    }
    depths = node_depths(analysis) if scope == "depth" else {}
    group_of = {
        timing.node_name: (depths.get(timing.node_name, 0) if scope == "depth" else 0)
        for timing in output_timings
    }
    target_by_group: dict[int, float] = {}
    for name, arrival in output_arrivals.items():
        group = group_of[name]
        target_by_group[group] = max(target_by_group.get(group, arrival), arrival)
    target_arrival = max(output_arrivals.values(), default=0.0)
    requirements: list[MissingLengthRequirement] = []
    outputs: list[dict[str, object]] = []

    for timing in output_timings:
        arrival = output_arrivals[timing.node_name]
        group = group_of[timing.node_name]
        missing = max(0.0, target_by_group[group] - arrival)
        if missing <= tolerance_um:
            missing = 0.0
        output_info: dict[str, object] = {
            "node_name": timing.node_name,
            "arrival_um": float(arrival),
            "missing_length_um": float(missing),
            "incoming_count": len(timing.incoming_edges),
            "status": "not_required" if missing == 0.0 else "requires_delay",
            "depth": int(depths.get(timing.node_name, 0)) if scope == "depth" else None,
            "group_target_arrival_um": float(target_by_group[group]),
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
        "scope": scope,
        "target_output_arrival_um": float(target_arrival),
        "group_target_arrivals_um": {str(g): float(t) for g, t in sorted(target_by_group.items())},
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
        delays[req.edge_key] = delays.get(req.edge_key, 0.0) + float(req.missing_length_um)
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
