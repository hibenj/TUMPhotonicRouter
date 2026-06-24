"""Path-length analysis helpers for Rust-routed photonic nets."""

from __future__ import annotations

from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
    MissingLengthRequirement,
    NodeIncomingEdgeTiming,
    NodeType,
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


def _analysis_edge_maps(
    analysis: PathLengthAnalysisResult,
) -> tuple[dict[str, list[RoutedEdgeKey]], dict[str, list[RoutedEdgeKey]]]:
    incoming_by_node: dict[str, list[RoutedEdgeKey]] = {
        node_name: []
        for node_name in analysis.node_timings
    }
    outgoing_by_node: dict[str, list[RoutedEdgeKey]] = {
        node_name: []
        for node_name in analysis.node_timings
    }
    for timing in analysis.node_timings.values():
        for edge_timing in timing.incoming_edges:
            edge_key = edge_timing.edge_key
            incoming_by_node.setdefault(edge_key.target.instance, []).append(edge_key)
            outgoing_by_node.setdefault(edge_key.source.instance, []).append(edge_key)
    return incoming_by_node, outgoing_by_node


def _requirement_delays_by_edge(
    requirements: list[MissingLengthRequirement],
) -> dict[RoutedEdgeKey, float]:
    delays: dict[RoutedEdgeKey, float] = {}
    for req in requirements:
        delays[req.edge_key] = delays.get(req.edge_key, 0.0) + float(
            req.missing_length_um
        )
    return delays


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


def delay_candidate_to_dict(candidate: DelayInsertionCandidate) -> dict[str, object]:
    return {
        "requirement_edge": edge_key_to_dict(candidate.requirement_edge_key),
        "edges": [edge_key_to_dict(edge_key) for edge_key in candidate.edge_keys],
        "extra_length_um": float(candidate.extra_length_um),
        "reason": candidate.reason,
        "affected_requirement_edges": [
            edge_key_to_dict(edge_key)
            for edge_key in candidate.affected_requirement_edge_keys
        ],
    }


def build_requirement_delay_candidates(
    analysis: PathLengthAnalysisResult,
    requirements: list[MissingLengthRequirement],
    *,
    transparent_prefixes: tuple[str, ...] = ("heater",),
    common_mode_prefixes: tuple[str, ...] = ("mmi",),
    max_recursive_pushback_depth: int = 8,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> dict[RoutedEdgeKey, list[DelayInsertionCandidate]]:
    """Build conservative insertion candidates for each timing requirement.

    Direct candidates always insert on the required edge. Upstream candidates
    are found by recursively moving equal-delay requirements through
    transparent serial elements and complete common-mode MMI output bundles.
    """
    incoming_by_node, outgoing_by_node = _analysis_edge_maps(analysis)
    missing_by_edge = {
        req.edge_key: float(req.missing_length_um)
        for req in requirements
    }
    candidates: dict[RoutedEdgeKey, list[DelayInsertionCandidate]] = {}

    def _edge_sort_key(edge_key: RoutedEdgeKey) -> tuple[str, str, str, str, str]:
        return _edge_identity(edge_key)

    def _edge_tuple_identity(
        edge_keys: tuple[RoutedEdgeKey, ...],
    ) -> tuple[tuple[str, str, str, str, str], ...]:
        return tuple(_edge_identity(edge_key) for edge_key in edge_keys)

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
        recursive_rank = (
            0
            if candidate.reason.startswith("recursive")
            else 1
        )
        return shared_rank, recursive_rank, edge_count_rank

    def _complete_common_mode_source(
        edge_keys: tuple[RoutedEdgeKey, ...],
    ) -> str | None:
        if len(edge_keys) < 2:
            return None
        source_instances = {edge_key.source.instance for edge_key in edge_keys}
        if len(source_instances) != 1:
            return None
        source_instance = next(iter(source_instances))
        if not source_instance.startswith(common_mode_prefixes):
            return None
        outgoing = tuple(outgoing_by_node.get(source_instance, ()))
        incoming = tuple(incoming_by_node.get(source_instance, ()))
        if len(outgoing) < 2 or len(incoming) < 2:
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
        if len(edge_keys) == 0:
            return
        if len(edge_keys) != len(set(edge_keys)):
            return
        affected_requirement_edge_keys = tuple(
            sorted(
                _dedupe_edge_tuple(affected_requirement_edge_keys),
                key=_edge_sort_key,
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
        ] = [
            (
                (edge_key,),
                (edge_key,),
                requested,
                "direct_edge",
                0,
            )
        ]

        source_instance = edge_key.source.instance
        source_outgoing = outgoing_by_node.get(source_instance, [])
        source_incoming = incoming_by_node.get(source_instance, [])
        if source_instance.startswith(common_mode_prefixes) and len(source_incoming) >= 2:
            common_delay = min(
                missing_by_edge.get(outgoing_edge, 0.0)
                for outgoing_edge in source_outgoing
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
            (
                current_edges,
                affected_edges,
                extra_length_um,
                reason,
                depth,
            ) = queue.pop(0)
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
                source = current_edge.source.instance
                incoming = incoming_by_node.get(source, [])
                outgoing = outgoing_by_node.get(source, [])
                if (
                    not source.startswith(transparent_prefixes)
                    or len(incoming) != 1
                    or len(outgoing) != 1
                    or outgoing[0] != current_edge
                ):
                    continue
                next_edges = tuple(
                    incoming[0] if i == index else edge
                    for i, edge in enumerate(current_edges)
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
        inserted_by_edge[edge_id] = inserted_by_edge.get(edge_id, 0.0) + float(
            item.get("inserted_extra_length_um", 0.0)
        )
        status = str(item.get("status", "unknown"))
        previous_status = status_by_edge.get(edge_id)
        if previous_status is None or previous_status != "planned":
            status_by_edge[edge_id] = status
        unmatched_by_edge[edge_id] = unmatched_by_edge.get(edge_id, 0.0) + float(
            item.get("unmatched_length_um", 0.0)
        )

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


def output_matching_diagnostics_to_info(
    output_matching_info: dict[str, object] | None,
    meander_report: dict[str, object] | None,
    *,
    tolerance_um: float = PATH_LENGTH_MATCH_TOLERANCE_UM,
) -> list[dict[str, object]]:
    """Build acceptance diagnostics for optional global output matching."""
    if not output_matching_info or not bool(output_matching_info.get("enabled", False)):
        return []

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
        inserted_by_edge[edge_id] = inserted_by_edge.get(edge_id, 0.0) + float(
            item.get("inserted_extra_length_um", 0.0)
        )
        status = str(item.get("status", "unknown"))
        previous_status = status_by_edge.get(edge_id)
        if previous_status is None or previous_status != "planned":
            status_by_edge[edge_id] = status
        unmatched_by_edge[edge_id] = unmatched_by_edge.get(edge_id, 0.0) + float(
            item.get("unmatched_length_um", 0.0)
        )

    raw_outputs = output_matching_info.get("outputs", [])
    if not isinstance(raw_outputs, list):
        raw_outputs = []

    edges: list[dict[str, object]] = []
    max_unmatched = 0.0
    max_physical_residual = 0.0
    max_disregarded = 0.0
    for raw_output in raw_outputs:
        if not isinstance(raw_output, dict):
            continue
        missing = _object_to_float(raw_output.get("missing_length_um", 0.0))
        if missing <= tolerance_um:
            continue
        edge_info = raw_output.get("edge", {})
        edge_id = _edge_identity_from_info(edge_info)
        inserted = inserted_by_edge.get(edge_id, 0.0) if edge_id is not None else 0.0
        physical_residual = max(0.0, missing - inserted)
        status = (
            status_by_edge.get(edge_id, "missing")
            if edge_id is not None
            else str(raw_output.get("status", "missing"))
        )
        accepted_unmatched = (
            max(0.0, unmatched_by_edge.get(edge_id, physical_residual))
            if edge_id is not None
            else physical_residual
        )
        disregarded = physical_residual if status == "below_minimum_bump" else 0.0
        if status == "below_minimum_bump":
            accepted_unmatched = 0.0
        max_unmatched = max(max_unmatched, accepted_unmatched)
        max_physical_residual = max(max_physical_residual, physical_residual)
        max_disregarded = max(max_disregarded, disregarded)
        edges.append(
            {
                "edge": edge_info,
                "node_name": raw_output.get("node_name", ""),
                "arrival_um": _object_to_float(raw_output.get("arrival_um", 0.0)),
                "adjusted_missing_length_um": missing,
                "missing_length_um": missing,
                "inserted_extra_length_um": inserted,
                "physical_residual_um": physical_residual,
                "accepted_unmatched_um": accepted_unmatched,
                "disregarded_residual_um": disregarded,
                "meander_status": status,
            }
        )

    if not edges:
        return []

    return [
        {
            "node_name": "output_arrivals",
            "node_type": "output",
            "incoming_count": len(edges),
            "target_input_arrival_um": _object_to_float(
                output_matching_info.get("target_output_arrival_um", 0.0)
            ),
            "target_lift_um": 0.0,
            "tolerance_um": float(tolerance_um),
            "max_accepted_unmatched_um": float(max_unmatched),
            "max_physical_residual_um": float(max_physical_residual),
            "max_disregarded_residual_um": float(max_disregarded),
            "within_tolerance": bool(max_unmatched <= tolerance_um),
            "has_disregarded_residual": bool(max_disregarded > tolerance_um),
            "incoming_edges": edges,
        }
    ]


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
