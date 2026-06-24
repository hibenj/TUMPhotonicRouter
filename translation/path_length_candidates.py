"""Delay insertion candidate propagation for path-length matching."""

from __future__ import annotations

from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
    MissingLengthRequirement,
    PathLengthAnalysisResult,
    RoutedEdgeKey,
)

PATH_LENGTH_MATCH_TOLERANCE_UM = 1.0e-6


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
    incoming_by_node, outgoing_by_node = analysis_edge_maps(analysis)
    missing_by_edge = {
        req.edge_key: float(req.missing_length_um)
        for req in requirements
    }
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
