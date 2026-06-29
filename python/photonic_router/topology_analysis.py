"""Topology depth/rank analysis for crossing-aware routing planning."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol

from photonic_router.path_length_graph import (
    PhotonicRoutingGraph,
    PortRef,
    RoutedEdgeKey,
    build_graph_from_schematic,
)


class SchematicLike(Protocol):
    netlist: Any
    placements: Mapping[str, Any]


@dataclass(frozen=True)
class TopologyEdgeRank:
    edge_key: RoutedEdgeKey
    source_depth: int
    target_depth: int
    source_rank: int
    target_rank: int


@dataclass(frozen=True)
class TopologyCrossing:
    source_depth: int
    target_depth: int
    edge_a: RoutedEdgeKey
    edge_b: RoutedEdgeKey
    edge_a_source_rank: int
    edge_a_target_rank: int
    edge_b_source_rank: int
    edge_b_target_rank: int


@dataclass
class TopologyAnalysisResult:
    topological_order: list[str]
    node_depths: dict[str, int]
    node_ranks: dict[str, int]
    edge_ranks: dict[RoutedEdgeKey, TopologyEdgeRank]
    crossings: list[TopologyCrossing] = field(default_factory=list)

    def crossings_by_depth(self) -> dict[tuple[int, int], list[TopologyCrossing]]:
        grouped: dict[tuple[int, int], list[TopologyCrossing]] = {}
        for crossing in self.crossings:
            grouped.setdefault((crossing.source_depth, crossing.target_depth), []).append(
                crossing
            )
        return grouped


def analyze_schematic_topology(
    schematic: SchematicLike,
    *,
    node_depths: Mapping[str, int] | None = None,
    node_ranks: Mapping[str, int] | None = None,
    edge_ranks: Mapping[str, Mapping[str, int]] | None = None,
) -> TopologyAnalysisResult:
    """Analyze a schematic DAG and report rank inversions as crossing candidates.

    Explicit benchmark metadata is used when supplied. Missing edge ranks fall
    back to absolute placed port coordinates, which works for left-to-right
    DAG-like benchmarks with ordered lanes.
    """
    graph = build_graph_from_schematic(schematic)
    return analyze_graph_topology(
        graph,
        schematic=schematic,
        node_depths=node_depths,
        node_ranks=node_ranks,
        edge_ranks=edge_ranks,
    )


def analyze_graph_topology(
    graph: PhotonicRoutingGraph,
    *,
    schematic: SchematicLike,
    node_depths: Mapping[str, int] | None = None,
    node_ranks: Mapping[str, int] | None = None,
    edge_ranks: Mapping[str, Mapping[str, int]] | None = None,
) -> TopologyAnalysisResult:
    """Analyze topology annotations on an existing Python routing graph."""
    order = graph.topological_order()
    resolved_depths = (
        {str(name): int(depth) for name, depth in node_depths.items()}
        if node_depths is not None
        else _derive_node_depths(order, graph.incoming_edges, graph.edges)
    )
    resolved_ranks = (
        {str(name): int(rank) for name, rank in node_ranks.items()}
        if node_ranks is not None
        else _derive_node_ranks_from_placements(schematic, resolved_depths)
    )

    ranked_edges = _derive_edge_ranks(
        schematic,
        graph_edges=tuple(graph.edges.keys()),
        node_depths=resolved_depths,
        node_ranks=resolved_ranks,
        explicit_edge_ranks=edge_ranks or {},
    )
    crossings = _find_rank_inversions(ranked_edges)

    return TopologyAnalysisResult(
        topological_order=order,
        node_depths=resolved_depths,
        node_ranks=resolved_ranks,
        edge_ranks=ranked_edges,
        crossings=crossings,
    )


def _derive_node_depths(
    order: list[str],
    incoming_edges: Mapping[str, list[RoutedEdgeKey]],
    edges: Mapping[RoutedEdgeKey, Any],
) -> dict[str, int]:
    depths: dict[str, int] = {}
    for node_name in order:
        incoming = incoming_edges.get(node_name, [])
        if not incoming:
            depths[node_name] = 0
            continue
        depths[node_name] = 1 + max(
            depths.get(edges[edge_key].source.instance, 0)
            for edge_key in incoming
        )
    return depths


def _derive_node_ranks_from_placements(
    schematic: SchematicLike,
    node_depths: Mapping[str, int],
) -> dict[str, int]:
    by_depth: dict[int, list[tuple[float, float, str]]] = {}
    for node_name, depth in node_depths.items():
        placement = schematic.placements.get(node_name)
        x = float(getattr(placement, "x", 0.0) or 0.0)
        y = float(getattr(placement, "y", 0.0) or 0.0)
        by_depth.setdefault(int(depth), []).append((-y, x, node_name))

    ranks: dict[str, int] = {}
    for items in by_depth.values():
        for rank, (_neg_y, _x, node_name) in enumerate(sorted(items)):
            ranks[node_name] = rank
    return ranks


def _derive_edge_ranks(
    schematic: SchematicLike,
    *,
    graph_edges: tuple[RoutedEdgeKey, ...],
    node_depths: Mapping[str, int],
    node_ranks: Mapping[str, int],
    explicit_edge_ranks: Mapping[str, Mapping[str, int]],
) -> dict[RoutedEdgeKey, TopologyEdgeRank]:
    ranked_edges: dict[RoutedEdgeKey, TopologyEdgeRank] = {}
    unresolved_edges: list[RoutedEdgeKey] = []

    for edge_key in graph_edges:
        explicit = explicit_edge_ranks.get(edge_key.net_name)
        if explicit is None:
            unresolved_edges.append(edge_key)
            continue
        ranked_edges[edge_key] = TopologyEdgeRank(
            edge_key=edge_key,
            source_depth=int(explicit["source_depth"]),
            target_depth=int(explicit["target_depth"]),
            source_rank=int(explicit["source_rank"]),
            target_rank=int(explicit["target_rank"]),
        )

    if unresolved_edges:
        ranked_edges.update(
            _derive_missing_edge_ranks_from_ports(
                schematic,
                unresolved_edges=tuple(unresolved_edges),
                node_depths=node_depths,
                node_ranks=node_ranks,
            )
        )
    return ranked_edges


def _derive_missing_edge_ranks_from_ports(
    schematic: SchematicLike,
    *,
    unresolved_edges: tuple[RoutedEdgeKey, ...],
    node_depths: Mapping[str, int],
    node_ranks: Mapping[str, int],
) -> dict[RoutedEdgeKey, TopologyEdgeRank]:
    port_positions = _placed_port_positions(schematic)
    endpoint_records: dict[
        tuple[int, int],
        list[tuple[RoutedEdgeKey, tuple[float, float], tuple[float, float]]],
    ] = {}

    for edge_key in unresolved_edges:
        source_depth = int(node_depths[edge_key.source.instance])
        target_depth = int(node_depths[edge_key.target.instance])
        source_pos = port_positions.get((edge_key.source.instance, edge_key.source.port))
        target_pos = port_positions.get((edge_key.target.instance, edge_key.target.port))
        if source_pos is None:
            source_pos = _fallback_endpoint_position(
                schematic,
                edge_key.source,
                node_ranks,
            )
        if target_pos is None:
            target_pos = _fallback_endpoint_position(
                schematic,
                edge_key.target,
                node_ranks,
            )
        endpoint_records.setdefault((source_depth, target_depth), []).append(
            (edge_key, source_pos, target_pos)
        )

    ranked_edges: dict[RoutedEdgeKey, TopologyEdgeRank] = {}
    for (source_depth, target_depth), records in endpoint_records.items():
        source_order = _rank_endpoints(
            (edge_key, source_pos)
            for edge_key, source_pos, _target_pos in records
        )
        target_order = _rank_endpoints(
            (edge_key, target_pos)
            for edge_key, _source_pos, target_pos in records
        )
        for edge_key, _source_pos, _target_pos in records:
            ranked_edges[edge_key] = TopologyEdgeRank(
                edge_key=edge_key,
                source_depth=source_depth,
                target_depth=target_depth,
                source_rank=source_order[edge_key],
                target_rank=target_order[edge_key],
            )
    return ranked_edges


def _placed_port_positions(schematic: SchematicLike) -> dict[tuple[str, str], tuple[float, float]]:
    try:
        from translation.layout_from_schematic import layout_from_schematic

        layout = layout_from_schematic(schematic)  # type: ignore[arg-type]
    except Exception:
        return {}

    positions: dict[tuple[str, str], tuple[float, float]] = {}
    for inst in layout.insts:
        instance_name = str(inst.name)
        for port in inst.ports:
            center = getattr(port, "center", None)
            if center is None:
                continue
            positions[(instance_name, str(port.name))] = (
                float(center[0]),
                float(center[1]),
            )
    return positions


def _fallback_endpoint_position(
    schematic: SchematicLike,
    port_ref: PortRef,
    node_ranks: Mapping[str, int],
) -> tuple[float, float]:
    placement = schematic.placements.get(port_ref.instance)
    x = float(getattr(placement, "x", 0.0) or 0.0)
    y = float(getattr(placement, "y", 0.0) or 0.0)
    rank = float(node_ranks.get(port_ref.instance, 0))
    return (x, y - rank)


def _rank_endpoints(
    endpoints: Any,
) -> dict[RoutedEdgeKey, int]:
    ordered = sorted(
        (
            (-float(position[1]), float(position[0]), edge_key.net_name, edge_key)
            for edge_key, position in endpoints
        )
    )
    return {edge_key: rank for rank, (_neg_y, _x, _net_name, edge_key) in enumerate(ordered)}


def _find_rank_inversions(
    edge_ranks: Mapping[RoutedEdgeKey, TopologyEdgeRank],
) -> list[TopologyCrossing]:
    by_depth: dict[tuple[int, int], list[TopologyEdgeRank]] = {}
    for edge_rank in edge_ranks.values():
        by_depth.setdefault((edge_rank.source_depth, edge_rank.target_depth), []).append(
            edge_rank
        )

    crossings: list[TopologyCrossing] = []
    for (source_depth, target_depth), group in by_depth.items():
        ordered = sorted(group, key=lambda edge: (edge.source_rank, edge.target_rank))
        for index, edge_a in enumerate(ordered):
            for edge_b in ordered[index + 1 :]:
                source_delta = edge_a.source_rank - edge_b.source_rank
                target_delta = edge_a.target_rank - edge_b.target_rank
                if source_delta * target_delta >= 0:
                    continue
                crossings.append(
                    TopologyCrossing(
                        source_depth=source_depth,
                        target_depth=target_depth,
                        edge_a=edge_a.edge_key,
                        edge_b=edge_b.edge_key,
                        edge_a_source_rank=edge_a.source_rank,
                        edge_a_target_rank=edge_a.target_rank,
                        edge_b_source_rank=edge_b.source_rank,
                        edge_b_target_rank=edge_b.target_rank,
                    )
                )
    return crossings
