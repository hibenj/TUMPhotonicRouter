"""Directed graph model and path-length balancing analysis for routed designs."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from graphlib import TopologicalSorter
from typing import Any, Iterable, Mapping, Protocol

class SchematicLike(Protocol):
    netlist: Any


class NodeType(str, Enum):
    INPUT = "input"
    OUTPUT = "output"
    GATE = "gate"
    SPLITTER = "splitter"
    BRANCH = "branch"


class PortDirection(str, Enum):
    INPUT = "input"
    OUTPUT = "output"


@dataclass(frozen=True)
class PortRef:
    instance: str
    port: str


@dataclass(frozen=True)
class RoutedEdgeKey:
    net_name: str
    source: PortRef
    target: PortRef


@dataclass
class PortMetadata:
    name: str
    direction: PortDirection


@dataclass
class GraphNode:
    name: str
    node_type: NodeType
    internal_delay_um: float = 0.0
    ports: dict[str, PortMetadata] = field(default_factory=dict)


@dataclass
class GraphEdge:
    key: RoutedEdgeKey
    source: PortRef
    target: PortRef
    routed_length_um: float | None = None
    required_extra_length_um: float = 0.0


@dataclass
class MissingLengthRequirement:
    edge_key: RoutedEdgeKey
    missing_length_um: float


@dataclass
class NodeIncomingEdgeTiming:
    edge_key: RoutedEdgeKey
    routed_length_um: float
    edge_arrival_um: float
    missing_length_um: float


@dataclass
class NodeTiming:
    node_name: str
    node_type: NodeType
    internal_delay_um: float
    input_arrival_um: float
    output_arrival_um: float
    incoming_edges: list[NodeIncomingEdgeTiming]


@dataclass
class PathLengthAnalysisResult:
    topological_order: list[str]
    node_arrival_um: dict[str, float]
    node_arrival_input_um: dict[str, float]
    node_arrival_output_um: dict[str, float]
    node_timings: dict[str, NodeTiming]
    edge_missing_lengths_um: dict[RoutedEdgeKey, float]
    requirements: list[MissingLengthRequirement]


@dataclass
class PhotonicRoutingGraph:
    nodes: dict[str, GraphNode] = field(default_factory=dict)
    edges: dict[RoutedEdgeKey, GraphEdge] = field(default_factory=dict)
    incoming_edges: dict[str, list[RoutedEdgeKey]] = field(default_factory=dict)
    outgoing_edges: dict[str, list[RoutedEdgeKey]] = field(default_factory=dict)

    def topological_order(self) -> list[str]:
        sorter = TopologicalSorter()
        for node_name in self.nodes:
            sorter.add(node_name)
        for edge in self.edges.values():
            sorter.add(edge.target.instance, edge.source.instance)
        return list(sorter.static_order())

    def analyze_missing_lengths(
        self, *, tolerance_um: float = 1.0e-9
    ) -> PathLengthAnalysisResult:
        order = self.topological_order()
        input_arrivals: dict[str, float] = {}
        output_arrivals: dict[str, float] = {}
        edge_missing: dict[RoutedEdgeKey, float] = {}
        node_timings: dict[str, NodeTiming] = {}
        requirements: list[MissingLengthRequirement] = []

        for node_name in order:
            node = self.nodes[node_name]
            in_edges = self.incoming_edges.get(node_name, [])
            if not in_edges:
                input_arrival = 0.0
                incoming_timings: list[NodeIncomingEdgeTiming] = []
            else:
                incoming_arrivals: dict[RoutedEdgeKey, float] = {}
                for edge_key in in_edges:
                    edge = self.edges[edge_key]
                    src_arrival = output_arrivals.get(edge.source.instance, 0.0)
                    if edge.routed_length_um is None:
                        raise ValueError(
                            f"Missing routed length for edge {edge.key.net_name} "
                            f"{edge.source.instance},{edge.source.port} -> "
                            f"{edge.target.instance},{edge.target.port}"
                        )
                    edge_arrival = src_arrival + edge.routed_length_um
                    incoming_arrivals[edge_key] = edge_arrival

                input_arrival = max(incoming_arrivals.values())
                incoming_timings = []
                for edge_key, arrival in incoming_arrivals.items():
                    missing = max(0.0, input_arrival - arrival)
                    if missing <= tolerance_um:
                        missing = 0.0
                    self.edges[edge_key].required_extra_length_um = missing
                    edge_missing[edge_key] = missing
                    routed_length_um = self.edges[edge_key].routed_length_um
                    if routed_length_um is None:
                        raise ValueError(
                            f"Missing routed length for edge {edge_key.net_name}"
                        )
                    incoming_timings.append(
                        NodeIncomingEdgeTiming(
                            edge_key=edge_key,
                            routed_length_um=float(routed_length_um),
                            edge_arrival_um=arrival,
                            missing_length_um=missing,
                        )
                    )
                    if missing > 0.0:
                        requirements.append(
                            MissingLengthRequirement(
                                edge_key=edge_key,
                                missing_length_um=missing,
                            )
                        )

            output_arrival = input_arrival + float(node.internal_delay_um)
            input_arrivals[node_name] = input_arrival
            output_arrivals[node_name] = output_arrival
            node_timings[node_name] = NodeTiming(
                node_name=node.name,
                node_type=node.node_type,
                internal_delay_um=float(node.internal_delay_um),
                input_arrival_um=float(input_arrival),
                output_arrival_um=float(output_arrival),
                incoming_edges=incoming_timings,
            )

        return PathLengthAnalysisResult(
            topological_order=order,
            node_arrival_um=output_arrivals,
            node_arrival_input_um=input_arrivals,
            node_arrival_output_um=output_arrivals,
            node_timings=node_timings,
            edge_missing_lengths_um=edge_missing,
            requirements=requirements,
        )


def _parse_port_ref(spec: str) -> PortRef:
    instance, port = spec.split(",", maxsplit=1)
    return PortRef(instance=instance, port=port)


def _resolve_node_type(
    instance_name: str,
    overrides: Mapping[str, NodeType | str] | None,
) -> NodeType:
    if overrides is None or instance_name not in overrides:
        return NodeType.GATE
    raw = overrides[instance_name]
    if isinstance(raw, NodeType):
        return raw
    return NodeType(raw)


def build_graph_from_schematic(
    schematic: SchematicLike,
    *,
    node_types: Mapping[str, NodeType | str] | None = None,
    internal_delays_um: Mapping[str, float] | None = None,
) -> PhotonicRoutingGraph:
    graph = PhotonicRoutingGraph()
    internal_delays_um = internal_delays_um or {}

    for instance_name in schematic.netlist.instances.keys():
        graph.nodes[instance_name] = GraphNode(
            name=instance_name,
            node_type=_resolve_node_type(instance_name, node_types),
            internal_delay_um=float(internal_delays_um.get(instance_name, 0.0)),
        )
        graph.incoming_edges[instance_name] = []
        graph.outgoing_edges[instance_name] = []

    for net_name, bundle in schematic.netlist.routes.items():
        links = bundle.links
        for src_spec, dst_spec in links.items():
            src = _parse_port_ref(src_spec)
            dst = _parse_port_ref(dst_spec)

            if src.instance not in graph.nodes:
                graph.nodes[src.instance] = GraphNode(
                    name=src.instance,
                    node_type=_resolve_node_type(src.instance, node_types),
                    internal_delay_um=float(internal_delays_um.get(src.instance, 0.0)),
                )
                graph.incoming_edges.setdefault(src.instance, [])
                graph.outgoing_edges.setdefault(src.instance, [])
            if dst.instance not in graph.nodes:
                graph.nodes[dst.instance] = GraphNode(
                    name=dst.instance,
                    node_type=_resolve_node_type(dst.instance, node_types),
                    internal_delay_um=float(internal_delays_um.get(dst.instance, 0.0)),
                )
                graph.incoming_edges.setdefault(dst.instance, [])
                graph.outgoing_edges.setdefault(dst.instance, [])

            graph.nodes[src.instance].ports.setdefault(
                src.port,
                PortMetadata(name=src.port, direction=PortDirection.OUTPUT),
            )
            graph.nodes[dst.instance].ports.setdefault(
                dst.port,
                PortMetadata(name=dst.port, direction=PortDirection.INPUT),
            )

            key = RoutedEdgeKey(net_name=net_name, source=src, target=dst)
            edge = GraphEdge(key=key, source=src, target=dst)
            graph.edges[key] = edge
            graph.outgoing_edges[src.instance].append(key)
            graph.incoming_edges[dst.instance].append(key)

    return graph


def annotate_edge_lengths(
    graph: PhotonicRoutingGraph,
    lengths_um: Mapping[RoutedEdgeKey, float],
) -> None:
    for edge_key, length_um in lengths_um.items():
        if edge_key not in graph.edges:
            raise KeyError(
                f"Unknown routed edge key: {edge_key.net_name} "
                f"{edge_key.source.instance},{edge_key.source.port} -> "
                f"{edge_key.target.instance},{edge_key.target.port}"
            )
        graph.edges[edge_key].routed_length_um = float(length_um)


def list_edges_requiring_meander(
    analysis: PathLengthAnalysisResult,
) -> Iterable[MissingLengthRequirement]:
    return analysis.requirements
