"""Directed graph model and path-length balancing analysis for routed designs."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from graphlib import TopologicalSorter
from typing import Any, Iterable, Mapping, Protocol

from photonic_router.port_roles import INPUT, OUTPUT, PortRole, generic_port_roles


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
    """One net. ``key`` names it as routed (the routed record's source and
    target, what requirements and meander candidates are matched by);
    ``source`` and ``target`` are the light's direction, which
    `build_graph_from_schematic` derives from the components' fan-in and
    fan-out, so the two pairs may be swapped."""

    key: RoutedEdgeKey
    source: PortRef
    target: PortRef


@dataclass
class MissingLengthRequirement:
    edge_key: RoutedEdgeKey
    missing_length_um: float


@dataclass(frozen=True)
class DelayInsertionCandidate:
    requirement_edge_key: RoutedEdgeKey
    edge_keys: tuple[RoutedEdgeKey, ...]
    extra_length_um: float
    reason: str = ""
    affected_requirement_edge_keys: tuple[RoutedEdgeKey, ...] = ()


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
class PathLengthGraphAnnotations:
    """PLM-owned annotations for a structural routing graph."""

    edge_lengths_um: dict[RoutedEdgeKey, float] = field(default_factory=dict)
    edge_missing_lengths_um: dict[RoutedEdgeKey, float] = field(default_factory=dict)

    @classmethod
    def from_edge_lengths(
        cls,
        graph: "PhotonicRoutingGraph",
        lengths_um: Mapping[RoutedEdgeKey, float],
    ) -> "PathLengthGraphAnnotations":
        annotations = cls()
        annotations.set_edge_lengths(graph, lengths_um)
        return annotations

    def set_edge_lengths(
        self,
        graph: "PhotonicRoutingGraph",
        lengths_um: Mapping[RoutedEdgeKey, float],
    ) -> None:
        for edge_key, length_um in lengths_um.items():
            if edge_key not in graph.edges:
                raise KeyError(
                    f"Unknown routed edge key: {edge_key.net_name} "
                    f"{edge_key.source.instance},{edge_key.source.port} -> "
                    f"{edge_key.target.instance},{edge_key.target.port}"
                )
            self.edge_lengths_um[edge_key] = float(length_um)

    def analyze_missing_lengths(
        self,
        graph: "PhotonicRoutingGraph",
        *,
        tolerance_um: float = 1.0e-9,
    ) -> PathLengthAnalysisResult:
        order = graph.topological_order()
        input_arrivals: dict[str, float] = {}
        output_arrivals: dict[str, float] = {}
        edge_missing: dict[RoutedEdgeKey, float] = {}
        node_timings: dict[str, NodeTiming] = {}
        requirements: list[MissingLengthRequirement] = []

        for node_name in order:
            node = graph.nodes[node_name]
            in_edges = graph.incoming_edges.get(node_name, [])
            if not in_edges:
                input_arrival = 0.0
                incoming_timings: list[NodeIncomingEdgeTiming] = []
            else:
                incoming_arrivals: dict[RoutedEdgeKey, float] = {}
                for edge_key in in_edges:
                    edge = graph.edges[edge_key]
                    src_arrival = output_arrivals.get(edge.source.instance, 0.0)
                    routed_length_um = self.edge_lengths_um.get(edge_key)
                    if routed_length_um is None:
                        raise ValueError(
                            f"Missing routed length for edge {edge.key.net_name} "
                            f"{edge.source.instance},{edge.source.port} -> "
                            f"{edge.target.instance},{edge.target.port}"
                        )
                    edge_arrival = src_arrival + routed_length_um
                    incoming_arrivals[edge_key] = edge_arrival

                input_arrival = max(incoming_arrivals.values())
                incoming_timings = []
                for edge_key, arrival in incoming_arrivals.items():
                    missing = max(0.0, input_arrival - arrival)
                    if missing <= tolerance_um:
                        missing = 0.0
                    edge_missing[edge_key] = missing
                    routed_length_um = self.edge_lengths_um.get(edge_key)
                    if routed_length_um is None:
                        raise ValueError(f"Missing routed length for edge {edge_key.net_name}")
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

        self.edge_missing_lengths_um = edge_missing
        return PathLengthAnalysisResult(
            topological_order=order,
            node_arrival_um=output_arrivals,
            node_arrival_input_um=input_arrivals,
            node_arrival_output_um=output_arrivals,
            node_timings=node_timings,
            edge_missing_lengths_um=edge_missing,
            requirements=requirements,
        )


@dataclass
class PhotonicRoutingGraph:
    nodes: dict[str, GraphNode] = field(default_factory=dict)
    edges: dict[RoutedEdgeKey, GraphEdge] = field(default_factory=dict)
    incoming_edges: dict[str, list[RoutedEdgeKey]] = field(default_factory=dict)
    outgoing_edges: dict[str, list[RoutedEdgeKey]] = field(default_factory=dict)
    #: nets with no fan-in or fan-out role at either end (two couplers, a
    #: component the table does not know): they keep their written order
    undecided_nets: list[str] = field(default_factory=list)

    def topological_order(self) -> list[str]:
        sorter = TopologicalSorter()
        for node_name in self.nodes:
            sorter.add(node_name)
        for edge in self.edges.values():
            sorter.add(edge.target.instance, edge.source.instance)
        return list(sorter.static_order())


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
    port_role: PortRole | None = None,
) -> PhotonicRoutingGraph:
    """The directed graph of a schematic: a node per instance, an edge per net.

    A net carries no direction of its own, so each one is pointed along the
    light from the components' fan-in and fan-out (`port_roles`: a net runs
    from an output-side port to an input-side port) whichever way it was
    written. The benchmark's labels come first: a node of type ``input`` is
    always a source and one of type ``output`` always a target. A net with
    no role at either end keeps its written order and is listed in
    ``undecided_nets``. ``port_role`` replaces the generic table (tests).
    """

    graph = PhotonicRoutingGraph()
    internal_delays_um = internal_delays_um or {}
    role = generic_port_roles(schematic) if port_role is None else port_role

    def ensure_node(name: str) -> None:
        if name in graph.nodes:
            return
        graph.nodes[name] = GraphNode(
            name=name,
            node_type=_resolve_node_type(name, node_types),
            internal_delay_um=float(internal_delays_um.get(name, 0.0)),
        )
        graph.incoming_edges.setdefault(name, [])
        graph.outgoing_edges.setdefault(name, [])

    for instance_name in schematic.netlist.instances:
        ensure_node(instance_name)

    for net_name, bundle in schematic.netlist.routes.items():
        for src_spec, dst_spec in bundle.links.items():
            declared_src = _parse_port_ref(src_spec)
            declared_dst = _parse_port_ref(dst_spec)
            ensure_node(declared_src.instance)
            ensure_node(declared_dst.instance)
            oriented = _oriented_along_the_light(
                declared_src,
                declared_dst,
                graph.nodes[declared_src.instance].node_type,
                graph.nodes[declared_dst.instance].node_type,
                role,
            )
            if oriented is None:
                graph.undecided_nets.append(net_name)
                src, dst = declared_src, declared_dst
            else:
                src, dst = oriented
            graph.nodes[src.instance].ports.setdefault(
                src.port,
                PortMetadata(name=src.port, direction=PortDirection.OUTPUT),
            )
            graph.nodes[dst.instance].ports.setdefault(
                dst.port,
                PortMetadata(name=dst.port, direction=PortDirection.INPUT),
            )
            key = RoutedEdgeKey(net_name=net_name, source=declared_src, target=declared_dst)
            graph.edges[key] = GraphEdge(key=key, source=src, target=dst)
            graph.outgoing_edges[src.instance].append(key)
            graph.incoming_edges[dst.instance].append(key)

    return graph


def _oriented_along_the_light(
    src: PortRef,
    dst: PortRef,
    src_type: NodeType,
    dst_type: NodeType,
    role: PortRole,
) -> tuple[PortRef, PortRef] | None:
    """``(source, target)`` along the light, or None when nothing decides it."""

    if src_type == NodeType.INPUT or dst_type == NodeType.OUTPUT:
        return src, dst
    if dst_type == NodeType.INPUT or src_type == NodeType.OUTPUT:
        return dst, src
    src_role = role(src.instance, src.port)
    dst_role = role(dst.instance, dst.port)
    if src_role == dst_role:
        return None  # no role at either end, or two input sides / two output sides
    if src_role == OUTPUT or dst_role == INPUT:
        return src, dst
    return dst, src


def annotate_edge_lengths(
    graph: PhotonicRoutingGraph,
    lengths_um: Mapping[RoutedEdgeKey, float],
) -> PathLengthGraphAnnotations:
    return PathLengthGraphAnnotations.from_edge_lengths(graph, lengths_um)


def list_edges_requiring_meander(
    analysis: PathLengthAnalysisResult,
) -> Iterable[MissingLengthRequirement]:
    return analysis.requirements
