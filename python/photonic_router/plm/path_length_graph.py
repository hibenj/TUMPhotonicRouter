"""Directed graph model and path-length balancing analysis for routed designs."""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from graphlib import TopologicalSorter
from typing import Any, Protocol

from photonic_router.port_roles import INPUT, OUTPUT, PortRole, component_port_sides


class SchematicLike(Protocol):
    netlist: Any


class NodeType(str, Enum):
    INPUT = "input"
    OUTPUT = "output"
    #: a dead end (an end cap, an open end): light only ever goes into it,
    #: like an output, but it is not one of the circuit's outputs, so output
    #: matching leaves it alone
    SINK = "sink"
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
    #: the instance the light comes from on this edge: the oriented source,
    #: which is the key's target when the net was written against the light
    source_instance: str = ""


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
    #: every edge's ends along the light, by its key (the key names the net
    #: as routed; a net written against the light has them swapped)
    edge_ends: dict[RoutedEdgeKey, tuple[PortRef, PortRef]] = field(default_factory=dict)

    def source_instance(self, edge_key: RoutedEdgeKey) -> str:
        ends = self.edge_ends.get(edge_key)
        return ends[0].instance if ends is not None else edge_key.source.instance

    def target_instance(self, edge_key: RoutedEdgeKey) -> str:
        ends = self.edge_ends.get(edge_key)
        return ends[1].instance if ends is not None else edge_key.target.instance


@dataclass
class PathLengthGraphAnnotations:
    """PLM-owned annotations for a structural routing graph."""

    edge_lengths_um: dict[RoutedEdgeKey, float] = field(default_factory=dict)
    edge_missing_lengths_um: dict[RoutedEdgeKey, float] = field(default_factory=dict)

    @classmethod
    def from_edge_lengths(
        cls,
        graph: PhotonicRoutingGraph,
        lengths_um: Mapping[RoutedEdgeKey, float],
    ) -> PathLengthGraphAnnotations:
        annotations = cls()
        annotations.set_edge_lengths(graph, lengths_um)
        return annotations

    def set_edge_lengths(
        self,
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
            self.edge_lengths_um[edge_key] = float(length_um)

    def analyze_missing_lengths(
        self,
        graph: PhotonicRoutingGraph,
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
                            source_instance=graph.edges[edge_key].source.instance,
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
            edge_ends={key: (edge.source, edge.target) for key, edge in graph.edges.items()},
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
    #: components the light reached from both sides: a benchmark error
    conflicting_nodes: list[str] = field(default_factory=list)

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
    input_ports_by_component: Mapping[str, Iterable[str]] | None = None,
) -> PhotonicRoutingGraph:
    """The directed graph of a schematic: a node per instance, an edge per net.

    A net carries no direction of its own, so the direction is propagated
    from the benchmark's inputs: a node of type ``input`` sends light out of
    every port; where the light arrives at a component, the ports on that
    side are its inputs and the other side its outputs (`port_roles`
    knows each component's two sides, the benchmark adds its own cells
    with ``INPUT_PORTS_BY_COMPONENT``), and the light goes on through the
    outputs; an ``output`` or ``sink`` node ends it. A folded mesh that
    runs an MMI against its port numbering is oriented right this way. A
    component with sides that no input reaches takes the table's side as
    its input side; a component light reaches from both sides is listed in
    ``conflicting_nodes`` and keeps the first arrival. A net with no role
    at either end keeps its written order and is listed in
    ``undecided_nets``. ``port_role`` replaces all of this (tests).
    """

    graph = PhotonicRoutingGraph()
    internal_delays_um = internal_delays_um or {}

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

    links: list[tuple[str, PortRef, PortRef]] = []
    for net_name, bundle in schematic.netlist.routes.items():
        for src_spec, dst_spec in bundle.links.items():
            declared_src = _parse_port_ref(src_spec)
            declared_dst = _parse_port_ref(dst_spec)
            ensure_node(declared_src.instance)
            ensure_node(declared_dst.instance)
            links.append((net_name, declared_src, declared_dst))

    if port_role is None:
        roles, graph.conflicting_nodes = _propagate_roles(
            schematic, graph, links, input_ports_by_component
        )

        def role(instance: str, port: str) -> str | None:
            return roles.get((instance, port))

    else:
        role = port_role

    for net_name, declared_src, declared_dst in links:
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


def _propagate_roles(
    schematic: SchematicLike,
    graph: PhotonicRoutingGraph,
    links: list[tuple[str, PortRef, PortRef]],
    input_ports_by_component: Mapping[str, Iterable[str]] | None,
) -> tuple[dict[tuple[str, str], str], list[str]]:
    """``(instance, port) -> role`` from the light's propagation out of the inputs."""

    component_of: dict[str, str] = {}
    for name, instance in schematic.netlist.instances.items():
        component = getattr(instance, "component", None)
        if isinstance(component, str):
            component_of[str(name)] = component
    sides_of: dict[str, tuple[frozenset[str], frozenset[str]] | None] = {
        name: component_port_sides(component, input_ports_by_component)
        for name, component in component_of.items()
    }
    neighbours: dict[tuple[str, str], list[tuple[str, str]]] = {}
    for _net_name, a, b in links:
        neighbours.setdefault((a.instance, a.port), []).append((b.instance, b.port))
        neighbours.setdefault((b.instance, b.port), []).append((a.instance, a.port))

    roles: dict[tuple[str, str], str] = {}
    entered_by: dict[str, str] = {}  # instance -> the port the light first arrived at
    conflicts: list[str] = []

    def assign(instance: str, input_side: frozenset[str], output_side: frozenset[str]) -> None:
        for port in input_side:
            roles[(instance, port)] = INPUT
        for port in output_side:
            roles[(instance, port)] = OUTPUT
        for (name, port), _targets in neighbours.items():
            # a declared cell lists only its input side: every other port is an output
            if name == instance and (name, port) not in roles:
                roles[(name, port)] = OUTPUT

    queue: deque[tuple[str, str]] = deque()
    for name, node in graph.nodes.items():
        if node.node_type != NodeType.INPUT:
            continue
        for (instance, port), targets in neighbours.items():
            if instance == name:
                roles[(instance, port)] = OUTPUT
                queue.extend(targets)
    while queue:
        instance, port = queue.popleft()
        node_type = graph.nodes[instance].node_type
        if node_type in (NodeType.OUTPUT, NodeType.SINK, NodeType.INPUT):
            continue
        sides = sides_of.get(instance)
        if sides is None:
            continue  # a coupler, or a cell nothing knows: the light ends here
        if instance in entered_by:
            if roles.get((instance, port)) != INPUT:
                conflicts.append(instance)
            continue
        entered_by[instance] = port
        side_a, side_b = sides
        if port in side_a:
            input_side, output_side = side_a, side_b
        elif port in side_b or not side_b:
            input_side, output_side = side_b or frozenset(), side_a
            if not side_b:  # a declared cell entered on its other side
                input_side = frozenset({port})
        else:
            continue  # an electrical or unknown port
        assign(instance, input_side, output_side)
        for (name, out_port), targets in neighbours.items():
            if name == instance and roles.get((name, out_port)) == OUTPUT:
                queue.extend(targets)

    # components no input reaches: the table's side is the input side
    for instance, sides in sides_of.items():
        if sides is None or instance in entered_by:
            continue
        side_a, side_b = sides
        assign(instance, side_a, side_b)
    return roles, conflicts


def _oriented_along_the_light(
    src: PortRef,
    dst: PortRef,
    src_type: NodeType,
    dst_type: NodeType,
    role: PortRole,
) -> tuple[PortRef, PortRef] | None:
    """``(source, target)`` along the light, or None when nothing decides it."""

    if src_type == NodeType.INPUT or dst_type in (NodeType.OUTPUT, NodeType.SINK):
        return src, dst
    if dst_type == NodeType.INPUT or src_type in (NodeType.OUTPUT, NodeType.SINK):
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
