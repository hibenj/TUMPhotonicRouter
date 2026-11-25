"""Directed graph model and path-length balancing analysis for routed designs."""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from graphlib import TopologicalSorter

from photonic_router.port_roles import INPUT, OUTPUT, PortRole, component_port_sides
from photonic_router.records.route_keys import PortRef, RoutedEdgeKey
from photonic_router.schematic_protocols import SchematicLike


class NodeType(str, Enum):
    """What an instance is to the light, as the benchmark's node types say.

    ``input``, ``output`` and ``sink`` decide the direction of the nets at
    them (`build_graph_from_schematic`); the graph construction and the
    timing walk treat the other types alike. An instance with no declared
    type is a ``gate``.
    """

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
    """Whether the light enters (``input``) or leaves (``output``) a node at a port."""

    INPUT = "input"
    OUTPUT = "output"


@dataclass
class PortMetadata:
    """One connected port of a node and the direction the light takes through it."""

    name: str
    direction: PortDirection


@dataclass
class GraphNode:
    """One instance of the schematic.

    ``internal_delay_um`` is the path length the light travels inside the
    instance, added between its input and its output arrival. ``ports``
    holds only the ports a net connects, by port name.
    """

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
    """An edge and the extra length, in um, it needs.

    The timing analysis produces one for every edge that arrives before the
    latest edge into the same node; the group lifts and the output matching
    of `path_length_requirements` build theirs as the same record.
    """

    edge_key: RoutedEdgeKey
    missing_length_um: float


@dataclass(frozen=True)
class DelayInsertionCandidate:
    """One place where the extra length of a requirement may be inserted.

    ``requirement_edge_key`` is the edge whose requirement this serves.
    ``edge_keys`` are the edges that each take ``extra_length_um`` (um)
    when the candidate is used: the required edge itself, or edges
    upstream of it. ``affected_requirement_edge_keys`` are the required
    edges the insertion delays, more than one where an upstream insertion
    is shared. ``reason`` names the rule that produced the candidate
    (``direct_edge``, ``common_mode_upstream_bundle``, ...); see
    `path_length_requirements.build_requirement_delay_candidates`.
    """

    requirement_edge_key: RoutedEdgeKey
    edge_keys: tuple[RoutedEdgeKey, ...]
    extra_length_um: float
    reason: str = ""
    affected_requirement_edge_keys: tuple[RoutedEdgeKey, ...] = ()


@dataclass
class NodeIncomingEdgeTiming:
    """The timing of one edge into a node, all in um.

    ``edge_arrival_um`` is the source node's output arrival plus
    ``routed_length_um``; ``missing_length_um`` is how far that is behind
    the node's latest incoming edge (0.0 within the analysis tolerance).
    """

    edge_key: RoutedEdgeKey
    routed_length_um: float
    edge_arrival_um: float
    missing_length_um: float
    #: the instance the light comes from on this edge: the oriented source,
    #: which is the key's target when the net was written against the light
    source_instance: str = ""


@dataclass
class NodeTiming:
    """The arrival of the light at one node, as accumulated path length in um.

    ``input_arrival_um`` is the latest arrival over the incoming edges (0.0
    for a node without any); ``output_arrival_um`` adds
    ``internal_delay_um`` to it.
    """

    node_name: str
    node_type: NodeType
    internal_delay_um: float
    input_arrival_um: float
    output_arrival_um: float
    incoming_edges: list[NodeIncomingEdgeTiming]


@dataclass
class PathLengthAnalysisResult:
    """The result of `PathLengthGraphAnnotations.analyze_missing_lengths`.

    Arrivals are accumulated path lengths in um, by node name;
    ``node_arrival_um`` is the same mapping as ``node_arrival_output_um``.
    ``edge_missing_lengths_um`` has an entry for every edge of the graph,
    ``requirements`` only for those with a missing length above zero.
    """

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
        """The instance the light leaves on this edge.

        Falls back to the key's own source when ``edge_ends`` does not list
        the edge.
        """
        ends = self.edge_ends.get(edge_key)
        return ends[0].instance if ends is not None else edge_key.source.instance

    def target_instance(self, edge_key: RoutedEdgeKey) -> str:
        """The instance the light arrives at on this edge.

        Falls back to the key's own target when ``edge_ends`` does not list
        the edge.
        """
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
        """New annotations holding the given routed lengths (um) of the graph's edges.

        Raises ``KeyError`` for a key that is not an edge of ``graph``.
        """
        annotations = cls()
        annotations.set_edge_lengths(graph, lengths_um)
        return annotations

    def set_edge_lengths(
        self,
        graph: PhotonicRoutingGraph,
        lengths_um: Mapping[RoutedEdgeKey, float],
    ) -> None:
        """Record the routed length, in um, of the given edges.

        Updates ``edge_lengths_um`` in place; edges not named keep the
        length they had. Raises ``KeyError`` at the first key that is not
        an edge of ``graph``, with the edges before it already recorded.
        """
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
        """Walk the graph along the light and find what every edge is missing.

        In topological order, a node's input arrival is the latest of its
        incoming edges (the source's output arrival plus the edge's routed
        length; 0.0 for a node without incoming edges) and its output
        arrival adds the node's internal delay. An incoming edge is
        missing the difference to that latest arrival; a difference of at
        most ``tolerance_um`` counts as 0.0. All values are in um.

        Replaces ``edge_missing_lengths_um`` with the result. Raises
        ``ValueError`` when an edge has no recorded length, and
        ``graphlib.CycleError`` when the graph has a cycle.
        """
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
    """The schematic as a directed graph along the light.

    ``nodes`` is keyed by instance name, ``edges`` by the net's key as
    routed. ``incoming_edges`` and ``outgoing_edges`` list, per instance
    name, the keys of the edges whose light arrives at or leaves that
    node. `build_graph_from_schematic` fills all of it.
    """

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
        """The node names, each after every node its incoming light comes from.

        Raises ``graphlib.CycleError`` when the edges form a cycle.
        """
        sorter: TopologicalSorter[str] = TopologicalSorter()
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

    role: PortRole
    if port_role is None:
        roles, graph.conflicting_nodes = _propagate_roles(
            schematic, graph, links, input_ports_by_component
        )

        def propagated_role(instance: str, port: str) -> str | None:
            """The role the propagation from the inputs gave this port, None if it gave none."""
            return roles.get((instance, port))

        role = propagated_role
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


def _component_sides(
    schematic: SchematicLike,
    input_ports_by_component: Mapping[str, Iterable[str]] | None,
) -> dict[str, tuple[frozenset[str], frozenset[str]] | None]:
    """The two port sides of every instance whose component is named by a string.

    The value is what `component_port_sides` gives for the component, None
    for a component it does not know.
    """

    component_of: dict[str, str] = {}
    for name, instance in schematic.netlist.instances.items():
        component = getattr(instance, "component", None)
        if isinstance(component, str):
            component_of[str(name)] = component
    return {
        name: component_port_sides(component, input_ports_by_component)
        for name, component in component_of.items()
    }


def _port_neighbours(
    links: list[tuple[str, PortRef, PortRef]],
) -> dict[tuple[str, str], list[tuple[str, str]]]:
    """``(instance, port) -> the ports it is linked to``, both directions, in link order."""

    neighbours: dict[tuple[str, str], list[tuple[str, str]]] = {}
    for _net_name, a, b in links:
        neighbours.setdefault((a.instance, a.port), []).append((b.instance, b.port))
        neighbours.setdefault((b.instance, b.port), []).append((a.instance, a.port))
    return neighbours


class _LightFront:
    """The ports the light has reached so far, with the role each got."""

    def __init__(self, neighbours: Mapping[tuple[str, str], list[tuple[str, str]]]) -> None:
        self.neighbours = neighbours
        self.roles: dict[tuple[str, str], str] = {}
        #: instance -> the port the light first arrived at
        self.entered_by: dict[str, str] = {}
        #: the instances the light reached a second time on a port that is not an input
        self.conflicts: list[str] = []

    def assign(self, instance: str, input_side: Iterable[str], output_side: Iterable[str]) -> None:
        """Give the ports of an instance their roles: the two sides, every other linked port an output."""
        for port in input_side:
            self.roles[(instance, port)] = INPUT
        for port in output_side:
            self.roles[(instance, port)] = OUTPUT
        for name, port in self.neighbours:
            # a declared cell lists only its input side: every other port is an output
            if name == instance and (name, port) not in self.roles:
                self.roles[(name, port)] = OUTPUT

    def beyond_outputs(self, instance: str) -> list[tuple[str, str]]:
        """The ports the outputs of an instance are linked to."""
        beyond: list[tuple[str, str]] = []
        for (name, port), targets in self.neighbours.items():
            if name == instance and self.roles.get((name, port)) == OUTPUT:
                beyond.extend(targets)
        return beyond


def _sides_as_entered(
    sides: tuple[frozenset[str], frozenset[str]], port: str
) -> tuple[frozenset[str], frozenset[str]] | None:
    """``(input side, output side)`` of a component the light enters at ``port``.

    None for a port on neither side (an electrical or unknown port). A
    declared cell lists one side only; entered on another port, that port
    alone is its input side.
    """
    side_a, side_b = sides
    if port in side_a:
        return side_a, side_b
    if not side_b:
        return frozenset({port}), side_a
    if port in side_b:
        return side_b, side_a
    return None


def _beyond_the_inputs(graph: PhotonicRoutingGraph, front: _LightFront) -> deque[tuple[str, str]]:
    """Make every linked port of the input nodes an output; returns the ports they are linked to."""
    queue: deque[tuple[str, str]] = deque()
    for name, node in graph.nodes.items():
        if node.node_type != NodeType.INPUT:
            continue
        for (instance, port), targets in front.neighbours.items():
            if instance == name:
                front.roles[(instance, port)] = OUTPUT
                queue.extend(targets)
    return queue


def _propagate_roles(
    schematic: SchematicLike,
    graph: PhotonicRoutingGraph,
    links: list[tuple[str, PortRef, PortRef]],
    input_ports_by_component: Mapping[str, Iterable[str]] | None,
) -> tuple[dict[tuple[str, str], str], list[str]]:
    """``(instance, port) -> role`` from the light's propagation out of the inputs.

    The light starts at every port of the input nodes and passes each
    component from the side it enters to the other side; it ends at
    outputs, sinks and cells whose sides nothing knows. A component no
    input reaches takes the table's side as its input side. Also returns
    the instances the light reached a second time on a port that is not
    an input.
    """
    sides_of = _component_sides(schematic, input_ports_by_component)
    front = _LightFront(_port_neighbours(links))
    queue = _beyond_the_inputs(graph, front)
    while queue:
        instance, port = queue.popleft()
        sides = sides_of.get(instance)
        if (
            graph.nodes[instance].node_type in (NodeType.OUTPUT, NodeType.SINK, NodeType.INPUT)
            or sides is None  # a coupler, or a cell nothing knows: the light ends here
        ):
            continue
        if instance in front.entered_by:
            if front.roles.get((instance, port)) != INPUT:
                front.conflicts.append(instance)
            continue
        front.entered_by[instance] = port
        entered = _sides_as_entered(sides, port)
        if entered is not None:
            front.assign(instance, *entered)
            queue.extend(front.beyond_outputs(instance))

    # components no input reaches: the table's side is the input side
    for instance, sides in sides_of.items():
        if sides is not None and instance not in front.entered_by:
            front.assign(instance, *sides)
    return front.roles, front.conflicts


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
    """Annotations holding the routed lengths (um) of the graph's edges.

    The function form of `PathLengthGraphAnnotations.from_edge_lengths`;
    raises ``KeyError`` for a key that is not an edge of ``graph``.
    """
    return PathLengthGraphAnnotations.from_edge_lengths(graph, lengths_um)


def list_edges_requiring_meander(
    analysis: PathLengthAnalysisResult,
) -> Iterable[MissingLengthRequirement]:
    """The analysis's requirements: the edges with a missing length above zero.

    Returns the analysis's own list, not a copy.
    """
    return analysis.requirements
