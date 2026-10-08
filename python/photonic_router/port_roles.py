"""Fan-in and fan-out of the generic components: which ports light enters by, which it leaves by.

The path-length graph (`path_length_graph.py`) is directed, and a net alone
has no direction: a waveguide carries light either way, and a `Net` is a pair
of ports in whatever order its author wrote them. The direction comes from
the components, exactly as a logic network's comes from its gates' fan-in and
fan-out: every component type declares its input-side ports, every other
optical port is on its output side, and a net points from an output-side
port to an input-side port whichever way it was written. A component with
one optical port (a grating or edge coupler) has no side of its own; its net
takes the direction from the other end. A component missing from the table
gives its ports no role; a net with no role at either end keeps its written
order and the graph lists it (`PhotonicRoutingGraph.undecided_nets`).

The table follows the generic PDK's port numbering (``o1``, ``o2`` on the
input side of a 2x2, ``o3``, ``o4`` on the output side; ``o1`` in and ``o2``
out of a two-port). A new component the benchmarks connect nets to gets an
entry here.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from typing import Any

INPUT = "input"
OUTPUT = "output"

#: ``(instance, port) -> "input" | "output" | None``
PortRole = Callable[[str, str], str | None]

#: component -> its input-side ports; the other optical ports are its output side.
#: A one-port coupler lists no ports: its single port has no side.
INPUT_PORTS_BY_COMPONENT: dict[str, frozenset[str]] = {
    "mmi2x2": frozenset({"o1", "o2"}),
    "mmi2x2_with_sbend": frozenset({"o1", "o2"}),
    "coupler": frozenset({"o1", "o2"}),
    "mmi1x2": frozenset({"o1"}),
    "mzi": frozenset({"o1"}),
    "straight": frozenset({"o1"}),
    "straight_heater_metal": frozenset({"o1"}),
    "straight_heater_metal_undercut": frozenset({"o1"}),
    "arm_pair": frozenset({"o1", "o2"}),
    "ring_single": frozenset({"o1"}),
    "ring_double": frozenset({"o1", "o3"}),
    "crossing": frozenset({"o1", "o2"}),
    "grating_coupler_te": frozenset(),
    "grating_coupler_elliptical_trenches": frozenset(),
    "grating_coupler_elliptical_lumerical": frozenset(),
    "edge_coupler_silicon": frozenset(),
}

#: component -> its optical ports, for the output side of the components above
OPTICAL_PORTS_BY_COMPONENT: dict[str, frozenset[str]] = {
    "mmi2x2": frozenset({"o1", "o2", "o3", "o4"}),
    "mmi2x2_with_sbend": frozenset({"o1", "o2", "o3", "o4"}),
    "coupler": frozenset({"o1", "o2", "o3", "o4"}),
    "mmi1x2": frozenset({"o1", "o2", "o3"}),
    "mzi": frozenset({"o1", "o2"}),
    "straight": frozenset({"o1", "o2"}),
    "straight_heater_metal": frozenset({"o1", "o2"}),
    "straight_heater_metal_undercut": frozenset({"o1", "o2"}),
    "arm_pair": frozenset({"o1", "o2", "o3", "o4"}),
    "ring_single": frozenset({"o1", "o2"}),
    "ring_double": frozenset({"o1", "o2", "o3", "o4"}),
    "crossing": frozenset({"o1", "o2", "o3", "o4"}),
    "grating_coupler_te": frozenset({"o1"}),
    "grating_coupler_elliptical_trenches": frozenset({"o1"}),
    "grating_coupler_elliptical_lumerical": frozenset({"o1"}),
    "edge_coupler_silicon": frozenset({"o1"}),
}


def component_port_role(
    component: str,
    port: str,
    declared_input_ports: Mapping[str, Iterable[str]] | None = None,
) -> str | None:
    """The role of one port of a component type, or None when nothing knows it.

    ``declared_input_ports`` is a benchmark's own fan-in declaration for its
    cells (``INPUT_PORTS_BY_COMPONENT`` in its module): every other port of
    such a cell is on its output side; a cell declared with no input ports
    (a coupler) gives its ports no role. It takes precedence over the table.
    """

    if declared_input_ports is not None and component in declared_input_ports:
        inputs = frozenset(declared_input_ports[component])
        if not inputs:
            return None
        return INPUT if port in inputs else OUTPUT
    inputs = INPUT_PORTS_BY_COMPONENT.get(component)
    if inputs is None:
        return None
    if port in inputs:
        return INPUT
    if port in OPTICAL_PORTS_BY_COMPONENT.get(component, frozenset()) and inputs:
        return OUTPUT
    return None


def component_port_sides(
    component: str,
    declared_input_ports: Mapping[str, Iterable[str]] | None = None,
) -> tuple[frozenset[str], frozenset[str]] | None:
    """A component's two sides, ``(the side the table calls input, the other side)``.

    Which side the light enters by is decided per instance by where the
    light arrives (`path_length_graph.build_graph_from_schematic`): a mesh
    may fold back and run an MMI against its numbering. None for a one-port
    coupler or a component nothing knows.
    """

    if declared_input_ports is not None and component in declared_input_ports:
        inputs = frozenset(declared_input_ports[component])
        if not inputs:
            return None
        return inputs, frozenset()  # every other port is the other side
    inputs = INPUT_PORTS_BY_COMPONENT.get(component)
    if not inputs:
        return None
    return inputs, OPTICAL_PORTS_BY_COMPONENT.get(component, frozenset()) - inputs


def no_port_roles(instance: str, port: str) -> str | None:
    """The resolver that knows nothing: every net keeps its written direction."""

    return None


def generic_port_roles(
    schematic: Any, input_ports_by_component: Mapping[str, Iterable[str]] | None = None
) -> PortRole:
    """The port roles of ``schematic``'s instances from their component types,
    the benchmark's own fan-in declaration first."""

    component_of: dict[str, str] = {}
    instances = getattr(getattr(schematic, "netlist", None), "instances", None) or {}
    for name, instance in instances.items():
        component = getattr(instance, "component", None)
        if isinstance(component, str):
            component_of[str(name)] = component

    def role(instance: str, port: str) -> str | None:
        component = component_of.get(instance)
        if component is None:
            return None
        return component_port_role(component, port, input_ports_by_component)

    return role
