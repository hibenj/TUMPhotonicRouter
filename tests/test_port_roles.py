"""The path-length graph orients every net along the light, whichever way the net was written.

Private-circuits plan, Milestone 4 (2026-10-07): the generated circuits declare
their nets in extraction order, so an output coupler on an MMI's o4 counted as
an input with a millimetres-long feed. The direction now comes from the port
roles of the generic components (input ports face 180 degrees, outputs 0) and
from the node types; a coupler's net follows the other end.
"""

from __future__ import annotations

import pytest
from gdsfactory.gpdk import get_generic_pdk
from gdsfactory.schematic import Instance, Net, Schematic
from photonic_router.path_length_graph import (
    PathLengthGraphAnnotations,
    PortRef,
    RoutedEdgeKey,
    build_graph_from_schematic,
)
from photonic_router.port_roles import component_port_roles, generic_port_roles, no_port_roles

pytestmark = pytest.mark.integration


def _mzi_schematic(reversed_nets: bool) -> Schematic:
    """Coupler -> MMI -> two arms (heater, straight) -> MMI -> coupler, with every
    net written either along the light or against it."""

    get_generic_pdk().activate()
    schematic = Schematic()
    schematic.add_instance("gc_in", Instance(component="grating_coupler_te"))
    schematic.add_instance("mmi_a", Instance(component="mmi2x2", settings={}))
    schematic.add_instance("heater", Instance(component="straight_heater_metal", settings={}))
    schematic.add_instance("arm", Instance(component="straight", settings={"length": 100.0}))
    schematic.add_instance("mmi_b", Instance(component="mmi2x2", settings={}))
    schematic.add_instance("gc_out", Instance(component="grating_coupler_te"))
    nets = [
        ("feed", "gc_in,o1", "mmi_a,o1"),
        ("upper_in", "mmi_a,o3", "heater,o1"),
        ("lower_in", "mmi_a,o4", "arm,o1"),
        ("upper_out", "heater,o2", "mmi_b,o2"),
        ("lower_out", "arm,o2", "mmi_b,o1"),
        ("drain", "mmi_b,o3", "gc_out,o1"),
    ]
    for name, p1, p2 in nets:
        if reversed_nets:
            p1, p2 = p2, p1
        schematic.add_net(Net(p1=p1, p2=p2, name=name))
    return schematic


def _flow(graph) -> dict[str, tuple[str, str]]:
    return {
        key.net_name: (edge.source.instance, edge.target.instance)
        for key, edge in graph.edges.items()
    }


def test_generic_components_expose_their_port_roles():
    get_generic_pdk().activate()
    assert component_port_roles("mmi2x2", {}) == {
        "o1": "input",
        "o2": "input",
        "o3": "output",
        "o4": "output",
    }
    assert component_port_roles("straight", {"length": 10.0}) == {"o1": "input", "o2": "output"}
    # one optical port: no role, the net follows the other end
    assert component_port_roles("grating_coupler_te", {}) == {}
    assert component_port_roles("no_such_component", {}) == {}


def test_every_net_flows_along_the_light_whichever_way_it_was_written():
    expected = {
        "feed": ("gc_in", "mmi_a"),
        "upper_in": ("mmi_a", "heater"),
        "lower_in": ("mmi_a", "arm"),
        "upper_out": ("heater", "mmi_b"),
        "lower_out": ("arm", "mmi_b"),
        "drain": ("mmi_b", "gc_out"),
    }
    assert _flow(build_graph_from_schematic(_mzi_schematic(reversed_nets=False))) == expected
    reversed_graph = build_graph_from_schematic(_mzi_schematic(reversed_nets=True))
    assert _flow(reversed_graph) == expected
    # the key still names the net as written, so routed records match it
    assert RoutedEdgeKey("drain", PortRef("gc_out", "o1"), PortRef("mmi_b", "o3")) in (
        reversed_graph.edges
    )
    assert reversed_graph.nodes["mmi_b"].ports["o3"].direction.value == "output"
    assert reversed_graph.nodes["gc_out"].ports["o1"].direction.value == "input"


def test_without_roles_the_declared_order_stays_and_node_types_override_it():
    declared = build_graph_from_schematic(
        _mzi_schematic(reversed_nets=True), port_role=no_port_roles
    )
    assert _flow(declared)["drain"] == ("gc_out", "mmi_b")
    typed = build_graph_from_schematic(
        _mzi_schematic(reversed_nets=True),
        port_role=no_port_roles,
        node_types={"gc_in": "input", "gc_out": "output"},
    )
    assert _flow(typed)["drain"] == ("mmi_b", "gc_out")
    assert _flow(typed)["feed"] == ("gc_in", "mmi_a")


def test_the_arrival_analysis_does_not_depend_on_the_declaration_order():
    lengths = {
        "feed": 50.0,
        "upper_in": 20.0,
        "lower_in": 20.0,
        "upper_out": 20.0,
        "lower_out": 50.0,
        "drain": 30.0,
    }
    delays = {"heater": 320.0, "arm": 100.0}
    results = []
    for reversed_nets in (False, True):
        graph = build_graph_from_schematic(_mzi_schematic(reversed_nets), internal_delays_um=delays)
        annotations = PathLengthGraphAnnotations()
        annotations.set_edge_lengths(graph, {key: lengths[key.net_name] for key in graph.edges})
        analysis = annotations.analyze_missing_lengths(graph)
        results.append(
            {req.edge_key.net_name: req.missing_length_um for req in analysis.requirements}
        )
    # the lower arm (100 + 50) trails the heater arm (320 + 20) by 190 um
    assert results[0] == {"lower_out": pytest.approx(190.0)}
    assert results[1] == results[0]


def test_generic_port_roles_resolver_covers_every_instance():
    role = generic_port_roles(_mzi_schematic(reversed_nets=False))
    assert role("mmi_a", "o1") == "input" and role("mmi_a", "o4") == "output"
    assert role("heater", "o2") == "output"
    assert role("gc_in", "o1") is None
    assert role("unknown", "o1") is None
