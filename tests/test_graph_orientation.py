"""The path-length graph points every net along the light from the components' fan-in and fan-out.

Private-circuits plan, Milestone 4 (owner's decision 2026-10-07): a waveguide
has no direction; the benchmark defines fan-in and fan-out per element
(`port_roles.INPUT_PORTS_BY_COMPONENT`) and labels its inputs and outputs, and
the graph takes its direction from that, whichever way a net was written.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import pytest
from photonic_router.path_length_graph import (
    PathLengthGraphAnnotations,
    PortRef,
    RoutedEdgeKey,
    build_graph_from_schematic,
)
from photonic_router.port_roles import component_port_role, generic_port_roles, no_port_roles

from benchmark_metadata import load_benchmark_metadata
from translation.path_length_requirements import (
    analyze_path_length_matching,
    build_requirement_delay_candidates,
    compute_output_matching_requirements,
    node_depths,
)
from translation.route_rust_types import RoutedNetRecord

COMPONENTS = {
    "gc_in": "grating_coupler_te",
    "gc_out": "grating_coupler_te",
    "gc_x": "grating_coupler_te",
    "gc_y": "grating_coupler_te",
    "mmi_a": "mmi2x2",
    "mmi_b": "mmi2x2",
    "heater": "straight_heater_metal",
    "arm": "straight",
    "endcap": "straight",
    "mystery": "no_such_component",
}


def _schematic(nets: list[tuple[str, str, str]], reversed_nets: bool = False) -> SimpleNamespace:
    """A schematic-like object: instances from the nets' ends, one bundle per net."""

    instances = {}
    routes = {}
    for name, p1, p2 in nets:
        if reversed_nets:
            p1, p2 = p2, p1
        for spec in (p1, p2):
            instance = spec.split(",")[0]
            instances.setdefault(
                instance, SimpleNamespace(component=COMPONENTS[instance], settings={})
            )
        routes[name] = SimpleNamespace(links={p1: p2})
    return SimpleNamespace(netlist=SimpleNamespace(instances=instances, routes=routes))


MZI = [
    ("feed", "gc_in,o1", "mmi_a,o1"),
    ("upper_in", "mmi_a,o3", "heater,o1"),
    ("lower_in", "mmi_a,o4", "arm,o1"),
    ("upper_out", "heater,o2", "mmi_b,o2"),
    ("lower_out", "arm,o2", "mmi_b,o1"),
    ("drain", "mmi_b,o3", "gc_out,o1"),
    ("stub", "mmi_b,o4", "endcap,o1"),
]
EXPECTED_FLOW = {
    "feed": ("gc_in", "mmi_a"),
    "upper_in": ("mmi_a", "heater"),
    "lower_in": ("mmi_a", "arm"),
    "upper_out": ("heater", "mmi_b"),
    "lower_out": ("arm", "mmi_b"),
    "drain": ("mmi_b", "gc_out"),
    "stub": ("mmi_b", "endcap"),
}


def _flow(graph) -> dict[str, tuple[str, str]]:
    return {
        key.net_name: (edge.source.instance, edge.target.instance)
        for key, edge in graph.edges.items()
    }


def test_the_table_knows_the_generic_components_fan_in_and_fan_out():
    assert component_port_role("mmi2x2", "o1") == "input"
    assert component_port_role("mmi2x2", "o4") == "output"
    assert component_port_role("straight", "o2") == "output"
    assert component_port_role("arm_pair", "o3") == "output"
    # a one-port coupler has no side; an unknown component or port no role
    assert component_port_role("grating_coupler_te", "o1") is None
    assert component_port_role("no_such_component", "o1") is None
    assert component_port_role("mmi2x2", "e1") is None


def test_every_net_points_along_the_light_whichever_way_it_was_written():
    for reversed_nets in (False, True):
        graph = build_graph_from_schematic(_schematic(MZI, reversed_nets))
        assert _flow(graph) == EXPECTED_FLOW
        assert graph.undecided_nets == []
    # the key still names the net as written, so routed records match it
    reversed_graph = build_graph_from_schematic(_schematic(MZI, reversed_nets=True))
    assert RoutedEdgeKey("drain", PortRef("gc_out", "o1"), PortRef("mmi_b", "o3")) in (
        reversed_graph.edges
    )
    assert reversed_graph.nodes["mmi_b"].ports["o3"].direction.value == "output"
    assert reversed_graph.nodes["gc_out"].ports["o1"].direction.value == "input"


def test_a_direct_arm_between_two_mmis_is_oriented_by_their_sides():
    nets = [
        ("feed", "gc_in,o1", "mmi_a,o1"),
        ("upper_in", "mmi_a,o3", "heater,o1"),
        ("upper_out", "mmi_b,o2", "heater,o2"),  # written against the light
        ("lower", "mmi_b,o1", "mmi_a,o4"),  # the bare arm as one net, written against the light
        ("drain", "gc_out,o1", "mmi_b,o3"),
    ]
    graph = build_graph_from_schematic(_schematic(nets))
    assert _flow(graph) == {
        "feed": ("gc_in", "mmi_a"),
        "upper_in": ("mmi_a", "heater"),
        "upper_out": ("heater", "mmi_b"),
        "lower": ("mmi_a", "mmi_b"),
        "drain": ("mmi_b", "gc_out"),
    }


def test_nets_without_a_role_keep_their_written_order_and_labels_decide_them():
    nets = [
        ("loop", "gc_x,o1", "gc_y,o1"),  # two couplers
        ("odd", "mystery,p1", "mmi_a,o3"),  # an unknown component on an output side
        ("stub", "endcap,o1", "mmi_b,o4"),  # an unused output written stub first
    ]
    graph = build_graph_from_schematic(_schematic(nets))
    flow = _flow(graph)
    assert flow["loop"] == ("gc_x", "gc_y")
    assert flow["odd"] == ("mmi_a", "mystery")  # the MMI's output side decides
    assert flow["stub"] == ("mmi_b", "endcap")  # the endcap's own input side decides
    assert graph.undecided_nets == ["loop"]
    labelled = build_graph_from_schematic(
        _schematic(nets), node_types={"gc_x": "output"}, port_role=no_port_roles
    )
    assert _flow(labelled)["loop"] == ("gc_y", "gc_x")
    assert _flow(labelled)["odd"] == ("mystery", "mmi_a")  # no roles: written order
    assert sorted(labelled.undecided_nets) == ["odd", "stub"]


def test_the_arrival_analysis_does_not_depend_on_the_written_order():
    lengths = {
        "feed": 50.0,
        "upper_in": 20.0,
        "lower_in": 20.0,
        "upper_out": 20.0,
        "lower_out": 50.0,
        "drain": 30.0,
        "stub": 10.0,
    }
    delays = {"heater": 320.0, "arm": 100.0}
    results = []
    for reversed_nets in (False, True):
        graph = build_graph_from_schematic(
            _schematic(MZI, reversed_nets), internal_delays_um=delays
        )
        annotations = PathLengthGraphAnnotations()
        annotations.set_edge_lengths(graph, {key: lengths[key.net_name] for key in graph.edges})
        analysis = annotations.analyze_missing_lengths(graph)
        results.append(
            {req.edge_key.net_name: req.missing_length_um for req in analysis.requirements}
        )
    # the lower arm (100 + 50) trails the heater arm (320 + 20) by 190 um; the
    # stub hangs below the MMI and asks for nothing
    assert results[0] == {"lower_out": pytest.approx(190.0)}
    assert results[1] == results[0]


def test_generic_port_roles_resolver_reads_the_instances_components():
    role = generic_port_roles(_schematic(MZI))
    assert role("mmi_a", "o1") == "input" and role("mmi_a", "o4") == "output"
    assert role("heater", "o2") == "output"
    assert role("gc_in", "o1") is None
    assert role("unknown", "o1") is None


def test_the_consumers_of_the_analysis_walk_the_light_not_the_written_order():
    """Output matching, depths and the upstream candidates read the oriented ends of
    an edge, not its key: the beam splitter's tap written coupler-first got a depth
    of 1 and a wrong meander before this."""

    lengths = {
        "feed": 50.0,
        "upper_in": 20.0,
        "lower_in": 20.0,
        "upper_out": 20.0,
        "lower_out": 50.0,
        "drain": 30.0,
        "stub": 10.0,
    }
    results = []
    for reversed_nets in (False, True):
        schematic = _schematic(MZI, reversed_nets)
        records = []
        for name, bundle in schematic.netlist.routes.items():
            ((p1, p2),) = bundle.links.items()
            a, pa = p1.split(",")
            b, pb = p2.split(",")
            records.append(
                RoutedNetRecord(
                    net_name=name,
                    source=PortRef(a, pa),
                    target=PortRef(b, pb),
                    route_obj=None,
                    total_length_um=lengths[name],
                )
            )
        analysis, requirements = analyze_path_length_matching(
            schematic,
            routed_net_records=records,
            node_types={"gc_in": "input", "gc_out": "output", "endcap": "output"},
            internal_delays_um={"heater": 320.0, "arm": 100.0},
        )
        depths = node_depths(analysis)
        outputs, _info = compute_output_matching_requirements(
            analysis, existing_requirements=requirements, scope="depth"
        )
        candidates = build_requirement_delay_candidates(analysis, requirements)
        results.append(
            (
                {req.edge_key.net_name: req.missing_length_um for req in requirements},
                {name: depths[name] for name in ("gc_in", "mmi_a", "heater", "mmi_b", "gc_out")},
                {req.edge_key.net_name: round(req.missing_length_um, 1) for req in outputs},
                {key.net_name: [c.reason for c in cands] for key, cands in candidates.items()},
            )
        )
    assert results[0][0] == {"lower_out": pytest.approx(190.0)}
    assert results[0][1] == {"gc_in": 0, "mmi_a": 1, "heater": 2, "mmi_b": 3, "gc_out": 4}
    # the endcap and gc_out are both outputs of depth 4: the stub trails by 20 um
    assert results[0][2] == {"stub": 20.0}
    assert "transparent_serial_upstream" in results[0][3]["lower_out"]
    assert results[1] == results[0]


def test_a_sink_is_always_a_target_and_takes_no_part_in_output_matching():
    """An end cap on an unused MMI input: both ends are input sides, so only the
    label can point the stub into the dead end; and the dead end must not be
    pulled to the outputs' arrival (the second Clements mesh asked 500 um of a
    10 um stub while its real outputs needed 20)."""

    nets = [
        ("feed", "gc_in,o1", "mmi_a,o1"),
        ("stub", "endcap,o1", "mmi_a,o2"),  # written stub first, both ends input sides
        ("drain", "mmi_a,o3", "gc_out,o1"),
    ]
    graph = build_graph_from_schematic(
        _schematic(nets), node_types={"gc_in": "input", "endcap": "sink", "gc_out": "output"}
    )
    assert _flow(graph)["stub"] == ("mmi_a", "endcap")
    lengths = {"feed": 50.0, "stub": 10.0, "drain": 30.0}
    schematic = _schematic(nets)
    records = []
    for name, bundle in schematic.netlist.routes.items():
        ((p1, p2),) = bundle.links.items()
        a, pa = p1.split(",")
        b, pb = p2.split(",")
        records.append(
            RoutedNetRecord(
                net_name=name,
                source=PortRef(a, pa),
                target=PortRef(b, pb),
                route_obj=None,
                total_length_um=lengths[name],
            )
        )
    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={"gc_in": "input", "endcap": "sink", "gc_out": "output"},
    )
    for scope in ("all", "depth"):
        requirements, info = compute_output_matching_requirements(analysis, scope=scope)
        assert requirements == []  # one output: nothing to align; the sink is no output
        assert info["output_count"] == 1


def test_a_benchmark_declares_the_fan_in_of_its_own_cells():
    """A cell the generic table does not know (the Benes switch, an 8x8 MZI) is
    oriented by the benchmark's own INPUT_PORTS_BY_COMPONENT."""

    nets = [
        ("feed", "gc_in,o1", "mystery,p_in"),
        ("drain", "gc_out,o1", "mystery,p_out"),  # written coupler first
    ]
    declared = {"no_such_component": ("p_in",)}
    graph = build_graph_from_schematic(_schematic(nets), input_ports_by_component=declared)
    assert _flow(graph) == {"feed": ("gc_in", "mystery"), "drain": ("mystery", "gc_out")}
    assert graph.undecided_nets == []
    # without the declaration the second net keeps its written order
    assert _flow(build_graph_from_schematic(_schematic(nets)))["drain"] == ("gc_out", "mystery")
    # a cell declared with no input ports is a coupler: no role of its own
    assert component_port_role("x", "o1", {"x": ()}) is None
    assert component_port_role("x", "o2", {"x": ("o1",)}) == "output"
    # the metadata loader reads the declaration from the benchmark module
    module = types.ModuleType("benchmarks.declared_fan_in")
    module.INPUT_PORTS_BY_COMPONENT = {"switch": ["i0", "i1"]}
    sys.modules["benchmarks.declared_fan_in"] = module
    try:
        metadata = load_benchmark_metadata("declared_fan_in")
    finally:
        del sys.modules["benchmarks.declared_fan_in"]
    assert metadata["input_ports_by_component"] == {"switch": ("i0", "i1")}


def test_a_folded_section_runs_its_components_against_their_numbering():
    """clements_1's edge: an MMI output feeds the next MMI's output port o4, the light
    goes on through that MMI's o1/o2 side and through a heater from o2 to o1; the
    direction is propagated from the input over each component's two sides."""

    nets = [
        ("feed", "gc_in,o1", "mmi_a,o1"),
        ("fold", "mmi_a,o4", "mmi_b,o4"),  # output into output: the fold
        ("back_arm", "mmi_b,o1", "heater,o2"),  # the heater traversed o2 -> o1
        ("back_in", "heater,o1", "arm,o2"),
        ("drain", "arm,o1", "gc_out,o1"),
    ]
    graph = build_graph_from_schematic(_schematic(nets), node_types={"gc_in": "input"})
    assert _flow(graph) == {
        "feed": ("gc_in", "mmi_a"),
        "fold": ("mmi_a", "mmi_b"),
        "back_arm": ("mmi_b", "heater"),
        "back_in": ("heater", "arm"),
        "drain": ("arm", "gc_out"),
    }
    assert graph.undecided_nets == [] and graph.conflicting_nodes == []
    assert graph.nodes["mmi_b"].ports["o4"].direction.value == "input"
    assert graph.nodes["heater"].ports["o2"].direction.value == "input"
    # written the other way round, the same
    assert _flow(
        build_graph_from_schematic(_schematic(nets, True), node_types={"gc_in": "input"})
    ) == _flow(graph)
    # without an input to propagate from, the table's side is the input side and
    # the fold is undecided
    plain = build_graph_from_schematic(_schematic(nets))
    assert plain.undecided_nets == ["fold"]


def test_light_arriving_from_both_sides_is_reported():
    nets = [
        ("feed", "gc_in,o1", "mmi_a,o1"),
        ("back", "gc_x,o1", "mmi_a,o3"),  # a second input on the other side
        ("drain", "mmi_a,o4", "gc_out,o1"),
    ]
    graph = build_graph_from_schematic(
        _schematic(nets), node_types={"gc_in": "input", "gc_x": "input", "gc_out": "output"}
    )
    assert graph.conflicting_nodes == ["mmi_a"]
