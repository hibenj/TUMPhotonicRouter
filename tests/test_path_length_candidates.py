"""Upstream delay candidates are told by structure, not by instance names.

Private-circuits plan, Milestone 5 (owner's rule 2026-10-07: a requirement's
length may move upstream only where every group stays matched). A serial node
with one net in and one out passes the delay unchanged whatever it is called;
a node whose whole output bundle needs the same delay can take it on its
inputs, because every path through it gets the delay.
"""

from __future__ import annotations

from types import SimpleNamespace

from photonic_router.path_length_graph import MissingLengthRequirement, PortRef, RoutedEdgeKey

from translation.path_length_candidates import build_requirement_delay_candidates
from translation.path_length_requirements import analyze_path_length_matching
from translation.route_rust_types import RoutedNetRecord


def _schematic(nets: dict[str, tuple[str, str]], components: dict[str, str]) -> SimpleNamespace:
    instances = {
        name: SimpleNamespace(component=component, settings={})
        for name, component in components.items()
    }
    routes = {name: SimpleNamespace(links={p1: p2}) for name, (p1, p2) in nets.items()}
    return SimpleNamespace(netlist=SimpleNamespace(instances=instances, routes=routes))


def _key(net_name: str, p1: str, p2: str) -> RoutedEdgeKey:
    a, pa = p1.split(",")
    b, pb = p2.split(",")
    return RoutedEdgeKey(net_name=net_name, source=PortRef(a, pa), target=PortRef(b, pb))


def _records(nets: dict[str, tuple[str, str]], lengths: dict[str, float]) -> list[RoutedNetRecord]:
    return [
        RoutedNetRecord(
            net_name=name,
            source=_key(name, *ends).source,
            target=_key(name, *ends).target,
            route_obj=None,
            total_length_um=lengths[name],
        )
        for name, ends in nets.items()
    ]


def test_a_bare_arm_called_waveguide_passes_the_delay_upstream():
    # the beam splitter's case: the requirement lands on the short feed behind a
    # bare arm; the long net before the arm is where the design meanders
    nets = {
        "long": ("src0,o1", "waveguide0,o1"),
        "feed": ("waveguide0,o2", "mmi0,o1"),
        "direct": ("src1,o1", "mmi0,o2"),
    }
    components = {
        "src0": "grating_coupler_te",
        "src1": "grating_coupler_te",
        "waveguide0": "straight",
        "mmi0": "mmi2x2",
    }
    analysis, requirements = analyze_path_length_matching(
        _schematic(nets, components),
        routed_net_records=_records(nets, {"long": 500.0, "feed": 50.0, "direct": 1435.0}),
        node_types={"src0": "input", "src1": "input"},
        internal_delays_um={"waveguide0": 0.0},
    )
    feed = _key("feed", *nets["feed"])
    assert requirements == [MissingLengthRequirement(edge_key=feed, missing_length_um=885.0)]
    candidates = build_requirement_delay_candidates(analysis, requirements)
    assert [c.reason for c in candidates[feed]] == ["direct_edge", "transparent_serial_upstream"]
    assert candidates[feed][1].edge_keys == (_key("long", *nets["long"]),)
    assert candidates[feed][1].extra_length_um == 885.0


def test_a_one_input_splitter_whose_outputs_all_need_the_delay_takes_it_on_its_input():
    nets = {
        "in": ("src0,o1", "split0,o1"),
        "up": ("split0,o2", "mmi0,o1"),
        "down": ("split0,o3", "mmi1,o1"),
        "ref_up": ("src1,o1", "mmi0,o2"),
        "ref_down": ("src2,o1", "mmi1,o2"),
    }
    components = {
        "src0": "grating_coupler_te",
        "src1": "grating_coupler_te",
        "src2": "grating_coupler_te",
        "split0": "mmi1x2",
        "mmi0": "mmi2x2",
        "mmi1": "mmi2x2",
    }
    analysis, requirements = analyze_path_length_matching(
        _schematic(nets, components),
        routed_net_records=_records(
            nets, {"in": 10.0, "up": 20.0, "down": 20.0, "ref_up": 130.0, "ref_down": 130.0}
        ),
        node_types={"src0": "input", "src1": "input", "src2": "input"},
        internal_delays_um={},
    )
    up, down = _key("up", *nets["up"]), _key("down", *nets["down"])
    assert {req.edge_key: req.missing_length_um for req in requirements} == {up: 100.0, down: 100.0}
    candidates = build_requirement_delay_candidates(analysis, requirements)
    bundle = [c for c in candidates[up] if c.reason == "common_mode_upstream_bundle"]
    assert len(bundle) == 1
    assert bundle[0].edge_keys == (_key("in", *nets["in"]),)
    assert set(bundle[0].affected_requirement_edge_keys) == {up, down}
    assert bundle[0].extra_length_um == 100.0


def test_a_bundle_with_unequal_needs_does_not_move_upstream():
    nets = {
        "in": ("src0,o1", "split0,o1"),
        "up": ("split0,o2", "mmi0,o1"),
        "down": ("split0,o3", "mmi1,o1"),
        "ref_up": ("src1,o1", "mmi0,o2"),
        "ref_down": ("src2,o1", "mmi1,o2"),
    }
    components = {
        "src0": "grating_coupler_te",
        "src1": "grating_coupler_te",
        "src2": "grating_coupler_te",
        "split0": "mmi1x2",
        "mmi0": "mmi2x2",
        "mmi1": "mmi2x2",
    }
    analysis, requirements = analyze_path_length_matching(
        _schematic(nets, components),
        routed_net_records=_records(
            nets, {"in": 10.0, "up": 20.0, "down": 20.0, "ref_up": 130.0, "ref_down": 70.0}
        ),
        node_types={"src0": "input", "src1": "input", "src2": "input"},
        internal_delays_um={},
    )
    up = _key("up", *nets["up"])
    candidates = build_requirement_delay_candidates(analysis, requirements)
    # the down branch needs 40, the up branch 100: only the shared 40 may move up
    bundle = [c for c in candidates[up] if c.reason == "common_mode_upstream_bundle"]
    assert bundle == [] or all(c.extra_length_um == 40.0 for c in bundle)
