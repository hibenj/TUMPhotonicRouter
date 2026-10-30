"""Path-length requirements and where their length may go: group-lifted and
output-matching requirements, merging, the minimum insertable length, and the
upstream delay candidates (told by structure, not by instance names: the
owner's rule of 2026-10-07 that a requirement's length may move upstream only
where every group stays matched -- a serial node with one net in and one out
passes the delay unchanged whatever it is called, a node whose whole output
bundle needs the same delay can take it on its inputs)."""

from __future__ import annotations

import pytest
from photonic_router.plm.path_length_graph import (
    MissingLengthRequirement,
    PortRef,
    RoutedEdgeKey,
)
from photonic_router.plm.path_length_requirements import (
    build_requirement_delay_candidates,
    compute_group_lifted_requirements,
    compute_output_matching_requirements,
    merge_missing_length_requirements,
    minimum_four_bend_extra_length_um,
)
from photonic_router.routing.route_rust import (
    RoutedNetRecord,
    analyze_path_length_matching,
)

from tests.fixtures.path_length import (
    SchematicLike,
    edge_key,
    records_from_links,
    schematic_from_links,
)
from tests.fixtures.synthetic_layouts import (
    PATH_LENGTH_INTERNAL_DELAYS_UM,
    PATH_LENGTH_NODE_TYPES,
    path_length_schematic,
)


def _schematic(nets: dict[str, tuple[str, str]], components: dict[str, str]) -> SchematicLike:
    """One bundle per net between two `inst,port` ends, the instances with
    their component names."""
    return schematic_from_links(components, {name: {p1: p2} for name, (p1, p2) in nets.items()})


def test_group_lifted_requirements_raise_sub_bump_deficit_to_reachable_target():
    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=None,
            total_length_um=99.5,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    requirements, groups = compute_group_lifted_requirements(
        result,
        minimum_insertable_extra_um=25.0,
    )
    required_by_edge = {req.edge_key: req.missing_length_um for req in requirements}

    assert required_by_edge[short_edge] == pytest.approx(25.5)
    assert required_by_edge[long_edge] == pytest.approx(25.0)
    assert groups[0]["target_lift_um"] == pytest.approx(25.0)
    assert groups[0]["edges_requiring_meander"] == 2


def test_group_lifted_requirements_do_not_raise_already_reachable_deficits():
    schematic = path_length_schematic()
    short_edge = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    long_edge = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=short_edge.net_name,
            source=short_edge.source,
            target=short_edge.target,
            route_obj=None,
            total_length_um=70.0,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=None,
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=None,
            total_length_um=60.0,
        ),
    ]
    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=PATH_LENGTH_NODE_TYPES,
        internal_delays_um=PATH_LENGTH_INTERNAL_DELAYS_UM,
    )

    requirements, groups = compute_group_lifted_requirements(
        result,
        minimum_insertable_extra_um=25.0,
    )

    assert requirements == [MissingLengthRequirement(edge_key=short_edge, missing_length_um=30.0)]
    assert groups[0]["target_lift_um"] == pytest.approx(0.0)
    assert groups[0]["edges_requiring_meander"] == 1


def test_output_matching_requirements_align_one_input_outputs_after_existing_delays():
    schematic = schematic_from_links(
        ["src0", "src1", "out0", "out1"],
        {
            "n0": {"src0,o1": "out0,o1"},
            "n1": {"src1,o1": "out1,o1"},
        },
    )
    edge_short = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_long = RoutedEdgeKey(
        net_name="n1",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="out1", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_short.net_name,
            source=edge_short.source,
            target=edge_short.target,
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name=edge_long.net_name,
            source=edge_long.source,
            target=edge_long.target,
            route_obj=None,
            total_length_um=100.0,
        ),
    ]

    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={"src0": "input", "src1": "input", "out0": "output", "out1": "output"},
        internal_delays_um={},
    )
    requirements, info = compute_output_matching_requirements(
        analysis,
        existing_requirements=[
            MissingLengthRequirement(edge_key=edge_short, missing_length_um=5.0)
        ],
    )

    assert requirements == [MissingLengthRequirement(edge_key=edge_short, missing_length_um=15.0)]
    assert info["target_output_arrival_um"] == pytest.approx(100.0)
    assert info["output_count"] == 2


def test_output_matching_by_depth_aligns_only_outputs_of_the_same_depth():
    """The beam splitter's shape: two outputs tapped behind one gate, two behind two
    gates; with scope ``depth`` each pair is aligned among itself and the pairs
    stay apart, with scope ``all`` every output rises to the latest."""

    schematic = schematic_from_links(
        ["src", "g1", "g2", "tap0", "tap1", "out0", "out1"],
        {
            "feed": {"src,o1": "g1,o1"},
            "t0": {"g1,o2": "tap0,o1"},
            "t1": {"g1,o3": "tap1,o1"},
            "link": {"g1,o4": "g2,o1"},
            "e0": {"g2,o2": "out0,o1"},
            "e1": {"g2,o3": "out1,o1"},
        },
    )
    lengths = {"feed": 10.0, "t0": 100.0, "t1": 130.0, "link": 500.0, "e0": 50.0, "e1": 80.0}

    def key(net: str) -> RoutedEdgeKey:
        ((p1, p2),) = schematic.netlist.routes[net].links.items()
        a, pa = p1.split(",")
        b, pb = p2.split(",")
        return RoutedEdgeKey(net_name=net, source=PortRef(a, pa), target=PortRef(b, pb))

    records = [
        RoutedNetRecord(
            net_name=net,
            source=key(net).source,
            target=key(net).target,
            route_obj=None,
            total_length_um=length,
        )
        for net, length in lengths.items()
    ]
    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src": "input",
            "tap0": "output",
            "tap1": "output",
            "out0": "output",
            "out1": "output",
        },
        internal_delays_um={},
    )
    by_depth, info = compute_output_matching_requirements(analysis, scope="depth")
    assert {req.edge_key.net_name: req.missing_length_um for req in by_depth} == {
        "t0": pytest.approx(30.0),
        "e0": pytest.approx(30.0),
    }
    assert info["scope"] == "depth"
    assert info["group_target_arrivals_um"] == {
        "2": pytest.approx(140.0),
        "3": pytest.approx(590.0),
    }
    everything, info_all = compute_output_matching_requirements(analysis, scope="all")
    assert {req.edge_key.net_name: round(req.missing_length_um, 1) for req in everything} == {
        "t0": 480.0,
        "t1": 450.0,
        "e0": 30.0,
    }
    assert info_all["target_output_arrival_um"] == pytest.approx(590.0)
    with pytest.raises(ValueError):
        compute_output_matching_requirements(analysis, scope="pairs")


def test_merge_missing_length_requirements_adds_independent_requirements_per_edge():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )

    merged = merge_missing_length_requirements(
        [MissingLengthRequirement(edge_key=edge, missing_length_um=20.0)],
        [MissingLengthRequirement(edge_key=edge, missing_length_um=5.0)],
    )

    assert merged == [MissingLengthRequirement(edge_key=edge, missing_length_um=25.0)]


def test_delay_candidates_include_transparent_heater_upstream_edge():
    schematic = schematic_from_links(
        ["src0", "src1", "heater0", "mmi0"],
        {
            "src_to_heater": {"src0,o1": "heater0,o1"},
            "heater_to_mmi": {"heater0,o2": "mmi0,i0"},
            "src_to_mmi": {"src1,o1": "mmi0,i1"},
        },
    )
    edge_upstream = RoutedEdgeKey(
        net_name="src_to_heater",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="heater0", port="o1"),
    )
    edge_via_heater = RoutedEdgeKey(
        net_name="heater_to_mmi",
        source=PortRef(instance="heater0", port="o2"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_direct = RoutedEdgeKey(
        net_name="src_to_mmi",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_upstream.net_name,
            source=edge_upstream.source,
            target=edge_upstream.target,
            route_obj=None,
            total_length_um=50.0,
        ),
        RoutedNetRecord(
            net_name=edge_via_heater.net_name,
            source=edge_via_heater.source,
            target=edge_via_heater.target,
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name=edge_direct.net_name,
            source=edge_direct.source,
            target=edge_direct.target,
            route_obj=None,
            total_length_um=100.0,
        ),
    ]

    analysis, requirements = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src0": "input",
            "src1": "input",
            "heater0": "gate",
            "mmi0": "gate",
        },
        internal_delays_um={},
    )
    candidates = build_requirement_delay_candidates(analysis, requirements)

    assert requirements == [
        MissingLengthRequirement(edge_key=edge_via_heater, missing_length_um=40.0)
    ]
    assert [candidate.reason for candidate in candidates[edge_via_heater]] == [
        "direct_edge",
        "transparent_serial_upstream",
    ]
    assert candidates[edge_via_heater][1].edge_keys == (edge_upstream,)


def test_delay_candidates_include_common_mode_bundle_only_for_shared_deficit():
    schematic = schematic_from_links(
        ["src0", "src1", "mmi0", "out0", "out1"],
        {
            "src0_to_mmi": {"src0,o1": "mmi0,i0"},
            "src1_to_mmi": {"src1,o1": "mmi0,i1"},
            "mmi_to_out0": {"mmi0,o0": "out0,o1"},
            "mmi_to_out1": {"mmi0,o1": "out1,o1"},
        },
    )
    edge_in0 = RoutedEdgeKey(
        net_name="src0_to_mmi",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_in1 = RoutedEdgeKey(
        net_name="src1_to_mmi",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    edge_out0 = RoutedEdgeKey(
        net_name="mmi_to_out0",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_out1 = RoutedEdgeKey(
        net_name="mmi_to_out1",
        source=PortRef(instance="mmi0", port="o1"),
        target=PortRef(instance="out1", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_in0.net_name,
            source=edge_in0.source,
            target=edge_in0.target,
            route_obj=None,
            total_length_um=50.0,
        ),
        RoutedNetRecord(
            net_name=edge_in1.net_name,
            source=edge_in1.source,
            target=edge_in1.target,
            route_obj=None,
            total_length_um=50.0,
        ),
        RoutedNetRecord(
            net_name=edge_out0.net_name,
            source=edge_out0.source,
            target=edge_out0.target,
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name=edge_out1.net_name,
            source=edge_out1.source,
            target=edge_out1.target,
            route_obj=None,
            total_length_um=10.0,
        ),
    ]

    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src0": "input",
            "src1": "input",
            "mmi0": "gate",
            "out0": "output",
            "out1": "output",
        },
        internal_delays_um={},
    )
    req0 = MissingLengthRequirement(edge_key=edge_out0, missing_length_um=30.0)
    req1 = MissingLengthRequirement(edge_key=edge_out1, missing_length_um=50.0)
    candidates = build_requirement_delay_candidates(analysis, [req0, req1])

    assert [candidate.reason for candidate in candidates[edge_out0]] == [
        "common_mode_upstream_bundle",
        "direct_edge",
    ]
    assert candidates[edge_out0][0].edge_keys == (edge_in0, edge_in1)
    assert candidates[edge_out0][0].affected_requirement_edge_keys == (
        edge_out0,
        edge_out1,
    )
    assert [candidate.reason for candidate in candidates[edge_out1]] == ["direct_edge"]


def test_delay_candidates_recursively_push_common_delay_through_heater_and_mmi():
    schematic = schematic_from_links(
        ["src0", "src1", "mmi0", "heater0", "mmi1", "out0", "out1"],
        {
            "src0_to_mmi0": {"src0,o1": "mmi0,i0"},
            "src1_to_mmi0": {"src1,o1": "mmi0,i1"},
            "mmi0_upper_to_heater": {"mmi0,o0": "heater0,o1"},
            "heater_to_mmi1": {"heater0,o2": "mmi1,i0"},
            "mmi0_lower_to_mmi1": {"mmi0,o1": "mmi1,i1"},
            "mmi1_to_out0": {"mmi1,o0": "out0,o1"},
            "mmi1_to_out1": {"mmi1,o1": "out1,o1"},
        },
    )
    edge_in0 = RoutedEdgeKey(
        net_name="src0_to_mmi0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_in1 = RoutedEdgeKey(
        net_name="src1_to_mmi0",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    edge_mmi0_upper = RoutedEdgeKey(
        net_name="mmi0_upper_to_heater",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="heater0", port="o1"),
    )
    edge_heater = RoutedEdgeKey(
        net_name="heater_to_mmi1",
        source=PortRef(instance="heater0", port="o2"),
        target=PortRef(instance="mmi1", port="i0"),
    )
    edge_mmi0_lower = RoutedEdgeKey(
        net_name="mmi0_lower_to_mmi1",
        source=PortRef(instance="mmi0", port="o1"),
        target=PortRef(instance="mmi1", port="i1"),
    )
    edge_out0 = RoutedEdgeKey(
        net_name="mmi1_to_out0",
        source=PortRef(instance="mmi1", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_out1 = RoutedEdgeKey(
        net_name="mmi1_to_out1",
        source=PortRef(instance="mmi1", port="o1"),
        target=PortRef(instance="out1", port="o1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_in0.net_name,
            source=edge_in0.source,
            target=edge_in0.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_in1.net_name,
            source=edge_in1.source,
            target=edge_in1.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_mmi0_upper.net_name,
            source=edge_mmi0_upper.source,
            target=edge_mmi0_upper.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_heater.net_name,
            source=edge_heater.source,
            target=edge_heater.target,
            route_obj=None,
            total_length_um=20.0,
        ),
        RoutedNetRecord(
            net_name=edge_mmi0_lower.net_name,
            source=edge_mmi0_lower.source,
            target=edge_mmi0_lower.target,
            route_obj=None,
            total_length_um=40.0,
        ),
        RoutedNetRecord(
            net_name=edge_out0.net_name,
            source=edge_out0.source,
            target=edge_out0.target,
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name=edge_out1.net_name,
            source=edge_out1.source,
            target=edge_out1.target,
            route_obj=None,
            total_length_um=10.0,
        ),
    ]
    analysis, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types={
            "src0": "input",
            "src1": "input",
            "mmi0": "gate",
            "heater0": "gate",
            "mmi1": "gate",
            "out0": "output",
            "out1": "output",
        },
        internal_delays_um={},
    )
    req0 = MissingLengthRequirement(edge_key=edge_out0, missing_length_um=50.0)
    req1 = MissingLengthRequirement(edge_key=edge_out1, missing_length_um=50.0)

    candidates = build_requirement_delay_candidates(analysis, [req0, req1])
    candidate_edges_by_reason = {
        candidate.reason: candidate.edge_keys for candidate in candidates[edge_out0]
    }

    assert candidate_edges_by_reason["common_mode_upstream_bundle"] == (
        edge_heater,
        edge_mmi0_lower,
    )
    assert candidate_edges_by_reason["recursive_transparent_serial_upstream"] == (
        edge_mmi0_upper,
        edge_mmi0_lower,
    )
    assert candidate_edges_by_reason["recursive_common_mode_upstream_bundle"] == (
        edge_in0,
        edge_in1,
    )


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
        routed_net_records=records_from_links(
            nets, {"long": 500.0, "feed": 50.0, "direct": 1435.0}
        ),
        node_types={"src0": "input", "src1": "input"},
        internal_delays_um={"waveguide0": 0.0},
    )
    feed = edge_key("feed", *nets["feed"])
    assert requirements == [MissingLengthRequirement(edge_key=feed, missing_length_um=885.0)]
    candidates = build_requirement_delay_candidates(analysis, requirements)
    assert [c.reason for c in candidates[feed]] == ["direct_edge", "transparent_serial_upstream"]
    assert candidates[feed][1].edge_keys == (edge_key("long", *nets["long"]),)
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
        routed_net_records=records_from_links(
            nets, {"in": 10.0, "up": 20.0, "down": 20.0, "ref_up": 130.0, "ref_down": 130.0}
        ),
        node_types={"src0": "input", "src1": "input", "src2": "input"},
        internal_delays_um={},
    )
    up, down = edge_key("up", *nets["up"]), edge_key("down", *nets["down"])
    assert {req.edge_key: req.missing_length_um for req in requirements} == {up: 100.0, down: 100.0}
    candidates = build_requirement_delay_candidates(analysis, requirements)
    bundle = [c for c in candidates[up] if c.reason == "common_mode_upstream_bundle"]
    assert len(bundle) == 1
    assert bundle[0].edge_keys == (edge_key("in", *nets["in"]),)
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
        routed_net_records=records_from_links(
            nets, {"in": 10.0, "up": 20.0, "down": 20.0, "ref_up": 130.0, "ref_down": 70.0}
        ),
        node_types={"src0": "input", "src1": "input", "src2": "input"},
        internal_delays_um={},
    )
    up = edge_key("up", *nets["up"])
    candidates = build_requirement_delay_candidates(analysis, requirements)
    # the down branch needs 40, the up branch 100: only the shared 40 may move up
    bundle = [c for c in candidates[up] if c.reason == "common_mode_upstream_bundle"]
    assert bundle == [] or all(c.extra_length_um == 40.0 for c in bundle)


def test_minimum_insertable_uses_fill_box_meander_lower_bound():
    minimum = minimum_four_bend_extra_length_um(
        grid_size_um=1.0,
        bend_radius_cells=8,
    )

    # One U-turn at the minimum amplitude 2r + min_straight, two legs.
    assert minimum == pytest.approx(2.0 * (1.0 + (3.141592653589793 - 2.0) * 8.0))
    assert minimum < 46.75
