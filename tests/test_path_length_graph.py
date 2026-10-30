"""The path-length graph and its arrival analysis: port directions, the
arrival at every node, the missing length per edge and the matching groups."""

from __future__ import annotations

from typing import cast

import pytest
from photonic_router.plm.path_length_diagnostics import (
    analysis_to_info_dict,
)
from photonic_router.plm.path_length_graph import (
    PortRef,
    RoutedEdgeKey,
    build_graph_from_schematic,
)
from photonic_router.routing.route_rust import (
    RoutedNetRecord,
    analyze_path_length_matching,
)

from tests.fixtures.path_length import (
    SchematicLike,
    schematic_from_links,
)
from tests.fixtures.synthetic_layouts import (
    PATH_LENGTH_INTERNAL_DELAYS_UM,
    PATH_LENGTH_NODE_TYPES,
    path_length_schematic,
)


def _two_stage_schematic() -> SchematicLike:
    """input_a -> gate_x -> gate_z <- input_b: the convergence at gate_z reads
    gate_x's internal delay."""
    return schematic_from_links(
        ["input_a", "input_b", "gate_x", "gate_z"],
        {
            "a_to_x": {"input_a,o1": "gate_x,i0"},
            "b_to_z": {"input_b,o1": "gate_z,i1"},
            "x_to_z": {"gate_x,o1": "gate_z,i0"},
        },
    )


def test_build_graph_from_schematic_tracks_port_directions_and_fanout_shape():
    schematic = path_length_schematic()
    graph = build_graph_from_schematic(schematic, node_types=PATH_LENGTH_NODE_TYPES)

    assert graph.nodes["gc_0"].node_type.value == "input"
    assert graph.nodes["mmi_0"].node_type.value == "gate"
    assert graph.nodes["gc_2"].node_type.value == "output"

    key = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    assert key in graph.edges
    assert graph.nodes["gc_0"].ports["o1"].direction.value == "output"
    assert graph.nodes["mmi_0"].ports["o2"].direction.value == "input"


def test_missing_length_analysis_balances_multi_input_node():
    schematic = path_length_schematic()
    records = [
        RoutedNetRecord(
            net_name="gc0_to_mmi_in1",
            source=PortRef(instance="gc_0", port="o1"),
            target=PortRef(instance="mmi_0", port="o2"),
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
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

    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    edge_long = RoutedEdgeKey(
        net_name="gc1_to_mmi_in2",
        source=PortRef(instance="gc_1", port="o1"),
        target=PortRef(instance="mmi_0", port="o1"),
    )

    assert result.edge_missing_lengths_um[edge_short] == 20.0
    assert result.edge_missing_lengths_um[edge_long] == 0.0
    assert any(req.edge_key == edge_short for req in result.requirements)


def test_path_length_analysis_reports_matching_groups():
    schematic = path_length_schematic()
    records = [
        RoutedNetRecord(
            net_name="gc0_to_mmi_in1",
            source=PortRef(instance="gc_0", port="o1"),
            target=PortRef(instance="mmi_0", port="o2"),
            route_obj=None,
            total_length_um=80.0,
        ),
        RoutedNetRecord(
            net_name="gc1_to_mmi_in2",
            source=PortRef(instance="gc_1", port="o1"),
            target=PortRef(instance="mmi_0", port="o1"),
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

    info = analysis_to_info_dict(result)
    groups = cast(list[dict[str, object]], info["matching_groups"])

    assert len(groups) == 1
    assert groups[0]["node_name"] == "mmi_0"
    assert groups[0]["incoming_count"] == 2
    assert groups[0]["edges_requiring_meander"] == 1
    assert groups[0]["max_missing_length_um"] == pytest.approx(20.0)


def test_missing_length_analysis_uses_internal_delay_at_output_node():
    schematic = _two_stage_schematic()
    records = [
        RoutedNetRecord(
            net_name="a_to_x",
            source=PortRef(instance="input_a", port="o1"),
            target=PortRef(instance="gate_x", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="b_to_z",
            source=PortRef(instance="input_b", port="o1"),
            target=PortRef(instance="gate_z", port="i1"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="x_to_z",
            source=PortRef(instance="gate_x", port="o1"),
            target=PortRef(instance="gate_z", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
    ]

    node_types = {
        "input_a": "input",
        "input_b": "input",
        "gate_x": "gate",
        "gate_z": "gate",
    }
    internal_delays = {"gate_x": 100.0}

    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=node_types,
        internal_delays_um=internal_delays,
    )

    edge_direct = RoutedEdgeKey(
        net_name="b_to_z",
        source=PortRef(instance="input_b", port="o1"),
        target=PortRef(instance="gate_z", port="i1"),
    )
    edge_via_x = RoutedEdgeKey(
        net_name="x_to_z",
        source=PortRef(instance="gate_x", port="o1"),
        target=PortRef(instance="gate_z", port="i0"),
    )

    assert result.node_arrival_input_um["gate_x"] == 10.0
    assert result.node_arrival_output_um["gate_x"] == 110.0
    assert result.node_arrival_input_um["gate_z"] == 120.0
    assert result.node_arrival_output_um["gate_z"] == 120.0
    assert result.edge_missing_lengths_um[edge_direct] == 110.0
    assert result.edge_missing_lengths_um[edge_via_x] == 0.0
    assert any(req.edge_key == edge_direct for req in result.requirements)
    assert any(req.edge_key == edge_via_x for req in result.requirements) is False
    timing = result.node_timings["gate_z"]
    assert timing.input_arrival_um == 120.0
    assert timing.output_arrival_um == 120.0
    assert timing.internal_delay_um == 0.0
    assert [timing_edge.edge_key for timing_edge in timing.incoming_edges] == [
        edge_direct,
        edge_via_x,
    ]


def test_missing_length_analysis_without_internal_delay_reduces_to_edge_length_sum():
    schematic = _two_stage_schematic()
    records = [
        RoutedNetRecord(
            net_name="a_to_x",
            source=PortRef(instance="input_a", port="o1"),
            target=PortRef(instance="gate_x", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="b_to_z",
            source=PortRef(instance="input_b", port="o1"),
            target=PortRef(instance="gate_z", port="i1"),
            route_obj=None,
            total_length_um=10.0,
        ),
        RoutedNetRecord(
            net_name="x_to_z",
            source=PortRef(instance="gate_x", port="o1"),
            target=PortRef(instance="gate_z", port="i0"),
            route_obj=None,
            total_length_um=10.0,
        ),
    ]

    node_types = {
        "input_a": "input",
        "input_b": "input",
        "gate_x": "gate",
        "gate_z": "gate",
    }

    result, _ = analyze_path_length_matching(
        schematic,
        routed_net_records=records,
        node_types=node_types,
        internal_delays_um={},
    )

    edge_direct = RoutedEdgeKey(
        net_name="b_to_z",
        source=PortRef(instance="input_b", port="o1"),
        target=PortRef(instance="gate_z", port="i1"),
    )
    assert result.node_arrival_input_um["gate_z"] == 20.0
    assert result.edge_missing_lengths_um[edge_direct] == 10.0
