from benchmark_metadata import load_benchmark_metadata
from benchmarks.benes import (
    SWITCH_COMPONENT,
    benes_connection_pattern,
    benes_mmi_heater_switch,
    benes_topology_metadata,
)
from benchmarks.benes_4x4 import (
    EXPECTED_CROSSINGS,
    EDGE_RANKS,
    INTERNAL_DELAYS_UM,
    NODE_DEPTHS,
    NODE_RANKS,
    NODE_TYPES,
    build_schematic,
)
from benchmarks.benes_8x8 import (
    EDGE_RANKS as EDGE_RANKS_8X8,
    EXPECTED_CROSSINGS as EXPECTED_CROSSINGS_8X8,
    NODE_DEPTHS as NODE_DEPTHS_8X8,
    NODE_RANKS as NODE_RANKS_8X8,
    build_schematic as build_schematic_8x8,
)
from photonic_router.path_length_graph import build_graph_from_schematic
from photonic_router.crossing_plan import build_crossing_plan
from photonic_router.topology_analysis import analyze_schematic_topology
from translation.layout_from_schematic import layout_from_schematic


def _boxes_overlap(a, b) -> bool:
    return not (
        float(a.right) <= float(b.left)
        or float(b.right) <= float(a.left)
        or float(a.top) <= float(b.bottom)
        or float(b.top) <= float(a.bottom)
    )


def test_benes_4x4_topology_shape_and_crossings():
    metadata = benes_topology_metadata(4)

    assert metadata["stage_count"] == 3
    assert metadata["switches_per_stage"] == 2
    assert benes_connection_pattern(4) == (
        ((0, 1), (0, 1)),
        ((0, 1), (0, 1)),
    )
    assert len(metadata["interstage_edges"]) == 8
    assert len(metadata["crossings"]) == 2
    assert set(metadata["crossings_by_stage"]) == {0, 1}


def test_benes_8x8_topology_shape_is_available_for_next_benchmark():
    metadata = benes_topology_metadata(8)

    assert metadata["stage_count"] == 5
    assert metadata["switches_per_stage"] == 4
    assert len(metadata["interstage_edges"]) == 32
    assert len(metadata["crossings"]) == 16
    crossing_counts = {
        stage: len(crossings)
        for stage, crossings in metadata["crossings_by_stage"].items()
    }
    assert crossing_counts == {
        0: 6,
        1: 2,
        2: 2,
        3: 6,
    }


def test_benes_8x8_schematic_can_be_built():
    schematic = build_schematic_8x8()

    assert len(schematic.netlist.instances) == 36
    assert len(schematic.netlist.routes) == 48


def test_benes_8x8_top_level_placements_do_not_overlap():
    layout = layout_from_schematic(build_schematic_8x8())
    instances = list(layout.insts)

    for index, left in enumerate(instances):
        for right in instances[index + 1 :]:
            assert not _boxes_overlap(left.dbbox(), right.dbbox()), (
                left.name,
                right.name,
                left.dbbox(),
                right.dbbox(),
            )


def test_benes_4x4_schematic_is_dag_with_depth_metadata():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("benes_4x4", schematic)
    graph = build_graph_from_schematic(
        schematic,
        node_types=NODE_TYPES,
        internal_delays_um=metadata["internal_delays_um"],
    )

    assert len(schematic.netlist.instances) == 14
    assert len(schematic.netlist.routes) == 16
    assert len(graph.edges) == 16
    assert set(graph.topological_order()) == set(schematic.netlist.instances)
    assert NODE_DEPTHS["in_0"] == 0
    assert NODE_DEPTHS["sw_s0_0"] == 1
    assert NODE_DEPTHS["sw_s2_1"] == 3
    assert NODE_DEPTHS["out_3"] == 4
    assert NODE_RANKS["in_0"] == 0
    assert NODE_RANKS["out_3"] == 3
    assert len(EXPECTED_CROSSINGS) == 2
    assert INTERNAL_DELAYS_UM["sw_s0_0"] == "auto"
    assert graph.nodes["sw_s0_0"].internal_delay_um == 320.0


def test_benes_4x4_metadata_loader_exposes_topology_fields():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("benes_4x4", schematic)

    assert metadata["topology_metadata"]["network"] == "benes"
    assert metadata["node_depths"]["sw_s1_0"] == 2
    assert metadata["node_ranks"]["out_2"] == 2
    assert len(metadata["expected_crossings"]) == 2
    assert "n_s0_0_o1_to_s1_1_i0" in metadata["edge_ranks"]
    assert metadata["internal_delays_um"]["sw_s0_0"] == 320.0


def test_topology_analysis_uses_benes_edge_ranks_for_crossing_oracle():
    result = analyze_schematic_topology(
        build_schematic(),
        node_depths=NODE_DEPTHS,
        node_ranks=NODE_RANKS,
        edge_ranks=EDGE_RANKS,
    )

    crossing_pairs = {
        (crossing.edge_a.net_name, crossing.edge_b.net_name)
        for crossing in result.crossings
    }
    expected_pairs = {
        (str(crossing["edge_a"]), str(crossing["edge_b"]))
        for crossing in EXPECTED_CROSSINGS
    }
    assert crossing_pairs == expected_pairs
    assert len(result.crossings_by_depth()[(1, 2)]) == 1
    assert len(result.crossings_by_depth()[(2, 3)]) == 1


def test_crossing_plan_orders_4x4_benes_events_by_stage():
    topology = analyze_schematic_topology(
        build_schematic(),
        node_depths=NODE_DEPTHS,
        node_ranks=NODE_RANKS,
        edge_ranks=EDGE_RANKS,
    )
    plan = build_crossing_plan(topology)

    assert len(plan.events) == 2
    assert set(plan.stages) == {(0, 1), (1, 2), (2, 3), (3, 4)}
    assert {key for key, stage in plan.stages.items() if stage.events} == {
        (1, 2),
        (2, 3),
    }
    for stage_plan in (stage for stage in plan.stages.values() if stage.events):
        assert len(stage_plan.events) == 1
        assert stage_plan.apply_events() == stage_plan.final_edge_order
    for event in plan.events:
        assert plan.event_for_pair(event.edge_a, event.edge_b) is event
        assert event in plan.events_for_edge(event.edge_a)
        assert event in plan.events_for_edge(event.edge_b)

    summary = str(plan)
    assert "CrossingPlan: 2 crossing(s), 4 stage(s)" in summary
    assert "stage 1->2: 1 crossing(s)" in summary
    assert "level 0:" in summary
    assert "source order:" in summary
    assert "target order:" in summary
    assert "stage 0->1" not in summary
    assert "stage 0->1" in plan.to_text(include_empty_stages=True)


def test_topology_analysis_matches_8x8_benes_crossing_oracle():
    result = analyze_schematic_topology(
        build_schematic_8x8(),
        node_depths=NODE_DEPTHS_8X8,
        node_ranks=NODE_RANKS_8X8,
        edge_ranks=EDGE_RANKS_8X8,
    )

    crossing_pairs = {
        (crossing.edge_a.net_name, crossing.edge_b.net_name)
        for crossing in result.crossings
    }
    expected_pairs = {
        (str(crossing["edge_a"]), str(crossing["edge_b"]))
        for crossing in EXPECTED_CROSSINGS_8X8
    }
    assert crossing_pairs == expected_pairs


def test_crossing_plan_orders_8x8_benes_events_by_stage():
    topology = analyze_schematic_topology(
        build_schematic_8x8(),
        node_depths=NODE_DEPTHS_8X8,
        node_ranks=NODE_RANKS_8X8,
        edge_ranks=EDGE_RANKS_8X8,
    )
    plan = build_crossing_plan(topology)

    assert len(plan.events) == 16
    assert {
        key: len(stage.events)
        for key, stage in plan.stages.items()
        if stage.events
    } == {
        (1, 2): 6,
        (2, 3): 2,
        (3, 4): 2,
        (4, 5): 6,
    }
    for stage_plan in plan.stages.values():
        assert stage_plan.apply_events() == stage_plan.final_edge_order
        assert stage_plan.events == tuple(
            sorted(stage_plan.events, key=lambda event: (event.level, event.order_index))
        )


def test_benes_switch_cell_contains_two_mmis_and_two_heaters():
    cell = benes_mmi_heater_switch()

    assert SWITCH_COMPONENT in cell.name
    assert {port.name for port in cell.ports} == {"o1", "o2", "o3", "o4"}
    instance_names = {inst.name for inst in cell.insts}
    assert {"mmi_in", "mmi_out", "heater_top", "heater_bottom"} <= instance_names
    assert not {"left_top", "left_bottom", "right_top", "right_bottom"} & instance_names
    ports = {port.name: port for port in cell.ports}
    assert tuple(float(v) for v in ports["o1"].center) == (-10.0, -0.625)
    assert tuple(float(v) for v in ports["o2"].center) == (-10.0, 0.625)
    assert tuple(float(v) for v in ports["o3"].center) == (445.5, 0.625)
    assert tuple(float(v) for v in ports["o4"].center) == (445.5, -0.625)
    assert cell.info["optical_length_um"] == 320.0


def test_benes_4x4_unrouted_layout_can_be_built():
    layout = layout_from_schematic(build_schematic())

    assert len(layout.insts) == 14


def test_benes_4x4_top_level_placements_do_not_overlap():
    layout = layout_from_schematic(build_schematic())
    instances = list(layout.insts)

    for index, left in enumerate(instances):
        for right in instances[index + 1 :]:
            assert not _boxes_overlap(left.dbbox(), right.dbbox()), (
                left.name,
                right.name,
                left.dbbox(),
                right.dbbox(),
            )
