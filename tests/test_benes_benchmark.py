from __future__ import annotations

from photonic_router.crossing_plan import CrossingEvent, CrossingPlan, build_crossing_plan
from photonic_router.path_length_graph import build_graph_from_schematic
from photonic_router.topology_analysis import analyze_schematic_topology

from benchmark_metadata import load_benchmark_metadata
from benchmarks import benes_4x4, benes_8x8, benes_16x16, benes_32x32
from benchmarks.benes import (
    SWITCH_COMPONENT,
    benes_connection_pattern,
    benes_mmi_heater_switch,
    benes_topology_metadata,
)
from benchmarks.benes_4x4 import (
    EDGE_RANKS,
    INTERNAL_DELAYS_UM,
    NODE_DEPTHS,
    NODE_RANKS,
    NODE_TYPES,
    build_schematic,
)
from benchmarks.benes_8x8 import (
    EDGE_RANKS as EDGE_RANKS_8X8,
)
from benchmarks.benes_8x8 import (
    NODE_DEPTHS as NODE_DEPTHS_8X8,
)
from benchmarks.benes_8x8 import (
    NODE_RANKS as NODE_RANKS_8X8,
)
from benchmarks.benes_8x8 import (
    build_schematic as build_schematic_8x8,
)
from benchmarks.benes_16x16 import build_schematic as build_schematic_16x16
from benchmarks.benes_32x32 import build_schematic as build_schematic_32x32
from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import _resolve_crossing_half_size_cells


def _plan_for(schematic, module) -> CrossingPlan:
    """The crossing plan of a Benes benchmark module: its topology metadata
    through `analyze_schematic_topology` and `build_crossing_plan`, the one
    producer of planned crossings (the rank inversions) since phase 4 slice A
    of the 2026-10 do-over."""
    topology = analyze_schematic_topology(
        schematic,
        node_depths=module.NODE_DEPTHS,
        node_ranks=module.NODE_RANKS,
        edge_ranks=module.EDGE_RANKS,
    )
    return build_crossing_plan(topology)


def _events_by_stage(plan: CrossingPlan) -> dict[int, int]:
    """Planned crossings per switch stage transition `s -> s + 1` (the inputs
    sit at depth 0, switch stage `s` at depth `s + 1`)."""
    return {key[0] - 1: len(stage.events) for key, stage in plan.stages.items() if stage.events}


def _is_rank_inversion(event: CrossingEvent) -> bool:
    source_delta = event.edge_a_source_rank - event.edge_b_source_rank
    target_delta = event.edge_a_target_rank - event.edge_b_target_rank
    return source_delta * target_delta < 0


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
    plan = _plan_for(build_schematic(), benes_4x4)
    assert len(plan.events) == 2
    assert set(_events_by_stage(plan)) == {0, 1}


def test_benes_8x8_topology_shape_is_available_for_next_benchmark():
    metadata = benes_topology_metadata(8)

    assert metadata["stage_count"] == 5
    assert metadata["switches_per_stage"] == 4
    assert len(metadata["interstage_edges"]) == 32
    plan = _plan_for(build_schematic_8x8(), benes_8x8)
    assert len(plan.events) == 16
    assert _events_by_stage(plan) == {0: 6, 1: 2, 2: 2, 3: 6}


def test_larger_benes_topology_shapes_are_available_for_stress_benchmarks():
    cases = {
        16: {
            "stage_count": 7,
            "switches_per_stage": 8,
            "interstage_edges": 96,
            "crossings": 88,
            "crossings_by_stage": {0: 28, 1: 12, 2: 4, 3: 4, 4: 12, 5: 28},
        },
        32: {
            "stage_count": 9,
            "switches_per_stage": 16,
            "interstage_edges": 256,
            "crossings": 416,
            "crossings_by_stage": {
                0: 120,
                1: 56,
                2: 24,
                3: 8,
                4: 8,
                5: 24,
                6: 56,
                7: 120,
            },
        },
    }

    modules = {16: benes_16x16, 32: benes_32x32}
    for size, expected in cases.items():
        metadata = benes_topology_metadata(size)
        assert metadata["stage_count"] == expected["stage_count"]
        assert metadata["switches_per_stage"] == expected["switches_per_stage"]
        assert len(metadata["interstage_edges"]) == expected["interstage_edges"]
        plan = _plan_for(modules[size].build_schematic(), modules[size])
        assert len(plan.events) == expected["crossings"]
        assert _events_by_stage(plan) == expected["crossings_by_stage"]


def test_benes_8x8_schematic_can_be_built():
    schematic = build_schematic_8x8()

    assert len(schematic.netlist.instances) == 36
    assert len(schematic.netlist.routes) == 48


def test_larger_benes_schematics_can_be_built():
    schematic_16 = build_schematic_16x16()
    schematic_32 = build_schematic_32x32()

    assert len(schematic_16.netlist.instances) == 88
    assert len(schematic_16.netlist.routes) == 128
    assert len(schematic_32.netlist.instances) == 208
    assert len(schematic_32.netlist.routes) == 320


def _plan_from_loaded_metadata(name: str, schematic) -> CrossingPlan:
    metadata = load_benchmark_metadata(name, schematic)
    topology = analyze_schematic_topology(
        schematic,
        node_depths=metadata["node_depths"],
        node_ranks=metadata["node_ranks"],
        edge_ranks=metadata["edge_ranks"],
    )
    return build_crossing_plan(topology)


def test_larger_benes_metadata_loader_exposes_crossing_oracles():
    metadata_16 = load_benchmark_metadata("benes_16x16")
    metadata_32 = load_benchmark_metadata("benes_32x32")

    assert len(metadata_16["edge_ranks"]) == 96
    assert len(metadata_32["edge_ranks"]) == 256
    assert len(_plan_from_loaded_metadata("benes_16x16", build_schematic_16x16()).events) == 88
    assert len(_plan_from_loaded_metadata("benes_32x32", build_schematic_32x32()).events) == 416


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
    assert len(_plan_for(schematic, benes_4x4).events) == 2
    assert INTERNAL_DELAYS_UM["sw_s0_0"] == "auto"
    assert graph.nodes["sw_s0_0"].internal_delay_um == 320.0


def test_benes_4x4_metadata_loader_exposes_topology_fields():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("benes_4x4", schematic)

    assert metadata["topology_metadata"]["network"] == "benes"
    assert metadata["node_depths"]["sw_s1_0"] == 2
    assert metadata["node_ranks"]["out_2"] == 2
    assert len(_plan_from_loaded_metadata("benes_4x4", schematic).events) == 2
    assert "n_s0_0_o1_to_s1_1_i0" in metadata["edge_ranks"]
    assert metadata["internal_delays_um"]["sw_s0_0"] == 320.0


def test_crossing_plan_events_are_the_benes_4x4_rank_inversions():
    plan = _plan_for(build_schematic(), benes_4x4)

    assert len(plan.events) == 2
    assert all(_is_rank_inversion(event) for event in plan.events)
    assert _events_by_stage(plan) == {0: 1, 1: 1}
    for event in plan.events:
        # every event pairs two inter-stage edges of one stage transition
        assert event.target_depth == event.source_depth + 1
        assert event.edge_a != event.edge_b


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


def test_crossing_keepout_size_can_be_derived_from_pdk_component():
    half_size_cells, info = _resolve_crossing_half_size_cells(
        requested_half_size_cells=0,
        enable_crossings=True,
        grid_size_um=2.0,
        clearance_um=0.0,
    )

    assert half_size_cells >= 1
    assert info["derived_from_component"] is True
    assert info["half_size_cells"] == half_size_cells
    assert info["component_bbox_um"][0] > 0
    assert info["component_bbox_um"][1] > 0


def test_crossing_plan_events_are_the_benes_8x8_rank_inversions():
    plan = _plan_for(build_schematic_8x8(), benes_8x8)

    assert len(plan.events) == 16
    assert all(_is_rank_inversion(event) for event in plan.events)
    assert _events_by_stage(plan) == {0: 6, 1: 2, 2: 2, 3: 6}
    assert len({event.edge_pair for event in plan.events}) == 16


def test_crossing_plan_orders_8x8_benes_events_by_stage():
    topology = analyze_schematic_topology(
        build_schematic_8x8(),
        node_depths=NODE_DEPTHS_8X8,
        node_ranks=NODE_RANKS_8X8,
        edge_ranks=EDGE_RANKS_8X8,
    )
    plan = build_crossing_plan(topology)

    assert len(plan.events) == 16
    assert {key: len(stage.events) for key, stage in plan.stages.items() if stage.events} == {
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


def test_flat_benes_8x8_is_the_netlist_of_the_lidar_bridge():
    """Expanded switches: 16 couplers + 20 switches x 4 primitives, 48 routed
    nets of the switch benchmark + 80 switch-internal nets (LiDAR's 128)."""
    from benchmarks.benes import SWITCH_INTERNAL_NETS, primitive_name
    from benchmarks.benes_8x8_flat import build_schematic as build_flat_8x8

    flat = build_flat_8x8()
    parent = build_schematic_8x8()
    assert len(flat.netlist.instances) == 16 + 20 * 4
    assert len(flat.nets) == len(parent.nets) + 20 * len(SWITCH_INTERNAL_NETS) == 128
    assert SWITCH_COMPONENT not in {inst.component for inst in flat.netlist.instances.values()}
    endpoints = {net.p1 for net in flat.nets} | {net.p2 for net in flat.nets}
    for switch in ("sw_s0_0", "sw_s4_3"):
        for primitive, port in (("mmi_in", "o3"), ("heater_top", "o2"), ("mmi_out", "o1")):
            assert f"{primitive_name(switch, primitive)},{port}" in endpoints


def test_flat_benes_primitives_sit_where_the_switch_cell_puts_them():
    from benchmarks.benes_4x4_flat import build_schematic as build_flat_4x4

    parent_layout = layout_from_schematic(build_schematic())
    flat_layout = layout_from_schematic(build_flat_4x4())
    flat_ports = {
        (str(ref.name), port.name): (round(port.dx, 3), round(port.dy, 3))
        for ref in flat_layout.insts
        for port in ref.ports
    }
    checked = 0
    for ref in parent_layout.insts:
        if not ref.cell.name.startswith(SWITCH_COMPONENT):
            continue
        for sub in ref.cell.insts:
            if sub.name not in ("mmi_in", "mmi_out", "heater_top", "heater_bottom"):
                continue
            for port in sub.ports:
                if port.port_type != "optical":
                    continue
                absolute = ref.dcplx_trans * port.dcplx_trans
                expected = (round(absolute.disp.x, 3), round(absolute.disp.y, 3))
                assert flat_ports[(f"{ref.name}__{sub.name}", port.name)] == expected
                checked += 1
    assert checked == 6 * (4 + 4 + 2 + 2)  # 6 switches: MMI 4 optical ports, heater 2


def test_flat_benes_topology_keeps_the_switch_level_crossing_oracle():
    from benchmarks import benes_8x8_flat

    flat = analyze_schematic_topology(
        benes_8x8_flat.build_schematic(),
        node_depths=benes_8x8_flat.NODE_DEPTHS,
        node_ranks=benes_8x8_flat.NODE_RANKS,
        edge_ranks=benes_8x8_flat.EDGE_RANKS,
    )
    parent = analyze_schematic_topology(
        build_schematic_8x8(),
        node_depths=NODE_DEPTHS_8X8,
        node_ranks=NODE_RANKS_8X8,
        edge_ranks=EDGE_RANKS_8X8,
    )
    flat_plan = build_crossing_plan(flat)
    parent_plan = build_crossing_plan(parent)
    pairs = lambda plan: {  # noqa: E731
        frozenset((e.edge_a.net_name, e.edge_b.net_name)) for e in plan.events
    }
    assert pairs(flat_plan) == pairs(parent_plan)
    assert len(pairs(flat_plan)) == len(flat_plan.events) == 16
    internal = [
        e for e in flat_plan.events if "__" in e.edge_a.net_name or "__" in e.edge_b.net_name
    ]
    assert internal == []
