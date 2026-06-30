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
from benchmarks.benes_16x16 import build_schematic as build_schematic_16x16
from benchmarks.benes_32x32 import build_schematic as build_schematic_32x32
from photonic_router.path_length_graph import build_graph_from_schematic
from photonic_router.crossing_plan import build_crossing_plan
from photonic_router.topology_analysis import analyze_schematic_topology
from translation.layout_from_schematic import layout_from_schematic
from translation.route_rust import (
    _augment_crossing_plan_with_realized_overlaps,
    _build_crossing_plan_info,
    _resolve_crossing_half_size_cells,
)
from translation.route_rust_types import RouteJob


class _FakeCrossingConstraint:
    def __init__(
        self,
        net_id,
        partner_net_id,
        *,
        level=0,
        source_depth=0,
        target_depth=0,
    ):
        self.net_id = net_id
        self.partner_net_id = partner_net_id
        self.level = level
        self.source_depth = source_depth
        self.target_depth = target_depth


class _FakeCrossingConfig:
    def __init__(
        self,
        *,
        enabled=False,
        crossing_loss=0.0,
        crossing_half_size_cells=0,
        min_straight_cells_per_crossing=0,
        allow_only_expected_pairs=True,
    ):
        self.enabled = enabled
        self.crossing_loss = crossing_loss
        self.crossing_half_size_cells = crossing_half_size_cells
        self.min_straight_cells_per_crossing = min_straight_cells_per_crossing
        self.allow_only_expected_pairs = allow_only_expected_pairs


class _FakeCrossingBackend:
    CrossingConstraint = _FakeCrossingConstraint
    CrossingConfig = _FakeCrossingConfig


class _FakeCrossingRouter:
    def __init__(self):
        self.config = None
        self.constraints = []
        self.core_cells_by_net_id = {}

    def set_crossing_config(self, config):
        self.config = config

    def set_crossing_constraints(self, constraints):
        self.constraints = list(constraints)

    def crossing_expected_count(self, net_id):
        return sum(
            1
            for constraint in self.constraints
            if constraint.net_id == net_id or constraint.partner_net_id == net_id
        )

    def all_net_core_cells(self):
        return sorted(self.core_cells_by_net_id.items())


class _FakeRouteObj:
    def __init__(self, compressed_waypoints):
        self.compressed_waypoints = compressed_waypoints


class _FakeRouteRecord:
    def __init__(self, route_obj):
        self.route_obj = route_obj


def _route_jobs_from_schematic(schematic):
    jobs = []
    net_id = 1
    for net_name, bundle in schematic.netlist.routes.items():
        for port1_spec, port2_spec in bundle.links.items():
            inst1, port1 = port1_spec.split(",")
            inst2, port2 = port2_spec.split(",")
            jobs.append(
                RouteJob(
                    net_id=net_id,
                    route_index=net_id,
                    net_name=net_name,
                    inst1=inst1,
                    port1=port1,
                    inst2=inst2,
                    port2=port2,
                    source_port=object(),
                    target_port=object(),
                )
            )
            net_id += 1
    return jobs


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

    for size, expected in cases.items():
        metadata = benes_topology_metadata(size)
        assert metadata["stage_count"] == expected["stage_count"]
        assert metadata["switches_per_stage"] == expected["switches_per_stage"]
        assert len(metadata["interstage_edges"]) == expected["interstage_edges"]
        assert len(metadata["crossings"]) == expected["crossings"]
        assert {
            stage: len(crossings)
            for stage, crossings in metadata["crossings_by_stage"].items()
        } == expected["crossings_by_stage"]


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


def test_larger_benes_metadata_loader_exposes_crossing_oracles():
    metadata_16 = load_benchmark_metadata("benes_16x16")
    metadata_32 = load_benchmark_metadata("benes_32x32")

    assert len(metadata_16["expected_crossings"]) == 88
    assert len(metadata_16["edge_ranks"]) == 96
    assert len(metadata_32["expected_crossings"]) == 416
    assert len(metadata_32["edge_ranks"]) == 256


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


def test_benes_crossing_plan_can_be_loaded_into_router_context():
    schematic = build_schematic()
    router = _FakeCrossingRouter()

    info = _build_crossing_plan_info(
        rust_backend=_FakeCrossingBackend,
        router=router,
        schematic=schematic,
        route_jobs=_route_jobs_from_schematic(schematic),
        enable_crossings=True,
        node_depths=NODE_DEPTHS,
        node_ranks=NODE_RANKS,
        edge_ranks=EDGE_RANKS,
        crossing_loss=1.25,
        crossing_half_size_cells=3,
        min_straight_cells_per_crossing=6,
        allow_only_expected_crossings=True,
    )

    assert info["enabled"] is True
    assert info["event_count"] == 2
    assert info["constraint_count"] == 2
    assert info["missing_event_count"] == 0
    assert router.config.enabled is True
    assert router.config.crossing_loss == 1.25
    assert router.config.crossing_half_size_cells == 3
    assert router.config.min_straight_cells_per_crossing == 6
    assert len(router.constraints) == 2
    assert all(constraint.net_id != constraint.partner_net_id for constraint in router.constraints)
    assert sum(info["expected_crossings_by_net_id"].values()) == 4
    assert "CrossingPlan:" in info["plan_text"]
    assert len(info["events"]) == 2
    assert all(event["loaded"] for event in info["events"])

    first = router.constraints[0]
    router.core_cells_by_net_id = {
        first.net_id: [(10, 20), (11, 20)],
        first.partner_net_id: [(11, 20), (12, 20)],
    }
    _augment_crossing_plan_with_realized_overlaps(
        router=router,
        crossing_plan_info=info,
    )
    assert info["actual_crossing_count"] == 1
    assert info["actual_crossings"][0]["cell_count"] == 1
    assert info["unrealized_expected_crossing_count"] == 1


def test_benes_crossing_plan_counts_geometric_route_intersections():
    schematic = build_schematic()
    router = _FakeCrossingRouter()

    info = _build_crossing_plan_info(
        rust_backend=_FakeCrossingBackend,
        router=router,
        schematic=schematic,
        route_jobs=_route_jobs_from_schematic(schematic),
        enable_crossings=True,
        node_depths=NODE_DEPTHS,
        node_ranks=NODE_RANKS,
        edge_ranks=EDGE_RANKS,
        crossing_loss=1.25,
        crossing_half_size_cells=3,
        min_straight_cells_per_crossing=5,
        allow_only_expected_crossings=True,
    )

    first = router.constraints[0]
    router.core_cells_by_net_id = {}
    _augment_crossing_plan_with_realized_overlaps(
        router=router,
        crossing_plan_info=info,
        routed_records_by_net_id={
            first.net_id: _FakeRouteRecord(
                _FakeRouteObj([(0, 0), (10, 10)]),
            ),
            first.partner_net_id: _FakeRouteRecord(
                _FakeRouteObj([(0, 10), (10, 0)]),
            ),
        },
    )

    assert info["actual_crossing_count"] == 1
    assert info["actual_geometric_crossing_count"] == 1
    assert info["actual_crossing_cell_count"] == 0
    assert info["actual_crossings"][0]["geometric"] is True
    assert info["actual_crossings"][0]["point"] == [5.0, 5.0]
    assert info["unrealized_expected_crossing_count"] == 1


def test_benes_crossing_plan_rejects_geometric_intersection_without_margin():
    schematic = build_schematic()
    router = _FakeCrossingRouter()

    info = _build_crossing_plan_info(
        rust_backend=_FakeCrossingBackend,
        router=router,
        schematic=schematic,
        route_jobs=_route_jobs_from_schematic(schematic),
        enable_crossings=True,
        node_depths=NODE_DEPTHS,
        node_ranks=NODE_RANKS,
        edge_ranks=EDGE_RANKS,
        crossing_loss=1.25,
        crossing_half_size_cells=3,
        min_straight_cells_per_crossing=6,
        allow_only_expected_crossings=True,
    )

    first = router.constraints[0]
    router.core_cells_by_net_id = {}
    _augment_crossing_plan_with_realized_overlaps(
        router=router,
        crossing_plan_info=info,
        routed_records_by_net_id={
            first.net_id: _FakeRouteRecord(
                _FakeRouteObj([(0, 0), (10, 10)]),
            ),
            first.partner_net_id: _FakeRouteRecord(
                _FakeRouteObj([(0, 10), (10, 0)]),
            ),
        },
    )

    assert info["actual_crossing_count"] == 0
    assert info["unrealized_expected_crossing_count"] == 2
    invalid = next(
        crossing
        for crossing in info["unrealized_expected_crossings"]
        if crossing.get("unrealized_reason") == "insufficient_straight_margin"
    )
    assert invalid["geometric"] is True
    assert invalid["unrealized_reason"] == "insufficient_straight_margin"
    assert invalid["required_margin_cells"] == 6


def test_benes_crossing_plan_rejects_non_perpendicular_route_intersections():
    schematic = build_schematic()
    router = _FakeCrossingRouter()

    info = _build_crossing_plan_info(
        rust_backend=_FakeCrossingBackend,
        router=router,
        schematic=schematic,
        route_jobs=_route_jobs_from_schematic(schematic),
        enable_crossings=True,
        node_depths=NODE_DEPTHS,
        node_ranks=NODE_RANKS,
        edge_ranks=EDGE_RANKS,
        crossing_loss=1.25,
        crossing_half_size_cells=3,
        min_straight_cells_per_crossing=6,
        allow_only_expected_crossings=True,
    )

    first = router.constraints[0]
    router.core_cells_by_net_id = {}
    _augment_crossing_plan_with_realized_overlaps(
        router=router,
        crossing_plan_info=info,
        routed_records_by_net_id={
            first.net_id: _FakeRouteRecord(
                _FakeRouteObj([(0, 5), (10, 5)]),
            ),
            first.partner_net_id: _FakeRouteRecord(
                _FakeRouteObj([(0, 0), (10, 10)]),
            ),
        },
    )

    assert info["actual_crossing_count"] == 0
    assert info["actual_geometric_crossing_count"] == 0
    assert info["unrealized_expected_crossing_count"] == 2


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
