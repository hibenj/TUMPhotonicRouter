from benchmark_metadata import load_benchmark_metadata
from benchmarks.TOY import build_schematic
import pytest
from dataclasses import replace
from typing import Any, Protocol, cast
from gdsfactory.component import Component
from photonic_router.static_obstacle_builder import _load_rust_backend
from photonic_router.path_length_graph import (
    MissingLengthRequirement,
    PortRef,
    RoutedEdgeKey,
    build_graph_from_schematic,
)
from translation.route_rust import (
    RouteRustPipelineResult,
    RustRouteDebugArtifacts,
    RoutedNetRecord,
    MeanderInsertionConfig,
    analyze_meander_insertion_for_requirements,
    analyze_path_length_matching,
    insert_meanders_for_requirements,
)
from translation.route_rust_analysis import (
    analysis_to_info_dict,
    compute_group_lifted_requirements,
    matching_group_diagnostics_to_info,
)
import translation.route_rust as route_rust
import routing_flow


class _SchematicLike(Protocol):
    netlist: Any
    placements: dict[str, object]


def _build_real_route_obj_for_test(x0: int, y0: int, x1: int, y1: int):
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for M2 candidate-analysis test.")
    grid = rust_backend.GridSpec(256, 256, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=4,
        allow_45_degree_turns=True,
    )
    astar = rust_backend.AStarConfig(max_iterations=200_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    source = rust_backend.State(x0, y0, 0)
    target = rust_backend.State(x1, y1, 0)
    return router.route_single_net(source, target)


def test_build_graph_from_schematic_tracks_port_directions_and_fanout_shape():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("TOY")
    graph = build_graph_from_schematic(schematic, node_types=metadata["node_types"])

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
    schematic = build_schematic()
    metadata = load_benchmark_metadata("TOY")
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
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
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
    schematic = build_schematic()
    metadata = load_benchmark_metadata("TOY")
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
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
    )

    info = analysis_to_info_dict(result)
    groups = cast(list[dict[str, object]], info["matching_groups"])

    assert len(groups) == 1
    assert groups[0]["node_name"] == "mmi_0"
    assert groups[0]["incoming_count"] == 2
    assert groups[0]["edges_requiring_meander"] == 1
    assert groups[0]["max_missing_length_um"] == pytest.approx(20.0)


def test_group_lifted_requirements_raise_sub_bump_deficit_to_reachable_target():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("TOY")
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
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
    )

    requirements, groups = compute_group_lifted_requirements(
        result,
        minimum_insertable_extra_um=25.0,
    )
    required_by_edge = {
        req.edge_key: req.missing_length_um
        for req in requirements
    }

    assert required_by_edge[short_edge] == pytest.approx(25.5)
    assert required_by_edge[long_edge] == pytest.approx(25.0)
    assert groups[0]["target_lift_um"] == pytest.approx(25.0)
    assert groups[0]["edges_requiring_meander"] == 2


def test_group_lifted_requirements_do_not_raise_already_reachable_deficits():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("TOY")
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
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
    )

    requirements, groups = compute_group_lifted_requirements(
        result,
        minimum_insertable_extra_um=25.0,
    )

    assert requirements == [
        MissingLengthRequirement(edge_key=short_edge, missing_length_um=30.0)
    ]
    assert groups[0]["target_lift_um"] == pytest.approx(0.0)
    assert groups[0]["edges_requiring_meander"] == 1


def test_route_match_and_realize_plans_lifted_sub_bump_group(monkeypatch):
    schematic = build_schematic()
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
            route_obj=object(),
            total_length_um=99.5,
        ),
        RoutedNetRecord(
            net_name=long_edge.net_name,
            source=long_edge.source,
            target=long_edge.target,
            route_obj=object(),
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out1_to_gc2",
            source=PortRef(instance="mmi_0", port="o3"),
            target=PortRef(instance="gc_2", port="o1"),
            route_obj=object(),
            total_length_um=60.0,
        ),
        RoutedNetRecord(
            net_name="mmi_out2_to_gc3",
            source=PortRef(instance="mmi_0", port="o4"),
            target=PortRef(instance="gc_3", port="o1"),
            route_obj=object(),
            total_length_um=60.0,
        ),
    ]
    routed_layout = Component(name="lifted_pipeline")
    captured: dict[str, object] = {}

    monkeypatch.setattr(
        route_rust,
        "route_nets_rust",
        lambda *args, **kwargs: (
            routed_layout,
            RustRouteDebugArtifacts(
                obstacle_svg=None,
                route_svgs=[],
                routed_edge_lengths_um={},
                routed_net_records=records,
                realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
                realization_bend_radius_cells=4,
            ),
        ),
    )

    class _ObstacleMap:
        blocked_cells: tuple[tuple[int, int], ...] = ()

    monkeypatch.setattr(
        route_rust,
        "build_static_obstacle_map",
        lambda *args, **kwargs: _ObstacleMap(),
    )

    def _fake_meander_planner(
        routed_net_records: list[RoutedNetRecord],
        requirements: list[MissingLengthRequirement],
        **_kwargs: object,
    ) -> tuple[list[RoutedNetRecord], dict[str, object]]:
        required_by_edge = {req.edge_key: req.missing_length_um for req in requirements}
        assert required_by_edge[short_edge] == pytest.approx(25.5)
        assert required_by_edge[long_edge] == pytest.approx(25.0)
        captured["planner_requirements"] = requirements
        updated = [
            replace(
                record,
                meander_auto_plan={
                    "requested_extra_length_um": required_by_edge[
                        RoutedEdgeKey(
                            net_name=record.net_name,
                            source=record.source,
                            target=record.target,
                        )
                    ],
                    "selected_meander_centerline": [(0.0, 0.0), (1.0, 0.0)],
                    "selected_run_start_index": 0,
                    "selected_run_end_index": 1,
                },
            )
            if RoutedEdgeKey(
                net_name=record.net_name,
                source=record.source,
                target=record.target,
            )
            in required_by_edge
            else record
            for record in routed_net_records
        ]
        return updated, {
            "results": [
                {
                    "edge": {
                        "net_name": short_edge.net_name,
                        "source": {
                            "instance": short_edge.source.instance,
                            "port": short_edge.source.port,
                        },
                        "target": {
                            "instance": short_edge.target.instance,
                            "port": short_edge.target.port,
                        },
                    },
                    "status": "planned",
                    "requested_extra_length_um": 25.5,
                    "inserted_extra_length_um": 25.5,
                    "unmatched_length_um": 0.0,
                },
                {
                    "edge": {
                        "net_name": long_edge.net_name,
                        "source": {
                            "instance": long_edge.source.instance,
                            "port": long_edge.source.port,
                        },
                        "target": {
                            "instance": long_edge.target.instance,
                            "port": long_edge.target.port,
                        },
                    },
                    "status": "planned",
                    "requested_extra_length_um": 25.0,
                    "inserted_extra_length_um": 25.0,
                    "unmatched_length_um": 0.0,
                },
            ],
            "total_requested_extra_length_um": 50.5,
            "total_inserted_extra_length_um": 50.5,
            "total_disregarded_extra_length_um": 0.0,
            "unmatched_length_um": 0.0,
            "planner_calls": 2,
            "minimum_insertable_extra_length_um": 25.132741228718345,
        }

    monkeypatch.setattr(
        route_rust,
        "minimum_four_bend_extra_length_um",
        lambda **_kwargs: 25.0,
    )
    monkeypatch.setattr(
        route_rust,
        "analyze_meander_insertion_for_requirements",
        _fake_meander_planner,
    )
    monkeypatch.setattr(
        route_rust,
        "realize_routed_net_records",
        lambda _layout, routed_net_records, **_kwargs: captured.update(
            realized_records=routed_net_records
        ),
    )

    result = route_rust.route_match_and_realize(
        Component(name="unrouted"),
        schematic,
        enable_path_length_matching=True,
        node_types={
            "gc_0": "input",
            "gc_1": "input",
            "mmi_0": "gate",
            "gc_2": "output",
            "gc_3": "output",
        },
        internal_delays_um={},
    )

    analysis_info = cast(dict[str, object], result.path_length_analysis_info)
    requirements_info = cast(list[dict[str, object]], result.meander_requirements_info)
    diagnostics = cast(
        list[dict[str, object]],
        analysis_info["matching_group_diagnostics"],
    )
    realized_records = cast(list[RoutedNetRecord], captured["realized_records"])
    realized_by_edge = {
        RoutedEdgeKey(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
        ): record
        for record in realized_records
    }

    assert len(requirements_info) == 2
    assert analysis_info["minimum_insertable_extra_length_um"] == pytest.approx(25.0)
    assert diagnostics[0]["target_lift_um"] == pytest.approx(25.0)
    assert diagnostics[0]["max_physical_residual_um"] == pytest.approx(0.0)
    assert diagnostics[0]["max_disregarded_residual_um"] == pytest.approx(0.0)
    assert diagnostics[0]["within_tolerance"] is True
    assert diagnostics[0]["edges_requiring_meander"] == 2
    assert realized_by_edge[short_edge].meander_auto_plan is not None
    assert realized_by_edge[long_edge].meander_auto_plan is not None
    assert cast(dict[str, object], realized_by_edge[short_edge].meander_auto_plan)[
        "requested_extra_length_um"
    ] == pytest.approx(25.5)
    assert cast(dict[str, object], realized_by_edge[long_edge].meander_auto_plan)[
        "requested_extra_length_um"
    ] == pytest.approx(25.0)


def test_matching_group_diagnostics_reports_post_meander_residuals():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("TOY")
    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
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
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
    )
    report = {
        "results": [
            {
                "edge": {
                    "net_name": edge_short.net_name,
                    "source": {
                        "instance": edge_short.source.instance,
                        "port": edge_short.source.port,
                    },
                    "target": {
                        "instance": edge_short.target.instance,
                        "port": edge_short.target.port,
                    },
                },
                "status": "planned",
                "inserted_extra_length_um": 20.0,
                "unmatched_length_um": 0.0,
            }
        ]
    }

    groups = matching_group_diagnostics_to_info(result, report)

    assert len(groups) == 1
    assert groups[0]["within_tolerance"] is True
    assert groups[0]["max_accepted_unmatched_um"] == pytest.approx(0.0)
    assert groups[0]["max_physical_residual_um"] == pytest.approx(0.0)


def test_matching_group_diagnostics_tracks_disregarded_small_residual():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("TOY")
    edge_short = RoutedEdgeKey(
        net_name="gc0_to_mmi_in1",
        source=PortRef(instance="gc_0", port="o1"),
        target=PortRef(instance="mmi_0", port="o2"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_short.net_name,
            source=edge_short.source,
            target=edge_short.target,
            route_obj=None,
            total_length_um=99.5,
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
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
    )
    report = {
        "results": [
            {
                "edge": {
                    "net_name": edge_short.net_name,
                    "source": {
                        "instance": edge_short.source.instance,
                        "port": edge_short.source.port,
                    },
                    "target": {
                        "instance": edge_short.target.instance,
                        "port": edge_short.target.port,
                    },
                },
                "status": "below_minimum_bump",
                "inserted_extra_length_um": 0.0,
                "unmatched_length_um": 0.0,
            }
        ]
    }

    groups = matching_group_diagnostics_to_info(result, report)

    assert groups[0]["within_tolerance"] is True
    assert groups[0]["has_disregarded_residual"] is True
    assert groups[0]["max_accepted_unmatched_um"] == pytest.approx(0.0)
    assert groups[0]["max_physical_residual_um"] == pytest.approx(0.5)
    assert groups[0]["max_disregarded_residual_um"] == pytest.approx(0.5)


def _build_two_stage_schematic_for_convergence() -> _SchematicLike:
    class _Bundle:
        def __init__(self, links: dict[str, str]):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {"input_a": object(), "input_b": object(), "gate_x": object(), "gate_z": object()}
            self.routes = {
                "a_to_x": _Bundle({"input_a,o1": "gate_x,i0"}),
                "b_to_z": _Bundle({"input_b,o1": "gate_z,i1"}),
                "x_to_z": _Bundle({"gate_x,o1": "gate_z,i0"}),
            }

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    return _Schematic()


def test_missing_length_analysis_uses_internal_delay_at_output_node():
    schematic = _build_two_stage_schematic_for_convergence()
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
    schematic = _build_two_stage_schematic_for_convergence()
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
def test_main_flow_flag_enables_path_length_matching(monkeypatch):
    class _Bundle:
        def __init__(self, links):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {"src0": object(), "src1": object(), "gate0": object()}
            self.routes = {
                "n0": _Bundle({"src0,o1": "gate0,i0"}),
                "n1": _Bundle({"src1,o1": "gate0,i1"}),
            }

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    class _Layout:
        def __init__(self):
            self.name = "dummy_layout"
            self.bbox = (0, 0, 1, 1)
            self.info = {}

        def show(self):
            return None

    schematic = _Schematic()
    lengths = {
        RoutedEdgeKey(
            net_name="n0",
            source=PortRef(instance="src0", port="o1"),
            target=PortRef(instance="gate0", port="i0"),
        ): 80.0,
        RoutedEdgeKey(
            net_name="n1",
            source=PortRef(instance="src1", port="o1"),
            target=PortRef(instance="gate0", port="i1"),
        ): 100.0,
    }

    monkeypatch.setattr(routing_flow, "load_benchmark", lambda _: schematic)
    monkeypatch.setattr(
        routing_flow,
        "load_benchmark_metadata",
        lambda *args, **kwargs: {
            "node_types": {"src0": "input", "src1": "input", "gate0": "gate"},
            "internal_delays_um": {},
        },
    )
    monkeypatch.setattr(routing_flow, "layout_from_schematic", lambda _: _Layout())
    monkeypatch.setattr(
        routing_flow,
        "route_match_and_realize",
        lambda *args, **kwargs: RouteRustPipelineResult(
            routed_layout=Component(name="dummy_routed"),
            debug_artifacts=RustRouteDebugArtifacts(
                obstacle_svg=None,
                route_svgs=[],
                routed_edge_lengths_um=lengths,
                routed_net_records=[
                    RoutedNetRecord(
                        net_name="n0",
                        source=PortRef(instance="src0", port="o1"),
                        target=PortRef(instance="gate0", port="i0"),
                        route_obj=object(),
                        total_length_um=80.0,
                    ),
                    RoutedNetRecord(
                        net_name="n1",
                        source=PortRef(instance="src1", port="o1"),
                        target=PortRef(instance="gate0", port="i1"),
                        route_obj=object(),
                        total_length_um=100.0,
                    ),
                ],
                realization_grid_spec=(10, 10, 0.5, 0.0, 0.0),
            ),
            path_length_analysis_info={"requirements": [{}]},
            meander_requirements_info=[{"edge": {"net_name": "n0"}, "missing_length_um": 20.0}],
        ),
    )

    layout = routing_flow.run_routing_flow(
        "DUMMY",
        enable_path_length_matching=True,
    )

    assert "path_length_analysis" in layout.info
    assert "meander_requirements" in layout.info
    assert len(layout.info["meander_requirements"]) == 1


def test_main_flow_matching_uses_record_lengths(monkeypatch):
    class _Bundle:
        def __init__(self, links):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {"src0": object(), "src1": object(), "gate0": object()}
            self.routes = {
                "n0": _Bundle({"src0,o1": "gate0,i0"}),
                "n1": _Bundle({"src1,o1": "gate0,i1"}),
            }

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    class _Layout:
        def __init__(self):
            self.name = "dummy"
            self.bbox = (0, 0, 1, 1)
            self.info = {}

    monkeypatch.setattr(routing_flow, "load_benchmark", lambda _: _Schematic())
    monkeypatch.setattr(
        routing_flow,
        "load_benchmark_metadata",
        lambda *args, **kwargs: {
            "node_types": {"src0": "input", "src1": "input", "gate0": "gate"},
            "internal_delays_um": {},
        },
    )
    monkeypatch.setattr(routing_flow, "layout_from_schematic", lambda _: _Layout())
    monkeypatch.setattr(
        routing_flow,
        "route_match_and_realize",
        lambda *args, **kwargs: RouteRustPipelineResult(
            routed_layout=Component(name="dummy"),
            debug_artifacts=RustRouteDebugArtifacts(
                obstacle_svg=None,
                route_svgs=[],
                routed_edge_lengths_um={},
                routed_net_records=[],
                realization_grid_spec=(10, 10, 0.5, 0.0, 0.0),
            ),
            path_length_analysis_info={"requirements": [{}]},
            meander_requirements_info=[{"edge": {"net_name": "n0"}, "missing_length_um": 20.0}],
        ),
    )

    layout = routing_flow.run_routing_flow("DUMMY", enable_path_length_matching=True)
    assert len(layout.info["meander_requirements"]) == 1
    assert layout.info["meander_requirements"][0]["edge"]["net_name"] == "n0"


def test_main_meander_report_uses_auto_multi_bump_path():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    route_obj = _build_real_route_obj_for_test(10, 10, 60, 10)
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=route_obj,
        total_length_um=30.0,
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=32.0)

    updated, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=5.0,
            max_meander_height_um=40.0,
            auto_meander_endpoint_inset_um=0.0,
        ),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
    )

    assert updated[0].total_length_um == 30.0
    results = cast(list[dict[str, object]], report["results"])
    assert len(results) == 1
    entry = results[0]
    assert entry["status"] != "unsupported_route_object"
    assert entry.get("reason", "") != "route-object mutation not implemented yet"
    assert entry.get("planning_mode") == "fill_box_multi_bump"
    bumps_obj = entry.get("bumps", 0)
    assert isinstance(bumps_obj, (int, float))
    assert int(bumps_obj) >= 1
    assert entry.get("using_legacy_meander_path") is False
    assert (
        entry.get("effective_bend_radius_um")
        == entry.get("primitive_bend_radius_um")
    )


def test_meander_insertion_adapts_bump_cap_for_large_matching_request():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for adaptive meander test.")

    grid = rust_backend.GridSpec(1200, 100, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=2,
        allow_45_degree_turns=False,
    )
    astar = rust_backend.AStarConfig(max_iterations=200_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    route_obj = router.route_single_net(
        rust_backend.State(10, 10, 0),
        rust_backend.State(1051, 10, 0),
    )

    edge = RoutedEdgeKey(
        net_name="long_match",
        source=PortRef(instance="src", port="o1"),
        target=PortRef(instance="dst", port="o1"),
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=route_obj,
        total_length_um=float(route_obj.total_length_um),
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=1459.0)

    _, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=2.0,
            max_meander_height_um=20.0,
        ),
        realization_grid_spec=(1200, 100, 1.0, 0.0, 0.0),
        allow_45_degree_turns=False,
        bend_radius_cells=2,
    )

    results = cast(list[dict[str, object]], report["results"])
    assert len(results) == 1
    entry = results[0]
    assert entry["status"] == "planned"
    assert entry["inserted_extra_length_um"] == pytest.approx(1459.0)
    assert entry["unmatched_length_um"] == pytest.approx(0.0)
    assert int(cast(int, entry["max_bumps"])) == 259
    assert int(cast(int, entry["bumps"])) == 81
    assert entry.get("effective_bend_radius_um") is not None
    assert float(cast(float, entry["planning_elapsed_s"])) >= 0.0
    assert float(cast(float, report["planner_elapsed_s"])) >= 0.0


def test_meander_planning_does_not_open_port_or_static_cells(monkeypatch):
    captured: dict[str, object] = {}

    class _FakeRouter:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass

        def set_static_cells(self, cells: object) -> None:
            captured["static_cells"] = cells

        def add_static_cells(self, cells: object) -> None:
            captured["added_static_cells"] = cells

        def plan_auto_analytic_meander_for_route_depth_sweep(
            self,
            _route_obj: object,
            **kwargs: object,
        ) -> dict[str, object]:
            captured["opened_cells"] = kwargs.get("opened_cells")
            return {
                "inserted_extra_length_um": kwargs["requested_extra_length_um"],
                "effective_bend_radius_um": 4.0,
                "primitive_bend_radius_um": 4.0,
                "selected_box": (0.0, 10.0, 0.0, 10.0),
                "selected_grid_rect": (1, 2, 3, 4),
                "bumps": 1,
                "side": "left",
                "box_depth_um": 10.0,
                "selected_run_start_index": 0,
                "selected_run_end_index": 1,
                "centerline": [(0.0, 0.0), (1.0, 0.0)],
                "planning_mode": "fill_box_multi_bump",
            }

    class _FakeBackend:
        GridSpec = staticmethod(lambda *args, **kwargs: ("grid", args, kwargs))
        PrimitiveLibraryConfig = staticmethod(
            lambda *args, **kwargs: ("primitive", args, kwargs)
        )
        AStarConfig = staticmethod(lambda *args, **kwargs: ("astar", args, kwargs))
        PyPhotonicRouter = _FakeRouter

    monkeypatch.setattr(route_rust, "_load_rust_backend", lambda: _FakeBackend)

    class _RouteObj:
        cells = [(1, 1), (2, 2), (3, 3)]

    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=_RouteObj(),
        total_length_um=30.0,
        opened_cells=((7, 8), (9, 10)),
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=32.0)

    updated, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(enabled=True, min_candidate_straight_length_um=5.0),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
        static_blocked_cells=[(1, 1)],
    )

    opened_cells = cast(list[tuple[int, int]], captured["opened_cells"])
    static_cells = set(cast(list[tuple[int, int]], captured["static_cells"]))
    added_static_cells = set(
        cast(list[tuple[int, int]], captured["added_static_cells"])
    )
    assert set(opened_cells) == {(2, 2), (3, 3)}
    assert (1, 1) not in opened_cells
    assert (7, 8) not in opened_cells
    assert (9, 10) not in opened_cells
    assert (1, 1) in static_cells
    assert (2, 2) in added_static_cells
    assert (3, 3) in added_static_cells
    assert (7, 8) not in static_cells
    assert (9, 10) not in static_cells
    assert updated[0].opened_cells == record.opened_cells
    results = cast(list[dict[str, object]], report["results"])
    assert results[0]["status"] == "planned"


def test_m2_skeleton_reports_no_candidate_when_too_short():
    edge = RoutedEdgeKey(
        net_name="n0",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    route_obj = _build_real_route_obj_for_test(10, 10, 12, 10)
    record = RoutedNetRecord(
        net_name=edge.net_name,
        source=edge.source,
        target=edge.target,
        route_obj=route_obj,
        total_length_um=2.0,
    )
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=10.0)

    _, report = insert_meanders_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(enabled=True, min_candidate_straight_length_um=10.0),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
    )

    assert len(report.results) == 1
    assert report.results[0].status in {
        "below_minimum_bump",
        "no_candidate",
        "insufficient_space",
    }
