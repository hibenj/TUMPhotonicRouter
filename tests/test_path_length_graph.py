from benchmark_metadata import load_benchmark_metadata
from benchmarks.TOY import build_schematic
import pytest
from collections import Counter
from dataclasses import replace
from typing import Any, Protocol, cast
from gdsfactory.component import Component
from photonic_router.static_obstacle_builder import _load_rust_backend
from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
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
    build_requirement_delay_candidates,
    compute_group_lifted_requirements,
    compute_output_matching_requirements,
    format_path_length_acceptance_failure,
    matching_group_diagnostics_to_info,
    merge_missing_length_requirements,
    output_matching_diagnostics_to_info,
    path_length_acceptance_summary,
)
from translation.route_rust_meanders import _MeanderPlannerContext
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
        Component(name="unrouted_lifted_pipeline_pass"),
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
    acceptance = cast(dict[str, object], analysis_info["path_length_acceptance"])
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
    assert acceptance["passed"] is True
    assert acceptance["failed_edge_count"] == 0
    assert diagnostics[0]["edges_requiring_meander"] == 2
    assert realized_by_edge[short_edge].meander_auto_plan is not None
    assert realized_by_edge[long_edge].meander_auto_plan is not None
    assert cast(dict[str, object], realized_by_edge[short_edge].meander_auto_plan)[
        "requested_extra_length_um"
    ] == pytest.approx(25.5)
    assert cast(dict[str, object], realized_by_edge[long_edge].meander_auto_plan)[
        "requested_extra_length_um"
    ] == pytest.approx(25.0)


def test_route_match_and_realize_rejects_unrealized_lifted_plm(monkeypatch):
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
    captured: dict[str, object] = {"realized": False}

    monkeypatch.setattr(
        route_rust,
        "route_nets_rust",
        lambda *args, **kwargs: (
            Component(name="failed_lifted_pipeline"),
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

    def _fake_failed_planner(
        _routed_net_records: list[RoutedNetRecord],
        requirements: list[MissingLengthRequirement],
        **_kwargs: object,
    ) -> tuple[list[RoutedNetRecord], dict[str, object]]:
        required_by_edge = {req.edge_key: req.missing_length_um for req in requirements}
        return records, {
            "results": [
                {
                    "edge": {
                        "net_name": edge.net_name,
                        "source": {
                            "instance": edge.source.instance,
                            "port": edge.source.port,
                        },
                        "target": {
                            "instance": edge.target.instance,
                            "port": edge.target.port,
                        },
                    },
                    "status": "no_candidate",
                    "reason": "synthetic planning failure",
                    "requested_extra_length_um": required_by_edge[edge],
                    "inserted_extra_length_um": 0.0,
                    "unmatched_length_um": required_by_edge[edge],
                }
                for edge in (short_edge, long_edge)
            ],
            "total_requested_extra_length_um": sum(required_by_edge.values()),
            "total_inserted_extra_length_um": 0.0,
            "total_disregarded_extra_length_um": 0.0,
            "unmatched_length_um": sum(required_by_edge.values()),
            "planner_calls": 2,
        }

    monkeypatch.setattr(
        route_rust,
        "minimum_four_bend_extra_length_um",
        lambda **_kwargs: 25.0,
    )
    monkeypatch.setattr(
        route_rust,
        "analyze_meander_insertion_for_requirements",
        _fake_failed_planner,
    )
    monkeypatch.setattr(
        route_rust,
        "realize_routed_net_records",
        lambda *args, **kwargs: captured.update(realized=True),
    )

    with pytest.raises(RuntimeError, match="Path-length matching failed") as exc_info:
        route_rust.route_match_and_realize(
            Component(name="unrouted_lifted_pipeline_fail"),
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

    assert "gc0_to_mmi_in1" in str(exc_info.value)
    assert "no_candidate" in str(exc_info.value)
    assert captured["realized"] is False


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


def test_path_length_acceptance_uses_physical_residual_not_accepted_unmatched():
    diagnostics = [
        {
            "node_name": "gate0",
            "node_type": "gate",
            "target_input_arrival_um": 100.0,
            "target_lift_um": 25.0,
            "max_physical_residual_um": 25.0,
            "incoming_edges": [
                {
                    "edge": {
                        "net_name": "n0",
                        "source": {"instance": "src0", "port": "o1"},
                        "target": {"instance": "gate0", "port": "i0"},
                    },
                    "adjusted_missing_length_um": 25.0,
                    "inserted_extra_length_um": 0.0,
                    "physical_residual_um": 25.0,
                    "accepted_unmatched_um": 0.0,
                    "disregarded_residual_um": 25.0,
                    "meander_status": "below_minimum_bump",
                }
            ],
        }
    ]

    summary = path_length_acceptance_summary(diagnostics)
    message = format_path_length_acceptance_failure(summary)

    assert summary["passed"] is False
    assert summary["failed_group_count"] == 1
    assert summary["failed_edge_count"] == 1
    assert summary["max_physical_residual_um"] == pytest.approx(25.0)
    assert "below_minimum_bump" in message
    assert "residual=25" in message


def test_path_length_acceptance_passes_exactly_realized_group():
    diagnostics = [
        {
            "node_name": "gate0",
            "node_type": "gate",
            "target_input_arrival_um": 125.0,
            "target_lift_um": 25.0,
            "max_physical_residual_um": 0.0,
            "incoming_edges": [
                {
                    "edge": {
                        "net_name": "n0",
                        "source": {"instance": "src0", "port": "o1"},
                        "target": {"instance": "gate0", "port": "i0"},
                    },
                    "adjusted_missing_length_um": 25.0,
                    "inserted_extra_length_um": 25.0,
                    "physical_residual_um": 0.0,
                    "accepted_unmatched_um": 0.0,
                    "disregarded_residual_um": 0.0,
                    "meander_status": "planned",
                }
            ],
        }
    ]

    summary = path_length_acceptance_summary(diagnostics)

    assert summary["passed"] is True
    assert summary["failed_group_count"] == 0
    assert summary["failed_edge_count"] == 0
    assert summary["max_physical_residual_um"] == pytest.approx(0.0)


def test_output_matching_diagnostics_fail_unrealized_output_requirement():
    edge_info = {
        "net_name": "out_short",
        "source": {"instance": "mmi0", "port": "o3"},
        "target": {"instance": "gc_out0", "port": "o1"},
    }
    output_info = {
        "enabled": True,
        "target_output_arrival_um": 100.0,
        "output_count": 2,
        "outputs": [
            {
                "node_name": "gc_out0",
                "arrival_um": 80.0,
                "missing_length_um": 20.0,
                "incoming_count": 1,
                "status": "requires_delay",
                "edge": edge_info,
            },
            {
                "node_name": "gc_out1",
                "arrival_um": 100.0,
                "missing_length_um": 0.0,
                "incoming_count": 1,
                "status": "not_required",
            },
        ],
    }
    report = {
        "results": [
            {
                "edge": edge_info,
                "status": "no_candidate",
                "inserted_extra_length_um": 0.0,
                "unmatched_length_um": 20.0,
            }
        ]
    }

    diagnostics = output_matching_diagnostics_to_info(output_info, report)
    summary = path_length_acceptance_summary(diagnostics)

    assert diagnostics[0]["node_name"] == "output_arrivals"
    assert diagnostics[0]["max_physical_residual_um"] == pytest.approx(20.0)
    assert summary["passed"] is False
    assert summary["failed_edge_count"] == 1


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


def _build_schematic_from_links(
    instances: list[str],
    links_by_net: dict[str, dict[str, str]],
) -> _SchematicLike:
    class _Bundle:
        def __init__(self, links: dict[str, str]):
            self.links = links

    class _Netlist:
        def __init__(self):
            self.instances = {name: object() for name in instances}
            self.routes = {
                net_name: _Bundle(links)
                for net_name, links in links_by_net.items()
            }

    class _Schematic:
        def __init__(self):
            self.netlist = _Netlist()
            self.placements = {}

    return _Schematic()


def test_output_matching_requirements_align_one_input_outputs_after_existing_delays():
    schematic = _build_schematic_from_links(
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

    assert requirements == [
        MissingLengthRequirement(edge_key=edge_short, missing_length_um=15.0)
    ]
    assert info["target_output_arrival_um"] == pytest.approx(100.0)
    assert info["output_count"] == 2


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

    assert merged == [
        MissingLengthRequirement(edge_key=edge, missing_length_um=25.0)
    ]


def test_delay_candidates_include_transparent_heater_upstream_edge():
    schematic = _build_schematic_from_links(
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
    schematic = _build_schematic_from_links(
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
    assert [candidate.reason for candidate in candidates[edge_out1]] == [
        "direct_edge"
    ]


def test_delay_candidates_recursively_push_common_delay_through_heater_and_mmi():
    schematic = _build_schematic_from_links(
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
        candidate.reason: candidate.edge_keys
        for candidate in candidates[edge_out0]
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
    captured: dict[str, object] = {}

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
    def _fake_route_match_and_realize(*args: object, **kwargs: object) -> RouteRustPipelineResult:
        captured.update(kwargs)
        return RouteRustPipelineResult(
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
        )

    monkeypatch.setattr(
        routing_flow,
        "route_match_and_realize",
        _fake_route_match_and_realize,
    )

    layout = routing_flow.run_routing_flow(
        "DUMMY",
        enable_path_length_matching=True,
        path_length_match_outputs=True,
    )

    assert captured["path_length_match_outputs"] is True
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


def test_meander_planner_commits_bundle_candidate_atomically(monkeypatch):
    class _RouteObj:
        def __init__(self, name: str):
            self.name = name
            self.cells = [(1, 1), (2, 2)]

    class _FakeRouter:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass

        def set_static_cells(self, _cells: object) -> None:
            return None

        def add_static_cells(self, _cells: object) -> None:
            return None

        def plan_auto_analytic_meander_for_route_depth_sweep(
            self,
            route_obj: _RouteObj,
            **kwargs: object,
        ) -> dict[str, object]:
            requested = float(cast(float, kwargs["requested_extra_length_um"]))
            offset = 10 if route_obj.name == "a" else 20
            return {
                "inserted_extra_length_um": requested,
                "effective_bend_radius_um": 4.0,
                "primitive_bend_radius_um": 4.0,
                "selected_box": (0.0, 10.0, 0.0, 10.0),
                "selected_grid_rect": (offset, offset, offset, offset),
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

    requirement_edge = RoutedEdgeKey(
        net_name="mmi_to_out",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_a = RoutedEdgeKey(
        net_name="src_a_to_mmi",
        source=PortRef(instance="src_a", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_b = RoutedEdgeKey(
        net_name="src_b_to_mmi",
        source=PortRef(instance="src_b", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_a.net_name,
            source=edge_a.source,
            target=edge_a.target,
            route_obj=_RouteObj("a"),
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name=edge_b.net_name,
            source=edge_b.source,
            target=edge_b.target,
            route_obj=_RouteObj("b"),
            total_length_um=100.0,
        ),
    ]
    req = MissingLengthRequirement(
        edge_key=requirement_edge,
        missing_length_um=32.0,
    )

    updated, report = analyze_meander_insertion_for_requirements(
        records,
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=2.0,
        ),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
        requirement_delay_candidates={
            requirement_edge: [
                DelayInsertionCandidate(
                    requirement_edge_key=requirement_edge,
                    edge_keys=(edge_a, edge_b),
                    extra_length_um=32.0,
                    reason="common_mode_upstream_bundle",
                )
            ]
        },
    )

    updated_by_edge = {
        RoutedEdgeKey(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
        ): record
        for record in updated
    }
    results = cast(list[dict[str, object]], report["results"])
    assert results[0]["status"] == "planned"
    assert results[0]["selected_candidate_reason"] == "common_mode_upstream_bundle"
    assert results[0]["selected_candidate_edge_count"] == 2
    assert results[0]["inserted_extra_length_um"] == pytest.approx(32.0)
    assert results[0]["physical_inserted_extra_length_um"] == pytest.approx(64.0)
    assert report["planner_calls"] == 2
    assert updated_by_edge[edge_a].meander_auto_plan is not None
    assert updated_by_edge[edge_b].meander_auto_plan is not None


def test_meander_planner_rejects_partial_bundle_candidate(monkeypatch):
    class _RouteObj:
        def __init__(self, name: str):
            self.name = name
            self.cells = [(1, 1), (2, 2)]

    class _FakeRouter:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass

        def set_static_cells(self, _cells: object) -> None:
            return None

        def add_static_cells(self, _cells: object) -> None:
            return None

        def plan_auto_analytic_meander_for_route_depth_sweep(
            self,
            route_obj: _RouteObj,
            **kwargs: object,
        ) -> dict[str, object] | None:
            if route_obj.name == "b":
                return None
            requested = float(cast(float, kwargs["requested_extra_length_um"]))
            return {
                "inserted_extra_length_um": requested,
                "effective_bend_radius_um": 4.0,
                "primitive_bend_radius_um": 4.0,
                "selected_box": (0.0, 10.0, 0.0, 10.0),
                "selected_grid_rect": (10, 10, 10, 10),
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

    requirement_edge = RoutedEdgeKey(
        net_name="mmi_to_out",
        source=PortRef(instance="mmi0", port="o0"),
        target=PortRef(instance="out0", port="o1"),
    )
    edge_a = RoutedEdgeKey(
        net_name="src_a_to_mmi",
        source=PortRef(instance="src_a", port="o1"),
        target=PortRef(instance="mmi0", port="i0"),
    )
    edge_b = RoutedEdgeKey(
        net_name="src_b_to_mmi",
        source=PortRef(instance="src_b", port="o1"),
        target=PortRef(instance="mmi0", port="i1"),
    )
    records = [
        RoutedNetRecord(
            net_name=edge_a.net_name,
            source=edge_a.source,
            target=edge_a.target,
            route_obj=_RouteObj("a"),
            total_length_um=100.0,
        ),
        RoutedNetRecord(
            net_name=edge_b.net_name,
            source=edge_b.source,
            target=edge_b.target,
            route_obj=_RouteObj("b"),
            total_length_um=100.0,
        ),
    ]
    req = MissingLengthRequirement(
        edge_key=requirement_edge,
        missing_length_um=32.0,
    )

    updated, report = analyze_meander_insertion_for_requirements(
        records,
        [req],
        config=MeanderInsertionConfig(
            enabled=True,
            min_candidate_straight_length_um=2.0,
        ),
        realization_grid_spec=(200, 200, 1.0, 0.0, 0.0),
        allow_45_degree_turns=True,
        bend_radius_cells=4,
        requirement_delay_candidates={
            requirement_edge: [
                DelayInsertionCandidate(
                    requirement_edge_key=requirement_edge,
                    edge_keys=(edge_a, edge_b),
                    extra_length_um=32.0,
                    reason="common_mode_upstream_bundle",
                )
            ]
        },
    )

    results = cast(list[dict[str, object]], report["results"])
    assert results[0]["status"] == "no_candidate"
    assert results[0]["inserted_extra_length_um"] == pytest.approx(0.0)
    assert report["planner_calls"] == 2
    assert all(record.meander_auto_plan is None for record in updated)


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


def test_registered_meander_route_cells_match_legacy_opened_cells():
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        pytest.skip("Rust backend unavailable for registered meander path test.")

    grid = rust_backend.GridSpec(240, 120, 1.0, 0.0, 0.0)
    primitive = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=1.0,
        bend_radius_cells=2,
        allow_45_degree_turns=False,
    )
    astar = rust_backend.AStarConfig(max_iterations=200_000)
    router = rust_backend.PyPhotonicRouter(grid, primitive, astar)
    if not hasattr(
        router,
        "plan_auto_analytic_meander_for_centerline_depth_sweep_registered_opened",
    ):
        pytest.skip("Rust backend lacks registered meander opened-cell API.")

    route_obj = router.route_single_net(
        rust_backend.State(20, 60, 0),
        rust_backend.State(180, 60, 0),
    )
    centerline = router.route_port_corrected_centerline(route_obj)
    indices, open_counts, unique_count = router.register_meander_route_cells_as_static(
        [route_obj],
        [],
    )

    assert indices == [0]
    assert open_counts == [len(route_obj.cells)]
    assert unique_count == len(route_obj.cells)
    assert router.registered_meander_open_cell_count(0) == len(route_obj.cells)

    kwargs = {
        "requested_extra_length_um": 75.311,
        "box_depths_um": [40.0, 30.0, 24.0, 20.0],
        "min_bend_radius_um": None,
        "min_straight_um": 2.0,
        "max_bumps": 21,
        "max_meander_height_um": 40.0,
        "min_segment_length_um": 2.0,
        "endpoint_inset_um": 0.0,
        "clearance_radius_cells": 0,
        "side_policy": "both",
        "planning_mode": "fill_box_multi_bump",
    }
    registered = (
        router.plan_auto_analytic_meander_for_centerline_depth_sweep_registered_opened(
            centerline,
            0,
            **kwargs,
        )
    )
    legacy = router.plan_auto_analytic_meander_for_centerline_depth_sweep(
        centerline,
        opened_cells=route_obj.cells,
        extra_blocked_cells=None,
        **kwargs,
    )

    assert registered["inserted_extra_length_um"] == pytest.approx(
        legacy["inserted_extra_length_um"]
    )
    assert registered["selected_grid_rect"] == legacy["selected_grid_rect"]
    assert registered["bumps"] == legacy["bumps"]

    bundle = router.plan_auto_analytic_meander_candidate_bundle_registered_opened(
        [centerline],
        [0],
        max_bumps_by_edge=[kwargs["max_bumps"]],
        **{key: value for key, value in kwargs.items() if key != "max_bumps"},
    )
    assert bundle["status"] == "planned"
    bundle_plans = cast(list[dict[str, object]], bundle["plans"])
    assert len(bundle_plans) == 1
    assert bundle_plans[0]["inserted_extra_length_um"] == pytest.approx(
        legacy["inserted_extra_length_um"]
    )
    assert bundle_plans[0]["selected_grid_rect"] == legacy["selected_grid_rect"]

    selected_rect = cast(tuple[int, int, int, int], registered["selected_grid_rect"])
    router.add_registered_meander_reserved_cells(
        [
            (x, y)
            for x in range(selected_rect[0], selected_rect[1] + 1)
            for y in range(selected_rect[2], selected_rect[3] + 1)
        ]
    )
    shifted = router.plan_auto_analytic_meander_for_centerline_depth_sweep_registered_opened(
        centerline,
        0,
        **kwargs,
    )

    assert shifted["selected_grid_rect"] != selected_rect


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


def test_meander_commit_defers_python_reserved_cells_until_fallback_planning():
    captured: dict[str, object] = {}

    class _RouteObj:
        cells = [(1, 1), (2, 2)]
        compressed_waypoints = [(0, 0), (100, 0)]

    class _FakeRouter:
        def add_registered_meander_reserved_grid_rect(
            self,
            min_x: int,
            max_x: int,
            min_y: int,
            max_y: int,
        ) -> int:
            captured["registered_rect"] = (min_x, max_x, min_y, max_y)
            return (max_x - min_x + 1) * (max_y - min_y + 1)

        def plan_auto_analytic_meander_for_route_depth_sweep(
            self,
            _route_obj: object,
            **kwargs: object,
        ) -> dict[str, object]:
            captured["extra_blocked_cells"] = kwargs.get("extra_blocked_cells")
            return {
                "inserted_extra_length_um": kwargs["requested_extra_length_um"],
                "effective_bend_radius_um": 4.0,
                "primitive_bend_radius_um": 4.0,
                "selected_box": (0.0, 10.0, 0.0, 10.0),
                "selected_grid_rect": (10, 10, 10, 10),
                "bumps": 1,
                "side": "left",
                "box_depth_um": 10.0,
                "selected_run_start_index": 0,
                "selected_run_end_index": 1,
                "centerline": [(0.0, 0.0), (1.0, 0.0)],
                "planning_mode": "fill_box_multi_bump",
            }

    committed_edge = RoutedEdgeKey(
        net_name="committed",
        source=PortRef(instance="src0", port="o1"),
        target=PortRef(instance="gate0", port="i0"),
    )
    fallback_edge = RoutedEdgeKey(
        net_name="fallback",
        source=PortRef(instance="src1", port="o1"),
        target=PortRef(instance="gate1", port="i0"),
    )
    committed_record = RoutedNetRecord(
        net_name=committed_edge.net_name,
        source=committed_edge.source,
        target=committed_edge.target,
        route_obj=_RouteObj(),
        total_length_um=30.0,
    )
    fallback_record = RoutedNetRecord(
        net_name=fallback_edge.net_name,
        source=fallback_edge.source,
        target=fallback_edge.target,
        route_obj=_RouteObj(),
        total_length_um=30.0,
    )
    context = _MeanderPlannerContext(
        router=cast(Any, _FakeRouter()),
        by_edge={
            committed_edge: committed_record,
            fallback_edge: fallback_record,
        },
        updated={
            committed_edge: committed_record,
            fallback_edge: fallback_record,
        },
        route_cells_by_edge={
            fallback_edge: {(1, 1), (2, 2)},
        },
        route_cell_refcounts=Counter({(1, 1): 1, (2, 2): 1}),
        registered_open_cell_index_by_edge={},
        registered_open_cell_count_by_edge={},
        registered_geometry_index_by_edge={},
        base_open_cells_by_edge={},
        base_open_cell_lists_by_edge={},
        max_bumps_by_edge={},
        centerline_lists_by_edge={},
        footprint_prefilter_enabled_by_edge={fallback_edge: False},
        base_static_cells=set(),
        reserved_meander_cells=set(),
        pending_reserved_meander_rects=[],
        grid_size_um=1.0,
        bend_radius_um=4.0,
        setup_profile={},
        candidate_setup_profile={},
        commit_profile={},
        rust_planner_profile={},
        rust_wrapper_profile={},
    )

    context.commit_planned_edge(
        selected_edge_key=committed_edge,
        record=committed_record,
        rr={
            "inserted_extra_length_um": 12.0,
            "selected_grid_rect": (3, 4, 5, 6),
            "box_depth_um": 10.0,
        },
        requested=12.0,
        used_reserved_overlay=True,
        max_bumps=1,
        min_straight_um=5.0,
        max_height_um=40.0,
        min_seg_um=5.0,
        endpoint_inset_um=0.0,
    )

    assert captured["registered_rect"] == (3, 4, 5, 6)
    assert context.reserved_meander_cells == set()
    assert context.pending_reserved_meander_rects == [(3, 4, 5, 6)]
    assert "grid_rect_cells_s" not in context.commit_profile
    assert "python_reserved_update_s" not in context.commit_profile

    attempt, rr, _record, used_reserved_overlay, *_rest = context.plan_edge_candidate(
        candidate_edge_key=fallback_edge,
        requested=10.0,
        extra_reserved_cells=set(),
        box_depths_um=[10.0],
        min_straight_um=5.0,
        min_seg_um=5.0,
        max_height_um=40.0,
        endpoint_inset_um=0.0,
        use_probe=False,
    )

    blocked_cells = set(cast(list[tuple[int, int]], captured["extra_blocked_cells"]))
    assert attempt["status"] == "planned"
    assert rr is not None
    assert used_reserved_overlay is True
    assert {(3, 5), (4, 5), (3, 6), (4, 6)} <= blocked_cells
    assert context.pending_reserved_meander_rects == []
    assert {(3, 5), (4, 5), (3, 6), (4, 6)} <= context.reserved_meander_cells
    assert context.candidate_setup_profile["materialize_reserved_rects_s"] >= 0.0


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
