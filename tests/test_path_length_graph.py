from routing_flow import load_benchmark_metadata
from benchmarks.TOY import build_schematic
import pytest
from typing import cast
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
import routing_flow


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
            self.name = "dummy"
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
        lambda _: {
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
        lambda _: {
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
    req = MissingLengthRequirement(edge_key=edge, missing_length_um=12.0)

    updated, report = analyze_meander_insertion_for_requirements(
        [record],
        [req],
        config=MeanderInsertionConfig(enabled=True, min_candidate_straight_length_um=5.0),
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
    assert int(bumps_obj) > 1
    assert entry.get("using_legacy_meander_path") is False
    assert (
        entry.get("effective_bend_radius_um")
        == entry.get("primitive_bend_radius_um")
    )
    assert entry.get("effective_bend_radius_um") is not None


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
    assert report.results[0].status in {"no_candidate", "insufficient_space"}
