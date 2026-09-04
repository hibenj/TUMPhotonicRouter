from benchmark_metadata import load_benchmark_metadata
from benchmarks.benes_4x4 import build_schematic
from photonic_router.crossing_plan import build_crossing_plan
from photonic_router.graph_analysis import GraphAnalysisContext
from photonic_router.path_length_graph import build_graph_from_schematic
from photonic_router.topology_analysis import analyze_schematic_topology


def test_graph_analysis_context_reuses_one_graph_for_plm_and_topology():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("benes_4x4", schematic)
    context = GraphAnalysisContext.from_schematic(
        schematic,
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
    )
    edge_lengths = {edge_key: 100.0 for edge_key in context.graph.edges}

    timing = context.analyze_path_lengths(edge_lengths)
    topology = context.analyze_topology(
        node_depths=metadata["node_depths"],
        node_ranks=metadata["node_ranks"],
        edge_ranks=metadata["edge_ranks"],
    )
    crossing_plan = context.build_crossing_plan()

    assert context.timing is timing
    assert context.plm is not None
    assert context.plm.edge_lengths_um == edge_lengths
    assert len(context.plm.edge_missing_lengths_um) > 0
    assert context.topology is topology
    assert context.crossing_plan is crossing_plan
    assert set(timing.topological_order) == set(topology.topological_order)
    assert len(topology.crossings) == len(metadata["expected_crossings"])
    assert len(crossing_plan.events) == len(metadata["expected_crossings"])


def test_base_graph_edges_do_not_hold_plm_annotations():
    graph = build_graph_from_schematic(build_schematic())
    edge = next(iter(graph.edges.values()))

    assert not hasattr(edge, "routed_length_um")
    assert not hasattr(edge, "required_extra_length_um")


def test_topology_plan_derives_depths_and_ranks_when_metadata_dicts_are_empty():
    # The multiportmmi benchmarks ship EMPTY node_depths/node_ranks/edge_ranks
    # dicts (no explicit topology metadata). An empty mapping carries no
    # information and must mean "derive", exactly like None -- otherwise the
    # first fanout node raises KeyError and no crossing plan exists for the
    # whole benchmark family (found 2026-09-04, contribution-1 design).
    from benchmarks.multiportmmi_8x8 import build_schematic as build_mm8

    schematic = build_mm8()
    metadata = load_benchmark_metadata("multiportmmi_8x8", schematic)
    assert metadata["node_depths"] == {}
    assert metadata["node_ranks"] == {}
    assert metadata["edge_ranks"] == {}

    # Same call path as the router's crossing-plan construction
    # (`translation.route_rust_crossing_plan._build_crossing_plan_info`).
    topology = analyze_schematic_topology(
        schematic,
        node_depths=metadata["node_depths"],
        node_ranks=metadata["node_ranks"],
        edge_ranks=metadata["edge_ranks"],
    )
    plan = build_crossing_plan(topology)

    assert topology.node_depths["fanout_yb_0_0"] >= 0
    assert len(plan.events) == 33  # equals the realized lidar-pure crossings of multiportmmi_8x8
