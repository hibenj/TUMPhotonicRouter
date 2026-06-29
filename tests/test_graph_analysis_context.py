from benchmark_metadata import load_benchmark_metadata
from benchmarks.benes_4x4 import build_schematic
from photonic_router.graph_analysis import GraphAnalysisContext
from photonic_router.path_length_graph import build_graph_from_schematic


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
