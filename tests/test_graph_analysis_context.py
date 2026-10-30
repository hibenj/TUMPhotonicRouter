from __future__ import annotations

from photonic_router.benchmark_metadata import load_benchmark_metadata
from photonic_router.crossings.crossing_demand import build_crossing_demand
from photonic_router.graph_analysis import GraphAnalysisContext
from photonic_router.plm.path_length_graph import build_graph_from_schematic
from photonic_router.records.layout_from_schematic import layout_from_schematic

from benchmarks.benes_4x4 import build_schematic


def test_graph_analysis_context_owns_one_graph_for_the_path_length_analysis():
    schematic = build_schematic()
    metadata = load_benchmark_metadata("benes_4x4", schematic)
    context = GraphAnalysisContext.from_schematic(
        schematic,
        node_types=metadata["node_types"],
        internal_delays_um=metadata["internal_delays_um"],
    )
    edge_lengths = dict.fromkeys(context.graph.edges, 100.0)

    timing = context.analyze_path_lengths(edge_lengths)

    assert context.timing is timing
    assert context.plm is not None
    assert context.plm.edge_lengths_um == edge_lengths
    assert len(context.plm.edge_missing_lengths_um) > 0
    assert set(timing.topological_order) == set(schematic.netlist.instances)


def test_base_graph_edges_do_not_hold_plm_annotations():
    graph = build_graph_from_schematic(build_schematic())
    edge = next(iter(graph.edges.values()))

    assert not hasattr(edge, "routed_length_um")
    assert not hasattr(edge, "required_extra_length_um")


def test_crossing_plan_needs_no_topology_metadata():
    # The multiportmmi benchmarks ship EMPTY node_depths/node_ranks/edge_ranks
    # dicts. Until phase 4 slice C of the 2026-10 do-over the plan derived
    # depths and ranks from the graph in that case (an empty mapping had to
    # mean "derive", found 2026-09-04); the plan now comes from the placed
    # layout alone (`photonic_router.crossings.crossing_demand`), the same call path
    # as the router's `_build_crossing_plan_info`.
    from benchmarks.multiportmmi_8x8 import build_schematic as build_mm8

    schematic = build_mm8()
    metadata = load_benchmark_metadata("multiportmmi_8x8", schematic)
    assert metadata["node_depths"] == {}
    assert metadata["node_ranks"] == {}
    assert metadata["edge_ranks"] == {}

    demand = build_crossing_demand(schematic, layout_from_schematic(schematic))

    assert demand.outside == ()
    assert len(demand.plan.events) == 33  # equals the realized lidar-pure crossings
    assert demand.lower_bound() == 33
