"""Python preprocessing utilities for the Rust photonic router."""

from photonic_router.benchmark_extractor import ExtractedBenchmark, Port, extract_benchmark
from photonic_router.static_obstacle_builder import (
    GridSpec,
    StaticObstacleMapData,
    StaticObstacleMapConfig,
    build_static_obstacle_map,
)
from photonic_router.routing_layers import (
    ComponentPortAccessRule,
    HEATER_METAL_OBSTACLE_LAYERS,
    HEATER_OPTICAL_PORT_ACCESS_RULES,
    OPTICAL_OBSTACLE_LAYERS,
    find_component_port_access_rule,
    get_routing_obstacle_layers,
)
from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
    GraphEdge,
    GraphNode,
    MissingLengthRequirement,
    NodeType,
    PathLengthAnalysisResult,
    PathLengthGraphAnnotations,
    PhotonicRoutingGraph,
    PortDirection,
    PortRef,
    RoutedEdgeKey,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
)
from photonic_router.graph_analysis import GraphAnalysisContext
from photonic_router.crossing_plan import (
    CrossingEvent,
    CrossingPlan,
    CrossingStagePlan,
    build_crossing_plan,
)
from photonic_router.topology_analysis import (
    TopologyAnalysisResult,
    TopologyCrossing,
    TopologyEdgeRank,
    analyze_graph_topology,
    analyze_schematic_topology,
)

__all__ = [
    "ExtractedBenchmark",
    "GridSpec",
    "Port",
    "PortDirection",
    "PortRef",
    "RoutedEdgeKey",
    "NodeType",
    "GraphNode",
    "GraphEdge",
    "PhotonicRoutingGraph",
    "PathLengthAnalysisResult",
    "PathLengthGraphAnnotations",
    "MissingLengthRequirement",
    "DelayInsertionCandidate",
    "GraphAnalysisContext",
    "CrossingEvent",
    "CrossingPlan",
    "CrossingStagePlan",
    "TopologyAnalysisResult",
    "TopologyCrossing",
    "TopologyEdgeRank",
    "ComponentPortAccessRule",
    "StaticObstacleMapConfig",
    "StaticObstacleMapData",
    "HEATER_METAL_OBSTACLE_LAYERS",
    "HEATER_OPTICAL_PORT_ACCESS_RULES",
    "OPTICAL_OBSTACLE_LAYERS",
    "annotate_edge_lengths",
    "analyze_graph_topology",
    "analyze_schematic_topology",
    "build_crossing_plan",
    "build_graph_from_schematic",
    "build_static_obstacle_map",
    "extract_benchmark",
    "find_component_port_access_rule",
    "get_routing_obstacle_layers",
    "list_edges_requiring_meander",
]
