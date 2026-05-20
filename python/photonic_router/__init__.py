"""Python preprocessing utilities for the Rust photonic router."""

from photonic_router.benchmark_extractor import ExtractedBenchmark, Port, extract_benchmark
from photonic_router.static_obstacle_builder import (
    GridSpec,
    StaticObstacleMapData,
    StaticObstacleMapConfig,
    build_static_obstacle_map,
)
from photonic_router.path_length_graph import (
    GraphEdge,
    GraphNode,
    MissingLengthRequirement,
    NodeType,
    PathLengthAnalysisResult,
    PhotonicRoutingGraph,
    PortDirection,
    PortRef,
    RoutedEdgeKey,
    annotate_edge_lengths,
    build_graph_from_schematic,
    list_edges_requiring_meander,
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
    "MissingLengthRequirement",
    "StaticObstacleMapConfig",
    "StaticObstacleMapData",
    "annotate_edge_lengths",
    "build_graph_from_schematic",
    "build_static_obstacle_map",
    "extract_benchmark",
    "list_edges_requiring_meander",
]
