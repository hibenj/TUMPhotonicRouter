"""Python preprocessing utilities for the Rust photonic router."""

from __future__ import annotations

import contextlib
import os
import sys
from pathlib import Path

# Matplotlib config directory (moved here from routing_flow.py in Milestone 5,
# Slice 3, so that every entry point gets it: `python -m photonic_router`,
# `python routing_flow.py` and the benchmark drivers). Set before the imports
# below, which reach gdsfactory and matplotlib, and never against an explicit
# choice already in the environment.
if "MPLCONFIGDIR" not in os.environ:
    _default_mpl_config_dir = Path(__file__).resolve().parents[2] / "build" / "mpl"
    with contextlib.suppress(OSError):
        _default_mpl_config_dir.mkdir(parents=True, exist_ok=True)
    os.environ["MPLCONFIGDIR"] = str(_default_mpl_config_dir)


_DLL_DIRECTORY_HANDLES: list[object] = []


def _register_windows_rust_dll_directories() -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return

    package_dir = Path(__file__).resolve().parent
    home = Path.home()
    gnullvm_toolchain = home / ".rustup" / "toolchains" / "stable-x86_64-pc-windows-gnullvm"
    candidate_dirs = (
        package_dir,
        Path(sys.base_prefix),
        Path(sys.executable).resolve().parent,
        gnullvm_toolchain / "bin",
        gnullvm_toolchain / "lib" / "rustlib" / "x86_64-pc-windows-gnullvm" / "bin",
        Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32" / "downlevel",  # noqa: SIM112  (Windows spells it this way)
    )
    for directory in candidate_dirs:
        if not directory.is_dir():
            continue
        try:
            _DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(str(directory)))
        except OSError:
            continue


_register_windows_rust_dll_directories()

from photonic_router.benchmark_extractor import ExtractedBenchmark, Port, extract_benchmark
from photonic_router.crossings.crossing_demand import (
    Band,
    Column,
    CrossingDemand,
    OutsideNet,
    build_crossing_demand,
)
from photonic_router.crossings.crossing_plan import (
    CrossingEvent,
    CrossingPlan,
    CrossingStagePlan,
    LaneRank,
    build_crossing_plan,
)
from photonic_router.graph_analysis import GraphAnalysisContext
from photonic_router.plm.path_length_graph import (
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
from photonic_router.routing_layers import (
    HEATER_METAL_OBSTACLE_LAYERS,
    HEATER_OPTICAL_PORT_ACCESS_RULES,
    OPTICAL_OBSTACLE_LAYERS,
    ComponentPortAccessRule,
    find_component_port_access_rule,
    get_routing_obstacle_layers,
)
from photonic_router.static_obstacle_builder import (
    GridSpec,
    StaticObstacleMapConfig,
    StaticObstacleMapData,
    build_static_obstacle_map,
)

__all__ = [
    "HEATER_METAL_OBSTACLE_LAYERS",
    "HEATER_OPTICAL_PORT_ACCESS_RULES",
    "OPTICAL_OBSTACLE_LAYERS",
    "Band",
    "Column",
    "ComponentPortAccessRule",
    "CrossingDemand",
    "CrossingEvent",
    "CrossingPlan",
    "CrossingStagePlan",
    "DelayInsertionCandidate",
    "ExtractedBenchmark",
    "GraphAnalysisContext",
    "GraphEdge",
    "GraphNode",
    "GridSpec",
    "LaneRank",
    "MissingLengthRequirement",
    "NodeType",
    "OutsideNet",
    "PathLengthAnalysisResult",
    "PathLengthGraphAnnotations",
    "PhotonicRoutingGraph",
    "Port",
    "PortDirection",
    "PortRef",
    "RoutedEdgeKey",
    "StaticObstacleMapConfig",
    "StaticObstacleMapData",
    "annotate_edge_lengths",
    "build_crossing_demand",
    "build_crossing_plan",
    "build_graph_from_schematic",
    "build_static_obstacle_map",
    "extract_benchmark",
    "find_component_port_access_rule",
    "get_routing_obstacle_layers",
    "list_edges_requiring_meander",
]
