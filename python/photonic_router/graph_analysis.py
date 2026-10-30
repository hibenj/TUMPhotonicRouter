"""Shared graph analysis context for optional routing analyses."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from photonic_router.plm.path_length_graph import (
    PathLengthAnalysisResult,
    PathLengthGraphAnnotations,
    PhotonicRoutingGraph,
    RoutedEdgeKey,
    build_graph_from_schematic,
)


class SchematicLike(Protocol):
    netlist: Any
    placements: Mapping[str, Any]


@dataclass
class GraphAnalysisContext:
    """Own one schematic graph and attach optional analysis products to it."""

    schematic: SchematicLike
    graph: PhotonicRoutingGraph
    plm: PathLengthGraphAnnotations | None = None
    timing: PathLengthAnalysisResult | None = None

    @classmethod
    def from_schematic(
        cls,
        schematic: SchematicLike,
        *,
        node_types: Mapping[str, str] | None = None,
        internal_delays_um: Mapping[str, float] | None = None,
    ) -> GraphAnalysisContext:
        graph = build_graph_from_schematic(
            schematic,
            node_types=node_types,
            internal_delays_um=internal_delays_um,
        )
        return cls(schematic=schematic, graph=graph)

    def analyze_path_lengths(
        self,
        edge_lengths_um: Mapping[RoutedEdgeKey, float],
    ) -> PathLengthAnalysisResult:
        self.plm = PathLengthGraphAnnotations.from_edge_lengths(
            self.graph,
            edge_lengths_um,
        )
        self.timing = self.plm.analyze_missing_lengths(self.graph)
        return self.timing
