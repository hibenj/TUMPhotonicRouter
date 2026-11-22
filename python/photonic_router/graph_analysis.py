"""Shared graph analysis context for optional routing analyses."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from photonic_router.plm.path_length_graph import (
    PathLengthAnalysisResult,
    PathLengthGraphAnnotations,
    PhotonicRoutingGraph,
    build_graph_from_schematic,
)
from photonic_router.records.route_keys import RoutedEdgeKey
from photonic_router.schematic_protocols import PlacedSchematicLike


@dataclass
class GraphAnalysisContext:
    """Own one schematic graph and attach optional analysis products to it."""

    schematic: PlacedSchematicLike
    graph: PhotonicRoutingGraph
    plm: PathLengthGraphAnnotations | None = None
    timing: PathLengthAnalysisResult | None = None

    @classmethod
    def from_schematic(
        cls,
        schematic: PlacedSchematicLike,
        *,
        node_types: Mapping[str, str] | None = None,
        internal_delays_um: Mapping[str, float] | None = None,
    ) -> GraphAnalysisContext:
        """Build the context of a schematic: its directed graph, no analysis yet.

        `node_types` and `internal_delays_um` (um), both keyed by instance
        name, are handed to `build_graph_from_schematic`. `plm` and `timing`
        stay None until `analyze_path_lengths` runs.
        """
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
        """Analyse the path lengths of the graph for the given routed lengths (um).

        Replaces `plm` with fresh annotations carrying `edge_lengths_um` and
        `timing` with the result of their missing-length analysis, which is
        also returned. Raises `KeyError` when a key of `edge_lengths_um` is
        not an edge of the graph.
        """
        self.plm = PathLengthGraphAnnotations.from_edge_lengths(
            self.graph,
            edge_lengths_um,
        )
        self.timing = self.plm.analyze_missing_lengths(self.graph)
        return self.timing
