"""Route record bookkeeping for the Rust-backed photonic router."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from photonic_router.path_length_graph import PortRef, RoutedEdgeKey

from translation.route_rust_types import (
    RouteJob,
    RouteAttemptRecord,
    RouteSearchSummary,
    RoutedNetRecord,
    RustRouteDebugArtifacts,
)


def route_edge_key(job: RouteJob) -> RoutedEdgeKey:
    return RoutedEdgeKey(
        net_name=job.net_name,
        source=PortRef(instance=job.inst1, port=job.port1),
        target=PortRef(instance=job.inst2, port=job.port2),
    )


def routed_edge_lengths_from_records(
    records: list[RoutedNetRecord],
) -> dict[RoutedEdgeKey, float]:
    return {
        RoutedEdgeKey(
            net_name=record.net_name,
            source=record.source,
            target=record.target,
        ): float(record.total_length_um)
        for record in records
    }


@dataclass
class RouteBookkeeping:
    route_order: list[int]
    diagnostics_enabled: bool = False
    records_by_id: dict[int, RoutedNetRecord] = field(default_factory=dict)
    lengths_by_id: dict[int, float] = field(default_factory=dict)
    committed_dynamic_cells_by_id: dict[int, set[tuple[int, int]]] = field(default_factory=dict)

    def record_route(
        self,
        job: RouteJob,
        route_obj: object,
        opened_cells: list[tuple[int, int]],
        *,
        route_cells: set[tuple[int, int]] | None = None,
    ) -> None:
        edge_key = route_edge_key(job)
        total_length_um = float(getattr(route_obj, "total_length_um"))
        self.records_by_id[job.net_id] = RoutedNetRecord(
            net_name=job.net_name,
            source=edge_key.source,
            target=edge_key.target,
            route_obj=route_obj,
            total_length_um=total_length_um,
            opened_cells=tuple(opened_cells),
        )
        self.lengths_by_id[job.net_id] = total_length_um
        if self.diagnostics_enabled and route_cells is not None:
            self.committed_dynamic_cells_by_id[job.net_id] = set(route_cells)
        else:
            self.committed_dynamic_cells_by_id.pop(job.net_id, None)

    def clear_route(self, net_id: int) -> None:
        self.records_by_id.pop(net_id, None)
        self.lengths_by_id.pop(net_id, None)
        self.committed_dynamic_cells_by_id.pop(net_id, None)

    def set_committed_cells(self, net_id: int, cells: set[tuple[int, int]]) -> None:
        if self.diagnostics_enabled:
            self.committed_dynamic_cells_by_id[net_id] = set(cells)
        else:
            self.committed_dynamic_cells_by_id.pop(net_id, None)

    def committed_dynamic_cells(
        self,
        *,
        exclude_net_id: int | None = None,
    ) -> set[tuple[int, int]]:
        if not self.diagnostics_enabled:
            return set()
        merged: set[tuple[int, int]] = set()
        for net_id, cells in self.committed_dynamic_cells_by_id.items():
            if exclude_net_id is not None and int(net_id) == int(exclude_net_id):
                continue
            merged.update(cells)
        return merged

    def restore_records(
        self,
        snapshot_records: dict[int, RoutedNetRecord],
        snapshot_lengths: dict[int, float],
    ) -> None:
        self.records_by_id.update(snapshot_records)
        self.lengths_by_id.update(snapshot_lengths)

    def ordered_records(self) -> list[RoutedNetRecord]:
        return [
            self.records_by_id[net_id]
            for net_id in self.route_order
            if net_id in self.records_by_id
        ]

    def routed_edge_lengths(self) -> dict[RoutedEdgeKey, float]:
        return routed_edge_lengths_from_records(self.ordered_records())


def static_obstacle_count_from_map(obstacle_map: object) -> int:
    blocked_cells = getattr(obstacle_map, "blocked_cells", ())
    static_obstacle_count = len(blocked_cells)
    build_stats = getattr(obstacle_map, "build_stats", None)
    if isinstance(build_stats, dict):
        raw_count = build_stats.get("blocked_cell_count")
        if isinstance(raw_count, int):
            static_obstacle_count = raw_count
    return int(static_obstacle_count)


def build_route_debug_artifacts(
    *,
    obstacle_svg: Path | None,
    route_svgs: list[Path],
    obstacle_map: object,
    routed_net_records: list[RoutedNetRecord],
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    route_search_summary: RouteSearchSummary | None = None,
    route_attempt_records: list[RouteAttemptRecord] | None = None,
) -> RustRouteDebugArtifacts:
    return RustRouteDebugArtifacts(
        obstacle_svg=obstacle_svg,
        route_svgs=route_svgs,
        routed_edge_lengths_um=routed_edge_lengths_from_records(routed_net_records),
        routed_net_records=routed_net_records,
        route_search_summary=route_search_summary or RouteSearchSummary(),
        route_attempt_records=list(route_attempt_records or []),
        # Keep meander planning base obstacles limited to layout-static geometry.
        # Port-access reservation lanes are routing-time guards and are added to
        # `static_cells` for net-to-net A* ordering, but they should not globally
        # block post-route meander box checks.
        static_blocked_cells=tuple(sorted(set(getattr(obstacle_map, "blocked_cells", ())))),
        static_obstacle_count=static_obstacle_count_from_map(obstacle_map),
        realization_grid_spec=realization_grid_spec,
        realization_allow_45_degree_turns=allow_45_degree_turns,
        realization_bend_radius_cells=bend_radius_cells,
    )
