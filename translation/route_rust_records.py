"""Route record bookkeeping for the Rust-backed photonic router."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Protocol, Sequence, cast

from photonic_router.path_length_graph import PortRef, RoutedEdgeKey

from translation.route_rust_types import (
    RouteJob,
    RouteAttemptRecord,
    RouteSearchSummary,
    RoutedNetRecord,
    RustRouteDebugArtifacts,
)


def _port_center_um(port: object) -> tuple[float, float] | None:
    center = getattr(port, "center", None)
    if center is None:
        center = getattr(port, "dcenter", None)
    if center is None:
        return None
    center_seq = cast(Sequence[Any], center)
    try:
        return (float(center_seq[0]), float(center_seq[1]))
    except (TypeError, ValueError, IndexError):
        return None


def _port_orientation_deg(port: object) -> float | None:
    orientation = getattr(port, "orientation", None)
    if orientation is None:
        return None
    try:
        return float(cast(float | str, orientation))
    except (TypeError, ValueError):
        return None


def _route_endpoint_cells(
    route_obj: object,
) -> tuple[tuple[int, int] | None, tuple[int, int] | None]:
    states = getattr(route_obj, "states", None)
    if not states:
        return None, None
    try:
        first = states[0]
        last = states[-1]
        return (int(first.x), int(first.y)), (int(last.x), int(last.y))
    except (AttributeError, TypeError, ValueError, IndexError):
        return None, None


def _port_ref_label(ref: PortRef) -> str:
    return f"{ref.instance}.{ref.port}"


def _grid_cell_center_um(
    cell: tuple[int, int] | None,
    *,
    grid_size_um: float,
    origin_x_um: float,
    origin_y_um: float,
) -> tuple[float, float] | None:
    if cell is None:
        return None
    return (
        float(origin_x_um) + (float(cell[0]) + 0.5) * float(grid_size_um),
        float(origin_y_um) + (float(cell[1]) + 0.5) * float(grid_size_um),
    )


def _alignment_entry(
    *,
    port_center_um: tuple[float, float] | None,
    route_cell: tuple[int, int] | None,
    route_center_um: tuple[float, float] | None,
    orientation_deg: float | None,
) -> dict[str, object]:
    entry: dict[str, object] = {
        "port_center_um": list(port_center_um) if port_center_um is not None else None,
        "route_cell": list(route_cell) if route_cell is not None else None,
        "route_grid_center_um": (
            list(route_center_um) if route_center_um is not None else None
        ),
        "port_orientation_deg": orientation_deg,
        "mu_x_um": None,
        "mu_y_um": None,
        "offset_abs_um": None,
    }
    if port_center_um is None or route_center_um is None:
        return entry
    mu_x = float(port_center_um[0]) - float(route_center_um[0])
    mu_y = float(port_center_um[1]) - float(route_center_um[1])
    entry["mu_x_um"] = mu_x
    entry["mu_y_um"] = mu_y
    entry["offset_abs_um"] = (mu_x * mu_x + mu_y * mu_y) ** 0.5
    return entry


def format_port_endpoint_correction_error(
    record: RoutedNetRecord,
    reason: object,
    *,
    realization_grid_spec: tuple[int, int, float, float, float] | None = None,
) -> str:
    source_cell, target_cell = _route_endpoint_cells(record.route_obj)
    source_center = None
    target_center = None
    if realization_grid_spec is not None:
        _, _, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
        source_center = _grid_cell_center_um(
            source_cell,
            grid_size_um=float(grid_size_um),
            origin_x_um=float(origin_x_um),
            origin_y_um=float(origin_y_um),
        )
        target_center = _grid_cell_center_um(
            target_cell,
            grid_size_um=float(grid_size_um),
            origin_x_um=float(origin_x_um),
            origin_y_um=float(origin_y_um),
        )

    return (
        "Grid-to-port endpoint correction failed for net "
        f"{record.net_name!r} ({_port_ref_label(record.source)} -> "
        f"{_port_ref_label(record.target)}): {reason}. "
        f"source_port_um={record.source_port_center_um}, "
        f"target_port_um={record.target_port_center_um}, "
        f"source_route_cell={source_cell}, target_route_cell={target_cell}, "
        f"source_route_center_um={source_center}, "
        f"target_route_center_um={target_center}."
    )


def build_port_alignment_diagnostics(
    records: list[RoutedNetRecord],
    *,
    realization_grid_spec: tuple[int, int, float, float, float],
) -> list[dict[str, object]]:
    """Describe current port-to-grid endpoint deltas without changing routes."""
    _, _, grid_size_um, origin_x_um, origin_y_um = realization_grid_spec
    diagnostics: list[dict[str, object]] = []
    for record in records:
        source_cell, target_cell = _route_endpoint_cells(record.route_obj)
        source_center = _grid_cell_center_um(
            source_cell,
            grid_size_um=float(grid_size_um),
            origin_x_um=float(origin_x_um),
            origin_y_um=float(origin_y_um),
        )
        target_center = _grid_cell_center_um(
            target_cell,
            grid_size_um=float(grid_size_um),
            origin_x_um=float(origin_x_um),
            origin_y_um=float(origin_y_um),
        )
        source = _alignment_entry(
            port_center_um=record.source_port_center_um,
            route_cell=source_cell,
            route_center_um=source_center,
            orientation_deg=record.source_port_orientation_deg,
        )
        target = _alignment_entry(
            port_center_um=record.target_port_center_um,
            route_cell=target_cell,
            route_center_um=target_center,
            orientation_deg=record.target_port_orientation_deg,
        )
        offsets = [
            value
            for value in (
                source.get("offset_abs_um"),
                target.get("offset_abs_um"),
            )
            if isinstance(value, (int, float))
        ]
        diagnostics.append(
            {
                "net_name": record.net_name,
                "source": {
                    "instance": record.source.instance,
                    "port": record.source.port,
                    **source,
                },
                "target": {
                    "instance": record.target.instance,
                    "port": record.target.port,
                    **target,
                },
                "route_total_length_um": float(record.total_length_um),
                "max_endpoint_offset_abs_um": max(offsets) if offsets else None,
            }
        )
    return diagnostics


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


def _centerline_tuple(points: object) -> tuple[tuple[float, float], ...]:
    if not isinstance(points, list):
        return ()
    out: list[tuple[float, float]] = []
    for point in points:
        if not isinstance(point, (tuple, list)) or len(point) != 2:
            return ()
        try:
            out.append((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            return ()
    return tuple(out)


class EndpointCorrectionRouter(Protocol):
    def route_port_corrected_centerline(
        self,
        route: object,
        *,
        source_port_um: tuple[float, float] | None = None,
        target_port_um: tuple[float, float] | None = None,
    ) -> object:
        ...

    def centerline_length_um(self, centerline: list[tuple[float, float]]) -> float:
        ...


def apply_port_endpoint_corrections(
    records: list[RoutedNetRecord],
    *,
    router: EndpointCorrectionRouter,
    realization_grid_spec: tuple[int, int, float, float, float] | None = None,
) -> list[RoutedNetRecord]:
    """Attach corrected physical centerlines and lengths to routed records."""
    updated: list[RoutedNetRecord] = []
    for record in records:
        if record.corrected_centerline_um:
            updated.append(record)
            continue
        if record.endpoint_correction_error is not None:
            updated.append(record)
            continue
        source_port = record.source_port_center_um
        target_port = record.target_port_center_um
        if source_port is None and target_port is None:
            updated.append(record)
            continue
        try:
            raw_centerline = router.route_port_corrected_centerline(
                record.route_obj,
                source_port_um=source_port,
                target_port_um=target_port,
            )
        except (TypeError, ValueError) as exc:
            message = format_port_endpoint_correction_error(
                record,
                exc,
                realization_grid_spec=realization_grid_spec,
            )
            print("ERROR: " + message)
            updated.append(replace(record, endpoint_correction_error=message))
            continue
        centerline = _centerline_tuple(raw_centerline)
        if not centerline:
            message = format_port_endpoint_correction_error(
                record,
                "router returned an empty or invalid corrected centerline",
                realization_grid_spec=realization_grid_spec,
            )
            print("ERROR: " + message)
            updated.append(replace(record, endpoint_correction_error=message))
            continue
        corrected_length_um = float(router.centerline_length_um(list(centerline)))
        updated.append(
            replace(
                record,
                total_length_um=corrected_length_um,
                base_total_length_um=(
                    record.base_total_length_um
                    if record.base_total_length_um is not None
                    else float(record.total_length_um)
                ),
                corrected_centerline_um=centerline,
                endpoint_correction_error=None,
            )
        )
    return updated


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
        corrected_centerline_um: tuple[tuple[float, float], ...] = (),
        corrected_total_length_um: float | None = None,
    ) -> None:
        edge_key = route_edge_key(job)
        route_total_length_um = float(getattr(route_obj, "total_length_um"))
        total_length_um = (
            float(corrected_total_length_um)
            if corrected_total_length_um is not None
            else route_total_length_um
        )
        self.records_by_id[job.net_id] = RoutedNetRecord(
            net_name=job.net_name,
            source=edge_key.source,
            target=edge_key.target,
            route_obj=route_obj,
            total_length_um=total_length_um,
            opened_cells=tuple(opened_cells),
            source_port_center_um=_port_center_um(job.source_port),
            target_port_center_um=_port_center_um(job.target_port),
            source_port_orientation_deg=_port_orientation_deg(job.source_port),
            target_port_orientation_deg=_port_orientation_deg(job.target_port),
            base_total_length_um=(
                route_total_length_um if corrected_centerline_um else None
            ),
            corrected_centerline_um=corrected_centerline_um,
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


def static_port_open_count_from_map(obstacle_map: object) -> int:
    port_open_cells = getattr(obstacle_map, "port_open_cells", ())
    port_open_count = len(port_open_cells)
    build_stats = getattr(obstacle_map, "build_stats", None)
    if isinstance(build_stats, dict):
        raw_count = build_stats.get("port_open_cell_count")
        if isinstance(raw_count, int):
            port_open_count = raw_count
    return int(port_open_count)


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
        static_port_open_count=static_port_open_count_from_map(obstacle_map),
        port_alignment_diagnostics=build_port_alignment_diagnostics(
            routed_net_records,
            realization_grid_spec=realization_grid_spec,
        ),
        realization_grid_spec=realization_grid_spec,
        realization_allow_45_degree_turns=allow_45_degree_turns,
        realization_bend_radius_cells=bend_radius_cells,
    )
