"""Shared data structures for the Rust-backed photonic router."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.typings import Port

from photonic_router.path_length_graph import PortRef, RoutedEdgeKey


def _as_float(value: object, default: float = 0.0) -> float:
    try:
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return float(value)
        return default
    except (TypeError, ValueError):
        return default


def _as_int(value: object, default: int = 0) -> int:
    try:
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, (int, float, str, bytes, bytearray)):
            return int(value)
        return default
    except (TypeError, ValueError):
        return default


def _get_route_int_stat(route_obj: object, attr: str) -> int:
    return max(0, _as_int(getattr(route_obj, attr, 0), 0))


@dataclass(frozen=True)
class RustRouteDebugArtifacts:
    obstacle_svg: Path | None
    route_svgs: list[Path]
    routed_edge_lengths_um: dict[RoutedEdgeKey, float]
    routed_net_records: list["RoutedNetRecord"] = field(default_factory=list)
    static_blocked_cells: tuple[tuple[int, int], ...] = ()
    static_obstacle_count: int = 0
    realization_grid_spec: tuple[int, int, float, float, float] | None = None
    realization_allow_45_degree_turns: bool = True
    realization_bend_radius_cells: int = 4


@dataclass(frozen=True)
class RoutedNetRecord:
    net_name: str
    source: PortRef
    target: PortRef
    route_obj: object
    total_length_um: float
    meander_auto_plan: dict[str, object] | None = None
    opened_cells: tuple[tuple[int, int], ...] = ()


@dataclass(frozen=True)
class RouteJob:
    net_id: int
    route_index: int
    net_name: str
    inst1: str
    port1: str
    inst2: str
    port2: str
    source_port: Port
    target_port: Port


@dataclass(frozen=True)
class RipupRerouteConfig:
    enabled: bool = True
    max_rounds: int = 4
    max_victims_per_failure: int = 8
    history_weight: float = 2.0
    history_increment: int = 1


@dataclass
class RouteTimingBucket:
    calls: int = 0
    failures: int = 0
    elapsed_s: float = 0.0
    expanded_states: int = 0
    window_attempts: int = 0
    window_rejects: int = 0
    footprint_rejects: int = 0
    footprint_checks: int = 0
    footprint_cells_tested: int = 0
    footprint_rect_checks: int = 0
    footprint_rect_rejects: int = 0
    dense_grid_build_failures: int = 0
    dense_grid_cells: int = 0
    dense_grid_build_time_us: int = 0
    max_window_area_cells: int = 0
    full_grid_fallbacks: int = 0

    @property
    def successes(self) -> int:
        return self.calls - self.failures

    def record_elapsed(self, elapsed_s: float, *, failed: bool = False) -> None:
        self.calls += 1
        if failed:
            self.failures += 1
        self.elapsed_s += elapsed_s

    def record_route(self, elapsed_s: float, route_obj: object, *, failed: bool = False) -> None:
        self.record_elapsed(elapsed_s, failed=failed)
        if failed:
            return
        self.expanded_states += _get_route_int_stat(route_obj, "expanded_states")
        self.window_attempts += _get_route_int_stat(route_obj, "window_attempts")
        self.window_rejects += _get_route_int_stat(route_obj, "window_rejects")
        self.footprint_rejects += _get_route_int_stat(route_obj, "footprint_rejects")
        self.footprint_checks += _get_route_int_stat(route_obj, "primitive_footprint_checks")
        self.footprint_cells_tested += _get_route_int_stat(
            route_obj,
            "primitive_footprint_cells_tested",
        )
        self.footprint_rect_checks += _get_route_int_stat(
            route_obj,
            "primitive_footprint_rect_checks",
        )
        self.footprint_rect_rejects += _get_route_int_stat(
            route_obj,
            "primitive_footprint_rect_rejects",
        )
        self.dense_grid_build_failures += _get_route_int_stat(
            route_obj,
            "dense_grid_build_failures",
        )
        self.dense_grid_cells += _get_route_int_stat(route_obj, "dense_grid_cells")
        self.dense_grid_build_time_us += _get_route_int_stat(
            route_obj,
            "dense_grid_build_time_us",
        )
        self.max_window_area_cells = max(
            self.max_window_area_cells,
            _get_route_int_stat(route_obj, "max_window_area_cells"),
        )
        if bool(getattr(route_obj, "used_full_grid_fallback", False)):
            self.full_grid_fallbacks += 1


@dataclass(frozen=True)
class MeanderInsertionConfig:
    enabled: bool = True
    min_candidate_straight_length_um: float = 2.0
    max_extra_length_per_region_um: float = 200.0
    conservative_legal_check: bool = True
    max_meander_height_um: float = 20.0
    auto_meander_endpoint_inset_um: float | None = None


@dataclass(frozen=True)
class MeanderInsertionResult:
    edge: RoutedEdgeKey
    requested_extra_length_um: float
    inserted_extra_length_um: float
    status: str
    reason: str


@dataclass(frozen=True)
class MeanderInsertionReport:
    results: list[MeanderInsertionResult]
    total_requested_extra_length_um: float
    total_inserted_extra_length_um: float
    unmatched_length_um: float


@dataclass(frozen=True)
class RouteRustPipelineResult:
    routed_layout: Component
    debug_artifacts: RustRouteDebugArtifacts
    path_length_analysis_info: dict[str, object] | None = None
    meander_requirements_info: list[dict[str, object]] | None = None
    meander_insertion_report_info: dict[str, object] | None = None
