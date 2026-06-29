"""Shared data structures for the Rust-backed photonic router."""

from __future__ import annotations

import math
from collections.abc import Iterable as IterableABC
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping

from gdsfactory.component import Component
from gdsfactory.typings import Port

from photonic_router.path_length_graph import PortRef, RoutedEdgeKey

PRIMITIVE_TRANSITION_CLASSES = (
    "straight_short",
    "straight_long",
    "bend45",
    "bend90",
)

DEFAULT_BEND_RADIUS_UM = 10.0
DEFAULT_MEANDER_MAX_HEIGHT_UM = 80.0


@dataclass(frozen=True)
class OpticalRouteClearancePolicy:
    """Grid radii used for optical routing and PLM occupancy.

    Static obstacles are already rasterized with their own physical clearance.
    Dynamic routes need two radii: one to expand existing dynamic obstacles
    during centerline search, and one to reserve the routed waveguide keepout
    for later routes. PLM uses the committed keepout as occupied space, while
    its candidate box clearance stays explicit so it can be changed separately.
    """

    route_width_um: float
    grid_size_um: float
    route_clearance_um: float
    waveguide_core_radius_cells: int
    dynamic_obstacle_search_expansion_radius_cells: int
    dynamic_route_commit_keepout_radius_cells: int
    dynamic_route_core_radius_cells: int
    plm_registered_route_keepout_radius_cells: int
    plm_candidate_box_clearance_radius_cells: int

    @classmethod
    def from_dimensions(
        cls,
        *,
        route_width_um: float,
        grid_size_um: float,
        route_clearance_um: float,
        plm_candidate_box_clearance_radius_cells: int = 0,
    ) -> "OpticalRouteClearancePolicy":
        route_width_um = float(route_width_um)
        grid_size_um = float(grid_size_um)
        route_clearance_um = max(0.0, float(route_clearance_um))
        if not math.isfinite(route_width_um) or route_width_um <= 0.0:
            raise ValueError("route_width_um must be finite and > 0")
        if not math.isfinite(grid_size_um) or grid_size_um <= 0.0:
            raise ValueError("grid_size_um must be finite and > 0")
        if not math.isfinite(route_clearance_um):
            raise ValueError("route_clearance_um must be finite")

        if route_clearance_um <= 0.0:
            core_radius_cells = 0
            keepout_radius_cells = 0
        else:
            core_radius_cells = max(
                0,
                math.ceil((route_width_um / 2.0) / grid_size_um),
            )
            keepout_radius_cells = max(
                core_radius_cells,
                math.ceil(((route_width_um / 2.0) + route_clearance_um) / grid_size_um),
            )
        box_clearance_radius_cells = max(
            0,
            int(plm_candidate_box_clearance_radius_cells),
        )
        return cls(
            route_width_um=route_width_um,
            grid_size_um=grid_size_um,
            route_clearance_um=route_clearance_um,
            waveguide_core_radius_cells=core_radius_cells,
            dynamic_obstacle_search_expansion_radius_cells=core_radius_cells,
            dynamic_route_commit_keepout_radius_cells=keepout_radius_cells,
            dynamic_route_core_radius_cells=0,
            plm_registered_route_keepout_radius_cells=keepout_radius_cells,
            plm_candidate_box_clearance_radius_cells=box_clearance_radius_cells,
        )

    def to_debug_dict(self) -> dict[str, float | int]:
        return {
            "route_width_um": self.route_width_um,
            "grid_size_um": self.grid_size_um,
            "route_clearance_um": self.route_clearance_um,
            "waveguide_core_radius_cells": self.waveguide_core_radius_cells,
            "dynamic_obstacle_search_expansion_radius_cells": (
                self.dynamic_obstacle_search_expansion_radius_cells
            ),
            "dynamic_route_commit_keepout_radius_cells": (
                self.dynamic_route_commit_keepout_radius_cells
            ),
            "dynamic_route_core_radius_cells": self.dynamic_route_core_radius_cells,
            "plm_registered_route_keepout_radius_cells": (
                self.plm_registered_route_keepout_radius_cells
            ),
            "plm_candidate_box_clearance_radius_cells": (
                self.plm_candidate_box_clearance_radius_cells
            ),
        }


def bend_radius_cells_from_um(
    bend_radius_um: float | None,
    *,
    grid_size_um: float,
) -> int:
    """Convert a minimum bend radius in micrometers to grid cells."""
    if bend_radius_um is None:
        bend_radius_um = DEFAULT_BEND_RADIUS_UM
    bend_radius_um = float(bend_radius_um)
    grid_size_um = float(grid_size_um)
    if not math.isfinite(bend_radius_um) or bend_radius_um <= 0.0:
        raise ValueError("bend_radius_um must be finite and > 0")
    if not math.isfinite(grid_size_um) or grid_size_um <= 0.0:
        raise ValueError("grid_size_um must be finite and > 0")
    return max(1, math.ceil(bend_radius_um / grid_size_um))


def _empty_primitive_counter_dict() -> dict[str, int]:
    return {name: 0 for name in PRIMITIVE_TRANSITION_CLASSES}


def _get_route_counter_dict(route_obj: object | None, attr: str) -> dict[str, int]:
    counters = _empty_primitive_counter_dict()
    if route_obj is None:
        return counters
    values = getattr(route_obj, attr, None)
    if not isinstance(values, IterableABC):
        return counters
    sequence = list(values)
    for name, value in zip(PRIMITIVE_TRANSITION_CLASSES, sequence):
        try:
            counters[name] = int(value)
        except (TypeError, ValueError):
            counters[name] = 0
    return counters


def _add_counter_dict(target: dict[str, int], source: Mapping[str, int]) -> None:
    for name in PRIMITIVE_TRANSITION_CLASSES:
        target[name] = int(target.get(name, 0)) + int(source.get(name, 0))


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
    route_search_summary: "RouteSearchSummary" = field(
        default_factory=lambda: RouteSearchSummary()
    )
    route_attempt_records: list["RouteAttemptRecord"] = field(default_factory=list)
    static_blocked_cells: tuple[tuple[int, int], ...] = ()
    static_obstacle_count: int = 0
    static_port_open_count: int = 0
    port_alignment_diagnostics: list[dict[str, object]] = field(default_factory=list)
    route_nets_timings_s: dict[str, float] = field(default_factory=dict)
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
    source_port_center_um: tuple[float, float] | None = None
    target_port_center_um: tuple[float, float] | None = None
    source_port_orientation_deg: float | None = None
    target_port_orientation_deg: float | None = None
    base_total_length_um: float | None = None
    corrected_centerline_um: tuple[tuple[float, float], ...] = ()
    endpoint_correction_error: str | None = None


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
    generated_neighbors: int = 0
    heap_pushes: int = 0
    heap_pops: int = 0
    skipped_duplicate_heap_entries: int = 0
    stale_generation_heap_entries: int = 0
    closed_heap_entries: int = 0
    max_heap_size: int = 0
    dense_search_states: int = 0
    dense_search_storage_bytes: int = 0
    best_cost_updates: int = 0
    parent_updates: int = 0
    obstacle_clearance_checks: int = 0
    window_attempts: int = 0
    window_rejects: int = 0
    footprint_rejects: int = 0
    primitive_generated_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_bounds_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_closed_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_cost_pruned_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_checks_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_accepted_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    footprint_checks: int = 0
    footprint_cells_tested: int = 0
    footprint_rect_checks: int = 0
    footprint_rect_rejects: int = 0
    dense_grid_build_failures: int = 0
    dense_grid_cells: int = 0
    route_search_total_time_us: int = 0
    dense_grid_build_time_us: int = 0
    search_loop_time_us: int = 0
    obstacle_map_prepare_time_us: int = 0
    simple_route_time_us: int = 0
    commit_prepare_time_us: int = 0
    commit_time_us: int = 0
    neighbor_generation_time_us: int = 0
    heap_operation_time_us: int = 0
    legality_check_time_us: int = 0
    reconstruction_time_us: int = 0
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
        self.generated_neighbors += _get_route_int_stat(route_obj, "generated_neighbors")
        self.heap_pushes += _get_route_int_stat(route_obj, "heap_pushes")
        self.heap_pops += _get_route_int_stat(route_obj, "heap_pops")
        self.skipped_duplicate_heap_entries += _get_route_int_stat(
            route_obj,
            "skipped_duplicate_heap_entries",
        )
        self.stale_generation_heap_entries += _get_route_int_stat(
            route_obj,
            "stale_generation_heap_entries",
        )
        self.closed_heap_entries += _get_route_int_stat(
            route_obj,
            "closed_heap_entries",
        )
        self.max_heap_size = max(
            self.max_heap_size,
            _get_route_int_stat(route_obj, "max_heap_size"),
        )
        self.dense_search_states += _get_route_int_stat(
            route_obj,
            "dense_search_states",
        )
        self.dense_search_storage_bytes += _get_route_int_stat(
            route_obj,
            "dense_search_storage_bytes",
        )
        self.best_cost_updates += _get_route_int_stat(
            route_obj,
            "best_cost_updates",
        )
        self.parent_updates += _get_route_int_stat(route_obj, "parent_updates")
        self.obstacle_clearance_checks += _get_route_int_stat(
            route_obj,
            "obstacle_clearance_checks",
        )
        self.window_attempts += _get_route_int_stat(route_obj, "window_attempts")
        self.window_rejects += _get_route_int_stat(route_obj, "window_rejects")
        self.footprint_rejects += _get_route_int_stat(route_obj, "footprint_rejects")
        _add_counter_dict(
            self.primitive_generated_by_class,
            _get_route_counter_dict(route_obj, "primitive_generated_by_class"),
        )
        _add_counter_dict(
            self.primitive_bounds_rejects_by_class,
            _get_route_counter_dict(route_obj, "primitive_bounds_rejects_by_class"),
        )
        _add_counter_dict(
            self.primitive_closed_rejects_by_class,
            _get_route_counter_dict(route_obj, "primitive_closed_rejects_by_class"),
        )
        _add_counter_dict(
            self.primitive_cost_pruned_by_class,
            _get_route_counter_dict(route_obj, "primitive_cost_pruned_by_class"),
        )
        _add_counter_dict(
            self.primitive_footprint_checks_by_class,
            _get_route_counter_dict(route_obj, "primitive_footprint_checks_by_class"),
        )
        _add_counter_dict(
            self.primitive_footprint_rejects_by_class,
            _get_route_counter_dict(route_obj, "primitive_footprint_rejects_by_class"),
        )
        _add_counter_dict(
            self.primitive_accepted_by_class,
            _get_route_counter_dict(route_obj, "primitive_accepted_by_class"),
        )
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
        self.route_search_total_time_us += _get_route_int_stat(
            route_obj,
            "route_search_total_time_us",
        )
        self.dense_grid_build_time_us += _get_route_int_stat(
            route_obj,
            "dense_grid_build_time_us",
        )
        self.search_loop_time_us += _get_route_int_stat(
            route_obj,
            "search_loop_time_us",
        )
        self.obstacle_map_prepare_time_us += _get_route_int_stat(
            route_obj,
            "obstacle_map_prepare_time_us",
        )
        self.simple_route_time_us += _get_route_int_stat(
            route_obj,
            "simple_route_time_us",
        )
        self.commit_prepare_time_us += _get_route_int_stat(
            route_obj,
            "commit_prepare_time_us",
        )
        self.commit_time_us += _get_route_int_stat(
            route_obj,
            "commit_time_us",
        )
        self.neighbor_generation_time_us += _get_route_int_stat(
            route_obj,
            "neighbor_generation_time_us",
        )
        self.heap_operation_time_us += _get_route_int_stat(
            route_obj,
            "heap_operation_time_us",
        )
        self.legality_check_time_us += _get_route_int_stat(
            route_obj,
            "legality_check_time_us",
        )
        self.reconstruction_time_us += _get_route_int_stat(
            route_obj,
            "reconstruction_time_us",
        )
        self.max_window_area_cells = max(
            self.max_window_area_cells,
            _get_route_int_stat(route_obj, "max_window_area_cells"),
        )
        if bool(getattr(route_obj, "used_full_grid_fallback", False)):
            self.full_grid_fallbacks += 1


@dataclass(frozen=True)
class RouteAttemptRecord:
    attempt_index: int
    bucket_name: str
    net_id: int
    route_index: int
    net_name: str
    source: str
    target: str
    elapsed_s: float
    failed: bool = False
    repair_round: int | None = None
    error: str | None = None
    total_length_um: float | None = None
    route_cells: int = 0
    expanded_states: int = 0
    generated_neighbors: int = 0
    heap_pushes: int = 0
    heap_pops: int = 0
    skipped_duplicate_heap_entries: int = 0
    stale_generation_heap_entries: int = 0
    closed_heap_entries: int = 0
    max_heap_size: int = 0
    dense_search_states: int = 0
    dense_search_storage_bytes: int = 0
    best_cost_updates: int = 0
    parent_updates: int = 0
    obstacle_clearance_checks: int = 0
    window_attempts: int = 0
    last_window_min_x: int = 0
    last_window_max_x: int = 0
    last_window_min_y: int = 0
    last_window_max_y: int = 0
    last_window_area_cells: int = 0
    primitive_generated_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_bounds_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_closed_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_cost_pruned_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_checks_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_accepted_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    footprint_checks: int = 0
    footprint_rect_checks: int = 0
    route_search_total_time_us: int = 0
    dense_grid_build_time_us: int = 0
    dense_grid_cells: int = 0
    search_loop_time_us: int = 0
    obstacle_map_prepare_time_us: int = 0
    simple_route_time_us: int = 0
    commit_prepare_time_us: int = 0
    commit_time_us: int = 0
    neighbor_generation_time_us: int = 0
    heap_operation_time_us: int = 0
    legality_check_time_us: int = 0
    reconstruction_time_us: int = 0
    max_window_area_cells: int = 0
    used_full_grid_fallback: bool = False
    diagnostics: dict[str, object] = field(default_factory=dict)

    @property
    def used_simple_route(self) -> bool:
        return not self.failed and self.expanded_states == 0

    def as_dict(self) -> dict[str, object]:
        return {
            "attempt_index": self.attempt_index,
            "bucket_name": self.bucket_name,
            "net_id": self.net_id,
            "route_index": self.route_index,
            "net_name": self.net_name,
            "source": self.source,
            "target": self.target,
            "elapsed_s": self.elapsed_s,
            "failed": self.failed,
            "repair_round": self.repair_round,
            "error": self.error,
            "total_length_um": self.total_length_um,
            "route_cells": self.route_cells,
            "used_simple_route": self.used_simple_route,
            "expanded_states": self.expanded_states,
            "generated_neighbors": self.generated_neighbors,
            "heap_pushes": self.heap_pushes,
            "heap_pops": self.heap_pops,
            "skipped_duplicate_heap_entries": self.skipped_duplicate_heap_entries,
            "stale_generation_heap_entries": self.stale_generation_heap_entries,
            "closed_heap_entries": self.closed_heap_entries,
            "max_heap_size": self.max_heap_size,
            "dense_search_states": self.dense_search_states,
            "dense_search_storage_bytes": self.dense_search_storage_bytes,
            "best_cost_updates": self.best_cost_updates,
            "parent_updates": self.parent_updates,
            "obstacle_clearance_checks": self.obstacle_clearance_checks,
            "window_attempts": self.window_attempts,
            "last_window_min_x": self.last_window_min_x,
            "last_window_max_x": self.last_window_max_x,
            "last_window_min_y": self.last_window_min_y,
            "last_window_max_y": self.last_window_max_y,
            "last_window_area_cells": self.last_window_area_cells,
            "primitive_generated_by_class": dict(self.primitive_generated_by_class),
            "primitive_bounds_rejects_by_class": dict(
                self.primitive_bounds_rejects_by_class
            ),
            "primitive_closed_rejects_by_class": dict(
                self.primitive_closed_rejects_by_class
            ),
            "primitive_cost_pruned_by_class": dict(self.primitive_cost_pruned_by_class),
            "primitive_footprint_checks_by_class": dict(
                self.primitive_footprint_checks_by_class
            ),
            "primitive_footprint_rejects_by_class": dict(
                self.primitive_footprint_rejects_by_class
            ),
            "primitive_accepted_by_class": dict(self.primitive_accepted_by_class),
            "footprint_checks": self.footprint_checks,
            "footprint_rect_checks": self.footprint_rect_checks,
            "route_search_total_time_s": (
                self.route_search_total_time_us / 1_000_000.0
            ),
            "dense_grid_build_time_s": self.dense_grid_build_time_us / 1_000_000.0,
            "dense_grid_cells": self.dense_grid_cells,
            "search_loop_time_s": self.search_loop_time_us / 1_000_000.0,
            "obstacle_map_prepare_time_s": (
                self.obstacle_map_prepare_time_us / 1_000_000.0
            ),
            "simple_route_time_s": self.simple_route_time_us / 1_000_000.0,
            "commit_prepare_time_s": self.commit_prepare_time_us / 1_000_000.0,
            "commit_time_s": self.commit_time_us / 1_000_000.0,
            "neighbor_generation_time_s": self.neighbor_generation_time_us / 1_000_000.0,
            "heap_operation_time_s": self.heap_operation_time_us / 1_000_000.0,
            "legality_check_time_s": self.legality_check_time_us / 1_000_000.0,
            "reconstruction_time_s": self.reconstruction_time_us / 1_000_000.0,
            "max_window_area_cells": self.max_window_area_cells,
            "used_full_grid_fallback": self.used_full_grid_fallback,
            "diagnostics": dict(self.diagnostics),
        }


def route_attempt_record_from_route(
    *,
    attempt_index: int,
    bucket_name: str,
    net_id: int,
    route_index: int,
    net_name: str,
    source: str,
    target: str,
    elapsed_s: float,
    route_obj: object | None = None,
    failed: bool = False,
    repair_round: int | None = None,
    error: str | None = None,
    diagnostics: Mapping[str, object] | None = None,
) -> RouteAttemptRecord:
    route_cells = getattr(route_obj, "cells", None) if route_obj is not None else None
    total_length_um = (
        _as_float(getattr(route_obj, "total_length_um"), 0.0)
        if route_obj is not None and hasattr(route_obj, "total_length_um")
        else None
    )
    return RouteAttemptRecord(
        attempt_index=int(attempt_index),
        bucket_name=bucket_name,
        net_id=int(net_id),
        route_index=int(route_index),
        net_name=net_name,
        source=source,
        target=target,
        elapsed_s=float(elapsed_s),
        failed=bool(failed),
        repair_round=repair_round,
        error=error,
        total_length_um=total_length_um,
        route_cells=len(route_cells or ()),
        expanded_states=_get_route_int_stat(route_obj, "expanded_states"),
        generated_neighbors=_get_route_int_stat(route_obj, "generated_neighbors"),
        heap_pushes=_get_route_int_stat(route_obj, "heap_pushes"),
        heap_pops=_get_route_int_stat(route_obj, "heap_pops"),
        skipped_duplicate_heap_entries=_get_route_int_stat(
            route_obj,
            "skipped_duplicate_heap_entries",
        ),
        stale_generation_heap_entries=_get_route_int_stat(
            route_obj,
            "stale_generation_heap_entries",
        ),
        closed_heap_entries=_get_route_int_stat(
            route_obj,
            "closed_heap_entries",
        ),
        max_heap_size=_get_route_int_stat(route_obj, "max_heap_size"),
        dense_search_states=_get_route_int_stat(route_obj, "dense_search_states"),
        dense_search_storage_bytes=_get_route_int_stat(
            route_obj,
            "dense_search_storage_bytes",
        ),
        best_cost_updates=_get_route_int_stat(route_obj, "best_cost_updates"),
        parent_updates=_get_route_int_stat(route_obj, "parent_updates"),
        obstacle_clearance_checks=_get_route_int_stat(
            route_obj,
            "obstacle_clearance_checks",
        ),
        window_attempts=_get_route_int_stat(route_obj, "window_attempts"),
        last_window_min_x=_get_route_int_stat(route_obj, "last_window_min_x"),
        last_window_max_x=_get_route_int_stat(route_obj, "last_window_max_x"),
        last_window_min_y=_get_route_int_stat(route_obj, "last_window_min_y"),
        last_window_max_y=_get_route_int_stat(route_obj, "last_window_max_y"),
        last_window_area_cells=_get_route_int_stat(route_obj, "last_window_area_cells"),
        primitive_generated_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_generated_by_class",
        ),
        primitive_bounds_rejects_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_bounds_rejects_by_class",
        ),
        primitive_closed_rejects_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_closed_rejects_by_class",
        ),
        primitive_cost_pruned_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_cost_pruned_by_class",
        ),
        primitive_footprint_checks_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_footprint_checks_by_class",
        ),
        primitive_footprint_rejects_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_footprint_rejects_by_class",
        ),
        primitive_accepted_by_class=_get_route_counter_dict(
            route_obj,
            "primitive_accepted_by_class",
        ),
        footprint_checks=_get_route_int_stat(route_obj, "primitive_footprint_checks"),
        footprint_rect_checks=_get_route_int_stat(
            route_obj,
            "primitive_footprint_rect_checks",
        ),
        route_search_total_time_us=_get_route_int_stat(
            route_obj,
            "route_search_total_time_us",
        ),
        dense_grid_build_time_us=_get_route_int_stat(
            route_obj,
            "dense_grid_build_time_us",
        ),
        dense_grid_cells=_get_route_int_stat(route_obj, "dense_grid_cells"),
        search_loop_time_us=_get_route_int_stat(
            route_obj,
            "search_loop_time_us",
        ),
        obstacle_map_prepare_time_us=_get_route_int_stat(
            route_obj,
            "obstacle_map_prepare_time_us",
        ),
        simple_route_time_us=_get_route_int_stat(
            route_obj,
            "simple_route_time_us",
        ),
        commit_prepare_time_us=_get_route_int_stat(
            route_obj,
            "commit_prepare_time_us",
        ),
        commit_time_us=_get_route_int_stat(
            route_obj,
            "commit_time_us",
        ),
        neighbor_generation_time_us=_get_route_int_stat(
            route_obj,
            "neighbor_generation_time_us",
        ),
        heap_operation_time_us=_get_route_int_stat(
            route_obj,
            "heap_operation_time_us",
        ),
        legality_check_time_us=_get_route_int_stat(
            route_obj,
            "legality_check_time_us",
        ),
        reconstruction_time_us=_get_route_int_stat(
            route_obj,
            "reconstruction_time_us",
        ),
        max_window_area_cells=_get_route_int_stat(route_obj, "max_window_area_cells"),
        used_full_grid_fallback=bool(
            getattr(route_obj, "used_full_grid_fallback", False)
        ),
        diagnostics=dict(diagnostics or {}),
    )


@dataclass(frozen=True)
class RouteSearchSummary:
    route_count: int = 0
    route_attempts: int = 0
    route_failures: int = 0
    simple_route_count: int = 0
    repair_count: int = 0
    astar_elapsed_s: float = 0.0
    endpoint_correction_time_s: float = 0.0
    endpoint_correction_calls: int = 0
    endpoint_correction_failures: int = 0
    normal_route_time_s: float = 0.0
    expanded_states: int = 0
    generated_neighbors: int = 0
    heap_pushes: int = 0
    heap_pops: int = 0
    skipped_duplicate_heap_entries: int = 0
    stale_generation_heap_entries: int = 0
    closed_heap_entries: int = 0
    max_heap_size: int = 0
    dense_search_states: int = 0
    dense_search_storage_bytes: int = 0
    best_cost_updates: int = 0
    parent_updates: int = 0
    obstacle_clearance_checks: int = 0
    primitive_generated_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_bounds_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_closed_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_cost_pruned_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_checks_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_footprint_rejects_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    primitive_accepted_by_class: dict[str, int] = field(
        default_factory=_empty_primitive_counter_dict
    )
    footprint_checks: int = 0
    footprint_rejects: int = 0
    footprint_rect_checks: int = 0
    footprint_rect_rejects: int = 0
    route_search_total_time_us: int = 0
    dense_grid_build_time_us: int = 0
    dense_grid_cells: int = 0
    search_loop_time_us: int = 0
    obstacle_map_prepare_time_us: int = 0
    simple_route_time_us: int = 0
    commit_prepare_time_us: int = 0
    commit_time_us: int = 0
    neighbor_generation_time_us: int = 0
    heap_operation_time_us: int = 0
    legality_check_time_us: int = 0
    reconstruction_time_us: int = 0
    max_window_area_cells: int = 0
    full_grid_fallbacks: int = 0


def summarize_route_search(
    route_timing_buckets: Mapping[str, RouteTimingBucket],
    *,
    route_count: int,
    simple_route_count: int,
    repair_count: int,
    astar_elapsed_s: float,
) -> RouteSearchSummary:
    """Aggregate route-search counters used by quiet benchmark reporting."""
    route_bucket_names = (
        "normal_route",
        "probe_route",
        "repair_failed_net",
        "reroute_victims",
    )
    buckets = [
        bucket
        for bucket_name in route_bucket_names
        if (bucket := route_timing_buckets.get(bucket_name)) is not None
    ]
    def sum_bucket_counters(attr: str) -> dict[str, int]:
        counters = _empty_primitive_counter_dict()
        for bucket in buckets:
            _add_counter_dict(counters, getattr(bucket, attr))
        return counters

    return RouteSearchSummary(
        route_count=int(route_count),
        route_attempts=sum(bucket.calls for bucket in buckets),
        route_failures=sum(bucket.failures for bucket in buckets),
        simple_route_count=int(simple_route_count),
        repair_count=int(repair_count),
        astar_elapsed_s=float(astar_elapsed_s),
        endpoint_correction_time_s=float(
            route_timing_buckets.get("endpoint_correction", RouteTimingBucket()).elapsed_s
        ),
        endpoint_correction_calls=int(
            route_timing_buckets.get("endpoint_correction", RouteTimingBucket()).calls
        ),
        endpoint_correction_failures=int(
            route_timing_buckets.get("endpoint_correction", RouteTimingBucket()).failures
        ),
        normal_route_time_s=float(
            route_timing_buckets.get("normal_route", RouteTimingBucket()).elapsed_s
        ),
        expanded_states=sum(bucket.expanded_states for bucket in buckets),
        generated_neighbors=sum(bucket.generated_neighbors for bucket in buckets),
        heap_pushes=sum(bucket.heap_pushes for bucket in buckets),
        heap_pops=sum(bucket.heap_pops for bucket in buckets),
        skipped_duplicate_heap_entries=sum(
            bucket.skipped_duplicate_heap_entries for bucket in buckets
        ),
        stale_generation_heap_entries=sum(
            bucket.stale_generation_heap_entries for bucket in buckets
        ),
        closed_heap_entries=sum(bucket.closed_heap_entries for bucket in buckets),
        max_heap_size=max((bucket.max_heap_size for bucket in buckets), default=0),
        dense_search_states=sum(bucket.dense_search_states for bucket in buckets),
        dense_search_storage_bytes=sum(
            bucket.dense_search_storage_bytes for bucket in buckets
        ),
        best_cost_updates=sum(bucket.best_cost_updates for bucket in buckets),
        parent_updates=sum(bucket.parent_updates for bucket in buckets),
        obstacle_clearance_checks=sum(
            bucket.obstacle_clearance_checks for bucket in buckets
        ),
        primitive_generated_by_class=sum_bucket_counters("primitive_generated_by_class"),
        primitive_bounds_rejects_by_class=sum_bucket_counters(
            "primitive_bounds_rejects_by_class"
        ),
        primitive_closed_rejects_by_class=sum_bucket_counters(
            "primitive_closed_rejects_by_class"
        ),
        primitive_cost_pruned_by_class=sum_bucket_counters("primitive_cost_pruned_by_class"),
        primitive_footprint_checks_by_class=sum_bucket_counters(
            "primitive_footprint_checks_by_class"
        ),
        primitive_footprint_rejects_by_class=sum_bucket_counters(
            "primitive_footprint_rejects_by_class"
        ),
        primitive_accepted_by_class=sum_bucket_counters("primitive_accepted_by_class"),
        footprint_checks=sum(bucket.footprint_checks for bucket in buckets),
        footprint_rejects=sum(bucket.footprint_rejects for bucket in buckets),
        footprint_rect_checks=sum(bucket.footprint_rect_checks for bucket in buckets),
        footprint_rect_rejects=sum(bucket.footprint_rect_rejects for bucket in buckets),
        route_search_total_time_us=sum(
            bucket.route_search_total_time_us for bucket in buckets
        ),
        dense_grid_build_time_us=sum(bucket.dense_grid_build_time_us for bucket in buckets),
        dense_grid_cells=sum(bucket.dense_grid_cells for bucket in buckets),
        search_loop_time_us=sum(bucket.search_loop_time_us for bucket in buckets),
        obstacle_map_prepare_time_us=sum(
            bucket.obstacle_map_prepare_time_us for bucket in buckets
        ),
        simple_route_time_us=sum(bucket.simple_route_time_us for bucket in buckets),
        commit_prepare_time_us=sum(bucket.commit_prepare_time_us for bucket in buckets),
        commit_time_us=sum(bucket.commit_time_us for bucket in buckets),
        neighbor_generation_time_us=sum(
            bucket.neighbor_generation_time_us for bucket in buckets
        ),
        heap_operation_time_us=sum(bucket.heap_operation_time_us for bucket in buckets),
        legality_check_time_us=sum(bucket.legality_check_time_us for bucket in buckets),
        reconstruction_time_us=sum(bucket.reconstruction_time_us for bucket in buckets),
        max_window_area_cells=max(
            (bucket.max_window_area_cells for bucket in buckets),
            default=0,
        ),
        full_grid_fallbacks=sum(bucket.full_grid_fallbacks for bucket in buckets),
    )


DEFAULT_MEANDER_MIN_STRAIGHT_UM = 1.0


@dataclass(frozen=True)
class MeanderInsertionConfig:
    enabled: bool = True
    min_candidate_straight_length_um: float = DEFAULT_MEANDER_MIN_STRAIGHT_UM
    max_extra_length_per_region_um: float = 200.0
    conservative_legal_check: bool = True
    max_meander_height_um: float = DEFAULT_MEANDER_MAX_HEIGHT_UM
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
    pipeline_timings_s: dict[str, float] = field(default_factory=dict)
