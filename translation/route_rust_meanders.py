"""Meander planning helpers for path-length matching."""

from __future__ import annotations

import importlib
import math
import time
from collections import Counter
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any, Protocol, cast

from photonic_router.path_length_graph import (
    DelayInsertionCandidate,
    MissingLengthRequirement,
    PortRef,
    RoutedEdgeKey,
)

from translation.route_rust_analysis import edge_key_to_dict
from translation.route_rust_types import (
    DEFAULT_MEANDER_MIN_STRAIGHT_UM,
    MeanderInsertionConfig,
    MeanderInsertionReport,
    MeanderInsertionResult,
    RoutedNetRecord,
    _as_float,
    _as_int,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
_load_rust_backend = _sob._load_rust_backend

EXACT_MEANDER_EPS_UM = 1.0e-6
GridCell = tuple[int, int]


class _RustBackendProtocol(Protocol):
    def GridSpec(self, *args: object, **kwargs: object) -> object: ...
    def PrimitiveLibraryConfig(self, *args: object, **kwargs: object) -> object: ...
    def AStarConfig(self, *args: object, **kwargs: object) -> object: ...
    def auto_meander_search_config_rs(
        self,
        *args: object,
        **kwargs: object,
    ) -> dict[str, object]: ...
    def PyPhotonicRouter(self, *args: object, **kwargs: object) -> "_MeanderRouterProtocol": ...


class _MeanderRouterProtocol(Protocol):
    def set_static_cells(self, cells: list[GridCell]) -> None: ...
    def add_static_cells(self, cells: list[GridCell]) -> None: ...
    def add_registered_meander_reserved_cells(self, cells: list[GridCell]) -> int: ...
    def add_registered_meander_reserved_grid_rect(
        self,
        min_x: int,
        max_x: int,
        min_y: int,
        max_y: int,
    ) -> int: ...
    def register_meander_route_cells_as_static(
        self,
        routes: list[object],
        base_static_cells: list[GridCell],
        route_occupancy_radius_cells: int = 0,
    ) -> tuple[list[int], list[int], int]: ...
    def set_static_and_register_meander_route_cells_as_static(
        self,
        routes: list[object],
        base_static_cells: list[GridCell],
        route_occupancy_radius_cells: int = 0,
    ) -> tuple[list[int], list[int], int]: ...
    def set_static_and_register_meander_route_cells_as_static_handle(
        self,
        routes: list[object],
        base_static_cell_handle: object,
        route_occupancy_radius_cells: int = 0,
    ) -> tuple[list[int], list[int], int]: ...
    def last_meander_registration_profile(self) -> dict[str, int | float]: ...
    def register_meander_route_geometries(
        self,
        centerlines: list[list[tuple[float, float]]],
        registered_opened_cell_indices: list[int],
        max_bumps_by_edge: list[int],
    ) -> list[int]: ...
    def plan_auto_analytic_meander_for_centerline_depth_sweep_registered_opened(
        self,
        centerline: list[tuple[float, float]],
        registered_opened_cells_index: int,
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_requirement_candidates_registered_opened(
        self,
        candidate_centerlines: list[list[list[tuple[float, float]]]],
        candidate_registered_opened_cell_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[float],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_requirement_candidates_registered_opened_endpoint_sweep(
        self,
        candidate_centerlines: list[list[list[tuple[float, float]]]],
        candidate_registered_opened_cell_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[float],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_requirement_candidates_registered_opened_auto_config(
        self,
        candidate_centerlines: list[list[list[tuple[float, float]]]],
        candidate_registered_opened_cell_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[float],
        candidate_max_bumps_by_edge: list[list[int]],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_requirement_candidate_indices_registered_opened(
        self,
        candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[float],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_requirement_candidate_indices_registered_opened_endpoint_sweep(
        self,
        candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[float],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_requirement_candidate_indices_registered_opened_auto_config(
        self,
        candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[float],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_for_centerline_depth_sweep(
        self,
        centerline: list[tuple[float, float]],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_auto_analytic_meander_for_route_depth_sweep(
        self,
        route: object,
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def route_port_corrected_centerline(
        self,
        route: object,
        **kwargs: Any,
    ) -> list[tuple[float, float]]: ...
    def probe_auto_analytic_meander_for_centerline_depth_sweep(
        self,
        centerline: list[tuple[float, float]],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def probe_auto_analytic_meander_for_route_depth_sweep(
        self,
        route: object,
        **kwargs: Any,
    ) -> dict[str, object]: ...


PlannedEdgeInsertion = tuple[
    RoutedEdgeKey,
    RoutedNetRecord,
    dict[str, object],
    bool,
    int,
    set[GridCell],
]
EdgePlanAttempt = tuple[
    dict[str, object],
    dict[str, object] | None,
    RoutedNetRecord | None,
    bool,
    int,
    set[GridCell],
    float,
    float,
    Exception | None,
]
RequirementCandidatesAttempt = tuple[
    int | None,
    list[PlannedEdgeInsertion],
    list[list[dict[str, object]]],
    list[list[int]],
    list[int],
    list[int],
    list[float],
    Exception | None,
]


@dataclass
class _CandidateWorkItem:
    candidate: DelayInsertionCandidate
    affected_edges: tuple[RoutedEdgeKey, ...]
    requested: float
    info: dict[str, object]
    edge_attempts: list[dict[str, object]]
    already_satisfied: bool = False


@dataclass(frozen=True)
class _MeanderSearchConfig:
    min_straight_um: float
    min_segment_um: float
    max_height_um: float
    box_depths_um: list[float]
    endpoint_inset_um: float
    endpoint_insets_um: list[float]

def _float_list_from_mapping_value(value: object) -> list[float] | None:
    if not isinstance(value, list):
        return None
    values: list[float] = []
    for item in value:
        try:
            values.append(float(item))
        except (TypeError, ValueError):
            return None
    return values

@dataclass
class _MeanderPlannerContext:
    router: _MeanderRouterProtocol
    by_edge: dict[RoutedEdgeKey, RoutedNetRecord]
    updated: dict[RoutedEdgeKey, RoutedNetRecord]
    route_cells_by_edge: dict[RoutedEdgeKey, set[GridCell]]
    route_cell_refcounts: Counter[GridCell]
    registered_open_cell_index_by_edge: dict[RoutedEdgeKey, int]
    registered_open_cell_count_by_edge: dict[RoutedEdgeKey, int]
    registered_geometry_index_by_edge: dict[RoutedEdgeKey, int]
    base_open_cells_by_edge: dict[RoutedEdgeKey, set[GridCell]]
    base_open_cell_lists_by_edge: dict[RoutedEdgeKey, list[GridCell]]
    max_bumps_by_edge: dict[RoutedEdgeKey, int]
    centerline_lists_by_edge: dict[RoutedEdgeKey, list[tuple[float, float]]]
    base_static_cells: set[GridCell]
    reserved_meander_cells: set[GridCell]
    pending_reserved_meander_rects: list[tuple[int, int, int, int]]
    grid_size_um: float
    bend_radius_um: float
    setup_profile: dict[str, float]
    candidate_setup_profile: dict[str, float]
    commit_profile: dict[str, float]
    rust_planner_profile: dict[str, float]
    rust_wrapper_profile: dict[str, float]
    candidate_overhead_s: float = 0.0
    commit_elapsed_s: float = 0.0
    grid_width_cells: int = 0
    grid_height_cells: int = 0
    route_occupancy_radius_cells: int = 0
    meander_box_clearance_radius_cells: int = 0

    def _add_candidate_setup_time(self, key: str, elapsed_s: float) -> None:
        self.candidate_setup_profile[key] = (
            self.candidate_setup_profile.get(key, 0.0) + max(0.0, float(elapsed_s))
        )

    def _add_commit_time(self, key: str, elapsed_s: float) -> None:
        self.commit_profile[key] = (
            self.commit_profile.get(key, 0.0) + max(0.0, float(elapsed_s))
        )

    def add_rust_planner_profile(self, profile: object) -> None:
        self._add_numeric_profile(self.rust_planner_profile, profile)

    def add_rust_wrapper_profile(self, profile: object) -> None:
        self._add_numeric_profile(self.rust_wrapper_profile, profile)

    def registered_requirement_candidate_engine(self) -> str | None:
        if (
            hasattr(
                self.router,
                "plan_auto_analytic_meander_requirement_candidate_indices_registered_opened_auto_config",
            )
            and bool(self.registered_geometry_index_by_edge)
        ):
            return "rust_registered_geometry_indices_auto_config"
        if hasattr(
            self.router,
            "plan_auto_analytic_meander_requirement_candidates_registered_opened_auto_config",
        ):
            return "rust_registered_centerlines_auto_config"
        if (
            hasattr(
                self.router,
                "plan_auto_analytic_meander_requirement_candidate_indices_registered_opened_endpoint_sweep",
            )
            and bool(self.registered_geometry_index_by_edge)
        ):
            return "rust_registered_geometry_indices"
        if hasattr(
            self.router,
            "plan_auto_analytic_meander_requirement_candidates_registered_opened_endpoint_sweep",
        ):
            return "rust_registered_centerlines"
        return None

    @staticmethod
    def _add_numeric_profile(target: dict[str, float], profile: object) -> None:
        if not isinstance(profile, Mapping):
            return
        for key, value in profile.items():
            if not isinstance(key, str):
                continue
            try:
                numeric_value = float(value)
            except (TypeError, ValueError):
                continue
            target[key] = target.get(key, 0.0) + numeric_value

    def reserved_meander_cells_for_python_planning(self) -> set[GridCell]:
        if not self.pending_reserved_meander_rects:
            return self.reserved_meander_cells

        t_materialize_start = time.perf_counter()
        for min_x, max_x, min_y, max_y in self.pending_reserved_meander_rects:
            for x in range(min_x, max_x + 1):
                for y in range(min_y, max_y + 1):
                    self.reserved_meander_cells.add((x, y))
        self.pending_reserved_meander_rects.clear()
        self._add_candidate_setup_time(
            "materialize_reserved_rects_s",
            time.perf_counter() - t_materialize_start,
        )
        return self.reserved_meander_cells

    def _reserved_rects_overlap_grid_rect(
        self,
        parsed_grid_rect: tuple[int, int, int, int],
    ) -> bool:
        for reserved_rect in self.pending_reserved_meander_rects:
            if _grid_rects_overlap(parsed_grid_rect, reserved_rect):
                return True
        return False

    def ensure_route_cell_refcounts(self) -> None:
        if self.route_cell_refcounts:
            return

        t_refcount_start = time.perf_counter()
        for edge_key, record in self.by_edge.items():
            route_cells = self.route_cells_by_edge.get(edge_key)
            if route_cells is None:
                route_cells = _record_route_cells(record)
                if self.route_occupancy_radius_cells > 0:
                    route_cells = _inflate_grid_cells(
                        route_cells,
                        radius_cells=self.route_occupancy_radius_cells,
                        width_cells=self.grid_width_cells,
                        height_cells=self.grid_height_cells,
                    )
                self.route_cells_by_edge[edge_key] = route_cells
            self.route_cell_refcounts.update(route_cells)
        self._add_candidate_setup_time(
            "lazy_route_cell_refcount_s",
            time.perf_counter() - t_refcount_start,
        )

    def _candidate_grid_rect_overlaps_blocked(
        self,
        parsed_grid_rect: tuple[int, int, int, int],
        *,
        extra_blocked_cells: set[GridCell] | None = None,
    ) -> tuple[bool, int]:
        base_overlap_count = _grid_rect_cell_set_overlap_count(
            parsed_grid_rect,
            self.base_static_cells,
            stop_after_first=True,
        )
        if base_overlap_count > 0:
            return True, base_overlap_count
        if self._reserved_rects_overlap_grid_rect(parsed_grid_rect):
            return True, 1
        reserved_overlap_count = _grid_rect_cell_set_overlap_count(
            parsed_grid_rect,
            self.reserved_meander_cells,
            stop_after_first=True,
        )
        if reserved_overlap_count > 0:
            return True, reserved_overlap_count
        if extra_blocked_cells:
            extra_overlap_count = _grid_rect_cell_set_overlap_count(
                parsed_grid_rect,
                extra_blocked_cells,
                stop_after_first=True,
            )
            if extra_overlap_count > 0:
                return True, extra_overlap_count
        return False, 0

    def base_open_cells_for_edge(
        self,
        edge_key: RoutedEdgeKey,
        record: RoutedNetRecord,
    ) -> set[GridCell]:
        cells = self.base_open_cells_by_edge.get(edge_key)
        if cells is None:
            if not self.route_cell_refcounts:
                self.ensure_route_cell_refcounts()
            route_cells = self.route_cells_by_edge.get(edge_key)
            if route_cells is None:
                route_cells = _record_route_cells(record)
                self.route_cells_by_edge[edge_key] = route_cells
            cells = {
                cell
                for cell in route_cells
                if self.route_cell_refcounts.get(cell, 0) == 1
                and cell not in self.base_static_cells
            }
            self.base_open_cells_by_edge[edge_key] = cells
        return cells

    def base_open_cell_list_for_edge(
        self,
        edge_key: RoutedEdgeKey,
        base_open_cells: set[GridCell],
    ) -> list[GridCell]:
        cells = self.base_open_cell_lists_by_edge.get(edge_key)
        if cells is None:
            cells = sorted(base_open_cells)
            self.base_open_cell_lists_by_edge[edge_key] = cells
        return cells

    def max_bumps_for_edge(
        self,
        edge_key: RoutedEdgeKey,
        record: RoutedNetRecord,
    ) -> int:
        max_bumps = self.max_bumps_by_edge.get(edge_key)
        if max_bumps is None:
            max_bumps = _route_geometry_max_meander_bumps(
                record=record,
                grid_size_um=self.grid_size_um,
                bend_radius_um=self.bend_radius_um,
            )
            self.max_bumps_by_edge[edge_key] = max_bumps
        return max_bumps

    def centerline_list_for_edge(
        self,
        edge_key: RoutedEdgeKey,
        record: RoutedNetRecord,
    ) -> list[tuple[float, float]] | None:
        centerline = self.centerline_lists_by_edge.get(edge_key)
        if centerline is None and record.corrected_centerline_um:
            centerline = list(record.corrected_centerline_um)
            self.centerline_lists_by_edge[edge_key] = centerline
        return centerline

    def plan_edge_candidate(
        self,
        *,
        candidate_edge_key: RoutedEdgeKey,
        requested: float,
        extra_reserved_cells: set[GridCell],
        box_depths_um: list[float],
        min_straight_um: float,
        min_seg_um: float,
        max_height_um: float,
        endpoint_inset_um: float,
        use_probe: bool,
    ) -> EdgePlanAttempt:
        t_candidate_start = time.perf_counter()
        record = self.by_edge.get(candidate_edge_key)
        attempt_info: dict[str, object] = {
            "edge": edge_key_to_dict(candidate_edge_key),
            "status": "no_candidate",
            "reason": "",
        }
        if record is None:
            attempt_info["reason"] = "no_matching_routed_record"
            attempt_info["planner_called"] = False
            elapsed = time.perf_counter() - t_candidate_start
            self.candidate_overhead_s += elapsed
            self._add_candidate_setup_time("missing_record_s", elapsed)
            return attempt_info, None, None, True, 1, set(), 0.0, 0.0, None

        blocked_by_planned: set[GridCell] | None = None
        t_centerline_start = time.perf_counter()
        centerline = self.centerline_list_for_edge(candidate_edge_key, record)
        self._add_candidate_setup_time(
            "centerline_s",
            time.perf_counter() - t_centerline_start,
        )
        registered_open_index = self.registered_open_cell_index_by_edge.get(
            candidate_edge_key,
        )
        use_registered_open = (
            centerline is not None
            and registered_open_index is not None
            and hasattr(
                self.router,
                "plan_auto_analytic_meander_for_centerline_depth_sweep_registered_opened",
            )
        )
        if use_registered_open:
            candidate_route_open_cells = set()
            opened_cells_arg: list[GridCell] = []
            attempt_info["opened_route_cell_count"] = (
                self.registered_open_cell_count_by_edge.get(candidate_edge_key, 0)
            )
        else:
            blocked_by_planned = (
                self.reserved_meander_cells_for_python_planning()
                | extra_reserved_cells
            )
            t_base_open_start = time.perf_counter()
            base_open_cells = self.base_open_cells_for_edge(candidate_edge_key, record)
            self._add_candidate_setup_time(
                "base_open_cells_s",
                time.perf_counter() - t_base_open_start,
            )
            t_open_arg_start = time.perf_counter()
            if blocked_by_planned:
                candidate_route_open_cells = base_open_cells - blocked_by_planned
                opened_cells_arg = sorted(candidate_route_open_cells)
            else:
                candidate_route_open_cells = base_open_cells
                opened_cells_arg = self.base_open_cell_list_for_edge(
                    candidate_edge_key,
                    base_open_cells,
                )
            self._add_candidate_setup_time(
                "opened_cells_arg_s",
                time.perf_counter() - t_open_arg_start,
            )
        t_max_bumps_start = time.perf_counter()
        candidate_max_bumps = self.max_bumps_for_edge(candidate_edge_key, record)
        self._add_candidate_setup_time(
            "max_bumps_s",
            time.perf_counter() - t_max_bumps_start,
        )
        attempt_info["planner_called"] = True
        candidate_best_rr: dict[str, object] | None = None
        candidate_last_exc: Exception | None = None
        candidate_used_reserved_overlay = True
        probe_elapsed_s = 0.0
        probe = None
        full_probe_available = False
        if use_probe:
            if blocked_by_planned is None:
                blocked_by_planned = (
                    self.reserved_meander_cells_for_python_planning()
                    | extra_reserved_cells
                )
            assert blocked_by_planned is not None
            t_probe_kwargs_start = time.perf_counter()
            probe_kwargs = dict(
                requested_extra_length_um=float(requested),
                box_depths_um=box_depths_um,
                min_bend_radius_um=None,
                min_straight_um=min_straight_um,
                max_bumps=candidate_max_bumps,
                max_meander_height_um=max_height_um,
                min_segment_length_um=min_seg_um,
                endpoint_inset_um=endpoint_inset_um,
                clearance_radius_cells=self.meander_box_clearance_radius_cells,
                side_policy="both",
                opened_cells=opened_cells_arg,
                planning_mode="fill_box_multi_bump",
                extra_blocked_cells=list(blocked_by_planned),
            )
            self._add_candidate_setup_time(
                "probe_kwargs_s",
                time.perf_counter() - t_probe_kwargs_start,
            )
            t_probe_start = time.perf_counter()
            try:
                if centerline is not None and hasattr(
                    self.router,
                    "probe_auto_analytic_meander_for_centerline_depth_sweep",
                ):
                    full_probe_available = True
                    probe = cast(
                        dict[str, object],
                        self.router.probe_auto_analytic_meander_for_centerline_depth_sweep(
                            centerline,
                            **probe_kwargs,
                        ),
                    )
                elif centerline is None and hasattr(
                    self.router,
                    "probe_auto_analytic_meander_for_route_depth_sweep",
                ):
                    full_probe_available = True
                    probe = cast(
                        dict[str, object],
                        self.router.probe_auto_analytic_meander_for_route_depth_sweep(
                            record.route_obj,
                            **probe_kwargs,
                        ),
                    )
            except TypeError:
                probe_kwargs.pop("extra_blocked_cells", None)
                try:
                    if centerline is not None and hasattr(
                        self.router,
                        "probe_auto_analytic_meander_for_centerline_depth_sweep",
                    ):
                        full_probe_available = True
                        probe = cast(
                            dict[str, object],
                            self.router.probe_auto_analytic_meander_for_centerline_depth_sweep(
                                centerline,
                                **probe_kwargs,
                            ),
                        )
                    elif centerline is None and hasattr(
                        self.router,
                        "probe_auto_analytic_meander_for_route_depth_sweep",
                    ):
                        full_probe_available = True
                        probe = cast(
                            dict[str, object],
                            self.router.probe_auto_analytic_meander_for_route_depth_sweep(
                                record.route_obj,
                                **probe_kwargs,
                            ),
                        )
                except Exception as exc:
                    candidate_last_exc = exc
            except Exception as exc:
                candidate_last_exc = exc
            probe_elapsed_s = time.perf_counter() - t_probe_start
            if isinstance(probe, dict):
                attempt_info["probe"] = probe
                attempt_info["probe_feasible"] = bool(probe.get("feasible", False))
                if not bool(probe.get("feasible", False)):
                    attempt_info["planner_called"] = False
                    attempt_info["reason"] = "probe found no legal meander interval"
                    self.candidate_overhead_s += (
                        time.perf_counter() - t_candidate_start - probe_elapsed_s
                    )
                    return (
                        attempt_info,
                        None,
                        record,
                        True,
                        candidate_max_bumps,
                        candidate_route_open_cells,
                        0.0,
                        probe_elapsed_s,
                        None,
                    )
            elif full_probe_available and candidate_last_exc is not None:
                attempt_info["probe_error"] = str(candidate_last_exc)
                candidate_last_exc = None
        t_plan_start = time.perf_counter()
        try:
            t_planner_kwargs_start = time.perf_counter()
            if use_registered_open:
                extra_blocked_arg = extra_reserved_cells
            else:
                if blocked_by_planned is None:
                    blocked_by_planned = (
                        self.reserved_meander_cells_for_python_planning()
                        | extra_reserved_cells
                    )
                assert blocked_by_planned is not None
                extra_blocked_arg = blocked_by_planned
            planner_kwargs = dict(
                requested_extra_length_um=float(requested),
                box_depths_um=box_depths_um,
                min_bend_radius_um=None,
                min_straight_um=min_straight_um,
                max_bumps=candidate_max_bumps,
                max_meander_height_um=max_height_um,
                min_segment_length_um=min_seg_um,
                endpoint_inset_um=endpoint_inset_um,
                clearance_radius_cells=self.meander_box_clearance_radius_cells,
                side_policy="both",
                planning_mode="fill_box_multi_bump",
                extra_blocked_cells=list(extra_blocked_arg),
            )
            if not use_registered_open:
                planner_kwargs["opened_cells"] = opened_cells_arg
            self._add_candidate_setup_time(
                "planner_kwargs_s",
                time.perf_counter() - t_planner_kwargs_start,
            )
            t_plan_start = time.perf_counter()
            try:
                if use_registered_open:
                    assert centerline is not None
                    assert registered_open_index is not None
                    candidate_best_rr = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_for_centerline_depth_sweep_registered_opened(
                            centerline,
                            registered_open_index,
                            **planner_kwargs,
                        ),
                    )
                elif centerline is not None:
                    candidate_best_rr = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_for_centerline_depth_sweep(
                            centerline,
                            **planner_kwargs,
                        ),
                    )
                else:
                    candidate_best_rr = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_for_route_depth_sweep(
                            record.route_obj,
                            **planner_kwargs,
                        ),
                    )
            except TypeError:
                candidate_used_reserved_overlay = False
                planner_kwargs.pop("extra_blocked_cells", None)
                planner_kwargs["opened_cells"] = opened_cells_arg
                if centerline is not None:
                    candidate_best_rr = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_for_centerline_depth_sweep(
                            centerline,
                            **planner_kwargs,
                        ),
                    )
                else:
                    candidate_best_rr = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_for_route_depth_sweep(
                            record.route_obj,
                            **planner_kwargs,
                        ),
                    )
        except Exception as exc:
            candidate_last_exc = exc
        elapsed_s = time.perf_counter() - t_plan_start
        self.candidate_overhead_s += (
            time.perf_counter() - t_candidate_start - elapsed_s - probe_elapsed_s
        )

        if candidate_best_rr is None:
            attempt_info["reason"] = (
                str(candidate_last_exc)
                if candidate_last_exc is not None
                else f"no exact meander candidate found (|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)"
            )
            return (
                attempt_info,
                None,
                record,
                candidate_used_reserved_overlay,
                candidate_max_bumps,
                candidate_route_open_cells,
                elapsed_s,
                probe_elapsed_s,
                candidate_last_exc,
            )

        candidate_inserted = _as_float(
            candidate_best_rr.get("inserted_extra_length_um", 0.0),
            0.0,
        )
        self.add_rust_planner_profile(candidate_best_rr.get("planner_profile"))
        self.add_rust_wrapper_profile(candidate_best_rr.get("wrapper_profile"))
        if abs(candidate_inserted - requested) > EXACT_MEANDER_EPS_UM:
            attempt_info["reason"] = (
                f"candidate residual {abs(candidate_inserted - requested):.6g} um "
                f"exceeds hard limit {EXACT_MEANDER_EPS_UM} um"
            )
            attempt_info["inserted_extra_length_um"] = candidate_inserted
            attempt_info["rejected_exact_length_mismatch"] = True
            return (
                attempt_info,
                None,
                record,
                candidate_used_reserved_overlay,
                candidate_max_bumps,
                candidate_route_open_cells,
                elapsed_s,
                probe_elapsed_s,
                None,
            )

        selected_grid_rect = candidate_best_rr.get("selected_grid_rect")
        candidate_selected_grid_rect = _parse_grid_rect(selected_grid_rect)
        if candidate_selected_grid_rect is None:
            attempt_info["reason"] = "candidate selected_grid_rect is missing or invalid"
            return (
                attempt_info,
                None,
                record,
                candidate_used_reserved_overlay,
                candidate_max_bumps,
                candidate_route_open_cells,
                elapsed_s,
                probe_elapsed_s,
                None,
            )
        blocked, overlap_count = self._candidate_grid_rect_overlaps_blocked(
            candidate_selected_grid_rect,
            extra_blocked_cells=extra_reserved_cells,
        )
        if blocked:
            attempt_info["reason"] = (
                f"candidate selected_grid_rect overlaps blocked cells ({overlap_count} cells)"
            )
            attempt_info["rejected_box_blocked"] = True
            attempt_info["blocked_overlap_count"] = overlap_count
            return (
                attempt_info,
                None,
                record,
                candidate_used_reserved_overlay,
                candidate_max_bumps,
                candidate_route_open_cells,
                elapsed_s,
                probe_elapsed_s,
                None,
            )

        attempt_info["status"] = "planned"
        attempt_info["reason"] = ""
        attempt_info["inserted_extra_length_um"] = candidate_inserted
        candidate_best_rr["rejected_box_blocked"] = False
        return (
            attempt_info,
            candidate_best_rr,
            record,
            candidate_used_reserved_overlay,
            candidate_max_bumps,
            candidate_route_open_cells,
            elapsed_s,
            probe_elapsed_s,
            None,
        )

    def plan_requirement_candidates_registered(
        self,
        *,
        work_items: list[_CandidateWorkItem],
        box_depths_um: list[float],
        min_straight_um: float,
        min_seg_um: float,
        max_height_um: float,
        endpoint_insets_um: list[float],
        auto_endpoint_inset_um: float | None,
    ) -> RequirementCandidatesAttempt | None:
        candidate_engine = self.registered_requirement_candidate_engine()
        if candidate_engine is None:
            return None
        use_registered_geometry_indices = candidate_engine.startswith(
            "rust_registered_geometry_indices",
        )
        use_auto_config = candidate_engine.endswith("_auto_config")
        edge_attempts_by_work: list[list[dict[str, object]]] = [
            [] for _ in work_items
        ]
        max_bumps_by_work: list[list[int]] = [[] for _ in work_items]
        open_counts_by_work: list[int] = [0 for _ in work_items]
        edge_calls_by_work: list[int] = [0 for _ in work_items]
        elapsed_by_work: list[float] = [0.0 for _ in work_items]
        rust_work_indices: list[int] = []
        candidate_geometry_indices: list[list[int]] = []
        candidate_centerlines: list[list[list[tuple[float, float]]]] = []
        candidate_registered_indices: list[list[int]] = []
        candidate_requested: list[float] = []
        candidate_max_bumps_by_edge: list[list[int]] = []

        for work_index, work_item in enumerate(work_items):
            if work_item.already_satisfied:
                continue
            edge_attempts = edge_attempts_by_work[work_index]
            geometry_indices: list[int] = []
            centerlines: list[list[tuple[float, float]]] = []
            registered_indices: list[int] = []
            max_bumps_values: list[int] = []
            for candidate_edge_key in work_item.candidate.edge_keys:
                record = self.by_edge.get(candidate_edge_key)
                attempt_info: dict[str, object] = {
                    "edge": edge_key_to_dict(candidate_edge_key),
                    "status": "no_candidate",
                    "reason": "",
                }
                if record is None:
                    return None
                edge_max_bumps = self.max_bumps_for_edge(
                    candidate_edge_key,
                    record,
                )
                attempt_info["planner_called"] = True
                attempt_info["max_bumps"] = edge_max_bumps
                attempt_info["opened_route_cell_count"] = (
                    self.registered_open_cell_count_by_edge.get(candidate_edge_key, 0)
                )
                edge_attempts.append(attempt_info)
                if use_registered_geometry_indices:
                    geometry_index = self.registered_geometry_index_by_edge.get(
                        candidate_edge_key,
                    )
                    if geometry_index is None:
                        return None
                    geometry_indices.append(geometry_index)
                else:
                    centerline = self.centerline_list_for_edge(candidate_edge_key, record)
                    registered_index = self.registered_open_cell_index_by_edge.get(
                        candidate_edge_key,
                    )
                    if centerline is None or registered_index is None:
                        return None
                    centerlines.append(centerline)
                    registered_indices.append(registered_index)
                max_bumps_values.append(edge_max_bumps)
                open_counts_by_work[work_index] += _as_int(
                    attempt_info.get("opened_route_cell_count"),
                    0,
                )
            max_bumps_by_work[work_index] = max_bumps_values
            rust_work_indices.append(work_index)
            if use_registered_geometry_indices:
                candidate_geometry_indices.append(geometry_indices)
            else:
                candidate_centerlines.append(centerlines)
                candidate_registered_indices.append(registered_indices)
            candidate_requested.append(work_item.requested)
            candidate_max_bumps_by_edge.append(max_bumps_values)

        if not rust_work_indices:
            return (
                None,
                [],
                edge_attempts_by_work,
                max_bumps_by_work,
                open_counts_by_work,
                edge_calls_by_work,
                elapsed_by_work,
                None,
            )

        t_plan_start = time.perf_counter()
        result: dict[str, object] | None = None
        last_exc: Exception | None = None
        try:
            if use_registered_geometry_indices:
                if use_auto_config:
                    result = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_requirement_candidate_indices_registered_opened_auto_config(
                            candidate_geometry_indices,
                            candidate_requested,
                            min_bend_radius_um=None,
                            min_straight_um=min_straight_um,
                            max_meander_height_um=max_height_um,
                            min_segment_length_um=min_seg_um,
                            auto_endpoint_inset_um=auto_endpoint_inset_um,
                            clearance_radius_cells=self.meander_box_clearance_radius_cells,
                            side_policy="both",
                            planning_mode="fill_box_multi_bump",
                        ),
                    )
                else:
                    result = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_requirement_candidate_indices_registered_opened_endpoint_sweep(
                            candidate_geometry_indices,
                            candidate_requested,
                            box_depths_um=box_depths_um,
                            endpoint_insets_um=endpoint_insets_um,
                            min_bend_radius_um=None,
                            min_straight_um=min_straight_um,
                            max_meander_height_um=max_height_um,
                            min_segment_length_um=min_seg_um,
                            clearance_radius_cells=self.meander_box_clearance_radius_cells,
                            side_policy="both",
                            planning_mode="fill_box_multi_bump",
                        ),
                    )
            else:
                if use_auto_config:
                    result = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_requirement_candidates_registered_opened_auto_config(
                            candidate_centerlines,
                            candidate_registered_indices,
                            candidate_requested,
                            candidate_max_bumps_by_edge,
                            min_bend_radius_um=None,
                            min_straight_um=min_straight_um,
                            max_meander_height_um=max_height_um,
                            min_segment_length_um=min_seg_um,
                            auto_endpoint_inset_um=auto_endpoint_inset_um,
                            clearance_radius_cells=self.meander_box_clearance_radius_cells,
                            side_policy="both",
                            planning_mode="fill_box_multi_bump",
                        ),
                    )
                else:
                    result = cast(
                        dict[str, object],
                        self.router.plan_auto_analytic_meander_requirement_candidates_registered_opened_endpoint_sweep(
                            candidate_centerlines,
                            candidate_registered_indices,
                            candidate_requested,
                            box_depths_um=box_depths_um,
                            candidate_max_bumps_by_edge=candidate_max_bumps_by_edge,
                            min_bend_radius_um=None,
                            min_straight_um=min_straight_um,
                            max_meander_height_um=max_height_um,
                            min_segment_length_um=min_seg_um,
                            endpoint_insets_um=endpoint_insets_um,
                            clearance_radius_cells=self.meander_box_clearance_radius_cells,
                            side_policy="both",
                            planning_mode="fill_box_multi_bump",
                        ),
                    )
        except Exception as exc:
            last_exc = exc
        elapsed_s = time.perf_counter() - t_plan_start
        selected_work_index: int | None = None
        selected_plans: list[PlannedEdgeInsertion] = []
        if result is None:
            if rust_work_indices:
                first_work = rust_work_indices[0]
                if edge_attempts_by_work[first_work]:
                    edge_attempts_by_work[first_work][0]["reason"] = (
                        str(last_exc)
                        if last_exc is not None
                        else f"no exact meander candidate found (|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)"
                    )
                elapsed_by_work[first_work] = elapsed_s
                edge_calls_by_work[first_work] = len(edge_attempts_by_work[first_work])
            return (
                None,
                [],
                edge_attempts_by_work,
                max_bumps_by_work,
                open_counts_by_work,
                edge_calls_by_work,
                elapsed_by_work,
                last_exc,
            )

        self.add_rust_planner_profile(result.get("planner_profile_total"))
        self.add_rust_wrapper_profile(result.get("wrapper_profile_total"))
        rust_box_depths_um = _float_list_from_mapping_value(
            result.get("box_depths_um"),
        )
        rust_endpoint_insets_um = _float_list_from_mapping_value(
            result.get("endpoint_insets_um"),
        )
        if rust_endpoint_insets_um is None:
            rust_endpoint_insets_um = _float_list_from_mapping_value(
                result.get("endpoint_insets_attempted_um"),
            )
        selected_endpoint_inset_from_rust = _as_float(
            result.get("endpoint_inset_um"),
            (rust_endpoint_insets_um or endpoint_insets_um or [0.0])[0],
        )
        raw_candidate_results = result.get("candidate_results", [])
        validated_plans_by_candidate: dict[int, list[PlannedEdgeInsertion]] = {}
        if isinstance(raw_candidate_results, list):
            for rust_index, raw_candidate_result in enumerate(raw_candidate_results):
                if rust_index >= len(rust_work_indices) or not isinstance(
                    raw_candidate_result,
                    dict,
                ):
                    continue
                work_index = rust_work_indices[rust_index]
                edge_attempts = edge_attempts_by_work[work_index]
                work_item = work_items[work_index]
                candidate_edge_keys = work_item.candidate.edge_keys
                work_item_requested = work_item.requested
                candidate_max_bumps = max_bumps_by_work[work_index]
                candidate_records = [
                    self.by_edge.get(candidate_edge_key)
                    for candidate_edge_key in candidate_edge_keys
                ]
                failed_edge_index = _as_int(
                    raw_candidate_result.get("failed_edge_index"),
                    0,
                )
                status = str(raw_candidate_result.get("status", "no_candidate"))
                if status == "planned":
                    raw_plans = raw_candidate_result.get("plans")
                    candidate_failed = False
                    candidate_plans: list[PlannedEdgeInsertion] = []
                    candidate_reserved_cells: set[GridCell] = set()
                    if (
                        not isinstance(raw_plans, list)
                        or any(record is None for record in candidate_records)
                        or len(raw_plans) != len(candidate_records)
                    ):
                        candidate_failed = True
                    else:
                        for (
                            candidate_edge_key,
                            rr_obj,
                            candidate_record,
                            candidate_max_bumps_value,
                        ) in zip(
                            candidate_edge_keys,
                            raw_plans,
                            candidate_records,
                            candidate_max_bumps,
                        ):
                            if not isinstance(rr_obj, dict) or candidate_record is None:
                                candidate_failed = True
                                break
                            rr = cast(dict[str, object], rr_obj)
                            rr["endpoint_inset_um"] = selected_endpoint_inset_from_rust
                            if rust_box_depths_um is not None:
                                rr["box_depths_um"] = rust_box_depths_um
                            if rust_endpoint_insets_um is not None:
                                rr["endpoint_insets_um"] = rust_endpoint_insets_um
                            selected_grid_rect = rr.get("selected_grid_rect")
                            parsed_grid_rect = _parse_grid_rect(selected_grid_rect)
                            if parsed_grid_rect is None:
                                candidate_failed = True
                                break
                            blocked, overlap_count = self._candidate_grid_rect_overlaps_blocked(
                                parsed_grid_rect,
                                extra_blocked_cells=candidate_reserved_cells,
                            )
                            if blocked:
                                raw_candidate_result["rejected_box_blocked"] = True
                                attempt_reason = str(
                                    raw_candidate_result.get("reason", "")
                                )
                                if not attempt_reason:
                                    attempt_reason = (
                                        f"candidate selected_grid_rect overlaps blocked cells "
                                        f"({overlap_count} cells)"
                                    )
                                raw_candidate_result["reason"] = attempt_reason
                                candidate_failed = True
                                break
                            candidate_inserted = _as_float(
                                rr.get("inserted_extra_length_um", 0.0),
                                0.0,
                            )
                            if abs(candidate_inserted - work_item_requested) > EXACT_MEANDER_EPS_UM:
                                raw_candidate_result["rejected_exact_length_mismatch"] = True
                                candidate_failed = True
                                break
                            candidate_plans.append(
                                (
                                    candidate_edge_key,
                                    candidate_record,
                                    rr,
                                    True,
                                    candidate_max_bumps_value,
                                    set(),
                                )
                            )
                            candidate_reserved_cells.update(_grid_rect_cells(parsed_grid_rect))
                    if candidate_failed:
                        for attempt_info in edge_attempts:
                            attempt_info["status"] = "no_candidate"
                            attempt_info["reason"] = str(
                                raw_candidate_result.get("reason", "")
                            )
                        edge_calls_by_work[work_index] = min(
                            len(edge_attempts),
                            failed_edge_index + 1,
                        )
                    else:
                        for attempt_info in edge_attempts:
                            attempt_info["status"] = "planned"
                            attempt_info["reason"] = ""
                            attempt_info["rejected_box_blocked"] = False
                        edge_calls_by_work[work_index] = len(edge_attempts)
                        validated_plans_by_candidate[rust_index] = candidate_plans
                        for _, _, candidate_rr, *_ in candidate_plans:
                            if isinstance(candidate_rr, dict):
                                candidate_rr["rejected_box_blocked"] = False
                else:
                    if 0 <= failed_edge_index < len(edge_attempts):
                        edge_attempts[failed_edge_index]["reason"] = str(
                            raw_candidate_result.get("reason", "")
                        )
                    edge_calls_by_work[work_index] = min(
                        len(edge_attempts),
                        failed_edge_index + 1,
                    )
        if rust_work_indices:
            elapsed_by_work[rust_work_indices[0]] = elapsed_s

        if result.get("status") == "planned":
            selected_rust_index = _as_int(result.get("selected_candidate_index"), -1)
            if selected_rust_index not in validated_plans_by_candidate:
                candidate_indices = sorted(validated_plans_by_candidate.keys())
                if candidate_indices:
                    selected_rust_index = candidate_indices[0]
            if not (0 <= selected_rust_index < len(rust_work_indices)):
                return None
            if selected_rust_index in validated_plans_by_candidate:
                selected_work_index = rust_work_indices[selected_rust_index]
                selected_plans = validated_plans_by_candidate[selected_rust_index]

        return (
            selected_work_index,
            selected_plans,
            edge_attempts_by_work,
            max_bumps_by_work,
            open_counts_by_work,
            edge_calls_by_work,
            elapsed_by_work,
            None,
        )

    def commit_planned_edge(
        self,
        *,
        selected_edge_key: RoutedEdgeKey,
        record: RoutedNetRecord,
        rr: dict[str, object],
        requested: float,
        used_reserved_overlay: bool,
        max_bumps: int,
        min_straight_um: float,
        max_height_um: float,
        min_seg_um: float,
        endpoint_inset_um: float,
    ) -> None:
        t_commit_start = time.perf_counter()
        grid_rect = rr.get("selected_grid_rect")
        parsed_grid_rect = _parse_grid_rect(grid_rect)
        if parsed_grid_rect is None:
            return
        t_blocked_check_start = time.perf_counter()
        blocked, _overlap_count = self._candidate_grid_rect_overlaps_blocked(
            parsed_grid_rect,
            extra_blocked_cells=set(),
        )
        self._add_commit_time(
            "blocked_recheck_s",
            time.perf_counter() - t_blocked_check_start,
        )
        if blocked:
            return
        min_x, max_x, min_y, max_y = parsed_grid_rect
        used_rust_reserved_rect = False
        if hasattr(self.router, "add_registered_meander_reserved_grid_rect"):
            t_rust_reserved_start = time.perf_counter()
            self.router.add_registered_meander_reserved_grid_rect(
                min_x,
                max_x,
                min_y,
                max_y,
            )
            self._add_commit_time(
                "rust_reserved_rect_update_s",
                time.perf_counter() - t_rust_reserved_start,
            )
            used_rust_reserved_rect = True
        needs_python_reserved_cells = (
            not used_rust_reserved_rect
            or not used_reserved_overlay
        )
        new_reserved_cells: set[GridCell] = set()
        if needs_python_reserved_cells:
            t_grid_rect_start = time.perf_counter()
            new_reserved_cells = _grid_rect_cells(parsed_grid_rect)
            self._add_commit_time(
                "grid_rect_cells_s",
                time.perf_counter() - t_grid_rect_start,
            )
            t_python_reserved_start = time.perf_counter()
            self.reserved_meander_cells.update(new_reserved_cells)
            self._add_commit_time(
                "python_reserved_update_s",
                time.perf_counter() - t_python_reserved_start,
            )
            if not used_rust_reserved_rect and hasattr(
                self.router,
                "add_registered_meander_reserved_cells",
            ):
                t_rust_reserved_start = time.perf_counter()
                self.router.add_registered_meander_reserved_cells(list(new_reserved_cells))
                self._add_commit_time(
                    "rust_reserved_update_s",
                    time.perf_counter() - t_rust_reserved_start,
                )
        else:
            self.pending_reserved_meander_rects.append(parsed_grid_rect)
        if not used_reserved_overlay:
            t_static_update_start = time.perf_counter()
            self.router.add_static_cells(list(new_reserved_cells))
            self._add_commit_time(
                "rust_static_update_s",
                time.perf_counter() - t_static_update_start,
            )
        t_record_start = time.perf_counter()
        self.updated[selected_edge_key] = _planned_record(
            record=record,
            requested=requested,
            rr=rr,
            min_straight_um=min_straight_um,
            max_bumps=max_bumps,
            max_height_um=max_height_um,
            min_seg_um=min_seg_um,
            endpoint_inset_um=endpoint_inset_um,
            meander_box_clearance_radius_cells=self.meander_box_clearance_radius_cells,
        )
        self._add_commit_time(
            "planned_record_s",
            time.perf_counter() - t_record_start,
        )
        self.commit_elapsed_s += time.perf_counter() - t_commit_start


def _record_edge_key(record: RoutedNetRecord) -> RoutedEdgeKey:
    return RoutedEdgeKey(
        net_name=record.net_name,
        source=record.source,
        target=record.target,
    )


def _record_route_cells(record: RoutedNetRecord) -> set[GridCell]:
    cells = getattr(record.route_obj, "cells", None) or []
    return {(int(x), int(y)) for x, y in cells}


def _record_centerline_for_registration(
    record: RoutedNetRecord,
    router: _MeanderRouterProtocol,
) -> list[tuple[float, float]] | None:
    if record.corrected_centerline_um:
        return [
            (float(x_um), float(y_um))
            for x_um, y_um in record.corrected_centerline_um
        ]
    if not hasattr(router, "route_port_corrected_centerline"):
        return None
    try:
        raw_centerline = router.route_port_corrected_centerline(record.route_obj)
    except Exception:
        return None
    if not isinstance(raw_centerline, Iterable):
        return None
    centerline: list[tuple[float, float]] = []
    for point in raw_centerline:
        if not isinstance(point, (tuple, list)) or len(point) != 2:
            return None
        try:
            centerline.append((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            return None
    return centerline if len(centerline) >= 2 else None


def _inflate_grid_cells(
    cells: Iterable[GridCell],
    *,
    radius_cells: int,
    width_cells: int,
    height_cells: int,
) -> set[GridCell]:
    radius = max(0, int(radius_cells))
    width = max(0, int(width_cells))
    height = max(0, int(height_cells))
    inflated: set[GridCell] = set()
    for x, y in cells:
        ix = int(x)
        iy = int(y)
        for dx in range(-radius, radius + 1):
            nx = ix + dx
            if nx < 0 or nx >= width:
                continue
            for dy in range(-radius, radius + 1):
                ny = iy + dy
                if 0 <= ny < height:
                    inflated.add((nx, ny))
    return inflated


def _grid_rect_cells(grid_rect: object) -> set[GridCell]:
    parsed = _parse_grid_rect(grid_rect)
    if parsed is None:
        return set()
    min_x, max_x, min_y, max_y = parsed
    return {
        (x, y)
        for x in range(min_x, max_x + 1)
        for y in range(min_y, max_y + 1)
    }


def _grid_rect_area(parsed_grid_rect: tuple[int, int, int, int]) -> int:
    min_x, max_x, min_y, max_y = parsed_grid_rect
    return max(0, max_x - min_x + 1) * max(0, max_y - min_y + 1)


def _grid_rects_overlap(
    a: tuple[int, int, int, int],
    b: tuple[int, int, int, int],
) -> bool:
    return a[0] <= b[1] and b[0] <= a[1] and a[2] <= b[3] and b[2] <= a[3]


def _grid_rect_cell_set_overlap_count(
    parsed_grid_rect: tuple[int, int, int, int],
    cells: set[GridCell],
    *,
    stop_after_first: bool = False,
) -> int:
    if not cells:
        return 0
    min_x, max_x, min_y, max_y = parsed_grid_rect
    if max_x < min_x or max_y < min_y:
        return 0
    rect_area = _grid_rect_area(parsed_grid_rect)
    count = 0
    if rect_area <= len(cells):
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                if (x, y) in cells:
                    count += 1
                    if stop_after_first:
                        return count
    else:
        for x, y in cells:
            if min_x <= x <= max_x and min_y <= y <= max_y:
                count += 1
                if stop_after_first:
                    return count
    return count


def _parse_grid_rect(grid_rect: object) -> tuple[int, int, int, int] | None:
    if not isinstance(grid_rect, (tuple, list)) or len(grid_rect) != 4:
        return None
    min_x = _as_int(grid_rect[0], 0)
    max_x = _as_int(grid_rect[1], -1)
    min_y = _as_int(grid_rect[2], 0)
    max_y = _as_int(grid_rect[3], -1)
    if max_x < min_x or max_y < min_y:
        return None
    return min_x, max_x, min_y, max_y


def _planned_record(
    *,
    record: RoutedNetRecord,
    requested: float,
    rr: dict[str, object],
    min_straight_um: float,
    max_bumps: int,
    max_height_um: float,
    min_seg_um: float,
    endpoint_inset_um: float,
    meander_box_clearance_radius_cells: int,
) -> RoutedNetRecord:
    return RoutedNetRecord(
        net_name=record.net_name,
        source=record.source,
        target=record.target,
        route_obj=record.route_obj,
        total_length_um=record.total_length_um,
        meander_auto_plan={
            "requested_extra_length_um": requested,
            "min_bend_radius_um": None,
            "min_straight_um": min_straight_um,
            "max_bumps": max_bumps,
            "max_meander_height_um": max_height_um,
            "box_depth_um": _as_float(rr.get("box_depth_um", 20.0), 20.0),
            "min_segment_length_um": min_seg_um,
            "endpoint_inset_um": endpoint_inset_um,
            "clearance_radius_cells": int(meander_box_clearance_radius_cells),
            "side_policy": "both",
            "selected_side": rr.get("side"),
            "selected_box": rr.get("selected_box"),
            "selected_grid_rect": rr.get("selected_grid_rect"),
            "selected_run_start_index": rr.get("selected_run_start_index"),
            "selected_run_end_index": rr.get("selected_run_end_index"),
            "selected_meander_centerline": rr.get("centerline"),
            "planning_mode": "fill_box_multi_bump",
        },
        opened_cells=record.opened_cells,
        source_port_center_um=record.source_port_center_um,
        target_port_center_um=record.target_port_center_um,
        source_port_orientation_deg=record.source_port_orientation_deg,
        target_port_orientation_deg=record.target_port_orientation_deg,
        base_total_length_um=record.base_total_length_um,
        corrected_centerline_um=record.corrected_centerline_um,
    )


def _build_planner_context(
    *,
    rust_backend: _RustBackendProtocol,
    routed_net_records: list[RoutedNetRecord],
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    static_blocked_cells: Iterable[tuple[int, int]] | None,
    static_blocked_cell_handle: object | None = None,
    route_occupancy_radius_cells: int | None = None,
    meander_box_clearance_radius_cells: int = 0,
    route_clearance_radius_cells: int | None = None,
) -> _MeanderPlannerContext:
    t_total_start = time.perf_counter()
    width, height, grid_size_um_cfg, origin_x_um, origin_y_um = realization_grid_spec
    t_router_start = time.perf_counter()
    grid_spec = rust_backend.GridSpec(
        int(width),
        int(height),
        float(grid_size_um_cfg),
        float(origin_x_um),
        float(origin_y_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(grid_size_um_cfg),
        bend_radius_cells=int(bend_radius_cells),
        allow_45_degree_turns=allow_45_degree_turns,
    )
    astar_cfg = rust_backend.AStarConfig(max_iterations=1)
    router = rust_backend.PyPhotonicRouter(grid_spec, primitive_cfg, astar_cfg)
    router_init_s = time.perf_counter() - t_router_start
    t_by_edge_start = time.perf_counter()
    by_edge = {_record_edge_key(r): r for r in routed_net_records}
    by_edge_s = time.perf_counter() - t_by_edge_start
    t_refcount_start = time.perf_counter()
    route_cell_refcounts: Counter[GridCell] = Counter()
    route_cells_by_edge: dict[RoutedEdgeKey, set[GridCell]] = {}
    if route_occupancy_radius_cells is None:
        route_occupancy_radius_cells = (
            0
            if route_clearance_radius_cells is None
            else int(route_clearance_radius_cells)
        )
    route_occupancy_radius_cells = max(0, int(route_occupancy_radius_cells))
    meander_box_clearance_radius_cells = max(0, int(meander_box_clearance_radius_cells))
    centerline_lists_by_edge: dict[RoutedEdgeKey, list[tuple[float, float]]] = {}
    for edge_key, record in by_edge.items():
        centerline = _record_centerline_for_registration(record, router)
        if centerline is not None:
            centerline_lists_by_edge[edge_key] = centerline
    registerable_edge_keys = list(centerline_lists_by_edge)
    registerable_edge_key_set = set(registerable_edge_keys)
    unregistered_edge_keys = [
        edge_key
        for edge_key in by_edge
        if edge_key not in registerable_edge_key_set
    ]
    can_use_registered_route_cells = bool(registerable_edge_keys) and hasattr(
        router,
        "register_meander_route_cells_as_static",
    )
    can_set_static_and_register_route_cells = bool(registerable_edge_keys) and hasattr(
        router,
        "set_static_and_register_meander_route_cells_as_static",
    )
    can_set_static_and_register_route_cells_with_handle = (
        static_blocked_cell_handle is not None
        and hasattr(
            router,
            "set_static_and_register_meander_route_cells_as_static_handle",
        )
        and bool(registerable_edge_keys)
        and not unregistered_edge_keys
    )
    can_use_registered_route_cells = (
        can_use_registered_route_cells
        or can_set_static_and_register_route_cells
        or can_set_static_and_register_route_cells_with_handle
    )
    if not can_use_registered_route_cells:
        for edge_key, record in by_edge.items():
            route_cells = _record_route_cells(record)
            if route_occupancy_radius_cells > 0:
                route_cells = _inflate_grid_cells(
                    route_cells,
                    radius_cells=route_occupancy_radius_cells,
                    width_cells=int(width),
                    height_cells=int(height),
                )
            route_cells_by_edge[edge_key] = route_cells
            route_cell_refcounts.update(route_cells)
    route_cell_refcount_s = time.perf_counter() - t_refcount_start
    t_base_static_start = time.perf_counter()
    base_static_reused = False
    base_static_from_handle = False
    if static_blocked_cells is None:
        if static_blocked_cell_handle is not None and hasattr(
            static_blocked_cell_handle,
            "cells",
        ):
            base_static_cells: set[GridCell] = {
                (int(x), int(y))
                for x, y in cast(
                    Iterable[tuple[int, int]],
                    static_blocked_cell_handle.cells(),
                )
            }
            base_static_from_handle = True
        else:
            base_static_cells = set()
    elif isinstance(static_blocked_cells, set):
        base_static_cells = cast(set[GridCell], static_blocked_cells)
        base_static_reused = True
    else:
        base_static_cells = {
            (int(x), int(y))
            for x, y in static_blocked_cells
        }
    base_static_collect_s = time.perf_counter() - t_base_static_start
    bend_radius_um = float(grid_size_um_cfg) * float(bend_radius_cells)
    t_fallback_static_start = time.perf_counter()
    fallback_route_static_cells: set[GridCell] = set()
    if can_use_registered_route_cells:
        for edge_key in unregistered_edge_keys:
            record = by_edge[edge_key]
            route_cells = _record_route_cells(record)
            if route_occupancy_radius_cells > 0:
                route_cells = _inflate_grid_cells(
                    route_cells,
                    radius_cells=route_occupancy_radius_cells,
                    width_cells=int(width),
                    height_cells=int(height),
                )
            route_cells_by_edge[edge_key] = route_cells
            fallback_route_static_cells.update(route_cells)
    fallback_route_static_collect_s = time.perf_counter() - t_fallback_static_start
    base_static_for_router = (
        base_static_cells | fallback_route_static_cells
        if fallback_route_static_cells
        else base_static_cells
    )
    set_static_s = 0.0
    if not (
        can_set_static_and_register_route_cells
        or can_set_static_and_register_route_cells_with_handle
    ):
        t_set_static_start = time.perf_counter()
        router.set_static_cells(list(base_static_for_router))
        set_static_s = time.perf_counter() - t_set_static_start
    add_route_static_s = 0.0
    register_route_cells_s = 0.0
    register_route_geometry_s = 0.0
    edge_order_s = 0.0
    route_object_list_s = 0.0
    base_static_registration_list_s = 0.0
    register_route_cells_call_s = 0.0
    registration_result_map_s = 0.0
    geometry_prepare_s = 0.0
    geometry_centerline_copy_s = 0.0
    geometry_max_bumps_s = 0.0
    geometry_call_s = 0.0
    geometry_result_map_s = 0.0
    rust_registration_profile: dict[str, float] = {}
    registered_open_cell_index_by_edge: dict[RoutedEdgeKey, int] = {}
    registered_open_cell_count_by_edge: dict[RoutedEdgeKey, int] = {}
    registered_geometry_index_by_edge: dict[RoutedEdgeKey, int] = {}
    registered_centerline_lists_by_edge: dict[RoutedEdgeKey, list[tuple[float, float]]] = {}
    registered_max_bumps_by_edge: dict[RoutedEdgeKey, int] = {}
    registered_unique_route_cell_count: int | None = None
    edge_order: list[RoutedEdgeKey] = []
    if can_use_registered_route_cells:
        t_register_start = time.perf_counter()
        t_edge_order_start = time.perf_counter()
        edge_order = list(registerable_edge_keys)
        edge_order_s = time.perf_counter() - t_edge_order_start
        t_route_objects_start = time.perf_counter()
        route_objects = [by_edge[edge_key].route_obj for edge_key in edge_order]
        route_object_list_s = time.perf_counter() - t_route_objects_start
        t_register_call_start = time.perf_counter()
        if can_set_static_and_register_route_cells_with_handle:
            indices, open_counts, unique_count = (
                router.set_static_and_register_meander_route_cells_as_static_handle(
                    route_objects,
                    static_blocked_cell_handle,
                    route_occupancy_radius_cells,
                )
            )
        else:
            t_base_static_list_start = time.perf_counter()
            base_static_registration_cells = list(base_static_for_router)
            base_static_registration_list_s = (
                time.perf_counter() - t_base_static_list_start
            )
            if can_set_static_and_register_route_cells:
                indices, open_counts, unique_count = (
                    router.set_static_and_register_meander_route_cells_as_static(
                        route_objects,
                        base_static_registration_cells,
                        route_occupancy_radius_cells,
                    )
                )
            else:
                indices, open_counts, unique_count = (
                    router.register_meander_route_cells_as_static(
                        route_objects,
                        base_static_registration_cells,
                        route_occupancy_radius_cells,
                    )
                )
        register_route_cells_call_s = time.perf_counter() - t_register_call_start
        if hasattr(router, "last_meander_registration_profile"):
            raw_profile = router.last_meander_registration_profile()
            if isinstance(raw_profile, Mapping):
                parsed_profile: dict[str, float] = {}
                for key, value in raw_profile.items():
                    if isinstance(key, str) and isinstance(value, (int, float)):
                        parsed_profile[key] = float(value)
                rust_registration_profile = parsed_profile
        register_route_cells_s = time.perf_counter() - t_register_start
        registered_unique_route_cell_count = int(unique_count)
        t_registration_map_start = time.perf_counter()
        for edge_key, raw_index, raw_count in zip(edge_order, indices, open_counts):
            registered_open_cell_index_by_edge[edge_key] = int(raw_index)
            registered_open_cell_count_by_edge[edge_key] = int(raw_count)
        registration_result_map_s = time.perf_counter() - t_registration_map_start
        for edge_key in edge_order:
            registered_centerline_lists_by_edge[edge_key] = list(
                centerline_lists_by_edge[edge_key]
            )
        if hasattr(router, "register_meander_route_geometries"):
            t_geometry_start = time.perf_counter()
            geometry_centerlines: list[list[tuple[float, float]]] = []
            geometry_open_indices: list[int] = []
            geometry_max_bumps: list[int] = []
            t_geometry_prepare_start = time.perf_counter()
            for edge_key in edge_order:
                t_centerline_copy_start = time.perf_counter()
                centerline = list(centerline_lists_by_edge[edge_key])
                geometry_centerline_copy_s += (
                    time.perf_counter() - t_centerline_copy_start
                )
                t_max_bumps_start = time.perf_counter()
                max_bumps = _route_geometry_max_meander_bumps(
                    record=by_edge[edge_key],
                    grid_size_um=float(grid_size_um_cfg),
                    bend_radius_um=bend_radius_um,
                )
                geometry_max_bumps_s += time.perf_counter() - t_max_bumps_start
                registered_max_bumps_by_edge[edge_key] = max_bumps
                geometry_centerlines.append(centerline)
                geometry_open_indices.append(registered_open_cell_index_by_edge[edge_key])
                geometry_max_bumps.append(max_bumps)
            geometry_prepare_s = time.perf_counter() - t_geometry_prepare_start
            t_geometry_call_start = time.perf_counter()
            geometry_indices = router.register_meander_route_geometries(
                geometry_centerlines,
                geometry_open_indices,
                geometry_max_bumps,
            )
            geometry_call_s = time.perf_counter() - t_geometry_call_start
            register_route_geometry_s = time.perf_counter() - t_geometry_start
            t_geometry_result_map_start = time.perf_counter()
            for edge_key, raw_index in zip(edge_order, geometry_indices):
                registered_geometry_index_by_edge[edge_key] = int(raw_index)
            geometry_result_map_s = time.perf_counter() - t_geometry_result_map_start
    elif route_cell_refcounts:
        t_add_route_static_start = time.perf_counter()
        router.add_static_cells(list(route_cell_refcounts.keys()))
        add_route_static_s = time.perf_counter() - t_add_route_static_start
    setup_profile = {
        "total_s": time.perf_counter() - t_total_start,
        "router_init_s": router_init_s,
        "by_edge_s": by_edge_s,
        "route_cell_refcount_s": route_cell_refcount_s,
        "base_static_collect_s": base_static_collect_s,
        "fallback_route_static_collect_s": fallback_route_static_collect_s,
        "base_static_reused": float(base_static_reused),
        "base_static_from_handle": float(base_static_from_handle),
        "set_static_cells_s": set_static_s,
        "add_route_static_cells_s": add_route_static_s,
        "register_route_cells_s": register_route_cells_s,
        "edge_order_s": edge_order_s,
        "route_object_list_s": route_object_list_s,
        "base_static_registration_list_s": base_static_registration_list_s,
        "register_route_cells_call_s": register_route_cells_call_s,
        "registration_result_map_s": registration_result_map_s,
        "combined_static_route_registration": float(can_set_static_and_register_route_cells),
        "combined_static_route_registration_handle": float(
            can_set_static_and_register_route_cells_with_handle
        ),
        "route_occupancy_radius_cells": float(route_occupancy_radius_cells),
        "meander_box_clearance_radius_cells": float(meander_box_clearance_radius_cells),
        "route_clearance_radius_cells": float(route_occupancy_radius_cells),
        "registered_route_cell_acceleration_enabled": float(
            can_use_registered_route_cells
        ),
        "registered_record_count": float(len(registerable_edge_keys)),
        "unregistered_record_count": float(len(unregistered_edge_keys)),
        "fallback_route_static_cell_count": float(len(fallback_route_static_cells)),
        "register_route_geometry_s": register_route_geometry_s,
        "geometry_prepare_s": geometry_prepare_s,
        "geometry_centerline_copy_s": geometry_centerline_copy_s,
        "geometry_max_bumps_s": geometry_max_bumps_s,
        "geometry_call_s": geometry_call_s,
        "geometry_result_map_s": geometry_result_map_s,
        "routed_record_count": float(len(by_edge)),
        "unique_route_cell_count": float(
            registered_unique_route_cell_count
            if registered_unique_route_cell_count is not None
            else len(route_cell_refcounts)
        ),
        "registered_open_cell_count": float(sum(registered_open_cell_count_by_edge.values())),
        "base_static_cell_count": float(len(base_static_cells)),
        "base_static_for_router_cell_count": float(len(base_static_for_router)),
    }
    for key, value in rust_registration_profile.items():
        setup_profile[f"rust_registration_{key}"] = float(value)
    return _MeanderPlannerContext(
        router=router,
        by_edge=by_edge,
        updated=dict(by_edge),
        route_cells_by_edge=route_cells_by_edge,
        route_cell_refcounts=route_cell_refcounts,
        registered_open_cell_index_by_edge=registered_open_cell_index_by_edge,
        registered_open_cell_count_by_edge=registered_open_cell_count_by_edge,
        registered_geometry_index_by_edge=registered_geometry_index_by_edge,
        base_open_cells_by_edge={},
        base_open_cell_lists_by_edge={},
        max_bumps_by_edge=registered_max_bumps_by_edge,
        centerline_lists_by_edge=registered_centerline_lists_by_edge,
        base_static_cells=base_static_cells,
        reserved_meander_cells=set(),
        pending_reserved_meander_rects=[],
        grid_width_cells=int(width),
        grid_height_cells=int(height),
        grid_size_um=float(grid_size_um_cfg),
        bend_radius_um=bend_radius_um,
        route_occupancy_radius_cells=route_occupancy_radius_cells,
        meander_box_clearance_radius_cells=meander_box_clearance_radius_cells,
        setup_profile=setup_profile,
        candidate_setup_profile={},
        commit_profile={},
        rust_planner_profile={},
        rust_wrapper_profile={},
    )


def _meander_search_config(
    *,
    config: MeanderInsertionConfig,
    bend_radius_um: float,
    rust_backend: _RustBackendProtocol | None = None,
) -> _MeanderSearchConfig:
    backend = rust_backend if rust_backend is not None else _load_rust_backend()
    if backend is not None and hasattr(backend, "auto_meander_search_config_rs"):
        raw_config = backend.auto_meander_search_config_rs(
            float(bend_radius_um),
            min_candidate_straight_length_um=float(
                config.min_candidate_straight_length_um,
            ),
            max_meander_height_um=float(config.max_meander_height_um),
            auto_endpoint_inset_um=config.auto_meander_endpoint_inset_um,
        )
        min_straight_um = _as_float(raw_config.get("min_straight_um"), 0.0)
        min_segment_um = _as_float(raw_config.get("min_segment_um"), 0.5)
        max_height_um = _as_float(
            raw_config.get("max_height_um"),
            float(config.max_meander_height_um),
        )
        box_depths_um = _float_list_from_mapping_value(
            raw_config.get("box_depths_um"),
        )
        endpoint_insets_um = _float_list_from_mapping_value(
            raw_config.get("endpoint_insets_um"),
        )
        if box_depths_um is None or endpoint_insets_um is None:
            raise RuntimeError("Rust PLM search config returned malformed candidate lists")
        endpoint_inset_um = _as_float(
            raw_config.get("endpoint_inset_um"),
            endpoint_insets_um[0] if endpoint_insets_um else 0.0,
        )
    else:
        min_straight_um = max(0.0, float(config.min_candidate_straight_length_um))
        min_segment_um = max(0.5, float(config.min_candidate_straight_length_um))
        max_height_um = max(0.0, float(config.max_meander_height_um))
        box_depths_um = [max_height_um] if max_height_um > 0.0 else [1.0]
        if config.auto_meander_endpoint_inset_um is None:
            endpoint_insets_um = [max(float(bend_radius_um), min_segment_um), 0.0]
        else:
            endpoint_insets_um = [
                max(0.0, float(config.auto_meander_endpoint_inset_um)),
            ]
        endpoint_inset_um = endpoint_insets_um[0] if endpoint_insets_um else 0.0
    return _MeanderSearchConfig(
        min_straight_um=min_straight_um,
        min_segment_um=min_segment_um,
        max_height_um=max_height_um,
        box_depths_um=box_depths_um,
        endpoint_inset_um=endpoint_inset_um,
        endpoint_insets_um=endpoint_insets_um,
    )


def _meander_search_config_to_debug_dict(
    search_config: _MeanderSearchConfig,
) -> dict[str, object]:
    return {
        "min_straight_um": float(search_config.min_straight_um),
        "min_segment_um": float(search_config.min_segment_um),
        "max_height_um": float(search_config.max_height_um),
        "box_depths_um": [float(value) for value in search_config.box_depths_um],
        "endpoint_inset_um": float(search_config.endpoint_inset_um),
        "endpoint_insets_um": [
            float(value) for value in search_config.endpoint_insets_um
        ],
        "endpoint_inset_policy": (
            "adaptive"
            if len(search_config.endpoint_insets_um) > 1
            else "fixed"
        ),
    }


def _candidates_for_requirement(
    req: MissingLengthRequirement,
    *,
    requirement_edge_alternatives: Mapping[
        RoutedEdgeKey,
        Iterable[RoutedEdgeKey],
    ]
    | None,
    requirement_delay_candidates: Mapping[
        RoutedEdgeKey,
        Iterable[DelayInsertionCandidate],
    ]
    | None,
) -> list[DelayInsertionCandidate]:
    if requirement_delay_candidates is not None:
        explicit_candidates = list(requirement_delay_candidates.get(req.edge_key, ()))
        if explicit_candidates:
            return explicit_candidates

    edge_keys = [req.edge_key]
    if requirement_edge_alternatives is not None:
        edge_keys.extend(requirement_edge_alternatives.get(req.edge_key, ()))

    candidates: list[DelayInsertionCandidate] = []
    seen_edges: set[RoutedEdgeKey] = set()
    for index, edge_key in enumerate(edge_keys):
        if edge_key in seen_edges:
            continue
        seen_edges.add(edge_key)
        candidates.append(
            DelayInsertionCandidate(
                requirement_edge_key=req.edge_key,
                edge_keys=(edge_key,),
                extra_length_um=float(req.missing_length_um),
                reason="direct_edge" if index == 0 else "legacy_single_edge_alternative",
                affected_requirement_edge_keys=(req.edge_key,),
            )
        )
    return candidates


def _requirement_missing_by_edge(
    requirements: Iterable[MissingLengthRequirement],
) -> dict[RoutedEdgeKey, float]:
    missing_by_edge: dict[RoutedEdgeKey, float] = {}
    for req in requirements:
        missing_by_edge[req.edge_key] = (
            missing_by_edge.get(req.edge_key, 0.0)
            + float(req.missing_length_um)
        )
    return missing_by_edge


def _record_candidate_profile(
    profile: dict[str, dict[str, object]],
    *,
    reason: str,
    status: str,
    edge_count: int,
    edge_calls: int,
    elapsed_s: float,
) -> None:
    entry = profile.setdefault(
        reason,
        {
            "candidate_attempts": 0,
            "edge_calls": 0,
            "elapsed_s": 0.0,
            "planned": 0,
            "no_candidate": 0,
            "already_satisfied": 0,
            "edge_count_total": 0,
        },
    )
    entry["candidate_attempts"] = _as_int(entry.get("candidate_attempts"), 0) + 1
    entry["edge_calls"] = _as_int(entry.get("edge_calls"), 0) + int(edge_calls)
    entry["elapsed_s"] = _as_float(entry.get("elapsed_s"), 0.0) + float(elapsed_s)
    entry["edge_count_total"] = _as_int(entry.get("edge_count_total"), 0) + int(edge_count)
    status_key = status if status in {"planned", "no_candidate", "already_satisfied"} else "no_candidate"
    entry[status_key] = _as_int(entry.get(status_key), 0) + 1


def _minimum_four_bend_extra_length_um(
    *,
    grid_size_um: float,
    bend_radius_cells: int,
    min_straight_um: float = DEFAULT_MEANDER_MIN_STRAIGHT_UM,
) -> float:
    """Minimum practical matching request for the analytic fill-box meander."""
    bend_radius_um = max(0.0, float(grid_size_um) * float(bend_radius_cells))
    min_straight = max(0.0, float(min_straight_um))
    return max(0.0, bend_radius_um * (2.0 * math.pi - 5.0) + min_straight)


def _normalize_minimum_insertable_request(
    requested: float,
    *,
    minimum_insertable_extra_um: float,
    tolerance_um: float = EXACT_MEANDER_EPS_UM,
) -> float:
    """Snap roundoff-sized deficits up to the minimum insertable request."""
    requested = float(requested)
    minimum = max(0.0, float(minimum_insertable_extra_um))
    tolerance = max(0.0, float(tolerance_um))
    if requested > 0.0 and requested < minimum and minimum - requested <= tolerance:
        return minimum
    return requested


def _route_geometry_max_meander_bumps(
    *,
    record: RoutedNetRecord,
    grid_size_um: float,
    bend_radius_um: float,
) -> int:
    """Derive the odd internal U-turn cap from visible lobe width.

    One visible lobe consumes four 90-degree bend radii along the selected
    straight run. Rust's comb planner reports odd internal bump counts where
    visual_bumps = (u_turns + 1) / 2. The returned value is still the legacy
    ``max_bumps`` U-turn cap accepted by the Rust binding.
    """
    radius = float(bend_radius_um)
    grid_size = float(grid_size_um)
    if (
        not math.isfinite(radius)
        or radius <= 0.0
        or not math.isfinite(grid_size)
        or grid_size <= 0.0
    ):
        return 1

    longest_straight_um = _longest_axis_aligned_centerline_run_um(
        record.corrected_centerline_um
    )
    if longest_straight_um <= 0.0:
        waypoints = getattr(record.route_obj, "compressed_waypoints", None) or []
        for p0, p1 in zip(waypoints, waypoints[1:]):
            if (
                not isinstance(p0, (tuple, list))
                or not isinstance(p1, (tuple, list))
                or len(p0) != 2
                or len(p1) != 2
            ):
                continue
            x0 = _as_int(p0[0], 0)
            y0 = _as_int(p0[1], 0)
            x1 = _as_int(p1[0], 0)
            y1 = _as_int(p1[1], 0)
            if x0 == x1:
                longest_straight_um = max(longest_straight_um, abs(y1 - y0) * grid_size)
            elif y0 == y1:
                longest_straight_um = max(longest_straight_um, abs(x1 - x0) * grid_size)

    visible_lobes = int(math.floor(longest_straight_um / (4.0 * radius)))
    return max(1, 2 * visible_lobes - 1)


def _longest_axis_aligned_centerline_run_um(
    centerline: tuple[tuple[float, float], ...],
) -> float:
    if len(centerline) < 2:
        return 0.0
    eps = 1.0e-9
    longest = 0.0
    current = 0.0
    current_axis: str | None = None
    current_line_coord = 0.0
    current_dir = 0
    for p0, p1 in zip(centerline, centerline[1:]):
        dx = float(p1[0]) - float(p0[0])
        dy = float(p1[1]) - float(p0[1])
        if abs(dy) <= eps and abs(dx) > eps:
            axis = "x"
            line_coord = float(p0[1])
            direction = 1 if dx > 0 else -1
            length = abs(dx)
        elif abs(dx) <= eps and abs(dy) > eps:
            axis = "y"
            line_coord = float(p0[0])
            direction = 1 if dy > 0 else -1
            length = abs(dy)
        else:
            longest = max(longest, current)
            current = 0.0
            current_axis = None
            continue
        if (
            axis == current_axis
            and direction == current_dir
            and abs(line_coord - current_line_coord) <= eps
        ):
            current += length
        else:
            longest = max(longest, current)
            current = length
            current_axis = axis
            current_line_coord = line_coord
            current_dir = direction
    return max(longest, current)


def analyze_meander_insertion_for_requirements(
    routed_net_records: list[RoutedNetRecord],
    requirements: list[MissingLengthRequirement],
    *,
    config: MeanderInsertionConfig,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    static_blocked_cells: Iterable[tuple[int, int]] | None = None,
    static_blocked_cell_handle: object | None = None,
    route_occupancy_radius_cells: int | None = None,
    meander_box_clearance_radius_cells: int = 0,
    route_clearance_radius_cells: int | None = None,
    requirement_edge_alternatives: Mapping[
        RoutedEdgeKey,
        Iterable[RoutedEdgeKey],
    ]
    | None = None,
    requirement_delay_candidates: Mapping[
        RoutedEdgeKey,
        Iterable[DelayInsertionCandidate],
    ]
    | None = None,
) -> tuple[list[RoutedNetRecord], dict[str, object]]:
    """Plan meander insertion using auto analytic multi-bump planning."""
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError("Rust router backend unavailable for meander analysis.")
    rust_backend = cast(_RustBackendProtocol, rust_backend)
    grid_size_um_cfg = float(realization_grid_spec[2])
    context = _build_planner_context(
        rust_backend=rust_backend,
        routed_net_records=routed_net_records,
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_cells=bend_radius_cells,
        static_blocked_cells=static_blocked_cells,
        static_blocked_cell_handle=static_blocked_cell_handle,
        route_occupancy_radius_cells=route_occupancy_radius_cells,
        meander_box_clearance_radius_cells=meander_box_clearance_radius_cells,
        route_clearance_radius_cells=route_clearance_radius_cells,
    )
    search_config = _meander_search_config(
        config=config,
        bend_radius_um=context.bend_radius_um,
        rust_backend=rust_backend,
    )
    results: list[dict[str, object]] = []
    total_requested = 0.0
    total_inserted = 0.0
    total_disregarded = 0.0
    planner_calls = 0
    planner_elapsed_s = 0.0
    probe_calls = 0
    probe_elapsed_s = 0.0
    bundle_candidate_calls = 0
    bundle_edge_calls = 0
    bundle_planned = 0
    bundle_no_candidate = 0
    requirement_batch_calls = 0
    requirement_batch_candidate_calls = 0
    requirement_batch_edge_calls = 0
    requirement_batch_planned = 0
    requirement_batch_no_candidate = 0
    fallback_candidate_calls = 0
    fallback_edge_calls = 0
    candidate_engine_counts: Counter[str] = Counter()
    candidate_profile: dict[str, dict[str, object]] = {}
    min_insertable_extra_um = _minimum_four_bend_extra_length_um(
        grid_size_um=float(grid_size_um_cfg),
        bend_radius_cells=int(bend_radius_cells),
        min_straight_um=search_config.min_straight_um,
    )
    normalized_requested_values = [
        _normalize_minimum_insertable_request(
            float(req.missing_length_um),
            minimum_insertable_extra_um=min_insertable_extra_um,
        )
        for req in requirements
    ]
    total_requested = sum(
        requested
        for requested in normalized_requested_values
        if requested >= min_insertable_extra_um
    )

    requirement_missing_by_edge = _requirement_missing_by_edge(requirements)
    effective_inserted_by_requirement_edge: dict[RoutedEdgeKey, float] = {}
    total_physical_inserted = 0.0

    for req in requirements:
        original_requested = float(req.missing_length_um)
        edge_key = req.edge_key
        already_inserted = effective_inserted_by_requirement_edge.get(edge_key, 0.0)
        requested = max(0.0, original_requested - already_inserted)
        requested = _normalize_minimum_insertable_request(
            requested,
            minimum_insertable_extra_um=min_insertable_extra_um,
        )
        if requested <= EXACT_MEANDER_EPS_UM:
            continue
        entry = {
            "edge": edge_key_to_dict(edge_key),
            "requested_extra_length_um": requested,
            "status": "no_candidate",
            "reason": "no_matching_routed_record",
            "planning_mode": "fill_box_multi_bump",
            "inserted_extra_length_um": 0.0,
            "unmatched_length_um": requested,
            "effective_bend_radius_um": None,
            "primitive_bend_radius_um": None,
            "selected_box": None,
            "selected_grid_rect": None,
            "bumps": 0,
            "side": None,
            "using_legacy_meander_path": False,
            "minimum_insertable_extra_length_um": min_insertable_extra_um,
            "planning_elapsed_s": 0.0,
        }
        if requested < min_insertable_extra_um:
            total_disregarded += requested
            entry["status"] = "below_minimum_bump"
            entry["reason"] = (
                "requested extra length is below the four-90-degree-bend "
                f"minimum ({min_insertable_extra_um:.6g} um)"
            )
            entry["unmatched_length_um"] = 0.0
            results.append(entry)
            continue
        entry["box_depth_candidates_um"] = list(search_config.box_depths_um)
        last_exc: Exception | None = None
        selected_plans: list[PlannedEdgeInsertion] = []
        selected_candidate: DelayInsertionCandidate | None = None
        selected_candidate_requested = requested
        selected_affected_edges: tuple[RoutedEdgeKey, ...] = (edge_key,)
        candidate_attempts: list[dict[str, object]] = []
        attempted_edges: list[dict[str, object]] = []
        planning_elapsed_for_entry_s = 0.0
        max_bumps = 1
        current_route_open_cell_count = 0
        entry_candidate_engine = "none"
        selected_endpoint_inset_um = search_config.endpoint_inset_um

        work_items: list[_CandidateWorkItem] = []
        for candidate in _candidates_for_requirement(
            req,
            requirement_edge_alternatives=requirement_edge_alternatives,
            requirement_delay_candidates=requirement_delay_candidates,
        ):
            affected_edges = (
                candidate.affected_requirement_edge_keys
                if candidate.affected_requirement_edge_keys
                else (edge_key,)
            )
            remaining_for_affected = [
                max(
                    0.0,
                    requirement_missing_by_edge.get(affected_edge, requested)
                    - effective_inserted_by_requirement_edge.get(affected_edge, 0.0),
                )
                for affected_edge in affected_edges
            ]
            if not remaining_for_affected:
                continue
            candidate_requested = min(
                float(candidate.extra_length_um),
                min(remaining_for_affected),
            )
            candidate_requested = _normalize_minimum_insertable_request(
                candidate_requested,
                minimum_insertable_extra_um=min_insertable_extra_um,
            )
            edge_attempts: list[dict[str, object]] = []
            candidate_info: dict[str, object] = {
                "candidate_reason": candidate.reason,
                "requested_extra_length_um": candidate_requested,
                "edges": [edge_key_to_dict(edge) for edge in candidate.edge_keys],
                "affected_requirement_edges": [
                    edge_key_to_dict(edge)
                    for edge in affected_edges
                ],
                "edge_count": len(candidate.edge_keys),
                "status": "no_candidate",
                "failure_reason": "",
                "edge_attempts": edge_attempts,
            }
            if candidate_requested <= EXACT_MEANDER_EPS_UM:
                candidate_info["status"] = "already_satisfied"
                work_items.append(
                    _CandidateWorkItem(
                        candidate=candidate,
                        affected_edges=tuple(affected_edges),
                        requested=candidate_requested,
                        info=candidate_info,
                        edge_attempts=edge_attempts,
                        already_satisfied=True,
                    )
                )
                continue
            work_items.append(
                _CandidateWorkItem(
                    candidate=candidate,
                    affected_edges=tuple(affected_edges),
                    requested=candidate_requested,
                    info=candidate_info,
                    edge_attempts=edge_attempts,
                )
            )

        attempted_endpoint_insets_um = [
            float(endpoint_inset_um)
            for endpoint_inset_um in search_config.endpoint_insets_um
        ]
        registered_candidate_engine = context.registered_requirement_candidate_engine()
        requirement_attempt = context.plan_requirement_candidates_registered(
            work_items=work_items,
            box_depths_um=search_config.box_depths_um,
            min_straight_um=search_config.min_straight_um,
            min_seg_um=search_config.min_segment_um,
            max_height_um=search_config.max_height_um,
            endpoint_insets_um=attempted_endpoint_insets_um,
            auto_endpoint_inset_um=config.auto_meander_endpoint_inset_um,
        )
        if requirement_attempt is not None:
            entry_candidate_engine = registered_candidate_engine or "rust_registered_unknown"
            candidate_engine_counts[entry_candidate_engine] += 1
            (
                selected_work_index,
                requirement_selected_plans,
                edge_attempts_by_work,
                max_bumps_by_work,
                open_counts_by_work,
                edge_calls_by_work,
                elapsed_by_work,
                requirement_last_exc,
            ) = requirement_attempt
            if requirement_last_exc is not None:
                last_exc = requirement_last_exc
            requirement_batch_was_called = any(elapsed > 0.0 for elapsed in elapsed_by_work)
            if requirement_batch_was_called:
                requirement_batch_calls += 1
            for work_index, work_item in enumerate(work_items):
                candidate = work_item.candidate
                candidate_info = work_item.info
                edge_attempts = edge_attempts_by_work[work_index]
                candidate_info["edge_attempts"] = edge_attempts
                if work_item.already_satisfied:
                    candidate_attempts.append(candidate_info)
                    _record_candidate_profile(
                        candidate_profile,
                        reason=candidate.reason,
                        status="already_satisfied",
                        edge_count=len(candidate.edge_keys),
                        edge_calls=0,
                        elapsed_s=0.0,
                    )
                    continue
                candidate_elapsed_s = elapsed_by_work[work_index]
                candidate_edge_calls = edge_calls_by_work[work_index]
                planning_elapsed_for_entry_s += candidate_elapsed_s
                planner_elapsed_s += candidate_elapsed_s
                planner_calls += candidate_edge_calls
                bundle_candidate_calls += 1
                bundle_edge_calls += candidate_edge_calls
                if requirement_batch_was_called:
                    requirement_batch_candidate_calls += 1
                    requirement_batch_edge_calls += candidate_edge_calls
                attempted_edges.extend(edge_attempts)
                if selected_work_index == work_index:
                    candidate_info["status"] = "planned"
                    candidate_info["failure_reason"] = ""
                    candidate_attempts.append(candidate_info)
                    _record_candidate_profile(
                        candidate_profile,
                        reason=candidate.reason,
                        status="planned",
                        edge_count=len(candidate.edge_keys),
                        edge_calls=candidate_edge_calls,
                        elapsed_s=candidate_elapsed_s,
                    )
                    bundle_planned += 1
                    if requirement_batch_was_called:
                        requirement_batch_planned += 1
                    selected_candidate = candidate
                    selected_candidate_requested = work_item.requested
                    selected_affected_edges = work_item.affected_edges
                    selected_plans = requirement_selected_plans
                    max_bumps = max(max_bumps_by_work[work_index], default=1)
                    current_route_open_cell_count = open_counts_by_work[work_index]
                    break
                candidate_info["status"] = "no_candidate"
                if edge_attempts:
                    candidate_info["failure_reason"] = edge_attempts[-1].get("reason", "")
                candidate_attempts.append(candidate_info)
                _record_candidate_profile(
                    candidate_profile,
                    reason=candidate.reason,
                    status="no_candidate",
                    edge_count=len(candidate.edge_keys),
                    edge_calls=candidate_edge_calls,
                    elapsed_s=candidate_elapsed_s,
                )
                bundle_no_candidate += 1
                if requirement_batch_was_called:
                    requirement_batch_no_candidate += 1
            if selected_candidate is None:
                last_exc = requirement_last_exc
        else:
            entry_candidate_engine = (
                "rust_registered_unavailable"
                if registered_candidate_engine is None
                else "rust_registered_unprepared"
            )
            candidate_engine_counts[entry_candidate_engine] += 1
            no_registered_reason = (
                "Rust registered PLM planner is required but no registered "
                "requirement candidate engine is available"
                if registered_candidate_engine is None
                else "Rust registered PLM planner could not prepare candidate inputs"
            )
            for work_item in work_items:
                candidate = work_item.candidate
                candidate_info = work_item.info
                candidate_info["status"] = "no_candidate"
                candidate_info["failure_reason"] = no_registered_reason
                edge_attempts = work_item.edge_attempts
                if not edge_attempts:
                    edge_attempts = [
                        {
                            "status": "no_candidate",
                            "reason": no_registered_reason,
                            "planner_called": False,
                        },
                    ]
                else:
                    for attempt_info in edge_attempts:
                        attempt_info["status"] = "no_candidate"
                        attempt_info["reason"] = no_registered_reason
                        attempt_info["planner_called"] = False
                candidate_info["edge_attempts"] = edge_attempts
                candidate_attempts.append(candidate_info)
                attempted_edges.extend(edge_attempts)
                _record_candidate_profile(
                    candidate_profile,
                    reason=candidate.reason,
                    status="no_candidate",
                    edge_count=len(candidate.edge_keys),
                    edge_calls=0,
                    elapsed_s=0.0,
                )
                bundle_no_candidate += 1
            last_exc = RuntimeError(no_registered_reason)

        entry["planning_elapsed_s"] = planning_elapsed_for_entry_s
        entry["candidate_edges"] = attempted_edges
        entry["candidate_attempts"] = candidate_attempts
        entry["candidate_engine"] = entry_candidate_engine
        entry["max_bumps"] = max_bumps
        entry["opened_route_cell_count"] = current_route_open_cell_count
        entry["endpoint_inset_candidates_um"] = attempted_endpoint_insets_um

        if selected_candidate is None or not selected_plans:
            entry["status"] = "no_candidate"
            attempted_edge_reason = next(
                (
                    str(attempt.get("reason", ""))
                    for attempt in attempted_edges
                    if str(attempt.get("reason", ""))
                ),
                "",
            )
            entry["reason"] = (
                str(last_exc)
                if last_exc is not None
                else attempted_edge_reason
                or f"no exact meander candidate found (|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)"
            )
            results.append(entry)
            continue
        inserted = selected_candidate_requested
        physical_inserted = sum(
            _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0)
            for _, _, rr, _, _, _ in selected_plans
        )
        unmatched = 0.0
        representative_rr = selected_plans[0][2]
        selected_endpoint_inset_um = _as_float(
            representative_rr.get("endpoint_inset_um"),
            selected_endpoint_inset_um,
        )
        rust_box_depths_um = _float_list_from_mapping_value(
            representative_rr.get("box_depths_um"),
        )
        if rust_box_depths_um is not None:
            entry["box_depth_candidates_um"] = rust_box_depths_um
        rust_endpoint_insets_um = _float_list_from_mapping_value(
            representative_rr.get("endpoint_insets_um"),
        )
        if rust_endpoint_insets_um is not None:
            attempted_endpoint_insets_um = rust_endpoint_insets_um
            entry["endpoint_inset_candidates_um"] = attempted_endpoint_insets_um
        entry["status"] = "planned"
        entry["reason"] = ""
        entry["inserted_extra_length_um"] = inserted
        entry["physical_inserted_extra_length_um"] = physical_inserted
        entry["unmatched_length_um"] = unmatched
        entry["effective_bend_radius_um"] = representative_rr.get("effective_bend_radius_um")
        entry["primitive_bend_radius_um"] = representative_rr.get("primitive_bend_radius_um")
        entry["selected_box"] = representative_rr.get("selected_box")
        entry["selected_grid_rect"] = representative_rr.get("selected_grid_rect")
        entry["bumps"] = representative_rr.get("bumps")
        entry["visual_bumps"] = representative_rr.get("visual_bumps")
        entry["u_turns"] = representative_rr.get("u_turns")
        entry["quarter_turns"] = representative_rr.get("quarter_turns")
        entry["side"] = representative_rr.get("side")
        entry["planning_mode"] = representative_rr.get("planning_mode", "fill_box_multi_bump")
        entry["candidate_runs"] = representative_rr.get("candidate_runs")
        entry["candidate_intervals"] = representative_rr.get("candidate_intervals")
        entry["rejected_box_blocked"] = representative_rr.get("rejected_box_blocked")
        entry["rejected_planning_failed"] = representative_rr.get("rejected_planning_failed")
        entry["rejected_exact_length_mismatch"] = representative_rr.get("rejected_exact_length_mismatch")
        entry["rejected_too_short"] = representative_rr.get("rejected_too_short")
        entry["selected_interval_length_um"] = representative_rr.get("selected_interval_length_um")
        entry["endpoint_inset_um"] = selected_endpoint_inset_um
        entry["requested_probe_length_um"] = requested
        entry["used_reserved_overlay"] = all(plan[3] for plan in selected_plans)
        entry["selected_candidate_reason"] = selected_candidate.reason
        entry["selected_candidate_edge_count"] = len(selected_candidate.edge_keys)
        entry["affected_requirement_edges"] = [
            edge_key_to_dict(affected_edge)
            for affected_edge in selected_affected_edges
        ]
        entry["planned_edge"] = edge_key_to_dict(selected_plans[0][0])
        entry["planned_edges"] = [
            edge_key_to_dict(candidate_edge_key)
                for candidate_edge_key, *_ in selected_plans
        ]
        if unmatched > 1.0e-9:
            entry["status"] = "planned_partial"
        for affected_edge in selected_affected_edges:
            effective_inserted_by_requirement_edge[affected_edge] = (
                effective_inserted_by_requirement_edge.get(affected_edge, 0.0)
                + inserted
            )
        total_inserted += inserted * len(selected_affected_edges)
        total_physical_inserted += physical_inserted
        for (
            selected_edge_key,
            record,
            rr,
            used_reserved_overlay,
            candidate_max_bumps,
            _candidate_route_open_cells,
        ) in selected_plans:
            context.commit_planned_edge(
                selected_edge_key=selected_edge_key,
                record=record,
                rr=rr,
                requested=float(selected_candidate_requested),
                used_reserved_overlay=used_reserved_overlay,
                min_straight_um=search_config.min_straight_um,
                max_bumps=candidate_max_bumps,
                max_height_um=search_config.max_height_um,
                min_seg_um=search_config.min_segment_um,
                endpoint_inset_um=selected_endpoint_inset_um,
            )
        results.append(entry)
        for affected_edge in selected_affected_edges:
            if affected_edge == edge_key:
                continue
            virtual_entry = {
                **entry,
                "edge": edge_key_to_dict(affected_edge),
                "satisfied_by_requirement_edge": edge_key_to_dict(edge_key),
            }
            results.append(virtual_entry)

    total_unmatched = max(0.0, total_requested - total_inserted)
    return (
        [context.updated.get(_record_edge_key(r), r) for r in routed_net_records],
        {
            "results": results,
            "total_requested_extra_length_um": float(total_requested),
            "total_inserted_extra_length_um": float(total_inserted),
            "total_physical_inserted_extra_length_um": float(total_physical_inserted),
            "total_disregarded_extra_length_um": float(total_disregarded),
            "unmatched_length_um": float(total_unmatched),
            "planner_calls": int(planner_calls),
            "planner_elapsed_s": float(planner_elapsed_s),
            "probe_calls": int(probe_calls),
            "probe_elapsed_s": float(probe_elapsed_s),
            "bundle_candidate_calls": int(bundle_candidate_calls),
            "bundle_edge_calls": int(bundle_edge_calls),
            "bundle_planned": int(bundle_planned),
            "bundle_no_candidate": int(bundle_no_candidate),
            "requirement_batch_calls": int(requirement_batch_calls),
            "requirement_batch_candidate_calls": int(requirement_batch_candidate_calls),
            "requirement_batch_edge_calls": int(requirement_batch_edge_calls),
            "requirement_batch_planned": int(requirement_batch_planned),
            "requirement_batch_no_candidate": int(requirement_batch_no_candidate),
            "fallback_candidate_calls": int(fallback_candidate_calls),
            "fallback_edge_calls": int(fallback_edge_calls),
            "candidate_engine_counts": dict(candidate_engine_counts),
            "setup_profile": context.setup_profile,
            "candidate_setup_profile": context.candidate_setup_profile,
            "candidate_overhead_s": float(context.candidate_overhead_s),
            "commit_profile": context.commit_profile,
            "commit_elapsed_s": float(context.commit_elapsed_s),
            "rust_planner_profile": context.rust_planner_profile,
            "rust_wrapper_profile": context.rust_wrapper_profile,
            "candidate_profile": candidate_profile,
            "search_config": _meander_search_config_to_debug_dict(search_config),
            "minimum_insertable_extra_length_um": float(min_insertable_extra_um),
            "using_legacy_meander_path": False,
        },
    )


def insert_meanders_for_requirements(
    routed_net_records: list[RoutedNetRecord],
    requirements: list[MissingLengthRequirement],
    *,
    config: MeanderInsertionConfig,
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
) -> tuple[list[RoutedNetRecord], MeanderInsertionReport]:
    """Compatibility API used by tests for M2 skeleton behavior."""
    if not config.enabled:
        return (
            routed_net_records,
            MeanderInsertionReport(
                results=[],
                total_requested_extra_length_um=0.0,
                total_inserted_extra_length_um=0.0,
                unmatched_length_um=0.0,
            ),
        )

    updated, raw_report = analyze_meander_insertion_for_requirements(
        routed_net_records,
        requirements,
        config=config,
        realization_grid_spec=realization_grid_spec,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_cells=bend_radius_cells,
    )
    results: list[MeanderInsertionResult] = []
    raw_results = cast(list[dict[str, object]], raw_report.get("results", []))
    for item in raw_results:
        edge_info = item.get("edge", {})
        if not isinstance(edge_info, dict):
            edge_info = {}
        source = edge_info.get("source", {})
        if not isinstance(source, dict):
            source = {}
        target = edge_info.get("target", {})
        if not isinstance(target, dict):
            target = {}
        edge = RoutedEdgeKey(
            net_name=str(edge_info.get("net_name", "")),
            source=PortRef(
                instance=str(source.get("instance", "")),
                port=str(source.get("port", "")),
            ),
            target=PortRef(
                instance=str(target.get("instance", "")),
                port=str(target.get("port", "")),
            ),
        )
        status = str(item.get("status", "unknown"))
        reason = str(item.get("reason", ""))
        results.append(
            MeanderInsertionResult(
                edge=edge,
                requested_extra_length_um=_as_float(
                    item.get("requested_extra_length_um", 0.0),
                    0.0,
                ),
                inserted_extra_length_um=_as_float(
                    item.get("inserted_extra_length_um", 0.0),
                    0.0,
                ),
                status=status,
                reason=reason,
            )
        )
    report = MeanderInsertionReport(
        results=results,
        total_requested_extra_length_um=float(
            cast(float, raw_report.get("total_requested_extra_length_um", 0.0))
        ),
        total_inserted_extra_length_um=float(
            cast(float, raw_report.get("total_inserted_extra_length_um", 0.0))
        ),
        unmatched_length_um=float(
            cast(float, raw_report.get("unmatched_length_um", 0.0))
        ),
    )
    return updated, report
