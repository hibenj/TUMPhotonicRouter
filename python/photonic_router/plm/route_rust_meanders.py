"""Meander planning for path-length matching: the Python side of the PLM stage.

Who decides what (phase 6 slice B): Python turns the timing requirements into
request vectors, Rust decides every plan. For each requirement the selection
pass sends one call to the registered Rust planner with the candidate bundles
(shared upstream bundle, recursive upstream, direct edge, in that order; one
requested extra length per edge, the edge's new total where it already carries
a planned meander) and the same-edge split fallback; Rust returns the first
bundle that plans as one sequence, or the split, or nothing. The final pass
sends every physical edge with its total as one vector; Rust tries the whole
sequence, then each edge direct, then split, and reports the plans that are
committed to the records. The timers, counters and per-candidate attempts are
built only under ``--debug-meanders`` (``MeanderPlanningProfile``); the report
the flow and the acceptance diagnostics read is ``MeanderInsertionReport``.
"""

from __future__ import annotations

import importlib
import itertools
import math
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol, cast

from photonic_router.plm.path_length_graph import (
    DelayInsertionCandidate,
    MissingLengthRequirement,
    RoutedEdgeKey,
)
from photonic_router.records.route_rust_types import (
    DEFAULT_MEANDER_MIN_STRAIGHT_UM,
    MeanderEdgeReport,
    MeanderInsertionConfig,
    MeanderInsertionReport,
    MeanderPlanningProfile,
    RoutedNetRecord,
    _as_float,
    _as_int,
    edge_key_to_dict,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
_load_rust_backend = _sob._load_rust_backend

EXACT_MEANDER_EPS_UM = 1.0e-6
#: the most same-edge chunks the split fallback tries (selection and final pass)
MAX_SPLIT_PARTS = 8
GridCell = tuple[int, int]


class MeanderRustBackendLike(Protocol):
    """Structural type for the loaded Rust backend used by meander planning."""

    def GridSpec(self, *args: object, **kwargs: object) -> object: ...
    def PrimitiveLibraryConfig(self, *args: object, **kwargs: object) -> object: ...
    def AStarConfig(self, *args: object, **kwargs: object) -> object: ...
    def auto_meander_search_config_rs(
        self,
        *args: object,
        **kwargs: object,
    ) -> dict[str, object]: ...
    def PyPhotonicRouter(self, *args: object, **kwargs: object) -> MeanderRouterLike: ...
    def MeanderPlanner(self, router: MeanderRouterLike) -> MeanderPlannerLike: ...


class MeanderRouterLike(Protocol):
    """Structural type for the PyO3 router the planner context is built on."""

    def set_static_cells(self, cells: list[GridCell]) -> None: ...
    def route_port_corrected_centerline(
        self,
        route: object,
        **kwargs: Any,
    ) -> list[tuple[float, float]]: ...


class MeanderPlannerLike(Protocol):
    """Structural type for the PyO3 `MeanderPlanner` (`src/bindings/meander_py.rs`),
    the registration state and planning calls of one router.

    The two planning methods are the request-vector form of the selection
    pass and the final pass (`plan_registered_geometry_requirement_requests`
    and `plan_registered_geometry_final_requests` in `src/meander/plan.rs`).
    """

    def register(
        self,
        routes: list[object],
        base_static_cells: list[GridCell] | None = None,
        base_static_cell_handle: object | None = None,
        route_clearance_radius_cells: int = 0,
        reset: bool = True,
    ) -> tuple[list[int], list[int], int]: ...
    def register_geometries(
        self,
        centerlines: list[list[tuple[float, float]]],
        registered_opened_cell_indices: list[int],
        max_bumps_by_edge: list[int],
    ) -> list[int]: ...
    def reserve_rect(self, min_x: int, max_x: int, min_y: int, max_y: int) -> int: ...
    def profile(self) -> dict[str, int | float]: ...
    def plan_requests(
        self,
        candidate_geometry_indices: list[list[int]],
        candidate_requested_extra_lengths_um: list[list[float]],
        **kwargs: Any,
    ) -> dict[str, object]: ...
    def plan_final(
        self,
        geometry_indices: list[int],
        requested_extra_lengths_um: list[float],
        min_insertable_extra_length_um: float,
        **kwargs: Any,
    ) -> dict[str, object]: ...


#: (edge, record, the Rust plan dict, max_bumps)
PlannedEdgeInsertion = tuple[RoutedEdgeKey, RoutedNetRecord, dict[str, object], int]


@dataclass
class _CandidateWorkItem:
    """One delay-insertion candidate of a requirement, as a planner request."""

    candidate: DelayInsertionCandidate
    affected_edges: tuple[RoutedEdgeKey, ...]
    #: requested extra length per candidate edge: the credit plus the extra
    #: the edge already carries from earlier requirements (its new total)
    requests_by_edge: dict[RoutedEdgeKey, float]
    #: what this requirement is credited when the candidate plans
    credit_extra_length: float
    existing_physical_extra_length: float
    already_satisfied: bool = False


@dataclass
class _RequirementPlanOutcome:
    """What Rust decided for one requirement's request vector."""

    #: the index into the work items Rust planned, or None
    selected_work_index: int | None
    #: the planned same-edge split, when the candidates failed and it planned
    split_part_count: int | None
    plans: list[PlannedEdgeInsertion]
    reason: str
    elapsed_s: float
    #: the plans Rust evaluated: one per edge of every tried candidate up to
    #: the failing edge (the old ``planner_calls`` count)
    edge_calls: int
    candidate_statuses: list[dict[str, object]] = field(default_factory=list)


@dataclass
class _SelectedMeanderRequirement:
    entry: MeanderEdgeReport
    virtual_entries: list[MeanderEdgeReport]
    edge_key: RoutedEdgeKey
    affected_edges: tuple[RoutedEdgeKey, ...]
    physical_edges: tuple[RoutedEdgeKey, ...]
    credit_extra_length: float


@dataclass
class _FinalMeanderPlanningResult:
    updated_records: list[RoutedNetRecord]
    total_inserted: float
    total_physical_inserted: float
    #: Realized extra length per physical edge (the edge the meander was
    #: drawn on, which is the requiring edge or a delay-insertion candidate
    #: upstream of it): what the bookkeeping must report per net.
    realized_extra_by_edge: dict[RoutedEdgeKey, float]
    planning_mode: str
    planner_calls: int
    planner_elapsed_s: float
    final_context: _MeanderPlannerContext


@dataclass(frozen=True)
class _MeanderSearchConfig:
    min_straight_um: float
    min_segment_um: float
    max_height_um: float
    box_depths_um: list[float]
    endpoint_inset_um: float
    endpoint_insets_um: list[float]


@dataclass
class _ContextProfile:
    """The planner context's timers, kept only under ``--debug-meanders``."""

    setup_profile: dict[str, float] = field(default_factory=dict)
    commit_profile: dict[str, float] = field(default_factory=dict)
    rust_planner_profile: dict[str, float] = field(default_factory=dict)
    rust_wrapper_profile: dict[str, float] = field(default_factory=dict)
    commit_elapsed_s: float = 0.0

    def add_commit_time(self, key: str, elapsed_s: float) -> None:
        self.commit_profile[key] = self.commit_profile.get(key, 0.0) + max(0.0, float(elapsed_s))

    def add_rust_profiles(self, result: Mapping[str, object]) -> None:
        _add_numeric_profile(self.rust_planner_profile, result.get("planner_profile_total"))
        _add_numeric_profile(self.rust_wrapper_profile, result.get("wrapper_profile_total"))


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


def _merged_numeric_profiles(
    *profiles: Mapping[str, float],
) -> dict[str, float]:
    merged: dict[str, float] = {}
    for profile in profiles:
        for key, value in profile.items():
            merged[key] = merged.get(key, 0.0) + float(value)
    return merged


def _planner_kwargs(
    *,
    search_config: _MeanderSearchConfig,
    auto_endpoint_inset_um: float | None,
    clearance_radius_cells: int,
) -> dict[str, object]:
    """The planner options every registered Rust call takes."""
    return {
        "min_bend_radius_um": None,
        "min_straight_um": search_config.min_straight_um,
        "max_meander_height_um": search_config.max_height_um,
        "min_segment_length_um": search_config.min_segment_um,
        "auto_endpoint_inset_um": auto_endpoint_inset_um,
        "clearance_radius_cells": clearance_radius_cells,
        "side_policy": "both",
        "planning_mode": "fill_box_multi_bump",
    }


def _first_failure_reason(result: Mapping[str, object], default: str) -> str:
    """The failing edge's reason of the first candidate Rust rejected."""
    raw_candidate_results = result.get("candidate_results", [])
    if isinstance(raw_candidate_results, list):
        for raw_candidate in raw_candidate_results:
            if isinstance(raw_candidate, Mapping) and str(raw_candidate.get("reason", "")):
                return str(raw_candidate["reason"])
    return str(result.get("reason", "")) or default


def _candidate_edge_calls(result: Mapping[str, object]) -> int:
    """The plans Rust evaluated: every planned edge plus the failing edge per candidate."""
    raw_candidate_results = result.get("candidate_results", [])
    if not isinstance(raw_candidate_results, list):
        return 0
    calls = 0
    for raw_candidate in raw_candidate_results:
        if not isinstance(raw_candidate, Mapping):
            continue
        plans = raw_candidate.get("plans", [])
        calls += len(plans) if isinstance(plans, list) else 0
        if raw_candidate.get("status") != "planned":
            calls += 1
    return calls


@dataclass
class _MeanderPlannerContext:
    """One Rust `MeanderPlanner` with the routes and their geometries registered."""

    planner: MeanderPlannerLike
    by_edge: dict[RoutedEdgeKey, RoutedNetRecord]
    updated: dict[RoutedEdgeKey, RoutedNetRecord]
    registered_geometry_index_by_edge: dict[RoutedEdgeKey, int]
    max_bumps_by_edge: dict[RoutedEdgeKey, int]
    base_static_cells: set[GridCell]
    grid_size_um: float
    bend_radius_um: float
    profile: _ContextProfile | None = None
    grid_width_cells: int = 0
    grid_height_cells: int = 0
    route_occupancy_radius_cells: int = 0
    meander_box_clearance_radius_cells: int = 0

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

    def plan_requirement_requests_registered(
        self,
        *,
        work_items: list[_CandidateWorkItem],
        split_edge_key: RoutedEdgeKey,
        split_requested: float,
        min_insertable_extra_um: float,
        search_config: _MeanderSearchConfig,
        auto_endpoint_inset_um: float | None,
    ) -> _RequirementPlanOutcome:
        """One Rust call decides the requirement: a candidate, the split, or nothing.

        The vector carries the work items that still need length (one
        request per edge); the split fallback is the requiring edge itself.
        A candidate whose edge is not registered (no routed record, no
        geometry) takes the candidates out of the call, as the batch did
        before; the split is still tried.
        """
        candidate_work_indices: list[int] = []
        candidate_geometry_indices: list[list[int]] = []
        candidate_requests: list[list[float]] = []
        prepared = True
        for work_index, work_item in enumerate(work_items):
            if work_item.already_satisfied:
                continue
            geometry_indices: list[int] = []
            requests: list[float] = []
            for candidate_edge_key in work_item.candidate.edge_keys:
                record = self.by_edge.get(candidate_edge_key)
                geometry_index = self.registered_geometry_index_by_edge.get(candidate_edge_key)
                if record is None or geometry_index is None:
                    prepared = False
                    break
                self.max_bumps_for_edge(candidate_edge_key, record)
                geometry_indices.append(geometry_index)
                requests.append(float(work_item.requests_by_edge.get(candidate_edge_key, 0.0)))
            if not prepared:
                break
            candidate_work_indices.append(work_index)
            candidate_geometry_indices.append(geometry_indices)
            candidate_requests.append(requests)
        if not prepared:
            candidate_work_indices = []
            candidate_geometry_indices = []
            candidate_requests = []

        split_geometry_index = (
            self.registered_geometry_index_by_edge.get(split_edge_key)
            if split_edge_key in self.by_edge and split_requested > 0.0
            else None
        )
        if not candidate_geometry_indices and split_geometry_index is None:
            return _RequirementPlanOutcome(
                selected_work_index=None,
                split_part_count=None,
                plans=[],
                reason=(
                    "no exact meander candidate found "
                    f"(|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)"
                    if prepared
                    else "Rust registered PLM planner could not prepare candidate inputs"
                ),
                elapsed_s=0.0,
                edge_calls=0,
            )

        t_plan_start = time.perf_counter()
        try:
            result = cast(
                dict[str, object],
                self.planner.plan_requests(
                    candidate_geometry_indices,
                    candidate_requests,
                    split_geometry_index=split_geometry_index,
                    split_requested_extra_length_um=(
                        None if split_geometry_index is None else float(split_requested)
                    ),
                    min_insertable_extra_length_um=float(min_insertable_extra_um),
                    max_split_parts=MAX_SPLIT_PARTS,
                    **_planner_kwargs(
                        search_config=search_config,
                        auto_endpoint_inset_um=auto_endpoint_inset_um,
                        clearance_radius_cells=self.meander_box_clearance_radius_cells,
                    ),
                ),
            )
        except Exception as exc:
            return _RequirementPlanOutcome(
                selected_work_index=None,
                split_part_count=None,
                plans=[],
                reason=str(exc),
                elapsed_s=time.perf_counter() - t_plan_start,
                edge_calls=len(candidate_geometry_indices),
            )
        elapsed_s = time.perf_counter() - t_plan_start
        if self.profile is not None:
            self.profile.add_rust_profiles(result)

        candidate_statuses: list[dict[str, object]] = []
        if self.profile is not None:
            raw_candidate_results = result.get("candidate_results", [])
            if isinstance(raw_candidate_results, list):
                for raw_candidate in raw_candidate_results:
                    if not isinstance(raw_candidate, Mapping):
                        continue
                    rust_index = _as_int(raw_candidate.get("candidate_index"), -1)
                    candidate_statuses.append(
                        {
                            "candidate_reason": (
                                work_items[candidate_work_indices[rust_index]].candidate.reason
                                if 0 <= rust_index < len(candidate_work_indices)
                                else "split_route_runs"
                            ),
                            "status": str(raw_candidate.get("status", "")),
                            "reason": str(raw_candidate.get("reason", "")),
                            "failed_edge_index": raw_candidate.get("failed_edge_index"),
                        }
                    )

        edge_calls = _candidate_edge_calls(result)
        if result.get("status") != "planned":
            return _RequirementPlanOutcome(
                selected_work_index=None,
                split_part_count=None,
                plans=[],
                reason=_first_failure_reason(
                    result,
                    "no exact meander candidate found "
                    f"(|inserted-requested| <= {EXACT_MEANDER_EPS_UM} um)",
                ),
                elapsed_s=elapsed_s,
                edge_calls=edge_calls,
                candidate_statuses=candidate_statuses,
            )

        raw_plans = cast(list[dict[str, object]], result.get("plans", []))
        planning_mode = str(result.get("planning_mode", ""))
        if planning_mode == "rust_registered_split_route_runs":
            record = self.by_edge[split_edge_key]
            max_bumps = self.max_bumps_for_edge(split_edge_key, record)
            return _RequirementPlanOutcome(
                selected_work_index=None,
                split_part_count=_as_int(result.get("split_part_count"), len(raw_plans)),
                plans=[(split_edge_key, record, rr, max_bumps) for rr in raw_plans],
                reason="",
                elapsed_s=elapsed_s,
                edge_calls=edge_calls,
                candidate_statuses=candidate_statuses,
            )
        selected_rust_index = _as_int(result.get("selected_candidate_index"), -1)
        if not (0 <= selected_rust_index < len(candidate_work_indices)):
            return _RequirementPlanOutcome(
                selected_work_index=None,
                split_part_count=None,
                plans=[],
                reason="Rust registered PLM planner selected an unknown candidate",
                elapsed_s=elapsed_s,
                edge_calls=edge_calls,
                candidate_statuses=candidate_statuses,
            )
        selected_work_index = candidate_work_indices[selected_rust_index]
        candidate_edge_keys = work_items[selected_work_index].candidate.edge_keys
        if len(raw_plans) != len(candidate_edge_keys):
            return _RequirementPlanOutcome(
                selected_work_index=None,
                split_part_count=None,
                plans=[],
                reason="Rust registered PLM planner returned wrong plan count",
                elapsed_s=elapsed_s,
                edge_calls=edge_calls,
                candidate_statuses=candidate_statuses,
            )
        plans: list[PlannedEdgeInsertion] = []
        for candidate_edge_key, rr in zip(candidate_edge_keys, raw_plans, strict=True):
            record = self.by_edge[candidate_edge_key]
            plans.append(
                (
                    candidate_edge_key,
                    record,
                    rr,
                    self.max_bumps_for_edge(candidate_edge_key, record),
                )
            )
        return _RequirementPlanOutcome(
            selected_work_index=selected_work_index,
            split_part_count=None,
            plans=plans,
            reason="",
            elapsed_s=elapsed_s,
            edge_calls=edge_calls,
            candidate_statuses=candidate_statuses,
        )

    def plan_final_requests_registered(
        self,
        *,
        edge_keys: list[RoutedEdgeKey],
        planner_requests_by_edge: Mapping[RoutedEdgeKey, float],
        min_insertable_extra_um: float,
        search_config: _MeanderSearchConfig,
        auto_endpoint_inset_um: float | None,
    ) -> tuple[list[PlannedEdgeInsertion], str, float, int, str] | None:
        """Plan every physical edge's total in one Rust call.

        Rust tries all edges as one sequence, then each edge direct, then
        split, and stops at the first edge nothing plans. Returns the plans,
        the failure reason (empty when planned), the elapsed time, the edge
        calls and Rust's planning mode; None when an edge is not registered.
        """
        if not edge_keys:
            return ([], "", 0.0, 0, "none")

        geometry_indices: list[int] = []
        requested_lengths: list[float] = []
        for edge_key in edge_keys:
            record = self.by_edge.get(edge_key)
            geometry_index = self.registered_geometry_index_by_edge.get(edge_key)
            if record is None or geometry_index is None:
                return None
            self.max_bumps_for_edge(edge_key, record)
            geometry_indices.append(int(geometry_index))
            requested_lengths.append(float(planner_requests_by_edge.get(edge_key, 0.0)))

        t_plan_start = time.perf_counter()
        try:
            result = cast(
                dict[str, object],
                self.planner.plan_final(
                    geometry_indices,
                    requested_lengths,
                    min_insertable_extra_um,
                    max_split_parts=MAX_SPLIT_PARTS,
                    **_planner_kwargs(
                        search_config=search_config,
                        auto_endpoint_inset_um=auto_endpoint_inset_um,
                        clearance_radius_cells=self.meander_box_clearance_radius_cells,
                    ),
                ),
            )
        except Exception as exc:
            return (
                [],
                str(exc),
                time.perf_counter() - t_plan_start,
                1,
                "rust_registered_final_requests",
            )
        elapsed_s = time.perf_counter() - t_plan_start
        if self.profile is not None:
            self.profile.add_rust_profiles(result)
        planning_mode = str(result.get("planning_mode", "rust_registered_final_requests"))
        edge_calls = _candidate_edge_calls(result)
        if result.get("status") != "planned":
            return (
                [],
                _first_failure_reason(result, "no exact final aggregate meander candidate found"),
                elapsed_s,
                edge_calls,
                planning_mode,
            )
        raw_plans = cast(list[dict[str, object]], result.get("plans", []))
        plan_input_indices = [
            _as_int(index, -1) for index in cast(list[object], result.get("plan_input_indices", []))
        ]
        if len(raw_plans) != len(plan_input_indices) or any(
            not (0 <= index < len(edge_keys)) for index in plan_input_indices
        ):
            return (
                [],
                "final aggregate planner returned mismatched plan indices",
                elapsed_s,
                edge_calls,
                planning_mode,
            )
        plans: list[PlannedEdgeInsertion] = []
        for raw_plan, input_index in zip(raw_plans, plan_input_indices, strict=True):
            edge_key = edge_keys[input_index]
            record = self.by_edge[edge_key]
            plans.append((edge_key, record, raw_plan, self.max_bumps_for_edge(edge_key, record)))
        return (plans, "", elapsed_s, max(len(edge_keys), len(plans)), planning_mode)

    def commit_planned_edge(
        self,
        *,
        selected_edge_key: RoutedEdgeKey,
        record: RoutedNetRecord,
        rr: dict[str, object],
        requested: float,
        max_bumps: int,
        search_config: _MeanderSearchConfig,
        endpoint_inset_um: float,
    ) -> None:
        """Reserve the plan's box in the router and record the planned meander."""
        t_commit_start = time.perf_counter()
        parsed_grid_rect = _parse_grid_rect(rr.get("selected_grid_rect"))
        if parsed_grid_rect is None:
            return
        t_blocked_check_start = time.perf_counter()
        blocked = (
            _grid_rect_cell_set_overlap_count(
                parsed_grid_rect,
                self.base_static_cells,
                stop_after_first=True,
            )
            > 0
        )
        if self.profile is not None:
            self.profile.add_commit_time(
                "blocked_recheck_s", time.perf_counter() - t_blocked_check_start
            )
        if blocked:
            return
        min_x, max_x, min_y, max_y = parsed_grid_rect
        t_rust_reserved_start = time.perf_counter()
        self.planner.reserve_rect(min_x, max_x, min_y, max_y)
        if self.profile is not None:
            self.profile.add_commit_time(
                "rust_reserved_rect_update_s", time.perf_counter() - t_rust_reserved_start
            )
        t_record_start = time.perf_counter()
        self.updated[selected_edge_key] = _planned_record(
            record=record,
            requested=requested,
            rr=rr,
            min_straight_um=search_config.min_straight_um,
            max_bumps=max_bumps,
            max_height_um=search_config.max_height_um,
            min_seg_um=search_config.min_segment_um,
            endpoint_inset_um=endpoint_inset_um,
            meander_box_clearance_radius_cells=self.meander_box_clearance_radius_cells,
        )
        if self.profile is not None:
            self.profile.add_commit_time("planned_record_s", time.perf_counter() - t_record_start)
            self.profile.commit_elapsed_s += time.perf_counter() - t_commit_start


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
    router: MeanderRouterLike,
) -> list[tuple[float, float]] | None:
    if record.corrected_centerline_um:
        return [(float(x_um), float(y_um)) for x_um, y_um in record.corrected_centerline_um]
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


def _grid_rect_area(parsed_grid_rect: tuple[int, int, int, int]) -> int:
    min_x, max_x, min_y, max_y = parsed_grid_rect
    return max(0, max_x - min_x + 1) * max(0, max_y - min_y + 1)


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
        net_id=record.net_id,
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
        endpoint_correction_error=record.endpoint_correction_error,
    )


def _build_planner_context(
    *,
    rust_backend: MeanderRustBackendLike,
    routed_net_records: list[RoutedNetRecord],
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    static_blocked_cells: Iterable[tuple[int, int]] | None,
    static_blocked_cell_handle: object | None = None,
    route_occupancy_radius_cells: int | None = None,
    meander_box_clearance_radius_cells: int = 0,
    route_clearance_radius_cells: int | None = None,
    debug_meanders: bool = False,
) -> _MeanderPlannerContext:
    """A fresh router and its `MeanderPlanner`: static cells, the routes as
    occupied cells (with their keepout radius) and the route geometries the
    planner needs. Records without a centerline cannot be registered; their cells join
    the static set. Timers only under ``debug_meanders``."""
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
    planner = rust_backend.MeanderPlanner(router)
    router_init_s = time.perf_counter() - t_router_start
    by_edge = {_record_edge_key(r): r for r in routed_net_records}
    if route_occupancy_radius_cells is None:
        route_occupancy_radius_cells = (
            0 if route_clearance_radius_cells is None else int(route_clearance_radius_cells)
        )
    route_occupancy_radius_cells = max(0, int(route_occupancy_radius_cells))
    meander_box_clearance_radius_cells = max(0, int(meander_box_clearance_radius_cells))
    centerline_lists_by_edge: dict[RoutedEdgeKey, list[tuple[float, float]]] = {}
    for edge_key, record in by_edge.items():
        centerline = _record_centerline_for_registration(record, router)
        if centerline is not None:
            centerline_lists_by_edge[edge_key] = centerline
    registerable_edge_keys = list(centerline_lists_by_edge)
    unregistered_edge_keys = [
        edge_key for edge_key in by_edge if edge_key not in centerline_lists_by_edge
    ]
    # One backend: the combined static-and-route registration runs whenever a
    # record has a centerline to register; the handle form when the static
    # cells arrive as a Rust handle and every record registers.
    can_use_registered_route_cells = bool(registerable_edge_keys)
    use_static_handle = (
        static_blocked_cell_handle is not None
        and can_use_registered_route_cells
        and not unregistered_edge_keys
    )
    base_static_reused = False
    base_static_from_handle = False
    if static_blocked_cells is None:
        if static_blocked_cell_handle is not None:
            base_static_cells: set[GridCell] = {
                (int(x), int(y))
                for x, y in cast(
                    Iterable[tuple[int, int]],
                    cast(Any, static_blocked_cell_handle).cells(),
                )
            }
            base_static_from_handle = True
        else:
            base_static_cells = set()
    elif isinstance(static_blocked_cells, set):
        base_static_cells = cast(set[GridCell], static_blocked_cells)
        base_static_reused = True
    else:
        base_static_cells = {(int(x), int(y)) for x, y in static_blocked_cells}
    bend_radius_um = float(grid_size_um_cfg) * float(bend_radius_cells)
    unregistered_route_static_cells: set[GridCell] = set()
    if can_use_registered_route_cells:
        for edge_key in unregistered_edge_keys:
            route_cells = _record_route_cells(by_edge[edge_key])
            if route_occupancy_radius_cells > 0:
                route_cells = _inflate_grid_cells(
                    route_cells,
                    radius_cells=route_occupancy_radius_cells,
                    width_cells=int(width),
                    height_cells=int(height),
                )
            unregistered_route_static_cells.update(route_cells)
    base_static_for_router = (
        base_static_cells | unregistered_route_static_cells
        if unregistered_route_static_cells
        else base_static_cells
    )
    if not can_use_registered_route_cells:
        router.set_static_cells(list(base_static_for_router))
    register_route_cells_s = 0.0
    register_route_geometry_s = 0.0
    rust_registration_profile: dict[str, float] = {}
    registered_geometry_index_by_edge: dict[RoutedEdgeKey, int] = {}
    registered_max_bumps_by_edge: dict[RoutedEdgeKey, int] = {}
    registered_open_cell_count = 0
    registered_unique_route_cell_count = 0
    if can_use_registered_route_cells:
        t_register_start = time.perf_counter()
        route_objects = [by_edge[edge_key].route_obj for edge_key in registerable_edge_keys]
        if use_static_handle:
            indices, open_counts, unique_count = planner.register(
                route_objects,
                base_static_cell_handle=static_blocked_cell_handle,
                route_clearance_radius_cells=route_occupancy_radius_cells,
            )
        else:
            indices, open_counts, unique_count = planner.register(
                route_objects,
                base_static_cells=list(base_static_for_router),
                route_clearance_radius_cells=route_occupancy_radius_cells,
            )
        register_route_cells_s = time.perf_counter() - t_register_start
        registered_unique_route_cell_count = int(unique_count)
        registered_open_cell_count = int(sum(int(raw_count) for raw_count in open_counts))
        if debug_meanders:
            raw_profile = planner.profile()
            if isinstance(raw_profile, Mapping):
                for key, value in raw_profile.items():
                    if isinstance(key, str) and isinstance(value, (int, float)):
                        rust_registration_profile[key] = float(value)
        t_geometry_start = time.perf_counter()
        geometry_max_bumps: list[int] = []
        for edge_key in registerable_edge_keys:
            max_bumps = _route_geometry_max_meander_bumps(
                record=by_edge[edge_key],
                grid_size_um=float(grid_size_um_cfg),
                bend_radius_um=bend_radius_um,
            )
            registered_max_bumps_by_edge[edge_key] = max_bumps
            geometry_max_bumps.append(max_bumps)
        geometry_indices = planner.register_geometries(
            [centerline_lists_by_edge[edge_key] for edge_key in registerable_edge_keys],
            [int(raw_index) for raw_index in indices],
            geometry_max_bumps,
        )
        register_route_geometry_s = time.perf_counter() - t_geometry_start
        for edge_key, raw_index in zip(registerable_edge_keys, geometry_indices, strict=False):
            registered_geometry_index_by_edge[edge_key] = int(raw_index)
    profile: _ContextProfile | None = None
    if debug_meanders:
        setup_profile = {
            "total_s": time.perf_counter() - t_total_start,
            "router_init_s": router_init_s,
            "register_route_cells_s": register_route_cells_s,
            "register_route_geometry_s": register_route_geometry_s,
            "base_static_reused": float(base_static_reused),
            "base_static_from_handle": float(base_static_from_handle),
            "combined_static_route_registration_handle": float(use_static_handle),
            "route_occupancy_radius_cells": float(route_occupancy_radius_cells),
            "meander_box_clearance_radius_cells": float(meander_box_clearance_radius_cells),
            "registered_route_cell_acceleration_enabled": float(can_use_registered_route_cells),
            "registered_record_count": float(len(registerable_edge_keys)),
            "unregistered_record_count": float(len(unregistered_edge_keys)),
            "unregistered_route_static_cell_count": float(len(unregistered_route_static_cells)),
            "routed_record_count": float(len(by_edge)),
            "unique_route_cell_count": float(registered_unique_route_cell_count),
            "registered_open_cell_count": float(registered_open_cell_count),
            "base_static_cell_count": float(len(base_static_cells)),
            "base_static_for_router_cell_count": float(len(base_static_for_router)),
        }
        for key, value in rust_registration_profile.items():
            setup_profile[f"rust_registration_{key}"] = float(value)
        profile = _ContextProfile(setup_profile=setup_profile)
    return _MeanderPlannerContext(
        planner=planner,
        by_edge=by_edge,
        updated=dict(by_edge),
        registered_geometry_index_by_edge=registered_geometry_index_by_edge,
        max_bumps_by_edge=registered_max_bumps_by_edge,
        base_static_cells=base_static_cells,
        grid_width_cells=int(width),
        grid_height_cells=int(height),
        grid_size_um=float(grid_size_um_cfg),
        bend_radius_um=bend_radius_um,
        route_occupancy_radius_cells=route_occupancy_radius_cells,
        meander_box_clearance_radius_cells=meander_box_clearance_radius_cells,
        profile=profile,
    )


def _meander_search_config(
    *,
    config: MeanderInsertionConfig,
    bend_radius_um: float,
    rust_backend: MeanderRustBackendLike | None = None,
) -> _MeanderSearchConfig:
    """The search policy (straights, box depths, endpoint insets) as Rust defines it."""
    backend = rust_backend if rust_backend is not None else _load_rust_backend()
    if backend is None:
        raise RuntimeError("Rust router backend unavailable for meander analysis.")
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
        "endpoint_insets_um": [float(value) for value in search_config.endpoint_insets_um],
        "endpoint_inset_policy": (
            "adaptive" if len(search_config.endpoint_insets_um) > 1 else "fixed"
        ),
    }


def _candidates_for_requirement(
    req: MissingLengthRequirement,
    *,
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

    return [
        DelayInsertionCandidate(
            requirement_edge_key=req.edge_key,
            edge_keys=(req.edge_key,),
            extra_length_um=float(req.missing_length_um),
            reason="direct_edge",
            affected_requirement_edge_keys=(req.edge_key,),
        )
    ]


def _requirement_missing_by_edge(
    requirements: Iterable[MissingLengthRequirement],
) -> dict[RoutedEdgeKey, float]:
    missing_by_edge: dict[RoutedEdgeKey, float] = {}
    for req in requirements:
        missing_by_edge[req.edge_key] = missing_by_edge.get(req.edge_key, 0.0) + float(
            req.missing_length_um
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
    status_key = (
        status if status in {"planned", "no_candidate", "already_satisfied"} else "no_candidate"
    )
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
    # One U-turn at the minimum amplitude (2r + min_straight), from the
    # fill-box model `extra = legs * (A - 4r + pi*r)` with legs = 2:
    return max(0.0, 2.0 * (min_straight + (math.pi - 2.0) * bend_radius_um))


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

    longest_straight_um = _longest_axis_aligned_centerline_run_um(record.corrected_centerline_um)
    if longest_straight_um <= 0.0:
        waypoints = getattr(record.route_obj, "compressed_waypoints", None) or []
        for p0, p1 in itertools.pairwise(waypoints):
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

    visible_lobes = math.floor(longest_straight_um / (4.0 * radius))
    return max(1, 2 * visible_lobes - 1)


def _longest_axis_aligned_centerline_run_um(
    centerline: tuple[tuple[float, float], ...],
) -> float:
    lengths = _axis_aligned_centerline_run_lengths_um(centerline)
    return max(lengths, default=0.0)


def _axis_aligned_centerline_run_lengths_um(
    centerline: tuple[tuple[float, float], ...],
) -> list[float]:
    if len(centerline) < 2:
        return []
    eps = 1.0e-9
    lengths: list[float] = []
    current = 0.0
    current_axis: str | None = None
    current_line_coord = 0.0
    current_dir = 0
    for p0, p1 in itertools.pairwise(centerline):
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
            if current > eps:
                lengths.append(current)
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
            if current > eps:
                lengths.append(current)
            current = length
            current_axis = axis
            current_line_coord = line_coord
        current_dir = direction
    if current > eps:
        lengths.append(current)
    lengths.sort(reverse=True)
    return lengths


def _plan_and_commit_final_physical_meanders(
    *,
    rust_backend: MeanderRustBackendLike,
    routed_net_records: list[RoutedNetRecord],
    realization_grid_spec: tuple[int, int, float, float, float],
    allow_45_degree_turns: bool,
    bend_radius_cells: int,
    static_blocked_cells: Iterable[tuple[int, int]] | None,
    static_blocked_cell_handle: object | None,
    route_occupancy_radius_cells: int | None,
    meander_box_clearance_radius_cells: int,
    route_clearance_radius_cells: int | None,
    config: MeanderInsertionConfig,
    search_config: _MeanderSearchConfig,
    min_insertable_extra_um: float,
    selected_requirements: list[_SelectedMeanderRequirement],
    physical_edge_order: list[RoutedEdgeKey],
    physical_planned_extra_by_edge: Mapping[RoutedEdgeKey, float],
    debug_meanders: bool,
) -> _FinalMeanderPlanningResult:
    """The final pass: every physical edge's total in one Rust call, committed.

    A fresh context (the selection pass reserved nothing), one vector with
    every edge that carries a planned total; Rust decides sequence, direct
    or split per edge. A requirement whose physical edges did not all plan
    is reported ``no_candidate`` with its credit unmatched.
    """
    final_context = _build_planner_context(
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
        debug_meanders=debug_meanders,
    )
    final_plan_by_edge: dict[RoutedEdgeKey, list[PlannedEdgeInsertion]] = {}
    final_failure_by_edge: dict[RoutedEdgeKey, str] = {}
    final_planning_mode = "none"
    final_planner_calls = 0
    final_planner_elapsed_s = 0.0

    final_edge_keys = [
        edge_key
        for edge_key in physical_edge_order
        if physical_planned_extra_by_edge.get(edge_key, 0.0) > EXACT_MEANDER_EPS_UM
    ]
    final_edge_requests = {
        edge_key: float(physical_planned_extra_by_edge[edge_key]) for edge_key in final_edge_keys
    }

    final_attempt = final_context.plan_final_requests_registered(
        edge_keys=final_edge_keys,
        planner_requests_by_edge=final_edge_requests,
        min_insertable_extra_um=min_insertable_extra_um,
        search_config=search_config,
        auto_endpoint_inset_um=config.auto_meander_endpoint_inset_um,
    )
    if final_attempt is not None:
        commit_plans, failure_reason, final_planner_elapsed_s, final_planner_calls, mode = (
            final_attempt
        )
        final_planning_mode = mode
        if commit_plans:
            for selected_edge_key, record, rr, max_bumps in commit_plans:
                requested = final_edge_requests.get(
                    selected_edge_key,
                    _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0),
                )
                final_plan_by_edge.setdefault(selected_edge_key, []).append(
                    (selected_edge_key, record, rr, max_bumps)
                )
                final_context.commit_planned_edge(
                    selected_edge_key=selected_edge_key,
                    record=record,
                    rr=rr,
                    requested=_as_float(rr.get("inserted_extra_length_um", requested), requested),
                    max_bumps=max_bumps,
                    search_config=search_config,
                    endpoint_inset_um=_as_float(
                        rr.get("endpoint_inset_um", search_config.endpoint_inset_um),
                        search_config.endpoint_inset_um,
                    ),
                )
        elif final_edge_keys:
            final_failure_by_edge[final_edge_keys[0]] = (
                failure_reason or "no exact final aggregate meander candidate found"
            )
    elif final_edge_keys:
        final_failure_by_edge[final_edge_keys[0]] = (
            "Rust registered PLM planner could not prepare final aggregate input"
        )

    total_inserted = 0.0
    for selected in selected_requirements:
        entry = selected.entry
        missing_final_edges = [
            edge_key for edge_key in selected.physical_edges if edge_key not in final_plan_by_edge
        ]
        if missing_final_edges:
            entry.status = "no_candidate"
            entry.reason = final_failure_by_edge.get(
                missing_final_edges[0],
                "final aggregate physical meander planning failed",
            )
            entry.inserted_extra_length_um = 0.0
            entry.physical_inserted_extra_length_um = 0.0
            entry.unmatched_length_um = selected.credit_extra_length
            entry.planned_edges = ()
            entry.plan = None
        else:
            final_plans = [
                final_plan
                for edge_key in selected.physical_edges
                for final_plan in final_plan_by_edge[edge_key]
            ]
            entry.physical_inserted_extra_length_um = sum(
                _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0)
                for _, _, rr, _ in final_plans
            )
            entry.plan = final_plans[0][2]
            entry.planned_edges = tuple(edge_key for edge_key, *_ in final_plans)
            total_inserted += selected.credit_extra_length * len(selected.affected_edges)
        for virtual_entry in selected.virtual_entries:
            _mirror_entry(entry, virtual_entry)

    total_physical_inserted = sum(
        _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0)
        for final_plans in final_plan_by_edge.values()
        for _, _, rr, _ in final_plans
    )
    realized_extra_by_edge: dict[RoutedEdgeKey, float] = {}
    for edge_key, final_plans in final_plan_by_edge.items():
        realized = sum(
            _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0) for _, _, rr, _ in final_plans
        )
        if realized > EXACT_MEANDER_EPS_UM:
            realized_extra_by_edge[edge_key] = realized
    return _FinalMeanderPlanningResult(
        updated_records=[
            final_context.updated.get(_record_edge_key(record), record)
            for record in routed_net_records
        ],
        total_inserted=total_inserted,
        total_physical_inserted=total_physical_inserted,
        realized_extra_by_edge=realized_extra_by_edge,
        planning_mode=final_planning_mode,
        planner_calls=final_planner_calls,
        planner_elapsed_s=final_planner_elapsed_s,
        final_context=final_context,
    )


def _mirror_entry(entry: MeanderEdgeReport, virtual_entry: MeanderEdgeReport) -> None:
    """A requirement satisfied through another's candidate reports that result."""
    virtual_entry.status = entry.status
    virtual_entry.reason = entry.reason
    virtual_entry.inserted_extra_length_um = entry.inserted_extra_length_um
    virtual_entry.physical_inserted_extra_length_um = entry.physical_inserted_extra_length_um
    virtual_entry.unmatched_length_um = entry.unmatched_length_um
    virtual_entry.candidate_reason = entry.candidate_reason
    virtual_entry.planned_edges = entry.planned_edges
    virtual_entry.planner_requested_extra_length_um = entry.planner_requested_extra_length_um
    virtual_entry.existing_physical_extra_length_um = entry.existing_physical_extra_length_um
    virtual_entry.plan = entry.plan


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
    requirement_delay_candidates: Mapping[
        RoutedEdgeKey,
        Iterable[DelayInsertionCandidate],
    ]
    | None = None,
    debug_meanders: bool = False,
) -> tuple[list[RoutedNetRecord], MeanderInsertionReport]:
    """Plan the meanders that satisfy the path-length requirements.

    Selection pass: for each requirement (in order, each credited with what
    earlier requirements already inserted on its edge) one Rust call decides
    among its candidates and the same-edge split. Final pass: one Rust call
    plans every physical edge's total and the plans are committed to the
    records. With ``debug_meanders`` the report carries a
    ``MeanderPlanningProfile``; without it no timer, counter or attempt list
    is built.
    """
    rust_backend = _load_rust_backend()
    if rust_backend is None:
        raise RuntimeError("Rust router backend unavailable for meander analysis.")
    rust_backend = cast(MeanderRustBackendLike, rust_backend)
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
        debug_meanders=debug_meanders,
    )
    search_config = _meander_search_config(
        config=config,
        bend_radius_um=context.bend_radius_um,
        rust_backend=rust_backend,
    )
    profile = MeanderPlanningProfile() if debug_meanders else None
    results: list[MeanderEdgeReport] = []
    total_disregarded = 0.0
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
    physical_planned_extra_by_edge: dict[RoutedEdgeKey, float] = {}
    physical_edge_order: list[RoutedEdgeKey] = []
    selected_requirements: list[_SelectedMeanderRequirement] = []

    for req in requirements:
        edge_key = req.edge_key
        already_inserted = effective_inserted_by_requirement_edge.get(edge_key, 0.0)
        requested = _normalize_minimum_insertable_request(
            max(0.0, float(req.missing_length_um) - already_inserted),
            minimum_insertable_extra_um=min_insertable_extra_um,
        )
        if requested <= EXACT_MEANDER_EPS_UM:
            continue
        entry = MeanderEdgeReport(
            edge=edge_key,
            requested_extra_length_um=requested,
            status="no_candidate",
            reason="no_matching_routed_record",
            unmatched_length_um=requested,
        )
        if requested < min_insertable_extra_um:
            total_disregarded += requested
            entry.status = "below_minimum_bump"
            entry.reason = (
                "requested extra length is below the four-90-degree-bend "
                f"minimum ({min_insertable_extra_um:.6g} um)"
            )
            entry.unmatched_length_um = 0.0
            results.append(entry)
            continue

        # The request vector: one work item per candidate, each edge asked
        # for its new total (the credit plus what it already carries).
        work_items: list[_CandidateWorkItem] = []
        for candidate in _candidates_for_requirement(
            req,
            requirement_delay_candidates=requirement_delay_candidates,
        ):
            affected_edges = candidate.affected_requirement_edge_keys or (edge_key,)
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
            candidate_requested = _normalize_minimum_insertable_request(
                min(float(candidate.extra_length_um), min(remaining_for_affected)),
                minimum_insertable_extra_um=min_insertable_extra_um,
            )
            existing_physical_extra = max(
                (
                    physical_planned_extra_by_edge.get(candidate_edge, 0.0)
                    for candidate_edge in candidate.edge_keys
                ),
                default=0.0,
            )
            already_satisfied = candidate_requested <= EXACT_MEANDER_EPS_UM
            work_items.append(
                _CandidateWorkItem(
                    candidate=candidate,
                    affected_edges=tuple(affected_edges),
                    requests_by_edge={
                        candidate_edge: candidate_requested
                        + physical_planned_extra_by_edge.get(candidate_edge, 0.0)
                        for candidate_edge in candidate.edge_keys
                    },
                    credit_extra_length=0.0 if already_satisfied else candidate_requested,
                    existing_physical_extra_length=existing_physical_extra,
                    already_satisfied=already_satisfied,
                )
            )

        outcome = context.plan_requirement_requests_registered(
            work_items=work_items,
            split_edge_key=edge_key,
            split_requested=requested,
            min_insertable_extra_um=min_insertable_extra_um,
            search_config=search_config,
            auto_endpoint_inset_um=config.auto_meander_endpoint_inset_um,
        )
        if profile is not None:
            profile.planner_calls += outcome.edge_calls
            profile.planner_elapsed_s += outcome.elapsed_s
            _record_requirement_attempt(
                profile,
                edge_key=edge_key,
                requested=requested,
                work_items=work_items,
                outcome=outcome,
            )

        if outcome.split_part_count is not None:
            selected_candidate = DelayInsertionCandidate(
                requirement_edge_key=edge_key,
                edge_keys=(edge_key,),
                extra_length_um=requested,
                reason="split_route_runs",
                affected_requirement_edge_keys=(edge_key,),
            )
            credit = requested
            selected_affected_edges: tuple[RoutedEdgeKey, ...] = (edge_key,)
            existing_physical_extra = 0.0
        elif outcome.selected_work_index is not None:
            selected_item = work_items[outcome.selected_work_index]
            selected_candidate = selected_item.candidate
            credit = selected_item.credit_extra_length
            selected_affected_edges = selected_item.affected_edges
            existing_physical_extra = selected_item.existing_physical_extra_length
        else:
            entry.status = "no_candidate"
            entry.reason = outcome.reason
            results.append(entry)
            continue

        selected_physical_total_by_edge: dict[RoutedEdgeKey, float] = {}
        for selected_edge_key, _record, rr, _max_bumps in outcome.plans:
            selected_physical_total_by_edge[selected_edge_key] = (
                selected_physical_total_by_edge.get(selected_edge_key, 0.0)
                + _as_float(rr.get("inserted_extra_length_um", 0.0), 0.0)
            )
        representative_rr = outcome.plans[0][2]
        entry.status = "planned"
        entry.reason = ""
        entry.inserted_extra_length_um = credit
        entry.physical_inserted_extra_length_um = sum(selected_physical_total_by_edge.values())
        entry.unmatched_length_um = 0.0
        entry.candidate_reason = selected_candidate.reason
        entry.planned_edges = tuple(planned_edge for planned_edge, *_ in outcome.plans)
        entry.planner_requested_extra_length_um = max(
            selected_physical_total_by_edge.values(), default=credit
        )
        entry.existing_physical_extra_length_um = existing_physical_extra
        entry.plan = representative_rr
        for affected_edge in selected_affected_edges:
            effective_inserted_by_requirement_edge[affected_edge] = (
                effective_inserted_by_requirement_edge.get(affected_edge, 0.0) + credit
            )
        for selected_edge_key, selected_physical_total in selected_physical_total_by_edge.items():
            physical_planned_extra_by_edge[selected_edge_key] = selected_physical_total
            if selected_edge_key not in physical_edge_order:
                physical_edge_order.append(selected_edge_key)
        results.append(entry)
        virtual_entries: list[MeanderEdgeReport] = []
        for affected_edge in selected_affected_edges:
            if affected_edge == edge_key:
                continue
            virtual_entry = MeanderEdgeReport(
                edge=affected_edge,
                requested_extra_length_um=requested,
                status=entry.status,
                satisfied_by_requirement_edge=edge_key,
            )
            _mirror_entry(entry, virtual_entry)
            results.append(virtual_entry)
            virtual_entries.append(virtual_entry)
        selected_requirements.append(
            _SelectedMeanderRequirement(
                entry=entry,
                virtual_entries=virtual_entries,
                edge_key=edge_key,
                affected_edges=selected_affected_edges,
                physical_edges=tuple(selected_physical_total_by_edge.keys()),
                credit_extra_length=credit,
            )
        )

    final_result = _plan_and_commit_final_physical_meanders(
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
        config=config,
        search_config=search_config,
        min_insertable_extra_um=min_insertable_extra_um,
        selected_requirements=selected_requirements,
        physical_edge_order=physical_edge_order,
        physical_planned_extra_by_edge=physical_planned_extra_by_edge,
        debug_meanders=debug_meanders,
    )
    if profile is not None:
        _fill_run_profile(
            profile,
            selection_profile=context.profile,
            final_result=final_result,
            search_config=search_config,
        )
    return (
        final_result.updated_records,
        MeanderInsertionReport(
            results=results,
            total_requested_extra_length_um=float(total_requested),
            total_inserted_extra_length_um=float(final_result.total_inserted),
            total_physical_inserted_extra_length_um=float(final_result.total_physical_inserted),
            total_disregarded_extra_length_um=float(total_disregarded),
            unmatched_length_um=float(max(0.0, total_requested - final_result.total_inserted)),
            realized_extra_by_edge=final_result.realized_extra_by_edge,
            minimum_insertable_extra_length_um=float(min_insertable_extra_um),
            final_planning_mode=final_result.planning_mode,
            profile=profile,
        ),
    )


def _fill_run_profile(
    profile: MeanderPlanningProfile,
    *,
    selection_profile: _ContextProfile | None,
    final_result: _FinalMeanderPlanningResult,
    search_config: _MeanderSearchConfig,
) -> None:
    """The run's timers: the two contexts' setup and Rust splits, the final pass, the commits."""
    final_profile = final_result.final_context.profile
    profile.planner_calls += final_result.planner_calls
    profile.planner_elapsed_s += final_result.planner_elapsed_s
    profile.final_planner_calls = final_result.planner_calls
    profile.final_planner_elapsed_s = final_result.planner_elapsed_s
    profile.search_config = _meander_search_config_to_debug_dict(search_config)
    if final_profile is None or selection_profile is None:
        return
    profile.commit_elapsed_s = final_profile.commit_elapsed_s
    profile.setup_profile = _merged_numeric_profiles(
        selection_profile.setup_profile, final_profile.setup_profile
    )
    profile.selection_setup_profile = selection_profile.setup_profile
    profile.final_setup_profile = final_profile.setup_profile
    profile.commit_profile = final_profile.commit_profile
    profile.rust_planner_profile = _merged_numeric_profiles(
        selection_profile.rust_planner_profile, final_profile.rust_planner_profile
    )
    profile.rust_wrapper_profile = _merged_numeric_profiles(
        selection_profile.rust_wrapper_profile, final_profile.rust_wrapper_profile
    )


def _record_requirement_attempt(
    profile: MeanderPlanningProfile,
    *,
    edge_key: RoutedEdgeKey,
    requested: float,
    work_items: list[_CandidateWorkItem],
    outcome: _RequirementPlanOutcome,
) -> None:
    """The debug record of one requirement: its request vector and what Rust decided."""
    status_by_reason = {
        str(status["candidate_reason"]): status for status in outcome.candidate_statuses
    }
    candidates: list[dict[str, object]] = []
    for work_index, work_item in enumerate(work_items):
        if work_item.already_satisfied:
            status = "already_satisfied"
        elif outcome.selected_work_index == work_index:
            status = "planned"
        else:
            status = "no_candidate"
        rust_status = status_by_reason.get(work_item.candidate.reason, {})
        candidates.append(
            {
                "candidate_reason": work_item.candidate.reason,
                "edges": [edge_key_to_dict(edge) for edge in work_item.candidate.edge_keys],
                "requested_extra_lengths_um": [
                    float(work_item.requests_by_edge[edge])
                    for edge in work_item.candidate.edge_keys
                ],
                "credit_extra_length_um": float(work_item.credit_extra_length),
                "status": status,
                "reason": str(rust_status.get("reason", "")) if status == "no_candidate" else "",
                "failed_edge_index": rust_status.get("failed_edge_index"),
            }
        )
        _record_candidate_profile(
            profile.candidate_profile,
            reason=work_item.candidate.reason,
            status=status,
            edge_count=len(work_item.candidate.edge_keys),
            edge_calls=0,
            elapsed_s=0.0,
        )
    split_tried = outcome.split_part_count is not None or any(
        status["candidate_reason"] == "split_route_runs" for status in outcome.candidate_statuses
    )
    if split_tried:
        split_status = "planned" if outcome.split_part_count is not None else "no_candidate"
        candidates.append(
            {
                "candidate_reason": "split_route_runs",
                "edges": [edge_key_to_dict(edge_key)],
                "requested_extra_lengths_um": [float(requested)],
                "split_part_count": outcome.split_part_count,
                "status": split_status,
                "reason": outcome.reason if split_status == "no_candidate" else "",
            }
        )
        _record_candidate_profile(
            profile.candidate_profile,
            reason="split_route_runs",
            status=split_status,
            edge_count=outcome.split_part_count or 1,
            edge_calls=0,
            elapsed_s=0.0,
        )
    if outcome.split_part_count is not None:
        selected_reason: str | None = "split_route_runs"
    elif outcome.selected_work_index is not None:
        selected_reason = work_items[outcome.selected_work_index].candidate.reason
    else:
        selected_reason = None
    # the call's cost goes to the candidate that planned (or, failing all, the last tried)
    charged_reason = selected_reason
    if charged_reason is None and candidates:
        charged_reason = str(candidates[-1]["candidate_reason"])
    if charged_reason is not None and charged_reason in profile.candidate_profile:
        charged = profile.candidate_profile[charged_reason]
        charged["edge_calls"] = _as_int(charged.get("edge_calls"), 0) + outcome.edge_calls
        charged["elapsed_s"] = _as_float(charged.get("elapsed_s"), 0.0) + outcome.elapsed_s
    profile.requirement_attempts.append(
        {
            "edge": edge_key_to_dict(edge_key),
            "requested_extra_length_um": float(requested),
            "planner_edge_calls": int(outcome.edge_calls),
            "planning_elapsed_s": float(outcome.elapsed_s),
            "selected_candidate_reason": selected_reason,
            "reason": outcome.reason,
            "candidates": candidates,
        }
    )
