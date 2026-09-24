"""The nine routing phases as declared interfaces.

Milestone 5 Slice 2b: one `typing.Protocol` per phase of `session.run()`, plus a
small frozen dataclass wherever a phase used to return a bare tuple. Every phase
has the same first two parameters -- the run's frozen inputs (`SessionSettings`,
Slice 2a) and its mutable record (`SessionState`, this slice) -- followed by that
phase's own main input, and returns that phase's product as a value:

    1 ObstacleContextBuilder (settings, state)             -> ObstacleContext
    2 RouterSetup            (settings, state, obstacle_map) -> None
    3 RouteJobBuilder        (settings, state, nets)        -> RouteJobsResult
    4 CrossingPlanner        (settings, state, jobs, ...)   -> PlannedJobs
    5 StaticHandoff          (settings, state, jobs, ...)   -> FinalJobs
    6 KernelDispatcher       (settings, state, final_jobs)  -> None
    7 ResultFinalizer        (settings, state, jobs, t0)    -> RoutedRecords
    8 GeometryVerifier       (settings, state, records)     -> VerifiedRecords
    9 Realizer               (settings, state, ...)         -> RustRouteDebugArtifacts

Two of the nine have no product of their own and return `None` because their
whole effect is on the state and on the kernel: phase 2 builds the router and its
configuration objects into `state` (`router`, `primitive_cfg`, `astar_cfg`, the
clearance radii, the raw static geometry), and phase 6 runs the kernel and leaves
the routed results in `state.route_bookkeeping` together with the loop's counters
(`deferred_count`, `repair_count`, `simple_route_count`, `total_expanded_states`,
`native_repair_trace_records`, `route_attempt_records`, `route_timing_buckets`).
Those writes are documented per field in `state.py`.

The Protocols are declared here, not next to the implementations, so that the
flow reads as a list of interfaces and so a second implementation of a phase (a
different crossing planner, say) has one place to answer to -- the same pattern
`photonic_router.static_obstacle_builder.ObstacleMapBuilder` already sets.
Slice 2c typed the last untyped thing the phases share, the crossing plan itself:
`state.crossing_plan_info` is a `CrossingPlanInfo` dataclass
(`translation/routing/crossing_plan_info.py`), one field per key, and `to_dict()`
is what reaches the mapping consumers outside the session.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from translation.route_rust_types import (
    RoutedNetRecord,
    RouteJob,
    RustRouteDebugArtifacts,
)
from translation.routing.settings import SessionSettings
from translation.routing.state import SessionState


@dataclass(frozen=True)
class ObstacleContext:
    """Phase 1's product: the static obstacle map and what phases 4 to 9 need of it."""

    obstacle_map: Any  # the obstacle builder's result object
    crossing_device_info: dict[str, Any]
    obstacle_svg: Path | None


@dataclass(frozen=True)
class RouteJobsResult:
    """Phase 3's product: the route jobs plus the two per-spec maps phase 4 consumes."""

    route_jobs: list[RouteJob]
    endpoint_port_specs_by_instance: dict[str, set[str]]
    dense_port_runway_length_by_spec: dict[str, int]


@dataclass(frozen=True)
class PlannedJobs:
    """Phase 4's product: the jobs after crossing planning, and the foreign keepouts."""

    route_jobs: list[RouteJob]
    foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]]


@dataclass(frozen=True)
class FinalJobs:
    """Phase 5's product: the jobs as handed to the kernel, and when the search started."""

    route_jobs: list[RouteJob]
    astar_start_s: float


@dataclass(frozen=True)
class RoutedRecords:
    """Phase 7's product: one record per routed net, and the search's elapsed time."""

    routed_net_records: list[RoutedNetRecord]
    astar_elapsed_s: float


@dataclass(frozen=True)
class VerifiedRecords:
    """Phase 8's product: the records after final repair, and the crossings still illegal."""

    routed_net_records: list[RoutedNetRecord]
    illegal_realized_crossings: list[dict[str, object]]


class ObstacleContextBuilder(Protocol):
    """Phase 1: build the static obstacle map and the grid it defines."""

    def __call__(self, settings: SessionSettings, state: SessionState) -> ObstacleContext: ...


class RouterSetup(Protocol):
    """Phase 2: build the kernel's router and its configuration objects.

    Returns nothing: its product is `state.router` and the derived grid,
    clearance and static-geometry fields listed in `state.py` under phase 2.
    """

    def __call__(
        self, settings: SessionSettings, state: SessionState, obstacle_map: Any
    ) -> None: ...


class RouteJobBuilder(Protocol):
    """Phase 3: turn the netlist into route jobs and cluster the dense fan-out ports."""

    def __call__(
        self, settings: SessionSettings, state: SessionState, nets: Mapping[str, Any]
    ) -> RouteJobsResult: ...


class CrossingPlanner(Protocol):
    """Phase 4: plan the crossings and resolve the port footprints."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        endpoint_port_specs_by_instance: dict[str, set[str]],
        dense_port_runway_length_by_spec: dict[str, int],
        obstacle_map: Any,
        crossing_device_info: dict[str, Any],
    ) -> PlannedJobs: ...


class StaticHandoff(Protocol):
    """Phase 5: order the jobs, reserve the static cells and hand them to the kernel."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]],
        obstacle_map: Any,
    ) -> FinalJobs: ...


class KernelDispatcher(Protocol):
    """Phase 6: run the kernel's negotiated loop over the final jobs.

    Returns nothing: it mutates the state. The routed results land in
    `state.route_bookkeeping` and the loop's counters in the fields `state.py`
    lists under phase 5/6 as shared writers.
    """

    def __call__(
        self, settings: SessionSettings, state: SessionState, route_jobs: list[RouteJob]
    ) -> None: ...


class ResultFinalizer(Protocol):
    """Phase 7: turn the kernel's routes into records and apply endpoint corrections."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        t_astar_start: float,
    ) -> RoutedRecords: ...


class GeometryVerifier(Protocol):
    """Phase 8: verify the final geometry and run the final repair passes."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        routed_net_records: list[RoutedNetRecord],
    ) -> VerifiedRecords: ...


class Realizer(Protocol):
    """Phase 9: realize the polygons into the layout and assemble the debug artifacts."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        routed_net_records: list[RoutedNetRecord],
        illegal_realized_crossings: list[dict[str, object]],
        obstacle_map: Any,
        obstacle_svg: Path | None,
        astar_elapsed_s: float,
    ) -> RustRouteDebugArtifacts: ...
