"""The nine routing phases as declared interfaces.

One `typing.Protocol` per phase of `session.run()`, plus a small frozen
dataclass for each product that is not one of `products.py`. Phases 1 and 2
run before the session state exists and return the two products the state
is built from; every later phase takes the run's frozen inputs
(`SessionSettings`) and its shared record (`SessionState`), followed by its
own main input, and returns its product as a value:

    1 ObstacleContextBuilder (settings, backend, timings)          -> ObstacleContext
    2 RouterSetup            (settings, backend, context, timings) -> KernelSetup
    3 RouteJobBuilder        (settings, state, nets)        -> RouteJobsResult
    4 CrossingPlanner        (settings, state, jobs, ...)   -> PlannedJobs
    5 StaticHandoff          (settings, state, jobs, ...)   -> FinalJobs
    6 KernelDispatcher       (settings, state, final_jobs)  -> None
    7 ResultFinalizer        (settings, state, jobs, t0)    -> RoutedRecords
    8 GeometryVerifier       (settings, state, records)     -> VerifiedRecords
    9 Realizer               (settings, state, jobs, routed, verified) -> RustRouteDebugArtifacts

Phase 6 has no product of its own and returns `None`: it runs the kernel and
leaves the routed results in `state.search.bookkeeping` together with the
loop's counters (`repair_count`, `simple_route_count`,
`total_expanded_states`, `native_repair_trace_records`,
`route_attempt_records`, `route_timing_buckets`). Those writes are
documented per field in `state.py`.

The Protocols are declared here, not next to the implementations, so that the
flow reads as a list of interfaces and so a second implementation of a phase (a
different crossing planner, say) has one place to answer to -- the same pattern
`photonic_router.static_obstacle_builder.ObstacleMapBuilder` already sets.
The crossing plan the phases share is typed as well: `state.crossing_plan_info`
is a `CrossingPlanInfo` dataclass (`crossing_plan_info.py`), and `to_dict()` is
what reaches the mapping consumers outside the session.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from photonic_router.records.route_rust_types import (
    RoutedNetRecord,
    RouteJob,
    RustRouteDebugArtifacts,
)
from photonic_router.routing.products import KernelSetup, ObstacleContext
from photonic_router.routing.settings import SessionSettings
from photonic_router.routing.state import SessionState


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
    #: nets the detour pass (phase 7b) re-routed; None when the pass did not run
    detour_pass_reroutes: int | None = None


@dataclass(frozen=True)
class VerifiedRecords:
    """Phase 8's product: the records after final repair, and the crossings still illegal."""

    routed_net_records: list[RoutedNetRecord]
    illegal_realized_crossings: list[dict[str, object]]


class ObstacleContextBuilder(Protocol):
    """Phase 1: build the static obstacle map and the grid it defines."""

    def __call__(
        self, settings: SessionSettings, rust_backend: Any, timings_s: dict[str, float]
    ) -> ObstacleContext:
        """Build the obstacle context of the session described by `settings`.

        `rust_backend` is the loaded kernel module. `timings_s` is the run's
        pipeline timing record (name to seconds); the phase adds its own
        entry to it in place.
        """
        ...


class RouterSetup(Protocol):
    """Phase 2: build the kernel's router and its configuration objects."""

    def __call__(
        self,
        settings: SessionSettings,
        rust_backend: Any,
        context: ObstacleContext,
        timings_s: dict[str, float],
    ) -> KernelSetup:
        """Build the kernel setup for the grid and obstacle map of `context`.

        `context` is phase 1's product and `rust_backend` the loaded kernel
        module. The phase adds its own entry to `timings_s` (name to
        seconds) in place.
        """
        ...


class RouteJobBuilder(Protocol):
    """Phase 3: turn the netlist into route jobs and cluster the dense fan-out ports."""

    def __call__(
        self, settings: SessionSettings, state: SessionState, nets: Mapping[str, Any]
    ) -> RouteJobsResult:
        """Build one route job per link of `nets`.

        `nets` is the netlist's routes: net name to a bundle whose `links`
        maps one `"instance,port"` string to another. The phase stores its
        endpoint, fan-out and runway records in `state`.
        """
        ...


class CrossingPlanner(Protocol):
    """Phase 4: plan the crossings and resolve the port footprints."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        endpoint_port_specs_by_instance: dict[str, set[str]],
        dense_port_runway_length_by_spec: dict[str, int],
    ) -> PlannedJobs:
        """Plan the crossings of `route_jobs` and size every port's footprint.

        The three arguments after `state` are the fields of phase 3's
        `RouteJobsResult`; the runway lengths are in cells. The phase stores
        the crossing plan (`state.crossing_plan_info`) and the port
        footprints (`state.ports`).
        """
        ...


class StaticHandoff(Protocol):
    """Phase 5: order the jobs, reserve the static cells and hand them to the kernel."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        foreign_port_keepout_cells_by_instance: dict[str, set[tuple[int, int]]],
    ) -> FinalJobs:
        """Put the jobs in routing order and give the kernel its static cells.

        `route_jobs` and `foreign_port_keepout_cells_by_instance` (grid cells
        by instance name) are the fields of phase 4's `PlannedJobs`. The
        phase stores `state.static_cells` and `state.run` and starts the
        search log `state.search`. The product's `astar_start_s` is a
        `time.perf_counter` reading, 0.0 when the run collects no timing.
        """
        ...


class KernelDispatcher(Protocol):
    """Phase 6: run the kernel's negotiated loop over the final jobs.

    Returns nothing: it mutates the state. The routed results land in
    `state.search.bookkeeping` and the loop's counters in the fields `state.py`
    lists under phase 5/6 as shared writers.
    """

    def __call__(
        self, settings: SessionSettings, state: SessionState, route_jobs: list[RouteJob]
    ) -> None:
        """Route `route_jobs`, the jobs of phase 5's `FinalJobs`, and record the results.

        Mutates `state` only. Raises `RuntimeError` when a net cannot be
        routed.
        """
        ...


class ResultFinalizer(Protocol):
    """Phase 7: turn the kernel's routes into records and apply endpoint corrections."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        t_astar_start: float,
    ) -> RoutedRecords:
        """Assemble one record per routed net from the search's bookkeeping.

        `route_jobs` and `t_astar_start` are the fields of phase 5's
        `FinalJobs`; the product's `astar_elapsed_s` is the time since
        `t_astar_start` in seconds, 0.0 when the run collects no timing.
        Raises `RuntimeError` when a net has more than one record.
        """
        ...


class DetourPass(Protocol):
    """Phase 7b (optional, `settings.detour_pass_factor > 0`): rip up every net whose
    route exceeds the factor times its free-space lower bound and route those nets
    again together, shortest first; returns the refreshed records."""

    def __call__(
        self, settings: SessionSettings, state: SessionState, routed: RoutedRecords
    ) -> RoutedRecords:
        """Re-route the detoured nets of `routed`, phase 7's product.

        Returns the refreshed records with the number of nets re-routed.
        Raises `RuntimeError` when a ripped-up net is not routed again.
        """
        ...


class GeometryVerifier(Protocol):
    """Phase 8: verify the final geometry and run the final repair passes."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        routed_net_records: list[RoutedNetRecord],
    ) -> VerifiedRecords:
        """Check the geometry of `routed_net_records` and repair what is illegal.

        The records are those of phase 7, after the detour pass when it ran.
        Returns the records as they are after repair and the realized
        crossings that are still illegal, an empty list for clean geometry.
        Raises `RuntimeError` when the geometry cannot be made legal.
        """
        ...


class Realizer(Protocol):
    """Phase 9: realize the polygons into the layout and assemble the debug artifacts."""

    def __call__(
        self,
        settings: SessionSettings,
        state: SessionState,
        route_jobs: list[RouteJob],
        routed: RoutedRecords,
        verified: VerifiedRecords,
    ) -> RustRouteDebugArtifacts:
        """Add the routes to `state.routed_layout` and build the run's debug record.

        `route_jobs` are the jobs of this run, `routed` phase 7's product
        (after the detour pass when it ran) and `verified` phase 8's. Nothing
        is added to the layout when `settings.defer_realization` is set.
        Raises `RuntimeError` when a realized crossing is still illegal.
        """
        ...
