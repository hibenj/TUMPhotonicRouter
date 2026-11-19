"""The routing session's per-run state: the record the phases from 3 on share.

A phase takes `(settings, state)`: `settings` is the frozen `SessionSettings`
(the run's inputs), `state` is this record. The phase numbers are the steps
of `session.run()`:

    1 obstacle_context   2 router_setup   3 route_jobs   4 crossing_plan_stage
    5 handoff   6 dispatch   7 finalize   8 verify_repair   9 realize

Phases 1 and 2 do not take the state. They return frozen products
(`products.py`), and the session builds the state from them with
`SessionState.after_setup`; the later phases read them as `state.obstacles`
and `state.kernel`. That is the form the coding standards ask of every
phase (rule 3.2: a stage hands its result on as a value).

Phases 3 to 5 store their results as frozen records of `products.py` in the
slots below, each assigned once and as a whole. Three things are still
mutated after their phase: the crossing plan (`crossing_plan_info`, which
phases 7 to 9 extend with the realized crossings), the search's log
(`SearchLog`) and the timing buckets of the pipeline. Per field the writing
(`w`), mutating (`m`) and reading (`r`) phases are named; `0` is the session
before phase 3, and `# shared writer` marks a field more than one phase
writes.

Apart from the four required fields the defaults are inert placeholders: a
field is created by its phase, and reading it earlier gives a recognisable
empty value.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from gdsfactory.component import Component

from photonic_router.records.route_rust_records import RouteBookkeeping
from photonic_router.records.route_rust_types import (
    RouteAttemptRecord,
    RoutedNetRecord,
    RouteTimingBucket,
)
from photonic_router.routing.crossing_plan_info import CrossingPlanInfo
from photonic_router.routing.products import (
    DenseRunways,
    EndpointIndex,
    FanoutStubs,
    KernelSetup,
    ObstacleContext,
    PortFootprints,
    RunPlan,
    StaticHandoffCells,
)

if TYPE_CHECKING:  # `route_jobs` imports this module, so keep the edge annotation-only
    pass


@dataclass
class SearchLog:
    """Everything the search writes down while it runs.

    The one record several phases write: phase 5 starts a fresh one, phase 6
    (and the detour pass, and phase 8's repairs) record routes and attempts
    in it, phase 7 adds the endpoint-correction timings, phase 9 reads it
    into the debug artifacts.
    """

    #: records, lengths and committed cells per net
    bookkeeping: RouteBookkeeping = field(default_factory=lambda: RouteBookkeeping(route_order=[]))
    #: per-attempt-kind timing buckets
    timing_buckets: dict[str, RouteTimingBucket] = field(default_factory=dict)
    #: one record per search attempt
    attempt_records: list[RouteAttemptRecord] = field(default_factory=list)
    #: the loop's per-attempt trace, as the kernel reported it
    native_repair_trace: list[dict[str, object]] = field(default_factory=list)
    #: per-route debug SVGs
    route_svgs: list[Path] = field(default_factory=list)
    repair_count: int = 0
    #: nets served by the simple-route shortcut
    simple_route_count: int = 0
    total_expanded_states: int = 0

    @classmethod
    def start(cls, route_order: list[int], *, track_dynamic_cells: bool) -> SearchLog:
        """A fresh log for the jobs of one run, with the four timing buckets."""
        return cls(
            bookkeeping=RouteBookkeeping(
                route_order=route_order, diagnostics_enabled=track_dynamic_cells
            ),
            timing_buckets={
                name: RouteTimingBucket()
                for name in ("normal_route", "probe_route", "braid_ripup", "endpoint_correction")
            },
        )


@dataclass
class ProbeCache:
    """The last internal photonic probe phase 8 built, for its failure artifacts."""

    #: how many probe layouts this run has built
    index: int = 0
    layout: Component | None = None
    #: the records that probe layout was built from
    records: list[RoutedNetRecord] = field(default_factory=list)


@dataclass
class SessionState:
    """Everything one routing run computes, in the order the phases produce it."""

    # ---- what the session has before the first phase that takes the state ---
    routed_layout: Component
    """The layout the run fills with routed geometry. w:0 r:3,4,9"""

    rust_backend: Any
    """The loaded `photonic_router._rust` module (PyO3). w:0 r:3,4,5"""

    obstacles: ObstacleContext
    """Phase 1's product (`products.py`), never rebound. r:3,4,5,6,7,8,9"""

    kernel: KernelSetup
    """Phase 2's product (`products.py`), never rebound. r:3,4,5,6,7,8,9"""

    route_nets_timings_s: dict[str, float] = field(default_factory=dict)
    """Per-phase pipeline timings (`timing.py`). w:0 m:1,2,6 r:6,9  # shared writer"""

    # ---- phase 3, route_jobs: one slot per step, assigned once ---------------
    endpoints: EndpointIndex = field(default_factory=EndpointIndex.empty)
    """Step 1's record: the ports the nets end on. w:3 r:3,4,7"""

    fanout: FanoutStubs = field(default_factory=FanoutStubs.empty)
    """Step 2's record: the static fan-out stubs. w:3 r:3,4,5,6,7"""

    runways: DenseRunways = field(default_factory=DenseRunways.empty)
    """Step 3's record: dense runway lengths and clusters. w:3 r:5,6"""

    # ---- phase 4, crossing_plan_stage -------------------------------------
    ports: PortFootprints = field(default_factory=PortFootprints.empty)
    """Phase 4's record: what every port opens and keeps clear. w:4 r:5,6"""

    crossing_plan_info: CrossingPlanInfo = field(default_factory=CrossingPlanInfo)
    """The crossing plan and its realized crossings. w:4 m:4,7,8,9 r:4,5,7,8,9  # shared writer"""

    # ---- phase 5, handoff --------------------------------------------------
    static_cells: StaticHandoffCells = field(default_factory=StaticHandoffCells.empty)
    """What the kernel was given beyond the obstacle map. w:5 r:6"""

    run: RunPlan = field(default_factory=RunPlan.empty)
    """The jobs of this run and what was precomputed for them. w:5 r:6,7,8,9"""

    # ---- the two records the later phases mutate ------------------------------
    search: SearchLog = field(default_factory=SearchLog)
    """What the search recorded. w:5 m:6,7,7b,8 r:6,7,8,9  # shared writer"""

    probe: ProbeCache = field(default_factory=ProbeCache)
    """Phase 8's own working record. w:8 m:8 r:8"""

    @classmethod
    def after_setup(
        cls,
        *,
        routed_layout: Component,
        rust_backend: Any,
        timings_s: dict[str, float],
        obstacles: ObstacleContext,
        kernel: KernelSetup,
    ) -> SessionState:
        """Build the state the phases from 3 on share, from the products of 1 and 2.

        `timings_s` is the bucket phases 1 and 2 already wrote to.
        """
        return cls(
            routed_layout=routed_layout,
            rust_backend=rust_backend,
            obstacles=obstacles,
            kernel=kernel,
            route_nets_timings_s=timings_s,
        )
