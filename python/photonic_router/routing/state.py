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

The fields below are what the phases from 3 on still write into the shared
record, grouped by the phase that first writes each one, with the writing
(`w`), mutating (`m`) and reading (`r`) phases named per field; `0` is the
session before phase 3. A field written by more than one phase is marked
`# shared writer`: that is real coupling between phases.

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

    route_svgs: list[Path] = field(default_factory=list)
    """Per-route debug SVGs. m:6 r:6,9"""

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

    # ---- the search's bookkeeping: phase 5 resets it, phase 6 fills it --------
    route_bookkeeping: RouteBookkeeping | None = None
    """Records, opened cells and committed cells per net. w:5 r:6,7,8  # shared writer"""

    detour_pass_reroutes: int | None = None
    """Nets the detour pass (phase 7b) re-routed; None until the pass ran. w:7 r:9"""

    route_timing_buckets: dict[str, RouteTimingBucket] = field(default_factory=dict)
    """Per-attempt-kind timing buckets. w:5 m:6 r:6,7,9  # shared writer"""

    route_attempt_records: list[RouteAttemptRecord] = field(default_factory=list)
    """One record per search attempt. w:5 m:6 r:6,9  # shared writer"""

    repair_count: int = 0
    """Repair attempts the loop made. w:5,6 r:7,9  # shared writer"""

    simple_route_count: int = 0
    """Nets served by the Z-route shortcut. w:5,6 r:7,9  # shared writer"""

    total_expanded_states: int = 0
    """States the searches expanded in total. w:5,6 r:7  # shared writer"""

    native_repair_trace_records: list[dict[str, object]] = field(default_factory=list)
    """The loop's per-attempt trace, as the kernel reported it. w:5,6 r:6  # shared writer"""

    # ---- phase 8, verify_repair -------------------------------------------
    last_photonic_probe_layout: Component | None = None
    """The most recent internal photonic probe layout. w:8 r:8"""

    last_photonic_probe_records: list[RoutedNetRecord] = field(default_factory=list)
    """The records that probe layout was built from. w:8 r:8"""

    photonic_probe_index: int = 0
    """How many probe layouts this run has built. w:8 r:8"""

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
