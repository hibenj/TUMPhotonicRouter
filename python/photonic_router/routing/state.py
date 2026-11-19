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
from gdsfactory.typings import Port

from photonic_router.records.route_rust_records import RouteBookkeeping
from photonic_router.records.route_rust_types import (
    RipupRerouteConfig,
    RouteAttemptRecord,
    RoutedNetRecord,
    RouteJob,
    RouteTimingBucket,
)
from photonic_router.routing.crossing_plan_info import CrossingPlanInfo
from photonic_router.routing.products import KernelSetup, ObstacleContext

if TYPE_CHECKING:  # `route_jobs` imports this module, so keep the edge annotation-only
    from photonic_router.routing.route_jobs import _FanoutAnchor


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

    # ---- indexes of phase 2's raw static geometry, built on demand ----------
    raw_static_cells_by_y: dict[int, set[int]] | None = None
    """Row index of `kernel.static.raw_static_cells`, built on demand. w:4 r:4"""

    raw_static_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
    """Row index of the raw static rectangles, built on demand. w:4 r:4"""

    heater_opening_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
    """Row index of the heater opening rectangles, on demand. w:4 r:4"""

    static_blocked_cells_before_port_reservations: set[tuple[int, int]] = field(default_factory=set)
    """Static blockers as of before port reservations; starts as the raw static cells. w:0,5 m:5 r:5,6  # shared writer"""

    # ---- phase 3, route_jobs ----------------------------------------------
    endpoint_ports_by_spec: dict[str, tuple[str, str, Port]] = field(default_factory=dict)
    """`"inst,port"` -> (instance, port name, port). w:3 r:3,4,7"""

    source_port_specs_by_instance: dict[str, set[str]] = field(default_factory=dict)
    """Source port specs per instance. w:3 r:3"""

    target_port_specs_by_instance: dict[str, set[str]] = field(default_factory=dict)
    """Target port specs per instance. w:3 r:3"""

    source_port_specs_by_instance_angle: dict[tuple[str, int], set[str]] = field(
        default_factory=dict
    )
    """Source port specs per (instance, outgoing angle). w:3 r:3"""

    target_port_specs_by_instance_angle: dict[tuple[str, int], set[str]] = field(
        default_factory=dict
    )
    """Target port specs per (instance, incoming angle). w:3 r:3"""

    dense_source_cluster_specs_by_port_spec: dict[str, set[str]] = field(default_factory=dict)
    """The dense fan-out cluster each source port belongs to. w:3 r:3,5,6"""

    port_access_cells_by_spec: dict[str, set[tuple[int, int]]] = field(default_factory=dict)
    """Cells opened for each port's access. w:3 m:4 r:4,5  # shared writer"""

    port_access_candidate_cells_by_spec: dict[str, set[tuple[int, int]]] = field(
        default_factory=dict
    )
    """Cells a port's access rule may open. w:3 m:4 r:4,5  # shared writer"""

    port_access_rule_by_spec: dict[str, str | None] = field(default_factory=dict)
    """Name of the access rule applied per port spec. w:3 m:4 r:4,6  # shared writer"""

    port_runway_cells_by_spec: dict[str, set[tuple[int, int]]] = field(default_factory=dict)
    """Runway (approach lane) cells per port spec. w:3 m:4 r:4,5,6  # shared writer"""

    fanout_anchor_by_port_spec: dict[str, _FanoutAnchor] = field(default_factory=dict)
    """Static fan-out stub anchor per port spec. w:3 r:3,4,5,6,7"""

    fanout_anchor_net_ids: set[int] = field(default_factory=set)
    """Nets with a static fan-out stub on either side. w:3 r:4,7"""

    fanout_anchor_source_net_ids: set[int] = field(default_factory=set)
    """Nets with a static fan-out stub on the source side. w:3 r:4,7"""

    fanout_anchor_target_net_ids: set[int] = field(default_factory=set)
    """Nets with a static fan-out stub on the target side. w:3 r:4,7"""

    fanout_stub_static_cells: set[tuple[int, int]] = field(default_factory=set)
    """All cells the static fan-out stubs occupy. w:3 r:3,4,5,6"""

    fanout_stub_center_cells: set[tuple[int, int]] = field(default_factory=set)
    """Centerline cells of the static fan-out stubs. w:3 r:3,4,6"""

    fanout_stub_static_cells_by_spec: dict[str, set[tuple[int, int]]] = field(default_factory=dict)
    """Inflated stub cells per port spec. w:3 r:3,5,7"""

    dense_source_port_runway_length_by_spec: dict[str, int] = field(default_factory=dict)
    """Staggered source runway length per dense port spec. w:3 r:3,6"""

    dense_target_port_runway_length_by_spec: dict[str, int] = field(default_factory=dict)
    """Staggered target runway length per dense port spec. w:3 r:3,6"""

    # ---- phase 4, crossing_plan_stage -------------------------------------
    crossing_plan_info: CrossingPlanInfo = field(default_factory=CrossingPlanInfo)
    """The crossing plan and its realized crossings. w:4 m:4,7,8,9 r:4,5,7,8,9  # shared writer"""

    foreign_port_keepout_cells_by_spec: dict[str, set[tuple[int, int]]] = field(
        default_factory=dict
    )
    """Cells kept clear around foreign ports, per spec. w:4 r:4,5,6"""

    normal_port_runway_cells: set[tuple[int, int]] = field(default_factory=set)
    """Runway cells of the non-dense ports. w:4 r:4,5,6"""

    dense_port_lateral_windows: dict[str, tuple[float, float, float, float, float]] = field(
        default_factory=dict
    )
    """Lateral window each dense port may use. w:4 r:4,5"""

    dense_port_lateral_owner_groups: dict[
        str, tuple[float, float, tuple[tuple[str, float], ...]]
    ] = field(default_factory=dict)
    """Owner group sharing one dense lateral window. w:4 r:4,5"""

    port_state_lane_offsets: dict[tuple[str, bool], tuple[int, int]] = field(default_factory=dict)
    """Lane offset assigned to each (port spec, as_target). w:4 r:4,5"""

    # ---- phase 5, handoff --------------------------------------------------
    route_order: list[int] = field(default_factory=list)
    """Net ids in the order the kernel routes them. w:5 r:5,8"""

    route_jobs_by_id: dict[int, RouteJob] = field(default_factory=dict)
    """The routed jobs by net id. w:5 r:6,7,8"""

    full_route_jobs_by_route_index: dict[int, RouteJob] = field(default_factory=dict)
    """Every job (routed or not) by route index. w:5 r:6"""

    route_state_openings_by_id: dict[
        int, tuple[Any, Any, set[tuple[int, int]], set[tuple[int, int]], list[tuple[int, int]]]
    ] = field(default_factory=dict)
    """Per net: source/target state, opened cells, exempt cells. w:5 r:5,6"""

    batch_clearance_exempt_cells_by_id: dict[int, list[tuple[int, int]]] = field(
        default_factory=dict
    )
    """Clearance-exempt cells the kernel batch was given. w:5 r:6"""

    foreign_port_keepout_static_cells: set[tuple[int, int]] = field(default_factory=set)
    """Foreign-port keepout cells added to the static map. w:5 m:5 r:5,6"""

    debug_port_keepout_cells: set[tuple[int, int]] = field(default_factory=set)
    """Port keepout cells, for the debug artifacts only. w:5 m:5 r:5,6"""

    blocked_static_rects_for_diagnostics: list[tuple[int, int, int, int]] = field(
        default_factory=list
    )
    """Static blocker rectangles, for diagnostics only. w:5 m:5 r:5,6"""

    chip_boundary_static_rects: list[tuple[int, int, int, int]] = field(default_factory=list)
    """The die keepout rectangles. w:5 r:-"""

    realization_grid_spec: tuple[int, int, float, float, float] | None = None
    """(width, height, cell size, origin x, origin y) for realization. w:5 r:7,8,9"""

    repair_config: RipupRerouteConfig | None = None
    """The rip-up / reroute configuration the loop runs with. w:5 r:6,8"""

    route_bookkeeping: RouteBookkeeping | None = None
    """Records, opened cells and committed cells per net. w:5 r:6,7,8  # shared writer"""

    detour_pass_reroutes: int | None = None
    """Nets the detour pass (phase 7b) re-routed; None until the pass ran. w:7 r:9"""

    collect_timing: bool = False
    """Whether per-net timing buckets are filled. w:5 r:5,6,7"""

    track_dynamic_cells: bool = False
    """Whether committed dynamic cells are tracked per net. w:5 r:5,6"""

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
            static_blocked_cells_before_port_reservations=kernel.static.raw_static_cells,
        )
