"""The routing session's per-run state: one mutable record the nine phases share.

Milestone 5 Slice 2b (see the ExecPlan's Decision Log entry of 2026-09-24 for why
the slice was reshaped): the session used to carry this as ~80 loose attributes
created wherever a phase happened to need them, so which phase produced a value
and which phase consumed it was only discoverable by reading all nine stage
modules. It is now one dataclass, grouped by the phase that FIRST writes each
field, with the writing and reading phases named per field. A stage function
takes `(settings, state)`: `settings` is the frozen `SessionSettings` of Slice 2a
(the run's inputs), `state` is this record (everything the run computes).

The phase numbers are the nine `run()` steps:

    1 obstacle_context   2 router_setup   3 route_jobs   4 crossing_plan_stage
    5 handoff   6 dispatch   7 finalize   8 verify_repair   9 realize

`0` is the session constructor. A field written by more than one phase is marked
`# shared writer`: that is real coupling between phases, not an accident of the
old attribute soup, and it is the reason this slice documents the state instead
of forcing a single owner on every field. The 19 shared writers are the concrete
list Milestone 8 negotiates (the reshaping note counted 16; the three extra are
`route_attempt_records`, `route_svgs` and `route_timing_buckets`, which phase 5
creates and phase 6 appends to, plus `route_bookkeeping`, whose phase-7 writes go
through its own attributes rather than a rebinding of the field).

Defaults are inert placeholders only. Every field is still created by the same
statement in the same phase as before, so nothing is computed earlier than it
used to be; the defaults exist so that `SessionState()` is constructible and so
that reading a field before its phase has run is a recognisable empty value
rather than an `AttributeError`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from gdsfactory.component import Component
from gdsfactory.typings import Port
from photonic_router.static_obstacle_builder import GridSpec, StaticObstacleMapConfig

from translation.route_rust_records import RouteBookkeeping
from translation.route_rust_types import (
    OpticalRouteClearancePolicy,
    RipupRerouteConfig,
    RouteAttemptRecord,
    RoutedNetRecord,
    RouteJob,
    RouteTimingBucket,
)

if TYPE_CHECKING:  # `route_jobs` imports this module, so keep the edge annotation-only
    from translation.routing.route_jobs import _FanoutAnchor


@dataclass
class SessionState:
    """Everything one routing run computes, in the order the phases produce it."""

    # ---- phase 0, the session constructor ---------------------------------
    routed_layout: Component | None = None
    """The layout the run fills with routed geometry. w:0 r:3,4,9"""

    rust_backend: Any = None
    """The loaded `photonic_router._rust` module (PyO3). w:0 r:1,2,3,4,5"""

    route_nets_timings_s: dict[str, float] = field(default_factory=dict)
    """Per-phase pipeline timings (`timing.py`). w:0 m:6 r:1,6,9  # shared writer"""

    # ---- phase 1, obstacle_context ----------------------------------------
    grid: GridSpec | None = None
    """The routing grid of the static obstacle map. w:1 r:1,2,3,4,5,6,7,8"""

    resolved_obstacle_config: StaticObstacleMapConfig | None = None
    """The obstacle configuration actually used. w:1 r:1,2,4"""

    resolved_crossing_half_size_cells: int = 0
    """Crossing footprint half-size after resolution. w:1 r:4,8"""

    debug_path: Path | None = None
    """The debug artifact directory, or None. w:1 r:1,6,8,9"""

    diagnostics_enabled: bool = False
    """Whether debug diagnostics are collected (debug_path is set). w:1 r:5,6,7"""

    route_svgs: list[Path] = field(default_factory=list)
    """Per-route debug SVGs. w:1 m:6 r:6,9  # shared writer"""

    # ---- phase 2, router_setup --------------------------------------------
    grid_width: int = 0
    """Grid width in cells. w:2 r:4,6"""

    grid_height: int = 0
    """Grid height in cells. w:2 r:4,6"""

    origin_x_um: float = 0.0
    """Grid origin x in micrometres. w:2 r:2,3,4,5,8"""

    origin_y_um: float = 0.0
    """Grid origin y in micrometres. w:2 r:2,3,4,5,8"""

    raw_static_cells: set[tuple[int, int]] = field(default_factory=set)
    """Static blocked cells before any clearance inflation. w:2 r:2,4,5"""

    raw_static_rects_for_openings: list[tuple[int, int, int, int]] = field(default_factory=list)
    """Raw static geometry as rectangles, for port openings. w:2 r:2,4"""

    heater_opening_rects_for_openings: list[tuple[int, int, int, int]] = field(
        default_factory=list
    )
    """Raw plus heater-layer rectangles, for heater openings. w:2 r:4"""

    raw_static_cells_by_y: dict[int, set[int]] | None = None
    """Row index of `raw_static_cells`, built on demand. w:2,4 r:4  # shared writer"""

    raw_static_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
    """Row index of the raw static rectangles, built on demand. w:2,4 r:4  # shared writer"""

    heater_opening_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
    """Row index of the heater opening rectangles, on demand. w:2,4 r:4  # shared writer"""

    static_blocked_cells_before_port_reservations: set[tuple[int, int]] = field(
        default_factory=set
    )
    """Static blockers as of before port reservations. w:2,5 m:5 r:5,6  # shared writer"""

    clearance_policy: OpticalRouteClearancePolicy | None = None
    """The derived clearance radii for this route width and grid. w:2 r:2,6"""

    route_clearance_um: float = 0.0
    """Clearance of the resolved obstacle configuration. w:2 r:2,4"""

    bend_radius_cells: int = 0
    """Bend radius in grid cells. w:2 r:2,3,4,5,6,7,8,9"""

    commit_radius_cells: int = 0
    """Keepout radius committed around a routed net. w:2 r:2,3,5,6,7,8"""

    core_commit_radius_cells: int = 0
    """Core (waveguide) radius of a committed route. w:2 r:6,7,8"""

    block_radius_cells: int = 0
    """Search expansion radius around dynamic obstacles. w:2 r:6,8"""

    port_lane_length_cells: int = 0
    """Length of a generic port access lane. w:2 r:4,5,6"""

    port_lane_half_width_cells: int = 0
    """Half width of a generic port access lane. w:2 r:3,4,6"""

    stub_port_lane_length_cells: int = 0
    """Length of a dense-fan-out stub port lane. w:2 r:4"""

    stub_port_lane_half_width_cells: int = 0
    """Half width of a dense-fan-out stub port lane. w:2 r:4"""

    primitive_cfg: Any = None
    """The kernel's `PrimitiveLibraryConfig` (PyO3). w:2 r:2,6"""

    astar_cfg: Any = None
    """The kernel's `AStarConfig` (PyO3). w:2 r:2,6"""

    router: Any = None
    """The kernel's `PyPhotonicRouter` (PyO3). w:2 r:2,4,5,6,7,8"""

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

    fanout_stub_static_cells_by_spec: dict[str, set[tuple[int, int]]] = field(
        default_factory=dict
    )
    """Inflated stub cells per port spec. w:3 r:3,5,7"""

    dense_source_port_runway_length_by_spec: dict[str, int] = field(default_factory=dict)
    """Staggered source runway length per dense port spec. w:3 r:3,6"""

    dense_target_port_runway_length_by_spec: dict[str, int] = field(default_factory=dict)
    """Staggered target runway length per dense port spec. w:3 r:3,6"""

    # ---- phase 4, crossing_plan_stage -------------------------------------
    crossing_plan_info: dict[str, Any] = field(default_factory=dict)
    """The crossing plan (typed in Slice 2c). w:4 m:4,7,8,9 r:4,5,7,8,9  # shared writer"""

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

    port_state_lane_offsets: dict[tuple[str, bool], tuple[int, int]] = field(
        default_factory=dict
    )
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

    collect_timing: bool = False
    """Whether per-net timing buckets are filled. w:5 r:5,6,7"""

    track_dynamic_cells: bool = False
    """Whether committed dynamic cells are tracked per net. w:5 r:5,6"""

    route_timing_buckets: dict[str, RouteTimingBucket] = field(default_factory=dict)
    """Per-attempt-kind timing buckets. w:5 m:6 r:6,7,9  # shared writer"""

    route_attempt_records: list[RouteAttemptRecord] = field(default_factory=list)
    """One record per search attempt. w:5 m:6 r:6,9  # shared writer"""

    deferred_count: int = 0
    """Nets deferred by the negotiated loop. w:5,6 r:7,9  # shared writer"""

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
