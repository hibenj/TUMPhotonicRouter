"""Typed configuration mirroring the Rust `RouterConfig` (`src/config.rs`).

Milestone 1 of `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`:
every `PHOTONIC_ROUTER_*` environment variable the Rust kernel used to read
directly becomes a typed field here, passed explicitly into
`rust_backend.PyPhotonicRouter`. These frozen dataclasses mirror the Rust
structs field-for-field and default-for-default; `RouterConfig.to_rust`
builds the flat-keyword `rust_backend.RouterConfig` the binding expects, and
`RouterConfig.from_environment` (implemented in `env_overlay.py`, imported
lazily here to avoid a circular import) applies the environment overlay for
callers that pass no configuration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

#: `NetNameTrace` (Rust `crate::config::NetNameTrace`), represented without a
#: dedicated class: `None` = no trace, the literal string `"*"` = every net,
#: a tuple of net-id strings = exactly those nets.
NetNameTrace = "None | tuple[str, ...] | str"

ALL_NETS: str = "*"


@dataclass(frozen=True)
class NegotiationConfig:
    """Mirrors Rust `NegotiationConfig`."""

    budget_first: int = 2_000_000
    budget_first_retry: int = 10_000_000
    budget_retry: int = 30_000_000
    braid_escalation: bool = False
    crossing_free_unplanned: bool = True
    disable_braid_repair: bool = False
    pending_straight_ripup_threshold: int = 100
    enable_orthogonal_repair_fallback: bool = False


@dataclass(frozen=True)
class SearchOverrides:
    """Mirrors Rust `SearchOverrides`. `None` means "no override" for every
    field, not any particular value."""

    astar_timeout_ms: int | None = None
    max_dense_states: int | None = None
    long_straight_congestion_weight: float | None = None


@dataclass(frozen=True)
class CrossingEngineConfig:
    """Mirrors Rust `CrossingEngineConfig`."""

    enable_guided_collision_crossing: bool = False
    disable_guided_collision_crossing: bool = False
    disable_rust_crossing_validation: bool = False


@dataclass(frozen=True)
class KernelDiagnostics:
    """Mirrors Rust `KernelDiagnostics`. Every field is off/unset by
    default, except `trace_crossing_candidate_max` (120, matching the
    kernel's own fallback)."""

    native_progress: bool = False
    native_repair_diag: bool = False
    search_failure_diag: bool = False
    search_failure_map: tuple[int, ...] | None = None
    chain_diag: bool = False
    hot_loop_timing: bool = False
    move_diag: bool = False
    move_diag_cell: tuple[int, int] | None = None
    pop_diag_below_y: int | None = None
    probe_cells: tuple[tuple[int, int], ...] = ()
    trace_crossing: bool = False
    trace_crossing_net: int | None = None
    trace_crossing_candidates: bool = False
    trace_crossing_candidate_max: int = 120
    trace_crossing_level1: bool = False
    trace_crossing_pending: bool = False
    trace_crossing_pending_threshold: int | None = None
    trace_crossing_perp_reject_threshold: int | None = None
    trace_partner_net: int | None = None
    trace_plain_route_net: int | None = None
    # `None` (no trace), `config.ALL_NETS` ("*", every net), or a tuple of
    # net-id strings -- see `NetNameTrace` above.
    trace_endpoint_bump_nets: None | tuple[str, ...] | str = None
    trace_endpoint_correction_net: int | None = None
    crossing_mismatch_dump: bool = False
    crossing_mismatch_dump_net: int | None = None
    crossing_mismatch_fatal: bool = False
    analysis_crossing_partner_counters: bool = False


@dataclass(frozen=True)
class CrossingPlanConfig:
    """Mirrors the env reads of `translation/route_rust_crossing_plan.py`.
    Every field is `None` when unset, in which case the site keeps its own
    literal default (named below)."""

    #: `PHOTONIC_ROUTER_COLLISION_CROSSING_SEARCH_LOSS_UM`: float, finite and
    #: non-negative. `None` = unset (site default
    #: `DEFAULT_COLLISION_CROSSING_SEARCH_LOSS_UM` = 200.0).
    collision_crossing_search_loss_um: float | None = None
    #: `PHOTONIC_ROUTER_PLANNED_CROSSING_SEARCH_LOSS_UM`: float, finite and
    #: non-negative. `None` = unset (site default
    #: `DEFAULT_PLANNED_CROSSING_SEARCH_LOSS_UM` = 0.0).
    planned_crossing_search_loss_um: float | None = None
    #: `PHOTONIC_ROUTER_PLANNED_CROSSING_BUDGET`: `"0"` -> False, `"1"` ->
    #: True, anything else raises. `None` = unset (site default
    #: `DEFAULT_SINGLE_DISCOUNTED_CROSSING_PER_PAIR` = False).
    planned_crossing_budget: bool | None = None


@dataclass(frozen=True)
class CrossingGridConfig:
    """Mirrors the env reads of `translation/preplaced_crossing_grids.py`'s
    `crossing_grid_geometry_from_config`/`_column_grid_stage` and
    `translation/crossing_structures.py`'s `select_layer_structure`. Every
    float/string field is `None` when unset, in which case the site's own
    (`CrossingGridGeometry` or literal) default applies."""

    #: `PHOTONIC_ROUTER_CROSSING_GRID_BAND_MARGIN_UM` (default 50.0).
    band_margin_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_BEND_RADIUS_UM` (default 5.0).
    bend_radius_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_LEAD_UM` (default 14.0).
    column_lead_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_COLUMN_PITCH_UM` (default 16.0).
    column_pitch_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_CORNER_MARGIN_UM` (default 14.0).
    corner_margin_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_ENTRY_STRAIGHT_UM` (default 4.0).
    entry_straight_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_FAN_COLUMN_PITCH_UM` (default 4.0).
    fan_column_pitch_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_LANE_PITCH_UM` (default 14.0).
    lane_pitch_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_PORT_PAIR_SPREAD_UM` (default 2.0).
    port_pair_spread_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_SLOT_SPREAD_UM` (default 11.0).
    slot_spread_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_STUB_STAGGER_UM` (default 6.0).
    stub_stagger_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_TILE_MIN_SPACING_UM` (default 14.0).
    tile_min_spacing_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_UNROUTED_SIBLING_CLEARANCE_UM` (default 0.0).
    unrouted_sibling_clearance_um: float | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_FAN_MODE`: string (site default
    #: `base.fan_mode`, "tiles"); a blank value is also "unset".
    fan_mode: str | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_TILE_PLACEMENT`: string (site default
    #: "auto"); a blank value is also "unset".
    tile_placement: str | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_CORNERS`: string (site default
    #: "always"); a blank value is also "unset".
    corners: str | None = None
    #: `PHOTONIC_ROUTER_CROSSING_GRID_ROUTER_LAYERS`: comma-separated ints
    #: (non-digit tokens dropped); default empty.
    router_layers: frozenset[int] = frozenset()
    #: `PHOTONIC_ROUTER_TRACE_COLUMN_GRID`: truthy string.
    trace_column_grid: bool = False


@dataclass(frozen=True)
class FanoutAccessConfig:
    """Mirrors the fan-out-related env reads of `translation/route_rust.py`."""

    #: `PHOTONIC_ROUTER_DENSE_FANOUT_INSTANCES`: comma-separated instance
    #: names. Unset (`None`) means no override. Also SELF-SET: the
    #: pre-placed crossing-grid stage computes this set and
    #: `run_routing_flow` folds it into the routing-stage config (see
    #: `translation/preplaced_crossing_grids.py`'s `_derive_crossing_tiles`).
    dense_fanout_instances: frozenset[str] | None = None
    #: `PHOTONIC_ROUTER_DENSE_FANOUT_MIN_PORTS`: int >= 2, else raises.
    #: `None` = unset (site default 3).
    dense_fanout_min_ports: int | None = None
    #: `PHOTONIC_ROUTER_FANOUT_ACCESS_MODE`: today this OVERRIDES the
    #: constructor argument (`os.environ.get(NAME, default_or_ctor_arg)`,
    #: then alias-normalized). `None` = unset -> the constructor
    #: argument/default is used.
    fanout_access_mode: str | None = None
    #: `PHOTONIC_ROUTER_FANOUT_LANE_SPACING_CELLS`: non-negative int, else
    #: raises. `None` = unset (site defaults: 11 at
    #: `_build_static_fanout_anchors`, 3 at the three other sites).
    fanout_lane_spacing_cells: int | None = None
    #: `PHOTONIC_ROUTER_FANOUT_PROTECTED_LANE_SPACING_CELLS`: non-negative
    #: int, else raises. `None` = unset -> falls back to
    #: `fanout_lane_spacing_cells`, then 3.
    fanout_protected_lane_spacing_cells: int | None = None
    #: `PHOTONIC_ROUTER_TARGET_PROTECTED_LANE_SPACING_CELLS`: non-negative
    #: int, else raises. `None` = unset -> outermost of that fallback chain.
    target_protected_lane_spacing_cells: int | None = None
    #: `PHOTONIC_ROUTER_FANOUT_STUB_BEND_DEGREES`: alias table (see
    #: `_fanout_stub_bend_steps`), site default `"90"`. `None` = unset.
    fanout_stub_bend_degrees: str | None = None
    #: `PHOTONIC_ROUTER_FANOUT_STUB_FORWARD_CELLS`: non-negative int, else
    #: raises. `None` = unset (site computes `max(3, bend_radius_cells + 3)`).
    fanout_stub_forward_cells: int | None = None
    #: `PHOTONIC_ROUTER_FANOUT_STUB_X_OFFSET_CELLS`: non-negative int, else
    #: raises. `None` = unset (site default 1).
    fanout_stub_x_offset_cells: int | None = None
    #: `PHOTONIC_ROUTER_STUB_PORT_LANE_HALF_WIDTH_CELLS`: non-negative int,
    #: else raises. `None` = unset (site default 0).
    stub_port_lane_half_width_cells: int | None = None
    #: `PHOTONIC_ROUTER_STUB_PORT_LANE_LENGTH_CELLS`: non-negative int, else
    #: raises. `None` = unset (site default 0).
    stub_port_lane_length_cells: int | None = None


@dataclass(frozen=True)
class EngineSelection:
    """Mirrors the repair-engine selection env reads of
    `translation/route_rust.py`'s `negotiated_repair_engine_enabled`."""

    #: `PHOTONIC_ROUTER_NEGOTIATED_REPAIR`: `!= "0"`, default True.
    negotiated_repair: bool = True
    #: `PHOTONIC_ROUTER_LEGACY_REPAIR_CHAIN`: `== "1"`, default False.
    #: When True it forces the legacy chain regardless of
    #: `negotiated_repair`.
    legacy_repair_chain: bool = False


@dataclass(frozen=True)
class SearchTuning:
    """Mirrors the search-tuning env reads of `translation/route_rust.py`."""

    #: `PHOTONIC_ROUTER_MIN_BEND_WEIGHT`: float, default 12.0.
    min_bend_weight: float = 12.0
    #: `PHOTONIC_ROUTER_MIN_HEURISTIC_WEIGHT`: float, default 1.0.
    min_heuristic_weight: float = 1.0
    #: `PHOTONIC_ROUTER_HEAP_TIE_BREAKER`: only exactly `"smaller_g"` or
    #: `"larger_g"` override; any other value (including unset) means
    #: `None`, and the site computes its own default.
    heap_tie_breaker: str | None = None
    #: `PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT`: `== "1"`.
    #: `None` = unset -- SELF-SET by `routing_flow.py` to True when
    #: pre-placed crossing grids are enabled and this field is still
    #: `None`; the site tests `is True`.
    long_straight_exempt_dense_fanout: bool | None = None


@dataclass(frozen=True)
class FlowDiagnostics:
    """Mirrors the Python-side diagnostic env reads of
    `translation/route_rust.py` and `translation/route_rust_endpoint_correction.py`."""

    #: `PHOTONIC_ROUTER_DEBUG_EXECUTION_LIMIT`: int >= 1, else raises.
    #: `None` = unset.
    debug_execution_limit: int | None = None
    #: `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_INSTANCE`: string, default `""`.
    debug_route_first_instance: str = ""
    #: `PHOTONIC_ROUTER_DEBUG_ROUTE_FIRST_NETS`: comma-separated ints, list
    #: order kept (non-digit tokens dropped); default empty.
    debug_route_first_nets: tuple[int, ...] = ()
    #: `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NETS`: comma-separated set
    #: of net names; default empty.
    trace_endpoint_correction_nets: frozenset[str] = frozenset()
    #: `PHOTONIC_ROUTER_TRACE_FANOUT_STUBS`: non-empty string (after
    #: `.strip()`) is truthy.
    trace_fanout_stubs: bool = False
    #: `PHOTONIC_ROUTER_TRACE_GRID`: truthy string.
    trace_grid: bool = False
    #: `PHOTONIC_ROUTER_TRACE_RUNWAY_INSTANCE`: string; truthy then
    #: re-read. `None` = unset (a blank value is also "unset").
    trace_runway_instance: str | None = None
    #: `PHOTONIC_ROUTER_TRACE_TERMINAL_BUMP_DISTANCE_CHECKS`: comma set of
    #: tokens, `"*"` matches every net; default empty.
    trace_terminal_bump_distance_checks: frozenset[str] = frozenset()


@dataclass(frozen=True)
class RouterConfig:
    """Mirrors Rust `RouterConfig`: the top-level configuration tree passed
    explicitly into `rust_backend.PyPhotonicRouter`, replacing every
    `PHOTONIC_ROUTER_*` variable the kernel used to read directly."""

    negotiation: NegotiationConfig = field(default_factory=NegotiationConfig)
    search: SearchOverrides = field(default_factory=SearchOverrides)
    crossing: CrossingEngineConfig = field(default_factory=CrossingEngineConfig)
    diagnostics: KernelDiagnostics = field(default_factory=KernelDiagnostics)

    def to_rust(self, rust_backend: Any) -> Any:
        """Build `rust_backend.RouterConfig(**flat_kwargs)` -- the flat,
        group-prefixed keyword constructor `src/py_router.rs`'s
        `PyRouterConfig` exposes."""
        n = self.negotiation
        s = self.search
        c = self.crossing
        d = self.diagnostics
        if d.trace_endpoint_bump_nets == ALL_NETS:
            bump_nets: list[str] | None = None
            bump_all = True
        elif d.trace_endpoint_bump_nets is None:
            bump_nets = None
            bump_all = False
        else:
            bump_nets = list(d.trace_endpoint_bump_nets)
            bump_all = False
        return rust_backend.RouterConfig(
            negotiation_budget_first=n.budget_first,
            negotiation_budget_first_retry=n.budget_first_retry,
            negotiation_budget_retry=n.budget_retry,
            negotiation_braid_escalation=n.braid_escalation,
            negotiation_crossing_free_unplanned=n.crossing_free_unplanned,
            negotiation_disable_braid_repair=n.disable_braid_repair,
            negotiation_pending_straight_ripup_threshold=n.pending_straight_ripup_threshold,
            negotiation_enable_orthogonal_repair_fallback=n.enable_orthogonal_repair_fallback,
            search_astar_timeout_ms=s.astar_timeout_ms,
            search_max_dense_states=s.max_dense_states,
            search_long_straight_congestion_weight=s.long_straight_congestion_weight,
            crossing_enable_guided_collision_crossing=c.enable_guided_collision_crossing,
            crossing_disable_guided_collision_crossing=c.disable_guided_collision_crossing,
            crossing_disable_rust_crossing_validation=c.disable_rust_crossing_validation,
            diag_native_progress=d.native_progress,
            diag_native_repair_diag=d.native_repair_diag,
            diag_search_failure_diag=d.search_failure_diag,
            diag_search_failure_map=(
                list(d.search_failure_map) if d.search_failure_map is not None else None
            ),
            diag_chain_diag=d.chain_diag,
            diag_hot_loop_timing=d.hot_loop_timing,
            diag_move_diag=d.move_diag,
            diag_move_diag_cell=d.move_diag_cell,
            diag_pop_diag_below_y=d.pop_diag_below_y,
            diag_probe_cells=list(d.probe_cells),
            diag_trace_crossing=d.trace_crossing,
            diag_trace_crossing_net=d.trace_crossing_net,
            diag_trace_crossing_candidates=d.trace_crossing_candidates,
            diag_trace_crossing_candidate_max=d.trace_crossing_candidate_max,
            diag_trace_crossing_level1=d.trace_crossing_level1,
            diag_trace_crossing_pending=d.trace_crossing_pending,
            diag_trace_crossing_pending_threshold=d.trace_crossing_pending_threshold,
            diag_trace_crossing_perp_reject_threshold=d.trace_crossing_perp_reject_threshold,
            diag_trace_partner_net=d.trace_partner_net,
            diag_trace_plain_route_net=d.trace_plain_route_net,
            diag_trace_endpoint_bump_nets=bump_nets,
            diag_trace_endpoint_bump_all_nets=bump_all,
            diag_trace_endpoint_correction_net=d.trace_endpoint_correction_net,
            diag_crossing_mismatch_dump=d.crossing_mismatch_dump,
            diag_crossing_mismatch_dump_net=d.crossing_mismatch_dump_net,
            diag_crossing_mismatch_fatal=d.crossing_mismatch_fatal,
            diag_analysis_crossing_partner_counters=d.analysis_crossing_partner_counters,
        )

    @classmethod
    def from_environment(cls, environ: Mapping[str, str] | None = None) -> "RouterConfig":
        """`apply_env_overlay(RouterConfig(), environ or os.environ)` --
        imported lazily so `config.py` never depends on `env_overlay.py` at
        module-import time (the overlay module imports the dataclasses from
        here)."""
        import os

        from photonic_router.env_overlay import apply_env_overlay

        return apply_env_overlay(cls(), environ if environ is not None else os.environ)


@dataclass(frozen=True)
class RoutingConfig:
    """Milestone 1, Slice 2: the top-level Python-side configuration tree.
    Every `PHOTONIC_ROUTER_*` variable read by `translation/` and
    `routing_flow*.py` (outside `env_overlay.py` itself) is a typed field
    somewhere under this tree, threaded explicitly from `run_routing_flow`
    down to `_RouteNetsRustSession` (whose `.router` field is `RouterConfig`,
    the Rust-boundary configuration Milestone 1 Slice 1 introduced)."""

    router: RouterConfig = field(default_factory=RouterConfig)
    crossing_plan: CrossingPlanConfig = field(default_factory=CrossingPlanConfig)
    crossing_grid: CrossingGridConfig = field(default_factory=CrossingGridConfig)
    fanout: FanoutAccessConfig = field(default_factory=FanoutAccessConfig)
    engine: EngineSelection = field(default_factory=EngineSelection)
    search: SearchTuning = field(default_factory=SearchTuning)
    diagnostics: FlowDiagnostics = field(default_factory=FlowDiagnostics)
    #: `PHOTONIC_ROUTER_WRITE_GDS_ON_PHOTONIC_VERIFICATION_FAILURE`
    #: (`routing_flow_verification.py`): `.strip().lower() in
    #: {"1", "true", "yes", "on"}`, default False.
    write_gds_on_photonic_verification_failure: bool = False

    @classmethod
    def from_environment(cls, environ: Mapping[str, str] | None = None) -> "RoutingConfig":
        """`apply_env_overlay(RoutingConfig(), environ or os.environ)` --
        imported lazily for the same reason as `RouterConfig.from_environment`."""
        import os

        from photonic_router.env_overlay import apply_env_overlay

        return apply_env_overlay(cls(), environ if environ is not None else os.environ)


__all__ = [
    "ALL_NETS",
    "CrossingEngineConfig",
    "CrossingGridConfig",
    "CrossingPlanConfig",
    "EngineSelection",
    "FanoutAccessConfig",
    "FlowDiagnostics",
    "KernelDiagnostics",
    "NegotiationConfig",
    "RouterConfig",
    "RoutingConfig",
    "SearchOverrides",
    "SearchTuning",
]
