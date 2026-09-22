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


__all__ = [
    "ALL_NETS",
    "CrossingEngineConfig",
    "KernelDiagnostics",
    "NegotiationConfig",
    "RouterConfig",
    "SearchOverrides",
]
