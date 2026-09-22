//! Typed configuration for the router, replacing `PHOTONIC_ROUTER_*`
//! environment variables read directly inside the algorithms.
//!
//! Milestone 1 of `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`:
//! every environment variable read in `src/astar.rs` and `src/py_router.rs`
//! becomes a typed field here. Parsing of the `PHOTONIC_ROUTER_*` strings
//! moves to Python (`photonic_router/env_overlay.py`); these structs are
//! plain data with exactly today's defaults, one field per variable, no
//! merging and no default changes.

/// Negotiated-repair engine parameters
/// (`PHOTONIC_ROUTER_NEGOTIATED_*`, `PHOTONIC_ROUTER_DISABLE_BRAID_REPAIR`,
/// `PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD`,
/// `PHOTONIC_ROUTER_ENABLE_ORTHOGONAL_REPAIR_FALLBACK`), read in
/// `src/py_router.rs`.
#[derive(Clone, Debug, PartialEq)]
pub struct NegotiationConfig {
    /// `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST`: expansion budget for the
    /// first negotiated attempt of a net.
    pub budget_first: u64,
    /// `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_FIRST_RETRY`: expansion budget for
    /// the first retry.
    pub budget_first_retry: u64,
    /// `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_RETRY`: expansion budget for every
    /// retry after the first.
    pub budget_retry: u64,
    /// `PHOTONIC_ROUTER_NEGOTIATED_BRAID_ESCALATION`: `== "1"`.
    pub braid_escalation: bool,
    /// `PHOTONIC_ROUTER_NEGOTIATED_CROSSING_FREE_UNPLANNED`: `!= "0"`.
    pub crossing_free_unplanned: bool,
    /// `PHOTONIC_ROUTER_DISABLE_BRAID_REPAIR`: presence.
    pub disable_braid_repair: bool,
    /// `PHOTONIC_ROUTER_PENDING_STRAIGHT_RIPUP_THRESHOLD`: parsed `usize`.
    pub pending_straight_ripup_threshold: usize,
    /// `PHOTONIC_ROUTER_ENABLE_ORTHOGONAL_REPAIR_FALLBACK`: presence.
    pub enable_orthogonal_repair_fallback: bool,
}

impl Default for NegotiationConfig {
    fn default() -> Self {
        Self {
            budget_first: 2_000_000,
            budget_first_retry: 10_000_000,
            budget_retry: 30_000_000,
            braid_escalation: false,
            crossing_free_unplanned: true,
            disable_braid_repair: false,
            pending_straight_ripup_threshold: 100,
            enable_orthogonal_repair_fallback: false,
        }
    }
}

/// Search-kernel overrides
/// (`PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS`/`_S`, `PHOTONIC_ROUTER_MAX_DENSE_STATES`,
/// `PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT`), read in
/// `src/py_router.rs`. All three are unset-sensitive: `None` means "no
/// override", not any particular value.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SearchOverrides {
    /// Timeout in milliseconds, already resolved from either
    /// `PHOTONIC_ROUTER_ASTAR_TIMEOUT_MS` or
    /// `PHOTONIC_ROUTER_ASTAR_TIMEOUT_S` (ms takes precedence; `_S` is
    /// converted to ms, rounded up). `None` = no timeout.
    pub astar_timeout_ms: Option<u64>,
    /// `PHOTONIC_ROUTER_MAX_DENSE_STATES`: `None` = the kernel's own
    /// `AStarConfig::default().max_dense_states`.
    pub max_dense_states: Option<usize>,
    /// `PHOTONIC_ROUTER_LONG_STRAIGHT_CONGESTION_WEIGHT`: `None` = no
    /// penalty.
    pub long_straight_congestion_weight: Option<f64>,
}

/// Crossing-engine selection
/// (`PHOTONIC_ROUTER_ENABLE_GUIDED_COLLISION_CROSSING`,
/// `PHOTONIC_ROUTER_DISABLE_GUIDED_COLLISION_CROSSING`,
/// `PHOTONIC_ROUTER_DISABLE_RUST_CROSSING_VALIDATION`), read in
/// `src/py_router.rs`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CrossingEngineConfig {
    pub enable_guided_collision_crossing: bool,
    pub disable_guided_collision_crossing: bool,
    pub disable_rust_crossing_validation: bool,
}

/// Endpoint-bump net-name trace selector
/// (`PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS`): unset, `*` (all nets), or a
/// comma-separated list of net names.
#[derive(Clone, Debug, Default, PartialEq)]
pub enum NetNameTrace {
    #[default]
    None,
    All,
    Names(Vec<String>),
}

/// Every diagnostic/trace `PHOTONIC_ROUTER_*` variable read in
/// `src/astar.rs` and `src/py_router.rs`. All are off/unset by default,
/// except `trace_crossing_candidate_max` (default 120, matching today's
/// `unwrap_or(120)`).
#[derive(Clone, Debug, PartialEq)]
pub struct KernelDiagnostics {
    /// `PHOTONIC_ROUTER_NATIVE_PROGRESS`: presence.
    pub native_progress: bool,
    /// `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG`: presence.
    pub native_repair_diag: bool,
    /// `PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG`: presence.
    pub search_failure_diag: bool,
    /// `PHOTONIC_ROUTER_SEARCH_FAILURE_MAP`: comma-separated `>=4` ints
    /// (`cx,cy,half_w,half_h[,step]`); `None` = unset (falls back to the
    /// bounding-box default at the print site).
    pub search_failure_map: Option<Vec<i32>>,
    /// `PHOTONIC_ROUTER_CHAIN_DIAG`: presence.
    pub chain_diag: bool,
    /// `PHOTONIC_ROUTER_HOT_LOOP_TIMING`: presence.
    pub hot_loop_timing: bool,
    /// `PHOTONIC_ROUTER_MOVE_DIAG`: presence (the boolean-gated call sites).
    pub move_diag: bool,
    /// `PHOTONIC_ROUTER_MOVE_DIAG`: parsed as `"x,y"` (the cell-gated call
    /// site); `None` if unset or unparsable.
    pub move_diag_cell: Option<(i32, i32)>,
    /// `PHOTONIC_ROUTER_POP_DIAG_BELOW_Y`: parsed `i32`.
    pub pop_diag_below_y: Option<i32>,
    /// `PHOTONIC_ROUTER_PROBE_CELLS`: semicolon-separated `"x,y"` cells.
    pub probe_cells: Vec<(i32, i32)>,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING`: presence (OR'd with
    /// `trace_crossing_net` matching the current net at each call site).
    pub trace_crossing: bool,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING_NET`: parsed `u64`.
    pub trace_crossing_net: Option<u64>,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING_CANDIDATES`: presence.
    pub trace_crossing_candidates: bool,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING_CANDIDATE_MAX`: parsed `usize`,
    /// default 120.
    pub trace_crossing_candidate_max: usize,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING_LEVEL1`: presence.
    pub trace_crossing_level1: bool,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING_PENDING`: presence.
    pub trace_crossing_pending: bool,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING_PENDING_THRESHOLD`: parsed `usize`.
    pub trace_crossing_pending_threshold: Option<usize>,
    /// `PHOTONIC_ROUTER_TRACE_CROSSING_PERP_REJECT_THRESHOLD`: parsed
    /// `usize`.
    pub trace_crossing_perp_reject_threshold: Option<usize>,
    /// `PHOTONIC_ROUTER_TRACE_PARTNER_NET`: parsed `u64`.
    pub trace_partner_net: Option<u64>,
    /// `PHOTONIC_ROUTER_TRACE_PLAIN_ROUTE_NET`: trimmed, parsed `u64`.
    pub trace_plain_route_net: Option<u64>,
    /// `PHOTONIC_ROUTER_TRACE_ENDPOINT_BUMP_NETS`: comma-separated net names
    /// or `"*"` for all.
    pub trace_endpoint_bump_nets: NetNameTrace,
    /// `PHOTONIC_ROUTER_TRACE_ENDPOINT_CORRECTION_NET`: parsed `u64`.
    pub trace_endpoint_correction_net: Option<u64>,
    /// `PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP`: presence.
    pub crossing_mismatch_dump: bool,
    /// `PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP`: its optional value, parsed
    /// as a `u64` net filter.
    pub crossing_mismatch_dump_net: Option<u64>,
    /// `PHOTONIC_ROUTER_CROSSING_MISMATCH_FATAL`: presence.
    pub crossing_mismatch_fatal: bool,
    /// `PHOTONIC_ROUTER_ANALYSIS_CROSSING_PARTNER_COUNTERS`: lowercased
    /// value in `{1, true, on, yes, enabled}`.
    pub analysis_crossing_partner_counters: bool,
}

impl Default for KernelDiagnostics {
    fn default() -> Self {
        Self {
            native_progress: false,
            native_repair_diag: false,
            search_failure_diag: false,
            search_failure_map: None,
            chain_diag: false,
            hot_loop_timing: false,
            move_diag: false,
            move_diag_cell: None,
            pop_diag_below_y: None,
            probe_cells: Vec::new(),
            trace_crossing: false,
            trace_crossing_net: None,
            trace_crossing_candidates: false,
            trace_crossing_candidate_max: 120,
            trace_crossing_level1: false,
            trace_crossing_pending: false,
            trace_crossing_pending_threshold: None,
            trace_crossing_perp_reject_threshold: None,
            trace_partner_net: None,
            trace_plain_route_net: None,
            trace_endpoint_bump_nets: NetNameTrace::None,
            trace_endpoint_correction_net: None,
            crossing_mismatch_dump: false,
            crossing_mismatch_dump_net: None,
            crossing_mismatch_fatal: false,
            analysis_crossing_partner_counters: false,
        }
    }
}

/// Top-level configuration tree passed explicitly from Python, replacing
/// every `PHOTONIC_ROUTER_*` environment variable read inside
/// `src/astar.rs` and `src/py_router.rs`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RouterConfig {
    pub negotiation: NegotiationConfig,
    pub search: SearchOverrides,
    pub crossing: CrossingEngineConfig,
    pub diagnostics: KernelDiagnostics,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn negotiation_defaults_match_todays_constants() {
        let cfg = NegotiationConfig::default();
        assert_eq!(cfg.budget_first, 2_000_000);
        assert_eq!(cfg.budget_first_retry, 10_000_000);
        assert_eq!(cfg.budget_retry, 30_000_000);
        assert!(!cfg.braid_escalation);
        assert!(cfg.crossing_free_unplanned);
        assert!(!cfg.disable_braid_repair);
        assert_eq!(cfg.pending_straight_ripup_threshold, 100);
        assert!(!cfg.enable_orthogonal_repair_fallback);
    }

    #[test]
    fn search_overrides_default_to_no_override() {
        let cfg = SearchOverrides::default();
        assert_eq!(cfg.astar_timeout_ms, None);
        assert_eq!(cfg.max_dense_states, None);
        assert_eq!(cfg.long_straight_congestion_weight, None);
    }

    #[test]
    fn crossing_engine_config_defaults_to_all_off() {
        let cfg = CrossingEngineConfig::default();
        assert!(!cfg.enable_guided_collision_crossing);
        assert!(!cfg.disable_guided_collision_crossing);
        assert!(!cfg.disable_rust_crossing_validation);
    }

    #[test]
    fn kernel_diagnostics_default_matches_todays_defaults() {
        let cfg = KernelDiagnostics::default();
        assert!(!cfg.native_progress);
        assert!(!cfg.native_repair_diag);
        assert!(!cfg.search_failure_diag);
        assert_eq!(cfg.search_failure_map, None);
        assert!(!cfg.chain_diag);
        assert!(!cfg.hot_loop_timing);
        assert!(!cfg.move_diag);
        assert_eq!(cfg.move_diag_cell, None);
        assert_eq!(cfg.pop_diag_below_y, None);
        assert!(cfg.probe_cells.is_empty());
        assert!(!cfg.trace_crossing);
        assert_eq!(cfg.trace_crossing_net, None);
        assert!(!cfg.trace_crossing_candidates);
        assert_eq!(cfg.trace_crossing_candidate_max, 120);
        assert!(!cfg.trace_crossing_level1);
        assert!(!cfg.trace_crossing_pending);
        assert_eq!(cfg.trace_crossing_pending_threshold, None);
        assert_eq!(cfg.trace_crossing_perp_reject_threshold, None);
        assert_eq!(cfg.trace_partner_net, None);
        assert_eq!(cfg.trace_plain_route_net, None);
        assert_eq!(cfg.trace_endpoint_bump_nets, NetNameTrace::None);
        assert_eq!(cfg.trace_endpoint_correction_net, None);
        assert!(!cfg.crossing_mismatch_dump);
        assert_eq!(cfg.crossing_mismatch_dump_net, None);
        assert!(!cfg.crossing_mismatch_fatal);
        assert!(!cfg.analysis_crossing_partner_counters);
    }

    #[test]
    fn router_config_default_composes_group_defaults() {
        let cfg = RouterConfig::default();
        assert_eq!(cfg.negotiation, NegotiationConfig::default());
        assert_eq!(cfg.search, SearchOverrides::default());
        assert_eq!(cfg.crossing, CrossingEngineConfig::default());
        assert_eq!(cfg.diagnostics, KernelDiagnostics::default());
    }
}
