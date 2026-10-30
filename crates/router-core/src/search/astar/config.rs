//! `AStarConfig` and the small enums/constants that parameterise a search:
//! primitive ordering, heuristic and heap tie-break modes, and the
//! primitive-transition-class and heap-position sentinels used across the
//! kernel. Moved out of `src/astar.rs` (Milestone 3, Slice 2); pure code
//! motion, no behaviour change.

use crate::config::RouterConfig;
use crate::error::RouterError;
use crate::obstacle_map::NetId;
use crate::search::astar::heuristic::TerminalApproach;
use std::sync::atomic::AtomicUsize;

pub(crate) static CROSSING_CANDIDATE_TRACE_COUNT: AtomicUsize = AtomicUsize::new(0);
pub(crate) const NO_PENDING_CROSSING_ANGLE: u8 = 255;
pub(crate) const NO_PENDING_CROSSING_PARTNER_INDEX: u8 = 255;
pub(crate) const SEARCH_TIMEOUT_CHECK_INTERVAL: usize = 4096;
pub(crate) const NO_DYNAMIC_OWNER: NetId = NetId::MAX;

/// Configuration for the first single-net A* router.
#[derive(Clone, Debug)]
pub struct AStarConfig {
    /// Diagnostic/trace switches (`PHOTONIC_ROUTER_*` variables read only
    /// inside this module). Cloned in from the router's `RouterConfig` when
    /// this `AStarConfig` is built.
    pub diagnostics: crate::config::KernelDiagnostics,
    pub max_iterations: usize,
    pub bend_weight: f64,
    pub target_tolerance_cells: i32,
    pub require_target_angle: bool,
    pub allowed_target_angles_mask: Option<u8>,
    pub use_routing_window: bool,
    pub routing_window_min_margin_cells: i32,
    pub routing_window_scale: f64,
    pub routing_window_max_expansions: u32,
    pub routing_window_fallback_full_grid: bool,
    pub routing_window_growth: f64,
    pub max_dense_states: usize,
    pub max_dense_obstacle_cells: usize,
    pub enable_simple_routes: bool,
    pub simple_route_max_offset_cells: i32,
    pub simple_route_min_leg_len_cells: i32,
    pub ignore_dynamic_obstacles: bool,
    pub history_weight: f64,
    pub long_straight_congestion_weight: f64,
    pub proactive_congestion_weight: f64,
    pub proactive_congestion_radius_cells: i32,
    pub collect_detailed_timing: bool,
    pub use_indexed_heap: bool,
    pub primitive_ordering: PrimitiveOrdering,
    pub heuristic_mode: HeuristicMode,
    pub heap_tie_breaker: HeapTieBreaker,
    pub require_terminal_straights: bool,
    pub max_search_time_ms: u64,
    // Caps the TOTAL expanded states of one windowed search call -- every
    // routing-window attempt (`routing_window_max_expansions`) plus the
    // full-grid fallback (`routing_window_fallback_full_grid`) together, not
    // any single attempt's own `max_iterations` -- so a search that cannot
    // find a route stops well short of retrying every window at the full
    // per-window cap. `None` (the default) leaves every window/fallback
    // attempt bounded only by `max_iterations`, as before this field
    // existed. Only `route_many_with_negotiated_repair_and_commit` ever
    // sets this (per search, via `SearchCall::budget`, applied by
    // `Router::astar_config`); every other caller leaves it
    // `None`. See
    // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    // Milestone 5.
    pub total_expansion_budget: Option<u64>,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            diagnostics: crate::config::KernelDiagnostics::default(),
            max_iterations: 100_000,
            bend_weight: 1.0,
            target_tolerance_cells: 0,
            require_target_angle: true,
            allowed_target_angles_mask: None,
            use_routing_window: true,
            routing_window_min_margin_cells: 12,
            routing_window_scale: 0.35,
            routing_window_max_expansions: 3,
            routing_window_fallback_full_grid: false,
            routing_window_growth: 0.5,
            // 100 M states = about 1.7 GB of eagerly allocated per-attempt
            // storage at the largest allowed window (f64 g-cost, u32
            // generation, u32 parent, primitive id, closed bit per state).
            // Was 20 M until 2026-09-16: the 128x128 mesh's heater-to-MMI
            // nets need 22-28 M states for their routing windows and every
            // window failed silently (`dense_storage_cap` diagnostic).
            // `PHOTONIC_ROUTER_MAX_DENSE_STATES` overrides it at router
            // construction (`configured_max_dense_states` in py_router.rs).
            max_dense_states: 100_000_000,
            max_dense_obstacle_cells: 10_000_000,
            enable_simple_routes: true,
            simple_route_max_offset_cells: 96,
            simple_route_min_leg_len_cells: 1,
            ignore_dynamic_obstacles: false,
            history_weight: 0.0,
            long_straight_congestion_weight: 0.0,
            proactive_congestion_weight: 0.0,
            proactive_congestion_radius_cells: 0,
            collect_detailed_timing: false,
            use_indexed_heap: false,
            primitive_ordering: PrimitiveOrdering::Library,
            heuristic_mode: HeuristicMode::HeadingAware,
            heap_tie_breaker: HeapTieBreaker::SmallerG,
            require_terminal_straights: false,
            max_search_time_ms: 0,
            total_expansion_budget: None,
        }
    }
}

/// The A* settings as the caller states them (the fields of the Python
/// `AStarConfig` binding): mode names as strings, the target angles as a
/// list. `AStarConfig::from_parts` validates and resolves them; the router
/// keeps them for the two fields it reads directly
/// (`target_tolerance_cells`, `collect_detailed_timing`). `Default` is the
/// binding's default.
#[derive(Clone, Debug, PartialEq)]
pub struct AStarParams {
    pub max_iterations: usize,
    pub bend_weight: f64,
    pub target_tolerance_cells: i32,
    pub require_target_angle: bool,
    pub allowed_target_angles: Option<Vec<u8>>,
    pub use_routing_window: bool,
    pub routing_window_min_margin_cells: i32,
    pub routing_window_scale: f64,
    pub routing_window_max_expansions: u32,
    pub routing_window_fallback_full_grid: bool,
    pub routing_window_growth: f64,
    pub max_dense_obstacle_cells: usize,
    pub enable_simple_routes: bool,
    pub simple_route_max_offset_cells: i32,
    pub simple_route_min_leg_len_cells: i32,
    pub ignore_dynamic_obstacles: bool,
    pub history_weight: f64,
    pub proactive_congestion_weight: f64,
    pub proactive_congestion_radius_cells: i32,
    pub collect_detailed_timing: bool,
    pub use_indexed_heap: bool,
    pub primitive_ordering: String,
    pub heuristic_mode: String,
    pub heap_tie_breaker: String,
    pub max_search_time_ms: u64,
}

impl Default for AStarParams {
    fn default() -> Self {
        Self {
            max_iterations: 100_000,
            bend_weight: 1.0,
            target_tolerance_cells: 0,
            require_target_angle: true,
            allowed_target_angles: None,
            use_routing_window: true,
            routing_window_min_margin_cells: 12,
            routing_window_scale: 0.35,
            routing_window_max_expansions: 3,
            routing_window_fallback_full_grid: true,
            routing_window_growth: 0.5,
            max_dense_obstacle_cells: 10_000_000,
            enable_simple_routes: true,
            simple_route_max_offset_cells: 96,
            simple_route_min_leg_len_cells: 1,
            ignore_dynamic_obstacles: false,
            history_weight: 0.0,
            proactive_congestion_weight: 0.0,
            proactive_congestion_radius_cells: 0,
            collect_detailed_timing: false,
            use_indexed_heap: false,
            primitive_ordering: "library".to_string(),
            heuristic_mode: "heading_aware".to_string(),
            heap_tie_breaker: "smaller_g".to_string(),
            max_search_time_ms: 0,
        }
    }
}

/// Per-call replacements of four `AStarParams` fields (`None` keeps the
/// stated value; `long_straight_weight_override` replaces the
/// `RouterConfig` weight).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AStarOverrides {
    pub ignore_dynamic_obstacles: Option<bool>,
    pub enable_simple_routes: Option<bool>,
    pub history_weight: Option<f64>,
    pub long_straight_weight_override: Option<f64>,
}

pub(crate) fn allowed_angles_to_mask(angles: Option<&Vec<u8>>) -> Result<Option<u8>, RouterError> {
    let Some(angles) = angles else {
        return Ok(None);
    };
    if angles.is_empty() {
        return Err(RouterError::Config(
            "allowed_target_angles must not be empty when provided".to_string(),
        ));
    }
    let mut mask = 0u8;
    for &angle in angles {
        if angle > 7 {
            return Err(RouterError::Config(
                "allowed_target_angles entries must be in [0, 7]".to_string(),
            ));
        }
        mask |= 1u8 << angle;
    }
    Ok(Some(mask))
}

pub(crate) fn parse_primitive_ordering(value: &str) -> Result<PrimitiveOrdering, RouterError> {
    match value.trim().to_ascii_lowercase().as_str() {
        "library" => Ok(PrimitiveOrdering::Library),
        "long_straight_first" => Ok(PrimitiveOrdering::LongStraightFirst),
        "target_biased" => Ok(PrimitiveOrdering::TargetBiased),
        _ => Err(RouterError::Config(
            "primitive_ordering must be one of 'library', 'long_straight_first', or 'target_biased'"
                .to_string(),
        )),
    }
}

pub(crate) fn parse_heuristic_mode(value: &str) -> Result<HeuristicMode, RouterError> {
    match value.trim().to_ascii_lowercase().as_str() {
        "distance" => Ok(HeuristicMode::Distance),
        "heading_aware" => Ok(HeuristicMode::HeadingAware),
        "diagonal_aware" => Ok(HeuristicMode::DiagonalAware),
        _ => Err(RouterError::Config(
            "heuristic_mode must be one of 'distance', 'heading_aware', or 'diagonal_aware'"
                .to_string(),
        )),
    }
}

pub(crate) fn parse_heap_tie_breaker(value: &str) -> Result<HeapTieBreaker, RouterError> {
    match value.trim().to_ascii_lowercase().as_str() {
        "smaller_g" => Ok(HeapTieBreaker::SmallerG),
        "larger_g" => Ok(HeapTieBreaker::LargerG),
        _ => Err(RouterError::Config(
            "heap_tie_breaker must be one of 'smaller_g' or 'larger_g'".to_string(),
        )),
    }
}

impl AStarConfig {
    /// The search configuration from the stated settings, the per-call
    /// overrides and the router's configuration tree (formerly
    /// `astar_config_from_py`, which read the Python binding directly).
    /// Errors are `RouterError::Config` with the binding's former
    /// `ValueError` messages.
    pub fn from_parts(
        params: &AStarParams,
        overrides: &AStarOverrides,
        router_config: &RouterConfig,
    ) -> Result<Self, RouterError> {
        let allowed_target_angles_mask =
            allowed_angles_to_mask(params.allowed_target_angles.as_ref())?;
        let primitive_ordering = parse_primitive_ordering(&params.primitive_ordering)?;
        let heuristic_mode = parse_heuristic_mode(&params.heuristic_mode)?;
        let heap_tie_breaker = parse_heap_tie_breaker(&params.heap_tie_breaker)?;
        let max_search_time_ms = router_config
            .search
            .astar_timeout_ms
            .unwrap_or(params.max_search_time_ms);
        let long_straight_congestion_weight = match overrides.long_straight_weight_override {
            Some(weight) => weight,
            None => router_config
                .search
                .long_straight_congestion_weight
                .unwrap_or(0.0),
        };
        if !params.proactive_congestion_weight.is_finite()
            || params.proactive_congestion_weight < 0.0
        {
            return Err(RouterError::Config(
                "proactive_congestion_weight must be finite and non-negative".to_string(),
            ));
        }
        if params.proactive_congestion_radius_cells < 0 {
            return Err(RouterError::Config(
                "proactive_congestion_radius_cells must be non-negative".to_string(),
            ));
        }
        Ok(AStarConfig {
            diagnostics: router_config.diagnostics.clone(),
            max_iterations: params.max_iterations,
            bend_weight: params.bend_weight,
            target_tolerance_cells: params.target_tolerance_cells,
            require_target_angle: params.require_target_angle,
            allowed_target_angles_mask,
            use_routing_window: params.use_routing_window,
            routing_window_min_margin_cells: params.routing_window_min_margin_cells,
            routing_window_scale: params.routing_window_scale,
            routing_window_max_expansions: params.routing_window_max_expansions,
            routing_window_fallback_full_grid: params.routing_window_fallback_full_grid,
            routing_window_growth: params.routing_window_growth,
            max_dense_states: router_config
                .search
                .max_dense_states
                .filter(|n| *n > 0)
                .unwrap_or_else(|| AStarConfig::default().max_dense_states),
            max_dense_obstacle_cells: params.max_dense_obstacle_cells,
            enable_simple_routes: overrides
                .enable_simple_routes
                .unwrap_or(params.enable_simple_routes),
            simple_route_max_offset_cells: params.simple_route_max_offset_cells,
            simple_route_min_leg_len_cells: params.simple_route_min_leg_len_cells,
            ignore_dynamic_obstacles: overrides
                .ignore_dynamic_obstacles
                .unwrap_or(params.ignore_dynamic_obstacles),
            history_weight: overrides.history_weight.unwrap_or(params.history_weight),
            long_straight_congestion_weight,
            proactive_congestion_weight: params.proactive_congestion_weight,
            proactive_congestion_radius_cells: params.proactive_congestion_radius_cells,
            collect_detailed_timing: params.collect_detailed_timing,
            use_indexed_heap: params.use_indexed_heap,
            primitive_ordering,
            heuristic_mode,
            heap_tie_breaker,
            require_terminal_straights: false,
            max_search_time_ms,
            // Never set from the stated settings: only
            // `route_many_with_negotiated_repair_and_commit` ever wants a
            // fail-fast budget, and it applies one per call via
            // `SearchCall::budget`/`astar_config`, not through this conversion.
            // See Milestone 5 of
            // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`.
            total_expansion_budget: None,
        })
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum PrimitiveOrdering {
    #[default]
    Library,
    LongStraightFirst,
    TargetBiased,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum HeuristicMode {
    #[default]
    Distance,
    HeadingAware,
    DiagonalAware,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum HeapTieBreaker {
    #[default]
    SmallerG,
    LargerG,
}

pub(crate) const PRIMITIVE_TRANSITION_CLASS_COUNT: usize = 4;
pub(crate) const PRIMITIVE_STRAIGHT_SHORT: usize = 0;
pub(crate) const PRIMITIVE_STRAIGHT_LONG: usize = 1;
pub(crate) const PRIMITIVE_BEND_45: usize = 2;
pub(crate) const PRIMITIVE_BEND_90: usize = 3;

pub(crate) const NO_PARENT: u32 = u32::MAX;
pub(crate) const NO_GENERATION: u32 = u32::MAX;
pub(crate) const BITSET_WORD_BITS: usize = u64::BITS as usize;

pub(crate) const NO_HEAP_POSITION: usize = usize::MAX;

// `DiagonalAware` carries its 64 terminal approaches inline so the mode stays
// `Copy` and is built once per search; the size difference is deliberate.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug)]
pub(crate) enum SearchHeuristicMode {
    Distance,
    HeadingAware {
        minimum_bend_cost: f64,
        tolerance: i32,
        target_angle_ok: [bool; 8],
    },
    DiagonalAware {
        minimum_bend_cost: f64,
        tolerance: i32,
        target_angle_ok: [bool; 8],
        terminal_approaches: [TerminalApproach; 64],
        terminal_approach_count: usize,
    },
}
