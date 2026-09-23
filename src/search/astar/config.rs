//! `AStarConfig` and the small enums/constants that parameterise a search:
//! primitive ordering, heuristic and heap tie-break modes, and the
//! primitive-transition-class and heap-position sentinels used across the
//! kernel. Moved out of `src/astar.rs` (Milestone 3, Slice 2); pure code
//! motion, no behaviour change.

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
    pub enable_jps4: bool,
    pub use_indexed_heap: bool,
    pub primitive_ordering: PrimitiveOrdering,
    pub heuristic_mode: HeuristicMode,
    pub heuristic_weight: f64,
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
    // existed. Only `route_many_with_negotiated_repair_and_commit` in
    // `py_router.rs` ever sets this (via its own `negotiated_search_budget`
    // field, read by `PyPhotonicRouter::astar_config`); every other caller,
    // including the older repair chain, leaves it `None`. See
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
            enable_jps4: false,
            use_indexed_heap: false,
            primitive_ordering: PrimitiveOrdering::Library,
            heuristic_mode: HeuristicMode::HeadingAware,
            heuristic_weight: 1.0,
            heap_tie_breaker: HeapTieBreaker::SmallerG,
            require_terminal_straights: false,
            max_search_time_ms: 0,
            total_expansion_budget: None,
        }
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

pub(crate) const JPS4_DIRECTIONS: [(i32, i32); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];

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
