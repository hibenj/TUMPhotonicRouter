//! Search state and result types: `State` (grid position plus heading),
//! `RouteResult`, `RouteSearchStats`, and the small helpers that convert
//! between a `State` and a `GridPoint` or look up a primitive by id.
//! Moved out of `src/astar.rs` (Milestone 3, Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`);
//! pure code motion, no behaviour change.

use crate::obstacle_map::NetId;
use crate::primitives::PrimitiveLibrary;
use crate::search::astar::config::PRIMITIVE_TRANSITION_CLASS_COUNT;
use crate::simple_routes::GridPoint;
use rustc_hash::FxHashMap;

/// Router search state: grid position plus 45-degree heading index.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct State {
    pub x: i32,
    pub y: i32,
    pub angle: u8,
}

impl State {
    pub fn new(x: i32, y: i32, angle: u8) -> Self {
        Self {
            x,
            y,
            angle: angle % 8,
        }
    }
}

/// Result of one successful single-net route.
#[derive(Clone, Debug)]
pub struct RouteResult {
    pub states: Vec<State>,
    pub primitives: Vec<u16>,
    pub cells: Vec<(i32, i32)>,
    pub compressed_waypoints: Vec<(i32, i32)>,
    pub total_length_um: f64,
    pub total_cost: f64,
    pub requested_target: State,
    pub reached_target: State,
    pub stats: RouteSearchStats,
}

#[derive(Clone, Debug, Default)]
pub struct RouteSearchStats {
    pub window_attempts: u32,
    pub used_full_grid_fallback: bool,
    pub last_window_min_x: i32,
    pub last_window_max_x: i32,
    pub last_window_min_y: i32,
    pub last_window_max_y: i32,
    pub last_window_area_cells: i64,
    pub expanded_states: usize,
    pub generated_neighbors: usize,
    pub heap_pushes: usize,
    pub heap_pops: usize,
    pub skipped_duplicate_heap_entries: usize,
    pub stale_generation_heap_entries: usize,
    pub closed_heap_entries: usize,
    pub max_heap_size: usize,
    pub dense_search_states: usize,
    pub dense_search_storage_bytes: usize,
    pub best_cost_updates: usize,
    pub parent_updates: usize,
    pub obstacle_clearance_checks: usize,
    pub window_rejects: usize,
    pub footprint_rejects: usize,
    /// Diagonal moves whose compact halo touched a blocked cell while their
    /// own footprint was free (see the Tier-1 gate in the unified kernel).
    pub diagonal_halo_contacts: usize,
    pub primitive_generated_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_bounds_rejects_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_closed_rejects_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_cost_pruned_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_footprint_checks_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_footprint_rejects_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_accepted_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_footprint_checks: usize,
    pub primitive_footprint_cells_tested: usize,
    pub primitive_footprint_rect_checks: usize,
    pub primitive_footprint_rect_rejects: usize,
    pub crossing_hotpath_no_contact: usize,
    pub crossing_hotpath_contact_checks: usize,
    pub crossing_hotpath_static_rejects: usize,
    pub crossing_hotpath_no_owner_contacts: usize,
    pub crossing_hotpath_single_owner_contacts: usize,
    pub crossing_hotpath_multi_owner_contacts: usize,
    pub crossing_hotpath_witness_cells_scanned: usize,
    pub crossing_hotpath_partner_segment_checks: usize,
    pub crossing_hotpath_partner_segment_bbox_rejects: usize,
    pub crossing_hotpath_intersection_hits: usize,
    pub crossing_hotpath_total_time_us: u128,
    pub crossing_hotpath_owner_scan_time_us: u128,
    pub crossing_hotpath_segment_time_us: u128,
    pub crossing_hotpath_reservation_time_us: u128,
    pub crossing_candidate_checks: usize,
    pub crossing_accepted: usize,
    pub crossing_reject_non_straight: usize,
    pub crossing_reject_not_perpendicular: usize,
    pub crossing_reject_margin: usize,
    pub crossing_reject_wrong_order: usize,
    pub crossing_reject_unexpected_owner: usize,
    /// Accepted crossings with a partner carrying a `crossing_loss_override`
    /// (a planned pair in crossing-guided mode).
    pub crossing_accepted_planned: usize,
    /// Accepted crossings with a budgeted planned partner that was already
    /// crossed on the path (priced at the full `crossing_loss`).
    pub crossing_accepted_over_budget: usize,
    pub crossing_reject_unmatched_owner: usize,
    pub crossing_reject_unmatched_centerline: usize,
    pub crossing_reject_unmatched_footprint: usize,
    pub crossing_reject_unmatched_route_centerline: usize,
    pub crossing_reject_unmatched_route_footprint: usize,
    pub crossing_reject_pending_straight: usize,
    /// Crossing rejected because its +-half_size reservation window overlaps
    /// a window of an earlier crossing of the same route (elements would
    /// overlap; the post-search `crossing_events_have_disjoint_reservations`
    /// check would discard the whole route).
    pub crossing_reject_reservation_overlap: usize,
    // Analysis-only partner breakdowns. These are disabled in normal routing
    // unless PHOTONIC_ROUTER_ANALYSIS_CROSSING_PARTNER_COUNTERS=1 is set.
    pub crossing_perpendicular_reject_by_partner: FxHashMap<NetId, usize>,
    pub crossing_after_margin_by_partner: FxHashMap<NetId, usize>,
    // Active repair metric: counts the first partner whose accepted crossing
    // cannot complete the required post-crossing straight.
    pub crossing_pending_straight_by_partner: FxHashMap<NetId, usize>,
    pub dense_grid_build_failures: usize,
    pub max_window_area_cells: i64,
    pub dense_grid_cells: usize,
    pub route_search_total_time_us: u128,
    pub dense_grid_build_time_us: u128,
    pub search_loop_time_us: u128,
    pub obstacle_map_prepare_time_us: u128,
    pub simple_route_time_us: u128,
    pub commit_prepare_time_us: u128,
    pub commit_time_us: u128,
    pub neighbor_generation_time_us: u128,
    pub heap_operation_time_us: u128,
    pub legality_check_time_us: u128,
    pub reconstruction_time_us: u128,
    pub jps4_requested: bool,
    pub jps4_eligible: bool,
    pub jps4_used: bool,
    pub jps4_fallbacks: usize,
    pub jps4_fallback_reason: String,
}

#[inline]
pub(crate) fn grid_point_from_state(state: State) -> GridPoint {
    GridPoint::new(state.x, state.y)
}

pub(crate) fn find_primitive(
    primitives: &PrimitiveLibrary,
    start_angle: u8,
    primitive_id: u16,
) -> Option<&crate::primitives::Primitive> {
    primitives
        .get_primitives_for_angle(start_angle)
        .iter()
        .find(|p| p.id == primitive_id)
}
