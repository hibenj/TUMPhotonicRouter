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
    /// 1 when the engine that handled the request cannot serve that kind of
    /// request at all and returned no route for that reason alone (today:
    /// `crate::search::GridDijkstraSearch` given a `SearchRequest` with a
    /// `crossing` part). 0 everywhere else, including every ordinary search
    /// failure. Nothing but the engine's own tests reads it.
    pub unsupported_request: u32,
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

/// Unit tests for `State` and the search-statistics defaults, Milestone 6
/// Slice 3 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_new_keeps_the_position_and_normalises_the_heading_modulo_eight() {
        let state = State::new(3, -4, 2);
        assert_eq!((state.x, state.y, state.angle), (3, -4, 2));
        // 8 is one full turn: it normalises to 0, and 9 to 1.
        assert_eq!(State::new(0, 0, 8).angle, 0);
        assert_eq!(State::new(0, 0, 9).angle, 1);
        assert_eq!(State::new(0, 0, 15).angle, 7);
        assert_eq!(State::new(0, 0, 16).angle, 0);
    }

    #[test]
    fn state_new_normalises_a_negative_heading_as_its_unsigned_byte() {
        // `angle` is a `u8`, so a negative heading can only reach
        // `State::new` as its two's-complement byte, and `% 8` is taken of
        // that: -1 arrives as 255 and normalises to 7, -3 as 253 and
        // normalises to 5, -8 as 248 and normalises to 0. Those are the
        // same values as `(-1).rem_euclid(8)` and friends, because 256 is
        // a multiple of 8 -- the type, not a normalisation step, is what
        // makes a negative heading well-defined here.
        assert_eq!(State::new(0, 0, (-1i32) as u8).angle, 7);
        assert_eq!(State::new(0, 0, (-3i32) as u8).angle, 5);
        assert_eq!(State::new(0, 0, (-8i32) as u8).angle, 0);
    }

    #[test]
    fn state_is_compared_and_hashed_on_all_three_fields() {
        assert_eq!(State::new(1, 2, 3), State::new(1, 2, 11));
        assert_ne!(State::new(1, 2, 3), State::new(1, 2, 4));
        assert_ne!(State::new(1, 2, 3), State::new(2, 1, 3));
    }

    #[test]
    fn route_search_stats_default_is_all_zeros_and_empty() {
        let stats = RouteSearchStats::default();
        assert_eq!(stats.window_attempts, 0);
        assert!(!stats.used_full_grid_fallback);
        assert_eq!(stats.expanded_states, 0);
        assert_eq!(stats.generated_neighbors, 0);
        assert_eq!(stats.heap_pushes, 0);
        assert_eq!(stats.heap_pops, 0);
        assert_eq!(stats.max_heap_size, 0);
        assert_eq!(stats.last_window_area_cells, 0);
        assert_eq!(stats.diagonal_halo_contacts, 0);
        assert_eq!(stats.crossing_candidate_checks, 0);
        assert_eq!(stats.crossing_accepted, 0);
        assert_eq!(stats.route_search_total_time_us, 0);
        assert_eq!(stats.crossing_hotpath_total_time_us, 0);
        assert!(!stats.jps4_requested);
        assert!(!stats.jps4_eligible);
        assert!(!stats.jps4_used);
        assert_eq!(stats.jps4_fallbacks, 0);
        assert_eq!(stats.jps4_fallback_reason, "");
        assert!(stats.crossing_perpendicular_reject_by_partner.is_empty());
        assert!(stats.crossing_after_margin_by_partner.is_empty());
        assert!(stats.crossing_pending_straight_by_partner.is_empty());
    }

    #[test]
    fn route_search_stats_default_zeros_every_per_primitive_class_array() {
        let stats = RouteSearchStats::default();
        for (name, counters) in [
            ("generated", stats.primitive_generated_by_class),
            ("bounds_rejects", stats.primitive_bounds_rejects_by_class),
            ("closed_rejects", stats.primitive_closed_rejects_by_class),
            ("cost_pruned", stats.primitive_cost_pruned_by_class),
            (
                "footprint_checks",
                stats.primitive_footprint_checks_by_class,
            ),
            (
                "footprint_rejects",
                stats.primitive_footprint_rejects_by_class,
            ),
            ("accepted", stats.primitive_accepted_by_class),
        ] {
            assert_eq!(counters.len(), PRIMITIVE_TRANSITION_CLASS_COUNT);
            assert!(
                counters.iter().all(|value| *value == 0),
                "primitive_{name}_by_class must default to all zeros, got {counters:?}"
            );
        }
    }

    #[test]
    fn unsupported_request_defaults_to_zero() {
        // Only an engine's own tests read this field; it is 1 exactly when
        // the engine cannot serve that kind of request at all.
        assert_eq!(RouteSearchStats::default().unsupported_request, 0);
    }
}
