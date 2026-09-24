use std::time::Instant;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::obstacle_map::{CellKey, ObstacleMap};
use crate::search::astar::crossing_rules::TerminalBumpGuard;
use crate::search::state::RouteResult;

use crate::engine::*;

#[derive(Clone)]
pub(crate) struct NativeRouteJob {
    pub(crate) net_id: u64,
    pub(crate) source: PyState,
    pub(crate) target: PyState,
    pub(crate) opened_cells: Vec<(i32, i32)>,
    pub(crate) opened_cell_keys: FxHashSet<CellKey>,
    pub(crate) clearance_exempt_cells: Vec<(i32, i32)>,
    pub(crate) clearance_exempt_cell_keys: FxHashSet<CellKey>,
    pub(crate) static_cleanup_cell_keys: FxHashSet<CellKey>,
    pub(crate) source_port_um: Option<(f64, f64)>,
    pub(crate) target_port_um: Option<(f64, f64)>,
}

impl NativeRouteJob {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        net_id: u64,
        source: PyState,
        target: PyState,
        opened_cells: Vec<(i32, i32)>,
        clearance_exempt_cells: Vec<(i32, i32)>,
        static_cleanup_cells: Vec<(i32, i32)>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Self {
        let opened_cell_keys = pack_cells(&opened_cells);
        let clearance_exempt_cell_keys = pack_cells(&clearance_exempt_cells);
        let static_cleanup_cell_keys = pack_cells(&static_cleanup_cells);
        Self {
            net_id,
            source,
            target,
            opened_cells,
            opened_cell_keys,
            clearance_exempt_cells,
            clearance_exempt_cell_keys,
            static_cleanup_cell_keys,
            source_port_um,
            target_port_um,
        }
    }
}

#[derive(Clone)]
pub(crate) struct NativeRouteAttempt {
    pub(crate) bucket_name: &'static str,
    pub(crate) net_id: u64,
    pub(crate) route: Option<RouteResult>,
    pub(crate) failed: bool,
    pub(crate) error: Option<String>,
    pub(crate) repair_round: Option<u32>,
    pub(crate) candidate_blockers: Vec<u64>,
    pub(crate) ripup_ids: Vec<u64>,
}

#[derive(Clone, Debug)]
pub(crate) struct NativeRepairTraceEvent {
    pub(crate) event_name: &'static str,
    pub(crate) route_order: Option<&'static str>,
    pub(crate) action: Option<&'static str>,
    pub(crate) net_id: u64,
    pub(crate) repair_round: Option<u32>,
    pub(crate) repair_set_index: Option<u64>,
    pub(crate) candidate_blockers: Vec<u64>,
    pub(crate) ripup_ids: Vec<u64>,
    pub(crate) victim_order: Vec<u64>,
    pub(crate) victim_first: Option<bool>,
    pub(crate) reverse_victim_order: Option<bool>,
    pub(crate) success: Option<bool>,
    pub(crate) error: Option<String>,
}

#[derive(Default)]
pub(crate) struct NativeBatchTimings {
    pub(crate) route_job_unpack_us: u128,
    pub(crate) obstacle_map_prepare_us: u128,
    pub(crate) route_search_total_us: u128,
    pub(crate) simple_route_candidate_us: u128,
    pub(crate) dense_astar_us: u128,
    pub(crate) commit_cell_build_us: u128,
    pub(crate) commit_update_dynamic_map_us: u128,
    pub(crate) normal_route_wall_us: u128,
    pub(crate) probe_route_wall_us: u128,
    pub(crate) repair_failed_net_wall_us: u128,
    pub(crate) reroute_victims_wall_us: u128,
    pub(crate) normal_route_failed_wall_us: u128,
    pub(crate) probe_route_failed_wall_us: u128,
    pub(crate) repair_failed_net_failed_wall_us: u128,
    pub(crate) reroute_victims_failed_wall_us: u128,
    pub(crate) repair_probe_victim_selection_us: u128,
    pub(crate) repair_state_reset_us: u128,
    pub(crate) ripup_us: u128,
    pub(crate) history_update_us: u128,
    pub(crate) route_result_construction_us: u128,
    pub(crate) python_return_dict_us: u128,
}

impl NativeBatchTimings {
    pub(crate) fn add_route_result_stats(&mut self, route: &RouteResult) {
        self.obstacle_map_prepare_us += route.stats.obstacle_map_prepare_time_us;
        self.route_search_total_us += route.stats.route_search_total_time_us;
        self.simple_route_candidate_us += route.stats.simple_route_time_us;
        self.dense_astar_us += route.stats.search_loop_time_us;
        self.commit_cell_build_us += route.stats.commit_prepare_time_us;
        self.commit_update_dynamic_map_us += route.stats.commit_time_us;
    }

    pub(crate) fn add_route_result_stats_if(&mut self, enabled: bool, route: &RouteResult) {
        if enabled {
            self.add_route_result_stats(route);
        }
    }
}

pub(crate) struct RepairBatchState {
    pub(crate) final_routes: FxHashMap<u64, RouteResult>,
    pub(crate) attempts: Vec<NativeRouteAttempt>,
    pub(crate) repair_trace: Vec<NativeRepairTraceEvent>,
    pub(crate) repair_count: u32,
    pub(crate) failed_net_id: Option<u64>,
    pub(crate) failed_error: Option<String>,
    pub(crate) retried_source_layers: FxHashSet<i32>,
    pub(crate) timings: NativeBatchTimings,
    pub(crate) trace_last_route_start: Option<Instant>,
    /// Job indices deferred by [`PyPhotonicRouter::try_source_layer_center_out_repair`]
    /// when it cannot restore or reroute some net(s) after a failed
    /// center-out attempt. Drained into the back of the work queue in
    /// [`PyPhotonicRouter::route_many_with_repair_and_commit`] instead of
    /// aborting the whole batch -- see
    /// `.agent/execplans/2026-09-14-loss-driven-endgame-at-64x64.md` option A.
    pub(crate) deferred_job_indices: Vec<usize>,
    pub(crate) deferred_count: u32,
    /// Distinct partner net ids from the crossing violation(s) that caused
    /// the most recent `try_commit_clean_probe` post-commit-validation
    /// rejection. Set by that function immediately before it returns
    /// `Err(())`; consumed and cleared by
    /// `route_many_with_negotiated_repair_and_commit`, which folds these
    /// into the rejected net's blockers instead of aborting the batch. See
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`,
    /// Milestone 4.
    pub(crate) last_rejected_commit_partners: Vec<u64>,
}

pub(crate) struct RepairAttemptState {
    pub(crate) repaired: bool,
    pub(crate) round_base_map: ObstacleMap,
    pub(crate) round_base_center_routes: FxHashMap<u64, Vec<(i32, i32)>>,
    pub(crate) round_base_realized_center_routes: FxHashMap<u64, Vec<(f64, f64)>>,
    pub(crate) round_base_target_terminal_bump_guards: FxHashMap<u64, TerminalBumpGuard>,
    pub(crate) round_base_opened_cell_keys: FxHashMap<u64, FxHashSet<CellKey>>,
    pub(crate) round_base_crossing_events: Vec<CrossingEvent>,
    pub(crate) round_base_routes: FxHashMap<u64, RouteResult>,
    pub(crate) repair_victim_sets: Vec<(u32, Vec<u64>)>,
    pub(crate) learned_repair_keepouts_by_ripup: FxHashMap<Vec<u64>, FxHashSet<CellKey>>,
    pub(crate) learned_victim_only_keepouts_by_ripup: FxHashMap<Vec<u64>, FxHashSet<CellKey>>,
    pub(crate) learned_repair_retry_counts: FxHashMap<Vec<u64>, usize>,
}

/// Minimal committed-state snapshot for
/// [`PyPhotonicRouter::try_negotiated_displacement`] -- the same fields
/// [`RepairAttemptState`]'s `round_base_*` fields capture, without that
/// struct's other repair-chain-specific bookkeeping this negotiation
/// mechanism does not need. See
/// `.agent/execplans/2026-08-25-negotiated-repair-engine.md` Milestone 5.
pub(crate) struct NegotiationSnapshot {
    pub(crate) obstacle_map: ObstacleMap,
    pub(crate) committed_center_routes: FxHashMap<u64, Vec<(i32, i32)>>,
    pub(crate) committed_realized_center_routes: FxHashMap<u64, Vec<(f64, f64)>>,
    pub(crate) committed_target_terminal_bump_guards: FxHashMap<u64, TerminalBumpGuard>,
    pub(crate) committed_opened_cell_keys: FxHashMap<u64, FxHashSet<CellKey>>,
    pub(crate) crossing_events: Vec<CrossingEvent>,
    pub(crate) final_routes: FxHashMap<u64, RouteResult>,
}

pub(crate) struct ProbeState {
    pub(crate) probe_route: RouteResult,
    pub(crate) crossing_repair_enabled: bool,
    pub(crate) allowed_crossing_partners: FxHashSet<u64>,
    pub(crate) probe_crossing_events: Vec<CrossingEvent>,
    pub(crate) strict_expected_crossing_probe: bool,
    pub(crate) probe_crossing_compliant: bool,
    pub(crate) probe_realized_crossing_violations: Vec<InvalidCrossingIntersection>,
    pub(crate) probe_grid_crossing_violations: Vec<InvalidCrossingIntersection>,
    pub(crate) probe_repair_keepout_keys: FxHashSet<CellKey>,
    pub(crate) candidate_blockers: Vec<u64>,
    /// Crossings disabled (contribution 2, or crossings off): the
    /// committed nets whose centerline the probe's free path geometrically
    /// intersects. Every one of them must move for this net to route, so
    /// `ripup_illegal_crossing_partners` rips them all (2026-09-16,
    /// multiportmmi_128x128 fan-out: the old single-candidate fallback
    /// ripped a halo neighbour and 17 of 20 post-rip-up searches failed).
    pub(crate) probe_intersecting_partners: Vec<u64>,
}

pub(crate) struct RepairModeAttemptState {
    pub(crate) route_order: &'static str,
    pub(crate) victim_reroute_ids: Vec<u64>,
    pub(crate) victim_first_probe_reservation: FxHashSet<CellKey>,
    pub(crate) temporary_probe_reservation: FxHashSet<CellKey>,
    pub(crate) temporary_probe_reservation_added: bool,
    pub(crate) victim_reroute_only_reservation: FxHashSet<CellKey>,
    pub(crate) victim_reroute_only_reservation_added: bool,
    pub(crate) mode_failed: bool,
    pub(crate) repaired_route: Option<RouteResult>,
}

pub(crate) enum LocalizedKeepoutOutcome {
    Routed,
    NotResolved,
}

pub(crate) enum CommitIfCleanOutcome {
    Routed,
    NotResolved,
}

pub(crate) enum PlainRouteOutcome {
    Routed,
    NotResolved,
}

/// Result of `try_ripup_single_victim_and_reroute_with`: both nets routed
/// (`Both`), the net routed but the victim could not be rerouted and, on
/// request, stays ripped with the net's new route kept (`NetOnly`), or the
/// attempt was rolled back entirely (`Failed`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum VictimRerouteOutcome {
    Both,
    NetOnly,
    Failed,
}

/// Result of one `try_braid_repair` pass: the pair was repaired (`Kept`),
/// rolled back to the braid (`Failed`), or -- the negotiated engine's
/// escalation (2026-09-15 fan-out analysis, Milestone 8 of
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`)
/// -- the net now holds its braid-free route and the partner stays ripped
/// for the caller to re-queue (`VictimRipped`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BraidRepairOutcome {
    Kept(u64),
    Failed(u64),
    VictimRipped(u64),
}

pub(crate) enum SourceLayerCenterOutOutcome {
    Routed,
    NotAttempted,
}

pub(crate) enum PendingStraightRepairOutcome {
    Routed,
    NotResolved,
}

pub(crate) enum LidarDirectCrossingOutcome {
    Routed,
    NotResolved,
}

pub(crate) enum VictimPlainRerouteOutcome {
    Routed,
    Failed,
}

pub(crate) enum CrossingAwareVictimRerouteOutcome {
    Routed,
    Blocked,
    NotAttempted,
}

pub(crate) enum RerouteCurrentNetAfterVictimsOutcome {
    Routed,
    Failed,
}

pub(crate) struct NativeEndpointCorrection {
    pub(crate) centerline: Vec<(f64, f64)>,
    pub(crate) committed_bump: bool,
    pub(crate) candidate_index: Option<usize>,
    pub(crate) candidate_label: Option<String>,
}

pub(crate) const CROSSING_SPACING_HISTORY_AMOUNT: u32 = 1;

pub(crate) const LONG_STRAIGHT_CONGESTION_MIN_UM: f64 = 200.0;

pub(crate) const LONG_STRAIGHT_CONGESTION_LATERAL_RADIUS_CELLS: i32 = 5;

pub(crate) const LONG_STRAIGHT_CONGESTION_AMOUNT: u32 = 1;

pub(crate) const SOURCE_LAYER_CENTER_OUT_MIN_JOBS: usize = 8;

pub(crate) const NEGOTIATED_BUDGET_FIRST_ATTEMPT: u64 = 2_000_000;

pub(crate) const NEGOTIATED_BUDGET_FIRST_RETRY: u64 = 10_000_000;

pub(crate) const NEGOTIATED_BUDGET_RETRY: u64 = 30_000_000;

pub(crate) const NEGOTIATED_BUDGET_BRAID: u64 = 10_000_000;

pub(crate) const NEGOTIATED_BRAID_MAX_PASSES: u32 = 4;

pub(crate) const NEGOTIATED_PROBE_GUIDANCE_LOSS: f64 = 0.0;

/// Unit tests for the job and batch-state constructors, Milestone 6
/// Slice 3 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::fresh_repair_batch_state;
    use crate::obstacle_map::pack_xy;

    #[test]
    fn native_route_job_new_maps_every_argument_to_its_field() {
        let job = NativeRouteJob::new(
            7,
            PyState::new(1, 2, 0),
            PyState::new(3, 4, 4),
            vec![(1, 2), (2, 2)],
            vec![(5, 5)],
            vec![(6, 6), (6, 6)],
            Some((1.5, 2.5)),
            None,
        );
        assert_eq!(job.net_id, 7);
        assert_eq!((job.source.x, job.source.y, job.source.angle), (1, 2, 0));
        assert_eq!((job.target.x, job.target.y, job.target.angle), (3, 4, 4));
        assert_eq!(job.opened_cells, vec![(1, 2), (2, 2)]);
        assert_eq!(job.clearance_exempt_cells, vec![(5, 5)]);
        assert_eq!(job.source_port_um, Some((1.5, 2.5)));
        assert_eq!(job.target_port_um, None);
    }

    #[test]
    fn native_route_job_new_packs_each_cell_list_into_its_key_set() {
        let job = NativeRouteJob::new(
            7,
            PyState::new(1, 2, 0),
            PyState::new(3, 4, 4),
            vec![(1, 2), (2, 2)],
            vec![(5, 5)],
            // The static-cleanup cells are kept only as keys, and the
            // duplicate collapses there.
            vec![(6, 6), (6, 6)],
            None,
            None,
        );
        assert_eq!(job.opened_cell_keys.len(), 2);
        assert!(job.opened_cell_keys.contains(&pack_xy(1, 2)));
        assert!(job.opened_cell_keys.contains(&pack_xy(2, 2)));
        assert_eq!(job.clearance_exempt_cell_keys.len(), 1);
        assert!(job.clearance_exempt_cell_keys.contains(&pack_xy(5, 5)));
        assert_eq!(job.static_cleanup_cell_keys.len(), 1);
        assert!(job.static_cleanup_cell_keys.contains(&pack_xy(6, 6)));
    }

    #[test]
    fn native_batch_timings_default_is_all_zeros() {
        let timings = NativeBatchTimings::default();
        for (name, value) in [
            ("route_job_unpack_us", timings.route_job_unpack_us),
            ("obstacle_map_prepare_us", timings.obstacle_map_prepare_us),
            ("route_search_total_us", timings.route_search_total_us),
            (
                "simple_route_candidate_us",
                timings.simple_route_candidate_us,
            ),
            ("dense_astar_us", timings.dense_astar_us),
            ("commit_cell_build_us", timings.commit_cell_build_us),
            (
                "commit_update_dynamic_map_us",
                timings.commit_update_dynamic_map_us,
            ),
            ("normal_route_wall_us", timings.normal_route_wall_us),
            ("probe_route_wall_us", timings.probe_route_wall_us),
            (
                "repair_failed_net_wall_us",
                timings.repair_failed_net_wall_us,
            ),
            ("reroute_victims_wall_us", timings.reroute_victims_wall_us),
            (
                "normal_route_failed_wall_us",
                timings.normal_route_failed_wall_us,
            ),
            (
                "probe_route_failed_wall_us",
                timings.probe_route_failed_wall_us,
            ),
            (
                "repair_failed_net_failed_wall_us",
                timings.repair_failed_net_failed_wall_us,
            ),
            (
                "reroute_victims_failed_wall_us",
                timings.reroute_victims_failed_wall_us,
            ),
            (
                "repair_probe_victim_selection_us",
                timings.repair_probe_victim_selection_us,
            ),
            ("repair_state_reset_us", timings.repair_state_reset_us),
            ("ripup_us", timings.ripup_us),
            ("history_update_us", timings.history_update_us),
            (
                "route_result_construction_us",
                timings.route_result_construction_us,
            ),
            ("python_return_dict_us", timings.python_return_dict_us),
        ] {
            assert_eq!(value, 0, "{name} must default to zero");
        }
    }

    #[test]
    fn a_fresh_repair_batch_state_starts_empty_with_no_failure_recorded() {
        let batch = fresh_repair_batch_state();
        assert!(batch.final_routes.is_empty());
        assert!(batch.attempts.is_empty());
        assert!(batch.repair_trace.is_empty());
        assert_eq!(batch.repair_count, 0);
        assert_eq!(batch.failed_net_id, None);
        assert_eq!(batch.failed_error, None);
        assert!(batch.retried_source_layers.is_empty());
        assert!(batch.trace_last_route_start.is_none());
        assert!(batch.deferred_job_indices.is_empty());
        assert_eq!(batch.deferred_count, 0);
        assert!(batch.last_rejected_commit_partners.is_empty());
        assert_eq!(batch.timings.route_search_total_us, 0);
    }
}
