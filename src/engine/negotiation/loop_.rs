use std::time::Instant;

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::config::NegotiationConfig;

use crate::engine::*;

impl PyPhotonicRouter {
    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn snapshot_negotiation_state(
        &self,
        batch: &RepairBatchState,
    ) -> NegotiationSnapshot {
        NegotiationSnapshot {
            obstacle_map: self.obstacle_map.clone(),
            committed_center_routes: self.committed_center_routes.clone(),
            committed_realized_center_routes: self.committed_realized_center_routes.clone(),
            committed_target_terminal_bump_guards: self
                .committed_target_terminal_bump_guards
                .clone(),
            committed_opened_cell_keys: self.committed_opened_cell_keys.clone(),
            crossing_events: self.crossing_events.clone(),
            final_routes: batch.final_routes.clone(),
        }
    }

    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn restore_negotiation_state(
        &mut self,
        batch: &mut RepairBatchState,
        snapshot: NegotiationSnapshot,
    ) {
        self.obstacle_map = snapshot.obstacle_map;
        self.committed_center_routes = snapshot.committed_center_routes;
        self.committed_realized_center_routes = snapshot.committed_realized_center_routes;
        self.committed_target_terminal_bump_guards = snapshot.committed_target_terminal_bump_guards;
        self.committed_opened_cell_keys = snapshot.committed_opened_cell_keys;
        self.crossing_events = snapshot.crossing_events;
        batch.final_routes = snapshot.final_routes;
        self.invalidate_meander_base_prefix();
    }

    /// Negotiated displacement, cascading up to `depth_remaining` levels:
    /// rip up every net in `blocker_ids`, try to commit `job` in the space
    /// that frees up, then try to reroute and recommit every displaced
    /// blocker in turn. If a blocker cannot be rerouted plainly and
    /// `depth_remaining > 0`, probe *that* blocker for its own blockers and
    /// recursively attempt to displace them too -- so a chain (`job` needs
    /// `B`'s spot, `B` needs `C`'s spot, ...) can resolve as one atomic
    /// negotiation, not just a single pairwise swap. Succeeds (returns
    /// `true`, `batch.repair_count` bumped once per top-level call, all
    /// routes left committed) only if `job` and the *entire* resulting
    /// displacement chain can be routed; any failure at any level restores
    /// every field the negotiation touched to exactly the snapshot taken at
    /// that level's own entry (nested restores compose correctly: an inner
    /// failure already undoes its own sub-chain before an outer failure
    /// restores everything above it), so a failed attempt at any depth is a
    /// pure no-op from its caller's perspective. A nested `probe_net_for_repair`
    /// call's own `batch.failed_net_id`/`failed_error` side effects (meant
    /// for a *top-level* probe failure to abort the whole batch) are
    /// explicitly cleared here when used for cascade exploration -- a
    /// blocker genuinely having no route at all only means this cascade
    /// branch fails, not that the batch should abort.
    ///
    /// Reuses `route_single_net_and_commit_native` (the same primitive
    /// `try_plain_normal_route` and every other repair strategy already
    /// uses) and `probe_net_for_repair` (unchanged, already crossing-
    /// legality-aware) rather than new search machinery. See
    /// `.agent/execplans/2026-08-25-negotiated-repair-engine.md` Milestones 5
    /// (single-level version) and 6 (cascading; the single-level version was
    /// found insufficient for `benes_16x16`, see Milestone 6's Surprises &
    /// Discoveries).
    ///
    /// As of Milestone 2 of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`,
    /// `route_many_with_negotiated_repair_and_commit` no longer calls this
    /// -- it was replaced by LiDAR's local rip-up rule
    /// (`ripup_illegal_crossing_partners`). Kept unused until Milestone 5
    /// decides its fate.
    #[allow(clippy::too_many_arguments)]
    #[allow(dead_code)]
    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn try_negotiated_displacement(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        blocker_ids: &[u64],
        job_by_id: &FxHashMap<u64, NativeRouteJob>,
        order_by_id: &FxHashMap<u64, usize>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
        depth_remaining: u32,
    ) -> bool {
        let snapshot = self.snapshot_negotiation_state(batch);
        self.ripup_repair_set_victims(batch, blocker_ids, collect_native_timing);

        let route_start = native_batch_timer(collect_native_timing);
        let route_result = self.route_single_net_and_commit_native(
            job.net_id,
            job.source,
            job.target,
            block_radius_cells,
            Some(&job.opened_cells),
            Some(&job.opened_cell_keys),
            commit_radius_cells,
            Some(&job.clearance_exempt_cells),
            Some(&job.clearance_exempt_cell_keys),
            core_radius_cells,
            job.source_port_um,
            job.target_port_um,
        );
        batch.timings.normal_route_wall_us += native_batch_elapsed_us(route_start);
        let route = match route_result {
            Ok(route) => route,
            Err(_) => {
                self.restore_negotiation_state(batch, snapshot);
                return false;
            }
        };
        remove_success_static_cleanup(&mut self.obstacle_map, job);
        batch.final_routes.insert(job.net_id, route);

        let mut all_blockers_rerouted = true;
        for blocker_id in blocker_ids {
            let Some(blocker_job) = job_by_id.get(blocker_id).cloned() else {
                continue;
            };
            let blocker_route_start = native_batch_timer(collect_native_timing);
            let blocker_result = self.route_single_net_and_commit_native(
                blocker_job.net_id,
                blocker_job.source,
                blocker_job.target,
                block_radius_cells,
                Some(&blocker_job.opened_cells),
                Some(&blocker_job.opened_cell_keys),
                commit_radius_cells,
                Some(&blocker_job.clearance_exempt_cells),
                Some(&blocker_job.clearance_exempt_cell_keys),
                core_radius_cells,
                blocker_job.source_port_um,
                blocker_job.target_port_um,
            );
            batch.timings.normal_route_wall_us += native_batch_elapsed_us(blocker_route_start);
            match blocker_result {
                Ok(blocker_route) => {
                    remove_success_static_cleanup(&mut self.obstacle_map, &blocker_job);
                    batch.final_routes.insert(blocker_job.net_id, blocker_route);
                }
                Err(_) if depth_remaining > 0 => {
                    let sub_probe = self.probe_net_for_repair(
                        batch,
                        &blocker_job,
                        order_by_id,
                        block_radius_cells,
                        commit_radius_cells,
                        collect_native_timing,
                        trace_native_repair,
                    );
                    let cascaded = match sub_probe {
                        Ok(sub_probe) if !sub_probe.candidate_blockers.is_empty() => self
                            .try_negotiated_displacement(
                                batch,
                                &blocker_job,
                                &sub_probe.candidate_blockers,
                                job_by_id,
                                order_by_id,
                                block_radius_cells,
                                commit_radius_cells,
                                core_radius_cells,
                                collect_native_timing,
                                trace_native_repair,
                                depth_remaining - 1,
                            ),
                        _ => false,
                    };
                    // A nested probe failure (no route exists for
                    // `blocker_job` even ignoring dynamic obstacles) is only
                    // this cascade branch failing, not a reason to abort the
                    // whole batch -- clear the side effect a top-level probe
                    // failure would otherwise leave behind.
                    batch.failed_net_id = None;
                    batch.failed_error = None;
                    if !cascaded {
                        all_blockers_rerouted = false;
                        break;
                    }
                }
                Err(_) => {
                    all_blockers_rerouted = false;
                    break;
                }
            }
        }

        if all_blockers_rerouted {
            batch.repair_count += 1;
            true
        } else {
            self.restore_negotiation_state(batch, snapshot);
            false
        }
    }

    /// LiDAR's `ripupfailedNets` (Milestone 3 of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`):
    /// called once at the end of every round of
    /// `route_many_with_negotiated_repair_and_commit`. Every net that ended
    /// the round still unrouted (`round_failed`) is ripped up together with
    /// the blockers its most recent probe reported (`last_blockers`) that
    /// are still committed; history cost is bumped on a ripped blocker's
    /// cells before it is ripped, via the same per-commit chokepoint
    /// (`add_repair_history_for_route`) every successful commit uses. The
    /// failed nets and their ripped blockers are then requeued at the front
    /// of `queue`, failed nets first in `round_failed` order and the ripped
    /// blockers after, so both are tried again first thing next round. A
    /// no-op when `round_failed` is empty (nothing failed this round). On
    /// the second call and every second call after that, the history map
    /// and the local rip-up rule's once-per-epoch set are cleared -- this
    /// is this milestone's own epoch boundary, independent of (and able to
    /// coincide with) the existing "two rounds without progress" reset.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn global_ripup_round(
        &mut self,
        batch: &mut RepairBatchState,
        round_failed: &[u64],
        last_blockers: &FxHashMap<u64, Vec<u64>>,
        queue: &mut std::collections::VecDeque<u64>,
        failed_counts: &mut FxHashMap<u64, u32>,
        ripped_once: &mut FxHashSet<u64>,
        global_ripups: &mut u32,
        round: u32,
        trace_native_repair: bool,
    ) {
        if round_failed.is_empty() {
            return;
        }
        *global_ripups += 1;

        let mut ripped_blocker_ids: Vec<u64> = Vec::new();
        let mut ripped_seen: FxHashSet<u64> = FxHashSet::default();
        for &failed_net_id in round_failed {
            let Some(blockers) = last_blockers.get(&failed_net_id) else {
                continue;
            };
            for &blocker_id in blockers {
                let Some(route) = batch.final_routes.get(&blocker_id) else {
                    // Not currently committed (already ripped by this or an
                    // earlier step this round) -- nothing to do for it.
                    continue;
                };
                let route = route.clone();
                self.add_repair_history_for_route(&route);
                self.ripup_route(blocker_id);
                batch.final_routes.remove(&blocker_id);
                *failed_counts.entry(blocker_id).or_insert(0) += 1;
                if ripped_seen.insert(blocker_id) {
                    ripped_blocker_ids.push(blocker_id);
                }
            }
        }

        // Rebuild the queue for the next round: pull every id about to be
        // requeued out of wherever it currently sits (a ripped blocker may
        // already be in `queue` from a same-round local rip-up of a
        // different net), then push the failed nets first (in
        // `round_failed` order) and the ripped blockers after them (in rip
        // order, deduplicated) to the front, so both are tried again first
        // thing next round -- LiDAR's "ripped nets go back to the queue".
        // Nets already routed stay out of the queue entirely (they were
        // never added to `round_failed`).
        let mut requeue_ids: Vec<u64> = Vec::new();
        let mut requeue_seen: FxHashSet<u64> = FxHashSet::default();
        for &failed_net_id in round_failed {
            if requeue_seen.insert(failed_net_id) {
                requeue_ids.push(failed_net_id);
            }
        }
        for &blocker_id in &ripped_blocker_ids {
            if requeue_seen.insert(blocker_id) {
                requeue_ids.push(blocker_id);
            }
        }
        queue.retain(|id| !requeue_seen.contains(id));
        for &id in requeue_ids.iter().rev() {
            queue.push_front(id);
        }

        push_native_repair_trace(
            &mut batch.repair_trace,
            "global_ripup",
            Some("negotiated"),
            Some("ripup_failed_and_blockers"),
            round_failed[0],
            Some(round),
            None,
            round_failed,
            &[],
            &ripped_blocker_ids,
            None,
            None,
            None,
            None,
        );
        if trace_native_repair {
            eprintln!(
                "{}native_repair_global_ripup round={} failed={:?} ripped={:?}",
                trace_t(self.negotiated_batch_start),
                round,
                round_failed,
                ripped_blocker_ids
            );
        }

        // This milestone's own epoch boundary (LiDAR clears the history
        // map after its second global rip-up so the cost landscape does
        // not calcify): fires exactly once, on the second global rip-up,
        // as in LiDAR (`ripup_times == 2`); clearing at every second call
        // (the first version, 2026-09-14) never let pressure build on a
        // shared corridor and two nets displaced each other for 10 rounds.
        // Independent of the "two rounds without progress" reset in the
        // caller, which may or may not fire the same round.
        if *global_ripups == 2 {
            self.obstacle_map.clear_history();
            ripped_once.clear();
            push_native_repair_trace(
                &mut batch.repair_trace,
                "history_clear",
                Some("negotiated"),
                Some("epoch_boundary"),
                round_failed[0],
                Some(round),
                None,
                &[],
                &[],
                &[],
                None,
                None,
                None,
                None,
            );
            if trace_native_repair {
                eprintln!(
                    "{}history_clear round={} global_ripups={}",
                    trace_t(self.negotiated_batch_start),
                    round,
                    *global_ripups
                );
            }
        }
    }

    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn ripup_repair_set_victims(
        &mut self,
        batch: &mut RepairBatchState,
        ripup_ids: &[u64],
        collect_native_timing: bool,
    ) {
        for old_id in ripup_ids {
            let ripup_start = native_batch_timer(collect_native_timing);
            self.remove_crossing_events_for_net(*old_id);
            self.obstacle_map.ripup_route(*old_id);
            self.committed_center_routes.remove(old_id);
            self.committed_realized_center_routes.remove(old_id);
            self.committed_target_terminal_bump_guards.remove(old_id);
            self.committed_opened_cell_keys.remove(old_id);
            batch.timings.ripup_us += native_batch_elapsed_us(ripup_start);
            batch.final_routes.remove(old_id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;

    #[test]
    fn global_ripup_round_rips_blockers_and_queues_failed_nets_first() {
        let (mut router, _jobs) = crossing_conflict_fixture();
        let mut batch = fresh_repair_batch_state();
        for net_id in [1u64, 2, 3] {
            let route = route_with_real_cells_for_net(&router, net_id);
            batch.final_routes.insert(net_id, route);
        }

        let round_failed: Vec<u64> = vec![4];
        let mut last_blockers: FxHashMap<u64, Vec<u64>> = FxHashMap::default();
        last_blockers.insert(4, vec![3]);
        let mut queue: std::collections::VecDeque<u64> = std::collections::VecDeque::new();
        let mut failed_counts: FxHashMap<u64, u32> = FxHashMap::default();
        let mut ripped_once: FxHashSet<u64> = FxHashSet::default();
        let mut global_ripups: u32 = 0;

        router.global_ripup_round(
            &mut batch,
            &round_failed,
            &last_blockers,
            &mut queue,
            &mut failed_counts,
            &mut ripped_once,
            &mut global_ripups,
            1,
            false,
        );

        assert!(
            router.obstacle_map.get_net_cells(3).is_none(),
            "net 3 must be ripped from the obstacle map"
        );
        assert!(
            !batch.final_routes.contains_key(&3),
            "net 3 must be removed from final_routes"
        );
        assert_eq!(
            failed_counts.get(&3).copied(),
            Some(1),
            "the ripped blocker's failed_counts must be bumped exactly once"
        );
        assert_eq!(
            queue,
            std::collections::VecDeque::from([4u64, 3u64]),
            "the queue must start with the failed net first, then its ripped blocker"
        );
        assert_eq!(global_ripups, 1);
    }

    #[test]
    fn global_ripup_round_bumps_history_on_ripped_cells() {
        let (mut router, _jobs) = crossing_conflict_fixture();
        let net3_cells = router.get_net_cells(3);
        assert!(
            !net3_cells.is_empty(),
            "fixture net 3 must have a non-empty committed footprint"
        );
        // `add_repair_history_for_route` is a no-op unless
        // `commit_history_increment` is nonzero -- set the same way
        // `route_many_with_negotiated_repair_and_commit` sets it from its
        // own `history_increment` argument.
        router.commit_history_increment = 1;

        let mut batch = fresh_repair_batch_state();
        for net_id in [1u64, 2, 3] {
            let route = route_with_real_cells_for_net(&router, net_id);
            batch.final_routes.insert(net_id, route);
        }

        let round_failed: Vec<u64> = vec![4];
        let mut last_blockers: FxHashMap<u64, Vec<u64>> = FxHashMap::default();
        last_blockers.insert(4, vec![3]);
        let mut queue: std::collections::VecDeque<u64> = std::collections::VecDeque::new();
        let mut failed_counts: FxHashMap<u64, u32> = FxHashMap::default();
        let mut ripped_once: FxHashSet<u64> = FxHashSet::default();
        let mut global_ripups: u32 = 0;

        router.global_ripup_round(
            &mut batch,
            &round_failed,
            &last_blockers,
            &mut queue,
            &mut failed_counts,
            &mut ripped_once,
            &mut global_ripups,
            1,
            false,
        );

        let bumped = net3_cells
            .iter()
            .any(|&(x, y)| router.obstacle_map.get_history_cost(x, y) > 0);
        assert!(
            bumped,
            "expected history > 0 on at least one of net 3's former cells: {:?}",
            net3_cells
        );
    }

    #[test]
    fn second_global_ripup_clears_history_and_epoch() {
        let (mut router, _jobs) = crossing_conflict_fixture();
        let net3_cells = router.get_net_cells(3);
        router.commit_history_increment = 1;

        let mut batch = fresh_repair_batch_state();
        for net_id in [1u64, 2, 3] {
            let route = route_with_real_cells_for_net(&router, net_id);
            batch.final_routes.insert(net_id, route);
        }

        let mut queue: std::collections::VecDeque<u64> = std::collections::VecDeque::new();
        let mut failed_counts: FxHashMap<u64, u32> = FxHashMap::default();
        let mut ripped_once: FxHashSet<u64> = FxHashSet::default();
        ripped_once.insert(99); // a dummy epoch marker, to prove it gets cleared
        let mut global_ripups: u32 = 0;

        // First call: net 4 failed, blamed net 3 -- ripped, history bumped,
        // no epoch clear yet (this is the first global rip-up).
        let mut last_blockers: FxHashMap<u64, Vec<u64>> = FxHashMap::default();
        last_blockers.insert(4, vec![3]);
        router.global_ripup_round(
            &mut batch,
            &[4],
            &last_blockers,
            &mut queue,
            &mut failed_counts,
            &mut ripped_once,
            &mut global_ripups,
            1,
            false,
        );
        assert_eq!(global_ripups, 1);
        assert!(net3_cells
            .iter()
            .any(|&(x, y)| router.obstacle_map.get_history_cost(x, y) > 0));
        assert!(
            ripped_once.contains(&99),
            "the epoch marker must survive the first global rip-up"
        );

        // Second call: a different failed net (6) blames net 2 (still
        // committed, untouched by the first call) -- this is the second
        // global rip-up, so the epoch boundary fires: history clears and
        // `ripped_once` resets.
        last_blockers.insert(6, vec![2]);
        router.global_ripup_round(
            &mut batch,
            &[6],
            &last_blockers,
            &mut queue,
            &mut failed_counts,
            &mut ripped_once,
            &mut global_ripups,
            2,
            false,
        );

        assert_eq!(global_ripups, 2);
        assert!(
            net3_cells
                .iter()
                .all(|&(x, y)| router.obstacle_map.get_history_cost(x, y) == 0),
            "history on net 3's former cells must be cleared again after the second \
             global rip-up"
        );
        assert!(
            ripped_once.is_empty(),
            "ripped_once must be cleared on the second global rip-up epoch boundary"
        );
        assert!(
            batch
                .repair_trace
                .iter()
                .any(|event| event.event_name == "history_clear"),
            "expected a history_clear trace event: {:?}",
            batch.repair_trace
        );
    }

    #[test]
    fn global_ripup_round_with_no_failures_is_a_noop() {
        let (mut router, _jobs) = crossing_conflict_fixture();
        let mut batch = fresh_repair_batch_state();
        for net_id in [1u64, 2, 3] {
            let route = route_with_real_cells_for_net(&router, net_id);
            batch.final_routes.insert(net_id, route);
        }

        let round_failed: Vec<u64> = Vec::new();
        let last_blockers: FxHashMap<u64, Vec<u64>> = FxHashMap::default();
        let mut queue: std::collections::VecDeque<u64> =
            std::collections::VecDeque::from([7u64, 8u64]);
        let mut failed_counts: FxHashMap<u64, u32> = FxHashMap::default();
        let mut ripped_once: FxHashSet<u64> = FxHashSet::default();
        let mut global_ripups: u32 = 0;

        router.global_ripup_round(
            &mut batch,
            &round_failed,
            &last_blockers,
            &mut queue,
            &mut failed_counts,
            &mut ripped_once,
            &mut global_ripups,
            1,
            false,
        );

        assert_eq!(
            global_ripups, 0,
            "no failures this round -> no global rip-up"
        );
        assert!(
            batch.final_routes.contains_key(&3),
            "net 3 must stay committed"
        );
        assert!(
            router.obstacle_map.get_net_cells(3).is_some(),
            "net 3 must stay committed on the obstacle map"
        );
        assert_eq!(
            queue,
            std::collections::VecDeque::from([7u64, 8u64]),
            "the queue must be untouched"
        );
        assert!(failed_counts.is_empty());
        assert!(batch.repair_trace.is_empty());
    }
}

/// The Python job tuple the `#[pymethods]` wrapper forwards unchanged.
pub(crate) type PyRouteJobTuple = (
    u64,
    PyState,
    PyState,
    Vec<(i32, i32)>,
    Vec<(i32, i32)>,
    Vec<(i32, i32)>,
    Option<(f64, f64)>,
    Option<(f64, f64)>,
);

/// The batch's jobs plus the three lookup tables the loop's steps read:
/// the caller's net order (`route_rust.py`'s `_topological_net_route_order`),
/// the jobs by id, and the job indices by source column (the probe-guided
/// step's structural widening -- the same construction as
/// `route_many_with_repair_and_commit`'s map of the same name).
struct NetTables {
    jobs: Vec<NativeRouteJob>,
    order_by_id: FxHashMap<u64, usize>,
    job_by_id: FxHashMap<u64, NativeRouteJob>,
    source_layer_indices_by_x: FxHashMap<i32, Vec<usize>>,
}

impl NetTables {
    fn from_py_jobs(jobs: Vec<PyRouteJobTuple>) -> Self {
        let jobs: Vec<NativeRouteJob> = jobs
            .into_iter()
            .map(
                |(
                    net_id,
                    source,
                    target,
                    opened_cells,
                    clearance_exempt_cells,
                    static_cleanup_cells,
                    source_port_um,
                    target_port_um,
                )| {
                    NativeRouteJob::new(
                        net_id,
                        source,
                        target,
                        opened_cells,
                        clearance_exempt_cells,
                        static_cleanup_cells,
                        source_port_um,
                        target_port_um,
                    )
                },
            )
            .collect();
        let mut source_layer_indices_by_x: FxHashMap<i32, Vec<usize>> = FxHashMap::default();
        for (index, job) in jobs.iter().enumerate() {
            source_layer_indices_by_x
                .entry(job.source.x)
                .or_default()
                .push(index);
        }
        NetTables {
            order_by_id: jobs
                .iter()
                .enumerate()
                .map(|(index, job)| (job.net_id, index))
                .collect(),
            job_by_id: jobs.iter().cloned().map(|job| (job.net_id, job)).collect(),
            source_layer_indices_by_x,
            jobs,
        }
    }
}

/// The counters the `native_negotiated_done` line reports.
#[derive(Default)]
struct NegotiationCounters {
    global_ripups: u32,
    local_ripups: u32,
    probe_guided: u32,
    crossing_free: u32,
    /// The negotiated chain no longer calls `try_commit_clean_probe`
    /// (owner decision 2026-09-15: the clean-probe commit shortcut is
    /// dropped here), so this always stays 0 -- kept only so the
    /// `native_negotiated_done` trace line's shape is unchanged.
    commit_rejected: u32,
}

/// What one round mutates: the committed batch, the queue and its
/// bookkeeping, and the braid pairs already tried this round.
struct RoundState<'a> {
    batch: &'a mut RepairBatchState,
    queue: &'a mut NetQueue,
    /// Once-per-round rule for `try_braid_repair`'s pairwise attempts
    /// (2026-09-15 11:54 trace finding): an unordered `{net, partner}`
    /// pair whose braid attempt ended `keep=false` is not retried again
    /// this round -- a pair that failed last round may look different this
    /// round, so it deserves one fresh attempt per round.
    braid_failed_pairs: &'a mut FxHashSet<(u64, u64)>,
}

fn empty_batch_state() -> RepairBatchState {
    RepairBatchState {
        final_routes: FxHashMap::default(),
        attempts: Vec::new(),
        repair_trace: Vec::new(),
        repair_count: 0,
        failed_net_id: None,
        failed_error: None,
        retried_source_layers: FxHashSet::default(),
        timings: NativeBatchTimings::default(),
        trace_last_route_start: None,
        deferred_job_indices: Vec::new(),
        deferred_count: 0,
        last_rejected_commit_partners: Vec::new(),
    }
}

/// This net's blockers as of its most recent probe, for
/// `global_ripup_round` to consult if the net is still unrouted at the end
/// of the round: the probe's candidate blockers, unioned with the partner
/// ids of its realized illegal crossings (the nets it would have to
/// cross), and -- for a crossing-free net, which may cross nobody -- the
/// partners of its legal crossings too.
fn probe_blockers(probe: &ProbeState, crossing_free: bool) -> Vec<u64> {
    let mut blockers: Vec<u64> = probe.candidate_blockers.clone();
    for violation in &probe.probe_realized_crossing_violations {
        if !blockers.contains(&violation.partner_net_id) {
            blockers.push(violation.partner_net_id);
        }
    }
    if crossing_free {
        for event in &probe.probe_crossing_events {
            if !blockers.contains(&event.partner_net_id) {
                blockers.push(event.partner_net_id);
            }
        }
    }
    blockers
}

/// The probe's crossing partners as this loop's two crossing-accepting
/// steps use them: the legal ones, and at most three illegal ones -- the
/// profile (2026-09-14 20:46 Surprises of
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`)
/// found the unfiltered sweep costing 406 s on net 313's 22 partners and
/// resolving nothing. A crossing-free net gets neither: it skips both
/// steps and goes straight to the rip-up of every probe partner.
fn loop_partner_split(probe: &ProbeState, crossing_free: bool) -> (Vec<u64>, Vec<u64>) {
    if crossing_free {
        return (Vec::new(), Vec::new());
    }
    let (legal_partner_ids, all_illegal_partner_ids) = split_probe_partners(probe);
    (
        legal_partner_ids,
        all_illegal_partner_ids.into_iter().take(3).collect(),
    )
}

impl PyPhotonicRouter {
    /// The braid passes that follow every route this loop commits, plus
    /// the requeue of whatever a braid escalation left ripped
    /// (`BraidRepairOutcome::VictimRipped`).
    fn braid_and_requeue(
        &mut self,
        state: &mut RoundState,
        job: &NativeRouteJob,
        tables: &NetTables,
        args: &SearchArgs,
    ) {
        let mut braid_ripped_victims: Vec<u64> = Vec::new();
        self.run_braid_repair_passes(
            state.batch,
            job,
            &tables.job_by_id,
            args.block_radius_cells,
            args.commit_radius_cells,
            args.core_radius_cells,
            args.collect_native_timing,
            args.trace,
            state.braid_failed_pairs,
            &mut braid_ripped_victims,
        );
        state.queue.requeue_braid_victims(&braid_ripped_victims);
    }

    /// The probe's legal partners, widened structurally (2026-09-15 15:00
    /// Decision Log entry, "Slice result 15:40"): the probe's free path
    /// threads between some lanes of a fan-out bundle rather than crossing
    /// all of them, so the partners it names alone underprice the search.
    fn widened_probe_partners(
        &self,
        legal_partner_ids: &[u64],
        tables: &NetTables,
        batch: &RepairBatchState,
        net_id: u64,
        trace: bool,
    ) -> Vec<u64> {
        let widened = widen_probe_partners_by_source_column(
            legal_partner_ids,
            &tables.job_by_id,
            &tables.source_layer_indices_by_x,
            &tables.jobs,
            &batch.final_routes,
            net_id,
        );
        if trace {
            let added_ids: Vec<u64> = widened
                .iter()
                .copied()
                .filter(|id| !legal_partner_ids.contains(id))
                .collect();
            eprintln!(
                "{}native_repair_probe_guided net={} partners={:?} widened={:?}",
                trace_t(self.negotiated_batch_start),
                net_id,
                legal_partner_ids,
                added_ids
            );
        }
        widened
    }

    /// Contribution 1's crossing-free rule on the nets named by at least
    /// one planned pair of the guidance; without guidance (lidar-pure) or
    /// with the rule switched off, nobody is exempt.
    fn negotiated_crossing_free_policy(
        &self,
        negotiation: &NegotiationConfig,
    ) -> Box<dyn CrossingFreePolicy> {
        match negotiation
            .crossing_free_unplanned
            .then(|| self.crossing_context.guidance())
            .flatten()
        {
            Some(guidance) => Box::new(PlannedPairsOnly(
                guidance
                    .pairs()
                    .into_iter()
                    .flat_map(|(a, b)| [a, b])
                    .collect(),
            )),
            None => Box::new(Never),
        }
    }

    /// LiDAR's `ripupfailedNets` at the end of every round, exactly as it
    /// has always run (`global_ripup_round`, unchanged): every net still
    /// unrouted is ripped up together with the blockers its last probe
    /// reported, and both are requeued at the front for the next round.
    /// Its own epoch reset fires when the global rip-up counter is exactly
    /// 2 -- the code the paper ran; whether it should instead fire at every
    /// second call, as its comment says, is open question D7 of
    /// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
    fn round_end_global_ripup(
        &mut self,
        state: &mut RoundState,
        round_failed_ids: &[u64],
        counters: &mut NegotiationCounters,
        round: u32,
        trace: bool,
    ) {
        self.global_ripup_round(
            state.batch,
            round_failed_ids,
            &state.queue.last_blockers,
            &mut state.queue.order,
            &mut state.queue.failed_counts,
            &mut state.queue.ripped_once,
            &mut counters.global_ripups,
            round,
            trace,
        );
    }

    /// The line that opens every net's turn.
    fn trace_net_start(&self, position: usize, round: u32, net_id: u64, failed: u32, free: bool) {
        eprintln!(
            "{}native_negotiated_net index={} round={} net={} failed_count={} crossing_free={}",
            trace_t(self.negotiated_batch_start),
            position + 1,
            round,
            net_id,
            failed,
            u8::from(free)
        );
    }

    /// The batch's closing line: how the loop ended and what its rules did.
    fn trace_negotiated_done(&self, counters: &NegotiationCounters, round: u32, unrouted: usize) {
        eprintln!(
            "native_negotiated_done t={:.1} rounds={} global_ripups={} local_ripups={} commit_rejected={} unrouted={} probe_guided={} crossing_free={}",
            self.negotiated_batch_start
                .map(|start| start.elapsed().as_secs_f64())
                .unwrap_or(0.0),
            round,
            counters.global_ripups,
            counters.local_ripups,
            counters.commit_rejected,
            unrouted,
            counters.probe_guided,
            counters.crossing_free
        );
    }

    /// Both non-error returns of the loop: the per-batch fields it set are
    /// cleared and the Python result dictionary is built. (The one `?`
    /// return, a direct-crossing error, propagates without clearing, as it
    /// always has.)
    fn finish_negotiated_batch(
        &mut self,
        py: Python<'_>,
        tables: &NetTables,
        batch: &mut RepairBatchState,
        args: &SearchArgs,
    ) -> PyResult<PyObject> {
        self.long_straight_weight_override = None;
        self.negotiated_batch_start = None;
        self.build_native_batch_result_dict(py, &tables.jobs, batch, args.collect_native_timing)
    }

    /// The negotiated rip-up-and-repair loop (Milestone 5 of
    /// `.agent/execplans/2026-08-25-negotiated-repair-engine.md`, rebuilt
    /// LiDAR-style in
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`):
    /// route the nets in the order given; when one fails, ask the probe who
    /// is blocking it, try the two crossing-accepting repairs, rip up what
    /// the rip-up policy names and search once more; requeue whoever did
    /// not resolve and iterate until every net routes or `max_rounds` is
    /// exhausted. Each rule lives in its own module: the budget ladder in
    /// `budget.rs`, the crossing-free rule in `crossing_free.rs`, the
    /// rip-up rule in `ripup.rs`, the queue and its bookkeeping in
    /// `queue.rs`, one attempt and its trace line in `attempt.rs`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_many_with_negotiated_repair_and_commit_impl(
        &mut self,
        py: Python<'_>,
        jobs: Vec<PyRouteJobTuple>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        max_rounds: u32,
        history_weight: f64,
        history_increment: u32,
    ) -> PyResult<PyObject> {
        self.obstacle_map.clear_congestion();
        self.obstacle_map.clear_history();
        self.long_straight_congestion_cells.clear();
        self.long_straight_congestion_records.clear();
        self.commit_history_increment = history_increment;
        self.commit_history_block_radius_cells = block_radius_cells;
        // Timestamps this loop's `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG`
        // trace lines; cleared before every return below.
        self.negotiated_batch_start = Some(Instant::now());
        let args = SearchArgs {
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
            collect_native_timing: self.astar_cfg.collect_detailed_timing,
            trace: self.router_config.diagnostics.native_repair_diag,
        };
        let mut batch = empty_batch_state();
        let tables = NetTables::from_py_jobs(jobs);
        // The policies. The configuration is cloned because no step of the
        // loop changes it and `self` must stay free for the searches.
        let negotiation = self.router_config.negotiation.clone();
        let budgets = LadderBudgets(&negotiation);
        let crossing_free_policy = self.negotiated_crossing_free_policy(&negotiation);
        // This loop has always ripped up LiDAR-style; `NoRipUp` is the
        // no-repair mode, which reaches this loop only once Milestone 8
        // re-points it here.
        let ripup_policy: Box<dyn RipUpPolicy> = Box::new(LidarStyleRipUp);

        let mut queue = NetQueue::new(tables.jobs.iter().map(|job| job.net_id));
        let mut counters = NegotiationCounters::default();
        let mut braid_failed_pairs: FxHashSet<(u64, u64)> = FxHashSet::default();
        let mut round = 0u32;
        let mut rounds_without_progress = 0u32;
        const RESET_AFTER_ROUNDS_WITHOUT_PROGRESS: u32 = 2;

        while round < max_rounds && !queue.is_empty() {
            round += 1;
            braid_failed_pairs.clear();
            let this_round = queue.take_round();
            let mut state = RoundState {
                batch: &mut batch,
                queue: &mut queue,
                braid_failed_pairs: &mut braid_failed_pairs,
            };
            let mut made_progress = false;
            // Every net that ends this round still unrouted, whether or
            // not a rip-up was attempted for it.
            let mut round_failed_ids: Vec<u64> = Vec::new();

            for (position, net_id) in this_round.into_iter().enumerate() {
                let job = tables
                    .job_by_id
                    .get(&net_id)
                    .expect("every queued net_id came from job_by_id's own keys")
                    .clone();
                let failed_count = state.queue.failed_count(net_id);
                let budget = |kind| budgets.budget(kind, failed_count, round, max_rounds);
                let crossing_free = crossing_free_policy.crossing_free(net_id, round, max_rounds);
                // History cost steers a net that has already failed, and
                // only such a net: any nonzero weight disables the JPS4
                // fast path for *every* search (`astar.rs`).
                let failed_before = failed_count != 0;
                self.commit_history_weight = if failed_before { history_weight } else { 0.0 };
                // 0 for the exempt dense fan-out nets, the configured
                // weight for every other net.
                let exempt = self.long_straight_exempt_net_ids.contains(&net_id);
                self.long_straight_weight_override = if exempt { Some(0.0) } else { None };
                if args.trace {
                    self.trace_net_start(position, round, net_id, failed_count, crossing_free);
                }

                // The plain search, and -- for a net that has not failed
                // yet, outside the last round -- one retry at the larger
                // first-retry budget before probe and rip-up run at all
                // (owner decision 2026-09-15 01:30).
                let first = PlainAttempt::first(budget(AttemptKind::First), crossing_free);
                let mut outcome = self.run_plain_attempt(state.batch, &job, &args, first);
                if matches!(outcome, PlainRouteOutcome::NotResolved)
                    && failed_count == 0
                    && round != max_rounds
                {
                    let retry_budget = budget(AttemptKind::FirstRetry);
                    let retry = PlainAttempt::first_retry(retry_budget, crossing_free);
                    outcome = self.run_plain_attempt(state.batch, &job, &args, retry);
                }
                if matches!(outcome, PlainRouteOutcome::Routed) {
                    counters.crossing_free += u32::from(crossing_free);
                    self.braid_and_requeue(&mut state, &job, &tables, &args);
                    made_progress = true;
                    continue;
                }

                // The probe is diagnostic only and is never itself
                // committed here; a probe failure aborts the whole batch,
                // this loop's only early return.
                let jobs_by_order = &tables.order_by_id;
                let Ok(probe) = self.run_probe_attempt(state.batch, &job, jobs_by_order, &args)
                else {
                    if args.trace {
                        self.trace_negotiated_done(&counters, round, state.queue.len());
                    }
                    return self.finish_negotiated_batch(py, &tables, state.batch, &args);
                };
                let blockers = probe_blockers(&probe, crossing_free);
                state.queue.record_blockers(net_id, blockers);
                let (legal_partner_ids, illegal_partner_ids) =
                    loop_partner_split(&probe, crossing_free);

                // Probe-guided search (2026-09-15 15:00 Decision Log entry
                // of the LiDAR endgame plan): every crossing on the probe's
                // free path is legal, so one more search prices exactly
                // those partners instead of paying the crossing price. The
                // mixed case is handled after the rip-up below.
                if illegal_partner_ids.is_empty() && !legal_partner_ids.is_empty() {
                    let partner_ids = self.widened_probe_partners(
                        &legal_partner_ids,
                        &tables,
                        state.batch,
                        net_id,
                        args.trace,
                    );
                    let guided_budget = budget(AttemptKind::ProbeGuided);
                    let guided = PlainAttempt::probe_guided(guided_budget, &partner_ids);
                    let guided_outcome = self.run_plain_attempt(state.batch, &job, &args, guided);
                    if matches!(guided_outcome, PlainRouteOutcome::Routed) {
                        counters.probe_guided += 1;
                        self.braid_and_requeue(&mut state, &job, &tables, &args);
                        made_progress = true;
                        continue;
                    }
                }

                // A legal crossing against the partners the probe itself
                // blamed for an illegal one (at most three).
                if !illegal_partner_ids.is_empty() {
                    let direct_crossing_outcome = self.run_direct_crossing_attempt(
                        state.batch,
                        &job,
                        &tables.order_by_id,
                        &args,
                        budget(AttemptKind::DirectCrossing),
                        &illegal_partner_ids,
                    )?;
                    if matches!(direct_crossing_outcome, LidarDirectCrossingOutcome::Routed) {
                        self.braid_and_requeue(&mut state, &job, &tables, &args);
                        made_progress = true;
                        continue;
                    }
                }

                // LiDAR's local rip-up rule (`ripuplocalnets`), then this
                // net's search once more on the map it freed.
                let ripped = ripup_policy.victims(
                    self,
                    state.batch,
                    &probe,
                    &mut state.queue.ripped_once,
                    net_id,
                    round,
                    args.trace,
                    crossing_free,
                );
                if ripped.is_empty() {
                    round_failed_ids.push(net_id);
                    state.queue.requeue_back(net_id);
                    continue;
                }
                counters.local_ripups += 1;
                // Mixed-case guided search (2026-09-15 16:25 Decision Log
                // entry, follow-up (2)): when the probe also reported legal
                // partners, this search runs under the same widened
                // guidance as the legal-only step above.
                let guided_partner_ids = if legal_partner_ids.is_empty() {
                    Vec::new()
                } else {
                    self.widened_probe_partners(
                        &legal_partner_ids,
                        &tables,
                        state.batch,
                        net_id,
                        args.trace,
                    )
                };
                let after = budget(AttemptKind::PostRipUp);
                let post = PlainAttempt::post_ripup(after, crossing_free, &guided_partner_ids);
                let post_outcome = self.run_plain_attempt(state.batch, &job, &args, post);
                if matches!(post_outcome, PlainRouteOutcome::Routed) {
                    counters.probe_guided += u32::from(!guided_partner_ids.is_empty());
                    counters.crossing_free += u32::from(crossing_free);
                    self.braid_and_requeue(&mut state, &job, &tables, &args);
                    made_progress = true;
                } else {
                    round_failed_ids.push(net_id);
                    state.queue.requeue_back(net_id);
                }
                state.queue.requeue_front_preserving_order(&ripped);
            }

            let failed = &round_failed_ids;
            self.round_end_global_ripup(&mut state, failed, &mut counters, round, args.trace);

            // Two rounds without a single committed route: the loop's own
            // epoch boundary, independent of the global rip-up's.
            if made_progress {
                rounds_without_progress = 0;
            } else {
                rounds_without_progress += 1;
                if rounds_without_progress >= RESET_AFTER_ROUNDS_WITHOUT_PROGRESS
                    && !queue.is_empty()
                {
                    self.obstacle_map.clear_history();
                    queue.epoch_reset(true);
                    rounds_without_progress = 0;
                }
            }
        }

        // The soft failure: the batch is returned with whatever routed.
        if let Some(failed_net_id) = queue.front() {
            batch.failed_net_id = Some(failed_net_id);
            batch.failed_error = Some(format!(
                "Negotiated repair did not converge for net {failed_net_id} within {max_rounds} rounds; {} net(s) still unrouted",
                queue.len()
            ));
        }
        if args.trace {
            self.trace_negotiated_done(&counters, round, queue.len());
        }
        self.finish_negotiated_batch(py, &tables, &mut batch, &args)
    }
}
