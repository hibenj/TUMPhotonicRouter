use rustc_hash::{FxHashMap, FxHashSet};

use crate::engine::*;

impl PyPhotonicRouter {
    /// `keep_net_on_victim_failure`: when the net routes without the victim
    /// but the victim cannot be rerouted afterwards, keep the net's new
    /// route committed and leave the victim ripped (`NetOnly`) instead of
    /// rolling both back -- the braid escalation of the negotiated engine.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn try_ripup_single_victim_and_reroute_with(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        victim_job: &NativeRouteJob,
        bucket_name: &'static str,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        history_weight: f64,
        collect_native_timing: bool,
        keep_net_on_victim_failure: bool,
    ) -> VictimRerouteOutcome {
        let victim_id = victim_job.net_id;
        let base_map = self.obstacle_map.clone();
        let base_center_routes = self.committed_center_routes.clone();
        let base_realized_center_routes = self.committed_realized_center_routes.clone();
        let base_target_terminal_bump_guards = self.committed_target_terminal_bump_guards.clone();
        let base_opened_cell_keys = self.committed_opened_cell_keys.clone();
        let base_crossing_events = self.crossing_events.clone();
        let base_routes = batch.final_routes.clone();

        let ripup_start = native_batch_timer(collect_native_timing);
        self.remove_crossing_events_for_net(victim_id);
        self.obstacle_map.ripup_route(victim_id);
        self.committed_center_routes.remove(&victim_id);
        self.committed_realized_center_routes.remove(&victim_id);
        self.committed_target_terminal_bump_guards
            .remove(&victim_id);
        self.committed_opened_cell_keys.remove(&victim_id);
        batch.final_routes.remove(&victim_id);
        batch.timings.ripup_us += native_batch_elapsed_us(ripup_start);

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
        let route_elapsed_us = native_batch_elapsed_us(route_start);
        batch.timings.repair_failed_net_wall_us += route_elapsed_us;
        let current_route = match route_result {
            Ok(route) => {
                batch
                    .timings
                    .add_route_result_stats_if(collect_native_timing, &route);
                remove_success_static_cleanup(&mut self.obstacle_map, job);
                route
            }
            Err(error) => {
                batch.timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name,
                    net_id: job.net_id,
                    route: None,
                    failed: true,
                    error: Some(error),
                    repair_round: Some(0),
                    candidate_blockers: vec![victim_id],
                    ripup_ids: vec![victim_id],
                });
                self.obstacle_map = base_map;
                self.committed_center_routes = base_center_routes;
                self.committed_realized_center_routes = base_realized_center_routes;
                self.committed_target_terminal_bump_guards = base_target_terminal_bump_guards;
                self.committed_opened_cell_keys = base_opened_cell_keys;
                self.crossing_events = base_crossing_events;
                batch.final_routes = base_routes;
                self.invalidate_meander_base_prefix();
                return VictimRerouteOutcome::Failed;
            }
        };

        let reroute_start = native_batch_timer(collect_native_timing);
        let reroute_result = self.route_single_net_and_commit_native(
            victim_job.net_id,
            victim_job.source,
            victim_job.target,
            block_radius_cells,
            Some(&victim_job.opened_cells),
            Some(&victim_job.opened_cell_keys),
            commit_radius_cells,
            Some(&victim_job.clearance_exempt_cells),
            Some(&victim_job.clearance_exempt_cell_keys),
            core_radius_cells,
            victim_job.source_port_um,
            victim_job.target_port_um,
        );
        let reroute_elapsed_us = native_batch_elapsed_us(reroute_start);
        batch.timings.reroute_victims_wall_us += reroute_elapsed_us;
        let victim_route = match reroute_result {
            Ok(route) => {
                batch
                    .timings
                    .add_route_result_stats_if(collect_native_timing, &route);
                remove_success_static_cleanup(&mut self.obstacle_map, victim_job);
                route
            }
            Err(normal_error) => {
                batch.timings.reroute_victims_failed_wall_us += reroute_elapsed_us;
                let repair_start = native_batch_timer(collect_native_timing);
                let repair_result = self.route_single_net_and_commit_repair_native(
                    victim_job.net_id,
                    victim_job.source,
                    victim_job.target,
                    block_radius_cells,
                    Some(&victim_job.opened_cells),
                    Some(&victim_job.opened_cell_keys),
                    history_weight,
                    commit_radius_cells,
                    Some(&victim_job.clearance_exempt_cells),
                    Some(&victim_job.clearance_exempt_cell_keys),
                    core_radius_cells,
                    victim_job.source_port_um,
                    victim_job.target_port_um,
                );
                let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                batch.timings.reroute_victims_wall_us += repair_elapsed_us;
                match repair_result {
                    Ok(route) => {
                        batch
                            .timings
                            .add_route_result_stats_if(collect_native_timing, &route);
                        remove_success_static_cleanup(&mut self.obstacle_map, victim_job);
                        route
                    }
                    Err(error) => {
                        batch.timings.reroute_victims_failed_wall_us += repair_elapsed_us;
                        if self.router_config.diagnostics.native_repair_diag {
                            eprintln!(
                                "{}native_repair_victim_reroute_failed bucket={} net={} victim={} net_waypoints={:?} normal_error={} repair_error={}",
                                trace_t(self.negotiated_batch_start),
                                bucket_name,
                                job.net_id,
                                victim_id,
                                current_route.compressed_waypoints,
                                normal_error,
                                error
                            );
                        }
                        batch.attempts.push(NativeRouteAttempt {
                            bucket_name,
                            net_id: victim_job.net_id,
                            route: None,
                            failed: true,
                            error: Some(format!("{normal_error}; repair fallback: {error}")),
                            repair_round: Some(0),
                            candidate_blockers: vec![victim_id],
                            ripup_ids: vec![victim_id],
                        });
                        if keep_net_on_victim_failure {
                            batch.attempts.push(NativeRouteAttempt {
                                bucket_name,
                                net_id: job.net_id,
                                route: Some(current_route.clone()),
                                failed: false,
                                error: None,
                                repair_round: Some(0),
                                candidate_blockers: vec![victim_id],
                                ripup_ids: vec![victim_id],
                            });
                            batch.final_routes.insert(job.net_id, current_route);
                            batch.repair_count = batch.repair_count.saturating_add(1);
                            return VictimRerouteOutcome::NetOnly;
                        }
                        self.obstacle_map = base_map;
                        self.committed_center_routes = base_center_routes;
                        self.committed_realized_center_routes = base_realized_center_routes;
                        self.committed_target_terminal_bump_guards =
                            base_target_terminal_bump_guards;
                        self.committed_opened_cell_keys = base_opened_cell_keys;
                        self.crossing_events = base_crossing_events;
                        batch.final_routes = base_routes;
                        self.invalidate_meander_base_prefix();
                        return VictimRerouteOutcome::Failed;
                    }
                }
            }
        };

        batch.attempts.push(NativeRouteAttempt {
            bucket_name,
            net_id: job.net_id,
            route: Some(current_route.clone()),
            failed: false,
            error: None,
            repair_round: Some(0),
            candidate_blockers: vec![victim_id],
            ripup_ids: vec![victim_id],
        });
        batch.attempts.push(NativeRouteAttempt {
            bucket_name,
            net_id: victim_job.net_id,
            route: Some(victim_route.clone()),
            failed: false,
            error: None,
            repair_round: Some(0),
            candidate_blockers: vec![victim_id],
            ripup_ids: vec![victim_id],
        });
        batch.final_routes.insert(job.net_id, current_route);
        batch.final_routes.insert(victim_job.net_id, victim_route);
        batch.repair_count = batch.repair_count.saturating_add(1);
        VictimRerouteOutcome::Both
    }

    /// Number of crossing events between two nets in the committed state.
    pub(crate) fn crossing_event_count_between(&self, a: u64, b: u64) -> usize {
        self.crossing_events
            .iter()
            .filter(|event| {
                (event.net_id == a && event.partner_net_id == b)
                    || (event.net_id == b && event.partner_net_id == a)
            })
            .count()
    }

    /// Braid repair (2026-09-04, multiportmmi_32x32 first layer vs LiDAR):
    /// a net that crosses the SAME partner twice swapped sides and swapped
    /// back -- never necessary; it means the pair was routed in the wrong
    /// order (the earlier net's greedy shortest path sealed the later net's
    /// target pocket, or laid its axial run where the later net had to
    /// pass). Fix the order locally: rip up both, route this net first, then
    /// the partner; keep the result only if the crossings between the two
    /// went down and both nets routed, otherwise restore. One bounded
    /// reroute per braid, no geometry assumption.
    ///
    /// `skip_pairs`, when set, is the negotiated loop's once-per-round set
    /// of unordered `{net, partner}` pairs whose most recent braid attempt
    /// ended with `keep=false`: a pair already in it is skipped outright
    /// (`None`, before any snapshot/rip-up/trace), so the same doomed pair
    /// is not retried every time either net routes again in the same
    /// round. The older repair chain's own call site passed `None`; that
    /// chain was deleted in Milestone 8 of
    /// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
    ///
    /// Returns `None` when no eligible partner was found (braid repair
    /// disabled, no partner with >=2 crossings, or the pair was skipped);
    /// otherwise `Some((partner_id, kept))` reporting which partner was
    /// tried and whether the swapped result was kept.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn try_braid_repair(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        job_by_id: &FxHashMap<u64, NativeRouteJob>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
        skip_pairs: Option<&FxHashSet<(u64, u64)>>,
        escalate_victim: bool,
    ) -> Option<BraidRepairOutcome> {
        if self.router_config.negotiation.disable_braid_repair {
            return None;
        }
        let mut counts: FxHashMap<u64, usize> = FxHashMap::default();
        for event in &self.crossing_events {
            let partner = if event.net_id == job.net_id {
                event.partner_net_id
            } else if event.partner_net_id == job.net_id {
                event.net_id
            } else {
                continue;
            };
            *counts.entry(partner).or_insert(0) += 1;
        }
        // Pick the most-braided partner that is committed and not already
        // skipped for this round: filtering BEFORE the max lets the caller's
        // bounded loop (`run_braid_repair_passes`) reach the next braided
        // partner after one pair failed, instead of re-selecting the failed
        // top pair forever (found 2026-09-15 on multiportmmi_64x64_bands8to9:
        // net 526 braided three neighbours and only one was repaired).
        let Some((&victim_id, &before)) = counts
            .iter()
            .filter(|(_, count)| **count >= 2)
            .filter(|(partner, _)| batch.final_routes.contains_key(*partner))
            .filter(|(partner, _)| {
                let pair_key = (job.net_id.min(**partner), job.net_id.max(**partner));
                !skip_pairs.is_some_and(|pairs| pairs.contains(&pair_key))
            })
            .max_by_key(|(partner, count)| (**count, std::cmp::Reverse(**partner)))
        else {
            return None;
        };
        let Some(victim_job) = job_by_id.get(&victim_id) else {
            return None;
        };
        if trace_native_repair {
            eprintln!(
                "native_repair_braid_start net={} partner={} crossings_between={} budget={}",
                job.net_id,
                victim_id,
                before,
                trace_budget_str(self.negotiated_search_budget)
            );
            // Braid geometry (2026-09-15 fan-out analysis): the crossing
            // points between the two nets and both routes' waypoints.
            let points: Vec<(f64, f64)> = self
                .crossing_events
                .iter()
                .filter(|event| {
                    (event.net_id == job.net_id && event.partner_net_id == victim_id)
                        || (event.net_id == victim_id && event.partner_net_id == job.net_id)
                })
                .map(|event| event.point)
                .collect();
            let net_waypoints = batch
                .final_routes
                .get(&job.net_id)
                .map(|route| route.compressed_waypoints.clone())
                .unwrap_or_default();
            let partner_waypoints = batch
                .final_routes
                .get(&victim_id)
                .map(|route| route.compressed_waypoints.clone())
                .unwrap_or_default();
            eprintln!(
                "native_repair_braid_geometry net={} partner={} points={:?} net_waypoints={:?} partner_waypoints={:?}",
                job.net_id, victim_id, points, net_waypoints, partner_waypoints
            );
            // Every other committed route inside the pair's bounding box.
            let bbox = net_waypoints
                .iter()
                .chain(partner_waypoints.iter())
                .fold((i32::MAX, i32::MAX, i32::MIN, i32::MIN), |acc, &(x, y)| {
                    (acc.0.min(x), acc.1.min(y), acc.2.max(x), acc.3.max(y))
                });
            let mut neighbours: Vec<(u64, Vec<(i32, i32)>)> = batch
                .final_routes
                .iter()
                .filter(|(id, _)| **id != job.net_id && **id != victim_id)
                .filter(|(_, route)| {
                    route.compressed_waypoints.iter().any(|&(x, y)| {
                        x >= bbox.0 - 8 && x <= bbox.2 + 8 && y >= bbox.1 - 8 && y <= bbox.3 + 8
                    })
                })
                .map(|(id, route)| (*id, route.compressed_waypoints.clone()))
                .collect();
            neighbours.sort_by_key(|(id, _)| *id);
            eprintln!(
                "native_repair_braid_neighbours net={} partner={} bbox={:?} neighbours={:?}",
                job.net_id, victim_id, bbox, neighbours
            );
        }
        // snapshot with both nets committed
        let base_map = self.obstacle_map.clone();
        let base_center_routes = self.committed_center_routes.clone();
        let base_realized_center_routes = self.committed_realized_center_routes.clone();
        let base_target_terminal_bump_guards = self.committed_target_terminal_bump_guards.clone();
        let base_opened_cell_keys = self.committed_opened_cell_keys.clone();
        let base_crossing_events = self.crossing_events.clone();
        let base_routes = batch.final_routes.clone();
        let base_attempts = batch.attempts.len();

        // rip up this net; the helper rips up the partner, routes this net
        // first and the partner second (restoring itself on failure)
        self.rollback_committed_route(job.net_id);
        batch.final_routes.remove(&job.net_id);
        let reroute_outcome = self.try_ripup_single_victim_and_reroute_with(
            batch,
            job,
            victim_job,
            "braid_ripup",
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
            0.0,
            collect_native_timing,
            escalate_victim,
        );
        let swapped = reroute_outcome == VictimRerouteOutcome::Both;
        let after = self.crossing_event_count_between(job.net_id, victim_id);
        let keep = swapped
            && batch.final_routes.contains_key(&job.net_id)
            && batch.final_routes.contains_key(&victim_id)
            && after < before;
        // Escalation: the net holds a braid-free route, the partner could
        // not be rerouted around it -- keep the net, hand the partner back
        // to the negotiation (its own blockers get ripped there) instead of
        // rolling back to the braid. Only if the net's new route really
        // lost the braid; otherwise fall through to the rollback.
        let escalated = reroute_outcome == VictimRerouteOutcome::NetOnly
            && batch.final_routes.contains_key(&job.net_id)
            && !batch.final_routes.contains_key(&victim_id)
            && after == 0;
        if trace_native_repair {
            eprintln!(
                "native_repair_braid_result net={} partner={} swapped={} crossings_between={}->{} keep={} escalated={}",
                job.net_id, victim_id, swapped, before, after, keep, escalated
            );
        }
        if keep {
            return Some(BraidRepairOutcome::Kept(victim_id));
        }
        if escalated {
            return Some(BraidRepairOutcome::VictimRipped(victim_id));
        }
        self.obstacle_map = base_map;
        self.committed_center_routes = base_center_routes;
        self.committed_realized_center_routes = base_realized_center_routes;
        self.committed_target_terminal_bump_guards = base_target_terminal_bump_guards;
        self.committed_opened_cell_keys = base_opened_cell_keys;
        self.crossing_events = base_crossing_events;
        batch.final_routes = base_routes;
        batch.attempts.truncate(base_attempts);
        self.invalidate_meander_base_prefix();
        Some(BraidRepairOutcome::Failed(victim_id))
    }

    /// Loops [`Self::try_braid_repair`] over every braided partner of one
    /// commit, not just the first (2026-09-15 16:25 Decision Log entry of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`,
    /// "Slice result 16:25" paragraph): `try_braid_repair` finds and treats
    /// one partner crossed twice or more per call, so a net double-crossing
    /// several committed neighbours after a single commit (net 80 against
    /// three neighbours, only one repaired) needs more than one call. Each
    /// pass either repairs a partner (`Some((_, true))` -- another partner
    /// may still be braided, so the loop tries again) or fails one
    /// (`Some((partner, false))` -- inserted into `braid_failed_pairs` so
    /// `try_braid_repair`'s own skip check will not pick the same doomed
    /// pair again, and the loop continues since a different partner may
    /// still be eligible) or finds nothing left to try (`None` -- stop).
    /// Bounded by `NEGOTIATED_BRAID_MAX_PASSES` regardless, so one commit
    /// can never turn into an unbounded chain of reroute searches. Returns
    /// the number of passes that ended `keep=true`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn run_braid_repair_passes(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        job_by_id: &FxHashMap<u64, NativeRouteJob>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
        braid_failed_pairs: &mut FxHashSet<(u64, u64)>,
        ripped_victims: &mut Vec<u64>,
    ) -> u32 {
        let mut kept_count = 0u32;
        let escalate = self.router_config.negotiation.braid_escalation;
        self.negotiated_search_budget = Some(NEGOTIATED_BUDGET_BRAID);
        for _ in 0..NEGOTIATED_BRAID_MAX_PASSES {
            let braid_outcome = self.try_braid_repair(
                batch,
                job,
                job_by_id,
                block_radius_cells,
                commit_radius_cells,
                core_radius_cells,
                collect_native_timing,
                trace_native_repair,
                Some(braid_failed_pairs),
                escalate,
            );
            match braid_outcome {
                None => break,
                Some(BraidRepairOutcome::Kept(_)) => kept_count += 1,
                Some(BraidRepairOutcome::Failed(partner_id)) => {
                    braid_failed_pairs
                        .insert((job.net_id.min(partner_id), job.net_id.max(partner_id)));
                }
                Some(BraidRepairOutcome::VictimRipped(partner_id)) => {
                    ripped_victims.push(partner_id);
                }
            }
        }
        self.negotiated_search_budget = None;
        kept_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;

    /// Once-per-round rule (2026-09-15 11:54 trace finding): a pair
    /// already in `skip_pairs` must be skipped outright, before any
    /// snapshot/rip-up/search -- the same doomed `{net, partner}` pair
    /// must not cost another reroute attempt every time either net routes
    /// again in the same round. `try_braid_repair` only needs the victim's
    /// id present in `batch.final_routes` (membership, not the stored
    /// route -- the same convention `probe_net_for_repair` uses, see the
    /// comment in
    /// `crossing_conflict_fixture_blocks_the_vertical_net_with_a_non_perpendicular_crossing`
    /// above) and >=2 crossing events between `job.net_id` and the victim
    /// to pick that victim; both are fabricated directly here instead of
    /// driving two nets into a real braid geometry, since the skip check
    /// fires before either the rip-up or the reroute searches ever run.
    #[test]
    fn try_braid_repair_skips_a_pair_already_in_skip_pairs() {
        let (mut router, jobs) = crossing_conflict_fixture();
        let diagonal = jobs[2].clone();
        assert_eq!(diagonal.net_id, 3);
        let vertical = jobs[3].clone();
        assert_eq!(vertical.net_id, 4);
        let job_by_id: FxHashMap<u64, NativeRouteJob> =
            jobs.iter().cloned().map(|job| (job.net_id, job)).collect();

        let mut batch = RepairBatchState {
            final_routes: FxHashMap::default(),
            attempts: Vec::new(),
            repair_trace: Vec::new(),
            repair_count: 0,
            failed_net_id: None,
            failed_error: None,
            timings: NativeBatchTimings::default(),
            deferred_count: 0,
        };
        batch
            .final_routes
            .insert(diagonal.net_id, empty_test_route());
        router.add_crossing_events(vec![
            CrossingEvent {
                net_id: vertical.net_id,
                partner_net_id: diagonal.net_id,
                point: (30.0, 30.0),
                route_segment: ((30, 20), (30, 40)),
                partner_segment: ((20, 20), (40, 40)),
                route_angle: 0,
                partner_angle: 1,
                reservation_keys: FxHashSet::default(),
            },
            CrossingEvent {
                net_id: vertical.net_id,
                partner_net_id: diagonal.net_id,
                point: (31.0, 31.0),
                route_segment: ((31, 20), (31, 40)),
                partner_segment: ((21, 21), (41, 41)),
                route_angle: 0,
                partner_angle: 1,
                reservation_keys: FxHashSet::default(),
            },
        ]);

        let pair_key = (
            vertical.net_id.min(diagonal.net_id),
            vertical.net_id.max(diagonal.net_id),
        );
        let mut skip_pairs: FxHashSet<(u64, u64)> = FxHashSet::default();
        skip_pairs.insert(pair_key);

        let final_routes_keys_before: Vec<u64> = {
            let mut keys: Vec<u64> = batch.final_routes.keys().copied().collect();
            keys.sort_unstable();
            keys
        };
        let attempts_before_len = batch.attempts.len();
        let crossing_events_before_len = router.crossing_events.len();

        let outcome = router.try_braid_repair(
            &mut batch,
            &vertical,
            &job_by_id,
            FIXTURE_CLEARANCE_RADIUS_CELLS,
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            false,
            true,
            Some(&skip_pairs),
            true,
        );

        assert_eq!(
            outcome, None,
            "a pair already in skip_pairs must be skipped outright, before any \
             snapshot/rip-up/search"
        );
        let final_routes_keys_after: Vec<u64> = {
            let mut keys: Vec<u64> = batch.final_routes.keys().copied().collect();
            keys.sort_unstable();
            keys
        };
        assert_eq!(
            final_routes_keys_after, final_routes_keys_before,
            "no route should be ripped up or committed when the pair is skipped"
        );
        assert_eq!(
            batch.attempts.len(),
            attempts_before_len,
            "no rip-up attempt should be recorded when the pair is skipped"
        );
        assert_eq!(
            router.crossing_events.len(),
            crossing_events_before_len,
            "crossing events must be untouched when the pair is skipped"
        );
    }

    /// 2026-09-15 16:25 Decision Log entry of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    /// (net 80 committed with double crossings against three neighbours,
    /// only one repaired by a single `try_braid_repair` call): a fixture
    /// with two committed nets (1 and 3, both borrowed from
    /// `crossing_conflict_fixture`, present in `batch.final_routes` as
    /// membership placeholders the same way
    /// `try_braid_repair_skips_a_pair_already_in_skip_pairs` above does) and
    /// a net 4 fabricated with a double crossing against each -- 2 events
    /// against net 1, 3 against net 3, so `try_braid_repair`'s own
    /// `max_by_key` selection picks net 3 first.
    ///
    /// On this fixture net 4's own reroute inside `try_braid_repair`'s
    /// swap always fails regardless of which partner is picked: `try_braid_repair`
    /// reroutes net 4 via a *plain* search (`try_ripup_single_victim_and_reroute`
    /// -> `route_single_net_and_commit_native`), and a plain search on this
    /// fixture cannot legally cross anything at all (confirmed directly:
    /// `try_plain_normal_route` for net 4 still fails with "No legal LiDAR
    /// crossing route found" even after net 3 is ripped up first, because a
    /// plain search never grants crossing permission -- only the probe-guided
    /// / direct-crossing steps in the negotiated loop do). So `keep` is
    /// always `false` here, and this test documents the resulting behaviour
    /// of `run_braid_repair_passes`'s loop rather than a successful swap:
    ///
    /// `try_braid_repair`'s candidate selection (`counts.iter().filter(...).max_by_key(...)`,
    /// filtering on a crossing count of 2 or more) picks the single
    /// best-ranked partner *before* consulting `skip_pairs` -- the skip check only vetoes that one
    /// already-selected candidate, it never falls through to the
    /// next-ranked partner. Since a failed pass restores `self.crossing_events`
    /// (via `try_braid_repair`'s own snapshot/restore), the recomputed counts
    /// are identical on every call, so the same top-ranked partner (net 3,
    /// count 3) is re-selected forever; once its pair is skip-paired, every
    /// further call returns `None` immediately without ever trying net 1
    /// (count 2, never reached). The loop therefore performs exactly one
    /// real attempt here, not two -- this is `try_braid_repair`'s
    /// pre-existing, documented selection contract (its own doc comment:
    /// "`None`... the pair was skipped"), unmodified by this change; the
    /// brief anticipated this exact fixture might not sustain a second real
    /// attempt and asked to assert on the attempt and on `braid_failed_pairs`
    /// instead, which is what this test does. Looping across *different*
    /// partners in the loop's remaining budget only happens when earlier
    /// passes actually succeed and change the real crossing data (see
    /// `run_braid_repair_passes_loops_again_after_a_kept_repair_and_stops_at_none`
    /// below for that path) -- a fixture where two *different* partners'
    /// swaps both genuinely succeed needs a real end-to-end braid geometry,
    /// which `astar_config_applies_negotiated_search_budget_when_set` above
    /// already notes is impractical to build as a unit test.
    #[test]
    fn run_braid_repair_passes_reaches_every_braided_partner_after_a_failed_pair() {
        let (mut router, jobs) = crossing_conflict_fixture();
        let horizontal_top = jobs[0].clone();
        assert_eq!(horizontal_top.net_id, 1);
        let diagonal = jobs[2].clone();
        assert_eq!(diagonal.net_id, 3);
        let vertical = jobs[3].clone();
        assert_eq!(vertical.net_id, 4);
        let job_by_id: FxHashMap<u64, NativeRouteJob> =
            jobs.iter().cloned().map(|job| (job.net_id, job)).collect();

        let mut batch = RepairBatchState {
            final_routes: FxHashMap::default(),
            attempts: Vec::new(),
            repair_trace: Vec::new(),
            repair_count: 0,
            failed_net_id: None,
            failed_error: None,
            timings: NativeBatchTimings::default(),
            deferred_count: 0,
        };
        batch
            .final_routes
            .insert(horizontal_top.net_id, empty_test_route());
        batch
            .final_routes
            .insert(diagonal.net_id, empty_test_route());

        // Net 4 vs net 1: 2 fabricated crossings (below the eligibility
        // floor's tie -- lower count than net 3's below).
        let mut events = vec![
            CrossingEvent {
                net_id: vertical.net_id,
                partner_net_id: horizontal_top.net_id,
                point: (30.0, 2.0),
                route_segment: ((30, 0), (30, 10)),
                partner_segment: ((5, 2), (54, 2)),
                route_angle: 0,
                partner_angle: 2,
                reservation_keys: FxHashSet::default(),
            },
            CrossingEvent {
                net_id: vertical.net_id,
                partner_net_id: horizontal_top.net_id,
                point: (31.0, 2.0),
                route_segment: ((31, 0), (31, 10)),
                partner_segment: ((5, 2), (54, 2)),
                route_angle: 0,
                partner_angle: 2,
                reservation_keys: FxHashSet::default(),
            },
        ];
        // Net 4 vs net 3: 3 fabricated crossings -- the strictly higher
        // count `max_by_key` always picks first.
        events.extend(vec![
            CrossingEvent {
                net_id: vertical.net_id,
                partner_net_id: diagonal.net_id,
                point: (30.0, 30.0),
                route_segment: ((30, 20), (30, 40)),
                partner_segment: ((20, 20), (40, 40)),
                route_angle: 0,
                partner_angle: 1,
                reservation_keys: FxHashSet::default(),
            },
            CrossingEvent {
                net_id: vertical.net_id,
                partner_net_id: diagonal.net_id,
                point: (31.0, 31.0),
                route_segment: ((31, 20), (31, 40)),
                partner_segment: ((21, 21), (41, 41)),
                route_angle: 0,
                partner_angle: 1,
                reservation_keys: FxHashSet::default(),
            },
            CrossingEvent {
                net_id: vertical.net_id,
                partner_net_id: diagonal.net_id,
                point: (32.0, 32.0),
                route_segment: ((32, 20), (32, 40)),
                partner_segment: ((22, 22), (42, 42)),
                route_angle: 0,
                partner_angle: 1,
                reservation_keys: FxHashSet::default(),
            },
        ]);
        router.add_crossing_events(events);

        let mut braid_failed_pairs: FxHashSet<(u64, u64)> = FxHashSet::default();
        let kept = router.run_braid_repair_passes(
            &mut batch,
            &vertical,
            &job_by_id,
            DIAGONAL_SETUP_BLOCK_RADIUS_CELLS,
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            false,
            true,
            &mut braid_failed_pairs,
            &mut Vec::new(),
        );

        assert_eq!(
            kept, 0,
            "net 4's own reroute cannot succeed via a plain search on this fixture, \
             so no pass can end keep=true"
        );
        assert_eq!(
            braid_failed_pairs,
            FxHashSet::from_iter([
                (
                    diagonal.net_id.min(vertical.net_id),
                    diagonal.net_id.max(vertical.net_id)
                ),
                (
                    horizontal_top.net_id.min(vertical.net_id),
                    horizontal_top.net_id.max(vertical.net_id)
                ),
            ]),
            "net 3 (the strictly higher fabricated count) is selected first and \
             skip-paired; since 2026-09-15 try_braid_repair filters skipped pairs \
             before choosing, so the second pass reaches net 1 as well, and the loop \
             stops when no candidate is left"
        );
    }

    /// Complements the test above with the success path: when
    /// `try_braid_repair`'s swap actually succeeds (`keep=true`), the real
    /// reroute registers no new crossing between the disjoint job/victim
    /// pair built here, so the fabricated double-crossing count drops to 0
    /// and `run_braid_repair_passes` loops again -- finding nothing left
    /// (the fabricated events were already cleared by the first pass's own
    /// `rollback_committed_route`, and the real reroute created none), it
    /// stops cleanly on the next call's `None` rather than exhausting
    /// `NEGOTIATED_BRAID_MAX_PASSES`.
    #[test]
    fn run_braid_repair_passes_loops_again_after_a_kept_repair_and_stops_at_none() {
        let grid = PyGridSpec::new(60, 60, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
            None,
        );

        let mut setup_batch = RepairBatchState {
            final_routes: FxHashMap::default(),
            attempts: Vec::new(),
            repair_trace: Vec::new(),
            repair_count: 0,
            failed_net_id: None,
            failed_error: None,
            timings: NativeBatchTimings::default(),
            deferred_count: 0,
        };
        // A committed net far from job's own route below -- neither
        // geometrically overlaps nor needs to cross the other, so both
        // reroute cleanly with a plain search once the swap runs them.
        let victim = NativeRouteJob::new(
            1,
            PyState::new(5, 5, 0),
            PyState::new(20, 5, 0),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
        let outcome =
            router.try_plain_normal_route(&mut setup_batch, &victim, 1, Some(1), Some(1), false);
        assert!(
            matches!(outcome, PlainRouteOutcome::Routed),
            "fixture setup: victim net must route/commit cleanly: {:?}",
            setup_batch.attempts.last().and_then(|a| a.error.clone())
        );

        let job = NativeRouteJob::new(
            4,
            PyState::new(5, 30, 0),
            PyState::new(20, 30, 0),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
        let job_by_id: FxHashMap<u64, NativeRouteJob> = [victim.clone(), job.clone()]
            .into_iter()
            .map(|j| (j.net_id, j))
            .collect();

        let mut batch = RepairBatchState {
            final_routes: FxHashMap::default(),
            attempts: Vec::new(),
            repair_trace: Vec::new(),
            repair_count: 0,
            failed_net_id: None,
            failed_error: None,
            timings: NativeBatchTimings::default(),
            deferred_count: 0,
        };
        batch.final_routes.insert(victim.net_id, empty_test_route());

        router.add_crossing_events(vec![
            CrossingEvent {
                net_id: job.net_id,
                partner_net_id: victim.net_id,
                point: (0.0, 0.0),
                route_segment: ((0, 0), (1, 1)),
                partner_segment: ((0, 0), (1, 1)),
                route_angle: 0,
                partner_angle: 0,
                reservation_keys: FxHashSet::default(),
            },
            CrossingEvent {
                net_id: job.net_id,
                partner_net_id: victim.net_id,
                point: (1.0, 1.0),
                route_segment: ((1, 1), (2, 2)),
                partner_segment: ((1, 1), (2, 2)),
                route_angle: 0,
                partner_angle: 0,
                reservation_keys: FxHashSet::default(),
            },
        ]);

        let attempts_before = batch.attempts.len();
        let mut braid_failed_pairs: FxHashSet<(u64, u64)> = FxHashSet::default();
        let kept = router.run_braid_repair_passes(
            &mut batch,
            &job,
            &job_by_id,
            1,
            Some(1),
            Some(1),
            false,
            true,
            &mut braid_failed_pairs,
            &mut Vec::new(),
        );

        assert_eq!(
            kept, 1,
            "the disjoint job/victim pair reroutes cleanly, so the one real \
             braided partner must be kept"
        );
        assert!(
            braid_failed_pairs.is_empty(),
            "a kept repair must never be recorded as a failed pair"
        );
        assert_eq!(
            batch.attempts.len(),
            attempts_before + 2,
            "the kept pass records both the job's and the victim's real reroute as \
             braid_ripup attempts; the loop's second call finds nothing left \
             (None) and adds nothing more"
        );
        assert!(batch.final_routes.contains_key(&job.net_id));
        assert!(batch.final_routes.contains_key(&victim.net_id));
    }
}
