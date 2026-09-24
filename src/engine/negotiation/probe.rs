use rustc_hash::{FxHashMap, FxHashSet};

use crate::search::state::RouteResult;

use crate::engine::*;

/// Widens a probe-guided search's priced-at-zero partner set structurally
/// (2026-09-15 15:00 Decision Log entry of
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`,
/// "Slice result 15:40" paragraph): `probe_partners` alone crosses only
/// some lanes of a fan-out bundle a real route must cross (the probe's
/// free path, routed with every committed net removed, can thread between
/// the rest), so the search keeps sweeping the unpriced remainder. Returns
/// `probe_partners` unioned with every other committed net (present in
/// `committed`) whose job shares `source.x` with any partner in
/// `probe_partners`, excluding `net_id` itself -- sorted and deduplicated.
pub(crate) fn widen_probe_partners_by_source_column(
    probe_partners: &[u64],
    job_by_id: &FxHashMap<u64, NativeRouteJob>,
    source_layer_indices_by_x: &FxHashMap<i32, Vec<usize>>,
    native_jobs: &[NativeRouteJob],
    committed: &FxHashMap<u64, RouteResult>,
    net_id: u64,
) -> Vec<u64> {
    let mut widened: FxHashSet<u64> = probe_partners.iter().copied().collect();
    for &partner_id in probe_partners {
        let Some(partner_job) = job_by_id.get(&partner_id) else {
            continue;
        };
        let Some(column_indices) = source_layer_indices_by_x.get(&partner_job.source.x) else {
            continue;
        };
        for &index in column_indices {
            let Some(column_job) = native_jobs.get(index) else {
                continue;
            };
            let column_net_id = column_job.net_id;
            if column_net_id == net_id {
                continue;
            }
            if committed.contains_key(&column_net_id) {
                widened.insert(column_net_id);
            }
        }
    }
    let mut result: Vec<u64> = widened.into_iter().collect();
    result.sort_unstable();
    result
}

/// Splits a probe's crossing partners into legal and illegal sets, so the
/// negotiated loop's legal-only probe-guided step and its mixed-case
/// post-rip-up guided step (2026-09-15 16:25 Decision Log entry of
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`,
/// "Slice result 16:25" paragraph, follow-up (2)) agree on what counts as
/// "legal" for the same probe. `illegal` is the deduplicated partner ids of
/// `probe.probe_realized_crossing_violations`, in first-appearance order;
/// `legal` is the deduplicated partner ids of `probe.probe_crossing_events`
/// that are not in `illegal`, also in first-appearance order -- a partner
/// crossed both legally and illegally (two different crossing points) is
/// illegal only, never both.
pub(crate) fn split_probe_partners(probe: &ProbeState) -> (Vec<u64>, Vec<u64>) {
    let mut illegal: Vec<u64> = Vec::new();
    let mut illegal_seen: FxHashSet<u64> = FxHashSet::default();
    for violation in &probe.probe_realized_crossing_violations {
        if illegal_seen.insert(violation.partner_net_id) {
            illegal.push(violation.partner_net_id);
        }
    }
    let mut legal: Vec<u64> = Vec::new();
    let mut legal_seen: FxHashSet<u64> = FxHashSet::default();
    for event in &probe.probe_crossing_events {
        let partner = event.partner_net_id;
        if illegal_seen.contains(&partner) {
            continue;
        }
        if legal_seen.insert(partner) {
            legal.push(partner);
        }
    }
    (legal, illegal)
}

impl PyPhotonicRouter {
    pub(crate) fn probe_net_for_repair(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        order_by_id: &FxHashMap<u64, usize>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
    ) -> Result<ProbeState, ()> {
        let probe_start = native_batch_timer(collect_native_timing);
        let probe_result = self.route_single_net_ignore_dynamic_native(
            job.source,
            job.target,
            Some(&job.opened_cells),
            Some(&job.opened_cell_keys),
        );
        let probe_elapsed_us = native_batch_elapsed_us(probe_start);
        batch.timings.probe_route_wall_us += probe_elapsed_us;
        let probe_route = match probe_result {
            Ok(route) => {
                batch
                    .timings
                    .add_route_result_stats_if(collect_native_timing, &route);
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "probe_route",
                    net_id: job.net_id,
                    route: Some(route.clone()),
                    failed: false,
                    error: None,
                    repair_round: None,
                    candidate_blockers: Vec::new(),
                    ripup_ids: Vec::new(),
                });
                route
            }
            Err(error) => {
                batch.timings.probe_route_failed_wall_us += probe_elapsed_us;
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "probe_route",
                    net_id: job.net_id,
                    route: None,
                    failed: true,
                    error: Some(error.clone()),
                    repair_round: None,
                    candidate_blockers: Vec::new(),
                    ripup_ids: Vec::new(),
                });
                batch.failed_net_id = Some(job.net_id);
                batch.failed_error = Some(error);
                return Err(());
            }
        };

        let victim_selection_start = native_batch_timer(collect_native_timing);
        let owner_lookup_radius_cells =
            block_radius_cells.max(commit_radius_cells.unwrap_or(block_radius_cells));
        let dynamic_probe_owners =
            self.dynamic_owners_for_native_route(&probe_route, owner_lookup_radius_cells);
        let mut candidate_blocker_priority: FxHashMap<u64, u8> = FxHashMap::default();
        let mut add_candidate_blocker = |owner: u64, priority: u8| {
            if owner == job.net_id || !batch.final_routes.contains_key(&owner) {
                return;
            }
            candidate_blocker_priority
                .entry(owner)
                .and_modify(|existing| *existing = (*existing).min(priority))
                .or_insert(priority);
        };
        let crossing_repair_enabled = self.crossing_context.is_enabled();
        let allowed_crossing_partners: FxHashSet<u64> = if crossing_repair_enabled {
            self.lidar_probe_partner_lookup_set(
                job.net_id,
                &probe_route,
                owner_lookup_radius_cells,
                &batch.final_routes,
            )
        } else {
            FxHashSet::default()
        };
        // Geometric reconstruction (not the grid-waypoint-based
        // `crossing_events_for_route`): the probe route's crossings are
        // checked against the physical centerline so a crossing that lands
        // exactly on a cell corner between two 45-degree diagonals is still
        // detected. `crossing_events_for_route` misses that case entirely.
        let probe_crossing_events =
            if crossing_repair_enabled && !allowed_crossing_partners.is_empty() {
                self.realized_crossing_events_for_route(
                    job.net_id,
                    &probe_route,
                    &allowed_crossing_partners,
                    job.source_port_um,
                    job.target_port_um,
                )
            } else {
                Vec::new()
            };
        // Pre-commit probe: `probe_route` has not been committed (it comes
        // from `route_single_net_ignore_dynamic_native`, ignoring dynamic
        // obstacles entirely), so it cannot have registered crossing
        // events yet -- `probe_crossing_events` above is this probe's own
        // view of which crossings it would register on a real commit.
        let probe_realized_crossing_violations = if crossing_repair_enabled {
            self.crossing_violations_for_route_with_ports(
                job.net_id,
                &probe_route,
                job.source_port_um,
                job.target_port_um,
                Some(&job.opened_cell_keys),
                false,
            )
        } else {
            Vec::new()
        };
        self.dump_crossing_mismatch(
            job.net_id,
            &probe_route,
            job.source_port_um,
            job.target_port_um,
            &probe_realized_crossing_violations,
        );
        let probe_grid_crossing_violations =
            if crossing_repair_enabled && !allowed_crossing_partners.is_empty() {
                self.invalid_crossing_intersections_for_route(
                    job.net_id,
                    &probe_route,
                    &allowed_crossing_partners,
                )
            } else {
                Vec::new()
            };
        let allowed_crossing_partner_list: Vec<u64> =
            allowed_crossing_partners.iter().copied().collect();
        let probe_repair_keepout_keys = if crossing_repair_enabled {
            let mut keys = self.crossing_physical_violation_repair_keepout_keys(
                &probe_realized_crossing_violations,
                &allowed_crossing_partner_list,
            );
            keys.extend(self.crossing_grid_violation_repair_keepout_keys(
                &probe_grid_crossing_violations,
                &allowed_crossing_partner_list,
            ));
            keys
        } else {
            FxHashSet::default()
        };
        if crossing_repair_enabled {
            let legal_crossed_partners =
                Self::crossing_partner_ids_from_events(&probe_crossing_events);
            for owner in dynamic_probe_owners {
                if !legal_crossed_partners.contains(&owner) {
                    add_candidate_blocker(owner, 0);
                }
            }
            for invalid in &probe_grid_crossing_violations {
                add_candidate_blocker(invalid.partner_net_id, 1);
            }
            for invalid in &probe_realized_crossing_violations {
                add_candidate_blocker(invalid.partner_net_id, 0);
            }
            for partner_id in
                Self::crossing_partners_with_overlapping_reservations(&probe_crossing_events)
            {
                add_candidate_blocker(partner_id, 1);
            }
            let reservation_blockers = self.crossing_reservation_blockers(
                job.net_id,
                &probe_crossing_events,
                Some(&job.opened_cell_keys),
            );
            for owner in reservation_blockers.dynamic_blockers {
                add_candidate_blocker(owner, 0);
            }
            if reservation_blockers.has_static_blocker {
                for event in &probe_crossing_events {
                    add_candidate_blocker(event.partner_net_id, 1);
                }
            }
        } else {
            for owner in dynamic_probe_owners {
                add_candidate_blocker(owner, 1);
            }
        }
        let probe_intersecting_partners: Vec<u64> = if crossing_repair_enabled {
            Vec::new()
        } else {
            let partners = self.committed_partners_intersecting_route(
                job.net_id,
                &probe_route,
                job.source_port_um,
                job.target_port_um,
            );
            for partner_id in &partners {
                add_candidate_blocker(*partner_id, 0);
            }
            partners
                .into_iter()
                .filter(|partner_id| batch.final_routes.contains_key(partner_id))
                .collect()
        };
        let mut candidate_blockers: Vec<u64> = candidate_blocker_priority.keys().copied().collect();
        candidate_blockers.sort_unstable_by_key(|owner| {
            (
                candidate_blocker_priority
                    .get(owner)
                    .copied()
                    .unwrap_or(u8::MAX),
                order_by_id.get(owner).copied().unwrap_or(usize::MAX),
            )
        });
        let probe = ProbeState {
            probe_route,
            crossing_repair_enabled,
            allowed_crossing_partners,
            probe_crossing_events,
            probe_realized_crossing_violations,
            probe_grid_crossing_violations,
            probe_repair_keepout_keys,
            candidate_blockers,
            probe_intersecting_partners,
        };
        if trace_native_repair {
            eprintln!(
                "{}native_repair_probe net={} allowed_partners={} crossing_events={} grid_violations={} realized_violations={} realized_reasons={:?} keepout_keys={} candidate_blockers={:?}",
                trace_t(self.negotiated_batch_start),
                job.net_id,
                probe.allowed_crossing_partners.len(),
                probe.probe_crossing_events.len(),
                probe.probe_grid_crossing_violations.len(),
                probe.probe_realized_crossing_violations.len(),
                probe.probe_realized_crossing_violations
                    .iter()
                    .map(|violation| (violation.partner_net_id, violation.reason))
                    .collect::<Vec<_>>(),
                probe.probe_repair_keepout_keys.len(),
                probe.candidate_blockers,
            );
        }
        batch.timings.repair_probe_victim_selection_us +=
            native_batch_elapsed_us(victim_selection_start);
        Ok(probe)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;

    /// Structural widening (2026-09-15 15:00 Decision Log entry, "Slice
    /// result 15:40" paragraph): probe partners `{5}` where jobs 5, 6 and 7
    /// share `source.x` 100 and job 8 has `source.x` 200. All of 5, 6 and 8
    /// are committed; 7 is not. The widened set must be `{5, 6}`: 7 is
    /// dropped for not being committed and 8 for being in a different
    /// source column, leaving the probe's own partner (5) plus the one
    /// other committed same-column net (6).
    #[test]
    fn widen_probe_partners_by_source_column_adds_same_source_column_committed_nets() {
        let native_jobs = vec![
            NativeRouteJob::new(
                5,
                PyState::new(100, 0, 0),
                PyState::new(100, 10, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
            NativeRouteJob::new(
                6,
                PyState::new(100, 1, 0),
                PyState::new(100, 11, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
            NativeRouteJob::new(
                7,
                PyState::new(100, 2, 0),
                PyState::new(100, 12, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
            NativeRouteJob::new(
                8,
                PyState::new(200, 0, 0),
                PyState::new(200, 10, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
        ];
        let job_by_id: FxHashMap<u64, NativeRouteJob> = native_jobs
            .iter()
            .cloned()
            .map(|job| (job.net_id, job))
            .collect();
        let mut source_layer_indices_by_x: FxHashMap<i32, Vec<usize>> = FxHashMap::default();
        for (index, job) in native_jobs.iter().enumerate() {
            source_layer_indices_by_x
                .entry(job.source.x)
                .or_default()
                .push(index);
        }
        let mut committed: FxHashMap<u64, RouteResult> = FxHashMap::default();
        committed.insert(5, empty_test_route());
        committed.insert(6, empty_test_route());
        committed.insert(8, empty_test_route());
        // Net 7 is intentionally left uncommitted.

        let widened = widen_probe_partners_by_source_column(
            &[5],
            &job_by_id,
            &source_layer_indices_by_x,
            &native_jobs,
            &committed,
            4,
        );
        assert_eq!(widened, vec![5, 6]);
    }

    /// An empty probe partner set widens to an empty set (nothing to widen
    /// from).
    #[test]
    fn widen_probe_partners_by_source_column_with_empty_probe_set_returns_empty() {
        let native_jobs = vec![NativeRouteJob::new(
            5,
            PyState::new(100, 0, 0),
            PyState::new(100, 10, 0),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            None,
            None,
        )];
        let job_by_id: FxHashMap<u64, NativeRouteJob> = native_jobs
            .iter()
            .cloned()
            .map(|job| (job.net_id, job))
            .collect();
        let mut source_layer_indices_by_x: FxHashMap<i32, Vec<usize>> = FxHashMap::default();
        for (index, job) in native_jobs.iter().enumerate() {
            source_layer_indices_by_x
                .entry(job.source.x)
                .or_default()
                .push(index);
        }
        let mut committed: FxHashMap<u64, RouteResult> = FxHashMap::default();
        committed.insert(5, empty_test_route());

        let widened = widen_probe_partners_by_source_column(
            &[],
            &job_by_id,
            &source_layer_indices_by_x,
            &native_jobs,
            &committed,
            4,
        );
        assert!(widened.is_empty());
    }

    /// 2026-09-15 16:25 Decision Log entry of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`,
    /// follow-up (2): a probe with crossing events against {5, 6, 7} and a
    /// realized violation against 7 must split into legal {5, 6} and
    /// illegal {7} -- the negotiated loop's legal-only probe-guided step
    /// and its mixed-case post-rip-up guided step both build their
    /// guidance set from this split, so they must agree on it.
    #[test]
    fn split_probe_partners_separates_legal_from_illegal_partners() {
        let events: Vec<CrossingEvent> = [5u64, 6, 7]
            .into_iter()
            .map(|partner_net_id| CrossingEvent {
                net_id: 4,
                partner_net_id,
                point: (0.0, 0.0),
                route_segment: ((0, 0), (1, 1)),
                partner_segment: ((0, 0), (1, 1)),
                route_angle: 0,
                partner_angle: 1,
                reservation_keys: FxHashSet::default(),
            })
            .collect();
        let probe = ProbeState {
            probe_route: empty_test_route(),
            crossing_repair_enabled: true,
            allowed_crossing_partners: FxHashSet::default(),
            probe_crossing_events: events,
            probe_realized_crossing_violations: vec![InvalidCrossingIntersection {
                net_id: 4,
                partner_net_id: 7,
                point: (0.0, 0.0),
                reason: "not_perpendicular",
            }],
            probe_grid_crossing_violations: Vec::new(),
            probe_repair_keepout_keys: FxHashSet::default(),
            candidate_blockers: Vec::new(),
            probe_intersecting_partners: Vec::new(),
        };

        let (legal, illegal) = split_probe_partners(&probe);
        assert_eq!(
            legal,
            vec![5, 6],
            "legal must be events not among the violations"
        );
        assert_eq!(
            illegal,
            vec![7],
            "illegal must be exactly the violation partners"
        );
    }
}
