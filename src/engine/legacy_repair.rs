use std::time::Instant;

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::astar::{RouteResult, State};
use crate::obstacle_map::{pack_xy, CellKey};
use crate::primitives::{Primitive, PrimitiveGeometry, PrimitiveLibrary};
use crate::search::{SearchEnvironment, SearchRequest};

#[cfg(test)]
use crate::crossings::CrossingConfig;
use crate::engine::*;

pub(crate) fn center_out_layer_job_indices(
    jobs: &[NativeRouteJob],
    layer_indices: &[usize],
    source_x: i32,
) -> Vec<usize> {
    if layer_indices.is_empty() {
        return Vec::new();
    }
    let mut indexed: Vec<(i32, usize)> = layer_indices
        .iter()
        .copied()
        .filter(|index| *index < jobs.len() && jobs[*index].source.x == source_x)
        .map(|index| (jobs[index].source.y, index))
        .collect();
    indexed.sort_unstable_by_key(|(source_y, index)| (*source_y, *index));
    if indexed.is_empty() {
        return Vec::new();
    }

    let mut ordered = Vec::with_capacity(indexed.len());
    let mut left = (indexed.len() - 1) / 2;
    let mut right = left + 1;
    ordered.push(indexed[left].1);
    while left > 0 || right < indexed.len() {
        if right < indexed.len() {
            ordered.push(indexed[right].1);
            right += 1;
        }
        if left > 0 {
            left -= 1;
            ordered.push(indexed[left].1);
        }
    }
    ordered
}

pub(crate) fn compute_repair_victim_sets(
    crossing_repair_enabled: bool,
    probe_realized_crossing_violations: &[InvalidCrossingIntersection],
    candidate_blockers: &[u64],
    max_victims: usize,
    max_rounds: u32,
) -> Vec<(u32, Vec<u64>)> {
    let mut repair_victim_sets: Vec<(u32, Vec<u64>)> = Vec::new();
    if crossing_repair_enabled {
        if !probe_realized_crossing_violations.is_empty() && candidate_blockers.len() > 2 {
            let single_victim_limit = candidate_blockers.len().min(max_victims);
            for owner in candidate_blockers.iter().take(single_victim_limit) {
                repair_victim_sets.push((1, vec![*owner]));
            }
            if max_victims >= 2 {
                let top_pair = vec![candidate_blockers[0], candidate_blockers[1]];
                if !repair_victim_sets
                    .iter()
                    .any(|(_, existing)| existing == &top_pair)
                {
                    repair_victim_sets.push((1, top_pair));
                }
            }
        } else {
            let single_victim_limit = candidate_blockers.len().min(max_victims);
            for owner in candidate_blockers.iter().take(single_victim_limit) {
                repair_victim_sets.push((1, vec![*owner]));
            }
        }
    }
    for round_idx in 1..=max_rounds {
        let ripup_ids: Vec<u64> = candidate_blockers
            .iter()
            .take((max_victims * round_idx as usize).min(candidate_blockers.len()))
            .copied()
            .collect();
        if !ripup_ids.is_empty()
            && !repair_victim_sets
                .iter()
                .any(|(_, existing)| existing == &ripup_ids)
        {
            repair_victim_sets.push((round_idx, ripup_ids));
        }
    }
    repair_victim_sets
}

pub(crate) fn enqueue_targeted_illegal_crossing_repair_set(
    repair_victim_sets: &mut Vec<(u32, Vec<u64>)>,
    candidate_blockers: &mut Vec<u64>,
    final_routes: &FxHashMap<u64, RouteResult>,
    current_net_id: u64,
    ripup_ids: &[u64],
    error: &str,
    round_idx: u32,
    max_rounds: u32,
    max_victims: usize,
) {
    const MAX_ADAPTIVE_REPAIR_SETS: usize = 8;
    let mut learned_ids = Vec::new();
    for extra_id in illegal_crossing_net_ids_from_error(error) {
        if extra_id == current_net_id
            || ripup_ids.contains(&extra_id)
            || !final_routes.contains_key(&extra_id)
        {
            continue;
        }
        if !candidate_blockers.contains(&extra_id) {
            candidate_blockers.push(extra_id);
        }
        if !learned_ids.contains(&extra_id) {
            learned_ids.push(extra_id);
        }
    }
    if learned_ids.is_empty() {
        return;
    }
    if round_idx >= max_rounds
        || ripup_ids.len() >= max_victims
        || repair_victim_sets.len() >= MAX_ADAPTIVE_REPAIR_SETS
    {
        return;
    }

    let mut next = ripup_ids.to_vec();
    for extra_id in learned_ids {
        if next.contains(&extra_id) {
            continue;
        }
        next.push(extra_id);
        if next.len() > max_victims {
            return;
        }
    }
    if repair_victim_sets
        .iter()
        .any(|(_, existing)| existing == &next)
    {
        return;
    }
    repair_victim_sets.push((round_idx.saturating_add(1), next));
}

pub(crate) fn enqueue_learned_keepout_repair_retry(
    repair_victim_sets: &mut Vec<(u32, Vec<u64>)>,
    retry_counts: &mut FxHashMap<Vec<u64>, usize>,
    ripup_ids: &[u64],
    round_idx: u32,
    next_repair_set_index: usize,
) {
    const MAX_LEARNED_KEEP_OUT_REPAIR_SETS: usize = 12;
    const MAX_LEARNED_KEEP_OUT_RETRIES_PER_SET: usize = 1;
    if ripup_ids.is_empty() || repair_victim_sets.len() >= MAX_LEARNED_KEEP_OUT_REPAIR_SETS {
        return;
    }
    let retry_key = ripup_ids.to_vec();
    if retry_counts.get(&retry_key).copied().unwrap_or(0) >= MAX_LEARNED_KEEP_OUT_RETRIES_PER_SET {
        return;
    }
    if repair_victim_sets
        .iter()
        .skip(next_repair_set_index)
        .any(|(_, existing)| existing.as_slice() == ripup_ids)
    {
        return;
    }
    *retry_counts.entry(retry_key.clone()).or_default() += 1;
    repair_victim_sets.push((round_idx, retry_key));
}

/// Appends each index in `deferred` to the back of `queue`, in order,
/// skipping any index already present in `queue`, then clears `deferred`.
/// Used by [`PyPhotonicRouter::route_many_with_repair_and_commit`] to give
/// a job deferred by
/// [`PyPhotonicRouter::try_source_layer_center_out_repair`] the full
/// per-net repair chain at the end of the batch instead of aborting --
/// see `.agent/execplans/2026-09-14-loss-driven-endgame-at-64x64.md` option A.
pub(crate) fn enqueue_deferred_jobs(
    queue: &mut std::collections::VecDeque<usize>,
    deferred: &mut Vec<usize>,
) {
    for &index in deferred.iter() {
        if !queue.contains(&index) {
            queue.push_back(index);
        }
    }
    deferred.clear();
}

impl PyPhotonicRouter {
    pub(crate) fn restore_source_layer_static_cleanup(
        &mut self,
        jobs: &[NativeRouteJob],
        layer_indices: &[usize],
    ) {
        for index in layer_indices {
            let Some(job) = jobs.get(*index) else {
                continue;
            };
            if job.static_cleanup_cell_keys.is_empty() {
                continue;
            }
            self.obstacle_map
                .add_static_keys(&job.static_cleanup_cell_keys);
            self.static_cells
                .extend(job.static_cleanup_cell_keys.iter().copied());
        }
        self.invalidate_meander_base_prefix();
    }

    pub(crate) fn rebuild_long_straight_congestion_from_routes(
        &mut self,
        final_routes: &FxHashMap<u64, RouteResult>,
    ) {
        self.obstacle_map.clear_congestion();
        self.long_straight_congestion_cells.clear();
        self.long_straight_congestion_records.clear();
        let mut route_ids: Vec<u64> = final_routes.keys().copied().collect();
        route_ids.sort_unstable();
        for net_id in route_ids {
            if let Some(route) = final_routes.get(&net_id) {
                self.add_long_straight_congestion_for_route(net_id, route);
            }
        }
    }

    pub(crate) fn rollback_source_layer_routes_for_retry(
        &mut self,
        jobs: &[NativeRouteJob],
        layer_indices: &[usize],
        final_routes: &mut FxHashMap<u64, RouteResult>,
    ) -> Vec<(usize, RouteResult)> {
        let mut saved_routes = Vec::new();
        for index in layer_indices {
            let Some(job) = jobs.get(*index) else {
                continue;
            };
            if let Some(route) = final_routes.remove(&job.net_id) {
                saved_routes.push((*index, route));
                self.rollback_committed_route(job.net_id);
            }
        }
        self.restore_source_layer_static_cleanup(jobs, layer_indices);
        self.rebuild_long_straight_congestion_from_routes(final_routes);
        saved_routes
    }

    /// Best-effort restore of a source layer's saved routes: attempts every
    /// entry in `saved_routes`, even after an earlier one fails to
    /// re-commit, so a single unrestorable net cannot silently strand every
    /// other net queued behind it in the same layer (see
    /// .agent/execplans/2026-08-19-fix-collision-crossing-zero-event-acceptance.md
    /// for the real, reproduced case this generalizes from -- a prior,
    /// all-or-nothing version of this function returned `Err` on the first
    /// failure and abandoned every remaining saved route unattempted).
    /// Returns the net ids whose saved route could not be re-committed, in
    /// `layer_indices` order; an empty vec means every saved route was
    /// restored. Callers must decide what to do about any returned net id
    /// (for example, attempt a fresh route for it) rather than treat a
    /// non-empty result as this function having silently succeeded.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn restore_saved_source_layer_routes(
        &mut self,
        jobs: &[NativeRouteJob],
        layer_indices: &[usize],
        saved_routes: Vec<(usize, RouteResult)>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        final_routes: &mut FxHashMap<u64, RouteResult>,
    ) -> Vec<u64> {
        for index in layer_indices {
            if let Some(job) = jobs.get(*index) {
                if final_routes.remove(&job.net_id).is_some() {
                    self.rollback_committed_route(job.net_id);
                }
            }
        }
        self.restore_source_layer_static_cleanup(jobs, layer_indices);
        let mut unrestored_net_ids = Vec::new();
        for (index, route) in saved_routes {
            let Some(job) = jobs.get(index) else {
                continue;
            };
            if !self.commit_native_route_with_clearance(
                job.net_id,
                &route,
                block_radius_cells,
                commit_radius_cells,
                &job.clearance_exempt_cells,
                core_radius_cells,
                job.source_port_um,
                job.target_port_um,
                Some(&job.opened_cell_keys),
            ) {
                unrestored_net_ids.push(job.net_id);
                continue;
            }
            remove_success_static_cleanup(&mut self.obstacle_map, job);
            final_routes.insert(job.net_id, route);
        }
        self.rebuild_long_straight_congestion_from_routes(final_routes);
        unrestored_net_ids
    }

    #[allow(clippy::too_many_arguments)]
    /// Low-level native routing primitive for one net within a source-layer
    /// center-out repair pass. Called only from
    /// [`Self::try_source_layer_center_out_repair`], never directly from
    /// `route_many_with_repair_and_commit`'s own loop.
    pub(crate) fn try_route_source_layer_center_out_native(
        &mut self,
        jobs: &[NativeRouteJob],
        layer_indices: &[usize],
        source_x: i32,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        timings: &mut NativeBatchTimings,
        final_routes: &mut FxHashMap<u64, RouteResult>,
        attempts: &mut Vec<NativeRouteAttempt>,
        trace_native_repair: bool,
    ) -> Result<Vec<u64>, String> {
        let ordered_indices = center_out_layer_job_indices(jobs, layer_indices, source_x);
        if ordered_indices.len() < 2 {
            return Err("source layer has fewer than two routed jobs".to_string());
        }

        let mut routed_net_ids = Vec::with_capacity(ordered_indices.len());
        for job_index in ordered_indices {
            let job = &jobs[job_index];
            if trace_native_repair {
                eprintln!(
                    "native_repair_source_layer_center_out_route source_x={} job_index={} net={}",
                    source_x,
                    job_index + 1,
                    job.net_id
                );
            }
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
            timings.repair_failed_net_wall_us += route_elapsed_us;
            let route = match route_result {
                Ok(route) => {
                    timings.add_route_result_stats_if(collect_native_timing, &route);
                    remove_success_static_cleanup(&mut self.obstacle_map, job);
                    route
                }
                Err(error) => {
                    timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                    return Err(format!(
                        "source-layer center-out failed for net {}: {}",
                        job.net_id, error
                    ));
                }
            };
            attempts.push(NativeRouteAttempt {
                bucket_name: "source_layer_center_out",
                net_id: job.net_id,
                route: Some(route.clone()),
                failed: false,
                error: None,
                repair_round: Some(0),
                candidate_blockers: Vec::new(),
                ripup_ids: Vec::new(),
            });
            final_routes.insert(job.net_id, route);
            routed_net_ids.push(job.net_id);
        }
        Ok(routed_net_ids)
    }

    #[allow(clippy::too_many_arguments)]
    /// Reroutes an entire source-x grid column of nets in center-out order
    /// (via [`Self::try_route_source_layer_center_out_native`]), not a
    /// single net or victim -- the coarsest-grained repair strategy in this
    /// file. Triggered only when lidar-pure collision-crossing routing is
    /// enabled, a `pending_straight_victim_hint_for` hint names an
    /// already-routed victim on this column, the column has enough jobs,
    /// and this column has not already been retried this batch
    /// (`batch.retried_source_layers`). Generalizes the "do not silently
    /// lose a net on partial restore" lesson from
    /// `.agent/execplans/2026-08-19-fix-collision-crossing-zero-event-acceptance.md`
    /// to this coarser repair granularity.
    pub(crate) fn try_source_layer_center_out_repair(
        &mut self,
        batch: &mut RepairBatchState,
        native_jobs: &[NativeRouteJob],
        job_by_id: &FxHashMap<u64, NativeRouteJob>,
        job: &NativeRouteJob,
        job_index: usize,
        hint: &PendingStraightVictimHint,
        source_layer_indices_by_x: &FxHashMap<i32, Vec<usize>>,
        order_by_id: &FxHashMap<u64, usize>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
    ) -> SourceLayerCenterOutOutcome {
        let source_layer_indices = source_layer_indices_by_x
            .get(&job.source.x)
            .cloned()
            .unwrap_or_default();
        if source_layer_indices.len() < SOURCE_LAYER_CENTER_OUT_MIN_JOBS
            || batch.retried_source_layers.contains(&job.source.x)
        {
            return SourceLayerCenterOutOutcome::NotAttempted;
        }
        batch.retried_source_layers.insert(job.source.x);
        if trace_native_repair {
            eprintln!(
                "native_repair_source_layer_center_out_start source_x={} layer_size={} trigger_index={} trigger_net={} victim={} count={}",
                job.source.x,
                source_layer_indices.len(),
                job_index + 1,
                job.net_id,
                hint.victim_net_id,
                hint.count
            );
        }
        let saved_layer_routes = self.rollback_source_layer_routes_for_retry(
            native_jobs,
            &source_layer_indices,
            &mut batch.final_routes,
        );
        match self.try_route_source_layer_center_out_native(
            native_jobs,
            &source_layer_indices,
            job.source.x,
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
            collect_native_timing,
            &mut batch.timings,
            &mut batch.final_routes,
            &mut batch.attempts,
            trace_native_repair,
        ) {
            Ok(routed_net_ids) => {
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "source_layer_center_out",
                    Some("center_out"),
                    Some("reroute_layer"),
                    job.net_id,
                    Some(0),
                    None,
                    &[hint.victim_net_id],
                    &[],
                    &routed_net_ids,
                    None,
                    None,
                    Some(true),
                    None,
                );
                batch.repair_count = batch.repair_count.saturating_add(1);
                return SourceLayerCenterOutOutcome::Routed;
            }
            Err(error) => {
                let unrestored_net_ids = self.restore_saved_source_layer_routes(
                    native_jobs,
                    &source_layer_indices,
                    saved_layer_routes,
                    block_radius_cells,
                    commit_radius_cells,
                    core_radius_cells,
                    &mut batch.final_routes,
                );
                // The restore above is best-effort: one saved
                // route failing to re-commit no longer aborts
                // restoring the rest of the layer. Any net id
                // still returned here has neither its original
                // route nor any route at all in final_routes --
                // attempt one fresh route for each, the same way
                // this function's own pending_straight/preemptive
                // repair mechanisms already recover a single
                // ripped-up victim, instead of leaving it silently
                // missing from the final output (the real,
                // reproduced bug this generalizes from -- see
                // .agent/execplans/2026-08-19-fix-collision-crossing-zero-event-acceptance.md).
                let mut still_missing_net_ids = Vec::new();
                for &unrestored_net_id in &unrestored_net_ids {
                    let Some(unrestored_job) = job_by_id.get(&unrestored_net_id) else {
                        still_missing_net_ids.push(unrestored_net_id);
                        continue;
                    };
                    let reroute_start = native_batch_timer(collect_native_timing);
                    let reroute_result = self.route_single_net_and_commit_native(
                        unrestored_job.net_id,
                        unrestored_job.source,
                        unrestored_job.target,
                        block_radius_cells,
                        Some(&unrestored_job.opened_cells),
                        Some(&unrestored_job.opened_cell_keys),
                        commit_radius_cells,
                        Some(&unrestored_job.clearance_exempt_cells),
                        Some(&unrestored_job.clearance_exempt_cell_keys),
                        core_radius_cells,
                        unrestored_job.source_port_um,
                        unrestored_job.target_port_um,
                    );
                    batch.timings.repair_failed_net_wall_us +=
                        native_batch_elapsed_us(reroute_start);
                    match reroute_result {
                        Ok(route) => {
                            batch
                                .timings
                                .add_route_result_stats_if(collect_native_timing, &route);
                            remove_success_static_cleanup(&mut self.obstacle_map, unrestored_job);
                            batch.attempts.push(NativeRouteAttempt {
                                bucket_name: "source_layer_restore_fallback",
                                net_id: unrestored_net_id,
                                route: Some(route.clone()),
                                failed: false,
                                error: None,
                                repair_round: Some(0),
                                candidate_blockers: Vec::new(),
                                ripup_ids: Vec::new(),
                            });
                            batch.final_routes.insert(unrestored_net_id, route);
                        }
                        Err(reroute_error) => {
                            batch.attempts.push(NativeRouteAttempt {
                                bucket_name: "source_layer_restore_fallback",
                                net_id: unrestored_net_id,
                                route: None,
                                failed: true,
                                error: Some(reroute_error),
                                repair_round: Some(0),
                                candidate_blockers: Vec::new(),
                                ripup_ids: Vec::new(),
                            });
                            still_missing_net_ids.push(unrestored_net_id);
                        }
                    }
                }
                let error = if unrestored_net_ids.is_empty() {
                    error
                } else {
                    format!(
                        "{error}; source-layer restore could not recommit \
                         net(s) {unrestored_net_ids:?}, fresh reroute {}",
                        if still_missing_net_ids.is_empty() {
                            "recovered all of them".to_string()
                        } else {
                            format!("still failed for net(s) {still_missing_net_ids:?}")
                        }
                    )
                };
                // Previously this deferred branch aborted the whole batch
                // (`batch.failed_net_id`/`batch.failed_error` + `Err(())`,
                // which `route_many_with_repair_and_commit` turned into
                // `break 'route_jobs`). Instead, defer the still-missing
                // net(s) to the end of the batch's work queue, where each
                // gets the full per-net repair chain like any other net --
                // see `.agent/execplans/2026-09-14-loss-driven-endgame-at-64x64.md`
                // option A.
                let error = if still_missing_net_ids.is_empty() {
                    error
                } else {
                    format!(
                        "{error}; deferred net(s) {still_missing_net_ids:?} to the end of the batch"
                    )
                };
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "source_layer_center_out",
                    Some("center_out"),
                    Some("reroute_layer"),
                    job.net_id,
                    Some(0),
                    None,
                    &[hint.victim_net_id],
                    &[],
                    &[],
                    None,
                    None,
                    Some(still_missing_net_ids.is_empty()),
                    Some(error),
                );
                if !still_missing_net_ids.is_empty() {
                    for &missing_net_id in &still_missing_net_ids {
                        if let Some(&missing_index) = order_by_id.get(&missing_net_id) {
                            batch.deferred_job_indices.push(missing_index);
                        }
                    }
                    batch.deferred_count = batch
                        .deferred_count
                        .saturating_add(still_missing_net_ids.len() as u32);
                    if trace_native_repair {
                        eprintln!(
                            "native_repair_source_layer_deferred trigger_net={} deferred={:?}",
                            job.net_id, still_missing_net_ids
                        );
                    }
                }
            }
        }
        SourceLayerCenterOutOutcome::NotAttempted
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn prepare_repair_attempt(
        &mut self,
        batch: &mut RepairBatchState,
        probe: &ProbeState,
        max_rounds: u32,
        max_victims: usize,
    ) -> RepairAttemptState {
        RepairAttemptState {
            repaired: false,
            round_base_map: self.obstacle_map.clone(),
            round_base_center_routes: self.committed_center_routes.clone(),
            round_base_realized_center_routes: self.committed_realized_center_routes.clone(),
            round_base_target_terminal_bump_guards: self
                .committed_target_terminal_bump_guards
                .clone(),
            round_base_opened_cell_keys: self.committed_opened_cell_keys.clone(),
            round_base_crossing_events: self.crossing_events.clone(),
            round_base_routes: batch.final_routes.clone(),
            repair_victim_sets: compute_repair_victim_sets(
                probe.crossing_repair_enabled,
                &probe.probe_realized_crossing_violations,
                &probe.candidate_blockers,
                max_victims,
                max_rounds,
            ),
            learned_repair_keepouts_by_ripup: FxHashMap::default(),
            learned_victim_only_keepouts_by_ripup: FxHashMap::default(),
            learned_repair_retry_counts: FxHashMap::default(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    /// Temporarily adds the failure probe's computed keepout cells as
    /// static obstacles, retries plain-with-orthogonal-preference then
    /// repair-native-with-keepout, then removes the keepout regardless of
    /// outcome. Triggered when the probe found a repairable (not
    /// invalid-collision) crossing conflict with a non-empty keepout and
    /// candidate-blocker set.
    pub(crate) fn try_localized_crossing_keepout_retry(
        &mut self,
        batch: &mut RepairBatchState,
        probe: &ProbeState,
        job: &NativeRouteJob,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        history_weight: f64,
        collect_native_timing: bool,
        trace_native_repair: bool,
    ) -> LocalizedKeepoutOutcome {
        let invalid_collision_crossing_candidate = probe.crossing_repair_enabled
            && self.use_collision_crossing_routing
            && !probe.probe_realized_crossing_violations.is_empty();
        if probe.crossing_repair_enabled
            && !invalid_collision_crossing_candidate
            && !probe.probe_repair_keepout_keys.is_empty()
            && !probe.candidate_blockers.is_empty()
        {
            let localized_probe_keepout = probe.probe_repair_keepout_keys.clone();
            let added_keepout = self.obstacle_map.add_static_keys(&localized_probe_keepout);
            if trace_native_repair {
                eprintln!(
                    "native_repair_local_keepout net={} keys={} added={} candidate_blockers={:?}",
                    job.net_id,
                    localized_probe_keepout.len(),
                    added_keepout,
                    probe.candidate_blockers,
                );
            }

            let mut local_retry_route: Option<RouteResult> = None;
            let route_start = native_batch_timer(collect_native_timing);
            let prefer_orthogonal_local_retry =
                !probe.probe_realized_crossing_violations.is_empty();
            let route_result = self
                .route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                    job.net_id,
                    job.source,
                    job.target,
                    block_radius_cells,
                    &job.opened_cells,
                    &job.opened_cell_keys,
                    commit_radius_cells,
                    &job.clearance_exempt_cells,
                    &job.clearance_exempt_cell_keys,
                    core_radius_cells,
                    &localized_probe_keepout,
                    job.source_port_um,
                    job.target_port_um,
                    prefer_orthogonal_local_retry,
                );
            let route_elapsed_us = native_batch_elapsed_us(route_start);
            batch.timings.repair_failed_net_wall_us += route_elapsed_us;
            match route_result {
                Ok(route) => {
                    batch
                        .timings
                        .add_route_result_stats_if(collect_native_timing, &route);
                    remove_success_static_cleanup(&mut self.obstacle_map, job);
                    batch.attempts.push(NativeRouteAttempt {
                        bucket_name: "localized_crossing_keepout",
                        net_id: job.net_id,
                        route: Some(route.clone()),
                        failed: false,
                        error: None,
                        repair_round: Some(0),
                        candidate_blockers: probe.candidate_blockers.clone(),
                        ripup_ids: Vec::new(),
                    });
                    local_retry_route = Some(route);
                }
                Err(normal_error) => {
                    let (repair_keepout, extra_repair_keepout) = self
                        .augmented_crossing_error_repair_keepout(
                            &localized_probe_keepout,
                            &normal_error,
                        );
                    if !extra_repair_keepout.is_empty() {
                        self.obstacle_map.add_static_keys(&extra_repair_keepout);
                    }
                    batch.timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                    batch.attempts.push(NativeRouteAttempt {
                        bucket_name: "localized_crossing_keepout",
                        net_id: job.net_id,
                        route: None,
                        failed: true,
                        error: Some(normal_error),
                        repair_round: Some(0),
                        candidate_blockers: probe.candidate_blockers.clone(),
                        ripup_ids: Vec::new(),
                    });
                    let repair_start = native_batch_timer(collect_native_timing);
                    let repair_result = self
                        .route_single_net_and_commit_repair_native_with_repair_keepout(
                            job.net_id,
                            job.source,
                            job.target,
                            block_radius_cells,
                            &job.opened_cells,
                            &job.opened_cell_keys,
                            history_weight,
                            commit_radius_cells,
                            &job.clearance_exempt_cells,
                            &job.clearance_exempt_cell_keys,
                            core_radius_cells,
                            &repair_keepout,
                            job.source_port_um,
                            job.target_port_um,
                        );
                    let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                    if !extra_repair_keepout.is_empty() {
                        self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                    }
                    batch.timings.repair_failed_net_wall_us += repair_elapsed_us;
                    match repair_result {
                        Ok(route) => {
                            batch
                                .timings
                                .add_route_result_stats_if(collect_native_timing, &route);
                            remove_success_static_cleanup(&mut self.obstacle_map, job);
                            batch.attempts.push(NativeRouteAttempt {
                                bucket_name: "localized_crossing_keepout",
                                net_id: job.net_id,
                                route: Some(route.clone()),
                                failed: false,
                                error: None,
                                repair_round: Some(0),
                                candidate_blockers: probe.candidate_blockers.clone(),
                                ripup_ids: Vec::new(),
                            });
                            local_retry_route = Some(route);
                        }
                        Err(error) => {
                            batch.timings.repair_failed_net_failed_wall_us += repair_elapsed_us;
                            batch.attempts.push(NativeRouteAttempt {
                                bucket_name: "localized_crossing_keepout",
                                net_id: job.net_id,
                                route: None,
                                failed: true,
                                error: Some(error),
                                repair_round: Some(0),
                                candidate_blockers: probe.candidate_blockers.clone(),
                                ripup_ids: Vec::new(),
                            });
                        }
                    }
                }
            }
            self.obstacle_map
                .remove_static_keys(&localized_probe_keepout);
            if let Some(route) = local_retry_route {
                batch.final_routes.insert(job.net_id, route);
                batch.repair_count = batch.repair_count.saturating_add(1);
                return LocalizedKeepoutOutcome::Routed;
            }
        }
        LocalizedKeepoutOutcome::NotResolved
    }

    #[allow(clippy::too_many_arguments)]
    /// The cheapest possible resolution: when the failure probe found zero
    /// conflicting nets (`candidate_blockers.is_empty()`), commits the
    /// already-computed probe route directly instead of searching again.
    /// Not really a "repair strategy" -- a fast path for the common case
    /// where the probe route was already legal.
    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn try_commit_clean_probe(
        &mut self,
        batch: &mut RepairBatchState,
        probe: &ProbeState,
        job: &NativeRouteJob,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
    ) -> Result<CommitIfCleanOutcome, ()> {
        if probe.candidate_blockers.is_empty() {
            if probe.probe_crossing_compliant {
                {
                    let commit_start = native_batch_timer(collect_native_timing);
                    let crossed_partner_ids =
                        Self::crossing_partner_ids_from_events(&probe.probe_crossing_events);
                    let allowed_crossing_core_keys =
                        Self::crossing_reservation_keys_for_events(&probe.probe_crossing_events);
                    let (route_cells, core_cells) = self.route_commit_and_core_cells(
                        &probe.probe_route,
                        block_radius_cells,
                        commit_radius_cells,
                        Some(&job.clearance_exempt_cells),
                        core_radius_cells,
                        job.source_port_um,
                        job.target_port_um,
                    );
                    if self
                        .obstacle_map
                        .commit_route_with_clearance_and_allowed_core_overlap_cells(
                            job.net_id,
                            &core_cells,
                            &route_cells,
                            &job.clearance_exempt_cells,
                            &crossed_partner_ids,
                            Some(&allowed_crossing_core_keys),
                        )
                    {
                        batch.timings.commit_update_dynamic_map_us +=
                            native_batch_elapsed_us(commit_start);
                        self.remove_crossing_events_for_net(job.net_id);
                        self.add_crossing_events(probe.probe_crossing_events.clone());
                        if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                            job.net_id,
                            &probe.probe_route,
                            job.source_port_um,
                            job.target_port_um,
                        ) {
                            // Same rejection-carries-the-culprit bookkeeping as
                            // the validation Err arm below (Milestone 4 of
                            // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`).
                            // In practice this is always empty here: the
                            // failure is a centerline-computation error, so
                            // `committed_crossing_violation_partners` hits the
                            // same failure internally and returns nothing --
                            // kept for symmetry with the validation arm, which
                            // sets the same two `batch` fields.
                            batch.last_rejected_commit_partners = self
                                .committed_crossing_violation_partners(
                                    job.net_id,
                                    &probe.probe_route,
                                    job.source_port_um,
                                    job.target_port_um,
                                    Some(&job.opened_cell_keys),
                                );
                            self.rollback_committed_route(job.net_id);
                            batch.failed_net_id = Some(job.net_id);
                            batch.failed_error = Some(error);
                            return Err(());
                        }
                        self.remember_committed_route_opened_cells(
                            job.net_id,
                            Some(&job.opened_cell_keys),
                        );
                        self.add_post_commit_guidance_for_route(job.net_id, &probe.probe_route);
                        self.invalidate_meander_base_prefix();
                        if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                            job.net_id,
                            &probe.probe_route,
                            job.source_port_um,
                            job.target_port_um,
                            Some(&job.opened_cell_keys),
                        ) {
                            // Record the violating partner ids before rolling
                            // back so `route_many_with_negotiated_repair_and_commit`
                            // can rip them up via the global round instead of
                            // aborting the whole batch (Milestone 4 of
                            // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`).
                            // Computed before rollback so the committed state
                            // this check itself just ran against is still
                            // intact.
                            batch.last_rejected_commit_partners = self
                                .committed_crossing_violation_partners(
                                    job.net_id,
                                    &probe.probe_route,
                                    job.source_port_um,
                                    job.target_port_um,
                                    Some(&job.opened_cell_keys),
                                );
                            self.rollback_committed_route(job.net_id);
                            batch.failed_net_id = Some(job.net_id);
                            batch.failed_error = Some(error);
                            return Err(());
                        }
                        batch
                            .final_routes
                            .insert(job.net_id, probe.probe_route.clone());
                        return Ok(CommitIfCleanOutcome::Routed);
                    }
                    batch.timings.commit_update_dynamic_map_us +=
                        native_batch_elapsed_us(commit_start);
                }
            }
            if probe.strict_expected_crossing_probe {
                batch.failed_net_id = Some(job.net_id);
                batch.failed_error =
                    Some("Probe route violates expected crossing constraints".to_string());
                return Err(());
            }
            let commit_start = native_batch_timer(collect_native_timing);
            if self.commit_native_route_with_clearance(
                job.net_id,
                &probe.probe_route,
                block_radius_cells,
                commit_radius_cells,
                &job.clearance_exempt_cells,
                core_radius_cells,
                job.source_port_um,
                job.target_port_um,
                Some(&job.opened_cell_keys),
            ) {
                batch.timings.commit_update_dynamic_map_us += native_batch_elapsed_us(commit_start);
                batch
                    .final_routes
                    .insert(job.net_id, probe.probe_route.clone());
                return Ok(CommitIfCleanOutcome::Routed);
            }
            batch.timings.commit_update_dynamic_map_us += native_batch_elapsed_us(commit_start);
            batch.failed_net_id = Some(job.net_id);
            batch.failed_error = Some("Failed to commit static-only probe route".to_string());
            return Err(());
        }

        Ok(CommitIfCleanOutcome::NotResolved)
    }

    pub(crate) fn reset_repair_attempt_state_from_round_base(
        &mut self,
        batch: &mut RepairBatchState,
        repair: &RepairAttemptState,
        collect_native_timing: bool,
    ) {
        let reset_start = native_batch_timer(collect_native_timing);
        self.obstacle_map = repair.round_base_map.clone();
        self.committed_center_routes = repair.round_base_center_routes.clone();
        self.committed_realized_center_routes = repair.round_base_realized_center_routes.clone();
        self.committed_target_terminal_bump_guards =
            repair.round_base_target_terminal_bump_guards.clone();
        self.committed_opened_cell_keys = repair.round_base_opened_cell_keys.clone();
        self.crossing_events = repair.round_base_crossing_events.clone();
        self.invalidate_meander_base_prefix();
        batch.final_routes = repair.round_base_routes.clone();
        batch.timings.repair_state_reset_us += native_batch_elapsed_us(reset_start);
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn label_and_trace_repair_mode_start(
        &self,
        batch: &mut RepairBatchState,
        probe: &ProbeState,
        job: &NativeRouteJob,
        round_idx: u32,
        active_repair_set_index: usize,
        ripup_ids: &[u64],
        victim_first: bool,
        reverse_victim_order: bool,
    ) -> (&'static str, Vec<u64>) {
        let mut victim_reroute_ids = ripup_ids.to_vec();
        if reverse_victim_order {
            victim_reroute_ids.reverse();
        }
        let route_order = if victim_first {
            "victim_first"
        } else {
            "current_first"
        };
        push_native_repair_trace(
            &mut batch.repair_trace,
            "repair_mode_start",
            Some(route_order),
            None,
            job.net_id,
            Some(round_idx),
            Some(active_repair_set_index as u64),
            &probe.candidate_blockers,
            ripup_ids,
            &victim_reroute_ids,
            Some(victim_first),
            Some(reverse_victim_order),
            None,
            None,
        );
        (route_order, victim_reroute_ids)
    }

    #[allow(clippy::too_many_arguments)]
    /// Reroutes the current net with a temporary reservation before any
    /// victim is touched (only when `!victim_first` in the round/victim-
    /// order loop). On failure, calls
    /// `enqueue_targeted_illegal_crossing_repair_set` to fold the new
    /// failure's blocking net into the ripup set -- this is the exact
    /// mechanism the n_67/n_70/n_71 fix
    /// (`.agent/execplans/2026-08-20-ripup-repair-orchestration-restructuring.md`)
    /// patched a string-prefix-matching bug in -- then falls back to
    /// `route_single_net_and_commit_repair_native_with_repair_keepout`.
    pub(crate) fn try_reroute_current_net_before_victims(
        &mut self,
        batch: &mut RepairBatchState,
        repair: &mut RepairAttemptState,
        probe: &mut ProbeState,
        mode: &mut RepairModeAttemptState,
        job: &NativeRouteJob,
        round_idx: u32,
        active_repair_set_index: usize,
        repair_set_index: usize,
        ripup_ids: &[u64],
        victim_first: bool,
        reverse_victim_order: bool,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        prefer_orthogonal_repair: bool,
        history_weight: f64,
        max_rounds: u32,
        max_victims: usize,
        collect_native_timing: bool,
    ) {
        let route_start = native_batch_timer(collect_native_timing);
        let route_result = self
            .route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                job.net_id,
                job.source,
                job.target,
                block_radius_cells,
                &job.opened_cells,
                &job.opened_cell_keys,
                commit_radius_cells,
                &job.clearance_exempt_cells,
                &job.clearance_exempt_cell_keys,
                core_radius_cells,
                &mode.temporary_probe_reservation,
                job.source_port_um,
                job.target_port_um,
                prefer_orthogonal_repair,
            );
        let route_elapsed_us = native_batch_elapsed_us(route_start);
        batch.timings.repair_failed_net_wall_us += route_elapsed_us;
        match route_result {
            Ok(route) => {
                batch
                    .timings
                    .add_route_result_stats_if(collect_native_timing, &route);
                remove_success_static_cleanup(&mut self.obstacle_map, job);
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "current_route",
                    Some(mode.route_order),
                    Some("normal_route"),
                    job.net_id,
                    Some(round_idx),
                    Some(active_repair_set_index as u64),
                    &probe.candidate_blockers,
                    ripup_ids,
                    &mode.victim_reroute_ids,
                    Some(victim_first),
                    Some(reverse_victim_order),
                    Some(true),
                    None,
                );
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "repair_failed_net",
                    net_id: job.net_id,
                    route: Some(route.clone()),
                    failed: false,
                    error: None,
                    repair_round: Some(round_idx),
                    candidate_blockers: probe.candidate_blockers.clone(),
                    ripup_ids: ripup_ids.to_vec(),
                });
                batch.final_routes.insert(job.net_id, route.clone());
                mode.repaired_route = Some(route);
            }
            Err(normal_error) => {
                enqueue_targeted_illegal_crossing_repair_set(
                    &mut repair.repair_victim_sets,
                    &mut probe.candidate_blockers,
                    &repair.round_base_routes,
                    job.net_id,
                    ripup_ids,
                    &normal_error,
                    round_idx,
                    max_rounds,
                    max_victims,
                );
                let learned_repair_keepout = repair
                    .learned_repair_keepouts_by_ripup
                    .entry(ripup_ids.to_vec())
                    .or_default();
                if self.remember_local_repair_error_keepout(learned_repair_keepout, &normal_error) {
                    enqueue_learned_keepout_repair_retry(
                        &mut repair.repair_victim_sets,
                        &mut repair.learned_repair_retry_counts,
                        ripup_ids,
                        round_idx,
                        repair_set_index,
                    );
                }
                let (repair_keepout, extra_repair_keepout) = self
                    .augmented_crossing_error_repair_keepout(
                        &mode.temporary_probe_reservation,
                        &normal_error,
                    );
                if !extra_repair_keepout.is_empty() {
                    self.obstacle_map.add_static_keys(&extra_repair_keepout);
                }
                batch.timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "current_route",
                    Some(mode.route_order),
                    Some("normal_route"),
                    job.net_id,
                    Some(round_idx),
                    Some(active_repair_set_index as u64),
                    &probe.candidate_blockers,
                    ripup_ids,
                    &mode.victim_reroute_ids,
                    Some(victim_first),
                    Some(reverse_victim_order),
                    Some(false),
                    Some(normal_error.clone()),
                );
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "repair_failed_net",
                    net_id: job.net_id,
                    route: None,
                    failed: true,
                    error: Some(normal_error),
                    repair_round: Some(round_idx),
                    candidate_blockers: probe.candidate_blockers.clone(),
                    ripup_ids: ripup_ids.to_vec(),
                });
                let repair_start = native_batch_timer(collect_native_timing);
                let repair_result = self
                    .route_single_net_and_commit_repair_native_with_repair_keepout(
                        job.net_id,
                        job.source,
                        job.target,
                        block_radius_cells,
                        &job.opened_cells,
                        &job.opened_cell_keys,
                        history_weight,
                        commit_radius_cells,
                        &job.clearance_exempt_cells,
                        &job.clearance_exempt_cell_keys,
                        core_radius_cells,
                        &repair_keepout,
                        job.source_port_um,
                        job.target_port_um,
                    );
                let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                if !extra_repair_keepout.is_empty() {
                    self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                }
                batch.timings.repair_failed_net_wall_us += repair_elapsed_us;
                match repair_result {
                    Ok(route) => {
                        batch
                            .timings
                            .add_route_result_stats_if(collect_native_timing, &route);
                        remove_success_static_cleanup(&mut self.obstacle_map, job);
                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "current_route",
                            Some(mode.route_order),
                            Some("repair_fallback"),
                            job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(true),
                            None,
                        );
                        batch.attempts.push(NativeRouteAttempt {
                            bucket_name: "repair_failed_net",
                            net_id: job.net_id,
                            route: Some(route.clone()),
                            failed: false,
                            error: None,
                            repair_round: Some(round_idx),
                            candidate_blockers: probe.candidate_blockers.clone(),
                            ripup_ids: ripup_ids.to_vec(),
                        });
                        batch.final_routes.insert(job.net_id, route.clone());
                        mode.repaired_route = Some(route);
                    }
                    Err(error) => {
                        enqueue_targeted_illegal_crossing_repair_set(
                            &mut repair.repair_victim_sets,
                            &mut probe.candidate_blockers,
                            &repair.round_base_routes,
                            job.net_id,
                            ripup_ids,
                            &error,
                            round_idx,
                            max_rounds,
                            max_victims,
                        );
                        let learned_repair_keepout = repair
                            .learned_repair_keepouts_by_ripup
                            .entry(ripup_ids.to_vec())
                            .or_default();
                        if self.remember_local_repair_error_keepout(learned_repair_keepout, &error)
                        {
                            enqueue_learned_keepout_repair_retry(
                                &mut repair.repair_victim_sets,
                                &mut repair.learned_repair_retry_counts,
                                ripup_ids,
                                round_idx,
                                repair_set_index,
                            );
                        }
                        batch.timings.repair_failed_net_failed_wall_us += repair_elapsed_us;
                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "current_route",
                            Some(mode.route_order),
                            Some("repair_fallback"),
                            job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(false),
                            Some(error.clone()),
                        );
                        batch.attempts.push(NativeRouteAttempt {
                            bucket_name: "repair_failed_net",
                            net_id: job.net_id,
                            route: None,
                            failed: true,
                            error: Some(error),
                            repair_round: Some(round_idx),
                            candidate_blockers: probe.candidate_blockers.clone(),
                            ripup_ids: ripup_ids.to_vec(),
                        });
                        mode.mode_failed = true;
                    }
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn reroute_victim_with_plain_fallback(
        &mut self,
        batch: &mut RepairBatchState,
        repair: &mut RepairAttemptState,
        probe: &mut ProbeState,
        mode: &mut RepairModeAttemptState,
        job: &NativeRouteJob,
        victim_job: &NativeRouteJob,
        round_idx: u32,
        active_repair_set_index: usize,
        repair_set_index: usize,
        ripup_ids: &[u64],
        victim_first: bool,
        reverse_victim_order: bool,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        prefer_orthogonal_repair: bool,
        history_weight: f64,
        max_rounds: u32,
        max_victims: usize,
        collect_native_timing: bool,
    ) -> VictimPlainRerouteOutcome {
        let reroute_start = native_batch_timer(collect_native_timing);
        let reroute_result = self
            .route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                victim_job.net_id,
                victim_job.source,
                victim_job.target,
                block_radius_cells,
                &victim_job.opened_cells,
                &victim_job.opened_cell_keys,
                commit_radius_cells,
                &victim_job.clearance_exempt_cells,
                &victim_job.clearance_exempt_cell_keys,
                core_radius_cells,
                &mode.temporary_probe_reservation,
                victim_job.source_port_um,
                victim_job.target_port_um,
                prefer_orthogonal_repair,
            );
        let reroute_elapsed_us = native_batch_elapsed_us(reroute_start);
        batch.timings.reroute_victims_wall_us += reroute_elapsed_us;
        let route = match reroute_result {
            Ok(route) => {
                batch
                    .timings
                    .add_route_result_stats_if(collect_native_timing, &route);
                remove_success_static_cleanup(&mut self.obstacle_map, victim_job);
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "victim_reroute",
                    Some(mode.route_order),
                    Some("normal_route"),
                    victim_job.net_id,
                    Some(round_idx),
                    Some(active_repair_set_index as u64),
                    &probe.candidate_blockers,
                    ripup_ids,
                    &mode.victim_reroute_ids,
                    Some(victim_first),
                    Some(reverse_victim_order),
                    Some(true),
                    None,
                );
                route
            }
            Err(normal_error) => {
                enqueue_targeted_illegal_crossing_repair_set(
                    &mut repair.repair_victim_sets,
                    &mut probe.candidate_blockers,
                    &repair.round_base_routes,
                    victim_job.net_id,
                    ripup_ids,
                    &normal_error,
                    round_idx,
                    max_rounds,
                    max_victims,
                );
                let learned_repair_keepout = repair
                    .learned_repair_keepouts_by_ripup
                    .entry(ripup_ids.to_vec())
                    .or_default();
                let victim_only_keepout = repair
                    .learned_victim_only_keepouts_by_ripup
                    .entry(ripup_ids.to_vec())
                    .or_default();
                if self.remember_victim_repair_error_keepout(
                    learned_repair_keepout,
                    victim_only_keepout,
                    &normal_error,
                    job.net_id,
                ) {
                    enqueue_learned_keepout_repair_retry(
                        &mut repair.repair_victim_sets,
                        &mut repair.learned_repair_retry_counts,
                        ripup_ids,
                        round_idx,
                        repair_set_index,
                    );
                }
                let (repair_keepout, extra_repair_keepout) = self
                    .augmented_crossing_error_repair_keepout(
                        &mode.temporary_probe_reservation,
                        &normal_error,
                    );
                if !extra_repair_keepout.is_empty() {
                    self.obstacle_map.add_static_keys(&extra_repair_keepout);
                }
                batch.timings.reroute_victims_failed_wall_us += reroute_elapsed_us;
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "victim_reroute",
                    Some(mode.route_order),
                    Some("normal_route"),
                    victim_job.net_id,
                    Some(round_idx),
                    Some(active_repair_set_index as u64),
                    &probe.candidate_blockers,
                    ripup_ids,
                    &mode.victim_reroute_ids,
                    Some(victim_first),
                    Some(reverse_victim_order),
                    Some(false),
                    Some(normal_error.clone()),
                );
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "reroute_victims",
                    net_id: victim_job.net_id,
                    route: None,
                    failed: true,
                    error: Some(normal_error),
                    repair_round: Some(round_idx),
                    candidate_blockers: probe.candidate_blockers.clone(),
                    ripup_ids: ripup_ids.to_vec(),
                });
                let repair_start = native_batch_timer(collect_native_timing);
                let repair_result = self
                    .route_single_net_and_commit_repair_native_with_repair_keepout(
                        victim_job.net_id,
                        victim_job.source,
                        victim_job.target,
                        block_radius_cells,
                        &victim_job.opened_cells,
                        &victim_job.opened_cell_keys,
                        history_weight,
                        commit_radius_cells,
                        &victim_job.clearance_exempt_cells,
                        &victim_job.clearance_exempt_cell_keys,
                        core_radius_cells,
                        &repair_keepout,
                        victim_job.source_port_um,
                        victim_job.target_port_um,
                    );
                let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                if !extra_repair_keepout.is_empty() {
                    self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                }
                batch.timings.reroute_victims_wall_us += repair_elapsed_us;
                match repair_result {
                    Ok(route) => {
                        batch
                            .timings
                            .add_route_result_stats_if(collect_native_timing, &route);
                        remove_success_static_cleanup(&mut self.obstacle_map, victim_job);
                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "victim_reroute",
                            Some(mode.route_order),
                            Some("repair_fallback"),
                            victim_job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(true),
                            None,
                        );
                        route
                    }
                    Err(error) => {
                        enqueue_targeted_illegal_crossing_repair_set(
                            &mut repair.repair_victim_sets,
                            &mut probe.candidate_blockers,
                            &repair.round_base_routes,
                            victim_job.net_id,
                            ripup_ids,
                            &error,
                            round_idx,
                            max_rounds,
                            max_victims,
                        );
                        let learned_repair_keepout = repair
                            .learned_repair_keepouts_by_ripup
                            .entry(ripup_ids.to_vec())
                            .or_default();
                        let victim_only_keepout = repair
                            .learned_victim_only_keepouts_by_ripup
                            .entry(ripup_ids.to_vec())
                            .or_default();
                        if self.remember_victim_repair_error_keepout(
                            learned_repair_keepout,
                            victim_only_keepout,
                            &error,
                            job.net_id,
                        ) {
                            enqueue_learned_keepout_repair_retry(
                                &mut repair.repair_victim_sets,
                                &mut repair.learned_repair_retry_counts,
                                ripup_ids,
                                round_idx,
                                repair_set_index,
                            );
                        }
                        batch.timings.reroute_victims_failed_wall_us += repair_elapsed_us;
                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "victim_reroute",
                            Some(mode.route_order),
                            Some("repair_fallback"),
                            victim_job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(false),
                            Some(error.clone()),
                        );
                        batch.attempts.push(NativeRouteAttempt {
                            bucket_name: "reroute_victims",
                            net_id: victim_job.net_id,
                            route: None,
                            failed: true,
                            error: Some(error),
                            repair_round: Some(round_idx),
                            candidate_blockers: probe.candidate_blockers.clone(),
                            ripup_ids: ripup_ids.to_vec(),
                        });
                        mode.mode_failed = true;
                        return VictimPlainRerouteOutcome::Failed;
                    }
                }
            }
        };
        batch.attempts.push(NativeRouteAttempt {
            bucket_name: "reroute_victims",
            net_id: victim_job.net_id,
            route: Some(route.clone()),
            failed: false,
            error: None,
            repair_round: Some(round_idx),
            candidate_blockers: probe.candidate_blockers.clone(),
            ripup_ids: ripup_ids.to_vec(),
        });
        batch.final_routes.insert(victim_job.net_id, route);
        VictimPlainRerouteOutcome::Routed
    }

    #[allow(clippy::too_many_arguments)]
    /// For each victim (when lidar-pure collision-crossing repair is
    /// enabled, victims are rerouted after the current net, and a repaired
    /// route exists), tries up to 3 A* search strategies in order before
    /// falling back to a plain reroute: a "seeded" partner-set search using
    /// crossing partners from the round-base committed events, a direct
    /// [`Self::try_route_with_collision_crossings_with_loss`] call, and a
    /// "guided" partner-set search using only the current job as sole
    /// partner. **This is the confirmed dominant cost in the
    /// `reroute_victims_wall` timing bucket** (see
    /// `.agent/REPOSITORY_STATE.md`'s "Investigation findings" note) --
    /// most of that bucket's wall-clock time is these up-to-3 preliminary
    /// searches, not the eventual committed route. Any future change here
    /// carries real performance risk, not just correctness risk.
    pub(crate) fn try_crossing_aware_victim_reroute(
        &mut self,
        batch: &mut RepairBatchState,
        repair: &RepairAttemptState,
        probe: &ProbeState,
        mode: &mut RepairModeAttemptState,
        job: &NativeRouteJob,
        victim_job: &NativeRouteJob,
        round_idx: u32,
        active_repair_set_index: usize,
        ripup_ids: &[u64],
        victim_first: bool,
        reverse_victim_order: bool,
        lidar_pure_crossing_repair: bool,
        guided_collision_crossing_enabled: bool,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
    ) -> PyResult<CrossingAwareVictimRerouteOutcome> {
        let mut guided_victim_route: Option<RouteResult> = None;
        let mut lidar_crossing_partners_available = false;
        if lidar_pure_crossing_repair && !victim_first && mode.repaired_route.is_some() {
            let victim_source_state = State::new(
                victim_job.source.x,
                victim_job.source.y,
                victim_job.source.angle,
            );
            let victim_target_state = State::new(
                victim_job.target.x,
                victim_job.target.y,
                victim_job.target.angle,
            );
            let opened_search_owned = self.opened_cells_without_dynamic_overlap(
                &victim_job.opened_cell_keys,
                victim_source_state,
                victim_target_state,
                Some(&victim_job.clearance_exempt_cell_keys),
            );
            let opened_search_ref = opened_search_owned
                .as_ref()
                .unwrap_or(&victim_job.opened_cell_keys);
            let dynamic_clearance_exempt_keys =
                if block_radius_cells > 0 && !victim_job.clearance_exempt_cells.is_empty() {
                    Some(&victim_job.clearance_exempt_cell_keys)
                } else {
                    None
                };
            let mut seeded_partner_ids = Self::crossing_partner_ids_for_net(
                &repair.round_base_crossing_events,
                victim_job.net_id,
            );
            seeded_partner_ids.retain(|partner_id| {
                *partner_id != victim_job.net_id
                    && !ripup_ids.contains(partner_id)
                    && self.committed_center_routes.contains_key(partner_id)
            });
            if self.committed_center_routes.contains_key(&job.net_id) {
                seeded_partner_ids.insert(job.net_id);
            }
            if seeded_partner_ids.len() > 1 {
                let mut seeded_cfg = self
                    .astar_config(None, None, Some(0.0))
                    .map_err(PyRuntimeError::new_err)?;
                seeded_cfg.require_terminal_straights = false;
                let seeded_start = native_batch_timer(collect_native_timing);
                let seeded_result = self
                    .try_route_through_collision_partner_set(
                        victim_job.net_id,
                        victim_source_state,
                        victim_target_state,
                        opened_search_ref,
                        &seeded_cfg,
                        block_radius_cells,
                        dynamic_clearance_exempt_keys,
                        &seeded_partner_ids,
                        victim_job.source_port_um,
                        victim_job.target_port_um,
                        Some(&victim_job.opened_cell_keys),
                    )
                    .map_err(PyRuntimeError::new_err)?;
                let seeded_elapsed_us = native_batch_elapsed_us(seeded_start);
                batch.timings.reroute_victims_wall_us += seeded_elapsed_us;
                if trace_native_repair {
                    eprintln!(
                        "victim_diag net={} strategy=seeded elapsed_us={} found={}",
                        victim_job.net_id,
                        seeded_elapsed_us,
                        seeded_result.is_some()
                    );
                }
                if let Some((route, crossing_events)) = seeded_result {
                    let crossed_partner_ids =
                        Self::crossing_partner_ids_from_events(&crossing_events);
                    let crossed_partner_vec: Vec<u64> =
                        crossed_partner_ids.iter().copied().collect();
                    match self.commit_native_route_with_clearance_allowing_core_overlap(
                        victim_job.net_id,
                        &route,
                        block_radius_cells,
                        commit_radius_cells,
                        &victim_job.clearance_exempt_cells,
                        core_radius_cells,
                        victim_job.source_port_um,
                        victim_job.target_port_um,
                        Some(&victim_job.opened_cell_keys),
                        &crossed_partner_vec,
                        true,
                    ) {
                        Ok(true) => {
                            batch
                                .timings
                                .add_route_result_stats_if(collect_native_timing, &route);
                            push_native_repair_trace(
                                &mut batch.repair_trace,
                                "victim_reroute",
                                Some(mode.route_order),
                                Some("lidar_seeded_collision_crossing"),
                                victim_job.net_id,
                                Some(round_idx),
                                Some(active_repair_set_index as u64),
                                &probe.candidate_blockers,
                                ripup_ids,
                                &mode.victim_reroute_ids,
                                Some(victim_first),
                                Some(reverse_victim_order),
                                Some(true),
                                None,
                            );
                            batch.attempts.push(NativeRouteAttempt {
                                bucket_name: "reroute_victims",
                                net_id: victim_job.net_id,
                                route: Some(route.clone()),
                                failed: false,
                                error: None,
                                repair_round: Some(round_idx),
                                candidate_blockers: probe.candidate_blockers.clone(),
                                ripup_ids: ripup_ids.to_vec(),
                            });
                            guided_victim_route = Some(route);
                        }
                        Ok(false) => {}
                        Err(error) => {
                            if trace_native_repair {
                                eprintln!(
                                    "native_repair_lidar_seeded_crossing_commit_failed net={} error={}",
                                    victim_job.net_id, error
                                );
                            }
                        }
                    }
                }
            }
            let collision_partner_ids = self.crossing_partner_lookup_set_for_route(
                victim_job.net_id,
                victim_source_state,
                victim_target_state,
            );
            lidar_crossing_partners_available = !collision_partner_ids.is_empty();
            if guided_victim_route.is_none() && lidar_crossing_partners_available {
                let mut crossing_cfg = self
                    .astar_config(None, None, Some(0.0))
                    .map_err(PyRuntimeError::new_err)?;
                crossing_cfg.require_terminal_straights = false;
                let crossing_start = native_batch_timer(collect_native_timing);
                let crossing_result = self
                    .try_route_with_collision_crossings_with_loss(
                        victim_job.net_id,
                        victim_source_state,
                        victim_target_state,
                        opened_search_ref,
                        &crossing_cfg,
                        block_radius_cells,
                        dynamic_clearance_exempt_keys,
                        &collision_partner_ids,
                        victim_job.source_port_um,
                        victim_job.target_port_um,
                        Some(&victim_job.opened_cell_keys),
                        Some(0.0),
                        false,
                    )
                    .map_err(PyRuntimeError::new_err)?;
                let crossing_elapsed_us = native_batch_elapsed_us(crossing_start);
                batch.timings.reroute_victims_wall_us += crossing_elapsed_us;
                if trace_native_repair {
                    eprintln!(
                        "victim_diag net={} strategy=collision elapsed_us={} found={}",
                        victim_job.net_id,
                        crossing_elapsed_us,
                        crossing_result.is_some()
                    );
                }
                if let Some((route, crossing_events)) = crossing_result {
                    let crossed_partner_ids =
                        Self::crossing_partner_ids_from_events(&crossing_events);
                    let crossed_partner_vec: Vec<u64> =
                        crossed_partner_ids.iter().copied().collect();
                    match self.commit_native_route_with_clearance_allowing_core_overlap(
                        victim_job.net_id,
                        &route,
                        block_radius_cells,
                        commit_radius_cells,
                        &victim_job.clearance_exempt_cells,
                        core_radius_cells,
                        victim_job.source_port_um,
                        victim_job.target_port_um,
                        Some(&victim_job.opened_cell_keys),
                        &crossed_partner_vec,
                        true,
                    ) {
                        Ok(true) => {
                            batch
                                .timings
                                .add_route_result_stats_if(collect_native_timing, &route);
                            push_native_repair_trace(
                                &mut batch.repair_trace,
                                "victim_reroute",
                                Some(mode.route_order),
                                Some("lidar_collision_crossing"),
                                victim_job.net_id,
                                Some(round_idx),
                                Some(active_repair_set_index as u64),
                                &probe.candidate_blockers,
                                ripup_ids,
                                &mode.victim_reroute_ids,
                                Some(victim_first),
                                Some(reverse_victim_order),
                                Some(true),
                                None,
                            );
                            batch.attempts.push(NativeRouteAttempt {
                                bucket_name: "reroute_victims",
                                net_id: victim_job.net_id,
                                route: Some(route.clone()),
                                failed: false,
                                error: None,
                                repair_round: Some(round_idx),
                                candidate_blockers: probe.candidate_blockers.clone(),
                                ripup_ids: ripup_ids.to_vec(),
                            });
                            guided_victim_route = Some(route);
                        }
                        Ok(false) => {}
                        Err(error) => {
                            if trace_native_repair {
                                eprintln!(
                                    "native_repair_lidar_crossing_commit_failed net={} error={}",
                                    victim_job.net_id, error
                                );
                            }
                        }
                    }
                }
            }
        }
        if probe.crossing_repair_enabled
            && self.use_collision_crossing_routing
            && guided_collision_crossing_enabled
            && !victim_first
            && mode.repaired_route.is_some()
            && self.committed_center_routes.contains_key(&job.net_id)
            && guided_victim_route.is_none()
        {
            let mut guided_partner_ids = FxHashSet::default();
            guided_partner_ids.insert(job.net_id);
            let victim_source_state = State::new(
                victim_job.source.x,
                victim_job.source.y,
                victim_job.source.angle,
            );
            let victim_target_state = State::new(
                victim_job.target.x,
                victim_job.target.y,
                victim_job.target.angle,
            );
            let opened_search_owned = self.opened_cells_without_dynamic_overlap(
                &victim_job.opened_cell_keys,
                victim_source_state,
                victim_target_state,
                Some(&victim_job.clearance_exempt_cell_keys),
            );
            let opened_search_ref = opened_search_owned
                .as_ref()
                .unwrap_or(&victim_job.opened_cell_keys);
            let dynamic_clearance_exempt_keys =
                if block_radius_cells > 0 && !victim_job.clearance_exempt_cells.is_empty() {
                    Some(&victim_job.clearance_exempt_cell_keys)
                } else {
                    None
                };
            let mut guided_cfg = self
                .astar_config(None, None, None)
                .map_err(PyRuntimeError::new_err)?;
            guided_cfg.require_terminal_straights = false;
            let guided_start = native_batch_timer(collect_native_timing);
            let guided_result = self
                .try_route_through_collision_partner_set(
                    victim_job.net_id,
                    victim_source_state,
                    victim_target_state,
                    opened_search_ref,
                    &guided_cfg,
                    block_radius_cells,
                    dynamic_clearance_exempt_keys,
                    &guided_partner_ids,
                    victim_job.source_port_um,
                    victim_job.target_port_um,
                    Some(&victim_job.opened_cell_keys),
                )
                .map_err(PyRuntimeError::new_err)?;
            let guided_elapsed_us = native_batch_elapsed_us(guided_start);
            batch.timings.reroute_victims_wall_us += guided_elapsed_us;
            if trace_native_repair {
                eprintln!(
                    "victim_diag net={} strategy=guided elapsed_us={} found={}",
                    victim_job.net_id,
                    guided_elapsed_us,
                    guided_result.is_some()
                );
            }
            if let Some((route, crossing_events)) = guided_result {
                let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
                let crossed_partner_vec: Vec<u64> = crossed_partner_ids.iter().copied().collect();
                match self.commit_native_route_with_clearance_allowing_core_overlap(
                    victim_job.net_id,
                    &route,
                    block_radius_cells,
                    commit_radius_cells,
                    &victim_job.clearance_exempt_cells,
                    core_radius_cells,
                    victim_job.source_port_um,
                    victim_job.target_port_um,
                    Some(&victim_job.opened_cell_keys),
                    &crossed_partner_vec,
                    true,
                ) {
                    Ok(true) => {
                        batch
                            .timings
                            .add_route_result_stats_if(collect_native_timing, &route);
                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "victim_reroute",
                            Some(mode.route_order),
                            Some("guided_collision_crossing"),
                            victim_job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(true),
                            None,
                        );
                        batch.attempts.push(NativeRouteAttempt {
                            bucket_name: "reroute_victims",
                            net_id: victim_job.net_id,
                            route: Some(route.clone()),
                            failed: false,
                            error: None,
                            repair_round: Some(round_idx),
                            candidate_blockers: probe.candidate_blockers.clone(),
                            ripup_ids: ripup_ids.to_vec(),
                        });
                        guided_victim_route = Some(route);
                    }
                    Ok(false) => {}
                    Err(error) => {
                        if trace_native_repair {
                            eprintln!(
                                "native_repair_guided_victim_crossing_commit_failed net={} partner={} error={}",
                                victim_job.net_id, job.net_id, error
                            );
                        }
                    }
                }
            }
        }
        if let Some(route) = guided_victim_route {
            batch.final_routes.insert(victim_job.net_id, route.clone());
            return Ok(CrossingAwareVictimRerouteOutcome::Routed);
        }
        if lidar_crossing_partners_available {
            if trace_native_repair {
                eprintln!(
                    "native_repair_lidar_crossing_blocked_plain_victim_fallback net={} partners_available=true",
                    victim_job.net_id
                );
            }
            mode.mode_failed = true;
            return Ok(CrossingAwareVictimRerouteOutcome::Blocked);
        }
        Ok(CrossingAwareVictimRerouteOutcome::NotAttempted)
    }

    #[allow(clippy::too_many_arguments)]
    /// Mirror of [`Self::try_reroute_current_net_before_victims`], run when
    /// `victim_first` is set in the round/victim-order loop -- reroutes the
    /// current net after victims have already been processed.
    pub(crate) fn try_reroute_current_net_after_victims(
        &mut self,
        batch: &mut RepairBatchState,
        repair: &mut RepairAttemptState,
        probe: &mut ProbeState,
        mode: &mut RepairModeAttemptState,
        job: &NativeRouteJob,
        round_idx: u32,
        active_repair_set_index: usize,
        repair_set_index: usize,
        ripup_ids: &[u64],
        victim_first: bool,
        reverse_victim_order: bool,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        prefer_orthogonal_repair: bool,
        history_weight: f64,
        max_rounds: u32,
        max_victims: usize,
        collect_native_timing: bool,
    ) -> RerouteCurrentNetAfterVictimsOutcome {
        let route_start = native_batch_timer(collect_native_timing);
        let normal_result = self
            .route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                job.net_id,
                job.source,
                job.target,
                block_radius_cells,
                &job.opened_cells,
                &job.opened_cell_keys,
                commit_radius_cells,
                &job.clearance_exempt_cells,
                &job.clearance_exempt_cell_keys,
                core_radius_cells,
                &mode.temporary_probe_reservation,
                job.source_port_um,
                job.target_port_um,
                prefer_orthogonal_repair,
            );
        let route_elapsed_us = native_batch_elapsed_us(route_start);
        batch.timings.repair_failed_net_wall_us += route_elapsed_us;
        let route = match normal_result {
            Ok(route) => {
                batch
                    .timings
                    .add_route_result_stats_if(collect_native_timing, &route);
                remove_success_static_cleanup(&mut self.obstacle_map, job);
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "current_route",
                    Some(mode.route_order),
                    Some("normal_route"),
                    job.net_id,
                    Some(round_idx),
                    Some(active_repair_set_index as u64),
                    &probe.candidate_blockers,
                    ripup_ids,
                    &mode.victim_reroute_ids,
                    Some(victim_first),
                    Some(reverse_victim_order),
                    Some(true),
                    None,
                );
                route
            }
            Err(normal_error) => {
                enqueue_targeted_illegal_crossing_repair_set(
                    &mut repair.repair_victim_sets,
                    &mut probe.candidate_blockers,
                    &repair.round_base_routes,
                    job.net_id,
                    ripup_ids,
                    &normal_error,
                    round_idx,
                    max_rounds,
                    max_victims,
                );
                let learned_repair_keepout = repair
                    .learned_repair_keepouts_by_ripup
                    .entry(ripup_ids.to_vec())
                    .or_default();
                if self.remember_local_repair_error_keepout(learned_repair_keepout, &normal_error) {
                    enqueue_learned_keepout_repair_retry(
                        &mut repair.repair_victim_sets,
                        &mut repair.learned_repair_retry_counts,
                        ripup_ids,
                        round_idx,
                        repair_set_index,
                    );
                }
                let (repair_keepout, extra_repair_keepout) = self
                    .augmented_crossing_error_repair_keepout(
                        &mode.temporary_probe_reservation,
                        &normal_error,
                    );
                if !extra_repair_keepout.is_empty() {
                    self.obstacle_map.add_static_keys(&extra_repair_keepout);
                }
                batch.timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                push_native_repair_trace(
                    &mut batch.repair_trace,
                    "current_route",
                    Some(mode.route_order),
                    Some("normal_route"),
                    job.net_id,
                    Some(round_idx),
                    Some(active_repair_set_index as u64),
                    &probe.candidate_blockers,
                    ripup_ids,
                    &mode.victim_reroute_ids,
                    Some(victim_first),
                    Some(reverse_victim_order),
                    Some(false),
                    Some(normal_error.clone()),
                );
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "repair_failed_net",
                    net_id: job.net_id,
                    route: None,
                    failed: true,
                    error: Some(normal_error),
                    repair_round: Some(round_idx),
                    candidate_blockers: probe.candidate_blockers.clone(),
                    ripup_ids: ripup_ids.to_vec(),
                });
                let repair_start = native_batch_timer(collect_native_timing);
                let repair_result = self
                    .route_single_net_and_commit_repair_native_with_repair_keepout(
                        job.net_id,
                        job.source,
                        job.target,
                        block_radius_cells,
                        &job.opened_cells,
                        &job.opened_cell_keys,
                        history_weight,
                        commit_radius_cells,
                        &job.clearance_exempt_cells,
                        &job.clearance_exempt_cell_keys,
                        core_radius_cells,
                        &repair_keepout,
                        job.source_port_um,
                        job.target_port_um,
                    );
                let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                if !extra_repair_keepout.is_empty() {
                    self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                }
                batch.timings.repair_failed_net_wall_us += repair_elapsed_us;
                match repair_result {
                    Ok(route) => {
                        batch
                            .timings
                            .add_route_result_stats_if(collect_native_timing, &route);
                        remove_success_static_cleanup(&mut self.obstacle_map, job);
                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "current_route",
                            Some(mode.route_order),
                            Some("repair_fallback"),
                            job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(true),
                            None,
                        );
                        route
                    }
                    Err(error) => {
                        enqueue_targeted_illegal_crossing_repair_set(
                            &mut repair.repair_victim_sets,
                            &mut probe.candidate_blockers,
                            &repair.round_base_routes,
                            job.net_id,
                            ripup_ids,
                            &error,
                            round_idx,
                            max_rounds,
                            max_victims,
                        );
                        let learned_repair_keepout = repair
                            .learned_repair_keepouts_by_ripup
                            .entry(ripup_ids.to_vec())
                            .or_default();
                        if self.remember_local_repair_error_keepout(learned_repair_keepout, &error)
                        {
                            enqueue_learned_keepout_repair_retry(
                                &mut repair.repair_victim_sets,
                                &mut repair.learned_repair_retry_counts,
                                ripup_ids,
                                round_idx,
                                repair_set_index,
                            );
                        }
                        batch.timings.repair_failed_net_failed_wall_us += repair_elapsed_us;
                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "current_route",
                            Some(mode.route_order),
                            Some("repair_fallback"),
                            job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(false),
                            Some(error.clone()),
                        );
                        batch.attempts.push(NativeRouteAttempt {
                            bucket_name: "repair_failed_net",
                            net_id: job.net_id,
                            route: None,
                            failed: true,
                            error: Some(error),
                            repair_round: Some(round_idx),
                            candidate_blockers: probe.candidate_blockers.clone(),
                            ripup_ids: ripup_ids.to_vec(),
                        });
                        if mode.temporary_probe_reservation_added {
                            self.obstacle_map
                                .remove_static_keys(&mode.temporary_probe_reservation);
                        }
                        return RerouteCurrentNetAfterVictimsOutcome::Failed;
                    }
                }
            }
        };
        batch.attempts.push(NativeRouteAttempt {
            bucket_name: "repair_failed_net",
            net_id: job.net_id,
            route: Some(route.clone()),
            failed: false,
            error: None,
            repair_round: Some(round_idx),
            candidate_blockers: probe.candidate_blockers.clone(),
            ripup_ids: ripup_ids.to_vec(),
        });
        batch.final_routes.insert(job.net_id, route.clone());
        mode.repaired_route = Some(route);
        RerouteCurrentNetAfterVictimsOutcome::Routed
    }

    pub(crate) fn build_repair_mode_reservation(
        &self,
        batch: &RepairBatchState,
        repair: &RepairAttemptState,
        probe: &ProbeState,
        ripup_ids: &[u64],
        victim_first: bool,
    ) -> (FxHashSet<CellKey>, FxHashSet<CellKey>) {
        let mut victim_first_probe_reservation = FxHashSet::default();
        let temporary_probe_reservation: FxHashSet<CellKey> = if victim_first {
            let mut conflict_keys = self.crossing_physical_violation_repair_keepout_keys(
                &probe.probe_realized_crossing_violations,
                ripup_ids,
            );
            conflict_keys.extend(self.crossing_grid_violation_repair_keepout_keys(
                &probe.probe_grid_crossing_violations,
                ripup_ids,
            ));
            if let Some(learned_keepout) = repair.learned_repair_keepouts_by_ripup.get(ripup_ids) {
                conflict_keys.extend(learned_keepout.iter().copied());
            }
            if let Some(victim_only_keepout) =
                repair.learned_victim_only_keepouts_by_ripup.get(ripup_ids)
            {
                for key in victim_only_keepout {
                    if conflict_keys.insert(*key) {
                        victim_first_probe_reservation.insert(*key);
                    }
                }
            }
            let probe_keys: FxHashSet<CellKey> = probe
                .probe_route
                .cells
                .iter()
                .map(|(x, y)| pack_xy(*x, *y))
                .collect();
            for old_id in ripup_ids {
                if let Some(old_route) = batch.final_routes.get(old_id) {
                    for (x, y) in &old_route.cells {
                        let key = pack_xy(*x, *y);
                        if probe_keys.contains(&key) {
                            if conflict_keys.insert(key) {
                                victim_first_probe_reservation.insert(key);
                            }
                        }
                    }
                }
            }
            conflict_keys
        } else {
            let mut conflict_keys = FxHashSet::default();
            if let Some(learned_keepout) = repair.learned_repair_keepouts_by_ripup.get(ripup_ids) {
                conflict_keys.extend(learned_keepout.iter().copied());
            }
            conflict_keys
        };
        (victim_first_probe_reservation, temporary_probe_reservation)
    }

    /// Shared single-victim "rip up, reroute the current net, reroute the
    /// victim (plain then repair-native fallback)" attempt, extracted
    /// (Milestone 6 of `.agent/execplans/2026-08-24-modular-routing-strategies.md`)
    /// from what were two near-duplicate ~200-line method bodies. Only
    /// [`Self::try_pending_straight_victim_repair`] calls it today (exactly
    /// once, for its single hinted victim); a second caller,
    /// `try_preemptive_crossing_ripup` (an undocumented, off-by-default,
    /// speculative pre-route ripup with no historical bug motivating it),
    /// called this once per candidate victim in a loop and was deleted in
    /// `.agent/execplans/2026-08-25-unify-astar-kernel-and-clean-repair-baseline.md`'s
    /// Milestone 5 after an explicit resolve-or-delete decision found no
    /// rationale for it anywhere in this repository's history. Snapshots
    /// `self`/`batch` state at entry and restores it on any failure, so
    /// repeated calls in a loop are safe -- each call leaves state exactly
    /// as it found it unless it returns `true`. `bucket_name` controls the
    /// `NativeRouteAttempt.bucket_name` recorded for both the current-net
    /// and victim route attempts. History cost for the ripped-up victim's
    /// vacated cells is no longer applied here explicitly (removed
    /// `add_history_for_victim` parameter, dead in practice since the
    /// deleted caller was its only `true` user) -- history now accrues
    /// systematically on every successful commit via
    /// `add_post_commit_guidance_for_route`/`add_repair_history_for_route`,
    /// see `.agent/execplans/2026-08-25-negotiated-repair-engine.md`
    /// Milestone 4. Returns
    /// `true` if both the current net and the victim were successfully
    /// rerouted and committed (`batch.repair_count` bumped, two
    /// `NativeRouteAttempt`s pushed), `false` if any step failed (state
    /// fully restored, one failing `NativeRouteAttempt` pushed).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn try_ripup_single_victim_and_reroute(
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
    ) -> bool {
        self.try_ripup_single_victim_and_reroute_with(
            batch,
            job,
            victim_job,
            bucket_name,
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
            history_weight,
            collect_native_timing,
            false,
        ) == VictimRerouteOutcome::Both
    }

    #[allow(clippy::too_many_arguments)]
    /// Single-victim rip/reroute-both repair, structurally identical to
    /// [`Self::try_preemptive_crossing_ripup`] (same snapshot/rip/reroute-
    /// current/reroute-victim/restore shape) but selects its victim from
    /// the accumulated pending-straight-failure hint (`hint.count` against
    /// `pending_straight_ripup_threshold()`) instead of a bounding-box
    /// scan. Tried as [`Self::try_source_layer_center_out_repair`]'s
    /// fallback when that method's own gate applies but it does not
    /// resolve the net.
    pub(crate) fn try_pending_straight_victim_repair(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        victim_job: &NativeRouteJob,
        hint: &PendingStraightVictimHint,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
    ) -> PendingStraightRepairOutcome {
        if trace_native_repair {
            eprintln!(
                "native_repair_pending_straight_start net={} victim={} count={}",
                job.net_id, hint.victim_net_id, hint.count
            );
        }
        if self.try_ripup_single_victim_and_reroute(
            batch,
            job,
            victim_job,
            "pending_straight_ripup",
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
            0.0,
            collect_native_timing,
        ) {
            PendingStraightRepairOutcome::Routed
        } else {
            PendingStraightRepairOutcome::NotResolved
        }
    }

    #[allow(clippy::too_many_arguments)]
    /// Last resort when every other strategy in this file has failed
    /// (`!repair.repaired`): resets to round-base state, and -- only if
    /// collision-crossing repair is enabled, not restricted to expected
    /// pairs, and the probe found realized (not grid) violations --
    /// attempts committing the original probe route despite its violations
    /// anyway, retrying up to 12 rounds against validation feedback via
    /// `crossing_error_repair_keepout_keys_with_options`. If this also
    /// fails, constructs the final
    /// `"No repair route found; candidate_blockers=...; recent_errors=..."`
    /// error text -- the literal source of every `RuntimeError` this
    /// repository's n_70 and n_50 investigations have parsed; any future
    /// redesign touching this method's error format must update
    /// `illegal_crossing_net_ids_from_error` and its siblings in lockstep.
    pub(crate) fn try_final_repair_fallback(
        &mut self,
        batch: &mut RepairBatchState,
        repair: RepairAttemptState,
        probe: ProbeState,
        job: &NativeRouteJob,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        max_rounds: u32,
        collect_native_timing: bool,
    ) -> Result<(), ()> {
        if repair.repaired {
            return Ok(());
        }
        let reset_start = native_batch_timer(collect_native_timing);
        self.obstacle_map = repair.round_base_map;
        self.committed_center_routes = repair.round_base_center_routes;
        self.committed_realized_center_routes = repair.round_base_realized_center_routes;
        self.committed_target_terminal_bump_guards = repair.round_base_target_terminal_bump_guards;
        self.committed_opened_cell_keys = repair.round_base_opened_cell_keys;
        self.crossing_events = repair.round_base_crossing_events;
        self.invalidate_meander_base_prefix();
        batch.final_routes = repair.round_base_routes;
        batch.timings.repair_state_reset_us += native_batch_elapsed_us(reset_start);
        let allow_lidar_pure_probe_commit = probe.crossing_repair_enabled
            && !self.crossing_context.config().allow_only_expected_pairs
            && probe.probe_grid_crossing_violations.is_empty()
            && !probe.probe_realized_crossing_violations.is_empty();
        if allow_lidar_pure_probe_commit {
            let validate_lidar_pure_probe_commit = true;
            let commit_start = native_batch_timer(collect_native_timing);
            let commit_result = self.commit_native_route_with_clearance_allowing_core_overlap(
                job.net_id,
                &probe.probe_route,
                block_radius_cells,
                commit_radius_cells,
                &job.clearance_exempt_cells,
                core_radius_cells,
                job.source_port_um,
                job.target_port_um,
                Some(&job.opened_cell_keys),
                &probe.candidate_blockers,
                validate_lidar_pure_probe_commit,
            );
            let commit_elapsed_us = native_batch_elapsed_us(commit_start);
            batch.timings.commit_update_dynamic_map_us += commit_elapsed_us;
            match commit_result {
                Ok(true) => {
                    batch.attempts.push(NativeRouteAttempt {
                        bucket_name: "lidar_pure_probe_commit",
                        net_id: job.net_id,
                        route: Some(probe.probe_route.clone()),
                        failed: false,
                        error: None,
                        repair_round: Some(max_rounds),
                        candidate_blockers: probe.candidate_blockers.clone(),
                        ripup_ids: Vec::new(),
                    });
                    batch.final_routes.insert(job.net_id, probe.probe_route);
                    batch.repair_count = batch.repair_count.saturating_add(1);
                    return Ok(());
                }
                Err(error) if validate_lidar_pure_probe_commit => {
                    batch.attempts.push(NativeRouteAttempt {
                        bucket_name: "lidar_pure_probe_commit",
                        net_id: job.net_id,
                        route: None,
                        failed: true,
                        error: Some(error.clone()),
                        repair_round: Some(max_rounds),
                        candidate_blockers: probe.candidate_blockers.clone(),
                        ripup_ids: Vec::new(),
                    });
                    let mut validation_keepout =
                        self.crossing_error_repair_keepout_keys_with_options(&error, true);
                    if !validation_keepout.is_empty() {
                        let repair_start = native_batch_timer(collect_native_timing);
                        let mut repair_result = Err(error.clone());
                        for _feedback_attempt in 0..12 {
                            self.obstacle_map.add_static_keys(&validation_keepout);
                            let probe_result = self.route_single_net_ignore_dynamic_native(
                                job.source,
                                job.target,
                                Some(&job.opened_cells),
                                Some(&job.opened_cell_keys),
                            );
                            self.obstacle_map.remove_static_keys(&validation_keepout);

                            let attempt_result = match probe_result {
                                Ok(route) => {
                                    match self
                                        .commit_native_route_with_clearance_allowing_core_overlap(
                                            job.net_id,
                                            &route,
                                            block_radius_cells,
                                            commit_radius_cells,
                                            &job.clearance_exempt_cells,
                                            core_radius_cells,
                                            job.source_port_um,
                                            job.target_port_um,
                                            Some(&job.opened_cell_keys),
                                            &probe.candidate_blockers,
                                            true,
                                        ) {
                                        Ok(true) => Ok(route),
                                        Ok(false) => {
                                            Err("Failed to commit validation-feedback route"
                                                .to_string())
                                        }
                                        Err(error) => Err(error),
                                    }
                                }
                                Err(error) => Err(error),
                            };

                            match attempt_result {
                                Ok(route) => {
                                    repair_result = Ok(route);
                                    break;
                                }
                                Err(retry_error) => {
                                    let extra_keepout = self
                                        .crossing_error_repair_keepout_keys_with_options(
                                            &retry_error,
                                            true,
                                        );
                                    let mut added_any = false;
                                    for key in extra_keepout {
                                        if validation_keepout.insert(key) {
                                            added_any = true;
                                        }
                                    }
                                    repair_result = Err(retry_error);
                                    if !added_any {
                                        break;
                                    }
                                }
                            }
                        }
                        let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                        batch.timings.repair_failed_net_wall_us += repair_elapsed_us;
                        match repair_result {
                            Ok(route) => {
                                batch
                                    .timings
                                    .add_route_result_stats_if(collect_native_timing, &route);
                                remove_success_static_cleanup(&mut self.obstacle_map, job);
                                batch.attempts.push(NativeRouteAttempt {
                                    bucket_name: "repair_failed_net",
                                    net_id: job.net_id,
                                    route: Some(route.clone()),
                                    failed: false,
                                    error: None,
                                    repair_round: Some(max_rounds),
                                    candidate_blockers: probe.candidate_blockers.clone(),
                                    ripup_ids: Vec::new(),
                                });
                                batch.final_routes.insert(job.net_id, route);
                                batch.repair_count = batch.repair_count.saturating_add(1);
                                return Ok(());
                            }
                            Err(retry_error) => {
                                batch.timings.repair_failed_net_failed_wall_us += repair_elapsed_us;
                                batch.attempts.push(NativeRouteAttempt {
                                    bucket_name: "repair_failed_net",
                                    net_id: job.net_id,
                                    route: None,
                                    failed: true,
                                    error: Some(retry_error),
                                    repair_round: Some(max_rounds),
                                    candidate_blockers: probe.candidate_blockers.clone(),
                                    ripup_ids: Vec::new(),
                                });
                            }
                        }
                    }
                }
                Ok(false) | Err(_) => {}
            }
        }
        batch.failed_net_id = Some(job.net_id);
        let recent_errors: Vec<String> = batch
            .attempts
            .iter()
            .rev()
            .filter(|attempt| {
                attempt.repair_round.is_some()
                    && (attempt.net_id == job.net_id
                        || probe.candidate_blockers.contains(&attempt.net_id))
            })
            .filter_map(|attempt| {
                attempt.error.as_ref().map(|error| {
                    format!(
                        "{}:net{}:round{:?}:rip{:?}:{}",
                        attempt.bucket_name,
                        attempt.net_id,
                        attempt.repair_round,
                        attempt.ripup_ids,
                        error
                    )
                })
            })
            .take(8)
            .collect();
        batch.failed_error = Some(format!(
            "No repair route found; candidate_blockers={:?}; recent_errors={recent_errors:?}",
            probe.candidate_blockers
        ));
        Err(())
    }

    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn orthogonal_repair_primitives(&self) -> PrimitiveLibrary {
        let primitives_per_angle: Vec<Vec<Primitive>> = (0u8..8u8)
            .map(|angle| {
                if angle % 2 != 0 {
                    return Vec::new();
                }
                self.primitives
                    .get_primitives_for_angle(angle)
                    .iter()
                    .filter(|primitive| match primitive.geometry {
                        PrimitiveGeometry::Straight { .. } => primitive.end_angle % 2 == 0,
                        PrimitiveGeometry::Bend { angle_delta, .. } => {
                            primitive.end_angle % 2 == 0 && angle_delta.unsigned_abs() == 2
                        }
                    })
                    .cloned()
                    .collect()
            })
            .collect();
        PrimitiveLibrary::new(primitives_per_angle, self.primitives.grid_size_um())
    }

    #[allow(clippy::too_many_arguments)]
    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn route_single_net_and_commit_orthogonal_native_with_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        let filtered_opened;
        let filtered_opened_keys;
        let (_, opened_keys_for_route) = if repair_keepout.is_empty() {
            (opened_cells, opened_cell_keys)
        } else {
            filtered_opened =
                opened_cells_excluding_keepout(opened_cells, repair_keepout, source, target);
            if filtered_opened.len() == opened_cells.len() {
                (opened_cells, opened_cell_keys)
            } else {
                filtered_opened_keys = pack_cells(&filtered_opened);
                (filtered_opened.as_slice(), &filtered_opened_keys)
            }
        };
        let mut cfg = self.astar_config(Some(false), Some(false), Some(0.0))?;
        cfg.require_terminal_straights = false;
        cfg.enable_simple_routes = false;
        cfg.enable_jps4 = false;
        let orthogonal_primitives = self.orthogonal_repair_primitives();
        let env = SearchEnvironment {
            obstacle_map: &self.obstacle_map,
            primitives: &orthogonal_primitives,
        };
        let request = SearchRequest {
            source: State::new(source.x, source.y, source.angle),
            target: State::new(target.x, target.y, target.angle),
            port_open_cells: Some(opened_keys_for_route),
            dynamic_expansion: None,
            crossing: None,
            config: &cfg,
        };
        let route = self
            .search_engine
            .search(&env, &request)
            .route
            .ok_or_else(|| "No route found".to_string())?;
        if self.commit_native_route_with_clearance(
            net_id,
            &route,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
            Some(opened_keys_for_route),
        ) {
            Ok(route)
        } else {
            Err("Failed to commit orthogonal repair route".to_string())
        }
    }

    #[allow(clippy::too_many_arguments)]
    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        clearance_exempt_cell_keys: &FxHashSet<CellKey>,
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        prefer_orthogonal: bool,
    ) -> Result<RouteResult, String> {
        // Try the normal, diagonal-capable search first even when
        // `prefer_orthogonal` is set: it already enforces the same
        // crossing-legality checks (including perpendicularity) used
        // everywhere else in the router, so a route it commits is exactly as
        // legal as one from the orthogonal-only search -- just potentially
        // shorter and less likely to force an unnecessary crossing with a
        // neighboring net (confirmed visually on multiportmmi_16x16, where
        // an orthogonal-first detour forced crossings in an otherwise
        // trivially routable region). The orthogonal-only search
        // (`orthogonal_repair_primitives`, which excludes every diagonal
        // primitive) remains the fallback for when the normal search can't
        // find any legal route at all -- unchanged safety net, just no
        // longer tried first.
        let normal_result = self.route_single_net_and_commit_native_with_repair_keepout(
            net_id,
            source,
            target,
            block_radius_cells,
            opened_cells,
            opened_cell_keys,
            commit_radius_cells,
            clearance_exempt_cells,
            clearance_exempt_cell_keys,
            core_radius_cells,
            repair_keepout,
            source_port_um,
            target_port_um,
        );
        if normal_result.is_ok()
            || !prefer_orthogonal
            || !self
                .router_config
                .negotiation
                .enable_orthogonal_repair_fallback
        {
            return normal_result;
        }
        self.route_single_net_and_commit_orthogonal_native_with_repair_keepout(
            net_id,
            source,
            target,
            block_radius_cells,
            opened_cells,
            opened_cell_keys,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            repair_keepout,
            source_port_um,
            target_port_um,
        )
        .or(normal_result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;

    #[test]
    fn enqueue_deferred_jobs_appends_in_order_skips_dupes_and_clears() {
        let mut queue: std::collections::VecDeque<usize> = [0usize, 1, 2].into_iter().collect();
        let mut deferred = vec![2usize, 5, 3];
        enqueue_deferred_jobs(&mut queue, &mut deferred);
        // 2 is already in the queue and is skipped; 5 and 3 are appended
        // to the back, in order.
        assert_eq!(
            queue,
            [0usize, 1, 2, 5, 3]
                .into_iter()
                .collect::<std::collections::VecDeque<usize>>()
        );
        assert!(deferred.is_empty());
    }

    #[test]
    fn compute_repair_victim_sets_disabled_uses_round_expansion_only() {
        assert_eq!(
            compute_repair_victim_sets(false, &[], &[10, 20, 30], 2, 2),
            vec![(1, vec![10, 20]), (2, vec![10, 20, 30])]
        );
    }

    #[test]
    fn compute_repair_victim_sets_enabled_without_violations_adds_singletons() {
        assert_eq!(
            compute_repair_victim_sets(true, &[], &[10, 20, 30], 2, 2),
            vec![
                (1, vec![10]),
                (1, vec![20]),
                (1, vec![10, 20]),
                (2, vec![10, 20, 30]),
            ]
        );
    }

    #[test]
    fn compute_repair_victim_sets_enabled_with_realized_violations_adds_top_pair() {
        let violations = vec![dummy_invalid_crossing_intersection()];

        assert_eq!(
            compute_repair_victim_sets(true, &violations, &[10, 20, 30, 40], 2, 2),
            vec![
                (1, vec![10]),
                (1, vec![20]),
                (1, vec![10, 20]),
                (2, vec![10, 20, 30, 40]),
            ]
        );
    }

    #[test]
    fn compute_repair_victim_sets_enabled_with_empty_blockers_returns_empty() {
        assert_eq!(
            compute_repair_victim_sets(true, &[], &[], 2, 2),
            Vec::<(u32, Vec<u64>)>::new()
        );
    }

    #[test]
    fn targeted_illegal_crossing_repair_promotes_learned_blocker() {
        let mut final_routes = FxHashMap::default();
        final_routes.insert(31, empty_test_route());
        final_routes.insert(33, empty_test_route());
        final_routes.insert(36, empty_test_route());

        let mut repair_victim_sets = vec![(1, vec![36]), (1, vec![31])];
        let mut candidate_blockers = vec![36, 31];
        enqueue_targeted_illegal_crossing_repair_set(
            &mut repair_victim_sets,
            &mut candidate_blockers,
            &final_routes,
            36,
            &[36, 31],
            "Illegal realized crossing: net 36 intersects net 33 at (0.000, 0.000) (not_perpendicular)",
            2,
            4,
            8,
        );

        assert_eq!(candidate_blockers, vec![36, 31, 33]);
        assert!(repair_victim_sets
            .iter()
            .any(|(round, ids)| *round == 3 && ids == &vec![36, 31, 33]));
    }

    #[test]
    fn targeted_illegal_crossing_repair_promotes_blocker_when_queue_capped() {
        let mut final_routes = FxHashMap::default();
        final_routes.insert(31, empty_test_route());
        final_routes.insert(33, empty_test_route());
        final_routes.insert(36, empty_test_route());

        let mut repair_victim_sets = vec![
            (1, vec![36]),
            (1, vec![31]),
            (2, vec![36, 31]),
            (2, vec![31, 36]),
            (3, vec![36]),
            (3, vec![31]),
            (4, vec![36, 31]),
            (4, vec![31, 36]),
        ];
        let mut candidate_blockers = vec![36, 31];
        enqueue_targeted_illegal_crossing_repair_set(
            &mut repair_victim_sets,
            &mut candidate_blockers,
            &final_routes,
            36,
            &[36, 31],
            "Illegal realized crossing: net 36 intersects net 33 at (0.000, 0.000) (not_perpendicular)",
            2,
            4,
            8,
        );

        assert_eq!(candidate_blockers, vec![36, 31, 33]);
        assert_eq!(repair_victim_sets.len(), 8);
    }

    #[test]
    fn learned_keepout_retry_is_capped_per_ripup_set() {
        let mut repair_victim_sets = Vec::new();
        let mut retry_counts = FxHashMap::default();
        for _ in 0..8 {
            let next_repair_set_index = repair_victim_sets.len();
            enqueue_learned_keepout_repair_retry(
                &mut repair_victim_sets,
                &mut retry_counts,
                &[36, 31],
                1,
                next_repair_set_index,
            );
        }

        assert_eq!(repair_victim_sets.len(), 1);
        assert_eq!(retry_counts.get(&vec![36, 31]).copied(), Some(1));
    }

    /// Crossings disabled: the vertical net routes straight down once the
    /// horizontal partner is ripped, and the partner can then not be
    /// rerouted (it would have to cross the vertical). With
    /// `keep_net_on_victim_failure` the net's route survives and the
    /// partner stays ripped; without it everything rolls back.
    #[test]
    fn victim_reroute_keeps_the_net_and_leaves_the_victim_ripped_when_asked() {
        for keep in [false, true] {
            let (mut router, mut batch, vertical) = single_horizontal_crossing_fixture();
            router.set_collision_crossing_routing(false);
            router.crossing_context.set_config(CrossingConfig {
                enabled: false,
                ..CrossingConfig::default()
            });
            let horizontal = NativeRouteJob::new(
                1,
                PyState::new(5, 30, 0),
                PyState::new(54, 30, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            );

            let outcome = router.try_ripup_single_victim_and_reroute_with(
                &mut batch,
                &vertical,
                &horizontal,
                "braid_ripup",
                FIXTURE_CLEARANCE_RADIUS_CELLS,
                Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                0.0,
                false,
                keep,
            );

            if keep {
                assert_eq!(outcome, VictimRerouteOutcome::NetOnly);
                assert!(
                    batch.final_routes.contains_key(&2)
                        && router.obstacle_map.get_net_cells(2).is_some(),
                    "the net's braid-free route is kept"
                );
                assert!(
                    !batch.final_routes.contains_key(&1)
                        && router.obstacle_map.get_net_cells(1).is_none(),
                    "the victim stays ripped for the caller to re-queue"
                );
            } else {
                assert_eq!(outcome, VictimRerouteOutcome::Failed);
                assert!(
                    !batch.final_routes.contains_key(&2)
                        && router.obstacle_map.get_net_cells(2).is_none(),
                    "without the flag the net's attempt rolls back"
                );
                assert!(
                    router.obstacle_map.get_net_cells(1).is_some(),
                    "without the flag the victim is restored"
                );
            }
        }
    }

    #[test]
    fn local_repair_error_keepout_learns_dynamic_overlap_for_capped_retry() {
        let router = PyPhotonicRouter::new(
            PyGridSpec::new(40, 40, 0.5, 0.0, 0.0).unwrap(),
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
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

        let mut learned_keepout = FxHashSet::default();
        let learned = router.remember_local_repair_error_keepout(
            &mut learned_keepout,
            "Failed to commit routed cells to obstacle map: dynamic_overlap_count=1 dynamic_overlap_owners=[36] dynamic_overlap_bbox=(10,10,20,20) dynamic_overlap_sample=(10,20)",
        );

        assert!(learned);
        assert!(learned_keepout.contains(&pack_xy(9, 19)));
        assert!(learned_keepout.contains(&pack_xy(11, 21)));
        assert!(!learned_keepout.contains(&pack_xy(8, 18)));

        let mut repair_victim_sets = Vec::new();
        let mut retry_counts = FxHashMap::default();
        enqueue_learned_keepout_repair_retry(
            &mut repair_victim_sets,
            &mut retry_counts,
            &[36, 31],
            1,
            0,
        );
        enqueue_learned_keepout_repair_retry(
            &mut repair_victim_sets,
            &mut retry_counts,
            &[36, 31],
            1,
            0,
        );

        assert_eq!(repair_victim_sets, vec![(1, vec![36, 31])]);
        assert_eq!(retry_counts.get(&vec![36, 31]).copied(), Some(1));
    }

    #[test]
    fn victim_repair_error_keepout_routes_current_owned_dynamic_overlap_to_victim_only() {
        let router = PyPhotonicRouter::new(
            PyGridSpec::new(40, 40, 0.5, 0.0, 0.0).unwrap(),
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
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
        let error = "Failed to commit routed cells to obstacle map: dynamic_overlap_count=1 dynamic_overlap_owners=[33] dynamic_overlap_bbox=(10,10,20,20) dynamic_overlap_sample=(10,20)";
        assert_eq!(dynamic_commit_error_overlap_owner_ids(error), vec![33]);

        let mut shared_keepout = FxHashSet::default();
        let mut victim_only_keepout = FxHashSet::default();
        let learned = router.remember_victim_repair_error_keepout(
            &mut shared_keepout,
            &mut victim_only_keepout,
            error,
            33,
        );

        assert!(learned);
        assert!(shared_keepout.is_empty());
        assert!(victim_only_keepout.contains(&pack_xy(9, 19)));
        assert!(victim_only_keepout.contains(&pack_xy(11, 21)));

        let mut shared_for_other_owner = FxHashSet::default();
        let mut victim_only_for_other_owner = FxHashSet::default();
        let learned_other = router.remember_victim_repair_error_keepout(
            &mut shared_for_other_owner,
            &mut victim_only_for_other_owner,
            "Failed to commit routed cells to obstacle map: dynamic_overlap_count=1 dynamic_overlap_owners=[36] dynamic_overlap_bbox=(10,10,20,20) dynamic_overlap_sample=(10,20)",
            33,
        );

        assert!(learned_other);
        assert!(shared_for_other_owner.contains(&pack_xy(9, 19)));
        assert!(victim_only_for_other_owner.is_empty());

        let mut crossing_shared = FxHashSet::default();
        let mut crossing_victim_only = FxHashSet::default();
        let learned_crossing = router.remember_victim_repair_error_keepout(
            &mut crossing_shared,
            &mut crossing_victim_only,
            "Illegal realized crossing: net 36 intersects net 33 at (5.000, 5.000) (not_perpendicular)",
            33,
        );

        assert!(learned_crossing);
        assert!(crossing_shared.is_empty());
        assert!(crossing_victim_only.contains(&pack_xy(10, 10)));
    }
}

impl PyPhotonicRouter {
    #[allow(clippy::too_many_arguments)]
    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn route_many_with_repair_and_commit_impl(
        &mut self,
        py: Python<'_>,
        jobs: Vec<(
            u64,
            PyState,
            PyState,
            Vec<(i32, i32)>,
            Vec<(i32, i32)>,
            Vec<(i32, i32)>,
            Option<(f64, f64)>,
            Option<(f64, f64)>,
        )>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        max_rounds: u32,
        max_victims_per_failure: usize,
        history_weight: f64,
        history_increment: u32,
    ) -> PyResult<PyObject> {
        self.obstacle_map.clear_congestion();
        self.long_straight_congestion_cells.clear();
        self.long_straight_congestion_records.clear();
        self.commit_history_increment = history_increment;
        self.commit_history_block_radius_cells = block_radius_cells;
        // This engine already threads its own `history_weight` explicitly
        // into the specific `try_*` methods that need it
        // (`self.astar_config(Some(false), Some(false), Some(history_weight))`);
        // `route_single_net_and_commit_native` (used by `try_plain_normal_route`,
        // "no repair awareness" by design) must stay at its historical,
        // unweighted baseline here, not pick up a stale value left by a
        // prior `route_many_with_negotiated_repair_and_commit` call on this
        // router instance.
        self.commit_history_weight = 0.0;
        // Same reasoning as `commit_history_weight` above: this engine's
        // searches must stay unbounded (the chain keeps its existing
        // per-window `max_iterations` behaviour untouched), not inherit a
        // fail-fast budget a prior negotiated-engine call left set.
        self.negotiated_search_budget = None;
        let collect_native_timing = self.astar_cfg.collect_detailed_timing;
        let mut batch = RepairBatchState {
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
        };
        let unpack_start = native_batch_timer(collect_native_timing);
        let native_jobs: Vec<NativeRouteJob> = jobs
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
        batch.timings.route_job_unpack_us += native_batch_elapsed_us(unpack_start);
        let order_by_id: FxHashMap<u64, usize> = native_jobs
            .iter()
            .enumerate()
            .map(|(index, job)| (job.net_id, index))
            .collect();
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
        let trace_native_progress = self.router_config.diagnostics.native_progress;
        let trace_native_repair = self.router_config.diagnostics.native_repair_diag;

        let mut queue: std::collections::VecDeque<usize> = (0..native_jobs.len()).collect();
        'route_jobs: loop {
            enqueue_deferred_jobs(&mut queue, &mut batch.deferred_job_indices);
            let Some(job_index) = queue.pop_front() else {
                break;
            };
            let job = &native_jobs[job_index];
            if batch.final_routes.contains_key(&job.net_id) {
                if trace_native_progress {
                    eprintln!(
                        "native_route_skip_already_routed index={} net_id={}",
                        job_index + 1,
                        job.net_id
                    );
                }
                continue;
            }
            if trace_native_progress {
                let now = Instant::now();
                if let Some(last_start) = batch.trace_last_route_start.replace(now) {
                    eprintln!(
                        "native_route_elapsed previous_index={} elapsed_s={:.6}",
                        job_index,
                        last_start.elapsed().as_secs_f64()
                    );
                }
                eprintln!(
                    "native_route_start index={} net_id={} source=({}, {}, {}) target=({}, {}, {})",
                    job_index + 1,
                    job.net_id,
                    job.source.x,
                    job.source.y,
                    job.source.angle,
                    job.target.x,
                    job.target.y,
                    job.target.angle
                );
            }
            match self.try_plain_normal_route(
                &mut batch,
                job,
                block_radius_cells,
                commit_radius_cells,
                core_radius_cells,
                collect_native_timing,
            ) {
                PlainRouteOutcome::Routed => {
                    // Chain path: `self.negotiated_search_budget` stays
                    // `None` here (this function never sets it), so both of
                    // `try_braid_repair`'s searches keep this chain's
                    // existing unbounded behaviour -- see
                    // `astar_config_leaves_total_expansion_budget_none_by_default`.
                    self.try_braid_repair(
                        &mut batch,
                        job,
                        &job_by_id,
                        block_radius_cells,
                        commit_radius_cells,
                        core_radius_cells,
                        collect_native_timing,
                        trace_native_repair,
                        None,
                        false,
                    );
                    continue 'route_jobs;
                }
                PlainRouteOutcome::NotResolved => {}
            }

            if self.lidar_pure_crossing_enabled() && self.use_collision_crossing_routing {
                if let Some(hint) = self.pending_straight_victim_hint_for(job.net_id) {
                    if hint.victim_net_id != job.net_id
                        && batch.final_routes.contains_key(&hint.victim_net_id)
                    {
                        if let Some(victim_job) = job_by_id.get(&hint.victim_net_id) {
                            match self.try_source_layer_center_out_repair(
                                &mut batch,
                                &native_jobs,
                                &job_by_id,
                                job,
                                job_index,
                                &hint,
                                &source_layer_indices_by_x,
                                &order_by_id,
                                block_radius_cells,
                                commit_radius_cells,
                                core_radius_cells,
                                collect_native_timing,
                                trace_native_repair,
                            ) {
                                SourceLayerCenterOutOutcome::Routed => continue 'route_jobs,
                                SourceLayerCenterOutOutcome::NotAttempted => {}
                            }
                            match self.try_pending_straight_victim_repair(
                                &mut batch,
                                job,
                                victim_job,
                                &hint,
                                block_radius_cells,
                                commit_radius_cells,
                                core_radius_cells,
                                collect_native_timing,
                                trace_native_repair,
                            ) {
                                PendingStraightRepairOutcome::Routed => continue 'route_jobs,
                                PendingStraightRepairOutcome::NotResolved => {}
                            }
                        }
                    }
                }
            }

            match self.try_lidar_direct_crossing_subset(
                &mut batch,
                job,
                &order_by_id,
                block_radius_cells,
                commit_radius_cells,
                core_radius_cells,
                collect_native_timing,
                trace_native_repair,
                None,
            )? {
                LidarDirectCrossingOutcome::Routed => continue 'route_jobs,
                LidarDirectCrossingOutcome::NotResolved => {}
            }

            let mut probe = match self.probe_net_for_repair(
                &mut batch,
                job,
                &order_by_id,
                block_radius_cells,
                commit_radius_cells,
                collect_native_timing,
                trace_native_repair,
            ) {
                Ok(probe) => probe,
                Err(()) => break 'route_jobs,
            };
            // Still needed here even though `try_guided_collision_crossing`
            // (the only other former reader of this flag, deleted in
            // `.agent/execplans/2026-08-25-unify-astar-kernel-and-clean-repair-baseline.md`'s
            // Milestone 5) is gone: `try_crossing_aware_victim_reroute`,
            // called later in this same function's round-based repair loop,
            // still takes it as a parameter for its own "guided" per-victim
            // attempt.
            let guided_collision_crossing_enabled =
                self.router_config.crossing.enable_guided_collision_crossing
                    && !self
                        .router_config
                        .crossing
                        .disable_guided_collision_crossing;
            match self.try_localized_crossing_keepout_retry(
                &mut batch,
                &probe,
                job,
                block_radius_cells,
                commit_radius_cells,
                core_radius_cells,
                history_weight,
                collect_native_timing,
                trace_native_repair,
            ) {
                LocalizedKeepoutOutcome::Routed => continue 'route_jobs,
                LocalizedKeepoutOutcome::NotResolved => {}
            }
            match self.try_commit_clean_probe(
                &mut batch,
                &probe,
                job,
                block_radius_cells,
                commit_radius_cells,
                core_radius_cells,
                collect_native_timing,
            ) {
                Ok(CommitIfCleanOutcome::Routed) => continue 'route_jobs,
                Ok(CommitIfCleanOutcome::NotResolved) => {}
                Err(()) => break 'route_jobs,
            }

            let max_rounds = max_rounds.max(1);
            let max_victims = max_victims_per_failure.max(1);
            let mut repair =
                self.prepare_repair_attempt(&mut batch, &probe, max_rounds, max_victims);

            let prefer_orthogonal_repair = probe.crossing_repair_enabled
                && !probe.probe_realized_crossing_violations.is_empty()
                && (probe.probe_grid_crossing_violations.is_empty()
                    || probe.candidate_blockers.len() > 2);
            let mut repair_set_index = 0usize;
            while repair_set_index < repair.repair_victim_sets.len() {
                let active_repair_set_index = repair_set_index;
                let (round_idx, ripup_ids) =
                    repair.repair_victim_sets[active_repair_set_index].clone();
                repair_set_index += 1;
                for victim_first in [false, true] {
                    for reverse_victim_order in [false, true] {
                        if reverse_victim_order && ripup_ids.len() <= 1 {
                            continue;
                        }
                        self.reset_repair_attempt_state_from_round_base(
                            &mut batch,
                            &repair,
                            collect_native_timing,
                        );
                        let (route_order, victim_reroute_ids) = self
                            .label_and_trace_repair_mode_start(
                                &mut batch,
                                &probe,
                                job,
                                round_idx,
                                active_repair_set_index,
                                &ripup_ids,
                                victim_first,
                                reverse_victim_order,
                            );

                        let (victim_first_probe_reservation, temporary_probe_reservation) = self
                            .build_repair_mode_reservation(
                                &batch,
                                &repair,
                                &probe,
                                &ripup_ids,
                                victim_first,
                            );
                        if trace_native_repair && !temporary_probe_reservation.is_empty() {
                            eprintln!(
                            "native_repair_keepout net={} ripup={:?} victim_first={} reverse={} keys={}",
                            job.net_id,
                            ripup_ids,
                            victim_first,
                            reverse_victim_order,
                            temporary_probe_reservation.len(),
                        );
                        }

                        let lidar_pure_crossing_repair = probe.crossing_repair_enabled
                            && self.use_collision_crossing_routing
                            && !self.crossing_context.config().allow_only_expected_pairs;
                        self.ripup_repair_set_victims(
                            &mut batch,
                            &ripup_ids,
                            collect_native_timing,
                        );

                        let temporary_probe_reservation_added =
                            if !temporary_probe_reservation.is_empty() {
                                self.obstacle_map
                                    .add_static_keys(&temporary_probe_reservation);
                                true
                            } else {
                                false
                            };
                        let victim_reroute_only_reservation = FxHashSet::default();
                        let victim_reroute_only_reservation_added = false;
                        let mode_failed = false;
                        let repaired_route: Option<RouteResult> = None;
                        let mut mode = RepairModeAttemptState {
                            route_order,
                            victim_reroute_ids,
                            victim_first_probe_reservation,
                            temporary_probe_reservation,
                            temporary_probe_reservation_added,
                            victim_reroute_only_reservation,
                            victim_reroute_only_reservation_added,
                            mode_failed,
                            repaired_route,
                        };
                        if !victim_first {
                            self.try_reroute_current_net_before_victims(
                                &mut batch,
                                &mut repair,
                                &mut probe,
                                &mut mode,
                                job,
                                round_idx,
                                active_repair_set_index,
                                repair_set_index,
                                &ripup_ids,
                                victim_first,
                                reverse_victim_order,
                                block_radius_cells,
                                commit_radius_cells,
                                core_radius_cells,
                                prefer_orthogonal_repair,
                                history_weight,
                                max_rounds,
                                max_victims,
                                collect_native_timing,
                            );
                        }

                        if !mode.mode_failed
                            && !victim_first
                            && mode.temporary_probe_reservation_added
                            && (probe.candidate_blockers.len() > 2
                                || (probe.probe_grid_crossing_violations.is_empty()
                                    && !probe.probe_realized_crossing_violations.is_empty()))
                        {
                            self.obstacle_map
                                .remove_static_keys(&mode.temporary_probe_reservation);
                            mode.temporary_probe_reservation_added = false;
                        }

                        if !mode.mode_failed && !victim_first {
                            if let Some(victim_only_keepout) =
                                repair.learned_victim_only_keepouts_by_ripup.get(&ripup_ids)
                            {
                                for key in victim_only_keepout {
                                    if !mode.temporary_probe_reservation.contains(key) {
                                        mode.victim_reroute_only_reservation.insert(*key);
                                    }
                                }
                                if !mode.victim_reroute_only_reservation.is_empty() {
                                    self.obstacle_map
                                        .add_static_keys(&mode.victim_reroute_only_reservation);
                                    mode.victim_reroute_only_reservation_added = true;
                                }
                            }
                        }

                        if !mode.mode_failed {
                            let victim_reroute_ids_snapshot = mode.victim_reroute_ids.clone();
                            for old_id in &victim_reroute_ids_snapshot {
                                let Some(victim_job) = job_by_id.get(old_id) else {
                                    mode.mode_failed = true;
                                    break;
                                };
                                match self.try_crossing_aware_victim_reroute(
                                    &mut batch,
                                    &repair,
                                    &probe,
                                    &mut mode,
                                    job,
                                    victim_job,
                                    round_idx,
                                    active_repair_set_index,
                                    &ripup_ids,
                                    victim_first,
                                    reverse_victim_order,
                                    lidar_pure_crossing_repair,
                                    guided_collision_crossing_enabled,
                                    block_radius_cells,
                                    commit_radius_cells,
                                    core_radius_cells,
                                    collect_native_timing,
                                    trace_native_repair,
                                )? {
                                    CrossingAwareVictimRerouteOutcome::Routed => continue,
                                    CrossingAwareVictimRerouteOutcome::Blocked => break,
                                    CrossingAwareVictimRerouteOutcome::NotAttempted => {}
                                }
                                match self.reroute_victim_with_plain_fallback(
                                    &mut batch,
                                    &mut repair,
                                    &mut probe,
                                    &mut mode,
                                    job,
                                    victim_job,
                                    round_idx,
                                    active_repair_set_index,
                                    repair_set_index,
                                    &ripup_ids,
                                    victim_first,
                                    reverse_victim_order,
                                    block_radius_cells,
                                    commit_radius_cells,
                                    core_radius_cells,
                                    prefer_orthogonal_repair,
                                    history_weight,
                                    max_rounds,
                                    max_victims,
                                    collect_native_timing,
                                ) {
                                    VictimPlainRerouteOutcome::Routed => {}
                                    VictimPlainRerouteOutcome::Failed => break,
                                }
                            }
                        }

                        if !mode.mode_failed
                            && victim_first
                            && mode.temporary_probe_reservation_added
                            && !mode.victim_first_probe_reservation.is_empty()
                        {
                            self.obstacle_map
                                .remove_static_keys(&mode.victim_first_probe_reservation);
                            for key in &mode.victim_first_probe_reservation {
                                mode.temporary_probe_reservation.remove(key);
                            }
                            if mode.temporary_probe_reservation.is_empty() {
                                mode.temporary_probe_reservation_added = false;
                            }
                        }

                        if !mode.mode_failed && victim_first {
                            match self.try_reroute_current_net_after_victims(
                                &mut batch,
                                &mut repair,
                                &mut probe,
                                &mut mode,
                                job,
                                round_idx,
                                active_repair_set_index,
                                repair_set_index,
                                &ripup_ids,
                                victim_first,
                                reverse_victim_order,
                                block_radius_cells,
                                commit_radius_cells,
                                core_radius_cells,
                                prefer_orthogonal_repair,
                                history_weight,
                                max_rounds,
                                max_victims,
                                collect_native_timing,
                            ) {
                                RerouteCurrentNetAfterVictimsOutcome::Routed => {}
                                RerouteCurrentNetAfterVictimsOutcome::Failed => continue,
                            }
                        }

                        if mode.victim_reroute_only_reservation_added {
                            self.obstacle_map
                                .remove_static_keys(&mode.victim_reroute_only_reservation);
                        }

                        if mode.temporary_probe_reservation_added {
                            self.obstacle_map
                                .remove_static_keys(&mode.temporary_probe_reservation);
                        }

                        push_native_repair_trace(
                            &mut batch.repair_trace,
                            "repair_mode_result",
                            Some(mode.route_order),
                            None,
                            job.net_id,
                            Some(round_idx),
                            Some(active_repair_set_index as u64),
                            &probe.candidate_blockers,
                            &ripup_ids,
                            &mode.victim_reroute_ids,
                            Some(victim_first),
                            Some(reverse_victim_order),
                            Some(!mode.mode_failed && mode.repaired_route.is_some()),
                            if mode.mode_failed {
                                Some("mode_failed".to_string())
                            } else if mode.repaired_route.is_none() {
                                Some("no_repaired_route".to_string())
                            } else {
                                None
                            },
                        );

                        if !mode.mode_failed && mode.repaired_route.is_some() {
                            repair.repaired = true;
                            batch.repair_count += 1;
                            break;
                        }
                    }
                    if repair.repaired {
                        break;
                    }
                }
                if repair.repaired {
                    break;
                }
            }

            match self.try_final_repair_fallback(
                &mut batch,
                repair,
                probe,
                job,
                block_radius_cells,
                commit_radius_cells,
                core_radius_cells,
                max_rounds,
                collect_native_timing,
            ) {
                Ok(()) => continue 'route_jobs,
                Err(()) => break 'route_jobs,
            }
        }
        if trace_native_progress {
            if let Some(last_start) = batch.trace_last_route_start {
                eprintln!(
                    "native_route_elapsed previous_index={} elapsed_s={:.6}",
                    native_jobs.len(),
                    last_start.elapsed().as_secs_f64()
                );
            }
        }

        self.build_native_batch_result_dict(py, &native_jobs, &mut batch, collect_native_timing)
    }
}
