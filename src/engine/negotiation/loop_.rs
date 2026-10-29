//! The negotiated rip-up-and-repair loop (Milestone 5 of
//! `.agent/execplans/2026-08-25-negotiated-repair-engine.md`, rebuilt
//! LiDAR-style in
//! `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`,
//! brought to this shape in phase 3 slice D of the 2026-10 do-over):
//! `run_negotiated_batch` is the rounds, `route_net_this_round` is one net's
//! turn, `global_ripup_round` is LiDAR's `ripupfailedNets` at round end.
//! Each rule the loop applies lives in its own module: the budget ladder in
//! `budget.rs`, the crossing-free rule in `crossing_free.rs`, the local
//! rip-up rule in `ripup.rs`, the queue and its bookkeeping in `queue.rs`,
//! one attempt and its trace line in `attempt.rs`, the probe in `probe.rs`,
//! the braid passes in `braid.rs`, the trace lines in `trace.rs`.

use std::time::Instant;

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::config::NegotiationConfig;

use crate::engine::*;

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

/// The Python job tuples as the loop's own jobs -- the one conversion the
/// `#[pymethods]` wrapper's tuples need before anything native runs.
pub(crate) fn native_jobs_from_py(jobs: Vec<PyRouteJobTuple>) -> Vec<NativeRouteJob> {
    jobs.into_iter()
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
        .collect()
}

/// The batch's jobs plus the three lookup tables the loop's steps read:
/// the caller's net order (`route_rust.py`'s `_topological_net_route_order`),
/// the jobs by id, and the job indices by source column (the source-column
/// widening of the post-rip-up guidance).
struct NetTables {
    jobs: Vec<NativeRouteJob>,
    order_by_id: FxHashMap<u64, usize>,
    job_by_id: FxHashMap<u64, NativeRouteJob>,
    source_layer_indices_by_x: FxHashMap<i32, Vec<usize>>,
}

impl NetTables {
    fn from_jobs(jobs: Vec<NativeRouteJob>) -> Self {
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

/// The counters the `native_negotiated_done` line reports, plus the two
/// the line does not print but [`PyPhotonicRouter::run_negotiated_batch`]
/// hands back with the batch (`rounds` is the `rounds=` field of that same
/// line; `stall_resets` is how often the "two rounds without progress"
/// reset fired). Nothing in the loop reads them; they are written where
/// the loop already did the work they count.
#[derive(Default)]
pub(crate) struct NegotiationCounters {
    pub(crate) global_ripups: u32,
    pub(crate) local_ripups: u32,
    pub(crate) probe_guided: u32,
    pub(crate) crossing_free: u32,
    /// Rounds the loop actually ran.
    pub(crate) rounds: u32,
    /// How often the loop's own epoch boundary
    /// (`RESET_AFTER_ROUNDS_WITHOUT_PROGRESS` rounds in which not a single
    /// net committed) cleared the history map and the queue's failure
    /// counts.
    pub(crate) stall_resets: u32,
}

/// The per-batch arguments of the negotiated loop: the clearance radii
/// every search of the batch shares, the round limit, the history cost the
/// loop installs for the nets that have already failed, and whether the
/// local rip-up rule runs. Exactly the seven arguments
/// `route_many_with_negotiated_repair_and_commit` takes from Python beside
/// the jobs.
pub(crate) struct NegotiationParams {
    pub(crate) block_radius_cells: i32,
    pub(crate) commit_radius_cells: Option<i32>,
    pub(crate) core_radius_cells: Option<i32>,
    pub(crate) max_rounds: u32,
    pub(crate) history_weight: f64,
    pub(crate) history_increment: u32,
    /// `true` switches the local rip-up rule off: nothing is ripped for a
    /// blocked net and it just fails its round. With `max_rounds = 1` this
    /// is the no-repair mode (`RipupRerouteConfig(enabled=False)`), which
    /// reached the deleted separate non-repair batch binding this loop
    /// replaced in Milestone 8 of
    /// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`
    /// (2026-09-24).
    pub(crate) no_ripup: bool,
}

/// What every round of one batch shares, chosen once in the setup and never
/// written by the loop: the arguments, the configuration (a clone, so that
/// `self` stays free for the searches), the two policies, the job tables
/// and the `SearchArgs` every search of the batch starts from.
struct BatchContext<'a> {
    params: &'a NegotiationParams,
    negotiation: &'a NegotiationConfig,
    budgets: LadderBudgets<'a>,
    crossing_free_policy: Box<dyn CrossingFreePolicy>,
    tables: NetTables,
    /// The radii and flags; the two per-net weights are chosen at the top
    /// of each net's turn (`net_search_args`).
    args: SearchArgs,
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
    braid_failed_pairs: FxHashSet<(u64, u64)>,
}

/// How one net's turn ended.
enum NetOutcome {
    /// Committed, braid passes run, whatever they ripped requeued.
    Routed,
    /// Still unrouted, requeued at the back; the round-end global rip-up
    /// gets it.
    Failed,
    /// Its probe found no path at all: the batch ends here with the
    /// `failed_net_id`/`failed_error` the probe set.
    ProbeAborted,
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

impl PyPhotonicRouter {
    /// The `#[pymethods]` wrapper's entry point: the Python job tuples as
    /// the loop's own jobs, the loop, and the Python result dictionary.
    /// Everything between those two conversions is `run_negotiated_batch`,
    /// which needs no `Python<'_>` token at all -- the loop itself is
    /// native, and this is the only place in the batch that is not.
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
        no_ripup: bool,
    ) -> PyResult<PyObject> {
        let jobs = native_jobs_from_py(jobs);
        let params = NegotiationParams {
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
            max_rounds,
            history_weight,
            history_increment,
            no_ripup,
        };
        // Read before the loop runs, as `SearchArgs` has always read it;
        // no step of the loop writes `astar_cfg`.
        let collect_native_timing = self.astar_cfg.collect_detailed_timing;
        let (mut batch, _counters) = self.run_negotiated_batch(jobs.clone(), &params)?;
        self.build_native_batch_result_dict(py, &jobs, &mut batch, collect_native_timing)
    }

    /// The loop every paper row runs: route the nets in the order given;
    /// when one fails, ask the probe who is blocking it, rip up what the
    /// local rip-up rule names and search once more; requeue whoever did
    /// not resolve, rip up at round end what the probes blamed for the nets
    /// still failing, and iterate until every net routes or `max_rounds` is
    /// exhausted. One net's turn is `route_net_this_round`.
    ///
    /// Returns the committed batch and the counters of the rules that ran;
    /// `route_many_with_negotiated_repair_and_commit_impl` above turns the
    /// batch into the Python result dictionary and drops the counters (the
    /// trace line already reports them). No step below touches Python.
    pub(crate) fn run_negotiated_batch(
        &mut self,
        jobs: Vec<NativeRouteJob>,
        params: &NegotiationParams,
    ) -> PyResult<(RepairBatchState, NegotiationCounters)> {
        let args = self.begin_negotiated_batch(params);
        let negotiation = self.router_config.negotiation.clone();
        let ctx = self.batch_context(jobs, params, &negotiation, args);
        let max_rounds = params.max_rounds;
        let mut batch = RepairBatchState::default();
        let mut queue = NetQueue::new(ctx.tables.jobs.iter().map(|job| job.net_id));
        let mut counters = NegotiationCounters::default();
        let mut round = 0u32;
        let mut rounds_without_progress = 0u32;
        const RESET_AFTER_ROUNDS_WITHOUT_PROGRESS: u32 = 2;
        // Set by the one early exit (a probe that found no path at all):
        // that batch keeps the `failed_net_id`/`failed_error` the probe
        // itself set and never takes the soft non-convergence failure.
        let mut probe_aborted = false;

        'rounds: while round < max_rounds && !queue.is_empty() {
            round += 1;
            let this_round = queue.take_round();
            let mut state = RoundState {
                batch: &mut batch,
                queue: &mut queue,
                braid_failed_pairs: FxHashSet::default(),
            };
            let mut made_progress = false;
            // Every net that ends this round still unrouted, whether or
            // not a rip-up was attempted for it.
            let mut round_failed_ids: Vec<u64> = Vec::new();

            for (position, net_id) in this_round.into_iter().enumerate() {
                match self.route_net_this_round(
                    &ctx,
                    &mut state,
                    &mut counters,
                    position,
                    round,
                    net_id,
                ) {
                    NetOutcome::Routed => made_progress = true,
                    NetOutcome::Failed => round_failed_ids.push(net_id),
                    NetOutcome::ProbeAborted => {
                        probe_aborted = true;
                        break 'rounds;
                    }
                }
            }

            // LiDAR's `ripupfailedNets`, with its own once-per-batch epoch
            // reset on the second global rip-up.
            self.global_ripup_round(
                state.batch,
                &round_failed_ids,
                state.queue,
                &mut counters.global_ripups,
                round,
                args.trace,
            );

            // Two rounds without a single committed route: the loop's own
            // epoch boundary, independent of the global rip-up's. Both
            // resets are what the paper's runs had.
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
                    counters.stall_resets += 1;
                }
            }
        }
        counters.rounds = round;

        if !probe_aborted {
            // The soft failure: the batch is returned with whatever routed.
            if let Some(failed_net_id) = queue.front() {
                batch.failed_net_id = Some(failed_net_id);
                batch.failed_error = Some(format!(
                    "Negotiated repair did not converge for net {failed_net_id} within {max_rounds} rounds; {} net(s) still unrouted",
                    queue.len()
                ));
            }
        }
        if args.trace {
            self.trace_negotiated_done(&counters, round, queue.len());
        }
        self.finish_negotiated_batch();
        Ok((batch, counters))
    }

    /// One net's turn in a round: the plain search (and the first-retry
    /// rule), on failure the probe, the local rip-up and the search once
    /// more on the map it freed (guided by the probe's legal partners in
    /// the mixed case), then the requeues. Every search of the turn runs
    /// with the same `SearchArgs`.
    fn route_net_this_round(
        &mut self,
        ctx: &BatchContext,
        state: &mut RoundState,
        counters: &mut NegotiationCounters,
        position: usize,
        round: u32,
        net_id: u64,
    ) -> NetOutcome {
        let max_rounds = ctx.params.max_rounds;
        let job = ctx
            .tables
            .job_by_id
            .get(&net_id)
            .expect("every queued net_id came from job_by_id's own keys");
        let failed_count = state.queue.failed_count(net_id);
        let budget = |kind| ctx.budgets.budget(kind, failed_count, round, max_rounds);
        let crossing_free = ctx
            .crossing_free_policy
            .crossing_free(net_id, round, max_rounds);
        let args = self.net_search_args(ctx, net_id, failed_count);
        if args.trace {
            self.trace_net_start(position, round, net_id, failed_count, crossing_free);
        }

        // The plain search, and -- for a net that has not failed yet,
        // outside the last round -- one retry at the larger first-retry
        // budget before probe and rip-up run at all (owner decision
        // 2026-09-15 01:30).
        let first = PlainAttempt::first(budget(AttemptKind::First), crossing_free);
        let mut outcome = self.run_plain_attempt(state.batch, job, &args, first);
        if matches!(outcome, PlainRouteOutcome::NotResolved)
            && failed_count == 0
            && round != max_rounds
        {
            let retry = PlainAttempt::first_retry(budget(AttemptKind::FirstRetry), crossing_free);
            outcome = self.run_plain_attempt(state.batch, job, &args, retry);
        }
        if matches!(outcome, PlainRouteOutcome::Routed) {
            counters.crossing_free += u32::from(crossing_free);
            self.braid_and_requeue(state, job, &ctx.tables, &args);
            return NetOutcome::Routed;
        }

        // The probe is diagnostic only and is never itself committed here;
        // a probe failure aborts the whole batch, this loop's only early
        // return.
        let order_by_id = &ctx.tables.order_by_id;
        let Ok(probe) = self.run_probe_attempt(state.batch, job, order_by_id, &args) else {
            return NetOutcome::ProbeAborted;
        };
        state
            .queue
            .record_blockers(net_id, probe_blockers(&probe, crossing_free));

        // LiDAR's local rip-up rule (`ripuplocalnets`) -- switched off in
        // the no-repair mode -- then this net's search once more on the map
        // it freed.
        let ripped = if ctx.params.no_ripup {
            Vec::new()
        } else {
            self.ripup_illegal_crossing_partners(
                state.batch,
                &probe,
                state.queue.ripped_once_mut(),
                net_id,
                round,
                args.trace,
                crossing_free,
            )
        };
        if ripped.is_empty() {
            state.queue.requeue_back(net_id);
            return NetOutcome::Failed;
        }
        counters.local_ripups += 1;
        let guided_partner_ids =
            self.post_ripup_guidance(ctx, state.batch, &probe, net_id, crossing_free, args.trace);
        let post = PlainAttempt::post_ripup(
            budget(AttemptKind::PostRipUp),
            crossing_free,
            &guided_partner_ids,
        );
        let outcome = self.run_plain_attempt(state.batch, job, &args, post);
        let result = if matches!(outcome, PlainRouteOutcome::Routed) {
            counters.probe_guided += u32::from(!guided_partner_ids.is_empty());
            counters.crossing_free += u32::from(crossing_free);
            self.braid_and_requeue(state, job, &ctx.tables, &args);
            NetOutcome::Routed
        } else {
            state.queue.requeue_back(net_id);
            NetOutcome::Failed
        };
        state.queue.requeue_front_preserving_order(&ripped);
        result
    }

    /// The batch's setup on `self`: history and congestion cleared, the
    /// per-commit history amount installed, the trace clock started. Returns
    /// the `SearchArgs` every search of the batch starts from.
    fn begin_negotiated_batch(&mut self, params: &NegotiationParams) -> SearchArgs {
        self.obstacle_map.clear_congestion();
        self.obstacle_map.clear_history();
        self.long_straight_congestion_cells.clear();
        self.long_straight_congestion_records.clear();
        self.commit_history_increment = params.history_increment;
        self.commit_history_block_radius_cells = params.block_radius_cells;
        // Timestamps this loop's `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG`
        // trace lines; cleared again by `finish_negotiated_batch`.
        self.negotiated_batch_start = Some(Instant::now());
        SearchArgs {
            radii: SearchRadii::new(
                params.block_radius_cells,
                params.commit_radius_cells,
                params.core_radius_cells,
            ),
            collect_native_timing: self.astar_cfg.collect_detailed_timing,
            trace: self.router_config.diagnostics.native_repair_diag,
            // Chosen per net at the top of its turn (`net_search_args`).
            history_weight: 0.0,
            long_straight_weight_override: None,
        }
    }

    /// Both returns of the loop clear the per-batch field it set.
    fn finish_negotiated_batch(&mut self) {
        self.negotiated_batch_start = None;
    }

    /// The job tables and the two policies, chosen once for the batch.
    fn batch_context<'a>(
        &self,
        jobs: Vec<NativeRouteJob>,
        params: &'a NegotiationParams,
        negotiation: &'a NegotiationConfig,
        args: SearchArgs,
    ) -> BatchContext<'a> {
        BatchContext {
            params,
            negotiation,
            budgets: LadderBudgets(negotiation),
            crossing_free_policy: self.negotiated_crossing_free_policy(negotiation),
            tables: NetTables::from_jobs(jobs),
            args,
        }
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

    /// The `SearchArgs` of one net's turn -- the plain attempts, the probe
    /// and the braid passes all run with them: the batch's radii and flags
    /// plus the two weights chosen here. History cost steers a net that has
    /// already failed, and only such a net: a net's first attempt each
    /// epoch runs without it. The long-straight penalty is 0 for the exempt
    /// dense fan-out nets, the configured weight for every other net.
    fn net_search_args(&self, ctx: &BatchContext, net_id: u64, failed_count: u32) -> SearchArgs {
        let failed_before = failed_count != 0 || ctx.negotiation.history_on_first_attempt;
        let exempt = self.long_straight_exempt_net_ids.contains(&net_id);
        SearchArgs {
            history_weight: if failed_before {
                ctx.params.history_weight
            } else {
                0.0
            },
            long_straight_weight_override: if exempt { Some(0.0) } else { None },
            ..ctx.args
        }
    }

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
            args,
            &mut state.braid_failed_pairs,
            &mut braid_ripped_victims,
        );
        state.queue.requeue_braid_victims(&braid_ripped_victims);
    }

    /// Mixed-case guidance for the post-rip-up search (2026-09-15 16:25
    /// Decision Log entry, follow-up (2)): when the probe also reported
    /// legal crossing partners, the search prices them at
    /// `NEGOTIATED_PROBE_GUIDANCE_LOSS`, widened by source column (2026-09-15
    /// 15:00, "Slice result 15:40": the probe's free path threads between
    /// some lanes of a fan-out bundle rather than crossing all of them, so
    /// the partners it names alone underprice the search) unless
    /// `disable_source_column_widening`. A crossing-free net gets no
    /// guidance: it may cross nobody. Not covered by the 2026-10 ablation;
    /// `disable_post_ripup_guidance` is the switch for the next one.
    fn post_ripup_guidance(
        &self,
        ctx: &BatchContext,
        batch: &RepairBatchState,
        probe: &ProbeState,
        net_id: u64,
        crossing_free: bool,
        trace: bool,
    ) -> Vec<u64> {
        if crossing_free || ctx.negotiation.disable_post_ripup_guidance {
            return Vec::new();
        }
        let (legal_partner_ids, _illegal_partner_ids) = split_probe_partners(probe);
        if legal_partner_ids.is_empty() {
            return Vec::new();
        }
        if ctx.negotiation.disable_source_column_widening {
            return legal_partner_ids;
        }
        let tables = &ctx.tables;
        let widened = widen_probe_partners_by_source_column(
            &legal_partner_ids,
            &tables.job_by_id,
            &tables.source_layer_indices_by_x,
            &tables.jobs,
            &batch.final_routes,
            net_id,
        );
        if trace {
            self.trace_probe_widening(net_id, &legal_partner_ids, &widened);
        }
        widened
    }

    /// LiDAR's `ripupfailedNets` (Milestone 3 of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`):
    /// called once at the end of every round. Every net that ended the
    /// round still unrouted (`round_failed`) is ripped up together with the
    /// blockers its most recent probe reported (`queue.last_blockers`) that
    /// are still committed; history cost is bumped on a ripped blocker's
    /// cells before it is ripped, via the same per-commit chokepoint
    /// (`add_repair_history_for_route`) every successful commit uses. The
    /// failed nets and their ripped blockers are then requeued at the front
    /// of `queue`, failed nets first in `round_failed` order and the ripped
    /// blockers after, so both are tried again first thing next round. A
    /// no-op when `round_failed` is empty (nothing failed this round).
    /// Exactly once per batch -- on the call where the global rip-up
    /// counter reaches 2, and never again, however many further global
    /// rip-ups follow -- the history map and the local rip-up rule's
    /// once-per-epoch set are cleared: this is this milestone's own epoch
    /// boundary, independent of (and able to coincide with) the loop's
    /// "two rounds without progress" reset.
    ///
    /// This once-per-batch reset is the behaviour the paper's runs used
    /// (D7 of `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
    /// decided 2026-09-24: the code stands, the comment is corrected).
    pub(crate) fn global_ripup_round(
        &mut self,
        batch: &mut RepairBatchState,
        round_failed: &[u64],
        queue: &mut NetQueue,
        global_ripups: &mut u32,
        round: u32,
        trace: bool,
    ) {
        if round_failed.is_empty() {
            return;
        }
        *global_ripups += 1;

        let mut ripped_blocker_ids: Vec<u64> = Vec::new();
        let mut ripped_seen: FxHashSet<u64> = FxHashSet::default();
        for &failed_net_id in round_failed {
            for blocker_id in queue.last_blockers(failed_net_id).to_vec() {
                let Some(route) = batch.final_routes.get(&blocker_id) else {
                    // Not currently committed (already ripped by this or an
                    // earlier step this round) -- nothing to do for it.
                    continue;
                };
                let route = route.clone();
                self.add_repair_history_for_route(&route);
                self.ripup_route(blocker_id);
                batch.final_routes.remove(&blocker_id);
                queue.bump_failed_count(blocker_id);
                if ripped_seen.insert(blocker_id) {
                    ripped_blocker_ids.push(blocker_id);
                }
            }
        }

        // The failed nets first (in `round_failed` order), the ripped
        // blockers after them (in rip order), deduplicated, to the front of
        // the queue -- LiDAR's "ripped nets go back to the queue". Nets
        // already routed stay out of the queue entirely (they were never
        // added to `round_failed`).
        let mut requeue_ids: Vec<u64> = Vec::new();
        let mut requeue_seen: FxHashSet<u64> = FxHashSet::default();
        for &id in round_failed.iter().chain(&ripped_blocker_ids) {
            if requeue_seen.insert(id) {
                requeue_ids.push(id);
            }
        }
        queue.move_to_front(&requeue_ids);

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
        if trace {
            self.trace_global_ripup(round, round_failed, &ripped_blocker_ids);
        }

        // This milestone's own epoch boundary (LiDAR clears the history
        // map after its second global rip-up so the cost landscape does
        // not calcify): fires exactly once, on the second global rip-up,
        // as in LiDAR (`ripup_times == 2`); clearing at every second call
        // (the first version, 2026-09-14) never let pressure build on a
        // shared corridor and two nets displaced each other for 10 rounds.
        if *global_ripups == 2 {
            self.obstacle_map.clear_history();
            queue.epoch_reset(false);
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
            if trace {
                self.trace_history_clear(round, *global_ripups);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;

    /// A queue with nothing left to route this round, as `global_ripup_round`
    /// sees it after `take_round`.
    fn drained_queue() -> NetQueue {
        NetQueue::new(std::iter::empty())
    }

    #[test]
    fn global_ripup_round_rips_blockers_and_queues_failed_nets_first() {
        let (mut router, _jobs) = crossing_conflict_fixture();
        let mut batch = fresh_repair_batch_state();
        for net_id in [1u64, 2, 3] {
            let route = route_with_real_cells_for_net(&router, net_id);
            batch.final_routes.insert(net_id, route);
        }

        let round_failed: Vec<u64> = vec![4];
        let mut queue = drained_queue();
        queue.record_blockers(4, vec![3]);
        let mut global_ripups: u32 = 0;

        router.global_ripup_round(
            &mut batch,
            &round_failed,
            &mut queue,
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
            queue.failed_counts().get(&3).copied(),
            Some(1),
            "the ripped blocker's failed_counts must be bumped exactly once"
        );
        assert_eq!(
            queue.order(),
            vec![4u64, 3u64],
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
        let mut queue = drained_queue();
        queue.record_blockers(4, vec![3]);
        let mut global_ripups: u32 = 0;

        router.global_ripup_round(
            &mut batch,
            &round_failed,
            &mut queue,
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

        let mut queue = drained_queue();
        queue.ripped_once_mut().insert(99); // a dummy epoch marker, to prove it gets cleared
        let mut global_ripups: u32 = 0;

        // First call: net 4 failed, blamed net 3 -- ripped, history bumped,
        // no epoch clear yet (this is the first global rip-up).
        queue.record_blockers(4, vec![3]);
        router.global_ripup_round(&mut batch, &[4], &mut queue, &mut global_ripups, 1, false);
        assert_eq!(global_ripups, 1);
        assert!(net3_cells
            .iter()
            .any(|&(x, y)| router.obstacle_map.get_history_cost(x, y) > 0));
        assert!(
            queue.ripped_once().contains(&99),
            "the epoch marker must survive the first global rip-up"
        );

        // Second call: a different failed net (6) blames net 2 (still
        // committed, untouched by the first call) -- this is the second
        // global rip-up, so the epoch boundary fires: history clears and
        // `ripped_once` resets.
        queue.record_blockers(6, vec![2]);
        router.global_ripup_round(&mut batch, &[6], &mut queue, &mut global_ripups, 2, false);

        assert_eq!(global_ripups, 2);
        assert!(
            net3_cells
                .iter()
                .all(|&(x, y)| router.obstacle_map.get_history_cost(x, y) == 0),
            "history on net 3's former cells must be cleared again after the second \
             global rip-up"
        );
        assert!(
            queue.ripped_once().is_empty(),
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

    /// D7 (decided 2026-09-24): the epoch reset is once per batch, not
    /// every second global rip-up. The second call clears; a third call
    /// must leave the history it just bumped alone, and must leave
    /// `ripped_once` alone, so pressure keeps building after the single
    /// epoch boundary the paper's runs had.
    #[test]
    fn third_global_ripup_does_not_clear_history_again() {
        let (mut router, _jobs) = crossing_conflict_fixture();
        router.commit_history_increment = 1;

        let mut batch = fresh_repair_batch_state();
        for net_id in [1u64, 2, 3] {
            let route = route_with_real_cells_for_net(&router, net_id);
            batch.final_routes.insert(net_id, route);
        }

        let mut queue = drained_queue();
        let mut global_ripups: u32 = 0;

        // Calls one and two: rip nets 1 and 2, so the epoch boundary has
        // already fired (`global_ripups == 2`) before the third call.
        queue.record_blockers(4, vec![1]);
        router.global_ripup_round(&mut batch, &[4], &mut queue, &mut global_ripups, 1, false);
        queue.record_blockers(5, vec![2]);
        router.global_ripup_round(&mut batch, &[5], &mut queue, &mut global_ripups, 2, false);
        assert_eq!(global_ripups, 2);
        let clears_after_two = batch
            .repair_trace
            .iter()
            .filter(|event| event.event_name == "history_clear")
            .count();
        assert_eq!(
            clears_after_two, 1,
            "the epoch boundary must fire exactly once, on the second global rip-up"
        );

        // Third call: net 6 blames net 3, which is still committed. Its
        // history is bumped before it is ripped, and nothing may clear it.
        let net3_cells = router.get_net_cells(3);
        queue.ripped_once_mut().insert(99); // an epoch marker that must now survive
        queue.record_blockers(6, vec![3]);
        router.global_ripup_round(&mut batch, &[6], &mut queue, &mut global_ripups, 3, false);

        assert_eq!(global_ripups, 3);
        assert!(
            net3_cells
                .iter()
                .any(|&(x, y)| router.obstacle_map.get_history_cost(x, y) > 0),
            "a third global rip-up must not clear the history it just bumped on \
             net 3's former cells: {:?}",
            net3_cells
        );
        assert!(
            queue.ripped_once().contains(&99),
            "the once-per-epoch set must survive a third global rip-up"
        );
        assert_eq!(
            batch
                .repair_trace
                .iter()
                .filter(|event| event.event_name == "history_clear")
                .count(),
            1,
            "no second history_clear event on the third global rip-up"
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
        let mut queue = NetQueue::new([7u64, 8u64]);
        let mut global_ripups: u32 = 0;

        router.global_ripup_round(
            &mut batch,
            &round_failed,
            &mut queue,
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
            queue.order(),
            vec![7u64, 8u64],
            "the queue must be untouched"
        );
        assert!(queue.failed_counts().is_empty());
        assert!(batch.repair_trace.is_empty());
    }

    /// Every test below drives `run_negotiated_batch` -- the whole loop,
    /// from the clean map to the committed batch -- on a `test_support`
    /// fixture, at the radii the fixtures are built for: no dynamic
    /// expansion for the search and no extra commit or core clearance, so
    /// a committed net blocks exactly its own cells.
    fn plain_params(max_rounds: u32) -> NegotiationParams {
        NegotiationParams {
            block_radius_cells: 0,
            commit_radius_cells: None,
            core_radius_cells: None,
            max_rounds,
            history_weight: 2.0,
            history_increment: 1,
            no_ripup: false,
        }
    }

    fn local_ripup_events(batch: &RepairBatchState) -> Vec<&NativeRepairTraceEvent> {
        batch
            .repair_trace
            .iter()
            .filter(|event| event.event_name == "local_ripup")
            .collect()
    }

    /// (1) The loop's ordinary case: two nets that do not contend route on
    /// their first try, in one round, and nothing negotiates.
    #[test]
    fn every_net_routes_in_the_first_round_when_nothing_contends() {
        let mut router = small_test_router();
        let jobs = vec![
            NativeRouteJob::new(
                1,
                PyState::new(1, 2, 0),
                PyState::new(17, 2, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
            NativeRouteJob::new(
                2,
                PyState::new(1, 12, 0),
                PyState::new(17, 12, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
        ];

        let (batch, counters) = router
            .run_negotiated_batch(jobs, &plain_params(4))
            .expect("the negotiated loop must not raise");

        assert!(
            batch.final_routes.contains_key(&1) && batch.final_routes.contains_key(&2),
            "both nets must end committed: {:?}",
            batch.final_routes.keys().collect::<Vec<_>>()
        );
        assert_eq!(batch.repair_count, 0, "nothing was repaired");
        assert_eq!(counters.rounds, 1, "the queue emptied in the first round");
        assert_eq!(counters.local_ripups, 0);
        assert_eq!(counters.global_ripups, 0);
        assert_eq!(counters.stall_resets, 0);
        assert_eq!(batch.failed_net_id, None);
        assert!(
            batch.repair_trace.is_empty(),
            "no repair rule ran: {:?}",
            batch.repair_trace
        );
    }

    /// (2) The negotiation itself, end to end: the horizontal net 1
    /// commits first and closes the corridor, the four-cell 45-degree net
    /// 2 cannot cross it legally at any angle and has no room to swing
    /// into one, so the loop probes, the direct-crossing repair comes
    /// back not resolved, the local rip-up rule rips net 1, net 2 routes
    /// on the map that frees up, and net 1 -- requeued as the rip-up's
    /// victim -- reroutes around net 2 in the next round. Both end
    /// committed.
    ///
    /// Fixture: `crossing_ripup_job_fixture`, not
    /// `crossing_conflict_probe_fixture`. The NOTE in
    /// `negotiated_local_ripup_rips_the_illegal_partner_and_routes_the_net`
    /// (ripup.rs) records that the original fixture's vertical job cannot
    /// complete its post-rip-up reroute; measured while writing this
    /// test, that fixture's diagonal also cannot be rerouted at all once
    /// any other net holds the corridor, so it can never show the
    /// victim's side of a rip-up. See the fixture's own doc comment.
    #[test]
    fn a_blocked_net_rips_its_illegal_partner_and_both_end_routed() {
        let (mut router, jobs) = crossing_ripup_job_fixture();

        let (batch, counters) = router
            .run_negotiated_batch(jobs, &plain_params(4))
            .expect("the negotiated loop must not raise");

        let ripups = local_ripup_events(&batch);
        assert_eq!(
            ripups.len(),
            1,
            "exactly one local rip-up: {:?}",
            batch.repair_trace
        );
        assert_eq!(ripups[0].net_id, 2, "net 2 is the net that was blocked");
        assert_eq!(
            ripups[0].candidate_blockers,
            vec![1],
            "net 1 is the illegal crossing partner that was ripped"
        );
        assert_eq!(counters.local_ripups, 1);
        assert!(
            batch.final_routes.contains_key(&2),
            "the blocked net must route on the map the rip-up freed"
        );
        assert!(
            batch.final_routes.contains_key(&1),
            "the ripped victim must be rerouted and recommitted: {:?}",
            batch.failed_error
        );
        assert_eq!(batch.failed_net_id, None, "{:?}", batch.failed_error);
        assert_eq!(
            counters.rounds, 2,
            "round 1 negotiates, round 2 reroutes the victim"
        );
    }

    /// (3) The stall: a net whose only path is the far detour cannot be
    /// found within the (here: one-expansion) search budget, and there is
    /// no committed net anywhere to blame, so the rip-up rule rips
    /// nothing and no route is committed at all. After
    /// `RESET_AFTER_ROUNDS_WITHOUT_PROGRESS` such rounds the loop's own
    /// epoch boundary fires once -- history map cleared, every failure
    /// count cleared -- and the last round, which is never budgeted, then
    /// routes the net.
    #[test]
    fn two_rounds_without_progress_fire_the_loops_epoch_reset() {
        let (mut router, jobs) = walled_detour_job_fixture();
        router.router_config.negotiation.budget_first = 1;
        router.router_config.negotiation.budget_first_retry = 1;
        router.router_config.negotiation.budget_retry = 1;

        let (batch, counters) = router
            .run_negotiated_batch(jobs, &plain_params(3))
            .expect("the negotiated loop must not raise");

        assert_eq!(counters.rounds, 3);
        assert_eq!(
            counters.stall_resets, 1,
            "rounds 1 and 2 commit nothing, so the reset fires once, at the \
             end of round 2: {:?}",
            batch.repair_trace
        );
        assert_eq!(
            counters.local_ripups, 0,
            "there is no committed net to rip: {:?}",
            batch.repair_trace
        );
        assert!(
            batch.final_routes.contains_key(&1),
            "the last round is unbudgeted and routes the net: {:?}",
            batch.failed_error
        );
    }

    /// (4) The loop's one hard abort: a net whose probe finds no path at
    /// all (its target is walled in by static cells, which the probe --
    /// blind to dynamic obstacles only -- cannot route through either)
    /// ends the batch on the spot, carrying that net as `failed_net_id`.
    /// The nets before it keep their committed routes; the nets after it
    /// are never attempted.
    #[test]
    fn a_probe_that_finds_no_path_aborts_the_batch() {
        let (mut router, jobs) = enclosed_target_job_fixture();

        let (batch, counters) = router
            .run_negotiated_batch(jobs, &plain_params(4))
            .expect("the negotiated loop must not raise");

        assert_eq!(
            batch.failed_net_id,
            Some(2),
            "the net whose probe failed ends the batch: {:?}",
            batch.failed_error
        );
        assert!(
            batch
                .failed_error
                .as_deref()
                .is_some_and(|error| !error.contains("did not converge")),
            "the probe's own error is kept, not the soft non-convergence one: {:?}",
            batch.failed_error
        );
        assert!(
            batch.final_routes.contains_key(&1),
            "the net routed before the abort keeps its route"
        );
        assert!(
            !batch.final_routes.contains_key(&2) && !batch.final_routes.contains_key(&3),
            "the failed net and the net behind it are unrouted: {:?}",
            batch.final_routes.keys().collect::<Vec<_>>()
        );
        assert_eq!(counters.rounds, 1, "the abort ends the first round");
    }

    /// (5) Non-convergence: the same negotiation as (2), but with a
    /// single round, so the victim the rip-up displaced never gets its
    /// reroute. The batch is returned with whatever routed and the soft
    /// "did not converge" error, naming the net at the front of the queue
    /// and how many nets are still unrouted.
    #[test]
    fn a_batch_that_runs_out_of_rounds_reports_the_soft_failure() {
        let (mut router, jobs) = crossing_ripup_job_fixture();

        let (batch, counters) = router
            .run_negotiated_batch(jobs, &plain_params(1))
            .expect("the negotiated loop must not raise");

        assert_eq!(counters.rounds, 1);
        assert_eq!(counters.local_ripups, 1);
        assert!(
            batch.final_routes.contains_key(&2),
            "the blocked net still routes in the round it negotiates"
        );
        assert!(
            !batch.final_routes.contains_key(&1),
            "the ripped victim never got its reroute"
        );
        assert_eq!(
            batch.failed_net_id,
            Some(1),
            "the ripped victim is the net left at the front of the queue"
        );
        assert_eq!(
            batch.failed_error.as_deref(),
            Some(
                "Negotiated repair did not converge for net 1 within 1 rounds; \
                 1 net(s) still unrouted"
            ),
            "{:?}",
            batch.failed_error
        );
    }

    /// (6) The no-rip-up mode (`no_ripup = true` with `max_rounds = 1`, the
    /// test-only no-repair baseline): the same contention as (2), but the
    /// local rip-up rule is switched off, so the blocked net fails its round
    /// without displacing anybody -- no `local_ripup` event, no local
    /// rip-up counted. The round-end global rip-up is not part of the switch
    /// and runs as it always has.
    #[test]
    fn no_ripup_mode_rips_nothing_locally() {
        let (mut router, jobs) = crossing_ripup_job_fixture();
        let params = NegotiationParams {
            no_ripup: true,
            ..plain_params(1)
        };

        let (batch, counters) = router
            .run_negotiated_batch(jobs, &params)
            .expect("the negotiated loop must not raise");

        assert!(
            local_ripup_events(&batch).is_empty(),
            "no local rip-up may run: {:?}",
            batch.repair_trace
        );
        assert_eq!(counters.local_ripups, 0);
        assert!(
            !batch.final_routes.contains_key(&2),
            "the blocked net stays unrouted"
        );
        assert_eq!(batch.failed_net_id, Some(2), "{:?}", batch.failed_error);
        assert_eq!(counters.global_ripups, 1, "the round-end rule still ran");
    }
}
