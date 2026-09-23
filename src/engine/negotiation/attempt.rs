//! One attempt of the negotiated loop's per-net sequence: the search it
//! runs, the budget and guidance it runs under, and the
//! `native_negotiated_search` trace line it reports.
//!
//! Extracted from `loop_.rs` unchanged (Milestone 4 Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! 2026-09-23): same searches, same order, same trace text.

use std::time::Instant;

use pyo3::prelude::*;
use rustc_hash::FxHashMap;

use crate::engine::*;

/// The clearance radii and diagnostic flags every search of one batch
/// shares -- built once in the loop's setup and passed to every attempt.
pub(crate) struct SearchArgs {
    pub(crate) block_radius_cells: i32,
    pub(crate) commit_radius_cells: Option<i32>,
    pub(crate) core_radius_cells: Option<i32>,
    pub(crate) collect_native_timing: bool,
    pub(crate) trace: bool,
}

/// What one attempt did, as the `native_negotiated_search` trace line
/// reports it.
pub(crate) struct AttemptRecord {
    /// The ladder step this attempt belongs to.
    pub(crate) kind: AttemptKind,
    /// `kind=` in the trace line. Not derived from `kind`: the post-rip-up
    /// search reports which of its three flavours ran (plain, guided or
    /// crossing-free).
    pub(crate) trace_kind: &'static str,
    /// The expansion budget this attempt ran under; `None` is `full`.
    pub(crate) budget: Option<u64>,
    pub(crate) elapsed_s: f64,
    /// Expansions, from the committed route when it routed and from the
    /// router's last search otherwise; `?` when neither is available.
    pub(crate) expanded: String,
    pub(crate) outcome: &'static str,
    /// The failing attempt's error message, printed by the first plain
    /// attempt only (see [`trace_attempt`]).
    pub(crate) error: Option<String>,
}

/// The `native_negotiated_search` line, unchanged since 2026-09-15.
pub(crate) fn trace_attempt(batch_start: Option<Instant>, net_id: u64, record: &AttemptRecord) {
    // `error=` has only ever been printed by a net's first plain attempt;
    // every other attempt's line ends at `budget=`.
    let error_field = match &record.error {
        Some(error) if record.kind == AttemptKind::First => format!(" error={error:?}"),
        _ => String::new(),
    };
    eprintln!(
        "{}native_negotiated_search net={} kind={} elapsed_s={:.3} expanded={} outcome={} budget={}{}",
        trace_t(batch_start),
        net_id,
        record.trace_kind,
        record.elapsed_s,
        record.expanded,
        record.outcome,
        trace_budget_str(record.budget),
        error_field
    );
}

/// One plain search of the per-net sequence, fully described: which step
/// of the ladder it is, the budget it runs under, whether it runs
/// crossing-free, and which partners -- if any -- are priced as guidance
/// for it. The four constructors below are the loop's four plain searches.
pub(crate) struct PlainAttempt<'a> {
    kind: AttemptKind,
    trace_kind: &'static str,
    budget: Option<u64>,
    crossing_free: bool,
    guided_partner_ids: &'a [u64],
}

impl<'a> PlainAttempt<'a> {
    /// A net's first try this round.
    pub(crate) fn first(budget: Option<u64>, crossing_free: bool) -> Self {
        PlainAttempt {
            kind: AttemptKind::First,
            trace_kind: "plain",
            budget,
            crossing_free,
            guided_partner_ids: &[],
        }
    }

    /// The one retry a never-failed net gets before probe and rip-up run
    /// at all (owner decision 2026-09-15 01:30).
    pub(crate) fn first_retry(budget: Option<u64>, crossing_free: bool) -> Self {
        PlainAttempt {
            kind: AttemptKind::FirstRetry,
            trace_kind: "plain_retry",
            budget,
            crossing_free,
            guided_partner_ids: &[],
        }
    }

    /// The probe-guided search: the probe's (widened) legal partners
    /// priced at `NEGOTIATED_PROBE_GUIDANCE_LOSS` for one search. Never
    /// crossing-free -- a crossing-free net has no legal partners to price.
    pub(crate) fn probe_guided(budget: Option<u64>, partner_ids: &'a [u64]) -> Self {
        PlainAttempt {
            kind: AttemptKind::ProbeGuided,
            trace_kind: "probe_guided",
            budget,
            crossing_free: false,
            guided_partner_ids: partner_ids,
        }
    }

    /// The search on the map the rip-up freed: crossing-free, guided (the
    /// mixed case, where the probe also blamed legal partners) or plain,
    /// exactly as its trace kind reports it.
    pub(crate) fn post_ripup(
        budget: Option<u64>,
        crossing_free: bool,
        partner_ids: &'a [u64],
    ) -> Self {
        PlainAttempt {
            kind: AttemptKind::PostRipUp,
            trace_kind: if crossing_free {
                "post_ripup_crossing_free"
            } else if partner_ids.is_empty() {
                "post_ripup_plain"
            } else {
                "post_ripup_guided"
            },
            budget,
            crossing_free,
            guided_partner_ids: partner_ids,
        }
    }
}

impl PyPhotonicRouter {
    /// Runs one plain search under its budget, crossing-free flag and
    /// guidance, restores the router's per-attempt fields and reports the
    /// attempt's trace line.
    pub(crate) fn run_plain_attempt(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        args: &SearchArgs,
        attempt: PlainAttempt<'_>,
    ) -> PlainRouteOutcome {
        let PlainAttempt {
            kind,
            trace_kind,
            budget,
            crossing_free,
            guided_partner_ids,
        } = attempt;
        let net_id = job.net_id;
        let started = Instant::now();
        self.negotiated_search_budget = budget;
        self.negotiated_crossing_free_search = crossing_free;
        let outcome = if guided_partner_ids.is_empty() {
            self.try_plain_normal_route(
                batch,
                job,
                args.block_radius_cells,
                args.commit_radius_cells,
                args.core_radius_cells,
                args.collect_native_timing,
            )
        } else {
            self.with_probe_guided_guidance(net_id, guided_partner_ids, |router| {
                router.try_plain_normal_route(
                    batch,
                    job,
                    args.block_radius_cells,
                    args.commit_radius_cells,
                    args.core_radius_cells,
                    args.collect_native_timing,
                )
            })
        };
        self.negotiated_search_budget = None;
        self.negotiated_crossing_free_search = false;
        if args.trace {
            let (outcome_str, expanded) = match &outcome {
                PlainRouteOutcome::Routed => ("routed", self.committed_expanded_str(batch, net_id)),
                PlainRouteOutcome::NotResolved => {
                    ("failed", self.last_search_expanded_states.to_string())
                }
            };
            trace_attempt(
                self.negotiated_batch_start,
                net_id,
                &AttemptRecord {
                    kind,
                    trace_kind,
                    budget,
                    elapsed_s: started.elapsed().as_secs_f64(),
                    expanded,
                    outcome: outcome_str,
                    error: Some(
                        batch
                            .attempts
                            .last()
                            .and_then(|attempt| attempt.error.as_deref())
                            .unwrap_or("")
                            .to_string(),
                    ),
                },
            );
        }
        outcome
    }

    /// The diagnostic probe (`probe_net_for_repair`): the one attempt that
    /// commits nothing and the one whose trace line carries no budget.
    /// `Err(())` aborts the whole batch, the loop's only early return.
    pub(crate) fn run_probe_attempt(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        order_by_id: &FxHashMap<u64, usize>,
        args: &SearchArgs,
    ) -> Result<ProbeState, ()> {
        let started = Instant::now();
        let probe_result = self.probe_net_for_repair(
            batch,
            job,
            order_by_id,
            args.block_radius_cells,
            args.commit_radius_cells,
            args.collect_native_timing,
            args.trace,
        );
        if args.trace {
            let (outcome_str, expanded_str) = match &probe_result {
                Ok(probe) => (
                    "routed",
                    probe.probe_route.stats.expanded_states.to_string(),
                ),
                Err(()) => ("failed", "?".to_string()),
            };
            eprintln!(
                "{}native_negotiated_search net={} kind=probe elapsed_s={:.3} expanded={} outcome={}",
                trace_t(self.negotiated_batch_start),
                job.net_id,
                started.elapsed().as_secs_f64(),
                expanded_str,
                outcome_str
            );
        }
        probe_result
    }

    /// The direct-crossing search: one legal crossing against at most the
    /// three partners the probe blamed for an illegal one.
    pub(crate) fn run_direct_crossing_attempt(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        order_by_id: &FxHashMap<u64, usize>,
        args: &SearchArgs,
        budget: Option<u64>,
        illegal_partner_ids: &[u64],
    ) -> PyResult<LidarDirectCrossingOutcome> {
        let net_id = job.net_id;
        let started = Instant::now();
        self.negotiated_search_budget = budget;
        let result = self.try_lidar_direct_crossing_subset(
            batch,
            job,
            order_by_id,
            args.block_radius_cells,
            args.commit_radius_cells,
            args.core_radius_cells,
            args.collect_native_timing,
            args.trace,
            Some(illegal_partner_ids),
        );
        self.negotiated_search_budget = None;
        if args.trace {
            let (outcome_str, expanded) = match &result {
                Ok(LidarDirectCrossingOutcome::Routed) => {
                    ("routed", self.committed_expanded_str(batch, net_id))
                }
                Ok(LidarDirectCrossingOutcome::NotResolved) => ("not_resolved", "?".to_string()),
                Err(_) => ("failed", "?".to_string()),
            };
            trace_attempt(
                self.negotiated_batch_start,
                net_id,
                &AttemptRecord {
                    kind: AttemptKind::DirectCrossing,
                    trace_kind: "direct_crossing",
                    budget,
                    elapsed_s: started.elapsed().as_secs_f64(),
                    expanded,
                    outcome: outcome_str,
                    error: None,
                },
            );
        }
        result
    }

    /// Expansions of the route this net just committed, `?` if it is not in
    /// `final_routes` after all.
    fn committed_expanded_str(&self, batch: &RepairBatchState, net_id: u64) -> String {
        batch
            .final_routes
            .get(&net_id)
            .map(|route| route.stats.expanded_states.to_string())
            .unwrap_or_else(|| "?".to_string())
    }
}
