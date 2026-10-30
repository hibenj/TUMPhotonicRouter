//! One attempt of the negotiated loop's per-net sequence: the search it
//! runs, the budget and guidance it runs under, and the
//! `native_negotiated_search` trace line it reports.
//!
//! Extracted from `loop_.rs` unchanged (Milestone 4 Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! 2026-09-23): same searches, same order, same trace text.

use std::io::Write;
use std::time::Instant;

use rustc_hash::FxHashMap;

use crate::engine::*;

/// What every search of one net's turn shares: the clearance radii and
/// diagnostic flags of the batch (built once in the loop's setup) and the
/// two weights the loop chooses per net (copied per turn, see
/// `run_negotiated_batch`). Each attempt builds its `SearchCall` from these.
#[derive(Clone, Copy)]
pub(crate) struct SearchArgs {
    pub(crate) radii: SearchRadii,
    pub(crate) collect_native_timing: bool,
    pub(crate) trace: bool,
    /// `SearchCall::history_weight` of this net's plain searches: the
    /// batch's `history_weight` once the net has failed (or always, with
    /// `history_on_first_attempt`), 0.0 before.
    pub(crate) history_weight: f64,
    /// `Some(0.0)` for the long-straight-exempt dense fan-out nets, `None`
    /// for every other net.
    pub(crate) long_straight_weight_override: Option<f64>,
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

/// The `native_negotiated_search` line, unchanged since 2026-09-15. `out` is
/// `std::io::stderr()` at both production call sites; the tests pass a
/// `Vec<u8>` to read the line back.
pub(crate) fn trace_attempt(
    out: &mut dyn std::io::Write,
    batch_start: Option<Instant>,
    net_id: u64,
    record: &AttemptRecord,
) {
    // `error=` has only ever been printed by a net's first plain attempt;
    // every other attempt's line ends at `budget=`.
    let error_field = match &record.error {
        Some(error) if record.kind == AttemptKind::First => format!(" error={error:?}"),
        _ => String::new(),
    };
    let _ = writeln!(
        out,
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
/// for it. The three constructors below are the loop's three plain
/// searches (the legal-only probe-guided search was removed in phase 3
/// slice A of the 2026-10 do-over after the ablation found it inert).
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

impl Router {
    /// Runs one plain search under its budget, crossing-free flag and
    /// guidance and reports the attempt's trace line.
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
        let call = SearchCall::plain(job, args.radii)
            .budget(budget)
            .history_weight(args.history_weight)
            .long_straight_weight_override(args.long_straight_weight_override)
            .crossing_free(crossing_free)
            .guided(guided_partner_ids);
        let outcome = self.try_plain_normal_route(batch, &call, args.collect_native_timing);
        if args.trace {
            let (outcome_str, expanded) = match &outcome {
                PlainRouteOutcome::Routed => ("routed", self.committed_expanded_str(batch, net_id)),
                PlainRouteOutcome::NotResolved => {
                    ("failed", self.last_search_expanded_states.to_string())
                }
            };
            trace_attempt(
                &mut std::io::stderr(),
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
        let probe_result = self.probe_net_for_repair(batch, job, order_by_id, args);
        if args.trace {
            let (outcome_str, expanded_str) = match &probe_result {
                Ok(probe) => (
                    "routed",
                    probe.probe_route.stats.expanded_states.to_string(),
                ),
                Err(()) => ("failed", "?".to_string()),
            };
            let _ = writeln!(
                std::io::stderr(),
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

/// Unit tests for the `native_negotiated_search` trace line, Milestone 6
/// Slice 3 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
/// [`trace_attempt`] gained a `&mut dyn Write` first parameter so the line
/// can be read back here; production passes `std::io::stderr()`, so the
/// text is unchanged.
#[cfg(test)]
mod tests {
    use super::*;

    /// One trace line for `net=42` with no batch start (so no `t=` prefix).
    fn trace_line(record: &AttemptRecord) -> String {
        let mut out: Vec<u8> = Vec::new();
        trace_attempt(&mut out, None, 42, record);
        String::from_utf8(out).expect("the trace line is UTF-8")
    }

    fn record(kind: AttemptKind, trace_kind: &'static str, budget: Option<u64>) -> AttemptRecord {
        AttemptRecord {
            kind,
            trace_kind,
            budget,
            elapsed_s: 0.5,
            expanded: "123".to_string(),
            outcome: "routed",
            error: None,
        }
    }

    #[test]
    fn the_first_plain_attempt_prints_its_error_field_last() {
        let mut first = record(AttemptKind::First, "plain", Some(2_000_000));
        first.error = Some("No legal route".to_string());
        assert_eq!(
            trace_line(&first),
            "native_negotiated_search net=42 kind=plain elapsed_s=0.500 expanded=123 \
             outcome=routed budget=2000000 error=\"No legal route\"\n"
        );
    }

    #[test]
    fn a_first_attempt_without_an_error_ends_at_the_budget_field() {
        assert_eq!(
            trace_line(&record(AttemptKind::First, "plain", Some(2_000_000))),
            "native_negotiated_search net=42 kind=plain elapsed_s=0.500 expanded=123 \
             outcome=routed budget=2000000\n"
        );
    }

    #[test]
    fn a_none_budget_prints_as_full() {
        assert_eq!(
            trace_line(&record(AttemptKind::PostRipUp, "post_ripup_plain", None)),
            "native_negotiated_search net=42 kind=post_ripup_plain elapsed_s=0.500 expanded=123 \
             outcome=routed budget=full\n"
        );
    }

    #[test]
    fn no_attempt_kind_but_the_first_prints_an_error_field() {
        for kind in [AttemptKind::FirstRetry, AttemptKind::PostRipUp] {
            let mut attempt = record(kind, "plain_retry", Some(10_000_000));
            attempt.error = Some("suppressed".to_string());
            let line = trace_line(&attempt);
            assert!(
                !line.contains("error="),
                "{kind:?} must not print an error field, got {line:?}"
            );
            assert!(
                line.ends_with("budget=10000000\n"),
                "{kind:?}'s line must end at budget=, got {line:?}"
            );
        }
    }

    #[test]
    fn a_batch_start_prefixes_the_line_with_the_elapsed_batch_time() {
        let mut out: Vec<u8> = Vec::new();
        let start = Instant::now() - std::time::Duration::from_millis(12_300);
        trace_attempt(
            &mut out,
            Some(start),
            42,
            &record(AttemptKind::First, "plain", None),
        );
        let line = String::from_utf8(out).expect("the trace line is UTF-8");
        assert!(
            line.starts_with("t=12.") && line.contains(" native_negotiated_search net=42 "),
            "expected a leading batch-time field, got {line:?}"
        );
    }

    #[test]
    fn each_plain_attempt_constructor_carries_its_own_trace_kind() {
        assert_eq!(PlainAttempt::first(None, false).trace_kind, "plain");
        assert_eq!(PlainAttempt::first(None, false).kind, AttemptKind::First);
        assert_eq!(
            PlainAttempt::first_retry(None, false).trace_kind,
            "plain_retry"
        );
        assert_eq!(
            PlainAttempt::first_retry(None, false).kind,
            AttemptKind::FirstRetry
        );
        // The post-rip-up search reports which of its three flavours ran.
        assert_eq!(
            PlainAttempt::post_ripup(None, true, &[]).trace_kind,
            "post_ripup_crossing_free"
        );
        assert_eq!(
            PlainAttempt::post_ripup(None, false, &[]).trace_kind,
            "post_ripup_plain"
        );
        assert_eq!(
            PlainAttempt::post_ripup(None, false, &[7]).trace_kind,
            "post_ripup_guided"
        );
        // Crossing-free wins over guidance in the flavour name.
        assert_eq!(
            PlainAttempt::post_ripup(None, true, &[7]).trace_kind,
            "post_ripup_crossing_free"
        );
    }
}
