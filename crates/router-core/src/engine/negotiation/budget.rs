//! The negotiated loop's search-budget ladder: which expansion budget each
//! attempt of a net's per-round sequence runs under.
//!
//! Extracted from `loop_.rs` unchanged (Milestone 4 Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! 2026-09-23); the numbers are exactly the ones the loop used at every
//! call site before the extraction.

use crate::config::NegotiationConfig;

/// One step of a net's per-round attempt sequence, in the order the loop
/// runs them: the plain search, its fresh-net retry and the post-rip-up
/// search. The braid passes that follow every committed route set
/// `NEGOTIATED_BUDGET_BRAID` at their own call sites (`braid.rs`,
/// `search_calls.rs`) and are not on this ladder. The probe-guided and
/// direct-crossing steps were removed in phase 3 slice A of the 2026-10
/// do-over after the 27-cell ablation found them inert.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AttemptKind {
    First,
    FirstRetry,
    PostRipUp,
}

/// The rule that gives one attempt its expansion budget. `None` means
/// unbounded (the benchmark's full behaviour).
pub(crate) trait BudgetSchedule {
    fn budget(
        &self,
        kind: AttemptKind,
        failed_count: u32,
        round: u32,
        max_rounds: u32,
    ) -> Option<u64>;
}

/// Picks the expansion budget for one plain-search attempt of
/// `route_many_with_negotiated_repair_and_commit`'s per-net loop.
/// `attempt_index` is 0 for a net's first try this round, 1 for the retry
/// that follows a failed first try (only taken when `failed_count == 0`);
/// a net with `failed_count >= 1` gets a single attempt at the larger retry
/// budget regardless of `attempt_index`. The last round (`round ==
/// max_rounds`) is always unbounded.
pub(crate) fn negotiated_search_budget(
    failed_count: u32,
    round: u32,
    max_rounds: u32,
    attempt_index: u32,
    negotiation: &NegotiationConfig,
) -> Option<u64> {
    if round == max_rounds {
        return None;
    }
    if failed_count == 0 {
        if attempt_index == 0 {
            Some(negotiation.budget_first)
        } else {
            Some(negotiation.budget_first_retry)
        }
    } else {
        Some(negotiation.budget_retry)
    }
}

/// The ladder the negotiated loop has run since 2026-09-15 (owner decision
/// 01:30, `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
/// Milestone 5): the plain searches -- first try, fresh-net retry and the
/// post-rip-up try -- follow `negotiated_search_budget` and are unbounded
/// in the last round. The braid budget is a constant set at the braid call
/// sites themselves (`braid.rs`, `search_calls.rs`), not from here.
pub(crate) struct LadderBudgets<'a>(pub &'a NegotiationConfig);

impl BudgetSchedule for LadderBudgets<'_> {
    fn budget(
        &self,
        kind: AttemptKind,
        failed_count: u32,
        round: u32,
        max_rounds: u32,
    ) -> Option<u64> {
        match kind {
            // The net's plain searches: the small first-attempt budget for a
            // fresh net, the larger retry budget once it has failed once.
            AttemptKind::First | AttemptKind::PostRipUp => {
                negotiated_search_budget(failed_count, round, max_rounds, 0, self.0)
            }
            AttemptKind::FirstRetry => {
                negotiated_search_budget(failed_count, round, max_rounds, 1, self.0)
            }
        }
    }
}

/// Whether a retry-eligible net's first attempt runs at the retry budget
/// straight away (2026-11-09). The retry is the same search call with a
/// larger iteration cap, so running the first attempt at the larger cap
/// changes no route: a search that succeeds within the small cap succeeds
/// identically under the large one, and one that would have exhausted the
/// small cap is exactly what the retry recomputed. Only when the retry
/// budget is not larger than the first budget is there nothing to merge,
/// and the two-step sequence runs as configured.
pub(crate) fn first_attempt_runs_at_retry_budget(
    small_budget: Option<u64>,
    retry_budget: Option<u64>,
) -> bool {
    match (small_budget, retry_budget) {
        (Some(small), Some(retry)) => retry > small,
        (Some(_), None) => true,
        (None, _) => false,
    }
}

/// After a merged first attempt failed: does the retry still run? Yes when
/// every kernel search of the attempt stayed within the small cap
/// (`max_heap_pops <= budget_first`): the two-step sequence would have
/// failed its first attempt the same way (a route found and rejected at
/// commit, an open set exhausted) and then run the retry, which may see
/// state the failure changed. No when a search went beyond the small cap:
/// the two-step sequence's first attempt would have been cut off there and
/// its retry would have been this very search, which has just failed.
pub(crate) fn retry_after_failed_first(small_budget: Option<u64>, max_heap_pops: u64) -> bool {
    match small_budget {
        Some(small) => max_heap_pops <= small,
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The merged first attempt runs at the retry budget whenever that is
    /// the larger cap; equal or smaller retry budgets keep the two steps.
    #[test]
    fn first_attempt_merges_into_the_retry_budget_only_when_it_is_larger() {
        assert!(first_attempt_runs_at_retry_budget(
            Some(2_000_000),
            Some(10_000_000)
        ));
        assert!(first_attempt_runs_at_retry_budget(Some(2_000_000), None));
        assert!(!first_attempt_runs_at_retry_budget(
            Some(2_000_000),
            Some(2_000_000)
        ));
        assert!(!first_attempt_runs_at_retry_budget(
            Some(10_000_000),
            Some(2_000_000)
        ));
        assert!(!first_attempt_runs_at_retry_budget(None, Some(10_000_000)));
        assert!(!first_attempt_runs_at_retry_budget(None, None));
    }

    /// A failed merged attempt earns the retry exactly when it stayed within
    /// the small cap (the kernel fails at `iterations > cap`, so a goal pop
    /// at the cap itself still succeeds there).
    #[test]
    fn retry_follows_a_failed_first_attempt_only_within_the_small_cap() {
        assert!(retry_after_failed_first(Some(2_000_000), 0));
        assert!(retry_after_failed_first(Some(2_000_000), 2_000_000));
        assert!(!retry_after_failed_first(Some(2_000_000), 2_000_001));
        assert!(!retry_after_failed_first(None, 0));
    }

    /// The defaults of the three configured budgets are the numbers the
    /// paper ran (2 M / 10 M / 30 M); every other test here reads the
    /// configuration, so this is the one place the numbers are pinned.
    #[test]
    fn negotiation_config_budget_defaults_are_the_paper_numbers() {
        let negotiation = NegotiationConfig::default();
        assert_eq!(negotiation.budget_first, 2_000_000);
        assert_eq!(negotiation.budget_first_retry, 10_000_000);
        assert_eq!(negotiation.budget_retry, 30_000_000);
    }

    /// 2026-09-15 01:30 owner decision: a fresh net (`failed_count == 0`)
    /// gets a first attempt at `budget_first` and, only if that fails, one
    /// retry at `budget_first_retry` before probe/rip-up runs; a net that
    /// has already failed at least once this epoch gets a single attempt at
    /// `budget_retry` regardless of `attempt_index`; the last round is
    /// always unbounded.
    #[test]
    fn negotiated_search_budget_sequence_matches_failed_count() {
        let negotiation = NegotiationConfig::default();
        // Fresh net, mid-run round: (2 M, 10 M).
        assert_eq!(
            negotiated_search_budget(0, 1, 8, 0, &negotiation),
            Some(negotiation.budget_first)
        );
        assert_eq!(
            negotiated_search_budget(0, 1, 8, 1, &negotiation),
            Some(negotiation.budget_first_retry)
        );
        // A net that has already failed once this epoch: a single attempt
        // at the larger retry budget, regardless of attempt_index.
        assert_eq!(
            negotiated_search_budget(1, 1, 8, 0, &negotiation),
            Some(negotiation.budget_retry)
        );
        assert_eq!(
            negotiated_search_budget(1, 1, 8, 1, &negotiation),
            Some(negotiation.budget_retry)
        );
        // The last round is always unbounded, fresh or already-failed.
        assert_eq!(negotiated_search_budget(0, 8, 8, 0, &negotiation), None);
        assert_eq!(negotiated_search_budget(1, 8, 8, 0, &negotiation), None);
    }

    /// Config path: a custom `NegotiationConfig` (replacing
    /// `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_*`) picks the budget.
    #[test]
    fn negotiated_search_budget_uses_configured_values() {
        let negotiation = NegotiationConfig {
            budget_first: 111,
            budget_first_retry: 222,
            budget_retry: 333,
            ..NegotiationConfig::default()
        };
        assert_eq!(
            negotiated_search_budget(0, 1, 8, 0, &negotiation),
            Some(111)
        );
        assert_eq!(
            negotiated_search_budget(0, 1, 8, 1, &negotiation),
            Some(222)
        );
        assert_eq!(
            negotiated_search_budget(1, 1, 8, 0, &negotiation),
            Some(333)
        );
    }

    /// Every `AttemptKind` at the call site the loop uses it from, fresh
    /// net and already-failed net, mid-run round.
    #[test]
    fn ladder_budgets_match_the_loops_call_sites() {
        let negotiation = NegotiationConfig::default();
        let budgets = LadderBudgets(&negotiation);

        assert_eq!(
            budgets.budget(AttemptKind::First, 0, 1, 8),
            Some(negotiation.budget_first)
        );
        assert_eq!(
            budgets.budget(AttemptKind::First, 1, 1, 8),
            Some(negotiation.budget_retry)
        );
        assert_eq!(
            budgets.budget(AttemptKind::FirstRetry, 0, 1, 8),
            Some(negotiation.budget_first_retry)
        );
        // The post-rip-up search reuses the net's own plain-search budget.
        assert_eq!(
            budgets.budget(AttemptKind::PostRipUp, 0, 1, 8),
            Some(negotiation.budget_first)
        );
        assert_eq!(
            budgets.budget(AttemptKind::PostRipUp, 2, 1, 8),
            Some(negotiation.budget_retry)
        );
    }

    /// The last round: the three plain searches go unbounded.
    #[test]
    fn ladder_budgets_in_the_last_round_unbound_the_plain_searches() {
        let negotiation = NegotiationConfig::default();
        let budgets = LadderBudgets(&negotiation);

        assert_eq!(budgets.budget(AttemptKind::First, 0, 8, 8), None);
        assert_eq!(budgets.budget(AttemptKind::FirstRetry, 0, 8, 8), None);
        assert_eq!(budgets.budget(AttemptKind::PostRipUp, 1, 8, 8), None);
    }
}
