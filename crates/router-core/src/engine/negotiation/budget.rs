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

#[cfg(test)]
mod tests {
    use super::*;

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
