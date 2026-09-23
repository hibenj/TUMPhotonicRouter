//! The negotiated loop's search-budget ladder: which expansion budget each
//! attempt of a net's per-round sequence runs under.
//!
//! Extracted from `loop_.rs` unchanged (Milestone 4 Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! 2026-09-23); the numbers are exactly the ones the loop used at every
//! call site before the extraction.

use crate::config::NegotiationConfig;
use crate::engine::{
    NEGOTIATED_BUDGET_BRAID, NEGOTIATED_BUDGET_FIRST_ATTEMPT, NEGOTIATED_BUDGET_RETRY,
};

/// One step of a net's per-round attempt sequence, in the order the loop
/// runs them: the plain search, its fresh-net retry, the probe-guided
/// search, the direct-crossing search, the post-rip-up search, and the
/// braid passes that follow every committed route.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AttemptKind {
    First,
    FirstRetry,
    PostRipUp,
    ProbeGuided,
    DirectCrossing,
    /// The braid passes set `NEGOTIATED_BUDGET_BRAID` at their own two call
    /// sites (`braid.rs`, `search_calls.rs`), so nothing outside the tests
    /// builds this variant yet; the ladder carries the same number so the
    /// whole budget ladder can be read in one place.
    #[allow(dead_code)]
    Braid,
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
/// in the last round, while the probe-guided, direct-crossing and braid
/// searches run at their own fixed constants *in every round, the last one
/// included* (those three are bounded repairs of an already-failed search,
/// not the search that must not lose a route).
pub(crate) struct LadderBudgets<'a>(pub(crate) &'a NegotiationConfig);

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
            AttemptKind::ProbeGuided => Some(NEGOTIATED_BUDGET_RETRY),
            AttemptKind::DirectCrossing => Some(NEGOTIATED_BUDGET_FIRST_ATTEMPT),
            AttemptKind::Braid => Some(NEGOTIATED_BUDGET_BRAID),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::NEGOTIATED_BUDGET_FIRST_RETRY;

    /// 2026-09-15 01:30 owner decision: a fresh net (`failed_count == 0`)
    /// gets a first attempt at `NEGOTIATED_BUDGET_FIRST_ATTEMPT` (2 M) and,
    /// only if that fails, one retry at `NEGOTIATED_BUDGET_FIRST_RETRY`
    /// (10 M) before probe/rip-up runs; a net that has already failed at
    /// least once this epoch gets a single attempt at
    /// `NEGOTIATED_BUDGET_RETRY` (30 M) regardless of `attempt_index`; the
    /// last round is always unbounded.
    #[test]
    fn negotiated_search_budget_sequence_matches_failed_count() {
        let negotiation = NegotiationConfig::default();
        // Fresh net, mid-run round: (2 M, 10 M).
        assert_eq!(
            negotiated_search_budget(0, 1, 8, 0, &negotiation),
            Some(NEGOTIATED_BUDGET_FIRST_ATTEMPT)
        );
        assert_eq!(
            negotiated_search_budget(0, 1, 8, 1, &negotiation),
            Some(NEGOTIATED_BUDGET_FIRST_RETRY)
        );
        // A net that has already failed once this epoch: a single attempt
        // at the larger retry budget, regardless of attempt_index.
        assert_eq!(
            negotiated_search_budget(1, 1, 8, 0, &negotiation),
            Some(NEGOTIATED_BUDGET_RETRY)
        );
        assert_eq!(
            negotiated_search_budget(1, 1, 8, 1, &negotiation),
            Some(NEGOTIATED_BUDGET_RETRY)
        );
        // The last round is always unbounded, fresh or already-failed.
        assert_eq!(negotiated_search_budget(0, 8, 8, 0, &negotiation), None);
        assert_eq!(negotiated_search_budget(1, 8, 8, 0, &negotiation), None);
    }

    /// Config path: a custom `NegotiationConfig` (replacing
    /// `PHOTONIC_ROUTER_NEGOTIATED_BUDGET_*`) picks the budget instead of
    /// the constants.
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
            Some(NEGOTIATED_BUDGET_FIRST_ATTEMPT)
        );
        assert_eq!(
            budgets.budget(AttemptKind::First, 1, 1, 8),
            Some(NEGOTIATED_BUDGET_RETRY)
        );
        assert_eq!(
            budgets.budget(AttemptKind::FirstRetry, 0, 1, 8),
            Some(NEGOTIATED_BUDGET_FIRST_RETRY)
        );
        // The post-rip-up search reuses the net's own plain-search budget.
        assert_eq!(
            budgets.budget(AttemptKind::PostRipUp, 0, 1, 8),
            Some(NEGOTIATED_BUDGET_FIRST_ATTEMPT)
        );
        assert_eq!(
            budgets.budget(AttemptKind::PostRipUp, 2, 1, 8),
            Some(NEGOTIATED_BUDGET_RETRY)
        );
        assert_eq!(
            budgets.budget(AttemptKind::ProbeGuided, 0, 1, 8),
            Some(NEGOTIATED_BUDGET_RETRY)
        );
        assert_eq!(
            budgets.budget(AttemptKind::DirectCrossing, 3, 1, 8),
            Some(NEGOTIATED_BUDGET_FIRST_ATTEMPT)
        );
        assert_eq!(
            budgets.budget(AttemptKind::Braid, 0, 1, 8),
            Some(NEGOTIATED_BUDGET_BRAID)
        );
    }

    /// The last round: the three plain searches go unbounded, the three
    /// fixed-constant repairs do not -- exactly as the loop called them
    /// before this extraction.
    #[test]
    fn ladder_budgets_in_the_last_round_unbound_only_the_plain_searches() {
        let negotiation = NegotiationConfig::default();
        let budgets = LadderBudgets(&negotiation);

        assert_eq!(budgets.budget(AttemptKind::First, 0, 8, 8), None);
        assert_eq!(budgets.budget(AttemptKind::FirstRetry, 0, 8, 8), None);
        assert_eq!(budgets.budget(AttemptKind::PostRipUp, 1, 8, 8), None);
        assert_eq!(
            budgets.budget(AttemptKind::ProbeGuided, 0, 8, 8),
            Some(NEGOTIATED_BUDGET_RETRY)
        );
        assert_eq!(
            budgets.budget(AttemptKind::DirectCrossing, 0, 8, 8),
            Some(NEGOTIATED_BUDGET_FIRST_ATTEMPT)
        );
        assert_eq!(
            budgets.budget(AttemptKind::Braid, 0, 8, 8),
            Some(NEGOTIATED_BUDGET_BRAID)
        );
    }
}
