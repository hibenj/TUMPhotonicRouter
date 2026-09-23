//! Which nets the negotiated loop searches crossing-free (contribution 1).
//!
//! Extracted from `loop_.rs` unchanged (Milestone 4 Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! 2026-09-23).

use rustc_hash::FxHashSet;

/// The rule that decides whether one net's searches this round run
/// crossing-free (every committed net an obstacle, no crossing priced or
/// accepted) instead of paying the normal crossing price.
pub(crate) trait CrossingFreePolicy {
    fn crossing_free(&self, net_id: u64, round: u32, max_rounds: u32) -> bool;
}

/// Contribution 1's crossing-free search for unplanned nets (owner decision
/// 2026-09-16 00:10, Milestone 7 of
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`):
/// under `lidar-guided`, a net that has no crossing at all in the topology
/// plan (no planned pair of the crossing guidance names it) is searched
/// crossing-free, and when that fails, every net the probe's free path
/// crosses is ripped up (not only the illegal partners) and the net is
/// searched crossing-free again. The 64x64 mesh's input fan-out (bands
/// 1-2, 0 planned crossings) is where both 64x64 mesh runs realized their
/// 8 unplanned crossings; the plan already says those nets need none. The
/// last round keeps the baseline behaviour (crossings allowed) so the rule
/// can never lose a route the priced search would have found.
/// `PHOTONIC_ROUTER_NEGOTIATED_CROSSING_FREE_UNPLANNED=0` turns the rule
/// off for A/B runs; without guidance (lidar-pure) it never applies, which
/// is [`Never`] below.
pub(crate) fn negotiated_crossing_free_net(
    planned_net_ids: Option<&FxHashSet<u64>>,
    net_id: u64,
    round: u32,
    max_rounds: u32,
) -> bool {
    match planned_net_ids {
        Some(planned) => round < max_rounds && !planned.contains(&net_id),
        None => false,
    }
}

/// The rule above, holding the set of nets that appear in at least one
/// planned pair of the current guidance.
pub(crate) struct PlannedPairsOnly(pub(crate) FxHashSet<u64>);

impl CrossingFreePolicy for PlannedPairsOnly {
    fn crossing_free(&self, net_id: u64, round: u32, max_rounds: u32) -> bool {
        negotiated_crossing_free_net(Some(&self.0), net_id, round, max_rounds)
    }
}

/// No net is ever searched crossing-free: lidar-pure (no guidance), and
/// `PHOTONIC_ROUTER_NEGOTIATED_CROSSING_FREE_UNPLANNED=0`.
pub(crate) struct Never;

impl CrossingFreePolicy for Never {
    fn crossing_free(&self, _net_id: u64, _round: u32, _max_rounds: u32) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn negotiated_crossing_free_net_applies_only_to_unplanned_nets_before_the_last_round() {
        let planned: FxHashSet<u64> = [1u64, 2].into_iter().collect();
        assert!(
            negotiated_crossing_free_net(Some(&planned), 3, 1, 10),
            "a net without any planned pair is searched crossing-free"
        );
        assert!(
            !negotiated_crossing_free_net(Some(&planned), 1, 1, 10),
            "a net with a planned pair keeps the priced crossing search"
        );
        assert!(
            !negotiated_crossing_free_net(Some(&planned), 3, 10, 10),
            "the last round keeps the baseline behaviour for every net"
        );
        assert!(
            !negotiated_crossing_free_net(None, 3, 1, 10),
            "without guidance (lidar-pure) the rule never applies"
        );
    }

    #[test]
    fn planned_pairs_only_matches_the_rule_it_wraps() {
        let policy = PlannedPairsOnly([1u64, 2].into_iter().collect());
        assert!(policy.crossing_free(3, 1, 10));
        assert!(!policy.crossing_free(1, 1, 10));
        assert!(!policy.crossing_free(3, 10, 10), "last round is exempt");
    }

    #[test]
    fn never_is_crossing_free_for_nobody() {
        let policy = Never;
        assert!(!policy.crossing_free(3, 1, 10));
        assert!(!policy.crossing_free(3, 10, 10));
    }
}
