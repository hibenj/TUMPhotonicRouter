//! The negotiated loop's work queue and the per-net bookkeeping that
//! lives beside it across rounds.
//!
//! Extracted from `loop_.rs` unchanged (Milestone 4 Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! 2026-09-23): the same four pieces of state, with the loop's own
//! requeue orders as named methods.

use std::collections::VecDeque;

use rustc_hash::{FxHashMap, FxHashSet};

/// The nets still to route, plus the three tables the loop's rules read:
/// how often each net has failed (the history-cost gate and the budget
/// ladder), which nets the local rip-up rule has already ripped this epoch
/// (LiDAR's `ripuplocalnets` once-per-epoch rule), and each net's most
/// recent probe blockers (what `global_ripup_round` rips at round end).
///
/// The fields are `pub(crate)` because `global_ripup_round` -- kept exactly
/// as it was, including its own epoch boundary -- takes the queue, the
/// failure counts and the ripped-once set as three separate borrows.
pub(crate) struct NetQueue {
    pub(crate) order: VecDeque<u64>,
    pub(crate) failed_counts: FxHashMap<u64, u32>,
    pub(crate) ripped_once: FxHashSet<u64>,
    pub(crate) last_blockers: FxHashMap<u64, Vec<u64>>,
}

impl NetQueue {
    /// The batch's nets in the caller's order (`route_rust.py`'s
    /// `_topological_net_route_order`), nothing failed yet.
    pub(crate) fn new(net_ids: impl IntoIterator<Item = u64>) -> Self {
        NetQueue {
            order: net_ids.into_iter().collect(),
            failed_counts: FxHashMap::default(),
            ripped_once: FxHashSet::default(),
            last_blockers: FxHashMap::default(),
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    pub(crate) fn len(&self) -> usize {
        self.order.len()
    }

    /// The net a non-converged batch reports as the failed one.
    pub(crate) fn front(&self) -> Option<u64> {
        self.order.front().copied()
    }

    /// Drains the queue into this round's work list: nets requeued while
    /// the round runs land in the (now empty) queue and are tried again
    /// next round, never twice in the same round.
    pub(crate) fn take_round(&mut self) -> VecDeque<u64> {
        std::mem::take(&mut self.order)
    }

    /// How often this net has failed since the last epoch reset. 0 means a
    /// fresh net: no history weight, and the small first-attempt budget
    /// with one retry.
    pub(crate) fn failed_count(&self, net_id: u64) -> u32 {
        *self.failed_counts.get(&net_id).unwrap_or(&0)
    }

    /// A net that failed this round: it goes to the back, behind everything
    /// already requeued, with its failure count raised.
    pub(crate) fn requeue_back(&mut self, net_id: u64) {
        *self.failed_counts.entry(net_id).or_insert(0) += 1;
        self.order.push_back(net_id);
    }

    /// Nets this net displaced: to the front, in their original relative
    /// order, so each ripped partner reroutes right behind the net that
    /// displaced it -- LiDAR's "ripped nets go back to the queue and route
    /// after it". The `failed_counts` bump is what turns history cost on
    /// for that reroute (see the loop's `commit_history_weight` gate); it
    /// must stay off for a net's first attempt each epoch, or the JPS4 fast
    /// path is disabled for every search, not just contested ones. They are
    /// marked as ripped this epoch, which for the local rip-up's victims
    /// the rip-up itself has already done.
    pub(crate) fn requeue_front_preserving_order(&mut self, victims: &[u64]) {
        for &victim_id in victims.iter().rev() {
            *self.failed_counts.entry(victim_id).or_insert(0) += 1;
            self.ripped_once.insert(victim_id);
            self.order.push_front(victim_id);
        }
    }

    /// Partners a braid escalation of this net left ripped
    /// (`BraidRepairOutcome::VictimRipped`) -- requeued exactly like any
    /// other victim.
    pub(crate) fn requeue_braid_victims(&mut self, victims: &[u64]) {
        self.requeue_front_preserving_order(victims);
    }

    /// The blockers of this net's most recent probe, for
    /// `global_ripup_round` to consult at round end.
    pub(crate) fn record_blockers(&mut self, net_id: u64, blockers: Vec<u64>) {
        self.last_blockers.insert(net_id, blockers);
    }

    /// An epoch boundary: the once-per-epoch rip-up set always clears, and
    /// with `clear_failed_counts` (the loop's "two rounds without progress"
    /// reset) every net becomes fresh again -- first-attempt budget, no
    /// history weight. `global_ripup_round`'s own epoch boundary clears the
    /// rip-up set only.
    pub(crate) fn epoch_reset(&mut self, clear_failed_counts: bool) {
        if clear_failed_counts {
            self.failed_counts.clear();
        }
        self.ripped_once.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn take_round_drains_the_queue_and_leaves_requeues_for_the_next_round() {
        let mut queue = NetQueue::new([1u64, 2, 3]);

        let this_round: Vec<u64> = queue.take_round().into_iter().collect();

        assert_eq!(this_round, vec![1, 2, 3]);
        assert!(queue.is_empty(), "the queue is empty while the round runs");
        queue.requeue_back(2);
        assert_eq!(queue.len(), 1);
        assert_eq!(
            queue.take_round().into_iter().collect::<Vec<u64>>(),
            vec![2],
            "net 2 is tried again next round, not twice this round"
        );
    }

    #[test]
    fn requeue_back_raises_the_failure_count() {
        let mut queue = NetQueue::new([1u64]);
        let _ = queue.take_round();

        queue.requeue_back(1);
        queue.requeue_back(1);

        assert_eq!(queue.failed_count(1), 2);
        assert_eq!(queue.failed_count(7), 0, "an untouched net is fresh");
        assert_eq!(
            queue.order.iter().copied().collect::<Vec<u64>>(),
            vec![1, 1]
        );
    }

    #[test]
    fn requeue_front_keeps_the_relative_order_and_marks_ripped_once() {
        let mut queue = NetQueue::new([9u64, 10]);
        queue.requeue_back(7);
        queue.order.pop_back();
        assert_eq!(queue.failed_count(7), 1);

        queue.requeue_front_preserving_order(&[7, 8]);

        assert_eq!(
            queue.order.iter().copied().collect::<Vec<u64>>(),
            vec![7, 8, 9, 10],
            "victims go to the front in their original order"
        );
        assert_eq!(queue.failed_count(7), 2);
        assert_eq!(queue.failed_count(8), 1);
        assert!(queue.ripped_once.contains(&7) && queue.ripped_once.contains(&8));
    }

    #[test]
    fn requeue_braid_victims_is_the_same_front_requeue() {
        let mut queue = NetQueue::new([9u64, 10]);
        queue.failed_counts.insert(7, 1);

        queue.requeue_braid_victims(&[7, 8]);

        assert_eq!(
            queue.order.iter().copied().collect::<Vec<u64>>(),
            vec![7, 8, 9, 10]
        );
        assert_eq!(queue.failed_count(7), 2);
        assert_eq!(queue.failed_count(8), 1);
        assert!(queue.ripped_once.contains(&7) && queue.ripped_once.contains(&8));
    }

    #[test]
    fn both_epoch_resets_clear_what_they_are_meant_to() {
        let mut queue = NetQueue::new([1u64]);
        queue.requeue_back(1);
        queue.ripped_once.insert(5);
        queue.record_blockers(1, vec![5]);

        // `global_ripup_round`'s own boundary: the rip-up set only.
        queue.epoch_reset(false);
        assert!(queue.ripped_once.is_empty());
        assert_eq!(queue.failed_count(1), 1, "failure counts survive");

        // The loop's "two rounds without progress" reset: both.
        queue.ripped_once.insert(5);
        queue.epoch_reset(true);
        assert!(queue.ripped_once.is_empty());
        assert_eq!(queue.failed_count(1), 0);
        assert_eq!(
            queue.last_blockers.get(&1),
            Some(&vec![5]),
            "the probe blockers are not part of an epoch reset"
        );
    }
}
