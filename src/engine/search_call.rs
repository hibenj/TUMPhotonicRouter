//! One search-and-commit of one net, fully described as a value.
//!
//! Until phase 3 slice B of the 2026-10 do-over the engine had two
//! copy-pasted route-and-commit bodies (`route_single_net_and_commit_native`
//! and `..._repair_native`, 12 and 13 positional parameters) whose per-attempt
//! knobs travelled as four fields on `PyPhotonicRouter` that the negotiated
//! loop set before a search and reset after it. [`SearchCall`] carries all of
//! that; `PyPhotonicRouter::route_and_commit` is the one body it drives.

use crate::engine::*;

/// The three clearance radii of one search: the dynamic-obstacle expansion
/// radius the search itself honours, and the commit and core radii the
/// committed footprint is inflated by (`None` falls back to the block
/// radius, see `route_commit_and_core_cells`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SearchRadii {
    pub(crate) block_radius_cells: i32,
    pub(crate) commit_radius_cells: Option<i32>,
    pub(crate) core_radius_cells: Option<i32>,
}

impl SearchRadii {
    pub(crate) fn new(
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
    ) -> Self {
        SearchRadii {
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
        }
    }
}

/// The two flavours the old bodies were. Besides the knobs that
/// [`SearchCall`] names as fields (`try_order`, `simple_probe`,
/// `ignore_dynamic`), a `Repair` call differs from a `Plain` one in what the
/// body does after its searches, exactly as the old repair body did:
///
/// - a collision-crossing route whose commit is rejected falls through to
///   the ordinary search instead of returning the rejection error;
/// - the ordinary search runs on the call's own config at every block
///   radius, where a plain call at a positive block radius runs the
///   baseline config with simple routes off (baseline history weight);
/// - a failed ordinary search reports the bare `"No route found"` and does
///   not record `last_search_expanded_states`;
/// - no `"fallback route contains unreserved crossings"` check after an
///   upfront collision-crossing attempt;
/// - no retry with the rejected commit's keepout (the retry *is* a repair
///   call, see [`SearchCall::as_repair_retry`]);
/// - no `trace_plain_route ... stage=ordinary_search` line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SearchKind {
    Plain,
    Repair,
}

/// Everything one `route_and_commit` needs beyond the router's own state.
/// Built by [`SearchCall::plain`] or [`SearchCall::repair`] and refined with
/// the builder-style setters; the kind-derived fields (`kind`, `try_order`,
/// `simple_probe`, `ignore_dynamic`) are set by the constructors only.
#[derive(Clone, Copy)]
pub(crate) struct SearchCall<'a> {
    /// Source, target, opened cells and keys, clearance-exempt cells and
    /// keys, and the port positions all come from the job.
    pub(crate) job: &'a NativeRouteJob,
    pub(crate) radii: SearchRadii,
    pub(crate) kind: SearchKind,
    /// `PlainFirst` for a plain call, `CrossingFirst` for a repair call
    /// (see `CollisionCrossingTryOrder`'s own doc comment).
    pub(crate) try_order: CollisionCrossingTryOrder,
    /// Whether the straight/L/Z probe runs before the searches. False for a
    /// repair call, whose config also has `enable_simple_routes = false`.
    pub(crate) simple_probe: bool,
    /// `AStarConfig::ignore_dynamic_obstacles` override: `None` keeps the
    /// baseline (plain), `Some(false)` never ignores them (repair).
    pub(crate) ignore_dynamic: Option<bool>,
    /// Fail-fast expansion budget (`AStarConfig::total_expansion_budget`);
    /// `None` is unbounded.
    pub(crate) budget: Option<u64>,
    /// `AStarConfig::history_weight` for this call. 0.0 outside the
    /// negotiated loop, which matches the baseline (this codebase never
    /// constructs a router with a nonzero baseline history weight).
    pub(crate) history_weight: f64,
    /// `Some(weight)` replaces `AStarConfig::long_straight_congestion_weight`
    /// (the negotiated loop's exempt dense fan-out nets run at `Some(0.0)`).
    pub(crate) long_straight_weight_override: Option<f64>,
    /// Contribution 1's crossing-free search for a net without any planned
    /// crossing: the deferred lidar-pure crossing search is skipped, so a
    /// failed simple probe falls through to the ordinary (crossing-free)
    /// search. A plain-call knob; the old repair body never read it, so a
    /// repair call always carries `false`.
    pub(crate) crossing_free: bool,
    /// The probe-guided partner ids priced as temporary guidance for this
    /// one call (`with_probe_guided_guidance`); `None` or empty means no
    /// temporary guidance.
    pub(crate) guidance: Option<&'a [u64]>,
}

impl<'a> SearchCall<'a> {
    /// A fresh/native search: plain attempt first, simple probe on, the
    /// deferred lidar-pure crossing search if the plain attempt fails.
    pub(crate) fn plain(job: &'a NativeRouteJob, radii: SearchRadii) -> Self {
        SearchCall {
            job,
            radii,
            kind: SearchKind::Plain,
            try_order: CollisionCrossingTryOrder::PlainFirst,
            simple_probe: true,
            ignore_dynamic: None,
            budget: None,
            history_weight: 0.0,
            long_straight_weight_override: None,
            crossing_free: false,
            guidance: None,
        }
    }

    /// A repair search: collision-crossing partners first, no simple probe,
    /// dynamic obstacles never ignored, `history_weight` as given.
    pub(crate) fn repair(job: &'a NativeRouteJob, radii: SearchRadii, history_weight: f64) -> Self {
        SearchCall {
            job,
            radii,
            kind: SearchKind::Repair,
            try_order: CollisionCrossingTryOrder::CrossingFirst,
            simple_probe: false,
            ignore_dynamic: Some(false),
            budget: None,
            history_weight,
            long_straight_weight_override: None,
            crossing_free: false,
            guidance: None,
        }
    }

    pub(crate) fn budget(mut self, budget: Option<u64>) -> Self {
        self.budget = budget;
        self
    }

    pub(crate) fn history_weight(mut self, history_weight: f64) -> Self {
        self.history_weight = history_weight;
        self
    }

    pub(crate) fn long_straight_weight_override(mut self, weight: Option<f64>) -> Self {
        self.long_straight_weight_override = weight;
        self
    }

    pub(crate) fn crossing_free(mut self, crossing_free: bool) -> Self {
        self.crossing_free = crossing_free;
        self
    }

    pub(crate) fn guided(mut self, partner_ids: &'a [u64]) -> Self {
        self.guidance = Some(partner_ids);
        self
    }

    /// The one retry a plain call makes after a rejected commit with a
    /// non-empty keepout: the same job, radii, budget and long-straight
    /// override as a repair call at history weight 0.0. It runs inside the
    /// outer call's guidance scope, so it installs none of its own.
    pub(crate) fn as_repair_retry(&self) -> SearchCall<'a> {
        SearchCall::repair(self.job, self.radii, 0.0)
            .budget(self.budget)
            .long_straight_weight_override(self.long_straight_weight_override)
    }

    pub(crate) fn is_repair(&self) -> bool {
        self.kind == SearchKind::Repair
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn job() -> NativeRouteJob {
        NativeRouteJob::new(
            7,
            PyState::new(1, 2, 0),
            PyState::new(3, 4, 4),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            None,
            None,
        )
    }

    #[test]
    fn a_plain_call_carries_the_old_native_body_s_fixed_choices() {
        let job = job();
        let call = SearchCall::plain(&job, SearchRadii::new(1, Some(2), Some(3)));
        assert_eq!(call.kind, SearchKind::Plain);
        assert_eq!(call.try_order, CollisionCrossingTryOrder::PlainFirst);
        assert!(call.simple_probe);
        assert_eq!(call.ignore_dynamic, None);
        assert_eq!(call.budget, None);
        assert_eq!(call.history_weight, 0.0);
        assert_eq!(call.long_straight_weight_override, None);
        assert!(!call.crossing_free);
        assert!(call.guidance.is_none());
        assert_eq!(call.radii, SearchRadii::new(1, Some(2), Some(3)));
    }

    #[test]
    fn a_repair_call_carries_the_old_repair_body_s_fixed_choices() {
        let job = job();
        let call = SearchCall::repair(&job, SearchRadii::new(1, None, None), 2.0);
        assert_eq!(call.kind, SearchKind::Repair);
        assert!(call.is_repair());
        assert_eq!(call.try_order, CollisionCrossingTryOrder::CrossingFirst);
        assert!(!call.simple_probe);
        assert_eq!(call.ignore_dynamic, Some(false));
        assert_eq!(call.history_weight, 2.0);
        assert!(!call.crossing_free);
    }

    #[test]
    fn the_setters_refine_a_call_and_the_retry_keeps_budget_and_long_straight_override() {
        let job = job();
        let partners = [5u64, 6];
        let call = SearchCall::plain(&job, SearchRadii::new(1, Some(1), Some(1)))
            .budget(Some(2_000_000))
            .history_weight(2.0)
            .long_straight_weight_override(Some(0.0))
            .crossing_free(true)
            .guided(&partners);
        assert_eq!(call.budget, Some(2_000_000));
        assert_eq!(call.history_weight, 2.0);
        assert_eq!(call.long_straight_weight_override, Some(0.0));
        assert!(call.crossing_free);
        assert_eq!(call.guidance, Some(&partners[..]));

        let retry = call.as_repair_retry();
        assert!(retry.is_repair());
        assert_eq!(retry.budget, Some(2_000_000));
        assert_eq!(retry.long_straight_weight_override, Some(0.0));
        assert_eq!(
            retry.history_weight, 0.0,
            "the retry runs at history weight 0.0"
        );
        assert!(!retry.crossing_free);
        assert!(retry.guidance.is_none());
        assert_eq!(retry.radii, call.radii);
        assert!(std::ptr::eq(retry.job, call.job));
    }
}
