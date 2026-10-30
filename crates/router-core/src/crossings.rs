//! Crossing-specific routing configuration and guidance.
//!
//! This module intentionally stays independent from the A* hot path.  The
//! router can own a [`CrossingContext`] without consulting it until crossing
//! routing is explicitly enabled.

use rustc_hash::FxHashSet;

use crate::obstacle_map::NetId;

#[derive(Clone, Debug, PartialEq)]
pub struct CrossingConfig {
    pub enabled: bool,
    pub crossing_loss: f64,
    pub crossing_half_size_cells: i32,
}

impl Default for CrossingConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            crossing_loss: 0.0,
            crossing_half_size_cells: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct CrossingPair {
    pub low_net_id: NetId,
    pub high_net_id: NetId,
}

impl CrossingPair {
    pub fn new(a: NetId, b: NetId) -> Self {
        if a <= b {
            Self {
                low_net_id: a,
                high_net_id: b,
            }
        } else {
            Self {
                low_net_id: b,
                high_net_id: a,
            }
        }
    }
}

/// Contribution 1 (crossing-guided search): the topology plan's crossing
/// pairs as SOFT guidance. Guidance only changes the search price of a
/// crossing with a planned partner (`planned_crossing_loss`, normally 0);
/// unplanned crossings stay possible at the configured `crossing_loss`, and
/// nothing else in the router consults it. The router has no hard crossing
/// constraints (no whitelist, no per-net expectations) since phase 4 slice A
/// of the 2026-10 do-over.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CrossingGuidance {
    planned_pairs: FxHashSet<CrossingPair>,
    pub planned_crossing_loss: f64,
    /// S2: the plan predicts exactly one crossing per pair -- only the first
    /// crossing of a planned partner on a path is discounted, a second one
    /// (a braid) pays the full search price. `false` = every crossing of a
    /// planned partner is discounted (S1 behaviour).
    pub single_discounted_crossing_per_pair: bool,
}

impl CrossingGuidance {
    pub fn new(planned_pairs: &[(NetId, NetId)], planned_crossing_loss: f64) -> Self {
        Self {
            planned_pairs: planned_pairs
                .iter()
                .map(|&(a, b)| CrossingPair::new(a, b))
                .collect(),
            planned_crossing_loss,
            single_discounted_crossing_per_pair: true,
        }
    }

    pub fn with_single_discounted_crossing_per_pair(mut self, enabled: bool) -> Self {
        self.single_discounted_crossing_per_pair = enabled;
        self
    }

    #[inline]
    pub fn is_planned_pair(&self, net_id: NetId, partner_net_id: NetId) -> bool {
        self.planned_pairs
            .contains(&CrossingPair::new(net_id, partner_net_id))
    }

    pub fn planned_pair_count(&self) -> usize {
        self.planned_pairs.len()
    }

    /// The plan's pairs as `(net_id, net_id)` tuples, in no particular
    /// order. Exists only so a caller can rebuild a union guidance (e.g.
    /// the probe-guided search in `py_router.rs`'s negotiated engine,
    /// unioning the topology plan's pairs with a probe's own crossing
    /// partners) via `CrossingGuidance::new` -- nothing else needs to
    /// enumerate the set.
    pub fn pairs(&self) -> Vec<(NetId, NetId)> {
        self.planned_pairs
            .iter()
            .map(|pair| (pair.low_net_id, pair.high_net_id))
            .collect()
    }
}

#[derive(Clone, Debug, Default)]
pub struct CrossingContext {
    config: CrossingConfig,
    guidance: Option<CrossingGuidance>,
}

impl CrossingContext {
    pub fn new(config: CrossingConfig) -> Self {
        Self {
            config,
            guidance: None,
        }
    }

    #[inline]
    pub fn guidance(&self) -> Option<&CrossingGuidance> {
        self.guidance.as_ref()
    }

    pub fn set_guidance(&mut self, guidance: CrossingGuidance) {
        self.guidance = Some(guidance);
    }

    pub fn clear_guidance(&mut self) {
        self.guidance = None;
    }

    #[inline]
    pub fn config(&self) -> &CrossingConfig {
        &self.config
    }

    pub fn set_config(&mut self, config: CrossingConfig) {
        self.config = config;
    }

    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.config.enabled
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn guidance_is_soft_and_survives_a_config_replacement() {
        let mut context = CrossingContext::new(CrossingConfig::default());
        assert!(context.guidance().is_none());
        assert!(!context.is_enabled());

        context.set_guidance(CrossingGuidance::new(&[(5, 3), (7, 9)], 0.0));
        let guidance = context.guidance().expect("guidance set");
        assert_eq!(guidance.planned_pair_count(), 2);
        assert!(guidance.is_planned_pair(3, 5)); // unordered pair
        assert!(guidance.is_planned_pair(5, 3));
        assert!(!guidance.is_planned_pair(3, 7));
        assert_eq!(guidance.planned_crossing_loss, 0.0);
        assert!(guidance.single_discounted_crossing_per_pair); // S2 default

        // the config may be replaced without losing the guidance
        context.set_config(CrossingConfig {
            enabled: true,
            ..CrossingConfig::default()
        });
        assert!(context.is_enabled());
        assert_eq!(
            context
                .guidance()
                .map(|guidance| guidance.planned_pair_count()),
            Some(2)
        );

        context.clear_guidance();
        assert!(context.guidance().is_none());

        context.set_guidance(
            CrossingGuidance::new(&[(1, 2)], 0.0).with_single_discounted_crossing_per_pair(false),
        );
        assert!(
            !context
                .guidance()
                .unwrap()
                .single_discounted_crossing_per_pair
        );
    }
}
