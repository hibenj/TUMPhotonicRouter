//! Crossing-specific routing constraints.
//!
//! This module intentionally stays independent from the A* hot path.  The
//! router can own a [`CrossingContext`] without consulting it until crossing
//! routing is explicitly enabled.

use rustc_hash::{FxHashMap, FxHashSet};

use crate::obstacle_map::NetId;

#[derive(Clone, Debug, PartialEq)]
pub struct CrossingConfig {
    pub enabled: bool,
    pub crossing_loss: f64,
    pub crossing_half_size_cells: i32,
    pub min_straight_cells_per_crossing: i32,
    pub allow_only_expected_pairs: bool,
}

impl Default for CrossingConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            crossing_loss: 0.0,
            crossing_half_size_cells: 0,
            min_straight_cells_per_crossing: 0,
            allow_only_expected_pairs: true,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CrossingConstraint {
    pub net_id: NetId,
    pub partner_net_id: NetId,
    pub level: u32,
    pub source_depth: u32,
    pub target_depth: u32,
}

impl CrossingConstraint {
    pub fn pair(&self) -> CrossingPair {
        CrossingPair::new(self.net_id, self.partner_net_id)
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

#[derive(Clone, Debug, Default)]
pub struct CrossingContext {
    config: CrossingConfig,
    constraints: Vec<CrossingConstraint>,
    allowed_pairs: FxHashSet<CrossingPair>,
    expected_crossings_by_net: FxHashMap<NetId, u32>,
}

impl CrossingContext {
    pub fn new(config: CrossingConfig, constraints: Vec<CrossingConstraint>) -> Self {
        let mut allowed_pairs = FxHashSet::default();
        let mut expected_crossings_by_net = FxHashMap::default();
        for constraint in &constraints {
            allowed_pairs.insert(constraint.pair());
            *expected_crossings_by_net
                .entry(constraint.net_id)
                .or_insert(0) += 1;
            *expected_crossings_by_net
                .entry(constraint.partner_net_id)
                .or_insert(0) += 1;
        }
        Self {
            config,
            constraints,
            allowed_pairs,
            expected_crossings_by_net,
        }
    }

    #[inline]
    pub fn config(&self) -> &CrossingConfig {
        &self.config
    }

    pub fn set_config(&mut self, config: CrossingConfig) {
        self.config = config;
    }

    #[inline]
    pub fn constraints(&self) -> &[CrossingConstraint] {
        &self.constraints
    }

    pub fn replace_constraints(&mut self, constraints: Vec<CrossingConstraint>) {
        let config = self.config.clone();
        *self = Self::new(config, constraints);
    }

    pub fn clear_constraints(&mut self) {
        self.constraints.clear();
        self.allowed_pairs.clear();
        self.expected_crossings_by_net.clear();
    }

    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.config.enabled
    }

    #[inline]
    pub fn has_expected_pair(&self, a: NetId, b: NetId) -> bool {
        self.allowed_pairs.contains(&CrossingPair::new(a, b))
    }

    #[inline]
    pub fn allows_pair(&self, a: NetId, b: NetId) -> bool {
        if !self.config.enabled {
            return false;
        }
        !self.config.allow_only_expected_pairs || self.has_expected_pair(a, b)
    }

    #[inline]
    pub fn expected_crossing_count(&self, net_id: NetId) -> u32 {
        self.expected_crossings_by_net
            .get(&net_id)
            .copied()
            .unwrap_or(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crossing_context_counts_and_normalizes_pairs() {
        let config = CrossingConfig {
            enabled: true,
            ..CrossingConfig::default()
        };
        let context = CrossingContext::new(
            config,
            vec![
                CrossingConstraint {
                    net_id: 10,
                    partner_net_id: 2,
                    level: 0,
                    source_depth: 1,
                    target_depth: 2,
                },
                CrossingConstraint {
                    net_id: 10,
                    partner_net_id: 5,
                    level: 1,
                    source_depth: 1,
                    target_depth: 2,
                },
            ],
        );

        assert!(context.has_expected_pair(2, 10));
        assert!(context.has_expected_pair(10, 2));
        assert!(context.allows_pair(5, 10));
        assert!(!context.allows_pair(2, 5));
        assert_eq!(context.expected_crossing_count(10), 2);
        assert_eq!(context.expected_crossing_count(2), 1);
        assert_eq!(context.expected_crossing_count(99), 0);
    }

    #[test]
    fn disabled_context_never_allows_pairs() {
        let context = CrossingContext::new(
            CrossingConfig::default(),
            vec![CrossingConstraint {
                net_id: 1,
                partner_net_id: 2,
                level: 0,
                source_depth: 0,
                target_depth: 1,
            }],
        );

        assert!(context.has_expected_pair(1, 2));
        assert!(!context.allows_pair(1, 2));
    }
}
