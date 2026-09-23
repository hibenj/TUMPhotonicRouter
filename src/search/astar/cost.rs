//! Per-primitive search-cost metadata and the g-cost terms the kernel
//! loop sums for every move: the primitive's own length and bend cost
//! (`length_cost`, `bend_cost`, folded once per primitive into
//! `PrimitiveSearchMetadata::base_step_cost`), and the grid-dependent
//! history, long-straight-congestion and proactive lateral-congestion
//! terms (`history_cost_in_local_rect`, `congestion_cost_in_local_rect`
//! and their footprint-summing callers, `primitive_lateral_congestion`),
//! moved here from `DenseRoutingGrid` (which keeps its storage and
//! legality queries) as free functions over `&DenseRoutingGrid`.
//! `step_cost` sums the per-move terms in exactly the order the kernel
//! loop's four near-identical inline blocks used to (Milestone 3, Slice
//! 3); `heuristic_estimate` names the `SearchHeuristic::estimate` call.
//! No default, constant or floating-point operation order changes from
//! the pre-Slice-3 inline code.

use crate::obstacle_map::pack_xy;
use crate::primitives::{Primitive, DIRECTIONS};
use crate::search::astar::config::AStarConfig;
use crate::search::astar::dense::DenseRoutingGrid;
use crate::search::astar::expansion::{primitive_transition_class, FootprintCollisionProfile};
use crate::search::astar::heuristic::SearchHeuristic;
use crate::search::state::State;
use rustc_hash::FxHashSet;

#[derive(Clone, Copy, Debug)]
pub(crate) struct PrimitiveSearchMetadata {
    pub(crate) transition_class: usize,
    pub(crate) base_step_cost: f64,
}

impl PrimitiveSearchMetadata {
    pub(crate) fn from_primitive(primitive: &Primitive, bend_weight: f64) -> Self {
        Self {
            transition_class: primitive_transition_class(
                &primitive.geometry,
                primitive.dx,
                primitive.dy,
            ),
            base_step_cost: length_cost(primitive) + bend_cost(primitive, bend_weight),
        }
    }
}

/// The primitive's raw path length in micrometers, full weight in the
/// step cost (the term named `length_um` in every primitive's geometry).
#[inline]
pub(crate) fn length_cost(primitive: &Primitive) -> f64 {
    primitive.length_um
}

/// The primitive's weighted bend penalty: `bend_weight` scales the
/// primitive's precomputed `bend_cost` (0.0 for every straight
/// primitive).
#[inline]
pub(crate) fn bend_cost(primitive: &Primitive, bend_weight: f64) -> f64 {
    bend_weight * primitive.bend_cost
}

/// `SearchHeuristic::estimate`, named for symmetry with `step_cost` at
/// the loop's call sites; the hook's own `heuristic_bonus` is a separate,
/// pluggable term the loop still adds itself (unchanged: not every call
/// site added it even before this slice -- Tier 1's own neighbour push
/// never did, only the source seed and every Tier 2 push do).
#[inline]
pub(crate) fn heuristic_estimate(heuristic: &SearchHeuristic, state: State) -> f64 {
    heuristic.estimate(state)
}

/// Sums one move's g-cost terms in today's exact floating-point order:
/// `carry` first, then the primitive's precomputed base cost, then the
/// grid's history, long-straight-congestion and proactive
/// lateral-congestion terms. `carry` exists only so every one of the
/// kernel's four call sites can reuse this function without changing its
/// own addition order: pass `0.0` where the caller adds its running
/// total *after* this sum returns (today's Tier 1 and Tier 2 paths, and
/// the eager completion chain's per-step loop), or the running total
/// itself where today's code folds it into the very same left-to-right
/// sum (the eager completion chain's first step). `0.0 + x == x` bit for
/// bit for every finite, non-negative `x` these terms ever produce, so
/// both calling conventions reproduce their original inline expression
/// exactly.
#[inline]
#[allow(clippy::too_many_arguments)]
pub(crate) fn step_cost(
    grid: &DenseRoutingGrid,
    config: &AStarConfig,
    carry: f64,
    base_step_cost: f64,
    origin_x: i32,
    origin_y: i32,
    footprint: &[(i32, i32)],
    profile: &FootprintCollisionProfile,
    start_angle: u8,
    is_straight: bool,
) -> f64 {
    let history_cost = if config.history_weight > 0.0 {
        primitive_footprint_history_with_profile(grid, origin_x, origin_y, footprint, profile)
            as f64
            * config.history_weight
    } else {
        0.0
    };
    let long_straight_congestion_cost = if config.long_straight_congestion_weight > 0.0 {
        primitive_footprint_congestion_with_profile(grid, origin_x, origin_y, footprint, profile)
            as f64
            * config.long_straight_congestion_weight
    } else {
        0.0
    };
    let congestion_cost = if config.proactive_congestion_weight > 0.0
        && config.proactive_congestion_radius_cells > 0
        && is_straight
    {
        f64::from(primitive_lateral_congestion(
            grid,
            origin_x,
            origin_y,
            footprint,
            start_angle,
            config.proactive_congestion_radius_cells,
        )) * config.proactive_congestion_weight
    } else {
        0.0
    };
    carry + base_step_cost + history_cost + long_straight_congestion_cost + congestion_cost
}

/// The summed history value of a local rectangle of a `DenseRoutingGrid`,
/// via its 2D prefix sum, `None` if the rectangle falls outside the grid
/// or the grid never built a history layer. Moved from
/// `DenseRoutingGrid::history_cost_in_local_rect` (Milestone 3, Slice 3);
/// no behaviour change.
#[inline]
pub(crate) fn history_cost_in_local_rect(
    grid: &DenseRoutingGrid,
    local_min_x: i32,
    local_max_x: i32,
    local_min_y: i32,
    local_max_y: i32,
) -> Option<u64> {
    let width = grid.width;
    if local_min_x > local_max_x || local_min_y > local_max_y {
        return None;
    }
    if local_min_x < 0 || local_min_y < 0 || local_max_x >= width || local_max_y >= grid.height {
        return None;
    }

    let prefix = grid.history_prefix.as_ref()?;
    let width_usize = usize::try_from(width).ok()?;
    let stride = width_usize.checked_add(1)?;
    let x1 = usize::try_from(local_min_x).ok()?;
    let y1 = usize::try_from(local_min_y).ok()?;
    let x2 = usize::try_from(local_max_x).ok()?;
    let y2 = usize::try_from(local_max_y).ok()?;

    let a = prefix[(y2 + 1).checked_mul(stride)? + (x2 + 1)];
    let b = prefix[y1.checked_mul(stride)? + (x2 + 1)];
    let c = prefix[(y2 + 1).checked_mul(stride)? + x1];
    let d = prefix[y1.checked_mul(stride)? + x1];
    Some(a.saturating_add(d).saturating_sub(b).saturating_sub(c))
}

/// The summed congestion value of a local rectangle, the congestion-layer
/// twin of `history_cost_in_local_rect`. Moved from
/// `DenseRoutingGrid::congestion_cost_in_local_rect` (Milestone 3, Slice
/// 3); no behaviour change.
#[inline]
pub(crate) fn congestion_cost_in_local_rect(
    grid: &DenseRoutingGrid,
    local_min_x: i32,
    local_max_x: i32,
    local_min_y: i32,
    local_max_y: i32,
) -> Option<u64> {
    let width = grid.width;
    if local_min_x > local_max_x || local_min_y > local_max_y {
        return None;
    }
    if local_min_x < 0 || local_min_y < 0 || local_max_x >= width || local_max_y >= grid.height {
        return None;
    }

    let prefix = grid.congestion_prefix.as_ref()?;
    let width_usize = usize::try_from(width).ok()?;
    let stride = width_usize.checked_add(1)?;
    let x1 = usize::try_from(local_min_x).ok()?;
    let y1 = usize::try_from(local_min_y).ok()?;
    let x2 = usize::try_from(local_max_x).ok()?;
    let y2 = usize::try_from(local_max_y).ok()?;

    let a = prefix[(y2 + 1).checked_mul(stride)? + (x2 + 1)];
    let b = prefix[y1.checked_mul(stride)? + (x2 + 1)];
    let c = prefix[(y2 + 1).checked_mul(stride)? + x1];
    let d = prefix[y1.checked_mul(stride)? + x1];
    Some(a.saturating_add(d).saturating_sub(b).saturating_sub(c))
}

/// A primitive footprint's total history cost at one origin: the fast
/// rectangle path via `history_cost_in_local_rect` when the footprint's
/// collision profile is a full rectangle, else a per-cell sum over the
/// grid's history layer (0 if the grid never built one). Moved from
/// `DenseRoutingGrid::primitive_footprint_history_with_profile`
/// (Milestone 3, Slice 3); no behaviour change.
#[inline]
pub(crate) fn primitive_footprint_history_with_profile(
    grid: &DenseRoutingGrid,
    origin_x: i32,
    origin_y: i32,
    footprint: &[(i32, i32)],
    profile: &FootprintCollisionProfile,
) -> u64 {
    if profile.is_full_rect {
        let Some(rect_min_x) = origin_x.checked_add(profile.min_dx) else {
            return u64::MAX;
        };
        let Some(rect_max_x) = origin_x.checked_add(profile.max_dx) else {
            return u64::MAX;
        };
        let Some(rect_min_y) = origin_y.checked_add(profile.min_dy) else {
            return u64::MAX;
        };
        let Some(rect_max_y) = origin_y.checked_add(profile.max_dy) else {
            return u64::MAX;
        };
        let Some(local_min_x) = rect_min_x.checked_sub(grid.bounds.min_x) else {
            return u64::MAX;
        };
        let Some(local_max_x) = rect_max_x.checked_sub(grid.bounds.min_x) else {
            return u64::MAX;
        };
        let Some(local_min_y) = rect_min_y.checked_sub(grid.bounds.min_y) else {
            return u64::MAX;
        };
        let Some(local_max_y) = rect_max_y.checked_sub(grid.bounds.min_y) else {
            return u64::MAX;
        };
        return history_cost_in_local_rect(
            grid,
            local_min_x,
            local_max_x,
            local_min_y,
            local_max_y,
        )
        .unwrap_or(u64::MAX);
    }

    let mut total = 0u64;
    let Some(history) = grid.history.as_ref() else {
        return 0;
    };
    for (dx, dy) in footprint.iter().copied() {
        let Some(x) = origin_x.checked_add(dx) else {
            return u64::MAX;
        };
        let Some(y) = origin_y.checked_add(dy) else {
            return u64::MAX;
        };
        let Some(idx) = grid.idx_of(x, y) else {
            return u64::MAX;
        };
        total = total.saturating_add(u64::from(history[idx]));
    }
    total
}

/// A primitive footprint's total congestion cost at one origin, the
/// congestion-layer twin of `primitive_footprint_history_with_profile`.
/// Moved from `DenseRoutingGrid::primitive_footprint_congestion_with_profile`
/// (Milestone 3, Slice 3); no behaviour change.
#[inline]
pub(crate) fn primitive_footprint_congestion_with_profile(
    grid: &DenseRoutingGrid,
    origin_x: i32,
    origin_y: i32,
    footprint: &[(i32, i32)],
    profile: &FootprintCollisionProfile,
) -> u64 {
    if profile.is_full_rect {
        let Some(rect_min_x) = origin_x.checked_add(profile.min_dx) else {
            return u64::MAX;
        };
        let Some(rect_max_x) = origin_x.checked_add(profile.max_dx) else {
            return u64::MAX;
        };
        let Some(rect_min_y) = origin_y.checked_add(profile.min_dy) else {
            return u64::MAX;
        };
        let Some(rect_max_y) = origin_y.checked_add(profile.max_dy) else {
            return u64::MAX;
        };
        let Some(local_min_x) = rect_min_x.checked_sub(grid.bounds.min_x) else {
            return u64::MAX;
        };
        let Some(local_max_x) = rect_max_x.checked_sub(grid.bounds.min_x) else {
            return u64::MAX;
        };
        let Some(local_min_y) = rect_min_y.checked_sub(grid.bounds.min_y) else {
            return u64::MAX;
        };
        let Some(local_max_y) = rect_max_y.checked_sub(grid.bounds.min_y) else {
            return u64::MAX;
        };
        return congestion_cost_in_local_rect(
            grid,
            local_min_x,
            local_max_x,
            local_min_y,
            local_max_y,
        )
        .unwrap_or(u64::MAX);
    }

    let mut total = 0u64;
    let Some(congestion) = grid.congestion.as_ref() else {
        return 0;
    };
    for (dx, dy) in footprint.iter().copied() {
        let Some(x) = origin_x.checked_add(dx) else {
            return u64::MAX;
        };
        let Some(y) = origin_y.checked_add(dy) else {
            return u64::MAX;
        };
        let Some(idx) = grid.idx_of(x, y) else {
            return u64::MAX;
        };
        total = total.saturating_add(u64::from(congestion[idx]));
    }
    total
}

/// The proactive lateral-congestion term: how many already-blocked cells
/// lie within `radius_cells` to either side (perpendicular to `angle`) of
/// every footprint cell, deduplicated. Moved from
/// `DenseRoutingGrid::primitive_lateral_congestion` (Milestone 3, Slice
/// 3); no behaviour change.
#[inline]
pub(crate) fn primitive_lateral_congestion(
    grid: &DenseRoutingGrid,
    origin_x: i32,
    origin_y: i32,
    footprint: &[(i32, i32)],
    angle: u8,
    radius_cells: i32,
) -> u32 {
    if radius_cells <= 0 || footprint.is_empty() {
        return 0;
    }

    let side_a = DIRECTIONS[((angle + 2) % 8) as usize];
    let side_b = DIRECTIONS[((angle + 6) % 8) as usize];
    let mut seen = FxHashSet::default();
    let mut count = 0u32;
    for (dx, dy) in footprint.iter().copied() {
        let Some(base_x) = origin_x.checked_add(dx) else {
            continue;
        };
        let Some(base_y) = origin_y.checked_add(dy) else {
            continue;
        };
        for distance in 1..=radius_cells {
            for side in [side_a, side_b] {
                let Some(x) = base_x.checked_add(side.0.saturating_mul(distance)) else {
                    continue;
                };
                let Some(y) = base_y.checked_add(side.1.saturating_mul(distance)) else {
                    continue;
                };
                if !grid.contains(x, y) {
                    continue;
                }
                let key = pack_xy(x, y);
                if seen.insert(key) && grid.is_blocked(x, y) {
                    count = count.saturating_add(1);
                }
            }
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obstacle_map::ObstacleMap;
    use crate::primitives::PrimitiveGeometry;
    use crate::search::astar::window::RoutingBounds;

    fn whole_map_bounds(map: &ObstacleMap) -> RoutingBounds {
        RoutingBounds {
            min_x: 0,
            max_x: map.width() - 1,
            min_y: 0,
            max_y: map.height() - 1,
        }
    }

    fn straight_primitive(length_um: f64, bend_cost: f64) -> Primitive {
        Primitive {
            id: 1,
            start_angle: 0,
            end_angle: 0,
            dx: 2,
            dy: 0,
            footprint: vec![(0, 0), (1, 0)],
            length_um,
            bend_cost,
            geometry: PrimitiveGeometry::Straight { length_um },
        }
    }

    #[test]
    fn length_cost_returns_the_primitive_length_unweighted() {
        let primitive = straight_primitive(12.5, 0.0);
        assert_eq!(length_cost(&primitive), 12.5);
    }

    #[test]
    fn bend_cost_scales_the_primitive_bend_cost_by_the_configured_weight() {
        let primitive = straight_primitive(12.5, 4.0);
        // Hand computed: bend_weight(2.5) * primitive.bend_cost(4.0) = 10.0.
        assert_eq!(bend_cost(&primitive, 2.5), 10.0);
        // from_primitive folds length_cost + bend_cost, in that order.
        let metadata = PrimitiveSearchMetadata::from_primitive(&primitive, 2.5);
        assert_eq!(metadata.base_step_cost, 12.5 + 10.0);
    }

    #[test]
    fn history_rect_sums_the_hand_placed_history_costs() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_history_cost(2, 2, 3);
        map.add_history_cost(3, 2, 5);
        let bounds = whole_map_bounds(&map);
        let grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, None, 1_000, false, true, false)
                .expect("grid");

        // Hand computed: (2,2)=3 + (3,2)=5 = 8.
        assert_eq!(
            history_cost_in_local_rect(&grid, 2, 3, 2, 2),
            Some(8),
            "local rect sum over the two history-bearing cells"
        );

        let footprint = [(0, 0), (1, 0)];
        let profile = FootprintCollisionProfile::from_footprint(&footprint);
        assert!(profile.is_full_rect, "a contiguous 1x2 run is a full rect");
        assert_eq!(
            primitive_footprint_history_with_profile(&grid, 2, 2, &footprint, &profile),
            8
        );
    }

    #[test]
    fn congestion_rect_sums_the_hand_placed_congestion_costs() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_congestion_cost(2, 2, 4);
        map.add_congestion_cost(3, 2, 6);
        let bounds = whole_map_bounds(&map);
        let grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, None, 1_000, false, false, true)
                .expect("grid");

        // Hand computed: (2,2)=4 + (3,2)=6 = 10.
        assert_eq!(
            congestion_cost_in_local_rect(&grid, 2, 3, 2, 2),
            Some(10),
            "local rect sum over the two congestion-bearing cells"
        );

        let footprint = [(0, 0), (1, 0)];
        let profile = FootprintCollisionProfile::from_footprint(&footprint);
        assert_eq!(
            primitive_footprint_congestion_with_profile(&grid, 2, 2, &footprint, &profile),
            10
        );
    }

    #[test]
    fn step_cost_long_straight_congestion_term_applies_the_configured_weight() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_congestion_cost(2, 2, 4);
        map.add_congestion_cost(3, 2, 6);
        let bounds = whole_map_bounds(&map);
        let grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, None, 1_000, false, false, true)
                .expect("grid");
        let footprint = [(0, 0), (1, 0)];
        let profile = FootprintCollisionProfile::from_footprint(&footprint);
        let config = AStarConfig {
            history_weight: 0.0,
            long_straight_congestion_weight: 0.5,
            proactive_congestion_weight: 0.0,
            proactive_congestion_radius_cells: 0,
            ..AStarConfig::default()
        };

        // Hand computed: carry(0) + base(10.0) + history(0) +
        // long_straight(congestion 10 * weight 0.5 = 5.0) + proactive(0).
        let result = step_cost(
            &grid, &config, 0.0, 10.0, 2, 2, &footprint, &profile, 0, true,
        );
        assert_eq!(result, 15.0);
    }

    #[test]
    fn step_cost_proactive_congestion_term_counts_blocked_lateral_cells() {
        let mut map = ObstacleMap::new(8, 6);
        // Angle 0 (east) travels along dx; its lateral sides are north
        // (dy=+1, DIRECTIONS[2]) and south (dy=-1, DIRECTIONS[6]).
        map.add_static_cell(2, 3); // north of (2,2), distance 1: blocked.
        let bounds = whole_map_bounds(&map);
        let grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, None, 1_000, false, false, false)
                .expect("grid");
        let footprint = [(0, 0)];
        let profile = FootprintCollisionProfile::from_footprint(&footprint);
        let config = AStarConfig {
            history_weight: 0.0,
            long_straight_congestion_weight: 0.0,
            proactive_congestion_weight: 2.5,
            proactive_congestion_radius_cells: 1,
            ..AStarConfig::default()
        };

        // Hand computed: only the north cell is blocked within radius 1,
        // south is not, so the lateral count is 1; term = 1 * 2.5 = 2.5.
        let result = step_cost(
            &grid, &config, 0.0, 0.0, 2, 2, &footprint, &profile, 0, true,
        );
        assert_eq!(result, 2.5);
        assert_eq!(
            primitive_lateral_congestion(&grid, 2, 2, &footprint, 0, 1),
            1
        );

        // A non-straight move never pays the proactive term, even with
        // the same blocked neighbour (is_straight gates the term, as the
        // inline block did with `primitive_class_is_straight`).
        let bend_result = step_cost(
            &grid, &config, 0.0, 0.0, 2, 2, &footprint, &profile, 0, false,
        );
        assert_eq!(bend_result, 0.0);
    }
}
