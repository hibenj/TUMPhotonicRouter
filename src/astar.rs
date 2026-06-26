//! Single-net stateful A* router.
//!
//! This is the first routing kernel: one source state, one target state, and
//! primitive-based photonic transitions over the existing obstacle map.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::mem::size_of;
use std::time::Instant;

use rustc_hash::FxHashSet;

use crate::obstacle_map::{pack_xy, unpack_xy, CellKey, GridRect, ObstacleMap};
use crate::primitives::{Primitive, PrimitiveGeometry, PrimitiveLibrary, DIRECTIONS};
use crate::simple_routes::{
    direction_between as simple_direction_between, expand_candidate_to_grid_points,
    try_straight_l_or_z_candidate_with_config, GridPoint, SimpleRouteCandidate, SimpleZRouteConfig,
};

/// Router search state: grid position plus 45-degree heading index.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct State {
    pub x: i32,
    pub y: i32,
    pub angle: u8,
}

impl State {
    pub fn new(x: i32, y: i32, angle: u8) -> Self {
        Self {
            x,
            y,
            angle: angle % 8,
        }
    }
}

/// Configuration for the first single-net A* router.
#[derive(Clone, Debug)]
pub struct AStarConfig {
    pub max_iterations: usize,
    pub bend_weight: f64,
    pub target_tolerance_cells: i32,
    pub require_target_angle: bool,
    pub allowed_target_angles_mask: Option<u8>,
    pub use_routing_window: bool,
    pub routing_window_min_margin_cells: i32,
    pub routing_window_scale: f64,
    pub routing_window_max_expansions: u32,
    pub routing_window_fallback_full_grid: bool,
    pub routing_window_growth: f64,
    pub max_dense_states: usize,
    pub max_dense_obstacle_cells: usize,
    pub enable_simple_routes: bool,
    pub simple_route_max_offset_cells: i32,
    pub simple_route_min_leg_len_cells: i32,
    pub ignore_dynamic_obstacles: bool,
    pub history_weight: f64,
    pub collect_detailed_timing: bool,
    pub enable_jps4: bool,
    pub use_indexed_heap: bool,
    pub primitive_ordering: PrimitiveOrdering,
    pub heuristic_mode: HeuristicMode,
    pub heap_tie_breaker: HeapTieBreaker,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100_000,
            bend_weight: 1.0,
            target_tolerance_cells: 0,
            require_target_angle: true,
            allowed_target_angles_mask: None,
            use_routing_window: true,
            routing_window_min_margin_cells: 12,
            routing_window_scale: 0.35,
            routing_window_max_expansions: 3,
            routing_window_fallback_full_grid: false,
            routing_window_growth: 0.5,
            max_dense_states: 20_000_000,
            max_dense_obstacle_cells: 10_000_000,
            enable_simple_routes: true,
            simple_route_max_offset_cells: 96,
            simple_route_min_leg_len_cells: 1,
            ignore_dynamic_obstacles: false,
            history_weight: 0.0,
            collect_detailed_timing: false,
            enable_jps4: false,
            use_indexed_heap: false,
            primitive_ordering: PrimitiveOrdering::Library,
            heuristic_mode: HeuristicMode::HeadingAware,
            heap_tie_breaker: HeapTieBreaker::SmallerG,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum PrimitiveOrdering {
    #[default]
    Library,
    LongStraightFirst,
    TargetBiased,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum HeuristicMode {
    #[default]
    Distance,
    HeadingAware,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum HeapTieBreaker {
    #[default]
    SmallerG,
    LargerG,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RoutingBounds {
    min_x: i32,
    max_x: i32,
    min_y: i32,
    max_y: i32,
}

impl RoutingBounds {
    #[inline]
    fn contains(&self, x: i32, y: i32) -> bool {
        x >= self.min_x && x <= self.max_x && y >= self.min_y && y <= self.max_y
    }
}

/// Result of one successful single-net route.
#[derive(Clone, Debug)]
pub struct RouteResult {
    pub states: Vec<State>,
    pub primitives: Vec<u16>,
    pub cells: Vec<(i32, i32)>,
    pub compressed_waypoints: Vec<(i32, i32)>,
    pub total_length_um: f64,
    pub total_cost: f64,
    pub requested_target: State,
    pub reached_target: State,
    pub stats: RouteSearchStats,
}

const PRIMITIVE_TRANSITION_CLASS_COUNT: usize = 4;
const PRIMITIVE_STRAIGHT_SHORT: usize = 0;
const PRIMITIVE_STRAIGHT_LONG: usize = 1;
const PRIMITIVE_BEND_45: usize = 2;
const PRIMITIVE_BEND_90: usize = 3;

#[derive(Clone, Debug, Default)]
pub struct RouteSearchStats {
    pub window_attempts: u32,
    pub used_full_grid_fallback: bool,
    pub last_window_min_x: i32,
    pub last_window_max_x: i32,
    pub last_window_min_y: i32,
    pub last_window_max_y: i32,
    pub last_window_area_cells: i64,
    pub expanded_states: usize,
    pub generated_neighbors: usize,
    pub heap_pushes: usize,
    pub heap_pops: usize,
    pub skipped_duplicate_heap_entries: usize,
    pub stale_generation_heap_entries: usize,
    pub closed_heap_entries: usize,
    pub max_heap_size: usize,
    pub dense_search_states: usize,
    pub dense_search_storage_bytes: usize,
    pub best_cost_updates: usize,
    pub parent_updates: usize,
    pub obstacle_clearance_checks: usize,
    pub window_rejects: usize,
    pub footprint_rejects: usize,
    pub primitive_generated_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_bounds_rejects_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_closed_rejects_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_cost_pruned_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_footprint_checks_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_footprint_rejects_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_accepted_by_class: [usize; PRIMITIVE_TRANSITION_CLASS_COUNT],
    pub primitive_footprint_checks: usize,
    pub primitive_footprint_cells_tested: usize,
    pub primitive_footprint_rect_checks: usize,
    pub primitive_footprint_rect_rejects: usize,
    pub dense_grid_build_failures: usize,
    pub max_window_area_cells: i64,
    pub dense_grid_cells: usize,
    pub dense_grid_build_time_us: u128,
    pub neighbor_generation_time_us: u128,
    pub heap_operation_time_us: u128,
    pub legality_check_time_us: u128,
    pub reconstruction_time_us: u128,
    pub jps4_requested: bool,
    pub jps4_eligible: bool,
    pub jps4_used: bool,
    pub jps4_fallbacks: usize,
    pub jps4_fallback_reason: String,
}

#[derive(Clone, Debug)]
struct Jps4Eligibility {
    eligible: bool,
    reason: &'static str,
}

#[derive(Clone, Copy, Debug)]
struct FootprintCollisionProfile {
    is_full_rect: bool,
    min_dx: i32,
    max_dx: i32,
    min_dy: i32,
    max_dy: i32,
    cell_count: usize,
}

impl FootprintCollisionProfile {
    #[inline]
    fn from_footprint(footprint: &[(i32, i32)]) -> Self {
        let cell_count = footprint.len();
        if footprint.is_empty() {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
            };
        }

        let mut min_dx = i32::MAX;
        let mut max_dx = i32::MIN;
        let mut min_dy = i32::MAX;
        let mut max_dy = i32::MIN;
        for &(dx, dy) in footprint {
            min_dx = min_dx.min(dx);
            max_dx = max_dx.max(dx);
            min_dy = min_dy.min(dy);
            max_dy = max_dy.max(dy);
        }

        let width = max_dx.checked_sub(min_dx).and_then(|v| v.checked_add(1));
        let height = max_dy.checked_sub(min_dy).and_then(|v| v.checked_add(1));
        let (Some(width_usize), Some(height_usize)) = (
            width.and_then(|v| usize::try_from(v).ok()),
            height.and_then(|v| usize::try_from(v).ok()),
        ) else {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
            };
        };

        let area = width_usize.checked_mul(height_usize);
        if area != Some(cell_count) {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
            };
        }

        let mut sorted_footprint = footprint.to_vec();
        sorted_footprint.sort_unstable_by(|(a_x, a_y), (b_x, b_y)| a_y.cmp(b_y).then(a_x.cmp(b_x)));
        let mut idx = 0usize;
        for y in min_dy..=max_dy {
            for x in min_dx..=max_dx {
                if idx >= sorted_footprint.len() || sorted_footprint[idx] != (x, y) {
                    return Self {
                        is_full_rect: false,
                        min_dx: 0,
                        max_dx: -1,
                        min_dy: 0,
                        max_dy: -1,
                        cell_count,
                    };
                }
                idx += 1;
            }
        }

        Self {
            is_full_rect: true,
            min_dx,
            max_dx,
            min_dy,
            max_dy,
            cell_count,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct PrimitiveSearchMetadata {
    transition_class: usize,
    base_step_cost: f64,
}

impl PrimitiveSearchMetadata {
    fn from_primitive(primitive: &Primitive, bend_weight: f64) -> Self {
        Self {
            transition_class: primitive_transition_class(
                &primitive.geometry,
                primitive.dx,
                primitive.dy,
            ),
            base_step_cost: primitive.length_um + bend_weight * primitive.bend_cost,
        }
    }
}

const NO_PARENT: u32 = u32::MAX;
const NO_GENERATION: u32 = u32::MAX;
const BITSET_WORD_BITS: usize = u64::BITS as usize;

struct DenseSearchStorage {
    bounds: RoutingBounds,
    width_usize: usize,
    g_costs: Vec<f64>,
    best_generation: Vec<u32>,
    parent_idx: Vec<u32>,
    parent_primitive: Vec<u16>,
    closed: DenseBitset,
}

impl DenseSearchStorage {
    fn new(bounds: RoutingBounds, max_dense_states: usize) -> Option<Self> {
        let width = bounds.max_x.checked_sub(bounds.min_x)?.checked_add(1)?;
        let height = bounds.max_y.checked_sub(bounds.min_y)?.checked_add(1)?;
        if width <= 0 || height <= 0 {
            return None;
        }
        let width_usize = usize::try_from(width).ok()?;
        let height_usize = usize::try_from(height).ok()?;
        let state_count = width_usize.checked_mul(height_usize)?.checked_mul(8)?;
        if state_count > u32::MAX as usize {
            return None;
        }
        if state_count > max_dense_states {
            return None;
        }

        Some(Self {
            bounds,
            width_usize,
            g_costs: vec![f64::INFINITY; state_count],
            best_generation: vec![NO_GENERATION; state_count],
            parent_idx: vec![NO_PARENT; state_count],
            parent_primitive: vec![0; state_count],
            closed: DenseBitset::new(state_count)?,
        })
    }

    fn state_count(&self) -> usize {
        self.g_costs.len()
    }

    fn allocated_bytes(&self) -> usize {
        self.g_costs.len() * size_of::<f64>()
            + self.best_generation.len() * size_of::<u32>()
            + self.parent_idx.len() * size_of::<u32>()
            + self.parent_primitive.len() * size_of::<u16>()
            + self.closed.allocated_bytes()
    }

    fn state_to_idx(&self, state: State) -> Option<usize> {
        if state.angle >= 8 || !self.bounds.contains(state.x, state.y) {
            return None;
        }
        Some(self.in_bounds_state_to_idx(state))
    }

    #[inline]
    fn in_bounds_state_to_idx(&self, state: State) -> usize {
        self.in_bounds_parts_to_idx(state.x, state.y, state.angle)
    }

    #[inline]
    fn in_bounds_parts_to_idx(&self, x: i32, y: i32, angle: u8) -> usize {
        debug_assert!(angle < 8);
        debug_assert!(self.bounds.contains(x, y));
        let local_x = (x - self.bounds.min_x) as usize;
        let local_y = (y - self.bounds.min_y) as usize;
        ((local_y * self.width_usize) + local_x) << 3 | usize::from(angle)
    }

    #[inline]
    fn idx_to_state(&self, idx: usize) -> State {
        debug_assert!(idx < self.state_count());
        let angle = (idx & 7) as u8;
        let cell_idx = idx >> 3;
        let local_x = (cell_idx % self.width_usize) as i32;
        let local_y = (cell_idx / self.width_usize) as i32;
        State {
            x: self.bounds.min_x + local_x,
            y: self.bounds.min_y + local_y,
            angle,
        }
    }
}

struct DenseBitset {
    bits: Vec<u64>,
}

impl DenseBitset {
    fn new(cell_count: usize) -> Option<Self> {
        let words = cell_count
            .checked_add(BITSET_WORD_BITS - 1)?
            .checked_div(BITSET_WORD_BITS)?;
        Some(Self {
            bits: vec![0; words],
        })
    }

    #[inline]
    fn set(&mut self, idx: usize) -> Option<()> {
        let word_idx = idx.checked_div(BITSET_WORD_BITS)?;
        let bit_idx = idx.checked_rem(BITSET_WORD_BITS)?;
        let word = self.bits.get_mut(word_idx)?;
        *word |= 1u64 << bit_idx;
        Some(())
    }

    #[inline]
    fn get(&self, idx: usize) -> bool {
        let word_idx = idx / BITSET_WORD_BITS;
        let bit_idx = idx % BITSET_WORD_BITS;
        self.bits
            .get(word_idx)
            .map(|word| (word & (1u64 << bit_idx)) != 0)
            .unwrap_or(true)
    }

    fn allocated_bytes(&self) -> usize {
        self.bits.len() * size_of::<u64>()
    }
}

struct DenseRoutingGrid {
    bounds: RoutingBounds,
    width: i32,
    height: i32,
    blocked_bits: DenseBitset,
    blocked_prefix: Option<Vec<u32>>,
    history: Option<Vec<u32>>,
    history_prefix: Option<Vec<u64>>,
    blocked_count: usize,
    build_time_us: u128,
}

fn intersect_bounds_rect(bounds: RoutingBounds, rect: GridRect) -> Option<GridRect> {
    let x_min = bounds.min_x.max(rect.x_min);
    let y_min = bounds.min_y.max(rect.y_min);
    let x_max = bounds.max_x.min(rect.x_max);
    let y_max = bounds.max_y.min(rect.y_max);
    (x_min <= x_max && y_min <= y_max).then_some(GridRect {
        x_min,
        y_min,
        x_max,
        y_max,
    })
}

impl DenseRoutingGrid {
    fn from_obstacle_map(
        obstacle_map: &ObstacleMap,
        bounds: RoutingBounds,
        opened_cells: Option<&FxHashSet<CellKey>>,
        max_dense_obstacle_cells: usize,
        ignore_dynamic_obstacles: bool,
        build_history: bool,
    ) -> Option<Self> {
        let start = Instant::now();
        let width = bounds.max_x.checked_sub(bounds.min_x)?.checked_add(1)?;
        let height = bounds.max_y.checked_sub(bounds.min_y)?.checked_add(1)?;
        if width <= 0 || height <= 0 {
            return None;
        }

        let width_usize = usize::try_from(width).ok()?;
        let height_usize = usize::try_from(height).ok()?;
        let cell_count = width_usize.checked_mul(height_usize)?;
        if cell_count > max_dense_obstacle_cells {
            return None;
        }

        let mut blocked_cells = vec![0u8; cell_count];
        let mut blocked_bits = DenseBitset::new(cell_count)?;
        let mut history = if build_history {
            Some(vec![0u32; cell_count])
        } else {
            None
        };
        let mut blocked_count = 0usize;

        let local_idx = |x: i32, y: i32| -> Option<usize> {
            let local_x = x.checked_sub(bounds.min_x)?;
            let local_y = y.checked_sub(bounds.min_y)?;
            if local_x < 0 || local_x >= width || local_y < 0 || local_y >= height {
                return None;
            }
            usize::try_from(local_y)
                .ok()?
                .checked_mul(width_usize)?
                .checked_add(usize::try_from(local_x).ok()?)
        };

        let opened_contains = |x: i32, y: i32| -> bool {
            opened_cells
                .map(|cells| cells.contains(&pack_xy(x, y)))
                .unwrap_or(false)
        };

        let mark_blocked = |idx: usize,
                            blocked_cells: &mut [u8],
                            blocked_bits: &mut DenseBitset,
                            blocked_count: &mut usize|
         -> Option<()> {
            if blocked_cells[idx] == 0 {
                blocked_cells[idx] = 1;
                blocked_bits.set(idx)?;
                *blocked_count += 1;
            }
            Some(())
        };

        for rect in obstacle_map.static_rects() {
            let Some(rect) = intersect_bounds_rect(bounds, *rect) else {
                continue;
            };
            for y in rect.y_min..=rect.y_max {
                for x in rect.x_min..=rect.x_max {
                    if opened_contains(x, y) {
                        continue;
                    }
                    let idx = local_idx(x, y)?;
                    mark_blocked(
                        idx,
                        &mut blocked_cells,
                        &mut blocked_bits,
                        &mut blocked_count,
                    )?;
                }
            }
        }

        for key in obstacle_map.static_obstacle_keys() {
            let (x, y) = unpack_xy(key);
            if opened_contains(x, y) {
                continue;
            }
            let Some(idx) = local_idx(x, y) else {
                continue;
            };
            mark_blocked(
                idx,
                &mut blocked_cells,
                &mut blocked_bits,
                &mut blocked_count,
            )?;
        }

        if !ignore_dynamic_obstacles {
            for key in obstacle_map.dynamic_obstacle_keys() {
                let (x, y) = unpack_xy(key);
                let Some(idx) = local_idx(x, y) else {
                    continue;
                };
                mark_blocked(
                    idx,
                    &mut blocked_cells,
                    &mut blocked_bits,
                    &mut blocked_count,
                )?;
            }
        }

        if let Some(history) = history.as_mut() {
            for (key, cost) in obstacle_map.history_entries() {
                let (x, y) = unpack_xy(key);
                let Some(idx) = local_idx(x, y) else {
                    continue;
                };
                history[idx] = cost;
            }
        }

        let stride = usize::try_from(width).ok()?.checked_add(1)?;
        let mut blocked_prefix =
            vec![0u32; stride.checked_mul(usize::try_from(height).ok()?.checked_add(1)?)?];
        let mut history_prefix = if build_history {
            Some(vec![
                0u64;
                stride.checked_mul(
                    usize::try_from(height).ok()?.checked_add(1)?
                )?
            ])
        } else {
            None
        };
        for local_y in 0..height {
            let mut row_sum = 0u32;
            let mut history_row_sum = 0u64;
            let y_base = usize::try_from(local_y).ok()?;
            let src_base = y_base.checked_mul(width_usize)?;
            let prefix_row = (y_base + 1).checked_mul(stride)?;
            let prefix_above = y_base.checked_mul(stride)?;
            for local_x in 0..width {
                let x_idx = usize::try_from(local_x).ok()?;
                let blocked_idx = src_base.checked_add(x_idx)?;
                row_sum = row_sum.saturating_add(u32::from(blocked_cells[blocked_idx]));
                if let Some(history) = history.as_ref() {
                    history_row_sum =
                        history_row_sum.saturating_add(u64::from(history[blocked_idx]));
                }
                let prefix_idx = prefix_row.checked_add(x_idx + 1)?;
                let above = blocked_prefix[prefix_above + x_idx + 1];
                blocked_prefix[prefix_idx] = above.saturating_add(row_sum);
                if let Some(history_prefix) = history_prefix.as_mut() {
                    let history_above = history_prefix[prefix_above + x_idx + 1];
                    history_prefix[prefix_idx] = history_above.saturating_add(history_row_sum);
                }
            }
        }

        Some(Self {
            bounds,
            width,
            height,
            blocked_bits,
            blocked_prefix: Some(blocked_prefix),
            history,
            history_prefix,
            blocked_count,
            build_time_us: start.elapsed().as_micros(),
        })
    }

    #[inline]
    fn contains(&self, x: i32, y: i32) -> bool {
        self.bounds.contains(x, y)
    }

    #[inline]
    fn is_blocked(&self, x: i32, y: i32) -> bool {
        match self.idx_of(x, y) {
            Some(idx) => self.blocked_bits.get(idx),
            None => true,
        }
    }

    #[inline]
    fn blocked_count(&self) -> usize {
        self.blocked_count
    }

    #[inline]
    fn build_time_us(&self) -> u128 {
        self.build_time_us
    }

    #[inline]
    fn blocked_count_in_local_rect(
        &self,
        local_min_x: i32,
        local_max_x: i32,
        local_min_y: i32,
        local_max_y: i32,
    ) -> Option<u32> {
        let width = self.width;
        if local_min_x > local_max_x || local_min_y > local_max_y {
            return None;
        }
        if local_min_x < 0 || local_min_y < 0 || local_max_x >= width || local_max_y >= self.height
        {
            return None;
        }

        let prefix = self.blocked_prefix.as_ref()?;
        let width_usize = usize::try_from(width).ok()?;
        let stride = width_usize.checked_add(1)?;
        let x1 = usize::try_from(local_min_x).ok()?;
        let y1 = usize::try_from(local_min_y).ok()?;
        let x2 = usize::try_from(local_max_x).ok()?;
        let y2 = usize::try_from(local_max_y).ok()?;

        let a = i64::from(prefix[(y2 + 1).checked_mul(stride)? + (x2 + 1)]);
        let b = i64::from(prefix[y1.checked_mul(stride)? + (x2 + 1)]);
        let c = i64::from(prefix[(y2 + 1).checked_mul(stride)? + x1]);
        let d = i64::from(prefix[y1.checked_mul(stride)? + x1]);
        let total = a + d - b - c;
        if total < 0 {
            Some(0)
        } else {
            u32::try_from(total).ok()
        }
    }

    #[inline]
    fn blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Option<u32> {
        let local_min_x = min_x.checked_sub(self.bounds.min_x)?;
        let local_max_x = max_x.checked_sub(self.bounds.min_x)?;
        let local_min_y = min_y.checked_sub(self.bounds.min_y)?;
        let local_max_y = max_y.checked_sub(self.bounds.min_y)?;
        self.blocked_count_in_local_rect(local_min_x, local_max_x, local_min_y, local_max_y)
    }

    #[inline]
    fn rect_free(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> bool {
        matches!(
            self.blocked_count_in_rect(min_x, max_x, min_y, max_y),
            Some(0)
        )
    }

    #[inline]
    fn horizontal_segment_free(&self, y: i32, x0: i32, x1: i32) -> bool {
        let min_x = x0.min(x1);
        let max_x = x0.max(x1);
        self.rect_free(min_x, max_x, y, y)
    }

    #[inline]
    fn vertical_segment_free(&self, x: i32, y0: i32, y1: i32) -> bool {
        let min_y = y0.min(y1);
        let max_y = y0.max(y1);
        self.rect_free(x, x, min_y, max_y)
    }

    #[inline]
    fn history_cost_in_local_rect(
        &self,
        local_min_x: i32,
        local_max_x: i32,
        local_min_y: i32,
        local_max_y: i32,
    ) -> Option<u64> {
        let width = self.width;
        if local_min_x > local_max_x || local_min_y > local_max_y {
            return None;
        }
        if local_min_x < 0 || local_min_y < 0 || local_max_x >= width || local_max_y >= self.height
        {
            return None;
        }

        let prefix = self.history_prefix.as_ref()?;
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

    #[inline]
    fn primitive_footprint_free_with_profile(
        &self,
        origin_x: i32,
        origin_y: i32,
        footprint: &[(i32, i32)],
        profile: &FootprintCollisionProfile,
        stats: &mut RouteSearchStats,
    ) -> bool {
        if profile.is_full_rect {
            let rect_min_x = match origin_x.checked_add(profile.min_dx) {
                Some(x) => x,
                None => return false,
            };
            let rect_max_x = match origin_x.checked_add(profile.max_dx) {
                Some(x) => x,
                None => return false,
            };
            let rect_min_y = match origin_y.checked_add(profile.min_dy) {
                Some(y) => y,
                None => return false,
            };
            let rect_max_y = match origin_y.checked_add(profile.max_dy) {
                Some(y) => y,
                None => return false,
            };
            stats.primitive_footprint_rect_checks += 1;
            stats.primitive_footprint_cells_tested += profile.cell_count;
            if rect_min_y == rect_max_y {
                self.horizontal_segment_free(rect_min_y, rect_min_x, rect_max_x)
            } else if rect_min_x == rect_max_x {
                self.vertical_segment_free(rect_min_x, rect_min_y, rect_max_y)
            } else {
                self.rect_free(rect_min_x, rect_max_x, rect_min_y, rect_max_y)
            }
        } else {
            let tested_cells = footprint.len();
            stats.primitive_footprint_cells_tested += tested_cells;
            for (dx, dy) in footprint.iter().copied() {
                let x = match origin_x.checked_add(dx) {
                    Some(x) => x,
                    None => return false,
                };
                let y = match origin_y.checked_add(dy) {
                    Some(y) => y,
                    None => return false,
                };
                if self.is_blocked(x, y) {
                    return false;
                }
            }
            true
        }
    }

    #[inline]
    fn primitive_footprint_history_with_profile(
        &self,
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
            let Some(local_min_x) = rect_min_x.checked_sub(self.bounds.min_x) else {
                return u64::MAX;
            };
            let Some(local_max_x) = rect_max_x.checked_sub(self.bounds.min_x) else {
                return u64::MAX;
            };
            let Some(local_min_y) = rect_min_y.checked_sub(self.bounds.min_y) else {
                return u64::MAX;
            };
            let Some(local_max_y) = rect_max_y.checked_sub(self.bounds.min_y) else {
                return u64::MAX;
            };
            return self
                .history_cost_in_local_rect(local_min_x, local_max_x, local_min_y, local_max_y)
                .unwrap_or(u64::MAX);
        }

        let mut total = 0u64;
        let Some(history) = self.history.as_ref() else {
            return 0;
        };
        for (dx, dy) in footprint.iter().copied() {
            let Some(x) = origin_x.checked_add(dx) else {
                return u64::MAX;
            };
            let Some(y) = origin_y.checked_add(dy) else {
                return u64::MAX;
            };
            let Some(idx) = self.idx_of(x, y) else {
                return u64::MAX;
            };
            total = total.saturating_add(u64::from(history[idx]));
        }
        total
    }

    #[cfg(test)]
    #[inline]
    fn primitive_footprint_free(
        &self,
        origin_x: i32,
        origin_y: i32,
        footprint: &[(i32, i32)],
        stats: &mut RouteSearchStats,
    ) -> bool {
        for (dx, dy) in footprint.iter().copied() {
            let x = match origin_x.checked_add(dx) {
                Some(x) => x,
                None => return false,
            };
            let y = match origin_y.checked_add(dy) {
                Some(y) => y,
                None => return false,
            };
            if self.is_blocked(x, y) {
                return false;
            }
        }
        stats.primitive_footprint_cells_tested += footprint.len();
        true
    }

    #[inline]
    fn idx_of(&self, x: i32, y: i32) -> Option<usize> {
        if !self.contains(x, y) {
            return None;
        }
        let local_x = usize::try_from(x.checked_sub(self.bounds.min_x)?).ok()?;
        let local_y = usize::try_from(y.checked_sub(self.bounds.min_y)?).ok()?;
        let width = usize::try_from(self.width).ok()?;
        local_y.checked_mul(width)?.checked_add(local_x)
    }
}

#[derive(Clone, Copy, Debug)]
struct OpenEntry {
    f_score: f64,
    tie_score: f64,
    g_score: f64,
    counter: u32,
    generation: u32,
    idx: usize,
}

impl Eq for OpenEntry {}

impl PartialEq for OpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score && self.counter == other.counter
    }
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                other
                    .tie_score
                    .partial_cmp(&self.tie_score)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

const NO_HEAP_POSITION: usize = usize::MAX;

struct IndexedOpenSet {
    heap: Vec<OpenEntry>,
    positions: Vec<usize>,
}

impl IndexedOpenSet {
    fn new(state_count: usize) -> Self {
        Self {
            heap: Vec::new(),
            positions: vec![NO_HEAP_POSITION; state_count],
        }
    }

    fn len(&self) -> usize {
        self.heap.len()
    }

    fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    fn push_or_decrease(&mut self, entry: OpenEntry) -> bool {
        let Some(position) = self.positions.get(entry.idx).copied() else {
            return false;
        };
        if position == NO_HEAP_POSITION {
            self.heap.push(entry);
            let new_position = self.heap.len() - 1;
            self.positions[entry.idx] = new_position;
            self.sift_up(new_position);
            return true;
        }

        if !entry_is_better(&entry, &self.heap[position]) {
            return false;
        }
        self.heap[position] = entry;
        self.sift_up(position);
        true
    }

    fn pop(&mut self) -> Option<OpenEntry> {
        if self.is_empty() {
            return None;
        }
        let popped = self.heap.swap_remove(0);
        self.positions[popped.idx] = NO_HEAP_POSITION;
        if !self.heap.is_empty() {
            self.positions[self.heap[0].idx] = 0;
            self.sift_down(0);
        }
        Some(popped)
    }

    fn sift_up(&mut self, mut position: usize) {
        while position > 0 {
            let parent = (position - 1) / 2;
            if !entry_is_better(&self.heap[position], &self.heap[parent]) {
                break;
            }
            self.swap_positions(position, parent);
            position = parent;
        }
    }

    fn sift_down(&mut self, mut position: usize) {
        loop {
            let left = position * 2 + 1;
            let right = left + 1;
            let mut best = position;
            if left < self.heap.len() && entry_is_better(&self.heap[left], &self.heap[best]) {
                best = left;
            }
            if right < self.heap.len() && entry_is_better(&self.heap[right], &self.heap[best]) {
                best = right;
            }
            if best == position {
                break;
            }
            self.swap_positions(position, best);
            position = best;
        }
    }

    fn swap_positions(&mut self, a: usize, b: usize) {
        self.heap.swap(a, b);
        self.positions[self.heap[a].idx] = a;
        self.positions[self.heap[b].idx] = b;
    }
}

fn entry_is_better(candidate: &OpenEntry, current: &OpenEntry) -> bool {
    candidate.cmp(current) == Ordering::Greater
}

#[inline]
fn heap_tie_score(g_score: f64, tie_breaker: HeapTieBreaker) -> f64 {
    match tie_breaker {
        HeapTieBreaker::SmallerG => g_score,
        HeapTieBreaker::LargerG => -g_score,
    }
}

fn next_search_generation(counter: &mut u32) -> Option<u32> {
    if *counter == NO_GENERATION {
        return None;
    }
    let current = *counter;
    *counter = counter.checked_add(1)?;
    Some(current)
}

enum OpenSet {
    Duplicate(BinaryHeap<OpenEntry>),
    Indexed(IndexedOpenSet),
}

impl OpenSet {
    fn new(use_indexed_heap: bool, state_count: usize) -> Self {
        if use_indexed_heap {
            Self::Indexed(IndexedOpenSet::new(state_count))
        } else {
            Self::Duplicate(BinaryHeap::new())
        }
    }

    fn push(&mut self, entry: OpenEntry) -> bool {
        match self {
            Self::Duplicate(heap) => {
                heap.push(entry);
                true
            }
            Self::Indexed(heap) => heap.push_or_decrease(entry),
        }
    }

    fn pop(&mut self) -> Option<OpenEntry> {
        match self {
            Self::Duplicate(heap) => heap.pop(),
            Self::Indexed(heap) => heap.pop(),
        }
    }

    fn len(&self) -> usize {
        match self {
            Self::Duplicate(heap) => heap.len(),
            Self::Indexed(heap) => heap.len(),
        }
    }
}

/// Route a single net with default A* settings.
pub fn route_single_net(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
) -> Option<RouteResult> {
    route_single_net_with_config(
        obstacle_map,
        primitives,
        source,
        target,
        port_open_cells,
        &AStarConfig::default(),
    )
}

/// Route a single net with explicit A* settings.
pub fn try_simple_route_with_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
) -> Option<RouteResult> {
    if !config.enable_simple_routes {
        return None;
    }
    if config.target_tolerance_cells < 0 {
        return None;
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        if mask == 0 {
            return None;
        }
    }
    if target.angle > 7 {
        return None;
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        return None;
    }

    let mut anchor_open_cells = FxHashSet::default();
    if let Some(port_open_cells) = port_open_cells {
        anchor_open_cells.extend(port_open_cells.iter().copied());
    }
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));

    let bend_radius_cells = infer_bend_radius_cells(primitives).unwrap_or(0);
    let z_config = SimpleZRouteConfig {
        max_offset_cells: config.simple_route_max_offset_cells,
        include_zero_offset: true,
        min_leg_len_cells: config.simple_route_min_leg_len_cells.max(bend_radius_cells),
    };
    let candidate = try_straight_l_or_z_candidate_with_config(
        source,
        target,
        obstacle_map,
        Some(&anchor_open_cells),
        &z_config,
    )?;
    simple_candidate_to_route_result(
        &candidate,
        source,
        target,
        primitives,
        RouteSearchStats::default(),
    )
}

/// Route a single net with explicit A* settings.
pub fn route_single_net_with_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
) -> Option<RouteResult> {
    if config.target_tolerance_cells < 0 {
        return None;
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        if mask == 0 {
            return None;
        }
    }
    if target.angle > 7 {
        return None;
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        return None;
    }
    let mut anchor_open_cells = FxHashSet::default();
    if let Some(port_open_cells) = port_open_cells {
        anchor_open_cells.extend(port_open_cells.iter().copied());
    }
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));

    let mut stats = RouteSearchStats::default();
    let jps4_eligibility = evaluate_jps4_eligibility(primitives, source, target, config);
    stats.jps4_requested = config.enable_jps4;
    stats.jps4_eligible = jps4_eligibility.eligible;
    stats.jps4_fallback_reason = jps4_eligibility.reason.to_string();
    if config.enable_jps4 && jps4_eligibility.eligible {
        if let Some(route) = route_single_net_jps4(
            obstacle_map,
            source,
            target,
            Some(&anchor_open_cells),
            config,
            primitives.grid_size_um(),
            stats.clone(),
        ) {
            return Some(route);
        }
        stats.jps4_fallbacks += 1;
        stats.jps4_fallback_reason = "jps4 search failed".to_string();
    } else if config.enable_jps4 {
        stats.jps4_fallbacks += 1;
    }
    if let Some(mut simple_route) = try_simple_route_with_config(
        obstacle_map,
        primitives,
        source,
        target,
        port_open_cells,
        config,
    ) {
        simple_route.stats = stats;
        return Some(simple_route);
    }

    if !config.use_routing_window {
        return route_single_net_with_bounds(
            obstacle_map,
            primitives,
            source,
            target,
            Some(&anchor_open_cells),
            config,
            None,
            &mut stats,
        );
    }

    let mut last_bounds: Option<RoutingBounds> = None;
    for expansion_idx in 0..=config.routing_window_max_expansions {
        let bounds = compute_routing_bounds(obstacle_map, source, target, config, expansion_idx)?;
        if last_bounds == Some(bounds) {
            continue;
        }
        last_bounds = Some(bounds);
        stats.window_attempts += 1;
        stats.max_window_area_cells = stats.max_window_area_cells.max(window_area(bounds));
        stats.last_window_min_x = bounds.min_x;
        stats.last_window_max_x = bounds.max_x;
        stats.last_window_min_y = bounds.min_y;
        stats.last_window_max_y = bounds.max_y;
        stats.last_window_area_cells = window_area(bounds);

        if let Some(route) = route_single_net_with_bounds(
            obstacle_map,
            primitives,
            source,
            target,
            Some(&anchor_open_cells),
            config,
            Some(bounds),
            &mut stats,
        ) {
            return Some(route);
        }
    }

    if config.routing_window_fallback_full_grid {
        stats.window_attempts += 1;
        stats.used_full_grid_fallback = true;
        let full_bounds = RoutingBounds {
            min_x: 0,
            max_x: obstacle_map.width() - 1,
            min_y: 0,
            max_y: obstacle_map.height() - 1,
        };
        stats.last_window_min_x = full_bounds.min_x;
        stats.last_window_max_x = full_bounds.max_x;
        stats.last_window_min_y = full_bounds.min_y;
        stats.last_window_max_y = full_bounds.max_y;
        stats.last_window_area_cells = window_area(full_bounds);
        stats.max_window_area_cells = stats.max_window_area_cells.max(window_area(full_bounds));
        return route_single_net_with_bounds(
            obstacle_map,
            primitives,
            source,
            target,
            Some(&anchor_open_cells),
            config,
            None,
            &mut stats,
        );
    }

    None
}

fn evaluate_jps4_eligibility(
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    config: &AStarConfig,
) -> Jps4Eligibility {
    if !config.enable_jps4 {
        return Jps4Eligibility {
            eligible: false,
            reason: "disabled",
        };
    }
    if config.target_tolerance_cells != 0 {
        return Jps4Eligibility {
            eligible: false,
            reason: "target tolerance is nonzero",
        };
    }
    if config.require_target_angle || config.allowed_target_angles_mask.is_some() {
        return Jps4Eligibility {
            eligible: false,
            reason: "target heading constraints are active",
        };
    }
    if config.history_weight != 0.0 {
        return Jps4Eligibility {
            eligible: false,
            reason: "history costs are active",
        };
    }
    if source.angle % 2 != 0 || target.angle % 2 != 0 {
        return Jps4Eligibility {
            eligible: false,
            reason: "source or target heading is diagonal",
        };
    }
    if !primitive_library_is_plain_jps4_grid(primitives) {
        return Jps4Eligibility {
            eligible: false,
            reason: "primitive library is not plain 4-connected unit grid",
        };
    }

    Jps4Eligibility {
        eligible: true,
        reason: "eligible",
    }
}

fn primitive_library_is_plain_jps4_grid(primitives: &PrimitiveLibrary) -> bool {
    for angle in 0..8u8 {
        let bucket = primitives.get_primitives_for_angle(angle);
        if angle % 2 != 0 {
            if !bucket.is_empty() {
                return false;
            }
            continue;
        }
        if bucket.len() != 1 {
            return false;
        }
        let primitive = &bucket[0];
        if primitive.start_angle != angle || primitive.end_angle != angle {
            return false;
        }
        if primitive.bend_cost != 0.0 {
            return false;
        }
        if primitive.dx.abs() + primitive.dy.abs() != 1 {
            return false;
        }
        if primitive.dx != 0 && primitive.dy != 0 {
            return false;
        }
    }
    true
}

const JPS4_DIRECTIONS: [(i32, i32); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];

fn route_single_net_jps4(
    obstacle_map: &ObstacleMap,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    grid_size_um: f64,
    mut stats: RouteSearchStats,
) -> Option<RouteResult> {
    let bounds = RoutingBounds {
        min_x: 0,
        max_x: obstacle_map.width() - 1,
        min_y: 0,
        max_y: obstacle_map.height() - 1,
    };
    let dense_grid = DenseRoutingGrid::from_obstacle_map(
        obstacle_map,
        bounds,
        port_open_cells,
        config.max_dense_obstacle_cells,
        config.ignore_dynamic_obstacles,
        false,
    )?;
    stats.dense_grid_cells = dense_grid.blocked_count();
    stats.dense_grid_build_time_us = dense_grid.build_time_us();
    stats.window_attempts = 1;

    if dense_grid.is_blocked(source.x, source.y) || dense_grid.is_blocked(target.x, target.y) {
        return None;
    }

    if source.x == target.x && source.y == target.y {
        stats.jps4_used = true;
        return Some(RouteResult {
            states: vec![source],
            primitives: Vec::new(),
            cells: vec![(source.x, source.y)],
            compressed_waypoints: vec![(source.x, source.y)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: target,
            reached_target: target,
            stats,
        });
    }

    let width = usize::try_from(dense_grid.width).ok()?;
    let height = usize::try_from(dense_grid.height).ok()?;
    let cell_count = width.checked_mul(height)?;
    let mut g_costs = vec![f64::INFINITY; cell_count];
    let mut parent_idx = vec![NO_PARENT; cell_count];
    let mut closed = DenseBitset::new(cell_count)?;
    let mut open_set = BinaryHeap::new();
    let mut counter = 0u32;
    stats.dense_search_states = cell_count;
    stats.dense_search_storage_bytes = g_costs.len() * size_of::<f64>()
        + parent_idx.len() * size_of::<u32>()
        + closed.allocated_bytes();

    let source_idx = dense_grid.idx_of(source.x, source.y)?;
    let target_point = (target.x, target.y);
    g_costs[source_idx] = 0.0;
    stats.best_cost_updates += 1;
    let generation = next_search_generation(&mut counter)?;
    open_set.push(OpenEntry {
        f_score: jps4_heuristic(source.x, source.y, target_point, grid_size_um),
        tie_score: heap_tie_score(0.0, HeapTieBreaker::SmallerG),
        g_score: 0.0,
        counter: generation,
        generation,
        idx: source_idx,
    });
    stats.heap_pushes += 1;
    stats.max_heap_size = stats.max_heap_size.max(open_set.len());

    let mut reached_idx = None;
    let mut iterations = 0usize;
    while let Some(entry) = open_set.pop() {
        stats.heap_pops += 1;
        iterations += 1;
        if iterations > config.max_iterations {
            return None;
        }
        if closed.get(entry.idx) {
            stats.skipped_duplicate_heap_entries += 1;
            stats.closed_heap_entries += 1;
            continue;
        }
        closed.set(entry.idx)?;
        stats.expanded_states += 1;
        let (x, y) = jps4_idx_to_xy(entry.idx, &dense_grid)?;
        if (x, y) == target_point {
            reached_idx = Some(entry.idx);
            break;
        }

        for (dx, dy) in JPS4_DIRECTIONS {
            stats.generated_neighbors += 1;
            let Some((jump_x, jump_y, distance_cells)) =
                jps4_jump(&dense_grid, x, y, dx, dy, target_point, &mut stats)
            else {
                continue;
            };
            let jump_idx = dense_grid.idx_of(jump_x, jump_y)?;
            if closed.get(jump_idx) {
                continue;
            }
            let tentative_g = g_costs[entry.idx] + f64::from(distance_cells) * grid_size_um;
            if tentative_g >= g_costs[jump_idx] {
                continue;
            }
            g_costs[jump_idx] = tentative_g;
            parent_idx[jump_idx] = u32::try_from(entry.idx).ok()?;
            stats.best_cost_updates += 1;
            stats.parent_updates += 1;
            let generation = next_search_generation(&mut counter)?;
            open_set.push(OpenEntry {
                f_score: tentative_g + jps4_heuristic(jump_x, jump_y, target_point, grid_size_um),
                tie_score: heap_tie_score(tentative_g, HeapTieBreaker::SmallerG),
                g_score: tentative_g,
                counter: generation,
                generation,
                idx: jump_idx,
            });
            stats.heap_pushes += 1;
            stats.max_heap_size = stats.max_heap_size.max(open_set.len());
        }
    }

    let reached_idx = reached_idx?;
    stats.jps4_used = true;
    reconstruct_jps4_route(
        source,
        target,
        source_idx,
        reached_idx,
        &parent_idx,
        &dense_grid,
        g_costs[reached_idx],
        stats,
    )
}

fn jps4_jump(
    dense_grid: &DenseRoutingGrid,
    start_x: i32,
    start_y: i32,
    dx: i32,
    dy: i32,
    target: (i32, i32),
    stats: &mut RouteSearchStats,
) -> Option<(i32, i32, i32)> {
    let mut x = start_x;
    let mut y = start_y;
    let mut distance = 0i32;
    loop {
        let previous_x = x;
        let previous_y = y;
        x = x.checked_add(dx)?;
        y = y.checked_add(dy)?;
        distance = distance.checked_add(1)?;
        stats.obstacle_clearance_checks += 1;
        if dense_grid.is_blocked(x, y) {
            if distance > 1 {
                return Some((previous_x, previous_y, distance - 1));
            }
            return None;
        }
        if (x, y) == target {
            return Some((x, y, distance));
        }
        if (dx != 0 && x == target.0) || (dy != 0 && y == target.1) {
            return Some((x, y, distance));
        }
        if jps4_has_forced_neighbor(dense_grid, x, y, dx, dy) {
            return Some((x, y, distance));
        }
        if jps4_has_obstacle_corner_opening(dense_grid, x, y, dx, dy) {
            return Some((x, y, distance));
        }
    }
}

fn jps4_has_forced_neighbor(
    dense_grid: &DenseRoutingGrid,
    x: i32,
    y: i32,
    dx: i32,
    dy: i32,
) -> bool {
    if dx != 0 {
        (dense_grid.is_blocked(x, y + 1) && !dense_grid.is_blocked(x + dx, y + 1))
            || (dense_grid.is_blocked(x, y - 1) && !dense_grid.is_blocked(x + dx, y - 1))
    } else if dy != 0 {
        (dense_grid.is_blocked(x + 1, y) && !dense_grid.is_blocked(x + 1, y + dy))
            || (dense_grid.is_blocked(x - 1, y) && !dense_grid.is_blocked(x - 1, y + dy))
    } else {
        false
    }
}

fn jps4_has_obstacle_corner_opening(
    dense_grid: &DenseRoutingGrid,
    x: i32,
    y: i32,
    dx: i32,
    dy: i32,
) -> bool {
    if dx != 0 {
        (dense_grid.is_blocked(x - dx, y + 1) && !dense_grid.is_blocked(x, y + 1))
            || (dense_grid.is_blocked(x - dx, y - 1) && !dense_grid.is_blocked(x, y - 1))
    } else if dy != 0 {
        (dense_grid.is_blocked(x + 1, y - dy) && !dense_grid.is_blocked(x + 1, y))
            || (dense_grid.is_blocked(x - 1, y - dy) && !dense_grid.is_blocked(x - 1, y))
    } else {
        false
    }
}

fn jps4_heuristic(x: i32, y: i32, target: (i32, i32), grid_size_um: f64) -> f64 {
    f64::from((target.0 - x).abs() + (target.1 - y).abs()) * grid_size_um
}

fn jps4_idx_to_xy(idx: usize, dense_grid: &DenseRoutingGrid) -> Option<(i32, i32)> {
    let width = usize::try_from(dense_grid.width).ok()?;
    let local_x = i32::try_from(idx % width).ok()?;
    let local_y = i32::try_from(idx / width).ok()?;
    Some((
        dense_grid.bounds.min_x + local_x,
        dense_grid.bounds.min_y + local_y,
    ))
}

fn reconstruct_jps4_route(
    _source: State,
    target: State,
    source_idx: usize,
    reached_idx: usize,
    parent_idx: &[u32],
    dense_grid: &DenseRoutingGrid,
    total_cost: f64,
    stats: RouteSearchStats,
) -> Option<RouteResult> {
    let mut jump_indices = Vec::new();
    let mut current = reached_idx;
    loop {
        jump_indices.push(current);
        if current == source_idx {
            break;
        }
        let parent = *parent_idx.get(current)?;
        if parent == NO_PARENT {
            return None;
        }
        current = usize::try_from(parent).ok()?;
    }
    jump_indices.reverse();

    let mut cells = Vec::new();
    for idx in jump_indices {
        let point = jps4_idx_to_xy(idx, dense_grid)?;
        if cells.is_empty() {
            cells.push(point);
            continue;
        }
        let previous = *cells.last()?;
        let step_x = (point.0 - previous.0).signum();
        let step_y = (point.1 - previous.1).signum();
        let mut x = previous.0;
        let mut y = previous.1;
        while (x, y) != point {
            x = x.checked_add(step_x)?;
            y = y.checked_add(step_y)?;
            cells.push((x, y));
        }
    }

    let mut states = Vec::with_capacity(cells.len());
    for idx in 0..cells.len() {
        let angle = if idx + 1 < cells.len() {
            jps4_angle_between(cells[idx], cells[idx + 1])?
        } else {
            target.angle
        };
        states.push(State::new(cells[idx].0, cells[idx].1, angle));
    }
    let compressed_waypoints = compress_grid_waypoints(&cells);
    Some(RouteResult {
        states,
        primitives: Vec::new(),
        cells,
        compressed_waypoints,
        total_length_um: total_cost,
        total_cost,
        requested_target: target,
        reached_target: target,
        stats,
    })
}

fn jps4_angle_between(a: (i32, i32), b: (i32, i32)) -> Option<u8> {
    match ((b.0 - a.0).signum(), (b.1 - a.1).signum()) {
        (1, 0) => Some(0),
        (0, 1) => Some(2),
        (-1, 0) => Some(4),
        (0, -1) => Some(6),
        _ => None,
    }
}

fn simple_candidate_to_route_result(
    candidate: &SimpleRouteCandidate,
    source: State,
    target: State,
    primitives: &PrimitiveLibrary,
    stats: RouteSearchStats,
) -> Option<RouteResult> {
    let expanded = expand_candidate_to_grid_points(candidate);
    if expanded.len() < 2 || candidate.points.len() < 2 {
        return None;
    }
    let start_point = grid_point_from_state(source);
    let end_point = grid_point_from_state(target);
    if expanded.first().copied() != Some(start_point) || expanded.last().copied() != Some(end_point)
    {
        return None;
    }

    let segment_count = candidate.points.len() - 1;
    let mut headings = Vec::with_capacity(segment_count);
    let mut segment_lengths = Vec::with_capacity(segment_count);
    for i in 0..segment_count {
        let a = candidate.points[i];
        let b = candidate.points[i + 1];
        headings.push(simple_direction_between(a, b)?);
        segment_lengths.push((b.x - a.x).abs() + (b.y - a.y).abs());
    }

    let bend_radius_cells = infer_bend_radius_cells(primitives).unwrap_or(0);
    let min_bend_adjacent_len = bend_radius_cells;
    if bend_radius_cells > 0 {
        for (idx, &length) in segment_lengths.iter().enumerate() {
            let has_prev_bend = idx > 0 && headings[idx - 1] != headings[idx];
            let has_next_bend = idx + 1 < segment_count && headings[idx] != headings[idx + 1];
            let required_len = if has_prev_bend && has_next_bend {
                2 * min_bend_adjacent_len
            } else if has_prev_bend || has_next_bend {
                min_bend_adjacent_len
            } else {
                0
            };
            if length < required_len {
                return None;
            }
        }
    }
    let mut trimmed_lengths = Vec::with_capacity(segment_count);
    for (idx, &length) in segment_lengths.iter().enumerate() {
        let mut trimmed = length;
        if idx > 0 {
            trimmed -= bend_radius_cells;
        }
        if idx + 1 < segment_count {
            trimmed -= bend_radius_cells;
        }
        if trimmed < 0 {
            return None;
        }
        trimmed_lengths.push(trimmed);
    }

    if segment_count == 3 && bend_radius_cells > 0 {
        // Two bends consume one bend radius from both ends of the middle leg.
        if segment_lengths[1] < 2 * bend_radius_cells {
            return None;
        }
    }

    let mut states = vec![source];
    let mut primitive_ids = Vec::new();
    let mut total_length_um = 0.0;
    let mut current = source;

    for i in 0..segment_count {
        let straight_cells = trimmed_lengths[i];
        if straight_cells > 0 {
            let straight_primitive_ids =
                decompose_straight_cells(current.angle, straight_cells, primitives)?;
            for primitive_id in straight_primitive_ids {
                let primitive = find_primitive(primitives, current.angle, primitive_id)?;
                primitive_ids.push(primitive.id);
                total_length_um += primitive.length_um;
                current = State::new(
                    current.x.checked_add(primitive.dx)?,
                    current.y.checked_add(primitive.dy)?,
                    primitive.end_angle,
                );
                states.push(current);
            }
        }

        if i + 1 < segment_count {
            let delta = turn_delta(headings[i], headings[i + 1])?;
            let bend_primitive_id =
                find_bend_primitive_id(current.angle, delta, bend_radius_cells, primitives)?;
            let primitive = find_primitive(primitives, current.angle, bend_primitive_id)?;
            primitive_ids.push(primitive.id);
            total_length_um += primitive.length_um;
            current = State::new(
                current.x.checked_add(primitive.dx)?,
                current.y.checked_add(primitive.dy)?,
                primitive.end_angle,
            );
            states.push(current);
        }
    }

    if current != target {
        return None;
    }

    let mut cells = Vec::new();
    let mut seen_cells = FxHashSet::default();
    let mut ordered_path = Vec::new();
    push_if_different(&mut ordered_path, (source.x, source.y));
    for (idx, primitive_id) in primitive_ids.iter().copied().enumerate() {
        let origin = states[idx];
        let primitive = find_primitive(primitives, origin.angle, primitive_id)?;
        for (dx, dy) in primitive.footprint.iter().copied() {
            let cell = (origin.x + dx, origin.y + dy);
            push_if_different(&mut ordered_path, cell);
            if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                cells.push(cell);
            }
        }
    }
    push_if_different(&mut ordered_path, (target.x, target.y));

    let compressed_waypoints = compress_grid_waypoints(&ordered_path);
    Some(RouteResult {
        states,
        primitives: primitive_ids,
        cells,
        compressed_waypoints,
        total_length_um,
        total_cost: total_length_um,
        requested_target: target,
        reached_target: target,
        stats,
    })
}

#[inline]
fn grid_point_from_state(state: State) -> GridPoint {
    GridPoint::new(state.x, state.y)
}

fn infer_bend_radius_cells(primitives: &PrimitiveLibrary) -> Option<i32> {
    let grid_size = primitives.grid_size_um();
    if grid_size <= 0.0 {
        return None;
    }

    for angle in 0..8u8 {
        for primitive in primitives.get_primitives_for_angle(angle) {
            if let PrimitiveGeometry::Bend {
                radius_um,
                angle_delta,
            } = primitive.geometry
            {
                if angle_delta.unsigned_abs() == 2 {
                    let cells = (radius_um / grid_size).round() as i32;
                    if cells > 0 {
                        return Some(cells);
                    }
                }
            }
        }
    }
    None
}

fn decompose_straight_cells(
    start_angle: u8,
    total_cells: i32,
    primitives: &PrimitiveLibrary,
) -> Option<Vec<u16>> {
    if total_cells < 0 {
        return None;
    }
    if total_cells == 0 {
        return Some(Vec::new());
    }

    let mut options: Vec<(usize, u16)> = primitives
        .get_primitives_for_angle(start_angle)
        .iter()
        .filter_map(|primitive| {
            if let PrimitiveGeometry::Straight { .. } = primitive.geometry {
                if primitive.end_angle != start_angle {
                    return None;
                }
                let cells = (primitive.dx.abs() + primitive.dy.abs()) as usize;
                if cells == 0 {
                    return None;
                }
                Some((cells, primitive.id))
            } else {
                None
            }
        })
        .collect();
    options.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.cmp(&b.1)));
    options.dedup();
    if options.is_empty() {
        return None;
    }

    let target = usize::try_from(total_cells).ok()?;
    let mut best_count: Vec<usize> = vec![usize::MAX; target + 1];
    let mut prev_sum: Vec<usize> = vec![usize::MAX; target + 1];
    let mut prev_opt: Vec<usize> = vec![usize::MAX; target + 1];
    best_count[0] = 0;

    for sum in 0..=target {
        if best_count[sum] == usize::MAX {
            continue;
        }
        for (opt_idx, (cells, _)) in options.iter().copied().enumerate() {
            let next = sum + cells;
            if next > target {
                continue;
            }
            let candidate_count = best_count[sum] + 1;
            if candidate_count < best_count[next] {
                best_count[next] = candidate_count;
                prev_sum[next] = sum;
                prev_opt[next] = opt_idx;
            }
        }
    }

    if best_count[target] == usize::MAX {
        return None;
    }

    let mut ids_reversed = Vec::new();
    let mut cur = target;
    while cur > 0 {
        let opt_idx = prev_opt[cur];
        if opt_idx == usize::MAX {
            return None;
        }
        ids_reversed.push(options[opt_idx].1);
        cur = prev_sum[cur];
    }
    ids_reversed.reverse();
    Some(ids_reversed)
}

fn turn_delta(from: u8, to: u8) -> Option<i8> {
    let delta = (to as i16 - from as i16).rem_euclid(8) as u8;
    match delta {
        2 => Some(2),
        6 => Some(-2),
        _ => None,
    }
}

fn find_bend_primitive_id(
    start_angle: u8,
    angle_delta: i8,
    bend_radius_cells: i32,
    primitives: &PrimitiveLibrary,
) -> Option<u16> {
    let grid_size = primitives.grid_size_um();
    let mut candidates = primitives
        .get_primitives_for_angle(start_angle)
        .iter()
        .filter_map(|primitive| {
            if let PrimitiveGeometry::Bend {
                radius_um,
                angle_delta: primitive_delta,
            } = primitive.geometry
            {
                if primitive_delta != angle_delta {
                    return None;
                }
                let cells = (radius_um / grid_size).round() as i32;
                if bend_radius_cells > 0 && cells != bend_radius_cells {
                    return None;
                }
                Some(primitive.id)
            } else {
                None
            }
        })
        .collect::<Vec<_>>();
    candidates.sort_unstable();
    candidates.into_iter().next()
}

fn primitive_transition_class(geometry: &PrimitiveGeometry, dx: i32, dy: i32) -> usize {
    match geometry {
        PrimitiveGeometry::Straight { .. } => {
            if dx.abs().max(dy.abs()) <= 1 {
                PRIMITIVE_STRAIGHT_SHORT
            } else {
                PRIMITIVE_STRAIGHT_LONG
            }
        }
        PrimitiveGeometry::Bend { angle_delta, .. } => {
            if angle_delta.unsigned_abs() == 1 {
                PRIMITIVE_BEND_45
            } else {
                PRIMITIVE_BEND_90
            }
        }
    }
}

fn fixed_primitive_order(len: usize) -> ([usize; 8], usize) {
    let mut order = [0usize; 8];
    for (idx, slot) in order.iter_mut().enumerate().take(len.min(8)) {
        *slot = idx;
    }
    (order, len.min(8))
}

fn primitive_class_order_rank(class: usize) -> usize {
    match class {
        PRIMITIVE_STRAIGHT_LONG => 0,
        PRIMITIVE_STRAIGHT_SHORT => 1,
        PRIMITIVE_BEND_45 => 2,
        PRIMITIVE_BEND_90 => 3,
        _ => 4,
    }
}

fn target_biased_primitive_score(
    primitive: &Primitive,
    metadata: PrimitiveSearchMetadata,
    state: State,
    target: State,
    grid_size_um: f64,
) -> f64 {
    let Some(next_x) = state.x.checked_add(primitive.dx) else {
        return f64::INFINITY;
    };
    let Some(next_y) = state.y.checked_add(primitive.dy) else {
        return f64::INFINITY;
    };
    metadata.base_step_cost
        + distance_heuristic(
            State::new(next_x, next_y, primitive.end_angle),
            target,
            grid_size_um,
        )
}

fn primitive_iteration_order(
    primitives: &[Primitive],
    metadata: &[PrimitiveSearchMetadata],
    state: State,
    target: State,
    grid_size_um: f64,
    ordering: PrimitiveOrdering,
) -> ([usize; 8], usize) {
    let (mut order, len) = fixed_primitive_order(primitives.len());
    match ordering {
        PrimitiveOrdering::Library => {}
        PrimitiveOrdering::LongStraightFirst => {
            order[..len].sort_by(|a, b| {
                primitive_class_order_rank(metadata[*a].transition_class)
                    .cmp(&primitive_class_order_rank(metadata[*b].transition_class))
                    .then_with(|| a.cmp(b))
            });
        }
        PrimitiveOrdering::TargetBiased => {
            order[..len].sort_by(|a, b| {
                let a_score = target_biased_primitive_score(
                    &primitives[*a],
                    metadata[*a],
                    state,
                    target,
                    grid_size_um,
                );
                let b_score = target_biased_primitive_score(
                    &primitives[*b],
                    metadata[*b],
                    state,
                    target,
                    grid_size_um,
                );
                a_score
                    .partial_cmp(&b_score)
                    .unwrap_or(Ordering::Equal)
                    .then_with(|| a.cmp(b))
            });
        }
    }
    (order, len)
}

fn route_single_net_with_bounds(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    routing_bounds: Option<RoutingBounds>,
    stats: &mut RouteSearchStats,
) -> Option<RouteResult> {
    let bounds = if let Some(bounds) = routing_bounds {
        if !bounds.contains(source.x, source.y) || !bounds.contains(target.x, target.y) {
            return None;
        }
        bounds
    } else {
        RoutingBounds {
            min_x: 0,
            max_x: obstacle_map.width() - 1,
            min_y: 0,
            max_y: obstacle_map.height() - 1,
        }
    };

    let mut storage = DenseSearchStorage::new(bounds, config.max_dense_states)?;
    stats.dense_search_states = storage.state_count();
    stats.dense_search_storage_bytes = storage.allocated_bytes();
    let dense_grid = match DenseRoutingGrid::from_obstacle_map(
        obstacle_map,
        bounds,
        port_open_cells,
        config.max_dense_obstacle_cells,
        config.ignore_dynamic_obstacles,
        config.history_weight > 0.0,
    ) {
        Some(grid) => grid,
        None => {
            stats.dense_grid_build_failures += 1;
            return None;
        }
    };
    stats.dense_grid_cells = dense_grid.blocked_count();
    stats.dense_grid_build_time_us = dense_grid.build_time_us();
    let search_heuristic = SearchHeuristic::new(target, primitives, config);

    let primitive_buckets: [&[Primitive]; 8] =
        std::array::from_fn(|angle| primitives.get_primitives_for_angle(angle as u8));
    let primitive_search_metadata: Vec<Vec<PrimitiveSearchMetadata>> = primitive_buckets
        .iter()
        .map(|bucket| {
            bucket
                .iter()
                .map(|primitive| {
                    PrimitiveSearchMetadata::from_primitive(primitive, config.bend_weight)
                })
                .collect()
        })
        .collect();
    let primitive_footprint_profiles: Vec<Vec<FootprintCollisionProfile>> = primitive_buckets
        .iter()
        .map(|bucket| {
            bucket
                .iter()
                .map(|primitive| FootprintCollisionProfile::from_footprint(&primitive.footprint))
                .collect()
        })
        .collect();
    let target_tolerance = config.target_tolerance_cells.max(0);
    let accepted_target_angles = target_angle_acceptance(target, config);

    let mut open_set = OpenSet::new(config.use_indexed_heap, storage.state_count());
    let mut counter = 0u32;
    let collect_detailed_timing = config.collect_detailed_timing;
    let source_idx = storage.state_to_idx(source)?;

    storage.g_costs[source_idx] = 0.0;
    let generation = next_search_generation(&mut counter)?;
    storage.best_generation[source_idx] = generation;
    stats.best_cost_updates += 1;
    let source_entry = OpenEntry {
        f_score: search_heuristic.estimate(source),
        tie_score: heap_tie_score(0.0, config.heap_tie_breaker),
        g_score: 0.0,
        counter: generation,
        generation,
        idx: source_idx,
    };
    if collect_detailed_timing {
        let heap_start = Instant::now();
        let queued = open_set.push(source_entry);
        stats.heap_operation_time_us += heap_start.elapsed().as_micros();
        if queued {
            stats.heap_pushes += 1;
            stats.max_heap_size = stats.max_heap_size.max(open_set.len());
        }
    } else {
        let queued = open_set.push(source_entry);
        if queued {
            stats.heap_pushes += 1;
            stats.max_heap_size = stats.max_heap_size.max(open_set.len());
        }
    }
    let mut iterations = 0usize;
    loop {
        let entry = if collect_detailed_timing {
            let heap_start = Instant::now();
            let entry = open_set.pop();
            stats.heap_operation_time_us += heap_start.elapsed().as_micros();
            entry
        } else {
            open_set.pop()
        };
        let Some(entry) = entry else {
            break;
        };
        stats.heap_pops += 1;
        iterations += 1;
        if iterations > config.max_iterations {
            return None;
        }

        let idx = entry.idx;
        if storage.best_generation[idx] != entry.generation {
            stats.skipped_duplicate_heap_entries += 1;
            stats.stale_generation_heap_entries += 1;
            continue;
        }
        if storage.closed.get(idx) {
            stats.skipped_duplicate_heap_entries += 1;
            stats.closed_heap_entries += 1;
            continue;
        }
        let state = storage.idx_to_state(idx);
        if (state.x - target.x).abs() <= target_tolerance
            && (state.y - target.y).abs() <= target_tolerance
            && accepted_target_angles[state.angle as usize]
        {
            if collect_detailed_timing {
                let reconstruction_start = Instant::now();
                let mut route = reconstruct_route_dense(
                    source_idx,
                    idx,
                    target,
                    primitives,
                    entry.g_score,
                    stats.clone(),
                    &storage,
                )?;
                route.stats.reconstruction_time_us += reconstruction_start.elapsed().as_micros();
                return Some(route);
            }
            return reconstruct_route_dense(
                source_idx,
                idx,
                target,
                primitives,
                entry.g_score,
                stats.clone(),
                &storage,
            );
        }
        storage.closed.set(idx)?;
        stats.expanded_states += 1;

        let current_g = storage.g_costs[idx];
        let angle = state.angle as usize;
        let primitive_bucket = primitive_buckets[angle];
        let primitive_metadata = &primitive_search_metadata[angle];
        let footprint_profiles = &primitive_footprint_profiles[angle];
        let neighbor_loop_start = if collect_detailed_timing {
            Some(Instant::now())
        } else {
            None
        };
        let mut neighbor_loop_heap_time_us = 0u128;
        let mut neighbor_loop_legality_time_us = 0u128;
        let (primitive_order, primitive_order_len) = primitive_iteration_order(
            primitive_bucket,
            primitive_metadata,
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        );
        for primitive_idx in primitive_order.into_iter().take(primitive_order_len) {
            let primitive = &primitive_bucket[primitive_idx];
            let metadata = primitive_metadata[primitive_idx];
            let profile = &footprint_profiles[primitive_idx];
            let primitive_class = metadata.transition_class;
            stats.generated_neighbors += 1;
            stats.primitive_generated_by_class[primitive_class] += 1;
            let next_x = state.x.checked_add(primitive.dx)?;
            let next_y = state.y.checked_add(primitive.dy)?;
            let next_angle = primitive.end_angle % 8;

            if !bounds.contains(next_x, next_y) {
                stats.window_rejects += 1;
                stats.primitive_bounds_rejects_by_class[primitive_class] += 1;
                continue;
            }
            let next_idx = storage.in_bounds_parts_to_idx(next_x, next_y, next_angle);
            if storage.closed.get(next_idx) {
                stats.primitive_closed_rejects_by_class[primitive_class] += 1;
                continue;
            }

            let base_step_cost = metadata.base_step_cost;
            let tentative_g_lower_bound = current_g + base_step_cost;
            if tentative_g_lower_bound >= storage.g_costs[next_idx] {
                stats.primitive_cost_pruned_by_class[primitive_class] += 1;
                continue;
            }

            stats.primitive_footprint_checks += 1;
            stats.primitive_footprint_checks_by_class[primitive_class] += 1;
            stats.obstacle_clearance_checks += 1;
            // TODO: bounds may need primitive-footprint margin to avoid rejecting valid routes near window edges.
            let footprint_free = if collect_detailed_timing {
                let legality_start = Instant::now();
                let footprint_free = dense_grid.primitive_footprint_free_with_profile(
                    state.x,
                    state.y,
                    &primitive.footprint,
                    profile,
                    stats,
                );
                let legality_elapsed_us = legality_start.elapsed().as_micros();
                stats.legality_check_time_us += legality_elapsed_us;
                neighbor_loop_legality_time_us += legality_elapsed_us;
                footprint_free
            } else {
                dense_grid.primitive_footprint_free_with_profile(
                    state.x,
                    state.y,
                    &primitive.footprint,
                    profile,
                    stats,
                )
            };
            if !footprint_free {
                stats.footprint_rejects += 1;
                stats.primitive_footprint_rejects_by_class[primitive_class] += 1;
                if profile.is_full_rect {
                    stats.primitive_footprint_rect_rejects += 1;
                }
                continue;
            }

            let history_cost = if config.history_weight > 0.0 {
                dense_grid.primitive_footprint_history_with_profile(
                    state.x,
                    state.y,
                    &primitive.footprint,
                    profile,
                ) as f64
                    * config.history_weight
            } else {
                0.0
            };
            let step_cost = base_step_cost + history_cost;
            let tentative_g = current_g + step_cost;
            if tentative_g >= storage.g_costs[next_idx] {
                stats.primitive_cost_pruned_by_class[primitive_class] += 1;
                continue;
            }

            stats.primitive_accepted_by_class[primitive_class] += 1;
            storage.parent_idx[next_idx] = idx as u32;
            storage.parent_primitive[next_idx] = primitive.id;
            storage.g_costs[next_idx] = tentative_g;
            let generation = next_search_generation(&mut counter)?;
            storage.best_generation[next_idx] = generation;
            stats.best_cost_updates += 1;
            stats.parent_updates += 1;
            let next_entry = OpenEntry {
                f_score: tentative_g
                    + search_heuristic.estimate(State {
                        x: next_x,
                        y: next_y,
                        angle: next_angle,
                    }),
                tie_score: heap_tie_score(tentative_g, config.heap_tie_breaker),
                g_score: tentative_g,
                counter: generation,
                generation,
                idx: next_idx,
            };
            if collect_detailed_timing {
                let heap_start = Instant::now();
                let queued = open_set.push(next_entry);
                let heap_elapsed_us = heap_start.elapsed().as_micros();
                stats.heap_operation_time_us += heap_elapsed_us;
                neighbor_loop_heap_time_us += heap_elapsed_us;
                if queued {
                    stats.heap_pushes += 1;
                    stats.max_heap_size = stats.max_heap_size.max(open_set.len());
                }
            } else {
                let queued = open_set.push(next_entry);
                if queued {
                    stats.heap_pushes += 1;
                    stats.max_heap_size = stats.max_heap_size.max(open_set.len());
                }
            }
        }
        if let Some(neighbor_loop_start) = neighbor_loop_start {
            let neighbor_loop_elapsed_us = neighbor_loop_start.elapsed().as_micros();
            stats.neighbor_generation_time_us += neighbor_loop_elapsed_us
                .saturating_sub(neighbor_loop_heap_time_us)
                .saturating_sub(neighbor_loop_legality_time_us);
        }
    }

    None
}

fn compute_routing_bounds(
    obstacle_map: &ObstacleMap,
    source: State,
    target: State,
    config: &AStarConfig,
    expansion_idx: u32,
) -> Option<RoutingBounds> {
    let span_x = (target.x - source.x).abs();
    let span_y = (target.y - source.y).abs();

    let growth = 1.0 + (expansion_idx as f64) * config.routing_window_growth.max(0.0);
    let tolerance_padding = config.target_tolerance_cells.max(0);
    let margin_x = ((config.routing_window_scale.max(0.0) * (span_x as f64) * growth).ceil()
        as i32)
        .max(config.routing_window_min_margin_cells.max(0))
        .saturating_add(tolerance_padding);
    let margin_y = ((config.routing_window_scale.max(0.0) * (span_y as f64) * growth).ceil()
        as i32)
        .max(config.routing_window_min_margin_cells.max(0))
        .saturating_add(tolerance_padding);

    let min_x = source.x.min(target.x).saturating_sub(margin_x).max(0);
    let max_x = source
        .x
        .max(target.x)
        .saturating_add(margin_x)
        .min(obstacle_map.width() - 1);
    let min_y = source.y.min(target.y).saturating_sub(margin_y).max(0);
    let max_y = source
        .y
        .max(target.y)
        .saturating_add(margin_y)
        .min(obstacle_map.height() - 1);

    if min_x > max_x || min_y > max_y {
        return None;
    }
    Some(RoutingBounds {
        min_x,
        max_x,
        min_y,
        max_y,
    })
}

fn window_area(bounds: RoutingBounds) -> i64 {
    let width = (bounds.max_x - bounds.min_x + 1).max(0) as i64;
    let height = (bounds.max_y - bounds.min_y + 1).max(0) as i64;
    width * height
}

/// Export an SVG string showing obstacles and a routed path.
pub fn export_route_svg(obstacle_map: &ObstacleMap, route_result: &RouteResult) -> String {
    let width = obstacle_map.width();
    let height = obstacle_map.height();
    if width <= 0 || height <= 0 {
        return r#"<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1" />"#.to_string();
    }

    let cell_px = (1200 / width.max(height).max(1)).clamp(1, 8);
    let width_px = width * cell_px;
    let height_px = height * cell_px;
    let mut svg = String::new();

    svg.push_str(&format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" viewBox="0 0 {width} {height}">"#
    ));
    svg.push_str(r##"<rect width="100%" height="100%" fill="#eeeeee" />"##);
    svg.push_str(r#"<path d=""#);
    for x in 0..=width {
        svg.push_str(&format!("M {x} 0 V {height} "));
    }
    for y in 0..=height {
        svg.push_str(&format!("M 0 {y} H {width} "));
    }
    svg.push_str(
        r##"" stroke="#3fa34d" stroke-width="0.025" vector-effect="non-scaling-stroke" opacity="0.75" fill="none" />"##,
    );

    for x in 0..width {
        for y in 0..height {
            if obstacle_map.is_blocked(x, y) {
                let svg_y = height - y - 1;
                svg.push_str(&format!(
                    r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#000000" />"##
                ));
            }
        }
    }

    for &(x, y) in &route_result.cells {
        if obstacle_map.in_bounds(x, y) {
            let svg_y = height - y - 1;
            svg.push_str(&format!(
                r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#1a73e8" />"##
            ));
        }
    }

    svg.push_str("</svg>\n");
    svg
}

fn distance_heuristic(state: State, target: State, grid_size_um: f64) -> f64 {
    let dx = (target.x - state.x) as f64;
    let dy = (target.y - state.y) as f64;
    (dx * dx + dy * dy).sqrt() * grid_size_um
}

fn direction_reaches_target_ray(state: State, target: State, tolerance: i32) -> bool {
    let dx = target.x - state.x;
    let dy = target.y - state.y;
    if dx.abs() <= tolerance && dy.abs() <= tolerance {
        return true;
    }
    let (dir_x, dir_y) = DIRECTIONS[(state.angle % 8) as usize];
    match (dir_x, dir_y) {
        (0, 0) => false,
        (0, _) => dx.abs() <= tolerance && dy.signum() == dir_y,
        (_, 0) => dy.abs() <= tolerance && dx.signum() == dir_x,
        _ => {
            dx.signum() == dir_x && dy.signum() == dir_y && (dx.abs() - dy.abs()).abs() <= tolerance
        }
    }
}

fn minimum_positive_bend_cost(primitives: &PrimitiveLibrary, bend_weight: f64) -> f64 {
    if bend_weight <= 0.0 {
        return 0.0;
    }
    let min_bend_cost = (0u8..8u8)
        .flat_map(|angle| primitives.get_primitives_for_angle(angle).iter())
        .filter_map(|primitive| {
            if primitive.bend_cost > 0.0 {
                Some(primitive.bend_cost)
            } else {
                None
            }
        })
        .min_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    min_bend_cost.map_or(0.0, |cost| cost * bend_weight)
}

#[derive(Clone, Copy, Debug)]
enum SearchHeuristicMode {
    Distance,
    HeadingAware {
        minimum_bend_cost: f64,
        tolerance: i32,
        target_angle_ok: [bool; 8],
    },
}

#[derive(Clone, Copy, Debug)]
struct SearchHeuristic {
    target: State,
    grid_size_um: f64,
    mode: SearchHeuristicMode,
}

impl SearchHeuristic {
    fn new(target: State, primitives: &PrimitiveLibrary, config: &AStarConfig) -> Self {
        let minimum_bend_cost = if config.heuristic_mode == HeuristicMode::HeadingAware {
            minimum_positive_bend_cost(primitives, config.bend_weight)
        } else {
            0.0
        };
        let mode = if minimum_bend_cost > 0.0 {
            SearchHeuristicMode::HeadingAware {
                minimum_bend_cost,
                tolerance: config.target_tolerance_cells.max(0),
                target_angle_ok: target_angle_acceptance(target, config),
            }
        } else {
            SearchHeuristicMode::Distance
        };
        Self {
            target,
            grid_size_um: primitives.grid_size_um(),
            mode,
        }
    }

    fn estimate(&self, state: State) -> f64 {
        let distance = distance_heuristic(state, self.target, self.grid_size_um);
        let SearchHeuristicMode::HeadingAware {
            minimum_bend_cost,
            tolerance,
            target_angle_ok,
        } = self.mode
        else {
            return distance;
        };
        if !target_angle_ok[(state.angle % 8) as usize]
            || !direction_reaches_target_ray(state, self.target, tolerance)
        {
            distance + minimum_bend_cost
        } else {
            distance
        }
    }
}

fn target_angle_acceptance(target: State, config: &AStarConfig) -> [bool; 8] {
    let mut accepted = [true; 8];
    if let Some(mask) = config.allowed_target_angles_mask {
        for angle in 0u8..8u8 {
            accepted[angle as usize] = (mask & (1u8 << angle)) != 0;
        }
    } else if config.require_target_angle {
        accepted = [false; 8];
        accepted[(target.angle % 8) as usize] = true;
    }
    accepted
}

fn reconstruct_route_dense(
    source_idx: usize,
    reached_idx: usize,
    requested_target: State,
    primitives: &PrimitiveLibrary,
    total_cost: f64,
    stats: RouteSearchStats,
    storage: &DenseSearchStorage,
) -> Option<RouteResult> {
    let source = storage.idx_to_state(source_idx);
    let reached_target = storage.idx_to_state(reached_idx);
    let mut states_reversed = vec![reached_target];
    let mut primitive_steps_reversed = Vec::new();
    let mut current_idx = reached_idx;

    while current_idx != source_idx {
        let parent_idx_u32 = *storage.parent_idx.get(current_idx)?;
        if parent_idx_u32 == NO_PARENT {
            return None;
        }
        let parent_idx = usize::try_from(parent_idx_u32).ok()?;
        let previous = storage.idx_to_state(parent_idx);
        let primitive_id = *storage.parent_primitive.get(current_idx)?;
        primitive_steps_reversed.push((previous, primitive_id));
        current_idx = parent_idx;
        states_reversed.push(previous);
    }

    states_reversed.reverse();
    primitive_steps_reversed.reverse();

    let mut primitive_ids = Vec::with_capacity(primitive_steps_reversed.len());
    let mut cells = Vec::new();
    let mut seen_cells = FxHashSet::default();
    let mut ordered_path = Vec::new();
    push_if_different(&mut ordered_path, (source.x, source.y));
    let mut total_length_um = 0.0;

    for (origin, primitive_id) in primitive_steps_reversed {
        let primitive = find_primitive(primitives, origin.angle, primitive_id)?;
        primitive_ids.push(primitive_id);
        total_length_um += primitive.length_um;

        for (dx, dy) in primitive.footprint.iter().copied() {
            let cell = (origin.x + dx, origin.y + dy);
            push_if_different(&mut ordered_path, cell);
            if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                cells.push(cell);
            }
        }
    }
    push_if_different(&mut ordered_path, (reached_target.x, reached_target.y));
    let compressed_waypoints = compress_grid_waypoints(&ordered_path);

    Some(RouteResult {
        states: states_reversed,
        primitives: primitive_ids,
        cells,
        compressed_waypoints,
        total_length_um,
        total_cost,
        requested_target,
        reached_target,
        stats,
    })
}

fn find_primitive(
    primitives: &PrimitiveLibrary,
    start_angle: u8,
    primitive_id: u16,
) -> Option<&crate::primitives::Primitive> {
    primitives
        .get_primitives_for_angle(start_angle)
        .iter()
        .find(|p| p.id == primitive_id)
}

fn compress_grid_waypoints(path: &[(i32, i32)]) -> Vec<(i32, i32)> {
    if path.is_empty() {
        return Vec::new();
    }
    if path.len() == 1 {
        return vec![path[0]];
    }

    let mut waypoints = Vec::with_capacity(path.len());
    push_if_different(&mut waypoints, path[0]);

    let mut prev_dir = direction(path[0], path[1]);
    for i in 2..path.len() {
        let curr_dir = direction(path[i - 1], path[i]);
        if curr_dir != prev_dir {
            push_if_different(&mut waypoints, path[i - 1]);
        }
        prev_dir = curr_dir;
    }

    push_if_different(&mut waypoints, path[path.len() - 1]);
    waypoints
}

fn push_if_different(points: &mut Vec<(i32, i32)>, point: (i32, i32)) {
    if points.last().copied() != Some(point) {
        points.push(point);
    }
}

fn direction(a: (i32, i32), b: (i32, i32)) -> (i32, i32) {
    ((b.0 - a.0).signum(), (b.1 - a.1).signum())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry_realization::{route_to_primitive_centerline, GeometryGridSpec};
    use crate::obstacle_map::ObstacleMap;
    use crate::primitives::{
        create_photonic_primitive_library, Primitive, PrimitiveGeometry, PrimitiveLibraryConfig,
    };
    use std::collections::VecDeque;

    fn primitive_library() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
        })
    }

    fn primitive_library_no45_bend2() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 2,
            allow_45_degree_turns: false,
        })
    }

    fn plain_jps4_primitive_library() -> PrimitiveLibrary {
        let mut next_id = 0u16;
        let mut buckets = Vec::with_capacity(8);
        for angle in 0..8u8 {
            if angle % 2 == 0 {
                let (dx, dy) = match angle {
                    0 => (1, 0),
                    2 => (0, 1),
                    4 => (-1, 0),
                    6 => (0, -1),
                    _ => unreachable!(),
                };
                buckets.push(vec![Primitive {
                    id: next_id,
                    start_angle: angle,
                    end_angle: angle,
                    dx,
                    dy,
                    footprint: vec![(0, 0), (dx, dy)],
                    length_um: 1.0,
                    bend_cost: 0.0,
                    geometry: PrimitiveGeometry::Straight { length_um: 1.0 },
                }]);
                next_id += 1;
            } else {
                buckets.push(Vec::new());
            }
        }
        PrimitiveLibrary::new(buckets, 1.0)
    }

    fn reference_astar4_distance(
        map: &ObstacleMap,
        source: (i32, i32),
        target: (i32, i32),
    ) -> Option<usize> {
        if map.is_blocked(source.0, source.1) || map.is_blocked(target.0, target.1) {
            return None;
        }
        let width = usize::try_from(map.width()).ok()?;
        let height = usize::try_from(map.height()).ok()?;
        let mut distances = vec![usize::MAX; width.checked_mul(height)?];
        let idx = |x: i32, y: i32| -> Option<usize> {
            if !map.in_bounds(x, y) {
                return None;
            }
            let ux = usize::try_from(x).ok()?;
            let uy = usize::try_from(y).ok()?;
            uy.checked_mul(width)?.checked_add(ux)
        };
        let source_idx = idx(source.0, source.1)?;
        distances[source_idx] = 0;
        let mut queue = VecDeque::new();
        queue.push_back(source);
        while let Some((x, y)) = queue.pop_front() {
            let current_distance = distances[idx(x, y)?];
            if (x, y) == target {
                return Some(current_distance);
            }
            for (dx, dy) in JPS4_DIRECTIONS {
                let nx = x + dx;
                let ny = y + dy;
                if map.is_blocked(nx, ny) {
                    continue;
                }
                let next_idx = idx(nx, ny)?;
                if distances[next_idx] != usize::MAX {
                    continue;
                }
                distances[next_idx] = current_distance + 1;
                queue.push_back((nx, ny));
            }
        }
        None
    }

    fn route_with_jps4(map: &ObstacleMap, source: State, target: State) -> RouteResult {
        let mut config = AStarConfig::default();
        config.enable_jps4 = true;
        config.require_target_angle = false;
        config.enable_simple_routes = false;
        route_single_net_with_config(
            map,
            &plain_jps4_primitive_library(),
            source,
            target,
            None,
            &config,
        )
        .expect("jps4 route should exist")
    }

    #[test]
    fn jps4_eligibility_rejects_current_photonic_primitives() {
        let mut config = AStarConfig::default();
        config.enable_jps4 = true;
        config.require_target_angle = false;

        let eligibility = evaluate_jps4_eligibility(
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            &config,
        );

        assert!(!eligibility.eligible);
        assert_eq!(
            eligibility.reason,
            "primitive library is not plain 4-connected unit grid"
        );
    }

    #[test]
    fn jps4_eligibility_accepts_plain_cardinal_unit_grid() {
        let mut config = AStarConfig::default();
        config.enable_jps4 = true;
        config.require_target_angle = false;

        let eligibility = evaluate_jps4_eligibility(
            &plain_jps4_primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            &config,
        );

        assert!(eligibility.eligible);
        assert_eq!(eligibility.reason, "eligible");
    }

    #[test]
    fn jps4_request_falls_back_to_baseline_route_for_photonic_primitives() {
        let map = ObstacleMap::new(10, 5);
        let mut config = AStarConfig::default();
        config.enable_jps4 = true;
        config.require_target_angle = false;

        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 2, 0),
            State::new(5, 2, 0),
            None,
            &config,
        )
        .expect("baseline fallback route should exist");

        assert_eq!(result.states.first().copied(), Some(State::new(1, 2, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(5, 2, 0)));
        assert!(result.stats.jps4_requested);
        assert!(!result.stats.jps4_eligible);
        assert_eq!(result.stats.jps4_fallbacks, 1);
        assert_eq!(
            result.stats.jps4_fallback_reason,
            "primitive library is not plain 4-connected unit grid"
        );
    }

    #[test]
    fn jps4_routes_empty_map_like_reference_astar4() {
        let map = ObstacleMap::new(12, 9);
        let source = State::new(1, 1, 0);
        let target = State::new(9, 6, 0);
        let route = route_with_jps4(&map, source, target);
        let reference_distance =
            reference_astar4_distance(&map, (source.x, source.y), (target.x, target.y)).unwrap();

        assert!(route.stats.jps4_used);
        assert_eq!(route.total_length_um, reference_distance as f64);
        assert_eq!(route.cells.first().copied(), Some((source.x, source.y)));
        assert_eq!(route.cells.last().copied(), Some((target.x, target.y)));
    }

    #[test]
    fn jps4_routes_narrow_corridor_like_reference_astar4() {
        let mut map = ObstacleMap::new(14, 7);
        for y in 0..7 {
            if y == 3 {
                continue;
            }
            for x in 0..14 {
                map.add_static_cell(x, y);
            }
        }
        let source = State::new(1, 3, 0);
        let target = State::new(12, 3, 0);
        let route = route_with_jps4(&map, source, target);
        let reference_distance =
            reference_astar4_distance(&map, (source.x, source.y), (target.x, target.y)).unwrap();

        assert!(route.stats.jps4_used);
        assert_eq!(route.total_length_um, reference_distance as f64);
        assert_eq!(route.compressed_waypoints, vec![(1, 3), (12, 3)]);
    }

    #[test]
    fn jps4_routes_forced_detour_like_reference_astar4() {
        let mut map = ObstacleMap::new(12, 10);
        for y in 0..9 {
            if y != 7 {
                map.add_static_cell(5, y);
            }
        }
        let source = State::new(2, 2, 0);
        let target = State::new(9, 2, 0);
        let route = route_with_jps4(&map, source, target);
        let reference_distance =
            reference_astar4_distance(&map, (source.x, source.y), (target.x, target.y)).unwrap();

        assert!(route.stats.jps4_used);
        assert_eq!(route.total_length_um, reference_distance as f64);
        assert!(route.cells.contains(&(5, 7)));
    }

    #[test]
    fn routes_straight_without_obstacles() {
        let map = ObstacleMap::new(10, 5);
        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 2, 0),
            State::new(5, 2, 0),
            None,
        )
        .expect("straight route should exist");

        assert_eq!(result.states.first().copied(), Some(State::new(1, 2, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(5, 2, 0)));
        assert_eq!(result.compressed_waypoints, vec![(1, 2), (5, 2)]);
        assert!(result.cells.contains(&(5, 2)));
        assert_eq!(result.total_length_um, 4.0);
    }

    #[test]
    fn routes_around_simple_block() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 3);
        map.add_static_cell(4, 3);
        map.add_static_cell(5, 3);

        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 3, 0),
            State::new(8, 3, 0),
            None,
        )
        .expect("route around obstacle should exist");

        assert_eq!(result.states.last().copied(), Some(State::new(8, 3, 0)));
        assert!(!result
            .cells
            .iter()
            .any(|&(x, y)| map.is_static_blocked(x, y)));
    }

    #[test]
    fn routes_with_ninety_degree_turn() {
        let map = ObstacleMap::new(8, 8);
        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(3, 3, 2),
            None,
        )
        .expect("90 degree turn route should exist");

        assert_eq!(result.states.last().copied(), Some(State::new(3, 3, 2)));
        assert!(result.states.iter().any(|state| state.angle == 2));
        assert!(result.compressed_waypoints.len() >= 2);
    }

    #[test]
    fn respects_exact_angle_constraints() {
        let map = ObstacleMap::new(8, 8);
        let library = primitive_library();

        assert!(route_single_net(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
        )
        .is_some());
        let north_arrival = route_single_net(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 2),
            None,
        )
        .expect("same coordinate with a different exact target angle should be routable");
        assert_eq!(
            north_arrival.states.last().copied(),
            Some(State::new(5, 1, 2))
        );
    }

    #[test]
    fn footprint_collision_blocks_route() {
        let mut map = ObstacleMap::new(6, 3);
        map.add_static_cell(2, 1);

        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(3, 1, 0),
            None,
            &AStarConfig {
                max_iterations: 200,
                bend_weight: 1.0,
                target_tolerance_cells: 0,
                ..AStarConfig::default()
            },
        );

        assert!(result.is_none());
    }

    #[test]
    fn port_opening_allows_blocked_source_and_target_cells() {
        let mut map = ObstacleMap::new(8, 3);
        map.add_static_cell(1, 1);
        map.add_static_cell(5, 1);

        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(1, 1));
        opened.insert(pack_xy(5, 1));

        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            Some(&opened),
        )
        .expect("opened port cells should be routable");

        assert_eq!(result.states.last().copied(), Some(State::new(5, 1, 0)));
    }

    #[test]
    fn exports_route_svg() {
        let map = ObstacleMap::new(6, 3);
        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
        )
        .expect("route should exist");

        let svg = export_route_svg(&map, &result);
        assert!(svg.contains("<svg"));
        assert!(svg.contains("#1a73e8"));
    }

    #[test]
    fn supports_coordinate_tolerance() {
        let map = ObstacleMap::new(12, 6);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(5, 3, 0),
            None,
            &AStarConfig {
                target_tolerance_cells: 1,
                ..AStarConfig::default()
            },
        )
        .expect("route should terminate within tolerance");

        let reached = result.states.last().copied().unwrap();
        assert!((reached.x - 5).abs() <= 1);
        assert!((reached.y - 3).abs() <= 1);
        assert_eq!(result.reached_target, reached);
    }

    #[test]
    fn reconstruction_uses_primitive_ids_and_preserves_cells() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("route should exist");

        assert!(!result.primitives.is_empty());
        for primitive_id in &result.primitives {
            assert!(*primitive_id > 0);
        }

        let mut expected_cells = Vec::new();
        let mut seen_cells = FxHashSet::default();
        for (idx, primitive_id) in result.primitives.iter().enumerate() {
            let origin = result.states[idx];
            let primitive = library
                .get_primitives_for_angle(origin.angle)
                .iter()
                .find(|p| p.id == *primitive_id)
                .expect("primitive id should resolve in library");
            for (dx, dy) in primitive.footprint.iter().copied() {
                let cell = (origin.x + dx, origin.y + dy);
                if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                    expected_cells.push(cell);
                }
            }
        }

        assert_eq!(result.cells, expected_cells);
    }

    #[test]
    fn simple_straight_route_used_before_astar() {
        let map = ObstacleMap::new(10, 6);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple straight route should exist");
        assert_eq!(result.compressed_waypoints, vec![(1, 1), (5, 1)]);
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_l_route_used_before_astar() {
        let map = ObstacleMap::new(10, 10);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 4, 2),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple L route should exist");
        assert_eq!(result.compressed_waypoints, vec![(1, 1), (5, 1), (5, 4)]);
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_z_route_used_before_astar() {
        let map = ObstacleMap::new(20, 20);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(10, 10, 0),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple Z route should exist");
        assert_eq!(
            result.compressed_waypoints,
            vec![(1, 1), (2, 1), (2, 10), (10, 10)]
        );
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_z_route_rejects_too_short_middle_leg() {
        let map = ObstacleMap::new(20, 20);
        let library = primitive_library_no45_bend2();
        let mut config = AStarConfig::default();
        config.enable_simple_routes = true;

        let result = try_simple_route_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(10, 4, 0),
            None,
            &config,
        );

        assert!(result.is_none());
    }

    #[test]
    fn simple_z_route_has_primitives_and_replay_centerline() {
        let map = ObstacleMap::new(20, 20);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 4, 0),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple Z route should exist");

        assert_eq!(result.stats.expanded_states, 0);
        assert!(!result.primitives.is_empty());
        assert_eq!(result.states.len(), result.primitives.len() + 1);

        let grid = GeometryGridSpec::new(1.0, 0.0, 0.0).expect("grid spec");
        let centerline = route_to_primitive_centerline(&result, &library, &grid)
            .expect("primitive replay centerline should succeed");
        assert!(centerline.len() >= 2);
    }

    #[test]
    fn simple_route_disabled_uses_astar() {
        let map = ObstacleMap::new(10, 6);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("A* route should exist with simple routes disabled");
        assert!(result.stats.expanded_states > 0);
    }

    #[test]
    fn default_search_experiments_stay_gated_off() {
        let config = AStarConfig::default();
        assert!(!config.use_indexed_heap);
        assert_eq!(config.primitive_ordering, PrimitiveOrdering::Library);
        assert_eq!(config.heap_tie_breaker, HeapTieBreaker::SmallerG);
    }

    #[test]
    fn indexed_heap_matches_duplicate_heap_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };

        let duplicate_route =
            route_single_net_with_config(&map, &library, source, target, None, &base_config)
                .expect("duplicate-entry heap route should exist");
        let indexed_route = route_single_net_with_config(
            &map,
            &library,
            source,
            target,
            None,
            &AStarConfig {
                use_indexed_heap: true,
                ..base_config
            },
        )
        .expect("indexed heap route should exist");

        assert_eq!(indexed_route.reached_target, duplicate_route.reached_target);
        assert!((indexed_route.total_cost - duplicate_route.total_cost).abs() < 1.0e-9);
        assert_eq!(indexed_route.stats.skipped_duplicate_heap_entries, 0);
        assert_eq!(indexed_route.stats.stale_generation_heap_entries, 0);
        assert!(duplicate_route.stats.stale_generation_heap_entries > 0);
        assert!(indexed_route.stats.max_heap_size <= duplicate_route.stats.max_heap_size);
    }

    #[test]
    fn primitive_ordering_modes_preserve_route_cost_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        let baseline =
            route_single_net_with_config(&map, &library, source, target, None, &base_config)
                .expect("baseline route should exist");

        for primitive_ordering in [
            PrimitiveOrdering::LongStraightFirst,
            PrimitiveOrdering::TargetBiased,
        ] {
            let result = route_single_net_with_config(
                &map,
                &library,
                source,
                target,
                None,
                &AStarConfig {
                    primitive_ordering,
                    ..base_config.clone()
                },
            )
            .expect("ordered route should exist");
            assert_eq!(result.reached_target, baseline.reached_target);
            assert!((result.total_cost - baseline.total_cost).abs() < 1.0e-9);
        }
    }

    #[test]
    fn heap_tie_breaker_modes_preserve_route_cost_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        let baseline =
            route_single_net_with_config(&map, &library, source, target, None, &base_config)
                .expect("baseline route should exist");

        let larger_g = route_single_net_with_config(
            &map,
            &library,
            source,
            target,
            None,
            &AStarConfig {
                heap_tie_breaker: HeapTieBreaker::LargerG,
                ..base_config
            },
        )
        .expect("larger-g route should exist");

        assert_eq!(larger_g.reached_target, baseline.reached_target);
        assert!((larger_g.total_cost - baseline.total_cost).abs() < 1.0e-9);
    }

    fn assert_heading_aware_matches_distance(
        map: &ObstacleMap,
        library: &PrimitiveLibrary,
        source: State,
        target: State,
        mut base_config: AStarConfig,
    ) {
        base_config.heuristic_mode = HeuristicMode::Distance;
        let distance =
            route_single_net_with_config(map, library, source, target, None, &base_config)
                .expect("distance heuristic route should exist");
        let heading_aware = route_single_net_with_config(
            map,
            library,
            source,
            target,
            None,
            &AStarConfig {
                heuristic_mode: HeuristicMode::HeadingAware,
                ..base_config
            },
        )
        .expect("heading-aware heuristic route should exist");

        assert_eq!(heading_aware.reached_target, distance.reached_target);
        assert!((heading_aware.total_cost - distance.total_cost).abs() < 1.0e-9);
    }

    #[test]
    fn heading_aware_heuristic_adds_only_unavoidable_minimum_bend_bound() {
        let library = primitive_library_no45_bend2();
        let min_bend_cost = minimum_positive_bend_cost(&library, 1.0);
        let config = AStarConfig {
            heuristic_mode: HeuristicMode::HeadingAware,
            require_target_angle: false,
            ..AStarConfig::default()
        };
        let straight_source = State::new(0, 0, 0);
        let straight_target = State::new(10, 0, 0);
        let straight_heuristic = SearchHeuristic::new(straight_target, &library, &config);
        assert_eq!(
            straight_heuristic.estimate(straight_source),
            distance_heuristic(straight_source, straight_target, 1.0)
        );

        let off_ray_target = State::new(10, 5, 0);
        let off_ray_heuristic = SearchHeuristic::new(off_ray_target, &library, &config);
        assert_eq!(
            off_ray_heuristic.estimate(straight_source),
            distance_heuristic(straight_source, off_ray_target, 1.0) + min_bend_cost
        );

        let target_angle_config = AStarConfig {
            heuristic_mode: HeuristicMode::HeadingAware,
            require_target_angle: true,
            ..AStarConfig::default()
        };
        let mismatched_angle_target = State::new(10, 0, 2);
        let mismatched_angle_heuristic =
            SearchHeuristic::new(mismatched_angle_target, &library, &target_angle_config);
        assert_eq!(
            mismatched_angle_heuristic.estimate(straight_source),
            distance_heuristic(straight_source, mismatched_angle_target, 1.0) + min_bend_cost
        );
    }

    #[test]
    fn heading_aware_heuristic_preserves_route_cost_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        assert_heading_aware_matches_distance(&map, &library, source, target, base_config);
    }

    #[test]
    fn heading_aware_heuristic_preserves_route_cost_on_simple_block() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 3);
        map.add_static_cell(4, 3);
        map.add_static_cell(5, 3);
        assert_heading_aware_matches_distance(
            &map,
            &primitive_library(),
            State::new(1, 3, 0),
            State::new(8, 3, 0),
            AStarConfig {
                enable_simple_routes: false,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        );
    }

    #[test]
    fn heading_aware_heuristic_preserves_route_cost_around_two_large_blocks() {
        let mut map = ObstacleMap::new(90, 50);
        for x in 18..=32 {
            for y in 8..=38 {
                map.add_static_cell(x, y);
            }
        }
        for x in 52..=66 {
            for y in 12..=42 {
                map.add_static_cell(x, y);
            }
        }
        assert_heading_aware_matches_distance(
            &map,
            &primitive_library_no45_bend2(),
            State::new(6, 25, 0),
            State::new(82, 25, 0),
            AStarConfig {
                max_iterations: 500_000,
                require_target_angle: false,
                enable_simple_routes: false,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        );
    }

    #[test]
    fn simple_route_blocked_falls_back_to_astar() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("A* fallback should route around blocked simple path");
        assert!(result.stats.expanded_states > 0);
    }

    #[test]
    fn simple_route_respects_opened_cells() {
        let mut map = ObstacleMap::new(10, 6);
        map.add_static_cell(1, 1);
        map.add_static_cell(5, 1);
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(1, 1));
        opened.insert(pack_xy(5, 1));

        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            Some(&opened),
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple route should allow opened endpoint cells");
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_route_handles_same_heading_turnaround() {
        let map = ObstacleMap::new(220, 140);
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 8,
            bend_radius_cells: 20,
            allow_45_degree_turns: false,
        });
        let mut config = AStarConfig::default();
        config.simple_route_max_offset_cells = 120;

        let result = try_simple_route_with_config(
            &map,
            &library,
            State::new(120, 70, 0),
            State::new(40, 40, 0),
            None,
            &config,
        )
        .expect("turnaround simple route should be generated");

        assert_eq!(result.states.first().copied(), Some(State::new(120, 70, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(40, 40, 0)));
    }

    #[test]
    fn simple_route_accepts_two_bend_z_with_minimum_middle_leg() {
        let map = ObstacleMap::new(360, 180);
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 0.5,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 20,
            allow_45_degree_turns: false,
        });
        let mut config = AStarConfig::default();
        config.simple_route_max_offset_cells = 240;

        let result = try_simple_route_with_config(
            &map,
            &library,
            State::new(121, 65, 0),
            State::new(301, 106, 0),
            None,
            &config,
        )
        .expect("two-bend Z route with a 2R+1 middle leg should be simple");

        assert_eq!(result.states.first().copied(), Some(State::new(121, 65, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(301, 106, 0)));
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_route_rejects_blocked_middle_cell() {
        let mut map = ObstacleMap::new(7, 1);
        map.add_static_cell(3, 0);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 0, 0),
            State::new(5, 0, 0),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn opened_cells_can_unblock_explicit_static_cells() {
        let mut map = ObstacleMap::new(7, 1);
        map.add_static_cell(3, 0);
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(3, 0));
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 0, 0),
            State::new(5, 0, 0),
            Some(&opened),
            &AStarConfig {
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        );
        let route = result.expect("opened cells should allow exact static overlap");
        assert_eq!(route.stats.expanded_states, 0);
    }

    #[test]
    fn source_and_target_cells_are_opened_implicitly() {
        let mut map = ObstacleMap::new(7, 1);
        map.add_static_cell(1, 0);
        map.add_static_cell(5, 0);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 0, 0),
            State::new(5, 0, 0),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        )
        .expect("blocked endpoints should be routed using implicit anchor opening");
        assert_eq!(result.states.first().copied(), Some(State::new(1, 0, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(5, 0, 0)));
    }

    #[test]
    fn disabling_simple_routes_preserves_old_behavior() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(5, 2, 0),
            None,
            &AStarConfig {
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("A* should still find the old straight route");
        assert!(!result.primitives.is_empty());
        assert!(result.stats.expanded_states > 0);
    }

    #[test]
    fn supports_relaxed_target_angle() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 2),
            None,
            &AStarConfig {
                require_target_angle: false,
                ..AStarConfig::default()
            },
        )
        .expect("route should allow non-matching terminal angle");

        let reached = result.states.last().copied().unwrap();
        assert_eq!(reached.x, 5);
        assert_eq!(reached.y, 1);
        assert_ne!(reached.angle, 2);
    }

    #[test]
    fn supports_allowed_target_angle_mask() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 2),
            None,
            &AStarConfig {
                require_target_angle: true,
                allowed_target_angles_mask: Some((1u8 << 0) | (1u8 << 1)),
                ..AStarConfig::default()
            },
        )
        .expect("route should use allowed-angle mask override");

        let reached = result.states.last().copied().unwrap();
        assert!(((1u8 << reached.angle) & ((1u8 << 0) | (1u8 << 1))) != 0);
    }

    #[test]
    fn rejects_negative_tolerance() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                target_tolerance_cells: -1,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn routing_window_padding_includes_target_tolerance() {
        let map = ObstacleMap::new(20, 20);
        let source = State::new(10, 10, 0);
        let target = State::new(10, 10, 0);
        let bounds = compute_routing_bounds(
            &map,
            source,
            target,
            &AStarConfig {
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                target_tolerance_cells: 2,
                ..AStarConfig::default()
            },
            0,
        )
        .unwrap();
        assert_eq!(bounds.min_x, 8);
        assert_eq!(bounds.max_x, 12);
        assert_eq!(bounds.min_y, 8);
        assert_eq!(bounds.max_y, 12);
    }

    #[test]
    fn dense_state_index_roundtrip() {
        let storage = DenseSearchStorage::new(
            RoutingBounds {
                min_x: 10,
                max_x: 14,
                min_y: 20,
                max_y: 22,
            },
            10_000,
        )
        .expect("storage should allocate");
        let state = State::new(12, 21, 5);
        let idx = storage
            .state_to_idx(state)
            .expect("state should map to index");
        assert_eq!(storage.idx_to_state(idx), state);
    }

    #[test]
    fn dense_state_outside_bounds_returns_none() {
        let storage = DenseSearchStorage::new(
            RoutingBounds {
                min_x: 2,
                max_x: 4,
                min_y: 2,
                max_y: 4,
            },
            10_000,
        )
        .expect("storage should allocate");
        assert!(storage.state_to_idx(State::new(1, 2, 0)).is_none());
        assert!(storage.state_to_idx(State::new(2, 5, 0)).is_none());
    }

    #[test]
    fn nonzero_offset_window_routes() {
        let map = ObstacleMap::new(30, 30);
        let library = primitive_library();
        let result = route_single_net_with_bounds(
            &map,
            &library,
            State::new(11, 11, 0),
            State::new(15, 11, 0),
            None,
            &AStarConfig::default(),
            Some(RoutingBounds {
                min_x: 10,
                max_x: 20,
                min_y: 10,
                max_y: 20,
            }),
            &mut RouteSearchStats::default(),
        )
        .expect("route in offset bounds should exist");
        assert_eq!(result.states.first().copied(), Some(State::new(11, 11, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(15, 11, 0)));
    }

    #[test]
    fn full_grid_fallback_uses_dense_storage() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        map.add_static_cell(4, 1);
        map.add_static_cell(5, 1);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(7, 1, 0),
            None,
            &AStarConfig {
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                routing_window_max_expansions: 0,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        )
        .expect("full-grid fallback should find a detour");
        assert_eq!(result.states.last().copied(), Some(State::new(7, 1, 0)));
        assert!(result.stats.used_full_grid_fallback);
    }

    #[test]
    fn duplicate_routing_window_expansions_are_skipped() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        map.add_static_cell(4, 1);
        map.add_static_cell(5, 1);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(7, 1, 0),
            None,
            &AStarConfig {
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                routing_window_max_expansions: 3,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        )
        .expect("full-grid fallback should find a detour");
        assert!(result.stats.used_full_grid_fallback);
        assert_eq!(result.stats.window_attempts, 2);
    }

    #[test]
    fn dense_state_limit_can_fail_attempt() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                max_dense_states: 8,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn dense_grid_mirrors_obstacles_and_opened_cells() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_static_cell(3, 2);
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(3, 2));
        let bounds = RoutingBounds {
            min_x: 2,
            max_x: 5,
            min_y: 1,
            max_y: 4,
        };

        let closed_grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, None, 1_000, false, false)
                .expect("grid");
        assert!(closed_grid.is_blocked(3, 2));

        let opened_grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, Some(&opened), 1_000, false, false)
                .expect("grid");
        assert!(!opened_grid.is_blocked(3, 2));
    }

    #[test]
    fn dense_grid_opened_cells_do_not_unblock_dynamic_obstacles() {
        let mut map = ObstacleMap::new(8, 6);
        assert!(map.commit_route(1, &[(3, 2)]));
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(3, 2));
        let bounds = RoutingBounds {
            min_x: 2,
            max_x: 5,
            min_y: 1,
            max_y: 4,
        };

        let opened_grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, Some(&opened), 1_000, false, false)
                .expect("grid");
        assert!(opened_grid.is_blocked(3, 2));
    }

    #[test]
    fn route_can_probe_while_ignoring_dynamic_obstacles() {
        let mut map = ObstacleMap::new(10, 5);
        assert!(map.commit_route(1, &[(4, 0), (4, 1), (4, 2), (4, 3), (4, 4)]));
        let library = primitive_library();
        let blocked = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(8, 2, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        );
        assert!(blocked.is_none());

        let probe = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(8, 2, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                enable_simple_routes: false,
                ignore_dynamic_obstacles: true,
                ..AStarConfig::default()
            },
        );
        assert!(probe.is_some());
    }

    #[test]
    fn dense_grid_out_of_bounds_is_blocked() {
        let map = ObstacleMap::new(8, 6);
        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 5,
                min_y: 1,
                max_y: 4,
            },
            None,
            1_000,
            false,
            false,
        )
        .expect("grid");
        assert!(grid.is_blocked(1, 1));
        assert!(grid.is_blocked(2, 5));
    }

    #[test]
    fn dense_grid_bitset_and_prefix_queries_preserve_opening_semantics() {
        let mut map = ObstacleMap::new(10, 8);
        map.add_static_cell(4, 3);
        map.add_static_cell(6, 3);
        assert!(map.commit_route(7, &[(5, 5)]));
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(4, 3));

        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 8,
                min_y: 1,
                max_y: 6,
            },
            Some(&opened),
            1_000,
            false,
            false,
        )
        .expect("grid");

        assert!(!grid.is_blocked(4, 3));
        assert!(grid.is_blocked(6, 3));
        assert!(grid.is_blocked(5, 5));
        assert_eq!(grid.blocked_count_in_rect(2, 8, 1, 6), Some(2));
        assert_eq!(grid.blocked_count_in_rect(4, 4, 3, 3), Some(0));
        assert_eq!(grid.blocked_count_in_rect(6, 6, 3, 3), Some(1));
        assert_eq!(grid.blocked_count_in_rect(1, 8, 1, 6), None);
    }

    #[test]
    fn dense_grid_segment_queries_are_exact_and_bounds_checked() {
        let mut map = ObstacleMap::new(10, 8);
        map.add_static_cell(5, 3);
        map.add_static_cell(7, 5);
        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 8,
                min_y: 1,
                max_y: 6,
            },
            None,
            1_000,
            false,
            false,
        )
        .expect("grid");

        assert!(grid.horizontal_segment_free(3, 2, 4));
        assert!(!grid.horizontal_segment_free(3, 2, 5));
        assert!(!grid.horizontal_segment_free(3, 8, 1));
        assert!(grid.vertical_segment_free(7, 1, 4));
        assert!(!grid.vertical_segment_free(7, 1, 5));
        assert!(!grid.vertical_segment_free(9, 1, 5));
    }

    #[test]
    fn dense_grid_primitive_footprint_checks() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_static_cell(4, 2);
        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 5,
                min_y: 1,
                max_y: 4,
            },
            None,
            1_000,
            false,
            false,
        )
        .expect("grid");
        let mut stats = RouteSearchStats::default();
        let rect_profile = FootprintCollisionProfile::from_footprint(&[(0, 0), (1, 0)]);
        let mut footprint_rejects_profile = RouteSearchStats::default();

        assert!(grid.primitive_footprint_free(2, 2, &[(0, 0), (1, 0)], &mut stats));
        assert!(!grid.primitive_footprint_free(3, 2, &[(0, 0), (1, 0)], &mut stats));
        assert!(!grid.primitive_footprint_free(5, 2, &[(0, 0), (1, 0)], &mut stats));
        assert!(grid.primitive_footprint_free_with_profile(
            2,
            2,
            &[(0, 0), (1, 0)],
            &rect_profile,
            &mut footprint_rejects_profile,
        ));
        assert_eq!(footprint_rejects_profile.primitive_footprint_rect_checks, 1);
    }

    #[test]
    fn dense_grid_non_rect_footprint_uses_bitset_point_checks() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_static_cell(4, 3);
        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 5,
                min_y: 1,
                max_y: 4,
            },
            None,
            1_000,
            false,
            false,
        )
        .expect("grid");
        let footprint = &[(0, 0), (1, 0), (1, 1)];
        let profile = FootprintCollisionProfile::from_footprint(footprint);
        let mut stats = RouteSearchStats::default();

        assert!(!profile.is_full_rect);
        assert!(grid.primitive_footprint_free_with_profile(2, 2, footprint, &profile, &mut stats,));
        assert!(!grid.primitive_footprint_free_with_profile(3, 2, footprint, &profile, &mut stats,));
        assert!(!grid.primitive_footprint_free_with_profile(5, 2, footprint, &profile, &mut stats,));
        assert_eq!(stats.primitive_footprint_rect_checks, 0);
    }

    #[test]
    fn dense_grid_rect_profile_matches_opening_and_boundaries() {
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(3, 2));
        let mut map = ObstacleMap::new(8, 6);
        map.add_static_cell(3, 2);

        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 5,
                min_y: 1,
                max_y: 4,
            },
            Some(&opened),
            1_000,
            false,
            false,
        )
        .expect("grid");
        let footprint = &[(0, 0), (1, 0), (2, 0)];
        let profile = FootprintCollisionProfile::from_footprint(footprint);
        let mut stats = RouteSearchStats::default();

        assert!(grid.primitive_footprint_free_with_profile(2, 2, footprint, &profile, &mut stats,));
        assert!(!grid.primitive_footprint_free_with_profile(1, 2, footprint, &profile, &mut stats,));
        assert!(!grid.primitive_footprint_free_with_profile(4, 2, footprint, &profile, &mut stats,));
        assert!(!grid.primitive_footprint_free_with_profile(5, 2, footprint, &profile, &mut stats,));
    }

    #[test]
    fn dense_grid_obstacle_profile_counters_increase_during_astar() {
        let map = ObstacleMap::new(12, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(8, 2, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("route should exist with direct A* run");

        assert!(result.stats.primitive_footprint_checks > 0);
        assert!(result.stats.generated_neighbors > 0);
        assert!(result.stats.heap_pushes > 0);
        assert!(result.stats.heap_pops > 0);
        assert_eq!(
            result.stats.obstacle_clearance_checks,
            result.stats.primitive_footprint_checks
        );
        assert_eq!(result.stats.neighbor_generation_time_us, 0);
        assert_eq!(result.stats.heap_operation_time_us, 0);
        assert_eq!(result.stats.legality_check_time_us, 0);
        assert_eq!(result.stats.reconstruction_time_us, 0);
        assert!(result.stats.primitive_footprint_rect_checks > 0);
        assert_eq!(
            result
                .stats
                .primitive_generated_by_class
                .iter()
                .sum::<usize>(),
            result.stats.generated_neighbors
        );
        assert_eq!(
            result
                .stats
                .primitive_footprint_checks_by_class
                .iter()
                .sum::<usize>(),
            result.stats.primitive_footprint_checks
        );
        assert!(
            result
                .stats
                .primitive_accepted_by_class
                .iter()
                .sum::<usize>()
                > 0
        );
        assert_eq!(result.stats.dense_grid_cells, 0);
    }

    #[test]
    fn detailed_timing_is_opt_in() {
        let map = ObstacleMap::new(12, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(8, 2, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                enable_simple_routes: false,
                collect_detailed_timing: true,
                ..AStarConfig::default()
            },
        )
        .expect("route should exist with direct A* run");

        assert!(result.stats.heap_pushes > 0);
        assert!(result.stats.heap_pops > 0);
        assert!(
            result.stats.neighbor_generation_time_us
                + result.stats.heap_operation_time_us
                + result.stats.legality_check_time_us
                + result.stats.reconstruction_time_us
                > 0
        );
    }

    #[test]
    fn dense_obstacle_limit_can_fail_attempt() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                max_dense_obstacle_cells: 10,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn reconstruct_fails_cleanly_when_parent_missing() {
        let bounds = RoutingBounds {
            min_x: 0,
            max_x: 7,
            min_y: 0,
            max_y: 7,
        };
        let storage = DenseSearchStorage::new(bounds, 10_000).expect("storage");
        let lib = primitive_library();
        let source_idx = storage
            .state_to_idx(State::new(1, 1, 0))
            .expect("source index");
        let reached_idx = storage
            .state_to_idx(State::new(2, 1, 0))
            .expect("reached index");

        let result = reconstruct_route_dense(
            source_idx,
            reached_idx,
            State::new(2, 1, 0),
            &lib,
            1.0,
            RouteSearchStats::default(),
            &storage,
        );
        assert!(result.is_none());
    }
}
