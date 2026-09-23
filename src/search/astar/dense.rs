//! Dense per-search grid storage (`DenseRoutingGrid`, `DenseSearchStorage`,
//! `DenseBitset`, `DenseDynamicCoreOwnerGrid`) and the JPS4 shortcut search
//! that the plain kernel tries before falling back to full A*. Moved out of
//! `src/astar.rs` (Milestone 3, Slice 2); pure code motion, no behaviour
//! change.

use crate::obstacle_map::{pack_xy, unpack_xy, CellKey, GridRect, NetId, ObstacleMap};
use crate::primitives::PrimitiveLibrary;
use crate::search::astar::config::{
    AStarConfig, HeapTieBreaker, BITSET_WORD_BITS, JPS4_DIRECTIONS, NO_DYNAMIC_OWNER,
    NO_GENERATION, NO_PARENT,
};
use crate::search::astar::diagnostics::{
    search_timed_out, should_check_timeout, trace_search_timeout,
};
use crate::search::astar::expansion::FootprintCollisionProfile;
use crate::search::astar::kernel::{heap_tie_score, next_search_generation, OpenEntry};
use crate::search::astar::window::{window_area, RoutingBounds};
use crate::search::geometry::{compress_grid_waypoints, polyline_self_intersects};
use crate::search::state::{RouteResult, RouteSearchStats, State};
use rustc_hash::FxHashSet;
use std::collections::BinaryHeap;
use std::mem::size_of;
use std::time::Instant;

#[derive(Clone, Debug)]
pub(crate) struct Jps4Eligibility {
    pub(crate) eligible: bool,
    pub(crate) reason: &'static str,
}

pub(crate) struct DenseSearchStorage {
    pub(crate) bounds: RoutingBounds,
    pub(crate) width_usize: usize,
    pub(crate) g_costs: Vec<f64>,
    pub(crate) best_generation: Vec<u32>,
    pub(crate) parent_idx: Vec<u32>,
    pub(crate) parent_primitive: Vec<u16>,
    pub(crate) closed: DenseBitset,
}

impl DenseSearchStorage {
    pub(crate) fn new(bounds: RoutingBounds, max_dense_states: usize) -> Option<Self> {
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

    pub(crate) fn state_count(&self) -> usize {
        self.g_costs.len()
    }

    pub(crate) fn allocated_bytes(&self) -> usize {
        self.g_costs.len() * size_of::<f64>()
            + self.best_generation.len() * size_of::<u32>()
            + self.parent_idx.len() * size_of::<u32>()
            + self.parent_primitive.len() * size_of::<u16>()
            + self.closed.allocated_bytes()
    }

    pub(crate) fn state_to_idx(&self, state: State) -> Option<usize> {
        if state.angle >= 8 || !self.bounds.contains(state.x, state.y) {
            return None;
        }
        Some(self.in_bounds_state_to_idx(state))
    }

    #[inline]
    pub(crate) fn in_bounds_state_to_idx(&self, state: State) -> usize {
        self.in_bounds_parts_to_idx(state.x, state.y, state.angle)
    }

    #[inline]
    pub(crate) fn in_bounds_parts_to_idx(&self, x: i32, y: i32, angle: u8) -> usize {
        debug_assert!(angle < 8);
        debug_assert!(self.bounds.contains(x, y));
        let local_x = (x - self.bounds.min_x) as usize;
        let local_y = (y - self.bounds.min_y) as usize;
        ((local_y * self.width_usize) + local_x) << 3 | usize::from(angle)
    }

    #[inline]
    pub(crate) fn idx_to_state(&self, idx: usize) -> State {
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

pub(crate) struct DenseBitset {
    pub(crate) bits: Vec<u64>,
}

impl DenseBitset {
    pub(crate) fn new(cell_count: usize) -> Option<Self> {
        let words = cell_count
            .checked_add(BITSET_WORD_BITS - 1)?
            .checked_div(BITSET_WORD_BITS)?;
        Some(Self {
            bits: vec![0; words],
        })
    }

    #[inline]
    pub(crate) fn set(&mut self, idx: usize) -> Option<()> {
        let word_idx = idx.checked_div(BITSET_WORD_BITS)?;
        let bit_idx = idx.checked_rem(BITSET_WORD_BITS)?;
        let word = self.bits.get_mut(word_idx)?;
        *word |= 1u64 << bit_idx;
        Some(())
    }

    #[inline]
    pub(crate) fn get(&self, idx: usize) -> bool {
        let word_idx = idx / BITSET_WORD_BITS;
        let bit_idx = idx % BITSET_WORD_BITS;
        self.bits
            .get(word_idx)
            .map(|word| (word & (1u64 << bit_idx)) != 0)
            .unwrap_or(true)
    }

    pub(crate) fn allocated_bytes(&self) -> usize {
        self.bits.len() * size_of::<u64>()
    }
}

pub(crate) struct DenseRoutingGrid {
    pub(crate) bounds: RoutingBounds,
    pub(crate) width: i32,
    pub(crate) height: i32,
    pub(crate) blocked_bits: DenseBitset,
    pub(crate) blocked_prefix: Option<Vec<u32>>,
    pub(crate) history: Option<Vec<u32>>,
    pub(crate) history_prefix: Option<Vec<u64>>,
    pub(crate) congestion: Option<Vec<u32>>,
    pub(crate) congestion_prefix: Option<Vec<u64>>,
    pub(crate) blocked_count: usize,
    pub(crate) build_time_us: u128,
}

pub(crate) struct DenseDynamicCoreOwnerGrid {
    pub(crate) bounds: RoutingBounds,
    pub(crate) width_usize: usize,
    pub(crate) owners: Vec<NetId>,
}

impl DenseDynamicCoreOwnerGrid {
    pub(crate) fn from_obstacle_map(
        obstacle_map: &ObstacleMap,
        bounds: RoutingBounds,
    ) -> Option<Self> {
        let width = bounds.max_x.checked_sub(bounds.min_x)?.checked_add(1)?;
        let height = bounds.max_y.checked_sub(bounds.min_y)?.checked_add(1)?;
        if width <= 0 || height <= 0 {
            return None;
        }
        let width_usize = usize::try_from(width).ok()?;
        let height_usize = usize::try_from(height).ok()?;
        let cell_count = width_usize.checked_mul(height_usize)?;
        let mut owners = vec![NO_DYNAMIC_OWNER; cell_count];

        for (net_id, cells) in obstacle_map.net_core_route_entries() {
            if net_id == NO_DYNAMIC_OWNER {
                continue;
            }
            for &key in cells {
                let (x, y) = unpack_xy(key);
                if !bounds.contains(x, y) {
                    continue;
                }
                let Some(idx) = Self::idx_of_bounds(bounds, width_usize, x, y) else {
                    continue;
                };
                if owners[idx] == NO_DYNAMIC_OWNER {
                    owners[idx] = net_id;
                }
            }
        }

        Some(Self {
            bounds,
            width_usize,
            owners,
        })
    }

    #[inline]
    pub(crate) fn owner_at(&self, x: i32, y: i32) -> Option<NetId> {
        let idx = Self::idx_of_bounds(self.bounds, self.width_usize, x, y)?;
        let owner = self.owners[idx];
        (owner != NO_DYNAMIC_OWNER).then_some(owner)
    }

    #[inline]
    pub(crate) fn idx_of_bounds(
        bounds: RoutingBounds,
        width_usize: usize,
        x: i32,
        y: i32,
    ) -> Option<usize> {
        if !bounds.contains(x, y) {
            return None;
        }
        let local_x = usize::try_from(x.checked_sub(bounds.min_x)?).ok()?;
        let local_y = usize::try_from(y.checked_sub(bounds.min_y)?).ok()?;
        local_y.checked_mul(width_usize)?.checked_add(local_x)
    }
}

pub(crate) fn intersect_bounds_rect(bounds: RoutingBounds, rect: GridRect) -> Option<GridRect> {
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
    pub(crate) fn from_obstacle_map(
        obstacle_map: &ObstacleMap,
        bounds: RoutingBounds,
        opened_cells: Option<&FxHashSet<CellKey>>,
        max_dense_obstacle_cells: usize,
        ignore_dynamic_obstacles: bool,
        build_history: bool,
        build_congestion: bool,
    ) -> Option<Self> {
        Self::from_obstacle_map_with_dynamic_expansion(
            obstacle_map,
            bounds,
            opened_cells,
            max_dense_obstacle_cells,
            ignore_dynamic_obstacles,
            build_history,
            build_congestion,
            0,
            None,
        )
    }

    pub(crate) fn from_obstacle_map_with_dynamic_expansion(
        obstacle_map: &ObstacleMap,
        bounds: RoutingBounds,
        opened_cells: Option<&FxHashSet<CellKey>>,
        max_dense_obstacle_cells: usize,
        ignore_dynamic_obstacles: bool,
        build_history: bool,
        build_congestion: bool,
        dynamic_expansion_radius_cells: i32,
        dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
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
        let mut congestion = if build_congestion {
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
        let dynamic_clearance_exempt_contains = |x: i32, y: i32| -> bool {
            dynamic_clearance_exempt_cells
                .map(|cells| cells.contains(&pack_xy(x, y)))
                .unwrap_or(false)
                && !obstacle_map.is_dynamic_core_blocked(x, y)
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
            let dynamic_expansion_radius_cells = dynamic_expansion_radius_cells.max(0);
            for key in obstacle_map.dynamic_obstacle_keys() {
                let (x, y) = unpack_xy(key);
                for dx in -dynamic_expansion_radius_cells..=dynamic_expansion_radius_cells {
                    for dy in -dynamic_expansion_radius_cells..=dynamic_expansion_radius_cells {
                        let Some(nx) = x.checked_add(dx) else {
                            continue;
                        };
                        let Some(ny) = y.checked_add(dy) else {
                            continue;
                        };
                        if dynamic_clearance_exempt_contains(nx, ny) {
                            continue;
                        }
                        let Some(idx) = local_idx(nx, ny) else {
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
        if let Some(congestion) = congestion.as_mut() {
            for (key, cost) in obstacle_map.congestion_entries() {
                let (x, y) = unpack_xy(key);
                let Some(idx) = local_idx(x, y) else {
                    continue;
                };
                congestion[idx] = cost;
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
        let mut congestion_prefix = if build_congestion {
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
            let mut congestion_row_sum = 0u64;
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
                if let Some(congestion) = congestion.as_ref() {
                    congestion_row_sum =
                        congestion_row_sum.saturating_add(u64::from(congestion[blocked_idx]));
                }
                let prefix_idx = prefix_row.checked_add(x_idx + 1)?;
                let above = blocked_prefix[prefix_above + x_idx + 1];
                blocked_prefix[prefix_idx] = above.saturating_add(row_sum);
                if let Some(history_prefix) = history_prefix.as_mut() {
                    let history_above = history_prefix[prefix_above + x_idx + 1];
                    history_prefix[prefix_idx] = history_above.saturating_add(history_row_sum);
                }
                if let Some(congestion_prefix) = congestion_prefix.as_mut() {
                    let congestion_above = congestion_prefix[prefix_above + x_idx + 1];
                    congestion_prefix[prefix_idx] =
                        congestion_above.saturating_add(congestion_row_sum);
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
            congestion,
            congestion_prefix,
            blocked_count,
            build_time_us: start.elapsed().as_micros(),
        })
    }

    #[inline]
    pub(crate) fn contains(&self, x: i32, y: i32) -> bool {
        self.bounds.contains(x, y)
    }

    #[inline]
    pub(crate) fn is_blocked(&self, x: i32, y: i32) -> bool {
        match self.idx_of(x, y) {
            Some(idx) => self.blocked_bits.get(idx),
            None => true,
        }
    }

    #[inline]
    pub(crate) fn blocked_count(&self) -> usize {
        self.blocked_count
    }

    #[inline]
    pub(crate) fn build_time_us(&self) -> u128 {
        self.build_time_us
    }

    #[inline]
    pub(crate) fn blocked_count_in_local_rect(
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
    pub(crate) fn blocked_count_in_rect(
        &self,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
    ) -> Option<u32> {
        let local_min_x = min_x.checked_sub(self.bounds.min_x)?;
        let local_max_x = max_x.checked_sub(self.bounds.min_x)?;
        let local_min_y = min_y.checked_sub(self.bounds.min_y)?;
        let local_max_y = max_y.checked_sub(self.bounds.min_y)?;
        self.blocked_count_in_local_rect(local_min_x, local_max_x, local_min_y, local_max_y)
    }

    #[inline]
    pub(crate) fn rect_free(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> bool {
        matches!(
            self.blocked_count_in_rect(min_x, max_x, min_y, max_y),
            Some(0)
        )
    }

    #[inline]
    pub(crate) fn horizontal_segment_free(&self, y: i32, x0: i32, x1: i32) -> bool {
        let min_x = x0.min(x1);
        let max_x = x0.max(x1);
        self.rect_free(min_x, max_x, y, y)
    }

    #[inline]
    pub(crate) fn vertical_segment_free(&self, x: i32, y0: i32, y1: i32) -> bool {
        let min_y = y0.min(y1);
        let max_y = y0.max(y1);
        self.rect_free(x, x, min_y, max_y)
    }

    #[inline]
    pub(crate) fn primitive_footprint_free_with_profile(
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
            stats.primitive_footprint_cells_tested += profile.cell_count;
            if !profile.horizontal_runs.is_empty() {
                for &(dy, min_dx, max_dx) in &profile.horizontal_runs {
                    let y = match origin_y.checked_add(dy) {
                        Some(y) => y,
                        None => return false,
                    };
                    let min_x = match origin_x.checked_add(min_dx) {
                        Some(x) => x,
                        None => return false,
                    };
                    let max_x = match origin_x.checked_add(max_dx) {
                        Some(x) => x,
                        None => return false,
                    };
                    if !self.horizontal_segment_free(y, min_x, max_x) {
                        return false;
                    }
                }
                return true;
            }
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
    pub(crate) fn relative_offsets_free_with_profile(
        &self,
        origin_x: i32,
        origin_y: i32,
        offsets: &[(i32, i32)],
        profile: &FootprintCollisionProfile,
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
            if rect_min_y == rect_max_y {
                self.horizontal_segment_free(rect_min_y, rect_min_x, rect_max_x)
            } else if rect_min_x == rect_max_x {
                self.vertical_segment_free(rect_min_x, rect_min_y, rect_max_y)
            } else {
                self.rect_free(rect_min_x, rect_max_x, rect_min_y, rect_max_y)
            }
        } else if !profile.horizontal_runs.is_empty() {
            for &(dy, min_dx, max_dx) in &profile.horizontal_runs {
                let y = match origin_y.checked_add(dy) {
                    Some(y) => y,
                    None => return false,
                };
                let min_x = match origin_x.checked_add(min_dx) {
                    Some(x) => x,
                    None => return false,
                };
                let max_x = match origin_x.checked_add(max_dx) {
                    Some(x) => x,
                    None => return false,
                };
                if !self.horizontal_segment_free(y, min_x, max_x) {
                    return false;
                }
            }
            true
        } else {
            for (dx, dy) in offsets.iter().copied() {
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

    #[cfg(test)]
    #[inline]
    pub(crate) fn primitive_footprint_free(
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
    pub(crate) fn idx_of(&self, x: i32, y: i32) -> Option<usize> {
        if !self.contains(x, y) {
            return None;
        }
        let local_x = usize::try_from(x.checked_sub(self.bounds.min_x)?).ok()?;
        let local_y = usize::try_from(y.checked_sub(self.bounds.min_y)?).ok()?;
        let width = usize::try_from(self.width).ok()?;
        local_y.checked_mul(width)?.checked_add(local_x)
    }
}

pub(crate) fn evaluate_jps4_eligibility(
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

pub(crate) fn primitive_library_is_plain_jps4_grid(primitives: &PrimitiveLibrary) -> bool {
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

pub(crate) fn route_single_net_jps4(
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
    let Some(dense_grid) = DenseRoutingGrid::from_obstacle_map(
        obstacle_map,
        bounds,
        port_open_cells,
        config.max_dense_obstacle_cells,
        config.ignore_dynamic_obstacles,
        false,
        false,
    ) else {
        if config.diagnostics.search_failure_diag {
            eprintln!(
                "search-failure kind=dense_grid_not_built window=[{}..{}]x[{}..{}] area_cells={} max_dense_obstacle_cells={} source=({},{}) target=({},{})",
                bounds.min_x,
                bounds.max_x,
                bounds.min_y,
                bounds.max_y,
                window_area(bounds),
                config.max_dense_obstacle_cells,
                source.x,
                source.y,
                target.x,
                target.y
            );
        }
        return None;
    };
    stats.dense_grid_cells = dense_grid.blocked_count();
    stats.dense_grid_build_time_us = dense_grid.build_time_us();
    stats.window_attempts = 1;

    if dense_grid.is_blocked(source.x, source.y) || dense_grid.is_blocked(target.x, target.y) {
        if config.diagnostics.search_failure_diag {
            eprintln!(
                "search-failure kind=endpoint_blocked source=({},{}) source_blocked={} source_static={} source_dynamic={} target=({},{}) target_blocked={} target_static={} target_dynamic={}",
                source.x,
                source.y,
                dense_grid.is_blocked(source.x, source.y),
                obstacle_map.is_static_blocked(source.x, source.y),
                obstacle_map.is_dynamic_blocked(source.x, source.y),
                target.x,
                target.y,
                dense_grid.is_blocked(target.x, target.y),
                obstacle_map.is_static_blocked(target.x, target.y),
                obstacle_map.is_dynamic_blocked(target.x, target.y)
            );
        }
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
    let search_timeout_start = (config.max_search_time_ms > 0).then(Instant::now);
    while let Some(entry) = open_set.pop() {
        stats.heap_pops += 1;
        iterations += 1;
        if iterations > config.max_iterations {
            return None;
        }
        if should_check_timeout(iterations, config)
            && search_timed_out(search_timeout_start.as_ref(), config)
        {
            trace_search_timeout("jps4", config, &stats, iterations, open_set.len());
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

pub(crate) fn jps4_jump(
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

pub(crate) fn jps4_has_forced_neighbor(
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

pub(crate) fn jps4_has_obstacle_corner_opening(
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

pub(crate) fn jps4_heuristic(x: i32, y: i32, target: (i32, i32), grid_size_um: f64) -> f64 {
    f64::from((target.0 - x).abs() + (target.1 - y).abs()) * grid_size_um
}

pub(crate) fn jps4_idx_to_xy(idx: usize, dense_grid: &DenseRoutingGrid) -> Option<(i32, i32)> {
    let width = usize::try_from(dense_grid.width).ok()?;
    let local_x = i32::try_from(idx % width).ok()?;
    let local_y = i32::try_from(idx / width).ok()?;
    Some((
        dense_grid.bounds.min_x + local_x,
        dense_grid.bounds.min_y + local_y,
    ))
}

pub(crate) fn reconstruct_jps4_route(
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
    if polyline_self_intersects(&compressed_waypoints) {
        return None;
    }
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

pub(crate) fn jps4_angle_between(a: (i32, i32), b: (i32, i32)) -> Option<u8> {
    match ((b.0 - a.0).signum(), (b.1 - a.1).signum()) {
        (1, 0) => Some(0),
        (0, 1) => Some(2),
        (-1, 0) => Some(4),
        (0, -1) => Some(6),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::primitives::Primitive;
    use crate::primitives::PrimitiveGeometry;
    use crate::search::astar::crossing_rules::{CrossingSearchConfig, CrossingSearchPartner};
    use crate::search::astar::expansion::compact_diagonal_halo_cells;
    use crate::search::astar::route_single_net_with_config;
    use crate::search::astar::simple::{
        try_simple_route_with_config, try_simple_route_with_dynamic_expansion_config,
    };
    use crate::search::test_support::*;
    use std::collections::VecDeque;

    #[test]
    fn simple_dynamic_expansion_overlay_matches_materialized_map() {
        let mut map = ObstacleMap::new(24, 12);
        assert!(map.commit_route_with_clearance_overlap(1, &[(9, 4)], &[(9, 4)], &[],));
        let mut expanded = map.clone_with_expanded_dynamic_obstacles(2);
        let exemptions = vec![(9, 2)];
        expanded.clear_dynamic_clearance_in_cells(&exemptions);
        let exemption_keys = pack_cells_for_test(&exemptions);
        let library = primitive_library_no45_bend1();
        let source = State::new(1, 1, 0);
        let target = State::new(18, 1, 0);
        let config = AStarConfig {
            require_target_angle: true,
            ..AStarConfig::default()
        };

        let materialized =
            try_simple_route_with_config(&expanded, &library, source, target, None, &config);
        let overlay = try_simple_route_with_dynamic_expansion_config(
            &map,
            &library,
            source,
            target,
            None,
            &config,
            2,
            Some(&exemption_keys),
        );

        assert_eq!(overlay.is_some(), materialized.is_some());
        if let (Some(overlay), Some(materialized)) = (overlay, materialized) {
            assert_eq!(overlay.primitives, materialized.primitives);
            assert_eq!(overlay.cells, materialized.cells);
            assert_eq!(overlay.reached_target, materialized.reached_target);
        }
    }

    #[test]
    fn dense_dynamic_expansion_overlay_matches_materialized_map() {
        let mut map = ObstacleMap::new(40, 16);
        assert!(map.commit_route_with_clearance_overlap(
            1,
            &[(16, 7), (17, 7), (18, 7)],
            &[(16, 7), (17, 7), (18, 7)],
            &[],
        ));
        let mut expanded = map.clone_with_expanded_dynamic_obstacles(2);
        let exemptions = vec![(16, 5)];
        expanded.clear_dynamic_clearance_in_cells(&exemptions);
        let exemption_keys = pack_cells_for_test(&exemptions);
        let library = primitive_library_no45_bend1();
        let source = State::new(3, 7, 0);
        let target = State::new(34, 7, 0);
        let config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };

        let materialized =
            route_single_net_with_config(&expanded, &library, source, target, None, &config)
                .expect("materialized expanded map should route");
        let overlay = test_route_single_net_with_dynamic_expansion_config(
            &map,
            &library,
            source,
            target,
            None,
            &config,
            2,
            Some(&exemption_keys),
        )
        .expect("overlay expanded map should route");

        assert_eq!(overlay.reached_target, materialized.reached_target);
        assert!((overlay.total_cost - materialized.total_cost).abs() < 1.0e-9);
    }

    #[test]
    fn zero_radius_dynamic_overlay_matches_materialized_simple_map() {
        let mut map = ObstacleMap::new(24, 12);
        assert!(map.commit_route_with_clearance_overlap(1, &[(9, 4)], &[(9, 4)], &[]));
        let mut materialized_map = map.clone();
        let exemptions = vec![(9, 4)];
        materialized_map.clear_dynamic_clearance_in_cells(&exemptions);
        let exemption_keys = pack_cells_for_test(&exemptions);
        let library = primitive_library_no45_bend1();
        let source = State::new(1, 4, 0);
        let target = State::new(18, 4, 0);
        let config = AStarConfig {
            require_target_angle: true,
            ..AStarConfig::default()
        };

        let materialized = try_simple_route_with_config(
            &materialized_map,
            &library,
            source,
            target,
            None,
            &config,
        );
        let overlay = try_simple_route_with_dynamic_expansion_config(
            &map,
            &library,
            source,
            target,
            None,
            &config,
            0,
            Some(&exemption_keys),
        );

        assert_eq!(overlay.is_some(), materialized.is_some());
        if let (Some(overlay), Some(materialized)) = (overlay, materialized) {
            assert_eq!(overlay.primitives, materialized.primitives);
            assert_eq!(overlay.cells, materialized.cells);
            assert_eq!(overlay.reached_target, materialized.reached_target);
        }
    }

    #[test]
    fn zero_radius_dynamic_overlay_matches_materialized_dense_map() {
        let mut map = ObstacleMap::new(40, 16);
        assert!(map.commit_route_with_clearance_overlap(
            1,
            &[(16, 7), (17, 7), (18, 7)],
            &[(16, 7), (17, 7), (18, 7)],
            &[],
        ));
        let mut materialized_map = map.clone();
        let exemptions = vec![(17, 7)];
        materialized_map.clear_dynamic_clearance_in_cells(&exemptions);
        let exemption_keys = pack_cells_for_test(&exemptions);
        let library = primitive_library_no45_bend1();
        let source = State::new(3, 7, 0);
        let target = State::new(34, 7, 0);
        let config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };

        let materialized = route_single_net_with_config(
            &materialized_map,
            &library,
            source,
            target,
            None,
            &config,
        )
        .expect("materialized zero-radius map should route");
        let overlay = test_route_single_net_with_dynamic_expansion_config(
            &map,
            &library,
            source,
            target,
            None,
            &config,
            0,
            Some(&exemption_keys),
        )
        .expect("zero-radius overlay map should route");

        assert_eq!(overlay.reached_target, materialized.reached_target);
        assert!((overlay.total_cost - materialized.total_cost).abs() < 1.0e-9);
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
                diagnostics: KernelDiagnostics::default(),
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
            DenseRoutingGrid::from_obstacle_map(&map, bounds, None, 1_000, false, false, false)
                .expect("grid");
        assert!(closed_grid.is_blocked(3, 2));

        let opened_grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            bounds,
            Some(&opened),
            1_000,
            false,
            false,
            false,
        )
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

        let opened_grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            bounds,
            Some(&opened),
            1_000,
            false,
            false,
            false,
        )
        .expect("grid");
        assert!(opened_grid.is_blocked(3, 2));
    }

    #[test]
    fn plain_search_rejects_diagonal_adjacent_to_committed_diagonal() {
        // A committed 45-degree route along (k, k). A plain (crossings
        // disabled) search whose straight-line answer is the *adjacent*
        // diagonal (k, k + 1) shares no cell with it, but the two realized
        // waveguides would overlap (bend deviation > cell gap). The compact
        // diagonal halo must keep the search off that adjacent diagonal.
        let mut map = ObstacleMap::new(40, 40);
        let committed: Vec<(i32, i32)> = (10..26).map(|k| (k, k)).collect();
        assert!(map.commit_route(1, &committed));
        let committed_set: FxHashSet<(i32, i32)> = committed.iter().copied().collect();
        let library = primitive_library();
        let mut stats = RouteSearchStats::default();
        let route = test_route_single_net_with_config_reporting_stats(
            &map,
            &library,
            State::new(2, 3, 1),
            State::new(32, 33, 1),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                use_routing_window: false,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
            &mut stats,
        )
        .expect("a route away from the committed diagonal must exist");
        assert!(stats.diagonal_halo_contacts > 0);
        for pair in route.cells.windows(2) {
            let (start, end) = (pair[0], pair[1]);
            let dx = end.0 - start.0;
            let dy = end.1 - start.1;
            if dx == 0 || dy == 0 {
                continue;
            }
            for halo in compact_diagonal_halo_cells(start, end, dx.signum(), dy.signum()) {
                assert!(
                    !committed_set.contains(&halo),
                    "diagonal step {start:?}->{end:?} runs adjacent to the committed diagonal"
                );
            }
        }
    }

    #[test]
    fn crossing_search_rejects_parallel_diagonal_adjacent_to_committed_diagonal() {
        // Crossing-enabled mirror of
        // `plain_search_rejects_diagonal_adjacent_to_committed_diagonal`: a
        // parallel halo contact produces no crossing event, so the crossing
        // kernel must reject the move exactly like the plain kernel does
        // (multiportmmi_8x8's n_13/n_14 physically overlapped this way at
        // heuristic weight 1.0). Perpendicular halo contacts still legalize
        // as crossings and stay allowed.
        let mut map = ObstacleMap::new(40, 40);
        let committed: Vec<(i32, i32)> = (10..26).map(|k| (k, k)).collect();
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            1,
            &committed,
            &committed,
            &[],
            &FxHashSet::default()
        ));
        let committed_set: FxHashSet<(i32, i32)> = committed.iter().copied().collect();
        let library = primitive_library();
        let crossing = CrossingSearchConfig {
            diagnostics: KernelDiagnostics::default(),
            net_id: 2,
            partners: vec![CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(10, 10), (25, 25)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            }],
            min_straight_cells: 0,
            crossing_half_size_cells: 0,
            bend_runout_cells: 0,
            crossing_loss: 3.0,
            require_all_partners: false,
            terminal_bump_guard: None,
        };
        let (route, _stats) = test_route_single_net_with_collision_crossing_config_with_stats(
            &map,
            &library,
            State::new(2, 3, 1),
            State::new(32, 33, 1),
            None,
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                use_routing_window: false,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
            0,
            None,
            &crossing,
        );
        let route = route.expect("a route away from the committed diagonal must exist");
        for pair in route.cells.windows(2) {
            let (start, end) = (pair[0], pair[1]);
            let dx = end.0 - start.0;
            let dy = end.1 - start.1;
            if dx == 0 || dy == 0 || dx.signum() != dy.signum() {
                // Straights carry no halo; perpendicular diagonals may
                // legalize a halo contact as a real crossing.
                continue;
            }
            for halo in compact_diagonal_halo_cells(start, end, dx.signum(), dy.signum()) {
                assert!(
                    !committed_set.contains(&halo),
                    "parallel diagonal step {start:?}->{end:?} runs adjacent to the committed diagonal"
                );
            }
        }
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
                diagnostics: KernelDiagnostics::default(),
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
                diagnostics: KernelDiagnostics::default(),
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
                diagnostics: KernelDiagnostics::default(),
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
                diagnostics: KernelDiagnostics::default(),
                use_routing_window: false,
                max_dense_obstacle_cells: 10,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }
}
