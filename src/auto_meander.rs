//! Obstacle-aware analytic meander search.
//!
//! This module layers grid occupancy checks and search over the lower-level
//! analytic meander planner in `meander.rs`.

use std::borrow::Cow;
use std::time::Instant;

use rustc_hash::FxHashSet;

#[cfg(test)]
use crate::geometry_realization::generate_waveguide_polygon;
use crate::geometry_realization::{
    centerline_length_um, distance, push_physical_if_different, GeometryError, GeometryGridSpec,
    EPS,
};
use crate::meander::{
    plan_analytic_meander, plan_fill_box_multi_bump_footprint, AnalyticMeanderConfig,
    AnalyticMeanderPlan, MeanderBox, MeanderPlanningMode, MeanderSide, PhysicalPoint,
    StraightSegment,
};
use crate::obstacle_map::{unpack_xy, CellKey, ObstacleMap};
use crate::static_obstacle_builder::floor_snap_to_grid;
#[cfg(test)]
use crate::static_obstacle_builder::{rasterize_polygon, StaticGridSpec};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeanderGridRect {
    pub min_x: i32,
    pub max_x: i32,
    pub min_y: i32,
    pub max_y: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DenseOccupancyPrefix {
    width: i32,
    height: i32,
    prefix: Vec<u32>,
}

impl DenseOccupancyPrefix {
    pub fn from_blocked_keys(width: i32, height: i32, blocked_keys: &FxHashSet<CellKey>) -> Self {
        let w = usize::try_from(width).unwrap_or(0);
        let h = usize::try_from(height).unwrap_or(0);
        let mut occupancy = vec![0u8; w.saturating_mul(h)];

        for &key in blocked_keys {
            let (x, y) = unpack_xy(key);
            if x >= 0 && y >= 0 && x < width && y < height {
                let xu = usize::try_from(x).expect("non-negative x fits usize");
                let yu = usize::try_from(y).expect("non-negative y fits usize");
                occupancy[yu * w + xu] = 1;
            }
        }

        Self::from_occupancy(width, height, &occupancy)
    }

    pub fn from_obstacle_map(
        obstacle_map: &ObstacleMap,
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> Self {
        Self::from_obstacle_map_with_overrides(obstacle_map, opened_cells, None)
    }

    pub fn from_obstacle_map_with_overrides(
        obstacle_map: &ObstacleMap,
        opened_cells: Option<&FxHashSet<CellKey>>,
        extra_blocked_cells: Option<&FxHashSet<CellKey>>,
    ) -> Self {
        let width = obstacle_map.width();
        let height = obstacle_map.height();
        let w = usize::try_from(width).unwrap_or(0);
        let h = usize::try_from(height).unwrap_or(0);
        let mut occupancy = vec![0u8; w.saturating_mul(h)];

        for y in 0..h {
            for x in 0..w {
                let xi = i32::try_from(x).expect("x fits i32");
                let yi = i32::try_from(y).expect("y fits i32");
                if obstacle_map.is_blocked(xi, yi) {
                    occupancy[y * w + x] = 1;
                }
            }
        }

        if let Some(opened) = opened_cells {
            for key in opened {
                let (x, y) = unpack_xy(*key);
                if x >= 0 && y >= 0 && x < width && y < height {
                    let xu = usize::try_from(x).expect("non-negative x fits usize");
                    let yu = usize::try_from(y).expect("non-negative y fits usize");
                    occupancy[yu * w + xu] = 0;
                }
            }
        }

        if let Some(extra_blocked) = extra_blocked_cells {
            for key in extra_blocked {
                let (x, y) = unpack_xy(*key);
                if x >= 0 && y >= 0 && x < width && y < height {
                    let xu = usize::try_from(x).expect("non-negative x fits usize");
                    let yu = usize::try_from(y).expect("non-negative y fits usize");
                    occupancy[yu * w + xu] = 1;
                }
            }
        }

        Self::from_occupancy(width, height, &occupancy)
    }

    fn from_occupancy(width: i32, height: i32, occupancy: &[u8]) -> Self {
        let w = usize::try_from(width).unwrap_or(0);
        let h = usize::try_from(height).unwrap_or(0);
        let stride = w + 1;
        let mut prefix = vec![0u32; (w + 1) * (h + 1)];

        for y in 0..h {
            let mut row_sum = 0u32;
            for x in 0..w {
                row_sum = row_sum.saturating_add(u32::from(occupancy[y * w + x]));
                let idx = (y + 1) * stride + (x + 1);
                let above = prefix[y * stride + (x + 1)];
                prefix[idx] = above.saturating_add(row_sum);
            }
        }

        Self {
            width,
            height,
            prefix,
        }
    }

    pub fn blocked_count_in_rect(
        &self,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
    ) -> Option<u32> {
        if min_x > max_x || min_y > max_y {
            return None;
        }
        if min_x < 0 || min_y < 0 || max_x >= self.width || max_y >= self.height {
            return None;
        }
        let w = usize::try_from(self.width).ok()?;
        let stride = w + 1;
        let x1 = usize::try_from(min_x).ok()?;
        let y1 = usize::try_from(min_y).ok()?;
        let x2 = usize::try_from(max_x).ok()?;
        let y2 = usize::try_from(max_y).ok()?;
        let a = i64::from(self.prefix[(y2 + 1) * stride + (x2 + 1)]);
        let b = i64::from(self.prefix[y1 * stride + (x2 + 1)]);
        let c = i64::from(self.prefix[(y2 + 1) * stride + x1]);
        let d = i64::from(self.prefix[y1 * stride + x1]);
        let total = a + d - b - c;
        if total < 0 {
            return Some(0);
        }
        Some(total as u32)
    }
}

#[allow(dead_code)]
trait RectOccupancyQuery {
    fn grid_width(&self) -> i32;
    fn grid_height(&self) -> i32;
    fn blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Option<u32>;
}

impl RectOccupancyQuery for DenseOccupancyPrefix {
    fn grid_width(&self) -> i32 {
        self.width
    }

    fn grid_height(&self) -> i32 {
        self.height
    }

    fn blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Option<u32> {
        DenseOccupancyPrefix::blocked_count_in_rect(self, min_x, max_x, min_y, max_y)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SparseCellIndex {
    rows: Vec<Vec<i32>>,
    is_empty: bool,
}

impl SparseCellIndex {
    pub(crate) fn empty(height: i32) -> Self {
        let row_count = usize::try_from(height.max(0)).unwrap_or(0);
        Self {
            rows: vec![Vec::new(); row_count],
            is_empty: true,
        }
    }

    pub(crate) fn from_cells<I>(width: i32, height: i32, cells: I) -> Self
    where
        I: IntoIterator<Item = CellKey>,
    {
        let row_count = usize::try_from(height.max(0)).unwrap_or(0);
        let mut rows = vec![Vec::new(); row_count];
        for key in cells {
            let (x, y) = unpack_xy(key);
            if x < 0 || y < 0 || x >= width || y >= height {
                continue;
            }
            let yu = usize::try_from(y).expect("non-negative y fits usize");
            rows[yu].push(x);
        }
        for row in &mut rows {
            row.sort_unstable();
            row.dedup();
        }
        let is_empty = rows.iter().all(Vec::is_empty);
        Self { rows, is_empty }
    }

    pub(crate) fn insert_cells<I>(&mut self, width: i32, cells: I)
    where
        I: IntoIterator<Item = CellKey>,
    {
        let mut touched_rows = FxHashSet::default();
        let height = i32::try_from(self.rows.len()).unwrap_or(0);
        for key in cells {
            let (x, y) = unpack_xy(key);
            if x < 0 || y < 0 || x >= width || y >= height {
                continue;
            }
            let yu = usize::try_from(y).expect("non-negative y fits usize");
            self.rows[yu].push(x);
            touched_rows.insert(yu);
        }
        for row_index in touched_rows {
            let row = &mut self.rows[row_index];
            row.sort_unstable();
            row.dedup();
        }
        self.is_empty = self.rows.iter().all(Vec::is_empty);
    }

    pub(crate) fn insert_rect(
        &mut self,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
        width: i32,
    ) {
        if max_x < min_x || max_y < min_y {
            return;
        }
        let height = i32::try_from(self.rows.len()).unwrap_or(0);
        let x0 = min_x.max(0);
        let x1 = max_x.min(width.saturating_sub(1));
        let y0 = min_y.max(0);
        let y1 = max_y.min(height.saturating_sub(1));
        if x1 < x0 || y1 < y0 {
            return;
        }
        for y in y0..=y1 {
            let yu = usize::try_from(y).expect("non-negative y fits usize");
            let row = &mut self.rows[yu];
            row.extend(x0..=x1);
            row.sort_unstable();
            row.dedup();
        }
        self.is_empty = false;
    }

    fn is_empty(&self) -> bool {
        self.is_empty
    }

    pub(crate) fn from_opened_cells(
        base: &DenseOccupancyPrefix,
        opened_cells: &FxHashSet<CellKey>,
    ) -> Self {
        Self::from_cells(
            base.width,
            base.height,
            opened_cells.iter().copied().filter(|&key| {
                let (x, y) = unpack_xy(key);
                base.blocked_count_in_rect(x, x, y, y)
                    .is_some_and(|count| count > 0)
            }),
        )
    }

    fn count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> u32 {
        if min_x > max_x || min_y > max_y || self.is_empty() {
            return 0;
        }
        let first_y = usize::try_from(min_y.max(0)).unwrap_or(0);
        let last_y = usize::try_from(max_y.max(0)).unwrap_or(0);
        if first_y >= self.rows.len() {
            return 0;
        }
        let last_y = last_y.min(self.rows.len().saturating_sub(1));
        let mut total = 0usize;
        for row in &self.rows[first_y..=last_y] {
            let start = row.partition_point(|x| *x < min_x);
            let end = row.partition_point(|x| *x <= max_x);
            total = total.saturating_add(end.saturating_sub(start));
        }
        total.try_into().unwrap_or(u32::MAX)
    }
}

struct OverlayOccupancyQuery<'a> {
    base: &'a DenseOccupancyPrefix,
    opened_index: Cow<'a, SparseCellIndex>,
    extra_blocked_index: Cow<'a, SparseCellIndex>,
    extra_blocked_overlay_index: Option<&'a SparseCellIndex>,
}

impl<'a> OverlayOccupancyQuery<'a> {
    fn new(
        base: &'a DenseOccupancyPrefix,
        opened_cells: Option<&'a FxHashSet<CellKey>>,
        opened_index: Option<&'a SparseCellIndex>,
        extra_blocked_cells: Option<&'a FxHashSet<CellKey>>,
        extra_blocked_index: Option<&'a SparseCellIndex>,
        extra_blocked_overlay_index: Option<&'a SparseCellIndex>,
    ) -> Self {
        let opened_index = opened_index.map_or_else(
            || {
                Cow::Owned(SparseCellIndex::from_cells(
                    base.width,
                    base.height,
                    opened_cells
                        .into_iter()
                        .flat_map(|cells| cells.iter().copied())
                        .filter(|&key| {
                            let (x, y) = unpack_xy(key);
                            base.blocked_count_in_rect(x, x, y, y)
                                .is_some_and(|count| count > 0)
                        }),
                ))
            },
            Cow::Borrowed,
        );
        let extra_blocked_index = extra_blocked_index.map_or_else(
            || {
                Cow::Owned(SparseCellIndex::from_cells(
                    base.width,
                    base.height,
                    extra_blocked_cells
                        .into_iter()
                        .flat_map(|cells| cells.iter().copied())
                        .filter(|&key| {
                            let (x, y) = unpack_xy(key);
                            let opened = opened_index.count_in_rect(x, x, y, y) > 0;
                            let base_blocked = base
                                .blocked_count_in_rect(x, x, y, y)
                                .is_some_and(|count| count > 0);
                            !base_blocked || opened
                        }),
                ))
            },
            Cow::Borrowed,
        );
        Self {
            base,
            opened_index,
            extra_blocked_index,
            extra_blocked_overlay_index,
        }
    }

    fn opened_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> u32 {
        self.opened_index.count_in_rect(min_x, max_x, min_y, max_y)
    }

    fn extra_blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> u32 {
        let base_extra = self
            .extra_blocked_index
            .count_in_rect(min_x, max_x, min_y, max_y);
        let overlay_extra = self
            .extra_blocked_overlay_index
            .map_or(0, |index| index.count_in_rect(min_x, max_x, min_y, max_y));
        base_extra.saturating_add(overlay_extra)
    }
}

impl RectOccupancyQuery for OverlayOccupancyQuery<'_> {
    fn grid_width(&self) -> i32 {
        self.base.width
    }

    fn grid_height(&self) -> i32 {
        self.base.height
    }

    fn blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Option<u32> {
        let base = self
            .base
            .blocked_count_in_rect(min_x, max_x, min_y, max_y)?;
        let opened = self.opened_count_in_rect(min_x, max_x, min_y, max_y);
        let extra = self.extra_blocked_count_in_rect(min_x, max_x, min_y, max_y);
        Some(base.saturating_sub(opened).saturating_add(extra))
    }
}
pub fn meander_box_to_grid_rect(
    box_um: MeanderBox,
    grid: &GeometryGridSpec,
    clearance_radius_cells: i32,
) -> Result<MeanderGridRect, GeometryError> {
    if !box_um.min_x_um.is_finite()
        || !box_um.max_x_um.is_finite()
        || !box_um.min_y_um.is_finite()
        || !box_um.max_y_um.is_finite()
    {
        return Err(GeometryError::InvalidMeanderBox);
    }
    if box_um.min_x_um > box_um.max_x_um || box_um.min_y_um > box_um.max_y_um {
        return Err(GeometryError::InvalidMeanderBox);
    }
    if clearance_radius_cells < 0 {
        return Err(GeometryError::InvalidMeanderBox);
    }

    let gx0 = floor_snap_to_grid(box_um.min_x_um, grid.origin_x_um, grid.grid_size_um);
    let gy0 = floor_snap_to_grid(box_um.min_y_um, grid.origin_y_um, grid.grid_size_um);
    let gx1 = ((box_um.max_x_um - grid.origin_x_um) / grid.grid_size_um).ceil() as i32 - 1;
    let gy1 = ((box_um.max_y_um - grid.origin_y_um) / grid.grid_size_um).ceil() as i32 - 1;

    let rect = MeanderGridRect {
        min_x: gx0 - clearance_radius_cells,
        max_x: gx1 + clearance_radius_cells,
        min_y: gy0 - clearance_radius_cells,
        max_y: gy1 + clearance_radius_cells,
    };
    if rect.min_x > rect.max_x || rect.min_y > rect.max_y {
        return Err(GeometryError::InvalidMeanderBox);
    }
    Ok(rect)
}

pub fn check_meander_box_free_with_prefix(
    box_um: MeanderBox,
    grid: &GeometryGridSpec,
    prefix: &DenseOccupancyPrefix,
    clearance_radius_cells: i32,
) -> Result<MeanderGridRect, GeometryError> {
    check_meander_box_free_with_occupancy(box_um, grid, prefix, clearance_radius_cells)
}

fn check_meander_box_free_with_occupancy<Q: RectOccupancyQuery>(
    box_um: MeanderBox,
    grid: &GeometryGridSpec,
    occupancy: &Q,
    clearance_radius_cells: i32,
) -> Result<MeanderGridRect, GeometryError> {
    let rect = meander_box_to_grid_rect(box_um, grid, clearance_radius_cells)?;
    let blocked = occupancy
        .blocked_count_in_rect(rect.min_x, rect.max_x, rect.min_y, rect.max_y)
        .ok_or(GeometryError::MeanderBoxOutOfBounds(rect))?;
    if blocked > 0 {
        return Err(GeometryError::MeanderBoxBlocked {
            rect,
            blocked_count: blocked,
        });
    }
    Ok(rect)
}

#[cfg(test)]
fn check_meander_replacement_polygon_free<Q: RectOccupancyQuery>(
    replacement_centerline: &[PhysicalPoint],
    grid: &GeometryGridSpec,
    occupancy: &Q,
) -> Result<(), GeometryError> {
    if replacement_centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    let centerline: Vec<(f64, f64)> = replacement_centerline
        .iter()
        .map(|p| (p.x_um, p.y_um))
        .collect();
    let footprint_width_um = grid.grid_size_um.min(0.5);
    let polygon = generate_waveguide_polygon(&centerline, footprint_width_um)?;
    let static_grid = StaticGridSpec {
        width: occupancy.grid_width(),
        height: occupancy.grid_height(),
        grid_size_um: grid.grid_size_um,
        origin: (grid.origin_x_um, grid.origin_y_um),
        die_bbox: (
            grid.origin_x_um,
            grid.origin_y_um,
            grid.origin_x_um + f64::from(occupancy.grid_width()) * grid.grid_size_um,
            grid.origin_y_um + f64::from(occupancy.grid_height()) * grid.grid_size_um,
        ),
    };
    let polygon_cells = rasterize_polygon(&polygon, &static_grid);
    for key in polygon_cells {
        let (x, y) = unpack_xy(key);
        if occupancy
            .blocked_count_in_rect(x, x, y, y)
            .is_some_and(|count| count > 0)
        {
            return Err(GeometryError::MeanderBoxBlocked {
                rect: MeanderGridRect {
                    min_x: x,
                    max_x: x,
                    min_y: y,
                    max_y: y,
                },
                blocked_count: 1,
            });
        }
    }
    Ok(())
}

fn replacement_box_clearance_radius_cells(
    _grid: &GeometryGridSpec,
    configured_clearance_radius_cells: i32,
) -> i32 {
    configured_clearance_radius_cells
}

pub fn check_meander_box_free(
    box_um: MeanderBox,
    grid: &GeometryGridSpec,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    clearance_radius_cells: i32,
) -> Result<MeanderGridRect, GeometryError> {
    let prefix = DenseOccupancyPrefix::from_obstacle_map(obstacle_map, opened_cells);
    check_meander_box_free_with_prefix(box_um, grid, &prefix, clearance_radius_cells)
}

pub fn cells_in_grid_rect(rect: MeanderGridRect) -> Vec<(i32, i32)> {
    if rect.min_x > rect.max_x || rect.min_y > rect.max_y {
        return Vec::new();
    }
    let mut out = Vec::new();
    for y in rect.min_y..=rect.max_y {
        for x in rect.min_x..=rect.max_x {
            out.push((x, y));
        }
    }
    out
}

pub fn cell_count_in_grid_rect(rect: MeanderGridRect) -> usize {
    if rect.min_x > rect.max_x || rect.min_y > rect.max_y {
        return 0;
    }
    let width = i64::from(rect.max_x) - i64::from(rect.min_x) + 1;
    let height = i64::from(rect.max_y) - i64::from(rect.min_y) + 1;
    usize::try_from(width.saturating_mul(height)).unwrap_or(usize::MAX)
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AutoMeanderSidePolicy {
    Left,
    Right,
    Both,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AutoMeanderConfig {
    pub requested_extra_length_um: f64,
    pub min_bend_radius_um: f64,
    pub min_straight_um: f64,
    pub max_bumps: usize,
    pub max_meander_height_um: f64,
    pub box_depth_um: f64,
    pub min_segment_length_um: f64,
    pub endpoint_inset_um: f64,
    pub clearance_radius_cells: i32,
    pub side_policy: AutoMeanderSidePolicy,
    pub mode: MeanderPlanningMode,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AutoRouteAnalyticMeanderPlan {
    pub selected_segment_index: usize,
    pub selected_run_start_index: usize,
    pub selected_run_end_index: usize,
    pub selected_segment: StraightSegment,
    pub replacement_centerline: Vec<PhysicalPoint>,
    pub selected_run_length_um: f64,
    pub selected_box_depth_um: f64,
    pub selected_interval_length_um: f64,
    pub candidate_runs: usize,
    pub candidate_intervals: usize,
    pub rejected_box_blocked: usize,
    pub rejected_planning_failed: usize,
    pub rejected_exact_length_mismatch: usize,
    pub rejected_too_short: usize,
    pub selected_box: MeanderBox,
    pub selected_grid_rect: MeanderGridRect,
    pub plan: AnalyticMeanderPlan,
    pub profile: AutoMeanderPlanningProfile,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AutoMeanderRejectionDetail {
    pub reason: &'static str,
    pub side: MeanderSide,
    pub depth_um: f64,
    pub run_start_index: usize,
    pub run_end_index: usize,
    pub run_start: (f64, f64),
    pub run_end: (f64, f64),
    pub run_length_um: f64,
    pub allowed_interval_length_um: f64,
    pub required_interval_length_um: f64,
    pub strip_grid_rect: Option<MeanderGridRect>,
    pub strip_blocked_count: Option<u32>,
    pub strip_grid_error: Option<String>,
    pub planning_error: Option<String>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AutoMeanderPlanningProfile {
    pub total_s: f64,
    pub run_extraction_s: f64,
    pub footprint_s: f64,
    pub free_interval_s: f64,
    pub box_check_s: f64,
    pub analytic_plan_s: f64,
    pub replacement_check_s: f64,
    pub depth_count: usize,
    pub run_side_checks: usize,
    pub box_checks: usize,
    pub analytic_plan_calls: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AutoRouteAnalyticMeanderProbe {
    pub feasible: bool,
    pub candidate_runs: usize,
    pub candidate_intervals: usize,
    pub rejected_box_blocked: usize,
    pub rejected_planning_failed: usize,
    pub rejected_exact_length_mismatch: usize,
    pub rejected_too_short: usize,
    pub selected_run_start_index: Option<usize>,
    pub selected_run_end_index: Option<usize>,
    pub selected_run_length_um: Option<f64>,
    pub selected_interval_length_um: Option<f64>,
    pub selected_box_depth_um: Option<f64>,
    pub selected_grid_rect: Option<MeanderGridRect>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CenterlineStraightRun {
    pub start_index: usize,
    pub end_index: usize,
    pub start: (f64, f64),
    pub end: (f64, f64),
    pub length_um: f64,
}

pub fn build_meander_box_for_segment(
    segment: StraightSegment,
    side: MeanderSide,
    box_depth_um: f64,
) -> Result<MeanderBox, GeometryError> {
    if !box_depth_um.is_finite() || box_depth_um <= 0.0 {
        return Err(GeometryError::InvalidMeanderBox);
    }
    let dx = segment.end.x_um - segment.start.x_um;
    let dy = segment.end.y_um - segment.start.y_um;
    let (tx, ty) = if dx.abs() <= EPS && dy.abs() > EPS {
        (0.0, dy.signum())
    } else if dy.abs() <= EPS && dx.abs() > EPS {
        (dx.signum(), 0.0)
    } else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    let (mut nx, mut ny) = (-ty, tx);
    if side == MeanderSide::Right {
        nx = -nx;
        ny = -ny;
    }
    let s0 = segment.start;
    let s1 = segment.end;
    let q0 = PhysicalPoint {
        x_um: s0.x_um + nx * box_depth_um,
        y_um: s0.y_um + ny * box_depth_um,
    };
    let q1 = PhysicalPoint {
        x_um: s1.x_um + nx * box_depth_um,
        y_um: s1.y_um + ny * box_depth_um,
    };
    Ok(MeanderBox {
        min_x_um: s0.x_um.min(s1.x_um).min(q0.x_um).min(q1.x_um),
        max_x_um: s0.x_um.max(s1.x_um).max(q0.x_um).max(q1.x_um),
        min_y_um: s0.y_um.min(s1.y_um).min(q0.y_um).min(q1.y_um),
        max_y_um: s0.y_um.max(s1.y_um).max(q0.y_um).max(q1.y_um),
    })
}

fn point_on_run_at_distance(run: CenterlineStraightRun, distance_um: f64) -> PhysicalPoint {
    let t = if run.length_um <= EPS {
        0.0
    } else {
        (distance_um / run.length_um).clamp(0.0, 1.0)
    };
    PhysicalPoint {
        x_um: run.start.0 + (run.end.0 - run.start.0) * t,
        y_um: run.start.1 + (run.end.1 - run.start.1) * t,
    }
}

fn segment_from_axis_interval(
    run: CenterlineStraightRun,
    min_coord_um: f64,
    max_coord_um: f64,
) -> Result<StraightSegment, GeometryError> {
    if max_coord_um - min_coord_um <= EPS {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let horizontal = (run.start.1 - run.end.1).abs() <= EPS;
    let vertical = (run.start.0 - run.end.0).abs() <= EPS;
    if horizontal {
        let y = run.start.1;
        if run.end.0 >= run.start.0 {
            Ok(StraightSegment {
                start: PhysicalPoint {
                    x_um: min_coord_um,
                    y_um: y,
                },
                end: PhysicalPoint {
                    x_um: max_coord_um,
                    y_um: y,
                },
            })
        } else {
            Ok(StraightSegment {
                start: PhysicalPoint {
                    x_um: max_coord_um,
                    y_um: y,
                },
                end: PhysicalPoint {
                    x_um: min_coord_um,
                    y_um: y,
                },
            })
        }
    } else if vertical {
        let x = run.start.0;
        if run.end.1 >= run.start.1 {
            Ok(StraightSegment {
                start: PhysicalPoint {
                    x_um: x,
                    y_um: min_coord_um,
                },
                end: PhysicalPoint {
                    x_um: x,
                    y_um: max_coord_um,
                },
            })
        } else {
            Ok(StraightSegment {
                start: PhysicalPoint {
                    x_um: x,
                    y_um: max_coord_um,
                },
                end: PhysicalPoint {
                    x_um: x,
                    y_um: min_coord_um,
                },
            })
        }
    } else {
        Err(GeometryError::NoMeanderCandidateSegment)
    }
}

fn segment_length_um(segment: StraightSegment) -> f64 {
    distance(
        (segment.start.x_um, segment.start.y_um),
        (segment.end.x_um, segment.end.y_um),
    )
}

fn centered_subsegment(
    segment: StraightSegment,
    required_length_um: f64,
) -> Result<StraightSegment, GeometryError> {
    if !required_length_um.is_finite() || required_length_um <= 0.0 {
        return Err(GeometryError::InvalidMeanderBox);
    }
    let length = segment_length_um(segment);
    if length + EPS < required_length_um {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    if (length - required_length_um).abs() <= EPS {
        return Ok(segment);
    }
    let inset = 0.5 * (length - required_length_um);
    let dx = segment.end.x_um - segment.start.x_um;
    let dy = segment.end.y_um - segment.start.y_um;
    if dy.abs() <= EPS && dx.abs() > EPS {
        let sign = dx.signum();
        Ok(StraightSegment {
            start: PhysicalPoint {
                x_um: segment.start.x_um + sign * inset,
                y_um: segment.start.y_um,
            },
            end: PhysicalPoint {
                x_um: segment.start.x_um + sign * (inset + required_length_um),
                y_um: segment.start.y_um,
            },
        })
    } else if dx.abs() <= EPS && dy.abs() > EPS {
        let sign = dy.signum();
        Ok(StraightSegment {
            start: PhysicalPoint {
                x_um: segment.start.x_um,
                y_um: segment.start.y_um + sign * inset,
            },
            end: PhysicalPoint {
                x_um: segment.start.x_um,
                y_um: segment.start.y_um + sign * (inset + required_length_um),
            },
        })
    } else {
        Err(GeometryError::NoMeanderCandidateSegment)
    }
}

fn meander_projection_rejection_detail(
    run: CenterlineStraightRun,
    side: MeanderSide,
    box_depth_um: f64,
    endpoint_inset_um: f64,
    min_segment_length_um: f64,
    grid: &GeometryGridSpec,
    occupancy: &impl RectOccupancyQuery,
    clearance_radius_cells: i32,
    reason: &'static str,
    planning_error: Option<String>,
) -> AutoMeanderRejectionDetail {
    let mut strip_grid_rect = None;
    let mut strip_blocked_count = None;
    let mut strip_grid_error = None;
    let allowed_start_dist = if endpoint_inset_um >= 0.0 && endpoint_inset_um.is_finite() {
        endpoint_inset_um.min(run.length_um)
    } else {
        0.0
    };
    let allowed_end_dist = if endpoint_inset_um >= 0.0 && endpoint_inset_um.is_finite() {
        (run.length_um - endpoint_inset_um).max(0.0)
    } else {
        0.0
    };
    let allowed_interval_length_um = (allowed_end_dist - allowed_start_dist).max(0.0);
    if endpoint_inset_um >= 0.0 && endpoint_inset_um.is_finite() {
        if allowed_interval_length_um + EPS >= min_segment_length_um {
            let start_point = point_on_run_at_distance(run, allowed_start_dist);
            let end_point = point_on_run_at_distance(run, allowed_end_dist);
            let horizontal = (run.start.1 - run.end.1).abs() <= EPS;
            let vertical = (run.start.0 - run.end.0).abs() <= EPS;
            let allowed_min_coord = if horizontal {
                start_point.x_um.min(end_point.x_um)
            } else {
                start_point.y_um.min(end_point.y_um)
            };
            let allowed_max_coord = if horizontal {
                start_point.x_um.max(end_point.x_um)
            } else {
                start_point.y_um.max(end_point.y_um)
            };
            if (horizontal || vertical)
                && allowed_max_coord - allowed_min_coord + EPS >= min_segment_length_um
            {
                if let Ok(allowed_segment) =
                    segment_from_axis_interval(run, allowed_min_coord, allowed_max_coord)
                {
                    if let Ok(strip_box) =
                        build_meander_box_for_segment(allowed_segment, side, box_depth_um)
                    {
                        match meander_box_to_grid_rect(strip_box, grid, clearance_radius_cells) {
                            Ok(rect) => {
                                strip_blocked_count = occupancy.blocked_count_in_rect(
                                    rect.min_x, rect.max_x, rect.min_y, rect.max_y,
                                );
                                strip_grid_rect = Some(rect);
                            }
                            Err(err) => {
                                strip_grid_error = Some(err.to_string());
                            }
                        }
                    }
                }
            }
        }
    }
    AutoMeanderRejectionDetail {
        reason,
        side,
        depth_um: box_depth_um,
        run_start_index: run.start_index,
        run_end_index: run.end_index,
        run_start: run.start,
        run_end: run.end,
        run_length_um: run.length_um,
        allowed_interval_length_um,
        required_interval_length_um: min_segment_length_um,
        strip_grid_rect,
        strip_blocked_count,
        strip_grid_error,
        planning_error,
    }
}

fn projected_free_interval_segments(
    run: CenterlineStraightRun,
    side: MeanderSide,
    box_depth_um: f64,
    endpoint_inset_um: f64,
    min_segment_length_um: f64,
    grid: &GeometryGridSpec,
    occupancy: &impl RectOccupancyQuery,
    clearance_radius_cells: i32,
) -> Result<Vec<StraightSegment>, GeometryError> {
    if endpoint_inset_um < 0.0 || !endpoint_inset_um.is_finite() {
        return Err(GeometryError::InvalidMeanderBox);
    }
    let allowed_start_dist = endpoint_inset_um.min(run.length_um);
    let allowed_end_dist = (run.length_um - endpoint_inset_um).max(0.0);
    if allowed_end_dist - allowed_start_dist + EPS < min_segment_length_um {
        return Ok(Vec::new());
    }

    let start_point = point_on_run_at_distance(run, allowed_start_dist);
    let end_point = point_on_run_at_distance(run, allowed_end_dist);
    let horizontal = (run.start.1 - run.end.1).abs() <= EPS;
    let vertical = (run.start.0 - run.end.0).abs() <= EPS;
    if !(horizontal || vertical) {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }

    let allowed_min_coord = if horizontal {
        start_point.x_um.min(end_point.x_um)
    } else {
        start_point.y_um.min(end_point.y_um)
    };
    let allowed_max_coord = if horizontal {
        start_point.x_um.max(end_point.x_um)
    } else {
        start_point.y_um.max(end_point.y_um)
    };
    if allowed_max_coord - allowed_min_coord + EPS < min_segment_length_um {
        return Ok(Vec::new());
    }

    let allowed_segment = segment_from_axis_interval(run, allowed_min_coord, allowed_max_coord)?;
    let strip_box = build_meander_box_for_segment(allowed_segment, side, box_depth_um)?;
    let strip_rect = meander_box_to_grid_rect(strip_box, grid, clearance_radius_cells)?;
    let axis_origin = if horizontal {
        grid.origin_x_um
    } else {
        grid.origin_y_um
    };
    let first_idx = floor_snap_to_grid(allowed_min_coord, axis_origin, grid.grid_size_um);
    let last_idx = ((allowed_max_coord - axis_origin) / grid.grid_size_um).ceil() as i32 - 1;
    if first_idx > last_idx {
        return Ok(Vec::new());
    }

    let range_free = |start_idx: i32, end_idx: i32| -> bool {
        if start_idx > end_idx {
            return false;
        }
        let query = if horizontal {
            (
                start_idx - clearance_radius_cells,
                end_idx + clearance_radius_cells,
                strip_rect.min_y,
                strip_rect.max_y,
            )
        } else {
            (
                strip_rect.min_x,
                strip_rect.max_x,
                start_idx - clearance_radius_cells,
                end_idx + clearance_radius_cells,
            )
        };
        occupancy
            .blocked_count_in_rect(query.0, query.1, query.2, query.3)
            .map(|blocked| blocked == 0)
            .unwrap_or(false)
    };

    let mut intervals: Vec<(i32, i32)> = Vec::new();
    let mut idx = first_idx;
    while idx <= last_idx {
        if !range_free(idx, idx) {
            idx += 1;
            continue;
        }
        if range_free(idx, last_idx) {
            intervals.push((idx, last_idx));
            break;
        }

        let mut lo = idx;
        let mut hi = last_idx;
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if range_free(idx, mid) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        let first_blocked = lo;
        if first_blocked > idx {
            intervals.push((idx, first_blocked - 1));
        }
        idx = first_blocked + 1;
    }

    let mut segments = Vec::new();
    for (start_idx, end_idx) in intervals {
        let interval_min_coord =
            allowed_min_coord.max(axis_origin + (start_idx as f64) * grid.grid_size_um);
        let interval_max_coord =
            allowed_max_coord.min(axis_origin + ((end_idx + 1) as f64) * grid.grid_size_um);
        if interval_max_coord - interval_min_coord + EPS < min_segment_length_um {
            continue;
        }
        segments.push(segment_from_axis_interval(
            run,
            interval_min_coord,
            interval_max_coord,
        )?);
    }
    Ok(segments)
}

fn build_run_replacement_centerline(
    run: CenterlineStraightRun,
    meander_centerline: &[PhysicalPoint],
) -> Vec<PhysicalPoint> {
    let mut replacement = Vec::with_capacity(meander_centerline.len() + 2);
    push_physical_if_different(&mut replacement, (run.start.0, run.start.1));
    for point in meander_centerline {
        push_physical_if_different(&mut replacement, (point.x_um, point.y_um));
    }
    push_physical_if_different(&mut replacement, (run.end.0, run.end.1));
    replacement
        .into_iter()
        .map(|(x_um, y_um)| PhysicalPoint { x_um, y_um })
        .collect()
}
pub fn plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
    centerline: &[(f64, f64)],
    grid: &GeometryGridSpec,
    base_prefix: &DenseOccupancyPrefix,
    opened_cells: Option<&FxHashSet<CellKey>>,
    opened_index: Option<&SparseCellIndex>,
    extra_blocked_cells: Option<&FxHashSet<CellKey>>,
    extra_blocked_index: Option<&SparseCellIndex>,
    extra_blocked_overlay_index: Option<&SparseCellIndex>,
    config: &AutoMeanderConfig,
    box_depths_um: &[f64],
) -> Result<AutoRouteAnalyticMeanderPlan, GeometryError> {
    let total_start = Instant::now();
    let mut profile = AutoMeanderPlanningProfile::default();
    if !config.requested_extra_length_um.is_finite()
        || config.requested_extra_length_um <= 0.0
        || !config.min_bend_radius_um.is_finite()
        || config.min_bend_radius_um <= 0.0
        || !config.min_straight_um.is_finite()
        || config.min_straight_um < 0.0
        || config.max_bumps == 0
        || !config.max_meander_height_um.is_finite()
        || config.max_meander_height_um <= 0.0
        || !config.box_depth_um.is_finite()
        || config.box_depth_um <= 0.0
        || !config.min_segment_length_um.is_finite()
        || config.min_segment_length_um <= 0.0
        || !config.endpoint_inset_um.is_finite()
        || config.endpoint_inset_um < 0.0
        || config.clearance_radius_cells < 0
        || box_depths_um.is_empty()
    {
        return Err(GeometryError::InvalidMeanderBox);
    }
    let _ = centerline_length_um(centerline)?;
    for depth_um in box_depths_um {
        if !depth_um.is_finite() || *depth_um <= 0.0 {
            return Err(GeometryError::InvalidMeanderBox);
        }
    }

    let occupancy = OverlayOccupancyQuery::new(
        base_prefix,
        opened_cells,
        opened_index,
        extra_blocked_cells,
        extra_blocked_index,
        extra_blocked_overlay_index,
    );
    let side_order: &[MeanderSide] = match config.side_policy {
        AutoMeanderSidePolicy::Left => &[MeanderSide::Left],
        AutoMeanderSidePolicy::Right => &[MeanderSide::Right],
        AutoMeanderSidePolicy::Both => &[MeanderSide::Left, MeanderSide::Right],
    };

    let run_extraction_start = Instant::now();
    let runs = extract_axis_aligned_straight_runs(&centerline, config.min_segment_length_um);
    profile.run_extraction_s += run_extraction_start.elapsed().as_secs_f64();
    if runs.is_empty() {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let mut run_order: Vec<CenterlineStraightRun> = runs.clone();
    run_order.sort_by(|a, b| {
        let len_cmp = b.length_um.total_cmp(&a.length_um);
        if len_cmp.is_eq() {
            a.start_index.cmp(&b.start_index)
        } else {
            len_cmp
        }
    });

    let mut rejected_box_blocked = 0usize;
    let mut rejected_planning_failed = 0usize;
    let mut rejected_exact_length_mismatch = 0usize;
    let mut rejected_too_short = 0usize;
    let mut candidate_intervals = 0usize;
    let mut first_rejection: Option<AutoMeanderRejectionDetail> = None;
    let mut selected: Option<(MeanderBox, MeanderGridRect, AnalyticMeanderPlan, f64)> = None;
    let mut selected_run: Option<CenterlineStraightRun> = None;
    let mut selected_segment: Option<StraightSegment> = None;
    'outer: for &box_depth_um in box_depths_um {
        profile.depth_count += 1;
        let footprint_start = Instant::now();
        let footprint = match plan_fill_box_multi_bump_footprint(
            config.requested_extra_length_um,
            config.min_bend_radius_um,
            config.min_straight_um,
            config.max_bumps,
            box_depth_um.min(config.max_meander_height_um),
        ) {
            Ok(v) => v,
            Err(_) => {
                profile.footprint_s += footprint_start.elapsed().as_secs_f64();
                rejected_planning_failed += 1;
                continue;
            }
        };
        profile.footprint_s += footprint_start.elapsed().as_secs_f64();
        let actual_depth_um = footprint.amplitude_um;
        let required_interval_length_um = footprint
            .insertion_width_um
            .max(config.min_segment_length_um);
        for run in run_order.iter().copied() {
            for &side in side_order {
                profile.run_side_checks += 1;
                let free_interval_start = Instant::now();
                let free_segments = match projected_free_interval_segments(
                    run,
                    side,
                    actual_depth_um,
                    config.endpoint_inset_um,
                    required_interval_length_um,
                    grid,
                    &occupancy,
                    config.clearance_radius_cells,
                ) {
                    Ok(v) => v,
                    Err(_) => {
                        profile.free_interval_s += free_interval_start.elapsed().as_secs_f64();
                        rejected_planning_failed += 1;
                        continue;
                    }
                };
                profile.free_interval_s += free_interval_start.elapsed().as_secs_f64();
                if free_segments.is_empty() {
                    first_rejection.get_or_insert_with(|| {
                        meander_projection_rejection_detail(
                            run,
                            side,
                            actual_depth_um,
                            config.endpoint_inset_um,
                            required_interval_length_um,
                            grid,
                            &occupancy,
                            config.clearance_radius_cells,
                            "projected_strip_blocked_or_too_short",
                            None,
                        )
                    });
                    rejected_box_blocked += 1;
                    continue;
                }
                candidate_intervals += free_segments.len();
                for free_segment in free_segments {
                    let free_length = segment_length_um(free_segment);
                    if free_length + EPS < footprint.insertion_width_um {
                        rejected_too_short += 1;
                        continue;
                    }
                    let segment =
                        match centered_subsegment(free_segment, footprint.insertion_width_um) {
                            Ok(v) => v,
                            Err(_) => {
                                rejected_too_short += 1;
                                continue;
                            }
                        };
                    let box_um = match build_meander_box_for_segment(segment, side, actual_depth_um)
                    {
                        Ok(v) => v,
                        Err(_) => {
                            rejected_planning_failed += 1;
                            continue;
                        }
                    };
                    profile.box_checks += 1;
                    let box_check_start = Instant::now();
                    let rect = match check_meander_box_free_with_occupancy(
                        box_um,
                        grid,
                        &occupancy,
                        config.clearance_radius_cells,
                    ) {
                        Ok(v) => v,
                        Err(_) => {
                            profile.box_check_s += box_check_start.elapsed().as_secs_f64();
                            first_rejection.get_or_insert_with(|| {
                                meander_projection_rejection_detail(
                                    run,
                                    side,
                                    actual_depth_um,
                                    config.endpoint_inset_um,
                                    required_interval_length_um,
                                    grid,
                                    &occupancy,
                                    config.clearance_radius_cells,
                                    "candidate_box_blocked",
                                    None,
                                )
                            });
                            rejected_box_blocked += 1;
                            continue;
                        }
                    };
                    profile.box_check_s += box_check_start.elapsed().as_secs_f64();
                    let plan_cfg = AnalyticMeanderConfig {
                        requested_extra_length_um: config.requested_extra_length_um,
                        min_bend_radius_um: config.min_bend_radius_um,
                        min_straight_um: config.min_straight_um,
                        max_bumps: config.max_bumps,
                        max_meander_height_um: actual_depth_um,
                        side,
                        mode: config.mode,
                    };
                    profile.analytic_plan_calls += 1;
                    let analytic_plan_start = Instant::now();
                    let plan = match plan_analytic_meander(segment, box_um, &plan_cfg) {
                        Ok(v) => v,
                        Err(err) => {
                            profile.analytic_plan_s += analytic_plan_start.elapsed().as_secs_f64();
                            first_rejection.get_or_insert_with(|| {
                                meander_projection_rejection_detail(
                                    run,
                                    side,
                                    actual_depth_um,
                                    config.endpoint_inset_um,
                                    required_interval_length_um,
                                    grid,
                                    &occupancy,
                                    config.clearance_radius_cells,
                                    "analytic_planning_failed",
                                    Some(format!("{err:?}")),
                                )
                            });
                            rejected_planning_failed += 1;
                            continue;
                        }
                    };
                    profile.analytic_plan_s += analytic_plan_start.elapsed().as_secs_f64();
                    if (plan.inserted_extra_length_um - config.requested_extra_length_um).abs()
                        > 1.0e-6
                    {
                        rejected_exact_length_mismatch += 1;
                        continue;
                    }
                    let replacement_check_start = Instant::now();
                    let replacement_clearance_radius_cells =
                        replacement_box_clearance_radius_cells(grid, config.clearance_radius_cells);
                    let replacement_free = check_meander_box_free_with_occupancy(
                        box_um,
                        grid,
                        &occupancy,
                        replacement_clearance_radius_cells,
                    )
                    .is_ok();
                    profile.replacement_check_s += replacement_check_start.elapsed().as_secs_f64();
                    if !replacement_free {
                        first_rejection.get_or_insert_with(|| {
                            meander_projection_rejection_detail(
                                run,
                                side,
                                actual_depth_um,
                                config.endpoint_inset_um,
                                required_interval_length_um,
                                grid,
                                &occupancy,
                                config.clearance_radius_cells,
                                "replacement_box_blocked",
                                None,
                            )
                        });
                        rejected_box_blocked += 1;
                        continue;
                    }
                    selected_run = Some(run);
                    selected_segment = Some(segment);
                    selected = Some((box_um, rect, plan, actual_depth_um));
                    break 'outer;
                }
            }
        }
    }

    let no_auto_err = || GeometryError::NoAutoMeanderCandidate {
        candidate_runs: runs.len(),
        candidate_intervals,
        rejected_box_blocked,
        rejected_planning_failed,
        rejected_exact_length_mismatch,
        rejected_too_short,
        first_rejection: first_rejection.clone(),
    };
    let (selected_box, selected_grid_rect, plan, selected_box_depth_um) =
        selected.ok_or_else(no_auto_err)?;
    let run = selected_run.ok_or_else(no_auto_err)?;
    let segment = selected_segment.ok_or_else(no_auto_err)?;
    let replacement_centerline = build_run_replacement_centerline(run, &plan.centerline);
    profile.total_s = total_start.elapsed().as_secs_f64();
    Ok(AutoRouteAnalyticMeanderPlan {
        selected_segment_index: run.start_index,
        selected_run_start_index: run.start_index,
        selected_run_end_index: run.end_index,
        selected_segment: segment,
        replacement_centerline,
        selected_run_length_um: run.length_um,
        selected_box_depth_um,
        selected_interval_length_um: distance(
            (segment.start.x_um, segment.start.y_um),
            (segment.end.x_um, segment.end.y_um),
        ),
        candidate_runs: runs.len(),
        candidate_intervals,
        rejected_box_blocked,
        rejected_planning_failed,
        rejected_exact_length_mismatch,
        rejected_too_short,
        selected_box,
        selected_grid_rect,
        plan,
        profile,
    })
}
pub fn probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
    centerline: &[(f64, f64)],
    grid: &GeometryGridSpec,
    base_prefix: &DenseOccupancyPrefix,
    opened_cells: Option<&FxHashSet<CellKey>>,
    extra_blocked_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
    box_depths_um: &[f64],
) -> Result<AutoRouteAnalyticMeanderProbe, GeometryError> {
    if !config.requested_extra_length_um.is_finite()
        || config.requested_extra_length_um <= 0.0
        || !config.min_bend_radius_um.is_finite()
        || config.min_bend_radius_um <= 0.0
        || !config.min_straight_um.is_finite()
        || config.min_straight_um < 0.0
        || config.max_bumps == 0
        || !config.max_meander_height_um.is_finite()
        || config.max_meander_height_um <= 0.0
        || !config.box_depth_um.is_finite()
        || config.box_depth_um <= 0.0
        || !config.min_segment_length_um.is_finite()
        || config.min_segment_length_um <= 0.0
        || !config.endpoint_inset_um.is_finite()
        || config.endpoint_inset_um < 0.0
        || config.clearance_radius_cells < 0
        || box_depths_um.is_empty()
    {
        return Err(GeometryError::InvalidMeanderBox);
    }
    let _ = centerline_length_um(centerline)?;
    for depth_um in box_depths_um {
        if !depth_um.is_finite() || *depth_um <= 0.0 {
            return Err(GeometryError::InvalidMeanderBox);
        }
    }

    let occupancy = OverlayOccupancyQuery::new(
        base_prefix,
        opened_cells,
        None,
        extra_blocked_cells,
        None,
        None,
    );
    let side_order: &[MeanderSide] = match config.side_policy {
        AutoMeanderSidePolicy::Left => &[MeanderSide::Left],
        AutoMeanderSidePolicy::Right => &[MeanderSide::Right],
        AutoMeanderSidePolicy::Both => &[MeanderSide::Left, MeanderSide::Right],
    };
    let runs = extract_axis_aligned_straight_runs(&centerline, config.min_segment_length_um);
    if runs.is_empty() {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let mut run_order: Vec<CenterlineStraightRun> = runs.clone();
    run_order.sort_by(|a, b| {
        let len_cmp = b.length_um.total_cmp(&a.length_um);
        if len_cmp.is_eq() {
            a.start_index.cmp(&b.start_index)
        } else {
            len_cmp
        }
    });

    let mut rejected_box_blocked = 0usize;
    let mut rejected_planning_failed = 0usize;
    let rejected_exact_length_mismatch = 0usize;
    let mut rejected_too_short = 0usize;
    let mut candidate_intervals = 0usize;
    for &box_depth_um in box_depths_um {
        let footprint = match plan_fill_box_multi_bump_footprint(
            config.requested_extra_length_um,
            config.min_bend_radius_um,
            config.min_straight_um,
            config.max_bumps,
            box_depth_um.min(config.max_meander_height_um),
        ) {
            Ok(v) => v,
            Err(_) => {
                rejected_planning_failed += 1;
                continue;
            }
        };
        let actual_depth_um = footprint.amplitude_um;
        let required_interval_length_um = footprint
            .insertion_width_um
            .max(config.min_segment_length_um);
        for run in run_order.iter().copied() {
            for &side in side_order {
                let free_segments = match projected_free_interval_segments(
                    run,
                    side,
                    actual_depth_um,
                    config.endpoint_inset_um,
                    required_interval_length_um,
                    grid,
                    &occupancy,
                    config.clearance_radius_cells,
                ) {
                    Ok(v) => v,
                    Err(_) => {
                        rejected_planning_failed += 1;
                        continue;
                    }
                };
                if free_segments.is_empty() {
                    rejected_box_blocked += 1;
                    continue;
                }
                candidate_intervals += free_segments.len();
                for free_segment in free_segments {
                    let free_length = segment_length_um(free_segment);
                    if free_length + EPS < footprint.insertion_width_um {
                        rejected_too_short += 1;
                        continue;
                    }
                    let segment =
                        match centered_subsegment(free_segment, footprint.insertion_width_um) {
                            Ok(v) => v,
                            Err(_) => {
                                rejected_too_short += 1;
                                continue;
                            }
                        };
                    let box_um = match build_meander_box_for_segment(segment, side, actual_depth_um)
                    {
                        Ok(v) => v,
                        Err(_) => {
                            rejected_planning_failed += 1;
                            continue;
                        }
                    };
                    let rect = match check_meander_box_free_with_occupancy(
                        box_um,
                        grid,
                        &occupancy,
                        config.clearance_radius_cells,
                    ) {
                        Ok(v) => v,
                        Err(_) => {
                            rejected_box_blocked += 1;
                            continue;
                        }
                    };
                    return Ok(AutoRouteAnalyticMeanderProbe {
                        feasible: true,
                        candidate_runs: runs.len(),
                        candidate_intervals,
                        rejected_box_blocked,
                        rejected_planning_failed,
                        rejected_exact_length_mismatch,
                        rejected_too_short,
                        selected_run_start_index: Some(run.start_index),
                        selected_run_end_index: Some(run.end_index),
                        selected_run_length_um: Some(run.length_um),
                        selected_interval_length_um: Some(segment_length_um(segment)),
                        selected_box_depth_um: Some(actual_depth_um),
                        selected_grid_rect: Some(rect),
                    });
                }
            }
        }
    }

    Ok(AutoRouteAnalyticMeanderProbe {
        feasible: false,
        candidate_runs: runs.len(),
        candidate_intervals,
        rejected_box_blocked,
        rejected_planning_failed,
        rejected_exact_length_mismatch,
        rejected_too_short,
        selected_run_start_index: None,
        selected_run_end_index: None,
        selected_run_length_um: None,
        selected_interval_length_um: None,
        selected_box_depth_um: None,
        selected_grid_rect: None,
    })
}

pub(crate) fn extract_axis_aligned_straight_runs(
    centerline: &[(f64, f64)],
    min_length_um: f64,
) -> Vec<CenterlineStraightRun> {
    if centerline.len() < 2 {
        return Vec::new();
    }
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Axis {
        Horizontal,
        Vertical,
    }
    #[derive(Clone, Copy)]
    struct Acc {
        start_index: usize,
        end_index: usize,
        start: (f64, f64),
        end: (f64, f64),
        length_um: f64,
        axis: Axis,
        dir: i8,
        line_coord: f64,
    }
    let mut runs = Vec::new();
    let mut acc: Option<Acc> = None;
    let min_len = min_length_um.max(0.0);
    for i in 0..(centerline.len() - 1) {
        let p0 = centerline[i];
        let p1 = centerline[i + 1];
        let dx = p1.0 - p0.0;
        let dy = p1.1 - p0.1;
        let seg_len = distance(p0, p1);
        if seg_len <= EPS {
            continue;
        }
        let segment_kind = if dy.abs() <= EPS && dx.abs() > EPS {
            Some((
                Axis::Horizontal,
                if dx > 0.0 { 1 } else { -1 },
                p0.1,
                seg_len,
            ))
        } else if dx.abs() <= EPS && dy.abs() > EPS {
            Some((Axis::Vertical, if dy > 0.0 { 1 } else { -1 }, p0.0, seg_len))
        } else {
            None
        };
        let Some((axis, dir, line_coord, seg_len)) = segment_kind else {
            if let Some(a) = acc.take() {
                if a.length_um + EPS >= min_len {
                    runs.push(CenterlineStraightRun {
                        start_index: a.start_index,
                        end_index: a.end_index,
                        start: a.start,
                        end: a.end,
                        length_um: a.length_um,
                    });
                }
            }
            continue;
        };
        match acc.as_mut() {
            None => {
                acc = Some(Acc {
                    start_index: i,
                    end_index: i + 1,
                    start: p0,
                    end: p1,
                    length_um: seg_len,
                    axis,
                    dir,
                    line_coord,
                });
            }
            Some(a)
                if a.axis == axis
                    && a.dir == dir
                    && (a.line_coord - line_coord).abs() <= EPS
                    && a.end_index == i
                    && distance(a.end, p0) <= EPS =>
            {
                a.end_index = i + 1;
                a.end = p1;
                a.length_um += seg_len;
            }
            Some(_) => {
                let prev = acc.take().expect("acc exists");
                if prev.length_um + EPS >= min_len {
                    runs.push(CenterlineStraightRun {
                        start_index: prev.start_index,
                        end_index: prev.end_index,
                        start: prev.start,
                        end: prev.end,
                        length_um: prev.length_um,
                    });
                }
                acc = Some(Acc {
                    start_index: i,
                    end_index: i + 1,
                    start: p0,
                    end: p1,
                    length_um: seg_len,
                    axis,
                    dir,
                    line_coord,
                });
            }
        }
    }
    if let Some(a) = acc.take() {
        if a.length_um + EPS >= min_len {
            runs.push(CenterlineStraightRun {
                start_index: a.start_index,
                end_index: a.end_index,
                start: a.start,
                end: a.end,
                length_um: a.length_um,
            });
        }
    }
    runs
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> GeometryGridSpec {
        GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap()
    }

    fn default_config() -> AutoMeanderConfig {
        AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        }
    }

    fn straight_centerline() -> Vec<(f64, f64)> {
        vec![(1.5, 2.5), (5.5, 2.5)]
    }

    fn plan_centerline_with_map(
        centerline: &[(f64, f64)],
        map: &ObstacleMap,
        config: &AutoMeanderConfig,
        box_depths_um: &[f64],
    ) -> Result<AutoRouteAnalyticMeanderPlan, GeometryError> {
        let prefix = DenseOccupancyPrefix::from_obstacle_map(map, None);
        plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
            centerline,
            &grid(),
            &prefix,
            None,
            None,
            None,
            None,
            None,
            config,
            box_depths_um,
        )
    }

    #[test]
    fn dense_prefix_counts_blocked_cells_in_rectangles() {
        let mut map = ObstacleMap::new(6, 6);
        assert!(map.add_static_cell(2, 3));
        let prefix = DenseOccupancyPrefix::from_obstacle_map(&map, None);
        assert!(prefix.blocked_count_in_rect(1, 3, 2, 4).unwrap() > 0);
        assert_eq!(prefix.blocked_count_in_rect(0, 1, 0, 1).unwrap(), 0);
    }
    #[test]
    fn meander_replacement_polygon_rejects_blocked_footprint_cell() {
        let g = GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap();
        let replacement = vec![
            PhysicalPoint {
                x_um: 1.0,
                y_um: 1.5,
            },
            PhysicalPoint {
                x_um: 4.0,
                y_um: 1.5,
            },
        ];

        let empty = ObstacleMap::new(8, 4);
        let empty_prefix = DenseOccupancyPrefix::from_obstacle_map(&empty, None);
        assert!(check_meander_replacement_polygon_free(&replacement, &g, &empty_prefix).is_ok());

        let mut blocked = ObstacleMap::new(8, 4);
        assert!(blocked.add_static_cell(2, 1));
        let blocked_prefix = DenseOccupancyPrefix::from_obstacle_map(&blocked, None);
        assert!(check_meander_replacement_polygon_free(&replacement, &g, &blocked_prefix).is_err());
    }

    #[test]
    fn centerline_depth_sweep_selects_valid_box_on_empty_map() {
        let map = ObstacleMap::new(20, 20);
        let cfg = default_config();

        let plan =
            plan_centerline_with_map(&straight_centerline(), &map, &cfg, &[cfg.box_depth_um])
                .expect("empty map should support an auto meander");

        assert!(!plan.plan.centerline.is_empty());
        assert!(!plan.replacement_centerline.is_empty());
        assert_eq!(plan.selected_run_start_index, 0);
        assert_eq!(plan.selected_run_end_index, 1);
        assert_eq!(plan.candidate_runs, 1);
    }

    #[test]
    fn centerline_depth_sweep_chooses_right_when_left_box_blocked() {
        let mut map = ObstacleMap::new(20, 20);
        for x in 1..=5 {
            assert!(map.add_static_cell(x, 3));
        }
        let cfg = default_config();

        let plan =
            plan_centerline_with_map(&straight_centerline(), &map, &cfg, &[cfg.box_depth_um])
                .expect("right side should remain available");

        assert_eq!(plan.plan.side, MeanderSide::Right);
        assert!(plan.selected_box.max_y_um <= 2.5 + EPS);
        assert!(plan.rejected_box_blocked >= 1);
    }

    #[test]
    fn centerline_depth_sweep_returns_no_candidate_when_both_sides_blocked() {
        let mut map = ObstacleMap::new(20, 20);
        for x in 1..=5 {
            assert!(map.add_static_cell(x, 3));
            assert!(map.add_static_cell(x, 1));
        }
        let cfg = default_config();

        let err = plan_centerline_with_map(&straight_centerline(), &map, &cfg, &[cfg.box_depth_um])
            .unwrap_err();

        assert!(matches!(
            err,
            GeometryError::NoAutoMeanderCandidate {
                candidate_runs: 1,
                rejected_box_blocked: 2,
                ..
            }
        ));
    }

    #[test]
    fn centerline_depth_sweep_is_deterministic_for_tie() {
        let map = ObstacleMap::new(30, 30);
        let cfg = default_config();
        let centerline = vec![(2.5, 10.5), (6.5, 10.5), (10.5, 10.5)];

        let first = plan_centerline_with_map(&centerline, &map, &cfg, &[cfg.box_depth_um]).unwrap();
        let second =
            plan_centerline_with_map(&centerline, &map, &cfg, &[cfg.box_depth_um]).unwrap();

        assert_eq!(first.selected_segment_index, 0);
        assert_eq!(first.selected_run_start_index, 0);
        assert_eq!(first.selected_run_end_index, 2);
        assert_eq!(first.plan.side, MeanderSide::Left);
        assert_eq!(first.selected_segment, second.selected_segment);
        assert_eq!(first.selected_grid_rect, second.selected_grid_rect);
        assert_eq!(first.plan.side, second.plan.side);
    }

    #[test]
    fn centerline_depth_sweep_tries_later_depth_after_shallow_depth_fails() {
        let map = ObstacleMap::new(20, 20);
        let cfg = default_config();

        let plan = plan_centerline_with_map(&straight_centerline(), &map, &cfg, &[0.4, 1.6])
            .expect("second depth should satisfy the analytic footprint");

        assert_eq!(plan.profile.depth_count, 2);
        assert!((plan.selected_box_depth_um - 1.1433629385640827).abs() < 1.0e-9);
        assert!(!plan.plan.centerline.is_empty());
    }

    #[test]
    fn centerline_probe_reports_feasible_candidate_consistent_with_plan() {
        let map = ObstacleMap::new(20, 20);
        let prefix = DenseOccupancyPrefix::from_obstacle_map(&map, None);
        let cfg = default_config();
        let centerline = straight_centerline();
        let plan = plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
            &centerline,
            &grid(),
            &prefix,
            None,
            None,
            None,
            None,
            None,
            &cfg,
            &[cfg.box_depth_um],
        )
        .expect("planner should find the same candidate");

        let probe = probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
            &centerline,
            &grid(),
            &prefix,
            None,
            None,
            &cfg,
            &[cfg.box_depth_um],
        )
        .expect("probe should report feasibility");

        assert!(probe.feasible);
        assert_eq!(probe.candidate_runs, plan.candidate_runs);
        assert_eq!(
            probe.selected_run_start_index,
            Some(plan.selected_run_start_index)
        );
        assert_eq!(
            probe.selected_run_end_index,
            Some(plan.selected_run_end_index)
        );
        assert_eq!(probe.selected_grid_rect, Some(plan.selected_grid_rect));
        assert_eq!(
            probe.selected_box_depth_um,
            Some(plan.selected_box_depth_um)
        );
    }
}
