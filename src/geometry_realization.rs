//! Fast route-to-polygon realization.
//!
//! This module converts a routed state/primitive sequence into one closed
//! waveguide polygon in physical coordinates.

use std::error::Error;
use std::fmt;
use std::time::Instant;

use rustc_hash::FxHashSet;

use crate::astar::{RouteResult, State};
use crate::meander::{
    plan_analytic_meander, plan_fill_box_multi_bump_footprint, AnalyticMeanderConfig,
    AnalyticMeanderPlan, MeanderBox, MeanderPlanningError, MeanderPlanningMode, MeanderSide,
    PhysicalPoint, StraightSegment,
};
use crate::obstacle_map::{unpack_xy, CellKey, ObstacleMap};
use crate::primitives::{PrimitiveGeometry, PrimitiveLibrary};
use crate::static_obstacle_builder::{
    grid_cell_center, physical_to_grid, PortInput, StaticGridSpec,
};

const EPS: f64 = 1.0e-9;
const MITER_LIMIT: f64 = 4.0;
const DEFAULT_BEND_SAMPLES_PER_90_DEG: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AxisAlignedRunKind {
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AxisAlignedRun {
    start_index: usize,
    end_index: usize,
    kind: AxisAlignedRunKind,
}

#[derive(Clone, Debug, PartialEq)]
struct PrimitiveCenterlineReplay {
    centerline: Vec<(f64, f64)>,
    straight_runs: Vec<AxisAlignedRun>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GridRect {
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
        let stride = w + 1;
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

trait RectOccupancyQuery {
    fn blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Option<u32>;
}

impl RectOccupancyQuery for DenseOccupancyPrefix {
    fn blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Option<u32> {
        DenseOccupancyPrefix::blocked_count_in_rect(self, min_x, max_x, min_y, max_y)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct SparseCellIndex {
    rows: Vec<Vec<i32>>,
    is_empty: bool,
}

impl SparseCellIndex {
    fn from_cells<I>(width: i32, height: i32, cells: I) -> Self
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

    fn is_empty(&self) -> bool {
        self.is_empty
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
    opened_index: SparseCellIndex,
    extra_blocked_index: SparseCellIndex,
}

impl<'a> OverlayOccupancyQuery<'a> {
    fn new(
        base: &'a DenseOccupancyPrefix,
        opened_cells: Option<&'a FxHashSet<CellKey>>,
        extra_blocked_cells: Option<&'a FxHashSet<CellKey>>,
    ) -> Self {
        let opened_index = SparseCellIndex::from_cells(
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
        );
        let extra_blocked_index = SparseCellIndex::from_cells(
            base.width,
            base.height,
            extra_blocked_cells
                .into_iter()
                .flat_map(|cells| cells.iter().copied())
                .filter(|&key| {
                    let opened = opened_cells.is_some_and(|cells| cells.contains(&key));
                    let (x, y) = unpack_xy(key);
                    let base_blocked = base
                        .blocked_count_in_rect(x, x, y, y)
                        .is_some_and(|count| count > 0);
                    !base_blocked || opened
                }),
        );
        Self {
            base,
            opened_index,
            extra_blocked_index,
        }
    }

    fn opened_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> u32 {
        self.opened_index.count_in_rect(min_x, max_x, min_y, max_y)
    }

    fn extra_blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> u32 {
        self.extra_blocked_index
            .count_in_rect(min_x, max_x, min_y, max_y)
    }
}

impl RectOccupancyQuery for OverlayOccupancyQuery<'_> {
    fn blocked_count_in_rect(&self, min_x: i32, max_x: i32, min_y: i32, max_y: i32) -> Option<u32> {
        let base = self
            .base
            .blocked_count_in_rect(min_x, max_x, min_y, max_y)?;
        let opened = self.opened_count_in_rect(min_x, max_x, min_y, max_y);
        let extra = self.extra_blocked_count_in_rect(min_x, max_x, min_y, max_y);
        Some(base.saturating_sub(opened).saturating_add(extra))
    }
}

/// Minimal grid information required to convert grid cells to physical points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeometryGridSpec {
    pub grid_size_um: f64,
    pub origin_x_um: f64,
    pub origin_y_um: f64,
}

impl GeometryGridSpec {
    pub fn new(
        grid_size_um: f64,
        origin_x_um: f64,
        origin_y_um: f64,
    ) -> Result<Self, GeometryError> {
        if !grid_size_um.is_finite() || grid_size_um <= 0.0 {
            return Err(GeometryError::InvalidGridSize(grid_size_um));
        }
        if !origin_x_um.is_finite() || !origin_y_um.is_finite() {
            return Err(GeometryError::NonFiniteCoordinate);
        }

        Ok(Self {
            grid_size_um,
            origin_x_um,
            origin_y_um,
        })
    }

    pub fn cell_center(&self, x: i32, y: i32) -> (f64, f64) {
        (
            self.origin_x_um + (x as f64 + 0.5) * self.grid_size_um,
            self.origin_y_um + (y as f64 + 0.5) * self.grid_size_um,
        )
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum GeometryError {
    InvalidGridSize(f64),
    InvalidWidth(f64),
    EmptyRoute,
    DegenerateRoute,
    InvalidRouteTopology {
        states: usize,
        primitives: usize,
    },
    MissingPrimitive {
        id: u16,
        start_angle: u8,
    },
    ZeroLengthSegment,
    NonFiniteCoordinate,
    RouteStartDoesNotMatchSourceAnchor {
        route_start: (f64, f64),
        source_anchor: (f64, f64),
    },
    RouteEndDoesNotMatchTargetAnchor {
        route_end: (f64, f64),
        target_anchor: (f64, f64),
    },
    PrimitiveEndpointMismatch {
        primitive_id: u16,
        expected: (f64, f64),
        actual: (f64, f64),
    },
    InvalidMeanderBox,
    MeanderBoxOutOfBounds(GridRect),
    MeanderBoxBlocked {
        rect: GridRect,
        blocked_count: u32,
    },
    NoAutoMeanderCandidate {
        candidate_runs: usize,
        candidate_intervals: usize,
        rejected_box_blocked: usize,
        rejected_planning_failed: usize,
        rejected_exact_length_mismatch: usize,
        rejected_too_short: usize,
    },
    NoMeanderCandidateSegment,
    MeanderPlanningFailed(MeanderPlanningError),
    PortAccess(PortAccessError),
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeometryError::InvalidGridSize(value) => {
                write!(f, "grid_size_um must be finite and > 0, got {value}")
            }
            GeometryError::InvalidWidth(value) => {
                write!(f, "width_um must be finite and > 0, got {value}")
            }
            GeometryError::EmptyRoute => write!(f, "route has no states"),
            GeometryError::DegenerateRoute => {
                write!(f, "route must contain at least two distinct centerline points")
            }
            GeometryError::InvalidRouteTopology { states, primitives } => write!(
                f,
                "invalid route topology: expected states = primitives + 1, got states={states}, primitives={primitives}"
            ),
            GeometryError::MissingPrimitive { id, start_angle } => write!(
                f,
                "route references unknown primitive id={id} for start_angle={start_angle}"
            ),
            GeometryError::ZeroLengthSegment => write!(f, "centerline contains a zero-length segment"),
            GeometryError::NonFiniteCoordinate => write!(f, "geometry contains a non-finite coordinate"),
            GeometryError::RouteStartDoesNotMatchSourceAnchor {
                route_start,
                source_anchor,
            } => write!(
                f,
                "route start point {route_start:?} does not match source anchor point {source_anchor:?}"
            ),
            GeometryError::RouteEndDoesNotMatchTargetAnchor {
                route_end,
                target_anchor,
            } => write!(
                f,
                "route end point {route_end:?} does not match target anchor point {target_anchor:?}"
            ),
            GeometryError::PrimitiveEndpointMismatch {
                primitive_id,
                expected,
                actual,
            } => write!(
                f,
                "primitive id={primitive_id} endpoint mismatch: expected {expected:?}, got {actual:?}"
            ),
            GeometryError::InvalidMeanderBox => write!(f, "invalid meander box"),
            GeometryError::MeanderBoxOutOfBounds(rect) => {
                write!(f, "meander box out of bounds after grid conversion: {rect:?}")
            }
            GeometryError::MeanderBoxBlocked {
                rect,
                blocked_count,
            } => write!(
                f,
                "meander box overlaps blocked cells: rect={rect:?}, blocked_count={blocked_count}"
            ),
            GeometryError::NoAutoMeanderCandidate {
                candidate_runs,
                candidate_intervals,
                rejected_box_blocked,
                rejected_planning_failed,
                rejected_exact_length_mismatch,
                rejected_too_short,
            } => write!(
                f,
                "no legal auto-analytic meander candidate found (candidate_runs={candidate_runs}, candidate_intervals={candidate_intervals}, rejected_box_blocked={rejected_box_blocked}, rejected_planning_failed={rejected_planning_failed}, rejected_exact_length_mismatch={rejected_exact_length_mismatch}, rejected_too_short={rejected_too_short})"
            ),
            GeometryError::NoMeanderCandidateSegment => {
                write!(f, "no axis-aligned centerline segment is suitable for meander insertion")
            }
            GeometryError::MeanderPlanningFailed(err) => {
                write!(f, "analytic meander planning failed: {err:?}")
            }
            GeometryError::PortAccess(err) => write!(f, "{err}"),
        }
    }
}

impl Error for GeometryError {}

impl From<PortAccessError> for GeometryError {
    fn from(value: PortAccessError) -> Self {
        GeometryError::PortAccess(value)
    }
}

/// Configuration for deterministic local port-access geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PortAccessConfig {
    pub min_straight_um: f64,
    pub max_anchor_search_cells: i32,
    pub min_bend_radius_um: f64,
}

impl Default for PortAccessConfig {
    fn default() -> Self {
        Self {
            min_straight_um: 0.0,
            max_anchor_search_cells: 8,
            min_bend_radius_um: 0.0,
        }
    }
}

/// Deterministic pre-routing connector from physical port to grid anchor cell.
#[derive(Clone, Debug, PartialEq)]
pub struct PortAccess {
    pub port_name: String,
    pub port_point_um: (f64, f64),
    pub anchor_cell: (i32, i32),
    pub anchor_point_um: (f64, f64),
    pub port_angle: u8,
    pub anchor_angle: u8,
    /// Compatibility alias; equals `anchor_angle`.
    pub entry_angle: u8,
    pub access_centerline_um: Vec<(f64, f64)>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PortAccessError {
    NonFinitePortCoordinate {
        port_name: String,
    },
    InvalidConfig(String),
    AnchorOutOfBounds {
        port_name: String,
        anchor_cell: (i32, i32),
    },
    AnchorSearchFailed {
        port_name: String,
    },
    ZeroLengthAccess {
        port_name: String,
    },
    NonFiniteAccessCoordinate {
        port_name: String,
    },
}

impl fmt::Display for PortAccessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PortAccessError::NonFinitePortCoordinate { port_name } => {
                write!(f, "port '{port_name}' has non-finite physical coordinates")
            }
            PortAccessError::InvalidConfig(msg) => write!(f, "invalid port-access config: {msg}"),
            PortAccessError::AnchorOutOfBounds {
                port_name,
                anchor_cell,
            } => write!(
                f,
                "port '{port_name}' selected out-of-bounds anchor cell {anchor_cell:?}"
            ),
            PortAccessError::AnchorSearchFailed { port_name } => {
                write!(f, "failed to find valid anchor cell for port '{port_name}'")
            }
            PortAccessError::ZeroLengthAccess { port_name } => {
                write!(f, "port '{port_name}' access connector has zero length")
            }
            PortAccessError::NonFiniteAccessCoordinate { port_name } => write!(
                f,
                "port '{port_name}' access connector contains non-finite coordinates"
            ),
        }
    }
}

impl Error for PortAccessError {}

/// Compress route states by keeping only heading-change points.
pub fn compress_route_waypoints(states: &[State]) -> Vec<(i32, i32)> {
    if states.is_empty() {
        return Vec::new();
    }

    let mut waypoints = Vec::with_capacity(states.len());
    waypoints.push((states[0].x, states[0].y));

    for i in 1..states.len() {
        if states[i].angle != states[i - 1].angle {
            push_if_different(&mut waypoints, (states[i].x, states[i].y));
        }
    }

    if let Some(last) = states.last() {
        push_if_different(&mut waypoints, (last.x, last.y));
    }

    waypoints
}

/// Build a grid-cell polyline by following each primitive footprint in order.
pub fn route_to_grid_path(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
) -> Result<Vec<(i32, i32)>, GeometryError> {
    if route.states.is_empty() {
        return Err(GeometryError::EmptyRoute);
    }

    // Simple pre-routes may return grid-polyline results without primitive IDs.
    // In that case we can still realize geometry by walking the state polyline.
    if route.primitives.is_empty() {
        let mut path = Vec::with_capacity(route.states.len());
        for state in &route.states {
            push_if_different(&mut path, (state.x, state.y));
        }
        if path.len() < 2 {
            return Err(GeometryError::DegenerateRoute);
        }
        return Ok(path);
    }

    if route.states.len() != route.primitives.len() + 1 {
        return Err(GeometryError::InvalidRouteTopology {
            states: route.states.len(),
            primitives: route.primitives.len(),
        });
    }

    let mut path = Vec::new();
    push_if_different(&mut path, (route.states[0].x, route.states[0].y));

    for (index, primitive_id) in route.primitives.iter().enumerate() {
        let origin = route.states[index];
        let primitive = primitives
            .get_primitives_for_angle(origin.angle)
            .iter()
            .find(|p| p.id == *primitive_id)
            .ok_or(GeometryError::MissingPrimitive {
                id: *primitive_id,
                start_angle: origin.angle,
            })?;

        for (dx, dy) in primitive.footprint.iter().copied() {
            let cell = (origin.x + dx, origin.y + dy);
            push_if_different(&mut path, cell);
        }
    }

    let last_state = route.states[route.states.len() - 1];
    push_if_different(&mut path, (last_state.x, last_state.y));

    if path.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    Ok(path)
}

/// Compress a grid-cell polyline by keeping only heading-change points.
pub fn compress_grid_waypoints(path: &[(i32, i32)]) -> Vec<(i32, i32)> {
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

/// Convert ordered grid cells to physical centerline points at cell centers.
pub fn grid_path_to_centerline(
    path: &[(i32, i32)],
    grid: &GeometryGridSpec,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let mut centerline = Vec::with_capacity(path.len());

    for &(x, y) in path {
        let point = grid.cell_center(x, y);
        if !is_finite_point(point) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        push_physical_if_different(&mut centerline, point);
    }

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    Ok(centerline)
}

/// Convert a routed primitive sequence to a physical centerline by replaying
/// primitive geometry metadata.
pub fn route_to_primitive_centerline(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    Ok(route_to_primitive_centerline_with_runs(route, primitives, grid)?.centerline)
}

fn route_to_primitive_centerline_with_runs(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
) -> Result<PrimitiveCenterlineReplay, GeometryError> {
    if route.states.is_empty() {
        return Err(GeometryError::EmptyRoute);
    }
    if route.states.len() != route.primitives.len() + 1 {
        return Err(GeometryError::InvalidRouteTopology {
            states: route.states.len(),
            primitives: route.primitives.len(),
        });
    }

    let mut centerline = Vec::with_capacity(route.states.len());
    let mut straight_runs = Vec::new();
    let start = grid.cell_center(route.states[0].x, route.states[0].y);
    if !is_finite_point(start) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    push_physical_if_different(&mut centerline, start);

    for (index, primitive_id) in route.primitives.iter().enumerate() {
        let state = route.states[index];
        let next_state = route.states[index + 1];
        let primitive = primitives
            .get_primitives_for_angle(state.angle)
            .iter()
            .find(|p| p.id == *primitive_id)
            .ok_or(GeometryError::MissingPrimitive {
                id: *primitive_id,
                start_angle: state.angle,
            })?;

        let expected = grid.cell_center(next_state.x, next_state.y);
        if !is_finite_point(expected) {
            return Err(GeometryError::NonFiniteCoordinate);
        }

        match primitive.geometry {
            PrimitiveGeometry::Straight { .. } => {
                let start_index = centerline.len() - 1;
                push_physical_if_different(&mut centerline, expected);
                let end_index = centerline.len() - 1;
                if end_index > start_index {
                    let a = centerline[start_index];
                    let b = centerline[end_index];
                    let kind = if is_horizontal_segment(a, b) {
                        Some(AxisAlignedRunKind::Horizontal)
                    } else if is_vertical_segment(a, b) {
                        Some(AxisAlignedRunKind::Vertical)
                    } else {
                        None
                    };
                    if let Some(kind) = kind {
                        straight_runs.push(AxisAlignedRun {
                            start_index,
                            end_index,
                            kind,
                        });
                    }
                }
            }
            PrimitiveGeometry::Bend {
                radius_um,
                angle_delta,
            } => {
                let start_point = *centerline.last().ok_or(GeometryError::DegenerateRoute)?;
                append_circular_bend_centerline(
                    &mut centerline,
                    start_point,
                    state.angle,
                    expected,
                    next_state.angle,
                    radius_um,
                    angle_delta,
                    DEFAULT_BEND_SAMPLES_PER_90_DEG,
                )?;
            }
        }

        let actual = *centerline.last().ok_or(GeometryError::DegenerateRoute)?;
        if distance(actual, expected) > EPS {
            return Err(GeometryError::PrimitiveEndpointMismatch {
                primitive_id: primitive.id,
                expected,
                actual,
            });
        }
        if !is_finite_point(actual) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        push_physical_if_different(&mut centerline, actual);
    }

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }

    Ok(PrimitiveCenterlineReplay {
        centerline,
        straight_runs,
    })
}

/// Replace first/last centerline points with explicit source/target physical points.
pub fn snap_centerline_endpoints(
    centerline: &mut Vec<(f64, f64)>,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    if let Some(source) = source_port_um {
        if !is_finite_point(source) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        centerline[0] = source;
    }
    if let Some(target) = target_port_um {
        if !is_finite_point(target) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let last = centerline.len() - 1;
        centerline[last] = target;
    }

    let mut deduped = Vec::with_capacity(centerline.len());
    for point in centerline.iter().copied() {
        push_physical_if_different(&mut deduped, point);
    }
    *centerline = deduped;

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    Ok(())
}

/// Replay route primitives, then adjust physical endpoints to real ports.
///
/// This keeps the existing primitive-replay centerline and waveguide polygon
/// generator as the only geometry realization path. For a single straight this
/// becomes the exact physical straight between ports; for multi-segment routes
/// it anchors the realized centerline to the physical port coordinates while
/// preserving the routed interior points for later meander planning/splicing.
pub fn route_to_port_corrected_centerline(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let replay = route_to_primitive_centerline_with_runs(route, primitives, grid)?;
    let mut centerline = replay.centerline;
    if !try_apply_full_straight_port_correction(&mut centerline, source_port_um, target_port_um)? {
        absorb_endpoint_delta_into_axis_runs(
            &mut centerline,
            &replay.straight_runs,
            source_port_um,
            target_port_um,
        )?;
    }
    if centerline.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    Ok(centerline)
}

fn try_apply_full_straight_port_correction(
    centerline: &mut [(f64, f64)],
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<bool, GeometryError> {
    let (Some(source), Some(target)) = (source_port_um, target_port_um) else {
        return Ok(false);
    };
    if !is_finite_point(source) || !is_finite_point(target) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    if is_full_horizontal_centerline(centerline) && (source.1 - target.1).abs() <= EPS {
        for point in centerline.iter_mut() {
            point.1 = source.1;
        }
        centerline[0] = source;
        let last = centerline.len() - 1;
        centerline[last] = target;
        validate_full_straight_centerline(centerline, AxisAlignedRunKind::Horizontal)?;
        return Ok(true);
    }

    if is_full_vertical_centerline(centerline) && (source.0 - target.0).abs() <= EPS {
        for point in centerline.iter_mut() {
            point.0 = source.0;
        }
        centerline[0] = source;
        let last = centerline.len() - 1;
        centerline[last] = target;
        validate_full_straight_centerline(centerline, AxisAlignedRunKind::Vertical)?;
        return Ok(true);
    }

    Ok(false)
}

fn absorb_endpoint_delta_into_axis_runs(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    if let Some(source) = source_port_um {
        if !is_finite_point(source) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let start = centerline[0];
        let dx = source.0 - start.0;
        let dy = source.1 - start.1;
        absorb_source_x_delta(centerline, straight_runs, dx)?;
        absorb_source_y_delta(centerline, straight_runs, dy)?;
    }
    if let Some(target) = target_port_um {
        if !is_finite_point(target) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let last = centerline[centerline.len() - 1];
        let dx = target.0 - last.0;
        let dy = target.1 - last.1;
        absorb_target_x_delta(centerline, straight_runs, dx)?;
        absorb_target_y_delta(centerline, straight_runs, dy)?;
    }
    Ok(())
}

fn absorb_source_x_delta(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dx: f64,
) -> Result<(), GeometryError> {
    if dx.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = first_run(straight_runs, AxisAlignedRunKind::Horizontal) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().take(run.start_index + 1) {
        point.0 += dx;
    }
    Ok(())
}

fn absorb_source_y_delta(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dy: f64,
) -> Result<(), GeometryError> {
    if dy.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = first_run(straight_runs, AxisAlignedRunKind::Vertical) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().take(run.start_index + 1) {
        point.1 += dy;
    }
    Ok(())
}

fn absorb_target_x_delta(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dx: f64,
) -> Result<(), GeometryError> {
    if dx.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = last_run(straight_runs, AxisAlignedRunKind::Horizontal) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().skip(run.end_index) {
        point.0 += dx;
    }
    Ok(())
}

fn absorb_target_y_delta(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dy: f64,
) -> Result<(), GeometryError> {
    if dy.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = last_run(straight_runs, AxisAlignedRunKind::Vertical) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().skip(run.end_index) {
        point.1 += dy;
    }
    Ok(())
}

fn first_run(straight_runs: &[AxisAlignedRun], kind: AxisAlignedRunKind) -> Option<AxisAlignedRun> {
    straight_runs.iter().copied().find(|run| run.kind == kind)
}

fn last_run(straight_runs: &[AxisAlignedRun], kind: AxisAlignedRunKind) -> Option<AxisAlignedRun> {
    straight_runs.iter().copied().rfind(|run| run.kind == kind)
}

fn is_full_horizontal_centerline(centerline: &[(f64, f64)]) -> bool {
    centerline
        .windows(2)
        .all(|w| is_horizontal_segment(w[0], w[1]))
}

fn is_full_vertical_centerline(centerline: &[(f64, f64)]) -> bool {
    centerline
        .windows(2)
        .all(|w| is_vertical_segment(w[0], w[1]))
}

fn validate_full_straight_centerline(
    centerline: &[(f64, f64)],
    kind: AxisAlignedRunKind,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    let first_delta = axis_delta(centerline[0], centerline[1], kind);
    if first_delta.abs() <= EPS {
        return Err(GeometryError::ZeroLengthSegment);
    }
    for window in centerline.windows(2) {
        let delta = axis_delta(window[0], window[1], kind);
        if delta.abs() <= EPS {
            return Err(GeometryError::ZeroLengthSegment);
        }
        if first_delta * delta <= 0.0 {
            return Err(GeometryError::DegenerateRoute);
        }
        match kind {
            AxisAlignedRunKind::Horizontal => {
                if (window[1].1 - window[0].1).abs() > EPS {
                    return Err(GeometryError::DegenerateRoute);
                }
            }
            AxisAlignedRunKind::Vertical => {
                if (window[1].0 - window[0].0).abs() > EPS {
                    return Err(GeometryError::DegenerateRoute);
                }
            }
        }
    }
    Ok(())
}

fn axis_delta(a: (f64, f64), b: (f64, f64), kind: AxisAlignedRunKind) -> f64 {
    match kind {
        AxisAlignedRunKind::Horizontal => b.0 - a.0,
        AxisAlignedRunKind::Vertical => b.1 - a.1,
    }
}

fn is_horizontal_segment(a: (f64, f64), b: (f64, f64)) -> bool {
    (b.1 - a.1).abs() <= EPS && (b.0 - a.0).abs() > EPS
}

fn is_vertical_segment(a: (f64, f64), b: (f64, f64)) -> bool {
    (b.0 - a.0).abs() <= EPS && (b.1 - a.1).abs() > EPS
}

/// Build deterministic local port access from a physical port to a grid anchor.
pub fn build_port_access(
    port: &PortInput,
    grid: &StaticGridSpec,
    config: &PortAccessConfig,
) -> Result<PortAccess, PortAccessError> {
    validate_port_access_config(config)?;
    if !port.x.is_finite() || !port.y.is_finite() {
        return Err(PortAccessError::NonFinitePortCoordinate {
            port_name: port.name.clone(),
        });
    }

    let port_angle = orientation_to_angle(port.orientation);
    let (anchor_cell, anchor_angle, mut access_centerline_um) =
        select_anchor_and_build_access(port, grid, config)?;
    let anchor_point_um = grid_cell_center(anchor_cell.0, anchor_cell.1, grid);

    if access_centerline_um.iter().any(|&p| !is_finite_point(p)) {
        return Err(PortAccessError::NonFiniteAccessCoordinate {
            port_name: port.name.clone(),
        });
    }
    if access_centerline_um.len() < 2 {
        return Err(PortAccessError::ZeroLengthAccess {
            port_name: port.name.clone(),
        });
    }
    if access_centerline_um
        .windows(2)
        .any(|w| distance(w[0], w[1]) <= EPS)
    {
        return Err(PortAccessError::ZeroLengthAccess {
            port_name: port.name.clone(),
        });
    }

    access_centerline_um[0] = (port.x, port.y);
    let last = access_centerline_um.len() - 1;
    access_centerline_um[last] = anchor_point_um;

    Ok(PortAccess {
        port_name: port.name.clone(),
        port_point_um: (port.x, port.y),
        anchor_cell,
        anchor_point_um,
        port_angle,
        anchor_angle,
        entry_angle: anchor_angle,
        access_centerline_um,
    })
}

/// Build deterministic local port accesses for all physical ports.
pub fn build_port_accesses(
    ports: &[PortInput],
    grid: &StaticGridSpec,
    config: &PortAccessConfig,
) -> Result<Vec<PortAccess>, PortAccessError> {
    ports
        .iter()
        .map(|port| build_port_access(port, grid, config))
        .collect()
}

/// Generate one closed mitered/beveled waveguide polygon from a centerline.
pub fn generate_waveguide_polygon(
    centerline: &[(f64, f64)],
    width_um: f64,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    if !width_um.is_finite() || width_um <= 0.0 {
        return Err(GeometryError::InvalidWidth(width_um));
    }
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    let half_width = width_um / 2.0;
    let segment_count = centerline.len() - 1;

    let mut normals = Vec::with_capacity(segment_count);
    for i in 0..segment_count {
        let dir = sub(centerline[i + 1], centerline[i]);
        let len = length(dir);
        if len <= EPS {
            return Err(GeometryError::ZeroLengthSegment);
        }

        let unit = (dir.0 / len, dir.1 / len);
        normals.push((-unit.1, unit.0));
    }

    let mut left = Vec::with_capacity(centerline.len() + 4);
    let mut right = Vec::with_capacity(centerline.len() + 4);

    left.push(add(centerline[0], scale(normals[0], half_width)));
    right.push(sub(centerline[0], scale(normals[0], half_width)));

    for i in 1..centerline.len() - 1 {
        append_join(
            &mut left,
            centerline[i],
            normals[i - 1],
            normals[i],
            half_width,
            true,
        );
        append_join(
            &mut right,
            centerline[i],
            scale(normals[i - 1], -1.0),
            scale(normals[i], -1.0),
            half_width,
            false,
        );
    }

    let last_index = centerline.len() - 1;
    let last_normal = normals[normals.len() - 1];
    left.push(add(centerline[last_index], scale(last_normal, half_width)));
    right.push(sub(centerline[last_index], scale(last_normal, half_width)));

    let mut polygon = Vec::with_capacity(left.len() + right.len() + 1);
    polygon.extend(left);
    polygon.extend(right.into_iter().rev());
    if polygon.len() < 3 {
        return Err(GeometryError::DegenerateRoute);
    }

    polygon.push(polygon[0]);
    Ok(polygon)
}

/// Exact grid-footprint route realization.
///
/// This preserves the exact A* state/primitive footprint as a polyline through
/// occupied grid-cell centers, then expands that centerline into a waveguide
/// polygon. The Python flow uses primitive replay instead so bend geometry is
/// realized as sampled curves.
pub fn realize_route_polygon(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    _source_port_um: Option<(f64, f64)>,
    _target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let path = route_to_grid_path(route, primitives)?;
    let centerline = grid_path_to_centerline(&path, grid)?;
    generate_waveguide_polygon(&centerline, width_um)
}

/// Realize route polygon from primitive-replay centerline generation.
pub fn realize_route_polygon_from_primitives(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    generate_waveguide_polygon(&centerline, width_um)
}

/// Realize route polygon after anchoring endpoints to physical ports.
pub fn realize_route_polygon_with_endpoint_correction(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_port_corrected_centerline(
        route,
        primitives,
        grid,
        source_port_um,
        target_port_um,
    )?;
    generate_waveguide_polygon(&centerline, width_um)
}

pub fn centerline_length_um(centerline: &[(f64, f64)]) -> Result<f64, GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    let mut total = 0.0;
    for window in centerline.windows(2) {
        let segment_length = distance(window[0], window[1]);
        if segment_length <= EPS {
            return Err(GeometryError::ZeroLengthSegment);
        }
        total += segment_length;
    }
    Ok(total)
}

pub fn meander_box_to_grid_rect(
    box_um: MeanderBox,
    grid: &GeometryGridSpec,
    clearance_radius_cells: i32,
) -> Result<GridRect, GeometryError> {
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

    let gx0 = ((box_um.min_x_um - grid.origin_x_um) / grid.grid_size_um).floor() as i32;
    let gy0 = ((box_um.min_y_um - grid.origin_y_um) / grid.grid_size_um).floor() as i32;
    let gx1 = ((box_um.max_x_um - grid.origin_x_um) / grid.grid_size_um).ceil() as i32 - 1;
    let gy1 = ((box_um.max_y_um - grid.origin_y_um) / grid.grid_size_um).ceil() as i32 - 1;

    let rect = GridRect {
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
) -> Result<GridRect, GeometryError> {
    check_meander_box_free_with_occupancy(box_um, grid, prefix, clearance_radius_cells)
}

fn check_meander_box_free_with_occupancy<Q: RectOccupancyQuery>(
    box_um: MeanderBox,
    grid: &GeometryGridSpec,
    occupancy: &Q,
    clearance_radius_cells: i32,
) -> Result<GridRect, GeometryError> {
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

pub fn check_meander_box_free(
    box_um: MeanderBox,
    grid: &GeometryGridSpec,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    clearance_radius_cells: i32,
) -> Result<GridRect, GeometryError> {
    let prefix = DenseOccupancyPrefix::from_obstacle_map(obstacle_map, opened_cells);
    check_meander_box_free_with_prefix(box_um, grid, &prefix, clearance_radius_cells)
}

pub fn cells_in_grid_rect(rect: GridRect) -> Vec<(i32, i32)> {
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

#[derive(Clone, Debug, PartialEq)]
pub struct RouteAnalyticMeanderPlan {
    pub selected_segment_index: usize,
    pub selected_segment: StraightSegment,
    pub plan: AnalyticMeanderPlan,
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
    pub selected_grid_rect: GridRect,
    pub plan: AnalyticMeanderPlan,
    pub profile: AutoMeanderPlanningProfile,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AutoMeanderPlanningProfile {
    pub total_s: f64,
    pub run_extraction_s: f64,
    pub footprint_s: f64,
    pub free_interval_s: f64,
    pub box_check_s: f64,
    pub analytic_plan_s: f64,
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
    pub selected_grid_rect: Option<GridRect>,
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
    let first_idx = ((allowed_min_coord - axis_origin) / grid.grid_size_um).floor() as i32;
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

pub(crate) fn select_meander_segment(
    centerline: &[(f64, f64)],
    available_box: MeanderBox,
) -> Result<usize, GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    let mut best: Option<(usize, f64)> = None;
    for (i, w) in centerline.windows(2).enumerate() {
        let p0 = w[0];
        let p1 = w[1];
        let dx = (p1.0 - p0.0).abs();
        let dy = (p1.1 - p0.1).abs();
        let is_horizontal = dy <= EPS && dx > EPS;
        let is_vertical = dx <= EPS && dy > EPS;
        if !(is_horizontal || is_vertical) {
            continue;
        }
        let in0 = p0.0 >= available_box.min_x_um - EPS
            && p0.0 <= available_box.max_x_um + EPS
            && p0.1 >= available_box.min_y_um - EPS
            && p0.1 <= available_box.max_y_um + EPS;
        let in1 = p1.0 >= available_box.min_x_um - EPS
            && p1.0 <= available_box.max_x_um + EPS
            && p1.1 >= available_box.min_y_um - EPS
            && p1.1 <= available_box.max_y_um + EPS;
        if !(in0 && in1) {
            continue;
        }
        let len = distance(p0, p1);
        match best {
            Some((_, cur)) if cur >= len => {}
            _ => best = Some((i, len)),
        }
    }

    best.map(|(idx, _)| idx)
        .ok_or(GeometryError::NoMeanderCandidateSegment)
}

pub fn plan_analytic_meander_for_route(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    requested_extra_length_um: f64,
    min_bend_radius_um: f64,
    min_straight_um: f64,
    max_bumps: usize,
    meander_side: MeanderSide,
    available_box: MeanderBox,
    mode: MeanderPlanningMode,
) -> Result<RouteAnalyticMeanderPlan, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let selected_segment_index = select_meander_segment(&centerline, available_box)?;
    let selected_segment = StraightSegment {
        start: PhysicalPoint {
            x_um: centerline[selected_segment_index].0,
            y_um: centerline[selected_segment_index].1,
        },
        end: PhysicalPoint {
            x_um: centerline[selected_segment_index + 1].0,
            y_um: centerline[selected_segment_index + 1].1,
        },
    };
    let cfg = AnalyticMeanderConfig {
        requested_extra_length_um,
        min_bend_radius_um,
        min_straight_um,
        max_bumps,
        max_meander_height_um: 1.0e12,
        side: meander_side,
        mode,
    };
    let plan = plan_analytic_meander(selected_segment, available_box, &cfg)
        .map_err(GeometryError::MeanderPlanningFailed)?;

    Ok(RouteAnalyticMeanderPlan {
        selected_segment_index,
        selected_segment,
        plan,
    })
}

pub fn plan_auto_analytic_meander_for_route(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
) -> Result<AutoRouteAnalyticMeanderPlan, GeometryError> {
    plan_auto_analytic_meander_for_route_depth_sweep(
        route,
        primitives,
        grid,
        obstacle_map,
        opened_cells,
        config,
        &[config.box_depth_um],
    )
}

pub fn plan_auto_analytic_meander_for_route_depth_sweep(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
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
    for depth_um in box_depths_um {
        if !depth_um.is_finite() || *depth_um <= 0.0 {
            return Err(GeometryError::InvalidMeanderBox);
        }
    }

    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let prefix = DenseOccupancyPrefix::from_obstacle_map(obstacle_map, opened_cells);
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
    let mut selected: Option<(MeanderBox, GridRect, AnalyticMeanderPlan, f64)> = None;
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
                    &prefix,
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
                    let rect = match check_meander_box_free_with_prefix(
                        box_um,
                        grid,
                        &prefix,
                        config.clearance_radius_cells,
                    ) {
                        Ok(v) => v,
                        Err(_) => {
                            profile.box_check_s += box_check_start.elapsed().as_secs_f64();
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
                        Err(_) => {
                            profile.analytic_plan_s += analytic_plan_start.elapsed().as_secs_f64();
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

pub fn plan_auto_analytic_meander_for_route_depth_sweep_with_prefix(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    base_prefix: &DenseOccupancyPrefix,
    opened_cells: Option<&FxHashSet<CellKey>>,
    extra_blocked_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
    box_depths_um: &[f64],
) -> Result<AutoRouteAnalyticMeanderPlan, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
        &centerline,
        grid,
        base_prefix,
        opened_cells,
        extra_blocked_cells,
        config,
        box_depths_um,
    )
}

pub fn plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
    centerline: &[(f64, f64)],
    grid: &GeometryGridSpec,
    base_prefix: &DenseOccupancyPrefix,
    opened_cells: Option<&FxHashSet<CellKey>>,
    extra_blocked_cells: Option<&FxHashSet<CellKey>>,
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

    let occupancy = OverlayOccupancyQuery::new(base_prefix, opened_cells, extra_blocked_cells);
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
    let mut selected: Option<(MeanderBox, GridRect, AnalyticMeanderPlan, f64)> = None;
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
                        Err(_) => {
                            profile.analytic_plan_s += analytic_plan_start.elapsed().as_secs_f64();
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

pub fn probe_auto_analytic_meander_for_route_depth_sweep_with_prefix(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    base_prefix: &DenseOccupancyPrefix,
    opened_cells: Option<&FxHashSet<CellKey>>,
    extra_blocked_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
    box_depths_um: &[f64],
) -> Result<AutoRouteAnalyticMeanderProbe, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
        &centerline,
        grid,
        base_prefix,
        opened_cells,
        extra_blocked_cells,
        config,
        box_depths_um,
    )
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

    let occupancy = OverlayOccupancyQuery::new(base_prefix, opened_cells, extra_blocked_cells);
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

pub(crate) fn splice_meander_into_centerline(
    centerline: &[(f64, f64)],
    segment_index: usize,
    meander: &[PhysicalPoint],
) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(centerline.len() + meander.len());
    for p in centerline.iter().take(segment_index + 1).copied() {
        push_physical_if_different(&mut out, p);
    }
    for p in meander.iter().copied() {
        push_physical_if_different(&mut out, (p.x_um, p.y_um));
    }
    for p in centerline.iter().skip(segment_index + 1).copied() {
        push_physical_if_different(&mut out, p);
    }
    out
}

pub(crate) fn splice_meander_into_centerline_range(
    centerline: &[(f64, f64)],
    start_index: usize,
    end_index: usize,
    meander: &[PhysicalPoint],
) -> Result<Vec<(f64, f64)>, GeometryError> {
    if centerline.len() < 2 || start_index >= end_index || end_index >= centerline.len() {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let mut out = Vec::with_capacity(centerline.len() + meander.len());
    for p in centerline.iter().take(start_index).copied() {
        push_physical_if_different(&mut out, p);
    }
    for p in meander.iter().copied() {
        push_physical_if_different(&mut out, (p.x_um, p.y_um));
    }
    for p in centerline.iter().skip(end_index + 1).copied() {
        push_physical_if_different(&mut out, p);
    }
    Ok(out)
}

pub fn realize_route_polygon_with_analytic_meander(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    requested_extra_length_um: f64,
    min_bend_radius_um: f64,
    min_straight_um: f64,
    max_bumps: usize,
    meander_side: MeanderSide,
    available_box: MeanderBox,
    mode: MeanderPlanningMode,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let route_plan = plan_analytic_meander_for_route(
        route,
        primitives,
        grid,
        requested_extra_length_um,
        min_bend_radius_um,
        min_straight_um,
        max_bumps,
        meander_side,
        available_box,
        mode,
    )?;

    let modified = splice_meander_into_centerline(
        &centerline,
        route_plan.selected_segment_index,
        &route_plan.plan.centerline,
    );
    if modified.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if modified.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if modified.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    generate_waveguide_polygon(&modified, width_um)
}

pub fn realize_route_polygon_with_checked_analytic_meander_box(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    requested_extra_length_um: f64,
    min_bend_radius_um: f64,
    min_straight_um: f64,
    max_bumps: usize,
    meander_side: MeanderSide,
    available_box: MeanderBox,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    clearance_radius_cells: i32,
    mode: MeanderPlanningMode,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let _rect = check_meander_box_free(
        available_box,
        grid,
        obstacle_map,
        opened_cells,
        clearance_radius_cells,
    )?;
    realize_route_polygon_with_analytic_meander(
        route,
        primitives,
        grid,
        width_um,
        requested_extra_length_um,
        min_bend_radius_um,
        min_straight_um,
        max_bumps,
        meander_side,
        available_box,
        mode,
    )
}

pub fn realize_route_polygon_with_auto_checked_analytic_meander(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let auto = plan_auto_analytic_meander_for_route(
        route,
        primitives,
        grid,
        obstacle_map,
        opened_cells,
        config,
    )?;
    let modified = splice_meander_into_centerline_range(
        &centerline,
        auto.selected_run_start_index,
        auto.selected_run_end_index,
        &auto.replacement_centerline,
    )?;
    if modified.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if modified.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if modified.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    generate_waveguide_polygon(&modified, width_um)
}

pub fn realize_route_polygon_from_auto_plan(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    auto_plan: &AutoRouteAnalyticMeanderPlan,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let modified = splice_meander_into_centerline_range(
        &centerline,
        auto_plan.selected_run_start_index,
        auto_plan.selected_run_end_index,
        &auto_plan.replacement_centerline,
    )?;
    if modified.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if modified.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if modified.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    generate_waveguide_polygon(&modified, width_um)
}

/// Full route realization pipeline with explicit source/target port access.
pub fn realize_route_polygon_with_port_access(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    source_access: Option<&PortAccess>,
    target_access: Option<&PortAccess>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let primitive_centerline = route_to_primitive_centerline(route, primitives, grid)?;

    if let Some(source) = source_access {
        if distance(primitive_centerline[0], source.anchor_point_um) > EPS {
            return Err(GeometryError::RouteStartDoesNotMatchSourceAnchor {
                route_start: primitive_centerline[0],
                source_anchor: source.anchor_point_um,
            });
        }
    }
    if let Some(target) = target_access {
        let route_end = primitive_centerline[primitive_centerline.len() - 1];
        if distance(route_end, target.anchor_point_um) > EPS {
            return Err(GeometryError::RouteEndDoesNotMatchTargetAnchor {
                route_end,
                target_anchor: target.anchor_point_um,
            });
        }
    }

    let mut centerline = Vec::with_capacity(
        primitive_centerline.len()
            + source_access
                .map(|a| a.access_centerline_um.len())
                .unwrap_or(0)
            + target_access
                .map(|a| a.access_centerline_um.len())
                .unwrap_or(0),
    );

    if let Some(source) = source_access {
        for point in source.access_centerline_um.iter().copied() {
            push_physical_if_different(&mut centerline, point);
        }
    }
    for point in primitive_centerline.iter().copied() {
        push_physical_if_different(&mut centerline, point);
    }
    if let Some(target) = target_access {
        for point in target.access_centerline_um.iter().rev().copied() {
            push_physical_if_different(&mut centerline, point);
        }
    }

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    generate_waveguide_polygon(&centerline, width_um)
}

fn validate_port_access_config(config: &PortAccessConfig) -> Result<(), PortAccessError> {
    if !config.min_straight_um.is_finite() || config.min_straight_um < 0.0 {
        return Err(PortAccessError::InvalidConfig(
            "min_straight_um must be finite and >= 0".to_string(),
        ));
    }
    if config.max_anchor_search_cells < 0 {
        return Err(PortAccessError::InvalidConfig(
            "max_anchor_search_cells must be >= 0".to_string(),
        ));
    }
    if !config.min_bend_radius_um.is_finite() || config.min_bend_radius_um < 0.0 {
        return Err(PortAccessError::InvalidConfig(
            "min_bend_radius_um must be finite and >= 0".to_string(),
        ));
    }
    Ok(())
}

fn select_anchor_and_build_access(
    port: &PortInput,
    grid: &StaticGridSpec,
    config: &PortAccessConfig,
) -> Result<((i32, i32), u8, Vec<(f64, f64)>), PortAccessError> {
    let base_cell = physical_to_grid(port.x, port.y, grid);
    let port_angle = orientation_to_angle(port.orientation);
    let dir = angle_to_unit_vector(port_angle);
    let lateral_dir = rotate_left(dir);
    let mut candidates: Vec<(i32, i32, i32, i32, f64, f64)> = Vec::new();
    for radius in 1..=config.max_anchor_search_cells.max(1) {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                if dx.abs().max(dy.abs()) != radius {
                    continue;
                }
                let candidate = (base_cell.0 + dx, base_cell.1 + dy);
                if !in_bounds(candidate, grid) {
                    continue;
                }
                let anchor_point = grid_cell_center(candidate.0, candidate.1, grid);
                let delta = sub(anchor_point, (port.x, port.y));
                let forward = dot(delta, dir);
                if forward <= EPS {
                    continue;
                }
                let lateral = dot(delta, lateral_dir).abs();
                candidates.push((radius, dx, dy, candidate.0, lateral, forward));
            }
        }
    }
    candidates.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.4.total_cmp(&b.4))
            .then_with(|| b.5.total_cmp(&a.5))
            .then_with(|| a.3.cmp(&b.3))
            .then_with(|| a.2.cmp(&b.2))
    });
    for (_, dx, dy, _, _, _) in candidates {
        let candidate = (base_cell.0 + dx, base_cell.1 + dy);
        let anchor_point = grid_cell_center(candidate.0, candidate.1, grid);
        let anchor_angle = port_angle;
        if let Ok(centerline) = build_access_centerline(
            (port.x, port.y),
            anchor_point,
            port_angle,
            anchor_angle,
            &port.name,
            config,
        ) {
            return Ok((candidate, anchor_angle, centerline));
        }
    }

    Err(PortAccessError::AnchorSearchFailed {
        port_name: port.name.clone(),
    })
}

fn build_access_centerline(
    port_point: (f64, f64),
    anchor_point: (f64, f64),
    port_angle: u8,
    anchor_angle: u8,
    port_name: &str,
    config: &PortAccessConfig,
) -> Result<Vec<(f64, f64)>, PortAccessError> {
    if !is_finite_point(port_point) || !is_finite_point(anchor_point) {
        return Err(PortAccessError::NonFiniteAccessCoordinate {
            port_name: port_name.to_string(),
        });
    }

    if anchor_angle != port_angle {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }

    let u = angle_to_unit_vector(port_angle);
    let v = rotate_left(u);
    let delta = sub(anchor_point, port_point);
    let local_dx = dot(delta, u);
    let local_dy = dot(delta, v);
    if local_dx <= EPS {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }

    let launch_straight = config.min_straight_um.max(EPS * 10.0);
    let landing_straight = config.min_straight_um.max(EPS * 10.0);
    let mut out = Vec::new();

    // Build in local frame (port at origin, +x along port tangent, +y left-normal), then map back.
    let to_world =
        |p: (f64, f64)| -> (f64, f64) { add(port_point, add(scale(u, p.0), scale(v, p.1))) };
    push_physical_if_different(&mut out, to_world((0.0, 0.0)));

    if local_dy.abs() <= EPS {
        let x1 = launch_straight.min(local_dx);
        push_physical_if_different(&mut out, to_world((x1, 0.0)));
        push_physical_if_different(&mut out, to_world((local_dx, 0.0)));
        if out.len() < 2 {
            return Err(PortAccessError::ZeroLengthAccess {
                port_name: port_name.to_string(),
            });
        }
        return Ok(out);
    }

    let radius = if config.min_bend_radius_um <= EPS {
        (local_dy.abs() / 2.0).max(EPS * 10.0)
    } else {
        config.min_bend_radius_um
    };
    if local_dy.abs() + EPS < 2.0 * radius {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }
    let required_dx = launch_straight + landing_straight + 2.0 * radius;
    if local_dx + EPS < required_dx {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }
    let final_straight = local_dx - required_dx;
    if final_straight < -EPS {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }

    let sign = if local_dy >= 0.0 { 1.0 } else { -1.0 };
    let abs_dy = local_dy.abs();
    let p1 = (launch_straight, 0.0);
    let p2 = (launch_straight + radius, sign * radius);
    let p3 = (launch_straight + radius, sign * (abs_dy - radius));
    let p4 = (launch_straight + 2.0 * radius, sign * abs_dy);
    let p5 = (
        launch_straight + 2.0 * radius + final_straight,
        sign * abs_dy,
    );
    let p6 = (local_dx - landing_straight, sign * abs_dy);
    let p7 = (local_dx, sign * abs_dy);

    push_physical_if_different(&mut out, to_world(p1));
    append_arc_samples(
        &mut out,
        to_world(p1),
        to_world(p2),
        to_world((launch_straight, sign * radius)),
        sign > 0.0,
    );
    push_physical_if_different(&mut out, to_world(p3));
    append_arc_samples(
        &mut out,
        to_world(p3),
        to_world(p4),
        to_world((launch_straight + 2.0 * radius, sign * (abs_dy - radius))),
        sign < 0.0,
    );
    push_physical_if_different(&mut out, to_world(p5));
    push_physical_if_different(&mut out, to_world(p6));
    push_physical_if_different(&mut out, to_world(p7));

    if out.iter().any(|p| !is_finite_point(*p))
        || out.windows(2).any(|w| distance(w[0], w[1]) <= EPS)
    {
        return Err(PortAccessError::NonFiniteAccessCoordinate {
            port_name: port_name.to_string(),
        });
    }
    Ok(out)
}

fn append_arc_samples(
    out: &mut Vec<(f64, f64)>,
    start: (f64, f64),
    end: (f64, f64),
    center: (f64, f64),
    ccw: bool,
) {
    let radius = distance(start, center);
    if radius <= EPS {
        push_physical_if_different(out, end);
        return;
    }
    let a0 = (start.1 - center.1).atan2(start.0 - center.0);
    let mut a1 = (end.1 - center.1).atan2(end.0 - center.0);
    if ccw {
        while a1 <= a0 {
            a1 += std::f64::consts::TAU;
        }
    } else {
        while a1 >= a0 {
            a1 -= std::f64::consts::TAU;
        }
    }
    let steps = 4usize;
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        let a = a0 + (a1 - a0) * t;
        push_physical_if_different(
            out,
            (center.0 + radius * a.cos(), center.1 + radius * a.sin()),
        );
    }
    push_physical_if_different(out, end);
}

fn in_bounds(cell: (i32, i32), grid: &StaticGridSpec) -> bool {
    cell.0 >= 0 && cell.0 < grid.width && cell.1 >= 0 && cell.1 < grid.height
}

fn orientation_to_angle(orientation: Option<f64>) -> u8 {
    let value = orientation.unwrap_or(0.0).rem_euclid(360.0);
    (value / 45.0).round().rem_euclid(8.0) as u8
}

fn append_join(
    out: &mut Vec<(f64, f64)>,
    point: (f64, f64),
    normal_a: (f64, f64),
    normal_b: (f64, f64),
    half_width: f64,
    positive_side: bool,
) {
    let offset_a = add(point, scale(normal_a, half_width));
    let offset_b = add(point, scale(normal_b, half_width));

    let bisector = add(normal_a, normal_b);
    let bisector_len = length(bisector);

    if bisector_len <= EPS {
        out.push(offset_a);
        out.push(offset_b);
        return;
    }

    let miter_dir = (bisector.0 / bisector_len, bisector.1 / bisector_len);
    let denom = dot(miter_dir, normal_a);

    if denom.abs() <= EPS {
        out.push(offset_a);
        out.push(offset_b);
        return;
    }

    let miter_len = half_width / denom;
    if !miter_len.is_finite() || miter_len.abs() > MITER_LIMIT * half_width {
        if positive_side {
            out.push(offset_a);
            out.push(offset_b);
        } else {
            out.push(offset_b);
            out.push(offset_a);
        }
        return;
    }

    out.push(add(point, scale(miter_dir, miter_len)));
}

fn push_if_different(points: &mut Vec<(i32, i32)>, point: (i32, i32)) {
    if points.last().copied() != Some(point) {
        points.push(point);
    }
}

fn push_physical_if_different(points: &mut Vec<(f64, f64)>, point: (f64, f64)) {
    if points
        .last()
        .map(|&last| distance(last, point) > EPS)
        .unwrap_or(true)
    {
        points.push(point);
    }
}

fn is_finite_point(point: (f64, f64)) -> bool {
    point.0.is_finite() && point.1.is_finite()
}

fn direction(a: (i32, i32), b: (i32, i32)) -> (i32, i32) {
    ((b.0 - a.0).signum(), (b.1 - a.1).signum())
}

fn add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

fn sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

fn scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

fn dot(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn cross(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

fn length(a: (f64, f64)) -> f64 {
    dot(a, a).sqrt()
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    length(sub(a, b))
}

fn append_circular_bend_centerline(
    out: &mut Vec<(f64, f64)>,
    start_point: (f64, f64),
    start_angle: u8,
    end_point: (f64, f64),
    end_angle: u8,
    radius_um: f64,
    angle_delta: i8,
    samples_per_90_deg: usize,
) -> Result<(), GeometryError> {
    if !start_point.0.is_finite()
        || !start_point.1.is_finite()
        || !end_point.0.is_finite()
        || !end_point.1.is_finite()
        || !radius_um.is_finite()
        || radius_um <= 0.0
    {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    let start_dir = angle_to_unit_vector(start_angle);
    let end_dir = angle_to_unit_vector(end_angle);
    let chord = sub(end_point, start_point);
    let denom = cross(start_dir, end_dir);
    if denom.abs() <= EPS {
        push_physical_if_different(out, end_point);
        return Ok(());
    }
    let in_len = cross(chord, end_dir) / denom;
    let out_len = cross(start_dir, chord) / denom;
    if !in_len.is_finite() || !out_len.is_finite() || in_len <= EPS || out_len <= EPS {
        push_physical_if_different(out, end_point);
        return Ok(());
    }
    let corner = add(start_point, scale(start_dir, in_len));
    let turn_abs = (angle_delta as f64).abs() * (std::f64::consts::PI / 4.0);
    let trim = radius_um * (turn_abs / 2.0).tan();

    let trim_eff = trim.min(in_len - EPS).min(out_len - EPS);
    if !trim_eff.is_finite() || trim_eff <= EPS {
        push_physical_if_different(out, end_point);
        return Ok(());
    }

    let t_in = sub(corner, scale(start_dir, trim_eff));
    let t_out = add(corner, scale(end_dir, trim_eff));

    push_physical_if_different(out, t_in);

    let left_turn = angle_delta > 0;
    let n_start = if left_turn {
        rotate_left(start_dir)
    } else {
        rotate_right(start_dir)
    };
    let n_end = if left_turn {
        rotate_left(end_dir)
    } else {
        rotate_right(end_dir)
    };
    let c0 = add(t_in, scale(n_start, radius_um));
    let c1 = add(t_out, scale(n_end, radius_um));
    let center = scale(add(c0, c1), 0.5);

    let a0 = (t_in.1 - center.1).atan2(t_in.0 - center.0);
    let mut a1 = (t_out.1 - center.1).atan2(t_out.0 - center.0);
    if left_turn {
        while a1 <= a0 {
            a1 += std::f64::consts::TAU;
        }
    } else {
        while a1 >= a0 {
            a1 -= std::f64::consts::TAU;
        }
    }

    let arc_span = (a1 - a0).abs();
    let mut steps =
        ((arc_span / (std::f64::consts::PI / 2.0)) * samples_per_90_deg as f64).ceil() as usize;
    steps = steps.max(2);
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        let a = a0 + (a1 - a0) * t;
        let p = (
            center.0 + radius_um * a.cos(),
            center.1 + radius_um * a.sin(),
        );
        push_physical_if_different(out, p);
    }

    push_physical_if_different(out, t_out);
    push_physical_if_different(out, end_point);
    Ok(())
}

fn angle_to_unit_vector(angle: u8) -> (f64, f64) {
    let a = angle_to_radians(angle);
    (a.cos(), a.sin())
}

fn angle_to_radians(angle: u8) -> f64 {
    (angle as f64) * (std::f64::consts::PI / 4.0)
}

fn rotate_left(v: (f64, f64)) -> (f64, f64) {
    (-v.1, v.0)
}

fn rotate_right(v: (f64, f64)) -> (f64, f64) {
    (v.1, -v.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};
    use crate::static_obstacle_builder::make_grid_spec;
    use rustc_hash::FxHashSet;

    fn grid() -> GeometryGridSpec {
        GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap()
    }

    fn test_lib() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
        })
    }

    fn primitive_id_for<F>(lib: &PrimitiveLibrary, start_angle: u8, predicate: F) -> u16
    where
        F: Fn(&crate::primitives::Primitive) -> bool,
    {
        lib.get_primitives_for_angle(start_angle)
            .iter()
            .find(|p| predicate(p))
            .expect("missing primitive for test setup")
            .id
    }

    fn states_from_primitives(
        lib: &PrimitiveLibrary,
        start: State,
        primitive_ids: &[u16],
    ) -> Vec<State> {
        let mut out = Vec::with_capacity(primitive_ids.len() + 1);
        out.push(start);
        let mut cur = start;
        for primitive_id in primitive_ids {
            let primitive = lib
                .get_primitives_for_angle(cur.angle)
                .iter()
                .find(|p| p.id == *primitive_id)
                .expect("primitive id must exist for current angle");
            cur = State::new(
                cur.x + primitive.dx,
                cur.y + primitive.dy,
                primitive.end_angle,
            );
            out.push(cur);
        }
        out
    }

    #[test]
    fn compresses_straight_route_to_two_waypoints() {
        let states = vec![
            State::new(1, 2, 0),
            State::new(2, 2, 0),
            State::new(5, 2, 0),
        ];

        assert_eq!(compress_route_waypoints(&states), vec![(1, 2), (5, 2)]);
    }

    #[test]
    fn primitive_path_preserves_ninety_degree_turn_corner() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(2, 2, 2)],
            primitives: vec![4],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(2, 2, 2),
            reached_target: State::new(2, 2, 2),
            stats: Default::default(),
        };

        let path = route_to_grid_path(&route, &lib).unwrap();
        assert_eq!(path, vec![(1, 1), (2, 1), (2, 2)]);

        let waypoints = compress_grid_waypoints(&path);
        assert_eq!(waypoints, vec![(1, 1), (2, 1), (2, 2)]);
    }

    #[test]
    fn centerline_uses_cell_centers() {
        let centerline = grid_path_to_centerline(&[(0, 0), (2, 0)], &grid()).unwrap();

        assert_eq!(centerline, vec![(0.5, 0.5), (2.5, 0.5)]);
    }

    #[test]
    fn primitive_replay_straight_centerline_uses_state_endpoints() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let centerline = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        assert_eq!(centerline, vec![(1.5, 2.5), (5.5, 2.5)]);
    }

    #[test]
    fn primitive_replay_bend_centerline_dispatches_bend_branch() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(2, 2, 2)],
            primitives: vec![4],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(2, 2, 2),
            reached_target: State::new(2, 2, 2),
            stats: Default::default(),
        };
        let centerline = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        assert_eq!(centerline.first().copied(), Some((1.5, 1.5)));
        assert_eq!(centerline.last().copied(), Some((2.5, 2.5)));
        assert!(centerline.len() > 2);
        assert!(centerline
            .iter()
            .all(|(x, y)| x.is_finite() && y.is_finite()));
        assert!(centerline.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn primitive_replay_45_and_right_bends_are_sampled_and_oriented() {
        let lib = test_lib();
        let left_45_pid = lib.get_primitives_for_angle(0)[2].id;
        let right_45_pid = lib.get_primitives_for_angle(0)[3].id;
        let left_45 = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(3, 2, 1)],
            primitives: vec![left_45_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(3, 2, 1),
            reached_target: State::new(3, 2, 1),
            stats: Default::default(),
        };
        let right_45 = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(3, 0, 7)],
            primitives: vec![right_45_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(3, 0, 7),
            reached_target: State::new(3, 0, 7),
            stats: Default::default(),
        };
        let left = route_to_primitive_centerline(&left_45, &lib, &grid()).unwrap();
        let right = route_to_primitive_centerline(&right_45, &lib, &grid()).unwrap();
        assert!(left.len() > 2);
        assert!(right.len() > 2);
        assert_eq!(left.first().copied(), Some((1.5, 1.5)));
        assert_eq!(left.last().copied(), Some((3.5, 2.5)));
        assert_eq!(right.first().copied(), Some((1.5, 1.5)));
        assert_eq!(right.last().copied(), Some((3.5, 0.5)));

        let left_mid = left[left.len() / 2];
        let right_mid = right[right.len() / 2];
        assert!(left_mid.1 > 1.5);
        assert!(right_mid.1 < 1.5);
    }

    #[test]
    fn primitive_replay_diagonal_start_bend_tail_follows_end_angle() {
        let lib = test_lib();
        let left_45_pid = lib.get_primitives_for_angle(1)[2].id;
        let route = RouteResult {
            states: vec![State::new(1, 1, 1), State::new(2, 3, 2)],
            primitives: vec![left_45_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(2, 3, 2),
            reached_target: State::new(2, 3, 2),
            stats: Default::default(),
        };

        let centerline = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        let tail_start = centerline[centerline.len() - 2];
        let tail_end = centerline[centerline.len() - 1];

        assert!((tail_end.0 - tail_start.0).abs() <= EPS);
        assert!(tail_end.1 > tail_start.1);
    }

    #[test]
    fn primitive_replay_rejects_invalid_topology() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let err = route_to_primitive_centerline(&route, &lib, &grid()).unwrap_err();
        assert!(matches!(err, GeometryError::InvalidRouteTopology { .. }));
    }

    #[test]
    fn primitive_replay_rejects_missing_primitive_id() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![9999],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let err = route_to_primitive_centerline(&route, &lib, &grid()).unwrap_err();
        assert!(matches!(err, GeometryError::MissingPrimitive { .. }));
    }

    #[test]
    fn snapped_endpoints_override_cell_centers() {
        let mut centerline = vec![(0.5, 0.5), (2.5, 0.5)];
        snap_centerline_endpoints(&mut centerline, Some((0.0, 0.0)), Some((3.0, 0.0))).unwrap();
        assert_eq!(centerline, vec![(0.0, 0.0), (3.0, 0.0)]);
    }

    #[test]
    fn port_corrected_single_straight_absorbs_x_offsets_without_new_points() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };

        let centerline = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((1.2, 2.5)),
            Some((6.0, 2.5)),
        )
        .unwrap();

        assert_eq!(centerline, vec![(1.2, 2.5), (6.0, 2.5)]);
        let polygon = realize_route_polygon_with_endpoint_correction(
            &route,
            &lib,
            &grid(),
            0.5,
            Some((1.2, 2.5)),
            Some((6.0, 2.5)),
        )
        .unwrap();
        assert_eq!(
            polygon,
            generate_waveguide_polygon(&centerline, 0.5).unwrap()
        );
        assert_eq!(polygon.first(), polygon.last());
    }

    #[test]
    fn port_corrected_full_horizontal_straight_shifts_line_and_adjusts_length() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let primitive_ids = vec![straight_east, straight_east];
        let states = states_from_primitives(&lib, State::new(1, 2, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0,
            total_cost: 8.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((1.2, 2.75)),
            Some((9.9, 2.75)),
        )
        .unwrap();

        assert_eq!(corrected, vec![(1.2, 2.75), (5.5, 2.75), (9.9, 2.75)]);
        assert!((centerline_length_um(&corrected).unwrap() - 8.7).abs() <= 1.0e-9);
        assert!(corrected
            .windows(2)
            .all(|w| is_horizontal_segment(w[0], w[1])));
    }

    #[test]
    fn port_corrected_full_vertical_straight_shifts_line_and_adjusts_length() {
        let lib = test_lib();
        let straight_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_north, straight_north];
        let states = states_from_primitives(&lib, State::new(3, 1, 2), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0,
            total_cost: 8.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((3.25, 1.1)),
            Some((3.25, 9.8)),
        )
        .unwrap();

        assert_eq!(corrected, vec![(3.25, 1.1), (3.25, 5.5), (3.25, 9.8)]);
        assert!((centerline_length_um(&corrected).unwrap() - 8.7).abs() <= 1.0e-9);
        assert!(corrected
            .windows(2)
            .all(|w| is_vertical_segment(w[0], w[1])));
    }

    #[test]
    fn port_corrected_mixed_xy_route_anchors_endpoints_and_keeps_interior() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let bend_left = primitive_id_for(&lib, 0, |p| p.start_angle == 0 && p.end_angle == 2);
        let straight_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_east, bend_left, straight_north];
        let states = states_from_primitives(&lib, State::new(1, 1, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let base = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        assert!(base.len() > 2);
        let source_port = (1.1, 1.25);
        let target_port = (base.last().unwrap().0 + 0.35, base.last().unwrap().1 - 0.2);
        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        assert_eq!(corrected.len(), base.len());
        assert!(is_horizontal_segment(corrected[0], corrected[1]));
        assert!(is_vertical_segment(
            corrected[corrected.len() - 2],
            corrected[corrected.len() - 1],
        ));
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));

        let polygon = realize_route_polygon_with_endpoint_correction(
            &route,
            &lib,
            &grid(),
            0.5,
            Some(source_port),
            Some(target_port),
        )
        .unwrap();
        assert_eq!(
            polygon,
            generate_waveguide_polygon(&corrected, 0.5).unwrap()
        );
        assert_eq!(polygon.first(), polygon.last());
    }

    #[test]
    fn port_correction_absorbs_xy_offsets_only_in_straight_primitives() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let bend_left = primitive_id_for(&lib, 0, |p| p.start_angle == 0 && p.end_angle == 2);
        let straight_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_east, bend_left, straight_north];
        let states = states_from_primitives(&lib, State::new(1, 1, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let replay = route_to_primitive_centerline_with_runs(&route, &lib, &grid()).unwrap();
        let base = replay.centerline;
        let horizontal_run = first_run(&replay.straight_runs, AxisAlignedRunKind::Horizontal)
            .expect("expected horizontal straight primitive");
        let vertical_run = first_run(&replay.straight_runs, AxisAlignedRunKind::Vertical)
            .expect("expected vertical straight primitive");
        assert!(horizontal_run.end_index < vertical_run.start_index);

        let source_dx = 0.2;
        let source_dy = 0.25;
        let target_dx = -0.3;
        let target_dy = -0.15;
        let source_port = (base[0].0 + source_dx, base[0].1 + source_dy);
        let target_port = (
            base[base.len() - 1].0 + target_dx,
            base[base.len() - 1].1 + target_dy,
        );

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
        )
        .unwrap();

        assert_eq!(corrected.len(), base.len());
        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        assert!(is_horizontal_segment(
            corrected[horizontal_run.start_index],
            corrected[horizontal_run.end_index],
        ));
        assert!(is_vertical_segment(
            corrected[vertical_run.start_index],
            corrected[vertical_run.end_index],
        ));

        for i in horizontal_run.end_index..vertical_run.start_index {
            let base_delta = sub(base[i + 1], base[i]);
            let corrected_delta = sub(corrected[i + 1], corrected[i]);
            assert!((corrected_delta.0 - base_delta.0).abs() <= 1.0e-9);
            assert!((corrected_delta.1 - base_delta.1).abs() <= 1.0e-9);
        }

        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    fn static_grid() -> StaticGridSpec {
        make_grid_spec((0.0, 0.0, 10.0, 10.0), 1.0).unwrap()
    }

    fn tiny_grid() -> StaticGridSpec {
        make_grid_spec((0.0, 0.0, 1.0, 1.0), 1.0).unwrap()
    }

    #[test]
    fn port_at_grid_center_has_direct_access() {
        let port = PortInput::new("p0".to_string(), 0.5, 0.5, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert!(access.anchor_cell.0 >= 0 && access.anchor_cell.1 >= 0);
        assert_eq!(
            access.access_centerline_um.first().copied(),
            Some((0.5, 0.5))
        );
        assert_eq!(
            access.access_centerline_um.last().copied(),
            Some(access.anchor_point_um)
        );
        assert!(access.access_centerline_um.len() >= 2);
    }

    #[test]
    fn off_grid_port_ends_access_at_anchor_center() {
        let port = PortInput::new("p1".to_string(), 0.2, 0.7, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert!(access.anchor_cell.0 >= 0 && access.anchor_cell.1 >= 0);
        assert_eq!(
            access.access_centerline_um.first().copied(),
            Some((0.2, 0.7))
        );
        assert_eq!(
            access.access_centerline_um.last().copied(),
            Some(access.anchor_point_um)
        );
        assert!(access.access_centerline_um.len() > 2);
    }

    #[test]
    fn access_first_segment_follows_port_orientation() {
        let port = PortInput::new("p2".to_string(), 1.0, 1.0, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        let p0 = access.access_centerline_um[0];
        let p1 = access.access_centerline_um[1];
        assert!(p1.0 > p0.0);
        assert!((p1.1 - p0.1).abs() <= 1e-9);
    }

    #[test]
    fn access_last_segment_follows_anchor_angle() {
        let port = PortInput::new("p2b".to_string(), 1.0, 1.0, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        let n = access.access_centerline_um.len();
        let p0 = access.access_centerline_um[n - 2];
        let p1 = access.access_centerline_um[n - 1];
        let seg = sub(p1, p0);
        let dir = angle_to_unit_vector(access.anchor_angle);
        let perp = rotate_left(dir);
        assert!(dot(seg, dir) > EPS);
        assert!(dot(seg, perp).abs() <= 1e-6);
        assert_eq!(access.anchor_angle, access.entry_angle);
    }

    #[test]
    fn access_points_are_finite_and_non_degenerate() {
        let port = PortInput::new("p3".to_string(), 0.3, 0.8, Some(45.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert!(access
            .access_centerline_um
            .iter()
            .all(|p| p.0.is_finite() && p.1.is_finite()));
        assert!(access
            .access_centerline_um
            .windows(2)
            .all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn infeasible_access_returns_explicit_error() {
        let sg = tiny_grid();
        let port = PortInput::new("p4".to_string(), 0.1, 0.1, Some(0.0));
        let cfg = PortAccessConfig {
            min_straight_um: 5.0,
            max_anchor_search_cells: 1,
            min_bend_radius_um: 5.0,
        };
        let err = build_port_access(&port, &sg, &cfg).unwrap_err();
        assert!(matches!(err, PortAccessError::AnchorSearchFailed { .. }));
    }

    #[test]
    fn batch_build_port_accesses_returns_one_per_port() {
        let ports = vec![
            PortInput::new("b0".to_string(), 0.2, 0.7, Some(0.0)),
            PortInput::new("b1".to_string(), 0.3, 0.8, Some(45.0)),
        ];
        let accesses =
            build_port_accesses(&ports, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert_eq!(accesses.len(), ports.len());
    }

    #[test]
    fn straight_route_generates_closed_rectangle() {
        let centerline = vec![(0.5, 0.5), (4.5, 0.5)];
        let polygon = generate_waveguide_polygon(&centerline, 1.0).unwrap();

        assert_eq!(polygon.first(), polygon.last());
        assert_eq!(polygon.len(), 5);
        assert!(polygon.contains(&(0.5, 1.0)));
        assert!(polygon.contains(&(4.5, 0.0)));
    }

    #[test]
    fn ninety_degree_route_generates_closed_polygon() {
        let centerline = vec![(0.5, 0.5), (4.5, 0.5), (4.5, 4.5)];
        let polygon = generate_waveguide_polygon(&centerline, 1.0).unwrap();

        assert_eq!(polygon.first(), polygon.last());
        assert!(polygon.len() >= 5);
        assert!(polygon.iter().all(|&p| p.0.is_finite() && p.1.is_finite()));
    }

    #[test]
    fn realizes_route_polygon_without_port_access() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let polygon = realize_route_polygon(
            &route,
            &lib,
            &grid(),
            1.0,
            Some((1.0, 2.0)),
            Some((6.0, 2.0)),
        )
        .unwrap();

        assert_eq!(polygon.first(), polygon.last());
        let min_x = polygon.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_x = polygon
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(min_x <= 1.5 + 1e-9);
        assert!(max_x >= 5.5 - 1e-9);
    }

    #[test]
    fn realizes_route_polygon_with_explicit_port_access_endpoints() {
        let lib = test_lib();
        let sg = static_grid();
        let access_cfg = PortAccessConfig {
            min_straight_um: 0.0,
            max_anchor_search_cells: 8,
            min_bend_radius_um: 0.5,
        };
        let source = build_port_access(
            &PortInput::new("s".to_string(), 1.0, 2.5, Some(0.0)),
            &sg,
            &access_cfg,
        )
        .unwrap();
        let target = build_port_access(
            &PortInput::new("t".to_string(), 6.0, 2.5, Some(180.0)),
            &sg,
            &access_cfg,
        )
        .unwrap();

        let source_state = State::new(
            source.anchor_cell.0,
            source.anchor_cell.1,
            source.entry_angle,
        );
        let target_state = State::new(
            target.anchor_cell.0,
            target.anchor_cell.1,
            target.entry_angle,
        );
        let straight_pid = lib
            .get_primitives_for_angle(source_state.angle)
            .iter()
            .find(|p| {
                p.end_angle == source_state.angle
                    && p.dx == (target_state.x - source_state.x)
                    && p.dy == (target_state.y - source_state.y)
            })
            .map(|p| p.id)
            .unwrap_or(lib.get_primitives_for_angle(source_state.angle)[0].id);

        let route = RouteResult {
            states: vec![source_state, target_state],
            primitives: vec![straight_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: target_state,
            reached_target: target_state,
            stats: Default::default(),
        };

        let polygon = realize_route_polygon_with_port_access(
            &route,
            &lib,
            &grid(),
            1.0,
            Some(&source),
            Some(&target),
        )
        .unwrap();
        assert_eq!(polygon.first(), polygon.last());
        let min_x = polygon.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_x = polygon
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(min_x <= 1.0 + 1e-9);
        assert!(max_x >= 6.0 - 1e-9);
    }

    #[test]
    fn realizes_route_polygon_from_primitives_closed_polygon() {
        let lib = test_lib();
        let bend_pid = lib.get_primitives_for_angle(0)[4].id;
        let straight_north_pid = lib.get_primitives_for_angle(2)[0].id;
        let route = RouteResult {
            states: vec![
                State::new(1, 1, 0),
                State::new(2, 2, 2),
                State::new(2, 3, 2),
            ],
            primitives: vec![bend_pid, straight_north_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let polygon = realize_route_polygon_from_primitives(&route, &lib, &grid(), 1.0).unwrap();
        assert_eq!(polygon.first(), polygon.last());
    }

    #[test]
    fn splice_meander_preserves_endpoints_and_increases_length() {
        let centerline = vec![(0.0, 0.0), (10.0, 0.0), (15.0, 0.0)];
        let meander = vec![
            PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            PhysicalPoint {
                x_um: 3.0,
                y_um: 2.0,
            },
            PhysicalPoint {
                x_um: 7.0,
                y_um: 2.0,
            },
            PhysicalPoint {
                x_um: 10.0,
                y_um: 0.0,
            },
        ];
        let spliced = splice_meander_into_centerline(&centerline, 0, &meander);
        assert_eq!(spliced.first().copied(), Some((0.0, 0.0)));
        assert_eq!(spliced.last().copied(), Some((15.0, 0.0)));
        assert!(spliced.windows(2).all(|w| distance(w[0], w[1]) > EPS));
        let old_len = centerline
            .windows(2)
            .map(|w| distance(w[0], w[1]))
            .sum::<f64>();
        let new_len = spliced
            .windows(2)
            .map(|w| distance(w[0], w[1]))
            .sum::<f64>();
        assert!(new_len > old_len);
        let b = MeanderBox {
            min_x_um: -1.0,
            max_x_um: 11.0,
            min_y_um: -1.0,
            max_y_um: 3.0,
        };
        for p in meander {
            assert!(p.x_um >= b.min_x_um - EPS && p.x_um <= b.max_x_um + EPS);
            assert!(p.y_um >= b.min_y_um - EPS && p.y_um <= b.max_y_um + EPS);
        }
    }

    #[test]
    fn select_meander_segment_prefers_longest_fully_contained_axis_aligned() {
        let centerline = vec![(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (7.0, 1.0), (9.0, 3.0)];
        let idx = select_meander_segment(
            &centerline,
            MeanderBox {
                min_x_um: 1.0,
                max_x_um: 8.0,
                min_y_um: -1.0,
                max_y_um: 2.0,
            },
        )
        .unwrap();
        assert_eq!(idx, 2);
    }

    #[test]
    fn extract_straight_runs_merges_consecutive_collinear_segments() {
        let centerline = vec![(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0)];
        let runs = extract_axis_aligned_straight_runs(&centerline, 25.0);
        assert_eq!(runs.len(), 1);
        let r = runs[0];
        assert_eq!(r.start_index, 0);
        assert_eq!(r.end_index, 3);
        assert_eq!(r.start, (0.0, 0.0));
        assert_eq!(r.end, (30.0, 0.0));
        assert!((r.length_um - 30.0).abs() < EPS);
    }

    #[test]
    fn extract_straight_runs_splits_on_axis_change() {
        let centerline = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 15.0)];
        let runs = extract_axis_aligned_straight_runs(&centerline, 5.0);
        assert_eq!(runs.len(), 2);
        assert_eq!(runs[0].start_index, 0);
        assert_eq!(runs[0].end_index, 1);
        assert!((runs[0].length_um - 10.0).abs() < EPS);
        assert_eq!(runs[1].start_index, 1);
        assert_eq!(runs[1].end_index, 3);
        assert!((runs[1].length_um - 15.0).abs() < EPS);
    }

    #[test]
    fn plan_analytic_meander_for_route_returns_plan_and_segment() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let plan = plan_analytic_meander_for_route(
            &route,
            &lib,
            &grid(),
            1.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap();
        assert_eq!(plan.selected_segment_index, 0);
        assert!(plan.plan.bumps >= 1);
        assert!(plan.plan.inserted_extra_length_um > 0.0);
        assert!(!plan.plan.centerline.is_empty());
    }

    #[test]
    fn realizes_route_polygon_with_analytic_meander_success() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let poly = realize_route_polygon_with_analytic_meander(
            &route,
            &lib,
            &grid(),
            1.0,
            1.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap();
        assert!(poly.len() >= 4);
        assert!(poly.iter().all(|p| p.0.is_finite() && p.1.is_finite()));
    }

    #[test]
    fn analytic_meander_realization_returns_no_candidate_error() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(2, 2, 2)],
            primitives: vec![4],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(2, 2, 2),
            reached_target: State::new(2, 2, 2),
            stats: Default::default(),
        };
        let err = realize_route_polygon_with_analytic_meander(
            &route,
            &lib,
            &grid(),
            1.0,
            2.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 100.0,
                max_x_um: 110.0,
                min_y_um: 100.0,
                max_y_um: 110.0,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap_err();
        assert_eq!(err, GeometryError::NoMeanderCandidateSegment);
    }

    #[test]
    fn analytic_meander_realization_propagates_planning_failure() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let err = realize_route_polygon_with_analytic_meander(
            &route,
            &lib,
            &grid(),
            1.0,
            50.0,
            0.5,
            0.5,
            1,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 2.9,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap_err();
        assert!(matches!(
            err,
            GeometryError::MeanderPlanningFailed(
                MeanderPlanningError::AvailableBoxTooSmall
                    | MeanderPlanningError::RequestedExtraLengthDoesNotFit
            )
        ));
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
    fn meander_box_converts_to_expected_grid_rect() {
        let g = GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap();
        let rect = meander_box_to_grid_rect(
            MeanderBox {
                min_x_um: 1.0,
                max_x_um: 3.0,
                min_y_um: 2.0,
                max_y_um: 4.0,
            },
            &g,
            0,
        )
        .unwrap();
        assert_eq!(
            rect,
            GridRect {
                min_x: 1,
                max_x: 2,
                min_y: 2,
                max_y: 3
            }
        );
    }

    #[test]
    fn meander_box_out_of_bounds_is_error() {
        let map = ObstacleMap::new(5, 5);
        let g = GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap();
        let err = check_meander_box_free(
            MeanderBox {
                min_x_um: -1.0,
                max_x_um: 1.0,
                min_y_um: 0.0,
                max_y_um: 1.0,
            },
            &g,
            &map,
            None,
            0,
        )
        .unwrap_err();
        assert!(matches!(err, GeometryError::MeanderBoxOutOfBounds(_)));
    }

    #[test]
    fn checked_analytic_meander_box_realization_succeeds_when_free() {
        let lib = test_lib();
        let map = ObstacleMap::new(20, 20);
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let empty = FxHashSet::default();
        let poly = realize_route_polygon_with_checked_analytic_meander_box(
            &route,
            &lib,
            &grid(),
            1.0,
            1.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            &map,
            Some(&empty),
            0,
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap();
        assert!(poly.len() >= 4);
    }

    #[test]
    fn checked_analytic_meander_box_realization_fails_when_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(20, 20);
        assert!(map.add_static_cell(3, 3));
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let err = realize_route_polygon_with_checked_analytic_meander_box(
            &route,
            &lib,
            &grid(),
            1.0,
            3.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            &map,
            None,
            0,
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap_err();
        assert!(matches!(err, GeometryError::MeanderBoxBlocked { .. }));
    }

    #[test]
    fn cells_in_grid_rect_count_matches_area() {
        let rect = GridRect {
            min_x: 2,
            max_x: 4,
            min_y: 1,
            max_y: 3,
        };
        assert_eq!(cells_in_grid_rect(rect).len(), 9);
    }

    #[test]
    fn auto_meander_planner_selects_valid_box_on_empty_map() {
        let lib = test_lib();
        let map = ObstacleMap::new(20, 20);
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
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
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert!(!p.plan.centerline.is_empty());
    }

    #[test]
    fn auto_meander_chooses_right_when_left_box_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(20, 20);
        for x in 1..=5 {
            assert!(map.add_static_cell(x, 3));
        }
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
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
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.plan.side, MeanderSide::Right);
    }

    #[test]
    fn auto_meander_uses_middle_interval_when_endpoint_boxes_are_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(30, 20);
        for x in [1, 2, 3, 15, 16, 17] {
            assert!(map.add_static_cell(x, 3));
        }
        for x in 1..=17 {
            assert!(map.add_static_cell(x, 1));
        }
        let route = RouteResult {
            states: vec![
                State::new(1, 2, 0),
                State::new(5, 2, 0),
                State::new(9, 2, 0),
                State::new(13, 2, 0),
                State::new(17, 2, 0),
            ],
            primitives: vec![1, 1, 1, 1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 16.0,
            total_cost: 16.0,
            requested_target: State::new(17, 2, 0),
            reached_target: State::new(17, 2, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 8,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };

        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.plan.side, MeanderSide::Left);
        assert!(p.selected_segment.start.x_um > 3.0);
        assert!(p.selected_segment.end.x_um <= 15.0);
        assert!(p.selected_interval_length_um < p.selected_run_length_um);
        assert!(p.candidate_intervals >= 1);
        assert_eq!(
            p.replacement_centerline.first().copied(),
            Some(PhysicalPoint {
                x_um: 1.5,
                y_um: 2.5
            })
        );
        assert_eq!(
            p.replacement_centerline.last().copied(),
            Some(PhysicalPoint {
                x_um: 17.5,
                y_um: 2.5
            })
        );
        assert_eq!(
            p.plan.centerline.first().copied(),
            Some(p.selected_segment.start)
        );
    }

    #[test]
    fn auto_meander_tries_later_runs_when_first_run_is_blocked_on_both_sides() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(40, 40);

        // Build a route with multiple straight runs:
        // long east run -> bend -> north run -> bend -> east run.
        let east_long = primitive_id_for(&lib, 0, |p| {
            matches!(p.geometry, PrimitiveGeometry::Straight { .. }) && p.dx == 4 && p.dy == 0
        });
        let bend_left_from_east = primitive_id_for(&lib, 0, |p| {
            matches!(p.geometry, PrimitiveGeometry::Bend { angle_delta: 2, .. })
        });
        let north_long = primitive_id_for(&lib, 2, |p| {
            matches!(p.geometry, PrimitiveGeometry::Straight { .. }) && p.dx == 0 && p.dy == 4
        });
        let bend_right_from_north = primitive_id_for(&lib, 2, |p| {
            matches!(
                p.geometry,
                PrimitiveGeometry::Bend {
                    angle_delta: -2,
                    ..
                }
            )
        });

        let primitive_ids = vec![
            east_long,
            east_long,
            bend_left_from_east,
            north_long,
            bend_right_from_north,
            east_long,
        ];
        let states = states_from_primitives(&lib, State::new(2, 10, 0), &primitive_ids);
        let route = RouteResult {
            states: states.clone(),
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: *states.last().unwrap(),
            reached_target: *states.last().unwrap(),
            stats: Default::default(),
        };

        // Block both sides of the first long eastbound run (y=10).
        for x in 2..=10 {
            assert!(map.add_static_cell(x, 11));
            assert!(map.add_static_cell(x, 9));
        }

        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 8,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };

        let plan =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert!(plan.selected_run_start_index > 0);
    }

    #[test]
    fn auto_meander_returns_error_when_both_sides_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(20, 20);
        for x in 1..=5 {
            assert!(map.add_static_cell(x, 3));
            assert!(map.add_static_cell(x, 1));
        }
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
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
        };
        let err = plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg)
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
    fn auto_meander_is_deterministic_for_tie() {
        let lib = test_lib();
        let map = ObstacleMap::new(30, 30);
        let route = RouteResult {
            states: vec![
                State::new(2, 10, 0),
                State::new(6, 10, 0),
                State::new(10, 10, 0),
            ],
            primitives: vec![1, 1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0,
            total_cost: 8.0,
            requested_target: State::new(10, 10, 0),
            reached_target: State::new(10, 10, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
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
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.selected_segment_index, 0);
        assert_eq!(p.selected_run_start_index, 0);
        assert_eq!(p.selected_run_end_index, 2);
        assert_eq!(p.plan.side, MeanderSide::Left);
    }

    #[test]
    fn splice_meander_range_replaces_full_run_and_preserves_endpoints() {
        let centerline = vec![
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
            (15.0, 0.0),
            (20.0, 0.0),
        ];
        let meander = vec![
            PhysicalPoint {
                x_um: 5.0,
                y_um: 0.0,
            },
            PhysicalPoint {
                x_um: 7.0,
                y_um: 2.0,
            },
            PhysicalPoint {
                x_um: 10.0,
                y_um: 0.0,
            },
            PhysicalPoint {
                x_um: 13.0,
                y_um: -2.0,
            },
            PhysicalPoint {
                x_um: 15.0,
                y_um: 0.0,
            },
        ];
        let spliced = splice_meander_into_centerline_range(&centerline, 1, 3, &meander)
            .expect("valid splice");
        assert_eq!(spliced.first().copied(), Some((0.0, 0.0)));
        assert_eq!(spliced.last().copied(), Some((20.0, 0.0)));
        assert!(!spliced.windows(2).any(|w| distance(w[0], w[1]) <= EPS));
        assert!(spliced.contains(&(7.0, 2.0)));
        assert!(spliced.contains(&(13.0, -2.0)));
    }

    #[test]
    fn auto_meander_fill_mode_prefers_more_bumps() {
        let lib = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 12,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
        });
        let map = ObstacleMap::new(40, 40);
        let route = RouteResult {
            states: vec![State::new(2, 10, 0), State::new(14, 10, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: State::new(14, 10, 0),
            reached_target: State::new(14, 10, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 6,
            max_meander_height_um: 20.0,
            box_depth_um: 8.0,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert!(p.plan.bumps >= 1);
    }

    #[test]
    fn auto_meander_finds_candidate_on_merged_multi_segment_straight_run() {
        let lib = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
        });
        let map = ObstacleMap::new(40, 40);
        let route = RouteResult {
            states: vec![
                State::new(2, 10, 0),
                State::new(6, 10, 0),
                State::new(10, 10, 0),
                State::new(14, 10, 0),
            ],
            primitives: vec![1, 1, 1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: State::new(14, 10, 0),
            reached_target: State::new(14, 10, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.0,
            max_bumps: 6,
            max_meander_height_um: 20.0,
            box_depth_um: 8.0,
            min_segment_length_um: 10.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.selected_run_start_index, 0);
        assert_eq!(p.selected_run_end_index, 3);
    }

    #[test]
    fn fill_box_plan_stays_inside_box_and_uses_primitive_radius() {
        let grid = GeometryGridSpec::new(0.5, 0.0, 0.0).unwrap();
        let primitive_bend_radius_cells = 2;
        let primitive_bend_radius_um = primitive_bend_radius_cells as f64 * grid.grid_size_um;
        let lib = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: grid.grid_size_um,
            straight_short_cells: 1,
            straight_long_cells: 60,
            bend_radius_cells: primitive_bend_radius_cells,
            allow_45_degree_turns: true,
        });
        let map = ObstacleMap::new(120, 120);
        let route = RouteResult {
            states: vec![State::new(2, 20, 0), State::new(62, 20, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 30.0,
            total_cost: 30.0,
            requested_target: State::new(62, 20, 0),
            reached_target: State::new(62, 20, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 5.0,
            min_bend_radius_um: primitive_bend_radius_um,
            min_straight_um: 0.0,
            max_bumps: 20,
            max_meander_height_um: 20.0,
            box_depth_um: 8.0,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let plan =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid, &map, None, &cfg).unwrap();
        assert!(plan.plan.bumps >= 1);
        assert!((cfg.min_bend_radius_um - primitive_bend_radius_um).abs() < EPS);
        for p in &plan.plan.centerline {
            assert!(p.x_um >= plan.selected_box.min_x_um - EPS);
            assert!(p.x_um <= plan.selected_box.max_x_um + EPS);
            assert!(p.y_um >= plan.selected_box.min_y_um - EPS);
            assert!(p.y_um <= plan.selected_box.max_y_um + EPS);
        }
    }

    #[test]
    fn rejects_out_of_bounds_anchor_search() {
        let sg = static_grid();
        let config = PortAccessConfig {
            max_anchor_search_cells: 0,
            ..Default::default()
        };
        let port = PortInput::new("oob".to_string(), -10.0, -10.0, Some(180.0));
        let err = build_port_access(&port, &sg, &config).unwrap_err();
        assert!(matches!(err, PortAccessError::AnchorSearchFailed { .. }));
    }

    #[test]
    fn rejects_zero_length_access_connector() {
        let sg = tiny_grid();
        let port = PortInput::new("zero".to_string(), 0.5, 0.5, Some(0.0));
        let err = build_port_access(&port, &sg, &PortAccessConfig::default()).unwrap_err();
        assert!(matches!(
            err,
            PortAccessError::ZeroLengthAccess { .. } | PortAccessError::AnchorSearchFailed { .. }
        ));
    }

    #[test]
    fn rejects_mismatched_route_anchor_endpoints() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let bad_source = PortAccess {
            port_name: "bad".to_string(),
            port_point_um: (0.0, 0.0),
            anchor_cell: (0, 0),
            anchor_point_um: (0.5, 0.5),
            port_angle: 0,
            anchor_angle: 0,
            entry_angle: 0,
            access_centerline_um: vec![(0.0, 0.0), (0.5, 0.5)],
        };
        let err = realize_route_polygon_with_port_access(
            &route,
            &lib,
            &grid(),
            1.0,
            Some(&bad_source),
            None,
        )
        .unwrap_err();
        assert!(matches!(
            err,
            GeometryError::RouteStartDoesNotMatchSourceAnchor { .. }
        ));
    }

    #[test]
    fn rejects_degenerate_centerline() {
        let err = generate_waveguide_polygon(&[(0.0, 0.0)], 1.0).unwrap_err();

        assert_eq!(err, GeometryError::DegenerateRoute);
    }

    #[test]
    fn rejects_zero_width() {
        let err = generate_waveguide_polygon(&[(0.0, 0.0), (1.0, 0.0)], 0.0).unwrap_err();

        assert_eq!(err, GeometryError::InvalidWidth(0.0));
    }
}
