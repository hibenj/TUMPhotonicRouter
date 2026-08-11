//! Lightweight simple-route candidate representation and validation helpers.
//!
//! This module intentionally does not run search or integrate into the
//! production routing flow yet. It only models and validates pre-defined
//! octant-aligned polyline candidates.

use rustc_hash::FxHashSet;

use crate::astar::State;
use crate::obstacle_map::{CellKey, GridRect, ObstacleMap};

/// Discrete grid point in cell coordinates.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct GridPoint {
    pub x: i32,
    pub y: i32,
}

impl GridPoint {
    #[inline]
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// Octant-aligned candidate segment between two grid points.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct Segment {
    pub start: GridPoint,
    pub end: GridPoint,
}

impl Segment {
    #[inline]
    pub const fn new(start: GridPoint, end: GridPoint) -> Self {
        Self { start, end }
    }

    #[inline]
    pub fn is_horizontal(&self) -> bool {
        self.start.y == self.end.y
    }

    #[inline]
    pub fn is_vertical(&self) -> bool {
        self.start.x == self.end.x
    }

    #[inline]
    pub fn is_axis_aligned(&self) -> bool {
        self.is_horizontal() || self.is_vertical()
    }

    #[inline]
    pub fn is_diagonal_45(&self) -> bool {
        let dx = (self.end.x - self.start.x).abs();
        let dy = (self.end.y - self.start.y).abs();
        dx > 0 && dx == dy
    }

    #[inline]
    pub fn is_octant_aligned(&self) -> bool {
        self.is_axis_aligned() || self.is_diagonal_45()
    }

    /// Return the same segment with endpoints in a stable order.
    ///
    /// For horizontal segments: increasing x.
    /// For vertical segments: increasing y.
    /// For other segments: lexicographic `(x, y)`.
    pub fn normalized(&self) -> Self {
        if self.is_horizontal() {
            if self.start.x <= self.end.x {
                *self
            } else {
                Self::new(self.end, self.start)
            }
        } else if self.is_vertical() {
            if self.start.y <= self.end.y {
                *self
            } else {
                Self::new(self.end, self.start)
            }
        } else if (self.start.x, self.start.y) <= (self.end.x, self.end.y) {
            *self
        } else {
            Self::new(self.end, self.start)
        }
    }

    #[inline]
    pub fn manhattan_len(&self) -> i32 {
        (self.end.x - self.start.x).abs() + (self.end.y - self.start.y).abs()
    }

    #[inline]
    pub fn step_len(&self) -> i32 {
        (self.end.x - self.start.x)
            .abs()
            .max((self.end.y - self.start.y).abs())
    }
}

/// Intended simple route topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum SimpleRouteKind {
    Straight,
    LShape,
    ZShape,
    Turnaround,
}

/// Configuration for deterministic Z-shape exploration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SimpleZRouteConfig {
    pub max_offset_cells: i32,
    pub include_zero_offset: bool,
    pub min_leg_len_cells: i32,
}

impl Default for SimpleZRouteConfig {
    fn default() -> Self {
        Self {
            max_offset_cells: 16,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        }
    }
}

pub trait SimpleRouteObstacleQuery {
    fn rect_free(&self, rect: GridRect, opened_cells: Option<&FxHashSet<CellKey>>) -> bool;
    fn check_cells_free(
        &self,
        cells: &[(i32, i32)],
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool;

    fn check_segment_free(
        &self,
        segment: Segment,
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        let Some(seg_points) = expand_segment_points(segment) else {
            return false;
        };
        let seg_cells: Vec<(i32, i32)> = seg_points.into_iter().map(|p| (p.x, p.y)).collect();
        self.check_cells_free(&seg_cells, opened_cells)
    }
}

impl SimpleRouteObstacleQuery for ObstacleMap {
    fn rect_free(&self, rect: GridRect, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        if rect.x_min > rect.x_max || rect.y_min > rect.y_max {
            return true;
        }
        for y in rect.y_min..=rect.y_max {
            for x in rect.x_min..=rect.x_max {
                if !simple_cell_free_with_congestion(self, x, y, opened_cells) {
                    return false;
                }
            }
        }
        true
    }

    fn check_cells_free(
        &self,
        cells: &[(i32, i32)],
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        cells
            .iter()
            .all(|&(x, y)| simple_cell_free_with_congestion(self, x, y, opened_cells))
    }
}

#[inline]
fn simple_cell_free_with_congestion(
    obstacle_map: &ObstacleMap,
    x: i32,
    y: i32,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    if !obstacle_map.in_bounds(x, y) {
        return false;
    }

    let key = crate::obstacle_map::pack_xy(x, y);
    if obstacle_map.is_dynamic_core_blocked(x, y) {
        return false;
    }
    if opened_cells
        .map(|cells| cells.contains(&key))
        .unwrap_or(false)
    {
        return true;
    }
    if obstacle_map.is_static_blocked(x, y) {
        return false;
    }
    obstacle_map.get_congestion_cost(x, y) == 0
}

pub struct ExpandedDynamicObstacleQuery<'a> {
    obstacle_map: &'a ObstacleMap,
    dynamic_expansion_radius_cells: i32,
    clearance_exempt_cells: Option<&'a FxHashSet<CellKey>>,
}

impl<'a> ExpandedDynamicObstacleQuery<'a> {
    pub fn new(
        obstacle_map: &'a ObstacleMap,
        dynamic_expansion_radius_cells: i32,
        clearance_exempt_cells: Option<&'a FxHashSet<CellKey>>,
    ) -> Self {
        Self {
            obstacle_map,
            dynamic_expansion_radius_cells: dynamic_expansion_radius_cells.max(0),
            clearance_exempt_cells,
        }
    }

    #[inline]
    fn is_clearance_exempt(&self, x: i32, y: i32) -> bool {
        self.clearance_exempt_cells
            .map(|cells| cells.contains(&crate::obstacle_map::pack_xy(x, y)))
            .unwrap_or(false)
            && !self.obstacle_map.is_dynamic_core_blocked(x, y)
    }

    fn expanded_dynamic_blocked_at(&self, x: i32, y: i32) -> bool {
        if !self.obstacle_map.in_bounds(x, y) {
            return true;
        }
        if self.is_clearance_exempt(x, y) {
            return false;
        }

        let radius = self.dynamic_expansion_radius_cells;
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                let Some(nx) = x.checked_add(dx) else {
                    continue;
                };
                let Some(ny) = y.checked_add(dy) else {
                    continue;
                };
                if self.obstacle_map.in_bounds(nx, ny)
                    && self.obstacle_map.is_dynamic_blocked(nx, ny)
                {
                    return true;
                }
            }
        }
        false
    }

    #[inline]
    fn cell_free(&self, x: i32, y: i32, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        if !self.obstacle_map.in_bounds(x, y) || self.expanded_dynamic_blocked_at(x, y) {
            return false;
        }
        if opened_cells
            .map(|cells| cells.contains(&crate::obstacle_map::pack_xy(x, y)))
            .unwrap_or(false)
        {
            return true;
        }
        !self.obstacle_map.is_static_blocked(x, y)
            && self.obstacle_map.get_congestion_cost(x, y) == 0
    }

    #[inline]
    fn core_cell_free(&self, x: i32, y: i32, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        if !self.obstacle_map.in_bounds(x, y) {
            return false;
        }
        if self.obstacle_map.is_dynamic_core_blocked(x, y) {
            return false;
        }
        if opened_cells
            .map(|cells| cells.contains(&crate::obstacle_map::pack_xy(x, y)))
            .unwrap_or(false)
        {
            return true;
        }
        if self.obstacle_map.is_static_blocked(x, y) {
            return false;
        }
        self.obstacle_map.get_congestion_cost(x, y) == 0
    }

    fn compact_diagonal_halo_cells_for_segment(segment: Segment) -> Option<Vec<(i32, i32)>> {
        if !segment.is_diagonal_45() {
            return Some(Vec::new());
        }

        let points = expand_segment_points(segment)?;
        let mut cells = Vec::new();
        for pair in points.windows(2) {
            let start = pair[0];
            let end = pair[1];
            let dx = (end.x - start.x).signum();
            let dy = (end.y - start.y).signum();
            push_unique_grid_cell(&mut cells, (start.x + dx, start.y));
            push_unique_grid_cell(&mut cells, (end.x + dx, end.y));
            push_unique_grid_cell(&mut cells, (start.x, start.y + dy));
            push_unique_grid_cell(&mut cells, (end.x, end.y + dy));
        }
        Some(cells)
    }
}

impl SimpleRouteObstacleQuery for ExpandedDynamicObstacleQuery<'_> {
    fn rect_free(&self, rect: GridRect, opened_cells: Option<&FxHashSet<CellKey>>) -> bool {
        if rect.x_min > rect.x_max || rect.y_min > rect.y_max {
            return true;
        }
        for y in rect.y_min..=rect.y_max {
            for x in rect.x_min..=rect.x_max {
                if !self.cell_free(x, y, opened_cells) {
                    return false;
                }
            }
        }
        true
    }

    fn check_cells_free(
        &self,
        cells: &[(i32, i32)],
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        cells
            .iter()
            .all(|&(x, y)| self.cell_free(x, y, opened_cells))
    }

    fn check_segment_free(
        &self,
        segment: Segment,
        opened_cells: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        let Some(seg_points) = expand_segment_points(segment) else {
            return false;
        };

        for point in seg_points {
            if !self.core_cell_free(point.x, point.y, opened_cells) {
                return false;
            }
        }

        let Some(halo_cells) = Self::compact_diagonal_halo_cells_for_segment(segment) else {
            return false;
        };
        for (x, y) in halo_cells {
            if !self.core_cell_free(x, y, opened_cells) {
                return false;
            }
        }

        true
    }
}

/// Polyline candidate for deterministic pre-routing checks.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SimpleRouteCandidate {
    pub kind: SimpleRouteKind,
    // TODO: Switch to SmallVec<[GridPoint; 4]> if/when `smallvec` is added.
    pub points: Vec<GridPoint>,
}

impl SimpleRouteCandidate {
    #[inline]
    pub fn new(kind: SimpleRouteKind, points: Vec<GridPoint>) -> Self {
        Self { kind, points }
    }

    /// Consecutive segment list between route polyline corners.
    pub fn segments(&self) -> Vec<Segment> {
        self.points
            .windows(2)
            .map(|pair| Segment::new(pair[0], pair[1]))
            .collect()
    }

    /// Number of corner bends implied by the corner list.
    #[inline]
    pub fn num_bends(&self) -> usize {
        self.points.len().saturating_sub(2)
    }

    /// Total Manhattan length across all segments.
    pub fn total_manhattan_len(&self) -> i32 {
        self.segments().iter().map(Segment::manhattan_len).sum()
    }

    /// True when every segment is horizontal or vertical.
    pub fn is_axis_aligned(&self) -> bool {
        self.segments().iter().all(Segment::is_axis_aligned)
    }

    /// True when every segment follows a cardinal or 45-degree diagonal octant.
    pub fn is_octant_aligned(&self) -> bool {
        self.segments().iter().all(Segment::is_octant_aligned)
    }

    /// True when two adjacent corner points are identical.
    pub fn has_duplicate_consecutive_points(&self) -> bool {
        self.points.windows(2).any(|pair| pair[0] == pair[1])
    }
}

/// Return a cardinal heading in the existing octant-angle format.
///
/// Returns:
/// - `Some(0)` east
/// - `Some(2)` north
/// - `Some(4)` west
/// - `Some(6)` south
/// - `None` for equal points or non-axis-aligned points.
pub fn direction_between(a: GridPoint, b: GridPoint) -> Option<u8> {
    if a.x == b.x {
        return match b.y.cmp(&a.y) {
            std::cmp::Ordering::Greater => Some(2),
            std::cmp::Ordering::Less => Some(6),
            std::cmp::Ordering::Equal => None,
        };
    }
    if a.y == b.y {
        return match b.x.cmp(&a.x) {
            std::cmp::Ordering::Greater => Some(0),
            std::cmp::Ordering::Less => Some(4),
            std::cmp::Ordering::Equal => None,
        };
    }
    None
}

/// Return a cardinal or 45-degree diagonal heading in octant-angle format.
pub fn direction_between_octant(a: GridPoint, b: GridPoint) -> Option<u8> {
    if let Some(cardinal) = direction_between(a, b) {
        return Some(cardinal);
    }
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    if dx == 0 || dy == 0 || dx.abs() != dy.abs() {
        return None;
    }
    match (dx.signum(), dy.signum()) {
        (1, 1) => Some(1),
        (-1, 1) => Some(3),
        (-1, -1) => Some(5),
        (1, -1) => Some(7),
        _ => None,
    }
}

#[inline]
fn heading_vector(angle: u8) -> (i32, i32) {
    match angle % 8 {
        0 => (1, 0),
        1 => (1, 1),
        2 => (0, 1),
        3 => (-1, 1),
        4 => (-1, 0),
        5 => (-1, -1),
        6 => (0, -1),
        7 => (1, -1),
        _ => unreachable!(),
    }
}

#[inline]
fn offset_point(point: GridPoint, heading: u8, steps: i32) -> GridPoint {
    let (dx, dy) = heading_vector(heading);
    GridPoint::new(point.x + dx * steps, point.y + dy * steps)
}

/// Return true when heading is one of the Manhattan/cardinal octants.
#[inline]
pub fn is_cardinal_heading(angle: u8) -> bool {
    matches!(angle % 8, 0 | 2 | 4 | 6)
}

/// Return the opposite octant heading.
#[inline]
pub fn opposite_heading(angle: u8) -> u8 {
    (angle + 4) % 8
}

/// Return true when two octant headings are perpendicular (90-degree apart).
#[inline]
pub fn heading_delta_is_perpendicular(a: u8, b: u8) -> bool {
    let delta = (b as i16 - a as i16).rem_euclid(8) as u8;
    delta == 2 || delta == 6
}

/// Return true when two headings can be connected by one available 45/90 bend.
#[inline]
pub fn heading_delta_is_simple_bend(a: u8, b: u8) -> bool {
    let delta = (b as i16 - a as i16).rem_euclid(8) as u8;
    matches!(delta, 1 | 2 | 6 | 7)
}

/// Drop heading and keep only grid position.
#[inline]
pub fn grid_point_from_state(state: State) -> GridPoint {
    GridPoint::new(state.x, state.y)
}

/// Build a state from a grid point plus heading.
#[inline]
pub fn state_from_grid_point(point: GridPoint, angle: u8) -> State {
    State::new(point.x, point.y, angle)
}

/// Try constructing a deterministic straight-route candidate.
pub fn try_straight_candidate(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    try_straight_candidate_with_query(source, target, obstacle_map, opened_cells)
}

fn try_straight_candidate_with_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    if !is_cardinal_heading(source.angle) || !is_cardinal_heading(target.angle) {
        return None;
    }

    let source_point = grid_point_from_state(source);
    let target_point = grid_point_from_state(target);
    if source_point == target_point {
        return None;
    }

    let route_heading = direction_between(source_point, target_point)?;
    if route_heading != (source.angle % 8) || route_heading != (target.angle % 8) {
        return None;
    }

    let candidate =
        SimpleRouteCandidate::new(SimpleRouteKind::Straight, vec![source_point, target_point]);
    if check_simple_candidate_with_query(&candidate, obstacle_query, opened_cells) {
        Some(candidate)
    } else {
        None
    }
}

/// Try constructing a deterministic one-bend L-route candidate.
pub fn try_l_candidate(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    try_l_candidate_with_min_leg_len(source, target, obstacle_map, opened_cells, 1)
}

/// Try constructing a deterministic one-bend L-route candidate.
pub fn try_l_candidate_with_min_leg_len(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    min_leg_len_cells: i32,
) -> Option<SimpleRouteCandidate> {
    try_l_candidate_with_min_leg_len_query(
        source,
        target,
        obstacle_map,
        opened_cells,
        min_leg_len_cells,
    )
}

fn try_l_candidate_with_min_leg_len_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
    min_leg_len_cells: i32,
) -> Option<SimpleRouteCandidate> {
    if !is_cardinal_heading(source.angle) || !is_cardinal_heading(target.angle) {
        return None;
    }

    let source_point = grid_point_from_state(source);
    let target_point = grid_point_from_state(target);
    if source_point == target_point {
        return None;
    }

    let bend_candidates = [
        GridPoint::new(source_point.x, target_point.y),
        GridPoint::new(target_point.x, source_point.y),
    ];

    let mut best: Option<SimpleRouteCandidate> = None;
    for bend in bend_candidates {
        if bend == source_point || bend == target_point {
            continue;
        }

        let Some(first_heading) = direction_between(source_point, bend) else {
            continue;
        };
        let Some(second_heading) = direction_between(bend, target_point) else {
            continue;
        };

        if first_heading != (source.angle % 8) || second_heading != (target.angle % 8) {
            continue;
        }
        if !heading_delta_is_perpendicular(first_heading, second_heading) {
            continue;
        }

        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::LShape,
            vec![source_point, bend, target_point],
        );
        if !is_valid_l_candidate(
            &candidate,
            source.angle % 8,
            target.angle % 8,
            min_leg_len_cells.max(1),
            obstacle_query,
            opened_cells,
        ) {
            continue;
        }

        match &best {
            None => best = Some(candidate),
            Some(existing) => {
                if candidate.total_manhattan_len() < existing.total_manhattan_len() {
                    best = Some(candidate);
                }
            }
        }
    }

    best
}

/// Try deterministic straight first, then L-shape fallback.
pub fn try_straight_or_l_candidate(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    try_straight_candidate(source, target, obstacle_map, opened_cells)
        .or_else(|| try_l_candidate(source, target, obstacle_map, opened_cells))
}

/// Generate compact symmetric signed offsets.
///
/// Example:
/// `compact_offset_order(3, true)` -> `[0, 1, -1, 2, -2, 3, -3]`
pub fn compact_offset_order(max_offset_cells: i32, include_zero: bool) -> Vec<i32> {
    let max_offset_cells = max_offset_cells.max(0);
    let mut out = Vec::new();
    if include_zero {
        out.push(0);
    }
    for d in 1..=max_offset_cells {
        out.push(d);
        out.push(-d);
    }
    out
}

/// Try constructing a deterministic two-bend Z-route candidate with config.
pub fn try_z_candidate_with_config(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    try_z_candidate_with_config_query(source, target, obstacle_map, opened_cells, config)
}

fn try_z_candidate_with_config_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    let source_heading = source.angle % 8;
    let target_heading = target.angle % 8;
    if !is_cardinal_heading(source_heading) || !is_cardinal_heading(target_heading) {
        return None;
    }
    if source_heading != target_heading {
        return None;
    }

    let source_point = grid_point_from_state(source);
    let target_point = grid_point_from_state(target);
    if source_point == target_point {
        return None;
    }

    let min_leg_len = config.min_leg_len_cells.max(0);
    let offsets = compact_offset_order(config.max_offset_cells, config.include_zero_offset);
    let distances = distances_from_offsets(min_leg_len, &offsets);
    if distances.is_empty() || offsets.is_empty() {
        return None;
    }

    let lanes = z_middle_lanes(
        source_point,
        target_point,
        source_heading,
        &offsets,
        &distances,
        min_leg_len,
    );

    for lane in lanes {
        let candidate = if source_heading == 0 || source_heading == 4 {
            SimpleRouteCandidate::new(
                SimpleRouteKind::ZShape,
                vec![
                    source_point,
                    GridPoint::new(lane, source_point.y),
                    GridPoint::new(lane, target_point.y),
                    target_point,
                ],
            )
        } else {
            SimpleRouteCandidate::new(
                SimpleRouteKind::ZShape,
                vec![
                    source_point,
                    GridPoint::new(source_point.x, lane),
                    GridPoint::new(target_point.x, lane),
                    target_point,
                ],
            )
        };

        if !is_valid_z_candidate(
            &candidate,
            source_heading,
            target_heading,
            min_leg_len,
            obstacle_query,
            opened_cells,
        ) {
            continue;
        }
        return Some(candidate);
    }

    None
}

fn z_middle_lanes(
    source_point: GridPoint,
    target_point: GridPoint,
    source_heading: u8,
    offsets: &[i32],
    distances: &[i32],
    min_leg_len: i32,
) -> Vec<i32> {
    let min_forward = min_leg_len.max(0);
    let (source_primary, target_primary, direction) = match source_heading {
        0 => (source_point.x, target_point.x, 1),
        2 => (source_point.y, target_point.y, 1),
        4 => (source_point.x, target_point.x, -1),
        6 => (source_point.y, target_point.y, -1),
        _ => return Vec::new(),
    };
    let span = (target_primary - source_primary) * direction;
    if span < 2 * min_forward {
        return Vec::new();
    }

    let min_lane = min_forward;
    let max_lane = span - min_forward;
    let middle = min_lane + (max_lane - min_lane) / 2;
    let mut lanes = Vec::new();

    for &offset in offsets {
        push_forward_lane(
            &mut lanes,
            source_primary,
            direction,
            middle + offset,
            min_lane,
            max_lane,
        );
    }

    for &distance in distances {
        push_forward_lane(
            &mut lanes,
            source_primary,
            direction,
            distance,
            min_lane,
            max_lane,
        );
        push_forward_lane(
            &mut lanes,
            source_primary,
            direction,
            span - distance,
            min_lane,
            max_lane,
        );
    }

    lanes
}

fn push_forward_lane(
    lanes: &mut Vec<i32>,
    source_primary: i32,
    direction: i32,
    forward_distance: i32,
    min_lane: i32,
    max_lane: i32,
) {
    if forward_distance < min_lane || forward_distance > max_lane {
        return;
    }
    push_unique(lanes, source_primary + direction * forward_distance);
}

/// Try constructing a deterministic four-bend route for same-heading ports
/// where the target lies behind the source direction.
pub fn try_turnaround_candidate_with_config(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    try_turnaround_candidate_with_config_query(source, target, obstacle_map, opened_cells, config)
}

fn try_turnaround_candidate_with_config_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    let source_heading = source.angle % 8;
    let target_heading = target.angle % 8;
    if !is_cardinal_heading(source_heading) || source_heading != target_heading {
        return None;
    }

    let source_point = grid_point_from_state(source);
    let target_point = grid_point_from_state(target);
    if source_point == target_point {
        return None;
    }

    let min_leg_len = config.min_leg_len_cells.max(0);
    let offsets = compact_offset_order(config.max_offset_cells, config.include_zero_offset);
    let distances = distances_from_offsets(min_leg_len, &offsets);
    if distances.is_empty() {
        return None;
    }

    match source_heading {
        0 | 4 => {
            let direction = if source_heading == 0 { 1 } else { -1 };
            if (target_point.x - source_point.x) * direction >= 0 {
                return None;
            }
            let lanes = turnaround_lanes(source_point.y, target_point.y, &distances, min_leg_len);
            for distance in &distances {
                let source_turn_x = source_point.x + direction * *distance;
                let target_turn_x = target_point.x - direction * *distance;
                for lane_y in &lanes {
                    let candidate = SimpleRouteCandidate::new(
                        SimpleRouteKind::Turnaround,
                        vec![
                            source_point,
                            GridPoint::new(source_turn_x, source_point.y),
                            GridPoint::new(source_turn_x, *lane_y),
                            GridPoint::new(target_turn_x, *lane_y),
                            GridPoint::new(target_turn_x, target_point.y),
                            target_point,
                        ],
                    );
                    if is_valid_turnaround_candidate(
                        &candidate,
                        source_heading,
                        target_heading,
                        min_leg_len,
                        obstacle_query,
                        opened_cells,
                    ) {
                        return Some(candidate);
                    }
                }
            }
        }
        2 | 6 => {
            let direction = if source_heading == 2 { 1 } else { -1 };
            if (target_point.y - source_point.y) * direction >= 0 {
                return None;
            }
            let lanes = turnaround_lanes(source_point.x, target_point.x, &distances, min_leg_len);
            for distance in &distances {
                let source_turn_y = source_point.y + direction * *distance;
                let target_turn_y = target_point.y - direction * *distance;
                for lane_x in &lanes {
                    let candidate = SimpleRouteCandidate::new(
                        SimpleRouteKind::Turnaround,
                        vec![
                            source_point,
                            GridPoint::new(source_point.x, source_turn_y),
                            GridPoint::new(*lane_x, source_turn_y),
                            GridPoint::new(*lane_x, target_turn_y),
                            GridPoint::new(target_point.x, target_turn_y),
                            target_point,
                        ],
                    );
                    if is_valid_turnaround_candidate(
                        &candidate,
                        source_heading,
                        target_heading,
                        min_leg_len,
                        obstacle_query,
                        opened_cells,
                    ) {
                        return Some(candidate);
                    }
                }
            }
        }
        _ => return None,
    }

    None
}

/// Try constructing a deterministic two-bend Z-route candidate with defaults.
pub fn try_z_candidate(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    try_z_candidate_with_config(
        source,
        target,
        obstacle_map,
        opened_cells,
        &SimpleZRouteConfig::default(),
    )
}

/// Try deterministic straight, then L, then Z.
pub fn try_straight_l_or_z_candidate_with_config(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    z_config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    try_straight_l_or_z_candidate_with_query(source, target, obstacle_map, opened_cells, z_config)
}

pub fn try_straight_l_or_z_candidate_with_dynamic_expansion_config(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    z_config: &SimpleZRouteConfig,
    dynamic_expansion_radius_cells: i32,
    clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    let query = ExpandedDynamicObstacleQuery::new(
        obstacle_map,
        dynamic_expansion_radius_cells,
        clearance_exempt_cells,
    );
    try_straight_l_or_z_candidate_with_query(source, target, &query, opened_cells, z_config)
}

/// Try deterministic straight, L, then Z candidates using cardinal or diagonal
/// octant segments. Each leg is a single octant; this intentionally does not
/// split one logical leg into diagonal-plus-cardinal residual pieces yet.
pub fn try_45_degree_straight_l_or_z_candidate_with_config(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    z_config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    try_45_degree_straight_l_or_z_candidate_with_query(
        source,
        target,
        obstacle_map,
        opened_cells,
        z_config,
    )
}

/// Dynamic-expansion variant of the 45-degree simple-route candidate builder.
pub fn try_45_degree_straight_l_or_z_candidate_with_dynamic_expansion_config(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    z_config: &SimpleZRouteConfig,
    dynamic_expansion_radius_cells: i32,
    clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    let query = ExpandedDynamicObstacleQuery::new(
        obstacle_map,
        dynamic_expansion_radius_cells,
        clearance_exempt_cells,
    );
    try_45_degree_straight_l_or_z_candidate_with_query(
        source,
        target,
        &query,
        opened_cells,
        z_config,
    )
}

fn try_45_degree_straight_l_or_z_candidate_with_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
    z_config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    try_45_degree_straight_candidate_with_query(source, target, obstacle_query, opened_cells)
        .or_else(|| {
            try_45_degree_l_candidate_with_query(
                source,
                target,
                obstacle_query,
                opened_cells,
                z_config.min_leg_len_cells,
            )
        })
        .or_else(|| {
            try_45_degree_z_candidate_with_query(
                source,
                target,
                obstacle_query,
                opened_cells,
                z_config,
            )
        })
}

fn try_45_degree_straight_candidate_with_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    let source_heading = source.angle % 8;
    let target_heading = target.angle % 8;
    let source_point = grid_point_from_state(source);
    let target_point = grid_point_from_state(target);
    if source_point == target_point {
        return None;
    }
    let route_heading = direction_between_octant(source_point, target_point)?;
    if route_heading != source_heading || route_heading != target_heading {
        return None;
    }
    let candidate =
        SimpleRouteCandidate::new(SimpleRouteKind::Straight, vec![source_point, target_point]);
    if check_simple_candidate_with_query(&candidate, obstacle_query, opened_cells) {
        Some(candidate)
    } else {
        None
    }
}

fn try_45_degree_l_candidate_with_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
    min_leg_len_cells: i32,
) -> Option<SimpleRouteCandidate> {
    let source_heading = source.angle % 8;
    let target_heading = target.angle % 8;
    if !heading_delta_is_simple_bend(source_heading, target_heading) {
        return None;
    }
    let source_point = grid_point_from_state(source);
    let target_point = grid_point_from_state(target);
    let (first_len, second_len) =
        solve_two_heading_lengths(source_point, target_point, source_heading, target_heading)?;
    let min_leg_len = min_leg_len_cells.max(1);
    if first_len < min_leg_len || second_len < min_leg_len {
        return None;
    }
    let bend = offset_point(source_point, source_heading, first_len);
    let candidate = SimpleRouteCandidate::new(
        SimpleRouteKind::LShape,
        vec![source_point, bend, target_point],
    );
    if is_valid_octant_l_candidate(
        &candidate,
        source_heading,
        target_heading,
        min_leg_len,
        obstacle_query,
        opened_cells,
    ) {
        Some(candidate)
    } else {
        None
    }
}

fn try_45_degree_z_candidate_with_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    let source_heading = source.angle % 8;
    let target_heading = target.angle % 8;
    if source_heading != target_heading {
        return None;
    }
    let source_point = grid_point_from_state(source);
    let target_point = grid_point_from_state(target);
    if source_point == target_point {
        return None;
    }

    let min_leg_len = config.min_leg_len_cells.max(1);
    let offsets = compact_offset_order(config.max_offset_cells, config.include_zero_offset);
    let distances = distances_from_offsets(min_leg_len, &offsets);
    if distances.is_empty() {
        return None;
    }

    for middle_heading in z_middle_headings(source_heading) {
        let Some((forward_total, middle_len)) =
            solve_two_heading_lengths(source_point, target_point, source_heading, middle_heading)
        else {
            continue;
        };
        if middle_len < 2 * min_leg_len || forward_total < 2 * min_leg_len {
            continue;
        }
        let min_forward = min_leg_len;
        let max_forward = forward_total - min_leg_len;
        let middle_forward = min_forward + (max_forward - min_forward) / 2;
        let mut first_lengths = Vec::new();
        push_forward_lane(
            &mut first_lengths,
            0,
            1,
            middle_forward,
            min_forward,
            max_forward,
        );
        for distance in &distances {
            push_forward_lane(
                &mut first_lengths,
                0,
                1,
                *distance,
                min_forward,
                max_forward,
            );
            push_forward_lane(
                &mut first_lengths,
                0,
                1,
                forward_total - *distance,
                min_forward,
                max_forward,
            );
        }
        for first_len in first_lengths {
            let second_forward_len = forward_total - first_len;
            if second_forward_len < min_leg_len {
                continue;
            }
            let p1 = offset_point(source_point, source_heading, first_len);
            let p2 = offset_point(p1, middle_heading, middle_len);
            let candidate = SimpleRouteCandidate::new(
                SimpleRouteKind::ZShape,
                vec![source_point, p1, p2, target_point],
            );
            if is_valid_octant_z_candidate(
                &candidate,
                source_heading,
                target_heading,
                min_leg_len,
                obstacle_query,
                opened_cells,
            ) {
                return Some(candidate);
            }
        }
    }

    None
}

fn z_middle_headings(source_heading: u8) -> [u8; 2] {
    [(source_heading + 1) % 8, (source_heading + 7) % 8]
}

fn solve_two_heading_lengths(
    source: GridPoint,
    target: GridPoint,
    first_heading: u8,
    second_heading: u8,
) -> Option<(i32, i32)> {
    let (ax, ay) = heading_vector(first_heading);
    let (bx, by) = heading_vector(second_heading);
    let dx = target.x - source.x;
    let dy = target.y - source.y;
    let det = ax * by - ay * bx;
    if det == 0 {
        return None;
    }
    let first_num = dx * by - dy * bx;
    let second_num = ax * dy - ay * dx;
    if first_num % det != 0 || second_num % det != 0 {
        return None;
    }
    let first_len = first_num / det;
    let second_len = second_num / det;
    if first_len <= 0 || second_len <= 0 {
        return None;
    }
    Some((first_len, second_len))
}

fn try_straight_l_or_z_candidate_with_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    source: State,
    target: State,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
    z_config: &SimpleZRouteConfig,
) -> Option<SimpleRouteCandidate> {
    try_straight_candidate_with_query(source, target, obstacle_query, opened_cells)
        .or_else(|| {
            try_l_candidate_with_min_leg_len_query(
                source,
                target,
                obstacle_query,
                opened_cells,
                z_config.min_leg_len_cells,
            )
        })
        .or_else(|| {
            try_z_candidate_with_config_query(
                source,
                target,
                obstacle_query,
                opened_cells,
                z_config,
            )
        })
        .or_else(|| {
            try_turnaround_candidate_with_config_query(
                source,
                target,
                obstacle_query,
                opened_cells,
                z_config,
            )
        })
}

/// Try deterministic straight, then L, then Z using default Z config.
pub fn try_straight_l_or_z_candidate(
    source: State,
    target: State,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> Option<SimpleRouteCandidate> {
    try_straight_l_or_z_candidate_with_config(
        source,
        target,
        obstacle_map,
        opened_cells,
        &SimpleZRouteConfig::default(),
    )
}

/// Expand a corner polyline into every touched grid point.
///
/// Example:
/// `[A, B, C]` expands to `A..B` then `B..C`, without duplicating `B`.
pub fn expand_candidate_to_grid_points(candidate: &SimpleRouteCandidate) -> Vec<GridPoint> {
    let mut out = Vec::new();
    for (idx, segment) in candidate.segments().into_iter().enumerate() {
        let Some(seg_points) = expand_segment_points(segment) else {
            return Vec::new();
        };

        if idx == 0 {
            out.extend(seg_points);
        } else {
            out.extend(seg_points.into_iter().skip(1));
        }
    }

    out
}

/// Validate a pre-built simple candidate against geometric and obstacle rules.
pub fn check_simple_candidate(
    candidate: &SimpleRouteCandidate,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    check_simple_candidate_with_query(candidate, obstacle_map, opened_cells)
}

fn check_simple_candidate_with_query<Q: SimpleRouteObstacleQuery + ?Sized>(
    candidate: &SimpleRouteCandidate,
    obstacle_query: &Q,
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    if candidate.points.len() < 2 {
        return false;
    }
    if candidate.has_duplicate_consecutive_points() {
        return false;
    }
    for segment in candidate.segments() {
        if !obstacle_query.check_segment_free(segment, opened_cells) {
            return false;
        }
    }

    true
}

fn expand_segment_points(segment: Segment) -> Option<Vec<GridPoint>> {
    let dx = segment.end.x - segment.start.x;
    let dy = segment.end.y - segment.start.y;

    if dx != 0 && dy != 0 && dx.abs() != dy.abs() {
        return None;
    }

    let (step_x, step_y, steps) = if dx != 0 && dy != 0 {
        (dx.signum(), dy.signum(), dx.abs())
    } else if dx != 0 {
        (dx.signum(), 0, dx.abs())
    } else {
        (0, dy.signum(), dy.abs())
    };

    let steps = usize::try_from(steps).ok()?;
    let mut points = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let i = i as i32;
        points.push(GridPoint::new(
            segment.start.x + (i * step_x),
            segment.start.y + (i * step_y),
        ));
    }

    Some(points)
}

fn push_unique_grid_cell(cells: &mut Vec<(i32, i32)>, cell: (i32, i32)) {
    if !cells.contains(&cell) {
        cells.push(cell);
    }
}

fn distances_from_offsets(min_leg_len_cells: i32, offsets: &[i32]) -> Vec<i32> {
    let mut out = Vec::new();
    for &offset in offsets {
        let distance = min_leg_len_cells + offset.abs();
        if !out.contains(&distance) {
            out.push(distance);
        }
    }
    out
}

fn is_valid_z_candidate(
    candidate: &SimpleRouteCandidate,
    source_heading: u8,
    target_heading: u8,
    min_leg_len: i32,
    obstacle_query: &(impl SimpleRouteObstacleQuery + ?Sized),
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    if candidate.kind != SimpleRouteKind::ZShape || candidate.points.len() != 4 {
        return false;
    }
    if candidate.has_duplicate_consecutive_points() || !candidate.is_axis_aligned() {
        return false;
    }

    let p0 = candidate.points[0];
    let p1 = candidate.points[1];
    let p2 = candidate.points[2];
    let p3 = candidate.points[3];

    let Some(h01) = direction_between(p0, p1) else {
        return false;
    };
    let Some(h12) = direction_between(p1, p2) else {
        return false;
    };
    let Some(h23) = direction_between(p2, p3) else {
        return false;
    };

    if h01 != source_heading {
        return false;
    }
    if h23 != target_heading {
        return false;
    }
    if !heading_delta_is_perpendicular(h01, h12) || !heading_delta_is_perpendicular(h12, h23) {
        return false;
    }

    let segments = candidate.segments();
    for (idx, segment) in segments.iter().enumerate() {
        let required_len = if idx == 1 {
            2 * min_leg_len
        } else {
            min_leg_len
        };
        if segment.manhattan_len() < required_len {
            return false;
        }
    }

    check_simple_candidate_with_query(candidate, obstacle_query, opened_cells)
}

fn is_valid_l_candidate(
    candidate: &SimpleRouteCandidate,
    source_heading: u8,
    target_heading: u8,
    min_leg_len: i32,
    obstacle_query: &(impl SimpleRouteObstacleQuery + ?Sized),
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    if candidate.kind != SimpleRouteKind::LShape || candidate.points.len() != 3 {
        return false;
    }
    if candidate.has_duplicate_consecutive_points() || !candidate.is_axis_aligned() {
        return false;
    }

    let p0 = candidate.points[0];
    let p1 = candidate.points[1];
    let p2 = candidate.points[2];
    let Some(h01) = direction_between(p0, p1) else {
        return false;
    };
    let Some(h12) = direction_between(p1, p2) else {
        return false;
    };

    if h01 != source_heading || h12 != target_heading {
        return false;
    }
    if !heading_delta_is_perpendicular(h01, h12) {
        return false;
    }
    if candidate
        .segments()
        .iter()
        .any(|segment| segment.manhattan_len() < min_leg_len)
    {
        return false;
    }

    check_simple_candidate_with_query(candidate, obstacle_query, opened_cells)
}

fn is_valid_octant_l_candidate(
    candidate: &SimpleRouteCandidate,
    source_heading: u8,
    target_heading: u8,
    min_leg_len: i32,
    obstacle_query: &(impl SimpleRouteObstacleQuery + ?Sized),
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    if candidate.kind != SimpleRouteKind::LShape || candidate.points.len() != 3 {
        return false;
    }
    if candidate.has_duplicate_consecutive_points() || !candidate.is_octant_aligned() {
        return false;
    }

    let p0 = candidate.points[0];
    let p1 = candidate.points[1];
    let p2 = candidate.points[2];
    let Some(h01) = direction_between_octant(p0, p1) else {
        return false;
    };
    let Some(h12) = direction_between_octant(p1, p2) else {
        return false;
    };

    if h01 != source_heading || h12 != target_heading {
        return false;
    }
    if !heading_delta_is_simple_bend(h01, h12) {
        return false;
    }
    if candidate
        .segments()
        .iter()
        .any(|segment| segment.step_len() < min_leg_len)
    {
        return false;
    }

    check_simple_candidate_with_query(candidate, obstacle_query, opened_cells)
}

fn is_valid_octant_z_candidate(
    candidate: &SimpleRouteCandidate,
    source_heading: u8,
    target_heading: u8,
    min_leg_len: i32,
    obstacle_query: &(impl SimpleRouteObstacleQuery + ?Sized),
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    if candidate.kind != SimpleRouteKind::ZShape || candidate.points.len() != 4 {
        return false;
    }
    if candidate.has_duplicate_consecutive_points() || !candidate.is_octant_aligned() {
        return false;
    }

    let p0 = candidate.points[0];
    let p1 = candidate.points[1];
    let p2 = candidate.points[2];
    let p3 = candidate.points[3];
    let Some(h01) = direction_between_octant(p0, p1) else {
        return false;
    };
    let Some(h12) = direction_between_octant(p1, p2) else {
        return false;
    };
    let Some(h23) = direction_between_octant(p2, p3) else {
        return false;
    };

    if h01 != source_heading || h23 != target_heading || h01 != h23 {
        return false;
    }
    if !heading_delta_is_simple_bend(h01, h12) || !heading_delta_is_simple_bend(h12, h23) {
        return false;
    }

    let segments = candidate.segments();
    for (idx, segment) in segments.iter().enumerate() {
        let required_len = if idx == 1 {
            2 * min_leg_len
        } else {
            min_leg_len
        };
        if segment.step_len() < required_len {
            return false;
        }
    }

    check_simple_candidate_with_query(candidate, obstacle_query, opened_cells)
}

fn turnaround_lanes(
    source_orthogonal: i32,
    target_orthogonal: i32,
    distances: &[i32],
    min_leg_len: i32,
) -> Vec<i32> {
    let low = source_orthogonal.min(target_orthogonal);
    let high = source_orthogonal.max(target_orthogonal);
    let mut lanes = Vec::new();

    let min_separation = 2 * min_leg_len.max(0);
    if high - low >= 2 * min_separation {
        let middle = low + (high - low) / 2;
        push_unique(&mut lanes, middle);
        for &distance in distances {
            push_unique(&mut lanes, middle + distance);
            push_unique(&mut lanes, middle - distance);
        }
    }

    for &distance in distances {
        push_unique(&mut lanes, high + distance);
        push_unique(&mut lanes, low - distance);
    }

    lanes
}

fn push_unique(values: &mut Vec<i32>, value: i32) {
    if !values.contains(&value) {
        values.push(value);
    }
}

fn is_valid_turnaround_candidate(
    candidate: &SimpleRouteCandidate,
    source_heading: u8,
    target_heading: u8,
    min_leg_len: i32,
    obstacle_query: &(impl SimpleRouteObstacleQuery + ?Sized),
    opened_cells: Option<&FxHashSet<CellKey>>,
) -> bool {
    if candidate.kind != SimpleRouteKind::Turnaround || candidate.points.len() != 6 {
        return false;
    }
    if candidate.has_duplicate_consecutive_points() || !candidate.is_axis_aligned() {
        return false;
    }

    let segments = candidate.segments();
    if segments.len() != 5 {
        return false;
    }
    let mut headings = Vec::with_capacity(segments.len());
    for segment in &segments {
        let Some(heading) = direction_between(segment.start, segment.end) else {
            return false;
        };
        headings.push(heading);
    }

    if headings[0] != source_heading || headings[4] != target_heading {
        return false;
    }
    for pair in headings.windows(2) {
        if !heading_delta_is_perpendicular(pair[0], pair[1]) {
            return false;
        }
    }
    if headings[2] != opposite_heading(source_heading) {
        return false;
    }

    for (idx, segment) in segments.iter().enumerate() {
        let required_len = if idx == 0 || idx + 1 == segments.len() {
            min_leg_len
        } else {
            2 * min_leg_len
        };
        if segment.manhattan_len() < required_len {
            return false;
        }
    }

    check_simple_candidate_with_query(candidate, obstacle_query, opened_cells)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obstacle_map::{pack_xy, unpack_xy};

    #[test]
    fn segment_orientation_horizontal() {
        let seg = Segment::new(GridPoint::new(1, 2), GridPoint::new(5, 2));
        assert!(seg.is_horizontal());
        assert!(!seg.is_vertical());
        assert!(seg.is_axis_aligned());
        assert_eq!(seg.manhattan_len(), 4);
    }

    #[test]
    fn segment_orientation_vertical() {
        let seg = Segment::new(GridPoint::new(3, 7), GridPoint::new(3, 2));
        assert!(seg.is_vertical());
        assert!(!seg.is_horizontal());
        assert!(seg.is_axis_aligned());
        assert_eq!(seg.normalized().start, GridPoint::new(3, 2));
        assert_eq!(seg.normalized().end, GridPoint::new(3, 7));
    }

    #[test]
    fn segment_orientation_non_axis_aligned() {
        let seg = Segment::new(GridPoint::new(0, 0), GridPoint::new(1, 1));
        assert!(!seg.is_horizontal());
        assert!(!seg.is_vertical());
        assert!(!seg.is_axis_aligned());
        assert!(seg.is_diagonal_45());
        assert!(seg.is_octant_aligned());
        assert_eq!(seg.step_len(), 1);
    }

    #[test]
    fn candidate_bend_count_matches_shape() {
        let straight = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(0, 0), GridPoint::new(4, 0)],
        );
        let lshape = SimpleRouteCandidate::new(
            SimpleRouteKind::LShape,
            vec![
                GridPoint::new(0, 0),
                GridPoint::new(4, 0),
                GridPoint::new(4, 3),
            ],
        );
        let zshape = SimpleRouteCandidate::new(
            SimpleRouteKind::ZShape,
            vec![
                GridPoint::new(0, 0),
                GridPoint::new(3, 0),
                GridPoint::new(3, 2),
                GridPoint::new(6, 2),
            ],
        );
        assert_eq!(straight.num_bends(), 0);
        assert_eq!(lshape.num_bends(), 1);
        assert_eq!(zshape.num_bends(), 2);
    }

    #[test]
    fn simple_candidate_rejects_congested_cells() {
        let mut map = ObstacleMap::new(8, 4);
        assert!(map.add_congestion_cost(2, 1, 1));
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(4, 1)],
        );

        assert!(!check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn simple_candidate_allows_congested_opened_port_cells() {
        let mut map = ObstacleMap::new(8, 4);
        assert!(map.add_congestion_cost(2, 1, 1));
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(4, 1)],
        );
        let opened = FxHashSet::from_iter([pack_xy(2, 1)]);

        assert!(check_simple_candidate(&candidate, &map, Some(&opened)));
    }

    #[test]
    fn expanded_simple_candidate_rejects_congested_diagonal_halo_cells() {
        let mut map = ObstacleMap::new(8, 8);
        assert!(map.add_congestion_cost(2, 1, 1));
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(4, 4)],
        );
        let query = ExpandedDynamicObstacleQuery::new(&map, 0, None);

        assert!(!check_simple_candidate_with_query(&candidate, &query, None));
    }

    #[test]
    fn candidate_detects_duplicate_consecutive_points() {
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::LShape,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(1, 1),
                GridPoint::new(2, 1),
            ],
        );
        assert!(candidate.has_duplicate_consecutive_points());
    }

    #[test]
    fn expand_straight_candidate() {
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(0, 0), GridPoint::new(3, 0)],
        );
        let expanded = expand_candidate_to_grid_points(&candidate);
        assert_eq!(
            expanded,
            vec![
                GridPoint::new(0, 0),
                GridPoint::new(1, 0),
                GridPoint::new(2, 0),
                GridPoint::new(3, 0)
            ]
        );
    }

    #[test]
    fn expand_l_candidate_without_duplicate_bend_point() {
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::LShape,
            vec![
                GridPoint::new(0, 0),
                GridPoint::new(2, 0),
                GridPoint::new(2, 2),
            ],
        );
        let expanded = expand_candidate_to_grid_points(&candidate);
        assert_eq!(
            expanded,
            vec![
                GridPoint::new(0, 0),
                GridPoint::new(1, 0),
                GridPoint::new(2, 0),
                GridPoint::new(2, 1),
                GridPoint::new(2, 2),
            ]
        );
    }

    #[test]
    fn expand_diagonal_candidate() {
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(4, 4)],
        );
        let expanded = expand_candidate_to_grid_points(&candidate);
        assert_eq!(
            expanded,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(2, 2),
                GridPoint::new(3, 3),
                GridPoint::new(4, 4),
            ]
        );
    }

    #[test]
    fn candidate_validation_accepts_clear_straight_route() {
        let map = ObstacleMap::new(10, 10);
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(5, 1)],
        );
        assert!(check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn candidate_validation_accepts_diagonal_segment() {
        let map = ObstacleMap::new(10, 10);
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(2, 2)],
        );
        assert!(check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn candidate_validation_rejects_non_octant_segment() {
        let map = ObstacleMap::new(10, 10);
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(3, 2)],
        );
        assert!(!check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn candidate_validation_rejects_duplicate_consecutive_points() {
        let map = ObstacleMap::new(10, 10);
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::LShape,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(1, 1),
                GridPoint::new(2, 1),
            ],
        );
        assert!(!check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn candidate_validation_rejects_blocked_crossing_cell() {
        let mut map = ObstacleMap::new(10, 10);
        assert!(map.add_static_cell(3, 1));
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(5, 1)],
        );
        assert!(!check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn candidate_validation_matches_cell_checker_for_l_route() {
        let mut map = ObstacleMap::new(10, 10);
        assert!(map.add_static_cell(3, 2));
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::LShape,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(5, 1),
                GridPoint::new(5, 3),
            ],
        );
        let expanded = expand_candidate_to_grid_points(&candidate);
        let expanded_cells: Vec<(i32, i32)> = expanded.into_iter().map(|p| (p.x, p.y)).collect();

        let expected = map.check_cells_free(&expanded_cells, None);
        let actual = check_simple_candidate(&candidate, &map, None);
        assert_eq!(actual, expected);
    }

    #[test]
    fn candidate_validation_opened_cells_allow_blocked_endpoints() {
        let mut map = ObstacleMap::new(10, 10);
        assert!(map.add_static_cell(1, 1));
        assert!(map.add_static_cell(5, 1));

        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(5, 1)],
        );

        assert!(!check_simple_candidate(&candidate, &map, None));

        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(1, 1));
        opened.insert(pack_xy(5, 1));

        assert!(check_simple_candidate(&candidate, &map, Some(&opened)));
    }

    #[test]
    fn dynamic_simple_query_does_not_expand_cardinal_segments() {
        let mut map = ObstacleMap::new(10, 10);
        let core = vec![(1, 1), (2, 1), (3, 1)];
        let blocked = vec![(1, 1), (2, 1), (3, 1), (1, 2), (2, 2), (3, 2)];
        assert!(map.commit_route_with_clearance_overlap(7, &core, &blocked, &[]));

        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 2), GridPoint::new(3, 2)],
        );
        let query = ExpandedDynamicObstacleQuery::new(&map, 1, None);

        assert!(check_simple_candidate_with_query(&candidate, &query, None));
    }

    #[test]
    fn dynamic_simple_query_adds_halo_only_to_diagonal_segments() {
        let mut map = ObstacleMap::new(10, 10);
        assert!(map.commit_route_with_clearance_overlap(7, &[(2, 1)], &[(2, 1)], &[]));

        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(3, 3)],
        );
        let query = ExpandedDynamicObstacleQuery::new(&map, 1, None);

        assert!(!check_simple_candidate_with_query(&candidate, &query, None));
    }

    #[test]
    fn candidate_validation_matches_cell_checker_for_z_route() {
        let mut map = ObstacleMap::new(12, 12);
        assert!(map.add_static_cell(6, 4));
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::ZShape,
            vec![
                GridPoint::new(1, 2),
                GridPoint::new(6, 2),
                GridPoint::new(6, 5),
                GridPoint::new(10, 5),
            ],
        );
        let expanded = expand_candidate_to_grid_points(&candidate);
        let expanded_cells: Vec<(i32, i32)> = expanded.into_iter().map(|p| (p.x, p.y)).collect();

        let expected = map.check_cells_free(&expanded_cells, None);
        let actual = check_simple_candidate(&candidate, &map, None);
        assert_eq!(actual, expected);
        assert!(!actual);
    }

    #[test]
    fn straight_candidate_succeeds_for_aligned_matching_headings() {
        let map = ObstacleMap::new(10, 10);
        let source = State::new(1, 1, 0);
        let target = State::new(5, 1, 0);

        let candidate = try_straight_candidate(source, target, &map, None)
            .expect("aligned cardinal route should be valid");
        assert_eq!(candidate.kind, SimpleRouteKind::Straight);
        assert_eq!(
            candidate.points,
            vec![GridPoint::new(1, 1), GridPoint::new(5, 1)]
        );
    }

    #[test]
    fn straight_candidate_rejects_wrong_source_heading() {
        let map = ObstacleMap::new(10, 10);
        let source = State::new(1, 1, 2);
        let target = State::new(5, 1, 0);
        assert!(try_straight_candidate(source, target, &map, None).is_none());
    }

    #[test]
    fn straight_candidate_rejects_wrong_target_heading() {
        let map = ObstacleMap::new(10, 10);
        let source = State::new(1, 1, 0);
        let target = State::new(5, 1, 2);
        assert!(try_straight_candidate(source, target, &map, None).is_none());
    }

    #[test]
    fn straight_candidate_rejects_blocked_cell() {
        let mut map = ObstacleMap::new(10, 10);
        assert!(map.add_static_cell(3, 1));
        let source = State::new(1, 1, 0);
        let target = State::new(5, 1, 0);
        assert!(try_straight_candidate(source, target, &map, None).is_none());
    }

    #[test]
    fn l_candidate_succeeds_for_perpendicular_route() {
        let map = ObstacleMap::new(10, 10);
        let source = State::new(1, 1, 0);
        let target = State::new(5, 4, 2);

        let candidate =
            try_l_candidate(source, target, &map, None).expect("L route should be valid");
        assert_eq!(candidate.kind, SimpleRouteKind::LShape);
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(5, 1),
                GridPoint::new(5, 4)
            ]
        );
    }

    #[test]
    fn l_candidate_rejects_wrong_headings() {
        let map = ObstacleMap::new(10, 10);
        let wrong_source = try_l_candidate(State::new(1, 1, 2), State::new(5, 4, 2), &map, None);
        assert!(wrong_source.is_none());

        let wrong_target = try_l_candidate(State::new(1, 1, 0), State::new(5, 4, 0), &map, None);
        assert!(wrong_target.is_none());
    }

    #[test]
    fn l_candidate_rejects_blocked_first_leg() {
        let mut map = ObstacleMap::new(10, 10);
        assert!(map.add_static_cell(3, 1));
        let source = State::new(1, 1, 0);
        let target = State::new(5, 4, 2);
        assert!(try_l_candidate(source, target, &map, None).is_none());
    }

    #[test]
    fn l_candidate_rejects_blocked_second_leg() {
        let mut map = ObstacleMap::new(10, 10);
        assert!(map.add_static_cell(5, 3));
        let source = State::new(1, 1, 0);
        let target = State::new(5, 4, 2);
        assert!(try_l_candidate(source, target, &map, None).is_none());
    }

    #[test]
    fn l_candidate_does_not_return_degenerate_straight() {
        let map = ObstacleMap::new(10, 10);
        let source = State::new(1, 1, 0);
        let target = State::new(5, 1, 0);
        assert!(try_l_candidate(source, target, &map, None).is_none());
    }

    #[test]
    fn try_straight_or_l_prefers_straight() {
        let map = ObstacleMap::new(10, 10);
        let source = State::new(1, 1, 0);
        let target = State::new(5, 1, 0);
        let candidate = try_straight_or_l_candidate(source, target, &map, None)
            .expect("straight should be chosen");
        assert_eq!(candidate.kind, SimpleRouteKind::Straight);
    }

    #[test]
    fn diagonal_headings_are_rejected() {
        let map = ObstacleMap::new(10, 10);
        assert!(
            try_straight_or_l_candidate(State::new(1, 1, 1), State::new(5, 1, 0), &map, None)
                .is_none()
        );
        assert!(
            try_straight_or_l_candidate(State::new(1, 1, 0), State::new(5, 1, 7), &map, None)
                .is_none()
        );
    }

    #[test]
    fn forty_five_straight_candidate_succeeds_for_diagonal() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig::default();
        let candidate = try_45_degree_straight_l_or_z_candidate_with_config(
            State::new(1, 1, 1),
            State::new(5, 5, 1),
            &map,
            None,
            &cfg,
        )
        .expect("diagonal straight should be valid");
        assert_eq!(candidate.kind, SimpleRouteKind::Straight);
        assert_eq!(
            candidate.points,
            vec![GridPoint::new(1, 1), GridPoint::new(5, 5)]
        );
    }

    #[test]
    fn forty_five_straight_candidate_rejects_wrong_heading() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig::default();
        assert!(try_45_degree_straight_l_or_z_candidate_with_config(
            State::new(1, 1, 0),
            State::new(5, 5, 1),
            &map,
            None,
            &cfg,
        )
        .is_none());
    }

    #[test]
    fn forty_five_straight_candidate_rejects_blocked_diagonal() {
        let mut map = ObstacleMap::new(20, 20);
        assert!(map.add_static_cell(3, 3));
        let cfg = SimpleZRouteConfig::default();
        assert!(try_45_degree_straight_l_or_z_candidate_with_config(
            State::new(1, 1, 1),
            State::new(5, 5, 1),
            &map,
            None,
            &cfg,
        )
        .is_none());
    }

    #[test]
    fn forty_five_l_candidate_uses_single_diagonal_leg() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig::default();
        let candidate = try_45_degree_straight_l_or_z_candidate_with_config(
            State::new(1, 1, 0),
            State::new(7, 4, 1),
            &map,
            None,
            &cfg,
        )
        .expect("45-degree L should be valid");
        assert_eq!(candidate.kind, SimpleRouteKind::LShape);
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(4, 1),
                GridPoint::new(7, 4),
            ]
        );
    }

    #[test]
    fn forty_five_z_candidate_uses_single_diagonal_middle_leg() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 4,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        let candidate = try_45_degree_straight_l_or_z_candidate_with_config(
            State::new(1, 1, 0),
            State::new(9, 5, 0),
            &map,
            None,
            &cfg,
        )
        .expect("45-degree Z should be valid");
        assert_eq!(candidate.kind, SimpleRouteKind::ZShape);
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(3, 1),
                GridPoint::new(7, 5),
                GridPoint::new(9, 5),
            ]
        );
    }

    #[test]
    fn forty_five_z_candidate_avoids_blocked_middle_leg() {
        let mut map = ObstacleMap::new(20, 20);
        assert!(map.add_static_cell(5, 3));
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 4,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        let candidate = try_45_degree_straight_l_or_z_candidate_with_config(
            State::new(1, 1, 0),
            State::new(9, 5, 0),
            &map,
            None,
            &cfg,
        )
        .expect("alternative 45-degree Z should be valid");
        assert_ne!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(3, 1),
                GridPoint::new(7, 5),
                GridPoint::new(9, 5),
            ]
        );
        assert!(check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn forty_five_z_candidate_rejects_cardinal_only_middle_leg() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 8,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        assert!(try_45_degree_straight_l_or_z_candidate_with_config(
            State::new(1, 8, 0),
            State::new(8, 1, 0),
            &map,
            None,
            &cfg,
        )
        .is_none());
    }

    #[test]
    fn compact_offset_order_with_zero() {
        assert_eq!(compact_offset_order(3, true), vec![0, 1, -1, 2, -2, 3, -3]);
        assert_eq!(compact_offset_order(0, true), vec![0]);
        assert_eq!(compact_offset_order(-2, true), vec![0]);
    }

    #[test]
    fn compact_offset_order_without_zero() {
        assert_eq!(compact_offset_order(3, false), vec![1, -1, 2, -2, 3, -3]);
        assert!(compact_offset_order(0, false).is_empty());
    }

    #[test]
    fn z_candidate_horizontal_east_to_east_succeeds() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 4,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        let source = State::new(1, 1, 0);
        let target = State::new(5, 4, 0);
        let candidate = try_z_candidate_with_config(source, target, &map, None, &cfg)
            .expect("expected horizontal Z candidate");
        assert_eq!(candidate.kind, SimpleRouteKind::ZShape);
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(3, 1),
                GridPoint::new(3, 4),
                GridPoint::new(5, 4),
            ]
        );
    }

    #[test]
    fn z_candidate_horizontal_west_to_west_succeeds() {
        let map = ObstacleMap::new(20, 20);
        let source = State::new(5, 1, 4);
        let target = State::new(1, 4, 4);
        let candidate =
            try_z_candidate(source, target, &map, None).expect("expected horizontal Z candidate");
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(5, 1),
                GridPoint::new(3, 1),
                GridPoint::new(3, 4),
                GridPoint::new(1, 4),
            ]
        );
    }

    #[test]
    fn z_candidate_vertical_north_to_north_succeeds() {
        let map = ObstacleMap::new(20, 20);
        let source = State::new(1, 1, 2);
        let target = State::new(4, 5, 2);
        let candidate =
            try_z_candidate(source, target, &map, None).expect("expected vertical Z candidate");
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(1, 3),
                GridPoint::new(4, 3),
                GridPoint::new(4, 5),
            ]
        );
    }

    #[test]
    fn z_candidate_vertical_south_to_south_succeeds() {
        let map = ObstacleMap::new(20, 20);
        let source = State::new(1, 5, 6);
        let target = State::new(4, 1, 6);
        let candidate =
            try_z_candidate(source, target, &map, None).expect("expected vertical Z candidate");
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 5),
                GridPoint::new(1, 3),
                GridPoint::new(4, 3),
                GridPoint::new(4, 1),
            ]
        );
    }

    #[test]
    fn z_candidate_rejects_non_matching_headings() {
        let map = ObstacleMap::new(20, 20);
        assert!(try_z_candidate(State::new(1, 1, 0), State::new(5, 4, 2), &map, None).is_none());
    }

    #[test]
    fn z_candidate_rejects_diagonal_heading() {
        let map = ObstacleMap::new(20, 20);
        assert!(try_z_candidate(State::new(1, 1, 1), State::new(5, 4, 4), &map, None).is_none());
        assert!(try_z_candidate(State::new(1, 1, 0), State::new(5, 4, 7), &map, None).is_none());
    }

    #[test]
    fn z_candidate_rejects_degenerate_aligned_case() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 0,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        assert!(try_z_candidate_with_config(
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            &map,
            None,
            &cfg
        )
        .is_none());
    }

    #[test]
    fn z_candidate_uses_midpoint_lane_first() {
        let map = ObstacleMap::new(20, 20);
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 4,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        let candidate =
            try_z_candidate_with_config(State::new(1, 1, 0), State::new(9, 4, 0), &map, None, &cfg)
                .expect("midpoint Z lane should be selected");
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(5, 1),
                GridPoint::new(5, 4),
                GridPoint::new(9, 4),
            ]
        );
    }

    #[test]
    fn z_candidate_avoids_blocked_midpoint_lane_and_uses_next_offset() {
        let mut map = ObstacleMap::new(20, 20);
        assert!(map.add_static_cell(5, 2));
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 4,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        let candidate =
            try_z_candidate_with_config(State::new(1, 1, 0), State::new(9, 4, 0), &map, None, &cfg)
                .expect("next midpoint offset should be selected after midpoint is blocked");
        assert_eq!(
            candidate.points,
            vec![
                GridPoint::new(1, 1),
                GridPoint::new(6, 1),
                GridPoint::new(6, 4),
                GridPoint::new(9, 4),
            ]
        );
    }

    #[test]
    fn z_candidate_rejects_blocked_all_candidates() {
        let mut map = ObstacleMap::new(20, 20);
        assert!(map.add_static_cell(2, 1));
        assert!(map.add_static_cell(3, 1));
        assert!(map.add_static_cell(4, 1));
        let cfg = SimpleZRouteConfig {
            max_offset_cells: 2,
            include_zero_offset: true,
            min_leg_len_cells: 1,
        };
        assert!(try_z_candidate_with_config(
            State::new(1, 1, 0),
            State::new(5, 4, 0),
            &map,
            None,
            &cfg,
        )
        .is_none());
    }

    #[test]
    fn try_straight_l_or_z_prefers_straight() {
        let map = ObstacleMap::new(20, 20);
        let c = try_straight_l_or_z_candidate(State::new(1, 1, 0), State::new(5, 1, 0), &map, None)
            .expect("straight candidate expected");
        assert_eq!(c.kind, SimpleRouteKind::Straight);
    }

    #[test]
    fn try_straight_l_or_z_prefers_l_before_z() {
        let map = ObstacleMap::new(20, 20);
        let c = try_straight_l_or_z_candidate(State::new(1, 1, 0), State::new(5, 4, 2), &map, None)
            .expect("L candidate expected");
        assert_eq!(c.kind, SimpleRouteKind::LShape);
    }

    #[test]
    fn try_straight_l_or_z_uses_z_for_matching_headings() {
        let map = ObstacleMap::new(20, 20);
        let c = try_straight_l_or_z_candidate(State::new(1, 1, 0), State::new(5, 4, 0), &map, None)
            .expect("Z candidate expected");
        assert_eq!(c.kind, SimpleRouteKind::ZShape);
    }

    #[test]
    fn heading_helpers_and_state_converters_work() {
        assert!(is_cardinal_heading(0));
        assert!(is_cardinal_heading(2));
        assert!(is_cardinal_heading(4));
        assert!(is_cardinal_heading(6));
        assert!(!is_cardinal_heading(1));
        assert!(!is_cardinal_heading(7));

        assert_eq!(opposite_heading(0), 4);
        assert_eq!(opposite_heading(7), 3);
        assert!(heading_delta_is_perpendicular(0, 2));
        assert!(heading_delta_is_perpendicular(2, 0));
        assert!(!heading_delta_is_perpendicular(0, 4));

        let point = GridPoint::new(9, 3);
        let state = state_from_grid_point(point, 10);
        assert_eq!(state.x, 9);
        assert_eq!(state.y, 3);
        assert_eq!(state.angle, 2);
        let roundtrip = grid_point_from_state(state);
        assert_eq!(roundtrip, point);

        let packed = pack_xy(point.x, point.y);
        assert_eq!(unpack_xy(packed), (point.x, point.y));
    }
}
