//! Lightweight simple-route candidate representation and validation helpers.
//!
//! This module intentionally does not run search or integrate into the
//! production routing flow yet. It only models and validates pre-defined
//! axis-aligned polyline candidates.

use rustc_hash::FxHashSet;

use crate::astar::State;
use crate::obstacle_map::{CellKey, ObstacleMap};

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

/// Axis-aligned candidate segment between two grid points.
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
}

/// Intended simple route topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum SimpleRouteKind {
    Straight,
    LShape,
    ZShape,
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

    let candidate = SimpleRouteCandidate::new(
        SimpleRouteKind::Straight,
        vec![source_point, target_point],
    );
    if check_simple_candidate(&candidate, obstacle_map, opened_cells) {
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
        if !check_simple_candidate(&candidate, obstacle_map, opened_cells) {
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
    if candidate.points.len() < 2 {
        return false;
    }
    if candidate.has_duplicate_consecutive_points() {
        return false;
    }
    if !candidate.is_axis_aligned() {
        return false;
    }

    for segment in candidate.segments() {
        let Some(seg_points) = expand_segment_points(segment) else {
            return false;
        };
        let seg_cells: Vec<(i32, i32)> = seg_points.into_iter().map(|p| (p.x, p.y)).collect();
        if !obstacle_map.check_cells_free(&seg_cells, opened_cells) {
            return false;
        }
    }

    true
}

fn expand_segment_points(segment: Segment) -> Option<Vec<GridPoint>> {
    let dx = segment.end.x - segment.start.x;
    let dy = segment.end.y - segment.start.y;

    if dx != 0 && dy != 0 {
        return None;
    }

    let (step_x, step_y, steps) = if dx != 0 {
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
    fn candidate_validation_accepts_clear_straight_route() {
        let map = ObstacleMap::new(10, 10);
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(5, 1)],
        );
        assert!(check_simple_candidate(&candidate, &map, None));
    }

    #[test]
    fn candidate_validation_rejects_diagonal_segment() {
        let map = ObstacleMap::new(10, 10);
        let candidate = SimpleRouteCandidate::new(
            SimpleRouteKind::Straight,
            vec![GridPoint::new(1, 1), GridPoint::new(2, 2)],
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
        let wrong_source = try_l_candidate(
            State::new(1, 1, 2),
            State::new(5, 4, 2),
            &map,
            None,
        );
        assert!(wrong_source.is_none());

        let wrong_target = try_l_candidate(
            State::new(1, 1, 0),
            State::new(5, 4, 0),
            &map,
            None,
        );
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
        assert!(try_straight_or_l_candidate(
            State::new(1, 1, 1),
            State::new(5, 1, 0),
            &map,
            None
        )
        .is_none());
        assert!(try_straight_or_l_candidate(
            State::new(1, 1, 0),
            State::new(5, 1, 7),
            &map,
            None
        )
        .is_none());
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
