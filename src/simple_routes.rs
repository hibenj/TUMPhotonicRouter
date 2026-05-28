//! Lightweight simple-route candidate representation and validation helpers.
//!
//! This module intentionally does not run search or integrate into the
//! production routing flow yet. It only models and validates pre-defined
//! axis-aligned polyline candidates.

use rustc_hash::FxHashSet;

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
    use crate::obstacle_map::pack_xy;

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
}
