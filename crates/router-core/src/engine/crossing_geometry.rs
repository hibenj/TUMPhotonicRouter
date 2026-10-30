use crate::primitives::DIRECTIONS;

pub(crate) fn point_to_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length_sq = dx * dx + dy * dy;
    let t = if length_sq <= f64::EPSILON {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / length_sq).clamp(0.0, 1.0)
    };
    let (cx, cy) = (a.0 + t * dx, a.1 + t * dy);
    ((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt()
}

pub(crate) fn segments_cross(
    p1: (f64, f64),
    p2: (f64, f64),
    q1: (f64, f64),
    q2: (f64, f64),
) -> bool {
    let orient = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    };
    let d1 = orient(q1, q2, p1);
    let d2 = orient(q1, q2, p2);
    let d3 = orient(p1, p2, q1);
    let d4 = orient(p1, p2, q2);
    ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))
}

pub(crate) fn segment_to_segment_distance(
    p1: (f64, f64),
    p2: (f64, f64),
    q1: (f64, f64),
    q2: (f64, f64),
) -> f64 {
    if segments_cross(p1, p2, q1, q2) {
        return 0.0;
    }
    point_to_segment_distance(p1, q1, q2)
        .min(point_to_segment_distance(p2, q1, q2))
        .min(point_to_segment_distance(q1, p1, p2))
        .min(point_to_segment_distance(q2, p1, p2))
}

pub(crate) fn segment_touches_region(
    a: (f64, f64),
    b: (f64, f64),
    region: (f64, f64, f64, f64),
) -> bool {
    let (min_x, min_y, max_x, max_y) = region;
    a.0.min(b.0) <= max_x && a.0.max(b.0) >= min_x && a.1.min(b.1) <= max_y && a.1.max(b.1) >= min_y
}

/// True when any segment of `a` inside `region` passes closer than
/// `threshold_um` to any segment of `b` inside `region`.
pub(crate) fn polylines_closer_than(
    a: &[(f64, f64)],
    b: &[(f64, f64)],
    threshold_um: f64,
    region: (f64, f64, f64, f64),
) -> bool {
    let b_segments: Vec<((f64, f64), (f64, f64))> = b
        .windows(2)
        .map(|w| (w[0], w[1]))
        .filter(|&(q1, q2)| segment_touches_region(q1, q2, region))
        .collect();
    if b_segments.is_empty() {
        return false;
    }
    a.windows(2)
        .map(|w| (w[0], w[1]))
        .filter(|&(p1, p2)| segment_touches_region(p1, p2, region))
        .any(|(p1, p2)| {
            b_segments
                .iter()
                .any(|&(q1, q2)| segment_to_segment_distance(p1, p2, q1, q2) < threshold_um)
        })
}

/// First point (segment midpoint of `a`) where a near-parallel segment pair of
/// `a` and `b` comes closer than `threshold_um`. Near-perpendicular pairs are
/// skipped so the chords around a legal crossing stay exempt; two parallel
/// centerlines closer than the waveguide width always overlap as polygons.
pub(crate) fn polylines_parallel_overlap_point(
    a: &[(f64, f64)],
    b: &[(f64, f64)],
    threshold_um: f64,
) -> Option<(f64, f64)> {
    for pw in a.windows(2) {
        let (p1, p2) = (pw[0], pw[1]);
        let p_len = physical_segment_length(p1, p2);
        if p_len <= 0.0 {
            continue;
        }
        let p_dir = ((p2.0 - p1.0) / p_len, (p2.1 - p1.1) / p_len);
        for qw in b.windows(2) {
            let (q1, q2) = (qw[0], qw[1]);
            let q_len = physical_segment_length(q1, q2);
            if q_len <= 0.0 {
                continue;
            }
            let cross_sin =
                (p_dir.0 * (q2.1 - q1.1) / q_len - p_dir.1 * (q2.0 - q1.0) / q_len).abs();
            if cross_sin >= 0.5 {
                continue;
            }
            if segment_to_segment_distance(p1, p2, q1, q2) < threshold_um {
                return Some(((p1.0 + p2.0) / 2.0, (p1.1 + p2.1) / 2.0));
            }
        }
    }
    None
}

pub(crate) fn polyline_bbox(points: &[(f64, f64)]) -> Option<(f64, f64, f64, f64)> {
    if points.is_empty() {
        return None;
    }
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for &(x, y) in points {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    Some((min_x, min_y, max_x, max_y))
}

pub(crate) fn direction_angle_between_cells(a: (i32, i32), b: (i32, i32)) -> Option<u8> {
    let step = ((b.0 - a.0).signum(), (b.1 - a.1).signum());
    DIRECTIONS
        .iter()
        .position(|dir| *dir == step)
        .map(|idx| idx as u8)
}

pub(crate) fn axes_are_perpendicular(angle_a: u8, angle_b: u8) -> bool {
    (i16::from(angle_a % 4) - i16::from(angle_b % 4)).rem_euclid(4) == 2
}

pub(crate) fn segment_length_cells(a: (i32, i32), b: (i32, i32)) -> f64 {
    f64::from((a.0 - b.0).abs().max((a.1 - b.1).abs()))
}

pub(crate) fn segment_intersection_with_params(
    a0: (i32, i32),
    a1: (i32, i32),
    b0: (i32, i32),
    b1: (i32, i32),
) -> Option<(f64, f64, f64, f64)> {
    let ax = f64::from(a1.0 - a0.0);
    let ay = f64::from(a1.1 - a0.1);
    let bx = f64::from(b1.0 - b0.0);
    let by = f64::from(b1.1 - b0.1);
    let denom = ax * by - ay * bx;
    if denom.abs() < 1e-9 {
        return None;
    }

    let cx = f64::from(b0.0 - a0.0);
    let cy = f64::from(b0.1 - a0.1);
    let t = (cx * by - cy * bx) / denom;
    let u = (cx * ay - cy * ax) / denom;
    let eps = 1e-9;
    if (-eps..=(1.0 + eps)).contains(&t) && (-eps..=(1.0 + eps)).contains(&u) {
        Some((f64::from(a0.0) + t * ax, f64::from(a0.1) + t * ay, t, u))
    } else {
        None
    }
}

pub(crate) fn physical_segment_length(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    (dx * dx + dy * dy).sqrt()
}

/// The orthogonal repair fallback is DISABLED BY DEFAULT since the
/// forced-90-degree plan
/// (`.agent/execplans/2026-09-01-forced-90-degree-route-degradation.md`):
/// it committed diagonal-free routes (46 90-degree bends, zero 45s, 20-31%
/// octile excess across multiportmmi_16x16's 14 repair-path nets) whose
/// axis-aligned material cascaded into later nets' searches; with it off,
/// the full ladder stays clean (repair-path excess 1.31 -> 1.07 and
/// 1.20 -> 1.03, owner visual sign-off) and multiportmmi_32x32 progresses
/// from failing at net 109 to net 156. The fallback itself, and the
/// `PHOTONIC_ROUTER_ENABLE_ORTHOGONAL_REPAIR_FALLBACK` switch that could
/// restore it, were deleted in Milestone 8 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`
/// (2026-09-24).
pub(crate) fn physical_point_near_centerline_endpoint(
    point: (f64, f64),
    centerline: &[(f64, f64)],
    tolerance_um: f64,
) -> bool {
    let Some(first) = centerline.first() else {
        return false;
    };
    if physical_segment_length(point, *first) <= tolerance_um {
        return true;
    }
    centerline
        .last()
        .map(|last| physical_segment_length(point, *last) <= tolerance_um)
        .unwrap_or(false)
}

pub(crate) fn physical_segments_are_perpendicular(
    a0: (f64, f64),
    a1: (f64, f64),
    b0: (f64, f64),
    b1: (f64, f64),
) -> bool {
    let ax = a1.0 - a0.0;
    let ay = a1.1 - a0.1;
    let bx = b1.0 - b0.0;
    let by = b1.1 - b0.1;
    (ax * bx + ay * by).abs() < 1e-6
}

pub(crate) fn physical_point_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let abx = b.0 - a.0;
    let aby = b.1 - a.1;
    let len_sq = abx * abx + aby * aby;
    if len_sq < 1e-18 {
        return ((p.0 - a.0).powi(2) + (p.1 - a.1).powi(2)).sqrt();
    }
    let t = (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / len_sq).clamp(0.0, 1.0);
    let cx = a.0 + t * abx;
    let cy = a.1 + t * aby;
    ((p.0 - cx).powi(2) + (p.1 - cy).powi(2)).sqrt()
}

/// Distance from `p` to the nearest segment of `polyline`, plus that
/// segment's direction in degrees folded to [0, 180). Diagnostic use only.
pub(crate) fn nearest_segment_info(polyline: &[(f64, f64)], p: (f64, f64)) -> (f64, f64) {
    let mut best = (f64::INFINITY, 0.0);
    for segment in polyline.windows(2) {
        let distance = physical_point_segment_distance(p, segment[0], segment[1]);
        if distance < best.0 {
            let angle = (segment[1].1 - segment[0].1)
                .atan2(segment[1].0 - segment[0].0)
                .to_degrees()
                .rem_euclid(180.0);
            best = (distance, angle);
        }
    }
    best
}

/// Distance from `p` to the nearest intersection of any segment pair of the
/// two polylines, considering only intersections within `radius_um` of `p`.
/// Diagnostic use only.
pub(crate) fn nearest_polyline_intersection_distance(
    a: &[(f64, f64)],
    b: &[(f64, f64)],
    p: (f64, f64),
    radius_um: f64,
) -> Option<f64> {
    let mut best: Option<f64> = None;
    for sa in a.windows(2) {
        if physical_point_segment_distance(p, sa[0], sa[1]) > radius_um {
            continue;
        }
        for sb in b.windows(2) {
            let Some((x, y, _t, _u)) =
                physical_segment_intersection_with_params(sa[0], sa[1], sb[0], sb[1])
            else {
                continue;
            };
            let distance = ((p.0 - x).powi(2) + (p.1 - y).powi(2)).sqrt();
            if distance <= radius_um && best.is_none_or(|b| distance < b) {
                best = Some(distance);
            }
        }
    }
    best
}

pub(crate) fn physical_segment_intersection_with_params(
    a0: (f64, f64),
    a1: (f64, f64),
    b0: (f64, f64),
    b1: (f64, f64),
) -> Option<(f64, f64, f64, f64)> {
    let ax = a1.0 - a0.0;
    let ay = a1.1 - a0.1;
    let bx = b1.0 - b0.0;
    let by = b1.1 - b0.1;
    let denom = ax * by - ay * bx;
    if denom.abs() < 1e-9 {
        return None;
    }

    let cx = b0.0 - a0.0;
    let cy = b0.1 - a0.1;
    let t = (cx * by - cy * bx) / denom;
    let u = (cx * ay - cy * ax) / denom;
    let eps = 1e-9;
    if (-eps..=(1.0 + eps)).contains(&t) && (-eps..=(1.0 + eps)).contains(&u) {
        Some((a0.0 + t * ax, a0.1 + t * ay, t, u))
    } else {
        None
    }
}

pub(crate) fn physical_collinear_segment_overlap_midpoint(
    a0: (f64, f64),
    a1: (f64, f64),
    b0: (f64, f64),
    b1: (f64, f64),
) -> Option<(f64, f64)> {
    let ax = a1.0 - a0.0;
    let ay = a1.1 - a0.1;
    let bx = b1.0 - b0.0;
    let by = b1.1 - b0.1;
    let eps = 1e-9;
    if ax.abs() < eps && ay.abs() < eps {
        return None;
    }
    if bx.abs() < eps && by.abs() < eps {
        return None;
    }
    if (ax * by - ay * bx).abs() >= eps {
        return None;
    }
    let offset_x = b0.0 - a0.0;
    let offset_y = b0.1 - a0.1;
    if (offset_x * ay - offset_y * ax).abs() >= eps {
        return None;
    }

    let use_x_axis = ax.abs() >= ay.abs();
    let a_denom = if use_x_axis { ax } else { ay };
    let b0_coord = if use_x_axis { b0.0 } else { b0.1 };
    let b1_coord = if use_x_axis { b1.0 } else { b1.1 };
    let a0_coord = if use_x_axis { a0.0 } else { a0.1 };
    if a_denom.abs() < eps {
        return None;
    }

    let b_t0 = (b0_coord - a0_coord) / a_denom;
    let b_t1 = (b1_coord - a0_coord) / a_denom;
    let t_start = 0.0_f64.max(b_t0.min(b_t1));
    let t_end = 1.0_f64.min(b_t0.max(b_t1));
    if t_end - t_start <= eps {
        return None;
    }

    let midpoint_t = 0.5 * (t_start + t_end);
    Some((a0.0 + midpoint_t * ax, a0.1 + midpoint_t * ay))
}

pub(crate) fn physical_points_are_collinear(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    let ab = (b.0 - a.0, b.1 - a.1);
    let bc = (c.0 - b.0, c.1 - b.1);
    let cross = ab.0 * bc.1 - ab.1 * bc.0;
    let dot = ab.0 * bc.0 + ab.1 * bc.1;
    cross.abs() < 1e-9 && dot >= -1e-9
}

pub(crate) fn compress_physical_centerline(points: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    if points.len() < 3 {
        return points;
    }
    let mut out = Vec::with_capacity(points.len());
    for point in points {
        if out.len() >= 2 {
            let prev = out[out.len() - 1];
            let prev_prev = out[out.len() - 2];
            if physical_points_are_collinear(prev_prev, prev, point) {
                out.pop();
            }
        }
        if out.last().copied() != Some(point) {
            out.push(point);
        }
    }
    out
}

/// Which straight run of a crossing candidate fell short of the rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarginSide {
    /// The route's straight run before the crossing point.
    RouteBefore,
    /// The route's pure straight run after the crossing point.
    RouteAfter,
    /// The partner's straight run on either side of the crossing point.
    Partner,
}

impl MarginSide {
    pub fn name(self) -> &'static str {
        match self {
            MarginSide::RouteBefore => "route_before",
            MarginSide::RouteAfter => "route_after",
            MarginSide::Partner => "partner",
        }
    }
}

/// Whose crossing element the candidate's window overlaps (phase 4 slice
/// B-2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrossingNeighbour {
    /// Another crossing of the route being judged (an earlier or later one
    /// along its path).
    SameRoute,
    /// A crossing already registered on the map (another net's
    /// `CrossingEvent`; the partner's crossings with third nets among them).
    Registered,
}

impl CrossingNeighbour {
    pub fn name(self) -> &'static str {
        match self {
            CrossingNeighbour::SameRoute => "same_route",
            CrossingNeighbour::Registered => "registered",
        }
    }
}

/// The one verdict of [`crossing_legality`]: legal, or the single reason the
/// candidate is not. The reasons are tested in the order listed and the
/// first failing one is the verdict, so a non-perpendicular candidate is
/// `NotPerpendicular` whatever its margins.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CrossingVerdict {
    Legal,
    NotPerpendicular,
    MarginShort {
        side: MarginSide,
        have_cells: f64,
        need_cells: f64,
    },
    /// The candidate's reservation window shares a cell with another
    /// crossing element's (`other`); `have_cells` is the Chebyshev distance
    /// of the two centres in cells, `need_cells` the rule's
    /// `required_spacing_cells`.
    TooClose {
        other: CrossingNeighbour,
        have_cells: f64,
        need_cells: f64,
    },
    FootprintBlocked,
    MissingEvent,
}

impl CrossingVerdict {
    pub fn is_legal(&self) -> bool {
        matches!(self, CrossingVerdict::Legal)
    }

    /// The reason as the diagnostics, the commit errors and the Python
    /// verifier spell it: `legal`, `not_perpendicular`,
    /// `margin_short_<side>`, `too_close_<other>`, `footprint_blocked`,
    /// `missing_event`.
    pub fn reason(&self) -> &'static str {
        match self {
            CrossingVerdict::Legal => "legal",
            CrossingVerdict::NotPerpendicular => "not_perpendicular",
            CrossingVerdict::MarginShort {
                side: MarginSide::RouteBefore,
                ..
            } => "margin_short_route_before",
            CrossingVerdict::MarginShort {
                side: MarginSide::RouteAfter,
                ..
            } => "margin_short_route_after",
            CrossingVerdict::MarginShort {
                side: MarginSide::Partner,
                ..
            } => "margin_short_partner",
            CrossingVerdict::TooClose {
                other: CrossingNeighbour::SameRoute,
                ..
            } => "too_close_same_route",
            CrossingVerdict::TooClose {
                other: CrossingNeighbour::Registered,
                ..
            } => "too_close_registered",
            CrossingVerdict::FootprintBlocked => "footprint_blocked",
            CrossingVerdict::MissingEvent => "missing_event",
        }
    }

    pub fn margin_side(&self) -> Option<MarginSide> {
        match self {
            CrossingVerdict::MarginShort { side, .. } => Some(*side),
            _ => None,
        }
    }

    pub fn too_close_other(&self) -> Option<CrossingNeighbour> {
        match self {
            CrossingVerdict::TooClose { other, .. } => Some(*other),
            _ => None,
        }
    }
}

/// The margin rule of a crossing, in grid cells of the GRID route (the
/// compressed waypoints, on which a bend contributes its arms as straight
/// cells). `required_margin_cells` = `half_size + bend_runout` is the one
/// margin: the crossing element's half footprint plus the run-out that pays
/// for a bend arm counted as straight. It applies to the route's run
/// before the crossing and to both sides of the partner. After the crossing
/// the route owes `half_size` cells of PURE straight (the element's own
/// footprint; the search pays this debt with straight primitives only, so a
/// bend may follow it directly -- Point 2 of 2026-09-03, which found that
/// demanding `required_margin_cells` after the crossing discards
/// kernel-legal, realized-clean routes).
///
/// Spacing (phase 4 slice B-2): two crossing elements never share a grid
/// cell. Each crossing owns the reservation window of every cell within
/// `half_size` of its centre on both axes (`reservation_window_span`, the
/// window `crossing_reservation_window_keys` builds); the windows of two
/// crossings must be disjoint, which for centres on grid points is a centre
/// distance of `required_spacing_cells` = `2 * half_size + 1` cells on at
/// least one axis. This is the rule the legacy search paid through its
/// reservation bookkeeping (`reservation_overlap`, the kernel's chain check,
/// the pending straight debt) and the commit through
/// `crossing_events_have_disjoint_reservations` and the static reservation
/// cells of committed crossings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrossingLegalityRule {
    pub half_size_cells: i32,
    pub bend_runout_cells: i32,
}

impl CrossingLegalityRule {
    pub fn required_margin_cells(&self) -> i32 {
        self.half_size_cells.max(0) + self.bend_runout_cells.max(0)
    }

    pub fn required_after_cells(&self) -> i32 {
        self.half_size_cells.max(0)
    }

    /// The centre distance (Chebyshev, in cells) at which two reservation
    /// windows with grid-point centres become disjoint: `2 * half_size + 1`.
    pub fn required_spacing_cells(&self) -> i32 {
        2 * self.half_size_cells.max(0) + 1
    }
}

/// The cells `[floor(c - half), ceil(c + half)]` a crossing centred at `c`
/// reserves on one axis -- the span `crossing_reservation_window_keys` and
/// `local_crossing_reservation_window_keys` build (one cell wider on an
/// axis where the centre sits on a cell boundary).
pub(crate) fn reservation_window_span(centre: f64, half_size_cells: i32) -> (i32, i32) {
    let half = f64::from(half_size_cells.max(0));
    (
        (centre - half).floor() as i32,
        (centre + half).ceil() as i32,
    )
}

/// Whether the reservation windows of two crossings centred at `a` and `b`
/// (grid coordinates) share a cell.
pub(crate) fn crossing_windows_overlap(a: (f64, f64), b: (f64, f64), half_size_cells: i32) -> bool {
    let (a_x0, a_x1) = reservation_window_span(a.0, half_size_cells);
    let (b_x0, b_x1) = reservation_window_span(b.0, half_size_cells);
    if a_x1 < b_x0 || b_x1 < a_x0 {
        return false;
    }
    let (a_y0, a_y1) = reservation_window_span(a.1, half_size_cells);
    let (b_y0, b_y1) = reservation_window_span(b.1, half_size_cells);
    !(a_y1 < b_y0 || b_y1 < a_y0)
}

/// The Chebyshev distance of two centres in cells (the metric of the square
/// reservation windows).
pub(crate) fn chebyshev_cells(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).abs().max((a.1 - b.1).abs())
}

/// The neighbour a candidate at `point` is judged against: the nearest
/// (Chebyshev) of `others` whose window overlaps the candidate's, or, when
/// none does, the nearest overall (so the record still names the closest
/// element). A centre within 1e-6 cells of `point` is the candidate itself
/// and is skipped. The overlapping ones come first because the overlap
/// threshold of an axis depends on the centres' fractional parts: among two
/// neighbours at the same Chebyshev distance one may share a cell and the
/// other not.
pub(crate) fn nearest_crossing_neighbour(
    point: (f64, f64),
    half_size_cells: i32,
    others: impl IntoIterator<Item = (f64, f64)>,
) -> Option<(f64, f64)> {
    let mut best: Option<(bool, f64, (f64, f64))> = None;
    for other in others {
        let distance = chebyshev_cells(point, other);
        if distance <= 1e-6 {
            continue;
        }
        let overlaps = crossing_windows_overlap(point, other, half_size_cells);
        let better = match best {
            None => true,
            Some((best_overlaps, best_distance, _)) => {
                (overlaps && !best_overlaps)
                    || (overlaps == best_overlaps && distance < best_distance)
            }
        };
        if better {
            best = Some((overlaps, distance, other));
        }
    }
    best.map(|(_, _, other)| other)
}

/// One crossing as the predicate sees it: the two directions (any scale,
/// grid-angle unit vectors or physical segment vectors), the four straight
/// runs in grid cells, the centre in grid coordinates with the nearest
/// other crossing of the same route and the nearest crossing registered on
/// the map (`nearest_crossing_neighbour`; `None` when there is none),
/// whether the crossing element's footprint cells carry an unrelated net,
/// and whether a `CrossingEvent` is registered for it (`None` before
/// commit, when no event can exist yet).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrossingCandidate {
    pub route_direction: (f64, f64),
    pub partner_direction: (f64, f64),
    pub route_before_cells: f64,
    pub route_after_cells: f64,
    pub partner_before_cells: f64,
    pub partner_after_cells: f64,
    pub point: (f64, f64),
    pub nearest_same_route: Option<(f64, f64)>,
    pub nearest_registered: Option<(f64, f64)>,
    pub footprint_blocked: bool,
    pub event_registered: Option<bool>,
}

impl CrossingCandidate {
    /// Chebyshev distance in cells to the nearest other crossing of the
    /// same route, `None` when there is none.
    pub fn nearest_same_route_cells(&self) -> Option<f64> {
        self.nearest_same_route
            .map(|other| chebyshev_cells(self.point, other))
    }

    /// Chebyshev distance in cells to the nearest registered crossing,
    /// `None` when there is none.
    pub fn nearest_registered_cells(&self) -> Option<f64> {
        self.nearest_registered
            .map(|other| chebyshev_cells(self.point, other))
    }
}

/// The one perpendicularity test: the normalised dot product of the two
/// directions is at most 1e-6 in magnitude. Grid angles map onto it through
/// [`grid_angle_direction`]; a zero-length direction is never perpendicular.
pub fn directions_are_perpendicular(a: (f64, f64), b: (f64, f64)) -> bool {
    let len_a = (a.0 * a.0 + a.1 * a.1).sqrt();
    let len_b = (b.0 * b.0 + b.1 * b.1).sqrt();
    if len_a <= 1e-9 || len_b <= 1e-9 {
        return false;
    }
    ((a.0 * b.0 + a.1 * b.1) / (len_a * len_b)).abs() <= 1e-6
}

/// The unit step of a grid direction angle (0..8, see `DIRECTIONS`).
pub(crate) fn grid_angle_direction(angle: u8) -> (f64, f64) {
    let (dx, dy) = DIRECTIONS[usize::from(angle % 8)];
    (f64::from(dx), f64::from(dy))
}

/// THE crossing-legality predicate (do-over phase 4 slice B, behind
/// `RouterConfig::crossing.unified_legality`). The search hook, the commit
/// check and the post-commit validation all call it with the same rule, so
/// the search never produces a crossing the commit rejects. Tests, in
/// order: perpendicular; route before >= `required_margin_cells`; route
/// after >= `required_after_cells`; partner on both sides >=
/// `required_margin_cells`; reservation window disjoint from the nearest
/// same-route crossing's and from the nearest registered crossing's
/// (`TooClose`, slice B-2); footprint not blocked; event registered when
/// the caller knows (`Some(false)` is `MissingEvent`, `None` skips the
/// test). Margins compare with a 1e-9 tolerance, as every legacy path did.
pub(crate) fn crossing_legality(
    candidate: &CrossingCandidate,
    rule: &CrossingLegalityRule,
) -> CrossingVerdict {
    if !directions_are_perpendicular(candidate.route_direction, candidate.partner_direction) {
        return CrossingVerdict::NotPerpendicular;
    }
    let need = f64::from(rule.required_margin_cells());
    let need_after = f64::from(rule.required_after_cells());
    let eps = 1e-9;
    if candidate.route_before_cells + eps < need {
        return CrossingVerdict::MarginShort {
            side: MarginSide::RouteBefore,
            have_cells: candidate.route_before_cells,
            need_cells: need,
        };
    }
    if candidate.route_after_cells + eps < need_after {
        return CrossingVerdict::MarginShort {
            side: MarginSide::RouteAfter,
            have_cells: candidate.route_after_cells,
            need_cells: need_after,
        };
    }
    let partner_margin = candidate
        .partner_before_cells
        .min(candidate.partner_after_cells);
    if partner_margin + eps < need {
        return CrossingVerdict::MarginShort {
            side: MarginSide::Partner,
            have_cells: partner_margin,
            need_cells: need,
        };
    }
    let need_spacing = f64::from(rule.required_spacing_cells());
    for (other, neighbour) in [
        (CrossingNeighbour::SameRoute, candidate.nearest_same_route),
        (CrossingNeighbour::Registered, candidate.nearest_registered),
    ] {
        if let Some(neighbour) = neighbour {
            if crossing_windows_overlap(candidate.point, neighbour, rule.half_size_cells) {
                return CrossingVerdict::TooClose {
                    other,
                    have_cells: chebyshev_cells(candidate.point, neighbour),
                    need_cells: need_spacing,
                };
            }
        }
    }
    if candidate.footprint_blocked {
        return CrossingVerdict::FootprintBlocked;
    }
    if candidate.event_registered == Some(false) {
        return CrossingVerdict::MissingEvent;
    }
    CrossingVerdict::Legal
}

/// Unit tests for the pure geometry helpers, Milestone 6 Slice 3 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
/// Every number below is hand-computed on a right-angle or axis-aligned
/// configuration, so the expectations can be read without running the code.
#[cfg(test)]
mod tests {
    use super::*;

    /// Tolerance for the exact-arithmetic cases below (all of them land on
    /// halves or integers).
    const EPS: f64 = 1e-12;

    #[test]
    fn segment_intersection_with_params_returns_the_crossing_point_and_both_parameters() {
        // a: (0,0) -> (4,0), b: (2,-2) -> (2,2). They meet at (2,0), which
        // is halfway along both, so t = u = 0.5.
        let hit = segment_intersection_with_params((0, 0), (4, 0), (2, -2), (2, 2))
            .expect("the segments cross");
        let (x, y, t, u) = hit;
        assert!((x - 2.0).abs() < EPS, "x was {x}");
        assert!(y.abs() < EPS, "y was {y}");
        assert!((t - 0.5).abs() < EPS, "t was {t}");
        assert!((u - 0.5).abs() < EPS, "u was {u}");
    }

    #[test]
    fn segment_intersection_with_params_rejects_parallel_segments() {
        // Both horizontal: the determinant 4*0 - 0*4 is zero.
        assert!(segment_intersection_with_params((0, 0), (4, 0), (0, 1), (4, 1)).is_none());
        // Collinear is the same zero determinant, not an infinity of hits.
        assert!(segment_intersection_with_params((0, 0), (4, 0), (1, 0), (2, 0)).is_none());
    }

    #[test]
    fn segment_intersection_with_params_accepts_a_touch_at_an_endpoint() {
        // b starts exactly at a's end point (4,0): t = 1 at the end of a,
        // u = 0 at the start of b. Both are inside the inclusive [0,1]
        // parameter window, so the touch counts as an intersection.
        let (x, y, t, u) = segment_intersection_with_params((0, 0), (4, 0), (4, 0), (4, 4))
            .expect("an endpoint touch is an intersection");
        assert!(
            (x - 4.0).abs() < EPS && y.abs() < EPS,
            "point was ({x},{y})"
        );
        assert!((t - 1.0).abs() < EPS, "t was {t}");
        assert!(u.abs() < EPS, "u was {u}");
    }

    #[test]
    fn segment_intersection_with_params_rejects_an_intersection_past_the_segment_ends() {
        // The infinite lines meet at (4,0), but that is four times a's
        // length away, so t = 4 falls outside [0,1].
        assert!(segment_intersection_with_params((0, 0), (1, 0), (4, -2), (4, 2)).is_none());
    }

    #[test]
    fn point_to_segment_distance_projects_inside_the_segment() {
        // (2,3) projects onto the middle of (0,0)->(4,0): distance 3.
        let d = point_to_segment_distance((2.0, 3.0), (0.0, 0.0), (4.0, 0.0));
        assert!((d - 3.0).abs() < EPS, "distance was {d}");
    }

    #[test]
    fn point_to_segment_distance_clamps_to_the_nearer_endpoint() {
        // (-2,0) projects to t = -0.5, clamped to the start point (0,0):
        // distance 2. (6,0) clamps to the end point (4,0): distance 2.
        let before = point_to_segment_distance((-2.0, 0.0), (0.0, 0.0), (4.0, 0.0));
        let after = point_to_segment_distance((6.0, 0.0), (0.0, 0.0), (4.0, 0.0));
        assert!((before - 2.0).abs() < EPS, "before was {before}");
        assert!((after - 2.0).abs() < EPS, "after was {after}");
    }

    #[test]
    fn point_to_segment_distance_on_a_degenerate_segment_is_the_point_distance() {
        let d = point_to_segment_distance((1.0, 4.0), (1.0, 1.0), (1.0, 1.0));
        assert!((d - 3.0).abs() < EPS, "distance was {d}");
    }

    #[test]
    fn segment_to_segment_distance_is_zero_for_crossing_segments() {
        let d = segment_to_segment_distance((0.0, 0.0), (4.0, 0.0), (2.0, -2.0), (2.0, 2.0));
        assert_eq!(d, 0.0);
    }

    #[test]
    fn segment_to_segment_distance_of_parallel_segments_is_their_offset() {
        let d = segment_to_segment_distance((0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0));
        assert!((d - 3.0).abs() < EPS, "distance was {d}");
    }

    #[test]
    fn segment_to_segment_distance_of_collinear_disjoint_segments_is_the_gap() {
        // (0,0)->(1,0) and (4,0)->(5,0): the gap between (1,0) and (4,0)
        // is 3, which is the smallest of the four endpoint-to-segment
        // distances (4, 3, 3, 4).
        let d = segment_to_segment_distance((0.0, 0.0), (1.0, 0.0), (4.0, 0.0), (5.0, 0.0));
        assert!((d - 3.0).abs() < EPS, "distance was {d}");
    }

    #[test]
    fn polylines_parallel_overlap_point_reports_the_midpoint_of_the_offending_segment() {
        // Two horizontal centerlines 0.5 um apart, threshold 1.0: the
        // first (and only) segment pair is parallel (cross_sin = 0) and
        // 0.5 apart, so the midpoint of a's segment, (5,0), is reported.
        let a = [(0.0, 0.0), (10.0, 0.0)];
        let b = [(0.0, 0.5), (10.0, 0.5)];
        let point = polylines_parallel_overlap_point(&a, &b, 1.0).expect("an overlap");
        assert!(
            (point.0 - 5.0).abs() < EPS && point.1.abs() < EPS,
            "point was {point:?}"
        );
    }

    #[test]
    fn polylines_parallel_overlap_point_skips_perpendicular_pairs() {
        // A legal crossing: the pair is perpendicular (cross_sin = 1.0,
        // above the 0.5 gate), so it is exempt even though the segments
        // touch.
        let a = [(0.0, 0.0), (10.0, 0.0)];
        let b = [(5.0, -5.0), (5.0, 5.0)];
        assert!(polylines_parallel_overlap_point(&a, &b, 1.0).is_none());
    }

    #[test]
    fn polylines_parallel_overlap_point_ignores_parallel_pairs_beyond_the_threshold() {
        let a = [(0.0, 0.0), (10.0, 0.0)];
        let b = [(0.0, 5.0), (10.0, 5.0)];
        assert!(polylines_parallel_overlap_point(&a, &b, 1.0).is_none());
    }

    #[test]
    fn physical_points_are_collinear_accepts_a_straight_run() {
        assert!(physical_points_are_collinear(
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0)
        ));
        assert!(physical_points_are_collinear(
            (0.0, 0.0),
            (1.0, 1.0),
            (3.0, 3.0)
        ));
    }

    #[test]
    fn physical_points_are_collinear_rejects_a_bend() {
        // cross = 1*1 - 0*1 = 1, well above the 1e-9 tolerance.
        assert!(!physical_points_are_collinear(
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0)
        ));
    }

    #[test]
    fn physical_points_are_collinear_rejects_a_reversal() {
        // Collinear (cross = 0) but doubling back: the dot product is
        // -1, so the non-negative-dot guard rejects it.
        assert!(!physical_points_are_collinear(
            (0.0, 0.0),
            (1.0, 0.0),
            (0.0, 0.0)
        ));
    }

    #[test]
    fn compress_physical_centerline_removes_collinear_middle_points() {
        assert_eq!(
            compress_physical_centerline(vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]),
            vec![(0.0, 0.0), (3.0, 0.0)]
        );
    }

    #[test]
    fn compress_physical_centerline_keeps_a_bend() {
        // The straight run collapses to (0,0)->(2,0); (2,1) turns, so it
        // is kept and the corner survives.
        assert_eq!(
            compress_physical_centerline(vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0)]),
            vec![(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)]
        );
    }

    #[test]
    fn compress_physical_centerline_drops_repeated_points_and_passes_short_inputs_through() {
        assert_eq!(
            compress_physical_centerline(vec![(0.0, 0.0), (0.0, 0.0), (1.0, 1.0)]),
            vec![(0.0, 0.0), (1.0, 1.0)]
        );
        // Fewer than three points is returned untouched, duplicates included.
        assert_eq!(
            compress_physical_centerline(vec![(0.0, 0.0), (0.0, 0.0)]),
            vec![(0.0, 0.0), (0.0, 0.0)]
        );
    }

    /// A candidate that passes every test of `crossing_legality` under
    /// `RULE` (half 2, run-out 3: margin 5 before and on the partner, 2
    /// after): horizontal route, vertical partner, generous straights,
    /// clear footprint, event registered.
    const RULE: CrossingLegalityRule = CrossingLegalityRule {
        half_size_cells: 2,
        bend_runout_cells: 3,
    };

    fn legal_candidate() -> CrossingCandidate {
        CrossingCandidate {
            route_direction: (1.0, 0.0),
            partner_direction: (0.0, 1.0),
            route_before_cells: 8.0,
            route_after_cells: 8.0,
            partner_before_cells: 8.0,
            partner_after_cells: 8.0,
            point: (20.0, 20.0),
            nearest_same_route: None,
            nearest_registered: None,
            footprint_blocked: false,
            event_registered: Some(true),
        }
    }

    #[test]
    fn crossing_legality_rule_has_one_margin_and_the_half_footprint_after() {
        assert_eq!(RULE.required_margin_cells(), 5);
        assert_eq!(RULE.required_after_cells(), 2);
        // Two 5x5 windows (half 2) are disjoint from 5 cells apart.
        assert_eq!(RULE.required_spacing_cells(), 5);
        // Negative inputs clamp to zero, as the legacy
        // `crossing_required_margin_cells` did.
        let clamped = CrossingLegalityRule {
            half_size_cells: -1,
            bend_runout_cells: -4,
        };
        assert_eq!(clamped.required_margin_cells(), 0);
        assert_eq!(clamped.required_after_cells(), 0);
        assert_eq!(clamped.required_spacing_cells(), 1);
    }

    #[test]
    fn crossing_legality_accepts_a_perpendicular_crossing_with_margins_and_an_event() {
        assert_eq!(
            crossing_legality(&legal_candidate(), &RULE),
            CrossingVerdict::Legal
        );
        // The same crossing before commit (no event can exist yet).
        let pre_commit = CrossingCandidate {
            event_registered: None,
            ..legal_candidate()
        };
        assert!(crossing_legality(&pre_commit, &RULE).is_legal());
    }

    #[test]
    fn crossing_legality_rejects_a_non_perpendicular_pair_before_anything_else() {
        // A diagonal against a horizontal: 45 degrees, not 90. The margins
        // are also short, but the perpendicularity test comes first.
        let candidate = CrossingCandidate {
            partner_direction: (1.0, 1.0),
            route_before_cells: 0.0,
            ..legal_candidate()
        };
        let verdict = crossing_legality(&candidate, &RULE);
        assert_eq!(verdict, CrossingVerdict::NotPerpendicular);
        assert_eq!(verdict.reason(), "not_perpendicular");
        // The grid angles map onto the same test: angle 1 (1,1) against
        // angle 3 (-1,1) is perpendicular, 1 against 2 (0,1) is not.
        assert!(directions_are_perpendicular(
            grid_angle_direction(1),
            grid_angle_direction(3)
        ));
        assert!(!directions_are_perpendicular(
            grid_angle_direction(1),
            grid_angle_direction(2)
        ));
        // Opposite directions of the same axis are perpendicular too
        // (angle 0 against 6 as well as against 2).
        assert!(directions_are_perpendicular(
            grid_angle_direction(0),
            grid_angle_direction(6)
        ));
        // A zero-length direction is never perpendicular.
        assert!(!directions_are_perpendicular((0.0, 0.0), (0.0, 1.0)));
    }

    #[test]
    fn crossing_legality_accepts_a_margin_exactly_at_the_limit_and_rejects_one_cell_less() {
        let at_limit = CrossingCandidate {
            route_before_cells: 5.0,
            route_after_cells: 2.0,
            partner_before_cells: 5.0,
            partner_after_cells: 5.0,
            ..legal_candidate()
        };
        assert!(crossing_legality(&at_limit, &RULE).is_legal());
        // Floating-point noise below the limit still passes (1e-9 tolerance).
        let noisy = CrossingCandidate {
            route_before_cells: 5.0 - 1e-12,
            ..at_limit
        };
        assert!(crossing_legality(&noisy, &RULE).is_legal());

        let before_short = CrossingCandidate {
            route_before_cells: 4.0,
            ..at_limit
        };
        assert_eq!(
            crossing_legality(&before_short, &RULE),
            CrossingVerdict::MarginShort {
                side: MarginSide::RouteBefore,
                have_cells: 4.0,
                need_cells: 5.0,
            }
        );
        let after_short = CrossingCandidate {
            route_after_cells: 1.0,
            ..at_limit
        };
        let verdict = crossing_legality(&after_short, &RULE);
        assert_eq!(verdict.margin_side(), Some(MarginSide::RouteAfter));
        assert_eq!(verdict.reason(), "margin_short_route_after");
        // The partner side reports the smaller of its two runs.
        let partner_short = CrossingCandidate {
            partner_after_cells: 4.0,
            ..at_limit
        };
        assert_eq!(
            crossing_legality(&partner_short, &RULE),
            CrossingVerdict::MarginShort {
                side: MarginSide::Partner,
                have_cells: 4.0,
                need_cells: 5.0,
            }
        );
    }

    #[test]
    fn crossing_legality_rejects_a_blocked_footprint_and_a_missing_event() {
        let blocked = CrossingCandidate {
            footprint_blocked: true,
            ..legal_candidate()
        };
        assert_eq!(
            crossing_legality(&blocked, &RULE),
            CrossingVerdict::FootprintBlocked
        );
        let unregistered = CrossingCandidate {
            event_registered: Some(false),
            ..legal_candidate()
        };
        let verdict = crossing_legality(&unregistered, &RULE);
        assert_eq!(verdict, CrossingVerdict::MissingEvent);
        assert_eq!(verdict.reason(), "missing_event");
        assert!(!verdict.is_legal());
        // A blocked footprint outranks the missing event.
        let both = CrossingCandidate {
            footprint_blocked: true,
            event_registered: Some(false),
            ..legal_candidate()
        };
        assert_eq!(
            crossing_legality(&both, &RULE),
            CrossingVerdict::FootprintBlocked
        );
    }

    /// Slice B-2: two crossings of one route exactly `2 * half + 1` = 5
    /// cells apart own disjoint 5x5 windows ([18, 22] and [23, 27]) and
    /// pass; one cell closer the windows share the column x = 22 and the
    /// verdict is `TooClose { SameRoute }` with the centre distance.
    #[test]
    fn crossing_legality_accepts_spacing_at_the_limit_and_rejects_one_cell_less() {
        let at_limit = CrossingCandidate {
            nearest_same_route: Some((25.0, 20.0)),
            ..legal_candidate()
        };
        assert!(crossing_legality(&at_limit, &RULE).is_legal());
        assert_eq!(at_limit.nearest_same_route_cells(), Some(5.0));

        let one_closer = CrossingCandidate {
            nearest_same_route: Some((24.0, 20.0)),
            ..legal_candidate()
        };
        let verdict = crossing_legality(&one_closer, &RULE);
        assert_eq!(
            verdict,
            CrossingVerdict::TooClose {
                other: CrossingNeighbour::SameRoute,
                have_cells: 4.0,
                need_cells: 5.0,
            }
        );
        assert_eq!(verdict.reason(), "too_close_same_route");
        assert_eq!(
            verdict.too_close_other(),
            Some(CrossingNeighbour::SameRoute)
        );
        assert_eq!(verdict.margin_side(), None);
        // The windows are squares: a diagonal neighbour four cells away on
        // both axes shares the cell (22, 22) as well.
        let diagonal = CrossingCandidate {
            nearest_same_route: Some((24.0, 24.0)),
            ..legal_candidate()
        };
        assert_eq!(
            crossing_legality(&diagonal, &RULE).reason(),
            "too_close_same_route"
        );
        // Five cells away on one axis is enough whatever the other axis.
        let offset = CrossingCandidate {
            nearest_same_route: Some((25.0, 23.0)),
            ..legal_candidate()
        };
        assert!(crossing_legality(&offset, &RULE).is_legal());
    }

    /// Slice B-2: another net's registered crossing element inside the
    /// candidate's window is `TooClose { Registered }`; it is tested after
    /// the same-route neighbour and before the footprint.
    #[test]
    fn crossing_legality_rejects_a_registered_crossing_component_in_the_window() {
        let registered = CrossingCandidate {
            nearest_registered: Some((20.0, 17.0)),
            footprint_blocked: true,
            ..legal_candidate()
        };
        let verdict = crossing_legality(&registered, &RULE);
        assert_eq!(
            verdict,
            CrossingVerdict::TooClose {
                other: CrossingNeighbour::Registered,
                have_cells: 3.0,
                need_cells: 5.0,
            }
        );
        assert_eq!(verdict.reason(), "too_close_registered");
        assert_eq!(registered.nearest_registered_cells(), Some(3.0));
        let both = CrossingCandidate {
            nearest_same_route: Some((16.0, 20.0)),
            ..registered
        };
        assert_eq!(
            crossing_legality(&both, &RULE).too_close_other(),
            Some(CrossingNeighbour::SameRoute)
        );
        // A registered element five cells away is clear; the footprint
        // reason then follows.
        let clear = CrossingCandidate {
            nearest_registered: Some((20.0, 15.0)),
            ..registered
        };
        assert_eq!(
            crossing_legality(&clear, &RULE),
            CrossingVerdict::FootprintBlocked
        );
    }

    /// The window is cell-exact, as `crossing_reservation_window_keys`
    /// builds it: a centre on a cell boundary (a diagonal x diagonal
    /// crossing at a half cell) widens the window by one cell on that
    /// axis, so two such centres five cells apart still share a cell
    /// ([18, 23] and [23, 28]) while a grid-point centre five cells from
    /// a half-cell one does not ([18, 22] and [23, 28]).
    #[test]
    fn crossing_windows_overlap_follows_the_floor_ceil_window() {
        assert_eq!(reservation_window_span(20.0, 2), (18, 22));
        assert_eq!(reservation_window_span(20.5, 2), (18, 23));
        assert!(!crossing_windows_overlap((20.0, 20.0), (25.0, 20.0), 2));
        assert!(crossing_windows_overlap((20.0, 20.0), (24.0, 20.0), 2));
        assert!(crossing_windows_overlap((20.5, 20.0), (25.5, 20.0), 2));
        assert!(!crossing_windows_overlap((20.0, 20.0), (25.5, 20.0), 2));
        assert!(!crossing_windows_overlap((20.5, 20.0), (26.5, 20.0), 2));
        let half_cell = CrossingCandidate {
            point: (20.5, 20.0),
            nearest_same_route: Some((25.5, 20.0)),
            ..legal_candidate()
        };
        assert_eq!(
            crossing_legality(&half_cell, &RULE),
            CrossingVerdict::TooClose {
                other: CrossingNeighbour::SameRoute,
                have_cells: 5.0,
                need_cells: 5.0,
            }
        );
    }

    /// The neighbour the callers hand the predicate: the nearest
    /// overlapping one first (at equal distance a neighbour whose window
    /// shares a cell outranks one whose window does not), the candidate's
    /// own centre skipped, the nearest overall when nothing overlaps.
    #[test]
    fn nearest_crossing_neighbour_prefers_an_overlapping_window() {
        let point = (20.5, 20.0);
        // (25.5, 25.0): x half-half shares x = 23, y int-int at distance 5
        // is disjoint -> windows disjoint. (25.5, 20.0): overlaps.
        let others = [point, (25.5, 25.0), (25.5, 20.0), (40.0, 40.0)];
        assert_eq!(
            nearest_crossing_neighbour(point, 2, others),
            Some((25.5, 20.0))
        );
        assert_eq!(
            nearest_crossing_neighbour(point, 2, [point, (30.0, 20.0), (27.0, 20.0)]),
            Some((27.0, 20.0))
        );
        assert_eq!(nearest_crossing_neighbour(point, 2, [point]), None);
        assert_eq!(chebyshev_cells((1.0, 2.0), (4.0, -3.0)), 5.0);
    }
}
