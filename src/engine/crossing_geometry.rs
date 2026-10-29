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
pub(crate) enum MarginSide {
    /// The route's straight run before the crossing point.
    RouteBefore,
    /// The route's pure straight run after the crossing point.
    RouteAfter,
    /// The partner's straight run on either side of the crossing point.
    Partner,
}

impl MarginSide {
    pub(crate) fn name(self) -> &'static str {
        match self {
            MarginSide::RouteBefore => "route_before",
            MarginSide::RouteAfter => "route_after",
            MarginSide::Partner => "partner",
        }
    }
}

/// The one verdict of [`crossing_legality`]: legal, or the single reason the
/// candidate is not. The reasons are tested in the order listed and the
/// first failing one is the verdict, so a non-perpendicular candidate is
/// `NotPerpendicular` whatever its margins.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum CrossingVerdict {
    Legal,
    NotPerpendicular,
    MarginShort {
        side: MarginSide,
        have_cells: f64,
        need_cells: f64,
    },
    FootprintBlocked,
    MissingEvent,
}

impl CrossingVerdict {
    pub(crate) fn is_legal(&self) -> bool {
        matches!(self, CrossingVerdict::Legal)
    }

    /// The reason as the diagnostics, the commit errors and the Python
    /// verifier spell it: `legal`, `not_perpendicular`,
    /// `margin_short_<side>`, `footprint_blocked`, `missing_event`.
    pub(crate) fn reason(&self) -> &'static str {
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
            CrossingVerdict::FootprintBlocked => "footprint_blocked",
            CrossingVerdict::MissingEvent => "missing_event",
        }
    }

    pub(crate) fn margin_side(&self) -> Option<MarginSide> {
        match self {
            CrossingVerdict::MarginShort { side, .. } => Some(*side),
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct CrossingLegalityRule {
    pub(crate) half_size_cells: i32,
    pub(crate) bend_runout_cells: i32,
}

impl CrossingLegalityRule {
    pub(crate) fn required_margin_cells(&self) -> i32 {
        self.half_size_cells.max(0) + self.bend_runout_cells.max(0)
    }

    pub(crate) fn required_after_cells(&self) -> i32 {
        self.half_size_cells.max(0)
    }
}

/// One crossing as the predicate sees it: the two directions (any scale,
/// grid-angle unit vectors or physical segment vectors), the four straight
/// runs in grid cells, whether the crossing element's footprint cells carry
/// an unrelated net, and whether a `CrossingEvent` is registered for it
/// (`None` before commit, when no event can exist yet).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CrossingCandidate {
    pub(crate) route_direction: (f64, f64),
    pub(crate) partner_direction: (f64, f64),
    pub(crate) route_before_cells: f64,
    pub(crate) route_after_cells: f64,
    pub(crate) partner_before_cells: f64,
    pub(crate) partner_after_cells: f64,
    pub(crate) footprint_blocked: bool,
    pub(crate) event_registered: Option<bool>,
}

/// The one perpendicularity test: the normalised dot product of the two
/// directions is at most 1e-6 in magnitude. Grid angles map onto it through
/// [`grid_angle_direction`]; a zero-length direction is never perpendicular.
pub(crate) fn directions_are_perpendicular(a: (f64, f64), b: (f64, f64)) -> bool {
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
/// `required_margin_cells`; footprint not blocked; event registered when
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
            footprint_blocked: false,
            event_registered: Some(true),
        }
    }

    #[test]
    fn crossing_legality_rule_has_one_margin_and_the_half_footprint_after() {
        assert_eq!(RULE.required_margin_cells(), 5);
        assert_eq!(RULE.required_after_cells(), 2);
        // Negative inputs clamp to zero, as the legacy
        // `crossing_required_margin_cells` did.
        let clamped = CrossingLegalityRule {
            half_size_cells: -1,
            bend_runout_cells: -4,
        };
        assert_eq!(clamped.required_margin_cells(), 0);
        assert_eq!(clamped.required_after_cells(), 0);
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
}
