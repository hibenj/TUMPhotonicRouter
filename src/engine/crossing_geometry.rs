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
/// from failing at net 109 to net 156. Set
/// PHOTONIC_ROUTER_ENABLE_ORTHOGONAL_REPAIR_FALLBACK to restore the old
/// behavior.
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
