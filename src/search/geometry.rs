//! Grid-polyline geometry helpers shared by the simple-route candidates, the
//! dense JPS4 search and the unified kernel's reconstruction step: waypoint
//! compression, orientation/segment-intersection primitives and the
//! self-intersection check. Moved out of `src/astar.rs` (Milestone 3, Slice
//! 2); pure code motion, no behaviour change.

pub(crate) fn compress_grid_waypoints(path: &[(i32, i32)]) -> Vec<(i32, i32)> {
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

pub(crate) fn push_if_different(points: &mut Vec<(i32, i32)>, point: (i32, i32)) {
    if points.last().copied() != Some(point) {
        points.push(point);
    }
}

/// Twice the signed area of triangle `p, q, r` (the standard orientation
/// predicate for 2D segment intersection): positive for counter-clockwise,
/// negative for clockwise, zero for collinear. Exact in `i64` for `i32`
/// grid-cell inputs (no overflow: max magnitude well under `i64::MAX`).
pub(crate) fn orientation(p: (i32, i32), q: (i32, i32), r: (i32, i32)) -> i64 {
    (i64::from(q.1) - i64::from(p.1)) * (i64::from(r.0) - i64::from(q.0))
        - (i64::from(q.0) - i64::from(p.0)) * (i64::from(r.1) - i64::from(q.1))
}

/// True if `q` lies strictly between `p` and `r` (exclusive of both
/// endpoints), given `p`, `q`, `r` are already known collinear
/// (`orientation(p, q, r) == 0`). Used to detect a "T-junction" -- one
/// segment's endpoint landing in the *interior* of another, which is a
/// real problem (the path touches the interior of a stretch it already
/// laid down) -- while treating `q` landing exactly on `p` or `r` (a
/// shared vertex between the two segments) as not a T-junction.
pub(crate) fn strictly_between(p: (i32, i32), q: (i32, i32), r: (i32, i32)) -> bool {
    q != p
        && q != r
        && q.0 >= p.0.min(r.0)
        && q.0 <= p.0.max(r.0)
        && q.1 >= p.1.min(r.1)
        && q.1 <= p.1.max(r.1)
}

/// Segment intersection test tuned for polyline self-intersection, not
/// general-purpose geometry: a route that revisits an *exact* earlier
/// vertex is common and legitimate (e.g. a one-cell overshoot-and-return
/// to satisfy a required terminal heading), so touching only at a shared
/// endpoint between the two segments is deliberately **not** flagged --
/// only a proper transversal crossing (segments cross through both
/// interiors), a T-junction (one segment's endpoint lands in the other's
/// interior), or a genuine collinear overlap of more than a single point
/// count as a real self-intersection. This is a stricter notion of
/// "simple" than `shapely`'s `LineString.is_simple` (which flags any
/// repeated vertex, including simple shared-endpoint touches) -- verified
/// deliberately looser via a real regression this distinction fixed, see
/// `polyline_self_intersects`'s own doc comment.
pub(crate) fn segments_intersect(
    p1: (i32, i32),
    q1: (i32, i32),
    p2: (i32, i32),
    q2: (i32, i32),
) -> bool {
    let o1 = orientation(p1, q1, p2);
    let o2 = orientation(p1, q1, q2);
    let o3 = orientation(p2, q2, p1);
    let o4 = orientation(p2, q2, q1);

    // Proper transversal crossing: each segment's endpoints straddle the
    // other segment's line.
    if o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0 {
        return (o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0);
    }

    // T-junction: one segment's endpoint lies strictly inside the other.
    if o1 == 0 && strictly_between(p1, p2, q1) {
        return true;
    }
    if o2 == 0 && strictly_between(p1, q2, q1) {
        return true;
    }
    if o3 == 0 && strictly_between(p2, p1, q2) {
        return true;
    }
    if o4 == 0 && strictly_between(p2, q1, q2) {
        return true;
    }

    // Fully collinear (all four points on one line): overlap only counts
    // if it spans more than the single point the two segments might share
    // as a common vertex.
    if o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0 {
        let use_x = (q1.0 - p1.0).abs() >= (q1.1 - p1.1).abs();
        let (a0, a1) = if use_x {
            (p1.0.min(q1.0), p1.0.max(q1.0))
        } else {
            (p1.1.min(q1.1), p1.1.max(q1.1))
        };
        let (b0, b1) = if use_x {
            (p2.0.min(q2.0), p2.0.max(q2.0))
        } else {
            (p2.1.min(q2.1), p2.1.max(q2.1))
        };
        return a0.max(b0) < a1.min(b1);
    }
    false
}

/// Does this polyline (already-compressed waypoints, one vertex per
/// direction change) cross itself anywhere? Checks every pair of
/// non-adjacent segments (immediately-adjacent segments always share
/// exactly one endpoint -- that is just the polyline continuing, not a
/// crossing). A route revisiting an *exact* earlier vertex (not just any
/// two non-adjacent segments touching) is deliberately allowed -- see
/// `segments_intersect`'s own doc comment; found via a real regression
/// (`test_rust_batch_repair_rips_and_reroutes_dynamic_blocker`) where a
/// legitimate one-cell overshoot-and-return needed to satisfy a required
/// terminal heading was being rejected as if it were a genuine crossing
/// loop. `O(n^2)` in the number of waypoints, which is fine here: a
/// compressed route is at most a few hundred points, checked once per
/// candidate route, not once per search-loop iteration.
pub(crate) fn polyline_self_intersects(points: &[(i32, i32)]) -> bool {
    if points.len() < 4 {
        return false;
    }
    for i in 0..points.len() - 1 {
        let (p1, q1) = (points[i], points[i + 1]);
        for j in (i + 2)..points.len() - 1 {
            let (p2, q2) = (points[j], points[j + 1]);
            if segments_intersect(p1, q1, p2, q2) {
                return true;
            }
        }
    }
    false
}

pub(crate) fn direction(a: (i32, i32), b: (i32, i32)) -> (i32, i32) {
    ((b.0 - a.0).signum(), (b.1 - a.1).signum())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polyline_self_intersects_accepts_a_terminal_heading_overshoot_and_return() {
        // The exact waypoints from a real regression
        // (`test_rust_batch_repair_rips_and_reroutes_dynamic_blocker`): the
        // search overshoots the target by one cell then immediately
        // returns to it, satisfying a required terminal heading. The path
        // revisits the exact vertex (47, 10) non-adjacently, which makes
        // it "non-simple" by the strict textbook definition (confirmed via
        // `shapely.LineString(...).is_simple == False` on this exact
        // sequence) but is not a genuine crossing loop -- the two
        // segments touching at (47, 10) are perpendicular and share
        // nothing but that one vertex, not a transversal crossing, a
        // T-junction into an interior, or a collinear overlap.
        assert!(!polyline_self_intersects(&[
            (2, 10),
            (2, 14),
            (47, 14),
            (47, 10),
            (46, 10),
            (47, 10),
        ]));
    }

    #[test]
    fn polyline_self_intersects_accepts_simple_paths() {
        // A plain L-shaped bend and a longer zig-zag: neither crosses
        // itself, and both should be accepted.
        assert!(!polyline_self_intersects(&[(0, 0), (5, 0), (5, 5)]));
        assert!(!polyline_self_intersects(&[
            (0, 0),
            (4, 0),
            (4, 4),
            (8, 4),
            (8, 8),
        ]));
        assert!(!polyline_self_intersects(&[]));
        assert!(!polyline_self_intersects(&[(0, 0)]));
        assert!(!polyline_self_intersects(&[(0, 0), (1, 1)]));
    }

    #[test]
    fn polyline_self_intersects_rejects_a_genuine_crossing() {
        // A simple bowtie: two segments that cross at a single interior
        // point, non-adjacent in the polyline.
        assert!(polyline_self_intersects(&[
            (0, 0),
            (10, 10),
            (10, 0),
            (0, 10),
        ]));
    }

    #[test]
    fn polyline_self_intersects_rejects_collinear_overlap() {
        // Two non-adjacent segments lying on the same line, overlapping
        // without a single crossing point, still count as self-intersecting.
        assert!(polyline_self_intersects(&[
            (0, 0),
            (10, 0),
            (10, 5),
            (5, 5),
            (5, 0),
            (2, 0),
        ]));
    }

    #[test]
    fn polyline_self_intersects_flags_the_real_n_31_multiportmmi_8x8_loop() {
        // The exact raw grid cells a real routed net produced (`n_31`,
        // `multiportmmi_8x8`, tighter-than-default fanout lane spacing):
        // it needed to legally cross four other nets in a tight vertical
        // corridor, and doubled back through a small loop, crossing its
        // own earlier diagonal, before continuing to its target. Captured
        // verbatim via a temporary diagnostic print of `route_obj.cells`
        // for this exact net during a real reproduction run, not a
        // synthetic case -- and independently confirmed self-intersecting
        // via `shapely.LineString(cells).is_simple` on this same list
        // before this check existed. See
        // `.agent/execplans/2026-08-25-negotiated-repair-engine.md`.
        #[rustfmt::skip]
        let cells: Vec<(i32, i32)> = vec![
            (714, 149), (715, 149), (716, 149), (717, 149), (718, 148), (719, 147),
            (720, 146), (721, 145), (722, 144), (723, 143), (724, 143), (725, 143),
            (726, 143), (727, 143), (728, 143), (729, 143), (730, 143), (731, 143),
            (732, 143), (733, 143), (734, 143), (735, 143), (736, 143), (737, 143),
            (738, 144), (739, 145), (740, 146), (741, 147), (742, 148), (743, 149),
            (744, 150), (745, 151), (746, 152), (747, 153), (748, 154), (749, 155),
            (750, 156), (751, 157), (752, 158), (753, 159), (754, 160), (755, 161),
            (756, 162), (755, 163), (754, 164), (753, 165), (752, 166), (751, 167),
            (750, 168), (749, 168), (748, 168), (747, 168), (746, 168), (745, 168),
            (744, 168), (744, 167), (744, 166), (744, 165), (744, 164), (744, 163),
            (744, 162), (745, 162), (746, 162), (747, 162), (748, 162), (749, 162),
            (750, 162), (750, 163), (750, 164), (750, 165), (750, 166), (750, 167),
            (750, 169), (750, 170), (750, 171), (750, 172), (750, 173), (750, 174),
            (750, 175), (750, 176), (750, 177), (750, 178), (750, 179), (750, 180),
            (750, 181), (750, 182), (750, 183), (750, 184), (750, 185), (750, 186),
            (750, 187), (750, 188), (750, 189), (750, 190), (750, 191), (750, 192),
            (750, 193), (750, 194), (750, 195), (750, 196), (750, 197), (750, 198),
            (750, 199), (750, 200), (750, 201), (750, 202), (750, 203), (750, 204),
            (750, 205), (750, 206), (750, 207), (750, 208), (750, 209), (750, 210),
            (750, 211), (750, 212), (750, 213), (750, 214), (750, 215), (750, 216),
            (750, 217), (750, 218), (750, 219), (750, 220), (750, 221), (750, 222),
            (750, 223), (750, 224), (750, 225), (750, 226), (750, 227), (750, 228),
            (750, 229), (751, 230), (752, 231), (753, 232), (754, 233), (755, 234),
            (756, 235), (757, 236), (758, 237), (759, 238), (760, 239), (761, 240),
            (762, 241), (763, 242), (764, 243), (765, 243), (766, 243), (767, 243),
        ];
        assert!(polyline_self_intersects(&cells));
    }
}
