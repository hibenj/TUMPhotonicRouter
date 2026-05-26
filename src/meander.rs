use crate::astar::{RouteResult, State};
use crate::primitives::PrimitiveLibrary;

#[derive(Clone, Copy, Debug)]
pub(crate) struct MeanderCandidate {
    pub start_index: usize,
    pub end_index: usize,
    pub length_um: f64,
    pub heading_dx: i32,
    pub heading_dy: i32,
}

pub(crate) struct InsertSimpleMeanderReport {
    pub applied: bool,
    pub reason: &'static str,
    pub inserted_extra_length_um: f64,
    pub route: RouteResult,
}

pub(crate) struct AnalyzeMeanderInsertionReport {
    pub requested_extra_length_um: f64,
    pub inserted_extra_length_um: f64,
    pub conservative_legal_check: bool,
    pub candidates: Vec<MeanderCandidate>,
    pub status: &'static str,
    pub reason: &'static str,
}

fn primitive_length_by_id(lib: &PrimitiveLibrary, start_angle: u8, pid: u16) -> Option<f64> {
    lib.get_primitives_for_angle(start_angle)
        .iter()
        .find(|p| p.id == pid)
        .map(|p| p.length_um)
}

fn find_detour_sequence_between_states(
    lib: &PrimitiveLibrary,
    start: State,
    target: State,
    baseline_length_um: f64,
    requested_extra_length_um: f64,
    max_depth: usize,
) -> Option<(Vec<u16>, f64)> {
    #[derive(Clone)]
    struct Ctx<'a> {
        lib: &'a PrimitiveLibrary,
        target: State,
        baseline_length_um: f64,
        requested_extra_length_um: f64,
        best_seq: Option<Vec<u16>>,
        best_len: f64,
    }
    fn dfs(
        ctx: &mut Ctx<'_>,
        state: State,
        depth: usize,
        max_depth: usize,
        seq: &mut Vec<u16>,
        length_um: f64,
        turn_count: usize,
    ) {
        if depth > max_depth {
            return;
        }
        if depth > 0
            && state.x == ctx.target.x
            && state.y == ctx.target.y
            && state.angle == ctx.target.angle
            && turn_count >= 2
        {
            let extra = length_um - ctx.baseline_length_um;
            if extra > 0.0 && extra <= ctx.requested_extra_length_um + 1.0e-9 {
                if length_um > ctx.best_len {
                    ctx.best_len = length_um;
                    ctx.best_seq = Some(seq.clone());
                }
            }
            return;
        }

        if length_um - ctx.baseline_length_um > ctx.requested_extra_length_um + 1.0e-9 {
            return;
        }

        for p in ctx.lib.get_primitives_for_angle(state.angle) {
            let d = ((p.end_angle as i16 - p.start_angle as i16).rem_euclid(8)) as i16;
            let next_turn_count = if d == 0 { turn_count } else { turn_count + 1 };
            let next = State::new(state.x + p.dx, state.y + p.dy, p.end_angle);
            seq.push(p.id);
            dfs(
                ctx,
                next,
                depth + 1,
                max_depth,
                seq,
                length_um + p.length_um,
                next_turn_count,
            );
            seq.pop();
        }
    }

    let mut ctx = Ctx {
        lib,
        target,
        baseline_length_um,
        requested_extra_length_um,
        best_seq: None,
        best_len: f64::NEG_INFINITY,
    };
    let mut seq = Vec::new();
    dfs(&mut ctx, start, 0, max_depth, &mut seq, 0.0, 0);
    ctx.best_seq.map(|s| (s, ctx.best_len))
}

pub(crate) fn extract_straight_candidates(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid_size_um: f64,
    min_endpoint_margin_cells: i32,
    min_candidate_straight_length_um: f64,
) -> Vec<MeanderCandidate> {
    if route.states.len() < 2 || route.primitives.is_empty() {
        return Vec::new();
    }

    let margin = min_endpoint_margin_cells.max(0) as usize;
    let mut candidates: Vec<MeanderCandidate> = Vec::new();
    let mut i = 0usize;
    while i < route.primitives.len() {
        let start_angle = route.states[i].angle;
        let pid = route.primitives[i];
        let p = match primitives
            .get_primitives_for_angle(start_angle)
            .iter()
            .find(|pp| pp.id == pid)
        {
            Some(v) => v,
            None => {
                i += 1;
                continue;
            }
        };
        let d = ((p.end_angle as i16 - p.start_angle as i16).rem_euclid(8)) as i16;
        if d != 0 {
            i += 1;
            continue;
        }

        let run_start = i;
        let heading_angle = start_angle;
        let mut run_end = i;

        i += 1;
        while i < route.primitives.len() {
            let a = route.states[i].angle;
            let pid_i = route.primitives[i];
            let p_i = match primitives
                .get_primitives_for_angle(a)
                .iter()
                .find(|pp| pp.id == pid_i)
            {
                Some(v) => v,
                None => break,
            };
            let d_i = ((p_i.end_angle as i16 - p_i.start_angle as i16).rem_euclid(8)) as i16;
            if d_i != 0 || a != heading_angle {
                break;
            }
            run_end = i;
            i += 1;
        }

        let first_allowed_state = margin;
        let last_allowed_state = (route.states.len() - 1).saturating_sub(margin);
        if first_allowed_state >= last_allowed_state {
            continue;
        }
        let start_state_index = run_start.max(first_allowed_state);
        let end_state_index = (run_end + 1).min(last_allowed_state);
        if start_state_index >= end_state_index {
            continue;
        }

        let mut trimmed_length_um = 0.0f64;
        for prim_idx in start_state_index..end_state_index {
            let a = route.states[prim_idx].angle;
            let pid_t = route.primitives[prim_idx];
            let p_t = match primitives
                .get_primitives_for_angle(a)
                .iter()
                .find(|pp| pp.id == pid_t)
            {
                Some(v) => v,
                None => continue,
            };
            trimmed_length_um += p_t.length_um.max(grid_size_um);
        }
        if trimmed_length_um >= min_candidate_straight_length_um {
            let (heading_dx, heading_dy) = match heading_angle % 8 {
                0 => (1, 0),
                1 => (1, 1),
                2 => (0, 1),
                3 => (-1, 1),
                4 => (-1, 0),
                5 => (-1, -1),
                6 => (0, -1),
                _ => (1, -1),
            };
            candidates.push(MeanderCandidate {
                start_index: start_state_index,
                end_index: end_state_index,
                length_um: trimmed_length_um,
                heading_dx,
                heading_dy,
            });
        }
    }

    candidates.sort_by(|a, b| {
        b.length_um
            .partial_cmp(&a.length_um)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    candidates
}

pub(crate) fn insert_simple_meander_loop(
    primitives: &PrimitiveLibrary,
    base: &RouteResult,
    requested_extra_length_um: f64,
    insert_after_state_index: usize,
    insert_end_state_index: Option<usize>,
) -> Result<InsertSimpleMeanderReport, &'static str> {
    if requested_extra_length_um <= 0.0 {
        return Ok(InsertSimpleMeanderReport {
            applied: false,
            reason: "requested_extra_length_non_positive",
            inserted_extra_length_um: 0.0,
            route: base.clone(),
        });
    }
    let end_index = insert_end_state_index.unwrap_or(insert_after_state_index + 1);
    if base.states.len() < 2
        || insert_after_state_index >= base.states.len() - 1
        || end_index <= insert_after_state_index
        || end_index >= base.states.len()
    {
        return Ok(InsertSimpleMeanderReport {
            applied: false,
            reason: "invalid_insert_index",
            inserted_extra_length_um: 0.0,
            route: base.clone(),
        });
    }

    let start_state = base.states[insert_after_state_index];
    let end_state = base.states[end_index];
    let baseline_slice = &base.primitives[insert_after_state_index..end_index];
    let mut baseline_len = 0.0f64;
    let mut running_angle = start_state.angle;
    for pid in baseline_slice.iter().copied() {
        let p = primitives
            .get_primitives_for_angle(running_angle)
            .iter()
            .find(|p| p.id == pid)
            .ok_or("Baseline primitive lookup failed")?;
        baseline_len += p.length_um;
        running_angle = p.end_angle;
    }

    let (detour_ids, detour_len) = match find_detour_sequence_between_states(
        primitives,
        start_state,
        end_state,
        baseline_len,
        requested_extra_length_um,
        12,
    ) {
        Some(v) => v,
        None => {
            return Ok(InsertSimpleMeanderReport {
                applied: false,
                reason: "no_forward_meander_detour_found",
                inserted_extra_length_um: 0.0,
                route: base.clone(),
            });
        }
    };

    let inserted_extra = detour_len - baseline_len;
    if inserted_extra <= 0.0 {
        return Ok(InsertSimpleMeanderReport {
            applied: false,
            reason: "detour_does_not_increase_length",
            inserted_extra_length_um: 0.0,
            route: base.clone(),
        });
    }

    let mut new_primitives =
        Vec::with_capacity(base.primitives.len() - baseline_slice.len() + detour_ids.len());
    new_primitives.extend_from_slice(&base.primitives[..insert_after_state_index]);
    new_primitives.extend(detour_ids.iter().copied());
    new_primitives.extend_from_slice(&base.primitives[end_index..]);

    let mut new_states: Vec<State> = Vec::with_capacity(new_primitives.len() + 1);
    new_states.push(base.states[0]);
    for pid in new_primitives.iter().copied() {
        let cur = *new_states.last().ok_or("state build underflow")?;
        let p = primitives
            .get_primitives_for_angle(cur.angle)
            .iter()
            .find(|p| p.id == pid)
            .ok_or("Primitive id invalid for state angle")?;
        new_states.push(State::new(cur.x + p.dx, cur.y + p.dy, p.end_angle));
    }

    let mut new_total_length_um = 0.0f64;
    let mut running_angle2 = new_states[0].angle;
    for pid in new_primitives.iter().copied() {
        let len = primitive_length_by_id(primitives, running_angle2, pid)
            .ok_or("Primitive length lookup failed")?;
        new_total_length_um += len;
        let p = primitives
            .get_primitives_for_angle(running_angle2)
            .iter()
            .find(|p| p.id == pid)
            .ok_or("Primitive lookup failed")?;
        running_angle2 = p.end_angle;
    }

    let new_route = RouteResult {
        states: new_states,
        primitives: new_primitives,
        cells: base.cells.clone(),
        compressed_waypoints: base.compressed_waypoints.clone(),
        total_length_um: new_total_length_um,
        total_cost: base.total_cost + inserted_extra,
        requested_target: base.requested_target,
        reached_target: base.reached_target,
        stats: base.stats.clone(),
    };
    Ok(InsertSimpleMeanderReport {
        applied: true,
        reason: "applied_forward_meander_detour",
        inserted_extra_length_um: inserted_extra,
        route: new_route,
    })
}

pub(crate) fn analyze_meander_insertion_candidate(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid_size_um: f64,
    requested_extra_length_um: f64,
    min_endpoint_margin_cells: i32,
    min_candidate_straight_length_um: f64,
    max_extra_length_per_region_um: f64,
    conservative_legal_check: bool,
) -> AnalyzeMeanderInsertionReport {
    let candidates = extract_straight_candidates(
        route,
        primitives,
        grid_size_um,
        min_endpoint_margin_cells,
        min_candidate_straight_length_um,
    );

    let (status, reason) = if requested_extra_length_um <= 0.0 {
        ("illegal", "requested_extra_length_non_positive")
    } else if candidates.is_empty() {
        ("no_candidate", "no_valid_straight_candidate_region")
    } else if requested_extra_length_um > max_extra_length_per_region_um {
        ("illegal", "exceeds_max_extra_per_region")
    } else {
        (
            "unsupported_route_object",
            "route-object mutation not implemented yet",
        )
    };

    AnalyzeMeanderInsertionReport {
        requested_extra_length_um,
        inserted_extra_length_um: 0.0,
        conservative_legal_check,
        candidates,
        status,
        reason,
    }
}

// -----------------------------------------------------------------------------
// Analytic physical meander planner (new path, separate from legacy prototype)
// -----------------------------------------------------------------------------

const BEND_SAMPLES_PER_90_DEG: usize = 8;
const EPS: f64 = 1.0e-9;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MeanderSide {
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicalPoint {
    pub x_um: f64,
    pub y_um: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StraightSegment {
    pub start: PhysicalPoint,
    pub end: PhysicalPoint,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeanderBox {
    pub min_x_um: f64,
    pub max_x_um: f64,
    pub min_y_um: f64,
    pub max_y_um: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AnalyticMeanderConfig {
    pub requested_extra_length_um: f64,
    pub min_bend_radius_um: f64,
    pub min_straight_um: f64,
    pub max_bumps: usize,
    pub side: MeanderSide,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AnalyticMeanderPlan {
    pub centerline: Vec<PhysicalPoint>,
    pub inserted_extra_length_um: f64,
    pub bumps: usize,
    pub side: MeanderSide,
}

#[derive(Clone, Debug, PartialEq)]
pub enum MeanderPlanningError {
    NonFiniteInput,
    NonPositiveSegmentLength,
    NonPositiveBendRadius,
    NonPositiveRequestedExtraLength,
    UnsupportedSegmentOrientation,
    AvailableBoxTooSmall,
    RequestedExtraLengthDoesNotFit,
    MaxBumpsTooSmall,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum AxisOrientation {
    Horizontal,
    Vertical,
}

fn is_finite_point(p: PhysicalPoint) -> bool {
    p.x_um.is_finite() && p.y_um.is_finite()
}

fn is_finite_box(b: MeanderBox) -> bool {
    b.min_x_um.is_finite()
        && b.max_x_um.is_finite()
        && b.min_y_um.is_finite()
        && b.max_y_um.is_finite()
}

fn point_in_box(p: PhysicalPoint, b: MeanderBox) -> bool {
    p.x_um >= b.min_x_um - EPS
        && p.x_um <= b.max_x_um + EPS
        && p.y_um >= b.min_y_um - EPS
        && p.y_um <= b.max_y_um + EPS
}

fn centerline_length(points: &[PhysicalPoint]) -> f64 {
    points
        .windows(2)
        .map(|w| {
            let dx = w[1].x_um - w[0].x_um;
            let dy = w[1].y_um - w[0].y_um;
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
}

fn orientation_and_length(seg: StraightSegment) -> Result<(AxisOrientation, f64), MeanderPlanningError> {
    let dx = seg.end.x_um - seg.start.x_um;
    let dy = seg.end.y_um - seg.start.y_um;
    if dx.abs() <= EPS && dy.abs() <= EPS {
        return Err(MeanderPlanningError::NonPositiveSegmentLength);
    }
    if dy.abs() <= EPS {
        return Ok((AxisOrientation::Horizontal, dx.abs()));
    }
    if dx.abs() <= EPS {
        return Ok((AxisOrientation::Vertical, dy.abs()));
    }
    Err(MeanderPlanningError::UnsupportedSegmentOrientation)
}

fn world_from_local(
    seg: StraightSegment,
    orientation: AxisOrientation,
    side: MeanderSide,
    u: f64,
    v: f64,
) -> PhysicalPoint {
    let dx = seg.end.x_um - seg.start.x_um;
    let dy = seg.end.y_um - seg.start.y_um;
    let (tx, ty) = match orientation {
        AxisOrientation::Horizontal => (dx.signum(), 0.0),
        AxisOrientation::Vertical => (0.0, dy.signum()),
    };
    let (lx, ly) = (-ty, tx);
    let v_signed = match side {
        MeanderSide::Left => v,
        MeanderSide::Right => -v,
    };
    PhysicalPoint {
        x_um: seg.start.x_um + tx * u + lx * v_signed,
        y_um: seg.start.y_um + ty * u + ly * v_signed,
    }
}

fn side_capacity_um(
    seg: StraightSegment,
    orientation: AxisOrientation,
    b: MeanderBox,
    side: MeanderSide,
) -> f64 {
    match orientation {
        AxisOrientation::Horizontal => {
            let y0 = seg.start.y_um;
            match (seg.end.x_um >= seg.start.x_um, side) {
                (true, MeanderSide::Left) | (false, MeanderSide::Right) => b.max_y_um - y0,
                _ => y0 - b.min_y_um,
            }
        }
        AxisOrientation::Vertical => {
            let x0 = seg.start.x_um;
            match (seg.end.y_um >= seg.start.y_um, side) {
                (true, MeanderSide::Left) | (false, MeanderSide::Right) => x0 - b.min_x_um,
                _ => b.max_x_um - x0,
            }
        }
    }
}

fn append_line_local(
    out: &mut Vec<PhysicalPoint>,
    seg: StraightSegment,
    orientation: AxisOrientation,
    side: MeanderSide,
    u: f64,
    v: f64,
) {
    let p = world_from_local(seg, orientation, side, u, v);
    if out
        .last()
        .map(|last| (last.x_um - p.x_um).abs() <= EPS && (last.y_um - p.y_um).abs() <= EPS)
        .unwrap_or(false)
    {
        return;
    }
    out.push(p);
}

fn append_quarter_arc_local(
    out: &mut Vec<PhysicalPoint>,
    seg: StraightSegment,
    orientation: AxisOrientation,
    side: MeanderSide,
    cx: f64,
    cy: f64,
    r: f64,
    a0: f64,
    a1: f64,
) {
    for i in 1..=BEND_SAMPLES_PER_90_DEG {
        let t = (i as f64) / (BEND_SAMPLES_PER_90_DEG as f64);
        let a = a0 + (a1 - a0) * t;
        let u = cx + r * a.cos();
        let v = cy + r * a.sin();
        append_line_local(out, seg, orientation, side, u, v);
    }
}

pub fn plan_analytic_meander(
    segment: StraightSegment,
    available_box: MeanderBox,
    config: &AnalyticMeanderConfig,
) -> Result<AnalyticMeanderPlan, MeanderPlanningError> {
    if !is_finite_point(segment.start)
        || !is_finite_point(segment.end)
        || !is_finite_box(available_box)
        || !config.requested_extra_length_um.is_finite()
        || !config.min_bend_radius_um.is_finite()
        || !config.min_straight_um.is_finite()
    {
        return Err(MeanderPlanningError::NonFiniteInput);
    }
    if config.min_bend_radius_um <= 0.0 {
        return Err(MeanderPlanningError::NonPositiveBendRadius);
    }
    if config.requested_extra_length_um <= 0.0 {
        return Err(MeanderPlanningError::NonPositiveRequestedExtraLength);
    }
    if available_box.min_x_um > available_box.max_x_um || available_box.min_y_um > available_box.max_y_um {
        return Err(MeanderPlanningError::AvailableBoxTooSmall);
    }
    if !point_in_box(segment.start, available_box) || !point_in_box(segment.end, available_box) {
        return Err(MeanderPlanningError::AvailableBoxTooSmall);
    }

    let (orientation, segment_length_um) = orientation_and_length(segment)?;
    let r = config.min_bend_radius_um;
    let min_straight = config.min_straight_um.max(0.0);

    let side_capacity = side_capacity_um(segment, orientation, available_box, config.side);
    let min_amplitude = 2.0 * r + min_straight;
    if side_capacity + EPS < min_amplitude {
        return Err(MeanderPlanningError::AvailableBoxTooSmall);
    }
    let max_amplitude = side_capacity;

    if config.max_bumps == 0 {
        return Err(MeanderPlanningError::MaxBumpsTooSmall);
    }

    let min_span_per_bump = 6.0 * r + min_straight;
    let fit_by_span = (segment_length_um / min_span_per_bump).floor() as usize;
    if fit_by_span == 0 {
        return Err(MeanderPlanningError::AvailableBoxTooSmall);
    }
    let max_feasible_bumps = config.max_bumps.min(fit_by_span);
    if max_feasible_bumps == 0 {
        return Err(MeanderPlanningError::MaxBumpsTooSmall);
    }

    // extra_per_bump = 2*A + 2*r*(pi - 4), A in [min_amplitude, max_amplitude]
    let extra_per_bump_max = 2.0 * max_amplitude + 2.0 * r * (std::f64::consts::PI - 4.0);
    let max_total_extra = (max_feasible_bumps as f64) * extra_per_bump_max;
    if config.requested_extra_length_um > max_total_extra + EPS {
        return Err(MeanderPlanningError::RequestedExtraLengthDoesNotFit);
    }

    let mut chosen: Option<(usize, f64)> = None;
    for bumps in (1..=max_feasible_bumps).rev() {
        let target_per_bump = config.requested_extra_length_um / (bumps as f64);
        let amplitude =
            (target_per_bump - 2.0 * r * (std::f64::consts::PI - 4.0)) * 0.5;
        if amplitude + EPS < min_amplitude || amplitude - EPS > max_amplitude {
            continue;
        }
        let span = segment_length_um / (bumps as f64);
        let top_straight = span - 4.0 * r;
        if top_straight + EPS < 2.0 * r + min_straight {
            continue;
        }
        chosen = Some((bumps, amplitude.clamp(min_amplitude, max_amplitude)));
        break;
    }

    let (bumps, amplitude) = match chosen {
        Some(v) => v,
        None => {
            if config.max_bumps < fit_by_span {
                return Err(MeanderPlanningError::MaxBumpsTooSmall);
            }
            return Err(MeanderPlanningError::RequestedExtraLengthDoesNotFit);
        }
    };

    let span = segment_length_um / (bumps as f64);
    let top_straight = span - 4.0 * r;
    let vertical = amplitude - 2.0 * r;

    let mut centerline = Vec::new();
    append_line_local(&mut centerline, segment, orientation, config.side, 0.0, 0.0);

    for i in 0..bumps {
        let x0 = (i as f64) * span;
        append_line_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0,
            0.0,
        );

        append_quarter_arc_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0 + r,
            r,
            r,
            -std::f64::consts::FRAC_PI_2,
            0.0,
        );
        append_line_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0 + 2.0 * r,
            r + vertical,
        );
        append_quarter_arc_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0 + 3.0 * r,
            amplitude - r,
            r,
            std::f64::consts::PI,
            std::f64::consts::FRAC_PI_2,
        );
        append_line_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0 + 2.0 * r + top_straight,
            amplitude,
        );
        append_quarter_arc_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0 + 2.0 * r + top_straight,
            amplitude - r,
            r,
            std::f64::consts::FRAC_PI_2,
            0.0,
        );
        append_line_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0 + 3.0 * r + top_straight,
            r,
        );
        append_quarter_arc_local(
            &mut centerline,
            segment,
            orientation,
            config.side,
            x0 + 4.0 * r + top_straight,
            r,
            r,
            std::f64::consts::PI,
            std::f64::consts::FRAC_PI_2,
        );
    }
    append_line_local(
        &mut centerline,
        segment,
        orientation,
        config.side,
        segment_length_um,
        0.0,
    );

    for p in centerline.iter().copied() {
        if !point_in_box(p, available_box) {
            return Err(MeanderPlanningError::AvailableBoxTooSmall);
        }
    }

    let new_len = centerline_length(&centerline);
    let inserted_extra = new_len - segment_length_um;
    if inserted_extra <= 0.0 {
        return Err(MeanderPlanningError::RequestedExtraLengthDoesNotFit);
    }

    Ok(AnalyticMeanderPlan {
        centerline,
        inserted_extra_length_um: inserted_extra,
        bumps,
        side: config.side,
    })
}

#[cfg(test)]
mod analytic_tests {
    use super::*;

    fn assert_plan_basics(plan: &AnalyticMeanderPlan, seg: StraightSegment, b: MeanderBox) {
        assert!(!plan.centerline.is_empty());
        let first = plan.centerline.first().copied().unwrap();
        let last = plan.centerline.last().copied().unwrap();
        assert!((first.x_um - seg.start.x_um).abs() < 1.0e-6);
        assert!((first.y_um - seg.start.y_um).abs() < 1.0e-6);
        assert!((last.x_um - seg.end.x_um).abs() < 1.0e-6);
        assert!((last.y_um - seg.end.y_um).abs() < 1.0e-6);
        for p in &plan.centerline {
            assert!(point_in_box(*p, b));
        }
        let base_len = ((seg.end.x_um - seg.start.x_um).powi(2) + (seg.end.y_um - seg.start.y_um).powi(2)).sqrt();
        let planned_len = centerline_length(&plan.centerline);
        assert!(plan.inserted_extra_length_um > 0.0);
        assert!(planned_len > base_len);
    }

    #[test]
    fn analytic_meander_horizontal_success() {
        let seg = StraightSegment {
            start: PhysicalPoint { x_um: 0.0, y_um: 0.0 },
            end: PhysicalPoint { x_um: 200.0, y_um: 0.0 },
        };
        let b = MeanderBox {
            min_x_um: -10.0,
            max_x_um: 210.0,
            min_y_um: -5.0,
            max_y_um: 80.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 120.0,
            min_bend_radius_um: 5.0,
            min_straight_um: 2.0,
            max_bumps: 8,
            side: MeanderSide::Left,
        };
        let plan = plan_analytic_meander(seg, b, &cfg).expect("plan should succeed");
        assert_plan_basics(&plan, seg, b);
    }

    #[test]
    fn analytic_meander_vertical_success() {
        let seg = StraightSegment {
            start: PhysicalPoint { x_um: 50.0, y_um: 0.0 },
            end: PhysicalPoint { x_um: 50.0, y_um: 180.0 },
        };
        let b = MeanderBox {
            min_x_um: -20.0,
            max_x_um: 90.0,
            min_y_um: -10.0,
            max_y_um: 200.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 80.0,
            min_bend_radius_um: 4.0,
            min_straight_um: 2.0,
            max_bumps: 6,
            side: MeanderSide::Right,
        };
        let plan = plan_analytic_meander(seg, b, &cfg).expect("plan should succeed");
        assert_plan_basics(&plan, seg, b);
    }

    #[test]
    fn analytic_meander_requested_extra_too_large() {
        let seg = StraightSegment {
            start: PhysicalPoint { x_um: 0.0, y_um: 0.0 },
            end: PhysicalPoint { x_um: 80.0, y_um: 0.0 },
        };
        let b = MeanderBox {
            min_x_um: 0.0,
            max_x_um: 80.0,
            min_y_um: -2.0,
            max_y_um: 20.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 500.0,
            min_bend_radius_um: 3.0,
            min_straight_um: 2.0,
            max_bumps: 4,
            side: MeanderSide::Left,
        };
        let err = plan_analytic_meander(seg, b, &cfg).unwrap_err();
        assert!(matches!(
            err,
            MeanderPlanningError::RequestedExtraLengthDoesNotFit
                | MeanderPlanningError::AvailableBoxTooSmall
        ));
    }

    #[test]
    fn analytic_meander_diagonal_unsupported() {
        let seg = StraightSegment {
            start: PhysicalPoint { x_um: 0.0, y_um: 0.0 },
            end: PhysicalPoint { x_um: 100.0, y_um: 100.0 },
        };
        let b = MeanderBox {
            min_x_um: -10.0,
            max_x_um: 110.0,
            min_y_um: -10.0,
            max_y_um: 110.0,
        };
        let cfg = AnalyticMeanderConfig {
            requested_extra_length_um: 20.0,
            min_bend_radius_um: 2.0,
            min_straight_um: 1.0,
            max_bumps: 2,
            side: MeanderSide::Left,
        };
        let err = plan_analytic_meander(seg, b, &cfg).unwrap_err();
        assert_eq!(err, MeanderPlanningError::UnsupportedSegmentOrientation);
    }

    #[test]
    fn analytic_meander_invalid_inputs() {
        let seg = StraightSegment {
            start: PhysicalPoint { x_um: 0.0, y_um: 0.0 },
            end: PhysicalPoint { x_um: 100.0, y_um: 0.0 },
        };
        let b = MeanderBox {
            min_x_um: -5.0,
            max_x_um: 105.0,
            min_y_um: -5.0,
            max_y_um: 50.0,
        };

        let bad_radius = AnalyticMeanderConfig {
            requested_extra_length_um: 20.0,
            min_bend_radius_um: 0.0,
            min_straight_um: 1.0,
            max_bumps: 2,
            side: MeanderSide::Left,
        };
        let err = plan_analytic_meander(seg, b, &bad_radius).unwrap_err();
        assert_eq!(err, MeanderPlanningError::NonPositiveBendRadius);

        let bad_extra = AnalyticMeanderConfig {
            requested_extra_length_um: 0.0,
            min_bend_radius_um: 2.0,
            min_straight_um: 1.0,
            max_bumps: 2,
            side: MeanderSide::Left,
        };
        let err = plan_analytic_meander(seg, b, &bad_extra).unwrap_err();
        assert_eq!(err, MeanderPlanningError::NonPositiveRequestedExtraLength);
    }
}
