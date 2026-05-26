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
