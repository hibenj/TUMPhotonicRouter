//! Primitive-move expansion helpers: the footprint collision profile, the
//! primitive transition-class/ordering helpers used to iterate moves in a
//! consistent order, the target-biased primitive score, and the compact
//! diagonal halo used by the Tier-1 collision gate. Slice 3 of Milestone 3
//! adds the expansion functions extracted from the kernel loop. Moved out
//! of `src/astar.rs` (Milestone 3, Slice 2); pure code motion, no behaviour
//! change.

use crate::obstacle_map::{pack_xy, CellKey, NetId, ObstacleMap};
use crate::primitives::{Primitive, PrimitiveGeometry, DIRECTIONS};
use crate::search::astar::config::{
    AStarConfig, PrimitiveOrdering, PRIMITIVE_BEND_45, PRIMITIVE_BEND_90, PRIMITIVE_STRAIGHT_LONG,
    PRIMITIVE_STRAIGHT_SHORT,
};
use crate::search::astar::cost::PrimitiveSearchMetadata;
use crate::search::astar::crossing_rules::{
    CrossingExtension, CrossingLegalityHook, CrossingMoveOutcome, PrimitiveCrossingMetadata,
};
use crate::search::astar::dense::DenseRoutingGrid;
use crate::search::astar::heuristic::distance_heuristic;
use crate::search::astar::window::RoutingBounds;
use crate::search::state::{RouteSearchStats, State};
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::time::Instant;

#[derive(Clone, Debug)]
pub(crate) struct FootprintCollisionProfile {
    pub(crate) is_full_rect: bool,
    pub(crate) min_dx: i32,
    pub(crate) max_dx: i32,
    pub(crate) min_dy: i32,
    pub(crate) max_dy: i32,
    pub(crate) cell_count: usize,
    pub(crate) horizontal_runs: Vec<(i32, i32, i32)>,
}

impl FootprintCollisionProfile {
    #[inline]
    pub(crate) fn from_footprint(footprint: &[(i32, i32)]) -> Self {
        let cell_count = footprint.len();
        if footprint.is_empty() {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
                horizontal_runs: Vec::new(),
            };
        }

        let mut min_dx = i32::MAX;
        let mut max_dx = i32::MIN;
        let mut min_dy = i32::MAX;
        let mut max_dy = i32::MIN;
        for &(dx, dy) in footprint {
            min_dx = min_dx.min(dx);
            max_dx = max_dx.max(dx);
            min_dy = min_dy.min(dy);
            max_dy = max_dy.max(dy);
        }

        let width = max_dx.checked_sub(min_dx).and_then(|v| v.checked_add(1));
        let height = max_dy.checked_sub(min_dy).and_then(|v| v.checked_add(1));
        let (Some(width_usize), Some(height_usize)) = (
            width.and_then(|v| usize::try_from(v).ok()),
            height.and_then(|v| usize::try_from(v).ok()),
        ) else {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
                horizontal_runs: footprint_horizontal_runs(footprint),
            };
        };

        let area = width_usize.checked_mul(height_usize);
        if area != Some(cell_count) {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
                horizontal_runs: footprint_horizontal_runs(footprint),
            };
        }

        let mut sorted_footprint = footprint.to_vec();
        sorted_footprint.sort_unstable_by(|(a_x, a_y), (b_x, b_y)| a_y.cmp(b_y).then(a_x.cmp(b_x)));
        let mut idx = 0usize;
        for y in min_dy..=max_dy {
            for x in min_dx..=max_dx {
                if idx >= sorted_footprint.len() || sorted_footprint[idx] != (x, y) {
                    return Self {
                        is_full_rect: false,
                        min_dx: 0,
                        max_dx: -1,
                        min_dy: 0,
                        max_dy: -1,
                        cell_count,
                        horizontal_runs: footprint_horizontal_runs(footprint),
                    };
                }
                idx += 1;
            }
        }

        Self {
            is_full_rect: true,
            min_dx,
            max_dx,
            min_dy,
            max_dy,
            cell_count,
            horizontal_runs: Vec::new(),
        }
    }
}

pub(crate) fn footprint_horizontal_runs(footprint: &[(i32, i32)]) -> Vec<(i32, i32, i32)> {
    if footprint.is_empty() {
        return Vec::new();
    }
    let mut cells = footprint.to_vec();
    cells.sort_unstable_by(|(a_x, a_y), (b_x, b_y)| a_y.cmp(b_y).then(a_x.cmp(b_x)));
    cells.dedup();

    let mut runs = Vec::new();
    let mut current_y = cells[0].1;
    let mut start_x = cells[0].0;
    let mut end_x = cells[0].0;
    for &(x, y) in cells.iter().skip(1) {
        if y == current_y && x == end_x + 1 {
            end_x = x;
            continue;
        }
        runs.push((current_y, start_x, end_x));
        current_y = y;
        start_x = x;
        end_x = x;
    }
    runs.push((current_y, start_x, end_x));
    runs
}

pub(crate) fn primitive_transition_class(geometry: &PrimitiveGeometry, dx: i32, dy: i32) -> usize {
    match geometry {
        PrimitiveGeometry::Straight { .. } => {
            if dx.abs().max(dy.abs()) <= 1 {
                PRIMITIVE_STRAIGHT_SHORT
            } else {
                PRIMITIVE_STRAIGHT_LONG
            }
        }
        PrimitiveGeometry::Bend { angle_delta, .. } => {
            if angle_delta.unsigned_abs() == 1 {
                PRIMITIVE_BEND_45
            } else {
                PRIMITIVE_BEND_90
            }
        }
    }
}

pub(crate) fn fixed_primitive_order(len: usize) -> ([usize; 8], usize) {
    let mut order = [0usize; 8];
    for (idx, slot) in order.iter_mut().enumerate().take(len.min(8)) {
        *slot = idx;
    }
    (order, len.min(8))
}

pub(crate) fn primitive_class_order_rank(class: usize) -> usize {
    match class {
        PRIMITIVE_STRAIGHT_LONG => 0,
        PRIMITIVE_STRAIGHT_SHORT => 1,
        PRIMITIVE_BEND_45 => 2,
        PRIMITIVE_BEND_90 => 3,
        _ => 4,
    }
}

#[inline]
pub(crate) fn primitive_class_is_straight(class: usize) -> bool {
    matches!(class, PRIMITIVE_STRAIGHT_SHORT | PRIMITIVE_STRAIGHT_LONG)
}

pub(crate) fn primitive_initial_straight_run_distance(
    primitive: &Primitive,
    start_angle: u8,
) -> f64 {
    let dir = DIRECTIONS[(start_angle % 8) as usize];
    let mut run_cells = 0i32;
    for (idx, point) in primitive.footprint.iter().copied().enumerate() {
        let step = idx as i32;
        if point != (dir.0 * step, dir.1 * step) {
            break;
        }
        run_cells = step;
    }
    f64::from(run_cells)
}

pub(crate) fn primitive_terminal_straight_run_cells(primitive: &Primitive, end_angle: u8) -> i32 {
    let Some(end) = primitive.footprint.last().copied() else {
        return 0;
    };
    let dir = DIRECTIONS[(end_angle % 8) as usize];
    let mut run_cells = 0i32;
    for (idx, point) in primitive.footprint.iter().copied().enumerate().rev() {
        let step = (primitive.footprint.len() - 1 - idx) as i32;
        if point != (end.0 - dir.0 * step, end.1 - dir.1 * step) {
            break;
        }
        run_cells = step;
    }
    run_cells
}

pub(crate) fn target_biased_primitive_score(
    primitive: &Primitive,
    metadata: PrimitiveSearchMetadata,
    state: State,
    target: State,
    grid_size_um: f64,
) -> f64 {
    let Some(next_x) = state.x.checked_add(primitive.dx) else {
        return f64::INFINITY;
    };
    let Some(next_y) = state.y.checked_add(primitive.dy) else {
        return f64::INFINITY;
    };
    metadata.base_step_cost
        + distance_heuristic(
            State::new(next_x, next_y, primitive.end_angle),
            target,
            grid_size_um,
        )
}

pub(crate) fn primitive_iteration_order(
    primitives: &[Primitive],
    metadata: &[PrimitiveSearchMetadata],
    state: State,
    target: State,
    grid_size_um: f64,
    ordering: PrimitiveOrdering,
) -> ([usize; 8], usize) {
    let (mut order, len) = fixed_primitive_order(primitives.len());
    match ordering {
        PrimitiveOrdering::Library => {}
        PrimitiveOrdering::LongStraightFirst => {
            order[..len].sort_by(|a, b| {
                primitive_class_order_rank(metadata[*a].transition_class)
                    .cmp(&primitive_class_order_rank(metadata[*b].transition_class))
                    .then_with(|| a.cmp(b))
            });
        }
        PrimitiveOrdering::TargetBiased => {
            order[..len].sort_by(|a, b| {
                let a_score = target_biased_primitive_score(
                    &primitives[*a],
                    metadata[*a],
                    state,
                    target,
                    grid_size_um,
                );
                let b_score = target_biased_primitive_score(
                    &primitives[*b],
                    metadata[*b],
                    state,
                    target,
                    grid_size_um,
                );
                a_score
                    .partial_cmp(&b_score)
                    .unwrap_or(Ordering::Equal)
                    .then_with(|| a.cmp(b))
            });
        }
    }
    (order, len)
}

pub(crate) fn compact_diagonal_halo_cells(
    start: (i32, i32),
    end: (i32, i32),
    dx: i32,
    dy: i32,
) -> Vec<(i32, i32)> {
    let mut cells = Vec::with_capacity(4);
    push_unique_cell(&mut cells, (start.0 + dx, start.1));
    push_unique_cell(&mut cells, (end.0 + dx, end.1));
    push_unique_cell(&mut cells, (start.0, start.1 + dy));
    push_unique_cell(&mut cells, (end.0, end.1 + dy));
    cells
}

pub(crate) fn push_unique_cell(cells: &mut Vec<(i32, i32)>, cell: (i32, i32)) {
    if !cells.contains(&cell) {
        cells.push(cell);
    }
}

/// One primitive move considered from the current state, carrying the
/// per-primitive tables the expansion loop reads for it. Produced by
/// [`candidate_moves`] in the kernel's iteration order.
#[derive(Clone, Copy)]
pub(crate) struct CandidateMove<'a> {
    pub(crate) primitive: &'a Primitive,
    pub(crate) metadata: PrimitiveSearchMetadata,
    pub(crate) profile: &'a FootprintCollisionProfile,
    pub(crate) crossing: &'a PrimitiveCrossingMetadata,
    /// `metadata.transition_class`, the index of every per-class counter.
    pub(crate) class: usize,
    pub(crate) class_is_straight: bool,
}

/// The ordered primitive moves available from one state: the primitive
/// library's bucket for the state's angle, visited in
/// [`primitive_iteration_order`]. Allocation-free: the order is the same
/// fixed eight-slot array the kernel used inline.
pub(crate) struct CandidateMoves<'a> {
    order: [usize; 8],
    len: usize,
    next: usize,
    primitives: &'a [Primitive],
    metadata: &'a [PrimitiveSearchMetadata],
    profiles: &'a [FootprintCollisionProfile],
    crossing_metadata: &'a [PrimitiveCrossingMetadata],
}

impl<'a> Iterator for CandidateMoves<'a> {
    type Item = CandidateMove<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next >= self.len {
            return None;
        }
        let idx = self.order[self.next];
        self.next += 1;
        let metadata = self.metadata[idx];
        Some(CandidateMove {
            primitive: &self.primitives[idx],
            metadata,
            profile: &self.profiles[idx],
            crossing: &self.crossing_metadata[idx],
            class: metadata.transition_class,
            class_is_straight: primitive_class_is_straight(metadata.transition_class),
        })
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn candidate_moves<'a>(
    primitives: &'a [Primitive],
    metadata: &'a [PrimitiveSearchMetadata],
    profiles: &'a [FootprintCollisionProfile],
    crossing_metadata: &'a [PrimitiveCrossingMetadata],
    state: State,
    target: State,
    grid_size_um: f64,
    ordering: PrimitiveOrdering,
) -> CandidateMoves<'a> {
    let (order, len) =
        primitive_iteration_order(primitives, metadata, state, target, grid_size_um, ordering);
    CandidateMoves {
        order,
        len,
        next: 0,
        primitives,
        metadata,
        profiles,
        crossing_metadata,
    }
}

/// The search environment every move legality check reads. Immutable for
/// the whole search, so the kernel builds one before its loop.
pub(crate) struct MoveContext<'a, H: CrossingLegalityHook> {
    pub(crate) dense_grid: &'a DenseRoutingGrid,
    pub(crate) config: &'a AStarConfig,
    pub(crate) hook: &'a H,
    pub(crate) bounds: RoutingBounds,
    pub(crate) source: State,
    pub(crate) target: State,
    pub(crate) target_tolerance: i32,
    pub(crate) accepted_target_angles: &'a [bool; 8],
}

/// The diagnostics state a rejected move writes to: the target-ring and
/// probe-cell landing counters, the per-slot blocker-line budget, and the
/// env-gated move-diag cell. Borrowed for the duration of one
/// [`move_is_legal`] call only -- the insert branches in the kernel loop
/// keep writing the same counters directly afterwards.
pub(crate) struct MoveDiagnostics<'a> {
    pub(crate) failure_diag: bool,
    pub(crate) move_diag_cell: Option<(i32, i32)>,
    pub(crate) search_seq: u64,
    pub(crate) obstacle_map: &'a ObstacleMap,
    pub(crate) port_open_cells: Option<&'a FxHashSet<CellKey>>,
    pub(crate) target: State,
    pub(crate) probe_index: &'a FxHashMap<(i32, i32), usize>,
    pub(crate) target_ring: &'a mut [[u32; 7]; 25],
    pub(crate) probe_ring: &'a mut [[u32; 7]],
    pub(crate) ring_blocker_lines: &'a mut FxHashMap<usize, u32>,
}

impl MoveDiagnostics<'_> {
    /// The kernel's `ring_index`: the 5x5 ring around the target first,
    /// then the explicitly named probe cells (slots 25..).
    fn ring_index(&self, x: i32, y: i32) -> Option<usize> {
        let dx = x - self.target.x;
        let dy = y - self.target.y;
        if dx.abs() <= 2 && dy.abs() <= 2 {
            Some(((dy + 2) * 5 + (dx + 2)) as usize)
        } else {
            self.probe_index.get(&(x, y)).copied()
        }
    }

    fn count_landing(&mut self, ring_slot: usize, counter: usize) {
        if ring_slot >= 25 {
            self.probe_ring[ring_slot - 25][counter] += 1;
        } else {
            self.target_ring[ring_slot][counter] += 1;
        }
    }
}

/// Why a move was rejected. Names the kernel's reject branches in their
/// order; the counters and diagnostic lines have already been written by
/// the time the reason is returned, so the kernel loop only skips the move.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MoveRejection {
    /// `require_terminal_straights`: no bend out of the source.
    TerminalStraightAtSource,
    /// Inside an active post-crossing corridor, heading the wrong way.
    PendingCorridorAngle,
    /// Inside an active post-crossing corridor, and the primitive neither
    /// completes nor correctly extends the required straight run.
    PendingCorridorStraightRun,
    /// `require_terminal_straights`: no bend into the target.
    TerminalStraightAtTarget,
    /// Outside the routing window.
    OutOfBounds,
    /// Footprint (or clearance profile) blocked, and the legality hook
    /// could not legalize the contact.
    Footprint,
    /// Footprint free, but the legality hook rejected the move.
    Hook,
}

/// What the kernel's two insert-or-improve branches need about a move
/// that passed every reject rule.
pub(crate) struct LegalMove {
    pub(crate) next_state: State,
    /// Target-ring/probe-cell slot of the landing cell, `None` unless
    /// `search_failure_diag` is on.
    pub(crate) ring_slot: Option<usize>,
    /// Whether this primitive's own initial straight run already pays off
    /// the current state's post-crossing debt.
    pub(crate) pending_completed_by_primitive: bool,
}

pub(crate) enum MoveVerdict {
    /// The reason names the rule that rejected the move; the kernel loop
    /// only skips the move (every counter and diagnostic line has already
    /// been written), so the reason is read by the tests below.
    Illegal(#[cfg_attr(not(test), allow(dead_code))] MoveRejection),
    /// Free footprint, free halo, no crossing bookkeeping: the dense fast path.
    Tier1(LegalMove),
    /// The legality hook accepted the move: its outcome and extra cost.
    Tier2(LegalMove, CrossingMoveOutcome, f64),
}

/// Every reject rule of one primitive move, in the kernel's order, ending
/// in the tier the move belongs to. `None` is the kernel's `checked_add`
/// `?`: a coordinate overflow abandons the whole search.
#[allow(clippy::too_many_arguments)]
pub(crate) fn move_is_legal<H: CrossingLegalityHook>(
    context: &MoveContext<'_, H>,
    state: State,
    current_extension: CrossingExtension,
    candidate: &CandidateMove<'_>,
    diagnostics: &mut MoveDiagnostics<'_>,
    stats: &mut RouteSearchStats,
    neighbor_loop_legality_time_us: &mut u128,
) -> Option<MoveVerdict> {
    let config = context.config;
    let dense_grid = context.dense_grid;
    let primitive = candidate.primitive;
    let profile = candidate.profile;
    let primitive_crossing = candidate.crossing;
    let primitive_class = candidate.class;

    if config.require_terminal_straights && state == context.source && !candidate.class_is_straight
    {
        return Some(MoveVerdict::Illegal(
            MoveRejection::TerminalStraightAtSource,
        ));
    }

    // Mirrors today's crossing kernel: a state already inside an
    // active post-crossing corridor may only continue with a
    // primitive that keeps heading the required direction and
    // either completes or correctly extends the required
    // straight run. This is checked here, before the hook is
    // even consulted, exactly like today's crossing kernel --
    // not solely inside the reused legality functions, one of
    // which (`crossing_no_contact_outcome`) does not perform
    // this check on its own.
    let pending_initial_run = primitive_initial_straight_run_distance(primitive, state.angle);
    let pending_completed_by_primitive = current_extension.pending_after_crossing_cells > 0
        && pending_initial_run + 1.0e-9
            >= f64::from(current_extension.pending_after_crossing_cells);
    if current_extension.pending_after_crossing_cells > 0 {
        if current_extension.pending_after_crossing_angle != state.angle {
            stats.crossing_reject_pending_straight += 1;
            return Some(MoveVerdict::Illegal(MoveRejection::PendingCorridorAngle));
        }
        if !pending_completed_by_primitive
            && !(candidate.class_is_straight
                && primitive.end_angle % 8 == state.angle
                && pending_initial_run > 0.0)
        {
            stats.crossing_reject_pending_straight += 1;
            return Some(MoveVerdict::Illegal(
                MoveRejection::PendingCorridorStraightRun,
            ));
        }
    }

    let next_x = state.x.checked_add(primitive.dx)?;
    let next_y = state.y.checked_add(primitive.dy)?;
    let next_angle = primitive.end_angle % 8;
    if config.require_terminal_straights
        && (next_x - context.target.x).abs() <= context.target_tolerance
        && (next_y - context.target.y).abs() <= context.target_tolerance
        && context.accepted_target_angles[next_angle as usize]
        && !candidate.class_is_straight
    {
        return Some(MoveVerdict::Illegal(
            MoveRejection::TerminalStraightAtTarget,
        ));
    }
    if !context.bounds.contains(next_x, next_y) {
        stats.window_rejects += 1;
        stats.primitive_bounds_rejects_by_class[primitive_class] += 1;
        return Some(MoveVerdict::Illegal(MoveRejection::OutOfBounds));
    }

    let next_state = State::new(next_x, next_y, next_angle);
    let ring_slot = if diagnostics.failure_diag {
        let slot = diagnostics.ring_index(next_x, next_y);
        if let Some(i) = slot {
            diagnostics.count_landing(i, 0);
        }
        slot
    } else {
        None
    };
    stats.primitive_footprint_checks += 1;
    stats.primitive_footprint_checks_by_class[primitive_class] += 1;
    stats.obstacle_clearance_checks += 1;
    let footprint_free = if config.collect_detailed_timing && config.diagnostics.hot_loop_timing {
        let legality_start = Instant::now();
        let footprint_free = dense_grid.primitive_footprint_free_with_profile(
            state.x,
            state.y,
            &primitive.footprint,
            profile,
            stats,
        );
        let legality_elapsed_us = legality_start.elapsed().as_micros();
        stats.legality_check_time_us += legality_elapsed_us;
        *neighbor_loop_legality_time_us += legality_elapsed_us;
        footprint_free
    } else {
        dense_grid.primitive_footprint_free_with_profile(
            state.x,
            state.y,
            &primitive.footprint,
            profile,
            stats,
        )
    };

    // Compact diagonal halo (see `.agent/WORKFLOW.md`): a one-cell-wide
    // diagonal piece can have a completely free footprint while a
    // committed route runs through the *adjacent* diagonal cells;
    // the realized bends of the two waveguides then overlap even
    // though no cell is shared. Such a move must not take the
    // fast path -- it goes to the legality hook, which legalizes
    // it as a crossing when crossings are enabled and rejects it
    // when they are not. Straights carry no halo, so this costs
    // nothing on the common path.
    let halo_free = !primitive_crossing.has_extra_witnesses
        || dense_grid.relative_offsets_free_with_profile(
            state.x,
            state.y,
            &primitive_crossing.extra_witness_offsets,
            &primitive_crossing.extra_witness_profile,
        );
    if !halo_free {
        stats.diagonal_halo_contacts += 1;
    }

    let legal_move = LegalMove {
        next_state,
        ring_slot,
        pending_completed_by_primitive,
    };

    if footprint_free && halo_free && current_extension.is_default() {
        // Tier 1: identical fast path to today's plain kernel --
        // dense array storage, no crossing bookkeeping, no hook call.
        return Some(MoveVerdict::Tier1(legal_move));
    }

    // Tier 2: this step is either genuinely blocked, or the
    // current state already carries active post-crossing
    // bookkeeping -- only here does the pluggable legality
    // hook get called.
    // Permanent, env-gated: PHOTONIC_ROUTER_MOVE_DIAG="x,y" names the
    // reject counter a move landing on that cell trips inside the hook.
    let move_diag_hit = diagnostics
        .move_diag_cell
        .is_some_and(|(dx, dy)| dx == next_x && dy == next_y);
    let diag_before = if move_diag_hit {
        Some(stats.clone())
    } else {
        None
    };
    let Some((outcome, extra_cost)) = context.hook.evaluate(
        state,
        current_extension,
        primitive,
        candidate.class_is_straight,
        primitive_crossing,
        footprint_free,
        stats,
    ) else {
        let search_seq = diagnostics.search_seq;
        if let Some(before) = diag_before {
            let changed = [
                (
                    "unexpected_owner",
                    before.crossing_reject_unexpected_owner,
                    stats.crossing_reject_unexpected_owner,
                ),
                (
                    "non_straight",
                    before.crossing_reject_non_straight,
                    stats.crossing_reject_non_straight,
                ),
                (
                    "unmatched_footprint",
                    before.crossing_reject_unmatched_footprint,
                    stats.crossing_reject_unmatched_footprint,
                ),
                (
                    "unmatched_centerline",
                    before.crossing_reject_unmatched_centerline,
                    stats.crossing_reject_unmatched_centerline,
                ),
                (
                    "not_perpendicular",
                    before.crossing_reject_not_perpendicular,
                    stats.crossing_reject_not_perpendicular,
                ),
                (
                    "margin",
                    before.crossing_reject_margin,
                    stats.crossing_reject_margin,
                ),
                (
                    "pending_straight",
                    before.crossing_reject_pending_straight,
                    stats.crossing_reject_pending_straight,
                ),
                (
                    "wrong_order",
                    before.crossing_reject_wrong_order,
                    stats.crossing_reject_wrong_order,
                ),
                (
                    "static",
                    before.crossing_hotpath_static_rejects,
                    stats.crossing_hotpath_static_rejects,
                ),
                (
                    "no_contact_path",
                    before.crossing_hotpath_no_contact,
                    stats.crossing_hotpath_no_contact,
                ),
                (
                    "accepted",
                    before.crossing_accepted,
                    stats.crossing_accepted,
                ),
            ];
            let deltas: Vec<String> = changed
                .iter()
                .filter(|(_, b, a)| a != b)
                .map(|(n, b, a)| format!("{}+{}", n, a - b))
                .collect();
            eprintln!("move-diag seq={} landing=({},{}) from=({},{},{}) prim={} straight={} footprint_free={} pending={} straight_run={} -> hook None; counters: {:?}",
                    search_seq, next_x, next_y, state.x, state.y, state.angle, primitive.id, candidate.class_is_straight, footprint_free,
                    current_extension.pending_after_crossing_cells, current_extension.straight_run_cells, deltas);
        }
        if !footprint_free {
            stats.footprint_rejects += 1;
            stats.primitive_footprint_rejects_by_class[primitive_class] += 1;
            if profile.is_full_rect {
                stats.primitive_footprint_rect_rejects += 1;
            }
            if let Some(i) = ring_slot {
                diagnostics.count_landing(i, 2);
                // Name the actual blocking cells of this
                // footprint (capped), so the seal is a fact,
                // not an interpretation.
                if *diagnostics.ring_blocker_lines.get(&i).unwrap_or(&0) < 3 {
                    let mut named = false;
                    for (dx, dy) in &primitive.footprint {
                        let cx = state.x + dx;
                        let cy = state.y + dy;
                        if dense_grid.is_blocked(cx, cy) {
                            *diagnostics.ring_blocker_lines.entry(i).or_insert(0) += 1;
                            named = true;
                            eprintln!(
                                    "search-failure-blocker landing=({},{}) from=({},{},{}) prim={} blocked_cell=({},{}) static={}",
                                    next_x, next_y, state.x, state.y, state.angle,
                                    primitive.id, cx, cy,
                                    diagnostics.obstacle_map.is_static_blocked(cx, cy),
                                );
                            break;
                        }
                    }
                    // No core cell blocked: the clearance
                    // profile did. Name the first blocked
                    // profile cell with its obstacle-map view.
                    if !named {
                        'profile: for dy in profile.min_dy..=profile.max_dy {
                            for dx in profile.min_dx..=profile.max_dx {
                                let cx = state.x + dx;
                                let cy = state.y + dy;
                                if dense_grid.is_blocked(cx, cy) {
                                    *diagnostics.ring_blocker_lines.entry(i).or_insert(0) += 1;
                                    let owners: Vec<NetId> = diagnostics
                                        .obstacle_map
                                        .dynamic_owners_at(cx, cy)
                                        .into_iter()
                                        .collect();
                                    eprintln!(
                                            "search-failure-blocker-profile landing=({},{}) from=({},{},{}) prim={} blocked_cell=({},{}) static={} opened={} owners={:?} core={}",
                                            next_x, next_y, state.x, state.y, state.angle,
                                            primitive.id, cx, cy,
                                            diagnostics.obstacle_map.is_static_blocked(cx, cy),
                                            diagnostics.port_open_cells.is_some_and(|open| open.contains(&pack_xy(cx, cy))),
                                            owners,
                                            diagnostics.obstacle_map.is_dynamic_core_blocked(cx, cy),
                                        );
                                    break 'profile;
                                }
                            }
                        }
                    }
                }
            }
            return Some(MoveVerdict::Illegal(MoveRejection::Footprint));
        } else if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 3);
        }
        return Some(MoveVerdict::Illegal(MoveRejection::Hook));
    };

    Some(MoveVerdict::Tier2(legal_move, outcome, extra_cost))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::obstacle_map::ObstacleMap;
    use crate::search::astar::config::AStarConfig;
    use crate::search::astar::route_single_net_with_config;
    use crate::search::test_support::*;

    #[test]
    fn footprint_collision_blocks_route() {
        let mut map = ObstacleMap::new(6, 3);
        map.add_static_cell(2, 1);

        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(3, 1, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                max_iterations: 200,
                bend_weight: 1.0,
                target_tolerance_cells: 0,
                ..AStarConfig::default()
            },
        );

        assert!(result.is_none());
    }

    use crate::primitives::PrimitiveLibrary;
    use crate::search::astar::crossing_rules::{primitive_crossing_metadata, NoCrossingHook};
    use crate::search::astar::heuristic::target_angle_acceptance;

    struct ExpansionTables {
        metadata: Vec<PrimitiveSearchMetadata>,
        profiles: Vec<FootprintCollisionProfile>,
        crossings: Vec<PrimitiveCrossingMetadata>,
    }

    fn expansion_tables(
        primitives: &PrimitiveLibrary,
        angle: u8,
        bend_weight: f64,
    ) -> ExpansionTables {
        let bucket = primitives.get_primitives_for_angle(angle);
        ExpansionTables {
            metadata: bucket
                .iter()
                .map(|primitive| PrimitiveSearchMetadata::from_primitive(primitive, bend_weight))
                .collect(),
            profiles: bucket
                .iter()
                .map(|primitive| FootprintCollisionProfile::from_footprint(&primitive.footprint))
                .collect(),
            crossings: bucket.iter().map(primitive_crossing_metadata).collect(),
        }
    }

    /// Every candidate move from `state`, paired with the verdict
    /// `move_is_legal` returns for it on `map`, crossings disabled.
    fn move_verdicts(
        map: &ObstacleMap,
        primitives: &PrimitiveLibrary,
        config: &AStarConfig,
        source: State,
        state: State,
        target: State,
    ) -> Vec<(u16, MoveVerdict)> {
        let bounds = RoutingBounds {
            min_x: 0,
            max_x: map.width() - 1,
            min_y: 0,
            max_y: map.height() - 1,
        };
        let dense_grid = DenseRoutingGrid::from_obstacle_map(
            map,
            bounds,
            None,
            config.max_dense_obstacle_cells,
            false,
            false,
            false,
        )
        .expect("dense grid fits");
        let hook = NoCrossingHook;
        let accepted_target_angles = target_angle_acceptance(target, config);
        let context = MoveContext {
            dense_grid: &dense_grid,
            config,
            hook: &hook,
            bounds,
            source,
            target,
            target_tolerance: config.target_tolerance_cells.max(0),
            accepted_target_angles: &accepted_target_angles,
        };
        let bucket = primitives.get_primitives_for_angle(state.angle);
        let tables = expansion_tables(primitives, state.angle, config.bend_weight);
        let mut target_ring = [[0u32; 7]; 25];
        let mut probe_ring: Vec<[u32; 7]> = Vec::new();
        let mut ring_blocker_lines = FxHashMap::default();
        let probe_index = FxHashMap::default();
        let mut stats = RouteSearchStats::default();
        let mut legality_time_us = 0u128;

        candidate_moves(
            bucket,
            &tables.metadata,
            &tables.profiles,
            &tables.crossings,
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        )
        .map(|candidate| {
            let mut diagnostics = MoveDiagnostics {
                failure_diag: false,
                move_diag_cell: None,
                search_seq: 0,
                obstacle_map: map,
                port_open_cells: None,
                target,
                probe_index: &probe_index,
                target_ring: &mut target_ring,
                probe_ring: &mut probe_ring,
                ring_blocker_lines: &mut ring_blocker_lines,
            };
            let verdict = move_is_legal(
                &context,
                state,
                CrossingExtension::default(),
                &candidate,
                &mut diagnostics,
                &mut stats,
                &mut legality_time_us,
            )
            .expect("no coordinate overflow");
            (candidate.primitive.id, verdict)
        })
        .collect()
    }

    #[test]
    fn candidate_moves_on_an_empty_grid_are_the_librarys_moves_in_iteration_order() {
        let primitives = primitive_library();
        let config = AStarConfig::default();
        let state = State::new(10, 10, 0);
        let source = State::new(0, 10, 0);
        let target = State::new(30, 30, 0);
        let bucket = primitives.get_primitives_for_angle(state.angle);
        let tables = expansion_tables(&primitives, state.angle, config.bend_weight);

        let (order, len) = primitive_iteration_order(
            bucket,
            &tables.metadata,
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        );
        let expected: Vec<u16> = order[..len].iter().map(|idx| bucket[*idx].id).collect();

        let moves: Vec<CandidateMove<'_>> = candidate_moves(
            bucket,
            &tables.metadata,
            &tables.profiles,
            &tables.crossings,
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        )
        .collect();

        assert_eq!(
            moves.iter().map(|m| m.primitive.id).collect::<Vec<u16>>(),
            expected
        );
        assert_eq!(moves.len(), bucket.len().min(8));
        for (move_, idx) in moves.iter().zip(order[..len].iter()) {
            assert_eq!(move_.class, tables.metadata[*idx].transition_class);
            assert_eq!(
                move_.class_is_straight,
                primitive_class_is_straight(move_.class)
            );
            assert_eq!(move_.profile.cell_count, bucket[*idx].footprint.len());
        }

        // On an empty grid every one of them is a legal Tier-1 move.
        let map = ObstacleMap::new(40, 40);
        let verdicts = move_verdicts(&map, &primitives, &config, source, state, target);
        assert_eq!(
            verdicts.iter().map(|(id, _)| *id).collect::<Vec<u16>>(),
            expected
        );
        assert!(verdicts
            .iter()
            .all(|(_, verdict)| matches!(verdict, MoveVerdict::Tier1(_))));
    }

    #[test]
    fn a_blocked_cell_rejects_exactly_the_moves_that_touch_it() {
        let primitives = primitive_library_no45_bend2();
        let config = AStarConfig::default();
        let state = State::new(10, 10, 0);
        let source = State::new(0, 10, 0);
        let target = State::new(30, 30, 0);
        let blocked = (12, 10);

        let mut map = ObstacleMap::new(40, 40);
        let empty_grid = move_verdicts(&map, &primitives, &config, source, state, target);
        assert!(empty_grid
            .iter()
            .all(|(_, verdict)| matches!(verdict, MoveVerdict::Tier1(_))));

        // The cells a move occupies: its footprint plus, for a move that
        // carries one, its compact diagonal halo.
        let tables = expansion_tables(&primitives, state.angle, config.bend_weight);
        let bucket = primitives.get_primitives_for_angle(state.angle);
        let mut touching: Vec<u16> = Vec::new();
        for (idx, primitive) in bucket.iter().enumerate() {
            let crossing = &tables.crossings[idx];
            let halo: &[(i32, i32)] = if crossing.has_extra_witnesses {
                &crossing.extra_witness_offsets
            } else {
                &[]
            };
            if primitive
                .footprint
                .iter()
                .chain(halo.iter())
                .any(|(dx, dy)| (state.x + dx, state.y + dy) == blocked)
            {
                touching.push(primitive.id);
            }
        }
        touching.sort_unstable();
        assert!(!touching.is_empty(), "the fixture must block some moves");

        map.add_static_cell(blocked.0, blocked.1);
        let with_blocker = move_verdicts(&map, &primitives, &config, source, state, target);
        let mut rejected: Vec<u16> = with_blocker
            .iter()
            .filter(|(_, verdict)| matches!(verdict, MoveVerdict::Illegal(_)))
            .map(|(id, _)| *id)
            .collect();
        rejected.sort_unstable();

        assert_eq!(rejected, touching);
        assert!(with_blocker.iter().all(|(id, verdict)| {
            match verdict {
                MoveVerdict::Tier1(_) => !touching.contains(id),
                // Crossings are disabled, so a contact can never be legalized.
                MoveVerdict::Illegal(MoveRejection::Footprint) => touching.contains(id),
                _ => false,
            }
        }));
    }

    #[test]
    fn the_terminal_straight_rule_rejects_a_bend_in_the_terminal_run() {
        let primitives = primitive_library();
        let config = AStarConfig {
            require_terminal_straights: true,
            target_tolerance_cells: 0,
            ..AStarConfig::default()
        };
        let map = ObstacleMap::new(40, 40);
        let source = State::new(10, 10, 0);
        let far_target = State::new(30, 30, 0);
        let bucket = primitives.get_primitives_for_angle(source.angle);
        let tables = expansion_tables(&primitives, source.angle, config.bend_weight);

        // Leaving the source: only a straight may start the route.
        let at_source = move_verdicts(&map, &primitives, &config, source, source, far_target);
        let mut bends_seen = 0;
        for (id, verdict) in &at_source {
            let idx = bucket.iter().position(|p| p.id == *id).expect("known id");
            if primitive_class_is_straight(tables.metadata[idx].transition_class) {
                assert!(matches!(verdict, MoveVerdict::Tier1(_)), "primitive {}", id);
            } else {
                bends_seen += 1;
                assert!(
                    matches!(
                        verdict,
                        MoveVerdict::Illegal(MoveRejection::TerminalStraightAtSource)
                    ),
                    "primitive {}",
                    id
                );
            }
        }
        assert!(bends_seen > 0, "the fixture must offer bends");

        // Entering the target: a bend that lands on it is rejected too,
        // from a state that is not the source.
        let state = State::new(20, 10, 0);
        let bend_idx = bucket
            .iter()
            .enumerate()
            .find(|(idx, _)| !primitive_class_is_straight(tables.metadata[*idx].transition_class))
            .map(|(idx, _)| idx)
            .expect("the fixture must offer bends");
        let bend = &bucket[bend_idx];
        let bend_target = State::new(state.x + bend.dx, state.y + bend.dy, bend.end_angle % 8);
        let at_target = move_verdicts(&map, &primitives, &config, source, state, bend_target);
        let (_, verdict) = at_target
            .iter()
            .find(|(id, _)| *id == bend.id)
            .expect("the bend is a candidate");
        assert!(matches!(
            verdict,
            MoveVerdict::Illegal(MoveRejection::TerminalStraightAtTarget)
        ));
    }
}
