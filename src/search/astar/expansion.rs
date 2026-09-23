//! Primitive-move expansion helpers: the footprint collision profile, the
//! primitive transition-class/ordering helpers used to iterate moves in a
//! consistent order, the target-biased primitive score, and the compact
//! diagonal halo used by the Tier-1 collision gate. Slice 3 of Milestone 3
//! adds the expansion functions extracted from the kernel loop. Moved out
//! of `src/astar.rs` (Milestone 3, Slice 2); pure code motion, no behaviour
//! change.

use crate::obstacle_map::{pack_xy, CellKey, NetId, ObstacleMap};
use crate::primitives::{Primitive, PrimitiveGeometry, PrimitiveLibrary, DIRECTIONS};
use crate::search::astar::config::{
    AStarConfig, PrimitiveOrdering, PRIMITIVE_BEND_45, PRIMITIVE_BEND_90, PRIMITIVE_STRAIGHT_LONG,
    PRIMITIVE_STRAIGHT_SHORT,
};
use crate::search::astar::cost::{self, PrimitiveSearchMetadata};
use crate::search::astar::crossing_rules::{
    extend_unique_keys, primitive_crossing_metadata,
    unified_candidate_hits_active_local_crossing_reservation,
    unified_outcome_windows_overlap_own_reservations, CrossingExtension, CrossingLegalityHook,
    CrossingMoveOutcome, PrimitiveCrossingMetadata,
};
use crate::search::astar::dense::{DenseRoutingGrid, DenseSearchStorage};
use crate::search::astar::heuristic::{distance_heuristic, SearchHeuristic};
use crate::search::astar::kernel::{
    chain_has_reservations_from, heap_tie_score, next_search_generation, OpenEntry, OpenSet,
    UnifiedExtendedNode, UnifiedOpenRef, UnifiedParentRef,
};
use crate::search::astar::window::RoutingBounds;
use crate::search::state::{RouteSearchStats, State};
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
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
    pub(crate) fn ring_index(&self, x: i32, y: i32) -> Option<usize> {
        let dx = x - self.target.x;
        let dy = y - self.target.y;
        if dx.abs() <= 2 && dy.abs() <= 2 {
            Some(((dy + 2) * 5 + (dx + 2)) as usize)
        } else {
            self.probe_index.get(&(x, y)).copied()
        }
    }

    pub(crate) fn count_landing(&mut self, ring_slot: usize, counter: usize) {
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
#[derive(Clone, Copy)]
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

/// Whether `state` lies inside the configured target tolerance box. Kept
/// separate from [`goal_reached`] because the kernel's goal-miss
/// diagnostic counts the states that reach the box but fail one of the
/// remaining goal conditions.
pub(crate) fn within_target_tolerance<H: CrossingLegalityHook>(
    context: &MoveContext<'_, H>,
    state: State,
) -> bool {
    (state.x - context.target.x).abs() <= context.target_tolerance
        && (state.y - context.target.y).abs() <= context.target_tolerance
}

/// The kernel's goal test for a popped state: inside the target tolerance
/// box, arriving at an accepted target angle, with no open post-crossing
/// debt, and accepted by the legality hook's own goal condition.
pub(crate) fn goal_reached<H: CrossingLegalityHook>(
    context: &MoveContext<'_, H>,
    state: State,
    extension: CrossingExtension,
) -> bool {
    within_target_tolerance(context, state)
        && context.accepted_target_angles[state.angle as usize]
        && extension.pending_after_crossing_cells == 0
        && context.hook.goal_extra_ok(extension)
}

/// The Tier-1 insert-or-improve on the dense arrays: the fast path a move
/// takes when the legality hook was never consulted (free footprint, free
/// halo, no crossing bookkeeping). Runs the closed check, the base-cost
/// lower-bound prune and the full-cost prune in the kernel's own order,
/// then writes the parent link, the g-cost, the straight-run side array
/// and the generation stamp before pushing the entry. Returns whether an
/// entry was pushed onto the open set; `None` is the kernel's generation
/// `?`, which abandons the whole search.
#[allow(clippy::too_many_arguments)]
pub(crate) fn push_or_improve_dense<H: CrossingLegalityHook>(
    context: &MoveContext<'_, H>,
    heuristic: &SearchHeuristic,
    storage: &mut DenseSearchStorage,
    open: &mut OpenSet,
    dense_straight_run: &mut [i32],
    counter: &mut u32,
    state: State,
    current_extension: CrossingExtension,
    current_g: f64,
    current_dense_idx: usize,
    candidate: &CandidateMove<'_>,
    legal: &LegalMove,
    diagnostics: &mut MoveDiagnostics<'_>,
    stats: &mut RouteSearchStats,
    neighbor_loop_heap_time_us: &mut u128,
) -> Option<bool> {
    let config = context.config;
    let primitive = candidate.primitive;
    let primitive_class = candidate.class;
    let next_state = legal.next_state;
    let ring_slot = legal.ring_slot;

    let next_idx = storage.in_bounds_parts_to_idx(next_state.x, next_state.y, next_state.angle);
    if storage.closed.get(next_idx) {
        stats.primitive_closed_rejects_by_class[primitive_class] += 1;
        if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 4);
        }
        return Some(false);
    }
    let base_step_cost = candidate.metadata.base_step_cost;
    let tentative_g_lower_bound = current_g + base_step_cost;
    if tentative_g_lower_bound >= storage.g_costs[next_idx] {
        stats.primitive_cost_pruned_by_class[primitive_class] += 1;
        if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 5);
        }
        return Some(false);
    }
    let step_cost = cost::step_cost(
        context.dense_grid,
        config,
        0.0,
        base_step_cost,
        state.x,
        state.y,
        &primitive.footprint,
        candidate.profile,
        primitive.start_angle,
        candidate.class_is_straight,
    );
    let tentative_g = current_g + step_cost;
    if tentative_g >= storage.g_costs[next_idx] {
        stats.primitive_cost_pruned_by_class[primitive_class] += 1;
        if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 5);
        }
        return Some(false);
    }
    stats.primitive_accepted_by_class[primitive_class] += 1;
    if let Some(i) = ring_slot {
        diagnostics.count_landing(i, 1);
    }
    storage.parent_idx[next_idx] = current_dense_idx as u32;
    storage.parent_primitive[next_idx] = primitive.id;
    storage.g_costs[next_idx] = tentative_g;
    if H::TRACKS_STRAIGHT_RUN {
        let capped_required_margin = context.hook.capped_required_margin();
        dense_straight_run[next_idx] = if candidate.class_is_straight {
            current_extension
                .straight_run_cells
                .saturating_add(primitive.dx.abs().max(primitive.dy.abs()))
                .min(capped_required_margin)
        } else {
            primitive_terminal_straight_run_cells(primitive, primitive.end_angle)
                .min(capped_required_margin)
        };
    }
    let generation = next_search_generation(counter)?;
    storage.best_generation[next_idx] = generation;
    stats.best_cost_updates += 1;
    stats.parent_updates += 1;
    let heap_start =
        (config.collect_detailed_timing && config.diagnostics.hot_loop_timing).then(Instant::now);
    let queued = open.push(OpenEntry {
        f_score: tentative_g + cost::heuristic_estimate(heuristic, next_state),
        tie_score: heap_tie_score(tentative_g, config.heap_tie_breaker),
        g_score: tentative_g,
        counter: generation,
        generation,
        idx: next_idx,
    });
    if let Some(heap_start) = heap_start {
        let heap_elapsed_us = heap_start.elapsed().as_micros();
        stats.heap_operation_time_us += heap_elapsed_us;
        *neighbor_loop_heap_time_us += heap_elapsed_us;
    }
    if queued {
        stats.heap_pushes += 1;
        stats.max_heap_size = stats.max_heap_size.max(open.len());
    }
    Some(queued)
}

/// The Tier-2 insert-or-improve for a move the legality hook accepted
/// whose resulting state carries no open post-crossing debt: the closed
/// check and the cost prune against the `(state, extension)` bookkeeping
/// map, the two own-reservation guards, then the new extended node with
/// its promoted reservation keys and the Tier-2 heap push. Returns whether
/// an entry was pushed; `None` is the kernel's generation `?`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn push_or_improve_extended<H: CrossingLegalityHook>(
    context: &MoveContext<'_, H>,
    heuristic: &SearchHeuristic,
    extended_nodes: &mut Vec<UnifiedExtendedNode>,
    extended_state: &mut FxHashMap<(State, CrossingExtension), (f64, bool)>,
    tier2_open: &mut BinaryHeap<OpenEntry>,
    tier1_open_len: usize,
    counter: &mut u32,
    state: State,
    current_extension: CrossingExtension,
    current_g: f64,
    current_ref: UnifiedOpenRef,
    candidate: &CandidateMove<'_>,
    legal: &LegalMove,
    next_extension: CrossingExtension,
    outcome: &CrossingMoveOutcome,
    extra_cost: f64,
    best_crossings: &mut u16,
    best_crossing_ref: &mut Option<usize>,
    diagnostics: &mut MoveDiagnostics<'_>,
    stats: &mut RouteSearchStats,
    neighbor_loop_heap_time_us: &mut u128,
) -> Option<bool> {
    let config = context.config;
    let primitive = candidate.primitive;
    let primitive_class = candidate.class;
    let next_state = legal.next_state;
    let ring_slot = legal.ring_slot;
    let failure_diag = diagnostics.failure_diag;

    let key = (next_state, next_extension);
    let existing_bookkeeping = extended_state.get(&key).copied();
    if existing_bookkeeping.is_some_and(|(_, closed)| closed) {
        stats.primitive_closed_rejects_by_class[primitive_class] += 1;
        if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 4);
        }
        return Some(false);
    }
    let step_cost = cost::step_cost(
        context.dense_grid,
        config,
        0.0,
        candidate.metadata.base_step_cost,
        state.x,
        state.y,
        &primitive.footprint,
        candidate.profile,
        primitive.start_angle,
        candidate.class_is_straight,
    ) + extra_cost;
    let tentative_g = current_g + step_cost;
    let best_next_g = existing_bookkeeping.map(|(g, _)| g);
    if tentative_g >= best_next_g.unwrap_or(f64::INFINITY) {
        stats.primitive_cost_pruned_by_class[primitive_class] += 1;
        if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 5);
        }
        return Some(false);
    }
    if unified_candidate_hits_active_local_crossing_reservation(
        current_ref,
        state,
        candidate.crossing,
        extended_nodes,
    ) {
        stats.footprint_rejects += 1;
        stats.primitive_footprint_rejects_by_class[primitive_class] += 1;
        if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 6);
        }
        return Some(false);
    }
    if unified_outcome_windows_overlap_own_reservations(current_ref, outcome, extended_nodes) {
        stats.crossing_reject_reservation_overlap += 1;
        if let Some(i) = ring_slot {
            diagnostics.count_landing(i, 6);
        }
        return Some(false);
    }

    let parent = match current_ref {
        UnifiedOpenRef::Dense(idx) => UnifiedParentRef::Dense(idx),
        UnifiedOpenRef::Extended(ext_idx) => UnifiedParentRef::Extended(ext_idx),
    };
    let node_pending_local_reservation_keys: &[CellKey] = match current_ref {
        UnifiedOpenRef::Dense(_) => &[],
        UnifiedOpenRef::Extended(ext_idx) => {
            &extended_nodes[ext_idx].pending_local_reservation_keys
        }
    };
    let mut active_local_reservation_keys =
        Vec::with_capacity(outcome.active_reservation_keys.len());
    let mut pending_local_reservation_keys =
        Vec::with_capacity(outcome.pending_reservation_keys.len());
    if current_extension.pending_after_crossing_cells > 0 && legal.pending_completed_by_primitive {
        active_local_reservation_keys.extend(node_pending_local_reservation_keys.iter().copied());
    } else if current_extension.pending_after_crossing_cells > 0
        && outcome.pending_after_crossing_cells > 0
    {
        pending_local_reservation_keys.extend(node_pending_local_reservation_keys.iter().copied());
    }
    extend_unique_keys(
        &mut active_local_reservation_keys,
        &outcome.active_reservation_keys,
    );
    extend_unique_keys(
        &mut pending_local_reservation_keys,
        &outcome.pending_reservation_keys,
    );

    let node_crossings = if failure_diag {
        let parent_crossings = match current_ref {
            UnifiedOpenRef::Dense(_) => 0u16,
            UnifiedOpenRef::Extended(ext_idx) => extended_nodes[ext_idx].crossings,
        };
        parent_crossings.saturating_add(u16::try_from(outcome.crossing_count).unwrap_or(0))
    } else {
        0
    };
    let node_idx = extended_nodes.len();
    let node_chain_has_reservations = chain_has_reservations_from(
        parent,
        &active_local_reservation_keys,
        &pending_local_reservation_keys,
        extended_nodes,
    );
    extended_nodes.push(UnifiedExtendedNode {
        state: next_state,
        extension: next_extension,
        parent,
        primitive_id: primitive.id,
        g_score: tentative_g,
        active_local_reservation_keys,
        pending_local_reservation_keys,
        crossings: node_crossings,
        chain_has_reservations: node_chain_has_reservations,
    });
    if failure_diag && (node_crossings > *best_crossings || best_crossing_ref.is_none()) {
        *best_crossings = node_crossings;
        *best_crossing_ref = Some(node_idx);
    }
    extended_state.insert(key, (tentative_g, false));
    stats.primitive_accepted_by_class[primitive_class] += 1;
    if let Some(i) = ring_slot {
        diagnostics.count_landing(i, 1);
    }
    stats.best_cost_updates += 1;
    stats.parent_updates += 1;
    let generation = next_search_generation(counter)?;
    let heap_start =
        (config.collect_detailed_timing && config.diagnostics.hot_loop_timing).then(Instant::now);
    tier2_open.push(OpenEntry {
        f_score: tentative_g
            + cost::heuristic_estimate(heuristic, next_state)
            + context.hook.heuristic_bonus(next_state, next_extension),
        tie_score: heap_tie_score(tentative_g, config.heap_tie_breaker),
        g_score: tentative_g,
        counter: generation,
        generation,
        idx: node_idx,
    });
    if let Some(heap_start) = heap_start {
        let heap_elapsed_us = heap_start.elapsed().as_micros();
        stats.heap_operation_time_us += heap_elapsed_us;
        *neighbor_loop_heap_time_us += heap_elapsed_us;
    }
    stats.heap_pushes += 1;
    stats.max_heap_size = stats.max_heap_size.max(tier1_open_len + tier2_open.len());
    Some(true)
}

/// Every per-angle primitive table one search reads: the library's own
/// bucket per angle plus the search metadata, footprint collision profile
/// and crossing metadata derived from it once per search, and the
/// angle-preserving straight primitives the eager post-crossing
/// completion chain steps with.
pub(crate) struct PrimitiveTables<'a> {
    pub(crate) buckets: [&'a [Primitive]; 8],
    pub(crate) metadata: Vec<Vec<PrimitiveSearchMetadata>>,
    pub(crate) profiles: Vec<Vec<FootprintCollisionProfile>>,
    pub(crate) crossings: Vec<Vec<PrimitiveCrossingMetadata>>,
    /// Angle-preserving straight primitives per angle, longest first,
    /// used by the eager post-crossing completion chain. Longest-
    /// fitting-first matches the decomposition the old pending-state
    /// search preferred (per-primitive history/congestion costs make
    /// many short steps slightly pricier than one long straight over
    /// the same cells).
    pub(crate) straights_per_angle: [Vec<(usize, i32)>; 8],
}

impl<'a> PrimitiveTables<'a> {
    pub(crate) fn build(primitives: &'a PrimitiveLibrary, bend_weight: f64) -> Self {
        let buckets: [&[Primitive]; 8] =
            std::array::from_fn(|angle| primitives.get_primitives_for_angle(angle as u8));
        let metadata: Vec<Vec<PrimitiveSearchMetadata>> = buckets
            .iter()
            .map(|bucket| {
                bucket
                    .iter()
                    .map(|primitive| {
                        PrimitiveSearchMetadata::from_primitive(primitive, bend_weight)
                    })
                    .collect()
            })
            .collect();
        let profiles: Vec<Vec<FootprintCollisionProfile>> = buckets
            .iter()
            .map(|bucket| {
                bucket
                    .iter()
                    .map(|primitive| {
                        FootprintCollisionProfile::from_footprint(&primitive.footprint)
                    })
                    .collect()
            })
            .collect();
        let crossings: Vec<Vec<PrimitiveCrossingMetadata>> = buckets
            .iter()
            .map(|bucket| bucket.iter().map(primitive_crossing_metadata).collect())
            .collect();
        let straights_per_angle: [Vec<(usize, i32)>; 8] = std::array::from_fn(|angle| {
            let mut straights: Vec<(usize, i32)> = buckets[angle]
                .iter()
                .enumerate()
                .filter(|(idx, primitive)| {
                    primitive.end_angle % 8 == angle as u8
                        && primitive_class_is_straight(metadata[angle][*idx].transition_class)
                })
                .map(|(idx, primitive)| (idx, primitive.dx.abs().max(primitive.dy.abs())))
                .collect();
            straights.sort_by_key(|(_, cells)| std::cmp::Reverse(*cells));
            straights
        });
        Self {
            buckets,
            metadata,
            profiles,
            crossings,
            straights_per_angle,
        }
    }
}

/// The end of a completed eager post-crossing corridor: the last node the
/// chain appended to the extended-node arena, and the state, extension and
/// running g-cost the kernel's Tier-2 insert keys it under.
pub(crate) struct CompletedCorridor {
    pub(crate) last_idx: usize,
    pub(crate) state: State,
    pub(crate) extension: CrossingExtension,
    pub(crate) g_score: f64,
}

/// The eager post-crossing completion chain. Appends the accepted crossing
/// move plus the forced straight steps that pay off its
/// min-straight-after debt to the extended-node arena as parent
/// scaffolding, and returns the chain's end for the kernel's Tier-2
/// insert. `None` rejects the whole move (the counter for the rejecting
/// branch has already been incremented, and any nodes the abandoned chain
/// appended stay in the arena, unreferenced -- exactly as the inline
/// version left them).
#[allow(clippy::too_many_arguments)]
pub(crate) fn complete_crossing_corridor<H: CrossingLegalityHook>(
    context: &MoveContext<'_, H>,
    tables: &PrimitiveTables<'_>,
    extended_nodes: &mut Vec<UnifiedExtendedNode>,
    state: State,
    current_g: f64,
    current_ref: UnifiedOpenRef,
    candidate: &CandidateMove<'_>,
    legal: &LegalMove,
    next_extension: CrossingExtension,
    outcome: &CrossingMoveOutcome,
    extra_cost: f64,
    chain_diag: bool,
    diagnostics: &mut MoveDiagnostics<'_>,
    stats: &mut RouteSearchStats,
) -> Option<CompletedCorridor> {
    let config = context.config;
    let dense_grid = context.dense_grid;
    let hook = context.hook;
    let bounds = context.bounds;
    let primitive = candidate.primitive;
    let profile = candidate.profile;
    let primitive_crossing = candidate.crossing;
    let primitive_class = candidate.class;
    let next_state = legal.next_state;
    let search_seq = diagnostics.search_seq;
    let failure_diag = diagnostics.failure_diag;

    // Eager post-crossing completion: a crossing whose
    // min-straight-after debt is still open never becomes
    // a search state. Debt-carrying keys multiplied per
    // (pending cells x angle x partner index) and caused
    // the crossing-state explosion diagnosed on
    // multiportmmi_32x32 net 156 (~63 key variants per
    // cell). The debt's only legal continuation is the
    // straight run itself, so complete it immediately
    // with forced shortest-straight steps -- each
    // validated through the same legality hook, which
    // already consumes pending in both its contact and
    // no-contact paths and legalizes any NEW crossings
    // the completion run hits (the dense-bundle case,
    // where the debt chains across several partners).
    // Intermediate nodes are parent scaffolding only:
    // never keyed in `extended_state`, never heaped.
    // An illegal completion abandons the whole move --
    // the same rejection the old per-state bookkeeping
    // produced, just before any states are spawned.
    const EAGER_CHAIN_MAX_STEPS: usize = 512;
    if unified_candidate_hits_active_local_crossing_reservation(
        current_ref,
        state,
        primitive_crossing,
        extended_nodes,
    ) {
        stats.footprint_rejects += 1;
        stats.primitive_footprint_rejects_by_class[primitive_class] += 1;
        return None;
    }
    if unified_outcome_windows_overlap_own_reservations(current_ref, outcome, extended_nodes) {
        stats.crossing_reject_reservation_overlap += 1;
        return None;
    }
    let mut chain_g = cost::step_cost(
        dense_grid,
        config,
        current_g,
        candidate.metadata.base_step_cost,
        state.x,
        state.y,
        &primitive.footprint,
        profile,
        primitive.start_angle,
        candidate.class_is_straight,
    ) + extra_cost;
    let first_parent = match current_ref {
        UnifiedOpenRef::Dense(idx) => UnifiedParentRef::Dense(idx),
        UnifiedOpenRef::Extended(ext_idx) => UnifiedParentRef::Extended(ext_idx),
    };
    // Keyed states never carry open debt any more, so the
    // first move needs no promotion from the current
    // node's pending keys -- only the outcome's own keys.
    let mut first_active_keys = Vec::with_capacity(outcome.active_reservation_keys.len());
    let mut first_pending_keys = Vec::with_capacity(outcome.pending_reservation_keys.len());
    extend_unique_keys(&mut first_active_keys, &outcome.active_reservation_keys);
    extend_unique_keys(&mut first_pending_keys, &outcome.pending_reservation_keys);
    let first_crossings = if failure_diag {
        let parent_crossings = match current_ref {
            UnifiedOpenRef::Dense(_) => 0u16,
            UnifiedOpenRef::Extended(ext_idx) => extended_nodes[ext_idx].crossings,
        };
        parent_crossings.saturating_add(u16::try_from(outcome.crossing_count).unwrap_or(0))
    } else {
        0
    };
    let mut last_idx = extended_nodes.len();
    let first_chain_has_reservations = chain_has_reservations_from(
        first_parent,
        &first_active_keys,
        &first_pending_keys,
        extended_nodes,
    );
    extended_nodes.push(UnifiedExtendedNode {
        state: next_state,
        extension: next_extension,
        parent: first_parent,
        primitive_id: primitive.id,
        g_score: chain_g,
        active_local_reservation_keys: first_active_keys,
        pending_local_reservation_keys: first_pending_keys,
        crossings: first_crossings,
        chain_has_reservations: first_chain_has_reservations,
    });
    let mut chain_state = next_state;
    let mut chain_extension = next_extension;
    let mut chain_completed = true;
    let mut chain_steps = 0usize;
    while chain_extension.pending_after_crossing_cells > 0 {
        chain_steps += 1;
        let mut step_footprint_free_diag = true;
        if chain_steps > EAGER_CHAIN_MAX_STEPS {
            if chain_diag {
                eprintln!("chain-break seq={} reason=max_steps state=({},{},{}) pending={} step={} footprint_free={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps, step_footprint_free_diag);
            }
            chain_completed = false;
            break;
        }
        let pending_angle = chain_extension.pending_after_crossing_angle as usize;
        if pending_angle >= 8 || chain_state.angle as usize != pending_angle {
            if chain_diag {
                eprintln!("chain-break seq={} reason=angle_mismatch state=({},{},{}) pending={} step={} footprint_free={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps, step_footprint_free_diag);
            }
            chain_completed = false;
            break;
        }
        let remaining_debt = chain_extension.pending_after_crossing_cells;
        let Some(&(step_idx, _)) = tables.straights_per_angle[pending_angle]
            .iter()
            .find(|(_, cells)| *cells <= remaining_debt)
        else {
            if chain_diag {
                eprintln!("chain-break seq={} reason=no_fitting_straight state=({},{},{}) pending={} step={} footprint_free={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps, step_footprint_free_diag);
            }
            chain_completed = false;
            break;
        };
        let step_primitive = &tables.buckets[pending_angle][step_idx];
        let step_metadata = tables.metadata[pending_angle][step_idx];
        let step_profile = &tables.profiles[pending_angle][step_idx];
        let step_crossing = &tables.crossings[pending_angle][step_idx];
        let (Some(step_x), Some(step_y)) = (
            chain_state.x.checked_add(step_primitive.dx),
            chain_state.y.checked_add(step_primitive.dy),
        ) else {
            if chain_diag {
                eprintln!("chain-break seq={} reason=checked_add_overflow state=({},{},{}) pending={} step={} footprint_free={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps, step_footprint_free_diag);
            }
            chain_completed = false;
            break;
        };
        if !bounds.contains(step_x, step_y) {
            stats.window_rejects += 1;
            if chain_diag {
                eprintln!("chain-break seq={} reason=out_of_window state=({},{},{}) pending={} step={} footprint_free={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps, step_footprint_free_diag);
            }
            chain_completed = false;
            break;
        }
        stats.primitive_footprint_checks += 1;
        stats.obstacle_clearance_checks += 1;
        let step_footprint_free = dense_grid.primitive_footprint_free_with_profile(
            chain_state.x,
            chain_state.y,
            &step_primitive.footprint,
            step_profile,
            stats,
        );
        step_footprint_free_diag = step_footprint_free;
        let step_pending_completed =
            primitive_initial_straight_run_distance(step_primitive, chain_state.angle) + 1.0e-9
                >= f64::from(chain_extension.pending_after_crossing_cells);
        let Some((step_outcome, step_extra_cost)) = hook.evaluate(
            chain_state,
            chain_extension,
            step_primitive,
            true,
            step_crossing,
            step_footprint_free,
            stats,
        ) else {
            if let Some(i) = diagnostics.ring_index(step_x, step_y) {
                diagnostics.count_landing(i, if step_footprint_free { 3 } else { 2 });
            }
            if chain_diag {
                eprintln!("chain-break seq={} reason=hook_none state=({},{},{}) pending={} step={} footprint_free={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps, step_footprint_free_diag);
            }
            chain_completed = false;
            break;
        };
        if unified_candidate_hits_active_local_crossing_reservation(
            UnifiedOpenRef::Extended(last_idx),
            chain_state,
            step_crossing,
            extended_nodes,
        ) {
            if let Some(i) = diagnostics.ring_index(step_x, step_y) {
                diagnostics.count_landing(i, 6);
            }
            if chain_diag {
                eprintln!("chain-break seq={} reason=reservation_hit state=({},{},{}) pending={} step={} footprint_free={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps, step_footprint_free_diag);
            }
            chain_completed = false;
            break;
        }
        if unified_outcome_windows_overlap_own_reservations(
            UnifiedOpenRef::Extended(last_idx),
            &step_outcome,
            extended_nodes,
        ) {
            stats.crossing_reject_reservation_overlap += 1;
            if chain_diag {
                eprintln!("chain-break seq={} reason=reservation_overlap state=({},{},{}) pending={} step={}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_extension.pending_after_crossing_cells, chain_steps);
            }
            chain_completed = false;
            break;
        }
        chain_g += cost::step_cost(
            dense_grid,
            config,
            0.0,
            step_metadata.base_step_cost,
            chain_state.x,
            chain_state.y,
            &step_primitive.footprint,
            step_profile,
            step_primitive.start_angle,
            true,
        ) + step_extra_cost;
        let step_extension = CrossingExtension::from_outcome(&step_outcome);
        // Reservation-key promotion, mirroring the keyed
        // path: keys booked while the debt was open
        // become active once the debt completes, and
        // stay pending while it is still open.
        let prev_pending_keys = extended_nodes[last_idx]
            .pending_local_reservation_keys
            .clone();
        let mut step_active_keys = Vec::with_capacity(step_outcome.active_reservation_keys.len());
        let mut step_pending_keys = Vec::with_capacity(step_outcome.pending_reservation_keys.len());
        if step_pending_completed {
            step_active_keys.extend(prev_pending_keys);
        } else {
            step_pending_keys.extend(prev_pending_keys);
        }
        extend_unique_keys(&mut step_active_keys, &step_outcome.active_reservation_keys);
        extend_unique_keys(
            &mut step_pending_keys,
            &step_outcome.pending_reservation_keys,
        );
        let step_state = State::new(step_x, step_y, step_primitive.end_angle % 8);
        let step_crossings = if failure_diag {
            extended_nodes[last_idx]
                .crossings
                .saturating_add(u16::try_from(step_outcome.crossing_count).unwrap_or(0))
        } else {
            0
        };
        let new_idx = extended_nodes.len();
        let step_chain_has_reservations = chain_has_reservations_from(
            UnifiedParentRef::Extended(last_idx),
            &step_active_keys,
            &step_pending_keys,
            extended_nodes,
        );
        extended_nodes.push(UnifiedExtendedNode {
            state: step_state,
            extension: step_extension,
            parent: UnifiedParentRef::Extended(last_idx),
            primitive_id: step_primitive.id,
            g_score: chain_g,
            active_local_reservation_keys: step_active_keys,
            pending_local_reservation_keys: step_pending_keys,
            crossings: step_crossings,
            chain_has_reservations: step_chain_has_reservations,
        });
        last_idx = new_idx;
        chain_state = step_state;
        chain_extension = step_extension;
    }
    if !chain_completed {
        stats.crossing_reject_pending_straight += 1;
        return None;
    }
    Some(CompletedCorridor {
        last_idx,
        state: chain_state,
        extension: chain_extension,
        g_score: chain_g,
    })
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
    use crate::search::astar::crossing_rules::{
        primitive_crossing_metadata, CrossingHookContext, CrossingSearchConfig,
        CrossingSearchPartner, LiveCrossingHook, NoCrossingHook,
    };
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

    // ---------------------------------------------------------------
    // Milestone 3, Slice 3 step 4: the goal test, the two
    // insert-or-improve paths and the eager completion chain.
    // ---------------------------------------------------------------

    fn slice3_config() -> AStarConfig {
        AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            target_tolerance_cells: 1,
            require_terminal_straights: false,
            ..AStarConfig::default()
        }
    }

    fn full_map_bounds(map: &ObstacleMap) -> RoutingBounds {
        RoutingBounds {
            min_x: 0,
            max_x: map.width() - 1,
            min_y: 0,
            max_y: map.height() - 1,
        }
    }

    fn plain_dense_grid(
        map: &ObstacleMap,
        bounds: RoutingBounds,
        config: &AStarConfig,
    ) -> DenseRoutingGrid {
        DenseRoutingGrid::from_obstacle_map(
            map,
            bounds,
            None,
            config.max_dense_obstacle_cells,
            false,
            false,
            false,
        )
        .expect("dense grid fits")
    }

    /// The diagnostics sink the step-4 tests hand to the extracted
    /// functions: every counter on, no probe cells, nothing printed.
    struct TestRings {
        target_ring: [[u32; 7]; 25],
        probe_ring: Vec<[u32; 7]>,
        ring_blocker_lines: FxHashMap<usize, u32>,
        probe_index: FxHashMap<(i32, i32), usize>,
    }

    impl TestRings {
        fn new() -> Self {
            Self {
                target_ring: [[0u32; 7]; 25],
                probe_ring: Vec::new(),
                ring_blocker_lines: FxHashMap::default(),
                probe_index: FxHashMap::default(),
            }
        }

        fn diagnostics<'a>(
            &'a mut self,
            map: &'a ObstacleMap,
            target: State,
        ) -> MoveDiagnostics<'a> {
            MoveDiagnostics {
                failure_diag: false,
                move_diag_cell: None,
                search_seq: 0,
                obstacle_map: map,
                port_open_cells: None,
                target,
                probe_index: &self.probe_index,
                target_ring: &mut self.target_ring,
                probe_ring: &mut self.probe_ring,
                ring_blocker_lines: &mut self.ring_blocker_lines,
            }
        }
    }

    #[test]
    fn goal_reached_respects_the_tolerance_box_and_the_angle_mask() {
        let map = ObstacleMap::new(24, 24);
        let config = slice3_config();
        let bounds = full_map_bounds(&map);
        let grid = plain_dense_grid(&map, bounds, &config);
        let hook = NoCrossingHook;
        let source = State::new(2, 12, 0);
        let target = State::new(12, 12, 0);
        let accepted = target_angle_acceptance(target, &config);
        let context = MoveContext {
            dense_grid: &grid,
            config: &config,
            hook: &hook,
            bounds,
            source,
            target,
            target_tolerance: config.target_tolerance_cells.max(0),
            accepted_target_angles: &accepted,
        };
        let default_extension = CrossingExtension::default();

        // On the target, at the target angle: the goal.
        assert!(goal_reached(&context, target, default_extension));
        // One cell away in x and in y: still inside a tolerance of 1.
        let near = State::new(13, 13, 0);
        assert!(within_target_tolerance(&context, near));
        assert!(goal_reached(&context, near, default_extension));
        // Two cells away: outside the tolerance box.
        let far = State::new(14, 12, 0);
        assert!(!within_target_tolerance(&context, far));
        assert!(!goal_reached(&context, far, default_extension));
        // Inside the box, but arriving at an angle the mask rejects.
        let rejected_angle = (0..8u8)
            .find(|angle| !accepted[*angle as usize])
            .expect("the mask must reject at least one angle");
        let wrong_angle = State::new(target.x, target.y, rejected_angle);
        assert!(within_target_tolerance(&context, wrong_angle));
        assert!(!goal_reached(&context, wrong_angle, default_extension));
        // On the goal cell and angle, but still owing a straight run.
        let owing = CrossingExtension {
            pending_after_crossing_cells: 3,
            ..CrossingExtension::default()
        };
        assert!(!goal_reached(&context, target, owing));
    }

    #[test]
    fn push_or_improve_dense_keeps_the_better_g_and_ignores_a_worse_one() {
        let map = ObstacleMap::new(24, 24);
        let config = slice3_config();
        let bounds = full_map_bounds(&map);
        let grid = plain_dense_grid(&map, bounds, &config);
        let hook = NoCrossingHook;
        let source = State::new(2, 12, 0);
        let target = State::new(20, 12, 0);
        let accepted = target_angle_acceptance(target, &config);
        let context = MoveContext {
            dense_grid: &grid,
            config: &config,
            hook: &hook,
            bounds,
            source,
            target,
            target_tolerance: config.target_tolerance_cells.max(0),
            accepted_target_angles: &accepted,
        };
        let primitives = primitive_library();
        let state = State::new(10, 12, 0);
        let tables = expansion_tables(&primitives, state.angle, config.bend_weight);
        let bucket = primitives.get_primitives_for_angle(state.angle);
        let mut stats = RouteSearchStats::default();
        let mut rings = TestRings::new();

        let mut storage =
            DenseSearchStorage::new(bounds, config.max_dense_states).expect("dense storage fits");
        let heuristic = SearchHeuristic::new(target, &primitives, &config);
        let mut open = OpenSet::new(config.use_indexed_heap, storage.state_count());
        let mut straight_run: Vec<i32> = Vec::new();
        let mut counter = 0u32;
        let mut heap_time_us = 0u128;

        let candidate = candidate_moves(
            bucket,
            &tables.metadata,
            &tables.profiles,
            &tables.crossings,
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        )
        .find(|candidate| candidate.class_is_straight)
        .expect("the fixture offers a straight move");
        let mut diagnostics = rings.diagnostics(&map, target);
        let MoveVerdict::Tier1(legal) = move_is_legal(
            &context,
            state,
            CrossingExtension::default(),
            &candidate,
            &mut diagnostics,
            &mut stats,
            &mut heap_time_us,
        )
        .expect("no coordinate overflow") else {
            panic!("crossings are disabled, so the move lands in Tier 1");
        };
        let next_idx = storage.in_bounds_parts_to_idx(
            legal.next_state.x,
            legal.next_state.y,
            legal.next_state.angle,
        );

        // First insert from a parent at g = 100: accepted and pushed.
        let pushed = push_or_improve_dense(
            &context,
            &heuristic,
            &mut storage,
            &mut open,
            &mut straight_run,
            &mut counter,
            state,
            CrossingExtension::default(),
            100.0,
            7,
            &candidate,
            &legal,
            &mut diagnostics,
            &mut stats,
            &mut heap_time_us,
        )
        .expect("no generation overflow");
        assert!(pushed);
        let improved_g = storage.g_costs[next_idx];
        assert!(improved_g.is_finite());
        assert_eq!(storage.parent_idx[next_idx], 7);
        assert_eq!(storage.parent_primitive[next_idx], candidate.primitive.id);
        assert_eq!(stats.heap_pushes, 1);
        let pushes_after_first = stats.heap_pushes;

        // A strictly worse parent cost leaves the entry untouched.
        let worse = push_or_improve_dense(
            &context,
            &heuristic,
            &mut storage,
            &mut open,
            &mut straight_run,
            &mut counter,
            state,
            CrossingExtension::default(),
            1000.0,
            9,
            &candidate,
            &legal,
            &mut diagnostics,
            &mut stats,
            &mut heap_time_us,
        )
        .expect("no generation overflow");
        assert!(!worse);
        assert_eq!(storage.g_costs[next_idx], improved_g);
        assert_eq!(storage.parent_idx[next_idx], 7);
        assert_eq!(stats.heap_pushes, pushes_after_first);

        // A strictly better parent cost improves g and re-parents.
        let better = push_or_improve_dense(
            &context,
            &heuristic,
            &mut storage,
            &mut open,
            &mut straight_run,
            &mut counter,
            state,
            CrossingExtension::default(),
            10.0,
            11,
            &candidate,
            &legal,
            &mut diagnostics,
            &mut stats,
            &mut heap_time_us,
        )
        .expect("no generation overflow");
        assert!(better);
        assert!(storage.g_costs[next_idx] < improved_g);
        assert_eq!(storage.parent_idx[next_idx], 11);
        assert_eq!(stats.heap_pushes, pushes_after_first + 1);
    }

    /// One horizontal partner net across the middle of the map, for a
    /// route descending vertically through it -- the same shape as
    /// `crossing_rules::tests::vertical_descent_across_two_horizontals`,
    /// reduced to a single partner.
    fn crossing_corridor_fixture() -> (
        ObstacleMap,
        PrimitiveLibrary,
        AStarConfig,
        CrossingSearchConfig,
    ) {
        let mut map = ObstacleMap::new(40, 60);
        let partner_y = 30;
        let partner: Vec<(i32, i32)> = (0..40).map(|k| (k, partner_y)).collect();
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            1,
            &partner,
            &partner,
            &[],
            &FxHashSet::default()
        ));
        let crossing = CrossingSearchConfig {
            diagnostics: KernelDiagnostics::default(),
            net_id: 2,
            partners: vec![CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(0, partner_y), (39, partner_y)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            }],
            min_straight_cells: 2,
            crossing_half_size_cells: 2,
            bend_runout_cells: 3,
            crossing_loss: 1.0,
            require_all_partners: false,
            terminal_bump_guard: None,
        };
        let config = AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            require_terminal_straights: false,
            ..AStarConfig::default()
        };
        (map, primitive_library(), config, crossing)
    }

    /// The first straight move, scanning the descent towards the partner,
    /// whose hook outcome leaves a post-crossing straight-run debt open.
    fn first_pending_crossing_move<'a, 'b>(
        context: &MoveContext<'_, LiveCrossingHook<'_>>,
        primitives: &'a PrimitiveLibrary,
        tables: &'a ExpansionTables,
        rings: &'b mut TestRings,
        map: &'b ObstacleMap,
        approach: CrossingExtension,
        stats: &mut RouteSearchStats,
    ) -> Option<(
        State,
        CandidateMove<'a>,
        LegalMove,
        CrossingMoveOutcome,
        f64,
    )> {
        let mut legality_time_us = 0u128;
        let mut diagnostics = rings.diagnostics(map, context.target);
        for y in (32..=44).rev() {
            let state = State::new(20, y, 6);
            for candidate in candidate_moves(
                primitives.get_primitives_for_angle(state.angle),
                &tables.metadata,
                &tables.profiles,
                &tables.crossings,
                state,
                context.target,
                primitives.grid_size_um(),
                context.config.primitive_ordering,
            ) {
                if !candidate.class_is_straight {
                    continue;
                }
                let Some(MoveVerdict::Tier2(legal, outcome, extra_cost)) = move_is_legal(
                    context,
                    state,
                    approach,
                    &candidate,
                    &mut diagnostics,
                    stats,
                    &mut legality_time_us,
                ) else {
                    continue;
                };
                if CrossingExtension::from_outcome(&outcome).pending_after_crossing_cells > 0 {
                    return Some((state, candidate, legal, outcome, extra_cost));
                }
            }
        }
        None
    }

    #[test]
    fn complete_crossing_corridor_pays_the_debt_and_stops_when_blocked() {
        // The corridor below the partner spans y = 30 down to y = 28.
        // The second pass routes the same move inside a window whose
        // floor cuts that corridor in half, so the forced straight runs
        // out of the window before the debt clears.
        for window_min_y in [0, 29] {
            let truncated = window_min_y > 0;
            let (map, primitives, config, crossing) = crossing_corridor_fixture();
            let full_bounds = full_map_bounds(&map);
            let bounds = RoutingBounds {
                min_y: window_min_y,
                ..full_bounds
            };
            let hook_context =
                CrossingHookContext::build(&map, &primitives, &config, &crossing, full_bounds);
            let hook = hook_context.hook(
                &map,
                &crossing,
                &config,
                primitives.grid_size_um(),
                None,
                None,
            );
            let source = State::new(20, 50, 6);
            let target = State::new(20, 4, 6);
            let accepted = target_angle_acceptance(target, &config);
            let context = MoveContext {
                dense_grid: &hook_context.dense_grid,
                config: &config,
                hook: &hook,
                bounds,
                source,
                target,
                target_tolerance: config.target_tolerance_cells.max(0),
                accepted_target_angles: &accepted,
            };
            let tables = expansion_tables(&primitives, 6, config.bend_weight);
            let mut rings = TestRings::new();
            let mut stats = RouteSearchStats::default();
            // A state that already carries the full straight-run margin:
            // the approach a crossing needs, and a non-default extension,
            // so every move goes through the legality hook (Tier 2).
            let approach = CrossingExtension {
                straight_run_cells: hook.capped_required_margin(),
                ..CrossingExtension::default()
            };
            let (state, candidate, legal, outcome, extra_cost) = first_pending_crossing_move(
                &context,
                &primitives,
                &tables,
                &mut rings,
                &map,
                approach,
                &mut stats,
            )
            .expect("the fixture must offer a crossing that owes a straight run");
            let next_extension = CrossingExtension::from_outcome(&outcome);
            assert!(next_extension.pending_after_crossing_cells > 0);

            let mut extended_nodes: Vec<UnifiedExtendedNode> = Vec::new();
            let mut diagnostics = rings.diagnostics(&map, target);
            let mut chain_stats = RouteSearchStats::default();
            let completed = complete_crossing_corridor(
                &context,
                &PrimitiveTables::build(&primitives, config.bend_weight),
                &mut extended_nodes,
                state,
                100.0,
                UnifiedOpenRef::Dense(0),
                &candidate,
                &legal,
                next_extension,
                &outcome,
                extra_cost,
                false,
                &mut diagnostics,
                &mut chain_stats,
            );
            if truncated {
                assert!(
                    completed.is_none(),
                    "a corridor running out of the window must be abandoned"
                );
                assert_eq!(chain_stats.window_rejects, 1);
                assert_eq!(chain_stats.crossing_reject_pending_straight, 1);
            } else {
                let chain = completed.expect("the corridor must complete on a clear map");
                assert_eq!(chain.extension.pending_after_crossing_cells, 0);
                assert!(chain.g_score > 100.0);
                assert_eq!(chain.last_idx, extended_nodes.len() - 1);
                assert!(chain.state.y < state.y, "the chain keeps descending");
            }
        }
    }

    #[test]
    fn push_or_improve_extended_keeps_the_better_g_and_ignores_a_worse_one() {
        let (map, primitives, config, crossing) = crossing_corridor_fixture();
        let bounds = full_map_bounds(&map);
        let hook_context =
            CrossingHookContext::build(&map, &primitives, &config, &crossing, bounds);
        let hook = hook_context.hook(
            &map,
            &crossing,
            &config,
            primitives.grid_size_um(),
            None,
            None,
        );
        let source = State::new(20, 50, 6);
        let target = State::new(20, 4, 6);
        let accepted = target_angle_acceptance(target, &config);
        let context = MoveContext {
            dense_grid: &hook_context.dense_grid,
            config: &config,
            hook: &hook,
            bounds,
            source,
            target,
            target_tolerance: config.target_tolerance_cells.max(0),
            accepted_target_angles: &accepted,
        };
        // Far from the partner, but carrying a straight run, so the move
        // is a Tier-2 no-contact outcome with no debt of its own.
        let state = State::new(20, 48, 6);
        let approach = CrossingExtension {
            straight_run_cells: 1,
            ..CrossingExtension::default()
        };
        let tables = expansion_tables(&primitives, state.angle, config.bend_weight);
        let mut rings = TestRings::new();
        let mut stats = RouteSearchStats::default();
        let mut legality_time_us = 0u128;
        let mut diagnostics = rings.diagnostics(&map, target);
        let (candidate, legal, outcome, extra_cost) = candidate_moves(
            primitives.get_primitives_for_angle(state.angle),
            &tables.metadata,
            &tables.profiles,
            &tables.crossings,
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        )
        .filter(|candidate| candidate.class_is_straight)
        .find_map(|candidate| {
            match move_is_legal(
                &context,
                state,
                approach,
                &candidate,
                &mut diagnostics,
                &mut stats,
                &mut legality_time_us,
            ) {
                Some(MoveVerdict::Tier2(legal, outcome, extra_cost))
                    if outcome.pending_after_crossing_cells == 0 =>
                {
                    Some((candidate, legal, outcome, extra_cost))
                }
                _ => None,
            }
        })
        .expect("a debt-free Tier-2 move must exist away from the partner");
        let next_extension = CrossingExtension::from_outcome(&outcome);
        let key = (legal.next_state, next_extension);

        let heuristic = SearchHeuristic::new(target, &primitives, &config);
        let mut extended_nodes: Vec<UnifiedExtendedNode> = Vec::new();
        let mut extended_state: FxHashMap<(State, CrossingExtension), (f64, bool)> =
            FxHashMap::default();
        let mut tier2_open: BinaryHeap<OpenEntry> = BinaryHeap::new();
        let mut counter = 0u32;
        let mut best_crossings = 0u16;
        let mut best_crossing_ref: Option<usize> = None;
        let mut heap_time_us = 0u128;
        let mut push_stats = RouteSearchStats::default();

        let mut push = |current_g: f64,
                        current_ref: UnifiedOpenRef,
                        extended_nodes: &mut Vec<UnifiedExtendedNode>,
                        extended_state: &mut FxHashMap<(State, CrossingExtension), (f64, bool)>,
                        tier2_open: &mut BinaryHeap<OpenEntry>,
                        push_stats: &mut RouteSearchStats| {
            push_or_improve_extended(
                &context,
                &heuristic,
                extended_nodes,
                extended_state,
                tier2_open,
                0,
                &mut counter,
                state,
                approach,
                current_g,
                current_ref,
                &candidate,
                &legal,
                next_extension,
                &outcome,
                extra_cost,
                &mut best_crossings,
                &mut best_crossing_ref,
                &mut diagnostics,
                push_stats,
                &mut heap_time_us,
            )
            .expect("no generation overflow")
        };

        assert!(push(
            100.0,
            UnifiedOpenRef::Dense(3),
            &mut extended_nodes,
            &mut extended_state,
            &mut tier2_open,
            &mut push_stats
        ));
        let first_g = extended_state[&key].0;
        assert_eq!(extended_nodes.len(), 1);
        assert!(matches!(
            extended_nodes[0].parent,
            UnifiedParentRef::Dense(3)
        ));
        assert_eq!(push_stats.heap_pushes, 1);

        // A worse parent cost leaves the bookkeeping entry untouched and
        // spawns no node.
        assert!(!push(
            1000.0,
            UnifiedOpenRef::Dense(5),
            &mut extended_nodes,
            &mut extended_state,
            &mut tier2_open,
            &mut push_stats
        ));
        assert_eq!(extended_state[&key].0, first_g);
        assert_eq!(extended_nodes.len(), 1);
        assert!(matches!(
            extended_nodes[0].parent,
            UnifiedParentRef::Dense(3)
        ));
        assert_eq!(push_stats.heap_pushes, 1);

        // A better parent cost improves g, re-parents and pushes again.
        assert!(push(
            10.0,
            UnifiedOpenRef::Dense(7),
            &mut extended_nodes,
            &mut extended_state,
            &mut tier2_open,
            &mut push_stats
        ));
        assert!(extended_state[&key].0 < first_g);
        assert_eq!(extended_nodes.len(), 2);
        assert!(matches!(
            extended_nodes[1].parent,
            UnifiedParentRef::Dense(7)
        ));
        assert_eq!(push_stats.heap_pushes, 2);
    }
}
