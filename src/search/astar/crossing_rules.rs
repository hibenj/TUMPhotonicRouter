//! Crossing legality: the partner/config types the caller builds a request
//! from, the geometric crossing-move evaluator and its helpers, the
//! `CrossingLegalityHook` trait with its `NoCrossingHook`/`LiveCrossingHook`
//! implementations, `CrossingHookContext`, and the local-reservation checks
//! the kernel's Tier-2 path calls. Moved out of `src/astar.rs` (Milestone 3,
//! Slice 2, dissolving `mod unified_kernel`); pure code motion, no
//! behaviour change.

use crate::obstacle_map::{pack_xy, CellKey, NetId, ObstacleMap};
use crate::primitives::{Primitive, PrimitiveGeometry, PrimitiveLibrary, DIRECTIONS};
use crate::search::astar::config::{
    AStarConfig, NO_PENDING_CROSSING_ANGLE, NO_PENDING_CROSSING_PARTNER_INDEX,
};
use crate::search::astar::dense::{DenseDynamicCoreOwnerGrid, DenseRoutingGrid};
use crate::search::astar::diagnostics::{
    current_search_seq, trace_crossing_candidate, trace_crossing_level1_intersection,
    trace_crossing_pending, trace_crossing_pending_enabled,
};
use crate::search::astar::expansion::{
    compact_diagonal_halo_cells, primitive_initial_straight_run_distance,
    primitive_terminal_straight_run_cells, FootprintCollisionProfile,
};
use crate::search::astar::kernel::{UnifiedExtendedNode, UnifiedOpenRef, UnifiedParentRef};
use crate::search::astar::window::RoutingBounds;
use crate::search::state::{RouteSearchStats, State};
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::time::Instant;

#[derive(Clone, Debug)]
pub struct CrossingSearchPartner {
    pub net_id: NetId,
    pub waypoints: Vec<(i32, i32)>,
    pub target_terminal_bump_guard: Option<TerminalBumpGuard>,
    /// Contribution 1 (crossing-guided search): the search price of one
    /// crossing with THIS partner, replacing `CrossingSearchConfig::crossing_loss`
    /// when set. A planned pair (topology plan) gets its own, normally zero,
    /// price; `None` keeps the baseline price, so lidar-pure is unchanged.
    pub crossing_loss_override: Option<f64>,
    /// S2 of contribution 1: the plan predicts exactly ONE crossing per
    /// planned pair, so only the first crossing of this partner on a path
    /// gets `crossing_loss_override`; every further one (a braid) pays
    /// `CrossingSearchConfig::crossing_loss`. Tracked per partner in the
    /// search key (`crossed_mask`, one bit per budgeted partner, at most 64
    /// -- partners beyond that stay unbudgeted).
    pub single_discounted_crossing: bool,
}

#[derive(Clone, Copy, Debug)]
pub enum TerminalBumpAxis {
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug)]
pub struct TerminalBumpGuard {
    pub axis: TerminalBumpAxis,
    pub target_axis_coord: f64,
    pub target_along_coord: f64,
    pub required_bump_cells: i32,
}

#[derive(Clone, Debug)]
pub struct CrossingSearchConfig {
    /// Diagnostic/trace switches for the crossing search. Cloned in from
    /// the router's `RouterConfig` when this `CrossingSearchConfig` is
    /// built.
    pub diagnostics: crate::config::KernelDiagnostics,
    pub net_id: NetId,
    pub partners: Vec<CrossingSearchPartner>,
    pub min_straight_cells: i32,
    pub crossing_half_size_cells: i32,
    pub bend_runout_cells: i32,
    pub crossing_loss: f64,
    pub require_all_partners: bool,
    pub terminal_bump_guard: Option<TerminalBumpGuard>,
}

pub(crate) fn crossing_required_margin_cells(
    crossing_half_size_cells: i32,
    _min_straight_cells: i32,
    bend_runout_cells: i32,
) -> i32 {
    crossing_half_size_cells.max(0) + bend_runout_cells.max(0)
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub(crate) struct CrossingAStarKey {
    pub(crate) state: State,
    pub(crate) crossed_mask: u64,
    pub(crate) next_partner_index: u8,
    pub(crate) straight_run_cells: i32,
    pub(crate) pending_after_crossing_cells: i32,
    pub(crate) pending_after_crossing_angle: u8,
    pub(crate) pending_after_crossing_partner_index: u8,
}

#[derive(Clone, Debug)]
pub(crate) struct CrossingMoveOutcome {
    pub(crate) crossed_mask: u64,
    pub(crate) next_partner_index: u8,
    pub(crate) straight_run_cells: i32,
    pub(crate) pending_after_crossing_cells: i32,
    pub(crate) pending_after_crossing_angle: u8,
    pub(crate) pending_after_crossing_partner_index: u8,
    pub(crate) crossing_count: u32,
    /// Sum of the per-event search prices of this move's crossings
    /// (`partner.crossing_loss_override` or `CrossingSearchConfig::crossing_loss`).
    pub(crate) crossing_cost: f64,
    pub(crate) active_reservation_keys: Vec<CellKey>,
    pub(crate) pending_reservation_keys: Vec<CellKey>,
    /// True when the move touched committed cells but every touched cell
    /// belonged to the partner whose straight-after debt this move is
    /// paying -- the crossing's own neighbourhood, not a new contact.
    /// `LiveCrossingHook::evaluate` exempts such moves from its
    /// "contact without a recorded crossing" rejection.
    pub(crate) contact_only_pending_partner: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct PrimitivePathSegment {
    pub(crate) start: (i32, i32),
    pub(crate) end: (i32, i32),
    pub(crate) angle: u8,
    pub(crate) distance_before_segment: f64,
    pub(crate) length: f64,
    pub(crate) starts_after_kink: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct RelativePrimitivePathSegment {
    pub(crate) start: (i32, i32),
    pub(crate) end: (i32, i32),
    pub(crate) angle: u8,
    pub(crate) distance_before_segment: f64,
    pub(crate) length: f64,
    pub(crate) starts_after_kink: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct RelativeEffectiveCollisionWitness {
    pub(crate) offset: (i32, i32),
    pub(crate) route_segment_idx: Option<usize>,
}

#[derive(Clone, Debug)]
pub(crate) struct PrimitiveCrossingMetadata {
    pub(crate) segments: Vec<RelativePrimitivePathSegment>,
    pub(crate) footprint_witnesses: Vec<RelativeEffectiveCollisionWitness>,
    pub(crate) witnesses: Vec<RelativeEffectiveCollisionWitness>,
    pub(crate) extra_witnesses: Vec<RelativeEffectiveCollisionWitness>,
    pub(crate) extra_witness_offsets: Vec<(i32, i32)>,
    pub(crate) extra_witness_profile: FootprintCollisionProfile,
    pub(crate) has_extra_witnesses: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct PartnerPathSegment {
    pub(crate) start: (i32, i32),
    pub(crate) end: (i32, i32),
    pub(crate) angle: u8,
    pub(crate) length: f64,
    pub(crate) min_x: i32,
    pub(crate) max_x: i32,
    pub(crate) min_y: i32,
    pub(crate) max_y: i32,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct CrossingRouteIntersection {
    pub(crate) distance_from_primitive_start: f64,
    pub(crate) distance_before_on_segment: f64,
    pub(crate) distance_after_on_segment: f64,
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) route_angle: u8,
    pub(crate) partner_angle: u8,
    pub(crate) segment_is_terminal: bool,
    pub(crate) partner_idx: usize,
    pub(crate) bit: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CrossingContactReject {
    NotPerpendicular,
    Unmatched,
    Footprint,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct EffectiveCollisionWitness {
    pub(crate) cell: (i32, i32),
    pub(crate) route_segment_idx: Option<usize>,
}

#[derive(Clone, Debug)]
pub(crate) struct ContactedPartner {
    pub(crate) partner_idx: usize,
    pub(crate) first_witness: EffectiveCollisionWitness,
    pub(crate) extra_witnesses: Vec<EffectiveCollisionWitness>,
}

impl ContactedPartner {
    pub(crate) fn new(partner_idx: usize, witness: EffectiveCollisionWitness) -> Self {
        Self {
            partner_idx,
            first_witness: witness,
            extra_witnesses: Vec::new(),
        }
    }

    pub(crate) fn push_witness(&mut self, witness: EffectiveCollisionWitness) {
        self.extra_witnesses.push(witness);
    }
}

#[derive(Default)]
pub(crate) struct ContactedPartners {
    pub(crate) first: Option<ContactedPartner>,
    pub(crate) extra: Vec<ContactedPartner>,
}

impl ContactedPartners {
    pub(crate) fn len(&self) -> usize {
        usize::from(self.first.is_some()) + self.extra.len()
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = &ContactedPartner> {
        self.first.iter().chain(self.extra.iter())
    }

    pub(crate) fn push_witness(&mut self, partner_idx: usize, witness: EffectiveCollisionWitness) {
        let Some(first) = self.first.as_mut() else {
            self.first = Some(ContactedPartner::new(partner_idx, witness));
            return;
        };
        if first.partner_idx == partner_idx {
            first.push_witness(witness);
            return;
        }
        if partner_idx < first.partner_idx {
            let old_first = std::mem::replace(first, ContactedPartner::new(partner_idx, witness));
            Self::insert_extra_sorted(&mut self.extra, old_first);
            return;
        }
        match self
            .extra
            .binary_search_by_key(&partner_idx, |contact| contact.partner_idx)
        {
            Ok(idx) => self.extra[idx].push_witness(witness),
            Err(idx) => self
                .extra
                .insert(idx, ContactedPartner::new(partner_idx, witness)),
        }
    }

    pub(crate) fn insert_extra_sorted(
        extra: &mut Vec<ContactedPartner>,
        contact: ContactedPartner,
    ) {
        match extra.binary_search_by_key(&contact.partner_idx, |item| item.partner_idx) {
            Ok(idx) => {
                let ContactedPartner {
                    first_witness,
                    mut extra_witnesses,
                    ..
                } = contact;
                extra[idx].push_witness(first_witness);
                extra[idx].extra_witnesses.append(&mut extra_witnesses);
            }
            Err(idx) => extra.insert(idx, contact),
        }
    }
}

pub(crate) fn primitive_crossing_metadata(primitive: &Primitive) -> PrimitiveCrossingMetadata {
    let mut segments = Vec::new();
    let mut path_distance = 0.0;
    let mut current_start: Option<(i32, i32)> = None;
    let mut current_end: Option<(i32, i32)> = None;
    let mut current_angle: Option<u8> = None;
    let mut current_distance_before = 0.0;
    let mut segment_starts_after_kink = false;

    for pair in primitive.footprint.windows(2) {
        let start = pair[0];
        let end = pair[1];
        if start == end {
            continue;
        }
        let Some(angle) = direction_angle_between_grid_cells(start, end) else {
            continue;
        };
        let step_len = grid_segment_step_count(start, end);
        if current_angle == Some(angle) && current_end == Some(start) {
            current_end = Some(end);
            path_distance += step_len;
            continue;
        }
        if let (Some(seg_start), Some(seg_end), Some(seg_angle)) =
            (current_start, current_end, current_angle)
        {
            segments.push(RelativePrimitivePathSegment {
                start: seg_start,
                end: seg_end,
                angle: seg_angle,
                distance_before_segment: current_distance_before,
                length: grid_segment_step_count(seg_start, seg_end),
                starts_after_kink: segment_starts_after_kink,
            });
            segment_starts_after_kink = true;
        }
        current_start = Some(start);
        current_end = Some(end);
        current_angle = Some(angle);
        current_distance_before = path_distance;
        path_distance += step_len;
    }

    if let (Some(seg_start), Some(seg_end), Some(seg_angle)) =
        (current_start, current_end, current_angle)
    {
        segments.push(RelativePrimitivePathSegment {
            start: seg_start,
            end: seg_end,
            angle: seg_angle,
            distance_before_segment: current_distance_before,
            length: grid_segment_step_count(seg_start, seg_end),
            starts_after_kink: segment_starts_after_kink,
        });
    }

    let footprint_witnesses: Vec<RelativeEffectiveCollisionWitness> = primitive
        .footprint
        .iter()
        .copied()
        .map(|offset| RelativeEffectiveCollisionWitness {
            offset,
            route_segment_idx: None,
        })
        .collect();
    let witnesses = effective_collision_witness_offsets(primitive, &segments);
    let extra_witnesses: Vec<RelativeEffectiveCollisionWitness> = witnesses
        .iter()
        .copied()
        .filter(|witness| !primitive.footprint.contains(&witness.offset))
        .collect();
    let extra_witness_offsets: Vec<(i32, i32)> = extra_witnesses
        .iter()
        .map(|witness| witness.offset)
        .collect();
    let extra_witness_profile = FootprintCollisionProfile::from_footprint(&extra_witness_offsets);
    let has_extra_witnesses = !extra_witnesses.is_empty();
    PrimitiveCrossingMetadata {
        segments,
        footprint_witnesses,
        witnesses,
        extra_witnesses,
        extra_witness_offsets,
        extra_witness_profile,
        has_extra_witnesses,
    }
}

#[inline]
pub(crate) fn translate_primitive_path_segment(
    state: State,
    segment: RelativePrimitivePathSegment,
) -> PrimitivePathSegment {
    PrimitivePathSegment {
        start: (state.x + segment.start.0, state.y + segment.start.1),
        end: (state.x + segment.end.0, state.y + segment.end.1),
        angle: segment.angle,
        distance_before_segment: segment.distance_before_segment,
        length: segment.length,
        starts_after_kink: segment.starts_after_kink,
    }
}

pub(crate) fn max_crossing_witness_offset(metadata: &[Vec<PrimitiveCrossingMetadata>]) -> i32 {
    metadata
        .iter()
        .flat_map(|bucket| bucket.iter())
        .flat_map(|primitive| primitive.witnesses.iter())
        .map(|witness| witness.offset.0.abs().max(witness.offset.1.abs()))
        .max()
        .unwrap_or(0)
}

pub(crate) fn crossing_partner_path_segments(
    crossing: &CrossingSearchConfig,
) -> Vec<Vec<PartnerPathSegment>> {
    crossing
        .partners
        .iter()
        .map(|partner| {
            partner
                .waypoints
                .windows(2)
                .filter_map(|segment| {
                    let start = segment[0];
                    let end = segment[1];
                    let angle = direction_angle_between_grid_cells(start, end)?;
                    Some(PartnerPathSegment {
                        start,
                        end,
                        angle,
                        length: grid_segment_step_count(start, end),
                        min_x: start.0.min(end.0),
                        max_x: start.0.max(end.0),
                        min_y: start.1.min(end.1),
                        max_y: start.1.max(end.1),
                    })
                })
                .collect()
        })
        .collect()
}

#[inline]
pub(crate) fn route_partner_segment_bboxes_overlap(
    route_segment: &PrimitivePathSegment,
    partner_segment: &PartnerPathSegment,
) -> bool {
    let route_min_x = route_segment.start.0.min(route_segment.end.0);
    let route_max_x = route_segment.start.0.max(route_segment.end.0);
    let route_min_y = route_segment.start.1.min(route_segment.end.1);
    let route_max_y = route_segment.start.1.max(route_segment.end.1);
    route_min_x <= partner_segment.max_x
        && route_max_x >= partner_segment.min_x
        && route_min_y <= partner_segment.max_y
        && route_max_y >= partner_segment.min_y
}

pub(crate) fn terminal_bump_guard_satisfied(
    guard: Option<TerminalBumpGuard>,
    crossing_half_size_cells: i32,
    x: f64,
    y: f64,
    route_angle: u8,
) -> bool {
    let Some(guard) = guard else {
        return true;
    };
    if guard.required_bump_cells <= 0 {
        return true;
    }
    let eps = 1.0e-9;
    let crossing_half = f64::from(crossing_half_size_cells.max(0));
    let required = f64::from(guard.required_bump_cells.max(0));
    let axis_margin = f64::from((guard.required_bump_cells / 2).max(0));
    let available_is_blocked = |available: f64| -> bool {
        available + eps < required || (available - required).abs() <= eps
    };
    let axis_delta_is_blocked =
        |axis_delta: f64| -> bool { (axis_delta.round() - required).abs() <= eps };
    match guard.axis {
        TerminalBumpAxis::Horizontal => {
            if route_angle % 8 != 0 && route_angle % 8 != 4 {
                return true;
            }
            let axis_delta = (y - guard.target_axis_coord).abs();
            if axis_delta_is_blocked(axis_delta) {
                return false;
            }
            if axis_delta > axis_margin + eps {
                return true;
            }
            let available = (guard.target_along_coord - x).abs() - crossing_half;
            !available_is_blocked(available)
        }
        TerminalBumpAxis::Vertical => {
            if route_angle % 8 != 2 && route_angle % 8 != 6 {
                return true;
            }
            let axis_delta = (x - guard.target_axis_coord).abs();
            if axis_delta_is_blocked(axis_delta) {
                return false;
            }
            if axis_delta > axis_margin + eps {
                return true;
            }
            let available = (guard.target_along_coord - y).abs() - crossing_half;
            !available_is_blocked(available)
        }
    }
}

/// Every state the unified kernel visits carries one of these. The
/// `Default` value means "never touched a crossing" -- the only value
/// Tier-1 (dense-array-backed) states ever carry. Any other value means
/// the state is inside, or just past, an active crossing corridor, and
/// lives in Tier 2 (the small sparse side table) instead.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub(crate) struct CrossingExtension {
    pub(crate) crossed_mask: u64,
    pub(crate) next_partner_index: u8,
    pub(crate) straight_run_cells: i32,
    pub(crate) pending_after_crossing_cells: i32,
    pub(crate) pending_after_crossing_angle: u8,
    pub(crate) pending_after_crossing_partner_index: u8,
}

impl Default for CrossingExtension {
    fn default() -> Self {
        Self {
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 0,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        }
    }
}

impl CrossingExtension {
    pub(crate) fn is_default(&self) -> bool {
        *self == Self::default()
    }

    fn to_key(self, state: State) -> CrossingAStarKey {
        CrossingAStarKey {
            state,
            crossed_mask: self.crossed_mask,
            next_partner_index: self.next_partner_index,
            straight_run_cells: self.straight_run_cells,
            pending_after_crossing_cells: self.pending_after_crossing_cells,
            pending_after_crossing_angle: self.pending_after_crossing_angle,
            pending_after_crossing_partner_index: self.pending_after_crossing_partner_index,
        }
    }

    pub(crate) fn from_outcome(outcome: &CrossingMoveOutcome) -> Self {
        Self {
            crossed_mask: outcome.crossed_mask,
            next_partner_index: outcome.next_partner_index,
            straight_run_cells: outcome.straight_run_cells,
            pending_after_crossing_cells: outcome.pending_after_crossing_cells,
            pending_after_crossing_angle: outcome.pending_after_crossing_angle,
            pending_after_crossing_partner_index: outcome.pending_after_crossing_partner_index,
        }
    }
}

/// The unified kernel's one pluggable extension point. Called only when a
/// step's destination is blocked (`footprint_free == false`) or the
/// current state already carries a non-default `CrossingExtension` --
/// never on a step that is both free and extension-free, which is what
/// keeps crossing-disabled routing from paying for this call at all.
pub(crate) trait CrossingLegalityHook {
    /// Whether the kernel must keep a `straight_run_cells` count for
    /// every Tier-1 state too, not only Tier-2 ones. Crossing legality
    /// depends on how long the route has already run straight *before*
    /// the very first crossing it ever attempts -- a state that has
    /// never crossed anything (Tier 1) still needs an accurate count the
    /// instant it tries to. `false` for `NoCrossingHook`, so this bookkeeping
    /// (and its allocation) is compiled away entirely for crossings-disabled
    /// routing; `true` for `LiveCrossingHook`, matching what today's
    /// crossing kernel already tracks unconditionally for every state.
    const TRACKS_STRAIGHT_RUN: bool = false;

    /// The margin `TRACKS_STRAIGHT_RUN`'s bookkeeping caps its count at.
    /// Only ever consulted when `TRACKS_STRAIGHT_RUN` is `true`.
    fn capped_required_margin(&self) -> i32 {
        0
    }

    /// Returns `None` to reject the step (no legal crossing here, and
    /// this is not an active post-crossing corridor state either -- the
    /// unified kernel rejects it exactly like the plain kernel rejects a
    /// blocked cell). Returns `Some((outcome, extra_cost))` to accept it:
    /// `outcome` carries the `CrossingExtension` the destination state
    /// must now carry plus any local-crossing-reservation bookkeeping,
    /// and `extra_cost` is the already-priced cost this step adds on top
    /// of the primitive's own base step cost (0.0 for a step that only
    /// continues an existing post-crossing corridor without crossing
    /// anything new).
    #[allow(clippy::too_many_arguments)]
    fn evaluate(
        &self,
        state: State,
        current_extension: CrossingExtension,
        primitive: &Primitive,
        primitive_class_is_straight: bool,
        primitive_crossing: &PrimitiveCrossingMetadata,
        footprint_free: bool,
        stats: &mut RouteSearchStats,
    ) -> Option<(CrossingMoveOutcome, f64)>;

    /// Whether `extension` satisfies any extra crossing-specific goal
    /// condition (today: `require_all_partners`'s "every partner has
    /// been crossed" requirement). Always satisfied when crossings are
    /// disabled.
    fn goal_extra_ok(&self, _extension: CrossingExtension) -> bool {
        true
    }

    fn ignores_dynamic_obstacles(&self) -> bool {
        false
    }

    /// Optional search-guidance bonus added to a next state's f-score,
    /// mirroring today's `crossing_progress_heuristic`. Zero when
    /// crossings are disabled.
    fn heuristic_bonus(&self, _next_state: State, _next_extension: CrossingExtension) -> f64 {
        0.0
    }
}

/// Crossings disabled: makes the unified kernel behave exactly like
/// today's plain kernel. This hook is never reachable in a way that does
/// real work: every step is either free-with-default-extension (the
/// kernel's fast path, which never calls this hook at all) or, if
/// genuinely blocked, this hook always rejects it -- identical to the
/// plain kernel's own `if !footprint_free { continue; }`.
pub(crate) struct NoCrossingHook;

impl CrossingLegalityHook for NoCrossingHook {
    fn evaluate(
        &self,
        _state: State,
        _current_extension: CrossingExtension,
        _primitive: &Primitive,
        _primitive_class_is_straight: bool,
        _primitive_crossing: &PrimitiveCrossingMetadata,
        _footprint_free: bool,
        _stats: &mut RouteSearchStats,
    ) -> Option<(CrossingMoveOutcome, f64)> {
        None
    }
}

/// Crossings enabled: reuses today's crossing kernel's own legality
/// functions (`crossing_no_contact_outcome`, `crossing_move_outcome_with_segments`)
/// unchanged, so this kernel is validated against the real
/// crossing-legality logic, not a stand-in for it.
pub(crate) struct LiveCrossingHook<'a> {
    pub(super) obstacle_map: &'a ObstacleMap,
    pub(super) dense_grid: &'a DenseRoutingGrid,
    pub(super) crossing: &'a CrossingSearchConfig,
    pub(super) required_margin: i32,
    pub(super) capped_required_margin: i32,
    pub(super) reservation_margin: i32,
    /// Falls back to `port_open_cells` when absent, exactly like today's
    /// crossing kernel's own `reservation_open_cells.or(port_open_cells)`
    /// call -- see `evaluate` below.
    pub(super) reservation_open_cells: Option<&'a FxHashSet<CellKey>>,
    pub(super) port_open_cells: Option<&'a FxHashSet<CellKey>>,
    pub(super) partner_index_by_id: &'a FxHashMap<NetId, usize>,
    pub(super) partner_budget_bits: &'a [u64],
    pub(super) partner_segments: &'a [Vec<PartnerPathSegment>],
    pub(super) dynamic_core_owners: &'a DenseDynamicCoreOwnerGrid,
    pub(super) ignore_dynamic_obstacles: bool,
    pub(super) grid_size_um: f64,
    pub(super) all_partner_mask: u64,
    pub(super) all_partner_count: u8,
    pub(super) search_start: Instant,
}

impl CrossingLegalityHook for LiveCrossingHook<'_> {
    const TRACKS_STRAIGHT_RUN: bool = true;

    fn capped_required_margin(&self) -> i32 {
        self.capped_required_margin
    }

    fn ignores_dynamic_obstacles(&self) -> bool {
        self.ignore_dynamic_obstacles
    }

    fn evaluate(
        &self,
        state: State,
        current_extension: CrossingExtension,
        primitive: &Primitive,
        primitive_class_is_straight: bool,
        primitive_crossing: &PrimitiveCrossingMetadata,
        footprint_free: bool,
        stats: &mut RouteSearchStats,
    ) -> Option<(CrossingMoveOutcome, f64)> {
        let current_key = current_extension.to_key(state);
        let halo_checked = footprint_free
            && !self.ignore_dynamic_obstacles
            && primitive_crossing.has_extra_witnesses;
        let extra_halo_free = halo_checked
            && self.dense_grid.relative_offsets_free_with_profile(
                state.x,
                state.y,
                &primitive_crossing.extra_witness_offsets,
                &primitive_crossing.extra_witness_profile,
            );
        let halo_contact = halo_checked && !extra_halo_free;
        // While a crossing's straight-after debt is open, only a pure
        // straight in the pending direction may move (and pay it, partially
        // or fully). A bend's first arm is arc when realized, so it must
        // not start before the debt is zero -- same rule as the contact
        // path in `crossing_move_outcome_with_segments`.
        if current_extension.pending_after_crossing_cells > 0 {
            let pure_straight_in_pending_direction = primitive_class_is_straight
                && primitive.end_angle == state.angle
                && state.angle == current_extension.pending_after_crossing_angle
                && matches!(primitive.geometry, PrimitiveGeometry::Straight { .. });
            if !pure_straight_in_pending_direction {
                stats.crossing_reject_pending_straight += 1;
                return None;
            }
        }
        let outcome = if footprint_free
            && !self.ignore_dynamic_obstacles
            && (!primitive_crossing.has_extra_witnesses || extra_halo_free)
        {
            stats.crossing_hotpath_no_contact += 1;
            crossing_no_contact_outcome(
                current_key,
                state,
                primitive,
                primitive_class_is_straight,
                self.capped_required_margin,
            )
        } else {
            crossing_move_outcome_with_segments(
                self.obstacle_map,
                self.crossing,
                current_key,
                state,
                primitive,
                primitive_class_is_straight,
                self.required_margin,
                self.capped_required_margin,
                self.reservation_margin,
                self.reservation_open_cells.or(self.port_open_cells),
                self.partner_index_by_id,
                self.partner_budget_bits,
                self.partner_segments,
                self.dynamic_core_owners,
                primitive_crossing,
                footprint_free,
                !footprint_free,
                false,
                stats,
                Some(&self.search_start),
            )?
        };
        // A contact must legalize as a crossing to pass. This covers both
        // a blocked footprint and a free footprint whose compact diagonal
        // halo touches another net: two parallel adjacent diagonals share
        // no cell and produce no crossing event, but their realized
        // waveguides physically overlap (multiportmmi_8x8 n_13/n_14 at
        // heuristic weight 1.0), so a halo contact without a crossing is
        // rejected exactly like the crossings-disabled kernel rejects it.
        // Contact without a recorded crossing is grazing -- unless every
        // touched cell belongs to the partner whose straight-after debt
        // this move is paying (the crossing's own neighbourhood).
        if (!footprint_free || halo_contact)
            && outcome.crossing_count == 0
            && !outcome.contact_only_pending_partner
        {
            return None;
        }
        let extra_cost = outcome.crossing_cost;
        Some((outcome, extra_cost))
    }

    fn goal_extra_ok(&self, extension: CrossingExtension) -> bool {
        !self.crossing.require_all_partners
            || (extension.crossed_mask == self.all_partner_mask
                && extension.next_partner_index == self.all_partner_count)
    }

    fn heuristic_bonus(&self, next_state: State, next_extension: CrossingExtension) -> f64 {
        crossing_progress_heuristic(
            next_extension.to_key(next_state),
            self.crossing,
            self.required_margin,
            self.grid_size_um,
        )
    }
}

/// Owns everything a `LiveCrossingHook` borrows from, so tests can build
/// one context and construct as many short-lived hooks (plain or
/// call-counting) from it as they need.
pub(crate) struct CrossingHookContext {
    pub(crate) dense_grid: DenseRoutingGrid,
    pub(crate) dynamic_core_owners: DenseDynamicCoreOwnerGrid,
    pub(crate) partner_index_by_id: FxHashMap<NetId, usize>,
    pub(crate) partner_budget_bits: Vec<u64>,
    pub(crate) partner_segments: Vec<Vec<PartnerPathSegment>>,
    pub(crate) required_margin: i32,
    pub(crate) capped_required_margin: i32,
    pub(crate) all_partner_mask: u64,
    pub(crate) all_partner_count: u8,
}

impl CrossingHookContext {
    pub(super) fn build(
        obstacle_map: &ObstacleMap,
        primitives: &PrimitiveLibrary,
        config: &AStarConfig,
        crossing: &CrossingSearchConfig,
        full_bounds: RoutingBounds,
    ) -> Self {
        let dense_grid = DenseRoutingGrid::from_obstacle_map_with_dynamic_expansion(
            obstacle_map,
            full_bounds,
            None,
            config.max_dense_obstacle_cells,
            config.ignore_dynamic_obstacles,
            config.history_weight > 0.0,
            config.long_straight_congestion_weight > 0.0,
            0,
            None,
        )
        .expect("dense grid should build for a valid obstacle map");
        let primitive_buckets: [&[Primitive]; 8] =
            std::array::from_fn(|angle| primitives.get_primitives_for_angle(angle as u8));
        let primitive_crossing_metadata: Vec<Vec<PrimitiveCrossingMetadata>> = primitive_buckets
            .iter()
            .map(|bucket| bucket.iter().map(primitive_crossing_metadata).collect())
            .collect();
        let crossing_lookup_bounds = full_bounds.expanded_and_clamped(
            max_crossing_witness_offset(&primitive_crossing_metadata),
            obstacle_map.width(),
            obstacle_map.height(),
        );
        let dynamic_core_owners =
            DenseDynamicCoreOwnerGrid::from_obstacle_map(obstacle_map, crossing_lookup_bounds)
                .expect("owner grid should build for a valid obstacle map");
        let required_margin = crossing_required_margin_cells(
            crossing.crossing_half_size_cells,
            crossing.min_straight_cells,
            crossing.bend_runout_cells,
        );
        let capped_required_margin = required_margin.max(1);
        let all_partner_mask = if crossing.require_all_partners {
            (1u64 << crossing.partners.len()) - 1
        } else {
            0
        };
        let all_partner_count = u8::try_from(crossing.partners.len()).unwrap_or(u8::MAX);
        let partner_index_by_id = crossing
            .partners
            .iter()
            .enumerate()
            .map(|(idx, partner)| (partner.net_id, idx))
            .collect();
        let partner_segments = crossing_partner_path_segments(crossing);
        Self {
            dense_grid,
            dynamic_core_owners,
            partner_index_by_id,
            partner_budget_bits: crossing_partner_budget_bits(crossing),
            partner_segments,
            required_margin,
            capped_required_margin,
            all_partner_mask,
            all_partner_count,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn hook<'a>(
        &'a self,
        obstacle_map: &'a ObstacleMap,
        crossing: &'a CrossingSearchConfig,
        config: &AStarConfig,
        grid_size_um: f64,
        reservation_open_cells: Option<&'a FxHashSet<CellKey>>,
        port_open_cells: Option<&'a FxHashSet<CellKey>>,
    ) -> LiveCrossingHook<'a> {
        LiveCrossingHook {
            obstacle_map,
            dense_grid: &self.dense_grid,
            crossing,
            required_margin: self.required_margin,
            capped_required_margin: self.capped_required_margin,
            reservation_margin: crossing.crossing_half_size_cells,
            reservation_open_cells,
            port_open_cells,
            partner_index_by_id: &self.partner_index_by_id,
            partner_budget_bits: &self.partner_budget_bits,
            partner_segments: &self.partner_segments,
            dynamic_core_owners: &self.dynamic_core_owners,
            ignore_dynamic_obstacles: config.ignore_dynamic_obstacles,
            grid_size_um,
            all_partner_mask: self.all_partner_mask,
            all_partner_count: self.all_partner_count,
            search_start: Instant::now(),
        }
    }
}

pub(crate) fn unified_candidate_hits_active_local_crossing_reservation(
    current: UnifiedOpenRef,
    state: State,
    primitive_crossing: &PrimitiveCrossingMetadata,
    extended_nodes: &[UnifiedExtendedNode],
) -> bool {
    if let UnifiedOpenRef::Extended(ext_idx) = current {
        if !extended_nodes[ext_idx].chain_has_reservations {
            return false;
        }
    } else {
        return false;
    }
    let current_key = pack_xy(state.x, state.y);
    let mut candidate_keys = FxHashSet::default();
    for witness in &primitive_crossing.witnesses {
        let key = pack_xy(state.x + witness.offset.0, state.y + witness.offset.1);
        if key != current_key {
            candidate_keys.insert(key);
        }
    }
    if candidate_keys.is_empty() {
        return false;
    }

    let mut cursor = current;
    loop {
        let UnifiedOpenRef::Extended(ext_idx) = cursor else {
            // Tier-1 (dense) states never carry a local reservation.
            return false;
        };
        let Some(node) = extended_nodes.get(ext_idx) else {
            return false;
        };
        for key in &node.active_local_reservation_keys {
            if candidate_keys.contains(key) {
                return true;
            }
        }
        cursor = match node.parent {
            UnifiedParentRef::Dense(idx) => UnifiedOpenRef::Dense(idx),
            UnifiedParentRef::Extended(idx) => UnifiedOpenRef::Extended(idx),
        };
    }
}

/// The unified kernel. See this module's own doc comment above for the
/// full design summary.
/// Reservation-window disjointness across moves: the windows the
/// candidate move reserves (`outcome.active_reservation_keys` and
/// `outcome.pending_reservation_keys`, one +-half_size box per new
/// crossing) must not share a cell with any window reserved earlier on
/// this route -- active or still pending -- so two crossing elements
/// never overlap. Walks the parent chain like
/// `unified_candidate_hits_active_local_crossing_reservation` and stops
/// at the first Tier-1 ancestor (which never carries a reservation).
pub(crate) fn unified_outcome_windows_overlap_own_reservations(
    current: UnifiedOpenRef,
    outcome: &CrossingMoveOutcome,
    extended_nodes: &[UnifiedExtendedNode],
) -> bool {
    if outcome.active_reservation_keys.is_empty() && outcome.pending_reservation_keys.is_empty() {
        return false;
    }
    match current {
        UnifiedOpenRef::Extended(ext_idx) if extended_nodes[ext_idx].chain_has_reservations => {}
        _ => return false,
    }
    let hits = |keys: &[CellKey]| {
        keys.iter().any(|key| {
            outcome.active_reservation_keys.contains(key)
                || outcome.pending_reservation_keys.contains(key)
        })
    };
    let mut cursor = current;
    loop {
        let UnifiedOpenRef::Extended(ext_idx) = cursor else {
            return false;
        };
        let Some(node) = extended_nodes.get(ext_idx) else {
            return false;
        };
        if hits(&node.active_local_reservation_keys) || hits(&node.pending_local_reservation_keys) {
            return true;
        }
        cursor = match node.parent {
            UnifiedParentRef::Dense(idx) => UnifiedOpenRef::Dense(idx),
            UnifiedParentRef::Extended(idx) => UnifiedOpenRef::Extended(idx),
        };
    }
}

pub(crate) fn crossing_no_contact_outcome(
    current_key: CrossingAStarKey,
    state: State,
    primitive: &Primitive,
    is_straight: bool,
    capped_required_margin: i32,
) -> CrossingMoveOutcome {
    let primitive_steps = primitive.dx.abs().max(primitive.dy.abs());
    let mut straight_run = if is_straight && primitive.end_angle == state.angle {
        current_key
            .straight_run_cells
            .saturating_add(primitive_steps)
            .min(capped_required_margin)
    } else {
        0
    };
    if !is_straight {
        straight_run = primitive_terminal_straight_run_cells(primitive, primitive.end_angle)
            .min(capped_required_margin);
    }
    let pending_initial_run = primitive_initial_straight_run_distance(primitive, state.angle);
    let remaining_pending_after_crossing = if current_key.pending_after_crossing_cells > 0 {
        (f64::from(current_key.pending_after_crossing_cells) - pending_initial_run)
            .ceil()
            .max(0.0) as i32
    } else {
        0
    };
    CrossingMoveOutcome {
        crossed_mask: current_key.crossed_mask,
        next_partner_index: current_key.next_partner_index,
        straight_run_cells: straight_run,
        pending_after_crossing_cells: remaining_pending_after_crossing.min(capped_required_margin),
        pending_after_crossing_angle: if remaining_pending_after_crossing > 0 {
            current_key.pending_after_crossing_angle
        } else {
            NO_PENDING_CROSSING_ANGLE
        },
        pending_after_crossing_partner_index: if remaining_pending_after_crossing > 0 {
            current_key.pending_after_crossing_partner_index
        } else {
            NO_PENDING_CROSSING_PARTNER_INDEX
        },
        crossing_count: 0,
        crossing_cost: 0.0,
        active_reservation_keys: Vec::new(),
        pending_reservation_keys: Vec::new(),
        contact_only_pending_partner: false,
    }
}

pub(crate) fn record_pending_after_crossing_reject(
    stats: &mut RouteSearchStats,
    crossing: &CrossingSearchConfig,
    pending_partner_index: u8,
    search_start: Option<&Instant>,
) {
    let Some(partner) = crossing.partners.get(usize::from(pending_partner_index)) else {
        return;
    };
    record_perpendicular_crossing_reject(stats, crossing, partner.net_id, search_start);
    record_pending_after_crossing_partner(stats, crossing, pending_partner_index, search_start);
}

pub(crate) fn record_pending_after_crossing_partner(
    stats: &mut RouteSearchStats,
    crossing: &CrossingSearchConfig,
    pending_partner_index: u8,
    search_start: Option<&Instant>,
) {
    let Some(partner) = crossing.partners.get(usize::from(pending_partner_index)) else {
        return;
    };
    let count = stats
        .crossing_pending_straight_by_partner
        .entry(partner.net_id)
        .and_modify(|count| *count += 1)
        .or_insert(1);
    let Some(search_start) = search_start else {
        return;
    };
    let Some(threshold) = crossing.diagnostics.trace_crossing_pending_threshold else {
        return;
    };
    if *count != threshold {
        return;
    }
    if let Some(trace_net_id) = crossing.diagnostics.trace_crossing_net {
        if trace_net_id != crossing.net_id {
            return;
        }
    }
    eprintln!(
        "crossing-pending-straight-threshold net={} partner={} threshold={} count={} elapsed_ms={}",
        crossing.net_id,
        partner.net_id,
        threshold,
        count,
        search_start.elapsed().as_millis(),
    );
}

pub(crate) fn record_crossing_hotpath_owner_count(
    stats: &mut RouteSearchStats,
    owner_count: usize,
) {
    match owner_count {
        0 => stats.crossing_hotpath_no_owner_contacts += 1,
        1 => stats.crossing_hotpath_single_owner_contacts += 1,
        _ => stats.crossing_hotpath_multi_owner_contacts += 1,
    }
}

pub(crate) fn record_perpendicular_crossing_reject(
    stats: &mut RouteSearchStats,
    crossing: &CrossingSearchConfig,
    partner_net_id: NetId,
    search_start: Option<&Instant>,
) {
    if !crossing.diagnostics.analysis_crossing_partner_counters {
        return;
    }
    let count = stats
        .crossing_perpendicular_reject_by_partner
        .entry(partner_net_id)
        .and_modify(|count| *count += 1)
        .or_insert(1);
    let Some(search_start) = search_start else {
        return;
    };
    let Some(threshold) = crossing.diagnostics.trace_crossing_perp_reject_threshold else {
        return;
    };
    if *count != threshold {
        return;
    }
    if let Some(trace_net_id) = crossing.diagnostics.trace_crossing_net {
        if trace_net_id != crossing.net_id {
            return;
        }
    }
    eprintln!(
        "crossing-perpendicular-reject-threshold net={} partner={} threshold={} count={} elapsed_ms={}",
        crossing.net_id,
        partner_net_id,
        threshold,
        count,
        search_start.elapsed().as_millis(),
    );
}

/// One `crossed_mask` bit per partner with `single_discounted_crossing`
/// (0 for the others and for budgeted partners beyond the 64th). In lidar
/// modes `crossed_mask` is otherwise unused (window-mode partner tracking is
/// off), so the budget bits can live in the same key field.
pub(crate) fn crossing_partner_budget_bits(crossing: &CrossingSearchConfig) -> Vec<u64> {
    let mut next_bit = 0u32;
    crossing
        .partners
        .iter()
        .map(|partner| {
            if partner.single_discounted_crossing && next_bit < 64 {
                let bit = 1u64 << next_bit;
                next_bit += 1;
                bit
            } else {
                0
            }
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn crossing_move_outcome_with_segments(
    obstacle_map: &ObstacleMap,
    crossing: &CrossingSearchConfig,
    current_key: CrossingAStarKey,
    state: State,
    primitive: &Primitive,
    is_straight: bool,
    required_margin: i32,
    capped_required_margin: i32,
    reservation_margin: i32,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    partner_index_by_id: &FxHashMap<NetId, usize>,
    partner_budget_bits: &[u64],
    partner_segments: &[Vec<PartnerPathSegment>],
    dynamic_core_owners: &DenseDynamicCoreOwnerGrid,
    primitive_crossing: &PrimitiveCrossingMetadata,
    scan_extra_witnesses_only: bool,
    require_dynamic_owner_contact: bool,
    collect_detailed_timing: bool,
    stats: &mut RouteSearchStats,
    search_start: Option<&Instant>,
) -> Option<CrossingMoveOutcome> {
    let hotpath_total_start = collect_detailed_timing.then(Instant::now);
    stats.crossing_hotpath_contact_checks += 1;
    let primitive_steps = primitive.dx.abs().max(primitive.dy.abs());
    let pending_before = f64::from(current_key.pending_after_crossing_cells);
    let initial_run_distance = primitive_initial_straight_run_distance(primitive, state.angle);
    let mut partial_pending_carry: i32 = 0;
    if pending_before > 0.0 {
        if current_key.pending_after_crossing_angle != state.angle {
            trace_crossing_pending(
                crossing,
                "reject_pending_direction",
                state,
                primitive,
                current_key.pending_after_crossing_cells,
                initial_run_distance,
                Some(current_key.pending_after_crossing_angle),
                None,
            );
            stats.crossing_reject_pending_straight += 1;
            record_pending_after_crossing_reject(
                stats,
                crossing,
                current_key.pending_after_crossing_partner_index,
                search_start,
            );
            return None;
        }
        // A pure straight in the pending direction that is shorter than the
        // remaining debt pays PART of it and carries the rest (4 + 1 = 5):
        // the rule is "straight for `required_margin` cells after the
        // crossing", and consecutive straight moves satisfy it exactly as
        // one long move would. Only a move that would BEND before the debt
        // is paid (or leaves the pending direction) is rejected. Without
        // this, a debt larger than the longest straight primitive is
        // unpayable whenever the crossing lands in the last cell of its
        // move -- the multiportmmi_32x32 fan wall (see
        // .agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md).
        // The debt is the crossing element's own straight (+-half_size around
        // the point) and is paid ONLY by pure straight primitives in the
        // pending direction, partially if the primitive is shorter than the
        // remaining debt (4 + 1 pays 5). A bend never pays: its first arm is
        // `bend_radius` cells of ARC when realized (make_turn arms equal the
        // radius), so counting it as straight would let the arc start on the
        // crossing point. Once the debt is zero a bend may follow -- its arc
        // lies inside its own footprint and never reaches back.
        let pure_straight_in_pending_direction = is_straight
            && primitive.end_angle == state.angle
            && matches!(primitive.geometry, PrimitiveGeometry::Straight { .. });
        if !pure_straight_in_pending_direction || initial_run_distance + 1.0e-9 < pending_before {
            if !pure_straight_in_pending_direction {
                trace_crossing_pending(
                    crossing,
                    "reject_pending_initial_run",
                    state,
                    primitive,
                    current_key.pending_after_crossing_cells,
                    initial_run_distance,
                    Some(current_key.pending_after_crossing_angle),
                    None,
                );
                stats.crossing_reject_pending_straight += 1;
                record_pending_after_crossing_reject(
                    stats,
                    crossing,
                    current_key.pending_after_crossing_partner_index,
                    search_start,
                );
                return None;
            }
            partial_pending_carry = current_key
                .pending_after_crossing_cells
                .saturating_sub(initial_run_distance.floor() as i32)
                .max(1);
            trace_crossing_pending(
                crossing,
                "consume_pending_partial",
                state,
                primitive,
                current_key.pending_after_crossing_cells,
                initial_run_distance,
                Some(current_key.pending_after_crossing_angle),
                None,
            );
        }
        trace_crossing_pending(
            crossing,
            "consume_pending_initial_run",
            state,
            primitive,
            current_key.pending_after_crossing_cells,
            initial_run_distance,
            Some(current_key.pending_after_crossing_angle),
            None,
        );
    }
    let mut straight_run = if is_straight && primitive.end_angle == state.angle {
        current_key
            .straight_run_cells
            .saturating_add(primitive_steps)
            .min(capped_required_margin)
    } else {
        0
    };
    let mut pending_after = 0;
    let mut pending_after_angle = NO_PENDING_CROSSING_ANGLE;
    let mut pending_after_partner_index = NO_PENDING_CROSSING_PARTNER_INDEX;
    if partial_pending_carry > 0 {
        pending_after = partial_pending_carry;
        pending_after_angle = current_key.pending_after_crossing_angle;
        pending_after_partner_index = current_key.pending_after_crossing_partner_index;
    }
    let mut active_reservation_keys = Vec::new();
    let mut pending_reservation_keys = Vec::new();
    let mut crossed_mask = current_key.crossed_mask;
    let mut next_partner_index = current_key.next_partner_index;
    let track_crossed_partners = crossing.require_all_partners;
    let mut crossing_count = 0u32;
    let mut crossing_cost = 0.0_f64;
    let primitive_segments = &primitive_crossing.segments;
    let mut contacted_partners = ContactedPartners::default();
    // A free footprint only needs its halo witnesses (the between-cells X
    // of each diagonal step). A BLOCKED footprint must still scan the halo
    // too: a multi-cell diagonal can carry a footprint contact with one
    // partner and, on another step, a between-cells X with a second partner
    // -- scanning footprint witnesses only let that second crossing pass
    // unregistered (benes_32x32 net 273, two parallel diagonals 2.5 steps
    // apart, 2026-09-03). `require_dynamic_owner_contact` keeps its meaning:
    // at least one owner must be found.
    let witness_scan = if scan_extra_witnesses_only {
        primitive_crossing.extra_witnesses.as_slice()
    } else {
        primitive_crossing.witnesses.as_slice()
    };
    let owner_scan_start = collect_detailed_timing.then(Instant::now);
    for witness in witness_scan {
        stats.crossing_hotpath_witness_cells_scanned += 1;
        let cell = (state.x + witness.offset.0, state.y + witness.offset.1);
        if !obstacle_map.in_bounds(cell.0, cell.1) {
            stats.crossing_reject_unmatched_footprint += 1;
            return None;
        }
        if obstacle_map.is_static_blocked(cell.0, cell.1)
            && !port_open_cells.is_some_and(|open| open.contains(&pack_xy(cell.0, cell.1)))
        {
            stats.crossing_hotpath_static_rejects += 1;
            stats.crossing_reject_unmatched_footprint += 1;
            return None;
        }
        let Some(owner) = dynamic_core_owners.owner_at(cell.0, cell.1) else {
            continue;
        };
        if owner == crossing.net_id {
            continue;
        }
        if !is_straight {
            record_crossing_hotpath_owner_count(stats, contacted_partners.len().max(1));
            stats.crossing_reject_non_straight += 1;
            return None;
        }
        let Some(partner_idx) = partner_index_by_id.get(&owner).copied() else {
            record_crossing_hotpath_owner_count(stats, contacted_partners.len() + 1);
            stats.crossing_reject_unexpected_owner += 1;
            if crossing.diagnostics.move_diag {
                eprintln!(
                    "unexpected-owner seq={} net={} cell=({},{}) owner={} state=({},{},{}) partners={}",
                    current_search_seq(),
                    crossing.net_id,
                    cell.0,
                    cell.1,
                    owner,
                    state.x,
                    state.y,
                    state.angle,
                    partner_index_by_id.len()
                );
            }
            return None;
        };
        contacted_partners.push_witness(
            partner_idx,
            EffectiveCollisionWitness {
                cell,
                route_segment_idx: witness.route_segment_idx,
            },
        );
    }
    if let Some(owner_scan_start) = owner_scan_start {
        stats.crossing_hotpath_owner_scan_time_us += owner_scan_start.elapsed().as_micros();
    }
    let dynamic_owner_count = contacted_partners.len();
    record_crossing_hotpath_owner_count(stats, dynamic_owner_count);
    if require_dynamic_owner_contact && dynamic_owner_count == 0 {
        if let Some(hotpath_total_start) = hotpath_total_start {
            stats.crossing_hotpath_total_time_us += hotpath_total_start.elapsed().as_micros();
        }
        return None;
    }
    let mut route_intersections = Vec::new();
    let segment_search_start = collect_detailed_timing.then(Instant::now);
    if primitive_steps > 0 {
        for contact in contacted_partners.iter() {
            let partner_idx = contact.partner_idx;
            let partner = &crossing.partners[partner_idx];
            let bit = if track_crossed_partners {
                1u64 << partner_idx
            } else {
                0
            };
            let intersection_count_before = route_intersections.len();
            for (route_segment_idx, relative_route_segment) in primitive_segments.iter().enumerate()
            {
                let route_segment =
                    translate_primitive_path_segment(state, *relative_route_segment);
                for partner_segment in partner_segments
                    .get(partner_idx)
                    .map(Vec::as_slice)
                    .unwrap_or(&[])
                {
                    stats.crossing_hotpath_partner_segment_checks += 1;
                    if !route_partner_segment_bboxes_overlap(&route_segment, partner_segment) {
                        stats.crossing_hotpath_partner_segment_bbox_rejects += 1;
                        continue;
                    }
                    let Some((x, y, t, u)) = grid_segment_intersection_with_params(
                        route_segment.start,
                        route_segment.end,
                        partner_segment.start,
                        partner_segment.end,
                    ) else {
                        continue;
                    };
                    stats.crossing_hotpath_intersection_hits += 1;
                    stats.crossing_candidate_checks += 1;
                    let distance_on_segment = t * route_segment.length;
                    let distance_from_primitive_start =
                        route_segment.distance_before_segment + distance_on_segment;
                    if track_crossed_partners
                        && crossed_mask & bit != 0
                        && distance_from_primitive_start <= 1.0e-9
                    {
                        continue;
                    }
                    // Same situation without partner tracking (lidar-pure):
                    // when a crossing sat at the END cell of the move that
                    // recorded it, the move paying its straight-after debt
                    // starts ON that cell and re-finds the same intersection
                    // at t=0. That is the crossing just recorded, not a new
                    // one inside the pending zone -- skip it. Without this
                    // every end-of-move crossing was rejected one move later
                    // (the multiportmmi_32x32 fan wall, where each crossing's
                    // debt run lands exactly on the next partner).
                    if pending_before > 0.0
                        && partner_idx
                            == usize::from(current_key.pending_after_crossing_partner_index)
                        && distance_from_primitive_start <= 1.0e-9
                    {
                        continue;
                    }
                    if track_crossed_partners && crossed_mask & bit != 0 {
                        stats.crossing_reject_wrong_order += 1;
                        return None;
                    }
                    if !grid_axes_are_perpendicular(route_segment.angle, partner_segment.angle) {
                        trace_crossing_candidate(
                            crossing,
                            partner.net_id,
                            "not_perpendicular",
                            x,
                            y,
                            route_segment.angle,
                            partner_segment.angle,
                            required_margin,
                            0.0,
                            distance_from_primitive_start,
                            route_segment.length - distance_on_segment,
                        );
                        stats.crossing_reject_not_perpendicular += 1;
                        return None;
                    }
                    let partner_margin =
                        (u * partner_segment.length).min((1.0 - u) * partner_segment.length);
                    if partner_margin + 1.0e-9 < f64::from(required_margin) {
                        record_perpendicular_crossing_reject(
                            stats,
                            crossing,
                            partner.net_id,
                            search_start,
                        );
                        trace_crossing_candidate(
                            crossing,
                            partner.net_id,
                            "partner_margin",
                            x,
                            y,
                            route_segment.angle,
                            partner_segment.angle,
                            required_margin,
                            partner_margin,
                            distance_from_primitive_start,
                            route_segment.length - distance_on_segment,
                        );
                        stats.crossing_reject_margin += 1;
                        return None;
                    }
                    if !crossing_reservation_window_is_clear(
                        obstacle_map,
                        crossing.net_id,
                        partner.net_id,
                        port_open_cells,
                        x,
                        y,
                        reservation_margin,
                    ) {
                        record_perpendicular_crossing_reject(
                            stats,
                            crossing,
                            partner.net_id,
                            search_start,
                        );
                        trace_crossing_candidate(
                            crossing,
                            partner.net_id,
                            "reservation_footprint",
                            x,
                            y,
                            route_segment.angle,
                            partner_segment.angle,
                            required_margin,
                            partner_margin,
                            distance_from_primitive_start,
                            route_segment.length - distance_on_segment,
                        );
                        stats.crossing_reject_unmatched_footprint += 1;
                        return None;
                    }
                    if !terminal_bump_guard_satisfied(
                        crossing.terminal_bump_guard,
                        crossing.crossing_half_size_cells,
                        x,
                        y,
                        route_segment.angle,
                    ) {
                        record_perpendicular_crossing_reject(
                            stats,
                            crossing,
                            partner.net_id,
                            search_start,
                        );
                        trace_crossing_candidate(
                            crossing,
                            partner.net_id,
                            "terminal_bump_distance",
                            x,
                            y,
                            route_segment.angle,
                            partner_segment.angle,
                            required_margin,
                            partner_margin,
                            distance_from_primitive_start,
                            route_segment.length - distance_on_segment,
                        );
                        stats.crossing_reject_margin += 1;
                        return None;
                    }
                    if !terminal_bump_guard_satisfied(
                        partner.target_terminal_bump_guard,
                        crossing.crossing_half_size_cells,
                        x,
                        y,
                        partner_segment.angle,
                    ) {
                        record_perpendicular_crossing_reject(
                            stats,
                            crossing,
                            partner.net_id,
                            search_start,
                        );
                        trace_crossing_candidate(
                            crossing,
                            partner.net_id,
                            "partner_terminal_bump_distance",
                            x,
                            y,
                            route_segment.angle,
                            partner_segment.angle,
                            required_margin,
                            partner_margin,
                            distance_from_primitive_start,
                            route_segment.length - distance_on_segment,
                        );
                        stats.crossing_reject_margin += 1;
                        return None;
                    }
                    if !route_intersections
                        .iter()
                        .any(|existing: &CrossingRouteIntersection| {
                            existing.partner_idx == partner_idx
                                && (existing.x - x).abs() <= 1.0e-9
                                && (existing.y - y).abs() <= 1.0e-9
                        })
                    {
                        route_intersections.push(CrossingRouteIntersection {
                            distance_from_primitive_start,
                            distance_before_on_segment: if route_segment.starts_after_kink {
                                distance_on_segment
                            } else {
                                current_key.straight_run_cells as f64 + distance_on_segment
                            },
                            distance_after_on_segment: route_segment.length - distance_on_segment,
                            x,
                            y,
                            route_angle: route_segment.angle,
                            partner_angle: partner_segment.angle,
                            segment_is_terminal: route_segment_idx + 1 == primitive_segments.len(),
                            partner_idx,
                            bit,
                        });
                    }
                }
            }
            if route_intersections.len() == intersection_count_before {
                if track_crossed_partners && crossed_mask & bit != 0 {
                    if !is_straight {
                        stats.crossing_reject_non_straight += 1;
                        return None;
                    }
                    continue;
                }
                // Contact with the partner whose straight-after debt this
                // move is paying is the crossing's own neighbourhood: the
                // partner's centerline continues from the crossing cell in
                // both directions and its cells sit in this straight move's
                // diagonal halo. The partner-margin check already guarantees
                // the partner is straight throughout the pending zone, so a
                // second genuine intersection here is geometrically
                // impossible -- this is the lidar-pure counterpart of the
                // tracked-partner `continue` above.
                if pending_before > 0.0
                    && partner_idx == usize::from(current_key.pending_after_crossing_partner_index)
                {
                    continue;
                }
                let reject = classify_unresolved_crossing_contact(
                    contact,
                    primitive_segments,
                    state,
                    partner,
                );
                if crossing.diagnostics.trace_crossing_level1 {
                    trace_crossing_level1_intersection(
                        crossing,
                        partner.net_id,
                        match reject {
                            CrossingContactReject::NotPerpendicular => {
                                "reject_contact_not_perpendicular"
                            }
                            CrossingContactReject::Unmatched => "reject_contact_unmatched",
                            CrossingContactReject::Footprint => "reject_contact_footprint",
                        },
                        f64::from(contact.first_witness.cell.0),
                        f64::from(contact.first_witness.cell.1),
                        255,
                        255,
                        required_margin,
                        0.0,
                        0.0,
                        0,
                    );
                }
                match reject {
                    CrossingContactReject::NotPerpendicular => {
                        stats.crossing_reject_not_perpendicular += 1
                    }
                    CrossingContactReject::Unmatched => {
                        stats.crossing_reject_unmatched_centerline += 1
                    }
                    CrossingContactReject::Footprint => {
                        stats.crossing_reject_unmatched_footprint += 1
                    }
                }
                return None;
            }
        }
        route_intersections.sort_by(|a, b| {
            a.distance_from_primitive_start
                .partial_cmp(&b.distance_from_primitive_start)
                .unwrap_or(Ordering::Equal)
        });
    }
    if let Some(segment_search_start) = segment_search_start {
        stats.crossing_hotpath_segment_time_us += segment_search_start.elapsed().as_micros();
    }

    let reservation_start = collect_detailed_timing.then(Instant::now);
    for intersection in &route_intersections {
        if pending_before > 0.0
            && intersection.distance_from_primitive_start + 1.0e-9 < pending_before
        {
            let partner = &crossing.partners[intersection.partner_idx];
            record_perpendicular_crossing_reject(stats, crossing, partner.net_id, search_start);
            record_pending_after_crossing_partner(
                stats,
                crossing,
                current_key.pending_after_crossing_partner_index,
                search_start,
            );
            stats.crossing_reject_pending_straight += 1;
            return None;
        }
        if crossing.require_all_partners
            && intersection.partner_idx != usize::from(next_partner_index)
        {
            stats.crossing_reject_wrong_order += 1;
            return None;
        }
        if intersection.distance_before_on_segment + 1.0e-9 < f64::from(required_margin) {
            let partner = &crossing.partners[intersection.partner_idx];
            record_perpendicular_crossing_reject(stats, crossing, partner.net_id, search_start);
            trace_crossing_candidate(
                crossing,
                partner.net_id,
                "route_before_margin",
                intersection.x,
                intersection.y,
                255,
                255,
                required_margin,
                0.0,
                intersection.distance_before_on_segment,
                intersection.distance_after_on_segment,
            );
            stats.crossing_reject_margin += 1;
            return None;
        }
        // Straight-after debt = what the crossing element needs (+-half_size
        // around the point), measured from the intersection itself, not from
        // the halo contact or the move end. `required_margin` (half_size +
        // bend_radius) stays the rule for the PARTNER's straight and for
        // `route_before` (where a preceding bend's terminal arm is counted
        // as straight run but is arc when realized); after the crossing the
        // debt is paid by pure straights only, so no bend-radius allowance
        // is needed here. This matches the realized validator
        // (`realized_crossing_margin_um` = half_size * grid).
        let debt_basis = crossing.crossing_half_size_cells.max(0);
        let missing_after = (f64::from(debt_basis) - intersection.distance_after_on_segment)
            .ceil()
            .max(0.0) as i32;
        let reservation_keys = local_crossing_reservation_window_keys(
            intersection.x,
            intersection.y,
            crossing.crossing_half_size_cells,
            obstacle_map.width(),
            obstacle_map.height(),
        );
        // Two crossing elements of one route must not overlap: the new
        // window must be disjoint from every window this move already
        // reserved (earlier moves are checked by the kernel against the
        // node chain). Same predicate as the post-search
        // `crossing_events_have_disjoint_reservations`.
        if reservation_keys.iter().any(|key| {
            active_reservation_keys.contains(key) || pending_reservation_keys.contains(key)
        }) {
            let partner = &crossing.partners[intersection.partner_idx];
            record_perpendicular_crossing_reject(stats, crossing, partner.net_id, search_start);
            trace_crossing_candidate(
                crossing,
                partner.net_id,
                "reservation_overlap",
                intersection.x,
                intersection.y,
                intersection.route_angle,
                intersection.partner_angle,
                required_margin,
                0.0,
                intersection.distance_before_on_segment,
                intersection.distance_after_on_segment,
            );
            stats.crossing_reject_reservation_overlap += 1;
            return None;
        }
        if missing_after > 0 {
            if !intersection.segment_is_terminal {
                let partner = &crossing.partners[intersection.partner_idx];
                record_perpendicular_crossing_reject(stats, crossing, partner.net_id, search_start);
                trace_crossing_candidate(
                    crossing,
                    partner.net_id,
                    "route_after_kink_margin",
                    intersection.x,
                    intersection.y,
                    intersection.route_angle,
                    intersection.partner_angle,
                    required_margin,
                    0.0,
                    intersection.distance_before_on_segment,
                    intersection.distance_after_on_segment,
                );
                stats.crossing_reject_pending_straight += 1;
                if crossing.diagnostics.analysis_crossing_partner_counters {
                    *stats
                        .crossing_after_margin_by_partner
                        .entry(partner.net_id)
                        .or_insert(0) += 1;
                }
                return None;
            }
            extend_unique_keys(&mut pending_reservation_keys, &reservation_keys);
            let partner = &crossing.partners[intersection.partner_idx];
            trace_crossing_pending(
                crossing,
                "set_pending_after_crossing",
                state,
                primitive,
                missing_after,
                initial_run_distance,
                primitive_segments
                    .iter()
                    .find(|segment| {
                        segment.distance_before_segment
                            <= intersection.distance_from_primitive_start + 1.0e-9
                            && intersection.distance_from_primitive_start
                                <= segment.distance_before_segment + segment.length + 1.0e-9
                    })
                    .map(|segment| segment.angle),
                Some(intersection.distance_after_on_segment),
            );
            if trace_crossing_pending_enabled(crossing) {
                eprintln!(
                    "crossing-pending seq={} net={} event=set_pending_partner partner={} point=({:.3},{:.3}) required={} distance_after={:.3}",
                    current_search_seq(),
                    crossing.net_id,
                    partner.net_id,
                    intersection.x,
                    intersection.y,
                    required_margin,
                    intersection.distance_after_on_segment,
                );
            }
        } else {
            extend_unique_keys(&mut active_reservation_keys, &reservation_keys);
        }
        let partner = &crossing.partners[intersection.partner_idx];
        trace_crossing_level1_intersection(
            crossing,
            partner.net_id,
            if missing_after > 0 {
                "accept_with_pending"
            } else {
                "accept"
            },
            intersection.x,
            intersection.y,
            intersection.route_angle,
            intersection.partner_angle,
            required_margin,
            intersection.distance_before_on_segment,
            intersection.distance_after_on_segment,
            missing_after,
        );
        if missing_after > pending_after {
            pending_after = missing_after;
            pending_after_angle = intersection.route_angle;
            pending_after_partner_index =
                u8::try_from(intersection.partner_idx).unwrap_or(NO_PENDING_CROSSING_PARTNER_INDEX);
        }
        if track_crossed_partners {
            crossed_mask |= intersection.bit;
            next_partner_index = next_partner_index.checked_add(1)?;
        }
        crossing_count += 1;
        stats.crossing_accepted += 1;
        let budget_bit = partner_budget_bits
            .get(intersection.partner_idx)
            .copied()
            .unwrap_or(0);
        match partner.crossing_loss_override {
            Some(_) if budget_bit != 0 && crossed_mask & budget_bit != 0 => {
                // S2: the planned crossing with this partner was already
                // spent on this path -- a braid pays the full price.
                crossing_cost += crossing.crossing_loss;
                stats.crossing_accepted_over_budget += 1;
            }
            Some(price) => {
                crossing_cost += price;
                crossed_mask |= budget_bit;
                stats.crossing_accepted_planned += 1;
            }
            None => crossing_cost += crossing.crossing_loss,
        }
    }
    if let Some(reservation_start) = reservation_start {
        stats.crossing_hotpath_reservation_time_us += reservation_start.elapsed().as_micros();
    }

    if !is_straight {
        straight_run = primitive_terminal_straight_run_cells(primitive, primitive.end_angle)
            .min(capped_required_margin);
    }

    let outcome = Some(CrossingMoveOutcome {
        crossed_mask,
        next_partner_index,
        straight_run_cells: straight_run,
        pending_after_crossing_cells: pending_after.min(capped_required_margin),
        pending_after_crossing_angle: if pending_after > 0 {
            pending_after_angle
        } else {
            NO_PENDING_CROSSING_ANGLE
        },
        pending_after_crossing_partner_index: if pending_after > 0 {
            pending_after_partner_index
        } else {
            NO_PENDING_CROSSING_PARTNER_INDEX
        },
        crossing_count,
        crossing_cost,
        active_reservation_keys,
        pending_reservation_keys,
        contact_only_pending_partner: dynamic_owner_count > 0
            && contacted_partners.iter().all(|contact| {
                pending_before > 0.0
                    && contact.partner_idx
                        == usize::from(current_key.pending_after_crossing_partner_index)
            }),
    });
    if let Some(hotpath_total_start) = hotpath_total_start {
        stats.crossing_hotpath_total_time_us += hotpath_total_start.elapsed().as_micros();
    }
    outcome
}

pub(crate) fn effective_collision_witness_offsets(
    primitive: &Primitive,
    route_segments: &[RelativePrimitivePathSegment],
) -> Vec<RelativeEffectiveCollisionWitness> {
    let mut witnesses = Vec::new();
    for (dx, dy) in primitive.footprint.iter().copied() {
        push_unique_relative_witness(&mut witnesses, (dx, dy), None);
    }

    for (segment_idx, route_segment) in route_segments.iter().enumerate() {
        let dx = (route_segment.end.0 - route_segment.start.0).signum();
        let dy = (route_segment.end.1 - route_segment.start.1).signum();
        let steps = (route_segment.end.0 - route_segment.start.0)
            .abs()
            .max((route_segment.end.1 - route_segment.start.1).abs());
        if steps <= 0 || dx == 0 || dy == 0 {
            continue;
        }
        for step in 0..steps {
            let start = (
                route_segment.start.0 + dx * step,
                route_segment.start.1 + dy * step,
            );
            let end = (start.0 + dx, start.1 + dy);
            for cell in compact_diagonal_halo_cells(start, end, dx, dy) {
                push_unique_relative_witness(&mut witnesses, cell, Some(segment_idx));
            }
        }
    }

    witnesses
}

pub(crate) fn push_unique_relative_witness(
    witnesses: &mut Vec<RelativeEffectiveCollisionWitness>,
    offset: (i32, i32),
    route_segment_idx: Option<usize>,
) {
    if witnesses.iter().any(|witness| witness.offset == offset) {
        return;
    }
    witnesses.push(RelativeEffectiveCollisionWitness {
        offset,
        route_segment_idx,
    });
}

pub(crate) fn classify_unresolved_crossing_contact(
    contact: &ContactedPartner,
    route_segments: &[RelativePrimitivePathSegment],
    state: State,
    partner: &CrossingSearchPartner,
) -> CrossingContactReject {
    let mut saw_partner_segment = false;
    let mut saw_non_perpendicular = false;
    update_unresolved_crossing_contact_classification(
        contact.first_witness,
        route_segments,
        state,
        partner,
        &mut saw_partner_segment,
        &mut saw_non_perpendicular,
    );
    for witness in contact.extra_witnesses.iter().copied() {
        update_unresolved_crossing_contact_classification(
            witness,
            route_segments,
            state,
            partner,
            &mut saw_partner_segment,
            &mut saw_non_perpendicular,
        );
    }
    if saw_non_perpendicular {
        CrossingContactReject::NotPerpendicular
    } else if saw_partner_segment {
        CrossingContactReject::Unmatched
    } else {
        CrossingContactReject::Footprint
    }
}

pub(crate) fn update_unresolved_crossing_contact_classification(
    witness: EffectiveCollisionWitness,
    route_segments: &[RelativePrimitivePathSegment],
    state: State,
    partner: &CrossingSearchPartner,
    saw_partner_segment: &mut bool,
    saw_non_perpendicular: &mut bool,
) {
    for partner_segment in partner.waypoints.windows(2) {
        if grid_point_on_segment_with_param(witness.cell, partner_segment[0], partner_segment[1])
            .is_none()
        {
            continue;
        }
        *saw_partner_segment = true;
        let Some(route_segment_idx) = witness.route_segment_idx else {
            continue;
        };
        let Some(relative_route_segment) = route_segments.get(route_segment_idx) else {
            continue;
        };
        let route_segment = translate_primitive_path_segment(state, *relative_route_segment);
        let Some(partner_angle) =
            direction_angle_between_grid_cells(partner_segment[0], partner_segment[1])
        else {
            continue;
        };
        if !grid_axes_are_perpendicular(route_segment.angle, partner_angle) {
            *saw_non_perpendicular = true;
        }
    }
}

pub(crate) fn crossing_reservation_window_is_clear(
    obstacle_map: &ObstacleMap,
    net_id: NetId,
    partner_net_id: NetId,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    center_x: f64,
    center_y: f64,
    half_size_cells: i32,
) -> bool {
    if half_size_cells < 0 {
        return false;
    }
    let min_x = (center_x - f64::from(half_size_cells)).floor() as i32;
    let max_x = (center_x + f64::from(half_size_cells)).ceil() as i32;
    let min_y = (center_y - f64::from(half_size_cells)).floor() as i32;
    let max_y = (center_y + f64::from(half_size_cells)).ceil() as i32;
    for x in min_x..=max_x {
        for y in min_y..=max_y {
            if !obstacle_map.in_bounds(x, y) {
                return false;
            }
            let is_opened_static = port_open_cells
                .map(|open| open.contains(&pack_xy(x, y)))
                .unwrap_or(false);
            if obstacle_map.is_static_blocked(x, y) && !is_opened_static {
                return false;
            }
            for owner in obstacle_map.dynamic_owners_at(x, y) {
                if owner != net_id && owner != partner_net_id {
                    return false;
                }
            }
        }
    }
    true
}

pub(crate) fn local_crossing_reservation_window_keys(
    center_x: f64,
    center_y: f64,
    half_size_cells: i32,
    width: i32,
    height: i32,
) -> Vec<CellKey> {
    let mut keys = Vec::new();
    if half_size_cells < 0 {
        return keys;
    }
    let min_x = (center_x - f64::from(half_size_cells)).floor() as i32;
    let max_x = (center_x + f64::from(half_size_cells)).ceil() as i32;
    let min_y = (center_y - f64::from(half_size_cells)).floor() as i32;
    let max_y = (center_y + f64::from(half_size_cells)).ceil() as i32;
    for x in min_x..=max_x {
        if x < 0 || x >= width {
            continue;
        }
        for y in min_y..=max_y {
            if y < 0 || y >= height {
                continue;
            }
            let key = pack_xy(x, y);
            if !keys.contains(&key) {
                keys.push(key);
            }
        }
    }
    keys
}

pub(crate) fn extend_unique_keys(out: &mut Vec<CellKey>, keys: &[CellKey]) {
    for key in keys {
        if !out.contains(key) {
            out.push(*key);
        }
    }
}

pub(crate) fn grid_point_on_segment_with_param(
    point: (i32, i32),
    start: (i32, i32),
    end: (i32, i32),
) -> Option<f64> {
    let dx = i64::from(end.0 - start.0);
    let dy = i64::from(end.1 - start.1);
    if dx == 0 && dy == 0 {
        return None;
    }
    let qx = i64::from(point.0 - start.0);
    let qy = i64::from(point.1 - start.1);
    if qx * dy - qy * dx != 0 {
        return None;
    }
    let dot = qx * dx + qy * dy;
    let len_sq = dx * dx + dy * dy;
    if dot < 0 || dot > len_sq {
        return None;
    }
    Some(dot as f64 / len_sq as f64)
}

pub(crate) fn crossing_progress_heuristic(
    key: CrossingAStarKey,
    crossing: &CrossingSearchConfig,
    required_margin: i32,
    grid_size_um: f64,
) -> f64 {
    if !crossing.require_all_partners {
        return 0.0;
    }
    let next_idx = usize::from(key.next_partner_index);
    let progress_bonus = next_idx as f64 * 10_000.0;
    // Use a weak guide toward the next scheduled partner. The guide is small
    // enough that target-directed path shape still dominates simple X routes.
    let Some(partner) = crossing.partners.get(next_idx) else {
        return -progress_bonus;
    };
    let point = (f64::from(key.state.x), f64::from(key.state.y));
    let mut best_distance = f64::INFINITY;
    for segment in partner.waypoints.windows(2) {
        let distance =
            distance_to_trimmed_grid_segment(point, segment[0], segment[1], required_margin);
        if distance < best_distance {
            best_distance = distance;
        }
    }
    if best_distance.is_finite() {
        best_distance * grid_size_um * 0.5 - progress_bonus
    } else {
        -progress_bonus
    }
}

pub(crate) fn distance_to_trimmed_grid_segment(
    point: (f64, f64),
    start: (i32, i32),
    end: (i32, i32),
    trim_cells: i32,
) -> f64 {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let steps = dx.abs().max(dy.abs());
    if steps <= 0 || steps < 2 * trim_cells {
        return f64::INFINITY;
    }
    let dir_x = dx.signum();
    let dir_y = dy.signum();
    let trimmed_start = (
        f64::from(start.0 + dir_x * trim_cells),
        f64::from(start.1 + dir_y * trim_cells),
    );
    let trimmed_end = (
        f64::from(end.0 - dir_x * trim_cells),
        f64::from(end.1 - dir_y * trim_cells),
    );
    distance_to_segment(point, trimmed_start, trimmed_end)
}

pub(crate) fn distance_to_segment(point: (f64, f64), start: (f64, f64), end: (f64, f64)) -> f64 {
    let vx = end.0 - start.0;
    let vy = end.1 - start.1;
    let wx = point.0 - start.0;
    let wy = point.1 - start.1;
    let len_sq = vx * vx + vy * vy;
    if len_sq <= 1.0e-12 {
        let dx = point.0 - start.0;
        let dy = point.1 - start.1;
        return (dx * dx + dy * dy).sqrt();
    }
    let t = ((wx * vx + wy * vy) / len_sq).clamp(0.0, 1.0);
    let projection = (start.0 + t * vx, start.1 + t * vy);
    let dx = point.0 - projection.0;
    let dy = point.1 - projection.1;
    (dx * dx + dy * dy).sqrt()
}

pub(crate) fn direction_angle_between_grid_cells(a: (i32, i32), b: (i32, i32)) -> Option<u8> {
    let dx = (b.0 - a.0).signum();
    let dy = (b.1 - a.1).signum();
    DIRECTIONS
        .iter()
        .position(|dir| *dir == (dx, dy))
        .map(|idx| idx as u8)
}

pub(crate) fn grid_axes_are_perpendicular(a: u8, b: u8) -> bool {
    let delta = (i16::from(a) - i16::from(b)).rem_euclid(8);
    delta == 2 || delta == 6
}

pub(crate) fn grid_segment_step_count(a: (i32, i32), b: (i32, i32)) -> f64 {
    f64::from((b.0 - a.0).abs().max((b.1 - a.1).abs()))
}

pub(crate) fn grid_segment_intersection_with_params(
    a0: (i32, i32),
    a1: (i32, i32),
    b0: (i32, i32),
    b1: (i32, i32),
) -> Option<(f64, f64, f64, f64)> {
    let ax = f64::from(a0.0);
    let ay = f64::from(a0.1);
    let arx = f64::from(a1.0 - a0.0);
    let ary = f64::from(a1.1 - a0.1);
    let bx = f64::from(b0.0);
    let by = f64::from(b0.1);
    let brx = f64::from(b1.0 - b0.0);
    let bry = f64::from(b1.1 - b0.1);
    let denom = arx * bry - ary * brx;
    if denom.abs() < 1.0e-9 {
        return None;
    }
    let qpx = bx - ax;
    let qpy = by - ay;
    let t = (qpx * bry - qpy * brx) / denom;
    let u = (qpx * ary - qpy * arx) / denom;
    if !(-1.0e-9..=1.0 + 1.0e-9).contains(&t) || !(-1.0e-9..=1.0 + 1.0e-9).contains(&u) {
        return None;
    }
    Some((
        ax + t * arx,
        ay + t * ary,
        t.clamp(0.0, 1.0),
        u.clamp(0.0, 1.0),
    ))
}

#[cfg(test)]
mod tests;
