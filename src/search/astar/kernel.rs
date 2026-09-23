//! The unified A* kernel loop: `run` (formerly
//! `route_single_net_with_bounds_unified`), generic over a
//! `CrossingLegalityHook`, plus its open-set/node types and
//! route reconstruction. This is the single search kernel the whole crate
//! runs single-net routing through, whether crossings are enabled or not
//! (see the module doc comment below, kept from `mod unified_kernel`).
//! Moved out of `src/astar.rs` (Milestone 3, Slice 2, dissolving `mod
//! unified_kernel`); pure code motion, no behaviour change. Slice 3
//! rewrote the loop body into named steps: the kernel entry point is
//! `run`, and the expansion, insert-or-improve and eager post-crossing
//! completion steps live in `expansion.rs`.

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::PrimitiveLibrary;
use crate::search::astar::config::{
    AStarConfig, HeapTieBreaker, NO_GENERATION, NO_HEAP_POSITION, NO_PARENT,
};
use crate::search::astar::cost;
use crate::search::astar::crossing_rules::{CrossingExtension, CrossingLegalityHook};
use crate::search::astar::dense::{DenseRoutingGrid, DenseSearchStorage};
use crate::search::astar::diagnostics::{
    print_probe_success_report, print_search_failure_report, search_timed_out,
    should_check_timeout, trace_search_timeout, ExploredBox, SearchFailureEnv, SearchFailureState,
    CURRENT_SEARCH_SEQ,
};
use crate::search::astar::expansion::{
    candidate_moves, complete_crossing_corridor, goal_reached, move_is_legal,
    push_or_improve_dense, push_or_improve_extended, within_target_tolerance, CompletedCorridor,
    MoveContext, MoveDiagnostics, MoveVerdict, PrimitiveTables,
};
use crate::search::astar::heuristic::{target_angle_acceptance, SearchHeuristic};
use crate::search::astar::window::{window_area, RoutingBounds};
use crate::search::geometry::{
    compress_grid_waypoints, polyline_self_intersects, push_if_different,
};
use crate::search::state::{find_primitive, RouteResult, RouteSearchStats, State};
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::time::Instant;

#[derive(Clone, Copy, Debug)]
pub(crate) struct OpenEntry {
    pub(crate) f_score: f64,
    pub(crate) tie_score: f64,
    pub(crate) g_score: f64,
    pub(crate) counter: u32,
    pub(crate) generation: u32,
    pub(crate) idx: usize,
}

impl Eq for OpenEntry {}

impl PartialEq for OpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score && self.counter == other.counter
    }
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                other
                    .tie_score
                    .partial_cmp(&self.tie_score)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub(crate) struct IndexedOpenSet {
    pub(crate) heap: Vec<OpenEntry>,
    pub(crate) positions: Vec<usize>,
}

impl IndexedOpenSet {
    pub(crate) fn new(state_count: usize) -> Self {
        Self {
            heap: Vec::new(),
            positions: vec![NO_HEAP_POSITION; state_count],
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.heap.len()
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    pub(crate) fn push_or_decrease(&mut self, entry: OpenEntry) -> bool {
        let Some(position) = self.positions.get(entry.idx).copied() else {
            return false;
        };
        if position == NO_HEAP_POSITION {
            self.heap.push(entry);
            let new_position = self.heap.len() - 1;
            self.positions[entry.idx] = new_position;
            self.sift_up(new_position);
            return true;
        }

        if !entry_is_better(&entry, &self.heap[position]) {
            return false;
        }
        self.heap[position] = entry;
        self.sift_up(position);
        true
    }

    pub(crate) fn pop(&mut self) -> Option<OpenEntry> {
        if self.is_empty() {
            return None;
        }
        let popped = self.heap.swap_remove(0);
        self.positions[popped.idx] = NO_HEAP_POSITION;
        if !self.heap.is_empty() {
            self.positions[self.heap[0].idx] = 0;
            self.sift_down(0);
        }
        Some(popped)
    }

    pub(crate) fn peek(&self) -> Option<&OpenEntry> {
        self.heap.first()
    }

    pub(crate) fn sift_up(&mut self, mut position: usize) {
        while position > 0 {
            let parent = (position - 1) / 2;
            if !entry_is_better(&self.heap[position], &self.heap[parent]) {
                break;
            }
            self.swap_positions(position, parent);
            position = parent;
        }
    }

    pub(crate) fn sift_down(&mut self, mut position: usize) {
        loop {
            let left = position * 2 + 1;
            let right = left + 1;
            let mut best = position;
            if left < self.heap.len() && entry_is_better(&self.heap[left], &self.heap[best]) {
                best = left;
            }
            if right < self.heap.len() && entry_is_better(&self.heap[right], &self.heap[best]) {
                best = right;
            }
            if best == position {
                break;
            }
            self.swap_positions(position, best);
            position = best;
        }
    }

    pub(crate) fn swap_positions(&mut self, a: usize, b: usize) {
        self.heap.swap(a, b);
        self.positions[self.heap[a].idx] = a;
        self.positions[self.heap[b].idx] = b;
    }
}

pub(crate) fn entry_is_better(candidate: &OpenEntry, current: &OpenEntry) -> bool {
    candidate.cmp(current) == Ordering::Greater
}

/// Builds the per-search dense obstacle grid and per-search state storage
/// for one bounded window: `DenseSearchStorage` (g-costs, parent links,
/// the closed bitset) and `DenseRoutingGrid` (blocked/history/congestion
/// bitmaps, expanded for dynamic obstacles and opened port cells). Returns
/// `None`, with `stats.dense_grid_build_failures` incremented, if either
/// construction fails -- unchanged from the loop's own inline checks
/// (dense_storage_cap logs under `search_failure_diag`; the grid's own
/// build failure does not). `ignores_dynamic_obstacles` is threaded
/// through unchanged; it is only consulted later, by the pop-diagnostic.
/// Shared with `crate::search::grid_dijkstra`, which builds the same grid
/// for its own single full-grid attempt.
#[allow(clippy::too_many_arguments)]
pub(crate) fn build_search_grid(
    obstacle_map: &ObstacleMap,
    bounds: RoutingBounds,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    dynamic_expansion_radius_cells: i32,
    dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
    ignores_dynamic_obstacles: bool,
    source: State,
    target: State,
    stats: &mut RouteSearchStats,
) -> Option<(DenseSearchStorage, DenseRoutingGrid, bool)> {
    let Some(storage) = DenseSearchStorage::new(bounds, config.max_dense_states) else {
        // Silent before 2026-09-16: a window whose 8 x area exceeds
        // `max_dense_states` failed with 0 expansions and no trace
        // (multiportmmi_128x128, heater-to-MMI nets). Now counted and,
        // under PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG, reported.
        stats.dense_grid_build_failures += 1;
        if config.diagnostics.search_failure_diag {
            let area = window_area(bounds) as u64;
            eprintln!(
                    "search-failure kind=dense_storage_cap window=[{}..{}]x[{}..{}] area_cells={} states={} max_dense_states={} source=({},{}) target=({},{})",
                    bounds.min_x,
                    bounds.max_x,
                    bounds.min_y,
                    bounds.max_y,
                    area,
                    area * 8,
                    config.max_dense_states,
                    source.x,
                    source.y,
                    target.x,
                    target.y
                );
        }
        return None;
    };
    stats.dense_search_states = storage.state_count();
    stats.dense_search_storage_bytes = storage.allocated_bytes();
    let dense_grid = match DenseRoutingGrid::from_obstacle_map_with_dynamic_expansion(
        obstacle_map,
        bounds,
        port_open_cells,
        config.max_dense_obstacle_cells,
        config.ignore_dynamic_obstacles,
        config.history_weight > 0.0,
        config.long_straight_congestion_weight > 0.0,
        dynamic_expansion_radius_cells,
        dynamic_clearance_exempt_cells,
    ) {
        Some(grid) => grid,
        None => {
            stats.dense_grid_build_failures += 1;
            return None;
        }
    };
    stats.dense_grid_cells = dense_grid.blocked_count();
    stats.dense_grid_build_time_us = dense_grid.build_time_us();
    Some((storage, dense_grid, ignores_dynamic_obstacles))
}

/// Seeds the source state (g = 0) into Tier 1's open set, exactly like a
/// plain A* seeding its start node -- the source can never itself carry
/// crossing bookkeeping, so this never touches Tier 2. Returns the
/// source's dense-array index, needed again at goal reconstruction. Shared
/// with `crate::search::grid_dijkstra`, whose zero heuristic makes the
/// seeded `f_score` equal to the source's g-cost of 0.
pub(crate) fn seed_open_set<H: CrossingLegalityHook>(
    storage: &mut DenseSearchStorage,
    tier1_open: &mut OpenSet,
    source: State,
    search_heuristic: &SearchHeuristic,
    hook: &H,
    config: &AStarConfig,
    stats: &mut RouteSearchStats,
    counter: &mut u32,
) -> Option<usize> {
    let source_idx = storage.state_to_idx(source)?;
    storage.g_costs[source_idx] = 0.0;
    let source_generation = next_search_generation(counter)?;
    storage.best_generation[source_idx] = source_generation;
    stats.best_cost_updates += 1;
    tier1_open.push(OpenEntry {
        f_score: cost::heuristic_estimate(search_heuristic, source)
            + hook.heuristic_bonus(source, CrossingExtension::default()),
        tie_score: heap_tie_score(0.0, config.heap_tie_breaker),
        g_score: 0.0,
        counter: source_generation,
        generation: source_generation,
        idx: source_idx,
    });
    stats.heap_pushes += 1;
    stats.max_heap_size = stats.max_heap_size.max(tier1_open.len());
    Some(source_idx)
}

#[inline]
pub(crate) fn heap_tie_score(g_score: f64, tie_breaker: HeapTieBreaker) -> f64 {
    match tie_breaker {
        HeapTieBreaker::SmallerG => g_score,
        HeapTieBreaker::LargerG => -g_score,
    }
}

pub(crate) fn next_search_generation(counter: &mut u32) -> Option<u32> {
    if *counter == NO_GENERATION {
        return None;
    }
    let current = *counter;
    *counter = counter.checked_add(1)?;
    Some(current)
}

pub(crate) enum OpenSet {
    Duplicate(BinaryHeap<OpenEntry>),
    Indexed(IndexedOpenSet),
}

impl OpenSet {
    pub(crate) fn new(use_indexed_heap: bool, state_count: usize) -> Self {
        if use_indexed_heap {
            Self::Indexed(IndexedOpenSet::new(state_count))
        } else {
            Self::Duplicate(BinaryHeap::new())
        }
    }

    pub(crate) fn push(&mut self, entry: OpenEntry) -> bool {
        match self {
            Self::Duplicate(heap) => {
                heap.push(entry);
                true
            }
            Self::Indexed(heap) => heap.push_or_decrease(entry),
        }
    }

    pub(crate) fn pop(&mut self) -> Option<OpenEntry> {
        match self {
            Self::Duplicate(heap) => heap.pop(),
            Self::Indexed(heap) => heap.pop(),
        }
    }

    pub(crate) fn peek(&self) -> Option<&OpenEntry> {
        match self {
            Self::Duplicate(heap) => heap.peek(),
            Self::Indexed(heap) => heap.peek(),
        }
    }

    pub(crate) fn len(&self) -> usize {
        match self {
            Self::Duplicate(heap) => heap.len(),
            Self::Indexed(heap) => heap.len(),
        }
    }
}

/// The single A* search kernel for single-net routing, replacing the two
/// independently-implemented kernels this repository used to carry (see
/// `.agent/execplans/2026-08-25-unify-astar-kernel-and-clean-repair-baseline.md`
/// for the full design history and migration record). Wired into every
/// production call site via `crate::search::AStarSearch`'s `NetSearch` impl,
/// plus the crate's own stable `route_single_net_with_config`-family entry
/// points (also above), both of which now delegate to this module's entry
/// points instead of the two old kernel functions -- which no longer exist;
/// the full benchmark ladder validated byte-identical route costs against
/// them before they were deleted.
///
/// Every step behaves exactly like a plain, crossing-unaware A* search --
/// dense-array `DenseSearchStorage`/`OpenSet` state, no extra bookkeeping --
/// until an accepted primitive's destination is blocked or the current state
/// already carries a non-default `CrossingExtension` (meaning its own path
/// has already crossed something). Only then does the kernel consult a
/// pluggable `CrossingLegalityHook` and, if it accepts, store the resulting
/// state in a small side table (Tier 2) instead of the dense array (Tier 1).
/// `NoCrossingHook` (crossings disabled) never does real work in that hook at
/// all, so crossing-disabled routing is the same code path a plain kernel
/// would have been, at the same cost; `LiveCrossingHook` (crossings enabled)
/// reuses the real crossing-legality functions (`crossing_no_contact_outcome`,
/// `crossing_move_outcome_with_segments`) unchanged.
///
/// Two design points worth knowing when reading this module:
/// - Tier 1 honors `AStarConfig::use_indexed_heap` via the plain `OpenSet`/`IndexedOpenSet`
///   types (which gained a `peek` method for this). Tier 2 (a small minority
///   of states in any real search) uses a plain `BinaryHeap<OpenEntry>` --
///   never slower than what the old crossing kernel used unconditionally for
///   *every* state. The two heaps are merged each iteration by peeking both
///   and popping whichever holds the better candidate.
/// - `unified_candidate_hits_active_local_crossing_reservation` (the local
///   self-overlap guard preventing a route from illegally crossing the same
///   partner twice) walks `UnifiedExtendedNode`'s own parent chain and stops
///   the instant it reaches a Tier-1 (dense) ancestor, since Tier-1 states
///   structurally never carry a reservation -- bounded by how far into an
///   active crossing corridor the current state is, not by the whole route.
/// - `CrossingExtension::straight_run_cells` is tracked for Tier-1 states too
///   (via the `dense_straight_run` side array, gated on the `H::TRACKS_STRAIGHT_RUN`
///   associated const so it costs nothing when crossings are disabled) --
///   not just for Tier-2 ones. A route's straight-run margin going into its
///   *first* crossing depends on geometry from before that crossing ever
///   started, so a state that has never crossed anything still needs an
///   accurate count the instant it tries to. Migration's own benchmark
///   ladder is what caught this: `benes_4x4` started failing with
///   `"insufficient_straight_margin"` crossing rejections until this was
///   added, because the earlier bounded-prototype fixtures never had a long
///   enough straight approach to expose the gap.

/// Identifies a state the unified kernel has reached: either a Tier-1
/// dense-array index, or an index into the Tier-2 `UnifiedExtendedNode`
/// arena. Reused as the `idx` payload of the existing `OpenEntry` for
/// both tiers' open sets -- exactly how today's crossing kernel already
/// reuses `OpenEntry.idx` to mean "index into its own `nodes` arena"
/// rather than a dense grid index.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum UnifiedOpenRef {
    Dense(usize),
    Extended(usize),
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum UnifiedParentRef {
    Dense(usize),
    Extended(usize),
}

pub(crate) struct UnifiedExtendedNode {
    pub(crate) state: State,
    pub(crate) extension: CrossingExtension,
    pub(crate) parent: UnifiedParentRef,
    pub(crate) primitive_id: u16,
    pub(crate) g_score: f64,
    pub(crate) active_local_reservation_keys: Vec<CellKey>,
    pub(crate) pending_local_reservation_keys: Vec<CellKey>,
    /// Cumulative legalized crossings along this node's parent chain.
    /// Maintained only under PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG (0
    /// otherwise); powers the best-crossing-path dump on failure.
    pub(crate) crossings: u16,
    /// True iff this node or any extended ancestor carries a reservation
    /// key. Lets the two parent-chain walks (own-window re-entry, own
    /// window overlap) return at once for the common key-free chain
    /// (perf pass 2026-09-03: the walk was 8.8 % of a mm16 run).
    pub(crate) chain_has_reservations: bool,
}

pub(crate) fn chain_has_reservations_from(
    parent: UnifiedParentRef,
    active: &[CellKey],
    pending: &[CellKey],
    extended_nodes: &[UnifiedExtendedNode],
) -> bool {
    if !active.is_empty() || !pending.is_empty() {
        return true;
    }
    match parent {
        UnifiedParentRef::Dense(_) => false,
        UnifiedParentRef::Extended(idx) => extended_nodes[idx].chain_has_reservations,
    }
}

#[allow(clippy::too_many_arguments)]
/// The A* search itself: build the dense search grid and the per-angle
/// primitive tables, seed the open set, then per iteration pop the best
/// entry across the two open tiers, stop on the goal, generate the
/// primitive moves, check each one's legality, compute its step cost and
/// push or improve the resulting entry. The timeout and iteration-budget
/// checks sit at the top of the iteration; every diagnostic mechanism is
/// behind its own `KernelDiagnostics` flag.
pub(crate) fn run<H: CrossingLegalityHook>(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    routing_bounds: Option<RoutingBounds>,
    stats: &mut RouteSearchStats,
    dynamic_expansion_radius_cells: i32,
    dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
    hook: &H,
) -> Option<RouteResult> {
    let bounds = if let Some(bounds) = routing_bounds {
        if !bounds.contains(source.x, source.y) || !bounds.contains(target.x, target.y) {
            return None;
        }
        bounds
    } else {
        RoutingBounds {
            min_x: 0,
            max_x: obstacle_map.width() - 1,
            min_y: 0,
            max_y: obstacle_map.height() - 1,
        }
    };

    let (mut storage, dense_grid, ignore_dyn_flag) = build_search_grid(
        obstacle_map,
        bounds,
        port_open_cells,
        config,
        dynamic_expansion_radius_cells,
        dynamic_clearance_exempt_cells,
        hook.ignores_dynamic_obstacles(),
        source,
        target,
        stats,
    )?;
    let search_heuristic = SearchHeuristic::new(target, primitives, config);

    let tables = PrimitiveTables::build(primitives, config.bend_weight);
    let target_tolerance = config.target_tolerance_cells.max(0);
    let accepted_target_angles = target_angle_acceptance(target, config);
    // Everything a single move's legality check reads that never
    // changes during the search (see `expansion::move_is_legal`).
    let move_context = MoveContext {
        dense_grid: &dense_grid,
        config,
        hook,
        bounds,
        source,
        target,
        target_tolerance,
        accepted_target_angles: &accepted_target_angles,
    };
    let mut counter = 0u32;

    // Tier 1's own `straight_run_cells` count, needed the instant a
    // Tier-1 state first attempts a crossing (its `CrossingExtension` is
    // otherwise always the default, but a crossing's straight-run margin
    // depends on the geometry leading up to that point, not just from
    // the moment a crossing corridor starts). Compiled away entirely
    // when `H::TRACKS_STRAIGHT_RUN` is `false` (crossings disabled).
    let mut dense_straight_run: Vec<i32> = if H::TRACKS_STRAIGHT_RUN {
        vec![0; storage.state_count()]
    } else {
        Vec::new()
    };

    let mut extended_nodes: Vec<UnifiedExtendedNode> = Vec::new();
    // Merged best-cost + closed bookkeeping for extension-carrying states
    // ((best_g, closed)): in lidar-pure mode the straight-run tracking
    // makes most states extended, and the previous two separate hash
    // structures cost two probes per touch plus rehash growth from empty
    // (measured ~10% of kernel time). Pre-sized to skip the growth
    // ladder; semantics identical.
    let mut extended_state: FxHashMap<(State, CrossingExtension), (f64, bool)> =
        FxHashMap::with_capacity_and_hasher(4096, Default::default());

    // Search-failure diagnosis (owner request 2026-09-01, after a failed
    // search's "why" -- proven-empty reachable space vs iteration cap vs
    // timeout -- was invisible and cost a day of misdirected budget
    // experiments): under PHOTONIC_ROUTER_SEARCH_FAILURE_DIAG=1 every
    // failing search prints its termination kind plus the bounding box of
    // the explored region, which localizes the wall the frontier dies at.
    let failure_diag = config.diagnostics.search_failure_diag;
    let mut explored = ExploredBox::new();
    // Target-ring diagnosis (owner request 2026-09-02): for the 5x5 cells
    // around the target, count what happened to every successor attempt
    // landing there, so a failing search names the check that seals the
    // goal entry instead of leaving it to interpretation.
    // Slots: 0=generated 1=accepted 2=footprint 3=hook 4=closed 5=pruned 6=reservation
    let mut target_ring = [[0u32; 7]; 25];
    let mut goal_miss_angle = 0u32;
    let mut goal_miss_hook = 0u32;
    // Blocker lines are capped PER ring/probe slot (not globally), so
    // landings from one side cannot starve the others of diagnostics.
    let mut ring_blocker_lines: FxHashMap<usize, u32> = FxHashMap::default();
    // Names the branch that abandons an eager completion chain (the
    // silent killer of consecutive crossings), under
    // `KernelDiagnostics::chain_diag`.
    let chain_diag = config.diagnostics.chain_diag;
    let move_diag_cell: Option<(i32, i32)> = config.diagnostics.move_diag_cell;
    // Per-process search sequence number so diagnostic lines from
    // different searches can be told apart in a shared stderr stream.
    static SEARCH_SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let search_seq = SEARCH_SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    CURRENT_SEARCH_SEQ.with(|c| c.set(search_seq));
    // `KernelDiagnostics::pop_diag_below_y = Some(y)` logs every Tier-2
    // push/pop/skip whose state lies below that y (first 200).
    let pop_diag_below_y: Option<i32> = config.diagnostics.pop_diag_below_y;
    let mut pop_diag_lines = 0u32;
    // Best-crossing-path tracking (owner request 2026-09-02): remember
    // the accepted state with the most legalized crossings so a failing
    // search can dump that path -- its endpoint is the first crossing
    // the search could not get past.
    // (Dense/Tier-1 states are crossing-free by construction -- their
    // parents are always dense -- so only extended nodes carry counts.)
    let mut best_crossings: u16 = 0;
    let mut best_crossing_ref: Option<usize> = None;
    // Probe cells (`KernelDiagnostics::probe_cells`) get the same
    // landing counters as the target ring: slot 25.. in
    // `target_ring`-like storage.
    let probe_cells: Vec<(i32, i32)> = config.diagnostics.probe_cells.clone();
    let probe_index: FxHashMap<(i32, i32), usize> = probe_cells
        .iter()
        .enumerate()
        .map(|(i, cell)| (*cell, 25 + i))
        .collect();
    let mut probe_ring: Vec<[u32; 7]> = vec![[0u32; 7]; probe_cells.len()];
    // Everything the failure and probe reports read that never changes
    // during the search (see `diagnostics::print_search_failure_report`).
    let failure_env = SearchFailureEnv {
        search_seq,
        obstacle_map,
        port_open_cells,
        dense_grid: &dense_grid,
        source,
        target,
        bounds,
        probe_cells: &probe_cells,
        failure_map: config.diagnostics.search_failure_map.as_deref(),
    };
    let mut tier1_open = OpenSet::new(config.use_indexed_heap, storage.state_count());
    let mut tier2_open: BinaryHeap<OpenEntry> = BinaryHeap::new();
    let source_idx = seed_open_set(
        &mut storage,
        &mut tier1_open,
        source,
        &search_heuristic,
        hook,
        config,
        stats,
        &mut counter,
    )?;

    let mut iterations = 0usize;
    let search_loop_start = if config.collect_detailed_timing {
        Some(Instant::now())
    } else {
        None
    };
    let search_timeout_start = (config.max_search_time_ms > 0).then(Instant::now);

    loop {
        let take_tier1 = match (tier1_open.peek(), tier2_open.peek()) {
            (Some(t1), Some(t2)) => entry_is_better(t1, t2) || t1 == t2,
            (Some(_), None) => true,
            (None, Some(_)) => false,
            (None, None) => break,
        };
        let (entry, current_ref) = if take_tier1 {
            let entry = tier1_open.pop().expect("peeked Some above");
            let idx = entry.idx;
            (entry, UnifiedOpenRef::Dense(idx))
        } else {
            let entry = tier2_open.pop().expect("peeked Some above");
            let idx = entry.idx;
            (entry, UnifiedOpenRef::Extended(idx))
        };
        stats.heap_pops += 1;
        iterations += 1;
        if iterations > config.max_iterations {
            if let Some(search_loop_start) = search_loop_start.as_ref() {
                stats.search_loop_time_us += search_loop_start.elapsed().as_micros();
            }
            if failure_diag {
                print_search_failure_report(
                    &failure_env,
                    "iteration_cap",
                    &SearchFailureState {
                        iterations,
                        explored,
                        target_ring: &target_ring,
                        probe_ring: &probe_ring,
                        goal_miss_angle,
                        goal_miss_hook,
                        best_crossings,
                        best_crossing_ref,
                        storage: &storage,
                        extended_nodes: &extended_nodes,
                    },
                    stats,
                );
            }
            return None;
        }
        if should_check_timeout(iterations, config)
            && search_timed_out(search_timeout_start.as_ref(), config)
        {
            if let Some(search_loop_start) = search_loop_start.as_ref() {
                stats.search_loop_time_us += search_loop_start.elapsed().as_micros();
            }
            trace_search_timeout(
                "unified_astar",
                config,
                stats,
                iterations,
                tier1_open.len() + tier2_open.len(),
            );
            if failure_diag {
                print_search_failure_report(
                    &failure_env,
                    "timeout",
                    &SearchFailureState {
                        iterations,
                        explored,
                        target_ring: &target_ring,
                        probe_ring: &probe_ring,
                        goal_miss_angle,
                        goal_miss_hook,
                        best_crossings,
                        best_crossing_ref,
                        storage: &storage,
                        extended_nodes: &extended_nodes,
                    },
                    stats,
                );
            }
            return None;
        }

        let (state, current_extension, current_g) = match current_ref {
            UnifiedOpenRef::Dense(idx) => {
                if storage.best_generation[idx] != entry.generation {
                    stats.skipped_duplicate_heap_entries += 1;
                    stats.stale_generation_heap_entries += 1;
                    continue;
                }
                if storage.closed.get(idx) {
                    stats.skipped_duplicate_heap_entries += 1;
                    stats.closed_heap_entries += 1;
                    continue;
                }
                let straight_run_cells = if H::TRACKS_STRAIGHT_RUN {
                    dense_straight_run[idx]
                } else {
                    0
                };
                (
                    storage.idx_to_state(idx),
                    CrossingExtension {
                        straight_run_cells,
                        ..CrossingExtension::default()
                    },
                    storage.g_costs[idx],
                )
            }
            UnifiedOpenRef::Extended(ext_idx) => {
                let node = &extended_nodes[ext_idx];
                let key = (node.state, node.extension);
                let bookkeeping = extended_state.get(&key).copied();
                if let Some(th) = pop_diag_below_y {
                    if node.state.y < th && pop_diag_lines < 200 {
                        pop_diag_lines += 1;
                        eprintln!("pop-diag seq={} ignore_dyn={} tier2 pop state=({},{},{}) g={:.1} bookkeeping={:?} pending={}", search_seq, ignore_dyn_flag, node.state.x, node.state.y, node.state.angle, entry.g_score, bookkeeping, node.extension.pending_after_crossing_cells);
                    }
                }
                if entry.g_score > bookkeeping.map_or(f64::INFINITY, |(g, _)| g) + 1.0e-9 {
                    stats.skipped_duplicate_heap_entries += 1;
                    stats.stale_generation_heap_entries += 1;
                    continue;
                }
                if bookkeeping.is_some_and(|(_, closed)| closed) {
                    stats.skipped_duplicate_heap_entries += 1;
                    stats.closed_heap_entries += 1;
                    continue;
                }
                (node.state, node.extension, node.g_score)
            }
        };

        let in_tolerance = within_target_tolerance(&move_context, state);
        let at_goal = goal_reached(&move_context, state, current_extension);
        if failure_diag && in_tolerance && !at_goal {
            if !accepted_target_angles[state.angle as usize] {
                goal_miss_angle += 1;
            } else {
                goal_miss_hook += 1;
            }
        }
        if at_goal {
            if let Some(search_loop_start) = search_loop_start.as_ref() {
                stats.search_loop_time_us += search_loop_start.elapsed().as_micros();
            }
            // Harness: probe cells report on SUCCESS too (where did the
            // search go / not go), not only on failure.
            print_probe_success_report(&failure_env, state, &probe_ring);
            if config.collect_detailed_timing {
                let reconstruction_start = Instant::now();
                let mut route = reconstruct_route_unified(
                    source_idx,
                    current_ref,
                    current_g,
                    target,
                    primitives,
                    stats.clone(),
                    &storage,
                    &extended_nodes,
                )?;
                route.stats.reconstruction_time_us += reconstruction_start.elapsed().as_micros();
                return Some(route);
            }
            return reconstruct_route_unified(
                source_idx,
                current_ref,
                current_g,
                target,
                primitives,
                stats.clone(),
                &storage,
                &extended_nodes,
            );
        }

        match current_ref {
            UnifiedOpenRef::Dense(idx) => storage.closed.set(idx)?,
            UnifiedOpenRef::Extended(_) => {
                extended_state
                    .entry((state, current_extension))
                    .and_modify(|(_, closed)| *closed = true)
                    .or_insert((current_g, true));
            }
        }
        stats.expanded_states += 1;
        if failure_diag {
            explored.note(state);
        }

        let angle = state.angle as usize;
        let primitive_bucket = tables.buckets[angle];
        let primitive_metadata = &tables.metadata[angle];
        let footprint_profiles = &tables.profiles[angle];
        let crossing_metadata = &tables.crossings[angle];
        let neighbor_loop_start =
            if config.collect_detailed_timing && config.diagnostics.hot_loop_timing {
                Some(Instant::now())
            } else {
                None
            };
        let mut neighbor_loop_heap_time_us = 0u128;
        let mut neighbor_loop_legality_time_us = 0u128;

        for candidate in candidate_moves(
            primitive_bucket,
            primitive_metadata,
            footprint_profiles,
            crossing_metadata,
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        ) {
            let primitive_class = candidate.class;
            stats.generated_neighbors += 1;
            stats.primitive_generated_by_class[primitive_class] += 1;

            let mut move_diagnostics = MoveDiagnostics {
                failure_diag,
                move_diag_cell,
                search_seq,
                obstacle_map,
                port_open_cells,
                target,
                probe_index: &probe_index,
                target_ring: &mut target_ring,
                probe_ring: &mut probe_ring,
                ring_blocker_lines: &mut ring_blocker_lines,
            };
            // Every reject rule this move must pass, in the kernel's own
            // order, plus the tier it lands in. A `None` here is the old
            // `checked_add` `?`: a coordinate overflow abandons the search.
            let (legal, hook_outcome) = match move_is_legal(
                &move_context,
                state,
                current_extension,
                &candidate,
                &mut move_diagnostics,
                stats,
                &mut neighbor_loop_legality_time_us,
            )? {
                MoveVerdict::Illegal(_) => continue,
                MoveVerdict::Tier1(legal) => (legal, None),
                MoveVerdict::Tier2(legal, outcome, extra_cost) => {
                    (legal, Some((outcome, extra_cost)))
                }
            };
            let Some((outcome, extra_cost)) = hook_outcome else {
                // Tier 1: identical fast path to today's plain kernel --
                // dense array storage, no crossing bookkeeping, no hook call.
                let UnifiedOpenRef::Dense(current_dense_idx) = current_ref else {
                    unreachable!(
                        "tier-1 fast path only runs from a default-extension \
                             state, which is always stored densely"
                    );
                };
                push_or_improve_dense(
                    &move_context,
                    &search_heuristic,
                    &mut storage,
                    &mut tier1_open,
                    &mut dense_straight_run,
                    &mut counter,
                    state,
                    current_extension,
                    current_g,
                    current_dense_idx,
                    &candidate,
                    &legal,
                    &mut move_diagnostics,
                    stats,
                    &mut neighbor_loop_heap_time_us,
                )?;
                continue;
            };
            let next_extension = CrossingExtension::from_outcome(&outcome);

            if next_extension.pending_after_crossing_cells > 0 {
                let Some(chain) = complete_crossing_corridor(
                    &move_context,
                    &tables,
                    &mut extended_nodes,
                    state,
                    current_g,
                    current_ref,
                    &candidate,
                    &legal,
                    next_extension,
                    &outcome,
                    extra_cost,
                    chain_diag,
                    &mut move_diagnostics,
                    stats,
                ) else {
                    continue;
                };
                let CompletedCorridor {
                    last_idx,
                    state: chain_state,
                    extension: chain_extension,
                    g_score: chain_g,
                } = chain;
                let chain_ring = move_diagnostics.ring_index(chain_state.x, chain_state.y);
                let final_key = (chain_state, chain_extension);
                let final_bookkeeping = extended_state.get(&final_key).copied();
                if final_bookkeeping.is_some_and(|(_, closed)| closed) {
                    if chain_diag {
                        eprintln!(
                            "chain-final seq={} closed state=({},{},{}) g={:.1}",
                            search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_g
                        );
                    }
                    stats.primitive_closed_rejects_by_class[primitive_class] += 1;
                    if let Some(i) = chain_ring {
                        move_diagnostics.count_landing(i, 4);
                    }
                    continue;
                }
                if chain_g >= final_bookkeeping.map(|(g, _)| g).unwrap_or(f64::INFINITY) {
                    if chain_diag {
                        eprintln!("chain-final seq={} cost_pruned state=({},{},{}) g={:.1} existing={:.1}", search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_g, final_bookkeeping.map(|(g, _)| g).unwrap_or(f64::INFINITY));
                    }
                    stats.primitive_cost_pruned_by_class[primitive_class] += 1;
                    if let Some(i) = chain_ring {
                        move_diagnostics.count_landing(i, 5);
                    }
                    continue;
                }
                extended_state.insert(final_key, (chain_g, false));
                if let Some(th) = pop_diag_below_y {
                    if chain_state.y < th && pop_diag_lines < 200 {
                        pop_diag_lines += 1;
                        eprintln!(
                            "pop-diag seq={} chain-final PUSH state=({},{},{}) g={:.1}",
                            search_seq, chain_state.x, chain_state.y, chain_state.angle, chain_g
                        );
                    }
                }
                stats.primitive_accepted_by_class[primitive_class] += 1;
                if let Some(i) = chain_ring {
                    move_diagnostics.count_landing(i, 1);
                }
                if failure_diag {
                    let count = extended_nodes[last_idx].crossings;
                    if count > best_crossings || best_crossing_ref.is_none() {
                        best_crossings = count;
                        best_crossing_ref = Some(last_idx);
                    }
                }
                stats.best_cost_updates += 1;
                stats.parent_updates += 1;
                let generation = next_search_generation(&mut counter)?;
                tier2_open.push(OpenEntry {
                    f_score: chain_g
                        + cost::heuristic_estimate(&search_heuristic, chain_state)
                        + hook.heuristic_bonus(chain_state, chain_extension),
                    tie_score: heap_tie_score(chain_g, config.heap_tie_breaker),
                    g_score: chain_g,
                    counter: generation,
                    generation,
                    idx: last_idx,
                });
                stats.heap_pushes += 1;
                stats.max_heap_size = stats.max_heap_size.max(tier1_open.len() + tier2_open.len());
                continue;
            }

            push_or_improve_extended(
                &move_context,
                &search_heuristic,
                &mut extended_nodes,
                &mut extended_state,
                &mut tier2_open,
                tier1_open.len(),
                &mut counter,
                state,
                current_extension,
                current_g,
                current_ref,
                &candidate,
                &legal,
                next_extension,
                &outcome,
                extra_cost,
                &mut best_crossings,
                &mut best_crossing_ref,
                &mut move_diagnostics,
                stats,
                &mut neighbor_loop_heap_time_us,
            )?;
        }
        if let Some(neighbor_loop_start) = neighbor_loop_start {
            let neighbor_loop_elapsed_us = neighbor_loop_start.elapsed().as_micros();
            stats.neighbor_generation_time_us += neighbor_loop_elapsed_us
                .saturating_sub(neighbor_loop_heap_time_us)
                .saturating_sub(neighbor_loop_legality_time_us);
        }
    }

    if let Some(search_loop_start) = search_loop_start.as_ref() {
        stats.search_loop_time_us += search_loop_start.elapsed().as_micros();
    }
    if failure_diag {
        print_search_failure_report(
            &failure_env,
            "open_set_exhausted",
            &SearchFailureState {
                iterations,
                explored,
                target_ring: &target_ring,
                probe_ring: &probe_ring,
                goal_miss_angle,
                goal_miss_hook,
                best_crossings,
                best_crossing_ref,
                storage: &storage,
                extended_nodes: &extended_nodes,
            },
            stats,
        );
    }
    None
}

pub(crate) fn route_result_from_step_chain(
    mut states_reversed: Vec<State>,
    mut primitive_steps_reversed: Vec<(State, u16)>,
    primitives: &PrimitiveLibrary,
    total_cost: f64,
    requested_target: State,
    stats: RouteSearchStats,
) -> Option<RouteResult> {
    states_reversed.reverse();
    primitive_steps_reversed.reverse();

    let source = *states_reversed.first()?;
    let reached_target = *states_reversed.last()?;
    let mut primitive_ids = Vec::with_capacity(primitive_steps_reversed.len());
    let mut cells = Vec::new();
    let mut seen_cells = FxHashSet::default();
    let mut ordered_path = Vec::new();
    push_if_different(&mut ordered_path, (source.x, source.y));
    let mut total_length_um = 0.0;

    for (origin, primitive_id) in primitive_steps_reversed {
        let primitive = find_primitive(primitives, origin.angle, primitive_id)?;
        primitive_ids.push(primitive_id);
        total_length_um += primitive.length_um;
        for (dx, dy) in primitive.footprint.iter().copied() {
            let cell = (origin.x + dx, origin.y + dy);
            push_if_different(&mut ordered_path, cell);
            if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                cells.push(cell);
            }
        }
    }
    push_if_different(&mut ordered_path, (reached_target.x, reached_target.y));
    let compressed_waypoints = compress_grid_waypoints(&ordered_path);
    if polyline_self_intersects(&compressed_waypoints) {
        return None;
    }

    Some(RouteResult {
        states: states_reversed,
        primitives: primitive_ids,
        cells,
        compressed_waypoints,
        total_length_um,
        total_cost,
        requested_target,
        reached_target,
        stats,
    })
}

pub(crate) fn reconstruct_route_unified(
    source_idx: usize,
    reached: UnifiedOpenRef,
    reached_g: f64,
    requested_target: State,
    primitives: &PrimitiveLibrary,
    stats: RouteSearchStats,
    storage: &DenseSearchStorage,
    extended_nodes: &[UnifiedExtendedNode],
) -> Option<RouteResult> {
    let reached_target = match reached {
        UnifiedOpenRef::Dense(idx) => storage.idx_to_state(idx),
        UnifiedOpenRef::Extended(ext_idx) => extended_nodes.get(ext_idx)?.state,
    };
    let mut states_reversed = vec![reached_target];
    let mut primitive_steps_reversed = Vec::new();
    let mut cursor = reached;

    loop {
        match cursor {
            UnifiedOpenRef::Dense(idx) => {
                if idx == source_idx {
                    break;
                }
                let parent_idx_u32 = *storage.parent_idx.get(idx)?;
                if parent_idx_u32 == NO_PARENT {
                    return None;
                }
                let parent_idx = usize::try_from(parent_idx_u32).ok()?;
                let previous = storage.idx_to_state(parent_idx);
                let primitive_id = *storage.parent_primitive.get(idx)?;
                primitive_steps_reversed.push((previous, primitive_id));
                states_reversed.push(previous);
                cursor = UnifiedOpenRef::Dense(parent_idx);
            }
            UnifiedOpenRef::Extended(ext_idx) => {
                let node = extended_nodes.get(ext_idx)?;
                match node.parent {
                    UnifiedParentRef::Dense(parent_idx) => {
                        let previous = storage.idx_to_state(parent_idx);
                        primitive_steps_reversed.push((previous, node.primitive_id));
                        states_reversed.push(previous);
                        cursor = UnifiedOpenRef::Dense(parent_idx);
                    }
                    UnifiedParentRef::Extended(parent_ext_idx) => {
                        let previous = extended_nodes.get(parent_ext_idx)?.state;
                        primitive_steps_reversed.push((previous, node.primitive_id));
                        states_reversed.push(previous);
                        cursor = UnifiedOpenRef::Extended(parent_ext_idx);
                    }
                }
            }
        }
    }

    route_result_from_step_chain(
        states_reversed,
        primitive_steps_reversed,
        primitives,
        reached_g,
        requested_target,
        stats,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::primitives::PrimitiveGeometry;
    use crate::search::astar::config::PrimitiveOrdering;
    use crate::search::astar::crossing_rules::CrossingSearchConfig;
    use crate::search::astar::crossing_rules::CrossingSearchPartner;
    use crate::search::astar::route_single_net_with_config;
    use crate::search::test_support::*;

    #[test]
    fn terminal_straight_requirement_rejects_immediate_port_bends() {
        let map = ObstacleMap::new(8, 8);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(3, 3, 2),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                require_terminal_straights: true,
                ..AStarConfig::default()
            },
        )
        .expect("route should exist with straight launch and landing");

        let first_primitive = library
            .get_primitives_for_angle(result.states[0].angle)
            .iter()
            .find(|p| p.id == result.primitives[0])
            .expect("first primitive should exist");
        let last_start_angle = result.states[result.states.len() - 2].angle;
        let last_primitive = library
            .get_primitives_for_angle(last_start_angle)
            .iter()
            .find(|p| p.id == *result.primitives.last().unwrap())
            .expect("last primitive should exist");

        assert!(matches!(
            first_primitive.geometry,
            PrimitiveGeometry::Straight { .. }
        ));
        assert!(matches!(
            last_primitive.geometry,
            PrimitiveGeometry::Straight { .. }
        ));
    }

    #[test]
    fn terminal_straight_requirement_rejects_immediate_port_bends_in_crossing_kernel() {
        let map = ObstacleMap::new(16, 16);
        let library = primitive_library();
        let crossing = CrossingSearchConfig {
            diagnostics: KernelDiagnostics::default(),
            net_id: 1,
            partners: vec![CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(12, 12), (14, 12)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            }],
            min_straight_cells: 2,
            crossing_half_size_cells: 0,
            bend_runout_cells: 0,
            crossing_loss: 0.0,
            require_all_partners: false,
            terminal_bump_guard: None,
        };
        let result = test_route_single_net_with_crossing_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(3, 3, 2),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                require_terminal_straights: true,
                ..AStarConfig::default()
            },
            0,
            None,
            &crossing,
        )
        .expect("route should exist with straight launch and landing");

        let first_primitive = library
            .get_primitives_for_angle(result.states[0].angle)
            .iter()
            .find(|p| p.id == result.primitives[0])
            .expect("first primitive should exist");
        let last_start_angle = result.states[result.states.len() - 2].angle;
        let last_primitive = library
            .get_primitives_for_angle(last_start_angle)
            .iter()
            .find(|p| p.id == *result.primitives.last().unwrap())
            .expect("last primitive should exist");

        assert!(matches!(
            first_primitive.geometry,
            PrimitiveGeometry::Straight { .. }
        ));
        assert!(matches!(
            last_primitive.geometry,
            PrimitiveGeometry::Straight { .. }
        ));
    }

    #[test]
    fn reconstruction_uses_primitive_ids_and_preserves_cells() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("route should exist");

        assert!(!result.primitives.is_empty());
        for primitive_id in &result.primitives {
            assert!(*primitive_id > 0);
        }

        let mut expected_cells = Vec::new();
        let mut seen_cells = FxHashSet::default();
        for (idx, primitive_id) in result.primitives.iter().enumerate() {
            let origin = result.states[idx];
            let primitive = library
                .get_primitives_for_angle(origin.angle)
                .iter()
                .find(|p| p.id == *primitive_id)
                .expect("primitive id should resolve in library");
            for (dx, dy) in primitive.footprint.iter().copied() {
                let cell = (origin.x + dx, origin.y + dy);
                if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                    expected_cells.push(cell);
                }
            }
        }

        assert_eq!(result.cells, expected_cells);
    }

    #[test]
    fn indexed_heap_matches_duplicate_heap_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };

        let duplicate_route =
            route_single_net_with_config(&map, &library, source, target, None, &base_config)
                .expect("duplicate-entry heap route should exist");
        let indexed_route = route_single_net_with_config(
            &map,
            &library,
            source,
            target,
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                use_indexed_heap: true,
                ..base_config
            },
        )
        .expect("indexed heap route should exist");

        assert_eq!(indexed_route.reached_target, duplicate_route.reached_target);
        assert!((indexed_route.total_cost - duplicate_route.total_cost).abs() < 1.0e-9);
        assert_eq!(indexed_route.stats.skipped_duplicate_heap_entries, 0);
        assert_eq!(indexed_route.stats.stale_generation_heap_entries, 0);
        assert!(duplicate_route.stats.stale_generation_heap_entries > 0);
        assert!(indexed_route.stats.max_heap_size <= duplicate_route.stats.max_heap_size);
    }

    #[test]
    fn primitive_ordering_modes_preserve_route_cost_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        let baseline =
            route_single_net_with_config(&map, &library, source, target, None, &base_config)
                .expect("baseline route should exist");

        for primitive_ordering in [
            PrimitiveOrdering::LongStraightFirst,
            PrimitiveOrdering::TargetBiased,
        ] {
            let result = route_single_net_with_config(
                &map,
                &library,
                source,
                target,
                None,
                &AStarConfig {
                    diagnostics: KernelDiagnostics::default(),
                    primitive_ordering,
                    ..base_config.clone()
                },
            )
            .expect("ordered route should exist");
            assert_eq!(result.reached_target, baseline.reached_target);
            assert!((result.total_cost - baseline.total_cost).abs() < 1.0e-9);
        }
    }

    #[test]
    fn heap_tie_breaker_modes_preserve_route_cost_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        let baseline =
            route_single_net_with_config(&map, &library, source, target, None, &base_config)
                .expect("baseline route should exist");

        let larger_g = route_single_net_with_config(
            &map,
            &library,
            source,
            target,
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                heap_tie_breaker: HeapTieBreaker::LargerG,
                ..base_config
            },
        )
        .expect("larger-g route should exist");

        assert_eq!(larger_g.reached_target, baseline.reached_target);
        assert!((larger_g.total_cost - baseline.total_cost).abs() < 1.0e-9);
    }

    #[test]
    fn detailed_timing_is_opt_in() {
        let map = ObstacleMap::new(12, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(8, 2, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                use_routing_window: false,
                enable_simple_routes: false,
                collect_detailed_timing: true,
                ..AStarConfig::default()
            },
        )
        .expect("route should exist with direct A* run");

        assert!(result.stats.heap_pushes > 0);
        assert!(result.stats.heap_pops > 0);
        assert!(
            result.stats.neighbor_generation_time_us
                + result.stats.heap_operation_time_us
                + result.stats.legality_check_time_us
                + result.stats.reconstruction_time_us
                > 0
        );
    }
}
