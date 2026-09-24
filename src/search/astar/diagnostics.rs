//! Diagnostic tracing and ASCII-dump helpers gated by `KernelDiagnostics`:
//! search-timeout tracing, per-net crossing-candidate/level1 tracing, and
//! the failure-map/target-ring/probe-cells/best-crossing-path reports.
//! Moved out of `src/astar.rs` (Milestone 3, Slice 2); pure code motion, no
//! behaviour change.

use crate::obstacle_map::{pack_xy, CellKey, NetId, ObstacleMap};
use crate::primitives::Primitive;
use crate::search::astar::config::{
    AStarConfig, CROSSING_CANDIDATE_TRACE_COUNT, NO_PARENT, SEARCH_TIMEOUT_CHECK_INTERVAL,
};
use crate::search::astar::crossing_rules::CrossingSearchConfig;
use crate::search::astar::dense::{DenseRoutingGrid, DenseSearchStorage};
use crate::search::astar::kernel::{UnifiedExtendedNode, UnifiedParentRef};
use crate::search::astar::window::RoutingBounds;
use crate::search::state::{RouteSearchStats, State};
use rustc_hash::FxHashSet;
use std::io::Write;
use std::sync::atomic::Ordering as AtomicOrdering;
use std::time::Instant;

pub(crate) fn search_timed_out(search_start: Option<&Instant>, config: &AStarConfig) -> bool {
    let Some(search_start) = search_start else {
        return false;
    };
    config.max_search_time_ms > 0
        && search_start.elapsed().as_millis() >= u128::from(config.max_search_time_ms)
}

pub(crate) fn should_check_timeout(iterations: usize, config: &AStarConfig) -> bool {
    config.max_search_time_ms > 0 && iterations % SEARCH_TIMEOUT_CHECK_INTERVAL == 0
}

pub(crate) fn trace_search_timeout(
    label: &str,
    config: &AStarConfig,
    stats: &RouteSearchStats,
    iterations: usize,
    open_len: usize,
) {
    eprintln!(
        "astar_timeout label={} max_search_time_ms={} iterations={} open_len={} expanded={} generated={} heap_pops={} heap_pushes={} footprint_checks={} rect_checks={} crossing_checks={} crossing_accepted={} crossing_reject_margin={} crossing_reject_pending_straight={}",
        label,
        config.max_search_time_ms,
        iterations,
        open_len,
        stats.expanded_states,
        stats.generated_neighbors,
        stats.heap_pops,
        stats.heap_pushes,
        stats.primitive_footprint_checks,
        stats.primitive_footprint_rect_checks,
        stats.crossing_candidate_checks,
        stats.crossing_accepted,
        stats.crossing_reject_margin,
        stats.crossing_reject_pending_straight,
    );
}

pub(crate) fn trace_crossing_pending_enabled(crossing: &CrossingSearchConfig) -> bool {
    if !crossing.diagnostics.trace_crossing_pending {
        return false;
    }
    if let Some(trace_net_id) = crossing.diagnostics.trace_crossing_net {
        return trace_net_id == crossing.net_id;
    }
    true
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn trace_crossing_pending(
    crossing: &CrossingSearchConfig,
    event: &str,
    state: State,
    primitive: &Primitive,
    pending_cells: i32,
    initial_run_distance: f64,
    intersection_angle: Option<u8>,
    intersection_after: Option<f64>,
) {
    if !trace_crossing_pending_enabled(crossing) {
        return;
    }
    eprintln!(
        "crossing-pending seq={} net={} event={} state=({}, {}, {}) primitive_id={} primitive_start={} primitive_end={} pending_cells={} initial_run={:.3} intersection_angle={:?} intersection_after={:?}",
        current_search_seq(),
        crossing.net_id,
        event,
        state.x,
        state.y,
        state.angle,
        primitive.id,
        primitive.start_angle,
        primitive.end_angle,
        pending_cells,
        initial_run_distance,
        intersection_angle,
        intersection_after,
    );
}

/// Tier 2's local self-overlap guard: today's crossing kernel's
/// `crossing_candidate_hits_active_local_crossing_reservation`, ported to
/// walk `UnifiedExtendedNode`'s parent chain instead of a flat `nodes`
/// arena. Tier-1 (dense) states never carry a reservation, so the walk
/// stops as soon as it reaches one -- bounded by how far into an active
/// crossing corridor the current state is, not by the whole route.
/// Companion to the search-failure diagnosis: prints, for the 5x5 cells
/// around the target, occupancy plus what happened to every successor
/// attempt landing there (see the slot legend at the counter array).
pub(crate) fn print_target_ring_report(
    out: &mut dyn Write,
    target: State,
    target_ring: &[[u32; 7]; 25],
    goal_miss_angle: u32,
    goal_miss_hook: u32,
    obstacle_map: &ObstacleMap,
    port_open_cells: Option<&FxHashSet<CellKey>>,
) {
    let _ = writeln!(
        out,
        "search-failure-goalmiss angle={} hook={}",
        goal_miss_angle, goal_miss_hook
    );
    for dy in -2i32..=2 {
        for dx in -2i32..=2 {
            let i = ((dy + 2) * 5 + (dx + 2)) as usize;
            let c = &target_ring[i];
            let x = target.x + dx;
            let y = target.y + dy;
            let static_blocked = obstacle_map.is_static_blocked(x, y);
            let opened = port_open_cells.is_some_and(|open| open.contains(&pack_xy(x, y)));
            if c.iter().any(|&v| v > 0) || static_blocked || opened || (dx == 0 && dy == 0) {
                let _ = writeln!(
                    out,
                        "search-failure-ring cell=({},{}) off=({},{}) static={} opened={} gen={} acc={} foot={} hook={} closed={} pruned={} resv={}",
                        x, y, dx, dy, static_blocked, opened,
                        c[0], c[1], c[2], c[3], c[4], c[5], c[6]
                    );
            }
        }
    }
}

/// Companion to the search-failure diagnosis (owner request 2026-09-02):
/// dump the accepted path that legalized the most crossings. Its
/// endpoint is the first crossing the search never got past.

/// Permanent, env-gated: `PHOTONIC_ROUTER_PROBE_CELLS="x,y;x,y;..."` prints,
/// on every search failure, whether each listed cell is static, opened for
/// this search, and which dynamic owners occupy it -- the direct answer to
/// "what is the wall made of" at any coordinate, not just the target ring.
pub(crate) fn print_probe_cells_report(
    out: &mut dyn Write,
    search_seq: u64,
    obstacle_map: &ObstacleMap,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    dense_grid: &DenseRoutingGrid,
    probe_cells: &[(i32, i32)],
) {
    for &(x, y) in probe_cells {
        if !obstacle_map.in_bounds(x, y) {
            let _ = writeln!(
                out,
                "probe-cell seq={} cell=({},{}) out_of_bounds",
                search_seq, x, y
            );
            continue;
        }
        let owners: Vec<NetId> = obstacle_map.dynamic_owners_at(x, y).into_iter().collect();
        let _ = writeln!(
            out,
                "probe-cell seq={} cell=({},{}) static={} opened={} dynamic_owners={:?} core={} dense_blocked={}",
                search_seq,
                x,
                y,
                obstacle_map.is_static_blocked(x, y),
                port_open_cells.is_some_and(|open| open.contains(&pack_xy(x, y))),
                owners,
                obstacle_map.is_dynamic_core_blocked(x, y),
                dense_grid.is_blocked(x, y),
            );
    }
}

/// An ASCII picture of the obstacle map around a failed search, gated
/// by `KernelDiagnostics::search_failure_map` (`None` = disabled).
/// `Some(nums)` with `nums.len() < 4` = the source/target bounding box
/// plus a margin, downsampled so a row fits about 150 characters;
/// `[cx, cy, half_w, half_h, step?]` = an explicit window in cells and
/// block size. Per block: `D` any dynamic (routed) cell, `#` static
/// only, `.` free, `S`/`T` source/target block. Rows top (max y) to
/// bottom.
pub(crate) fn print_failure_map_window(
    out: &mut dyn Write,
    search_seq: u64,
    obstacle_map: &ObstacleMap,
    source: State,
    target: State,
    window_spec: Option<&[i32]>,
) {
    let Some(nums) = window_spec else {
        return;
    };
    let (min_x, max_x, min_y, max_y, step) = if nums.len() >= 4 {
        let step = nums.get(4).copied().unwrap_or(1).max(1);
        (
            nums[0] - nums[2],
            nums[0] + nums[2],
            nums[1] - nums[3],
            nums[1] + nums[3],
            step,
        )
    } else {
        let margin = 40;
        let min_x = source.x.min(target.x) - margin;
        let max_x = source.x.max(target.x) + margin;
        let min_y = source.y.min(target.y) - margin;
        let max_y = source.y.max(target.y) + margin;
        let step = (((max_x - min_x) as f64) / 150.0).ceil().max(1.0) as i32;
        (min_x, max_x, min_y, max_y, step)
    };
    let _ = writeln!(
        out,
            "search-failure-map seq={} window=[{}..{}]x[{}..{}] step={} legend=D:dynamic #:static .:free S:source T:target",
            search_seq, min_x, max_x, min_y, max_y, step
        );
    let mut y = max_y;
    while y >= min_y {
        let mut row = String::with_capacity(((max_x - min_x) / step + 2) as usize);
        let mut x = min_x;
        while x <= max_x {
            let mut ch = '.';
            let mut any_static = false;
            let mut any_dynamic = false;
            let mut has_source = false;
            let mut has_target = false;
            for dx in 0..step {
                for dy in 0..step {
                    let cx = x + dx;
                    let cy = y - dy;
                    if cx == source.x && cy == source.y {
                        has_source = true;
                    }
                    if cx == target.x && cy == target.y {
                        has_target = true;
                    }
                    if !obstacle_map.in_bounds(cx, cy) {
                        continue;
                    }
                    if obstacle_map.is_dynamic_blocked(cx, cy) {
                        any_dynamic = true;
                    } else if obstacle_map.is_static_blocked(cx, cy) {
                        any_static = true;
                    }
                }
            }
            if any_dynamic {
                ch = 'D';
            } else if any_static {
                ch = '#';
            }
            if has_target {
                ch = 'T';
            } else if has_source {
                ch = 'S';
            }
            row.push(ch);
            x += step;
        }
        let _ = writeln!(out, "map y={:>6} {}", y, row);
        y -= step;
    }
}

pub(crate) fn print_best_crossing_path(
    out: &mut dyn Write,
    best_crossings: u16,
    best_ref: Option<usize>,
    storage: &DenseSearchStorage,
    extended_nodes: &[UnifiedExtendedNode],
) {
    let Some(ext_idx) = best_ref else {
        let _ = writeln!(
            out,
            "search-failure-bestpath crossings=0 (no crossing state ever accepted)"
        );
        return;
    };
    let endpoint = extended_nodes[ext_idx].state;
    let mut states: Vec<State> = vec![endpoint];
    let mut cursor = extended_nodes[ext_idx].parent;
    loop {
        match cursor {
            UnifiedParentRef::Extended(i) => {
                states.push(extended_nodes[i].state);
                cursor = extended_nodes[i].parent;
            }
            UnifiedParentRef::Dense(idx) => {
                states.push(storage.idx_to_state(idx));
                let parent = storage.parent_idx.get(idx).copied().unwrap_or(NO_PARENT);
                if parent == NO_PARENT {
                    break;
                }
                cursor = UnifiedParentRef::Dense(parent as usize);
            }
        }
        if states.len() > 200_000 {
            break;
        }
    }
    states.reverse();
    let mut waypoints: Vec<(i32, i32)> = Vec::new();
    for state in &states {
        let point = (state.x, state.y);
        if waypoints.len() >= 2 {
            let a = waypoints[waypoints.len() - 2];
            let b = waypoints[waypoints.len() - 1];
            let d1 = ((b.0 - a.0).signum(), (b.1 - a.1).signum());
            let d2 = ((point.0 - b.0).signum(), (point.1 - b.1).signum());
            if d1 == d2 {
                *waypoints.last_mut().unwrap() = point;
                continue;
            }
        }
        if waypoints.last() != Some(&point) {
            waypoints.push(point);
        }
    }
    let _ = writeln!(
        out,
        "search-failure-bestpath crossings={} endpoint=({},{},{}) waypoints={:?}",
        best_crossings, endpoint.x, endpoint.y, endpoint.angle, waypoints
    );
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn trace_crossing_candidate(
    crossing: &CrossingSearchConfig,
    partner_net_id: u64,
    reason: &str,
    x: f64,
    y: f64,
    route_angle: u8,
    partner_angle: u8,
    required_margin: i32,
    partner_margin: f64,
    route_before: f64,
    route_after: f64,
) {
    if !crossing.diagnostics.trace_crossing_candidates {
        return;
    }
    if let Some(trace_net_id) = crossing.diagnostics.trace_crossing_net {
        if trace_net_id != crossing.net_id {
            return;
        }
    }
    if let Some(trace_partner_id) = crossing.diagnostics.trace_partner_net {
        if trace_partner_id != partner_net_id {
            return;
        }
    }
    let max_count = crossing.diagnostics.trace_crossing_candidate_max;
    let trace_index = CROSSING_CANDIDATE_TRACE_COUNT.fetch_add(1, AtomicOrdering::Relaxed);
    if trace_index >= max_count {
        return;
    }
    eprintln!(
        "crossing-candidate seq={} net={} partner={} reason={} grid=({:.3},{:.3}) route_angle={} partner_angle={} required_margin={} partner_margin={:.3} route_before={:.3} route_after={:.3}",
        current_search_seq(),
        crossing.net_id,
        partner_net_id,
        reason,
        x,
        y,
        route_angle,
        partner_angle,
        required_margin,
        partner_margin,
        route_before,
        route_after,
    );
}

thread_local! {
    /// Sequence number of the search currently running on this thread, so
    /// every crossing trace line can be attributed to one search.
    pub(crate) static CURRENT_SEARCH_SEQ: std::cell::Cell<u64> = const { std::cell::Cell::new(u64::MAX) };
}

pub(crate) fn current_search_seq() -> u64 {
    CURRENT_SEARCH_SEQ.with(|c| c.get())
}

pub(crate) fn trace_crossing_level1_intersection(
    crossing: &CrossingSearchConfig,
    partner_net_id: u64,
    event: &str,
    x: f64,
    y: f64,
    route_angle: u8,
    partner_angle: u8,
    required_margin: i32,
    route_before: f64,
    route_after: f64,
    pending_after: i32,
) {
    if !crossing.diagnostics.trace_crossing_level1 {
        return;
    }
    if let Some(trace_net_id) = crossing.diagnostics.trace_crossing_net {
        if trace_net_id != crossing.net_id {
            return;
        }
    }
    if let Some(trace_partner_id) = crossing.diagnostics.trace_partner_net {
        if trace_partner_id != partner_net_id {
            return;
        }
    }
    eprintln!(
        "crossing-level1 seq={} net={} partner={} event={} grid=({:.3},{:.3}) route_angle={} partner_angle={} required_margin={} route_before={:.3} route_after={:.3} pending_after={}",
        current_search_seq(),
        crossing.net_id,
        partner_net_id,
        event,
        x,
        y,
        route_angle,
        partner_angle,
        required_margin,
        route_before,
        route_after,
        pending_after,
    );
}

/// The bounding box of the region a search actually explored, maintained
/// only under `KernelDiagnostics::search_failure_diag`. Printed with every
/// search-failure line: it localizes the wall the frontier died at.
#[derive(Clone, Copy)]
pub(crate) struct ExploredBox {
    pub(crate) min_x: i32,
    pub(crate) max_x: i32,
    pub(crate) min_y: i32,
    pub(crate) max_y: i32,
}

impl ExploredBox {
    pub(crate) fn new() -> Self {
        Self {
            min_x: i32::MAX,
            max_x: i32::MIN,
            min_y: i32::MAX,
            max_y: i32::MIN,
        }
    }

    pub(crate) fn note(&mut self, state: State) {
        self.min_x = self.min_x.min(state.x);
        self.max_x = self.max_x.max(state.x);
        self.min_y = self.min_y.min(state.y);
        self.max_y = self.max_y.max(state.y);
    }
}

/// Everything the search-failure and probe reports read that does not
/// change during a search. The kernel builds one before its loop.
pub(crate) struct SearchFailureEnv<'a> {
    pub(crate) search_seq: u64,
    pub(crate) obstacle_map: &'a ObstacleMap,
    pub(crate) port_open_cells: Option<&'a FxHashSet<CellKey>>,
    pub(crate) dense_grid: &'a DenseRoutingGrid,
    pub(crate) source: State,
    pub(crate) target: State,
    pub(crate) bounds: RoutingBounds,
    pub(crate) probe_cells: &'a [(i32, i32)],
    pub(crate) failure_map: Option<&'a [i32]>,
}

/// The mutable diagnostic state one search accumulated, as the failure
/// report reads it at the moment the search gives up.
pub(crate) struct SearchFailureState<'a> {
    pub(crate) iterations: usize,
    pub(crate) explored: ExploredBox,
    pub(crate) target_ring: &'a [[u32; 7]; 25],
    pub(crate) probe_ring: &'a [[u32; 7]],
    pub(crate) goal_miss_angle: u32,
    pub(crate) goal_miss_hook: u32,
    pub(crate) best_crossings: u16,
    pub(crate) best_crossing_ref: Option<usize>,
    pub(crate) storage: &'a DenseSearchStorage,
    pub(crate) extended_nodes: &'a [UnifiedExtendedNode],
}

/// One `probe-landing` line per probe cell that saw any successor attempt.
fn print_probe_landing_lines(
    out: &mut dyn Write,
    search_seq: u64,
    probe_cells: &[(i32, i32)],
    probe_ring: &[[u32; 7]],
) {
    for (i, cell) in probe_cells.iter().enumerate() {
        let c = probe_ring[i];
        if c.iter().any(|v| *v > 0) {
            let _ = writeln!(out, "probe-landing seq={} cell=({},{}) gen={} acc={} foot={} hook={} closed={} pruned={} resv={}", search_seq, cell.0, cell.1, c[0], c[1], c[2], c[3], c[4], c[5], c[6]);
        }
    }
}

/// The full search-failure diagnosis the kernel prints under
/// `KernelDiagnostics::search_failure_diag` at each of its three
/// termination points: the `search-failure` line naming `kind`, then the
/// target-ring, best-crossing-path, probe-cell, failure-map and
/// probe-landing reports, in that order.
pub(crate) fn print_search_failure_report(
    out: &mut dyn Write,
    env: &SearchFailureEnv<'_>,
    kind: &str,
    state: &SearchFailureState<'_>,
    stats: &RouteSearchStats,
) {
    let (search_seq, source, target, bounds) = (env.search_seq, env.source, env.target, env.bounds);
    let explored = state.explored;
    let _ = writeln!(
        out,
            "search-failure seq={} kind={} iterations={} expanded={} generated={} source=({},{},{}) target=({},{},{}) window=[{}..{},{}..{}] explored_bbox=[{}..{},{}..{}]",
            search_seq, kind, state.iterations, stats.expanded_states, stats.generated_neighbors,
            source.x, source.y, source.angle, target.x, target.y, target.angle,
            bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y,
            explored.min_x, explored.max_x, explored.min_y, explored.max_y,
        );
    print_target_ring_report(
        out,
        target,
        state.target_ring,
        state.goal_miss_angle,
        state.goal_miss_hook,
        env.obstacle_map,
        env.port_open_cells,
    );
    print_best_crossing_path(
        out,
        state.best_crossings,
        state.best_crossing_ref,
        state.storage,
        state.extended_nodes,
    );
    print_probe_cells_report(
        out,
        search_seq,
        env.obstacle_map,
        env.port_open_cells,
        env.dense_grid,
        env.probe_cells,
    );
    print_failure_map_window(
        out,
        search_seq,
        env.obstacle_map,
        source,
        target,
        env.failure_map,
    );
    print_probe_landing_lines(out, search_seq, env.probe_cells, state.probe_ring);
}

/// The probe-cell report on a SUCCESSFUL search (where did the search go /
/// not go), not only on failure. A no-op unless probe cells are named.
pub(crate) fn print_probe_success_report(
    out: &mut dyn Write,
    env: &SearchFailureEnv<'_>,
    goal: State,
    probe_ring: &[[u32; 7]],
) {
    if env.probe_cells.is_empty() {
        return;
    }
    let _ = writeln!(
        out,
        "probe-success seq={} goal=({},{},{})",
        env.search_seq, goal.x, goal.y, goal.angle
    );
    print_probe_cells_report(
        out,
        env.search_seq,
        env.obstacle_map,
        env.port_open_cells,
        env.dense_grid,
        env.probe_cells,
    );
    print_failure_map_window(
        out,
        env.search_seq,
        env.obstacle_map,
        env.source,
        env.target,
        env.failure_map,
    );
    print_probe_landing_lines(out, env.search_seq, env.probe_cells, probe_ring);
}

/// Unit tests for the two report entry points, Milestone 6 Slice 3 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
/// Both (and the five helpers they call) gained a `&mut dyn Write` first
/// parameter so the report can be read back here; production passes
/// `std::io::stderr()`, so every line is byte-for-byte what the kernel
/// printed before.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::search::astar::kernel::UnifiedExtendedNode;

    /// The smallest map the reports can run on: 3x3 cells with one static
    /// obstacle at (1, 2), the cell directly left of the target.
    fn tiny_map() -> ObstacleMap {
        let mut map = ObstacleMap::new(3, 3);
        assert!(map.add_static_cell(1, 2));
        map
    }

    fn tiny_bounds() -> RoutingBounds {
        RoutingBounds {
            min_x: 0,
            max_x: 2,
            min_y: 0,
            max_y: 2,
        }
    }

    fn tiny_dense_grid(map: &ObstacleMap) -> DenseRoutingGrid {
        DenseRoutingGrid::from_obstacle_map(
            map,
            tiny_bounds(),
            None,
            1_000_000,
            false,
            false,
            false,
        )
        .expect("a 3x3 dense grid fits every budget")
    }

    fn tiny_storage() -> DenseSearchStorage {
        DenseSearchStorage::new(tiny_bounds(), 1_000_000)
            .expect("3 * 3 * 8 = 72 dense states fit every budget")
    }

    const SOURCE: State = State {
        x: 0,
        y: 0,
        angle: 0,
    };
    const TARGET: State = State {
        x: 2,
        y: 2,
        angle: 0,
    };

    fn env<'a>(
        map: &'a ObstacleMap,
        dense_grid: &'a DenseRoutingGrid,
        probe_cells: &'a [(i32, i32)],
    ) -> SearchFailureEnv<'a> {
        SearchFailureEnv {
            search_seq: 7,
            obstacle_map: map,
            port_open_cells: None,
            dense_grid,
            source: SOURCE,
            target: TARGET,
            bounds: tiny_bounds(),
            probe_cells,
            // The ASCII map dump stays off, so the report is the five
            // lines the other helpers print.
            failure_map: None,
        }
    }

    fn explored_source_to_target() -> ExploredBox {
        let mut explored = ExploredBox::new();
        explored.note(SOURCE);
        explored.note(TARGET);
        explored
    }

    #[test]
    fn print_search_failure_report_prints_the_header_ring_and_bestpath_lines() {
        let map = tiny_map();
        let dense_grid = tiny_dense_grid(&map);
        let storage = tiny_storage();
        let extended_nodes: Vec<UnifiedExtendedNode> = Vec::new();
        let target_ring = [[0u32; 7]; 25];
        let probe_ring: Vec<[u32; 7]> = Vec::new();
        let stats = RouteSearchStats {
            expanded_states: 11,
            generated_neighbors: 22,
            ..RouteSearchStats::default()
        };
        let mut out: Vec<u8> = Vec::new();
        print_search_failure_report(
            &mut out,
            &env(&map, &dense_grid, &[]),
            "open_set_exhausted",
            &SearchFailureState {
                iterations: 5,
                explored: explored_source_to_target(),
                target_ring: &target_ring,
                probe_ring: &probe_ring,
                goal_miss_angle: 3,
                goal_miss_hook: 4,
                best_crossings: 0,
                best_crossing_ref: None,
                storage: &storage,
                extended_nodes: &extended_nodes,
            },
            &stats,
        );
        // Two ring lines only: the static cell one step left of the target
        // and the target itself (the center always prints). Every other
        // cell of the 5x5 ring is free, unopened and has zero counters.
        assert_eq!(
            String::from_utf8(out).expect("the report is UTF-8"),
            concat!(
                "search-failure seq=7 kind=open_set_exhausted iterations=5 expanded=11 ",
                "generated=22 source=(0,0,0) target=(2,2,0) window=[0..2,0..2] ",
                "explored_bbox=[0..2,0..2]\n",
                "search-failure-goalmiss angle=3 hook=4\n",
                "search-failure-ring cell=(1,2) off=(-1,0) static=true opened=false ",
                "gen=0 acc=0 foot=0 hook=0 closed=0 pruned=0 resv=0\n",
                "search-failure-ring cell=(2,2) off=(0,0) static=false opened=false ",
                "gen=0 acc=0 foot=0 hook=0 closed=0 pruned=0 resv=0\n",
                "search-failure-bestpath crossings=0 (no crossing state ever accepted)\n",
            )
        );
    }

    #[test]
    fn print_search_failure_report_names_the_kind_it_was_called_with() {
        let map = tiny_map();
        let dense_grid = tiny_dense_grid(&map);
        let storage = tiny_storage();
        let extended_nodes: Vec<UnifiedExtendedNode> = Vec::new();
        let target_ring = [[0u32; 7]; 25];
        let probe_ring: Vec<[u32; 7]> = Vec::new();
        for kind in ["iteration_cap", "timeout", "open_set_exhausted"] {
            let mut out: Vec<u8> = Vec::new();
            print_search_failure_report(
                &mut out,
                &env(&map, &dense_grid, &[]),
                kind,
                &SearchFailureState {
                    iterations: 5,
                    explored: explored_source_to_target(),
                    target_ring: &target_ring,
                    probe_ring: &probe_ring,
                    goal_miss_angle: 0,
                    goal_miss_hook: 0,
                    best_crossings: 0,
                    best_crossing_ref: None,
                    storage: &storage,
                    extended_nodes: &extended_nodes,
                },
                &RouteSearchStats::default(),
            );
            let report = String::from_utf8(out).expect("the report is UTF-8");
            assert!(
                report.starts_with(&format!("search-failure seq=7 kind={kind} ")),
                "expected kind={kind} in the header, got {report:?}"
            );
        }
    }

    #[test]
    fn print_probe_success_report_prints_the_goal_and_one_line_per_probe_cell() {
        let map = tiny_map();
        let dense_grid = tiny_dense_grid(&map);
        let probe_cells = [(1, 1)];
        // One generated successor landed on the probe cell (slot 0).
        let probe_ring = [[1u32, 0, 0, 0, 0, 0, 0]];
        let mut out: Vec<u8> = Vec::new();
        print_probe_success_report(
            &mut out,
            &env(&map, &dense_grid, &probe_cells),
            TARGET,
            &probe_ring,
        );
        assert_eq!(
            String::from_utf8(out).expect("the report is UTF-8"),
            concat!(
                "probe-success seq=7 goal=(2,2,0)\n",
                "probe-cell seq=7 cell=(1,1) static=false opened=false dynamic_owners=[] ",
                "core=false dense_blocked=false\n",
                "probe-landing seq=7 cell=(1,1) gen=1 acc=0 foot=0 hook=0 closed=0 ",
                "pruned=0 resv=0\n",
            )
        );
    }

    #[test]
    fn print_probe_success_report_is_a_no_op_without_probe_cells() {
        let map = tiny_map();
        let dense_grid = tiny_dense_grid(&map);
        let probe_ring: Vec<[u32; 7]> = Vec::new();
        let mut out: Vec<u8> = Vec::new();
        print_probe_success_report(&mut out, &env(&map, &dense_grid, &[]), TARGET, &probe_ring);
        assert!(out.is_empty(), "expected no output, got {out:?}");
    }

    #[test]
    fn a_probe_cell_outside_the_map_reports_out_of_bounds_and_no_landing_line() {
        let map = tiny_map();
        let dense_grid = tiny_dense_grid(&map);
        let probe_cells = [(9, 9)];
        // All-zero counters: nothing ever tried to land there.
        let probe_ring = [[0u32; 7]];
        let mut out: Vec<u8> = Vec::new();
        print_probe_success_report(
            &mut out,
            &env(&map, &dense_grid, &probe_cells),
            TARGET,
            &probe_ring,
        );
        assert_eq!(
            String::from_utf8(out).expect("the report is UTF-8"),
            concat!(
                "probe-success seq=7 goal=(2,2,0)\n",
                "probe-cell seq=7 cell=(9,9) out_of_bounds\n",
            )
        );
    }
}
