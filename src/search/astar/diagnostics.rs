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
use crate::search::state::{RouteSearchStats, State};
use rustc_hash::FxHashSet;
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
    target: State,
    target_ring: &[[u32; 7]; 25],
    goal_miss_angle: u32,
    goal_miss_hook: u32,
    obstacle_map: &ObstacleMap,
    port_open_cells: Option<&FxHashSet<CellKey>>,
) {
    eprintln!(
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
                eprintln!(
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
    search_seq: u64,
    obstacle_map: &ObstacleMap,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    dense_grid: &DenseRoutingGrid,
    probe_cells: &[(i32, i32)],
) {
    for &(x, y) in probe_cells {
        if !obstacle_map.in_bounds(x, y) {
            eprintln!(
                "probe-cell seq={} cell=({},{}) out_of_bounds",
                search_seq, x, y
            );
            continue;
        }
        let owners: Vec<NetId> = obstacle_map.dynamic_owners_at(x, y).into_iter().collect();
        eprintln!(
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
    eprintln!(
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
        eprintln!("map y={:>6} {}", y, row);
        y -= step;
    }
}

pub(crate) fn print_best_crossing_path(
    best_crossings: u16,
    best_ref: Option<usize>,
    storage: &DenseSearchStorage,
    extended_nodes: &[UnifiedExtendedNode],
) {
    let Some(ext_idx) = best_ref else {
        eprintln!("search-failure-bestpath crossings=0 (no crossing state ever accepted)");
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
    eprintln!(
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
