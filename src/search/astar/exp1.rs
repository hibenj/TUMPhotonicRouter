//! Experiment 1 instrumentation (2026-10-29,
//! `.agent/TUMPhotonicRouter/experiments/01_instrumentation.md`): one
//! `EXP1 search` line per search and one `EXP1 run-summary` line per
//! negotiated batch, both on stderr, both only under
//! `KernelDiagnostics::exp_kernel_profile`
//! (`PHOTONIC_ROUTER_EXP_KERNEL_PROFILE`). Nothing here changes a search.
//!
//! The per-search line carries what the heuristic-quality and
//! cost-per-expansion questions need: the final route cost and length,
//! the admissible estimate at the source (`h0`, the kernel's own
//! `SearchHeuristic`), the plain Euclidean grid distance, the route's
//! cell count, the expansions and generated neighbours summed over all
//! window attempts, the kernel-loop wall time and the time inside the
//! crossing-legality hook.

use std::sync::atomic::{AtomicU64, Ordering};

use super::config::AStarConfig;
use super::heuristic::{distance_heuristic, SearchHeuristic};
use crate::primitives::PrimitiveLibrary;
use crate::search::state::{RouteResult, RouteSearchStats, State};

static SEARCHES: AtomicU64 = AtomicU64::new(0);
static SUCCESSES: AtomicU64 = AtomicU64::new(0);
static EXPANDED: AtomicU64 = AtomicU64::new(0);
static GENERATED: AtomicU64 = AtomicU64::new(0);
static HOOK_CALLS: AtomicU64 = AtomicU64::new(0);
static HOOK_NS: AtomicU64 = AtomicU64::new(0);
static LOOP_US: AtomicU64 = AtomicU64::new(0);
static CROSSING_SEARCHES: AtomicU64 = AtomicU64::new(0);

#[allow(clippy::too_many_arguments)]
pub(crate) fn record_search(
    route: Option<&RouteResult>,
    stats: &RouteSearchStats,
    source: State,
    target: State,
    primitives: &PrimitiveLibrary,
    config: &AStarConfig,
    crossing_hook: bool,
) {
    let grid = primitives.grid_size_um();
    let h0 = SearchHeuristic::new(target, primitives, config).estimate(source);
    let euclid = distance_heuristic(source, target, grid);
    let (ok, cost, length_um, cells) = match route {
        Some(route) => (
            1,
            route.total_cost,
            route.total_length_um,
            route.cells.len(),
        ),
        None => (0, f64::NAN, f64::NAN, 0),
    };
    eprintln!(
        "EXP1 search ok={ok} crossing={} ignore_dyn={} cost={cost:.3} len_um={length_um:.3} \
         h0={h0:.3} euclid={euclid:.3} cells={cells} expanded={} generated={} \
         hook_calls={} hook_ns={} loop_us={} windows={} fullgrid={} \
         simple={} hotpath_us={}",
        u8::from(crossing_hook),
        u8::from(config.ignore_dynamic_obstacles),
        stats.expanded_states,
        stats.generated_neighbors,
        stats.exp_hook_calls,
        stats.exp_hook_time_ns,
        stats.search_loop_time_us,
        stats.window_attempts,
        u8::from(stats.used_full_grid_fallback),
        u8::from(ok == 1 && stats.expanded_states == 0),
        stats.crossing_hotpath_total_time_us,
    );
    SEARCHES.fetch_add(1, Ordering::Relaxed);
    SUCCESSES.fetch_add(ok, Ordering::Relaxed);
    EXPANDED.fetch_add(stats.expanded_states as u64, Ordering::Relaxed);
    GENERATED.fetch_add(stats.generated_neighbors as u64, Ordering::Relaxed);
    HOOK_CALLS.fetch_add(stats.exp_hook_calls as u64, Ordering::Relaxed);
    HOOK_NS.fetch_add(stats.exp_hook_time_ns as u64, Ordering::Relaxed);
    LOOP_US.fetch_add(stats.search_loop_time_us as u64, Ordering::Relaxed);
    CROSSING_SEARCHES.fetch_add(u64::from(crossing_hook), Ordering::Relaxed);
}

/// Prints the totals since the process started (or the last call) and
/// resets them. Called once per negotiated batch by the engine's loop.
pub fn print_run_summary(label: &str) {
    let searches = SEARCHES.swap(0, Ordering::Relaxed);
    let successes = SUCCESSES.swap(0, Ordering::Relaxed);
    let expanded = EXPANDED.swap(0, Ordering::Relaxed);
    let generated = GENERATED.swap(0, Ordering::Relaxed);
    let hook_calls = HOOK_CALLS.swap(0, Ordering::Relaxed);
    let hook_ns = HOOK_NS.swap(0, Ordering::Relaxed);
    let loop_us = LOOP_US.swap(0, Ordering::Relaxed);
    let crossing_searches = CROSSING_SEARCHES.swap(0, Ordering::Relaxed);
    let us_per_expansion = if expanded > 0 {
        loop_us as f64 / expanded as f64
    } else {
        f64::NAN
    };
    let hook_share = if loop_us > 0 {
        hook_ns as f64 / 1000.0 / loop_us as f64
    } else {
        f64::NAN
    };
    eprintln!(
        "EXP1 run-summary label={label} searches={searches} ok={successes} \
         crossing_searches={crossing_searches} expanded={expanded} generated={generated} \
         loop_ms={:.1} hook_calls={hook_calls} hook_ms={:.1} \
         us_per_expansion={us_per_expansion:.2} hook_share={hook_share:.3}",
        loop_us as f64 / 1000.0,
        hook_ns as f64 / 1.0e6,
    );
}
