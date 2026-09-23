//! Routing-window bookkeeping: the bounds type, the total-expansion-budget
//! check, the per-attempt iteration cap, and the windowed search driver that
//! tries progressively larger windows (falling back to the full grid)
//! before giving up. Moved out of `src/astar.rs` (Milestone 3, Slice 2);
//! pure code motion, no behaviour change.

use crate::obstacle_map::ObstacleMap;
use crate::search::astar::config::AStarConfig;
use crate::search::state::{RouteResult, RouteSearchStats, State};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RoutingBounds {
    pub(crate) min_x: i32,
    pub(crate) max_x: i32,
    pub(crate) min_y: i32,
    pub(crate) max_y: i32,
}

impl RoutingBounds {
    #[inline]
    pub(crate) fn contains(&self, x: i32, y: i32) -> bool {
        x >= self.min_x && x <= self.max_x && y >= self.min_y && y <= self.max_y
    }

    #[inline]
    pub(crate) fn expanded_and_clamped(&self, margin: i32, width: i32, height: i32) -> Self {
        let margin = margin.max(0);
        Self {
            min_x: self.min_x.saturating_sub(margin).max(0),
            max_x: self
                .max_x
                .saturating_add(margin)
                .min(width.saturating_sub(1)),
            min_y: self.min_y.saturating_sub(margin).max(0),
            max_y: self
                .max_y
                .saturating_add(margin)
                .min(height.saturating_sub(1)),
        }
    }
}

/// `true` once `stats.expanded_states` (accumulated across every window and
/// fallback attempt this search call has already made) has reached `budget`.
/// `budget = None` never exhausts. Checked between attempts, not inside one,
/// so a single attempt already in flight can still overshoot `budget` by its
/// own expansion count -- see `total_expansion_budget`'s doc comment.
pub(crate) fn budget_exhausted(budget: Option<u64>, stats: &RouteSearchStats) -> bool {
    match budget {
        Some(budget) => stats.expanded_states as u64 >= budget,
        None => false,
    }
}

/// The `max_iterations` cap for one window or full-grid attempt: the
/// smaller of the configured per-attempt cap (`config.max_iterations`) and
/// whatever `total_expansion_budget` still allows given what earlier
/// attempts in this same search call already expanded (`stats`). Without
/// this, a search whose cumulative budget is nearly spent could still let
/// its next attempt run up to the full per-window cap before the
/// between-attempt `budget_exhausted` check ever saw it -- observed as a
/// failing search with a 2,000,000-state budget still taking 96-132 s
/// because each window ran under the unrelated 20,000,000 `max_iterations`
/// cap. `budget_exhausted` already keeps an attempt from launching at all
/// once no budget remains, so `remaining` here is always >= 1 in practice;
/// `.max(1)` only guards against a caller skipping that check. `None`
/// (no budget) leaves `config.max_iterations` unchanged. See
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
/// Milestone 5.
pub(crate) fn effective_max_iterations(config: &AStarConfig, stats: &RouteSearchStats) -> usize {
    match config.total_expansion_budget {
        Some(budget) => {
            let remaining = budget.saturating_sub(stats.expanded_states as u64).max(1);
            (config.max_iterations as u64).min(remaining) as usize
        }
        None => config.max_iterations,
    }
}

pub(crate) fn run_windowed_single_net_search<F>(
    obstacle_map: &ObstacleMap,
    source: State,
    target: State,
    config: &AStarConfig,
    stats: &mut RouteSearchStats,
    mut try_bounds: F,
) -> Option<RouteResult>
where
    F: FnMut(
        &ObstacleMap,
        Option<RoutingBounds>,
        &mut RouteSearchStats,
        usize,
    ) -> Option<RouteResult>,
{
    if !config.use_routing_window {
        return try_bounds(obstacle_map, None, stats, config.max_iterations);
    }

    let diag = config.diagnostics.search_failure_diag;
    let mut last_bounds: Option<RoutingBounds> = None;
    for expansion_idx in 0..=config.routing_window_max_expansions {
        if budget_exhausted(config.total_expansion_budget, stats) {
            if diag {
                eprintln!(
                    "search-failure kind=budget_exhausted_before_window expansion_idx={} expanded={} budget={:?}",
                    expansion_idx, stats.expanded_states, config.total_expansion_budget
                );
            }
            return None;
        }
        let Some(bounds) =
            compute_routing_bounds(obstacle_map, source, target, config, expansion_idx)
        else {
            if diag {
                eprintln!(
                    "search-failure kind=no_routing_bounds expansion_idx={} source=({},{}) target=({},{}) map={}x{}",
                    expansion_idx, source.x, source.y, target.x, target.y, obstacle_map.width(), obstacle_map.height()
                );
            }
            return None;
        };
        if last_bounds == Some(bounds) {
            continue;
        }
        last_bounds = Some(bounds);
        stats.window_attempts += 1;
        stats.max_window_area_cells = stats.max_window_area_cells.max(window_area(bounds));
        stats.last_window_min_x = bounds.min_x;
        stats.last_window_max_x = bounds.max_x;
        stats.last_window_min_y = bounds.min_y;
        stats.last_window_max_y = bounds.max_y;
        stats.last_window_area_cells = window_area(bounds);

        let effective_cap = effective_max_iterations(config, stats);
        if let Some(route) = try_bounds(obstacle_map, Some(bounds), stats, effective_cap) {
            return Some(route);
        }
    }

    if config.routing_window_fallback_full_grid
        && !budget_exhausted(config.total_expansion_budget, stats)
    {
        stats.window_attempts += 1;
        stats.used_full_grid_fallback = true;
        let full_bounds = RoutingBounds {
            min_x: 0,
            max_x: obstacle_map.width() - 1,
            min_y: 0,
            max_y: obstacle_map.height() - 1,
        };
        stats.last_window_min_x = full_bounds.min_x;
        stats.last_window_max_x = full_bounds.max_x;
        stats.last_window_min_y = full_bounds.min_y;
        stats.last_window_max_y = full_bounds.max_y;
        stats.last_window_area_cells = window_area(full_bounds);
        stats.max_window_area_cells = stats.max_window_area_cells.max(window_area(full_bounds));
        let effective_cap = effective_max_iterations(config, stats);
        return try_bounds(obstacle_map, None, stats, effective_cap);
    }

    if diag {
        eprintln!(
            "search-failure kind=windows_exhausted_no_fallback window_attempts={} expanded={} fallback_full_grid={} budget={:?}",
            stats.window_attempts, stats.expanded_states, config.routing_window_fallback_full_grid, config.total_expansion_budget
        );
    }
    None
}

pub(crate) fn compute_routing_bounds(
    obstacle_map: &ObstacleMap,
    source: State,
    target: State,
    config: &AStarConfig,
    expansion_idx: u32,
) -> Option<RoutingBounds> {
    let span_x = (target.x - source.x).abs();
    let span_y = (target.y - source.y).abs();

    let growth = 1.0 + (expansion_idx as f64) * config.routing_window_growth.max(0.0);
    let tolerance_padding = config.target_tolerance_cells.max(0);
    let margin_x = ((config.routing_window_scale.max(0.0) * (span_x as f64) * growth).ceil()
        as i32)
        .max(config.routing_window_min_margin_cells.max(0))
        .saturating_add(tolerance_padding);
    let margin_y = ((config.routing_window_scale.max(0.0) * (span_y as f64) * growth).ceil()
        as i32)
        .max(config.routing_window_min_margin_cells.max(0))
        .saturating_add(tolerance_padding);

    let min_x = source.x.min(target.x).saturating_sub(margin_x).max(0);
    let max_x = source
        .x
        .max(target.x)
        .saturating_add(margin_x)
        .min(obstacle_map.width() - 1);
    let min_y = source.y.min(target.y).saturating_sub(margin_y).max(0);
    let max_y = source
        .y
        .max(target.y)
        .saturating_add(margin_y)
        .min(obstacle_map.height() - 1);

    if min_x > max_x || min_y > max_y {
        return None;
    }
    Some(RoutingBounds {
        min_x,
        max_x,
        min_y,
        max_y,
    })
}

pub(crate) fn window_area(bounds: RoutingBounds) -> i64 {
    let width = (bounds.max_x - bounds.min_x + 1).max(0) as i64;
    let height = (bounds.max_y - bounds.min_y + 1).max(0) as i64;
    width * height
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::search::astar::crossing_rules::NoCrossingHook;
    use crate::search::astar::kernel::route_single_net_with_bounds_unified;
    use crate::search::astar::route_single_net_with_config;
    use crate::search::test_support::*;

    fn stub_route_result(source: State, target: State) -> RouteResult {
        RouteResult {
            states: vec![source, target],
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: Vec::new(),
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: target,
            reached_target: target,
            stats: RouteSearchStats::default(),
        }
    }

    #[test]
    fn windowed_single_net_search_without_window_calls_try_bounds_once_with_none() {
        let map = ObstacleMap::new(12, 5);
        let source = State::new(1, 2, 0);
        let target = State::new(8, 2, 0);
        let config = AStarConfig {
            use_routing_window: false,
            ..AStarConfig::default()
        };
        let mut stats = RouteSearchStats::default();
        let mut calls: Vec<Option<RoutingBounds>> = Vec::new();

        let result = run_windowed_single_net_search(
            &map,
            source,
            target,
            &config,
            &mut stats,
            |_obstacle_map, bounds, _stats, _effective_max_iterations| {
                calls.push(bounds);
                Some(stub_route_result(source, target))
            },
        );

        assert!(result.is_some());
        assert_eq!(calls, vec![None]);
        assert_eq!(stats.window_attempts, 0);
        assert!(!stats.used_full_grid_fallback);
    }

    #[test]
    fn windowed_single_net_search_returns_on_first_windowed_success() {
        let map = ObstacleMap::new(12, 5);
        let source = State::new(1, 2, 0);
        let target = State::new(8, 2, 0);
        let config = AStarConfig {
            use_routing_window: true,
            routing_window_max_expansions: 3,
            ..AStarConfig::default()
        };
        let mut stats = RouteSearchStats::default();
        let mut call_count = 0;

        let result = run_windowed_single_net_search(
            &map,
            source,
            target,
            &config,
            &mut stats,
            |_obstacle_map, bounds, _stats, _effective_max_iterations| {
                call_count += 1;
                assert!(bounds.is_some(), "windowed attempts must pass Some(bounds)");
                Some(stub_route_result(source, target))
            },
        );

        assert!(result.is_some());
        assert_eq!(call_count, 1);
        assert_eq!(stats.window_attempts, 1);
        assert!(!stats.used_full_grid_fallback);
    }

    #[test]
    fn windowed_single_net_search_falls_back_to_full_grid_when_expansions_exhausted() {
        let map = ObstacleMap::new(12, 5);
        let source = State::new(1, 2, 0);
        let target = State::new(8, 2, 0);
        let config = AStarConfig {
            use_routing_window: true,
            routing_window_max_expansions: 1,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        let mut stats = RouteSearchStats::default();
        let mut calls: Vec<Option<RoutingBounds>> = Vec::new();

        let result = run_windowed_single_net_search(
            &map,
            source,
            target,
            &config,
            &mut stats,
            |_obstacle_map, bounds, _stats, _effective_max_iterations| {
                calls.push(bounds);
                None
            },
        );

        // The first windowed attempt is never skipped by the last-bounds dedup
        // (there is no prior bounds to compare against), and the full-grid
        // fallback always fires one final unconditional attempt with
        // bounds=None regardless of how many windowed attempts were actually
        // distinct -- so at least 2 calls are guaranteed, with the last one
        // being the full-grid attempt, independent of this test's specific
        // map/config geometry.
        assert!(result.is_none());
        assert!(
            calls.len() >= 2,
            "expected at least a windowed attempt and a full-grid fallback attempt, got {calls:?}"
        );
        assert_eq!(
            calls.last(),
            Some(&None),
            "the final attempt must be the full-grid fallback (bounds=None)"
        );
        assert!(stats.used_full_grid_fallback);
    }

    #[test]
    fn windowed_single_net_search_gives_up_without_full_grid_fallback() {
        let map = ObstacleMap::new(12, 5);
        let source = State::new(1, 2, 0);
        let target = State::new(8, 2, 0);
        let config = AStarConfig {
            use_routing_window: true,
            routing_window_max_expansions: 1,
            routing_window_fallback_full_grid: false,
            ..AStarConfig::default()
        };
        let mut stats = RouteSearchStats::default();
        let mut calls: Vec<Option<RoutingBounds>> = Vec::new();

        let result = run_windowed_single_net_search(
            &map,
            source,
            target,
            &config,
            &mut stats,
            |_obstacle_map, bounds, _stats, _effective_max_iterations| {
                calls.push(bounds);
                None
            },
        );

        assert!(result.is_none());
        assert!(calls.iter().all(Option::is_some), "no full-grid fallback attempt should occur when routing_window_fallback_full_grid is false, got {calls:?}");
        assert!(!stats.used_full_grid_fallback);
    }

    /// A window config whose margin grows enough between successive
    /// `expansion_idx` values that `compute_routing_bounds` never produces
    /// the same bounds twice (so the windowed loop's last-bounds dedup never
    /// skips an attempt) -- used by the `total_expansion_budget` tests below
    /// to exercise several real window attempts before the budget can stop
    /// the loop between them. See Milestone 5 of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`.
    fn wide_growth_window_config() -> (ObstacleMap, State, State, AStarConfig) {
        let map = ObstacleMap::new(1000, 20);
        let source = State::new(10, 10, 0);
        let target = State::new(400, 10, 0);
        let config = AStarConfig {
            use_routing_window: true,
            routing_window_min_margin_cells: 5,
            routing_window_scale: 0.1,
            routing_window_growth: 1.0,
            routing_window_max_expansions: 5,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        (map, source, target, config)
    }

    #[test]
    fn total_expansion_budget_stops_the_windowed_loop_once_reached() {
        let (map, source, target, mut config) = wide_growth_window_config();
        config.total_expansion_budget = Some(100);
        let mut stats = RouteSearchStats::default();
        let mut calls: Vec<Option<RoutingBounds>> = Vec::new();

        let result = run_windowed_single_net_search(
            &map,
            source,
            target,
            &config,
            &mut stats,
            |_obstacle_map, bounds, stats, _effective_max_iterations| {
                calls.push(bounds);
                // Never finds a route, however many expansions it is given --
                // stands in for a net whose route genuinely needs more
                // expansions than the budget allows.
                stats.expanded_states += 80;
                None
            },
        );

        assert!(
            result.is_none(),
            "a budget smaller than what the route needs must fail fast, not eventually succeed"
        );
        assert_eq!(
            calls.len(),
            2,
            "the loop must stop once the cumulative budget is spent, before a third window or the full-grid fallback, got {calls:?}"
        );
        assert!(
            (stats.expanded_states as u64) <= 100 + 80,
            "cumulative expansions must not exceed the budget by more than one window's own overshoot, got {}",
            stats.expanded_states
        );
        assert!(
            !stats.used_full_grid_fallback,
            "the full-grid fallback must not run once the budget is already exhausted"
        );
    }

    #[test]
    fn total_expansion_budget_none_lets_the_windowed_loop_run_to_success() {
        let (map, source, target, config) = wide_growth_window_config();
        assert_eq!(
            config.total_expansion_budget, None,
            "this test's baseline must be the unbudgeted default"
        );
        let mut stats = RouteSearchStats::default();
        let mut call_count = 0;

        let result = run_windowed_single_net_search(
            &map,
            source,
            target,
            &config,
            &mut stats,
            |_obstacle_map, bounds, stats, _effective_max_iterations| {
                call_count += 1;
                stats.expanded_states += 80;
                assert!(bounds.is_some(), "windowed attempts must pass Some(bounds)");
                // Succeeds only on the third distinct window -- an unbudgeted
                // search must be allowed to reach it, unlike the budgeted
                // test above which is cut off after the second.
                (call_count >= 3).then(|| stub_route_result(source, target))
            },
        );

        assert!(
            result.is_some(),
            "without a budget the loop must keep trying windows until one finds the route"
        );
        assert_eq!(call_count, 3);
    }

    #[test]
    fn total_expansion_budget_caps_each_attempts_own_max_iterations() {
        // A large open map with the target sealed behind a one-cell ring of
        // static obstacles, so no primitive can ever reach it and the real
        // kernel must keep expanding reachable cells until either the open
        // set is empty or `max_iterations` is hit -- there are ~12,000
        // reachable cells here (the whole window), far more than the 5,000
        // budget below, so if the per-attempt cap were not applied the
        // attempt would run under its full `max_iterations` (100,000, from
        // `AStarConfig::default()`) instead of stopping near the budget.
        let mut map = ObstacleMap::new(200, 60);
        let (tx, ty) = (100, 30);
        for dx in -1..=1i32 {
            for dy in -1..=1i32 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                map.add_static_cell(tx + dx, ty + dy);
            }
        }
        let library = primitive_library();
        let source = State::new(5, 5, 0);
        let target = State::new(tx, ty, 0);
        let config = AStarConfig {
            use_routing_window: true,
            routing_window_min_margin_cells: 500,
            routing_window_max_expansions: 0,
            routing_window_fallback_full_grid: false,
            total_expansion_budget: Some(5_000),
            ..AStarConfig::default()
        };
        let mut stats = RouteSearchStats::default();

        let result = run_windowed_single_net_search(
            &map,
            source,
            target,
            &config,
            &mut stats,
            |obstacle_map, bounds, stats, effective_max_iterations| {
                let mut attempt_config = config.clone();
                attempt_config.max_iterations = effective_max_iterations;
                route_single_net_with_bounds_unified(
                    obstacle_map,
                    &library,
                    source,
                    target,
                    None,
                    &attempt_config,
                    bounds,
                    stats,
                    0,
                    None,
                    &NoCrossingHook,
                )
            },
        );

        assert!(
            result.is_none(),
            "the sealed-off target must never be reached"
        );
        assert!(
            (stats.expanded_states as u64) <= 5_000,
            "the attempt's own expansion count must be bounded by the budget, got {}",
            stats.expanded_states
        );
        assert!(
            (stats.expanded_states as usize) < config.max_iterations,
            "the attempt must stop well short of the window's own max_iterations cap, got {} of {}",
            stats.expanded_states,
            config.max_iterations
        );
    }

    #[test]
    fn nonzero_offset_window_routes() {
        let map = ObstacleMap::new(30, 30);
        let library = primitive_library();
        let result = route_single_net_with_bounds_unified(
            &map,
            &library,
            State::new(11, 11, 0),
            State::new(15, 11, 0),
            None,
            &AStarConfig::default(),
            Some(RoutingBounds {
                min_x: 10,
                max_x: 20,
                min_y: 10,
                max_y: 20,
            }),
            &mut RouteSearchStats::default(),
            0,
            None,
            &NoCrossingHook,
        )
        .expect("route in offset bounds should exist");
        assert_eq!(result.states.first().copied(), Some(State::new(11, 11, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(15, 11, 0)));
    }

    #[test]
    fn full_grid_fallback_uses_dense_storage() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        map.add_static_cell(4, 1);
        map.add_static_cell(5, 1);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(7, 1, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                routing_window_max_expansions: 0,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        )
        .expect("full-grid fallback should find a detour");
        assert_eq!(result.states.last().copied(), Some(State::new(7, 1, 0)));
        assert!(result.stats.used_full_grid_fallback);
    }

    #[test]
    fn duplicate_routing_window_expansions_are_skipped() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        map.add_static_cell(4, 1);
        map.add_static_cell(5, 1);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(7, 1, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                routing_window_max_expansions: 3,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        )
        .expect("full-grid fallback should find a detour");
        assert!(result.stats.used_full_grid_fallback);
        assert_eq!(result.stats.window_attempts, 2);
    }

    #[test]
    fn routing_window_padding_includes_target_tolerance() {
        let map = ObstacleMap::new(20, 20);
        let source = State::new(10, 10, 0);
        let target = State::new(10, 10, 0);
        let bounds = compute_routing_bounds(
            &map,
            source,
            target,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                target_tolerance_cells: 2,
                ..AStarConfig::default()
            },
            0,
        )
        .unwrap();
        assert_eq!(bounds.min_x, 8);
        assert_eq!(bounds.max_x, 12);
        assert_eq!(bounds.min_y, 8);
        assert_eq!(bounds.max_y, 12);
    }
}
