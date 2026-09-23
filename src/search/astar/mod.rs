//! The A* search engine: `AStarSearch` (the `NetSearch` implementation),
//! its two stable free-function conveniences (`route_single_net`,
//! `route_single_net_with_config`), and the four
//! `route_single_net_with_unified_kernel_*` wrappers `AStarSearch::search`
//! dispatches to. The wrappers' own submodules: `config` (types and
//! constants), `state` and `geometry` live one level up in `crate::search`
//! since they are shared with a future second engine, `heuristic`, `cost`,
//! `expansion`, `crossing_rules`, `window`, `dense`, `kernel`, `simple`,
//! `svg`, `diagnostics`.
//!
//! Moved out of `src/astar.rs` (Milestone 3, Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! continuing Slice 1's `src/search/astar_engine.rs`); pure code motion, no
//! behaviour change.

pub mod config;
pub(crate) mod cost;
pub(crate) mod crossing_rules;
pub(crate) mod dense;
pub(crate) mod diagnostics;
pub(crate) mod expansion;
pub(crate) mod heuristic;
pub(crate) mod kernel;
pub(crate) mod simple;
pub mod svg;
pub(crate) mod window;

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::PrimitiveLibrary;
use config::AStarConfig;
use crossing_rules::{CrossingHookContext, CrossingSearchConfig, NoCrossingHook};
use dense::{evaluate_jps4_eligibility, route_single_net_jps4};
use kernel::route_single_net_with_bounds_unified;
use simple::try_simple_route_with_config;
use window::{run_windowed_single_net_search, RoutingBounds};

use crate::search::state::{RouteResult, RouteSearchStats, State};
use rustc_hash::FxHashSet;
use std::time::Instant;

use super::{NetSearch, SearchEnvironment, SearchOutcome, SearchRequest};

/// The kernel today's production callers have always used, now reached
/// through `NetSearch` instead of the deleted four-method single-net-search
/// trait.
pub struct AStarSearch;

impl NetSearch for AStarSearch {
    /// Reproduces the dispatch the old trait's four methods encoded as four
    /// distinct call sites: which wrapper runs depends only on whether the
    /// request carries a `crossing` part and,
    /// if so, whether that part carries its own `reservation_open_cells`.
    ///
    /// `crossing: None` (dynamic_expansion either way) selects between the
    /// plain wrapper (dynamic_expansion: None -- the only variant that
    /// tries the JPS4 and simple-route shortcuts before falling back to the
    /// windowed kernel) and the dynamic-expansion wrapper (dynamic_expansion:
    /// Some(_) -- no shortcuts, a widened clearance halo instead).
    ///
    /// `crossing: Some(c)` always reaches one of the two crossing wrappers.
    /// Reading both (`route_single_net_with_unified_kernel_collision_crossing_config_with_stats`
    /// and `route_single_net_with_unified_kernel_crossing_config`,
    /// `src/search/astar/mod.rs`) shows they differ in exactly one place: the collision
    /// wrapper builds a `reservation_anchor_open_cells` set from
    /// `reservation_open_cells.or(port_open_cells)` plus the source/target
    /// cells and passes it as the hook's *reservation* anchor slot, while
    /// the *port* anchor slot always gets the `port_open_cells`-derived set;
    /// the crossing-config wrapper has no `reservation_open_cells` parameter
    /// at all and passes its one `port_open_cells`-derived set for *both*
    /// hook slots. When `c.reservation_open_cells` is `None`, the collision
    /// wrapper's `.or(port_open_cells)` fallback makes
    /// `reservation_anchor_open_cells` built from the exact same source
    /// (`port_open_cells` plus source/target) as its `anchor_open_cells` --
    /// so the two sets are equal in content, and the collision wrapper's
    /// hook call becomes identical to the crossing-config wrapper's. Every
    /// other step (guard clauses, `CrossingHookContext::build`, the
    /// windowed kernel call) is already the same code path in both
    /// wrappers. So "reservation_open_cells absent" *is* behaviourally
    /// identical to what the crossing-config wrapper does today -- no
    /// `crossing_variant` compatibility flag is needed on `CrossingSearch`.
    ///
    /// That leaves a choice of *which* wrapper to call for
    /// `reservation_open_cells: None`: the crossing-config wrapper named by
    /// the dispatch rule, or the (proven-equivalent) collision wrapper. This
    /// picks the crossing-config wrapper, literally, because: (1) it is the
    /// wrapper the three `search_with_crossing_config` production call
    /// sites reached before this slice, so naming it here keeps that fact
    /// visible in the dispatch instead of relying only on the equivalence
    /// proof above; (2) calling the collision wrapper instead would leave
    /// the crossing-config wrapper referenced only from tests, which is a
    /// `dead_code` warning risk in a non-test build the gate checks for
    /// (`cargo build --release` warning count). Its cost is stats: the
    /// crossing-config wrapper's signature is `-> Option<RouteResult>` with
    /// no stats output at all, and this slice does not change wrapper
    /// bodies, so there is truly no way to plumb real counters out of it
    /// without touching its signature. `SearchOutcome::stats` is therefore
    /// `RouteSearchStats::default()` for this one variant. This is zero
    /// behaviour change: all three call sites that reach this branch
    /// (`src/engine/search_calls.rs`) already discarded the trait's
    /// `search_with_crossing_config` return value's stats too -- that
    /// method never had a stats output either, old or new.
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome {
        if let Some(crossing) = &request.crossing {
            let dynamic_expansion_radius_cells = request
                .dynamic_expansion
                .as_ref()
                .map_or(0, |dynamic| dynamic.radius_cells);
            let dynamic_clearance_exempt_cells = request
                .dynamic_expansion
                .as_ref()
                .and_then(|dynamic| dynamic.clearance_exempt_cells);
            return if let Some(reservation_open_cells) = crossing.reservation_open_cells {
                let (route, stats) =
                    route_single_net_with_unified_kernel_collision_crossing_config_with_stats(
                        env.obstacle_map,
                        env.primitives,
                        request.source,
                        request.target,
                        request.port_open_cells,
                        Some(reservation_open_cells),
                        request.config,
                        dynamic_expansion_radius_cells,
                        dynamic_clearance_exempt_cells,
                        crossing.config,
                    );
                SearchOutcome { route, stats }
            } else {
                let route = route_single_net_with_unified_kernel_crossing_config(
                    env.obstacle_map,
                    env.primitives,
                    request.source,
                    request.target,
                    request.port_open_cells,
                    request.config,
                    dynamic_expansion_radius_cells,
                    dynamic_clearance_exempt_cells,
                    crossing.config,
                );
                SearchOutcome {
                    route,
                    stats: RouteSearchStats::default(),
                }
            };
        }

        if let Some(dynamic) = &request.dynamic_expansion {
            let mut stats = RouteSearchStats::default();
            let route =
                route_single_net_with_unified_kernel_dynamic_expansion_config_reporting_stats(
                    env.obstacle_map,
                    env.primitives,
                    request.source,
                    request.target,
                    request.port_open_cells,
                    request.config,
                    dynamic.radius_cells,
                    dynamic.clearance_exempt_cells,
                    &mut stats,
                );
            return SearchOutcome { route, stats };
        }

        let mut stats = RouteSearchStats::default();
        let route = route_single_net_with_unified_kernel_config_reporting_stats(
            env.obstacle_map,
            env.primitives,
            request.source,
            request.target,
            request.port_open_cells,
            request.config,
            &mut stats,
        );
        SearchOutcome { route, stats }
    }
}

pub(crate) fn with_route_search_total_time(
    mut route: RouteResult,
    route_search_total_start: Option<&Instant>,
) -> RouteResult {
    if let Some(route_search_total_start) = route_search_total_start {
        route.stats.route_search_total_time_us += route_search_total_start.elapsed().as_micros();
    }
    route
}

/// Route a single net with default A* settings.
pub fn route_single_net(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
) -> Option<RouteResult> {
    route_single_net_with_config(
        obstacle_map,
        primitives,
        source,
        target,
        port_open_cells,
        &AStarConfig::default(),
    )
}

/// Route a single net with explicit A* settings. A convenience over
/// `AStarSearch::search` with a plain request (no dynamic expansion, no
/// crossing) -- kept as its own free function because dozens of tests
/// exercise general A* properties (heuristics, tie-breaking, congestion,
/// indexed-heap parity, dense-grid construction) through this exact
/// signature.
pub fn route_single_net_with_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
) -> Option<RouteResult> {
    let env = crate::search::SearchEnvironment {
        obstacle_map,
        primitives,
    };
    let request = crate::search::SearchRequest {
        source,
        target,
        port_open_cells,
        dynamic_expansion: None,
        crossing: None,
        config,
    };
    crate::search::AStarSearch.search(&env, &request).route
}

/// Production entry points, one per `crate::search::NetSearch::search`
/// dispatch branch (`crate::search::astar_engine::AStarSearch::search`).
/// Each mirrors its plain/crossing-kernel counterpart above exactly
/// (same validation guards, same anchor-cell construction, same
/// `run_windowed_single_net_search` wiring), swapping only which kernel
/// function the windowed closure calls.
pub(crate) fn route_single_net_with_unified_kernel_config_reporting_stats(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    out_stats: &mut RouteSearchStats,
) -> Option<RouteResult> {
    if config.target_tolerance_cells < 0 {
        *out_stats = RouteSearchStats::default();
        return None;
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        if mask == 0 {
            *out_stats = RouteSearchStats::default();
            return None;
        }
    }
    if target.angle > 7 {
        *out_stats = RouteSearchStats::default();
        return None;
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        *out_stats = RouteSearchStats::default();
        return None;
    }
    let mut anchor_open_cells = FxHashSet::default();
    if let Some(port_open_cells) = port_open_cells {
        anchor_open_cells.extend(port_open_cells.iter().copied());
    }
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));
    let mut stats = RouteSearchStats::default();
    let route_search_total_start = if config.collect_detailed_timing {
        Some(Instant::now())
    } else {
        None
    };
    let jps4_eligibility = evaluate_jps4_eligibility(primitives, source, target, config);
    stats.jps4_requested = config.enable_jps4;
    stats.jps4_eligible = jps4_eligibility.eligible;
    stats.jps4_fallback_reason = jps4_eligibility.reason.to_string();
    if config.enable_jps4 && jps4_eligibility.eligible {
        if let Some(route) = route_single_net_jps4(
            obstacle_map,
            source,
            target,
            Some(&anchor_open_cells),
            config,
            primitives.grid_size_um(),
            stats.clone(),
        ) {
            *out_stats = stats.clone();
            return Some(with_route_search_total_time(
                route,
                route_search_total_start.as_ref(),
            ));
        }
        stats.jps4_fallbacks += 1;
        stats.jps4_fallback_reason = "jps4 search failed".to_string();
    } else if config.enable_jps4 {
        stats.jps4_fallbacks += 1;
    }
    let simple_route_start = if config.collect_detailed_timing {
        Some(Instant::now())
    } else {
        None
    };
    let simple_route = try_simple_route_with_config(
        obstacle_map,
        primitives,
        source,
        target,
        port_open_cells,
        config,
    );
    if let Some(simple_route_start) = simple_route_start.as_ref() {
        stats.simple_route_time_us += simple_route_start.elapsed().as_micros();
    }
    if let Some(mut simple_route) = simple_route {
        simple_route.stats = stats.clone();
        *out_stats = stats.clone();
        return Some(with_route_search_total_time(
            simple_route,
            route_search_total_start.as_ref(),
        ));
    }

    let route = run_windowed_single_net_search(
        obstacle_map,
        source,
        target,
        config,
        &mut stats,
        |obstacle_map, bounds, stats, effective_max_iterations| {
            let mut attempt_config = config.clone();
            attempt_config.max_iterations = effective_max_iterations;
            route_single_net_with_bounds_unified(
                obstacle_map,
                primitives,
                source,
                target,
                Some(&anchor_open_cells),
                &attempt_config,
                bounds,
                stats,
                0,
                None,
                &NoCrossingHook,
            )
        },
    )
    .map(|route| with_route_search_total_time(route, route_search_total_start.as_ref()));
    *out_stats = stats.clone();
    route
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn route_single_net_with_unified_kernel_dynamic_expansion_config_reporting_stats(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    dynamic_expansion_radius_cells: i32,
    dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
    out_stats: &mut RouteSearchStats,
) -> Option<RouteResult> {
    if config.target_tolerance_cells < 0 {
        *out_stats = RouteSearchStats::default();
        return None;
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        if mask == 0 {
            *out_stats = RouteSearchStats::default();
            return None;
        }
    }
    if target.angle > 7 {
        *out_stats = RouteSearchStats::default();
        return None;
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        *out_stats = RouteSearchStats::default();
        return None;
    }
    let mut anchor_open_cells = FxHashSet::default();
    if let Some(port_open_cells) = port_open_cells {
        anchor_open_cells.extend(port_open_cells.iter().copied());
    }
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));

    let mut stats = RouteSearchStats::default();
    let route_search_total_start = if config.collect_detailed_timing {
        Some(Instant::now())
    } else {
        None
    };
    stats.jps4_requested = config.enable_jps4;
    stats.jps4_eligible = false;
    stats.jps4_fallback_reason = "unified kernel uses dense A*".to_string();
    if config.enable_jps4 {
        stats.jps4_fallbacks += 1;
    }

    let route = run_windowed_single_net_search(
        obstacle_map,
        source,
        target,
        config,
        &mut stats,
        |obstacle_map, bounds, stats, effective_max_iterations| {
            let mut attempt_config = config.clone();
            attempt_config.max_iterations = effective_max_iterations;
            route_single_net_with_bounds_unified(
                obstacle_map,
                primitives,
                source,
                target,
                Some(&anchor_open_cells),
                &attempt_config,
                bounds,
                stats,
                dynamic_expansion_radius_cells,
                dynamic_clearance_exempt_cells,
                &NoCrossingHook,
            )
        },
    )
    .map(|route| with_route_search_total_time(route, route_search_total_start.as_ref()));
    *out_stats = stats.clone();
    route
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn route_single_net_with_unified_kernel_collision_crossing_config_with_stats(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    reservation_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    dynamic_expansion_radius_cells: i32,
    dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
    crossing: &CrossingSearchConfig,
) -> (Option<RouteResult>, RouteSearchStats) {
    if crossing.partners.is_empty()
        || (crossing.require_all_partners && crossing.partners.len() >= u64::BITS as usize)
    {
        return (None, RouteSearchStats::default());
    }
    if config.target_tolerance_cells < 0 {
        return (None, RouteSearchStats::default());
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        if mask == 0 {
            return (None, RouteSearchStats::default());
        }
    }
    if target.angle > 7 {
        return (None, RouteSearchStats::default());
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        return (None, RouteSearchStats::default());
    }
    let mut anchor_open_cells = FxHashSet::default();
    if let Some(port_open_cells) = port_open_cells {
        anchor_open_cells.extend(port_open_cells.iter().copied());
    }
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));
    let mut reservation_anchor_open_cells = FxHashSet::default();
    if let Some(reservation_open_cells) = reservation_open_cells.or(port_open_cells) {
        reservation_anchor_open_cells.extend(reservation_open_cells.iter().copied());
    }
    reservation_anchor_open_cells.insert(pack_xy(source.x, source.y));
    reservation_anchor_open_cells.insert(pack_xy(target.x, target.y));

    let mut stats = RouteSearchStats::default();
    let route_search_total_start = if config.collect_detailed_timing {
        Some(Instant::now())
    } else {
        None
    };
    stats.jps4_requested = config.enable_jps4;
    stats.jps4_eligible = false;
    stats.jps4_fallback_reason = "unified kernel uses augmented A*".to_string();
    if config.enable_jps4 {
        stats.jps4_fallbacks += 1;
    }

    let result = run_windowed_single_net_search(
        obstacle_map,
        source,
        target,
        config,
        &mut stats,
        |obstacle_map, bounds, stats, effective_max_iterations| {
            let mut attempt_config = config.clone();
            attempt_config.max_iterations = effective_max_iterations;
            let config = &attempt_config;
            let resolved_bounds = bounds.unwrap_or(RoutingBounds {
                min_x: 0,
                max_x: obstacle_map.width() - 1,
                min_y: 0,
                max_y: obstacle_map.height() - 1,
            });
            let context = CrossingHookContext::build(
                obstacle_map,
                primitives,
                config,
                crossing,
                resolved_bounds,
            );
            let hook = context.hook(
                obstacle_map,
                crossing,
                config,
                primitives.grid_size_um(),
                Some(&reservation_anchor_open_cells),
                Some(&anchor_open_cells),
            );
            route_single_net_with_bounds_unified(
                obstacle_map,
                primitives,
                source,
                target,
                Some(&anchor_open_cells),
                config,
                bounds,
                stats,
                dynamic_expansion_radius_cells,
                dynamic_clearance_exempt_cells,
                &hook,
            )
        },
    )
    .map(|route| with_route_search_total_time(route, route_search_total_start.as_ref()));
    (result, stats)
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn route_single_net_with_unified_kernel_crossing_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    dynamic_expansion_radius_cells: i32,
    dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
    crossing: &CrossingSearchConfig,
) -> Option<RouteResult> {
    if crossing.partners.is_empty()
        || (crossing.require_all_partners && crossing.partners.len() >= u64::BITS as usize)
    {
        return None;
    }
    if config.target_tolerance_cells < 0 {
        return None;
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        if mask == 0 {
            return None;
        }
    }
    if target.angle > 7 {
        return None;
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        return None;
    }

    let mut anchor_open_cells = FxHashSet::default();
    if let Some(port_open_cells) = port_open_cells {
        anchor_open_cells.extend(port_open_cells.iter().copied());
    }
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));

    let mut stats = RouteSearchStats::default();
    let route_search_total_start = if config.collect_detailed_timing {
        Some(Instant::now())
    } else {
        None
    };
    stats.jps4_requested = config.enable_jps4;
    stats.jps4_eligible = false;
    stats.jps4_fallback_reason = "unified kernel uses augmented A*".to_string();
    if config.enable_jps4 {
        stats.jps4_fallbacks += 1;
    }

    run_windowed_single_net_search(
        obstacle_map,
        source,
        target,
        config,
        &mut stats,
        |obstacle_map, bounds, stats, effective_max_iterations| {
            let mut attempt_config = config.clone();
            attempt_config.max_iterations = effective_max_iterations;
            let config = &attempt_config;
            let resolved_bounds = bounds.unwrap_or(RoutingBounds {
                min_x: 0,
                max_x: obstacle_map.width() - 1,
                min_y: 0,
                max_y: obstacle_map.height() - 1,
            });
            let context = CrossingHookContext::build(
                obstacle_map,
                primitives,
                config,
                crossing,
                resolved_bounds,
            );
            let hook = context.hook(
                obstacle_map,
                crossing,
                config,
                primitives.grid_size_um(),
                Some(&anchor_open_cells),
                Some(&anchor_open_cells),
            );
            route_single_net_with_bounds_unified(
                obstacle_map,
                primitives,
                source,
                target,
                Some(&anchor_open_cells),
                config,
                bounds,
                stats,
                dynamic_expansion_radius_cells,
                dynamic_clearance_exempt_cells,
                &hook,
            )
        },
    )
    .map(|route| with_route_search_total_time(route, route_search_total_start.as_ref()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::search;
    use crate::search::astar::crossing_rules::CrossingSearchPartner;
    use crate::search::test_support::*;

    fn crossing_search_fixture() -> (
        ObstacleMap,
        PrimitiveLibrary,
        State,
        State,
        AStarConfig,
        CrossingSearchConfig,
    ) {
        let mut map = ObstacleMap::new(20, 14);
        for x in 3..=13 {
            map.add_static_cell(x, 5);
            map.add_static_cell(x, 7);
        }
        let partner_cells: Vec<(i32, i32)> = (2..=10).map(|y| (8, y)).collect();
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            1,
            &partner_cells,
            &partner_cells,
            &[],
            &FxHashSet::default()
        ));

        let crossing = CrossingSearchConfig {
            diagnostics: KernelDiagnostics::default(),
            net_id: 2,
            partners: vec![CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(8, 2), (8, 10)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            }],
            min_straight_cells: 1,
            crossing_half_size_cells: 0,
            bend_runout_cells: 0,
            crossing_loss: 3.0,
            require_all_partners: false,
            terminal_bump_guard: None,
        };
        let config = AStarConfig {
            use_routing_window: false,
            enable_simple_routes: false,
            require_target_angle: false,
            ..AStarConfig::default()
        };

        (
            map,
            primitive_library_no45_bend1(),
            State::new(2, 6, 0),
            State::new(14, 6, 0),
            config,
            crossing,
        )
    }

    #[test]
    fn net_search_matches_base_free_function() {
        let map = ObstacleMap::new(12, 5);
        let library = primitive_library_no45_bend1();
        let source = State::new(1, 2, 0);
        let target = State::new(8, 2, 0);
        let config = AStarConfig {
            require_target_angle: true,
            ..AStarConfig::default()
        };
        let env = search::SearchEnvironment {
            obstacle_map: &map,
            primitives: &library,
        };
        let request = search::SearchRequest {
            source,
            target,
            port_open_cells: None,
            dynamic_expansion: None,
            crossing: None,
            config: &config,
        };
        let outcome = search::AStarSearch.search(&env, &request);
        let mut direct_stats = RouteSearchStats::default();
        let direct_result = route_single_net_with_unified_kernel_config_reporting_stats(
            &map,
            &library,
            source,
            target,
            None,
            &config,
            &mut direct_stats,
        );

        assert_route_results_equivalent(&outcome.route, &direct_result);
        assert_route_search_stats_equivalent(&outcome.stats, &direct_stats);
    }

    #[test]
    fn net_search_matches_dynamic_expansion_free_function() {
        let mut map = ObstacleMap::new(12, 5);
        assert!(map.commit_route_with_clearance_overlap(1, &[(4, 1)], &[(4, 1)], &[]));
        let library = primitive_library_no45_bend1();
        let source = State::new(1, 2, 0);
        let target = State::new(8, 2, 0);
        let config = AStarConfig {
            require_target_angle: true,
            ..AStarConfig::default()
        };
        let exempt_cells = pack_cells_for_test(&[(4, 1)]);
        let env = search::SearchEnvironment {
            obstacle_map: &map,
            primitives: &library,
        };
        let request = search::SearchRequest {
            source,
            target,
            port_open_cells: None,
            dynamic_expansion: Some(search::DynamicExpansion {
                radius_cells: 1,
                clearance_exempt_cells: Some(&exempt_cells),
            }),
            crossing: None,
            config: &config,
        };
        let outcome = search::AStarSearch.search(&env, &request);
        let mut direct_stats = RouteSearchStats::default();
        let direct_result =
            route_single_net_with_unified_kernel_dynamic_expansion_config_reporting_stats(
                &map,
                &library,
                source,
                target,
                None,
                &config,
                1,
                Some(&exempt_cells),
                &mut direct_stats,
            );

        assert_route_results_equivalent(&outcome.route, &direct_result);
        assert_route_search_stats_equivalent(&outcome.stats, &direct_stats);
    }

    #[test]
    fn net_search_matches_collision_crossing_free_function() {
        let (map, library, source, target, config, crossing) = crossing_search_fixture();
        // A present-but-empty reservation set (as opposed to `None`) is what
        // selects the collision-crossing dispatch branch -- see
        // `AStarSearch::search`'s doc comment.
        let reservation_open_cells: FxHashSet<CellKey> = FxHashSet::default();
        let env = search::SearchEnvironment {
            obstacle_map: &map,
            primitives: &library,
        };
        let request = search::SearchRequest {
            source,
            target,
            port_open_cells: None,
            dynamic_expansion: None,
            crossing: Some(search::CrossingSearch {
                config: &crossing,
                reservation_open_cells: Some(&reservation_open_cells),
            }),
            config: &config,
        };
        let outcome = search::AStarSearch.search(&env, &request);
        let (direct_result, direct_stats) =
            route_single_net_with_unified_kernel_collision_crossing_config_with_stats(
                &map,
                &library,
                source,
                target,
                None,
                Some(&reservation_open_cells),
                &config,
                0,
                None,
                &crossing,
            );

        assert_route_results_equivalent(&outcome.route, &direct_result);
        assert_route_search_stats_equivalent(&outcome.stats, &direct_stats);
    }

    #[test]
    fn net_search_matches_crossing_config_free_function() {
        let (map, library, source, target, config, crossing) = crossing_search_fixture();
        let env = search::SearchEnvironment {
            obstacle_map: &map,
            primitives: &library,
        };
        let request = search::SearchRequest {
            source,
            target,
            port_open_cells: None,
            dynamic_expansion: None,
            crossing: Some(search::CrossingSearch {
                config: &crossing,
                reservation_open_cells: None,
            }),
            config: &config,
        };
        let outcome = search::AStarSearch.search(&env, &request);
        let direct_result = route_single_net_with_unified_kernel_crossing_config(
            &map, &library, source, target, None, &config, 0, None, &crossing,
        );

        assert_route_results_equivalent(&outcome.route, &direct_result);
        // The crossing-config wrapper's own signature has never reported
        // search stats (`-> Option<RouteResult>`, no out-param, no tuple),
        // and `AStarSearch::search` cannot plumb real ones out of it without
        // changing that wrapper's body (forbidden this slice) -- see
        // `AStarSearch::search`'s doc comment. So there is nothing to assert
        // stats-equivalent against here, same as this test before this
        // slice.
    }

    /// Milestone 3, Slice 3's determinism pin: routes the four
    /// `net_search_matches_*` fixtures above and checks `RouteSearchStats`
    /// counters plus the route's own cost and waypoint count against
    /// literals recorded on commit 5a5edb8 (Milestone 3, Slice 2, the last
    /// commit before Slice 3's kernel-loop rewrite began), by temporarily
    /// stashing the Slice 3 working tree and re-running this fixture set
    /// on that commit. Kept passing unchanged through the whole rewrite,
    /// it is the independent check that the extraction never changed a
    /// comparison, a tie-break or a floating-point operation's order: the
    /// base fixture never even enters the unified kernel loop (it takes
    /// the simple-route shortcut, hence the all-zero search counters).
    #[test]
    fn determinism_pin_matches_commit_5a5edb8() {
        {
            let map = ObstacleMap::new(12, 5);
            let library = primitive_library_no45_bend1();
            let source = State::new(1, 2, 0);
            let target = State::new(8, 2, 0);
            let config = AStarConfig {
                require_target_angle: true,
                ..AStarConfig::default()
            };
            let env = search::SearchEnvironment {
                obstacle_map: &map,
                primitives: &library,
            };
            let request = search::SearchRequest {
                source,
                target,
                port_open_cells: None,
                dynamic_expansion: None,
                crossing: None,
                config: &config,
            };
            let outcome = search::AStarSearch.search(&env, &request);
            let route = outcome.route.expect("base fixture should route");
            assert_eq!(outcome.stats.expanded_states, 0, "base expanded_states");
            assert_eq!(
                outcome.stats.generated_neighbors, 0,
                "base generated_neighbors"
            );
            assert_eq!(outcome.stats.heap_pushes, 0, "base heap_pushes");
            assert_eq!(outcome.stats.heap_pops, 0, "base heap_pops");
            assert_eq!(route.total_cost, 7.0, "base total_cost");
            assert_eq!(route.compressed_waypoints.len(), 2, "base waypoint count");
        }
        {
            let mut map = ObstacleMap::new(12, 5);
            assert!(map.commit_route_with_clearance_overlap(1, &[(4, 1)], &[(4, 1)], &[]));
            let library = primitive_library_no45_bend1();
            let source = State::new(1, 2, 0);
            let target = State::new(8, 2, 0);
            let config = AStarConfig {
                require_target_angle: true,
                ..AStarConfig::default()
            };
            let exempt_cells = pack_cells_for_test(&[(4, 1)]);
            let env = search::SearchEnvironment {
                obstacle_map: &map,
                primitives: &library,
            };
            let request = search::SearchRequest {
                source,
                target,
                port_open_cells: None,
                dynamic_expansion: Some(search::DynamicExpansion {
                    radius_cells: 1,
                    clearance_exempt_cells: Some(&exempt_cells),
                }),
                crossing: None,
                config: &config,
            };
            let outcome = search::AStarSearch.search(&env, &request);
            let route = outcome
                .route
                .expect("dynamic expansion fixture should route");
            assert_eq!(outcome.stats.expanded_states, 24, "dynexp expanded_states");
            assert_eq!(
                outcome.stats.generated_neighbors, 96,
                "dynexp generated_neighbors"
            );
            assert_eq!(outcome.stats.heap_pushes, 46, "dynexp heap_pushes");
            assert_eq!(outcome.stats.heap_pops, 25, "dynexp heap_pops");
            assert_eq!(route.total_cost, 19.0, "dynexp total_cost");
            assert_eq!(route.compressed_waypoints.len(), 6, "dynexp waypoint count");
        }
        {
            let (map, library, source, target, config, crossing) = crossing_search_fixture();
            let reservation_open_cells: FxHashSet<CellKey> = FxHashSet::default();
            let env = search::SearchEnvironment {
                obstacle_map: &map,
                primitives: &library,
            };
            let request = search::SearchRequest {
                source,
                target,
                port_open_cells: None,
                dynamic_expansion: None,
                crossing: Some(search::CrossingSearch {
                    config: &crossing,
                    reservation_open_cells: Some(&reservation_open_cells),
                }),
                config: &config,
            };
            let outcome = search::AStarSearch.search(&env, &request);
            let route = outcome
                .route
                .expect("collision crossing fixture should route");
            assert_eq!(
                outcome.stats.expanded_states, 13,
                "collision expanded_states"
            );
            assert_eq!(
                outcome.stats.generated_neighbors, 52,
                "collision generated_neighbors"
            );
            assert_eq!(outcome.stats.heap_pushes, 19, "collision heap_pushes");
            assert_eq!(outcome.stats.heap_pops, 14, "collision heap_pops");
            assert_eq!(route.total_cost, 15.0, "collision total_cost");
            assert_eq!(
                route.compressed_waypoints.len(),
                2,
                "collision waypoint count"
            );
        }
        {
            // No stats to pin here: the crossing-config wrapper has never
            // reported real `RouteSearchStats` (see
            // `net_search_matches_crossing_config_free_function` above).
            let (map, library, source, target, config, crossing) = crossing_search_fixture();
            let env = search::SearchEnvironment {
                obstacle_map: &map,
                primitives: &library,
            };
            let request = search::SearchRequest {
                source,
                target,
                port_open_cells: None,
                dynamic_expansion: None,
                crossing: Some(search::CrossingSearch {
                    config: &crossing,
                    reservation_open_cells: None,
                }),
                config: &config,
            };
            let outcome = search::AStarSearch.search(&env, &request);
            let route = outcome.route.expect("crossing config fixture should route");
            assert_eq!(route.total_cost, 15.0, "crossing-config total_cost");
            assert_eq!(
                route.compressed_waypoints.len(),
                2,
                "crossing-config waypoint count"
            );
        }
    }

    /// Same fixture as `net_search_matches_crossing_config_free_function`,
    /// but proves the equivalence `AStarSearch::search`'s dispatch doc
    /// comment relies on: with `reservation_open_cells: None`, the
    /// collision-crossing wrapper (which *does* report stats) produces the
    /// exact same route as the crossing-config wrapper, because its
    /// `.or(port_open_cells)` fallback makes its reservation anchor set
    /// equal in content to the crossing-config wrapper's one anchor set.
    #[test]
    fn collision_crossing_wrapper_with_no_reservation_matches_crossing_config_wrapper() {
        let (map, library, source, target, config, crossing) = crossing_search_fixture();

        let (collision_result, _collision_stats) =
            route_single_net_with_unified_kernel_collision_crossing_config_with_stats(
                &map, &library, source, target, None, None, &config, 0, None, &crossing,
            );
        let crossing_config_result = route_single_net_with_unified_kernel_crossing_config(
            &map, &library, source, target, None, &config, 0, None, &crossing,
        );

        assert_route_results_equivalent(&collision_result, &crossing_config_result);
    }

    #[test]
    fn routes_straight_without_obstacles() {
        let map = ObstacleMap::new(10, 5);
        let result = route_single_net(
            &map,
            &primitive_library_no45_bend1(),
            State::new(1, 2, 0),
            State::new(5, 2, 0),
            None,
        )
        .expect("straight route should exist");

        assert_eq!(result.states.first().copied(), Some(State::new(1, 2, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(5, 2, 0)));
        assert_eq!(result.compressed_waypoints, vec![(1, 2), (5, 2)]);
        assert!(result.cells.contains(&(5, 2)));
        assert_eq!(result.total_length_um, 4.0);
    }

    #[test]
    fn routes_around_simple_block() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 3);
        map.add_static_cell(4, 3);
        map.add_static_cell(5, 3);

        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 3, 0),
            State::new(8, 3, 0),
            None,
        )
        .expect("route around obstacle should exist");

        assert_eq!(result.states.last().copied(), Some(State::new(8, 3, 0)));
        assert!(!result
            .cells
            .iter()
            .any(|&(x, y)| map.is_static_blocked(x, y)));
    }

    #[test]
    fn routes_with_ninety_degree_turn() {
        let map = ObstacleMap::new(8, 8);
        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(3, 3, 2),
            None,
        )
        .expect("90 degree turn route should exist");

        assert_eq!(result.states.last().copied(), Some(State::new(3, 3, 2)));
        assert!(result.states.iter().any(|state| state.angle == 2));
        assert!(result.compressed_waypoints.len() >= 2);
    }

    #[test]
    fn respects_exact_angle_constraints() {
        let map = ObstacleMap::new(8, 8);
        let library = primitive_library();

        assert!(route_single_net(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
        )
        .is_some());
        let north_arrival = route_single_net(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 2),
            None,
        )
        .expect("same coordinate with a different exact target angle should be routable");
        assert_eq!(
            north_arrival.states.last().copied(),
            Some(State::new(5, 1, 2))
        );
    }

    #[test]
    fn reporting_stats_preserves_failed_dense_search_effort() {
        let mut map = ObstacleMap::new(12, 5);
        for y in 0..map.height() {
            map.add_static_cell(5, y);
        }
        let mut stats = RouteSearchStats::default();

        let result = test_route_single_net_with_config_reporting_stats(
            &map,
            &primitive_library_no45_bend1(),
            State::new(1, 2, 0),
            State::new(10, 2, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                require_target_angle: false,
                use_routing_window: false,
                max_iterations: 10_000,
                ..AStarConfig::default()
            },
            &mut stats,
        );

        assert!(result.is_none());
        assert!(stats.expanded_states > 0);
    }

    #[test]
    fn dynamic_reporting_stats_preserves_failed_dense_search_effort() {
        let mut map = ObstacleMap::new(12, 5);
        for y in 0..map.height() {
            map.add_static_cell(5, y);
        }
        let mut stats = RouteSearchStats::default();

        let result = test_route_single_net_with_dynamic_expansion_config_reporting_stats(
            &map,
            &primitive_library_no45_bend1(),
            State::new(1, 2, 0),
            State::new(10, 2, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                require_target_angle: false,
                use_routing_window: false,
                max_iterations: 10_000,
                ..AStarConfig::default()
            },
            1,
            None,
            &mut stats,
        );

        assert!(result.is_none());
        assert!(stats.expanded_states > 0);
    }

    #[test]
    fn public_wrappers_preserve_successful_routes() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        let library = primitive_library();
        let source = State::new(1, 1, 0);
        let target = State::new(5, 1, 0);
        let config = AStarConfig {
            enable_simple_routes: true,
            ..AStarConfig::default()
        };

        let public_route =
            route_single_net_with_config(&map, &library, source, target, None, &config)
                .expect("public wrapper should route");
        let mut reporting_stats = RouteSearchStats::default();
        let reporting_route = test_route_single_net_with_config_reporting_stats(
            &map,
            &library,
            source,
            target,
            None,
            &config,
            &mut reporting_stats,
        )
        .expect("reporting variant should route");

        assert_eq!(reporting_route.cells, public_route.cells);
        assert_eq!(reporting_route.states, public_route.states);
        assert_eq!(reporting_route.primitives, public_route.primitives);
        assert!((reporting_route.total_cost - public_route.total_cost).abs() < 1.0e-9);

        let mut dynamic_stats = RouteSearchStats::default();
        let public_dynamic = test_route_single_net_with_dynamic_expansion_config(
            &map, &library, source, target, None, &config, 0, None,
        )
        .expect("dynamic public wrapper should route");
        let reporting_dynamic =
            test_route_single_net_with_dynamic_expansion_config_reporting_stats(
                &map,
                &library,
                source,
                target,
                None,
                &config,
                0,
                None,
                &mut dynamic_stats,
            )
            .expect("dynamic reporting variant should route");

        assert_eq!(reporting_dynamic.cells, public_dynamic.cells);
        assert_eq!(reporting_dynamic.states, public_dynamic.states);
        assert_eq!(reporting_dynamic.primitives, public_dynamic.primitives);
        assert!((reporting_dynamic.total_cost - public_dynamic.total_cost).abs() < 1.0e-9);
    }

    #[test]
    fn port_opening_allows_blocked_source_and_target_cells() {
        let mut map = ObstacleMap::new(8, 3);
        map.add_static_cell(1, 1);
        map.add_static_cell(5, 1);

        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(1, 1));
        opened.insert(pack_xy(5, 1));

        let result = route_single_net(
            &map,
            &primitive_library_no45_bend1(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            Some(&opened),
        )
        .expect("opened port cells should be routable");

        assert_eq!(result.states.last().copied(), Some(State::new(5, 1, 0)));
    }

    #[test]
    fn supports_coordinate_tolerance() {
        let map = ObstacleMap::new(12, 6);
        let library = primitive_library_no45_bend2();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(5, 3, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                target_tolerance_cells: 1,
                ..AStarConfig::default()
            },
        )
        .expect("route should terminate within tolerance");

        let reached = result.states.last().copied().unwrap();
        assert!((reached.x - 5).abs() <= 1);
        assert!((reached.y - 3).abs() <= 1);
        assert_eq!(result.reached_target, reached);
    }

    #[test]
    fn supports_relaxed_target_angle() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 2),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                require_target_angle: false,
                ..AStarConfig::default()
            },
        )
        .expect("route should allow non-matching terminal angle");

        let reached = result.states.last().copied().unwrap();
        assert_eq!(reached.x, 5);
        assert_eq!(reached.y, 1);
        assert_ne!(reached.angle, 2);
    }

    #[test]
    fn supports_allowed_target_angle_mask() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 2),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                require_target_angle: true,
                allowed_target_angles_mask: Some((1u8 << 0) | (1u8 << 1)),
                ..AStarConfig::default()
            },
        )
        .expect("route should use allowed-angle mask override");

        let reached = result.states.last().copied().unwrap();
        assert!(((1u8 << reached.angle) & ((1u8 << 0) | (1u8 << 1))) != 0);
    }

    #[test]
    fn rejects_negative_tolerance() {
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
                target_tolerance_cells: -1,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }
}
