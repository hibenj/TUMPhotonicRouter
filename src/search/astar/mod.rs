//! The A* search engine: `AStarSearch` (the `NetSearch` implementation),
//! its two stable free-function conveniences (`route_single_net`,
//! `route_single_net_with_config`), and `run_search`, the one search body
//! `AStarSearch::search` is a straight call into. The submodules it uses:
//! `config` (types and constants), `heuristic`, `cost`, `expansion`,
//! `crossing_rules`, `window`, `dense`, `kernel`, `simple`, `svg`,
//! `diagnostics`; `state` and `geometry` live one level up in
//! `crate::search` since they are shared with a future second engine.
//!
//! Moved out of `src/astar.rs` (Milestone 3, Slice 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
//! continuing Slice 1's `src/search/astar_engine.rs`); the four
//! unified-kernel wrappers it carried (plain, dynamic-expansion,
//! collision-crossing, crossing-config) were unified into `run_search` in
//! Slice 3, Step 6, behaviour-preserving.

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
use crossing_rules::{CrossingHookContext, NoCrossingHook};
use dense::{evaluate_jps4_eligibility, route_single_net_jps4};
use simple::try_simple_route_with_config;
use window::{run_windowed_single_net_search, RoutingBounds};

use crate::search::state::{RouteResult, RouteSearchStats, State};
use rustc_hash::FxHashSet;
use std::time::Instant;

use super::{anchor_open_cells, NetSearch, SearchEnvironment, SearchOutcome, SearchRequest};

/// The kernel today's production callers have always used, now reached
/// through `NetSearch` instead of the deleted four-method single-net-search
/// trait.
pub struct AStarSearch;

impl NetSearch for AStarSearch {
    /// One call into `run_search`, the single body the four old
    /// unified-kernel wrappers collapsed into (Milestone 3, Slice 3,
    /// Step 6).
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome {
        run_search(env, request)
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

/// A rejected request: no route, and the all-zero counters every entry
/// guard in `run_search` has always reported.
fn rejected_outcome() -> SearchOutcome {
    SearchOutcome {
        route: None,
        stats: RouteSearchStats::default(),
    }
}

/// The one single-net A* search body. It replaces the four unified-kernel
/// wrappers this module used to carry (plain / dynamic-expansion /
/// collision-crossing / crossing-config), reproducing each of their paths
/// exactly; which path runs is read off the request alone:
///
/// * the crossing-partner entry guard runs only for `crossing: Some(_)`;
///   the four config/geometry guards run for every request, with the same
///   message-free `None` results as before;
/// * the JPS4 and simple-route shortcuts are the *plain* path's alone
///   (`crossing: None && dynamic_expansion: None`) -- exactly the wrapper
///   that had them; every other path goes straight to the windowed kernel
///   with the fixed `jps4_*` stats its wrapper set ("unified kernel uses
///   dense A*" without crossing, "unified kernel uses augmented A*" with);
/// * the separate *reservation* anchor set exists only when the crossing
///   part carries `reservation_open_cells: Some(_)` -- the old
///   collision-crossing wrapper's case. With `None` the old crossing-config
///   wrapper passed its single `port_open_cells`-derived anchor set for both
///   hook slots, which is what `unwrap_or(&anchor_open_cells)` does here;
///   Slice 1 proved that equals the collision wrapper's
///   `reservation_open_cells.or(port_open_cells)` fallback, and
///   `collision_crossing_with_no_reservation_matches_crossing_config` below
///   still pins it;
/// * the hook is `NoCrossingHook` without a crossing part and the
///   `CrossingHookContext::build(..).hook(..)` live hook with one.
///
/// Stats are now returned on every path, including the crossing-config one
/// whose old wrapper's `-> Option<RouteResult>` signature dropped them.
/// That is not a behaviour change for any caller -- none reads the stats of
/// that path (`src/engine/search_calls.rs` discards them) -- and the
/// counters are the ones the proven-equivalent collision path reports.
fn run_search(env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome {
    let obstacle_map = env.obstacle_map;
    let primitives = env.primitives;
    let source = request.source;
    let target = request.target;
    let port_open_cells = request.port_open_cells;
    let config = request.config;
    let crossing = request.crossing.as_ref().map(|crossing| crossing.config);

    if let Some(crossing) = crossing {
        if crossing.partners.is_empty()
            || (crossing.require_all_partners && crossing.partners.len() >= u64::BITS as usize)
        {
            return rejected_outcome();
        }
    }
    if config.target_tolerance_cells < 0 {
        return rejected_outcome();
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        if mask == 0 {
            return rejected_outcome();
        }
    }
    if target.angle > 7 {
        return rejected_outcome();
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        return rejected_outcome();
    }

    let anchor_open_cells = anchor_open_cells(source, target, port_open_cells);

    let reservation_anchor_open_cells = request
        .crossing
        .as_ref()
        .and_then(|crossing| crossing.reservation_open_cells)
        .map(|reservation_open_cells| {
            let mut cells: FxHashSet<CellKey> = FxHashSet::default();
            cells.extend(reservation_open_cells.iter().copied());
            cells.insert(pack_xy(source.x, source.y));
            cells.insert(pack_xy(target.x, target.y));
            cells
        });

    let dynamic_expansion_radius_cells = request
        .dynamic_expansion
        .as_ref()
        .map_or(0, |dynamic| dynamic.radius_cells);
    let dynamic_clearance_exempt_cells = request
        .dynamic_expansion
        .as_ref()
        .and_then(|dynamic| dynamic.clearance_exempt_cells);

    let mut stats = RouteSearchStats::default();
    let route_search_total_start = if config.collect_detailed_timing {
        Some(Instant::now())
    } else {
        None
    };

    let takes_shortcuts = request.crossing.is_none() && request.dynamic_expansion.is_none();
    if takes_shortcuts {
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
                return SearchOutcome {
                    route: Some(with_route_search_total_time(
                        route,
                        route_search_total_start.as_ref(),
                    )),
                    stats,
                };
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
            return SearchOutcome {
                route: Some(with_route_search_total_time(
                    simple_route,
                    route_search_total_start.as_ref(),
                )),
                stats,
            };
        }
    } else {
        stats.jps4_requested = config.enable_jps4;
        stats.jps4_eligible = false;
        stats.jps4_fallback_reason = if crossing.is_some() {
            "unified kernel uses augmented A*".to_string()
        } else {
            "unified kernel uses dense A*".to_string()
        };
        if config.enable_jps4 {
            stats.jps4_fallbacks += 1;
        }
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
            let config = &attempt_config;
            match crossing {
                Some(crossing) => {
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
                        Some(
                            reservation_anchor_open_cells
                                .as_ref()
                                .unwrap_or(&anchor_open_cells),
                        ),
                        Some(&anchor_open_cells),
                    );
                    kernel::run(
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
                }
                None => kernel::run(
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
                    &NoCrossingHook,
                ),
            }
        },
    )
    .map(|route| with_route_search_total_time(route, route_search_total_start.as_ref()));

    SearchOutcome { route, stats }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::search;
    use crate::search::test_support::*;

    /// The plain request (no crossing part, no dynamic expansion) is the
    /// only one `run_search` lets take the JPS4/simple-route shortcuts, so
    /// its `jps4_*` stats come from `evaluate_jps4_eligibility` rather than
    /// from one of the two fixed unified-kernel reasons.
    #[test]
    fn net_search_plain_request_takes_the_shortcut_path() {
        let (map, library, source, target, config) = plain_request_fixture();
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
        let eligibility = evaluate_jps4_eligibility(&library, source, target, &config);

        assert!(outcome.route.is_some(), "plain request should route");
        assert_eq!(outcome.stats.jps4_requested, config.enable_jps4);
        assert_eq!(outcome.stats.jps4_eligible, eligibility.eligible);
        assert_eq!(outcome.stats.jps4_fallback_reason, eligibility.reason);
    }

    /// A dynamic-expansion request forgoes the shortcuts and goes straight
    /// to the windowed dense kernel with `NoCrossingHook`, which
    /// `run_search` records in the stats as a fixed fallback reason.
    #[test]
    fn net_search_dynamic_expansion_request_takes_the_dense_kernel_path() {
        let (map, library, source, target, config, exempt_cells) =
            dynamic_expansion_request_fixture();
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

        assert!(
            outcome.route.is_some(),
            "dynamic expansion request should route"
        );
        assert!(!outcome.stats.jps4_eligible);
        assert_eq!(
            outcome.stats.jps4_fallback_reason,
            "unified kernel uses dense A*"
        );
        assert!(
            outcome.stats.expanded_states > 0,
            "dynamic expansion request should enter the kernel loop"
        );
    }

    /// A crossing request carrying its own reservation anchor set: the
    /// augmented (live crossing hook) kernel path, with the reservation
    /// anchor slot built from `reservation_open_cells` rather than from
    /// `port_open_cells`.
    #[test]
    fn net_search_collision_crossing_request_takes_the_augmented_kernel_path() {
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

        assert!(
            outcome.route.is_some(),
            "collision crossing request should route"
        );
        assert!(!outcome.stats.jps4_eligible);
        assert_eq!(
            outcome.stats.jps4_fallback_reason,
            "unified kernel uses augmented A*"
        );
        assert!(
            outcome.stats.expanded_states > 0,
            "collision crossing request should enter the kernel loop"
        );
    }

    /// A crossing request without a reservation anchor set: the same
    /// augmented kernel path, with `port_open_cells` (here: none, so just
    /// the source and target cells) filling both of the hook's anchor
    /// slots. Unlike the old crossing-config wrapper, `run_search` reports
    /// this path's real counters.
    #[test]
    fn net_search_crossing_config_request_takes_the_augmented_kernel_path() {
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

        assert!(
            outcome.route.is_some(),
            "crossing config request should route"
        );
        assert!(!outcome.stats.jps4_eligible);
        assert_eq!(
            outcome.stats.jps4_fallback_reason,
            "unified kernel uses augmented A*"
        );
        assert!(
            outcome.stats.expanded_states > 0,
            "crossing config request should enter the kernel loop"
        );
    }

    /// Milestone 3, Slice 3's determinism pin: routes the four
    /// `net_search_*_request_*` fixtures above and checks `RouteSearchStats`
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
            let (map, library, source, target, config) = plain_request_fixture();
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
            let (map, library, source, target, config, exempt_cells) =
                dynamic_expansion_request_fixture();
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
            // Stats pinned here since Slice 3 Step 6: the old
            // crossing-config wrapper's `-> Option<RouteResult>` signature
            // dropped them, so this block asserted only cost and waypoints
            // before. The counters are the ones the proven-equivalent
            // collision path reported on HEAD (511d91a) for this very
            // fixture with `reservation_open_cells: None`, measured before
            // the wrappers were unified -- identical, as the equivalence
            // predicts, to the collision block's above.
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
            assert_eq!(
                outcome.stats.expanded_states, 13,
                "crossing-config expanded_states"
            );
            assert_eq!(
                outcome.stats.generated_neighbors, 52,
                "crossing-config generated_neighbors"
            );
            assert_eq!(outcome.stats.heap_pushes, 19, "crossing-config heap_pushes");
            assert_eq!(outcome.stats.heap_pops, 14, "crossing-config heap_pops");
            assert_eq!(route.total_cost, 15.0, "crossing-config total_cost");
            assert_eq!(
                route.compressed_waypoints.len(),
                2,
                "crossing-config waypoint count"
            );
        }
    }

    /// Same fixture as the two crossing tests above, and the reason
    /// `run_search` needs no compatibility flag to tell the old
    /// collision-crossing and crossing-config wrappers apart: an absent
    /// `reservation_open_cells` (the old crossing-config wrapper, whose one
    /// `port_open_cells`-derived anchor set filled both hook slots) gives
    /// the same route *and* the same counters as a reservation anchor set
    /// built from the same cells the port anchor set is built from (the old
    /// collision wrapper's `reservation_open_cells.or(port_open_cells)`
    /// fallback). `port_open_cells` is `None` in this fixture, so the
    /// equal-content set is the empty one.
    #[test]
    fn crossing_reservation_none_matches_an_equal_reservation_anchor_set() {
        let (map, library, source, target, config, crossing) = crossing_search_fixture();
        let env = search::SearchEnvironment {
            obstacle_map: &map,
            primitives: &library,
        };
        let equal_reservation_open_cells: FxHashSet<CellKey> = FxHashSet::default();
        let with_reservation = search::AStarSearch.search(
            &env,
            &search::SearchRequest {
                source,
                target,
                port_open_cells: None,
                dynamic_expansion: None,
                crossing: Some(search::CrossingSearch {
                    config: &crossing,
                    reservation_open_cells: Some(&equal_reservation_open_cells),
                }),
                config: &config,
            },
        );
        let without_reservation = search::AStarSearch.search(
            &env,
            &search::SearchRequest {
                source,
                target,
                port_open_cells: None,
                dynamic_expansion: None,
                crossing: Some(search::CrossingSearch {
                    config: &crossing,
                    reservation_open_cells: None,
                }),
                config: &config,
            },
        );

        assert_route_results_equivalent(&with_reservation.route, &without_reservation.route);
        assert_route_search_stats_equivalent(&with_reservation.stats, &without_reservation.stats);
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
