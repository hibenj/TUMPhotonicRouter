//! The second `NetSearch` implementation: `GridDijkstraSearch`, a
//! uniform-cost (Dijkstra) search over the same `State` space, with the
//! same primitive moves, the same legality rules and the same step costs
//! as the A* engine -- because it runs them, by calling the A* kernel's own
//! building blocks (`astar::kernel::{build_search_grid, seed_open_set,
//! OpenSet, reconstruct_route_unified}`, `astar::expansion::{PrimitiveTables,
//! candidate_moves, move_is_legal, goal_reached, push_or_improve_dense}`,
//! and through those `astar::cost::step_cost`). Nothing here is a copy of
//! them.
//!
//! What it deliberately does *not* have, and why each is what makes it an
//! oracle rather than a second production engine:
//!
//! * **No heuristic.** `SearchHeuristic::zero` makes `f = g`, so states
//!   come off the open set in non-decreasing g order: the first time the
//!   goal test accepts a popped state, its g-cost is the cheapest any legal
//!   path to it can have under these same step costs. That is the property
//!   the A* engine's route cost is compared against in the tests below.
//! * **No routing window.** One attempt over the full grid bounds, so no
//!   route is ever lost to a window that was too small.
//! * **No crossing support.** The hook is `NoCrossingHook`, so `move_is_legal`
//!   can only ever return `Tier1` or `Illegal` and the kernel's Tier-2
//!   (crossing) machinery is unreachable. A `SearchRequest` carrying a
//!   `crossing` part is refused outright, with
//!   `RouteSearchStats::unsupported_request = 1`.
//! * **No JPS4 or simple-route shortcut.** Every request runs the real
//!   search, so a fixture's cost is the search's own answer, not a
//!   shortcut's.
//!
//! Dynamic expansion *is* supported: its radius and clearance-exempt cells
//! are arguments of the shared `build_search_grid`, so they cost nothing
//! extra here.
//!
//! Milestone 3, Slice 4 (Decision Log D5) of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.

use crate::obstacle_map::ObstacleMap;
use crate::primitives::PrimitiveLibrary;
use crate::search::astar::config::AStarConfig;
use crate::search::astar::crossing_rules::{CrossingExtension, NoCrossingHook};
use crate::search::astar::expansion::{
    candidate_moves, goal_reached, move_is_legal, push_or_improve_dense, MoveContext,
    MoveDiagnostics, MoveVerdict, PrimitiveTables,
};
use crate::search::astar::heuristic::{target_angle_acceptance, SearchHeuristic};
use crate::search::astar::kernel::{
    build_search_grid, reconstruct_route_unified, seed_open_set, OpenSet, UnifiedOpenRef,
};
use crate::search::astar::window::{effective_max_iterations, RoutingBounds};
use crate::search::state::{RouteResult, RouteSearchStats};
use crate::search::{
    anchor_open_cells, NetSearch, SearchEnvironment, SearchOutcome, SearchRequest,
};
use rustc_hash::FxHashMap;

/// The uniform-cost oracle engine. Selected by
/// `RouterConfig::search.engine == "grid-dijkstra"`.
pub struct GridDijkstraSearch;

impl NetSearch for GridDijkstraSearch {
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome {
        if request.crossing.is_some() {
            // Not a search failure: this engine has no crossing hook at
            // all, so the request was never searched.
            return SearchOutcome {
                route: None,
                stats: RouteSearchStats {
                    unsupported_request: 1,
                    ..RouteSearchStats::default()
                },
            };
        }
        let mut stats = RouteSearchStats::default();
        let route = run_dijkstra(env, request, &mut stats);
        SearchOutcome { route, stats }
    }
}

/// The search loop: build the shared dense grid over the full map, seed the
/// source, then pop the cheapest open state, stop on the kernel's own goal
/// test, and expand it with the kernel's own candidate moves, legality
/// rules and insert-or-improve step. `None` is returned for every failure
/// (infeasible, iteration cap or expansion budget reached, or a grid the
/// dense storage cap refuses).
fn run_dijkstra(
    env: &SearchEnvironment,
    request: &SearchRequest,
    stats: &mut RouteSearchStats,
) -> Option<RouteResult> {
    let obstacle_map: &ObstacleMap = env.obstacle_map;
    let primitives: &PrimitiveLibrary = env.primitives;
    let source = request.source;
    let target = request.target;
    let config: &AStarConfig = request.config;

    let bounds = RoutingBounds {
        min_x: 0,
        max_x: obstacle_map.width() - 1,
        min_y: 0,
        max_y: obstacle_map.height() - 1,
    };
    if !bounds.contains(source.x, source.y) || !bounds.contains(target.x, target.y) {
        return None;
    }

    let hook = NoCrossingHook;
    let anchor_open_cells = anchor_open_cells(source, target, request.port_open_cells);
    let dynamic_expansion_radius_cells = request
        .dynamic_expansion
        .as_ref()
        .map_or(0, |dynamic| dynamic.radius_cells);
    let dynamic_clearance_exempt_cells = request
        .dynamic_expansion
        .as_ref()
        .and_then(|dynamic| dynamic.clearance_exempt_cells);

    let (mut storage, dense_grid, _ignores_dynamic_obstacles) = build_search_grid(
        obstacle_map,
        bounds,
        Some(&anchor_open_cells),
        config,
        dynamic_expansion_radius_cells,
        dynamic_clearance_exempt_cells,
        false,
        source,
        target,
        stats,
    )?;

    let heuristic = SearchHeuristic::zero(target, primitives);
    let tables = PrimitiveTables::build(primitives, config.bend_weight);
    let accepted_target_angles = target_angle_acceptance(target, config);
    let move_context = MoveContext {
        dense_grid: &dense_grid,
        config,
        hook: &hook,
        bounds,
        source,
        target,
        target_tolerance: config.target_tolerance_cells.max(0),
        accepted_target_angles: &accepted_target_angles,
    };

    // `NoCrossingHook::TRACKS_STRAIGHT_RUN` is `false`, so the shared
    // insert-or-improve step never indexes this array -- same empty vector
    // the A* kernel passes with crossings disabled.
    let mut dense_straight_run: Vec<i32> = Vec::new();
    // The diagnostics the shared legality/insert helpers write into. This
    // engine runs none of the kernel's diagnostic reports, so the counters
    // are allocated once and never read.
    let probe_index: FxHashMap<(i32, i32), usize> = FxHashMap::default();
    let mut target_ring = [[0u32; 7]; 25];
    let mut probe_ring: Vec<[u32; 7]> = Vec::new();
    let mut ring_blocker_lines: FxHashMap<usize, u32> = FxHashMap::default();

    let mut counter = 0u32;
    let mut open = OpenSet::new(config.use_indexed_heap, storage.state_count());
    let source_idx = seed_open_set(
        &mut storage,
        &mut open,
        source,
        &heuristic,
        &hook,
        config,
        stats,
        &mut counter,
    )?;

    // One full-grid attempt, capped exactly like the A* engine's own
    // full-grid attempt: `config.max_iterations`, narrowed by whatever
    // `config.total_expansion_budget` still allows.
    let max_iterations = effective_max_iterations(config, stats);
    let mut iterations = 0usize;

    while let Some(entry) = open.pop() {
        stats.heap_pops += 1;
        iterations += 1;
        if iterations > max_iterations {
            return None;
        }
        let idx = entry.idx;
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
        let state = storage.idx_to_state(idx);
        let current_g = storage.g_costs[idx];
        // Every state this engine reaches is crossing-free by construction.
        let current_extension = CrossingExtension::default();

        if goal_reached(&move_context, state, current_extension) {
            return reconstruct_route_unified(
                source_idx,
                UnifiedOpenRef::Dense(idx),
                current_g,
                target,
                primitives,
                stats.clone(),
                &storage,
                &[],
            );
        }

        storage.closed.set(idx)?;
        stats.expanded_states += 1;

        let angle = state.angle as usize;
        for candidate in candidate_moves(
            tables.buckets[angle],
            &tables.metadata[angle],
            &tables.profiles[angle],
            &tables.crossings[angle],
            state,
            target,
            primitives.grid_size_um(),
            config.primitive_ordering,
        ) {
            stats.generated_neighbors += 1;
            stats.primitive_generated_by_class[candidate.class] += 1;
            let mut diagnostics = MoveDiagnostics {
                failure_diag: false,
                move_diag_cell: None,
                search_seq: 0,
                obstacle_map,
                port_open_cells: request.port_open_cells,
                target,
                probe_index: &probe_index,
                target_ring: &mut target_ring,
                probe_ring: &mut probe_ring,
                ring_blocker_lines: &mut ring_blocker_lines,
            };
            let mut legality_time_us = 0u128;
            let legal = match move_is_legal(
                &move_context,
                state,
                current_extension,
                &candidate,
                &mut diagnostics,
                stats,
                &mut legality_time_us,
            )? {
                MoveVerdict::Illegal(_) => continue,
                MoveVerdict::Tier1(legal) => legal,
                MoveVerdict::Tier2(..) => unreachable!(
                    "NoCrossingHook::evaluate always returns None, so no move can be \
                     legalized into the kernel's crossing tier"
                ),
            };
            let mut heap_time_us = 0u128;
            push_or_improve_dense(
                &move_context,
                &heuristic,
                &mut storage,
                &mut open,
                &mut dense_straight_run,
                &mut counter,
                state,
                current_extension,
                current_g,
                idx,
                &candidate,
                &legal,
                &mut diagnostics,
                stats,
                &mut heap_time_us,
            )?;
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bindings::PyState;
    use crate::engine::test_support::{small_test_router_with_router_config, test_router_config};
    use crate::obstacle_map::CellKey;
    use crate::search;
    use crate::search::test_support::{
        crossing_search_fixture, dynamic_expansion_request_fixture, plain_request_fixture,
    };
    use pyo3::prelude::*;
    use pyo3::types::PyDict;
    use rustc_hash::FxHashSet;

    /// Routes one request with both engines and returns `(astar_cost,
    /// dijkstra_cost)`; both must find a route.
    fn both_engine_costs(env: &SearchEnvironment, request: &SearchRequest) -> (f64, f64) {
        let astar = search::AStarSearch
            .search(env, request)
            .route
            .expect("A* should route this fixture");
        let dijkstra = GridDijkstraSearch
            .search(env, request)
            .route
            .expect("the Dijkstra engine should route this fixture");
        (astar.total_cost, dijkstra.total_cost)
    }

    /// Oracle check on the plain fixture of
    /// `net_search_plain_request_takes_the_shortcut_path`. A* answers this
    /// one from the simple-route shortcut, the Dijkstra runs the real
    /// uniform-cost search; both reach the same cost.
    #[test]
    fn dijkstra_matches_the_astar_cost_on_the_plain_fixture() {
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
        let (astar_cost, dijkstra_cost) = both_engine_costs(&env, &request);
        assert_eq!(dijkstra_cost, astar_cost, "plain fixture route cost");
    }

    /// Oracle check on the dynamic-expansion fixture of
    /// `net_search_dynamic_expansion_request_takes_the_dense_kernel_path`:
    /// the widened halo and the clearance-exempt cell reach the Dijkstra
    /// through the same `build_search_grid` call the A* kernel makes.
    #[test]
    fn dijkstra_matches_the_astar_cost_on_the_dynamic_expansion_fixture() {
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
        let (astar_cost, dijkstra_cost) = both_engine_costs(&env, &request);
        assert_eq!(
            dijkstra_cost, astar_cost,
            "dynamic expansion fixture route cost"
        );
    }

    /// A crossing request is not a search this engine can run: no route,
    /// and the marker that says so rather than "search failed".
    #[test]
    fn a_crossing_request_is_reported_unsupported() {
        let (map, library, source, target, config, crossing) = crossing_search_fixture();
        let reservation_open_cells: FxHashSet<CellKey> = FxHashSet::default();
        let env = search::SearchEnvironment {
            obstacle_map: &map,
            primitives: &library,
        };
        for reservation in [None, Some(&reservation_open_cells)] {
            let request = search::SearchRequest {
                source,
                target,
                port_open_cells: None,
                dynamic_expansion: None,
                crossing: Some(search::CrossingSearch {
                    config: &crossing,
                    reservation_open_cells: reservation,
                }),
                config: &config,
            };
            let outcome = GridDijkstraSearch.search(&env, &request);
            assert!(outcome.route.is_none(), "no route for a crossing request");
            assert_eq!(outcome.stats.unsupported_request, 1);
            assert_eq!(
                outcome.stats.expanded_states, 0,
                "the request must not have been searched at all"
            );
        }
        // The A* engine does route this same fixture, so the refusal is
        // this engine's own limit, not the fixture's.
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
        let astar = search::AStarSearch.search(&env, &request);
        assert!(astar.route.is_some());
        assert_eq!(astar.stats.unsupported_request, 0);
    }

    /// The seam end to end: a router built with `search_engine =
    /// "grid-dijkstra"` routes a two-net job list through the negotiated
    /// repair loop, so the second engine is reached by the production call
    /// path and not only by a direct `NetSearch::search` call.
    #[test]
    fn the_dijkstra_engine_routes_a_two_net_job_list_through_the_negotiated_loop() {
        pyo3::prepare_freethreaded_python();
        let router_config =
            test_router_config(crate::config::SEARCH_ENGINE_GRID_DIJKSTRA).expect("valid engine");
        let mut router = small_test_router_with_router_config(Some(router_config));
        let jobs = vec![
            (
                1u64,
                PyState::new(1, 2, 0),
                PyState::new(17, 2, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
            (
                2u64,
                PyState::new(1, 12, 0),
                PyState::new(17, 12, 0),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
                None,
            ),
        ];
        Python::with_gil(|py| {
            let result = router
                .route_many_with_negotiated_repair_and_commit_impl(
                    py, jobs, 0, None, None, 4, 2.0, 1,
                )
                .expect("the negotiated loop must not raise");
            let dict = result.bind(py).downcast::<PyDict>().unwrap().clone();
            let status: String = dict.get_item("status").unwrap().unwrap().extract().unwrap();
            assert_eq!(status, "routed", "both nets must route");
            let routes = dict.get_item("routes").unwrap().unwrap();
            assert_eq!(routes.len().unwrap(), 2, "one committed route per net");
        });
    }

    /// The name-to-engine mapping `PyPhotonicRouter::construct` uses, and
    /// the binding's rejection of anything else.
    #[test]
    fn the_engine_name_selects_the_engine_and_rejects_unknown_names() {
        pyo3::prepare_freethreaded_python();
        assert!(test_router_config(crate::config::SEARCH_ENGINE_ASTAR).is_ok());
        assert!(test_router_config(crate::config::SEARCH_ENGINE_GRID_DIJKSTRA).is_ok());
        let Err(err) = test_router_config("nonsense") else {
            panic!("an unknown engine must be rejected");
        };
        let message = err.to_string();
        assert!(
            message.contains(crate::config::SEARCH_ENGINE_ASTAR),
            "{message}"
        );
        assert!(
            message.contains(crate::config::SEARCH_ENGINE_GRID_DIJKSTRA),
            "{message}"
        );
    }
}
