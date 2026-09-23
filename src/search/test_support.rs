//! Fixtures and helper functions shared by several of the search test
//! modules under `src/search/`: primitive libraries, route/stats
//! equivalence assertions, and the six test-only wrappers over the four
//! `unified_kernel` wrappers that give the pre-Slice-1 free-function call
//! sites their old signatures back (see the comment on the wrapper group
//! below). Moved out of `src/astar.rs`'s single `mod tests` (Milestone 3,
//! Slice 2); pure code motion, no behaviour change.

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::{
    create_photonic_primitive_library, PrimitiveLibrary, PrimitiveLibraryConfig,
};
use crate::search;
use crate::search::astar::config::AStarConfig;
use crate::search::astar::crossing_rules::CrossingSearchConfig;
use crate::search::state::{RouteResult, RouteSearchStats, State};
use crate::search::NetSearch;
use rustc_hash::FxHashSet;

pub(crate) fn primitive_library() -> PrimitiveLibrary {
    create_photonic_primitive_library(PrimitiveLibraryConfig {
        grid_size_um: 1.0,
        straight_short_cells: 1,
        straight_long_cells: 4,
        bend_radius_cells: 1,
        allow_45_degree_turns: true,
    })
}

/// Benchmark-like library: grid 1.0, bend radius 3, 45-degree turns.
pub(crate) fn primitive_library_bend3() -> PrimitiveLibrary {
    create_photonic_primitive_library(PrimitiveLibraryConfig {
        grid_size_um: 1.0,
        straight_short_cells: 1,
        straight_long_cells: 4,
        bend_radius_cells: 3,
        allow_45_degree_turns: true,
    })
}

pub(crate) fn primitive_library_no45_bend2() -> PrimitiveLibrary {
    create_photonic_primitive_library(PrimitiveLibraryConfig {
        grid_size_um: 1.0,
        straight_short_cells: 1,
        straight_long_cells: 4,
        bend_radius_cells: 2,
        allow_45_degree_turns: false,
    })
}

pub(crate) fn primitive_library_no45_bend1() -> PrimitiveLibrary {
    create_photonic_primitive_library(PrimitiveLibraryConfig {
        grid_size_um: 1.0,
        straight_short_cells: 1,
        straight_long_cells: 4,
        bend_radius_cells: 1,
        allow_45_degree_turns: false,
    })
}

pub(crate) fn assert_route_results_equivalent(
    actual: &Option<RouteResult>,
    expected: &Option<RouteResult>,
) {
    assert_eq!(actual.is_some(), expected.is_some());
    if let (Some(actual), Some(expected)) = (actual, expected) {
        assert_eq!(actual.states, expected.states);
        assert_eq!(actual.primitives, expected.primitives);
        assert_eq!(actual.cells, expected.cells);
        assert_eq!(actual.compressed_waypoints, expected.compressed_waypoints);
        assert_eq!(actual.total_length_um, expected.total_length_um);
        assert_eq!(actual.total_cost, expected.total_cost);
        assert_eq!(actual.requested_target, expected.requested_target);
        assert_eq!(actual.reached_target, expected.reached_target);
        assert_route_search_stats_equivalent(&actual.stats, &expected.stats);
    }
}

pub(crate) fn assert_route_search_stats_equivalent(
    actual: &RouteSearchStats,
    expected: &RouteSearchStats,
) {
    let mut actual = actual.clone();
    let mut expected = expected.clone();
    zero_route_search_timing_stats(&mut actual);
    zero_route_search_timing_stats(&mut expected);
    assert_eq!(format!("{:?}", actual), format!("{:?}", expected));
}

pub(crate) fn zero_route_search_timing_stats(stats: &mut RouteSearchStats) {
    stats.crossing_hotpath_total_time_us = 0;
    stats.crossing_hotpath_owner_scan_time_us = 0;
    stats.crossing_hotpath_segment_time_us = 0;
    stats.crossing_hotpath_reservation_time_us = 0;
    stats.route_search_total_time_us = 0;
    stats.dense_grid_build_time_us = 0;
    stats.search_loop_time_us = 0;
    stats.obstacle_map_prepare_time_us = 0;
    stats.simple_route_time_us = 0;
    stats.commit_prepare_time_us = 0;
    stats.commit_time_us = 0;
    stats.neighbor_generation_time_us = 0;
    stats.heap_operation_time_us = 0;
    stats.legality_check_time_us = 0;
    stats.reconstruction_time_us = 0;
}

pub(crate) fn pack_cells_for_test(cells: &[(i32, i32)]) -> FxHashSet<CellKey> {
    cells.iter().map(|&(x, y)| pack_xy(x, y)).collect()
}

// The three `NetSearch` request fixtures. Introduced for the `net_search_*`
// tests in `src/search/astar/mod.rs` (Milestone 3, Slice 3) and moved here
// in Slice 4 so the second engine's tests (`src/search/grid_dijkstra.rs`)
// run the same geometry against the same A* settings rather than a copy of
// it.

/// An empty 12x5 map and a straight, unobstructed source-to-target pair:
/// the fixture of `net_search_plain_request_takes_the_shortcut_path`.
pub(crate) fn plain_request_fixture() -> (ObstacleMap, PrimitiveLibrary, State, State, AStarConfig)
{
    (
        ObstacleMap::new(12, 5),
        primitive_library_no45_bend1(),
        State::new(1, 2, 0),
        State::new(8, 2, 0),
        AStarConfig {
            require_target_angle: true,
            ..AStarConfig::default()
        },
    )
}

/// `plain_request_fixture` with net 1 committed on (4, 1) and that cell
/// exempted from the clearance check: the fixture of
/// `net_search_dynamic_expansion_request_takes_the_dense_kernel_path`. The
/// returned cell set is the request's `clearance_exempt_cells`.
pub(crate) fn dynamic_expansion_request_fixture() -> (
    ObstacleMap,
    PrimitiveLibrary,
    State,
    State,
    AStarConfig,
    FxHashSet<CellKey>,
) {
    let (mut map, library, source, target, config) = plain_request_fixture();
    assert!(map.commit_route_with_clearance_overlap(1, &[(4, 1)], &[(4, 1)], &[]));
    let exempt_cells = pack_cells_for_test(&[(4, 1)]);
    (map, library, source, target, config, exempt_cells)
}

/// A 20x14 corridor between two static walls with net 1 committed straight
/// across it: the fixture of both `net_search_*_crossing_*` tests, whose
/// request can only reach the target by crossing net 1.
pub(crate) fn crossing_search_fixture() -> (
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
        diagnostics: crate::config::KernelDiagnostics::default(),
        net_id: 2,
        partners: vec![
            crate::search::astar::crossing_rules::CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(8, 2), (8, 10)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
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

// The six deleted free functions (the old single-net search trait's
// free-function twins, see Milestone 3 Slice 1 of
// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`)
// had dozens of test call sites below that exercised them by their old
// signature. Rather than build a `SearchRequest`/`SearchEnvironment` by
// hand at each one, these test-only helpers build the request once,
// per shape, and keep the old signature so each call site's own
// behaviour-under-test stays exactly as it was.

pub(crate) fn test_route_single_net_with_config_reporting_stats(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    out_stats: &mut RouteSearchStats,
) -> Option<RouteResult> {
    let env = search::SearchEnvironment {
        obstacle_map,
        primitives,
    };
    let request = search::SearchRequest {
        source,
        target,
        port_open_cells,
        dynamic_expansion: None,
        crossing: None,
        config,
    };
    let outcome = search::AStarSearch.search(&env, &request);
    *out_stats = outcome.stats;
    outcome.route
}

pub(crate) fn test_route_single_net_with_dynamic_expansion_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    dynamic_expansion_radius_cells: i32,
    dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
) -> Option<RouteResult> {
    let env = search::SearchEnvironment {
        obstacle_map,
        primitives,
    };
    let request = search::SearchRequest {
        source,
        target,
        port_open_cells,
        dynamic_expansion: Some(search::DynamicExpansion {
            radius_cells: dynamic_expansion_radius_cells,
            clearance_exempt_cells: dynamic_clearance_exempt_cells,
        }),
        crossing: None,
        config,
    };
    search::AStarSearch.search(&env, &request).route
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn test_route_single_net_with_dynamic_expansion_config_reporting_stats(
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
    let env = search::SearchEnvironment {
        obstacle_map,
        primitives,
    };
    let request = search::SearchRequest {
        source,
        target,
        port_open_cells,
        dynamic_expansion: Some(search::DynamicExpansion {
            radius_cells: dynamic_expansion_radius_cells,
            clearance_exempt_cells: dynamic_clearance_exempt_cells,
        }),
        crossing: None,
        config,
    };
    let outcome = search::AStarSearch.search(&env, &request);
    *out_stats = outcome.stats;
    outcome.route
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn test_route_single_net_with_collision_crossing_config(
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
) -> Option<RouteResult> {
    test_route_single_net_with_collision_crossing_config_with_stats(
        obstacle_map,
        primitives,
        source,
        target,
        port_open_cells,
        reservation_open_cells,
        config,
        dynamic_expansion_radius_cells,
        dynamic_clearance_exempt_cells,
        crossing,
    )
    .0
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn test_route_single_net_with_collision_crossing_config_with_stats(
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
    let env = search::SearchEnvironment {
        obstacle_map,
        primitives,
    };
    // The deleted free function this replaces always ran the collision
    // wrapper, which reports real stats (a tuple, not an out-param)
    // even on a failed search -- unlike the crossing-config wrapper
    // `AStarSearch::search` reaches for `reservation_open_cells: None`
    // (see its doc comment), whose signature cannot report stats at
    // all. So this helper forces the collision-wrapper dispatch branch
    // by never handing `AStarSearch::search` a `None` reservation set,
    // precomputing the exact same `.or(port_open_cells)` fallback the
    // collision wrapper itself applies -- same anchor cells, same
    // route, and (unlike going through the crossing-config path) real
    // stats even on failure, matching the deleted function exactly.
    let empty_reservation_fallback: FxHashSet<CellKey> = FxHashSet::default();
    let reservation_open_cells = reservation_open_cells
        .or(port_open_cells)
        .unwrap_or(&empty_reservation_fallback);
    let request = search::SearchRequest {
        source,
        target,
        port_open_cells,
        dynamic_expansion: Some(search::DynamicExpansion {
            radius_cells: dynamic_expansion_radius_cells,
            clearance_exempt_cells: dynamic_clearance_exempt_cells,
        }),
        crossing: Some(search::CrossingSearch {
            config: crossing,
            reservation_open_cells: Some(reservation_open_cells),
        }),
        config,
    };
    let outcome = search::AStarSearch.search(&env, &request);
    (outcome.route, outcome.stats)
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn test_route_single_net_with_crossing_config(
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
    let env = search::SearchEnvironment {
        obstacle_map,
        primitives,
    };
    let request = search::SearchRequest {
        source,
        target,
        port_open_cells,
        dynamic_expansion: Some(search::DynamicExpansion {
            radius_cells: dynamic_expansion_radius_cells,
            clearance_exempt_cells: dynamic_clearance_exempt_cells,
        }),
        crossing: Some(search::CrossingSearch {
            config: crossing,
            reservation_open_cells: None,
        }),
        config,
    };
    search::AStarSearch.search(&env, &request).route
}
