use rustc_hash::{FxHashMap, FxHashSet};

use crate::crossings::CrossingConfig;
use crate::search::state::{RouteResult, RouteSearchStats, State};

use crate::engine::*;

/// Commit clearance radius (cells) used for net 1, net 2, net 3 and the
/// vertical test job (net 4) throughout `crossing_conflict_fixture` and
/// its test -- as `commit_radius_cells` and `core_radius_cells` always,
/// and as `block_radius_cells` except for net 3's own setup route (see
/// `DIAGONAL_SETUP_BLOCK_RADIUS_CELLS` below).
///
/// A first draft used radius 0 everywhere and the vertical job's plain
/// search routed clean: with zero clearance the diagonal net's
/// committed footprint is a one-cell-wide staircase ((5,5), (6,6),
/// (7,7), ...), which leaves gaps (e.g. (6,5) and (5,6) are both free
/// even though (5,5) and (6,6) are blocked) a vertical path can slip
/// through one column over without ever sharing a cell with the
/// diagonal, so no crossing (legal or illegal) is ever evaluated.
/// Radius 1 inflates each committed net's blocked footprint enough to
/// close those gaps (two diagonally-adjacent unit squares of radius 1
/// already overlap), which is what actually forces the vertical job
/// into the diagonal.
pub(crate) const FIXTURE_CLEARANCE_RADIUS_CELLS: i32 = 1;

/// `block_radius_cells` used only for net 3's (the diagonal's) own
/// setup route, instead of `FIXTURE_CLEARANCE_RADIUS_CELLS`.
///
/// Found while iterating on this fixture's geometry: with
/// `block_radius_cells=1` (the search's own dynamic-obstacle-expansion
/// radius, separate from the commit/core clearance that shapes what
/// gets marked blocked), routing net 3 with its endpoints directly
/// against the static border below failed with "No legal LiDAR
/// crossing route found" -- a search failure, not a commit rejection.
/// The same geometry with `block_radius_cells=0` for net 3's own route
/// call routes clean; net 1 and net 2 (cardinal-angle, not diagonal)
/// showed no such sensitivity at `block_radius_cells=1` in the same
/// setup. Not fully root-caused beyond this; net 3's own committed
/// footprint (and the map every other job's search sees) is identical
/// either way, since `commit_radius_cells`/`core_radius_cells` --
/// which govern what actually gets marked blocked -- are unchanged.
pub(crate) const DIAGONAL_SETUP_BLOCK_RADIUS_CELLS: i32 = 0;

/// Shared fixture for the probe-guided guidance tests below: a fresh
/// router with no crossing guidance set, same construction as
/// `astar_config_leaves_total_expansion_budget_none_by_default` and
/// `astar_config_applies_negotiated_search_budget_when_set` above.
pub(crate) fn small_test_router() -> PyPhotonicRouter {
    let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
    PyPhotonicRouter::new(
        grid,
        PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
        PyAStarConfig::new(
            10000,
            1.0,
            0,
            true,
            None,
            true,
            12,
            0.35,
            3,
            true,
            0.5,
            10_000_000,
            false,
            0.0,
            0.0,
            0,
            false,
            false,
            "library".to_string(),
            "distance".to_string(),
            1.0,
        ),
        None,
    )
}

pub(crate) fn empty_test_route() -> RouteResult {
    RouteResult {
        states: Vec::new(),
        primitives: Vec::new(),
        cells: Vec::new(),
        compressed_waypoints: Vec::new(),
        total_length_um: 0.0,
        total_cost: 0.0,
        requested_target: State::new(0, 0, 0),
        reached_target: State::new(0, 0, 0),
        stats: RouteSearchStats::default(),
    }
}

pub(crate) fn dummy_invalid_crossing_intersection() -> InvalidCrossingIntersection {
    InvalidCrossingIntersection {
        net_id: 1,
        partner_net_id: 2,
        point: (0.0, 0.0),
        reason: "dummy",
    }
}

/// Milestone 1 kernel fixture of
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`:
/// a small router, lidar-pure crossing mode enabled, with two
/// horizontal nets and one diagonal net committed through the middle
/// of a 60x60 grid, plus a fourth (not yet routed) vertical job whose
/// every possible path must cross the diagonal net non-perpendicularly.
///
/// Geometry: net 1 is horizontal at y=2, from (5, 2) to (54, 2); net 2
/// is horizontal at y=57, from (5, 57) to (54, 57); net 3 is the exact
/// 45-degree diagonal from (5, 5) to (54, 54) (dx == dy == 49), which
/// spans the entire corridor between the two horizontal nets at every
/// x column from 5 to 54 -- a vertical route can never cross it
/// perpendicularly, because a 90-degree crossing of a 45-degree line
/// does not exist. Static obstacles fill grid columns [0, 5) and
/// [55, 60) for the full grid height, closing off the only other way a
/// vertical net could reach the far side (looping around the ends of
/// the diagonal outside the horizontal nets' span, or around net 3's
/// own ends since it spans exactly as wide as net 1 and net 2). The
/// returned job (net 4) is a vertical net from (30, 0) to (30, 59): it
/// must cross net 1 and net 2 (both perpendicular, legal) and net 3
/// (never perpendicular, illegal) to get from one side to the other.
pub(crate) fn crossing_conflict_fixture() -> (PyPhotonicRouter, Vec<NativeRouteJob>) {
    let grid = PyGridSpec::new(60, 60, 1.0, 0.0, 0.0).unwrap();
    let mut router = PyPhotonicRouter::new(
        grid,
        PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
        PyAStarConfig::new(
            10000,
            1.0,
            0,
            true,
            None,
            true,
            12,
            0.35,
            3,
            true,
            0.5,
            10_000_000,
            false,
            0.0,
            0.0,
            0,
            false,
            false,
            "library".to_string(),
            "distance".to_string(),
            1.0,
        ),
        None,
    );
    router.set_collision_crossing_routing(true);
    router.crossing_context.set_config(CrossingConfig {
        enabled: true,
        allow_only_expected_pairs: false,
        ..CrossingConfig::default()
    });

    // Close off the left/right margins of the grid for their full
    // height so nothing can detour around the horizontal nets or the
    // diagonal net via the sides -- without this the vertical job's
    // search simply routes around the whole conflict instead of being
    // forced to cross the diagonal.
    let mut border_cells: Vec<(i32, i32)> = Vec::new();
    for y in 0..60 {
        for x in 0..5 {
            border_cells.push((x, y));
        }
        for x in 55..60 {
            border_cells.push((x, y));
        }
    }
    router.obstacle_map.add_static_cells(&border_cells);

    let mut setup_batch = RepairBatchState {
        final_routes: FxHashMap::default(),
        attempts: Vec::new(),
        repair_trace: Vec::new(),
        repair_count: 0,
        failed_net_id: None,
        failed_error: None,
        retried_source_layers: FxHashSet::default(),
        timings: NativeBatchTimings::default(),
        trace_last_route_start: None,
        deferred_job_indices: Vec::new(),
        deferred_count: 0,
        last_rejected_commit_partners: Vec::new(),
    };

    let horizontal_top = NativeRouteJob::new(
        1,
        PyState::new(5, 2, 0),
        PyState::new(54, 2, 0),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        None,
        None,
    );
    let horizontal_bottom = NativeRouteJob::new(
        2,
        PyState::new(5, 57, 0),
        PyState::new(54, 57, 0),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        None,
        None,
    );
    let diagonal = NativeRouteJob::new(
        3,
        PyState::new(5, 5, 1),
        PyState::new(54, 54, 1),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        None,
        None,
    );
    for job in [&horizontal_top, &horizontal_bottom, &diagonal] {
        let block_radius_cells = if job.net_id == 3 {
            DIAGONAL_SETUP_BLOCK_RADIUS_CELLS
        } else {
            FIXTURE_CLEARANCE_RADIUS_CELLS
        };
        let outcome = router.try_plain_normal_route(
            &mut setup_batch,
            job,
            block_radius_cells,
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            false,
        );
        assert!(
            matches!(outcome, PlainRouteOutcome::Routed),
            "crossing_conflict_fixture setup: net {} failed to route/commit: {:?}",
            job.net_id,
            setup_batch.attempts.last().and_then(|a| a.error.clone())
        );
    }

    let vertical = NativeRouteJob::new(
        4,
        PyState::new(30, 0, 2),
        PyState::new(30, 59, 2),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        None,
        None,
    );

    (
        router,
        vec![horizontal_top, horizontal_bottom, diagonal, vertical],
    )
}

/// Router for the B1 kernel tests below: lidar-pure crossing mode
/// enabled (`enabled: true, allow_only_expected_pairs: false`, matching
/// `crossing_conflict_fixture`), on a 60 x 60 grid at 1 um/cell, with no
/// committed routes yet -- the caller inserts those directly.
pub(crate) fn missing_crossing_event_fixture_router() -> PyPhotonicRouter {
    let grid = PyGridSpec::new(60, 60, 1.0, 0.0, 0.0).unwrap();
    let mut router = PyPhotonicRouter::new(
        grid,
        PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
        PyAStarConfig::new(
            10000,
            1.0,
            0,
            true,
            None,
            true,
            12,
            0.35,
            3,
            true,
            0.5,
            10_000_000,
            false,
            0.0,
            0.0,
            0,
            false,
            false,
            "library".to_string(),
            "distance".to_string(),
            1.0,
        ),
        None,
    );
    router.set_collision_crossing_routing(true);
    router.crossing_context.set_config(CrossingConfig {
        enabled: true,
        allow_only_expected_pairs: false,
        ..CrossingConfig::default()
    });
    router
}

/// A 45-degree committed partner (net 1) from (10, 10) to (50, 50) um,
/// installed directly into `committed_center_routes` (grid waypoints)
/// and `committed_realized_center_routes` (the um centerline) the way
/// the brief specifies -- no search, no commit machinery. With
/// `grid_size_um = 1.0` the grid cells are numerically identical to the
/// um coordinates.
pub(crate) fn install_diagonal_partner(router: &mut PyPhotonicRouter, net_id: u64) {
    router
        .committed_center_routes
        .insert(net_id, vec![(10, 10), (50, 50)]);
    router
        .committed_realized_center_routes
        .insert(net_id, vec![(10.0, 10.0), (50.0, 50.0)]);
}

/// The 135-degree centerline from (10, 50) to (50, 10) um that crosses
/// the diagonal partner installed by `install_diagonal_partner`
/// perpendicularly at (30, 30).
pub(crate) fn crossing_diagonal_centerline() -> Vec<(f64, f64)> {
    vec![(10.0, 50.0), (50.0, 10.0)]
}

/// Builds the fixture, drives it up to the point of a fresh
/// [`ProbeState`] for the vertical job (net 4) the same way
/// `crossing_conflict_fixture_blocks_the_vertical_net_with_a_non_perpendicular_crossing`
/// does, and returns the router, batch, order map and probe for the
/// Milestone 2 `ripup_illegal_crossing_partners` tests below.
pub(crate) fn crossing_conflict_probe_fixture() -> (
    PyPhotonicRouter,
    RepairBatchState,
    NativeRouteJob,
    ProbeState,
) {
    let (mut router, jobs) = crossing_conflict_fixture();
    let vertical = jobs[3].clone();
    assert_eq!(vertical.net_id, 4);
    let order_by_id: FxHashMap<u64, usize> = jobs
        .iter()
        .enumerate()
        .map(|(index, job)| (job.net_id, index))
        .collect();

    let mut batch = RepairBatchState {
        final_routes: FxHashMap::default(),
        attempts: Vec::new(),
        repair_trace: Vec::new(),
        repair_count: 0,
        failed_net_id: None,
        failed_error: None,
        retried_source_layers: FxHashSet::default(),
        timings: NativeBatchTimings::default(),
        trace_last_route_start: None,
        deferred_job_indices: Vec::new(),
        deferred_count: 0,
        last_rejected_commit_partners: Vec::new(),
    };
    for net_id in [1u64, 2, 3] {
        batch.final_routes.insert(net_id, empty_test_route());
    }

    let plain_outcome = router.try_plain_normal_route(
        &mut batch,
        &vertical,
        FIXTURE_CLEARANCE_RADIUS_CELLS,
        Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
        Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
        false,
    );
    assert!(
        matches!(plain_outcome, PlainRouteOutcome::NotResolved),
        "the vertical net must not find a legal plain route before any rip-up"
    );

    let probe = router
        .probe_net_for_repair(
            &mut batch,
            &vertical,
            &order_by_id,
            FIXTURE_CLEARANCE_RADIUS_CELLS,
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            false,
            false,
        )
        .expect("probe search itself must succeed (ignoring dynamic obstacles)");
    assert_eq!(
        probe.candidate_blockers,
        vec![3],
        "this fixture's probe reports only net 3 as a candidate blocker"
    );

    (router, batch, vertical, probe)
}

/// One horizontal net across the middle, then a vertical net that can
/// only reach its target by crossing it (perpendicular, legal).
pub(crate) fn single_horizontal_crossing_fixture(
) -> (PyPhotonicRouter, RepairBatchState, NativeRouteJob) {
    let mut router = missing_crossing_event_fixture_router();
    let mut border_cells: Vec<(i32, i32)> = Vec::new();
    for y in 0..60 {
        for x in 0..5 {
            border_cells.push((x, y));
        }
        for x in 55..60 {
            border_cells.push((x, y));
        }
    }
    router.obstacle_map.add_static_cells(&border_cells);
    let mut batch = fresh_repair_batch_state();
    let horizontal = NativeRouteJob::new(
        1,
        PyState::new(5, 30, 0),
        PyState::new(54, 30, 0),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        None,
        None,
    );
    let outcome = router.try_plain_normal_route(
        &mut batch,
        &horizontal,
        FIXTURE_CLEARANCE_RADIUS_CELLS,
        Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
        Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
        false,
    );
    assert!(
        matches!(outcome, PlainRouteOutcome::Routed),
        "fixture setup: the horizontal net must route: {:?}",
        batch.attempts.last().and_then(|a| a.error.clone())
    );
    let vertical = NativeRouteJob::new(
        2,
        PyState::new(30, 0, 2),
        PyState::new(30, 59, 2),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        None,
        None,
    );
    (router, batch, vertical)
}

/// Milestone 3's own extension of `crossing_conflict_fixture`: a second
/// vertical job (net 5) at a different x column, still within the
/// diagonal net 3's span (x in [5, 54]), so it is blocked by net 3 in
/// exactly the same non-perpendicular way as the first vertical job
/// (net 4) -- see
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
/// Milestone 3. In the full negotiated loop this is the net that shows
/// why `global_ripup_round` is needed: once net 3 has already been
/// ripped up once this epoch by net 4's local rip-up
/// (`ripup_illegal_crossing_partners`'s once-per-epoch rule,
/// `ripped_once`), a second net blocked by net 3 cannot be resolved by
/// the local rule again and must wait for the round-end global rip-up.
pub(crate) fn second_vertical_job_blocked_by_diagonal() -> NativeRouteJob {
    NativeRouteJob::new(
        5,
        PyState::new(40, 0, 2),
        PyState::new(40, 59, 2),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        None,
        None,
    )
}

/// Shared `RepairBatchState` literal for the Milestone 3
/// `global_ripup_round` tests below -- identical to the literal used
/// throughout the Milestone 1/2 fixtures above, factored out because
/// these tests build several of them (one per hand-built scenario).
pub(crate) fn fresh_repair_batch_state() -> RepairBatchState {
    RepairBatchState {
        final_routes: FxHashMap::default(),
        attempts: Vec::new(),
        repair_trace: Vec::new(),
        repair_count: 0,
        failed_net_id: None,
        failed_error: None,
        retried_source_layers: FxHashSet::default(),
        timings: NativeBatchTimings::default(),
        trace_last_route_start: None,
        deferred_job_indices: Vec::new(),
        deferred_count: 0,
        last_rejected_commit_partners: Vec::new(),
    }
}

/// A `RouteResult` carrying a committed net's *real* cells (read back
/// from `router.obstacle_map` via `get_net_cells`), for
/// `global_ripup_round` tests that need `add_repair_history_for_route`
/// to bump history on the net's actual former footprint -- unlike
/// `empty_test_route()`, which the Milestone 1/2 tests above use as a
/// membership-only stand-in (`ripup_illegal_crossing_partners` never
/// reads a `final_routes` entry's cells, only whether the id is
/// present), `global_ripup_round` does read them.
pub(crate) fn route_with_real_cells_for_net(router: &PyPhotonicRouter, net_id: u64) -> RouteResult {
    RouteResult {
        cells: router.get_net_cells(net_id),
        ..empty_test_route()
    }
}

pub(crate) fn crossing_partner_discovery_test_router() -> PyPhotonicRouter {
    let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
    let mut router = PyPhotonicRouter::new(
        grid,
        PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
        PyAStarConfig::new(
            10000,
            1.0,
            0,
            true,
            None,
            true,
            12,
            0.35,
            3,
            true,
            0.5,
            10_000_000,
            false,
            0.0,
            0.0,
            0,
            false,
            false,
            "library".to_string(),
            "distance".to_string(),
            1.0,
        ),
        None,
    );
    router.set_collision_crossing_routing(true);
    assert!(router.obstacle_map.commit_route(1, &[(3, 3)]));
    assert!(router.obstacle_map.commit_route(2, &[(25, 25)]));
    router
}
