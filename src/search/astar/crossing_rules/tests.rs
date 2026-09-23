//! Tests for `crossing_rules.rs`: the crossing-move evaluator, its
//! helpers, and the `CrossingLegalityHook` dispatch into the kernel.
//! Sibling file because the flat test module exceeds 2,000 lines.
//! Moved out of `src/astar.rs`'s `mod tests` (and the smaller
//! `mod unified_kernel::tests`, folded in below as
//! `kernel_hook_dispatch`); pure code motion, no behaviour change.

use super::*;
use crate::config::KernelDiagnostics;
use crate::primitives::create_grid4_unit_grid_primitive_library;
use crate::search::astar::kernel::route_single_net_with_bounds_unified;
use crate::search::astar::route_single_net_with_config;
use crate::search::state::RouteResult;
use crate::search::test_support::*;

#[allow(clippy::too_many_arguments)]
fn crossing_move_outcome(
    obstacle_map: &ObstacleMap,
    crossing: &CrossingSearchConfig,
    current_key: CrossingAStarKey,
    state: State,
    primitive: &Primitive,
    is_straight: bool,
    required_margin: i32,
    capped_required_margin: i32,
    reservation_margin: i32,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    partner_index_by_id: &FxHashMap<NetId, usize>,
    stats: &mut RouteSearchStats,
) -> Option<CrossingMoveOutcome> {
    let partner_segments = crossing_partner_path_segments(crossing);
    let primitive_crossing = primitive_crossing_metadata(primitive);
    let lookup_bounds = RoutingBounds {
        min_x: 0,
        max_x: obstacle_map.width().saturating_sub(1),
        min_y: 0,
        max_y: obstacle_map.height().saturating_sub(1),
    };
    let dynamic_core_owners = DenseDynamicCoreOwnerGrid::from_obstacle_map(
        obstacle_map,
        lookup_bounds.expanded_and_clamped(
            max_crossing_witness_offset(&[vec![primitive_crossing.clone()]]),
            obstacle_map.width(),
            obstacle_map.height(),
        ),
    )?;
    crossing_move_outcome_with_segments(
        obstacle_map,
        crossing,
        current_key,
        state,
        primitive,
        is_straight,
        required_margin,
        capped_required_margin,
        reservation_margin,
        port_open_cells,
        partner_index_by_id,
        &crossing_partner_budget_bits(crossing),
        &partner_segments,
        &dynamic_core_owners,
        &primitive_crossing,
        false,
        false,
        false,
        stats,
        None,
    )
}

#[test]
fn collision_crossing_search_accepts_expected_dynamic_core_collision() {
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

    let library = primitive_library_no45_bend1();
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
    let route = test_route_single_net_with_collision_crossing_config(
        &map,
        &library,
        State::new(2, 6, 0),
        State::new(14, 6, 0),
        None,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            require_target_angle: false,
            ..AStarConfig::default()
        },
        0,
        None,
        &crossing,
    )
    .expect("collision crossing route should be legal");

    assert_eq!(route.compressed_waypoints, vec![(2, 6), (14, 6)]);
    assert!(route.stats.crossing_accepted >= 1);
    assert!(route.stats.crossing_candidate_checks >= 1);
    assert!(route.total_cost >= route.total_length_um + crossing.crossing_loss);
}

/// Contribution 1 (crossing-guided search): a planned partner carries its
/// own crossing price (`crossing_loss_override`, 0 for a planned pair);
/// an unplanned partner pays the configured `crossing_loss`. Same
/// corridor as `collision_crossing_search_accepts_expected_dynamic_core_collision`
/// with two vertical partners; the straight route must cross both.
fn guided_pricing_fixture() -> (ObstacleMap, PrimitiveLibrary) {
    let mut map = ObstacleMap::new(24, 14);
    for x in 3..=17 {
        map.add_static_cell(x, 5);
        map.add_static_cell(x, 7);
    }
    for (net_id, x) in [(1_u64, 7_i32), (2, 13)] {
        let cells: Vec<(i32, i32)> = (2..=10).map(|y| (x, y)).collect();
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            net_id,
            &cells,
            &cells,
            &[],
            &FxHashSet::default()
        ));
    }
    (map, primitive_library_no45_bend1())
}

fn guided_pricing_route(
    map: &ObstacleMap,
    library: &PrimitiveLibrary,
    crossing: &CrossingSearchConfig,
) -> RouteResult {
    test_route_single_net_with_collision_crossing_config(
        map,
        library,
        State::new(2, 6, 0),
        State::new(18, 6, 0),
        None,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            require_target_angle: false,
            ..AStarConfig::default()
        },
        0,
        None,
        crossing,
    )
    .expect("collision crossing route should be legal")
}

#[test]
fn planned_partner_crossing_uses_its_own_price_and_unplanned_pays_crossing_loss() {
    let (map, library) = guided_pricing_fixture();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(7, 2), (7, 10)],
                target_terminal_bump_guard: None,
                crossing_loss_override: Some(0.0),
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(13, 2), (13, 10)],
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
    let route = guided_pricing_route(&map, &library, &crossing);
    assert_eq!(route.compressed_waypoints, vec![(2, 6), (18, 6)]);
    // stats count accepted crossing MOVES over the whole search, not
    // path events -- the price assertion below is the exact check.
    assert!(route.stats.crossing_accepted >= 2);
    assert!(route.stats.crossing_accepted_planned >= 1);
    assert!(route.stats.crossing_accepted_planned < route.stats.crossing_accepted);
    // exactly one crossing on the path is priced: the unplanned one
    assert!((route.total_cost - (route.total_length_um + 3.0)).abs() < 1e-9);
}

/// S2 fixture: ONE planned partner (net 1) shaped like a U -- down at
/// x=7, along y=10, up at x=13 -- so the straight route on y=6 crosses
/// the same partner twice (a braid in the plan's eyes: the plan predicts
/// exactly one crossing per pair).
fn guided_budget_fixture() -> (ObstacleMap, PrimitiveLibrary) {
    let mut map = ObstacleMap::new(24, 14);
    for x in 3..=17 {
        map.add_static_cell(x, 5);
        map.add_static_cell(x, 7);
    }
    let mut cells: Vec<(i32, i32)> = (2..=10).map(|y| (7, y)).collect();
    cells.extend((8..=12).map(|x| (x, 10)));
    cells.extend((2..=10).rev().map(|y| (13, y)));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        1,
        &cells,
        &cells,
        &[],
        &FxHashSet::default()
    ));
    (map, primitive_library_no45_bend1())
}

fn u_partner(single_discounted_crossing: bool) -> CrossingSearchPartner {
    CrossingSearchPartner {
        net_id: 1,
        waypoints: vec![(7, 2), (7, 10), (13, 10), (13, 2)],
        target_terminal_bump_guard: None,
        crossing_loss_override: Some(0.0),
        single_discounted_crossing,
    }
}

fn budget_config(partner: CrossingSearchPartner) -> CrossingSearchConfig {
    CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![partner],
        min_straight_cells: 1,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 3.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    }
}

#[test]
fn second_crossing_of_a_budgeted_planned_partner_pays_the_full_price() {
    let (map, library) = guided_budget_fixture();
    let route = guided_pricing_route(&map, &library, &budget_config(u_partner(true)));
    assert_eq!(route.compressed_waypoints, vec![(2, 6), (18, 6)]);
    // first crossing of net 1 discounted (0), the second one is a braid: 3.0
    assert!((route.total_cost - (route.total_length_um + 3.0)).abs() < 1e-9);
    assert!(route.stats.crossing_accepted_over_budget >= 1);
}

#[test]
fn without_the_budget_every_crossing_of_a_planned_partner_is_discounted() {
    let (map, library) = guided_budget_fixture();
    let route = guided_pricing_route(&map, &library, &budget_config(u_partner(false)));
    assert_eq!(route.compressed_waypoints, vec![(2, 6), (18, 6)]);
    assert!((route.total_cost - route.total_length_um).abs() < 1e-9);
    assert_eq!(route.stats.crossing_accepted_over_budget, 0);
}

#[test]
fn budget_is_per_partner_two_planned_partners_are_both_discounted() {
    let (map, library) = guided_pricing_fixture();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(7, 2), (7, 10)],
                target_terminal_bump_guard: None,
                crossing_loss_override: Some(0.0),
                single_discounted_crossing: true,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(13, 2), (13, 10)],
                target_terminal_bump_guard: None,
                crossing_loss_override: Some(0.0),
                single_discounted_crossing: true,
            },
        ],
        min_straight_cells: 1,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 3.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let route = guided_pricing_route(&map, &library, &crossing);
    // both first crossings are discounted on the final path (the
    // search-wide over-budget counter may still tick on explored
    // detours that re-cross a partner outside the corridor)
    assert!((route.total_cost - route.total_length_um).abs() < 1e-9);
}

#[test]
fn partners_without_price_override_price_like_the_baseline() {
    let (map, library) = guided_pricing_fixture();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(7, 2), (7, 10)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(13, 2), (13, 10)],
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
    let route = guided_pricing_route(&map, &library, &crossing);
    assert!(route.stats.crossing_accepted >= 2);
    assert_eq!(route.stats.crossing_accepted_planned, 0);
    assert!((route.total_cost - (route.total_length_um + 2.0 * 3.0)).abs() < 1e-9);
}

#[test]
fn proactive_congestion_discourages_crowded_straights() {
    let mut map = ObstacleMap::new(12, 7);
    for x in 1..=8 {
        assert!(map.add_static_cell(x, 2));
    }
    let library = create_grid4_unit_grid_primitive_library(1.0);
    let source = State::new(1, 3, 0);
    let target = State::new(9, 3, 0);

    let baseline = route_single_net_with_config(
        &map,
        &library,
        source,
        target,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            require_target_angle: false,
            ..AStarConfig::default()
        },
    )
    .expect("baseline route should exist");
    assert!(
        baseline.cells.iter().filter(|(_, y)| *y == 3).count() >= 8,
        "baseline should prefer the direct lane"
    );

    let congested = route_single_net_with_config(
        &map,
        &library,
        source,
        target,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            require_target_angle: false,
            proactive_congestion_weight: 4.0,
            proactive_congestion_radius_cells: 1,
            ..AStarConfig::default()
        },
    )
    .expect("congestion-aware route should exist");
    assert!(
        congested.cells.iter().filter(|(_, y)| *y == 3).count()
            < baseline.cells.iter().filter(|(_, y)| *y == 3).count(),
        "congestion-aware route should spend less path on the crowded lane"
    );
    assert!(congested.total_cost < baseline.total_cost + 8.0 * 4.0);
}

#[test]
fn crossing_move_accepts_benes8_route15_diagonal_sequence() {
    let mut map = ObstacleMap::new(700, 450);
    let partners = vec![
        CrossingSearchPartner {
            net_id: 14,
            waypoints: vec![(364, 168), (444, 168), (553, 59), (634, 59)],
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        },
        CrossingSearchPartner {
            net_id: 12,
            waypoints: vec![(364, 278), (444, 278), (554, 168), (634, 168)],
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        },
        CrossingSearchPartner {
            net_id: 10,
            waypoints: vec![(364, 388), (414, 388), (627, 175), (634, 169)],
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        },
    ];
    let mut committed_partner_ids = FxHashSet::default();
    for partner in &partners {
        let cells = rasterize_waypoints_for_test(&partner.waypoints);
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            partner.net_id,
            &cells,
            &cells,
            &[],
            &committed_partner_ids
        ));
        committed_partner_ids.insert(partner.net_id);
    }
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 15,
        partners,
        min_straight_cells: 2,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: true,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = crossing
        .partners
        .iter()
        .enumerate()
        .map(|(idx, partner)| (partner.net_id, idx))
        .collect();
    let length_um = 5.0 * 2.0_f64.sqrt();
    let primitive = Primitive {
        id: 0,
        start_angle: 1,
        end_angle: 1,
        dx: 5,
        dy: 5,
        footprint: (0..=5).map(|step| (step, step)).collect(),
        length_um,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um },
    };
    let mut key = CrossingAStarKey {
        state: State::new(414, 59, 1),
        crossed_mask: 0,
        next_partner_index: 0,
        straight_run_cells: 12,
        pending_after_crossing_cells: 0,
        pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
        pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
    };
    let mut stats = RouteSearchStats::default();
    while key.state.x < 630 {
        let next_state = State::new(key.state.x + primitive.dx, key.state.y + primitive.dy, 1);
        let outcome = crossing_move_outcome(
            &map,
            &crossing,
            key,
            key.state,
            &primitive,
            true,
            2,
            2,
            2,
            None,
            &partner_index_by_id,
            &mut stats,
        )
        .unwrap_or_else(|| {
            panic!(
                "diagonal move was rejected at state {:?} with next partner index {}",
                key.state, key.next_partner_index
            )
        });
        key = CrossingAStarKey {
            state: next_state,
            crossed_mask: outcome.crossed_mask,
            next_partner_index: outcome.next_partner_index,
            straight_run_cells: outcome.straight_run_cells,
            pending_after_crossing_cells: outcome.pending_after_crossing_cells,
            pending_after_crossing_angle: outcome.pending_after_crossing_angle,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        };
    }
    assert_eq!(key.next_partner_index, 3);
    assert_eq!(key.crossed_mask, 0b111);
    assert_eq!(stats.crossing_accepted, 3);
    assert!(stats.crossing_candidate_checks >= 3);
}

#[test]
fn crossing_move_accepts_benes8_route13_first_diagonal() {
    let mut map = ObstacleMap::new(700, 450);
    let partners = vec![
        CrossingSearchPartner {
            net_id: 12,
            waypoints: vec![(364, 278), (444, 278), (554, 168), (634, 168)],
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        },
        CrossingSearchPartner {
            net_id: 10,
            waypoints: vec![(364, 388), (414, 388), (627, 175), (634, 169)],
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        },
    ];
    let mut committed_partner_ids = FxHashSet::default();
    for partner in &partners {
        let cells = rasterize_waypoints_for_test(&partner.waypoints);
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            partner.net_id,
            &cells,
            &cells,
            &[],
            &committed_partner_ids
        ));
        committed_partner_ids.insert(partner.net_id);
    }
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 13,
        partners,
        min_straight_cells: 2,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: true,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = crossing
        .partners
        .iter()
        .enumerate()
        .map(|(idx, partner)| (partner.net_id, idx))
        .collect();
    let length_um = 4.0 * 2.0_f64.sqrt();
    let primitive = Primitive {
        id: 0,
        start_angle: 1,
        end_angle: 1,
        dx: 4,
        dy: 4,
        footprint: vec![(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)],
        length_um,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um },
    };
    let mut key = CrossingAStarKey {
        state: State::new(364, 169, 1),
        crossed_mask: 0,
        next_partner_index: 0,
        straight_run_cells: 12,
        pending_after_crossing_cells: 0,
        pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
        pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
    };
    let mut stats = RouteSearchStats::default();
    while key.state.x < 520 {
        let next_state = State::new(key.state.x + 4, key.state.y + 4, 1);
        let outcome = crossing_move_outcome(
            &map,
            &crossing,
            key,
            key.state,
            &primitive,
            true,
            2,
            2,
            2,
            None,
            &partner_index_by_id,
            &mut stats,
        )
        .unwrap_or_else(|| {
            panic!(
                "net13 diagonal move was rejected at state {:?} with next partner index {}",
                key.state, key.next_partner_index
            )
        });
        key = CrossingAStarKey {
            state: next_state,
            crossed_mask: outcome.crossed_mask,
            next_partner_index: outcome.next_partner_index,
            straight_run_cells: outcome.straight_run_cells,
            pending_after_crossing_cells: outcome.pending_after_crossing_cells,
            pending_after_crossing_angle: outcome.pending_after_crossing_angle,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        };
    }
    assert_eq!(key.next_partner_index, 2);
    assert_eq!(key.crossed_mask, 0b11);
    assert_eq!(stats.crossing_accepted, 2);
    assert!(stats.crossing_candidate_checks >= 2);
}

#[test]
fn crossing_move_accepts_perpendicular_shared_core_cell() {
    let mut map = ObstacleMap::new(16, 16);
    assert!(map.commit_route_with_clearance_overlap(
        2,
        &[(6, 5), (6, 6), (6, 7)],
        &[(6, 5), (6, 6), (6, 7)],
        &[],
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![CrossingSearchPartner {
            net_id: 2,
            waypoints: vec![(6, 5), (6, 7)],
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 1,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: true,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = crossing
        .partners
        .iter()
        .enumerate()
        .map(|(idx, p)| (p.net_id, idx))
        .collect();
    let primitive = Primitive {
        id: 0,
        start_angle: 0,
        end_angle: 0,
        dx: 4,
        dy: 0,
        footprint: vec![(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)],
        length_um: 4.0,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um: 4.0 },
    };
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 6, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 2,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 6, 0),
        &primitive,
        true,
        1,
        1,
        1,
        None,
        &partner_index_by_id,
        &mut stats,
    )
    .expect("shared perpendicular core cell should be a valid crossing");

    assert_eq!(outcome.crossed_mask, 0b1);
    assert_eq!(outcome.next_partner_index, 1);
    assert_eq!(outcome.crossing_count, 1);
    assert_eq!(stats.crossing_accepted, 1);
    assert_eq!(stats.crossing_reject_unmatched_owner, 0);
}

#[test]
fn crossing_move_detects_offset_diagonal_halo_contact() {
    let mut map = ObstacleMap::new(800, 300);
    let partner_waypoints = vec![(716, 162), (753, 199)];
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        32,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 33,
        partners: vec![CrossingSearchPartner {
            net_id: 32,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 0,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(32, 0)].into_iter().collect();
    let primitive = Primitive {
        id: 0,
        start_angle: 7,
        end_angle: 7,
        dx: 1,
        dy: -1,
        footprint: vec![(0, 0), (1, -1)],
        length_um: 2.0_f64.sqrt(),
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight {
            length_um: 2.0_f64.sqrt(),
        },
    };

    let mut accepted_stats = RouteSearchStats::default();
    let accepted = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(717, 164, 7),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(717, 164, 7),
        &primitive,
        true,
        1,
        1,
        0,
        None,
        &partner_index_by_id,
        &mut accepted_stats,
    )
    .expect("offset diagonal halo contact should be recognized as a crossing");
    assert_eq!(accepted.crossing_count, 1);
    assert_eq!(accepted_stats.crossing_accepted, 1);

    let mut rejected_stats = RouteSearchStats::default();
    let rejected = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(717, 164, 7),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(717, 164, 7),
        &primitive,
        true,
        5,
        5,
        0,
        None,
        &partner_index_by_id,
        &mut rejected_stats,
    );
    assert!(rejected.is_none());
    assert_eq!(rejected_stats.crossing_accepted, 0);
    assert_eq!(rejected_stats.crossing_reject_margin, 1);
}

/// The "X between cells" case, one cell EARLIER than
/// `crossing_move_detects_offset_diagonal_halo_contact`: the perpendicular
/// partner's centerline crosses the route half a cell PAST this move's
/// end, so no in-move intersection exists, but the diagonal halo
/// (`compact_diagonal_halo_cells`: end+(dx,0) and end+(0,dy)) already
/// touches the partner. Owner decision 2026-09-03 (see
/// .agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md): this
/// approach move is REJECTED as contact without a crossing -- no defer, no
/// eager extension -- and the move that contains the X (one cell later,
/// same fixture) records the crossing. The search reaches the X through
/// that containing move; no second mechanism is needed.
#[test]
fn perpendicular_diagonal_x_approach_move_is_rejected_and_containing_move_accepts() {
    let mut map = ObstacleMap::new(800, 300);
    // Partner: 45-degree diagonal on the line y = x - 554, long enough that
    // partner margin is never the limiting factor at the crossing point.
    let partner_waypoints = vec![(700, 146), (753, 199)];
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        32,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 33,
        partners: vec![CrossingSearchPartner {
            net_id: 32,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(32, 0)].into_iter().collect();
    let primitive = Primitive {
        id: 0,
        start_angle: 7,
        end_angle: 7,
        dx: 1,
        dy: -1,
        footprint: vec![(0, 0), (1, -1)],
        length_um: 2.0_f64.sqrt(),
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight {
            length_um: 2.0_f64.sqrt(),
        },
    };
    // Route line through (716,165) at angle 7 is y = -x + 881; it meets the
    // partner at x = (881 + 554) / 2 = 717.5, y = 163.5 -- half a cell past
    // the move's end (717,164).
    let state = State::new(716, 165, 7);

    // Fixture guard: the move must make halo contact with the partner, so
    // that a different halo shape shows up here as a fixture error rather
    // than as a false pass further down.
    let metadata = primitive_crossing_metadata(&primitive);
    let contact_cells: Vec<(i32, i32)> = metadata
        .witnesses
        .iter()
        .map(|w| (state.x + w.offset.0, state.y + w.offset.1))
        .filter(|&(x, y)| map.dynamic_owners_at(x, y).contains(&32))
        .collect();
    assert!(
        !contact_cells.is_empty(),
        "fixture must produce halo contact with the partner one cell before the X"
    );

    let required_margin = crossing_required_margin_cells(2, 2, 0);
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state,
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        state,
        &primitive,
        true,
        required_margin,
        required_margin,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );
    assert!(
        outcome.is_none(),
        "the approach move (halo contact, intersection half a cell past its end) is contact without a crossing"
    );
    assert_eq!(stats.crossing_accepted, 0);

    // One cell later the same 1-cell move contains the X at t = 0.5.
    let containing = State::new(717, 164, 7);
    let mut containing_stats = RouteSearchStats::default();
    let accepted = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: containing,
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        containing,
        &primitive,
        true,
        required_margin,
        required_margin,
        2,
        None,
        &partner_index_by_id,
        &mut containing_stats,
    )
    .expect("the move that contains the half-cell X records the crossing");
    assert_eq!(accepted.crossing_count, 1);
    assert_eq!(accepted.pending_after_crossing_partner_index, 0);
    // Debt = half_size (2) minus the 0.5 cells of this move past the X,
    // rounded up = 2 pure straight cells still to insert.
    assert_eq!(accepted.pending_after_crossing_cells, 2);
    assert_eq!(containing_stats.crossing_accepted, 1);
}

/// Two parallel 45-degree partners 11 cells apart in line offset (7.8
/// cells perpendicular -- the multiportmmi_32x32 fan spacing) that each
/// split the map, crossed by the shortest possible route: one straight
/// 135-degree line from source to target. On that line the first crossing
/// is an X between cells and the second shares a cell. A correct kernel
/// keeps the straight line and records both crossings; a kernel that
/// cannot complete the X crossing must bend or fail.
#[test]
fn consecutive_perpendicular_diagonal_crossings_on_one_line() {
    let mut map = ObstacleMap::new(90, 90);
    let partner_a: Vec<(i32, i32)> = (0..90).map(|k| (k, k)).collect();
    let partner_b: Vec<(i32, i32)> = (11..90).map(|k| (k, k - 11)).collect();
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        1,
        &partner_a,
        &partner_a,
        &[],
        &FxHashSet::default()
    ));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_b,
        &partner_b,
        &[],
        &FxHashSet::default()
    ));
    let library = primitive_library();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(0, 0), (89, 89)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(11, 0), (89, 78)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 0,
        crossing_loss: 1.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    // Source and target both lie on y = -x + 45; that line meets partner A
    // at (22.5, 22.5) (X between cells) and partner B at (28, 17) (shared
    // cell). Both partners separate source from target, so every route
    // must cross both; the straight line is the cheapest.
    let (route, stats) = test_route_single_net_with_collision_crossing_config_with_stats(
        &map,
        &library,
        State::new(5, 40, 7),
        State::new(40, 5, 7),
        None,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            ..AStarConfig::default()
        },
        0,
        None,
        &crossing,
    );
    let route =
        route.expect("a straight perpendicular line across two parallel diagonals must route");
    let off_line: Vec<(i32, i32)> = route
        .cells
        .iter()
        .copied()
        .filter(|(x, y)| x + y != 45)
        .collect();
    assert!(
        off_line.is_empty(),
        "route must stay on the straight line y = -x + 45 through both crossings; off-line cells: {off_line:?} (pending-straight rejects during search: {})",
        stats.crossing_reject_pending_straight
    );
}

/// Same geometry as `consecutive_perpendicular_diagonal_crossings_on_one_line`
/// but with the benchmark's crossing margin (half_size 2 + bend_runout 3 =
/// required_margin 5): two crossings 7.8 cells apart on one straight line
/// each need 5 cells of straight route on both sides.
#[test]
fn consecutive_perpendicular_diagonal_crossings_with_benchmark_margin() {
    let mut map = ObstacleMap::new(90, 90);
    let partner_a: Vec<(i32, i32)> = (0..90).map(|k| (k, k)).collect();
    let partner_b: Vec<(i32, i32)> = (11..90).map(|k| (k, k - 11)).collect();
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        1,
        &partner_a,
        &partner_a,
        &[],
        &FxHashSet::default()
    ));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_b,
        &partner_b,
        &[],
        &FxHashSet::default()
    ));
    let library = primitive_library();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(0, 0), (89, 89)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(11, 0), (89, 78)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 3,
        crossing_loss: 1.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let (route, stats) = test_route_single_net_with_collision_crossing_config_with_stats(
        &map,
        &library,
        State::new(5, 40, 7),
        State::new(40, 5, 7),
        None,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            ..AStarConfig::default()
        },
        0,
        None,
        &crossing,
    );
    let route = route.expect(
        "a straight perpendicular line across two parallel diagonals must route with margin 5 too",
    );
    let off_line: Vec<(i32, i32)> = route
        .cells
        .iter()
        .copied()
        .filter(|(x, y)| x + y != 45)
        .collect();
    assert!(
        off_line.is_empty(),
        "route must stay on y = -x + 45 through both crossings with margin 5; off-line cells: {off_line:?} (pending-straight rejects: {}, margin rejects: {})",
        stats.crossing_reject_pending_straight, stats.crossing_reject_margin
    );
}

/// Vertical route crossing two horizontal partners 19 cells apart with
/// the benchmark margin (the multiportmmi_32x32 descent column through
/// n_285/n_284). Routes in isolation; kept because the real net stopped
/// descending here after the half-size-debt change (2026-09-03), which
/// means the real blocker is contextual, not this geometry.
#[test]
fn vertical_route_crosses_two_horizontal_partners_with_benchmark_margin() {
    let mut map = ObstacleMap::new(60, 80);
    let partner_a: Vec<(i32, i32)> = (0..60).map(|k| (k, 50)).collect();
    let partner_b: Vec<(i32, i32)> = (0..60).map(|k| (k, 31)).collect();
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        1,
        &partner_a,
        &partner_a,
        &[],
        &FxHashSet::default()
    ));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_b,
        &partner_b,
        &[],
        &FxHashSet::default()
    ));
    let library = primitive_library();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(0, 50), (59, 50)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(0, 31), (59, 31)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 3,
        crossing_loss: 1.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let (route, stats) = test_route_single_net_with_collision_crossing_config_with_stats(
        &map,
        &library,
        State::new(30, 70, 6),
        State::new(30, 10, 6),
        None,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            ..AStarConfig::default()
        },
        0,
        None,
        &crossing,
    );
    assert!(route.is_some(), "vertical descent across two horizontal partners 19 cells apart must route (pending-straight rejects: {}, margin rejects: {}, accepted: {})", stats.crossing_reject_pending_straight, stats.crossing_reject_margin, stats.crossing_accepted);
}

/// Vertical descent across two full-width horizontal partners `gap`
/// cells apart, benchmark crossing config (half_size 2). Returns the
/// route (if any) and the search stats.
fn vertical_descent_across_two_horizontals(gap: i32) -> (Option<RouteResult>, RouteSearchStats) {
    let mut map = ObstacleMap::new(60, 80);
    let y_a = 40;
    let y_b = 40 - gap;
    let partner_a: Vec<(i32, i32)> = (0..60).map(|k| (k, y_a)).collect();
    let partner_b: Vec<(i32, i32)> = (0..60).map(|k| (k, y_b)).collect();
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        1,
        &partner_a,
        &partner_a,
        &[],
        &FxHashSet::default()
    ));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_b,
        &partner_b,
        &[],
        &FxHashSet::default()
    ));
    let library = primitive_library();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(0, y_a), (59, y_a)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![(0, y_b), (59, y_b)],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 3,
        crossing_loss: 1.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    test_route_single_net_with_collision_crossing_config_with_stats(
        &map,
        &library,
        State::new(30, 70, 6),
        State::new(30, 10, 6),
        None,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            ..AStarConfig::default()
        },
        0,
        None,
        &crossing,
    )
}

/// benes_32x32 net 273 (2026-09-03): the route's "/" diagonal crosses the
/// two parallel "\\" diagonals of a port-pair (nets 271 and 270, x+y =
/// 4820 and 4825, i.e. 2.5 diagonal steps apart). Both crossings are
/// perpendicular, but their +-half_size windows overlap in 9 cells --
/// predicate 2 must refuse the second one in the SEARCH (the post-search
/// `crossing_events_have_disjoint_reservations` discarded the whole
/// route). Partners span the whole map so no detour exists.
fn diagonal_route_across_two_parallel_backslash_partners(
    gap_xy: i32,
) -> (Option<RouteResult>, RouteSearchStats) {
    let size = 80;
    let mut map = ObstacleMap::new(size, size);
    // "\\" partner A: x + y = 78 ; partner B: x + y = 78 + gap_xy
    let cells_a: Vec<(i32, i32)> = (0..size)
        .map(|x| (x, 78 - x))
        .filter(|&(_, y)| y >= 0 && y < size)
        .collect();
    let cells_b: Vec<(i32, i32)> = (0..size)
        .map(|x| (x, 78 + gap_xy - x))
        .filter(|&(_, y)| y >= 0 && y < size)
        .collect();
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        1,
        &cells_a,
        &cells_a,
        &[],
        &FxHashSet::default()
    ));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &cells_b,
        &cells_b,
        &[],
        &FxHashSet::default()
    ));
    let library = primitive_library_bend3();
    let a0 = cells_a[0];
    let a1 = *cells_a.last().unwrap();
    let b0 = cells_b[0];
    let b1 = *cells_b.last().unwrap();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 3,
        partners: vec![
            CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![a0, a1],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 2,
                waypoints: vec![b0, b1],
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 3,
        crossing_loss: 1.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    // "/" route on x - y = -10 from the lower left to the upper right
    test_route_single_net_with_collision_crossing_config_with_stats(
        &map,
        &library,
        State::new(10, 20, 1),
        State::new(55, 65, 1),
        None,
        None,
        &AStarConfig {
            diagnostics: KernelDiagnostics::default(),
            use_routing_window: false,
            enable_simple_routes: false,
            ..AStarConfig::default()
        },
        0,
        None,
        &crossing,
    )
}

#[test]
fn diagonal_route_refuses_two_parallel_diagonal_crossings_two_and_a_half_steps_apart() {
    let (route, stats) = diagonal_route_across_two_parallel_backslash_partners(5);
    assert!(
        route.is_none(),
        "two crossing elements 2.5 diagonal steps apart cannot both be realized; got {:?}",
        route.map(|r| r.compressed_waypoints)
    );
    // the second crossing is refused either by the third-net window rule
    // (the first partner's cells lie inside the second window) or by the
    // own-window overlap rule -- both are predicate 2
    assert!(
        stats.crossing_reject_reservation_overlap + stats.crossing_reject_unmatched_footprint > 0,
        "the second crossing must be refused inside the search (accepted={})",
        stats.crossing_accepted
    );
}

/// x+y gap 10 = 5 diagonal steps: windows disjoint, routes.
#[test]
fn diagonal_route_crosses_two_parallel_diagonals_five_steps_apart() {
    let (route, stats) = diagonal_route_across_two_parallel_backslash_partners(10);
    assert!(
        route.is_some(),
        "disjoint windows must route (overlap rejects: {})",
        stats.crossing_reject_reservation_overlap
    );
}

/// The multiportmmi_32x32 finding of 2026-09-03: n_285 and n_284 only
/// THREE cells apart at x=3072. Two crossing elements (+-half_size = 5
/// cells each) cannot sit 3 cells apart -- their reservation windows
/// overlap, and the post-search `crossing_events_have_disjoint_reservations`
/// would discard the whole route. The search itself must refuse the
/// pair (predicate 2), so the full-width partners make this unroutable.
#[test]
fn vertical_descent_refuses_two_crossings_three_cells_apart() {
    let (route, stats) = vertical_descent_across_two_horizontals(3);
    assert!(
        stats.crossing_reject_reservation_overlap > 0,
        "the second crossing must be rejected for window overlap"
    );
    assert!(
        route.is_none(),
        "two crossing elements 3 cells apart cannot both be realized"
    );
}

/// Four cells apart the windows [38,42] and [34,38] still share y=38.
#[test]
fn vertical_descent_refuses_two_crossings_four_cells_apart() {
    let (route, stats) = vertical_descent_across_two_horizontals(4);
    assert!(stats.crossing_reject_reservation_overlap > 0);
    assert!(route.is_none());
}

/// Five cells apart (2*half_size + 1) the windows are disjoint: routes.
#[test]
fn vertical_descent_crosses_two_partners_five_cells_apart() {
    let (route, stats) = vertical_descent_across_two_horizontals(5);
    assert!(
        route.is_some(),
        "disjoint windows must route (overlap rejects: {}, accepted: {})",
        stats.crossing_reject_reservation_overlap,
        stats.crossing_accepted
    );
    assert_eq!(stats.crossing_reject_reservation_overlap, 0);
}

/// Predicate 1 (only straight cells inside the crossing window), route
/// side: a turn primitive never crosses. Route heads east from (2,5)
/// into a radius-3 turn (arms of 3 cells, corner at (5,5)); the partner
/// would be met inside an arm, where the realized fillet (`R*tan(theta/2)`)
/// leaves fewer than half_size real straight cells. The kernel refuses
/// any partner contact of a non-straight primitive outright
/// (`crossing_reject_non_straight`), before any intersection math --
/// so this cannot depend on how arm cells are counted. These tests pin
/// that rule for the 90-degree first arm, the 45-degree first arm and
/// the 90-degree second arm (2026-09-03 audit: the suspected "crossing
/// inside an arm" hole does not exist).
fn crossing_inside_turn_arm(
    end_angle: u8,
    partner_waypoints: Vec<(i32, i32)>,
) -> (Option<CrossingMoveOutcome>, RouteSearchStats) {
    let mut map = ObstacleMap::new(16, 16);
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let library = primitive_library_bend3();
    let primitive = library
        .get_primitives_for_angle(0)
        .iter()
        .find(|primitive| primitive.end_angle == end_angle)
        .expect("turn primitive should exist");
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![CrossingSearchPartner {
            net_id: 2,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 3,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(2, 0)].into_iter().collect();
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(2, 5, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 5,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(2, 5, 0),
        primitive,
        false,
        5,
        5,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );
    (outcome, stats)
}

#[test]
fn crossing_inside_first_arm_of_90_degree_turn_is_rejected() {
    let (outcome, stats) = crossing_inside_turn_arm(2, vec![(3, 0), (3, 15)]);
    assert!(
        outcome.is_none(),
        "no straight cell after a crossing inside a 90-degree arc"
    );
    assert_eq!(
        stats.crossing_reject_non_straight, 1,
        "turn primitives never touch a partner"
    );
    assert_eq!(stats.crossing_accepted, 0);
}

/// Same for a 45-degree turn: the fillet trims `R*tan(22.5)` = 1.24
/// cells before the corner, leaving 0.76 real straight cells after a
/// crossing 2 cells before the corner -- less than half_size.
#[test]
fn crossing_inside_first_arm_of_45_degree_turn_is_rejected() {
    let (outcome, stats) = crossing_inside_turn_arm(1, vec![(3, 0), (3, 15)]);
    assert!(
        outcome.is_none(),
        "0.76 real straight cells after the crossing are not enough"
    );
    assert_eq!(stats.crossing_reject_non_straight, 1);
    assert_eq!(stats.crossing_accepted, 0);
}

/// Crossing in the SECOND arm of the 90-degree turn (horizontal partner
/// at y=7, crossed at (5,7), 2 cells after the corner): the arc covers
/// the whole arm, so the real straight before the crossing is 0.
#[test]
fn crossing_inside_second_arm_of_90_degree_turn_is_rejected() {
    let (outcome, stats) = crossing_inside_turn_arm(2, vec![(0, 7), (15, 7)]);
    assert!(
        outcome.is_none(),
        "no straight cell before a crossing inside a 90-degree arc"
    );
    assert_eq!(stats.crossing_reject_non_straight, 1);
    assert_eq!(stats.crossing_accepted, 0);
}

// ------------------------------------------------------------------
// DIFFERENCE MATRIX baseline vs predicate 1 (2026-09-04). Each test pins
// the BASELINE verdict (required_margin = half_size + bend_radius = 5 on
// counted cells, turn arms counted as straight, partner margins on the
// raw polyline) and names, in its comment, the verdict predicate 1
// (half_size on REAL straight cells, fillet trims) will flip it to.
// Step 2 of the rework flips one row at a time, with benchmark evidence.
// ------------------------------------------------------------------

/// Row 1: cells a turn primitive carries as "straight run" into the next
/// move. Baseline: the whole terminal arm (3). Predicate 1: 90 degrees
/// -> 0 (the arm is arc), 45 degrees -> 1 (3 - ceil(3*tan(22.5))).
#[test]
fn matrix_row1_turn_arm_counts_as_straight_run() {
    let map = ObstacleMap::new(32, 32);
    let library = primitive_library_bend3();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: Vec::new(),
        min_straight_cells: 2,
        crossing_half_size_cells: 2,
        bend_runout_cells: 3,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    for (end_angle, baseline_cells) in [(2u8, 3i32), (1u8, 3i32)] {
        let primitive = library
            .get_primitives_for_angle(0)
            .iter()
            .find(|primitive| primitive.end_angle == end_angle)
            .expect("turn primitive");
        let mut stats = RouteSearchStats::default();
        let outcome = crossing_move_outcome(
            &map,
            &crossing,
            CrossingAStarKey {
                state: State::new(5, 5, 0),
                crossed_mask: 0,
                next_partner_index: 0,
                straight_run_cells: 2,
                pending_after_crossing_cells: 0,
                pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
                pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
            },
            State::new(5, 5, 0),
            primitive,
            false,
            5,
            5,
            2,
            None,
            &FxHashMap::default(),
            &mut stats,
        )
        .expect("a turn without contact is an ordinary move");
        assert_eq!(
            outcome.straight_run_cells, baseline_cells,
            "turn to angle {end_angle}: baseline counts the whole arm"
        );
    }
}

/// Row 2: straight required BEFORE a crossing on a route with no turn
/// anywhere (carried run 0, pure straight approach). Baseline: 5 counted
/// cells (the +3 compensation is applied even without a bend). Predicate
/// 1: 2 real cells. Same geometry H/V and diagonal.
#[test]
fn matrix_row2_before_margin_on_a_pure_straight_approach() {
    let cases = [
        (
            0u8,
            (0..32).map(|y| (12, y)).collect::<Vec<_>>(),
            vec![(12, 0), (12, 31)],
        ),
        (
            1u8,
            (0..32)
                .map(|x| (x, 24 - x))
                .filter(|&(_, y)| y >= 0)
                .collect::<Vec<_>>(),
            vec![(0, 24), (24, 0)],
        ),
    ];
    for (angle, partner_cells, partner_waypoints) in cases {
        let mut map = ObstacleMap::new(32, 32);
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            2,
            &partner_cells,
            &partner_cells,
            &[],
            &FxHashSet::default()
        ));
        let library = primitive_library_bend3();
        let crossing = CrossingSearchConfig {
            diagnostics: KernelDiagnostics::default(),
            net_id: 1,
            partners: vec![CrossingSearchPartner {
                net_id: 2,
                waypoints: partner_waypoints,
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            }],
            min_straight_cells: 2,
            crossing_half_size_cells: 2,
            bend_runout_cells: 3,
            crossing_loss: 0.0,
            require_all_partners: false,
            terminal_bump_guard: None,
        };
        let partner_index_by_id: FxHashMap<NetId, usize> = [(2, 0)].into_iter().collect();
        let straight = library
            .get_primitives_for_angle(angle)
            .iter()
            .find(|p| p.end_angle == angle && p.dx.abs().max(p.dy.abs()) == 4)
            .expect("4-cell straight");
        // crossing 2 cells after the move start with 0 carried cells:
        // baseline rejects (2 < 5); predicate 1 accepts (2 >= 2)
        let start = if angle == 0 {
            State::new(10, 10, 0)
        } else {
            State::new(9, 11, 1)
        };
        let mut stats = RouteSearchStats::default();
        let outcome = crossing_move_outcome(
            &map,
            &crossing,
            CrossingAStarKey {
                state: start,
                crossed_mask: 0,
                next_partner_index: 0,
                straight_run_cells: 0,
                pending_after_crossing_cells: 0,
                pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
                pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
            },
            start,
            straight,
            true,
            5,
            5,
            2,
            None,
            &partner_index_by_id,
            &mut stats,
        );
        assert!(
            outcome.is_none(),
            "angle {angle}: baseline demands 5 counted cells before a crossing (margin rejects {})",
            stats.crossing_reject_margin
        );
    }
}

/// Row 3: partner straight around the crossing point measured on the
/// raw polyline. Baseline: 5 cells to any corner. Predicate 1: 5 to a
/// 90-degree corner (2 + trim 3), 4 to a 45-degree corner (2 + 1.24).
#[test]
fn matrix_row3_partner_margin_on_the_raw_polyline() {
    let cases = [
        (
            "90deg corner, 4 cells before it",
            vec![(2, 10), (12, 10), (12, 20)],
            8,
            false,
        ),
        (
            "90deg corner, 5 cells before it",
            vec![(2, 10), (12, 10), (12, 20)],
            7,
            true,
        ),
        (
            "45deg corner, 3 cells before it",
            vec![(2, 10), (12, 10), (20, 18)],
            9,
            false,
        ),
        // predicate 1 target: accept (4 - 1.24 = 2.76 >= 2)
        (
            "45deg corner, 4 cells before it",
            vec![(2, 10), (12, 10), (20, 18)],
            8,
            false,
        ),
    ];
    for (label, waypoints, cross_x, baseline_accept) in cases {
        let mut map = ObstacleMap::new(32, 32);
        let cells = rasterize_waypoints_for_test(&waypoints);
        assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
            2,
            &cells,
            &cells,
            &[],
            &FxHashSet::default()
        ));
        let library = primitive_library_bend3();
        let crossing = CrossingSearchConfig {
            diagnostics: KernelDiagnostics::default(),
            net_id: 1,
            partners: vec![CrossingSearchPartner {
                net_id: 2,
                waypoints: waypoints.clone(),
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            }],
            min_straight_cells: 2,
            crossing_half_size_cells: 2,
            bend_runout_cells: 3,
            crossing_loss: 0.0,
            require_all_partners: false,
            terminal_bump_guard: None,
        };
        let partner_index_by_id: FxHashMap<NetId, usize> = [(2, 0)].into_iter().collect();
        let straight = library
            .get_primitives_for_angle(2)
            .iter()
            .find(|p| p.end_angle == 2 && p.dy == 4)
            .expect("4-cell north straight");
        let start = State::new(cross_x, 8, 2);
        let mut stats = RouteSearchStats::default();
        let outcome = crossing_move_outcome(
            &map,
            &crossing,
            CrossingAStarKey {
                state: start,
                crossed_mask: 0,
                next_partner_index: 0,
                straight_run_cells: 5,
                pending_after_crossing_cells: 0,
                pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
                pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
            },
            start,
            straight,
            true,
            5,
            5,
            2,
            None,
            &partner_index_by_id,
            &mut stats,
        );
        assert_eq!(
            outcome.is_some(),
            baseline_accept,
            "{label}: margin rejects {}",
            stats.crossing_reject_margin
        );
    }
}

/// Row 4: a crossing point at a partner segment END with `half_size` 0.
/// Baseline rejects via the bend_runout fallback (required 0 + 1 = 1 >
/// margin 0). Predicate 1 keeps the rejection through an explicit
/// "strictly inside the partner's straight" rule.
#[test]
fn matrix_row4_crossing_at_a_partner_endpoint_with_half_size_zero() {
    let mut map = ObstacleMap::new(20, 14);
    let partner_cells = vec![(8, 6), (8, 7)];
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        1,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let library = primitive_library_no45_bend1();
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 2,
        partners: vec![CrossingSearchPartner {
            net_id: 1,
            waypoints: vec![(8, 6), (8, 7)],
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 1,
        crossing_half_size_cells: 0,
        bend_runout_cells: 1,
        crossing_loss: 3.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(1, 0)].into_iter().collect();
    let straight = library
        .get_primitives_for_angle(0)
        .iter()
        .find(|p| p.end_angle == 0 && p.dx == 4)
        .expect("4-cell east straight");
    let start = State::new(6, 6, 0);
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: start,
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 4,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        start,
        straight,
        true,
        1,
        1,
        0,
        None,
        &partner_index_by_id,
        &mut stats,
    );
    assert!(
        outcome.is_none(),
        "the partner's straight has zero extent at its endpoint (8,6)"
    );
    assert_eq!(stats.crossing_reject_margin, 1);
}

#[test]
fn crossing_move_rejects_parallel_diagonal_on_mirrored_halo_side() {
    let mut map = ObstacleMap::new(800, 300);
    let partner_waypoints = vec![(740, 144), (738, 142)];
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        32,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 33,
        partners: vec![CrossingSearchPartner {
            net_id: 32,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 0,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(32, 0)].into_iter().collect();
    let primitive = Primitive {
        id: 0,
        start_angle: 5,
        end_angle: 5,
        dx: -1,
        dy: -1,
        footprint: vec![(0, 0), (-1, -1)],
        length_um: 2.0_f64.sqrt(),
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight {
            length_um: 2.0_f64.sqrt(),
        },
    };

    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(739, 144, 5),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(739, 144, 5),
        &primitive,
        true,
        0,
        0,
        0,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_accepted, 0);
    assert_eq!(stats.crossing_reject_not_perpendicular, 1);
}

#[test]
fn crossing_move_rejects_bend_arm_non_perpendicular_intersection() {
    let mut map = ObstacleMap::new(16, 16);
    let library = primitive_library_no45_bend2();
    let primitive = library
        .get_primitives_for_angle(0)
        .iter()
        .find(|primitive| primitive.end_angle == 2)
        .expect("east-to-north bend should exist");
    let partner_waypoints = vec![(5, 3), (7, 5)];
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![CrossingSearchPartner {
            net_id: 2,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 0,
        crossing_half_size_cells: 0,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = crossing
        .partners
        .iter()
        .enumerate()
        .map(|(idx, partner)| (partner.net_id, idx))
        .collect();
    let mut stats = RouteSearchStats::default();

    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 4,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 0),
        primitive,
        false,
        0,
        1,
        0,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_candidate_checks, 0);
    assert_eq!(stats.crossing_accepted, 0);
    assert_eq!(stats.crossing_reject_non_straight, 1);
}

/// Re-pinned 2026-09-03 (owner decision, .agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md):
/// the straight-after debt is the crossing element's own straight and is
/// paid by pure straights only. A bend's first arm equals the bend radius
/// and is ARC when realized (`make_turn`), so it never pays -- even when
/// the arm is as long as the remaining debt (the old rule accepted that).
/// Once the debt is zero the same bend is accepted.
#[test]
fn crossing_pending_rejects_bend_while_debt_open() {
    let map = ObstacleMap::new(16, 16);
    let library = primitive_library_no45_bend2();
    let primitive = library
        .get_primitives_for_angle(0)
        .iter()
        .find(|primitive| primitive.end_angle == 2)
        .expect("east-to-north bend should exist");
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: Vec::new(),
        min_straight_cells: 0,
        crossing_half_size_cells: 0,
        bend_runout_cells: 2,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 0,
            pending_after_crossing_cells: 2,
            pending_after_crossing_angle: 0,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 0),
        primitive,
        false,
        2,
        2,
        0,
        None,
        &FxHashMap::default(),
        &mut stats,
    );
    assert!(
        outcome.is_none(),
        "a bend must not start while the crossing's straight-after debt is open"
    );
    assert_eq!(stats.crossing_reject_pending_straight, 1);

    let mut paid_stats = RouteSearchStats::default();
    let paid = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 2,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 0),
        primitive,
        false,
        2,
        2,
        0,
        None,
        &FxHashMap::default(),
        &mut paid_stats,
    )
    .expect("once the debt is paid the bend is an ordinary move");
    assert_eq!(paid.pending_after_crossing_cells, 0);
    assert_eq!(paid_stats.crossing_reject_pending_straight, 0);
}

#[test]
fn crossing_pending_margin_rejects_bend_kink_inside_margin() {
    let map = ObstacleMap::new(16, 16);
    let library = primitive_library_no45_bend2();
    let primitive = library
        .get_primitives_for_angle(0)
        .iter()
        .find(|primitive| primitive.end_angle == 2)
        .expect("east-to-north bend should exist");
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: Vec::new(),
        min_straight_cells: 0,
        crossing_half_size_cells: 0,
        bend_runout_cells: 3,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 0,
            pending_after_crossing_cells: 3,
            pending_after_crossing_angle: 0,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 0),
        primitive,
        false,
        3,
        3,
        0,
        None,
        &FxHashMap::default(),
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_pending_straight, 1);
}

#[test]
fn crossing_pending_margin_rejects_wrong_followup_direction() {
    let map = ObstacleMap::new(16, 16);
    let primitive = Primitive {
        id: 0,
        start_angle: 2,
        end_angle: 2,
        dx: 0,
        dy: 4,
        footprint: vec![(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)],
        length_um: 4.0,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um: 4.0 },
    };
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: Vec::new(),
        min_straight_cells: 0,
        crossing_half_size_cells: 0,
        bend_runout_cells: 2,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let mut stats = RouteSearchStats::default();

    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 2),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 0,
            pending_after_crossing_cells: 2,
            pending_after_crossing_angle: 0,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 2),
        &primitive,
        true,
        2,
        2,
        0,
        None,
        &FxHashMap::default(),
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_pending_straight, 1);
}

#[test]
fn crossing_move_rejects_internal_bend_kink_before_after_margin() {
    let mut map = ObstacleMap::new(16, 16);
    let partner_waypoints = vec![(5, 0), (5, 8)];
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let primitive = Primitive {
        id: 0,
        start_angle: 0,
        end_angle: 2,
        dx: 2,
        dy: 2,
        footprint: vec![(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)],
        length_um: 4.0,
        bend_cost: 1.0,
        geometry: PrimitiveGeometry::Bend {
            radius_um: 2.0,
            angle_delta: 2,
        },
    };
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![CrossingSearchPartner {
            net_id: 2,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 0,
        crossing_half_size_cells: 1,
        bend_runout_cells: 2,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(2, 0)].into_iter().collect();
    let mut stats = RouteSearchStats::default();

    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 0),
        &primitive,
        false,
        3,
        3,
        1,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_accepted, 0);
    assert_eq!(stats.crossing_reject_non_straight, 1);
}

/// Re-pinned 2026-09-03 (owner decision, .agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md):
/// a bend never pays the straight-after debt, whatever its arm length --
/// the arm is arc when realized. Previously a three-cell diagonal arm was
/// allowed to satisfy a three-cell pending margin; now both the
/// too-short (4 pending) and the exactly-matching (3 pending) cases are
/// rejected, and only a zero debt lets the bend move.
#[test]
fn crossing_pending_rejects_bend_regardless_of_diagonal_arm_length() {
    let map = ObstacleMap::new(16, 16);
    let primitive = Primitive {
        id: 0,
        start_angle: 1,
        end_angle: 2,
        dx: 3,
        dy: 4,
        footprint: vec![(0, 0), (1, 1), (2, 2), (3, 3), (3, 4)],
        length_um: 3.0 * 2.0_f64.sqrt() + 1.0,
        bend_cost: 1.0,
        geometry: PrimitiveGeometry::Bend {
            radius_um: 3.0,
            angle_delta: 1,
        },
    };
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: Vec::new(),
        min_straight_cells: 0,
        crossing_half_size_cells: 0,
        bend_runout_cells: 4,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };

    let mut rejected_stats = RouteSearchStats::default();
    let rejected = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 1),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 0,
            pending_after_crossing_cells: 4,
            pending_after_crossing_angle: 1,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 1),
        &primitive,
        false,
        4,
        4,
        0,
        None,
        &FxHashMap::default(),
        &mut rejected_stats,
    );

    assert!(rejected.is_none());
    assert_eq!(rejected_stats.crossing_reject_pending_straight, 1);

    let mut accepted_stats = RouteSearchStats::default();
    let accepted = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 1),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 0,
            pending_after_crossing_cells: 3,
            pending_after_crossing_angle: 1,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 1),
        &primitive,
        false,
        4,
        4,
        0,
        None,
        &FxHashMap::default(),
        &mut accepted_stats,
    );

    assert!(
        accepted.is_none(),
        "a bend arm never pays the crossing's straight-after debt, even when it is as long as the debt"
    );
    assert_eq!(accepted_stats.crossing_reject_pending_straight, 1);
}

fn terminal_bump_guard_test_setup(
    target_along_coord: f64,
) -> (
    ObstacleMap,
    CrossingSearchConfig,
    CrossingAStarKey,
    Primitive,
    FxHashMap<NetId, usize>,
) {
    let mut map = ObstacleMap::new(32, 16);
    let partner = vec![(5, 0), (5, 8)];
    let partner_cells = rasterize_waypoints_for_test(&partner);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![CrossingSearchPartner {
            net_id: 2,
            waypoints: partner,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 0,
        crossing_half_size_cells: 2,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: Some(TerminalBumpGuard {
            axis: TerminalBumpAxis::Horizontal,
            target_axis_coord: 4.0,
            target_along_coord,
            required_bump_cells: 12,
        }),
    };
    let key = CrossingAStarKey {
        state: State::new(0, 4, 0),
        crossed_mask: 0,
        next_partner_index: 0,
        straight_run_cells: 12,
        pending_after_crossing_cells: 0,
        pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
        pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
    };
    let primitive = Primitive {
        id: 0,
        start_angle: 0,
        end_angle: 0,
        dx: 8,
        dy: 0,
        footprint: (0..=8).map(|x| (x, 0)).collect(),
        length_um: 8.0,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um: 8.0 },
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = crossing
        .partners
        .iter()
        .enumerate()
        .map(|(idx, partner)| (partner.net_id, idx))
        .collect();
    (map, crossing, key, primitive, partner_index_by_id)
}

#[test]
fn crossing_terminal_bump_guard_rejects_too_close_target_axis_crossing() {
    let (map, crossing, key, primitive, partner_index_by_id) = terminal_bump_guard_test_setup(13.0);
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        key,
        State::new(0, 4, 0),
        &primitive,
        true,
        2,
        2,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_margin, 1);
}

#[test]
fn crossing_terminal_bump_guard_rejects_at_exact_axis_margin() {
    let (map, mut crossing, key, primitive, partner_index_by_id) =
        terminal_bump_guard_test_setup(13.0);
    crossing.terminal_bump_guard = crossing.terminal_bump_guard.map(|mut guard| {
        guard.target_axis_coord = -2.0;
        guard
    });
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        key,
        State::new(0, 4, 0),
        &primitive,
        true,
        2,
        2,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_margin, 1);
}

#[test]
fn crossing_terminal_bump_guard_rejects_insufficient_available_distance() {
    let (map, crossing, key, primitive, partner_index_by_id) = terminal_bump_guard_test_setup(16.0);
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        key,
        State::new(0, 4, 0),
        &primitive,
        true,
        2,
        2,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_margin, 1);
}

#[test]
fn crossing_terminal_bump_guard_rejects_exact_required_target_axis_distance() {
    let (map, crossing, key, primitive, partner_index_by_id) = terminal_bump_guard_test_setup(19.0);
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        key,
        State::new(0, 4, 0),
        &primitive,
        true,
        2,
        2,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_margin, 1);
}

#[test]
fn crossing_terminal_bump_guard_rejects_rounded_required_axis_delta() {
    let (map, mut crossing, key, primitive, partner_index_by_id) =
        terminal_bump_guard_test_setup(20.0);
    crossing.terminal_bump_guard = crossing.terminal_bump_guard.map(|mut guard| {
        guard.target_axis_coord = -8.0;
        guard
    });
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        key,
        State::new(0, 4, 0),
        &primitive,
        true,
        2,
        2,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_margin, 1);
}

#[test]
fn crossing_terminal_bump_guard_accepts_more_than_required_target_axis_distance() {
    let (map, crossing, key, primitive, partner_index_by_id) = terminal_bump_guard_test_setup(20.0);
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        key,
        State::new(0, 4, 0),
        &primitive,
        true,
        2,
        2,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    )
    .expect("available distance exceeds the full terminal bump distance");

    assert_eq!(outcome.crossing_count, 1);
    assert_eq!(stats.crossing_accepted, 1);
    assert_eq!(stats.crossing_reject_margin, 0);
}

#[test]
fn crossing_partner_terminal_bump_guard_rejects_partner_target_axis_distance() {
    let (map, mut crossing, key, primitive, partner_index_by_id) =
        terminal_bump_guard_test_setup(20.0);
    crossing.terminal_bump_guard = None;
    crossing.partners[0].target_terminal_bump_guard = Some(TerminalBumpGuard {
        axis: TerminalBumpAxis::Vertical,
        target_axis_coord: -7.0,
        target_along_coord: 20.0,
        required_bump_cells: 12,
    });
    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        key,
        State::new(0, 4, 0),
        &primitive,
        true,
        2,
        2,
        2,
        None,
        &partner_index_by_id,
        &mut stats,
    );

    assert!(outcome.is_none());
    assert_eq!(stats.crossing_reject_margin, 1);
}

#[test]
fn crossing_move_rejects_two_crossings_with_overlapping_windows_in_one_move() {
    let mut map = ObstacleMap::new(32, 16);
    let partner_a = vec![(4, 0), (4, 11)];
    let partner_b = vec![(7, 0), (7, 11)];
    let cells_a = rasterize_waypoints_for_test(&partner_a);
    let cells_b = rasterize_waypoints_for_test(&partner_b);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &cells_a,
        &cells_a,
        &[],
        &FxHashSet::default()
    ));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        3,
        &cells_b,
        &cells_b,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![
            CrossingSearchPartner {
                net_id: 2,
                waypoints: partner_a,
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 3,
                waypoints: partner_b,
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
        min_straight_cells: 0,
        // Predicate 2 (2026-09-03): partners 3 cells apart with
        // half_size 5 have +-5 reservation windows that overlap -- two
        // crossing elements cannot both be realized, so the move that
        // would cross both is rejected (intra-move window check).
        crossing_half_size_cells: 5,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(2, 0), (3, 1)].into_iter().collect();
    let primitive = Primitive {
        id: 0,
        start_angle: 0,
        end_angle: 0,
        dx: 8,
        dy: 0,
        footprint: (0..=8).map(|x| (x, 0)).collect(),
        length_um: 8.0,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um: 8.0 },
    };

    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(0, 5, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(0, 5, 0),
        &primitive,
        true,
        5,
        5,
        0,
        None,
        &partner_index_by_id,
        &mut stats,
    );
    assert!(
        outcome.is_none(),
        "overlapping reservation windows must reject the move"
    );
    assert_eq!(stats.crossing_reject_reservation_overlap, 1);
}

#[test]
fn crossing_pending_after_keeps_debt_of_the_later_crossing() {
    let mut map = ObstacleMap::new(32, 16);
    let partner_a = vec![(2, 0), (2, 11)];
    let partner_b = vec![(7, 0), (7, 11)];
    let cells_a = rasterize_waypoints_for_test(&partner_a);
    let cells_b = rasterize_waypoints_for_test(&partner_b);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &cells_a,
        &cells_a,
        &[],
        &FxHashSet::default()
    ));
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        3,
        &cells_b,
        &cells_b,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![
            CrossingSearchPartner {
                net_id: 2,
                waypoints: partner_a,
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
            CrossingSearchPartner {
                net_id: 3,
                waypoints: partner_b,
                target_terminal_bump_guard: None,
                crossing_loss_override: None,
                single_discounted_crossing: false,
            },
        ],
        min_straight_cells: 0,
        // Re-pinned 2026-09-03 (predicate 2): partners 5 cells apart
        // (x=2 and x=7) so the +-2 windows are disjoint. The 8-cell
        // straight from x=0 crosses A with 6 cells after (no debt) and B
        // with 1 cell after (debt 1); the open debt of the later
        // crossing is what the outcome carries.
        crossing_half_size_cells: 2,
        bend_runout_cells: 0,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(2, 0), (3, 1)].into_iter().collect();
    let primitive = Primitive {
        id: 0,
        start_angle: 0,
        end_angle: 0,
        dx: 8,
        dy: 0,
        footprint: (0..=8).map(|x| (x, 0)).collect(),
        length_um: 8.0,
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight { length_um: 8.0 },
    };

    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(0, 5, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 10,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(0, 5, 0),
        &primitive,
        true,
        5,
        5,
        0,
        None,
        &partner_index_by_id,
        &mut stats,
    )
    .expect("both perpendicular crossings should be accepted with pending runout");

    assert_eq!(outcome.crossing_count, 2);
    assert_eq!(outcome.pending_after_crossing_cells, 1);
    assert_eq!(outcome.pending_after_crossing_partner_index, 1);
}

#[test]
fn crossing_move_marks_n35_n32_short_after_runout_as_pending() {
    let mut map = ObstacleMap::new(800, 300);
    let partner_waypoints = vec![(716, 165), (753, 128)];
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        33,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));
    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 36,
        partners: vec![CrossingSearchPartner {
            net_id: 33,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 0,
        crossing_half_size_cells: 3,
        bend_runout_cells: 2,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(33, 0)].into_iter().collect();
    let primitive = Primitive {
        id: 0,
        start_angle: 5,
        end_angle: 5,
        dx: -13,
        dy: -13,
        footprint: (0..=13).map(|step| (-step, -step)).collect(),
        length_um: 13.0 * 2.0_f64.sqrt(),
        bend_cost: 0.0,
        geometry: PrimitiveGeometry::Straight {
            length_um: 13.0 * 2.0_f64.sqrt(),
        },
    };

    let mut stats = RouteSearchStats::default();
    let outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(750, 155, 5),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 12,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(750, 155, 5),
        &primitive,
        true,
        5,
        5,
        3,
        None,
        &partner_index_by_id,
        &mut stats,
    )
    .expect("perpendicular n35/n32 crossing should be recognized by A*");

    assert_eq!(outcome.crossing_count, 1);
    // Re-pinned 2026-09-03: the intersection lies 1 cell before the move
    // end; the straight-after debt is the crossing element's half size
    // (3) minus that 1 cell = 2 (was 4 when it derived from
    // required_margin = half_size 3 + bend_runout 2).
    assert_eq!(outcome.pending_after_crossing_cells, 2);
    assert_eq!(stats.crossing_accepted, 1);
}

#[test]
fn crossing_margin_counts_terminal_bend_arm_before_next_crossing() {
    let mut map = ObstacleMap::new(16, 16);
    let library = primitive_library_no45_bend2();
    let bend = library
        .get_primitives_for_angle(0)
        .iter()
        .find(|primitive| primitive.end_angle == 2)
        .expect("east-to-north bend should exist");
    let partner_waypoints = vec![(2, 8), (10, 8)];
    let partner_cells = rasterize_waypoints_for_test(&partner_waypoints);
    assert!(map.commit_route_with_clearance_and_allowed_core_overlaps(
        2,
        &partner_cells,
        &partner_cells,
        &[],
        &FxHashSet::default()
    ));

    let empty_crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: Vec::new(),
        min_straight_cells: 0,
        crossing_half_size_cells: 2,
        bend_runout_cells: 2,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let mut stats = RouteSearchStats::default();
    let bend_outcome = crossing_move_outcome(
        &map,
        &empty_crossing,
        CrossingAStarKey {
            state: State::new(4, 4, 0),
            crossed_mask: 0,
            next_partner_index: 0,
            straight_run_cells: 0,
            pending_after_crossing_cells: 0,
            pending_after_crossing_angle: NO_PENDING_CROSSING_ANGLE,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(4, 4, 0),
        bend,
        false,
        4,
        4,
        0,
        None,
        &FxHashMap::default(),
        &mut stats,
    )
    .expect("bend without crossing should remain legal");
    assert_eq!(bend_outcome.straight_run_cells, 2);

    let crossing = CrossingSearchConfig {
        diagnostics: KernelDiagnostics::default(),
        net_id: 1,
        partners: vec![CrossingSearchPartner {
            net_id: 2,
            waypoints: partner_waypoints,
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        }],
        min_straight_cells: 0,
        crossing_half_size_cells: 2,
        bend_runout_cells: 2,
        crossing_loss: 0.0,
        require_all_partners: false,
        terminal_bump_guard: None,
    };
    let partner_index_by_id: FxHashMap<NetId, usize> = [(2, 0)].into_iter().collect();
    let straight = library
        .get_primitives_for_angle(2)
        .iter()
        .find(|primitive| primitive.end_angle == 2 && primitive.dx == 0 && primitive.dy == 4)
        .expect("north long straight should exist");
    let crossing_outcome = crossing_move_outcome(
        &map,
        &crossing,
        CrossingAStarKey {
            state: State::new(6, 6, 2),
            crossed_mask: bend_outcome.crossed_mask,
            next_partner_index: bend_outcome.next_partner_index,
            straight_run_cells: bend_outcome.straight_run_cells,
            pending_after_crossing_cells: bend_outcome.pending_after_crossing_cells,
            pending_after_crossing_angle: bend_outcome.pending_after_crossing_angle,
            pending_after_crossing_partner_index: NO_PENDING_CROSSING_PARTNER_INDEX,
        },
        State::new(6, 6, 2),
        straight,
        true,
        4,
        4,
        0,
        None,
        &partner_index_by_id,
        &mut stats,
    )
    .expect("terminal bend arm should satisfy pre-crossing runout");

    assert_eq!(crossing_outcome.crossing_count, 1);
    assert_eq!(stats.crossing_accepted, 1);
}

#[test]
fn crossing_reservation_window_rejects_static_and_unrelated_dynamic_cells() {
    let mut map = ObstacleMap::new(32, 32);
    assert!(map.commit_route(1, &[(10, 12)]));
    assert!(crossing_reservation_window_is_clear(
        &map, 3, 1, None, 10.0, 12.0, 1
    ));

    assert!(map.commit_route(4, &[(11, 12)]));
    assert!(!crossing_reservation_window_is_clear(
        &map, 3, 1, None, 10.0, 12.0, 1
    ));
    assert!(map.ripup_route(4));

    map.add_static_cells(&[(11, 12)]);
    assert!(!crossing_reservation_window_is_clear(
        &map, 3, 1, None, 10.0, 12.0, 1
    ));

    let opened_cells: FxHashSet<CellKey> = [pack_xy(11, 12)].into_iter().collect();
    assert!(crossing_reservation_window_is_clear(
        &map,
        3,
        1,
        Some(&opened_cells),
        10.0,
        12.0,
        1
    ));
}

fn rasterize_waypoints_for_test(waypoints: &[(i32, i32)]) -> Vec<(i32, i32)> {
    let mut cells = Vec::new();
    for segment in waypoints.windows(2) {
        let dx = (segment[1].0 - segment[0].0).signum();
        let dy = (segment[1].1 - segment[0].1).signum();
        let steps = (segment[1].0 - segment[0].0)
            .abs()
            .max((segment[1].1 - segment[0].1).abs());
        for step in 0..=steps {
            let cell = (segment[0].0 + dx * step, segment[0].1 + dy * step);
            if cells.last().copied() != Some(cell) {
                cells.push(cell);
            }
        }
    }
    cells
}

/// Folded in from `mod unified_kernel`'s own smaller `mod tests` (the
/// hook is called only on a real collision, not on every step; a
/// legal/illegal crossing via the hook inserts/rejects as expected).
mod kernel_hook_dispatch {
    use super::*;
    use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};
    use std::cell::Cell;

    fn primitive_library_no45_bend1() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
            allow_45_degree_turns: false,
        })
    }

    fn full_bounds_of(map: &ObstacleMap) -> RoutingBounds {
        RoutingBounds {
            min_x: 0,
            max_x: map.width() - 1,
            min_y: 0,
            max_y: map.height() - 1,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn route_single_net_unified_for_test(
        obstacle_map: &ObstacleMap,
        primitives: &PrimitiveLibrary,
        source: State,
        target: State,
        config: &AStarConfig,
        crossing: Option<&CrossingSearchConfig>,
    ) -> (Option<RouteResult>, RouteSearchStats) {
        let full_bounds = full_bounds_of(obstacle_map);
        let mut stats = RouteSearchStats::default();
        let route = match crossing {
            None => route_single_net_with_bounds_unified(
                obstacle_map,
                primitives,
                source,
                target,
                None,
                config,
                None,
                &mut stats,
                0,
                None,
                &NoCrossingHook,
            ),
            Some(crossing) => {
                let context = CrossingHookContext::build(
                    obstacle_map,
                    primitives,
                    config,
                    crossing,
                    full_bounds,
                );
                let hook = context.hook(
                    obstacle_map,
                    crossing,
                    config,
                    primitives.grid_size_um(),
                    None,
                    None,
                );
                route_single_net_with_bounds_unified(
                    obstacle_map,
                    primitives,
                    source,
                    target,
                    None,
                    config,
                    Some(full_bounds),
                    &mut stats,
                    0,
                    None,
                    &hook,
                )
            }
        };
        (route, stats)
    }

    // A parity test against the old plain kernel lived here through
    // Milestones 2-4; it served its purpose (it is what caught
    // Milestone 4's real `straight_run_cells` bug against the benchmark
    // ladder) and its comparison target no longer exists now that
    // Milestone 4 has deleted the old kernel. The unified kernel's own
    // behavior remains covered by the dozens of pre-existing tests in
    // `mod tests` below that call `route_single_net_with_config` and
    // friends directly -- those now run on the unified kernel too.

    fn crossing_fixture() -> (
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
    fn unified_kernel_inserts_legal_crossing_via_hook() {
        let (map, primitives, source, target, config, crossing) = crossing_fixture();
        let (route, stats) = route_single_net_unified_for_test(
            &map,
            &primitives,
            source,
            target,
            &config,
            Some(&crossing),
        );
        let route = route.expect("route should exist through a legal crossing");
        assert!(stats.expanded_states > 0);
        // The route must actually cross the partner's column (x == 8),
        // proving the collision-triggered hook -- not just an
        // obstacle-avoiding detour -- produced this route.
        assert!(route
            .cells
            .iter()
            .any(|&(x, y)| x == 8 && (2..=10).contains(&y)));
    }

    #[test]
    fn unified_kernel_rejects_crossing_when_no_hook_allows_it() {
        let (map, primitives, source, target, config, _crossing) = crossing_fixture();
        // With `NoCrossingHook`, the same obstacle map the crossing
        // fixture above successfully routes through must fail outright:
        // the wall at y=5 and y=7 spans the whole window and there is no
        // crossing to legalize a way through it.
        let (route, _stats) =
            route_single_net_unified_for_test(&map, &primitives, source, target, &config, None);
        assert!(route.is_none());
    }

    /// Wraps any hook and counts how many times `evaluate` actually ran,
    /// without changing its behavior.
    struct CountingHook<'a, H: CrossingLegalityHook> {
        inner: &'a H,
        calls: Cell<usize>,
    }

    impl<H: CrossingLegalityHook> CrossingLegalityHook for CountingHook<'_, H> {
        fn evaluate(
            &self,
            state: State,
            current_extension: CrossingExtension,
            primitive: &Primitive,
            primitive_class_is_straight: bool,
            primitive_crossing: &PrimitiveCrossingMetadata,
            footprint_free: bool,
            stats: &mut RouteSearchStats,
        ) -> Option<(CrossingMoveOutcome, f64)> {
            self.calls.set(self.calls.get() + 1);
            self.inner.evaluate(
                state,
                current_extension,
                primitive,
                primitive_class_is_straight,
                primitive_crossing,
                footprint_free,
                stats,
            )
        }

        fn goal_extra_ok(&self, extension: CrossingExtension) -> bool {
            self.inner.goal_extra_ok(extension)
        }

        fn heuristic_bonus(&self, next_state: State, next_extension: CrossingExtension) -> f64 {
            self.inner.heuristic_bonus(next_state, next_extension)
        }
    }

    #[test]
    fn hook_is_called_only_on_collision_not_on_every_step() {
        // Scenario A: an empty map with a crossing config present but
        // nothing ever actually blocking the route -- every step stays
        // in Tier 1, so the hook must never fire at all.
        let empty_map = ObstacleMap::new(20, 10);
        let primitives = primitive_library_no45_bend1();
        let far_crossing = CrossingSearchConfig {
            diagnostics: KernelDiagnostics::default(),
            net_id: 2,
            partners: vec![CrossingSearchPartner {
                net_id: 1,
                waypoints: vec![(1, 1), (1, 2)],
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
        let full_bounds = full_bounds_of(&empty_map);
        let context = CrossingHookContext::build(
            &empty_map,
            &primitives,
            &config,
            &far_crossing,
            full_bounds,
        );
        let hook = context.hook(
            &empty_map,
            &far_crossing,
            &config,
            primitives.grid_size_um(),
            None,
            None,
        );
        let counting = CountingHook {
            inner: &hook,
            calls: Cell::new(0),
        };
        let mut stats = RouteSearchStats::default();
        let route = route_single_net_with_bounds_unified(
            &empty_map,
            &primitives,
            State::new(1, 5, 0),
            State::new(18, 5, 0),
            None,
            &config,
            Some(full_bounds),
            &mut stats,
            0,
            None,
            &counting,
        );
        assert!(route.is_some());
        assert!(stats.generated_neighbors > 0);
        assert_eq!(
            counting.calls.get(),
            0,
            "hook must not be called when no step is ever blocked"
        );

        // Scenario B: the crossing fixture's wall forces a real collision
        // -- the hook must fire, but only for a small fraction of the
        // primitives the search actually generates.
        let (map, primitives, source, target, config, crossing) = crossing_fixture();
        let full_bounds = full_bounds_of(&map);
        let context =
            CrossingHookContext::build(&map, &primitives, &config, &crossing, full_bounds);
        let hook = context.hook(
            &map,
            &crossing,
            &config,
            primitives.grid_size_um(),
            None,
            None,
        );
        let counting = CountingHook {
            inner: &hook,
            calls: Cell::new(0),
        };
        let mut stats = RouteSearchStats::default();
        let route = route_single_net_with_bounds_unified(
            &map,
            &primitives,
            source,
            target,
            None,
            &config,
            Some(full_bounds),
            &mut stats,
            0,
            None,
            &counting,
        );
        assert!(route.is_some());
        assert!(
            counting.calls.get() > 0,
            "hook must be called when the search hits a real collision"
        );
        assert!(
            counting.calls.get() < stats.generated_neighbors,
            "hook must not be called on every generated neighbor"
        );
    }
}
