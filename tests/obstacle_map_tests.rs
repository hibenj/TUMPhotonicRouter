use photonic_router::{unpack_xy, ClearanceMetric, ObstacleMap};
use rustc_hash::FxHashSet;

fn unpack_set(keys: &FxHashSet<u64>) -> FxHashSet<(i32, i32)> {
    keys.iter().copied().map(unpack_xy).collect()
}

#[test]
fn creates_empty_map() {
    let map = ObstacleMap::new(10, 8);

    assert_eq!(map.width(), 10);
    assert_eq!(map.height(), 8);
    assert!(map.in_bounds(0, 0));
    assert!(map.in_bounds(9, 7));
    assert!(!map.in_bounds(10, 7));
    assert!(!map.in_bounds(9, 8));
    assert!(!map.is_static_blocked(1, 1));
    assert!(!map.is_dynamic_blocked(1, 1));
    assert!(!map.is_blocked(1, 1));
    assert!(map.is_blocked(10, 7));
    assert_eq!(map.ref_count(1, 1), 0);
}

#[test]
fn adds_static_obstacles() {
    let mut map = ObstacleMap::new(5, 5);

    assert!(map.add_static_cell(2, 3));
    assert!(!map.add_static_cell(5, 3));

    assert!(map.is_static_blocked(2, 3));
    assert!(map.is_blocked(2, 3));
    assert_eq!(map.ref_count(2, 3), 1);
}

#[test]
fn checks_blocked_and_free_cells() {
    let mut map = ObstacleMap::new(5, 5);
    map.add_static_cells(&[(1, 1), (3, 3)]);

    assert!(map.check_cells_free(&[(0, 0), (2, 2)], None));
    assert!(!map.check_cells_free(&[(0, 0), (1, 1)], None));
    assert!(!map.check_cells_free(&[(0, 0), (5, 0)], None));
}

#[test]
fn commits_route_for_one_net() {
    let mut map = ObstacleMap::new(10, 10);

    assert!(map.commit_route(42, &[(1, 1), (2, 1), (3, 1)]));

    assert!(map.is_dynamic_blocked(1, 1));
    assert!(map.is_dynamic_blocked(2, 1));
    assert!(map.is_dynamic_blocked(3, 1));
    assert_eq!(map.ref_count(2, 1), 1);

    let net_cells = map.get_net_cells(42).unwrap();
    assert_eq!(net_cells.len(), 3);
}

#[test]
fn rips_up_committed_route() {
    let mut map = ObstacleMap::new(10, 10);
    map.commit_route(7, &[(1, 1), (2, 1), (3, 1)]);

    assert!(map.ripup_route(7));
    assert!(!map.ripup_route(7));

    assert!(!map.is_dynamic_blocked(1, 1));
    assert!(!map.is_dynamic_blocked(2, 1));
    assert!(map.get_net_cells(7).is_none());
}

#[test]
fn reference_counts_overlapping_routes() {
    let mut map = ObstacleMap::new(10, 10);
    map.commit_route(1, &[(1, 1), (2, 1), (3, 1)]);
    map.commit_route(2, &[(2, 1), (2, 2)]);

    assert_eq!(map.ref_count(2, 1), 2);
    assert!(map.is_dynamic_blocked(2, 1));

    assert!(map.ripup_route(1));
    assert_eq!(map.ref_count(2, 1), 1);
    assert!(map.is_dynamic_blocked(2, 1));

    assert!(map.ripup_route(2));
    assert_eq!(map.ref_count(2, 1), 0);
    assert!(!map.is_dynamic_blocked(2, 1));
}

#[test]
fn clearance_inflation_manhattan() {
    let map = ObstacleMap::new(7, 7);
    let inflated = map.inflate_cells(&[(3, 3)], 1, ClearanceMetric::Manhattan);
    let coords = unpack_set(&inflated);

    let expected: FxHashSet<(i32, i32)> = [(3, 3), (2, 3), (4, 3), (3, 2), (3, 4)]
        .into_iter()
        .collect();

    assert_eq!(coords, expected);
}

#[test]
fn clearance_inflation_chebyshev() {
    let map = ObstacleMap::new(7, 7);
    let inflated = map.inflate_cells(&[(3, 3)], 1, ClearanceMetric::Chebyshev);
    let coords = unpack_set(&inflated);

    let expected: FxHashSet<(i32, i32)> = [
        (2, 2),
        (3, 2),
        (4, 2),
        (2, 3),
        (3, 3),
        (4, 3),
        (2, 4),
        (3, 4),
        (4, 4),
    ]
    .into_iter()
    .collect();

    assert_eq!(coords, expected);
}

#[test]
fn primitive_footprint_collision_checking() {
    let mut map = ObstacleMap::new(8, 8);
    map.add_static_cell(4, 4);

    let footprint = [(0, 0), (1, 0), (1, 1)];

    assert!(map.check_primitive_footprint_free(2, 2, &footprint, None));
    assert!(!map.check_primitive_footprint_free(3, 3, &footprint, None));
    assert!(!map.check_primitive_footprint_free(7, 7, &footprint, None));
}

#[test]
fn opened_cells_override_blocked_cells_during_query() {
    let mut map = ObstacleMap::new(6, 6);
    map.add_static_cell(1, 1);
    map.commit_route(10, &[(2, 2), (3, 2)]);

    assert!(!map.check_cells_free(&[(1, 1), (2, 2)], None));

    let mut opened = FxHashSet::default();
    opened.insert(ObstacleMap::pack_xy(1, 1));
    opened.insert(ObstacleMap::pack_xy(2, 2));

    assert!(map.check_cells_free(&[(1, 1), (2, 2)], Some(&opened)));
    assert!(!map.check_cells_free(&[(1, 1), (6, 6)], Some(&opened)));
}

#[test]
fn history_costs_can_be_accumulated_and_cleared() {
    let mut map = ObstacleMap::new(5, 5);

    assert!(map.add_history_cost(2, 2, 5));
    assert!(map.add_history_cost(2, 2, 7));
    assert!(!map.add_history_cost(5, 2, 9));
    assert_eq!(map.get_history_cost(2, 2), 12);
    assert_eq!(map.get_history_cost(5, 2), 0);

    map.clear_history();
    assert_eq!(map.get_history_cost(2, 2), 0);
}

#[test]
fn waveguide_path_helper_splits_core_and_clearance() {
    let map = ObstacleMap::new(5, 5);
    let footprint = map.waveguide_path_to_blocked_cells(&[(2, 2)], 1, ClearanceMetric::Manhattan);

    assert!(footprint.core_cells.contains(&ObstacleMap::pack_xy(2, 2)));
    assert!(footprint
        .blocked_cells
        .contains(&ObstacleMap::pack_xy(2, 2)));
    assert!(!footprint
        .clearance_cells
        .contains(&ObstacleMap::pack_xy(2, 2)));
    assert!(footprint
        .clearance_cells
        .contains(&ObstacleMap::pack_xy(1, 2)));
    assert_eq!(footprint.blocked_cells.len(), 5);
}
