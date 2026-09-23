//! The simple-route shortcut: straight/L/Z candidate generation and
//! conversion to a `RouteResult`, tried before falling back to full A* in
//! the plain (no dynamic expansion, no crossing) search path. Moved out of
//! `src/astar.rs` (Milestone 3, Slice 2); pure code motion, no behaviour
//! change.

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::{PrimitiveGeometry, PrimitiveLibrary};
use crate::search::astar::config::AStarConfig;
use crate::search::geometry::{
    compress_grid_waypoints, polyline_self_intersects, push_if_different,
};
use crate::search::state::{
    find_primitive, grid_point_from_state, RouteResult, RouteSearchStats, State,
};
use crate::simple_routes::{
    direction_between_octant as simple_direction_between, expand_candidate_to_grid_points,
    try_45_degree_straight_l_or_z_candidate_with_config,
    try_45_degree_straight_l_or_z_candidate_with_dynamic_expansion_config,
    try_straight_l_or_z_candidate_with_config,
    try_straight_l_or_z_candidate_with_dynamic_expansion_config, SimpleRouteCandidate,
    SimpleZRouteConfig,
};
use rustc_hash::FxHashSet;

/// Route a single net with explicit A* settings.
pub fn try_simple_route_with_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
) -> Option<RouteResult> {
    if !config.enable_simple_routes {
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

    let bend_radius_cells = infer_bend_radius_cells(primitives).unwrap_or(0);
    let terminal_straight_cells = if config.require_terminal_straights && bend_radius_cells > 0 {
        1
    } else {
        0
    };
    let z_config = SimpleZRouteConfig {
        max_offset_cells: config.simple_route_max_offset_cells,
        include_zero_offset: true,
        min_leg_len_cells: config
            .simple_route_min_leg_len_cells
            .max(bend_radius_cells + terminal_straight_cells),
    };
    let candidate = if primitive_library_allows_45_degree_turns(primitives) {
        try_45_degree_straight_l_or_z_candidate_with_config(
            source,
            target,
            obstacle_map,
            Some(&anchor_open_cells),
            &z_config,
        )
    } else {
        try_straight_l_or_z_candidate_with_config(
            source,
            target,
            obstacle_map,
            Some(&anchor_open_cells),
            &z_config,
        )
    }?;
    simple_candidate_to_route_result(
        &candidate,
        source,
        target,
        primitives,
        RouteSearchStats::default(),
        config.require_terminal_straights,
    )
}

pub fn try_simple_route_with_dynamic_expansion_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    dynamic_expansion_radius_cells: i32,
    dynamic_clearance_exempt_cells: Option<&FxHashSet<CellKey>>,
) -> Option<RouteResult> {
    if !config.enable_simple_routes {
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

    let bend_radius_cells = infer_bend_radius_cells(primitives).unwrap_or(0);
    let terminal_straight_cells = if config.require_terminal_straights && bend_radius_cells > 0 {
        1
    } else {
        0
    };
    let z_config = SimpleZRouteConfig {
        max_offset_cells: config.simple_route_max_offset_cells,
        include_zero_offset: true,
        min_leg_len_cells: config
            .simple_route_min_leg_len_cells
            .max(bend_radius_cells + terminal_straight_cells),
    };
    let candidate = if primitive_library_allows_45_degree_turns(primitives) {
        try_45_degree_straight_l_or_z_candidate_with_dynamic_expansion_config(
            source,
            target,
            obstacle_map,
            Some(&anchor_open_cells),
            &z_config,
            dynamic_expansion_radius_cells,
            dynamic_clearance_exempt_cells,
        )
    } else {
        try_straight_l_or_z_candidate_with_dynamic_expansion_config(
            source,
            target,
            obstacle_map,
            Some(&anchor_open_cells),
            &z_config,
            dynamic_expansion_radius_cells,
            dynamic_clearance_exempt_cells,
        )
    }?;
    simple_candidate_to_route_result(
        &candidate,
        source,
        target,
        primitives,
        RouteSearchStats::default(),
        config.require_terminal_straights,
    )
}

pub(crate) fn primitive_library_allows_45_degree_turns(primitives: &PrimitiveLibrary) -> bool {
    primitives
        .get_primitives_for_angle(0)
        .iter()
        .any(|primitive| match &primitive.geometry {
            PrimitiveGeometry::Bend { angle_delta, .. } => angle_delta.unsigned_abs() == 1,
            PrimitiveGeometry::Straight { .. } => false,
        })
}

pub(crate) fn simple_candidate_to_route_result(
    candidate: &SimpleRouteCandidate,
    source: State,
    target: State,
    primitives: &PrimitiveLibrary,
    stats: RouteSearchStats,
    require_terminal_straights: bool,
) -> Option<RouteResult> {
    let expanded = expand_candidate_to_grid_points(candidate);
    if expanded.len() < 2 || candidate.points.len() < 2 {
        return None;
    }
    let start_point = grid_point_from_state(source);
    let end_point = grid_point_from_state(target);
    if expanded.first().copied() != Some(start_point) || expanded.last().copied() != Some(end_point)
    {
        return None;
    }

    let segment_count = candidate.points.len() - 1;
    let mut headings = Vec::with_capacity(segment_count);
    let mut segment_lengths = Vec::with_capacity(segment_count);
    for i in 0..segment_count {
        let a = candidate.points[i];
        let b = candidate.points[i + 1];
        headings.push(simple_direction_between(a, b)?);
        segment_lengths.push((b.x - a.x).abs().max((b.y - a.y).abs()));
    }

    let bend_radius_cells = infer_bend_radius_cells(primitives).unwrap_or(0);
    let min_bend_adjacent_len = bend_radius_cells;
    if bend_radius_cells > 0 {
        for (idx, &length) in segment_lengths.iter().enumerate() {
            let has_prev_bend = idx > 0 && headings[idx - 1] != headings[idx];
            let has_next_bend = idx + 1 < segment_count && headings[idx] != headings[idx + 1];
            let required_len = if has_prev_bend && has_next_bend {
                2 * min_bend_adjacent_len
            } else if has_prev_bend || has_next_bend {
                min_bend_adjacent_len
            } else {
                0
            };
            if length < required_len {
                return None;
            }
        }
    }
    let mut trimmed_lengths = Vec::with_capacity(segment_count);
    for (idx, &length) in segment_lengths.iter().enumerate() {
        let mut trimmed = length;
        if idx > 0 {
            trimmed -= bend_radius_cells;
        }
        if idx + 1 < segment_count {
            trimmed -= bend_radius_cells;
        }
        if trimmed < 0 {
            return None;
        }
        trimmed_lengths.push(trimmed);
    }
    if require_terminal_straights && segment_count > 1 {
        if trimmed_lengths.first().copied().unwrap_or(0) == 0
            || trimmed_lengths.last().copied().unwrap_or(0) == 0
        {
            return None;
        }
    }

    if segment_count == 3 && bend_radius_cells > 0 {
        // Two bends consume one bend radius from both ends of the middle leg.
        if segment_lengths[1] < 2 * bend_radius_cells {
            return None;
        }
    }

    let mut states = vec![source];
    let mut primitive_ids = Vec::new();
    let mut total_length_um = 0.0;
    let mut current = source;

    for i in 0..segment_count {
        let straight_cells = trimmed_lengths[i];
        if straight_cells > 0 {
            let straight_primitive_ids =
                decompose_straight_cells(current.angle, straight_cells, primitives)?;
            for primitive_id in straight_primitive_ids {
                let primitive = find_primitive(primitives, current.angle, primitive_id)?;
                primitive_ids.push(primitive.id);
                total_length_um += primitive.length_um;
                current = State::new(
                    current.x.checked_add(primitive.dx)?,
                    current.y.checked_add(primitive.dy)?,
                    primitive.end_angle,
                );
                states.push(current);
            }
        }

        if i + 1 < segment_count {
            let delta = turn_delta(headings[i], headings[i + 1])?;
            let bend_primitive_id =
                find_bend_primitive_id(current.angle, delta, bend_radius_cells, primitives)?;
            let primitive = find_primitive(primitives, current.angle, bend_primitive_id)?;
            primitive_ids.push(primitive.id);
            total_length_um += primitive.length_um;
            current = State::new(
                current.x.checked_add(primitive.dx)?,
                current.y.checked_add(primitive.dy)?,
                primitive.end_angle,
            );
            states.push(current);
        }
    }

    if current != target {
        return None;
    }

    let mut cells = Vec::new();
    let mut seen_cells = FxHashSet::default();
    let mut ordered_path = Vec::new();
    push_if_different(&mut ordered_path, (source.x, source.y));
    for (idx, primitive_id) in primitive_ids.iter().copied().enumerate() {
        let origin = states[idx];
        let primitive = find_primitive(primitives, origin.angle, primitive_id)?;
        for (dx, dy) in primitive.footprint.iter().copied() {
            let cell = (origin.x + dx, origin.y + dy);
            push_if_different(&mut ordered_path, cell);
            if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                cells.push(cell);
            }
        }
    }
    push_if_different(&mut ordered_path, (target.x, target.y));

    let compressed_waypoints = compress_grid_waypoints(&ordered_path);
    if polyline_self_intersects(&compressed_waypoints) {
        return None;
    }
    Some(RouteResult {
        states,
        primitives: primitive_ids,
        cells,
        compressed_waypoints,
        total_length_um,
        total_cost: total_length_um,
        requested_target: target,
        reached_target: target,
        stats,
    })
}

pub(crate) fn infer_bend_radius_cells(primitives: &PrimitiveLibrary) -> Option<i32> {
    let grid_size = primitives.grid_size_um();
    if grid_size <= 0.0 {
        return None;
    }

    for angle in 0..8u8 {
        for primitive in primitives.get_primitives_for_angle(angle) {
            if let PrimitiveGeometry::Bend {
                radius_um,
                angle_delta,
            } = primitive.geometry
            {
                if angle_delta.unsigned_abs() == 2 {
                    let cells = (radius_um / grid_size).round() as i32;
                    if cells > 0 {
                        return Some(cells);
                    }
                }
            }
        }
    }
    None
}

pub(crate) fn decompose_straight_cells(
    start_angle: u8,
    total_cells: i32,
    primitives: &PrimitiveLibrary,
) -> Option<Vec<u16>> {
    if total_cells < 0 {
        return None;
    }
    if total_cells == 0 {
        return Some(Vec::new());
    }

    let mut options: Vec<(usize, u16)> = primitives
        .get_primitives_for_angle(start_angle)
        .iter()
        .filter_map(|primitive| {
            if let PrimitiveGeometry::Straight { .. } = primitive.geometry {
                if primitive.end_angle != start_angle {
                    return None;
                }
                let cells = primitive.dx.abs().max(primitive.dy.abs()) as usize;
                if cells == 0 {
                    return None;
                }
                Some((cells, primitive.id))
            } else {
                None
            }
        })
        .collect();
    options.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.cmp(&b.1)));
    options.dedup();
    if options.is_empty() {
        return None;
    }

    let target = usize::try_from(total_cells).ok()?;
    let mut best_count: Vec<usize> = vec![usize::MAX; target + 1];
    let mut prev_sum: Vec<usize> = vec![usize::MAX; target + 1];
    let mut prev_opt: Vec<usize> = vec![usize::MAX; target + 1];
    best_count[0] = 0;

    for sum in 0..=target {
        if best_count[sum] == usize::MAX {
            continue;
        }
        for (opt_idx, (cells, _)) in options.iter().copied().enumerate() {
            let next = sum + cells;
            if next > target {
                continue;
            }
            let candidate_count = best_count[sum] + 1;
            if candidate_count < best_count[next] {
                best_count[next] = candidate_count;
                prev_sum[next] = sum;
                prev_opt[next] = opt_idx;
            }
        }
    }

    if best_count[target] == usize::MAX {
        return None;
    }

    let mut ids_reversed = Vec::new();
    let mut cur = target;
    while cur > 0 {
        let opt_idx = prev_opt[cur];
        if opt_idx == usize::MAX {
            return None;
        }
        ids_reversed.push(options[opt_idx].1);
        cur = prev_sum[cur];
    }
    ids_reversed.reverse();
    Some(ids_reversed)
}

pub(crate) fn turn_delta(from: u8, to: u8) -> Option<i8> {
    let delta = (to as i16 - from as i16).rem_euclid(8) as u8;
    match delta {
        1 => Some(1),
        2 => Some(2),
        6 => Some(-2),
        7 => Some(-1),
        _ => None,
    }
}

pub(crate) fn find_bend_primitive_id(
    start_angle: u8,
    angle_delta: i8,
    bend_radius_cells: i32,
    primitives: &PrimitiveLibrary,
) -> Option<u16> {
    let grid_size = primitives.grid_size_um();
    let mut candidates = primitives
        .get_primitives_for_angle(start_angle)
        .iter()
        .filter_map(|primitive| {
            if let PrimitiveGeometry::Bend {
                radius_um,
                angle_delta: primitive_delta,
            } = primitive.geometry
            {
                if primitive_delta != angle_delta {
                    return None;
                }
                let cells = (radius_um / grid_size).round() as i32;
                if bend_radius_cells > 0 && cells != bend_radius_cells {
                    return None;
                }
                Some(primitive.id)
            } else {
                None
            }
        })
        .collect::<Vec<_>>();
    candidates.sort_unstable();
    candidates.into_iter().next()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::geometry_realization::{route_to_primitive_centerline, GeometryGridSpec};
    use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};
    use crate::search::astar::config::{HeapTieBreaker, PrimitiveOrdering};
    use crate::search::astar::route_single_net_with_config;
    use crate::search::test_support::*;

    #[test]
    fn simple_straight_route_used_before_astar() {
        let map = ObstacleMap::new(10, 6);
        let result = route_single_net_with_config(
            &map,
            &primitive_library_no45_bend2(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple straight route should exist");
        assert_eq!(result.compressed_waypoints, vec![(1, 1), (5, 1)]);
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_l_route_used_before_astar() {
        let map = ObstacleMap::new(10, 10);
        let result = route_single_net_with_config(
            &map,
            &primitive_library_no45_bend2(),
            State::new(1, 1, 0),
            State::new(5, 4, 2),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple L route should exist");
        assert_eq!(result.compressed_waypoints, vec![(1, 1), (5, 1), (5, 4)]);
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_z_route_used_before_astar() {
        let map = ObstacleMap::new(20, 20);
        let result = route_single_net_with_config(
            &map,
            &primitive_library_no45_bend2(),
            State::new(1, 1, 0),
            State::new(10, 10, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple Z route should exist");
        assert_eq!(
            result.compressed_waypoints,
            vec![(1, 1), (5, 1), (5, 10), (10, 10)]
        );
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_z_route_rejects_too_short_middle_leg() {
        let map = ObstacleMap::new(20, 20);
        let library = primitive_library_no45_bend2();
        let mut config = AStarConfig::default();
        config.enable_simple_routes = true;

        let result = try_simple_route_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(10, 4, 0),
            None,
            &config,
        );

        assert!(result.is_none());
    }

    #[test]
    fn simple_z_route_has_primitives_and_replay_centerline() {
        let map = ObstacleMap::new(20, 20);
        let library = primitive_library_no45_bend1();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 4, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple Z route should exist");

        assert_eq!(result.stats.expanded_states, 0);
        assert!(!result.primitives.is_empty());
        assert_eq!(result.states.len(), result.primitives.len() + 1);

        let grid = GeometryGridSpec::new(1.0, 0.0, 0.0).expect("grid spec");
        let centerline = route_to_primitive_centerline(&result, &library, &grid)
            .expect("primitive replay centerline should succeed");
        assert!(centerline.len() >= 2);
    }

    #[test]
    fn simple_45_diagonal_straight_route_used_before_astar() {
        let map = ObstacleMap::new(10, 10);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 1),
            State::new(5, 5, 1),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple 45-degree straight route should exist");
        assert_eq!(result.compressed_waypoints, vec![(1, 1), (5, 5)]);
        assert_eq!(result.states.last().copied(), Some(State::new(5, 5, 1)));
        assert_eq!(result.stats.expanded_states, 0);
        assert!(!result.primitives.is_empty());
    }

    #[test]
    fn simple_45_l_route_used_before_astar() {
        let map = ObstacleMap::new(12, 8);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(7, 4, 1),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple 45-degree L route should exist");
        assert_eq!(result.compressed_waypoints, vec![(1, 1), (4, 1), (7, 4)]);
        assert_eq!(result.states.last().copied(), Some(State::new(7, 4, 1)));
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_45_z_route_used_before_astar() {
        let map = ObstacleMap::new(14, 8);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(9, 5, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple 45-degree Z route should exist");
        assert_eq!(
            result.compressed_waypoints,
            vec![(1, 1), (3, 1), (7, 5), (9, 5)]
        );
        assert_eq!(result.states.last().copied(), Some(State::new(9, 5, 0)));
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_45_z_route_can_use_alternative_lane_when_preferred_is_blocked() {
        let mut map = ObstacleMap::new(14, 8);
        map.add_static_cell(5, 3);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(9, 5, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("alternative simple 45-degree Z route should exist");
        assert_eq!(result.stats.expanded_states, 0);
        assert_eq!(result.states.last().copied(), Some(State::new(9, 5, 0)));
        assert_ne!(
            result.compressed_waypoints,
            vec![(1, 1), (3, 1), (7, 5), (9, 5)]
        );
        assert!(!result.cells.iter().any(|&(x, y)| map.is_blocked(x, y)));
    }

    #[test]
    fn simple_route_disabled_uses_astar() {
        let map = ObstacleMap::new(10, 6);
        let result = route_single_net_with_config(
            &map,
            &primitive_library_no45_bend2(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("A* route should exist with simple routes disabled");
        assert!(result.stats.expanded_states > 0);
    }

    #[test]
    fn default_search_experiments_stay_gated_off() {
        let config = AStarConfig::default();
        assert!(!config.use_indexed_heap);
        assert_eq!(config.primitive_ordering, PrimitiveOrdering::Library);
        assert_eq!(config.heap_tie_breaker, HeapTieBreaker::SmallerG);
    }

    #[test]
    fn simple_route_blocked_falls_back_to_astar() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("A* fallback should route around blocked simple path");
        assert!(result.stats.expanded_states > 0);
    }

    #[test]
    fn simple_route_respects_opened_cells() {
        let mut map = ObstacleMap::new(10, 6);
        map.add_static_cell(1, 1);
        map.add_static_cell(5, 1);
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(1, 1));
        opened.insert(pack_xy(5, 1));

        let result = route_single_net_with_config(
            &map,
            &primitive_library_no45_bend2(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            Some(&opened),
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple route should allow opened endpoint cells");
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_route_handles_same_heading_turnaround() {
        let map = ObstacleMap::new(220, 140);
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 8,
            bend_radius_cells: 20,
            allow_45_degree_turns: false,
        });
        let mut config = AStarConfig::default();
        config.simple_route_max_offset_cells = 120;

        let result = try_simple_route_with_config(
            &map,
            &library,
            State::new(120, 70, 0),
            State::new(40, 40, 0),
            None,
            &config,
        )
        .expect("turnaround simple route should be generated");

        assert_eq!(result.states.first().copied(), Some(State::new(120, 70, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(40, 40, 0)));
    }

    #[test]
    fn simple_route_accepts_two_bend_z_with_minimum_middle_leg() {
        let map = ObstacleMap::new(360, 180);
        let library = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 0.5,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 20,
            allow_45_degree_turns: false,
        });
        let mut config = AStarConfig::default();
        config.simple_route_max_offset_cells = 240;

        let result = try_simple_route_with_config(
            &map,
            &library,
            State::new(121, 65, 0),
            State::new(301, 106, 0),
            None,
            &config,
        )
        .expect("two-bend Z route with a 2R+1 middle leg should be simple");

        assert_eq!(result.states.first().copied(), Some(State::new(121, 65, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(301, 106, 0)));
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_route_rejects_blocked_middle_cell() {
        let mut map = ObstacleMap::new(7, 1);
        map.add_static_cell(3, 0);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 0, 0),
            State::new(5, 0, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn opened_cells_can_unblock_explicit_static_cells() {
        let mut map = ObstacleMap::new(7, 1);
        map.add_static_cell(3, 0);
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(3, 0));
        let result = route_single_net_with_config(
            &map,
            &primitive_library_no45_bend1(),
            State::new(1, 0, 0),
            State::new(5, 0, 0),
            Some(&opened),
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        );
        let route = result.expect("opened cells should allow exact static overlap");
        assert_eq!(route.stats.expanded_states, 0);
    }

    #[test]
    fn source_and_target_cells_are_opened_implicitly() {
        let mut map = ObstacleMap::new(7, 1);
        map.add_static_cell(1, 0);
        map.add_static_cell(5, 0);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 0, 0),
            State::new(5, 0, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        )
        .expect("blocked endpoints should be routed using implicit anchor opening");
        assert_eq!(result.states.first().copied(), Some(State::new(1, 0, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(5, 0, 0)));
    }

    #[test]
    fn disabling_simple_routes_preserves_old_behavior() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(5, 2, 0),
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("A* should still find the old straight route");
        assert!(!result.primitives.is_empty());
        assert!(result.stats.expanded_states > 0);
    }
}
