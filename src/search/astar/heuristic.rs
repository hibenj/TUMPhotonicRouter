//! The A* heuristic: plain and diagonal distance estimates, the minimum
//! unavoidable bend cost, and the heading-aware `SearchHeuristic` (terminal
//! approach acceptance rings and target-angle acceptance). Moved out of
//! `src/astar.rs` (Milestone 3, Slice 2); pure code motion, no behaviour
//! change.

use crate::primitives::{PrimitiveLibrary, DIRECTIONS};
use crate::search::astar::config::{AStarConfig, HeuristicMode, SearchHeuristicMode};
use crate::search::state::State;
use std::cmp::Ordering;

pub(crate) fn distance_heuristic(state: State, target: State, grid_size_um: f64) -> f64 {
    let dx = (target.x - state.x) as f64;
    let dy = (target.y - state.y) as f64;
    (dx * dx + dy * dy).sqrt() * grid_size_um
}

pub(crate) fn diagonal_distance_heuristic(state: State, target: State, grid_size_um: f64) -> f64 {
    let dx = (target.x - state.x).abs() as f64;
    let dy = (target.y - state.y).abs() as f64;
    let diagonal = dx.min(dy);
    let straight = dx.max(dy) - diagonal;
    (straight + diagonal * 2.0_f64.sqrt()) * grid_size_um
}

pub(crate) fn direction_reaches_target_ray(state: State, target: State, tolerance: i32) -> bool {
    let dx = target.x - state.x;
    let dy = target.y - state.y;
    if dx.abs() <= tolerance && dy.abs() <= tolerance {
        return true;
    }
    let (dir_x, dir_y) = DIRECTIONS[(state.angle % 8) as usize];
    match (dir_x, dir_y) {
        (0, 0) => false,
        (0, _) => dx.abs() <= tolerance && dy.signum() == dir_y,
        (_, 0) => dy.abs() <= tolerance && dx.signum() == dir_x,
        _ => {
            dx.signum() == dir_x && dy.signum() == dir_y && (dx.abs() - dy.abs()).abs() <= tolerance
        }
    }
}

pub(crate) fn minimum_positive_bend_cost(primitives: &PrimitiveLibrary, bend_weight: f64) -> f64 {
    if bend_weight <= 0.0 {
        return 0.0;
    }
    let min_bend_cost = (0u8..8u8)
        .flat_map(|angle| primitives.get_primitives_for_angle(angle).iter())
        .filter_map(|primitive| {
            if primitive.bend_cost > 0.0 {
                Some(primitive.bend_cost)
            } else {
                None
            }
        })
        .min_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    min_bend_cost.map_or(0.0, |cost| cost * bend_weight)
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct TerminalApproach {
    pub(crate) predecessor_angle: u8,
    pub(crate) dx: i32,
    pub(crate) dy: i32,
    pub(crate) cost: f64,
}

impl Default for TerminalApproach {
    fn default() -> Self {
        Self {
            predecessor_angle: 0,
            dx: 0,
            dy: 0,
            cost: f64::INFINITY,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct SearchHeuristic {
    pub(crate) target: State,
    pub(crate) grid_size_um: f64,
    pub(crate) mode: SearchHeuristicMode,
    pub(crate) weight: f64,
}

impl SearchHeuristic {
    pub(crate) fn new(target: State, primitives: &PrimitiveLibrary, config: &AStarConfig) -> Self {
        let target_angle_ok = target_angle_acceptance(target, config);
        let minimum_bend_cost = match config.heuristic_mode {
            HeuristicMode::Distance => 0.0,
            HeuristicMode::HeadingAware | HeuristicMode::DiagonalAware => {
                minimum_positive_bend_cost(primitives, config.bend_weight)
            }
        };
        let mode = match (config.heuristic_mode, minimum_bend_cost > 0.0) {
            (HeuristicMode::DiagonalAware, true) => {
                let (terminal_approaches, terminal_approach_count) =
                    terminal_approaches_to_target(primitives, config.bend_weight, target_angle_ok);
                SearchHeuristicMode::DiagonalAware {
                    minimum_bend_cost,
                    tolerance: config.target_tolerance_cells.max(0),
                    target_angle_ok,
                    terminal_approaches,
                    terminal_approach_count,
                }
            }
            (HeuristicMode::HeadingAware, true) => SearchHeuristicMode::HeadingAware {
                minimum_bend_cost,
                tolerance: config.target_tolerance_cells.max(0),
                target_angle_ok,
            },
            _ => SearchHeuristicMode::Distance,
        };
        Self {
            target,
            grid_size_um: primitives.grid_size_um(),
            mode,
            weight: config.heuristic_weight.max(0.0),
        }
    }

    /// The identically-zero heuristic: a `Distance` estimate with weight
    /// 0, so `estimate` returns `0.0` for every state and the shared
    /// expansion helpers' `f = g + h` becomes `f = g`. This is what turns
    /// the A* kernel's building blocks into the uniform-cost search
    /// `crate::search::grid_dijkstra` runs.
    pub(crate) fn zero(target: State, primitives: &PrimitiveLibrary) -> Self {
        Self {
            target,
            grid_size_um: primitives.grid_size_um(),
            mode: SearchHeuristicMode::Distance,
            weight: 0.0,
        }
    }

    pub(crate) fn estimate(&self, state: State) -> f64 {
        match &self.mode {
            SearchHeuristicMode::Distance => {
                self.weight * distance_heuristic(state, self.target, self.grid_size_um)
            }
            SearchHeuristicMode::HeadingAware {
                minimum_bend_cost,
                tolerance,
                target_angle_ok,
            } => {
                let distance = distance_heuristic(state, self.target, self.grid_size_um);
                let estimate = if !target_angle_ok[(state.angle % 8) as usize]
                    || !direction_reaches_target_ray(state, self.target, *tolerance)
                {
                    distance + *minimum_bend_cost
                } else {
                    distance
                };
                self.weight * estimate
            }
            SearchHeuristicMode::DiagonalAware {
                minimum_bend_cost,
                tolerance,
                target_angle_ok,
                terminal_approaches,
                terminal_approach_count,
            } => {
                let distance = diagonal_distance_heuristic(state, self.target, self.grid_size_um);
                let dx = (self.target.x - state.x).abs();
                let dy = (self.target.y - state.y).abs();
                let diagonal_correction_needed = dx > *tolerance
                    && dy > *tolerance
                    && (state.angle % 2 == 0)
                    && !direction_reaches_target_ray(state, self.target, *tolerance);
                let estimate = if !target_angle_ok[(state.angle % 8) as usize]
                    || !direction_reaches_target_ray(state, self.target, *tolerance)
                    || diagonal_correction_needed
                {
                    distance + *minimum_bend_cost
                } else {
                    distance
                };
                let terminal_estimate = terminal_approach_estimate(
                    state,
                    self.target,
                    self.grid_size_um,
                    *minimum_bend_cost,
                    *tolerance,
                    terminal_approaches,
                    *terminal_approach_count,
                );
                self.weight * estimate.max(terminal_estimate)
            }
        }
    }
}

pub(crate) fn terminal_approaches_to_target(
    primitives: &PrimitiveLibrary,
    bend_weight: f64,
    target_angle_ok: [bool; 8],
) -> ([TerminalApproach; 64], usize) {
    let mut approaches = [TerminalApproach::default(); 64];
    let mut count = 0usize;
    for angle in 0u8..8u8 {
        for primitive in primitives.get_primitives_for_angle(angle) {
            if !target_angle_ok[(primitive.end_angle % 8) as usize] || count >= approaches.len() {
                continue;
            }
            approaches[count] = TerminalApproach {
                predecessor_angle: primitive.start_angle,
                dx: primitive.dx,
                dy: primitive.dy,
                cost: primitive.length_um + bend_weight * primitive.bend_cost,
            };
            count += 1;
        }
    }
    (approaches, count)
}

pub(crate) fn terminal_approach_estimate(
    state: State,
    target: State,
    grid_size_um: f64,
    minimum_bend_cost: f64,
    tolerance: i32,
    terminal_approaches: &[TerminalApproach; 64],
    terminal_approach_count: usize,
) -> f64 {
    if state.x == target.x && state.y == target.y {
        return 0.0;
    }

    let mut best = f64::INFINITY;
    for approach in terminal_approaches
        .iter()
        .take(terminal_approach_count.min(terminal_approaches.len()))
    {
        let predecessor = State::new(
            target.x - approach.dx,
            target.y - approach.dy,
            approach.predecessor_angle,
        );
        let mut estimate = diagonal_distance_heuristic(state, predecessor, grid_size_um);
        if state.angle != predecessor.angle
            || !direction_reaches_target_ray(state, predecessor, tolerance)
        {
            estimate += minimum_bend_cost;
        }
        estimate += approach.cost;
        if estimate < best {
            best = estimate;
        }
    }
    best
}

pub(crate) fn target_angle_acceptance(target: State, config: &AStarConfig) -> [bool; 8] {
    let mut accepted = [true; 8];
    if let Some(mask) = config.allowed_target_angles_mask {
        for angle in 0u8..8u8 {
            accepted[angle as usize] = (mask & (1u8 << angle)) != 0;
        }
    } else if config.require_target_angle {
        accepted = [false; 8];
        accepted[(target.angle % 8) as usize] = true;
    }
    accepted
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::obstacle_map::ObstacleMap;
    use crate::search::astar::route_single_net_with_config;
    use crate::search::test_support::*;

    fn assert_heading_aware_matches_distance(
        map: &ObstacleMap,
        library: &PrimitiveLibrary,
        source: State,
        target: State,
        mut base_config: AStarConfig,
    ) {
        base_config.heuristic_mode = HeuristicMode::Distance;
        let distance =
            route_single_net_with_config(map, library, source, target, None, &base_config)
                .expect("distance heuristic route should exist");
        let heading_aware = route_single_net_with_config(
            map,
            library,
            source,
            target,
            None,
            &AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                heuristic_mode: HeuristicMode::HeadingAware,
                ..base_config
            },
        )
        .expect("heading-aware heuristic route should exist");

        assert_eq!(heading_aware.reached_target, distance.reached_target);
        assert!((heading_aware.total_cost - distance.total_cost).abs() < 1.0e-9);
    }

    #[test]
    fn heading_aware_heuristic_adds_only_unavoidable_minimum_bend_bound() {
        let library = primitive_library_no45_bend2();
        let min_bend_cost = minimum_positive_bend_cost(&library, 1.0);
        let config = AStarConfig {
            heuristic_mode: HeuristicMode::HeadingAware,
            require_target_angle: false,
            ..AStarConfig::default()
        };
        let straight_source = State::new(0, 0, 0);
        let straight_target = State::new(10, 0, 0);
        let straight_heuristic = SearchHeuristic::new(straight_target, &library, &config);
        assert_eq!(
            straight_heuristic.estimate(straight_source),
            distance_heuristic(straight_source, straight_target, 1.0)
        );

        let off_ray_target = State::new(10, 5, 0);
        let off_ray_heuristic = SearchHeuristic::new(off_ray_target, &library, &config);
        assert_eq!(
            off_ray_heuristic.estimate(straight_source),
            distance_heuristic(straight_source, off_ray_target, 1.0) + min_bend_cost
        );

        let target_angle_config = AStarConfig {
            heuristic_mode: HeuristicMode::HeadingAware,
            require_target_angle: true,
            ..AStarConfig::default()
        };
        let mismatched_angle_target = State::new(10, 0, 2);
        let mismatched_angle_heuristic =
            SearchHeuristic::new(mismatched_angle_target, &library, &target_angle_config);
        assert_eq!(
            mismatched_angle_heuristic.estimate(straight_source),
            distance_heuristic(straight_source, mismatched_angle_target, 1.0) + min_bend_cost
        );
    }

    #[test]
    fn diagonal_aware_heuristic_uses_octile_distance_and_minimum_bend_bound() {
        let library = primitive_library();
        let min_bend_cost = minimum_positive_bend_cost(&library, 1.0);
        let config = AStarConfig {
            heuristic_mode: HeuristicMode::DiagonalAware,
            require_target_angle: false,
            ..AStarConfig::default()
        };

        let cardinal_source = State::new(0, 0, 0);
        let diagonal_target = State::new(10, 5, 0);
        let heuristic = SearchHeuristic::new(diagonal_target, &library, &config);
        assert_eq!(
            heuristic.estimate(cardinal_source),
            diagonal_distance_heuristic(cardinal_source, diagonal_target, 1.0) + min_bend_cost
        );

        let diagonal_source = State::new(0, 0, 1);
        let same_ray_target = State::new(5, 5, 1);
        let same_ray_heuristic = SearchHeuristic::new(same_ray_target, &library, &config);
        assert_eq!(
            same_ray_heuristic.estimate(diagonal_source),
            diagonal_distance_heuristic(diagonal_source, same_ray_target, 1.0)
        );
    }

    #[test]
    fn heading_aware_heuristic_preserves_route_cost_on_forced_detour() {
        let mut map = ObstacleMap::new(180, 80);
        for y in 4..=72 {
            if !(42..=50).contains(&y) {
                map.add_static_cell(85, y);
            }
        }
        let library = primitive_library_no45_bend2();
        let source = State::new(12, 20, 0);
        let target = State::new(160, 20, 0);
        let base_config = AStarConfig {
            max_iterations: 500_000,
            require_target_angle: false,
            enable_simple_routes: false,
            routing_window_fallback_full_grid: true,
            ..AStarConfig::default()
        };
        assert_heading_aware_matches_distance(&map, &library, source, target, base_config);
    }

    #[test]
    fn heading_aware_heuristic_preserves_route_cost_on_simple_block() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 3);
        map.add_static_cell(4, 3);
        map.add_static_cell(5, 3);
        assert_heading_aware_matches_distance(
            &map,
            &primitive_library(),
            State::new(1, 3, 0),
            State::new(8, 3, 0),
            AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                enable_simple_routes: false,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        );
    }

    #[test]
    fn heading_aware_heuristic_preserves_route_cost_around_two_large_blocks() {
        let mut map = ObstacleMap::new(90, 50);
        for x in 18..=32 {
            for y in 8..=38 {
                map.add_static_cell(x, y);
            }
        }
        for x in 52..=66 {
            for y in 12..=42 {
                map.add_static_cell(x, y);
            }
        }
        assert_heading_aware_matches_distance(
            &map,
            &primitive_library_no45_bend2(),
            State::new(6, 25, 0),
            State::new(82, 25, 0),
            AStarConfig {
                diagnostics: KernelDiagnostics::default(),
                max_iterations: 500_000,
                require_target_angle: false,
                enable_simple_routes: false,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        );
    }
}
