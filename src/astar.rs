//! Single-net stateful A* router.
//!
//! This is the first routing kernel: one source state, one target state, and
//! primitive-based photonic transitions over the existing obstacle map.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::{Primitive, PrimitiveLibrary};

/// Router search state: grid position plus 45-degree heading index.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct State {
    pub x: i32,
    pub y: i32,
    pub angle: u8,
}

impl State {
    pub fn new(x: i32, y: i32, angle: u8) -> Self {
        Self {
            x,
            y,
            angle: angle % 8,
        }
    }
}

/// Configuration for the first single-net A* router.
#[derive(Clone, Debug)]
pub struct AStarConfig {
    pub max_iterations: usize,
    pub bend_weight: f64,
    pub target_tolerance_cells: i32,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100_000,
            bend_weight: 1.0,
            target_tolerance_cells: 0,
        }
    }
}

/// Result of one successful single-net route.
#[derive(Clone, Debug)]
pub struct RouteResult {
    pub states: Vec<State>,
    pub primitives: Vec<u16>,
    pub cells: Vec<(i32, i32)>,
    pub total_length_um: f64,
    pub total_cost: f64,
}

#[derive(Clone, Debug)]
struct Parent {
    previous: State,
    primitive: Primitive,
}

#[derive(Clone, Copy, Debug)]
struct OpenEntry {
    f_score: f64,
    g_score: f64,
    counter: u64,
    state: State,
}

impl Eq for OpenEntry {}

impl PartialEq for OpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score && self.counter == other.counter
    }
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                other
                    .g_score
                    .partial_cmp(&self.g_score)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
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

/// Route a single net with explicit A* settings.
pub fn route_single_net_with_config(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
) -> Option<RouteResult> {
    if config.target_tolerance_cells != 0 {
        return None;
    }
    if !obstacle_map.in_bounds(source.x, source.y) || !obstacle_map.in_bounds(target.x, target.y) {
        return None;
    }

    let mut open_set = BinaryHeap::new();
    let mut g_costs: FxHashMap<State, f64> = FxHashMap::default();
    let mut parents: FxHashMap<State, Parent> = FxHashMap::default();
    let mut closed: FxHashSet<State> = FxHashSet::default();
    let mut counter = 0u64;

    g_costs.insert(source, 0.0);
    open_set.push(OpenEntry {
        f_score: heuristic(source, target, primitives.grid_size_um()),
        g_score: 0.0,
        counter,
        state: source,
    });
    counter += 1;

    let mut iterations = 0usize;
    while let Some(entry) = open_set.pop() {
        iterations += 1;
        if iterations > config.max_iterations {
            return None;
        }

        let state = entry.state;
        if closed.contains(&state) {
            continue;
        }
        if target_reached(state, target, config.target_tolerance_cells) {
            return Some(reconstruct_route(source, target, &parents, entry.g_score));
        }
        closed.insert(state);

        let current_g = *g_costs.get(&state).unwrap_or(&f64::INFINITY);
        for primitive in primitives.get_primitives_for_angle(state.angle) {
            let next_state = State::new(
                state.x.checked_add(primitive.dx)?,
                state.y.checked_add(primitive.dy)?,
                primitive.end_angle,
            );

            if closed.contains(&next_state) {
                continue;
            }
            if !obstacle_map.in_bounds(next_state.x, next_state.y) {
                continue;
            }
            if !obstacle_map.check_primitive_footprint_free(
                state.x,
                state.y,
                &primitive.footprint,
                port_open_cells,
            ) {
                continue;
            }

            let step_cost = primitive.length_um + config.bend_weight * primitive.bend_cost;
            let tentative_g = current_g + step_cost;
            if tentative_g >= *g_costs.get(&next_state).unwrap_or(&f64::INFINITY) {
                continue;
            }

            parents.insert(
                next_state,
                Parent {
                    previous: state,
                    primitive: primitive.clone(),
                },
            );
            g_costs.insert(next_state, tentative_g);
            open_set.push(OpenEntry {
                f_score: tentative_g + heuristic(next_state, target, primitives.grid_size_um()),
                g_score: tentative_g,
                counter,
                state: next_state,
            });
            counter += 1;
        }
    }

    None
}

/// Export an SVG string showing obstacles and a routed path.
pub fn export_route_svg(obstacle_map: &ObstacleMap, route_result: &RouteResult) -> String {
    let width = obstacle_map.width();
    let height = obstacle_map.height();
    if width <= 0 || height <= 0 {
        return r#"<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1" />"#.to_string();
    }

    let cell_px = (1200 / width.max(height).max(1)).clamp(1, 8);
    let width_px = width * cell_px;
    let height_px = height * cell_px;
    let mut svg = String::new();

    svg.push_str(&format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" viewBox="0 0 {width} {height}">"#
    ));
    svg.push_str(r##"<rect width="100%" height="100%" fill="#eeeeee" />"##);
    svg.push_str(r#"<path d=""#);
    for x in 0..=width {
        svg.push_str(&format!("M {x} 0 V {height} "));
    }
    for y in 0..=height {
        svg.push_str(&format!("M 0 {y} H {width} "));
    }
    svg.push_str(
        r##"" stroke="#3fa34d" stroke-width="0.025" vector-effect="non-scaling-stroke" opacity="0.75" fill="none" />"##,
    );

    for x in 0..width {
        for y in 0..height {
            if obstacle_map.is_blocked(x, y) {
                let svg_y = height - y - 1;
                svg.push_str(&format!(
                    r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#000000" />"##
                ));
            }
        }
    }

    for &(x, y) in &route_result.cells {
        if obstacle_map.in_bounds(x, y) {
            let svg_y = height - y - 1;
            svg.push_str(&format!(
                r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#1a73e8" />"##
            ));
        }
    }

    svg.push_str("</svg>\n");
    svg
}

fn target_reached(state: State, target: State, target_tolerance_cells: i32) -> bool {
    state.angle == target.angle
        && (state.x - target.x).abs() <= target_tolerance_cells
        && (state.y - target.y).abs() <= target_tolerance_cells
}

fn heuristic(state: State, target: State, grid_size_um: f64) -> f64 {
    let dx = (target.x - state.x) as f64;
    let dy = (target.y - state.y) as f64;
    (dx * dx + dy * dy).sqrt() * grid_size_um
}

fn reconstruct_route(
    source: State,
    target: State,
    parents: &FxHashMap<State, Parent>,
    total_cost: f64,
) -> RouteResult {
    let mut states_reversed = vec![target];
    let mut primitive_steps_reversed = Vec::new();
    let mut current = target;

    while current != source {
        let parent = parents
            .get(&current)
            .expect("missing parent during route reconstruction");
        primitive_steps_reversed.push((parent.previous, parent.primitive.clone()));
        current = parent.previous;
        states_reversed.push(current);
    }

    states_reversed.reverse();
    primitive_steps_reversed.reverse();

    let mut primitive_ids = Vec::with_capacity(primitive_steps_reversed.len());
    let mut cells = Vec::new();
    let mut seen_cells = FxHashSet::default();
    let mut total_length_um = 0.0;

    for (origin, primitive) in primitive_steps_reversed {
        primitive_ids.push(primitive.id);
        total_length_um += primitive.length_um;

        for (dx, dy) in primitive.footprint {
            let cell = (origin.x + dx, origin.y + dy);
            if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                cells.push(cell);
            }
        }
    }

    RouteResult {
        states: states_reversed,
        primitives: primitive_ids,
        cells,
        total_length_um,
        total_cost,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obstacle_map::ObstacleMap;
    use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};

    fn primitive_library() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
        })
    }

    #[test]
    fn routes_straight_without_obstacles() {
        let map = ObstacleMap::new(10, 5);
        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 2, 0),
            State::new(5, 2, 0),
            None,
        )
        .expect("straight route should exist");

        assert_eq!(result.states.first().copied(), Some(State::new(1, 2, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(5, 2, 0)));
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
    fn footprint_collision_blocks_route() {
        let mut map = ObstacleMap::new(6, 3);
        map.add_static_cell(2, 1);

        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(3, 1, 0),
            None,
            &AStarConfig {
                max_iterations: 200,
                bend_weight: 1.0,
                target_tolerance_cells: 0,
            },
        );

        assert!(result.is_none());
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
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            Some(&opened),
        )
        .expect("opened port cells should be routable");

        assert_eq!(result.states.last().copied(), Some(State::new(5, 1, 0)));
    }

    #[test]
    fn exports_route_svg() {
        let map = ObstacleMap::new(6, 3);
        let result = route_single_net(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
        )
        .expect("route should exist");

        let svg = export_route_svg(&map, &result);
        assert!(svg.contains("<svg"));
        assert!(svg.contains("#1a73e8"));
    }
}
