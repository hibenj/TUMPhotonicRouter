//! Single-net stateful A* router.
//!
//! This is the first routing kernel: one source state, one target state, and
//! primitive-based photonic transitions over the existing obstacle map.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::PrimitiveLibrary;

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
    pub require_target_angle: bool,
    pub allowed_target_angles_mask: Option<u8>,
    pub use_routing_window: bool,
    pub routing_window_min_margin_cells: i32,
    pub routing_window_scale: f64,
    pub routing_window_max_expansions: u32,
    pub routing_window_fallback_full_grid: bool,
    pub routing_window_growth: f64,
}

impl Default for AStarConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100_000,
            bend_weight: 1.0,
            target_tolerance_cells: 0,
            require_target_angle: true,
            allowed_target_angles_mask: None,
            use_routing_window: true,
            routing_window_min_margin_cells: 12,
            routing_window_scale: 0.35,
            routing_window_max_expansions: 3,
            routing_window_fallback_full_grid: true,
            routing_window_growth: 0.5,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct RoutingBounds {
    min_x: i32,
    max_x: i32,
    min_y: i32,
    max_y: i32,
}

impl RoutingBounds {
    #[inline]
    fn contains(&self, x: i32, y: i32) -> bool {
        x >= self.min_x && x <= self.max_x && y >= self.min_y && y <= self.max_y
    }
}

/// Result of one successful single-net route.
#[derive(Clone, Debug)]
pub struct RouteResult {
    pub states: Vec<State>,
    pub primitives: Vec<u16>,
    pub cells: Vec<(i32, i32)>,
    pub compressed_waypoints: Vec<(i32, i32)>,
    pub total_length_um: f64,
    pub total_cost: f64,
    pub requested_target: State,
    pub reached_target: State,
    pub stats: RouteSearchStats,
}

#[derive(Clone, Debug, Default)]
pub struct RouteSearchStats {
    pub window_attempts: u32,
    pub used_full_grid_fallback: bool,
    pub expanded_states: usize,
    pub window_rejects: usize,
    pub footprint_rejects: usize,
    pub max_window_area_cells: i64,
}

#[derive(Clone, Debug)]
struct Parent {
    previous: State,
    primitive_id: u16,
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

    let mut stats = RouteSearchStats::default();
    if !config.use_routing_window {
        return route_single_net_with_bounds(
            obstacle_map,
            primitives,
            source,
            target,
            port_open_cells,
            config,
            None,
            &mut stats,
        );
    }

    for expansion_idx in 0..=config.routing_window_max_expansions {
        let bounds = compute_routing_bounds(obstacle_map, source, target, config, expansion_idx)?;
        stats.window_attempts += 1;
        stats.max_window_area_cells = stats.max_window_area_cells.max(window_area(bounds));

        if let Some(route) = route_single_net_with_bounds(
            obstacle_map,
            primitives,
            source,
            target,
            port_open_cells,
            config,
            Some(bounds),
            &mut stats,
        ) {
            return Some(route);
        }
    }

    if config.routing_window_fallback_full_grid {
        stats.window_attempts += 1;
        stats.used_full_grid_fallback = true;
        return route_single_net_with_bounds(
            obstacle_map,
            primitives,
            source,
            target,
            port_open_cells,
            config,
            None,
            &mut stats,
        );
    }

    None
}

fn route_single_net_with_bounds(
    obstacle_map: &ObstacleMap,
    primitives: &PrimitiveLibrary,
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
    config: &AStarConfig,
    routing_bounds: Option<RoutingBounds>,
    stats: &mut RouteSearchStats,
) -> Option<RouteResult> {
    if let Some(bounds) = routing_bounds {
        if !bounds.contains(source.x, source.y) || !bounds.contains(target.x, target.y) {
            return None;
        }
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
        if target_reached(state, target, config) {
            return Some(reconstruct_route(
                source,
                target,
                state,
                &parents,
                primitives,
                entry.g_score,
                stats.clone(),
            ));
        }
        closed.insert(state);
        stats.expanded_states += 1;

        let current_g = *g_costs.get(&state).unwrap_or(&f64::INFINITY);
        let primitive_bucket = primitives.get_primitives_for_angle(state.angle);
        for primitive in primitive_bucket.iter() {
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
            if let Some(bounds) = routing_bounds {
                if !bounds.contains(next_state.x, next_state.y) {
                    stats.window_rejects += 1;
                    continue;
                }
            }
            if !obstacle_map.check_primitive_footprint_free(
                state.x,
                state.y,
                &primitive.footprint,
                port_open_cells,
            ) {
                stats.footprint_rejects += 1;
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
                    primitive_id: primitive.id,
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

fn compute_routing_bounds(
    obstacle_map: &ObstacleMap,
    source: State,
    target: State,
    config: &AStarConfig,
    expansion_idx: u32,
) -> Option<RoutingBounds> {
    let span_x = (target.x - source.x).abs();
    let span_y = (target.y - source.y).abs();

    let growth = 1.0 + (expansion_idx as f64) * config.routing_window_growth.max(0.0);
    let tolerance_padding = config.target_tolerance_cells.max(0);
    let margin_x = ((config.routing_window_scale.max(0.0) * (span_x as f64) * growth).ceil()
        as i32)
        .max(config.routing_window_min_margin_cells.max(0))
        .saturating_add(tolerance_padding);
    let margin_y = ((config.routing_window_scale.max(0.0) * (span_y as f64) * growth).ceil()
        as i32)
        .max(config.routing_window_min_margin_cells.max(0))
        .saturating_add(tolerance_padding);

    let min_x = source.x.min(target.x).saturating_sub(margin_x).max(0);
    let max_x = source
        .x
        .max(target.x)
        .saturating_add(margin_x)
        .min(obstacle_map.width() - 1);
    let min_y = source.y.min(target.y).saturating_sub(margin_y).max(0);
    let max_y = source
        .y
        .max(target.y)
        .saturating_add(margin_y)
        .min(obstacle_map.height() - 1);

    if min_x > max_x || min_y > max_y {
        return None;
    }
    Some(RoutingBounds {
        min_x,
        max_x,
        min_y,
        max_y,
    })
}

fn window_area(bounds: RoutingBounds) -> i64 {
    let width = (bounds.max_x - bounds.min_x + 1).max(0) as i64;
    let height = (bounds.max_y - bounds.min_y + 1).max(0) as i64;
    width * height
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

fn target_reached(state: State, target: State, config: &AStarConfig) -> bool {
    let tolerance = config.target_tolerance_cells.max(0);
    let pos_ok = (state.x - target.x).abs() <= tolerance && (state.y - target.y).abs() <= tolerance;
    if !pos_ok {
        return false;
    }
    if let Some(mask) = config.allowed_target_angles_mask {
        return (mask & (1u8 << (state.angle % 8))) != 0;
    }
    if config.require_target_angle {
        return state.angle == target.angle;
    }
    true
}

fn heuristic(state: State, target: State, grid_size_um: f64) -> f64 {
    let dx = (target.x - state.x) as f64;
    let dy = (target.y - state.y) as f64;
    (dx * dx + dy * dy).sqrt() * grid_size_um
}

fn reconstruct_route(
    source: State,
    requested_target: State,
    reached_target: State,
    parents: &FxHashMap<State, Parent>,
    primitives: &PrimitiveLibrary,
    total_cost: f64,
    stats: RouteSearchStats,
) -> RouteResult {
    let mut states_reversed = vec![reached_target];
    let mut primitive_steps_reversed = Vec::new();
    let mut current = reached_target;

    while current != source {
        let parent = parents
            .get(&current)
            .expect("missing parent during route reconstruction");
        primitive_steps_reversed.push((parent.previous, parent.primitive_id));
        current = parent.previous;
        states_reversed.push(current);
    }

    states_reversed.reverse();
    primitive_steps_reversed.reverse();

    let mut primitive_ids = Vec::with_capacity(primitive_steps_reversed.len());
    let mut cells = Vec::new();
    let mut seen_cells = FxHashSet::default();
    let mut ordered_path = Vec::new();
    push_if_different(&mut ordered_path, (source.x, source.y));
    let mut total_length_um = 0.0;

    for (origin, primitive_id) in primitive_steps_reversed {
        let primitive = find_primitive(primitives, origin.angle, primitive_id).expect(
            "missing primitive during route reconstruction; parent map references invalid primitive id",
        );
        primitive_ids.push(primitive_id);
        total_length_um += primitive.length_um;

        for (dx, dy) in primitive.footprint.iter().copied() {
            let cell = (origin.x + dx, origin.y + dy);
            push_if_different(&mut ordered_path, cell);
            if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                cells.push(cell);
            }
        }
    }
    push_if_different(&mut ordered_path, (reached_target.x, reached_target.y));
    let compressed_waypoints = compress_grid_waypoints(&ordered_path);

    RouteResult {
        states: states_reversed,
        primitives: primitive_ids,
        cells,
        compressed_waypoints,
        total_length_um,
        total_cost,
        requested_target,
        reached_target,
        stats,
    }
}

fn find_primitive(
    primitives: &PrimitiveLibrary,
    start_angle: u8,
    primitive_id: u16,
) -> Option<&crate::primitives::Primitive> {
    primitives
        .get_primitives_for_angle(start_angle)
        .iter()
        .find(|p| p.id == primitive_id)
}

fn compress_grid_waypoints(path: &[(i32, i32)]) -> Vec<(i32, i32)> {
    if path.is_empty() {
        return Vec::new();
    }
    if path.len() == 1 {
        return vec![path[0]];
    }

    let mut waypoints = Vec::with_capacity(path.len());
    push_if_different(&mut waypoints, path[0]);

    let mut prev_dir = direction(path[0], path[1]);
    for i in 2..path.len() {
        let curr_dir = direction(path[i - 1], path[i]);
        if curr_dir != prev_dir {
            push_if_different(&mut waypoints, path[i - 1]);
        }
        prev_dir = curr_dir;
    }

    push_if_different(&mut waypoints, path[path.len() - 1]);
    waypoints
}

fn push_if_different(points: &mut Vec<(i32, i32)>, point: (i32, i32)) {
    if points.last().copied() != Some(point) {
        points.push(point);
    }
}

fn direction(a: (i32, i32), b: (i32, i32)) -> (i32, i32) {
    ((b.0 - a.0).signum(), (b.1 - a.1).signum())
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
                ..AStarConfig::default()
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

    #[test]
    fn supports_coordinate_tolerance() {
        let map = ObstacleMap::new(12, 6);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 2, 0),
            State::new(5, 3, 0),
            None,
            &AStarConfig {
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
    fn reconstruction_uses_primitive_ids_and_preserves_cells() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
        )
        .expect("route should exist");

        assert!(!result.primitives.is_empty());
        for primitive_id in &result.primitives {
            assert!(*primitive_id > 0);
        }

        let mut expected_cells = Vec::new();
        let mut seen_cells = FxHashSet::default();
        for (idx, primitive_id) in result.primitives.iter().enumerate() {
            let origin = result.states[idx];
            let primitive = library
                .get_primitives_for_angle(origin.angle)
                .iter()
                .find(|p| p.id == *primitive_id)
                .expect("primitive id should resolve in library");
            for (dx, dy) in primitive.footprint.iter().copied() {
                let cell = (origin.x + dx, origin.y + dy);
                if seen_cells.insert(pack_xy(cell.0, cell.1)) {
                    expected_cells.push(cell);
                }
            }
        }

        assert_eq!(result.cells, expected_cells);
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
                target_tolerance_cells: -1,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn routing_window_padding_includes_target_tolerance() {
        let map = ObstacleMap::new(20, 20);
        let source = State::new(10, 10, 0);
        let target = State::new(10, 10, 0);
        let bounds = compute_routing_bounds(
            &map,
            source,
            target,
            &AStarConfig {
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                target_tolerance_cells: 2,
                ..AStarConfig::default()
            },
            0,
        )
        .unwrap();
        assert_eq!(bounds.min_x, 8);
        assert_eq!(bounds.max_x, 12);
        assert_eq!(bounds.min_y, 8);
        assert_eq!(bounds.max_y, 12);
    }
}
