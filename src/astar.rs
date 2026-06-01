//! Single-net stateful A* router.
//!
//! This is the first routing kernel: one source state, one target state, and
//! primitive-based photonic transitions over the existing obstacle map.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rustc_hash::FxHashSet;

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::PrimitiveGeometry;
use crate::primitives::PrimitiveLibrary;
use crate::simple_routes::{
    direction_between as simple_direction_between, expand_candidate_to_grid_points,
    try_straight_l_or_z_candidate_with_config, GridPoint, SimpleRouteCandidate, SimpleZRouteConfig,
};

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
    pub max_dense_states: usize,
    pub max_dense_obstacle_cells: usize,
    pub enable_simple_routes: bool,
    pub simple_route_max_offset_cells: i32,
    pub simple_route_min_leg_len_cells: i32,
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
            routing_window_fallback_full_grid: false,
            routing_window_growth: 0.5,
            max_dense_states: 20_000_000,
            max_dense_obstacle_cells: 10_000_000,
            enable_simple_routes: true,
            simple_route_max_offset_cells: 16,
            simple_route_min_leg_len_cells: 1,
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
    pub dense_grid_build_failures: usize,
    pub max_window_area_cells: i64,
}

const NO_PARENT: u32 = u32::MAX;

struct DenseSearchStorage {
    bounds: RoutingBounds,
    width: i32,
    g_costs: Vec<f64>,
    parent_idx: Vec<u32>,
    parent_primitive: Vec<u16>,
    closed: Vec<bool>,
}

impl DenseSearchStorage {
    fn new(bounds: RoutingBounds, max_dense_states: usize) -> Option<Self> {
        let width = bounds.max_x.checked_sub(bounds.min_x)?.checked_add(1)?;
        let height = bounds.max_y.checked_sub(bounds.min_y)?.checked_add(1)?;
        if width <= 0 || height <= 0 {
            return None;
        }
        let width_usize = usize::try_from(width).ok()?;
        let height_usize = usize::try_from(height).ok()?;
        let state_count = width_usize.checked_mul(height_usize)?.checked_mul(8)?;
        if state_count > u32::MAX as usize {
            return None;
        }
        if state_count > max_dense_states {
            return None;
        }

        Some(Self {
            bounds,
            width,
            g_costs: vec![f64::INFINITY; state_count],
            parent_idx: vec![NO_PARENT; state_count],
            parent_primitive: vec![0; state_count],
            closed: vec![false; state_count],
        })
    }

    fn state_to_idx(&self, state: State) -> Option<usize> {
        if state.angle >= 8 || !self.bounds.contains(state.x, state.y) {
            return None;
        }
        let local_x = usize::try_from(state.x.checked_sub(self.bounds.min_x)?).ok()?;
        let local_y = usize::try_from(state.y.checked_sub(self.bounds.min_y)?).ok()?;
        let width = usize::try_from(self.width).ok()?;
        local_y
            .checked_mul(width)?
            .checked_add(local_x)?
            .checked_mul(8)?
            .checked_add(usize::from(state.angle))
    }

    fn idx_to_state(&self, idx: usize) -> State {
        let width = usize::try_from(self.width).expect("width must be > 0");
        let angle = (idx % 8) as u8;
        let cell_idx = idx / 8;
        let local_x = (cell_idx % width) as i32;
        let local_y = (cell_idx / width) as i32;
        State::new(
            self.bounds.min_x + local_x,
            self.bounds.min_y + local_y,
            angle,
        )
    }
}

struct DenseRoutingGrid {
    bounds: RoutingBounds,
    width: i32,
    blocked: Vec<u8>,
}

impl DenseRoutingGrid {
    fn from_obstacle_map(
        obstacle_map: &ObstacleMap,
        bounds: RoutingBounds,
        opened_cells: Option<&FxHashSet<CellKey>>,
        max_dense_obstacle_cells: usize,
    ) -> Option<Self> {
        let width = bounds.max_x.checked_sub(bounds.min_x)?.checked_add(1)?;
        let height = bounds.max_y.checked_sub(bounds.min_y)?.checked_add(1)?;
        if width <= 0 || height <= 0 {
            return None;
        }

        let width_usize = usize::try_from(width).ok()?;
        let height_usize = usize::try_from(height).ok()?;
        let cell_count = width_usize.checked_mul(height_usize)?;
        if cell_count > max_dense_obstacle_cells {
            return None;
        }

        let mut blocked = vec![0u8; cell_count];
        for local_y in 0..height {
            for local_x in 0..width {
                let x = bounds.min_x + local_x;
                let y = bounds.min_y + local_y;
                let idx = usize::try_from(local_y)
                    .ok()?
                    .checked_mul(width_usize)?
                    .checked_add(usize::try_from(local_x).ok()?)?;
                let opened = opened_cells
                    .map(|cells| cells.contains(&pack_xy(x, y)))
                    .unwrap_or(false);
                blocked[idx] = if opened || !obstacle_map.is_blocked(x, y) {
                    0
                } else {
                    1
                };
            }
        }

        Some(Self {
            bounds,
            width,
            blocked,
        })
    }

    #[inline]
    fn contains(&self, x: i32, y: i32) -> bool {
        self.bounds.contains(x, y)
    }

    #[inline]
    fn is_blocked(&self, x: i32, y: i32) -> bool {
        match self.idx_of(x, y) {
            Some(idx) => self.blocked[idx] != 0,
            None => true,
        }
    }

    #[inline]
    fn primitive_footprint_free(
        &self,
        origin_x: i32,
        origin_y: i32,
        footprint: &[(i32, i32)],
    ) -> bool {
        for (dx, dy) in footprint.iter().copied() {
            let x = match origin_x.checked_add(dx) {
                Some(x) => x,
                None => return false,
            };
            let y = match origin_y.checked_add(dy) {
                Some(y) => y,
                None => return false,
            };
            if self.is_blocked(x, y) {
                return false;
            }
        }
        true
    }

    #[inline]
    fn idx_of(&self, x: i32, y: i32) -> Option<usize> {
        if !self.contains(x, y) {
            return None;
        }
        let local_x = usize::try_from(x.checked_sub(self.bounds.min_x)?).ok()?;
        let local_y = usize::try_from(y.checked_sub(self.bounds.min_y)?).ok()?;
        let width = usize::try_from(self.width).ok()?;
        local_y.checked_mul(width)?.checked_add(local_x)
    }
}

#[derive(Clone, Copy, Debug)]
struct OpenEntry {
    f_score: f64,
    g_score: f64,
    counter: u64,
    idx: usize,
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
    _port_open_cells: Option<&FxHashSet<CellKey>>,
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
    let mut anchor_open_cells = FxHashSet::default();
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));

    let mut stats = RouteSearchStats::default();
    if config.enable_simple_routes {
        let bend_radius_cells = infer_bend_radius_cells(primitives).unwrap_or(0);
        let z_config = SimpleZRouteConfig {
            max_offset_cells: config.simple_route_max_offset_cells,
            include_zero_offset: true,
            min_leg_len_cells: config
                .simple_route_min_leg_len_cells
                .max(bend_radius_cells),
        };
        if let Some(candidate) = try_straight_l_or_z_candidate_with_config(
            source,
            target,
            obstacle_map,
            Some(&anchor_open_cells),
            &z_config,
        ) {
            if let Some(simple_route) = simple_candidate_to_route_result(
                &candidate,
                source,
                target,
                primitives,
                stats.clone(),
            ) {
                return Some(simple_route);
            }
        }
    }

    if !config.use_routing_window {
        return route_single_net_with_bounds(
            obstacle_map,
            primitives,
            source,
            target,
            Some(&anchor_open_cells),
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
            Some(&anchor_open_cells),
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
            Some(&anchor_open_cells),
            config,
            None,
            &mut stats,
        );
    }

    None
}

fn simple_candidate_to_route_result(
    candidate: &SimpleRouteCandidate,
    source: State,
    target: State,
    primitives: &PrimitiveLibrary,
    stats: RouteSearchStats,
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
        segment_lengths.push((b.x - a.x).abs() + (b.y - a.y).abs());
    }

    let bend_radius_cells = infer_bend_radius_cells(primitives).unwrap_or(0);
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
            let bend_primitive_id = find_bend_primitive_id(
                current.angle,
                delta,
                bend_radius_cells,
                primitives,
            )?;
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

#[inline]
fn grid_point_from_state(state: State) -> GridPoint {
    GridPoint::new(state.x, state.y)
}

fn infer_bend_radius_cells(primitives: &PrimitiveLibrary) -> Option<i32> {
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

fn decompose_straight_cells(
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
                let cells = (primitive.dx.abs() + primitive.dy.abs()) as usize;
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

fn turn_delta(from: u8, to: u8) -> Option<i8> {
    let delta = (to as i16 - from as i16).rem_euclid(8) as u8;
    match delta {
        2 => Some(2),
        6 => Some(-2),
        _ => None,
    }
}

fn find_bend_primitive_id(
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
    let bounds = if let Some(bounds) = routing_bounds {
        if !bounds.contains(source.x, source.y) || !bounds.contains(target.x, target.y) {
            return None;
        }
        bounds
    } else {
        RoutingBounds {
            min_x: 0,
            max_x: obstacle_map.width() - 1,
            min_y: 0,
            max_y: obstacle_map.height() - 1,
        }
    };

    let mut open_set = BinaryHeap::new();
    let mut storage = DenseSearchStorage::new(bounds, config.max_dense_states)?;
    let dense_grid = match DenseRoutingGrid::from_obstacle_map(
        obstacle_map,
        bounds,
        port_open_cells,
        config.max_dense_obstacle_cells,
    ) {
        Some(grid) => grid,
        None => {
            stats.dense_grid_build_failures += 1;
            return None;
        }
    };
    let mut counter = 0u64;
    let source_idx = storage.state_to_idx(source)?;

    storage.g_costs[source_idx] = 0.0;
    open_set.push(OpenEntry {
        f_score: heuristic(source, target, primitives.grid_size_um()),
        g_score: 0.0,
        counter,
        idx: source_idx,
    });
    counter += 1;

    let mut iterations = 0usize;
    while let Some(entry) = open_set.pop() {
        iterations += 1;
        if iterations > config.max_iterations {
            return None;
        }

        let idx = entry.idx;
        if storage.closed[idx] {
            continue;
        }
        let state = storage.idx_to_state(idx);
        if target_reached(state, target, config) {
            return reconstruct_route_dense(
                source_idx,
                idx,
                target,
                primitives,
                entry.g_score,
                stats.clone(),
                &storage,
            );
        }
        storage.closed[idx] = true;
        stats.expanded_states += 1;

        let current_g = storage.g_costs[idx];
        let primitive_bucket = primitives.get_primitives_for_angle(state.angle);
        for primitive in primitive_bucket.iter() {
            let next_state = State::new(
                state.x.checked_add(primitive.dx)?,
                state.y.checked_add(primitive.dy)?,
                primitive.end_angle,
            );

            if !obstacle_map.in_bounds(next_state.x, next_state.y) {
                continue;
            }
            if !bounds.contains(next_state.x, next_state.y) {
                stats.window_rejects += 1;
                continue;
            }
            // TODO: bounds may need primitive-footprint margin to avoid rejecting valid routes near window edges.
            if !dense_grid.primitive_footprint_free(state.x, state.y, &primitive.footprint) {
                stats.footprint_rejects += 1;
                continue;
            }
            let Some(next_idx) = storage.state_to_idx(next_state) else {
                continue;
            };
            if storage.closed[next_idx] {
                continue;
            }

            let step_cost = primitive.length_um + config.bend_weight * primitive.bend_cost;
            let tentative_g = current_g + step_cost;
            if tentative_g >= storage.g_costs[next_idx] {
                continue;
            }

            storage.parent_idx[next_idx] = idx as u32;
            storage.parent_primitive[next_idx] = primitive.id;
            storage.g_costs[next_idx] = tentative_g;
            open_set.push(OpenEntry {
                f_score: tentative_g + heuristic(next_state, target, primitives.grid_size_um()),
                g_score: tentative_g,
                counter,
                idx: next_idx,
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

fn reconstruct_route_dense(
    source_idx: usize,
    reached_idx: usize,
    requested_target: State,
    primitives: &PrimitiveLibrary,
    total_cost: f64,
    stats: RouteSearchStats,
    storage: &DenseSearchStorage,
) -> Option<RouteResult> {
    let source = storage.idx_to_state(source_idx);
    let reached_target = storage.idx_to_state(reached_idx);
    let mut states_reversed = vec![reached_target];
    let mut primitive_steps_reversed = Vec::new();
    let mut current_idx = reached_idx;

    while current_idx != source_idx {
        let parent_idx_u32 = *storage.parent_idx.get(current_idx)?;
        if parent_idx_u32 == NO_PARENT {
            return None;
        }
        let parent_idx = usize::try_from(parent_idx_u32).ok()?;
        let previous = storage.idx_to_state(parent_idx);
        let primitive_id = *storage.parent_primitive.get(current_idx)?;
        primitive_steps_reversed.push((previous, primitive_id));
        current_idx = parent_idx;
        states_reversed.push(previous);
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
        let primitive = find_primitive(primitives, origin.angle, primitive_id)?;
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

    Some(RouteResult {
        states: states_reversed,
        primitives: primitive_ids,
        cells,
        compressed_waypoints,
        total_length_um,
        total_cost,
        requested_target,
        reached_target,
        stats,
    })
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
    use crate::geometry_realization::{route_to_primitive_centerline, GeometryGridSpec};
    use crate::obstacle_map::ObstacleMap;
    use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};

    fn primitive_library() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
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
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
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
    fn simple_straight_route_used_before_astar() {
        let map = ObstacleMap::new(10, 6);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
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
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 4, 2),
            None,
            &AStarConfig {
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
        let map = ObstacleMap::new(10, 10);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 4, 0),
            None,
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple Z route should exist");
        assert_eq!(
            result.compressed_waypoints,
            vec![(1, 1), (2, 1), (2, 4), (5, 4)]
        );
        assert_eq!(result.stats.expanded_states, 0);
    }

    #[test]
    fn simple_z_route_has_primitives_and_replay_centerline() {
        let map = ObstacleMap::new(20, 20);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 4, 0),
            None,
            &AStarConfig {
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
    fn simple_route_disabled_uses_astar() {
        let map = ObstacleMap::new(10, 6);
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("A* route should exist with simple routes disabled");
        assert!(result.stats.expanded_states > 0);
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
            &primitive_library(),
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            Some(&opened),
            &AStarConfig {
                enable_simple_routes: true,
                ..AStarConfig::default()
            },
        )
        .expect("simple route should allow opened endpoint cells");
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
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn opened_cells_cannot_unblock_non_anchor_static_cells() {
        let mut map = ObstacleMap::new(7, 1);
        map.add_static_cell(3, 0);
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(3, 0));
        let result = route_single_net_with_config(
            &map,
            &primitive_library(),
            State::new(1, 0, 0),
            State::new(5, 0, 0),
            Some(&opened),
            &AStarConfig {
                enable_simple_routes: true,
                use_routing_window: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
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
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        )
        .expect("A* should still find the old straight route");
        assert!(!result.primitives.is_empty());
        assert!(result.stats.expanded_states > 0);
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

    #[test]
    fn dense_state_index_roundtrip() {
        let storage = DenseSearchStorage::new(
            RoutingBounds {
                min_x: 10,
                max_x: 14,
                min_y: 20,
                max_y: 22,
            },
            10_000,
        )
        .expect("storage should allocate");
        let state = State::new(12, 21, 5);
        let idx = storage
            .state_to_idx(state)
            .expect("state should map to index");
        assert_eq!(storage.idx_to_state(idx), state);
    }

    #[test]
    fn dense_state_outside_bounds_returns_none() {
        let storage = DenseSearchStorage::new(
            RoutingBounds {
                min_x: 2,
                max_x: 4,
                min_y: 2,
                max_y: 4,
            },
            10_000,
        )
        .expect("storage should allocate");
        assert!(storage.state_to_idx(State::new(1, 2, 0)).is_none());
        assert!(storage.state_to_idx(State::new(2, 5, 0)).is_none());
    }

    #[test]
    fn nonzero_offset_window_routes() {
        let map = ObstacleMap::new(30, 30);
        let library = primitive_library();
        let result = route_single_net_with_bounds(
            &map,
            &library,
            State::new(11, 11, 0),
            State::new(15, 11, 0),
            None,
            &AStarConfig::default(),
            Some(RoutingBounds {
                min_x: 10,
                max_x: 20,
                min_y: 10,
                max_y: 20,
            }),
            &mut RouteSearchStats::default(),
        )
        .expect("route in offset bounds should exist");
        assert_eq!(result.states.first().copied(), Some(State::new(11, 11, 0)));
        assert_eq!(result.states.last().copied(), Some(State::new(15, 11, 0)));
    }

    #[test]
    fn full_grid_fallback_uses_dense_storage() {
        let mut map = ObstacleMap::new(12, 8);
        map.add_static_cell(3, 1);
        map.add_static_cell(4, 1);
        map.add_static_cell(5, 1);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(7, 1, 0),
            None,
            &AStarConfig {
                routing_window_min_margin_cells: 0,
                routing_window_scale: 0.0,
                routing_window_max_expansions: 0,
                routing_window_fallback_full_grid: true,
                ..AStarConfig::default()
            },
        )
        .expect("full-grid fallback should find a detour");
        assert_eq!(result.states.last().copied(), Some(State::new(7, 1, 0)));
        assert!(result.stats.used_full_grid_fallback);
    }

    #[test]
    fn dense_state_limit_can_fail_attempt() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                max_dense_states: 8,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn dense_grid_mirrors_obstacles_and_opened_cells() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_static_cell(3, 2);
        let mut opened = FxHashSet::default();
        opened.insert(pack_xy(3, 2));
        let bounds = RoutingBounds {
            min_x: 2,
            max_x: 5,
            min_y: 1,
            max_y: 4,
        };

        let closed_grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, None, 1_000).expect("grid");
        assert!(closed_grid.is_blocked(3, 2));

        let opened_grid =
            DenseRoutingGrid::from_obstacle_map(&map, bounds, Some(&opened), 1_000).expect("grid");
        assert!(!opened_grid.is_blocked(3, 2));
    }

    #[test]
    fn dense_grid_out_of_bounds_is_blocked() {
        let map = ObstacleMap::new(8, 6);
        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 5,
                min_y: 1,
                max_y: 4,
            },
            None,
            1_000,
        )
        .expect("grid");
        assert!(grid.is_blocked(1, 1));
        assert!(grid.is_blocked(2, 5));
    }

    #[test]
    fn dense_grid_primitive_footprint_checks() {
        let mut map = ObstacleMap::new(8, 6);
        map.add_static_cell(4, 2);
        let grid = DenseRoutingGrid::from_obstacle_map(
            &map,
            RoutingBounds {
                min_x: 2,
                max_x: 5,
                min_y: 1,
                max_y: 4,
            },
            None,
            1_000,
        )
        .expect("grid");

        assert!(grid.primitive_footprint_free(2, 2, &[(0, 0), (1, 0)]));
        assert!(!grid.primitive_footprint_free(3, 2, &[(0, 0), (1, 0)]));
        assert!(!grid.primitive_footprint_free(5, 2, &[(0, 0), (1, 0)]));
    }

    #[test]
    fn dense_obstacle_limit_can_fail_attempt() {
        let map = ObstacleMap::new(10, 5);
        let library = primitive_library();
        let result = route_single_net_with_config(
            &map,
            &library,
            State::new(1, 1, 0),
            State::new(5, 1, 0),
            None,
            &AStarConfig {
                use_routing_window: false,
                max_dense_obstacle_cells: 10,
                enable_simple_routes: false,
                ..AStarConfig::default()
            },
        );
        assert!(result.is_none());
    }

    #[test]
    fn reconstruct_fails_cleanly_when_parent_missing() {
        let bounds = RoutingBounds {
            min_x: 0,
            max_x: 7,
            min_y: 0,
            max_y: 7,
        };
        let storage = DenseSearchStorage::new(bounds, 10_000).expect("storage");
        let lib = primitive_library();
        let source_idx = storage
            .state_to_idx(State::new(1, 1, 0))
            .expect("source index");
        let reached_idx = storage
            .state_to_idx(State::new(2, 1, 0))
            .expect("reached index");

        let result = reconstruct_route_dense(
            source_idx,
            reached_idx,
            State::new(2, 1, 0),
            &lib,
            1.0,
            RouteSearchStats::default(),
            &storage,
        );
        assert!(result.is_none());
    }
}
