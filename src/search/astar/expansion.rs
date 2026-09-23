//! Primitive-move expansion helpers: the footprint collision profile, the
//! primitive transition-class/ordering helpers used to iterate moves in a
//! consistent order, the target-biased primitive score, and the compact
//! diagonal halo used by the Tier-1 collision gate. Slice 3 of Milestone 3
//! adds the expansion functions extracted from the kernel loop. Moved out
//! of `src/astar.rs` (Milestone 3, Slice 2); pure code motion, no behaviour
//! change.

use crate::primitives::{Primitive, PrimitiveGeometry, DIRECTIONS};
use crate::search::astar::config::{
    PrimitiveOrdering, PRIMITIVE_BEND_45, PRIMITIVE_BEND_90, PRIMITIVE_STRAIGHT_LONG,
    PRIMITIVE_STRAIGHT_SHORT,
};
use crate::search::astar::cost::PrimitiveSearchMetadata;
use crate::search::astar::heuristic::distance_heuristic;
use crate::search::state::State;
use std::cmp::Ordering;

#[derive(Clone, Debug)]
pub(crate) struct FootprintCollisionProfile {
    pub(crate) is_full_rect: bool,
    pub(crate) min_dx: i32,
    pub(crate) max_dx: i32,
    pub(crate) min_dy: i32,
    pub(crate) max_dy: i32,
    pub(crate) cell_count: usize,
    pub(crate) horizontal_runs: Vec<(i32, i32, i32)>,
}

impl FootprintCollisionProfile {
    #[inline]
    pub(crate) fn from_footprint(footprint: &[(i32, i32)]) -> Self {
        let cell_count = footprint.len();
        if footprint.is_empty() {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
                horizontal_runs: Vec::new(),
            };
        }

        let mut min_dx = i32::MAX;
        let mut max_dx = i32::MIN;
        let mut min_dy = i32::MAX;
        let mut max_dy = i32::MIN;
        for &(dx, dy) in footprint {
            min_dx = min_dx.min(dx);
            max_dx = max_dx.max(dx);
            min_dy = min_dy.min(dy);
            max_dy = max_dy.max(dy);
        }

        let width = max_dx.checked_sub(min_dx).and_then(|v| v.checked_add(1));
        let height = max_dy.checked_sub(min_dy).and_then(|v| v.checked_add(1));
        let (Some(width_usize), Some(height_usize)) = (
            width.and_then(|v| usize::try_from(v).ok()),
            height.and_then(|v| usize::try_from(v).ok()),
        ) else {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
                horizontal_runs: footprint_horizontal_runs(footprint),
            };
        };

        let area = width_usize.checked_mul(height_usize);
        if area != Some(cell_count) {
            return Self {
                is_full_rect: false,
                min_dx: 0,
                max_dx: -1,
                min_dy: 0,
                max_dy: -1,
                cell_count,
                horizontal_runs: footprint_horizontal_runs(footprint),
            };
        }

        let mut sorted_footprint = footprint.to_vec();
        sorted_footprint.sort_unstable_by(|(a_x, a_y), (b_x, b_y)| a_y.cmp(b_y).then(a_x.cmp(b_x)));
        let mut idx = 0usize;
        for y in min_dy..=max_dy {
            for x in min_dx..=max_dx {
                if idx >= sorted_footprint.len() || sorted_footprint[idx] != (x, y) {
                    return Self {
                        is_full_rect: false,
                        min_dx: 0,
                        max_dx: -1,
                        min_dy: 0,
                        max_dy: -1,
                        cell_count,
                        horizontal_runs: footprint_horizontal_runs(footprint),
                    };
                }
                idx += 1;
            }
        }

        Self {
            is_full_rect: true,
            min_dx,
            max_dx,
            min_dy,
            max_dy,
            cell_count,
            horizontal_runs: Vec::new(),
        }
    }
}

pub(crate) fn footprint_horizontal_runs(footprint: &[(i32, i32)]) -> Vec<(i32, i32, i32)> {
    if footprint.is_empty() {
        return Vec::new();
    }
    let mut cells = footprint.to_vec();
    cells.sort_unstable_by(|(a_x, a_y), (b_x, b_y)| a_y.cmp(b_y).then(a_x.cmp(b_x)));
    cells.dedup();

    let mut runs = Vec::new();
    let mut current_y = cells[0].1;
    let mut start_x = cells[0].0;
    let mut end_x = cells[0].0;
    for &(x, y) in cells.iter().skip(1) {
        if y == current_y && x == end_x + 1 {
            end_x = x;
            continue;
        }
        runs.push((current_y, start_x, end_x));
        current_y = y;
        start_x = x;
        end_x = x;
    }
    runs.push((current_y, start_x, end_x));
    runs
}

pub(crate) fn primitive_transition_class(geometry: &PrimitiveGeometry, dx: i32, dy: i32) -> usize {
    match geometry {
        PrimitiveGeometry::Straight { .. } => {
            if dx.abs().max(dy.abs()) <= 1 {
                PRIMITIVE_STRAIGHT_SHORT
            } else {
                PRIMITIVE_STRAIGHT_LONG
            }
        }
        PrimitiveGeometry::Bend { angle_delta, .. } => {
            if angle_delta.unsigned_abs() == 1 {
                PRIMITIVE_BEND_45
            } else {
                PRIMITIVE_BEND_90
            }
        }
    }
}

pub(crate) fn fixed_primitive_order(len: usize) -> ([usize; 8], usize) {
    let mut order = [0usize; 8];
    for (idx, slot) in order.iter_mut().enumerate().take(len.min(8)) {
        *slot = idx;
    }
    (order, len.min(8))
}

pub(crate) fn primitive_class_order_rank(class: usize) -> usize {
    match class {
        PRIMITIVE_STRAIGHT_LONG => 0,
        PRIMITIVE_STRAIGHT_SHORT => 1,
        PRIMITIVE_BEND_45 => 2,
        PRIMITIVE_BEND_90 => 3,
        _ => 4,
    }
}

#[inline]
pub(crate) fn primitive_class_is_straight(class: usize) -> bool {
    matches!(class, PRIMITIVE_STRAIGHT_SHORT | PRIMITIVE_STRAIGHT_LONG)
}

pub(crate) fn primitive_initial_straight_run_distance(
    primitive: &Primitive,
    start_angle: u8,
) -> f64 {
    let dir = DIRECTIONS[(start_angle % 8) as usize];
    let mut run_cells = 0i32;
    for (idx, point) in primitive.footprint.iter().copied().enumerate() {
        let step = idx as i32;
        if point != (dir.0 * step, dir.1 * step) {
            break;
        }
        run_cells = step;
    }
    f64::from(run_cells)
}

pub(crate) fn primitive_terminal_straight_run_cells(primitive: &Primitive, end_angle: u8) -> i32 {
    let Some(end) = primitive.footprint.last().copied() else {
        return 0;
    };
    let dir = DIRECTIONS[(end_angle % 8) as usize];
    let mut run_cells = 0i32;
    for (idx, point) in primitive.footprint.iter().copied().enumerate().rev() {
        let step = (primitive.footprint.len() - 1 - idx) as i32;
        if point != (end.0 - dir.0 * step, end.1 - dir.1 * step) {
            break;
        }
        run_cells = step;
    }
    run_cells
}

pub(crate) fn target_biased_primitive_score(
    primitive: &Primitive,
    metadata: PrimitiveSearchMetadata,
    state: State,
    target: State,
    grid_size_um: f64,
) -> f64 {
    let Some(next_x) = state.x.checked_add(primitive.dx) else {
        return f64::INFINITY;
    };
    let Some(next_y) = state.y.checked_add(primitive.dy) else {
        return f64::INFINITY;
    };
    metadata.base_step_cost
        + distance_heuristic(
            State::new(next_x, next_y, primitive.end_angle),
            target,
            grid_size_um,
        )
}

pub(crate) fn primitive_iteration_order(
    primitives: &[Primitive],
    metadata: &[PrimitiveSearchMetadata],
    state: State,
    target: State,
    grid_size_um: f64,
    ordering: PrimitiveOrdering,
) -> ([usize; 8], usize) {
    let (mut order, len) = fixed_primitive_order(primitives.len());
    match ordering {
        PrimitiveOrdering::Library => {}
        PrimitiveOrdering::LongStraightFirst => {
            order[..len].sort_by(|a, b| {
                primitive_class_order_rank(metadata[*a].transition_class)
                    .cmp(&primitive_class_order_rank(metadata[*b].transition_class))
                    .then_with(|| a.cmp(b))
            });
        }
        PrimitiveOrdering::TargetBiased => {
            order[..len].sort_by(|a, b| {
                let a_score = target_biased_primitive_score(
                    &primitives[*a],
                    metadata[*a],
                    state,
                    target,
                    grid_size_um,
                );
                let b_score = target_biased_primitive_score(
                    &primitives[*b],
                    metadata[*b],
                    state,
                    target,
                    grid_size_um,
                );
                a_score
                    .partial_cmp(&b_score)
                    .unwrap_or(Ordering::Equal)
                    .then_with(|| a.cmp(b))
            });
        }
    }
    (order, len)
}

pub(crate) fn compact_diagonal_halo_cells(
    start: (i32, i32),
    end: (i32, i32),
    dx: i32,
    dy: i32,
) -> Vec<(i32, i32)> {
    let mut cells = Vec::with_capacity(4);
    push_unique_cell(&mut cells, (start.0 + dx, start.1));
    push_unique_cell(&mut cells, (end.0 + dx, end.1));
    push_unique_cell(&mut cells, (start.0, start.1 + dy));
    push_unique_cell(&mut cells, (end.0, end.1 + dy));
    cells
}

pub(crate) fn push_unique_cell(cells: &mut Vec<(i32, i32)>, cell: (i32, i32)) {
    if !cells.contains(&cell) {
        cells.push(cell);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::KernelDiagnostics;
    use crate::obstacle_map::ObstacleMap;
    use crate::search::astar::config::AStarConfig;
    use crate::search::astar::route_single_net_with_config;
    use crate::search::test_support::*;

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
                diagnostics: KernelDiagnostics::default(),
                max_iterations: 200,
                bend_weight: 1.0,
                target_tolerance_cells: 0,
                ..AStarConfig::default()
            },
        );

        assert!(result.is_none());
    }
}
