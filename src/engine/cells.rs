use std::time::Instant;

use pyo3::prelude::*;
use pyo3::types::PyList;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::auto_meander::{DenseOccupancyPrefix, SparseCellIndex};
use crate::obstacle_map::{pack_xy, unpack_xy, CellKey, ObstacleMap};
use crate::static_obstacle_builder::{physical_to_grid, StaticGridSpec};

use crate::engine::*;

pub(crate) fn pack_cells(cells: &[(i32, i32)]) -> FxHashSet<CellKey> {
    cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect()
}

pub(crate) fn opened_cells_excluding_keepout(
    opened_cells: &[(i32, i32)],
    keepout: &FxHashSet<CellKey>,
    source: PyState,
    target: PyState,
) -> Vec<(i32, i32)> {
    if keepout.is_empty() {
        return opened_cells.to_vec();
    }
    let source_key = pack_xy(source.x, source.y);
    let target_key = pack_xy(target.x, target.y);
    opened_cells
        .iter()
        .copied()
        .filter(|(x, y)| {
            let key = pack_xy(*x, *y);
            !keepout.contains(&key) || key == source_key || key == target_key
        })
        .collect()
}

pub(crate) fn route_orientation_to_angle(orientation: Option<f64>) -> u8 {
    let value = orientation.unwrap_or(0.0).rem_euclid(360.0);
    (value / 45.0).round().rem_euclid(8.0) as u8
}

pub(crate) fn route_angle_to_step(angle: u8) -> (i32, i32) {
    match angle % 8 {
        0 => (1, 0),
        1 => (1, 1),
        2 => (0, 1),
        3 => (-1, 1),
        4 => (-1, 0),
        5 => (-1, -1),
        6 => (0, -1),
        _ => (1, -1),
    }
}

pub(crate) fn route_in_bounds(x: i32, y: i32, grid: &StaticGridSpec) -> bool {
    x >= 0 && x < grid.width && y >= 0 && y < grid.height
}

pub(crate) fn route_collect_inflated_step_cells(
    grid: &StaticGridSpec,
    base_x: i32,
    base_y: i32,
    step_x: i32,
    step_y: i32,
    length_cells: i32,
    half_width_cells: i32,
) -> FxHashSet<CellKey> {
    let mut cells = FxHashSet::default();
    let length = length_cells.max(0);
    let half_width = half_width_cells.max(0);
    for step_idx in 0..length {
        let cx = base_x + step_x * step_idx;
        let cy = base_y + step_y * step_idx;
        if !route_in_bounds(cx, cy, grid) {
            continue;
        }
        for dx in -half_width..=half_width {
            for dy in -half_width..=half_width {
                let nx = cx + dx;
                let ny = cy + dy;
                if step_x != 0 || step_y != 0 {
                    let rel_x = nx - base_x;
                    let rel_y = ny - base_y;
                    let forward_projection = rel_x * step_x.signum() + rel_y * step_y.signum();
                    if forward_projection < 0 {
                        continue;
                    }
                }
                if route_in_bounds(nx, ny, grid) {
                    cells.insert(pack_xy(nx, ny));
                }
            }
        }
    }
    cells
}

pub(crate) fn route_port_state_cell(
    grid: &StaticGridSpec,
    x_um: f64,
    y_um: f64,
    orientation: Option<f64>,
) -> (i32, i32) {
    let angle = route_orientation_to_angle(orientation);
    let (sx, sy) = route_angle_to_step(angle);
    physical_to_grid(
        x_um + sx as f64 * grid.grid_size_um,
        y_um + sy as f64 * grid.grid_size_um,
        grid,
    )
}

pub(crate) fn route_port_footprint_cells(
    grid: &StaticGridSpec,
    x_um: f64,
    y_um: f64,
    orientation: Option<f64>,
    length_cells: i32,
    half_width_cells: i32,
) -> FxHashSet<CellKey> {
    let angle = route_orientation_to_angle(orientation);
    let (sx, sy) = route_angle_to_step(angle);
    let (base_x, base_y) = route_port_state_cell(grid, x_um, y_um, orientation);
    let mut cells = route_collect_inflated_step_cells(
        grid,
        base_x,
        base_y,
        sx,
        sy,
        length_cells,
        half_width_cells,
    );
    if route_in_bounds(base_x, base_y, grid) {
        cells.insert(pack_xy(base_x, base_y));
    }
    cells
}

/// Cells where *other* nets' clearance halos are ignored for one net: a box
/// of the keepout radius around each endpoint plus a run-in corridor along
/// the port axis (ahead of the source, behind the target -- a target state's
/// angle is the arrival heading). Ports of one component can sit closer
/// together than the configured clearance (a 2x2 MMI's inputs are 1.25 um
/// apart), so without this the first committed net's halo would cover the
/// sibling port and its whole approach. How far two such nets must run side
/// by side is decided by whichever of them routes first, so the corridor is
/// not a fixed length: it runs from the port along its axis for as long as
/// the net's own `opened_cells` continue (the router's own notion of this
/// port's approach region), and never shorter than `min_run_in_length_cells`
/// (the port lane). Only the halo is waived: core overlap stays illegal in
/// every consumer of this set.
pub(crate) fn route_dynamic_clearance_exempt_cells(
    grid: &StaticGridSpec,
    opened_cells: &FxHashSet<CellKey>,
    source: PyState,
    target: PyState,
    commit_radius_cells: i32,
    min_run_in_length_cells: i32,
) -> FxHashSet<CellKey> {
    let radius = commit_radius_cells.max(0);
    let min_run_in = min_run_in_length_cells.max(0);
    let mut cells = FxHashSet::default();
    for (anchor, toward_route) in [(source, 1), (target, -1)] {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                let x = anchor.x + dx;
                let y = anchor.y + dy;
                if route_in_bounds(x, y, grid) {
                    cells.insert(pack_xy(x, y));
                }
            }
        }
        let (sx, sy) = route_angle_to_step(anchor.angle);
        let (sx, sy) = (sx * toward_route, sy * toward_route);
        let mut run_in = 0;
        loop {
            let x = anchor.x + sx * run_in;
            let y = anchor.y + sy * run_in;
            if !route_in_bounds(x, y, grid) {
                break;
            }
            if run_in >= min_run_in && !opened_cells.contains(&pack_xy(x, y)) {
                break;
            }
            run_in += 1;
        }
        cells.extend(route_collect_inflated_step_cells(
            grid, anchor.x, anchor.y, sx, sy, run_in, radius,
        ));
    }
    cells
}

pub(crate) fn sorted_cells(cells: FxHashSet<CellKey>) -> Vec<(i32, i32)> {
    let mut out: Vec<(i32, i32)> = cells.into_iter().map(unpack_xy).collect();
    out.sort_unstable();
    out
}

pub(crate) fn collect_meander_route_cell_sets(
    routes: &Bound<'_, PyList>,
    route_clearance_radius_cells: i32,
    width: i32,
    height: i32,
    profile: &mut MeanderRegistrationProfile,
) -> PyResult<(
    Vec<FxHashSet<CellKey>>,
    FxHashMap<CellKey, u32>,
    FxHashSet<CellKey>,
)> {
    let mut route_cell_sets: Vec<FxHashSet<CellKey>> = Vec::with_capacity(routes.len());
    let mut route_cell_refcounts: FxHashMap<CellKey, u32> = FxHashMap::default();
    let mut unique_route_cells: FxHashSet<CellKey> = FxHashSet::default();

    for item in routes.iter() {
        let route_extract_start = Instant::now();
        let route = item.extract::<PyRef<'_, PyRouteResult>>()?;
        profile.route_extract_s += route_extract_start.elapsed().as_secs_f64();
        let mut route_cells = FxHashSet::default();
        let route_cell_collect_start = Instant::now();
        let route_cells_for_registration = if route_clearance_radius_cells > 0 {
            inflate_route_cells(&route.cells, route_clearance_radius_cells, width, height)
        } else {
            route.cells.clone()
        };
        for &(x, y) in &route_cells_for_registration {
            let key = pack_xy(x, y);
            if route_cells.insert(key) {
                unique_route_cells.insert(key);
                *route_cell_refcounts.entry(key).or_insert(0) += 1;
            }
        }
        profile.route_cell_collect_s += route_cell_collect_start.elapsed().as_secs_f64();
        route_cell_sets.push(route_cells);
    }

    Ok((route_cell_sets, route_cell_refcounts, unique_route_cells))
}

pub(crate) fn build_registered_open_sets(
    route_cell_sets: Vec<FxHashSet<CellKey>>,
    route_cell_refcounts: &FxHashMap<CellKey, u32>,
    base_static_keys: &FxHashSet<CellKey>,
    profile: &mut MeanderRegistrationProfile,
) -> (Vec<FxHashSet<CellKey>>, Vec<usize>) {
    let mut registered_open_sets: Vec<FxHashSet<CellKey>> =
        Vec::with_capacity(route_cell_sets.len());
    let mut open_counts = Vec::with_capacity(route_cell_sets.len());
    let open_set_build_start = Instant::now();
    for route_cells in route_cell_sets {
        let open_cells: FxHashSet<CellKey> = route_cells
            .into_iter()
            .filter(|key| {
                route_cell_refcounts.get(key).copied().unwrap_or(0) == 1
                    && !base_static_keys.contains(key)
            })
            .collect();
        open_counts.push(open_cells.len());
        registered_open_sets.push(open_cells);
    }
    profile.open_set_build_s += open_set_build_start.elapsed().as_secs_f64();
    (registered_open_sets, open_counts)
}

pub(crate) fn build_registered_open_indices(
    base_prefix: &DenseOccupancyPrefix,
    registered_open_sets: &[FxHashSet<CellKey>],
) -> Vec<SparseCellIndex> {
    registered_open_sets
        .iter()
        .map(|cells| SparseCellIndex::from_opened_cells(base_prefix, cells))
        .collect()
}

pub(crate) fn inflate_route_cells(
    cells: &[(i32, i32)],
    radius_cells: i32,
    width: i32,
    height: i32,
) -> Vec<(i32, i32)> {
    let mut inflated = Vec::new();
    if radius_cells <= 0 {
        inflated.extend(
            cells
                .iter()
                .copied()
                .filter(|(x, y)| *x >= 0 && *x < width && *y >= 0 && *y < height),
        );
        return inflated;
    }

    let mut seen: FxHashSet<CellKey> = FxHashSet::default();
    for &(x, y) in cells {
        for dx in -radius_cells..=radius_cells {
            for dy in -radius_cells..=radius_cells {
                let nx = x + dx;
                let ny = y + dy;
                if nx >= 0 && nx < width && ny >= 0 && ny < height {
                    let key = pack_xy(nx, ny);
                    if seen.insert(key) {
                        inflated.push((nx, ny));
                    }
                }
            }
        }
    }
    inflated
}

pub(crate) fn route_commit_cells(
    center_cells: &[(i32, i32)],
    core_radius_cells: i32,
    clearance_radius_cells: i32,
    clearance_exempt_cells: Option<&[(i32, i32)]>,
    width: i32,
    height: i32,
) -> Vec<(i32, i32)> {
    let core_cells = inflate_route_cells(center_cells, core_radius_cells, width, height);
    let core_keys: FxHashSet<CellKey> = core_cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect();
    let mut blocked_cells = inflate_route_cells(
        center_cells,
        clearance_radius_cells.max(core_radius_cells),
        width,
        height,
    );

    let Some(exempt_cells) = clearance_exempt_cells else {
        return blocked_cells;
    };
    if exempt_cells.is_empty() {
        return blocked_cells;
    }

    let exempt_keys = pack_cells(exempt_cells);
    blocked_cells.retain(|(x, y)| {
        let key = pack_xy(*x, *y);
        core_keys.contains(&key) || !exempt_keys.contains(&key)
    });
    blocked_cells
}

pub(crate) fn route_core_cells(
    center_cells: &[(i32, i32)],
    core_radius_cells: i32,
    width: i32,
    height: i32,
) -> Vec<(i32, i32)> {
    inflate_route_cells(center_cells, core_radius_cells, width, height)
}

pub(crate) fn unique_cells<I>(cells: I) -> Vec<(i32, i32)>
where
    I: IntoIterator<Item = (i32, i32)>,
{
    let mut out = Vec::new();
    let mut seen: FxHashSet<CellKey> = FxHashSet::default();
    for (x, y) in cells {
        if seen.insert(pack_xy(x, y)) {
            out.push((x, y));
        }
    }
    out
}

pub(crate) fn static_grid_from_py_grid(grid: &PyGridSpec) -> StaticGridSpec {
    StaticGridSpec {
        width: grid.width as i32,
        height: grid.height as i32,
        grid_size_um: grid.grid_size_um,
        origin: (grid.origin_x_um, grid.origin_y_um),
        die_bbox: (
            grid.origin_x_um,
            grid.origin_y_um,
            grid.origin_x_um + f64::from(grid.width) * grid.grid_size_um,
            grid.origin_y_um + f64::from(grid.height) * grid.grid_size_um,
        ),
    }
}

pub(crate) fn sorted_other_owners_for_cells(
    obstacle_map: &ObstacleMap,
    cells: &[(i32, i32)],
    net_id: u64,
) -> Vec<u64> {
    let mut owners: Vec<u64> = obstacle_map
        .dynamic_owners_for_cells(cells)
        .into_iter()
        .filter(|owner| *owner != net_id)
        .collect();
    owners.sort_unstable();
    owners
}

pub(crate) fn cells_with_other_dynamic_owner(
    obstacle_map: &ObstacleMap,
    cells: &[(i32, i32)],
    clearance_exempt_keys: &FxHashSet<CellKey>,
    net_id: u64,
) -> Vec<(i32, i32)> {
    cells
        .iter()
        .copied()
        .filter(|&(x, y)| {
            if !obstacle_map.in_bounds(x, y) || !obstacle_map.is_dynamic_blocked(x, y) {
                return false;
            }
            let key = pack_xy(x, y);
            let allowed_clearance_overlap =
                clearance_exempt_keys.contains(&key) && !obstacle_map.is_dynamic_core_blocked(x, y);
            if allowed_clearance_overlap {
                return false;
            }
            obstacle_map
                .dynamic_owners_for_cells(&[(x, y)])
                .into_iter()
                .any(|owner| owner != net_id)
        })
        .collect()
}

/// Unit tests for the cell-set helpers, Milestone 6 Slice 3 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
/// Every expectation is a literal cell set hand-computed on a 5x5 grid of
/// 1 um cells whose origin is (0, 0), so a change in any of these
/// functions' geometry shows up as a changed list, not a changed count.
#[cfg(test)]
mod tests {
    use super::*;

    /// 5x5 grid, 1 um cells, origin at (0, 0): cell (x, y) covers
    /// [x, x+1) x [y, y+1) um.
    fn grid_5x5() -> StaticGridSpec {
        StaticGridSpec {
            width: 5,
            height: 5,
            grid_size_um: 1.0,
            origin: (0.0, 0.0),
            die_bbox: (0.0, 0.0, 5.0, 5.0),
        }
    }

    /// The three-cell horizontal route used by the inflation tests.
    fn three_cell_route() -> Vec<(i32, i32)> {
        vec![(1, 1), (2, 1), (3, 1)]
    }

    /// Every cell with `x` in `xs` and `y` in `ys`, sorted.
    fn band(
        xs: std::ops::RangeInclusive<i32>,
        ys: std::ops::RangeInclusive<i32>,
    ) -> Vec<(i32, i32)> {
        let mut out: Vec<(i32, i32)> = Vec::new();
        for x in xs {
            for y in ys.clone() {
                out.push((x, y));
            }
        }
        out.sort_unstable();
        out
    }

    fn sorted(mut cells: Vec<(i32, i32)>) -> Vec<(i32, i32)> {
        cells.sort_unstable();
        cells
    }

    #[test]
    fn inflate_route_cells_at_radius_one_is_the_three_by_five_band_around_the_route() {
        // (1,1), (2,1) and (3,1) each contribute their 3x3 box; the union
        // is x in 0..=4 (1-1 to 3+1) by y in 0..=2 (1-1 to 1+1) -- 15
        // cells, none of them clipped by the 5x5 bounds.
        let inflated = inflate_route_cells(&three_cell_route(), 1, 5, 5);
        assert_eq!(inflated.len(), 15);
        assert_eq!(sorted(inflated), band(0..=4, 0..=2));
    }

    #[test]
    fn inflate_route_cells_clips_to_the_grid_and_dedupes() {
        // A route in the bottom-left corner: the radius-1 box around (0,0)
        // reaches x=-1 and y=-1, which are dropped.
        let inflated = inflate_route_cells(&[(0, 0), (0, 0), (1, 0)], 1, 5, 5);
        assert_eq!(
            sorted(inflated),
            vec![(0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (2, 1)]
        );
    }

    #[test]
    fn inflate_route_cells_at_radius_zero_only_filters_out_of_bounds_cells() {
        let inflated = inflate_route_cells(&[(-1, 2), (2, 2), (5, 2)], 0, 5, 5);
        assert_eq!(inflated, vec![(2, 2)]);
    }

    #[test]
    fn route_core_cells_is_inflate_route_cells_at_the_core_radius() {
        let route = three_cell_route();
        assert_eq!(
            sorted(route_core_cells(&route, 1, 5, 5)),
            band(0..=4, 0..=2)
        );
        assert_eq!(sorted(route_core_cells(&route, 0, 5, 5)), sorted(route));
    }

    #[test]
    fn route_commit_cells_without_exempt_cells_is_the_clearance_inflation() {
        // core radius 0, clearance radius 1: the blocked set is the
        // radius-1 band, the same 15 cells `inflate_route_cells` gives.
        let commit = route_commit_cells(&three_cell_route(), 0, 1, None, 5, 5);
        assert_eq!(sorted(commit), band(0..=4, 0..=2));
    }

    #[test]
    fn route_commit_cells_uses_the_larger_of_the_core_and_clearance_radii() {
        // clearance radius 0 < core radius 1: the clearance inflation is
        // taken at radius 1, so the core is never left unblocked.
        let commit = route_commit_cells(&three_cell_route(), 1, 0, None, 5, 5);
        assert_eq!(sorted(commit), band(0..=4, 0..=2));
    }

    #[test]
    fn route_commit_cells_drops_exempt_cells_that_are_not_core_cells() {
        // core radius 0 => the core is the route itself, {(1,1),(2,1),(3,1)}.
        // (0,0) is exempt and not core, so it goes; (1,1) is exempt but
        // core, so it stays. 15 - 1 = 14 cells.
        let commit = route_commit_cells(&three_cell_route(), 0, 1, Some(&[(0, 0), (1, 1)]), 5, 5);
        assert_eq!(commit.len(), 14);
        let mut expected = band(0..=4, 0..=2);
        expected.retain(|cell| *cell != (0, 0));
        assert_eq!(sorted(commit), expected);
    }

    #[test]
    fn route_commit_cells_with_an_empty_exempt_slice_keeps_every_blocked_cell() {
        let commit = route_commit_cells(&three_cell_route(), 0, 1, Some(&[]), 5, 5);
        assert_eq!(sorted(commit), band(0..=4, 0..=2));
    }

    #[test]
    fn route_port_footprint_cells_starts_one_cell_ahead_of_the_port_and_runs_forward_only() {
        // Port at (1.5, 1.5) um facing +x (orientation 0): the state cell
        // is one grid step ahead, floor(2.5) = (2, 1). With length 2 and
        // half width 1 the footprint is the forward half-boxes of (2,1)
        // and (3,1): x in 2..=4 by y in 0..=2, nine cells. The cells at
        // x=1 are behind the port and are dropped by the forward
        // projection test.
        let grid = grid_5x5();
        let cells = route_port_footprint_cells(&grid, 1.5, 1.5, Some(0.0), 2, 1);
        assert_eq!(sorted_cells(cells), band(2..=4, 0..=2));
    }

    #[test]
    fn route_port_footprint_cells_at_zero_length_is_just_the_state_cell() {
        let grid = grid_5x5();
        let cells = route_port_footprint_cells(&grid, 1.5, 1.5, Some(0.0), 0, 1);
        assert_eq!(sorted_cells(cells), vec![(2, 1)]);
    }

    #[test]
    fn route_port_state_cell_steps_one_grid_cell_along_the_port_orientation() {
        let grid = grid_5x5();
        assert_eq!(route_port_state_cell(&grid, 1.5, 1.5, Some(0.0)), (2, 1));
        assert_eq!(route_port_state_cell(&grid, 1.5, 1.5, Some(90.0)), (1, 2));
        assert_eq!(route_port_state_cell(&grid, 1.5, 1.5, Some(180.0)), (0, 1));
        assert_eq!(route_port_state_cell(&grid, 1.5, 1.5, Some(270.0)), (1, 0));
        // No orientation is treated as 0 degrees.
        assert_eq!(route_port_state_cell(&grid, 1.5, 1.5, None), (2, 1));
    }

    #[test]
    fn route_dynamic_clearance_exempt_cells_at_radius_zero_is_exactly_the_opened_run_in() {
        // Source (1,2) heading +x, target (3,2) with arrival heading +x
        // (so its run-in walks back along -x). The opened cells are the
        // straight line (1,2), (2,2), (3,2); with commit radius 0 the
        // endpoint boxes collapse to the anchors and each corridor walks
        // while the opened cells continue, stopping at (4,2) resp. (0,2).
        let grid = grid_5x5();
        let opened = pack_cells(&[(1, 2), (2, 2), (3, 2)]);
        let cells = route_dynamic_clearance_exempt_cells(
            &grid,
            &opened,
            PyState::new(1, 2, 0),
            PyState::new(3, 2, 0),
            0,
            0,
        );
        assert_eq!(sorted_cells(cells), vec![(1, 2), (2, 2), (3, 2)]);
    }

    #[test]
    fn route_dynamic_clearance_exempt_cells_at_radius_one_is_the_inflated_run_in_band() {
        // Same geometry at commit radius 1 and minimum run-in 1: each
        // anchor contributes its 3x3 box plus the radius-1 inflation of
        // its three-cell corridor. Source gives x 0..=4 by y 1..=3 and so
        // does the target; the union is that 15-cell band.
        let grid = grid_5x5();
        let opened = pack_cells(&[(1, 2), (2, 2), (3, 2)]);
        let cells = route_dynamic_clearance_exempt_cells(
            &grid,
            &opened,
            PyState::new(1, 2, 0),
            PyState::new(3, 2, 0),
            1,
            1,
        );
        assert_eq!(sorted_cells(cells), band(0..=4, 1..=3));
    }

    #[test]
    fn route_dynamic_clearance_exempt_cells_honours_the_minimum_run_in_without_opened_cells() {
        // No opened cells at all: the corridor still runs
        // `min_run_in_length_cells` cells (the port lane). Source (1,2)
        // heading +x with radius 0 and minimum run-in 2 gives (1,2) and
        // (2,2); the target at (3,2) heading +x gives (3,2) and (2,2).
        let grid = grid_5x5();
        let opened: FxHashSet<CellKey> = FxHashSet::default();
        let cells = route_dynamic_clearance_exempt_cells(
            &grid,
            &opened,
            PyState::new(1, 2, 0),
            PyState::new(3, 2, 0),
            0,
            2,
        );
        assert_eq!(sorted_cells(cells), vec![(1, 2), (2, 2), (3, 2)]);
    }

    #[test]
    fn pack_cells_packs_and_dedupes() {
        let keys = pack_cells(&[(1, 2), (3, 4), (1, 2)]);
        assert_eq!(keys.len(), 2);
        assert!(keys.contains(&pack_xy(1, 2)));
        assert!(keys.contains(&pack_xy(3, 4)));
        // The packing itself: x in the high 32 bits, y in the low 32.
        assert_eq!(pack_xy(1, 2), (1u64 << 32) | 2);
    }

    #[test]
    fn unique_cells_keeps_the_first_occurrence_order() {
        assert_eq!(
            unique_cells(vec![(1, 1), (2, 2), (1, 1), (3, 3), (2, 2)]),
            vec![(1, 1), (2, 2), (3, 3)]
        );
    }

    #[test]
    fn sorted_cells_sorts_by_x_then_y() {
        assert_eq!(
            sorted_cells(pack_cells(&[(2, 1), (0, 3), (2, 0)])),
            vec![(0, 3), (2, 0), (2, 1)]
        );
    }
}
