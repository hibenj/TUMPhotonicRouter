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
