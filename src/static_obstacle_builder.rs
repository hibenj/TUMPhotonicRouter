//! Rust implementation of static obstacle-map construction.
//!
//! Python/gdsfactory remains responsible for extracting polygons and ports from
//! layout objects. This module owns the performance-sensitive grid work:
//! bounding-box/grid creation, polygon rasterization, clearance inflation, port
//! openings, and static [`ObstacleMap`] population.

use std::time::Instant;

use pyo3::prelude::*;
use pyo3::types::PyDict;
use rustc_hash::FxHashSet;

use crate::obstacle_map::{pack_xy, unpack_xy, CellKey, ClearanceMetric, ObstacleMap};

/// Physical point in micrometers.
pub type Point = (f64, f64);

/// Physical polygon in micrometers.
pub type Polygon = Vec<Point>;

/// Physical bounding box `(xmin, ymin, xmax, ymax)` in micrometers.
pub type BBox = (f64, f64, f64, f64);

/// Grid cell `(x, y)`.
pub type GridCell = (i32, i32);

/// Router-facing port data extracted by Python.
#[derive(Clone, Debug)]
pub struct PortInput {
    pub name: String,
    pub x: f64,
    pub y: f64,
    pub orientation: Option<f64>,
}

impl PortInput {
    pub fn new(name: String, x: f64, y: f64, orientation: Option<f64>) -> Self {
        Self {
            name,
            x,
            y,
            orientation,
        }
    }
}

/// Configuration for static obstacle-map construction.
#[derive(Clone, Debug)]
pub struct StaticObstacleBuildConfig {
    pub grid_size_um: f64,
    pub security_margin_um: f64,
    pub clearance_um: f64,
    pub clearance_metric: ClearanceMetric,
    pub port_open_radius_um: f64,
    pub die_bbox: Option<BBox>,
}

impl Default for StaticObstacleBuildConfig {
    fn default() -> Self {
        Self {
            grid_size_um: 0.4,
            security_margin_um: 20.0,
            clearance_um: 0.5,
            clearance_metric: ClearanceMetric::Chebyshev,
            port_open_radius_um: 0.5,
            die_bbox: None,
        }
    }
}

/// Rectangular grid definition used by the Rust router.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StaticGridSpec {
    pub width: i32,
    pub height: i32,
    pub grid_size_um: f64,
    pub origin: Point,
    pub die_bbox: BBox,
}

/// Timing and size data for one static obstacle build.
#[derive(Clone, Debug, Default)]
pub struct StaticObstacleBuildStats {
    pub bbox_time_s: f64,
    pub rasterization_time_s: f64,
    pub clearance_time_s: f64,
    pub port_opening_time_s: f64,
    pub obstacle_map_time_s: f64,
    pub total_time_s: f64,
    pub polygon_count: usize,
    pub port_count: usize,
    pub raw_blocked_cell_count: usize,
    pub blocked_cell_count: usize,
    pub port_open_cell_count: usize,
}

/// Static obstacle build result.
#[derive(Clone, Debug)]
pub struct StaticObstacleBuildResult {
    pub grid: StaticGridSpec,
    pub raw_blocked_cells: Vec<GridCell>,
    pub blocked_cells: Vec<GridCell>,
    pub port_open_cells: Vec<GridCell>,
    pub obstacle_map: ObstacleMap,
    pub stats: StaticObstacleBuildStats,
}

/// Build the static obstacle map from extracted physical polygons and ports.
pub fn build_static_obstacle_map_from_geometry(
    polygons: &[Polygon],
    ports: &[PortInput],
    config: &StaticObstacleBuildConfig,
) -> Result<StaticObstacleBuildResult, String> {
    validate_config(config)?;

    let total_start = Instant::now();
    let mut stats = StaticObstacleBuildStats {
        polygon_count: polygons.len(),
        port_count: ports.len(),
        ..Default::default()
    };

    let bbox_start = Instant::now();
    let layout_bbox = compute_bbox(polygons, ports);
    let die_bbox = config
        .die_bbox
        .unwrap_or_else(|| expand_bbox(layout_bbox, config.security_margin_um));
    let grid = make_grid_spec(die_bbox, config.grid_size_um)?;
    stats.bbox_time_s = elapsed_s(bbox_start);

    let raster_start = Instant::now();
    let mut raw_blocked_keys = FxHashSet::default();
    for polygon in polygons {
        rasterize_polygon_into(polygon, &grid, &mut raw_blocked_keys);
    }
    stats.rasterization_time_s = elapsed_s(raster_start);
    stats.raw_blocked_cell_count = raw_blocked_keys.len();

    let clearance_start = Instant::now();
    let clearance_radius = ceil_to_i32(config.clearance_um / config.grid_size_um)?;
    let mut blocked_keys = inflate_keys(
        raw_blocked_keys.iter().copied(),
        grid.width,
        grid.height,
        clearance_radius,
        config.clearance_metric,
    );
    stats.clearance_time_s = elapsed_s(clearance_start);

    let port_start = Instant::now();
    let port_radius = ceil_to_i32(config.port_open_radius_um / config.grid_size_um)?;
    let port_base_keys = ports
        .iter()
        .map(|port| physical_to_grid(port.x, port.y, &grid))
        .map(|(x, y)| pack_xy(x, y));
    let port_open_keys = inflate_keys(
        port_base_keys,
        grid.width,
        grid.height,
        port_radius,
        ClearanceMetric::Chebyshev,
    );
    for key in &port_open_keys {
        blocked_keys.remove(key);
    }
    stats.port_opening_time_s = elapsed_s(port_start);
    stats.port_open_cell_count = port_open_keys.len();
    stats.blocked_cell_count = blocked_keys.len();

    let map_start = Instant::now();
    let blocked_cells = sorted_cells_from_keys(&blocked_keys);
    let mut obstacle_map = ObstacleMap::new(grid.width, grid.height);
    obstacle_map.add_static_cells(&blocked_cells);
    stats.obstacle_map_time_s = elapsed_s(map_start);

    let raw_blocked_cells = sorted_cells_from_keys(&raw_blocked_keys);
    let port_open_cells = sorted_cells_from_keys(&port_open_keys);
    stats.total_time_s = elapsed_s(total_start);

    Ok(StaticObstacleBuildResult {
        grid,
        raw_blocked_cells,
        blocked_cells,
        port_open_cells,
        obstacle_map,
        stats,
    })
}

/// Compute bbox across all polygon vertices and ports.
pub fn compute_bbox(polygons: &[Polygon], ports: &[PortInput]) -> BBox {
    let mut xmin = f64::INFINITY;
    let mut ymin = f64::INFINITY;
    let mut xmax = f64::NEG_INFINITY;
    let mut ymax = f64::NEG_INFINITY;

    for polygon in polygons {
        for &(x, y) in polygon {
            xmin = xmin.min(x);
            ymin = ymin.min(y);
            xmax = xmax.max(x);
            ymax = ymax.max(y);
        }
    }

    for port in ports {
        xmin = xmin.min(port.x);
        ymin = ymin.min(port.y);
        xmax = xmax.max(port.x);
        ymax = ymax.max(port.y);
    }

    if xmin.is_infinite() {
        (0.0, 0.0, 0.0, 0.0)
    } else {
        (xmin, ymin, xmax, ymax)
    }
}

/// Expand a physical bbox by `margin_um` on all sides.
pub fn expand_bbox(bbox: BBox, margin_um: f64) -> BBox {
    let (xmin, ymin, xmax, ymax) = bbox;
    (
        xmin - margin_um,
        ymin - margin_um,
        xmax + margin_um,
        ymax + margin_um,
    )
}

/// Create the grid spec for a die bbox.
pub fn make_grid_spec(die_bbox: BBox, grid_size_um: f64) -> Result<StaticGridSpec, String> {
    let (xmin, ymin, xmax, ymax) = die_bbox;
    if grid_size_um <= 0.0 {
        return Err("grid_size_um must be positive".to_string());
    }
    if xmax < xmin || ymax < ymin {
        return Err(format!("invalid die bbox: {die_bbox:?}"));
    }

    let width = ceil_to_i32((xmax - xmin) / grid_size_um)?;
    let height = ceil_to_i32((ymax - ymin) / grid_size_um)?;

    Ok(StaticGridSpec {
        width,
        height,
        grid_size_um,
        origin: (xmin, ymin),
        die_bbox,
    })
}

/// Convert physical micrometer coordinates to integer grid coordinates.
pub fn physical_to_grid(x: f64, y: f64, grid: &StaticGridSpec) -> GridCell {
    let gx = ((x - grid.origin.0) / grid.grid_size_um).floor() as i32;
    let gy = ((y - grid.origin.1) / grid.grid_size_um).floor() as i32;
    (gx, gy)
}

/// Return the physical center of a grid cell.
pub fn grid_cell_center(gx: i32, gy: i32, grid: &StaticGridSpec) -> Point {
    (
        grid.origin.0 + (gx as f64 + 0.5) * grid.grid_size_um,
        grid.origin.1 + (gy as f64 + 0.5) * grid.grid_size_um,
    )
}

/// Rasterize one polygon into packed cell keys using cell-center tests.
pub fn rasterize_polygon(polygon: &Polygon, grid: &StaticGridSpec) -> FxHashSet<CellKey> {
    let mut cells = FxHashSet::default();
    rasterize_polygon_into(polygon, grid, &mut cells);
    cells
}

fn rasterize_polygon_into(
    polygon: &Polygon,
    grid: &StaticGridSpec,
    cells: &mut FxHashSet<CellKey>,
) {
    if polygon.len() < 3 || grid.width <= 0 || grid.height <= 0 {
        return;
    }

    let (mut min_x, mut min_y) = (f64::INFINITY, f64::INFINITY);
    let (mut max_x, mut max_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &(x, y) in polygon {
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }

    let (mut gx_min, mut gy_min) = physical_to_grid(min_x, min_y, grid);
    let (mut gx_max, mut gy_max) = physical_to_grid(max_x, max_y, grid);
    gx_min = gx_min.max(0);
    gy_min = gy_min.max(0);
    gx_max = gx_max.min(grid.width - 1);
    gy_max = gy_max.min(grid.height - 1);

    if gx_min > gx_max || gy_min > gy_max {
        return;
    }

    for gx in gx_min..=gx_max {
        for gy in gy_min..=gy_max {
            if point_in_polygon(grid_cell_center(gx, gy, grid), polygon) {
                cells.insert(pack_xy(gx, gy));
            }
        }
    }
}

/// Return true when `point` is inside or on the boundary of `polygon`.
pub fn point_in_polygon(point: Point, polygon: &[Point]) -> bool {
    let (x, y) = point;
    let mut inside = false;
    let count = polygon.len();

    for i in 0..count {
        let (x1, y1) = polygon[i];
        let (x2, y2) = polygon[(i + 1) % count];

        if point_on_segment(x, y, x1, y1, x2, y2) {
            return true;
        }

        let crosses = (y1 > y) != (y2 > y);
        if crosses {
            let x_intersection = (x2 - x1) * (y - y1) / (y2 - y1) + x1;
            if x <= x_intersection {
                inside = !inside;
            }
        }
    }

    inside
}

/// Expand packed cells by the requested metric, clipped to bounds.
pub fn inflate_keys<I>(
    cells: I,
    width: i32,
    height: i32,
    radius: i32,
    metric: ClearanceMetric,
) -> FxHashSet<CellKey>
where
    I: IntoIterator<Item = CellKey>,
{
    assert!(radius >= 0, "radius must be non-negative");
    let mut inflated = FxHashSet::default();

    for key in cells {
        let (gx, gy) = unpack_xy(key);
        if !in_bounds(gx, gy, width, height) {
            continue;
        }

        for dx in -radius..=radius {
            for dy in -radius..=radius {
                let include = match metric {
                    ClearanceMetric::Manhattan => dx.abs() + dy.abs() <= radius,
                    ClearanceMetric::Chebyshev => dx.abs().max(dy.abs()) <= radius,
                };

                if !include {
                    continue;
                }

                let nx = gx + dx;
                let ny = gy + dy;
                if in_bounds(nx, ny, width, height) {
                    inflated.insert(pack_xy(nx, ny));
                }
            }
        }
    }

    inflated
}

#[pyfunction]
#[pyo3(signature = (
    polygons,
    ports,
    grid_size_um,
    security_margin_um,
    clearance_um,
    clearance_metric,
    port_open_radius_um,
    die_bbox=None
))]
#[allow(clippy::too_many_arguments)]
fn build_static_obstacle_map_rs(
    py: Python<'_>,
    polygons: Vec<Vec<(f64, f64)>>,
    ports: Vec<(String, f64, f64, Option<f64>)>,
    grid_size_um: f64,
    security_margin_um: f64,
    clearance_um: f64,
    clearance_metric: String,
    port_open_radius_um: f64,
    die_bbox: Option<(f64, f64, f64, f64)>,
) -> PyResult<PyObject> {
    let metric = parse_clearance_metric(&clearance_metric).map_err(pyo3_value_error)?;
    let port_inputs = ports
        .into_iter()
        .map(|(name, x, y, orientation)| PortInput::new(name, x, y, orientation))
        .collect::<Vec<_>>();
    let config = StaticObstacleBuildConfig {
        grid_size_um,
        security_margin_um,
        clearance_um,
        clearance_metric: metric,
        port_open_radius_um,
        die_bbox,
    };

    let result = build_static_obstacle_map_from_geometry(&polygons, &port_inputs, &config)
        .map_err(pyo3_value_error)?;
    build_py_result(py, &result)
}

/// Python extension module used by `python/photonic_router/static_obstacle_builder.py`.
#[pymodule]
pub fn _rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(build_static_obstacle_map_rs, m)?)?;
    Ok(())
}

fn build_py_result(py: Python<'_>, result: &StaticObstacleBuildResult) -> PyResult<PyObject> {
    let dict = PyDict::new_bound(py);
    dict.set_item(
        "grid",
        (
            result.grid.width,
            result.grid.height,
            result.grid.grid_size_um,
            result.grid.origin,
            result.grid.die_bbox,
        ),
    )?;
    dict.set_item("raw_blocked_cells", &result.raw_blocked_cells)?;
    dict.set_item("blocked_cells", &result.blocked_cells)?;
    dict.set_item("port_open_cells", &result.port_open_cells)?;

    let stats = PyDict::new_bound(py);
    stats.set_item("bbox_time_s", result.stats.bbox_time_s)?;
    stats.set_item("rasterization_time_s", result.stats.rasterization_time_s)?;
    stats.set_item("clearance_time_s", result.stats.clearance_time_s)?;
    stats.set_item("port_opening_time_s", result.stats.port_opening_time_s)?;
    stats.set_item("obstacle_map_time_s", result.stats.obstacle_map_time_s)?;
    stats.set_item("total_time_s", result.stats.total_time_s)?;
    stats.set_item("polygon_count", result.stats.polygon_count)?;
    stats.set_item("port_count", result.stats.port_count)?;
    stats.set_item(
        "raw_blocked_cell_count",
        result.stats.raw_blocked_cell_count,
    )?;
    stats.set_item("blocked_cell_count", result.stats.blocked_cell_count)?;
    stats.set_item("port_open_cell_count", result.stats.port_open_cell_count)?;
    dict.set_item("stats", stats)?;

    Ok(dict.into_py(py))
}

fn parse_clearance_metric(metric: &str) -> Result<ClearanceMetric, String> {
    match metric.to_ascii_lowercase().as_str() {
        "manhattan" => Ok(ClearanceMetric::Manhattan),
        "chebyshev" => Ok(ClearanceMetric::Chebyshev),
        _ => Err("clearance_metric must be 'manhattan' or 'chebyshev'".to_string()),
    }
}

fn validate_config(config: &StaticObstacleBuildConfig) -> Result<(), String> {
    if config.grid_size_um <= 0.0 {
        return Err("grid_size_um must be positive".to_string());
    }
    if config.clearance_um < 0.0 {
        return Err("clearance_um must be non-negative".to_string());
    }
    if config.port_open_radius_um < 0.0 {
        return Err("port_open_radius_um must be non-negative".to_string());
    }
    Ok(())
}

fn ceil_to_i32(value: f64) -> Result<i32, String> {
    if !value.is_finite() {
        return Err("non-finite grid dimension or radius".to_string());
    }
    if value > i32::MAX as f64 {
        return Err("grid dimension or radius exceeds i32::MAX".to_string());
    }
    Ok(value.ceil().max(0.0) as i32)
}

fn sorted_cells_from_keys(keys: &FxHashSet<CellKey>) -> Vec<GridCell> {
    let mut cells = keys.iter().copied().map(unpack_xy).collect::<Vec<_>>();
    cells.sort_unstable();
    cells
}

#[inline]
fn in_bounds(gx: i32, gy: i32, width: i32, height: i32) -> bool {
    gx >= 0 && gy >= 0 && gx < width && gy < height
}

fn point_on_segment(px: f64, py: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> bool {
    const EPS: f64 = 1e-9;
    let cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
    if cross.abs() > EPS {
        return false;
    }

    px >= x1.min(x2) - EPS
        && px <= x1.max(x2) + EPS
        && py >= y1.min(y2) - EPS
        && py <= y1.max(y2) + EPS
}

fn elapsed_s(start: Instant) -> f64 {
    start.elapsed().as_secs_f64()
}

fn pyo3_value_error(message: String) -> PyErr {
    pyo3::exceptions::PyValueError::new_err(message)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_grid() -> StaticGridSpec {
        StaticGridSpec {
            width: 5,
            height: 5,
            grid_size_um: 1.0,
            origin: (0.0, 0.0),
            die_bbox: (0.0, 0.0, 5.0, 5.0),
        }
    }

    #[test]
    fn computes_bbox_from_polygons_and_ports() {
        let polygons = vec![vec![(1.0, 2.0), (3.0, 2.0), (3.0, 4.0)]];
        let ports = vec![PortInput::new("o1".to_string(), -1.0, 5.0, Some(0.0))];

        assert_eq!(compute_bbox(&polygons, &ports), (-1.0, 2.0, 3.0, 5.0));
    }

    #[test]
    fn converts_physical_coordinates_to_grid() {
        let grid = StaticGridSpec {
            width: 10,
            height: 10,
            grid_size_um: 0.5,
            origin: (-1.0, 2.0),
            die_bbox: (-1.0, 2.0, 4.0, 7.0),
        };

        assert_eq!(physical_to_grid(-1.0, 2.0, &grid), (0, 0));
        assert_eq!(physical_to_grid(-0.51, 2.49, &grid), (0, 0));
        assert_eq!(physical_to_grid(0.0, 3.0, &grid), (2, 2));
    }

    #[test]
    fn rasterizes_polygon_by_cell_centers() {
        let grid = StaticGridSpec {
            width: 2,
            height: 2,
            grid_size_um: 1.0,
            origin: (0.0, 0.0),
            die_bbox: (0.0, 0.0, 2.0, 2.0),
        };
        let polygon = vec![(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)];
        let cells = sorted_cells_from_keys(&rasterize_polygon(&polygon, &grid));

        assert_eq!(cells, vec![(0, 0), (0, 1), (1, 0), (1, 1)]);
    }

    #[test]
    fn inflates_with_manhattan_metric() {
        let inflated = inflate_keys([pack_xy(2, 2)], 5, 5, 1, ClearanceMetric::Manhattan);
        let cells = sorted_cells_from_keys(&inflated);

        assert_eq!(cells, vec![(1, 2), (2, 1), (2, 2), (2, 3), (3, 2)]);
    }

    #[test]
    fn inflates_with_chebyshev_metric() {
        let inflated = inflate_keys([pack_xy(2, 2)], 5, 5, 1, ClearanceMetric::Chebyshev);
        let cells = sorted_cells_from_keys(&inflated);

        assert_eq!(
            cells,
            vec![
                (1, 1),
                (1, 2),
                (1, 3),
                (2, 1),
                (2, 2),
                (2, 3),
                (3, 1),
                (3, 2),
                (3, 3),
            ]
        );
    }

    #[test]
    fn generates_port_openings() {
        let polygons = Vec::new();
        let ports = vec![PortInput::new("o1".to_string(), 1.0, 1.0, Some(0.0))];
        let config = StaticObstacleBuildConfig {
            grid_size_um: 1.0,
            security_margin_um: 0.0,
            clearance_um: 0.0,
            clearance_metric: ClearanceMetric::Chebyshev,
            port_open_radius_um: 1.0,
            die_bbox: Some((0.0, 0.0, 3.0, 3.0)),
        };

        let result = build_static_obstacle_map_from_geometry(&polygons, &ports, &config).unwrap();

        assert_eq!(
            result.port_open_cells,
            vec![
                (0, 0),
                (0, 1),
                (0, 2),
                (1, 0),
                (1, 1),
                (1, 2),
                (2, 0),
                (2, 1),
                (2, 2),
            ]
        );
    }

    #[test]
    fn removes_port_openings_from_permanent_blocked_cells() {
        let polygons = vec![vec![(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]];
        let ports = vec![PortInput::new("o1".to_string(), 1.0, 1.0, Some(0.0))];
        let config = StaticObstacleBuildConfig {
            grid_size_um: 1.0,
            security_margin_um: 0.0,
            clearance_um: 0.0,
            clearance_metric: ClearanceMetric::Chebyshev,
            port_open_radius_um: 0.0,
            die_bbox: Some((0.0, 0.0, 3.0, 3.0)),
        };

        let result = build_static_obstacle_map_from_geometry(&polygons, &ports, &config).unwrap();

        assert!(result.raw_blocked_cells.contains(&(1, 1)));
        assert!(result.port_open_cells.contains(&(1, 1)));
        assert!(!result.blocked_cells.contains(&(1, 1)));
    }

    #[test]
    fn builds_obstacle_map_from_result() {
        let polygons = vec![vec![(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]];
        let ports = Vec::new();
        let config = StaticObstacleBuildConfig {
            grid_size_um: 1.0,
            security_margin_um: 0.0,
            clearance_um: 0.0,
            clearance_metric: ClearanceMetric::Chebyshev,
            port_open_radius_um: 0.0,
            die_bbox: Some((0.0, 0.0, 2.0, 2.0)),
        };

        let result = build_static_obstacle_map_from_geometry(&polygons, &ports, &config).unwrap();

        assert!(result.obstacle_map.is_static_blocked(0, 0));
        assert!(result.obstacle_map.is_static_blocked(1, 1));
        assert_eq!(result.blocked_cells.len(), 4);
    }

    #[test]
    fn has_test_grid_helper() {
        assert_eq!(test_grid().width, 5);
    }
}
