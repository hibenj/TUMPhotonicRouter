//! The static obstacle-map builder's Python surface: the
//! `build_static_obstacle_map_rs` function (the core's
//! `build_static_obstacle_map_from_geometry` with its result as a Python
//! dictionary) and the `PyStaticCellSet` handle (the core's
//! `StaticCellSet`).

// pyo3 0.22's `#[pymethods]` / `#[pyfunction]` expansion converts the returned
// `PyErr` into `PyErr`, which clippy reports at every binding; nothing here to fix.
#![allow(clippy::useless_conversion)]

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use router_core::static_obstacle_builder::{
    build_static_obstacle_map_from_geometry, parse_clearance_metric, parse_obstacle_mode, GridCell,
    PortInput, StaticCellSet, StaticObstacleBuildConfig, StaticObstacleBuildResult,
};

/// Python-visible handle for passing packed static cells back into Rust without
/// rematerializing a Python list of `(x, y)` tuples.
#[pyclass(name = "PyStaticCellSet")]
#[derive(Clone)]
pub struct PyStaticCellSet {
    pub(crate) inner: StaticCellSet,
}

#[pymethods]
impl PyStaticCellSet {
    #[new]
    fn new(cells: Vec<GridCell>) -> Self {
        Self {
            inner: StaticCellSet::from_cells(&cells),
        }
    }

    fn __len__(&self) -> usize {
        self.inner.len()
    }

    fn cells(&self) -> Vec<GridCell> {
        self.inner.cells()
    }

    fn merged_with(&self, other: PyRef<'_, PyStaticCellSet>) -> Self {
        Self {
            inner: self.inner.merged_with(&other.inner),
        }
    }

    fn merged_with_cells(&self, cells: Vec<GridCell>) -> Self {
        Self {
            inner: self.inner.merged_with_cells(cells),
        }
    }

    fn merged_with_rects(&self, rects: Vec<(i32, i32, i32, i32)>, width: i32, height: i32) -> Self {
        Self {
            inner: self.inner.merged_with_rects(rects, width, height),
        }
    }
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
    die_bbox=None,
    obstacle_mode="bounding_boxes",
    clear_port_open_cells_from_static=false,
    materialize_bbox_cells=true,
    populate_obstacle_map=true,
    materialize_cell_sets=true
))]
#[allow(clippy::too_many_arguments)]
pub(crate) fn build_static_obstacle_map_rs(
    py: Python<'_>,
    polygons: Vec<Vec<(f64, f64)>>,
    ports: Vec<(String, f64, f64, Option<f64>)>,
    grid_size_um: f64,
    security_margin_um: f64,
    clearance_um: f64,
    clearance_metric: String,
    port_open_radius_um: f64,
    die_bbox: Option<(f64, f64, f64, f64)>,
    obstacle_mode: &str,
    clear_port_open_cells_from_static: bool,
    materialize_bbox_cells: bool,
    populate_obstacle_map: bool,
    materialize_cell_sets: bool,
) -> PyResult<PyObject> {
    let metric = parse_clearance_metric(&clearance_metric).map_err(PyValueError::new_err)?;
    let mode = parse_obstacle_mode(obstacle_mode).map_err(PyValueError::new_err)?;
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
        obstacle_mode: mode,
        clear_port_open_cells_from_static,
        materialize_bbox_cells,
        populate_obstacle_map,
        die_bbox,
    };

    let result = build_static_obstacle_map_from_geometry(&polygons, &port_inputs, &config)
        .map_err(PyValueError::new_err)?;
    build_py_result(py, &result, materialize_cell_sets)
}

fn build_py_result(
    py: Python<'_>,
    result: &StaticObstacleBuildResult,
    materialize_cell_sets: bool,
) -> PyResult<PyObject> {
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
    if materialize_cell_sets {
        dict.set_item("raw_blocked_cells", &result.raw_blocked_cells)?;
        dict.set_item("blocked_cells", &result.blocked_cells)?;
        dict.set_item("port_open_cells", &result.port_open_cells)?;
    } else {
        dict.set_item("raw_blocked_cells", Vec::<GridCell>::new())?;
        dict.set_item("blocked_cells", Vec::<GridCell>::new())?;
        dict.set_item("port_open_cells", Vec::<GridCell>::new())?;
    }
    dict.set_item(
        "blocked_cell_handle",
        Py::new(
            py,
            PyStaticCellSet {
                inner: StaticCellSet::from_cells(&result.blocked_cells),
            },
        )?,
    )?;
    let raw_static_rects: Vec<(i32, i32, i32, i32)> = result
        .raw_static_rects
        .iter()
        .map(|r| (r.x_min, r.y_min, r.x_max, r.y_max))
        .collect();
    let blocked_static_rects: Vec<(i32, i32, i32, i32)> = result
        .blocked_static_rects
        .iter()
        .map(|r| (r.x_min, r.y_min, r.x_max, r.y_max))
        .collect();
    dict.set_item("raw_static_rects", &raw_static_rects)?;
    dict.set_item("blocked_static_rects", &blocked_static_rects)?;

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
    stats.set_item("static_box_count", result.stats.static_box_count)?;
    stats.set_item("raw_box_area_cells", result.stats.raw_box_area_cells)?;
    stats.set_item(
        "blocked_cells_after_port_opening",
        result.stats.blocked_cells_after_port_opening,
    )?;
    stats.set_item("port_open_cell_count", result.stats.port_open_cell_count)?;
    stats.set_item(
        "clear_port_open_cells_from_static",
        result.stats.clear_port_open_cells_from_static,
    )?;
    dict.set_item("stats", stats)?;

    Ok(dict.into_py(py))
}
