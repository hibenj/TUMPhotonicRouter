//! PyO3 wrapper for the Rust A* router.

use pyo3::prelude::*;
use pyo3::types::PyDict;
use rustc_hash::FxHashSet;

use crate::astar::{export_route_svg, route_single_net_with_config, AStarConfig, RouteResult, State};
use crate::obstacle_map::{pack_xy, ObstacleMap};
use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};

fn build_config(
    max_iterations: Option<usize>,
    bend_weight: Option<f64>,
    target_tolerance_cells: Option<i32>,
) -> AStarConfig {
    let mut config = AStarConfig::default();
    if let Some(value) = max_iterations {
        config.max_iterations = value;
    }
    if let Some(value) = bend_weight {
        config.bend_weight = value;
    }
    if let Some(value) = target_tolerance_cells {
        config.target_tolerance_cells = value;
    }
    config
}

fn pack_cells(cells: Vec<(i32, i32)>) -> FxHashSet<u64> {
    let mut packed = FxHashSet::default();
    for (x, y) in cells {
        packed.insert(pack_xy(x, y));
    }
    packed
}

fn build_result<'py>(
    py: Python<'py>,
    result: &RouteResult,
    svg: Option<String>,
) -> PyResult<PyObject> {
    let dict = PyDict::new_bound(py);
    let states = result
        .states
        .iter()
        .map(|state| (state.x, state.y, state.angle))
        .collect::<Vec<_>>();

    dict.set_item("states", states)?;
    dict.set_item("primitives", &result.primitives)?;
    dict.set_item("cells", &result.cells)?;
    dict.set_item("total_length_um", result.total_length_um)?;
    dict.set_item("total_cost", result.total_cost)?;
    if let Some(svg_data) = svg {
        dict.set_item("svg", svg_data)?;
    }

    Ok(dict.into())
}

#[pyfunction]
#[pyo3(signature = (
    width,
    height,
    blocked_cells,
    port_open_cells,
    grid_size_um,
    source,
    target,
    *,
    max_iterations=None,
    bend_weight=None,
    target_tolerance_cells=None,
    export_svg=false
))]
#[allow(clippy::too_many_arguments)]
pub fn route_single_net_rs(
    py: Python<'_>,
    width: i32,
    height: i32,
    blocked_cells: Vec<(i32, i32)>,
    port_open_cells: Vec<(i32, i32)>,
    grid_size_um: f64,
    source: (i32, i32, u8),
    target: (i32, i32, u8),
    max_iterations: Option<usize>,
    bend_weight: Option<f64>,
    target_tolerance_cells: Option<i32>,
    export_svg: bool,
) -> PyResult<PyObject> {
    if width < 0 || height < 0 {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "width and height must be non-negative",
        ));
    }

    let mut obstacle_map = ObstacleMap::new(width, height);
    obstacle_map.add_static_cells(&blocked_cells);

    let opened = if port_open_cells.is_empty() {
        None
    } else {
        Some(pack_cells(port_open_cells))
    };

    let primitives = create_photonic_primitive_library(PrimitiveLibraryConfig {
        grid_size_um,
        ..PrimitiveLibraryConfig::default()
    });

    let config = build_config(max_iterations, bend_weight, target_tolerance_cells);
    let source_state = State::new(source.0, source.1, source.2);
    let target_state = State::new(target.0, target.1, target.2);

    let result = route_single_net_with_config(
        &obstacle_map,
        &primitives,
        source_state,
        target_state,
        opened.as_ref(),
        &config,
    )
    .ok_or_else(|| {
        pyo3::exceptions::PyRuntimeError::new_err("No route found for the requested net")
    })?;

    let svg = if export_svg {
        Some(export_route_svg(&obstacle_map, &result))
    } else {
        None
    };

    build_result(py, &result, svg)
}

