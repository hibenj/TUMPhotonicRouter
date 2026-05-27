use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use rustc_hash::FxHashSet;

use crate::astar::{
    export_route_svg, route_single_net_with_config, AStarConfig, RouteResult, RouteSearchStats,
    State,
};
use crate::geometry_realization::{
    plan_auto_analytic_meander_for_route as plan_auto_analytic_meander_for_route_rs,
    realize_route_polygon_from_auto_plan as realize_route_polygon_from_auto_plan_rs,
    realize_route_polygon_with_auto_checked_analytic_meander as realize_route_polygon_with_auto_checked_analytic_meander_rs,
    AutoMeanderConfig, AutoMeanderSidePolicy,
    build_port_access as build_port_access_rs, build_port_accesses as build_port_accesses_rs,
    cells_in_grid_rect as cells_in_grid_rect_rs,
    check_meander_box_free_with_prefix as check_meander_box_free_with_prefix_rs,
    meander_box_to_grid_rect as meander_box_to_grid_rect_rs,
    plan_analytic_meander_for_route as plan_analytic_meander_for_route_rs,
    realize_route_polygon_from_primitives as realize_route_polygon_from_primitives_rs,
    realize_route_polygon_with_analytic_meander as realize_route_polygon_with_analytic_meander_rs,
    realize_route_polygon_with_checked_analytic_meander_box as realize_route_polygon_with_checked_analytic_meander_box_rs,
    DenseOccupancyPrefix,
    realize_route_polygon_with_port_access as realize_route_polygon_with_port_access_rs,
    GeometryGridSpec, GeometryError, PortAccess, PortAccessConfig,
};
#[allow(deprecated)]
use crate::meander::{
    analyze_meander_insertion_candidate as analyze_meander_insertion_candidate_rs,
    actual_bend_radius_um_from_cells as actual_bend_radius_um_from_cells_rs,
    bend_radius_cells_from_min_radius as bend_radius_cells_from_min_radius_rs,
    insert_simple_meander_loop as insert_simple_meander_loop_rs,
    MeanderBox, MeanderPlanningMode, MeanderSide,
};
use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::{
    create_photonic_primitive_library, Primitive, PrimitiveLibrary, PrimitiveLibraryConfig,
};
use crate::static_obstacle_builder::{PortInput, StaticGridSpec};

#[pyclass(name = "GridSpec")]
#[derive(Clone)]
pub struct PyGridSpec {
    #[pyo3(get, set)]
    pub width: u32,
    #[pyo3(get, set)]
    pub height: u32,
    #[pyo3(get, set)]
    pub grid_size_um: f64,
    #[pyo3(get, set)]
    pub origin_x_um: f64,
    #[pyo3(get, set)]
    pub origin_y_um: f64,
}

#[pymethods]
impl PyGridSpec {
    #[new]
    fn new(
        width: u32,
        height: u32,
        grid_size_um: f64,
        origin_x_um: f64,
        origin_y_um: f64,
    ) -> PyResult<Self> {
        if grid_size_um <= 0.0 {
            return Err(PyValueError::new_err("grid_size_um must be > 0"));
        }
        Ok(Self {
            width,
            height,
            grid_size_um,
            origin_x_um,
            origin_y_um,
        })
    }
}

#[pyclass(name = "PrimitiveLibraryConfig")]
#[derive(Clone)]
pub struct PyPrimitiveLibraryConfig {
    #[pyo3(get, set)]
    pub grid_size_um: f64,
    #[pyo3(get, set)]
    pub straight_short_cells: i32,
    #[pyo3(get, set)]
    pub straight_long_cells: i32,
    #[pyo3(get, set)]
    pub bend_radius_cells: i32,
    #[pyo3(get, set)]
    pub allow_45_degree_turns: bool,
    #[pyo3(get, set)]
    pub bend_weight: f64,
}

#[pymethods]
impl PyPrimitiveLibraryConfig {
    #[new]
    #[pyo3(signature=(grid_size_um=0.5,straight_short_cells=1,straight_long_cells=4,bend_radius_cells=2,bend_weight=1.0,allow_45_degree_turns=true))]
    fn new(
        grid_size_um: f64,
        straight_short_cells: i32,
        straight_long_cells: i32,
        bend_radius_cells: i32,
        bend_weight: f64,
        allow_45_degree_turns: bool,
    ) -> Self {
        Self {
            grid_size_um,
            straight_short_cells,
            straight_long_cells,
            bend_radius_cells,
            allow_45_degree_turns,
            bend_weight,
        }
    }

    #[staticmethod]
    fn bend_radius_cells_from_min_radius(min_bend_radius_um: f64, grid_size_um: f64) -> PyResult<i32> {
        bend_radius_cells_from_min_radius_rs(min_bend_radius_um, grid_size_um)
            .map_err(PyValueError::new_err)
    }

    #[staticmethod]
    fn actual_bend_radius_um_from_cells(bend_radius_cells: i32, grid_size_um: f64) -> PyResult<f64> {
        actual_bend_radius_um_from_cells_rs(bend_radius_cells, grid_size_um)
            .map_err(PyValueError::new_err)
    }
}

#[pyclass(name = "AStarConfig")]
#[derive(Clone)]
pub struct PyAStarConfig {
    #[pyo3(get, set)]
    pub max_iterations: usize,
    #[pyo3(get, set)]
    pub bend_weight: f64,
    #[pyo3(get, set)]
    pub target_tolerance_cells: i32,
    #[pyo3(get, set)]
    pub require_target_angle: bool,
    #[pyo3(get, set)]
    pub allowed_target_angles: Option<Vec<u8>>,
    #[pyo3(get, set)]
    pub use_routing_window: bool,
    #[pyo3(get, set)]
    pub routing_window_min_margin_cells: i32,
    #[pyo3(get, set)]
    pub routing_window_scale: f64,
    #[pyo3(get, set)]
    pub routing_window_max_expansions: u32,
    #[pyo3(get, set)]
    pub routing_window_fallback_full_grid: bool,
    #[pyo3(get, set)]
    pub routing_window_growth: f64,
    #[pyo3(get, set)]
    pub max_dense_obstacle_cells: usize,
}
#[pymethods]
impl PyAStarConfig {
    #[new]
    #[pyo3(signature=(max_iterations=100_000,bend_weight=1.0,target_tolerance_cells=0,require_target_angle=true,allowed_target_angles=None,use_routing_window=true,routing_window_min_margin_cells=12,routing_window_scale=0.35,routing_window_max_expansions=3,routing_window_fallback_full_grid=true,routing_window_growth=0.5,max_dense_obstacle_cells=10_000_000))]
    fn new(
        max_iterations: usize,
        bend_weight: f64,
        target_tolerance_cells: i32,
        require_target_angle: bool,
        allowed_target_angles: Option<Vec<u8>>,
        use_routing_window: bool,
        routing_window_min_margin_cells: i32,
        routing_window_scale: f64,
        routing_window_max_expansions: u32,
        routing_window_fallback_full_grid: bool,
        routing_window_growth: f64,
        max_dense_obstacle_cells: usize,
    ) -> Self {
        Self {
            max_iterations,
            bend_weight,
            target_tolerance_cells,
            require_target_angle,
            allowed_target_angles,
            use_routing_window,
            routing_window_min_margin_cells,
            routing_window_scale,
            routing_window_max_expansions,
            routing_window_fallback_full_grid,
            routing_window_growth,
            max_dense_obstacle_cells,
        }
    }
}

#[pyclass(name = "State")]
#[derive(Clone, Copy)]
pub struct PyState {
    #[pyo3(get, set)]
    pub x: i32,
    #[pyo3(get, set)]
    pub y: i32,
    #[pyo3(get, set)]
    pub angle: u8,
}
#[pymethods]
impl PyState {
    #[new]
    fn new(x: i32, y: i32, angle: u8) -> Self {
        Self {
            x,
            y,
            angle: angle % 8,
        }
    }
}

#[pyclass(name = "RouteResult")]
pub struct PyRouteResult {
    #[pyo3(get)]
    pub states: Vec<PyState>,
    #[pyo3(get)]
    pub primitive_ids: Vec<u16>,
    #[pyo3(get)]
    pub cells: Vec<(i32, i32)>,
    #[pyo3(get)]
    pub compressed_waypoints: Vec<(i32, i32)>,
    #[pyo3(get)]
    pub total_length_um: f64,
    #[pyo3(get)]
    pub total_cost: f64,
    #[pyo3(get)]
    pub requested_target: PyState,
    #[pyo3(get)]
    pub reached_target: PyState,
    #[pyo3(get)]
    pub segments: Vec<PyObject>,
    #[pyo3(get)]
    pub window_attempts: u32,
    #[pyo3(get)]
    pub used_full_grid_fallback: bool,
    #[pyo3(get)]
    pub expanded_states: usize,
    #[pyo3(get)]
    pub window_rejects: usize,
    #[pyo3(get)]
    pub footprint_rejects: usize,
    #[pyo3(get)]
    pub dense_grid_build_failures: usize,
    #[pyo3(get)]
    pub max_window_area_cells: i64,
}

#[pyclass(name = "PortAccess")]
#[derive(Clone)]
pub struct PyPortAccess {
    inner: PortAccess,
}

#[pymethods]
impl PyPortAccess {
    #[getter]
    fn port_name(&self) -> String {
        self.inner.port_name.clone()
    }

    #[getter]
    fn port_point_um(&self) -> (f64, f64) {
        self.inner.port_point_um
    }

    #[getter]
    fn anchor_cell(&self) -> (i32, i32) {
        self.inner.anchor_cell
    }

    #[getter]
    fn anchor_point_um(&self) -> (f64, f64) {
        self.inner.anchor_point_um
    }

    #[getter]
    fn entry_angle(&self) -> u8 {
        self.inner.entry_angle
    }
    #[getter]
    fn port_angle(&self) -> u8 {
        self.inner.port_angle
    }
    #[getter]
    fn anchor_angle(&self) -> u8 {
        self.inner.anchor_angle
    }

    #[getter]
    fn access_centerline_um(&self) -> Vec<(f64, f64)> {
        self.inner.access_centerline_um.clone()
    }
}

#[pyclass(name = "PyPhotonicRouter")]
pub struct PyPhotonicRouter {
    grid: PyGridSpec,
    primitive_cfg: PyPrimitiveLibraryConfig,
    astar_cfg: PyAStarConfig,
    obstacle_map: ObstacleMap,
    primitives: PrimitiveLibrary,
    static_cells: FxHashSet<CellKey>,
    port_open_cells: FxHashSet<CellKey>,
}

fn pack_cells(cells: &[(i32, i32)]) -> FxHashSet<CellKey> {
    cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect()
}

fn inflate_route_cells(
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

fn primitive_kind(p: &Primitive) -> String {
    let d = ((p.end_angle as i16 - p.start_angle as i16).rem_euclid(8)) as i16;
    if d == 0 {
        "straight".into()
    } else if d == 1 || d == 7 {
        "turn45".into()
    } else {
        "turn90".into()
    }
}

fn allowed_angles_to_mask(angles: Option<&Vec<u8>>) -> PyResult<Option<u8>> {
    let Some(angles) = angles else {
        return Ok(None);
    };
    if angles.is_empty() {
        return Err(PyValueError::new_err(
            "allowed_target_angles must not be empty when provided",
        ));
    }
    let mut mask = 0u8;
    for &angle in angles {
        if angle > 7 {
            return Err(PyValueError::new_err(
                "allowed_target_angles entries must be in [0, 7]",
            ));
        }
        mask |= 1u8 << angle;
    }
    Ok(Some(mask))
}

fn parse_meander_side(side: &str) -> PyResult<MeanderSide> {
    let normalized = side.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "left" => Ok(MeanderSide::Left),
        "right" => Ok(MeanderSide::Right),
        _ => Err(PyValueError::new_err(
            "side must be either 'left' or 'right'",
        )),
    }
}

fn parse_auto_meander_side_policy(side_policy: &str) -> PyResult<AutoMeanderSidePolicy> {
    let normalized = side_policy.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "left" => Ok(AutoMeanderSidePolicy::Left),
        "right" => Ok(AutoMeanderSidePolicy::Right),
        "both" => Ok(AutoMeanderSidePolicy::Both),
        _ => Err(PyValueError::new_err(
            "side_policy must be one of 'left', 'right', or 'both'",
        )),
    }
}

fn parse_meander_planning_mode(mode: &str) -> PyResult<MeanderPlanningMode> {
    match mode.trim().to_ascii_lowercase().as_str() {
        "fill_box_multi_bump" => Ok(MeanderPlanningMode::FillBoxMultiBump),
        "exact_extra_length" => Ok(MeanderPlanningMode::ExactExtraLength),
        _ => Err(PyValueError::new_err(
            "planning_mode must be 'fill_box_multi_bump' or 'exact_extra_length'",
        )),
    }
}

fn planning_mode_to_str(mode: MeanderPlanningMode) -> &'static str {
    match mode {
        MeanderPlanningMode::FillBoxMultiBump => "fill_box_multi_bump",
        MeanderPlanningMode::ExactExtraLength => "exact_extra_length",
    }
}

fn add_bend_radius_debug_metadata(
    dict: &Bound<'_, PyDict>,
    requested_min_bend_radius_um: Option<f64>,
    effective_bend_radius_um: f64,
    primitive_bend_radius_cells: i32,
    primitive_bend_radius_um: f64,
    planning_mode: MeanderPlanningMode,
    box_depth_um: Option<f64>,
) -> PyResult<()> {
    dict.set_item(
        "requested_min_bend_radius_um",
        requested_min_bend_radius_um,
    )?;
    dict.set_item("effective_bend_radius_um", effective_bend_radius_um)?;
    dict.set_item("primitive_bend_radius_cells", primitive_bend_radius_cells)?;
    dict.set_item("primitive_bend_radius_um", primitive_bend_radius_um)?;
    dict.set_item("planning_mode", planning_mode_to_str(planning_mode))?;
    if let Some(depth_um) = box_depth_um {
        dict.set_item("box_depth_um", depth_um)?;
    }
    dict.set_item(
        "radius_matches_primitive",
        (effective_bend_radius_um - primitive_bend_radius_um).abs() <= 1.0e-12,
    )?;
    Ok(())
}

#[pymethods]
impl PyPhotonicRouter {
    #[pyo3(signature=(min_bend_radius_um=None))]
    fn effective_bend_radius_um(&self, min_bend_radius_um: Option<f64>) -> PyResult<f64> {
        let grid_size_um = self.grid.grid_size_um;
        if !grid_size_um.is_finite() || grid_size_um <= 0.0 {
            return Err(PyValueError::new_err("grid_size_um must be finite and > 0"));
        }
        match min_bend_radius_um {
            None => {
                let cells = self.primitive_cfg.bend_radius_cells;
                actual_bend_radius_um_from_cells_rs(cells, grid_size_um).map_err(PyValueError::new_err)
            }
            Some(v) => {
                if !v.is_finite() || v <= 0.0 {
                    return Err(PyValueError::new_err(
                        "min_bend_radius_um must be finite and > 0 when provided",
                    ));
                }
                let cells =
                    bend_radius_cells_from_min_radius_rs(v, grid_size_um).map_err(PyValueError::new_err)?;
                actual_bend_radius_um_from_cells_rs(cells, grid_size_um).map_err(PyValueError::new_err)
            }
        }
    }

    #[pyo3(signature=(min_bend_radius_um=None))]
    fn describe_bend_radius(&self, py: Python<'_>, min_bend_radius_um: Option<f64>) -> PyResult<PyObject> {
        let primitive_bend_radius_cells = self.primitive_cfg.bend_radius_cells;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            primitive_bend_radius_cells,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let effective_bend_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let effective_bend_radius_cells = bend_radius_cells_from_min_radius_rs(
            effective_bend_radius_um,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let d = PyDict::new_bound(py);
        d.set_item("grid_size_um", self.grid.grid_size_um)?;
        d.set_item("primitive_bend_radius_cells", primitive_bend_radius_cells)?;
        d.set_item("primitive_bend_radius_um", primitive_bend_radius_um)?;
        d.set_item("requested_min_bend_radius_um", min_bend_radius_um)?;
        d.set_item("effective_bend_radius_um", effective_bend_radius_um)?;
        d.set_item("effective_bend_radius_cells", effective_bend_radius_cells)?;
        Ok(d.into())
    }

    #[new]
    fn new(
        grid_spec: PyGridSpec,
        primitive_config: PyPrimitiveLibraryConfig,
        astar_config: PyAStarConfig,
    ) -> Self {
        let primitives = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: primitive_config.grid_size_um,
            straight_short_cells: primitive_config.straight_short_cells,
            straight_long_cells: primitive_config.straight_long_cells,
            bend_radius_cells: primitive_config.bend_radius_cells,
            allow_45_degree_turns: primitive_config.allow_45_degree_turns,
        });
        Self {
            obstacle_map: ObstacleMap::new(grid_spec.width as i32, grid_spec.height as i32),
            grid: grid_spec,
            primitive_cfg: primitive_config,
            astar_cfg: astar_config,
            primitives,
            static_cells: FxHashSet::default(),
            port_open_cells: FxHashSet::default(),
        }
    }
    fn add_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        for (x, y) in &cells {
            self.static_cells.insert(pack_xy(*x, *y));
        }
        self.obstacle_map.add_static_cells(&cells);
    }
    fn clear_static_cells(&mut self) {
        self.obstacle_map = ObstacleMap::new(self.grid.width as i32, self.grid.height as i32);
        self.static_cells.clear();
    }
    fn set_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.clear_static_cells();
        self.add_static_cells(cells);
    }
    fn add_port_open_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.port_open_cells.extend(pack_cells(&cells));
    }
    fn clear_port_open_cells(&mut self) {
        self.port_open_cells.clear();
    }

    #[pyo3(signature=(source,target,opened_cells=None))]
    fn route_single_net(
        &self,
        py: Python<'_>,
        source: PyState,
        target: PyState,
        opened_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<Py<PyRouteResult>> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err(PyValueError::new_err("target_tolerance_cells must be >= 0"));
        }
        let opened_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            &self.port_open_cells
        };
        let allowed_target_angles_mask =
            allowed_angles_to_mask(self.astar_cfg.allowed_target_angles.as_ref())?;
        let cfg = AStarConfig {
            max_iterations: self.astar_cfg.max_iterations,
            bend_weight: self.astar_cfg.bend_weight * self.primitive_cfg.bend_weight,
            target_tolerance_cells: self.astar_cfg.target_tolerance_cells,
            require_target_angle: self.astar_cfg.require_target_angle,
            allowed_target_angles_mask,
            use_routing_window: self.astar_cfg.use_routing_window,
            routing_window_min_margin_cells: self.astar_cfg.routing_window_min_margin_cells,
            routing_window_scale: self.astar_cfg.routing_window_scale,
            routing_window_max_expansions: self.astar_cfg.routing_window_max_expansions,
            routing_window_fallback_full_grid: self.astar_cfg.routing_window_fallback_full_grid,
            routing_window_growth: self.astar_cfg.routing_window_growth,
            max_dense_states: AStarConfig::default().max_dense_states,
            max_dense_obstacle_cells: self.astar_cfg.max_dense_obstacle_cells,
        };
        let result = route_single_net_with_config(
            &self.obstacle_map,
            &self.primitives,
            State::new(source.x, source.y, source.angle),
            State::new(target.x, target.y, target.angle),
            Some(opened_ref),
            &cfg,
        )
        .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;
        Py::new(py, convert_result(py, &self.primitives, &result)?)
    }

    #[pyo3(signature=(net_id,source,target,block_radius_cells=0,opened_cells=None))]
    fn route_single_net_and_commit(
        &mut self,
        py: Python<'_>,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<Py<PyRouteResult>> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err(PyValueError::new_err("target_tolerance_cells must be >= 0"));
        }
        let opened_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            &self.port_open_cells
        };
        let allowed_target_angles_mask =
            allowed_angles_to_mask(self.astar_cfg.allowed_target_angles.as_ref())?;
        let cfg = AStarConfig {
            max_iterations: self.astar_cfg.max_iterations,
            bend_weight: self.astar_cfg.bend_weight * self.primitive_cfg.bend_weight,
            target_tolerance_cells: self.astar_cfg.target_tolerance_cells,
            require_target_angle: self.astar_cfg.require_target_angle,
            allowed_target_angles_mask,
            use_routing_window: self.astar_cfg.use_routing_window,
            routing_window_min_margin_cells: self.astar_cfg.routing_window_min_margin_cells,
            routing_window_scale: self.astar_cfg.routing_window_scale,
            routing_window_max_expansions: self.astar_cfg.routing_window_max_expansions,
            routing_window_fallback_full_grid: self.astar_cfg.routing_window_fallback_full_grid,
            routing_window_growth: self.astar_cfg.routing_window_growth,
            max_dense_states: AStarConfig::default().max_dense_states,
            max_dense_obstacle_cells: self.astar_cfg.max_dense_obstacle_cells,
        };
        let result = route_single_net_with_config(
            &self.obstacle_map,
            &self.primitives,
            State::new(source.x, source.y, source.angle),
            State::new(target.x, target.y, target.angle),
            Some(opened_ref),
            &cfg,
        )
        .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;

        let route_cells = inflate_route_cells(
            &result.cells,
            block_radius_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        if !self.obstacle_map.commit_route(net_id, &route_cells) {
            return Err(PyRuntimeError::new_err(
                "Failed to commit routed cells to obstacle map",
            ));
        }

        Py::new(py, convert_result(py, &self.primitives, &result)?)
    }

    fn export_debug_svg(&self, route: &PyRouteResult) -> String {
        let r = to_route_result(route);
        export_route_svg(&self.obstacle_map, &r)
    }
    #[pyo3(signature=(port_name,x_um,y_um,orientation=None,min_straight_um=0.0,max_anchor_search_cells=8,min_bend_radius_um=0.0))]
    fn build_port_access(
        &self,
        port_name: String,
        x_um: f64,
        y_um: f64,
        orientation: Option<f64>,
        min_straight_um: f64,
        max_anchor_search_cells: i32,
        min_bend_radius_um: f64,
    ) -> PyResult<PyPortAccess> {
        let grid = StaticGridSpec {
            width: self.grid.width as i32,
            height: self.grid.height as i32,
            grid_size_um: self.grid.grid_size_um,
            origin: (self.grid.origin_x_um, self.grid.origin_y_um),
            die_bbox: (0.0, 0.0, 0.0, 0.0),
        };
        let port = PortInput::new(port_name, x_um, y_um, orientation);
        let config = PortAccessConfig {
            min_straight_um,
            max_anchor_search_cells,
            min_bend_radius_um,
        };
        let access = build_port_access_rs(&port, &grid, &config)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
        Ok(PyPortAccess { inner: access })
    }

    #[pyo3(signature=(ports,min_straight_um=0.0,max_anchor_search_cells=8,min_bend_radius_um=0.0))]
    fn build_port_accesses(
        &self,
        ports: Vec<(String, f64, f64, Option<f64>)>,
        min_straight_um: f64,
        max_anchor_search_cells: i32,
        min_bend_radius_um: f64,
    ) -> PyResult<Vec<PyPortAccess>> {
        let grid = StaticGridSpec {
            width: self.grid.width as i32,
            height: self.grid.height as i32,
            grid_size_um: self.grid.grid_size_um,
            origin: (self.grid.origin_x_um, self.grid.origin_y_um),
            die_bbox: (0.0, 0.0, 0.0, 0.0),
        };
        let cfg = PortAccessConfig {
            min_straight_um,
            max_anchor_search_cells,
            min_bend_radius_um,
        };
        let port_inputs: Vec<PortInput> = ports
            .into_iter()
            .map(|(name, x, y, orientation)| PortInput::new(name, x, y, orientation))
            .collect();
        let accesses = build_port_accesses_rs(&port_inputs, &grid, &cfg)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
        Ok(accesses
            .into_iter()
            .map(|inner| PyPortAccess { inner })
            .collect())
    }

    #[pyo3(signature=(route,width_um,source_access=None,target_access=None))]
    fn realize_route_polygon_with_port_access(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        source_access: Option<PyPortAccess>,
        target_access: Option<PyPortAccess>,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        realize_route_polygon_with_port_access_rs(
            &r,
            &self.primitives,
            &grid,
            width_um,
            source_access.as_ref().map(|s| &s.inner),
            target_access.as_ref().map(|s| &s.inner),
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route,width_um))]
    fn realize_route_polygon(
        &self,
        route: &PyRouteResult,
        width_um: f64,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        realize_route_polygon_from_primitives_rs(&r, &self.primitives, &grid, width_um)
            .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route,width_um,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,side="left",available_box=None,planning_mode="exact_extra_length"))]
    fn realize_route_polygon_with_analytic_meander(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        requested_extra_length_um: f64,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        side: &str,
        available_box: Option<(f64, f64, f64, f64)>,
        planning_mode: &str,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let meander_side = parse_meander_side(side)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let (min_x_um, max_x_um, min_y_um, max_y_um) = available_box.ok_or_else(|| {
            PyValueError::new_err(
                "available_box must be provided as (min_x_um, max_x_um, min_y_um, max_y_um)",
            )
        })?;
        if min_x_um > max_x_um || min_y_um > max_y_um {
            return Err(PyValueError::new_err(
                "available_box is malformed: expected min_x<=max_x and min_y<=max_y",
            ));
        }
        let meander_box = MeanderBox {
            min_x_um,
            max_x_um,
            min_y_um,
            max_y_um,
        };

        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        realize_route_polygon_with_analytic_meander_rs(
            &r,
            &self.primitives,
            &grid,
            width_um,
            requested_extra_length_um,
            effective_radius_um,
            min_straight_um,
            max_bumps,
            meander_side,
            meander_box,
            mode,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route,width_um,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,side="left",available_box=None,clearance_radius_cells=0,opened_cells=None,planning_mode="exact_extra_length"))]
    fn realize_route_polygon_with_checked_analytic_meander_box(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        requested_extra_length_um: f64,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        side: &str,
        available_box: Option<(f64, f64, f64, f64)>,
        clearance_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        if clearance_radius_cells < 0 {
            return Err(PyValueError::new_err("clearance_radius_cells must be >= 0"));
        }
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let meander_side = parse_meander_side(side)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let (min_x_um, max_x_um, min_y_um, max_y_um) = available_box.ok_or_else(|| {
            PyValueError::new_err(
                "available_box must be provided as (min_x_um, max_x_um, min_y_um, max_y_um)",
            )
        })?;
        if min_x_um > max_x_um || min_y_um > max_y_um {
            return Err(PyValueError::new_err(
                "available_box is malformed: expected min_x<=max_x and min_y<=max_y",
            ));
        }
        let meander_box = MeanderBox {
            min_x_um,
            max_x_um,
            min_y_um,
            max_y_um,
        };

        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };
        realize_route_polygon_with_checked_analytic_meander_box_rs(
            &r,
            &self.primitives,
            &grid,
            width_um,
            requested_extra_length_um,
            effective_radius_um,
            min_straight_um,
            max_bumps,
            meander_side,
            meander_box,
            &self.obstacle_map,
            opened_ref,
            clearance_radius_cells,
            mode,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(available_box,clearance_radius_cells=0,opened_cells=None))]
    fn check_meander_box_free(
        &self,
        py: Python<'_>,
        available_box: (f64, f64, f64, f64),
        clearance_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<PyObject> {
        if clearance_radius_cells < 0 {
            return Err(PyValueError::new_err("clearance_radius_cells must be >= 0"));
        }
        let (min_x_um, max_x_um, min_y_um, max_y_um) = available_box;
        if min_x_um > max_x_um || min_y_um > max_y_um {
            return Err(PyValueError::new_err(
                "available_box is malformed: expected min_x<=max_x and min_y<=max_y",
            ));
        }
        let meander_box = MeanderBox {
            min_x_um,
            max_x_um,
            min_y_um,
            max_y_um,
        };
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };
        let prefix = DenseOccupancyPrefix::from_obstacle_map(&self.obstacle_map, opened_ref);
        let d = PyDict::new_bound(py);

        match meander_box_to_grid_rect_rs(meander_box, &grid, clearance_radius_cells) {
            Err(e) => {
                d.set_item("free", false)?;
                d.set_item("grid_rect", py.None())?;
                d.set_item("blocked_count", 0u32)?;
                d.set_item("reason", e.to_string())?;
                Ok(d.into())
            }
            Ok(rect) => {
                d.set_item(
                    "grid_rect",
                    (rect.min_x, rect.max_x, rect.min_y, rect.max_y),
                )?;
                match check_meander_box_free_with_prefix_rs(
                    meander_box,
                    &grid,
                    &prefix,
                    clearance_radius_cells,
                ) {
                    Ok(_) => {
                        d.set_item("free", true)?;
                        d.set_item("blocked_count", 0u32)?;
                        d.set_item("reason", "free")?;
                        Ok(d.into())
                    }
                    Err(GeometryError::MeanderBoxBlocked { blocked_count, .. }) => {
                        d.set_item("free", false)?;
                        d.set_item("blocked_count", blocked_count)?;
                        d.set_item("reason", "box_blocked")?;
                        Ok(d.into())
                    }
                    Err(e @ GeometryError::MeanderBoxOutOfBounds(_)) => {
                        d.set_item("free", false)?;
                        d.set_item("blocked_count", 0u32)?;
                        d.set_item("reason", e.to_string())?;
                        Ok(d.into())
                    }
                    Err(e) => Err(PyValueError::new_err(e.to_string())),
                }
            }
        }
    }

    #[pyo3(signature=(route,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,box_depth_um=20.0,min_segment_length_um=10.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump"))]
    fn plan_auto_analytic_meander_for_route(
        &self,
        py: Python<'_>,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        box_depth_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
    ) -> PyResult<PyObject> {
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if min_straight_um < 0.0 {
            return Err(PyValueError::new_err("min_straight_um must be >= 0"));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        if max_meander_height_um <= 0.0 {
            return Err(PyValueError::new_err("max_meander_height_um must be > 0"));
        }
        if box_depth_um <= 0.0 {
            return Err(PyValueError::new_err("box_depth_um must be > 0"));
        }
        if min_segment_length_um <= 0.0 {
            return Err(PyValueError::new_err("min_segment_length_um must be > 0"));
        }
        if clearance_radius_cells < 0 {
            return Err(PyValueError::new_err("clearance_radius_cells must be >= 0"));
        }
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            self.primitive_cfg.bend_radius_cells,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let cfg = AutoMeanderConfig {
            requested_extra_length_um,
            min_bend_radius_um: effective_radius_um,
            min_straight_um,
            max_bumps,
            max_meander_height_um,
            box_depth_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy: policy,
            mode,
        };
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };
        let plan = plan_auto_analytic_meander_for_route_rs(
            &r,
            &self.primitives,
            &grid,
            &self.obstacle_map,
            opened_ref,
            &cfg,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        let d = PyDict::new_bound(py);
        d.set_item("selected_segment_index", plan.selected_segment_index)?;
        d.set_item("selected_run_start_index", plan.selected_run_start_index)?;
        d.set_item("selected_run_end_index", plan.selected_run_end_index)?;
        d.set_item("selected_run_length_um", plan.selected_run_length_um)?;
        d.set_item("candidate_runs", plan.candidate_runs)?;
        d.set_item("rejected_box_blocked", plan.rejected_box_blocked)?;
        d.set_item("rejected_planning_failed", plan.rejected_planning_failed)?;
        d.set_item("rejected_too_short", plan.rejected_too_short)?;
        d.set_item(
            "selected_segment",
            (
                (
                    plan.selected_segment.start.x_um,
                    plan.selected_segment.start.y_um,
                ),
                (plan.selected_segment.end.x_um, plan.selected_segment.end.y_um),
            ),
        )?;
        d.set_item(
            "selected_box",
            (
                plan.selected_box.min_x_um,
                plan.selected_box.max_x_um,
                plan.selected_box.min_y_um,
                plan.selected_box.max_y_um,
            ),
        )?;
        d.set_item(
            "selected_grid_rect",
            (
                plan.selected_grid_rect.min_x,
                plan.selected_grid_rect.max_x,
                plan.selected_grid_rect.min_y,
                plan.selected_grid_rect.max_y,
            ),
        )?;
        let cl = PyList::empty_bound(py);
        for p in &plan.plan.centerline {
            cl.append((p.x_um, p.y_um))?;
        }
        d.set_item("centerline", cl)?;
        d.set_item("inserted_extra_length_um", plan.plan.inserted_extra_length_um)?;
        d.set_item("bumps", plan.plan.bumps)?;
        d.set_item(
            "side",
            if plan.plan.side == MeanderSide::Left {
                "left"
            } else {
                "right"
            },
        )?;
        add_bend_radius_debug_metadata(
            &d,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            Some(box_depth_um),
        )?;
        let mut max_possible_bumps = (box_depth_um / (2.0 * effective_radius_um)).floor() as i32;
        if max_possible_bumps % 2 != 0 {
            max_possible_bumps -= 1;
        }
        d.set_item("max_possible_bumps_from_box_depth", max_possible_bumps.max(0))?;
        Ok(d.into())
    }

    #[pyo3(signature=(route,width_um,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,box_depth_um=20.0,min_segment_length_um=10.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump"))]
    fn realize_route_polygon_with_auto_checked_analytic_meander(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        requested_extra_length_um: f64,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        box_depth_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let cfg = AutoMeanderConfig {
            requested_extra_length_um,
            min_bend_radius_um: effective_radius_um,
            min_straight_um,
            max_bumps,
            max_meander_height_um,
            box_depth_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy: policy,
            mode,
        };
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };
        realize_route_polygon_with_auto_checked_analytic_meander_rs(
            &r,
            &self.primitives,
            &grid,
            width_um,
            &self.obstacle_map,
            opened_ref,
            &cfg,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,box_depth_um=20.0,min_segment_length_um=10.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump"))]
    fn cells_for_auto_analytic_meander_box(
        &self,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        box_depth_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
    ) -> PyResult<Vec<(i32, i32)>> {
        if max_meander_height_um <= 0.0 {
            return Err(PyValueError::new_err("max_meander_height_um must be > 0"));
        }
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let cfg = AutoMeanderConfig {
            requested_extra_length_um,
            min_bend_radius_um: effective_radius_um,
            min_straight_um,
            max_bumps,
            max_meander_height_um,
            box_depth_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy: policy,
            mode,
        };
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };
        let plan = plan_auto_analytic_meander_for_route_rs(
            &r,
            &self.primitives,
            &grid,
            &self.obstacle_map,
            opened_ref,
            &cfg,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        Ok(cells_in_grid_rect_rs(plan.selected_grid_rect))
    }

    #[pyo3(signature=(net_id,source,target,width_um,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,box_depth_um=20.0,min_segment_length_um=10.0,route_block_radius_cells=0,meander_clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump"))]
    fn route_single_net_with_auto_meander_and_commit(
        &mut self,
        py: Python<'_>,
        net_id: u64,
        source: PyState,
        target: PyState,
        width_um: f64,
        requested_extra_length_um: f64,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        box_depth_um: f64,
        min_segment_length_um: f64,
        route_block_radius_cells: i32,
        meander_clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
    ) -> PyResult<PyObject> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err(PyValueError::new_err("target_tolerance_cells must be >= 0"));
        }
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if min_straight_um < 0.0 {
            return Err(PyValueError::new_err("min_straight_um must be >= 0"));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        if max_meander_height_um <= 0.0 {
            return Err(PyValueError::new_err("max_meander_height_um must be > 0"));
        }
        if box_depth_um <= 0.0 {
            return Err(PyValueError::new_err("box_depth_um must be > 0"));
        }
        if min_segment_length_um <= 0.0 {
            return Err(PyValueError::new_err("min_segment_length_um must be > 0"));
        }
        if meander_clearance_radius_cells < 0 {
            return Err(PyValueError::new_err(
                "meander_clearance_radius_cells must be >= 0",
            ));
        }
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            self.primitive_cfg.bend_radius_cells,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;

        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };

        let allowed_target_angles_mask =
            allowed_angles_to_mask(self.astar_cfg.allowed_target_angles.as_ref())?;
        let cfg = AStarConfig {
            max_iterations: self.astar_cfg.max_iterations,
            bend_weight: self.astar_cfg.bend_weight * self.primitive_cfg.bend_weight,
            target_tolerance_cells: self.astar_cfg.target_tolerance_cells,
            require_target_angle: self.astar_cfg.require_target_angle,
            allowed_target_angles_mask,
            use_routing_window: self.astar_cfg.use_routing_window,
            routing_window_min_margin_cells: self.astar_cfg.routing_window_min_margin_cells,
            routing_window_scale: self.astar_cfg.routing_window_scale,
            routing_window_max_expansions: self.astar_cfg.routing_window_max_expansions,
            routing_window_fallback_full_grid: self.astar_cfg.routing_window_fallback_full_grid,
            routing_window_growth: self.astar_cfg.routing_window_growth,
            max_dense_states: AStarConfig::default().max_dense_states,
            max_dense_obstacle_cells: self.astar_cfg.max_dense_obstacle_cells,
        };

        let result = route_single_net_with_config(
            &self.obstacle_map,
            &self.primitives,
            State::new(source.x, source.y, source.angle),
            State::new(target.x, target.y, target.angle),
            opened_ref,
            &cfg,
        )
        .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;

        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let auto_cfg = AutoMeanderConfig {
            requested_extra_length_um,
            min_bend_radius_um: effective_radius_um,
            min_straight_um,
            max_bumps,
            max_meander_height_um,
            box_depth_um,
            min_segment_length_um,
            clearance_radius_cells: meander_clearance_radius_cells,
            side_policy: policy,
            mode,
        };
        let auto_plan = plan_auto_analytic_meander_for_route_rs(
            &result,
            &self.primitives,
            &grid,
            &self.obstacle_map,
            opened_ref,
            &auto_cfg,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        let polygon = realize_route_polygon_from_auto_plan_rs(
            &result,
            &self.primitives,
            &grid,
            width_um,
            &auto_plan,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        let route_cells = inflate_route_cells(
            &result.cells,
            route_block_radius_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        let reserved_cells = cells_in_grid_rect_rs(auto_plan.selected_grid_rect);
        let mut merged = Vec::with_capacity(route_cells.len() + reserved_cells.len());
        let mut seen = FxHashSet::default();
        for (x, y) in route_cells.into_iter().chain(reserved_cells.iter().copied()) {
            let key = pack_xy(x, y);
            if seen.insert(key) {
                merged.push((x, y));
            }
        }
        if !self.obstacle_map.commit_route(net_id, &merged) {
            return Err(PyRuntimeError::new_err(
                "Failed to commit merged route and meander reservation cells",
            ));
        }

        let py_route = Py::new(py, convert_result(py, &self.primitives, &result)?)?;
        let d = PyDict::new_bound(py);
        d.set_item("route", py_route)?;
        d.set_item("polygon", polygon)?;
        d.set_item(
            "selected_box",
            (
                auto_plan.selected_box.min_x_um,
                auto_plan.selected_box.max_x_um,
                auto_plan.selected_box.min_y_um,
                auto_plan.selected_box.max_y_um,
            ),
        )?;
        d.set_item(
            "selected_grid_rect",
            (
                auto_plan.selected_grid_rect.min_x,
                auto_plan.selected_grid_rect.max_x,
                auto_plan.selected_grid_rect.min_y,
                auto_plan.selected_grid_rect.max_y,
            ),
        )?;
        d.set_item("reserved_cells", reserved_cells)?;
        d.set_item("inserted_extra_length_um", auto_plan.plan.inserted_extra_length_um)?;
        d.set_item("bumps", auto_plan.plan.bumps)?;
        d.set_item(
            "side",
            if auto_plan.plan.side == MeanderSide::Left {
                "left"
            } else {
                "right"
            },
        )?;
        add_bend_radius_debug_metadata(
            &d,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            Some(box_depth_um),
        )?;
        Ok(d.into())
    }
    fn describe_primitives(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        describe_primitives(py, &self.primitives)
    }

    // Legacy prototype API: primitive-based meander insertion on discrete route objects.
    // Kept for compatibility only; use realize_route_polygon_with_analytic_meander instead.
    #[pyo3(signature=(route,requested_extra_length_um,insert_after_state_index=0,insert_end_state_index=None))]
    fn insert_simple_meander_loop(
        &self,
        py: Python<'_>,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        insert_after_state_index: usize,
        insert_end_state_index: Option<usize>,
    ) -> PyResult<PyObject> {
        let base = to_route_result(route);
        #[allow(deprecated)]
        let report = insert_simple_meander_loop_rs(
            &self.primitives,
            &base,
            requested_extra_length_um,
            insert_after_state_index,
            insert_end_state_index,
        )
        .map_err(PyRuntimeError::new_err)?;
        let py_route = Py::new(py, convert_result(py, &self.primitives, &report.route)?)?;
        let d = PyDict::new_bound(py);
        d.set_item("applied", report.applied)?;
        d.set_item("legacy", true)?;
        d.set_item("reason", report.reason)?;
        d.set_item("inserted_extra_length_um", report.inserted_extra_length_um)?;
        d.set_item(
            "warning",
            "legacy single-bump meander analysis; not used by main auto-meander flow",
        )?;
        d.set_item("route", py_route)?;
        Ok(d.into())
    }

    // Legacy prototype API: candidate analysis for primitive-based insertion.
    // Kept for compatibility only; use plan_analytic_meander_for_route instead.
    #[pyo3(signature=(route,requested_extra_length_um,min_endpoint_margin_cells=4,min_candidate_straight_length_um=10.0,max_extra_length_per_region_um=200.0,conservative_legal_check=true))]
    fn analyze_meander_insertion_candidate(
        &self,
        py: Python<'_>,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        min_endpoint_margin_cells: i32,
        min_candidate_straight_length_um: f64,
        max_extra_length_per_region_um: f64,
        conservative_legal_check: bool,
    ) -> PyResult<PyObject> {
        let route_rs = to_route_result(route);
        #[allow(deprecated)]
        let report_rs = analyze_meander_insertion_candidate_rs(
            &route_rs,
            &self.primitives,
            self.grid.grid_size_um,
            requested_extra_length_um,
            min_endpoint_margin_cells,
            min_candidate_straight_length_um,
            max_extra_length_per_region_um,
            conservative_legal_check,
        );

        let report = PyDict::new_bound(py);
        report.set_item(
            "requested_extra_length_um",
            report_rs.requested_extra_length_um,
        )?;
        report.set_item("inserted_extra_length_um", report_rs.inserted_extra_length_um)?;
        report.set_item(
            "conservative_legal_check",
            report_rs.conservative_legal_check,
        )?;

        let candidate_dicts = PyList::empty_bound(py);
        for c in report_rs.candidates.iter() {
            let d = PyDict::new_bound(py);
            d.set_item("start_index", c.start_index)?;
            d.set_item("end_index", c.end_index)?;
            d.set_item("length_um", c.length_um)?;
            d.set_item("heading_dx", c.heading_dx)?;
            d.set_item("heading_dy", c.heading_dy)?;
            candidate_dicts.append(d)?;
        }
        report.set_item("candidates", candidate_dicts)?;
        report.set_item("legacy", true)?;
        report.set_item("status", report_rs.status)?;
        report.set_item("reason", report_rs.reason)?;
        report.set_item(
            "warning",
            "legacy single-bump meander analysis; not used by main auto-meander flow",
        )?;
        Ok(report.into())
    }

    #[pyo3(signature=(route,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,side="left",available_box=None,planning_mode="exact_extra_length"))]
    fn plan_analytic_meander_for_route(
        &self,
        py: Python<'_>,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        side: &str,
        available_box: Option<(f64, f64, f64, f64)>,
        planning_mode: &str,
    ) -> PyResult<PyObject> {
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        let meander_side = parse_meander_side(side)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            self.primitive_cfg.bend_radius_cells,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let (min_x_um, max_x_um, min_y_um, max_y_um) = available_box.ok_or_else(|| {
            PyValueError::new_err(
                "available_box must be provided as (min_x_um, max_x_um, min_y_um, max_y_um)",
            )
        })?;
        if min_x_um > max_x_um || min_y_um > max_y_um {
            return Err(PyValueError::new_err(
                "available_box is malformed: expected min_x<=max_x and min_y<=max_y",
            ));
        }
        let meander_box = MeanderBox {
            min_x_um,
            max_x_um,
            min_y_um,
            max_y_um,
        };

        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let plan = plan_analytic_meander_for_route_rs(
            &r,
            &self.primitives,
            &grid,
            requested_extra_length_um,
            effective_radius_um,
            min_straight_um,
            max_bumps,
            meander_side,
            meander_box,
            mode,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        let d = PyDict::new_bound(py);
        d.set_item("selected_segment_index", plan.selected_segment_index)?;
        d.set_item(
            "selected_segment",
            (
                (
                    plan.selected_segment.start.x_um,
                    plan.selected_segment.start.y_um,
                ),
                (plan.selected_segment.end.x_um, plan.selected_segment.end.y_um),
            ),
        )?;
        let py_side = match plan.plan.side {
            MeanderSide::Left => "left",
            MeanderSide::Right => "right",
        };
        let cl = PyList::empty_bound(py);
        for p in &plan.plan.centerline {
            cl.append((p.x_um, p.y_um))?;
        }
        d.set_item("centerline", cl)?;
        d.set_item("inserted_extra_length_um", plan.plan.inserted_extra_length_um)?;
        d.set_item("bumps", plan.plan.bumps)?;
        d.set_item("side", py_side)?;
        add_bend_radius_debug_metadata(
            &d,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            None,
        )?;
        Ok(d.into())
    }
}

pub fn register_py_router(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyGridSpec>()?;
    m.add_class::<PyPrimitiveLibraryConfig>()?;
    m.add_class::<PyAStarConfig>()?;
    m.add_class::<PyState>()?;
    m.add_class::<PyRouteResult>()?;
    m.add_class::<PyPortAccess>()?;
    m.add_class::<PyPhotonicRouter>()?;
    Ok(())
}

fn describe_primitives(py: Python<'_>, lib: &PrimitiveLibrary) -> PyResult<Vec<PyObject>> {
    let mut out = Vec::new();
    for a in 0..8u8 {
        for p in lib.get_primitives_for_angle(a) {
            let d = pyo3::types::PyDict::new_bound(py);
            d.set_item("id", p.id)?;
            d.set_item("name", primitive_kind(p))?;
            d.set_item("start_angle", p.start_angle)?;
            d.set_item("end_angle", p.end_angle)?;
            d.set_item("dx", p.dx)?;
            d.set_item("dy", p.dy)?;
            d.set_item("length_um", p.length_um)?;
            d.set_item("bend_cost", p.bend_cost)?;
            d.set_item("footprint_cells", p.footprint.clone())?;
            out.push(d.into());
        }
    }
    Ok(out)
}

fn convert_result(
    py: Python<'_>,
    lib: &PrimitiveLibrary,
    r: &RouteResult,
) -> PyResult<PyRouteResult> {
    let mut segments = Vec::new();
    for (i, pid) in r.primitives.iter().enumerate() {
        let s0 = r.states[i];
        let s1 = r.states[i + 1];
        let p = lib
            .get_primitives_for_angle(s0.angle)
            .iter()
            .find(|p| p.id == *pid)
            .unwrap();
        let d = pyo3::types::PyDict::new_bound(py);
        d.set_item("primitive_id", pid)?;
        d.set_item("kind", primitive_kind(p))?;
        d.set_item("start", (s0.x, s0.y))?;
        d.set_item("end", (s1.x, s1.y))?;
        d.set_item("start_angle", s0.angle)?;
        d.set_item("end_angle", s1.angle)?;
        d.set_item("length_um", p.length_um)?;
        segments.push(d.into());
    }
    Ok(PyRouteResult {
        states: r
            .states
            .iter()
            .map(|s| PyState {
                x: s.x,
                y: s.y,
                angle: s.angle,
            })
            .collect(),
        primitive_ids: r.primitives.clone(),
        cells: r.cells.clone(),
        compressed_waypoints: r.compressed_waypoints.clone(),
        total_length_um: r.total_length_um,
        total_cost: r.total_cost,
        requested_target: PyState {
            x: r.requested_target.x,
            y: r.requested_target.y,
            angle: r.requested_target.angle,
        },
        reached_target: PyState {
            x: r.reached_target.x,
            y: r.reached_target.y,
            angle: r.reached_target.angle,
        },
        segments,
        window_attempts: r.stats.window_attempts,
        used_full_grid_fallback: r.stats.used_full_grid_fallback,
        expanded_states: r.stats.expanded_states,
        window_rejects: r.stats.window_rejects,
        footprint_rejects: r.stats.footprint_rejects,
        dense_grid_build_failures: r.stats.dense_grid_build_failures,
        max_window_area_cells: r.stats.max_window_area_cells,
    })
}

fn to_route_result(route: &PyRouteResult) -> RouteResult {
    RouteResult {
        states: route
            .states
            .iter()
            .map(|s| State::new(s.x, s.y, s.angle))
            .collect(),
        primitives: route.primitive_ids.clone(),
        cells: route.cells.clone(),
        compressed_waypoints: route.compressed_waypoints.clone(),
        total_length_um: route.total_length_um,
        total_cost: route.total_cost,
        requested_target: State::new(
            route.requested_target.x,
            route.requested_target.y,
            route.requested_target.angle,
        ),
        reached_target: State::new(
            route.reached_target.x,
            route.reached_target.y,
            route.reached_target.angle,
        ),
        stats: RouteSearchStats {
            window_attempts: route.window_attempts,
            used_full_grid_fallback: route.used_full_grid_fallback,
            expanded_states: route.expanded_states,
            window_rejects: route.window_rejects,
            footprint_rejects: route.footprint_rejects,
            dense_grid_build_failures: route.dense_grid_build_failures,
            max_window_area_cells: route.max_window_area_cells,
        },
    }
}

#[pymodule]
pub fn photonic_router_rust(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    register_py_router(m)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_route_and_describe() {
        let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        assert!(!router.primitives.get_primitives_for_angle(0).is_empty());
    }

    #[test]
    fn parse_meander_side_variants() {
        assert_eq!(parse_meander_side("left").unwrap(), MeanderSide::Left);
        assert_eq!(parse_meander_side("right").unwrap(), MeanderSide::Right);
        assert!(parse_meander_side("up").is_err());
    }

    #[test]
    fn analytic_meander_method_requires_available_box() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![
                PyState::new(1, 2, 0),
                PyState::new(13, 2, 0),
            ],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: PyState::new(13, 2, 0),
            reached_target: PyState::new(13, 2, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        let err = router
            .realize_route_polygon_with_analytic_meander(
                &route,
                1.0,
                3.0,
                Some(0.2),
                0.1,
                2,
                "left",
                None,
                "exact_extra_length",
            )
            .unwrap_err();
        assert!(err.to_string().contains("available_box must be provided"));
    }

    #[test]
    fn analytic_meander_method_success_and_too_small_box_error() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![
                PyState::new(1, 2, 0),
                PyState::new(13, 2, 0),
            ],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: PyState::new(13, 2, 0),
            reached_target: PyState::new(13, 2, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };

        let ok_poly = router
            .realize_route_polygon_with_analytic_meander(
                &route,
                1.0,
                3.0,
                Some(0.2),
                0.0,
                2,
                "left",
                Some((1.4, 13.6, 2.4, 8.0)),
                "exact_extra_length",
            )
            .unwrap();
        assert!(ok_poly.len() >= 4);
        assert!(ok_poly.iter().all(|(x, y)| x.is_finite() && y.is_finite()));

        let err = router
            .realize_route_polygon_with_analytic_meander(
                &route,
                1.0,
                50.0,
                Some(0.5),
                0.5,
                1,
                "left",
                Some((1.4, 13.6, 2.4, 2.9)),
                "exact_extra_length",
            )
            .unwrap_err();
        assert!(!err.to_string().is_empty());
    }

    #[test]
    fn analytic_meander_debug_method_returns_expected_fields() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 12, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![PyState::new(1, 2, 0), PyState::new(13, 2, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: PyState::new(13, 2, 0),
            reached_target: PyState::new(13, 2, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        Python::with_gil(|py| {
            let obj = router
                .plan_analytic_meander_for_route(
                    py,
                    &route,
                    3.0,
                    Some(0.2),
                    0.0,
                    4,
                    "left",
                    Some((1.4, 13.6, 2.4, 8.0)),
                    "exact_extra_length",
                )
                .unwrap();
            let d = obj.bind(py).downcast::<PyDict>().unwrap();
            assert!(d.contains("selected_segment_index").unwrap());
            assert!(d.contains("selected_segment").unwrap());
            assert!(d.contains("centerline").unwrap());
            assert!(d.contains("inserted_extra_length_um").unwrap());
            assert!(d.contains("bumps").unwrap());
            assert!(d.contains("side").unwrap());
        });
    }

    #[test]
    fn legacy_insert_simple_meander_loop_contains_warning_field() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![PyState::new(1, 2, 0), PyState::new(5, 2, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: PyState::new(5, 2, 0),
            reached_target: PyState::new(5, 2, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        Python::with_gil(|py| {
            let obj = router
                .insert_simple_meander_loop(py, &route, 1.0, 0, None)
                .unwrap();
            let d = obj.bind(py).downcast::<PyDict>().unwrap();
            assert!(d.contains("warning").unwrap());
        });
    }

    #[test]
    fn checked_analytic_meander_box_method_fails_on_blocked_box() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        router.add_static_cells(vec![(3, 3)]);
        let route = PyRouteResult {
            states: vec![PyState::new(1, 2, 0), PyState::new(5, 2, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: PyState::new(5, 2, 0),
            reached_target: PyState::new(5, 2, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        let err = router
            .realize_route_polygon_with_checked_analytic_meander_box(
                &route,
                1.0,
                3.0,
                Some(0.2),
                0.1,
                2,
                "left",
                Some((1.4, 5.6, 2.4, 4.0)),
                0,
                None,
                "exact_extra_length",
            )
            .unwrap_err();
        assert!(!err.to_string().is_empty());
    }

    #[test]
    fn check_meander_box_free_debug_reports_status() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        router.add_static_cells(vec![(3, 3)]);
        Python::with_gil(|py| {
            let obj = router
                .check_meander_box_free(py, (1.4, 5.6, 2.4, 4.0), 0, None)
                .unwrap();
            let d = obj.bind(py).downcast::<PyDict>().unwrap();
            assert!(d.contains("free").unwrap());
            assert!(d.contains("grid_rect").unwrap());
            assert!(d.contains("blocked_count").unwrap());
            assert!(d.contains("reason").unwrap());
        });
    }

    #[test]
    fn auto_plan_debug_returns_selected_box_and_grid_rect() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 12, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![PyState::new(1, 2, 0), PyState::new(13, 2, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: PyState::new(13, 2, 0),
            reached_target: PyState::new(13, 2, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        Python::with_gil(|py| {
            let obj = router
                .plan_auto_analytic_meander_for_route(
                    py,
                    &route,
                    3.0,
                    Some(0.2),
                    0.0,
                    2,
                    20.0,
                    8.0,
                    1.0,
                    0,
                    "both",
                    None,
                    "fill_box_multi_bump",
                )
                .unwrap();
            let d = obj.bind(py).downcast::<PyDict>().unwrap();
            assert!(d.contains("selected_box").unwrap());
            assert!(d.contains("selected_grid_rect").unwrap());
            assert!(d.contains("selected_run_start_index").unwrap());
            assert!(d.contains("selected_run_end_index").unwrap());
            assert!(d.contains("selected_run_length_um").unwrap());
            assert!(d.contains("candidate_runs").unwrap());
            assert!(d.contains("effective_bend_radius_um").unwrap());
            assert!(d.contains("primitive_bend_radius_um").unwrap());
            assert!(d.contains("planning_mode").unwrap());
            assert!(d.contains("box_depth_um").unwrap());
            assert!(d.contains("max_possible_bumps_from_box_depth").unwrap());
        });
    }

    #[test]
    fn auto_plan_debug_large_radius_reports_low_max_possible_bumps() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![PyState::new(1, 2, 0), PyState::new(5, 2, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: PyState::new(5, 2, 0),
            reached_target: PyState::new(5, 2, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        Python::with_gil(|py| {
            let obj = router
                .plan_auto_analytic_meander_for_route(
                    py,
                    &route,
                    1.0,
                    Some(10.0),
                    0.1,
                    20,
                    20.0,
                    8.0,
                    1.0,
                    0,
                    "both",
                    None,
                    "fill_box_multi_bump",
                )
                .unwrap_err();
            assert!(!obj.to_string().is_empty());
        });
    }

    #[test]
    fn auto_plan_default_radius_can_produce_many_bumps() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(80, 80, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 60, 60, 2, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![PyState::new(2, 20, 0), PyState::new(62, 20, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 30.0,
            total_cost: 30.0,
            requested_target: PyState::new(62, 20, 0),
            reached_target: PyState::new(62, 20, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        Python::with_gil(|py| {
            let err = router
                .plan_auto_analytic_meander_for_route(
                    py,
                    &route,
                    1.0,
                    None,
                    0.0,
                    20,
                    20.0,
                    8.0,
                    1.0,
                    0,
                    "both",
                    None,
                    "fill_box_multi_bump",
                )
                .unwrap_err();
            assert!(!err.to_string().is_empty());
        });
    }

    #[test]
    fn effective_bend_radius_defaults_to_primitive_radius() {
        let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let eff = router.effective_bend_radius_um(None).unwrap();
        assert!((eff - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn effective_bend_radius_rounds_up_explicit_request() {
        let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let eff = router.effective_bend_radius_um(Some(1.1)).unwrap();
        assert!((eff - 1.5).abs() < 1.0e-9);
    }

    #[test]
    fn describe_bend_radius_reports_effective_values() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        Python::with_gil(|py| {
            let obj = router.describe_bend_radius(py, Some(1.1)).unwrap();
            let d = obj.bind(py).downcast::<PyDict>().unwrap();
            let eff: f64 = d
                .get_item("effective_bend_radius_um")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            let eff_cells: i32 = d
                .get_item("effective_bend_radius_cells")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            assert!((eff - 1.5).abs() < 1.0e-9);
            assert_eq!(eff_cells, 3);
        });
    }

    #[test]
    fn explicit_plan_none_radius_matches_primitive_and_explicit_request_rounds_up() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(80, 80, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 60, 60, 2, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![PyState::new(2, 20, 0), PyState::new(62, 20, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 30.0,
            total_cost: 30.0,
            requested_target: PyState::new(62, 20, 0),
            reached_target: PyState::new(62, 20, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        Python::with_gil(|py| {
            let none_obj = router
                .plan_analytic_meander_for_route(
                    py,
                    &route,
                    5.0,
                    None,
                    0.0,
                    4,
                    "left",
                    Some((1.0, 32.0, 0.0, 30.0)),
                    "fill_box_multi_bump",
                )
                .unwrap();
            let none_d = none_obj.bind(py).downcast::<PyDict>().unwrap();
            let primitive_radius: f64 = none_d
                .get_item("primitive_bend_radius_um")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            let effective_radius_none: f64 = none_d
                .get_item("effective_bend_radius_um")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            let matches_primitive: bool = none_d
                .get_item("radius_matches_primitive")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            let none_bumps: usize = none_d.get_item("bumps").unwrap().unwrap().extract().unwrap();
            assert!((primitive_radius - 1.0).abs() < 1.0e-9);
            assert!((effective_radius_none - 1.0).abs() < 1.0e-9);
            assert!(matches_primitive);
            assert!(none_bumps >= 1);

            let req_obj = router
                .plan_analytic_meander_for_route(
                    py,
                    &route,
                    4.0,
                    Some(1.1),
                    0.0,
                    2,
                    "left",
                    Some((1.0, 32.0, 0.0, 30.0)),
                    "fill_box_multi_bump",
                )
                .unwrap();
            let req_d = req_obj.bind(py).downcast::<PyDict>().unwrap();
            let effective_radius_req: f64 = req_d
                .get_item("effective_bend_radius_um")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            assert!((effective_radius_req - 1.5).abs() < 1.0e-9);
        });
    }

    #[test]
    fn auto_and_explicit_none_radius_match() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(80, 80, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 60, 60, 2, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let route = PyRouteResult {
            states: vec![PyState::new(2, 20, 0), PyState::new(62, 20, 0)],
            primitive_ids: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 30.0,
            total_cost: 30.0,
            requested_target: PyState::new(62, 20, 0),
            reached_target: PyState::new(62, 20, 0),
            segments: vec![],
            window_attempts: 0,
            used_full_grid_fallback: false,
            expanded_states: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
        };
        Python::with_gil(|py| {
            let auto_obj = router
                .plan_auto_analytic_meander_for_route(
                    py,
                    &route,
                    5.0,
                    None,
                    0.0,
                    4,
                    20.0,
                    8.0,
                    1.0,
                    0,
                    "both",
                    None,
                    "fill_box_multi_bump",
                )
                .unwrap();
            let auto_d = auto_obj.bind(py).downcast::<PyDict>().unwrap();
            let auto_effective: f64 = auto_d
                .get_item("effective_bend_radius_um")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            let explicit_obj = router
                .plan_analytic_meander_for_route(
                    py,
                    &route,
                    5.0,
                    None,
                    0.0,
                    4,
                    "left",
                    Some((1.0, 32.0, 0.0, 30.0)),
                    "fill_box_multi_bump",
                )
                .unwrap();
            let explicit_d = explicit_obj.bind(py).downcast::<PyDict>().unwrap();
            let explicit_effective: f64 = explicit_d
                .get_item("effective_bend_radius_um")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
            assert!((auto_effective - explicit_effective).abs() < 1.0e-9);
        });
    }

    #[test]
    fn auto_meander_and_commit_reserves_selected_box_cells() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 26, 26, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        Python::with_gil(|py| {
            let obj = router
                .route_single_net_with_auto_meander_and_commit(
                    py,
                    123,
                    PyState::new(2, 10, 0),
                    PyState::new(28, 10, 0),
                    1.0,
                    4.0,
                    Some(0.2),
                    0.0,
                    2,
                    20.0,
                    8.0,
                    1.0,
                    0,
                    0,
                    "both",
                    None,
                    "fill_box_multi_bump",
                )
                .unwrap();
            let d = obj.bind(py).downcast::<PyDict>().unwrap();
            assert!(d.contains("route").unwrap());
            assert!(d.contains("polygon").unwrap());
            assert!(d.contains("selected_box").unwrap());
            assert!(d.contains("selected_grid_rect").unwrap());
            assert!(d.contains("reserved_cells").unwrap());
            assert!(d.contains("inserted_extra_length_um").unwrap());
            assert!(d.contains("bumps").unwrap());
            assert!(d.contains("side").unwrap());
        });

        let cells = router
            .obstacle_map
            .get_net_cells(123)
            .expect("net cells should exist");
        assert!(!cells.is_empty());
        for key in cells {
            let (x, y) = crate::obstacle_map::unpack_xy(*key);
            assert!(router.obstacle_map.is_blocked(x, y));
        }
    }

    #[test]
    fn auto_meander_commit_error_does_not_commit_route() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 26, 26, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let err = Python::with_gil(|py| {
            router
                .route_single_net_with_auto_meander_and_commit(
                    py,
                    124,
                    PyState::new(2, 10, 0),
                    PyState::new(10, 10, 0),
                    1.0,
                    4.0,
                    Some(0.2),
                    0.1,
                    2,
                    20.0,
                    1.6,
                    1000.0,
                    0,
                    0,
                    "both",
                    None,
                    "fill_box_multi_bump",
                )
                .unwrap_err()
        });
        assert!(!err.to_string().is_empty());
        assert!(router.obstacle_map.get_net_cells(124).is_none());
    }

    #[test]
    fn auto_meander_commit_blocks_cells_for_followup_checks() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 26, 26, 1, 1.0, true),
            PyAStarConfig::new(
                10000, 1.0, 0, true, None, true, 12, 0.35, 3, true, 0.5, 10_000_000,
            ),
        );
        let reserved_cells: Vec<(i32, i32)> = Python::with_gil(|py| {
            let obj = router
                .route_single_net_with_auto_meander_and_commit(
                    py,
                    125,
                    PyState::new(2, 10, 0),
                    PyState::new(28, 10, 0),
                    1.0,
                    4.0,
                    Some(0.2),
                    0.0,
                    2,
                    20.0,
                    8.0,
                    1.0,
                    0,
                    0,
                    "both",
                    None,
                    "fill_box_multi_bump",
                )
                .unwrap();
            let d = obj.bind(py).downcast::<PyDict>().unwrap();
            d.get_item("reserved_cells")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap()
        });
        assert!(!reserved_cells.is_empty());
        for (x, y) in reserved_cells {
            assert!(router.obstacle_map.is_blocked(x, y));
        }
    }
}
