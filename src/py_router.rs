use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use rustc_hash::FxHashSet;

use crate::astar::{
    export_route_svg, route_single_net_with_config, AStarConfig, RouteResult, RouteSearchStats,
    State,
};
use crate::geometry_realization::{
    build_port_access as build_port_access_rs, build_port_accesses as build_port_accesses_rs,
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
    insert_simple_meander_loop as insert_simple_meander_loop_rs,
    MeanderBox, MeanderSide,
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

#[pymethods]
impl PyPhotonicRouter {
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

    #[pyo3(signature=(route,width_um,requested_extra_length_um,min_bend_radius_um,min_straight_um=0.0,max_bumps=8,side="left",available_box=None))]
    fn realize_route_polygon_with_analytic_meander(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        requested_extra_length_um: f64,
        min_bend_radius_um: f64,
        min_straight_um: f64,
        max_bumps: usize,
        side: &str,
        available_box: Option<(f64, f64, f64, f64)>,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if min_bend_radius_um <= 0.0 {
            return Err(PyValueError::new_err("min_bend_radius_um must be > 0"));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        let meander_side = parse_meander_side(side)?;
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
            min_bend_radius_um,
            min_straight_um,
            max_bumps,
            meander_side,
            meander_box,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route,width_um,requested_extra_length_um,min_bend_radius_um,min_straight_um=0.0,max_bumps=8,side="left",available_box=None,clearance_radius_cells=0,opened_cells=None))]
    fn realize_route_polygon_with_checked_analytic_meander_box(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        requested_extra_length_um: f64,
        min_bend_radius_um: f64,
        min_straight_um: f64,
        max_bumps: usize,
        side: &str,
        available_box: Option<(f64, f64, f64, f64)>,
        clearance_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if min_bend_radius_um <= 0.0 {
            return Err(PyValueError::new_err("min_bend_radius_um must be > 0"));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        if clearance_radius_cells < 0 {
            return Err(PyValueError::new_err("clearance_radius_cells must be >= 0"));
        }
        let meander_side = parse_meander_side(side)?;
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
            min_bend_radius_um,
            min_straight_um,
            max_bumps,
            meander_side,
            meander_box,
            &self.obstacle_map,
            opened_ref,
            clearance_radius_cells,
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
        d.set_item("reason", report.reason)?;
        d.set_item("inserted_extra_length_um", report.inserted_extra_length_um)?;
        d.set_item(
            "warning",
            "legacy primitive-based meander prototype; use realize_route_polygon_with_analytic_meander instead",
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
        report.set_item("status", report_rs.status)?;
        report.set_item("reason", report_rs.reason)?;
        report.set_item(
            "warning",
            "legacy primitive-based meander prototype; use realize_route_polygon_with_analytic_meander instead",
        )?;
        Ok(report.into())
    }

    #[pyo3(signature=(route,requested_extra_length_um,min_bend_radius_um,min_straight_um=0.0,max_bumps=8,side="left",available_box=None))]
    fn plan_analytic_meander_for_route(
        &self,
        py: Python<'_>,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        min_bend_radius_um: f64,
        min_straight_um: f64,
        max_bumps: usize,
        side: &str,
        available_box: Option<(f64, f64, f64, f64)>,
    ) -> PyResult<PyObject> {
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if min_bend_radius_um <= 0.0 {
            return Err(PyValueError::new_err("min_bend_radius_um must be > 0"));
        }
        if max_bumps == 0 {
            return Err(PyValueError::new_err("max_bumps must be > 0"));
        }
        let meander_side = parse_meander_side(side)?;
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
            min_bend_radius_um,
            min_straight_um,
            max_bumps,
            meander_side,
            meander_box,
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
                PyState::new(5, 2, 0),
            ],
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
            .realize_route_polygon_with_analytic_meander(
                &route,
                1.0,
                3.0,
                0.2,
                0.1,
                2,
                "left",
                None,
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
                PyState::new(5, 2, 0),
            ],
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

        let ok_poly = router
            .realize_route_polygon_with_analytic_meander(
                &route,
                1.0,
                3.0,
                0.2,
                0.1,
                2,
                "left",
                Some((1.4, 5.6, 2.4, 4.0)),
            )
            .unwrap();
        assert!(ok_poly.len() >= 4);
        assert!(ok_poly.iter().all(|(x, y)| x.is_finite() && y.is_finite()));

        let err = router
            .realize_route_polygon_with_analytic_meander(
                &route,
                1.0,
                50.0,
                0.5,
                0.5,
                1,
                "left",
                Some((1.4, 5.6, 2.4, 2.9)),
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
                .plan_analytic_meander_for_route(
                    py,
                    &route,
                    3.0,
                    0.2,
                    0.1,
                    2,
                    "left",
                    Some((1.4, 5.6, 2.4, 4.0)),
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
                0.2,
                0.1,
                2,
                "left",
                Some((1.4, 5.6, 2.4, 4.0)),
                0,
                None,
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
}
