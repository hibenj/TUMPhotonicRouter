use std::cell::RefCell;
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use rustc_hash::{FxHashMap, FxHashSet};

use crate::astar::{
    export_route_svg, route_single_net_with_config, route_single_net_with_dynamic_expansion_config,
    try_simple_route_with_dynamic_expansion_config, AStarConfig, HeapTieBreaker, HeuristicMode,
    PrimitiveOrdering, RouteResult, RouteSearchStats, State,
};
use crate::geometry_realization::{
    build_port_access as build_port_access_rs, build_port_accesses as build_port_accesses_rs,
    cells_in_grid_rect as cells_in_grid_rect_rs, centerline_length_um as centerline_length_um_rs,
    check_meander_box_free_with_prefix as check_meander_box_free_with_prefix_rs,
    full_straight_offset_bump_candidates as full_straight_offset_bump_candidates_rs,
    generate_waveguide_polygon as generate_waveguide_polygon_rs,
    meander_box_to_grid_rect as meander_box_to_grid_rect_rs,
    plan_analytic_meander_for_route as plan_analytic_meander_for_route_rs,
    plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix as plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix_rs,
    plan_auto_analytic_meander_for_route as plan_auto_analytic_meander_for_route_rs,
    plan_auto_analytic_meander_for_route_depth_sweep_with_prefix as plan_auto_analytic_meander_for_route_depth_sweep_with_prefix_rs,
    probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix as probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix_rs,
    probe_auto_analytic_meander_for_route_depth_sweep_with_prefix as probe_auto_analytic_meander_for_route_depth_sweep_with_prefix_rs,
    realize_centerline_polygon_with_terminal_tangents as realize_centerline_polygon_with_terminal_tangents_rs,
    realize_route_polygon_from_auto_plan as realize_route_polygon_from_auto_plan_rs,
    realize_route_polygon_from_primitives as realize_route_polygon_from_primitives_rs,
    realize_route_polygon_with_analytic_meander as realize_route_polygon_with_analytic_meander_rs,
    realize_route_polygon_with_auto_checked_analytic_meander as realize_route_polygon_with_auto_checked_analytic_meander_rs,
    realize_route_polygon_with_checked_analytic_meander_box as realize_route_polygon_with_checked_analytic_meander_box_rs,
    realize_route_polygon_with_endpoint_correction as realize_route_polygon_with_endpoint_correction_rs,
    realize_route_polygon_with_port_access as realize_route_polygon_with_port_access_rs,
    route_to_port_corrected_centerline as route_to_port_corrected_centerline_rs,
    route_to_primitive_centerline as route_to_primitive_centerline_rs,
    splice_meander_into_centerline_range as splice_meander_into_centerline_range_rs,
    AutoMeanderConfig, AutoMeanderPlanningProfile, AutoMeanderSidePolicy, DenseOccupancyPrefix,
    GeometryError, GeometryGridSpec, PortAccess, PortAccessConfig, SparseCellIndex,
};
use crate::meander::{
    actual_bend_radius_um_from_cells as actual_bend_radius_um_from_cells_rs,
    bend_radius_cells_from_min_radius as bend_radius_cells_from_min_radius_rs, MeanderBox,
    MeanderPlanningMode, MeanderSide, PhysicalPoint,
};
use crate::obstacle_map::{pack_xy, unpack_xy, CellKey, GridRect, ObstacleMap};
use crate::plm::{
    plan_registered_geometry_request_sequence, plan_registered_geometry_requirement_candidates,
    MeanderPlanningProfileTotals, MeanderWrapperProfileTotals, RegisteredMeanderGeometry,
    RegisteredPlmContext, RegisteredRequirementResult,
};
use crate::primitives::{
    create_grid4_unit_grid_primitive_library, create_jps4_unit_grid_primitive_library,
    create_photonic_primitive_library, Primitive, PrimitiveLibrary, PrimitiveLibraryConfig,
};
use crate::static_obstacle_builder::{
    rasterize_polygon, PortInput, PyStaticCellSet, StaticGridSpec,
};

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
    #[pyo3(get, set)]
    pub jps4_unit_grid: bool,
    #[pyo3(get, set)]
    pub grid4_unit_grid: bool,
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
            jps4_unit_grid: false,
            grid4_unit_grid: false,
        }
    }

    #[staticmethod]
    fn bend_radius_cells_from_min_radius(
        min_bend_radius_um: f64,
        grid_size_um: f64,
    ) -> PyResult<i32> {
        bend_radius_cells_from_min_radius_rs(min_bend_radius_um, grid_size_um)
            .map_err(PyValueError::new_err)
    }

    #[staticmethod]
    fn actual_bend_radius_um_from_cells(
        bend_radius_cells: i32,
        grid_size_um: f64,
    ) -> PyResult<f64> {
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
    #[pyo3(get, set)]
    pub enable_simple_routes: bool,
    #[pyo3(get, set)]
    pub simple_route_max_offset_cells: i32,
    #[pyo3(get, set)]
    pub simple_route_min_leg_len_cells: i32,
    #[pyo3(get, set)]
    pub ignore_dynamic_obstacles: bool,
    #[pyo3(get, set)]
    pub history_weight: f64,
    #[pyo3(get, set)]
    pub collect_detailed_timing: bool,
    #[pyo3(get, set)]
    pub enable_jps4: bool,
    #[pyo3(get, set)]
    pub use_indexed_heap: bool,
    #[pyo3(get, set)]
    pub primitive_ordering: String,
    #[pyo3(get, set)]
    pub heuristic_mode: String,
    #[pyo3(get, set)]
    pub heap_tie_breaker: String,
}
#[pymethods]
impl PyAStarConfig {
    #[new]
    #[pyo3(signature=(max_iterations=100_000,bend_weight=1.0,target_tolerance_cells=0,require_target_angle=true,allowed_target_angles=None,use_routing_window=true,routing_window_min_margin_cells=12,routing_window_scale=0.35,routing_window_max_expansions=3,routing_window_fallback_full_grid=true,routing_window_growth=0.5,max_dense_obstacle_cells=10_000_000,ignore_dynamic_obstacles=false,history_weight=0.0,collect_detailed_timing=false,use_indexed_heap=false,primitive_ordering="library".to_string(),heuristic_mode="heading_aware".to_string()))]
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
        ignore_dynamic_obstacles: bool,
        history_weight: f64,
        collect_detailed_timing: bool,
        use_indexed_heap: bool,
        primitive_ordering: String,
        heuristic_mode: String,
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
            enable_simple_routes: true,
            simple_route_max_offset_cells: 96,
            simple_route_min_leg_len_cells: 1,
            ignore_dynamic_obstacles,
            history_weight,
            collect_detailed_timing,
            enable_jps4: false,
            use_indexed_heap,
            primitive_ordering,
            heuristic_mode,
            heap_tie_breaker: "smaller_g".to_string(),
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
    pub last_window_min_x: i32,
    #[pyo3(get)]
    pub last_window_max_x: i32,
    #[pyo3(get)]
    pub last_window_min_y: i32,
    #[pyo3(get)]
    pub last_window_max_y: i32,
    #[pyo3(get)]
    pub last_window_area_cells: i64,
    #[pyo3(get)]
    pub expanded_states: usize,
    #[pyo3(get)]
    pub generated_neighbors: usize,
    #[pyo3(get)]
    pub heap_pushes: usize,
    #[pyo3(get)]
    pub heap_pops: usize,
    #[pyo3(get)]
    pub skipped_duplicate_heap_entries: usize,
    #[pyo3(get)]
    pub stale_generation_heap_entries: usize,
    #[pyo3(get)]
    pub closed_heap_entries: usize,
    #[pyo3(get)]
    pub max_heap_size: usize,
    #[pyo3(get)]
    pub dense_search_states: usize,
    #[pyo3(get)]
    pub dense_search_storage_bytes: usize,
    #[pyo3(get)]
    pub best_cost_updates: usize,
    #[pyo3(get)]
    pub parent_updates: usize,
    #[pyo3(get)]
    pub obstacle_clearance_checks: usize,
    #[pyo3(get)]
    pub window_rejects: usize,
    #[pyo3(get)]
    pub footprint_rejects: usize,
    #[pyo3(get)]
    pub primitive_generated_by_class: Vec<usize>,
    #[pyo3(get)]
    pub primitive_bounds_rejects_by_class: Vec<usize>,
    #[pyo3(get)]
    pub primitive_closed_rejects_by_class: Vec<usize>,
    #[pyo3(get)]
    pub primitive_cost_pruned_by_class: Vec<usize>,
    #[pyo3(get)]
    pub primitive_footprint_checks_by_class: Vec<usize>,
    #[pyo3(get)]
    pub primitive_footprint_rejects_by_class: Vec<usize>,
    #[pyo3(get)]
    pub primitive_accepted_by_class: Vec<usize>,
    #[pyo3(get)]
    pub dense_grid_build_failures: usize,
    #[pyo3(get)]
    pub max_window_area_cells: i64,
    #[pyo3(get)]
    pub primitive_footprint_checks: usize,
    #[pyo3(get)]
    pub primitive_footprint_cells_tested: usize,
    #[pyo3(get)]
    pub primitive_footprint_rect_checks: usize,
    #[pyo3(get)]
    pub primitive_footprint_rect_rejects: usize,
    #[pyo3(get)]
    pub dense_grid_cells: usize,
    #[pyo3(get)]
    pub dense_grid_build_time_us: u64,
    #[pyo3(get)]
    pub search_loop_time_us: u64,
    #[pyo3(get)]
    pub obstacle_map_prepare_time_us: u64,
    #[pyo3(get)]
    pub simple_route_time_us: u64,
    #[pyo3(get)]
    pub commit_prepare_time_us: u64,
    #[pyo3(get)]
    pub commit_time_us: u64,
    #[pyo3(get)]
    pub neighbor_generation_time_us: u64,
    #[pyo3(get)]
    pub heap_operation_time_us: u64,
    #[pyo3(get)]
    pub legality_check_time_us: u64,
    #[pyo3(get)]
    pub reconstruction_time_us: u64,
    #[pyo3(get)]
    pub jps4_requested: bool,
    #[pyo3(get)]
    pub jps4_eligible: bool,
    #[pyo3(get)]
    pub jps4_used: bool,
    #[pyo3(get)]
    pub jps4_fallbacks: usize,
    #[pyo3(get)]
    pub jps4_fallback_reason: String,
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

#[derive(Clone, Default)]
struct MeanderRegistrationProfile {
    total_s: f64,
    reset_s: f64,
    base_static_pack_s: f64,
    base_static_obstacle_add_s: f64,
    base_prefix_build_s: f64,
    route_extract_s: f64,
    route_cell_collect_s: f64,
    open_set_build_s: f64,
    route_cell_list_s: f64,
    route_static_add_s: f64,
    registered_store_s: f64,
    route_count: usize,
    base_static_cell_count: usize,
    unique_route_cell_count: usize,
    registered_open_cell_count: usize,
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
    registered_plm: RefCell<RegisteredPlmContext>,
    last_meander_registration_profile: RefCell<Option<MeanderRegistrationProfile>>,
}

fn pack_cells(cells: &[(i32, i32)]) -> FxHashSet<CellKey> {
    cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect()
}

fn collect_meander_route_cell_sets(
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

fn build_registered_open_sets(
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

fn build_registered_open_indices(
    base_prefix: &DenseOccupancyPrefix,
    registered_open_sets: &[FxHashSet<CellKey>],
) -> Vec<SparseCellIndex> {
    registered_open_sets
        .iter()
        .map(|cells| SparseCellIndex::from_opened_cells(base_prefix, cells))
        .collect()
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

fn route_commit_cells(
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

fn route_core_cells(
    center_cells: &[(i32, i32)],
    core_radius_cells: i32,
    width: i32,
    height: i32,
) -> Vec<(i32, i32)> {
    inflate_route_cells(center_cells, core_radius_cells, width, height)
}

fn unique_cells<I>(cells: I) -> Vec<(i32, i32)>
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

fn static_grid_from_py_grid(grid: &PyGridSpec) -> StaticGridSpec {
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

fn centerline_core_cells(
    centerline: &[(f64, f64)],
    width_um: f64,
    static_grid: &StaticGridSpec,
) -> Result<Vec<(i32, i32)>, GeometryError> {
    let polygon = generate_waveguide_polygon_rs(centerline, width_um)?;
    Ok(rasterize_polygon(&polygon, static_grid)
        .into_iter()
        .map(unpack_xy)
        .collect())
}

fn compact_bump_portion(centerline: &[(f64, f64)], placement_is_start: bool) -> &[(f64, f64)] {
    if centerline.len() <= 2 {
        return centerline;
    }
    if placement_is_start {
        &centerline[..centerline.len() - 1]
    } else {
        &centerline[1..]
    }
}

fn cells_bbox(cells: &[(i32, i32)]) -> Option<(i32, i32, i32, i32)> {
    let first = cells.first()?;
    let mut min_x = first.0;
    let mut max_x = first.0;
    let mut min_y = first.1;
    let mut max_y = first.1;
    for &(x, y) in cells.iter().skip(1) {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    Some((min_x, max_x, min_y, max_y))
}

fn format_bbox(cells: &[(i32, i32)]) -> String {
    cells_bbox(cells)
        .map(|(min_x, max_x, min_y, max_y)| format!("({min_x},{max_x},{min_y},{max_y})"))
        .unwrap_or_else(|| "none".to_string())
}

fn format_cell_sample(cells: &[(i32, i32)], limit: usize) -> String {
    let mut sample = cells.to_vec();
    sample.sort_unstable();
    sample.dedup();
    let text = sample
        .iter()
        .take(limit)
        .map(|(x, y)| format!("({x},{y})"))
        .collect::<Vec<_>>()
        .join(",");
    if sample.len() > limit {
        format!("{text},...")
    } else {
        text
    }
}

fn sorted_other_owners_for_cells(
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

fn cells_with_other_dynamic_owner(
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

fn parse_primitive_ordering(value: &str) -> PyResult<PrimitiveOrdering> {
    match value.trim().to_ascii_lowercase().as_str() {
        "library" => Ok(PrimitiveOrdering::Library),
        "long_straight_first" => Ok(PrimitiveOrdering::LongStraightFirst),
        "target_biased" => Ok(PrimitiveOrdering::TargetBiased),
        _ => Err(PyValueError::new_err(
            "primitive_ordering must be one of 'library', 'long_straight_first', or 'target_biased'",
        )),
    }
}

fn parse_heuristic_mode(value: &str) -> PyResult<HeuristicMode> {
    match value.trim().to_ascii_lowercase().as_str() {
        "distance" => Ok(HeuristicMode::Distance),
        "heading_aware" => Ok(HeuristicMode::HeadingAware),
        _ => Err(PyValueError::new_err(
            "heuristic_mode must be one of 'distance' or 'heading_aware'",
        )),
    }
}

fn parse_heap_tie_breaker(value: &str) -> PyResult<HeapTieBreaker> {
    match value.trim().to_ascii_lowercase().as_str() {
        "smaller_g" => Ok(HeapTieBreaker::SmallerG),
        "larger_g" => Ok(HeapTieBreaker::LargerG),
        _ => Err(PyValueError::new_err(
            "heap_tie_breaker must be one of 'smaller_g' or 'larger_g'",
        )),
    }
}

fn astar_config_from_py(
    astar_cfg: &PyAStarConfig,
    primitive_cfg: &PyPrimitiveLibraryConfig,
    ignore_dynamic_obstacles: Option<bool>,
    enable_simple_routes: Option<bool>,
    history_weight: Option<f64>,
) -> PyResult<AStarConfig> {
    let allowed_target_angles_mask =
        allowed_angles_to_mask(astar_cfg.allowed_target_angles.as_ref())?;
    let primitive_ordering = parse_primitive_ordering(&astar_cfg.primitive_ordering)?;
    let heuristic_mode = parse_heuristic_mode(&astar_cfg.heuristic_mode)?;
    let heap_tie_breaker = parse_heap_tie_breaker(&astar_cfg.heap_tie_breaker)?;
    Ok(AStarConfig {
        max_iterations: astar_cfg.max_iterations,
        bend_weight: astar_cfg.bend_weight * primitive_cfg.bend_weight,
        target_tolerance_cells: astar_cfg.target_tolerance_cells,
        require_target_angle: astar_cfg.require_target_angle,
        allowed_target_angles_mask,
        use_routing_window: astar_cfg.use_routing_window,
        routing_window_min_margin_cells: astar_cfg.routing_window_min_margin_cells,
        routing_window_scale: astar_cfg.routing_window_scale,
        routing_window_max_expansions: astar_cfg.routing_window_max_expansions,
        routing_window_fallback_full_grid: astar_cfg.routing_window_fallback_full_grid,
        routing_window_growth: astar_cfg.routing_window_growth,
        max_dense_states: AStarConfig::default().max_dense_states,
        max_dense_obstacle_cells: astar_cfg.max_dense_obstacle_cells,
        enable_simple_routes: enable_simple_routes.unwrap_or(astar_cfg.enable_simple_routes),
        simple_route_max_offset_cells: astar_cfg.simple_route_max_offset_cells,
        simple_route_min_leg_len_cells: astar_cfg.simple_route_min_leg_len_cells,
        ignore_dynamic_obstacles: ignore_dynamic_obstacles
            .unwrap_or(astar_cfg.ignore_dynamic_obstacles),
        history_weight: history_weight.unwrap_or(astar_cfg.history_weight),
        collect_detailed_timing: astar_cfg.collect_detailed_timing,
        enable_jps4: astar_cfg.enable_jps4,
        use_indexed_heap: astar_cfg.use_indexed_heap,
        primitive_ordering,
        heuristic_mode,
        heap_tie_breaker,
        require_terminal_straights: false,
    })
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
        _ => Err(PyValueError::new_err(
            "planning_mode must be 'fill_box_multi_bump'",
        )),
    }
}

fn planning_mode_to_str(mode: MeanderPlanningMode) -> &'static str {
    let _ = mode;
    "fill_box_multi_bump"
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
    dict.set_item("requested_min_bend_radius_um", requested_min_bend_radius_um)?;
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

fn set_meander_planning_profile_totals_item(
    dict: &Bound<'_, PyDict>,
    key: &str,
    totals: &MeanderPlanningProfileTotals,
) -> PyResult<()> {
    let profile = PyDict::new_bound(dict.py());
    profile.set_item("total_s", totals.total_s)?;
    profile.set_item("run_extraction_s", totals.run_extraction_s)?;
    profile.set_item("footprint_s", totals.footprint_s)?;
    profile.set_item("free_interval_s", totals.free_interval_s)?;
    profile.set_item("box_check_s", totals.box_check_s)?;
    profile.set_item("analytic_plan_s", totals.analytic_plan_s)?;
    profile.set_item("replacement_check_s", totals.replacement_check_s)?;
    profile.set_item("depth_count", totals.depth_count)?;
    profile.set_item("run_side_checks", totals.run_side_checks)?;
    profile.set_item("box_checks", totals.box_checks)?;
    profile.set_item("analytic_plan_calls", totals.analytic_plan_calls)?;
    profile.set_item("plan_calls", totals.plan_calls)?;
    dict.set_item(key, profile)?;
    Ok(())
}

fn set_meander_wrapper_profile_totals_item(
    dict: &Bound<'_, PyDict>,
    key: &str,
    totals: &MeanderWrapperProfileTotals,
) -> PyResult<()> {
    let profile = PyDict::new_bound(dict.py());
    profile.set_item("reserved_snapshot_s", totals.reserved_snapshot_s)?;
    profile.set_item("planner_call_s", totals.planner_call_s)?;
    profile.set_item("selected_rect_cells_s", totals.selected_rect_cells_s)?;
    profile.set_item(
        "candidate_reserved_update_s",
        totals.candidate_reserved_update_s,
    )?;
    profile.set_item("py_plan_conversion_s", totals.py_plan_conversion_s)?;
    profile.set_item("py_plan_append_s", totals.py_plan_append_s)?;
    profile.set_item(
        "py_candidate_result_build_s",
        totals.py_candidate_result_build_s,
    )?;
    profile.set_item("py_result_build_s", totals.py_result_build_s)?;
    profile.set_item(
        "extra_blocked_prepare_calls",
        totals.extra_blocked_prepare_calls,
    )?;
    profile.set_item("selected_rect_cell_count", totals.selected_rect_cell_count)?;
    profile.set_item("py_plan_count", totals.py_plan_count)?;
    profile.set_item("candidate_result_count", totals.candidate_result_count)?;
    dict.set_item(key, profile)?;
    Ok(())
}

fn auto_meander_planning_profile_to_py_object(
    py: Python<'_>,
    profile: &AutoMeanderPlanningProfile,
) -> PyResult<PyObject> {
    let d = PyDict::new_bound(py);
    d.set_item("total_s", profile.total_s)?;
    d.set_item("run_extraction_s", profile.run_extraction_s)?;
    d.set_item("footprint_s", profile.footprint_s)?;
    d.set_item("free_interval_s", profile.free_interval_s)?;
    d.set_item("box_check_s", profile.box_check_s)?;
    d.set_item("analytic_plan_s", profile.analytic_plan_s)?;
    d.set_item("replacement_check_s", profile.replacement_check_s)?;
    d.set_item("depth_count", profile.depth_count)?;
    d.set_item("run_side_checks", profile.run_side_checks)?;
    d.set_item("box_checks", profile.box_checks)?;
    d.set_item("analytic_plan_calls", profile.analytic_plan_calls)?;
    d.set_item("plan_calls", 1usize)?;
    Ok(d.into())
}

fn auto_meander_plan_to_py_object(
    py: Python<'_>,
    plan: &crate::geometry_realization::AutoRouteAnalyticMeanderPlan,
    requested_min_bend_radius_um: Option<f64>,
    effective_bend_radius_um: f64,
    primitive_bend_radius_cells: i32,
    primitive_bend_radius_um: f64,
    planning_mode: MeanderPlanningMode,
) -> PyResult<PyObject> {
    let d = PyDict::new_bound(py);
    d.set_item("selected_segment_index", plan.selected_segment_index)?;
    d.set_item("selected_run_start_index", plan.selected_run_start_index)?;
    d.set_item("selected_run_end_index", plan.selected_run_end_index)?;
    d.set_item("selected_run_length_um", plan.selected_run_length_um)?;
    d.set_item(
        "selected_interval_length_um",
        plan.selected_interval_length_um,
    )?;
    d.set_item("box_depth_um", plan.selected_box_depth_um)?;
    d.set_item("candidate_runs", plan.candidate_runs)?;
    d.set_item("candidate_intervals", plan.candidate_intervals)?;
    d.set_item("rejected_box_blocked", plan.rejected_box_blocked)?;
    d.set_item("rejected_planning_failed", plan.rejected_planning_failed)?;
    d.set_item(
        "rejected_exact_length_mismatch",
        plan.rejected_exact_length_mismatch,
    )?;
    d.set_item("rejected_too_short", plan.rejected_too_short)?;
    d.set_item(
        "planner_profile",
        auto_meander_planning_profile_to_py_object(py, &plan.profile)?,
    )?;
    d.set_item(
        "selected_segment",
        (
            (
                plan.selected_segment.start.x_um,
                plan.selected_segment.start.y_um,
            ),
            (
                plan.selected_segment.end.x_um,
                plan.selected_segment.end.y_um,
            ),
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
    let centerline = PyList::empty_bound(py);
    for p in &plan.replacement_centerline {
        centerline.append((p.x_um, p.y_um))?;
    }
    d.set_item("centerline", centerline)?;
    d.set_item(
        "inserted_extra_length_um",
        plan.plan.inserted_extra_length_um,
    )?;
    d.set_item("bumps", plan.plan.bumps)?;
    d.set_item("visual_bumps", plan.plan.visual_bumps)?;
    d.set_item("u_turns", plan.plan.u_turns)?;
    d.set_item("quarter_turns", plan.plan.quarter_turns)?;
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
        requested_min_bend_radius_um,
        effective_bend_radius_um,
        primitive_bend_radius_cells,
        primitive_bend_radius_um,
        planning_mode,
        Some(plan.selected_box_depth_um),
    )?;
    let mut max_possible_bumps =
        (plan.selected_box_depth_um / (2.0 * effective_bend_radius_um)).floor() as i32;
    if max_possible_bumps % 2 != 0 {
        max_possible_bumps -= 1;
    }
    d.set_item(
        "max_possible_bumps_from_box_depth",
        max_possible_bumps.max(0),
    )?;
    Ok(d.into())
}

fn auto_meander_probe_to_py_object(
    py: Python<'_>,
    probe: &crate::geometry_realization::AutoRouteAnalyticMeanderProbe,
) -> PyResult<PyObject> {
    let d = PyDict::new_bound(py);
    d.set_item("feasible", probe.feasible)?;
    d.set_item("candidate_runs", probe.candidate_runs)?;
    d.set_item("candidate_intervals", probe.candidate_intervals)?;
    d.set_item("rejected_box_blocked", probe.rejected_box_blocked)?;
    d.set_item("rejected_planning_failed", probe.rejected_planning_failed)?;
    d.set_item(
        "rejected_exact_length_mismatch",
        probe.rejected_exact_length_mismatch,
    )?;
    d.set_item("rejected_too_short", probe.rejected_too_short)?;
    d.set_item("selected_run_start_index", probe.selected_run_start_index)?;
    d.set_item("selected_run_end_index", probe.selected_run_end_index)?;
    d.set_item("selected_run_length_um", probe.selected_run_length_um)?;
    d.set_item(
        "selected_interval_length_um",
        probe.selected_interval_length_um,
    )?;
    d.set_item("box_depth_um", probe.selected_box_depth_um)?;
    if let Some(rect) = probe.selected_grid_rect {
        d.set_item(
            "selected_grid_rect",
            (rect.min_x, rect.max_x, rect.min_y, rect.max_y),
        )?;
    } else {
        d.set_item("selected_grid_rect", Option::<(i32, i32, i32, i32)>::None)?;
    }
    Ok(d.into())
}

fn annotate_endpoint_inset_sweep_result(
    py: Python<'_>,
    result: &PyObject,
    selected_endpoint_inset_um: f64,
    attempted_endpoint_insets_um: &[f64],
) -> PyResult<String> {
    let d = result.bind(py).downcast::<PyDict>()?;
    d.set_item("endpoint_inset_um", selected_endpoint_inset_um)?;
    d.set_item("endpoint_insets_attempted_um", attempted_endpoint_insets_um)?;
    d.get_item("status")?
        .ok_or_else(|| PyRuntimeError::new_err("meander result missing status"))?
        .extract::<String>()
}

const DEFAULT_MEANDER_DEPTH_CANDIDATES_UM: [f64; 12] = [
    40.0, 30.0, 24.0, 20.0, 16.0, 12.0, 10.0, 8.0, 6.0, 4.0, 3.0, 2.0,
];
const MEANDER_POLICY_DEDUPE_EPS_UM: f64 = 1.0e-9;

fn default_meander_box_depths_um(max_meander_height_um: f64) -> PyResult<Vec<f64>> {
    if !max_meander_height_um.is_finite() || max_meander_height_um <= 0.0 {
        return Err(PyValueError::new_err(
            "max_meander_height_um must be finite and > 0",
        ));
    }
    let mut depths: Vec<f64> = DEFAULT_MEANDER_DEPTH_CANDIDATES_UM
        .iter()
        .copied()
        .filter(|depth| *depth <= max_meander_height_um + MEANDER_POLICY_DEDUPE_EPS_UM)
        .collect();
    let largest_depth = depths.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if depths.is_empty() || max_meander_height_um > largest_depth + MEANDER_POLICY_DEDUPE_EPS_UM {
        depths.insert(0, max_meander_height_um);
    }
    Ok(depths)
}

fn default_endpoint_insets_um(
    effective_radius_um: f64,
    min_segment_length_um: f64,
    auto_endpoint_inset_um: Option<f64>,
) -> PyResult<Vec<f64>> {
    if let Some(endpoint_inset_um) = auto_endpoint_inset_um {
        if !endpoint_inset_um.is_finite() {
            return Err(PyValueError::new_err(
                "auto_endpoint_inset_um must be finite when provided",
            ));
        }
        return Ok(vec![endpoint_inset_um.max(0.0)]);
    }
    if !effective_radius_um.is_finite() || effective_radius_um <= 0.0 {
        return Err(PyValueError::new_err(
            "effective bend radius must be finite and > 0",
        ));
    }
    if !min_segment_length_um.is_finite() || min_segment_length_um <= 0.0 {
        return Err(PyValueError::new_err(
            "min_segment_length_um must be finite and > 0",
        ));
    }
    let base_endpoint_inset_um = effective_radius_um.max(min_segment_length_um);
    let raw_endpoint_insets = [
        base_endpoint_inset_um,
        0.75 * effective_radius_um,
        0.5 * effective_radius_um,
        0.25 * effective_radius_um,
        0.0,
    ];
    let mut endpoint_insets_um: Vec<f64> = Vec::with_capacity(raw_endpoint_insets.len());
    for inset in raw_endpoint_insets {
        let inset = inset.max(0.0);
        if endpoint_insets_um
            .iter()
            .all(|existing| (inset - *existing).abs() > MEANDER_POLICY_DEDUPE_EPS_UM)
        {
            endpoint_insets_um.push(inset);
        }
    }
    Ok(endpoint_insets_um)
}

fn annotate_auto_meander_search_policy(
    py: Python<'_>,
    result: &PyObject,
    min_straight_um: f64,
    min_segment_length_um: f64,
    max_meander_height_um: f64,
    box_depths_um: &[f64],
    endpoint_insets_um: &[f64],
    fixed_endpoint_inset: bool,
) -> PyResult<()> {
    let d = result.bind(py).downcast::<PyDict>()?;
    let endpoint_inset_policy = if fixed_endpoint_inset {
        "fixed"
    } else {
        "adaptive"
    };
    d.set_item("box_depths_um", box_depths_um)?;
    d.set_item("endpoint_insets_um", endpoint_insets_um)?;
    d.set_item("endpoint_inset_policy", endpoint_inset_policy)?;

    let search_config = auto_meander_search_config_to_py_dict(
        py,
        min_straight_um,
        min_segment_length_um,
        max_meander_height_um,
        box_depths_um,
        endpoint_insets_um,
        fixed_endpoint_inset,
    )?;
    d.set_item("search_config", search_config)?;
    Ok(())
}

fn auto_meander_search_config_to_py_dict<'py>(
    py: Python<'py>,
    min_straight_um: f64,
    min_segment_length_um: f64,
    max_meander_height_um: f64,
    box_depths_um: &[f64],
    endpoint_insets_um: &[f64],
    fixed_endpoint_inset: bool,
) -> PyResult<Bound<'py, PyDict>> {
    let endpoint_inset_um = endpoint_insets_um.first().copied().unwrap_or(0.0);
    let endpoint_inset_policy = if fixed_endpoint_inset {
        "fixed"
    } else {
        "adaptive"
    };
    let search_config = PyDict::new_bound(py);
    search_config.set_item("min_straight_um", min_straight_um)?;
    search_config.set_item("min_segment_um", min_segment_length_um)?;
    search_config.set_item("max_height_um", max_meander_height_um)?;
    search_config.set_item("box_depths_um", box_depths_um)?;
    search_config.set_item("endpoint_inset_um", endpoint_inset_um)?;
    search_config.set_item("endpoint_insets_um", endpoint_insets_um)?;
    search_config.set_item("endpoint_inset_policy", endpoint_inset_policy)?;
    Ok(search_config)
}

#[allow(clippy::too_many_arguments)]
fn registered_requirement_result_to_py_object(
    py: Python<'_>,
    result: RegisteredRequirementResult,
    requested_min_bend_radius_um: Option<f64>,
    effective_bend_radius_um: f64,
    primitive_bend_radius_cells: i32,
    primitive_bend_radius_um: f64,
    mode: MeanderPlanningMode,
    min_straight_um: f64,
    min_segment_length_um: f64,
    max_meander_height_um: f64,
) -> PyResult<PyObject> {
    let candidate_results = PyList::empty_bound(py);
    let mut selected_plans: Option<PyObject> = None;
    let mut call_wrapper_profile = result.wrapper_profile_total.clone();

    for candidate in &result.candidate_results {
        let plans = PyList::empty_bound(py);
        let mut candidate_wrapper_profile = candidate.wrapper_profile_total.clone();
        let mut py_wrapper_delta = MeanderWrapperProfileTotals::default();
        for edge_plan in &candidate.plans {
            let py_plan_conversion_start = Instant::now();
            let py_plan = auto_meander_plan_to_py_object(
                py,
                &edge_plan.plan,
                requested_min_bend_radius_um,
                effective_bend_radius_um,
                primitive_bend_radius_cells,
                primitive_bend_radius_um,
                mode,
            )?;
            let py_plan_dict = py_plan.bind(py).downcast::<PyDict>()?;
            py_plan_dict.set_item("endpoint_inset_um", edge_plan.endpoint_inset_um)?;
            py_plan_dict.set_item("box_depths_um", &result.box_depths_um)?;
            py_plan_dict.set_item("endpoint_insets_um", &result.endpoint_insets_um)?;
            let py_plan_conversion_s = py_plan_conversion_start.elapsed().as_secs_f64();
            candidate_wrapper_profile.py_plan_conversion_s += py_plan_conversion_s;
            py_wrapper_delta.py_plan_conversion_s += py_plan_conversion_s;
            candidate_wrapper_profile.py_plan_count += 1;
            py_wrapper_delta.py_plan_count += 1;
            let py_plan_append_start = Instant::now();
            plans.append(py_plan)?;
            let py_plan_append_s = py_plan_append_start.elapsed().as_secs_f64();
            candidate_wrapper_profile.py_plan_append_s += py_plan_append_s;
            py_wrapper_delta.py_plan_append_s += py_plan_append_s;
        }

        let py_candidate_result_build_start = Instant::now();
        let candidate_entry = PyDict::new_bound(py);
        candidate_entry.set_item("candidate_index", candidate.candidate_index)?;
        candidate_entry.set_item("candidate_runs", candidate.candidate_runs)?;
        candidate_entry.set_item("candidate_intervals", candidate.candidate_intervals)?;
        candidate_entry.set_item("rejected_box_blocked", candidate.rejected_box_blocked)?;
        candidate_entry.set_item(
            "rejected_planning_failed",
            candidate.rejected_planning_failed,
        )?;
        candidate_entry.set_item(
            "rejected_exact_length_mismatch",
            candidate.rejected_exact_length_mismatch,
        )?;
        candidate_entry.set_item("rejected_too_short", candidate.rejected_too_short)?;
        set_meander_planning_profile_totals_item(
            &candidate_entry,
            "planner_profile_total",
            &candidate.planner_profile_total,
        )?;
        if let Some(reason) = &candidate.failed_reason {
            candidate_entry.set_item("status", "no_candidate")?;
            candidate_entry.set_item("reason", reason)?;
            candidate_entry.set_item(
                "failed_edge_index",
                candidate.failed_edge_index.unwrap_or(0),
            )?;
        } else {
            candidate_entry.set_item("status", "planned")?;
            candidate_entry.set_item("reason", "")?;
            candidate_entry.set_item("failed_edge_index", Option::<usize>::None)?;
        }
        candidate_entry.set_item("plans", &plans)?;
        let py_candidate_result_build_s = py_candidate_result_build_start.elapsed().as_secs_f64();
        candidate_wrapper_profile.py_candidate_result_build_s += py_candidate_result_build_s;
        py_wrapper_delta.py_candidate_result_build_s += py_candidate_result_build_s;
        candidate_wrapper_profile.candidate_result_count += 1;
        py_wrapper_delta.candidate_result_count += 1;
        set_meander_wrapper_profile_totals_item(
            &candidate_entry,
            "wrapper_profile_total",
            &candidate_wrapper_profile,
        )?;
        candidate_results.append(candidate_entry)?;
        if result.selected_candidate_index == Some(candidate.candidate_index) {
            selected_plans = Some(plans.into());
        }
        call_wrapper_profile.add(&py_wrapper_delta);
    }

    let py_result_build_start = Instant::now();
    let d = PyDict::new_bound(py);
    d.set_item("status", result.status())?;
    if let Some(index) = result.selected_candidate_index {
        d.set_item("selected_candidate_index", index)?;
        d.set_item(
            "plans",
            selected_plans.expect("selected plans exist for planned candidate"),
        )?;
        d.set_item("reason", "")?;
    } else {
        d.set_item("selected_candidate_index", Option::<usize>::None)?;
        d.set_item("plans", PyList::empty_bound(py))?;
        d.set_item("reason", "no registered requirement candidate planned")?;
    }
    d.set_item("candidate_results", candidate_results)?;
    set_meander_planning_profile_totals_item(
        &d,
        "planner_profile_total",
        &result.planner_profile_total,
    )?;
    call_wrapper_profile.py_result_build_s += py_result_build_start.elapsed().as_secs_f64();
    set_meander_wrapper_profile_totals_item(&d, "wrapper_profile_total", &call_wrapper_profile)?;
    let py_result: PyObject = d.into();
    annotate_endpoint_inset_sweep_result(
        py,
        &py_result,
        result.endpoint_inset_um,
        &result.attempted_endpoint_insets_um,
    )?;
    annotate_auto_meander_search_policy(
        py,
        &py_result,
        min_straight_um,
        min_segment_length_um,
        max_meander_height_um,
        &result.box_depths_um,
        &result.endpoint_insets_um,
        result.fixed_endpoint_inset,
    )?;
    Ok(py_result)
}

#[pyfunction]
#[pyo3(signature=(effective_bend_radius_um,min_candidate_straight_length_um=1.0,max_meander_height_um=20.0,auto_endpoint_inset_um=None))]
fn auto_meander_search_config_rs(
    py: Python<'_>,
    effective_bend_radius_um: f64,
    min_candidate_straight_length_um: f64,
    max_meander_height_um: f64,
    auto_endpoint_inset_um: Option<f64>,
) -> PyResult<PyObject> {
    if !min_candidate_straight_length_um.is_finite() {
        return Err(PyValueError::new_err(
            "min_candidate_straight_length_um must be finite",
        ));
    }
    let min_straight_um = min_candidate_straight_length_um.max(0.0);
    let min_segment_length_um = min_candidate_straight_length_um.max(0.5);
    let box_depths_um = default_meander_box_depths_um(max_meander_height_um)?;
    let endpoint_insets_um = default_endpoint_insets_um(
        effective_bend_radius_um,
        min_segment_length_um,
        auto_endpoint_inset_um,
    )?;
    Ok(auto_meander_search_config_to_py_dict(
        py,
        min_straight_um,
        min_segment_length_um,
        max_meander_height_um,
        &box_depths_um,
        &endpoint_insets_um,
        auto_endpoint_inset_um.is_some(),
    )?
    .into())
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
                actual_bend_radius_um_from_cells_rs(cells, grid_size_um)
                    .map_err(PyValueError::new_err)
            }
            Some(v) => {
                if !v.is_finite() || v <= 0.0 {
                    return Err(PyValueError::new_err(
                        "min_bend_radius_um must be finite and > 0 when provided",
                    ));
                }
                let cells = bend_radius_cells_from_min_radius_rs(v, grid_size_um)
                    .map_err(PyValueError::new_err)?;
                actual_bend_radius_um_from_cells_rs(cells, grid_size_um)
                    .map_err(PyValueError::new_err)
            }
        }
    }

    #[pyo3(signature=(min_bend_radius_um=None))]
    fn describe_bend_radius(
        &self,
        py: Python<'_>,
        min_bend_radius_um: Option<f64>,
    ) -> PyResult<PyObject> {
        let primitive_bend_radius_cells = self.primitive_cfg.bend_radius_cells;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            primitive_bend_radius_cells,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let effective_bend_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let effective_bend_radius_cells =
            bend_radius_cells_from_min_radius_rs(effective_bend_radius_um, self.grid.grid_size_um)
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
        let primitives = if primitive_config.jps4_unit_grid {
            create_jps4_unit_grid_primitive_library(primitive_config.grid_size_um)
        } else if primitive_config.grid4_unit_grid {
            create_grid4_unit_grid_primitive_library(primitive_config.grid_size_um)
        } else {
            create_photonic_primitive_library(PrimitiveLibraryConfig {
                grid_size_um: primitive_config.grid_size_um,
                straight_short_cells: primitive_config.straight_short_cells,
                straight_long_cells: primitive_config.straight_long_cells,
                bend_radius_cells: primitive_config.bend_radius_cells,
                allow_45_degree_turns: primitive_config.allow_45_degree_turns,
            })
        };
        Self {
            obstacle_map: ObstacleMap::new(grid_spec.width as i32, grid_spec.height as i32),
            grid: grid_spec,
            primitive_cfg: primitive_config,
            astar_cfg: astar_config,
            primitives,
            static_cells: FxHashSet::default(),
            port_open_cells: FxHashSet::default(),
            registered_plm: RefCell::new(RegisteredPlmContext::default()),
            last_meander_registration_profile: RefCell::new(None),
        }
    }
    fn invalidate_meander_base_prefix(&self) {
        self.registered_plm.borrow_mut().invalidate_base_prefix();
    }

    fn ensure_meander_base_prefix(&self) {
        self.registered_plm
            .borrow_mut()
            .ensure_base_prefix_from_obstacle_map(&self.obstacle_map);
    }

    fn invalidate_meander_registered_reserved_index(&self) {
        self.registered_plm.borrow_mut().invalidate_reserved_index();
    }

    fn ensure_meander_registered_reserved_index(&self) {
        self.registered_plm
            .borrow_mut()
            .ensure_reserved_index(self.grid.width as i32, self.grid.height as i32);
    }

    fn add_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.invalidate_meander_base_prefix();
        for (x, y) in &cells {
            self.static_cells.insert(pack_xy(*x, *y));
        }
        self.obstacle_map.add_static_cells(&cells);
    }
    fn clear_static_cells(&mut self) {
        self.invalidate_meander_base_prefix();
        self.obstacle_map = ObstacleMap::new(self.grid.width as i32, self.grid.height as i32);
        self.static_cells.clear();
        let mut plm = self.registered_plm.borrow_mut();
        plm.clear_registered_routes();
        plm.clear_reserved_cells_and_invalidate_index();
    }
    fn set_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.clear_static_cells();
        self.add_static_cells(cells);
    }
    fn clear_registered_meander_route_cells(&self) {
        self.registered_plm.borrow_mut().clear_registered_routes();
    }
    fn clear_registered_meander_reserved_cells(&self) {
        self.registered_plm
            .borrow_mut()
            .clear_reserved_cells(self.grid.height as i32);
    }
    fn add_registered_meander_reserved_cells(&self, cells: Vec<(i32, i32)>) -> usize {
        self.registered_plm
            .borrow_mut()
            .add_reserved_cells(&cells, self.grid.width as i32)
    }
    fn add_registered_meander_reserved_grid_rect(
        &self,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
    ) -> PyResult<usize> {
        if max_x < min_x || max_y < min_y {
            return Err(PyValueError::new_err(
                "registered meander reserved grid rect must be non-empty",
            ));
        }
        Ok(self.registered_plm.borrow_mut().add_reserved_grid_rect(
            min_x,
            max_x,
            min_y,
            max_y,
            self.grid.width as i32,
        ))
    }
    fn registered_meander_open_cell_count(&self, index: usize) -> PyResult<usize> {
        self.registered_plm
            .borrow()
            .open_cells
            .get(index)
            .map(FxHashSet::len)
            .ok_or_else(|| PyValueError::new_err("registered meander route index is out of range"))
    }
    fn last_meander_registration_profile(&self, py: Python<'_>) -> PyResult<PyObject> {
        let d = PyDict::new_bound(py);
        if let Some(profile) = self.last_meander_registration_profile.borrow().as_ref() {
            d.set_item("total_s", profile.total_s)?;
            d.set_item("reset_s", profile.reset_s)?;
            d.set_item("base_static_pack_s", profile.base_static_pack_s)?;
            d.set_item(
                "base_static_obstacle_add_s",
                profile.base_static_obstacle_add_s,
            )?;
            d.set_item("base_prefix_build_s", profile.base_prefix_build_s)?;
            d.set_item("route_extract_s", profile.route_extract_s)?;
            d.set_item("route_cell_collect_s", profile.route_cell_collect_s)?;
            d.set_item("open_set_build_s", profile.open_set_build_s)?;
            d.set_item("route_cell_list_s", profile.route_cell_list_s)?;
            d.set_item("route_static_add_s", profile.route_static_add_s)?;
            d.set_item("registered_store_s", profile.registered_store_s)?;
            d.set_item("route_count", profile.route_count)?;
            d.set_item("base_static_cell_count", profile.base_static_cell_count)?;
            d.set_item("unique_route_cell_count", profile.unique_route_cell_count)?;
            d.set_item(
                "registered_open_cell_count",
                profile.registered_open_cell_count,
            )?;
        }
        Ok(d.into())
    }
    fn register_meander_route_geometries(
        &self,
        centerlines: Vec<Vec<(f64, f64)>>,
        registered_opened_cell_indices: Vec<usize>,
        max_bumps_by_edge: Vec<usize>,
    ) -> PyResult<Vec<usize>> {
        let count = centerlines.len();
        if count != registered_opened_cell_indices.len() || count != max_bumps_by_edge.len() {
            return Err(PyValueError::new_err(
                "registered meander geometry inputs must have matching lengths",
            ));
        }
        let open_cell_count = self.registered_plm.borrow().open_cells.len();
        let mut geometries = Vec::with_capacity(count);
        for ((centerline, registered_open_index), max_bumps) in centerlines
            .into_iter()
            .zip(registered_opened_cell_indices.into_iter())
            .zip(max_bumps_by_edge.into_iter())
        {
            if registered_open_index >= open_cell_count {
                return Err(PyValueError::new_err(
                    "registered meander route index is out of range",
                ));
            }
            if max_bumps == 0 {
                return Err(PyValueError::new_err(
                    "registered meander max bump values must be > 0",
                ));
            }
            let _ = centerline_length_um_rs(&centerline)
                .map_err(|err| PyValueError::new_err(err.to_string()))?;
            geometries.push(RegisteredMeanderGeometry {
                centerline,
                registered_open_index,
                max_bumps,
            });
        }
        let indices: Vec<usize> = (0..geometries.len()).collect();
        self.registered_plm.borrow_mut().geometries = geometries;
        Ok(indices)
    }
    #[pyo3(signature=(routes,base_static_cells,route_clearance_radius_cells=0))]
    fn register_meander_route_cells_as_static(
        &mut self,
        routes: &Bound<'_, PyList>,
        base_static_cells: Vec<(i32, i32)>,
        route_clearance_radius_cells: i32,
    ) -> PyResult<(Vec<usize>, Vec<usize>, usize)> {
        if route_clearance_radius_cells < 0 {
            return Err(PyValueError::new_err(
                "route_clearance_radius_cells must be >= 0",
            ));
        }
        let total_start = Instant::now();
        let mut profile = MeanderRegistrationProfile {
            route_count: routes.len(),
            base_static_cell_count: base_static_cells.len(),
            ..MeanderRegistrationProfile::default()
        };
        let reset_start = Instant::now();
        self.registered_plm
            .borrow_mut()
            .clear_reserved_cells_and_invalidate_index();
        profile.reset_s += reset_start.elapsed().as_secs_f64();
        let base_static_pack_start = Instant::now();
        let base_static_keys = pack_cells(&base_static_cells);
        profile.base_static_pack_s += base_static_pack_start.elapsed().as_secs_f64();
        let (route_cell_sets, route_cell_refcounts, unique_route_cells) =
            collect_meander_route_cell_sets(
                routes,
                route_clearance_radius_cells,
                self.grid.width as i32,
                self.grid.height as i32,
                &mut profile,
            )?;
        let (registered_open_sets, open_counts) = build_registered_open_sets(
            route_cell_sets,
            &route_cell_refcounts,
            &base_static_keys,
            &mut profile,
        );

        let route_cell_list_start = Instant::now();
        let route_cell_list: Vec<(i32, i32)> =
            unique_route_cells.iter().copied().map(unpack_xy).collect();
        profile.route_cell_list_s += route_cell_list_start.elapsed().as_secs_f64();
        let unique_route_cell_count = route_cell_list.len();
        profile.unique_route_cell_count = unique_route_cell_count;
        profile.registered_open_cell_count = open_counts.iter().sum();
        if !route_cell_list.is_empty() {
            let route_static_add_start = Instant::now();
            self.add_static_cells(route_cell_list);
            profile.route_static_add_s += route_static_add_start.elapsed().as_secs_f64();
        }

        let indices: Vec<usize> = (0..registered_open_sets.len()).collect();
        let registered_store_start = Instant::now();
        self.ensure_meander_base_prefix();
        let registered_open_indices = {
            let plm = self.registered_plm.borrow();
            let base_prefix = plm
                .base_prefix
                .as_ref()
                .expect("meander base prefix should be initialized");
            build_registered_open_indices(base_prefix, &registered_open_sets)
        };
        {
            let mut plm = self.registered_plm.borrow_mut();
            plm.open_cells = registered_open_sets;
            plm.open_indices = registered_open_indices;
        }
        profile.registered_store_s += registered_store_start.elapsed().as_secs_f64();
        profile.total_s = total_start.elapsed().as_secs_f64();
        *self.last_meander_registration_profile.borrow_mut() = Some(profile);
        Ok((indices, open_counts, unique_route_cell_count))
    }
    #[pyo3(signature=(routes,base_static_cells,route_clearance_radius_cells=0))]
    fn set_static_and_register_meander_route_cells_as_static(
        &mut self,
        routes: &Bound<'_, PyList>,
        base_static_cells: Vec<(i32, i32)>,
        route_clearance_radius_cells: i32,
    ) -> PyResult<(Vec<usize>, Vec<usize>, usize)> {
        if route_clearance_radius_cells < 0 {
            return Err(PyValueError::new_err(
                "route_clearance_radius_cells must be >= 0",
            ));
        }
        let total_start = Instant::now();
        let mut profile = MeanderRegistrationProfile {
            route_count: routes.len(),
            base_static_cell_count: base_static_cells.len(),
            ..MeanderRegistrationProfile::default()
        };
        let reset_start = Instant::now();
        self.invalidate_meander_base_prefix();
        self.obstacle_map = ObstacleMap::new(self.grid.width as i32, self.grid.height as i32);
        profile.reset_s += reset_start.elapsed().as_secs_f64();
        let base_static_pack_start = Instant::now();
        let base_static_keys = pack_cells(&base_static_cells);
        profile.base_static_pack_s += base_static_pack_start.elapsed().as_secs_f64();
        self.static_cells = base_static_keys.clone();
        {
            let mut plm = self.registered_plm.borrow_mut();
            plm.clear_registered_routes();
            plm.clear_reserved_cells_and_invalidate_index();
        }
        if !base_static_cells.is_empty() {
            let base_static_add_start = Instant::now();
            self.obstacle_map.add_static_cells(&base_static_cells);
            profile.base_static_obstacle_add_s += base_static_add_start.elapsed().as_secs_f64();
        }

        let (route_cell_sets, route_cell_refcounts, unique_route_cells) =
            collect_meander_route_cell_sets(
                routes,
                route_clearance_radius_cells,
                self.grid.width as i32,
                self.grid.height as i32,
                &mut profile,
            )?;
        let (registered_open_sets, open_counts) = build_registered_open_sets(
            route_cell_sets,
            &route_cell_refcounts,
            &base_static_keys,
            &mut profile,
        );

        let route_cell_list_start = Instant::now();
        let route_cell_list: Vec<(i32, i32)> =
            unique_route_cells.iter().copied().map(unpack_xy).collect();
        profile.route_cell_list_s += route_cell_list_start.elapsed().as_secs_f64();
        let unique_route_cell_count = route_cell_list.len();
        profile.unique_route_cell_count = unique_route_cell_count;
        profile.registered_open_cell_count = open_counts.iter().sum();
        self.static_cells.extend(unique_route_cells);
        if !route_cell_list.is_empty() {
            let route_static_add_start = Instant::now();
            self.obstacle_map.add_static_cells(&route_cell_list);
            profile.route_static_add_s += route_static_add_start.elapsed().as_secs_f64();
        }

        let indices: Vec<usize> = (0..registered_open_sets.len()).collect();
        let registered_store_start = Instant::now();
        self.ensure_meander_base_prefix();
        let registered_open_indices = {
            let plm = self.registered_plm.borrow();
            let base_prefix = plm
                .base_prefix
                .as_ref()
                .expect("meander base prefix should be initialized");
            build_registered_open_indices(base_prefix, &registered_open_sets)
        };
        {
            let mut plm = self.registered_plm.borrow_mut();
            plm.open_cells = registered_open_sets;
            plm.open_indices = registered_open_indices;
        }
        profile.registered_store_s += registered_store_start.elapsed().as_secs_f64();
        profile.total_s = total_start.elapsed().as_secs_f64();
        *self.last_meander_registration_profile.borrow_mut() = Some(profile);
        Ok((indices, open_counts, unique_route_cell_count))
    }
    #[pyo3(signature=(routes,base_static_cell_handle,route_clearance_radius_cells=0))]
    fn set_static_and_register_meander_route_cells_as_static_handle(
        &mut self,
        routes: &Bound<'_, PyList>,
        base_static_cell_handle: PyRef<'_, PyStaticCellSet>,
        route_clearance_radius_cells: i32,
    ) -> PyResult<(Vec<usize>, Vec<usize>, usize)> {
        if route_clearance_radius_cells < 0 {
            return Err(PyValueError::new_err(
                "route_clearance_radius_cells must be >= 0",
            ));
        }
        let total_start = Instant::now();
        let base_static_keys = base_static_cell_handle.keys();
        let mut profile = MeanderRegistrationProfile {
            route_count: routes.len(),
            base_static_cell_count: base_static_keys.len(),
            ..MeanderRegistrationProfile::default()
        };
        let reset_start = Instant::now();
        self.invalidate_meander_base_prefix();
        self.obstacle_map = ObstacleMap::new(self.grid.width as i32, self.grid.height as i32);
        self.static_cells = base_static_keys.clone();
        {
            let mut plm = self.registered_plm.borrow_mut();
            plm.clear_registered_routes();
            plm.clear_reserved_cells_and_invalidate_index();
        }
        profile.reset_s += reset_start.elapsed().as_secs_f64();

        let (route_cell_sets, route_cell_refcounts, unique_route_cells) =
            collect_meander_route_cell_sets(
                routes,
                route_clearance_radius_cells,
                self.grid.width as i32,
                self.grid.height as i32,
                &mut profile,
            )?;
        let (registered_open_sets, open_counts) = build_registered_open_sets(
            route_cell_sets,
            &route_cell_refcounts,
            base_static_keys,
            &mut profile,
        );

        let route_cell_list_start = Instant::now();
        let unique_route_cell_count = unique_route_cells.len();
        profile.route_cell_list_s += route_cell_list_start.elapsed().as_secs_f64();
        profile.unique_route_cell_count = unique_route_cell_count;
        profile.registered_open_cell_count = open_counts.iter().sum();
        let base_prefix_build_start = Instant::now();
        let mut base_prefix_keys = base_static_keys.clone();
        base_prefix_keys.extend(unique_route_cells);
        self.static_cells = base_prefix_keys.clone();
        self.registered_plm.borrow_mut().set_base_prefix_from_keys(
            self.grid.width as i32,
            self.grid.height as i32,
            &base_prefix_keys,
        );
        profile.base_prefix_build_s += base_prefix_build_start.elapsed().as_secs_f64();

        let indices: Vec<usize> = (0..registered_open_sets.len()).collect();
        let registered_store_start = Instant::now();
        let registered_open_indices = {
            let plm = self.registered_plm.borrow();
            let base_prefix = plm
                .base_prefix
                .as_ref()
                .expect("meander base prefix should be initialized");
            build_registered_open_indices(base_prefix, &registered_open_sets)
        };
        {
            let mut plm = self.registered_plm.borrow_mut();
            plm.open_cells = registered_open_sets;
            plm.open_indices = registered_open_indices;
        }
        profile.registered_store_s += registered_store_start.elapsed().as_secs_f64();
        profile.total_s = total_start.elapsed().as_secs_f64();
        *self.last_meander_registration_profile.borrow_mut() = Some(profile);
        Ok((indices, open_counts, unique_route_cell_count))
    }
    fn set_static_rects(&mut self, rects: Vec<(i32, i32, i32, i32)>) {
        self.clear_static_cells();
        let obstacle_rects: Vec<GridRect> = rects
            .into_iter()
            .map(|(x_min, y_min, x_max, y_max)| GridRect {
                x_min,
                y_min,
                x_max,
                y_max,
            })
            .collect();
        self.obstacle_map.set_static_rects(&obstacle_rects);
        self.invalidate_meander_base_prefix();
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
        let cfg = astar_config_from_py(&self.astar_cfg, &self.primitive_cfg, None, None, None)?;
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

    #[pyo3(signature=(net_id,source,target,block_radius_cells=0,opened_cells=None,commit_radius_cells=None,clearance_exempt_cells=None,core_radius_cells=None))]
    fn route_single_net_and_commit(
        &mut self,
        py: Python<'_>,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<Vec<(i32, i32)>>,
        core_radius_cells: Option<i32>,
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
        let mut cfg = astar_config_from_py(&self.astar_cfg, &self.primitive_cfg, None, None, None)?;
        cfg.require_terminal_straights = true;
        let dynamic_clearance_exempt_cell_vec = clearance_exempt_cells.as_deref().unwrap_or(&[]);
        let collect_timing = self.astar_cfg.collect_detailed_timing;
        let mut obstacle_map_prepare_time_us = 0u128;
        let mut simple_route_time_us = 0u128;
        let mut commit_prepare_time_us = 0u128;
        let mut commit_time_us = 0u128;
        let prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let dynamic_clearance_exempt_keys;
        let mut opened_dynamic_obstacle_map;
        let search_obstacle_map = if block_radius_cells > 0 {
            dynamic_clearance_exempt_keys = (!dynamic_clearance_exempt_cell_vec.is_empty())
                .then(|| pack_cells(dynamic_clearance_exempt_cell_vec));
            None
        } else {
            dynamic_clearance_exempt_keys = None;
            opened_dynamic_obstacle_map = self.obstacle_map.clone();
            opened_dynamic_obstacle_map
                .clear_dynamic_clearance_in_cells(dynamic_clearance_exempt_cell_vec);
            Some(&opened_dynamic_obstacle_map)
        };
        if let Some(prepare_start) = prepare_start.as_ref() {
            obstacle_map_prepare_time_us += prepare_start.elapsed().as_micros();
        }
        if block_radius_cells > 0 {
            let simple_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            if let Some(mut result) = try_simple_route_with_dynamic_expansion_config(
                &self.obstacle_map,
                &self.primitives,
                State::new(source.x, source.y, source.angle),
                State::new(target.x, target.y, target.angle),
                Some(opened_ref),
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys.as_ref(),
            ) {
                if let Some(simple_start) = simple_start.as_ref() {
                    simple_route_time_us += simple_start.elapsed().as_micros();
                }
                let commit_prepare_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let route_cells = route_commit_cells(
                    &result.cells,
                    block_radius_cells,
                    commit_radius_cells.unwrap_or(block_radius_cells),
                    clearance_exempt_cells.as_deref(),
                    self.grid.width as i32,
                    self.grid.height as i32,
                );
                let core_cells = route_core_cells(
                    &result.cells,
                    core_radius_cells.unwrap_or(block_radius_cells),
                    self.grid.width as i32,
                    self.grid.height as i32,
                );
                if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
                    commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
                }
                let commit_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let committed = self.obstacle_map.commit_route_with_clearance_overlap(
                    net_id,
                    &core_cells,
                    &route_cells,
                    clearance_exempt_cells.as_deref().unwrap_or(&[]),
                );
                if let Some(commit_start) = commit_start.as_ref() {
                    commit_time_us += commit_start.elapsed().as_micros();
                }
                if committed {
                    if collect_timing {
                        result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
                        result.stats.simple_route_time_us += simple_route_time_us;
                        result.stats.commit_prepare_time_us += commit_prepare_time_us;
                        result.stats.commit_time_us += commit_time_us;
                    }
                    self.invalidate_meander_base_prefix();
                    return Py::new(py, convert_result(py, &self.primitives, &result)?);
                }
            } else if let Some(simple_start) = simple_start.as_ref() {
                simple_route_time_us += simple_start.elapsed().as_micros();
            }
        }
        let search_cfg = if block_radius_cells > 0 {
            let mut search_cfg = astar_config_from_py(
                &self.astar_cfg,
                &self.primitive_cfg,
                None,
                Some(false),
                None,
            )?;
            search_cfg.require_terminal_straights = true;
            search_cfg
        } else {
            cfg
        };
        let mut result = if block_radius_cells > 0 {
            route_single_net_with_dynamic_expansion_config(
                &self.obstacle_map,
                &self.primitives,
                State::new(source.x, source.y, source.angle),
                State::new(target.x, target.y, target.angle),
                Some(opened_ref),
                &search_cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys.as_ref(),
            )
        } else {
            route_single_net_with_config(
                search_obstacle_map.expect("zero-radius search map should be prepared"),
                &self.primitives,
                State::new(source.x, source.y, source.angle),
                State::new(target.x, target.y, target.angle),
                Some(opened_ref),
                &search_cfg,
            )
        }
        .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;

        if collect_timing {
            result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
            result.stats.simple_route_time_us += simple_route_time_us;
        }
        let commit_prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let route_cells = route_commit_cells(
            &result.cells,
            block_radius_cells,
            commit_radius_cells.unwrap_or(block_radius_cells),
            clearance_exempt_cells.as_deref(),
            self.grid.width as i32,
            self.grid.height as i32,
        );
        let core_cells = route_core_cells(
            &result.cells,
            core_radius_cells.unwrap_or(block_radius_cells),
            self.grid.width as i32,
            self.grid.height as i32,
        );
        if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
            result.stats.commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
        }
        let commit_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let committed = self.obstacle_map.commit_route_with_clearance_overlap(
            net_id,
            &core_cells,
            &route_cells,
            clearance_exempt_cells.as_deref().unwrap_or(&[]),
        );
        if let Some(commit_start) = commit_start.as_ref() {
            result.stats.commit_time_us += commit_start.elapsed().as_micros();
        }
        if !committed {
            return Err(PyRuntimeError::new_err(
                "Failed to commit routed cells to obstacle map",
            ));
        }
        self.invalidate_meander_base_prefix();

        Py::new(py, convert_result(py, &self.primitives, &result)?)
    }

    fn ripup_route(&mut self, net_id: u64) -> bool {
        let removed = self.obstacle_map.ripup_route(net_id);
        if removed {
            self.invalidate_meander_base_prefix();
        }
        removed
    }

    fn clear_dynamic(&mut self) {
        self.obstacle_map.clear_dynamic();
        self.invalidate_meander_base_prefix();
    }

    fn get_net_cells(&self, net_id: u64) -> Vec<(i32, i32)> {
        self.obstacle_map
            .get_net_cells(net_id)
            .map(|cells| cells.iter().copied().map(unpack_xy).collect())
            .unwrap_or_default()
    }

    fn commit_route_cells(&mut self, net_id: u64, cells: Vec<(i32, i32)>) -> bool {
        let committed = self.obstacle_map.commit_route(net_id, &cells);
        if committed {
            self.invalidate_meander_base_prefix();
        }
        committed
    }

    fn dynamic_owners_for_cells(&self, cells: Vec<(i32, i32)>) -> Vec<u64> {
        let mut owners: Vec<u64> = self
            .obstacle_map
            .dynamic_owners_for_cells(&cells)
            .into_iter()
            .collect();
        owners.sort_unstable();
        owners
    }

    #[pyo3(signature=(route,block_radius_cells=0))]
    fn inflated_route_cells(
        &self,
        route: &PyRouteResult,
        block_radius_cells: i32,
    ) -> Vec<(i32, i32)> {
        inflate_route_cells(
            &route.cells,
            block_radius_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        )
    }

    #[pyo3(signature=(route,block_radius_cells=0))]
    fn dynamic_owners_for_route(&self, route: &PyRouteResult, block_radius_cells: i32) -> Vec<u64> {
        let cells = inflate_route_cells(
            &route.cells,
            block_radius_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        self.dynamic_owners_for_cells(cells)
    }

    #[pyo3(signature=(cells,amount=1))]
    fn add_history_cells(&mut self, cells: Vec<(i32, i32)>, amount: u32) {
        for (x, y) in cells {
            self.obstacle_map.add_history_cost(x, y, amount);
        }
    }

    #[pyo3(signature=(route,block_radius_cells=0,amount=1))]
    fn add_history_for_route(
        &mut self,
        route: &PyRouteResult,
        block_radius_cells: i32,
        amount: u32,
    ) {
        let cells = inflate_route_cells(
            &route.cells,
            block_radius_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        self.add_history_cells(cells, amount);
    }

    fn clear_history(&mut self) {
        self.obstacle_map.clear_history();
    }

    #[pyo3(signature=(source,target,block_radius_cells=0,opened_cells=None))]
    fn route_single_net_ignore_dynamic(
        &self,
        py: Python<'_>,
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
        let cfg = astar_config_from_py(
            &self.astar_cfg,
            &self.primitive_cfg,
            Some(true),
            Some(false),
            Some(0.0),
        )?;
        let _ = block_radius_cells;
        let mut static_only_obstacle_map = self.obstacle_map.clone();
        static_only_obstacle_map.clear_dynamic();
        let result = route_single_net_with_config(
            &static_only_obstacle_map,
            &self.primitives,
            State::new(source.x, source.y, source.angle),
            State::new(target.x, target.y, target.angle),
            Some(opened_ref),
            &cfg,
        )
        .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;
        Py::new(py, convert_result(py, &self.primitives, &result)?)
    }

    #[pyo3(signature=(net_id,source,target,block_radius_cells=0,opened_cells=None,history_weight=1.0,commit_radius_cells=None,clearance_exempt_cells=None,core_radius_cells=None))]
    fn route_single_net_and_commit_repair(
        &mut self,
        py: Python<'_>,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
        history_weight: f64,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<Vec<(i32, i32)>>,
        core_radius_cells: Option<i32>,
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
        let mut cfg = astar_config_from_py(
            &self.astar_cfg,
            &self.primitive_cfg,
            Some(false),
            Some(false),
            Some(history_weight),
        )?;
        cfg.require_terminal_straights = true;
        let dynamic_clearance_exempt_cell_vec = clearance_exempt_cells.as_deref().unwrap_or(&[]);
        let collect_timing = self.astar_cfg.collect_detailed_timing;
        let prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let dynamic_clearance_exempt_keys;
        let mut opened_dynamic_obstacle_map;
        let search_obstacle_map = if block_radius_cells > 0 {
            dynamic_clearance_exempt_keys = (!dynamic_clearance_exempt_cell_vec.is_empty())
                .then(|| pack_cells(dynamic_clearance_exempt_cell_vec));
            None
        } else {
            dynamic_clearance_exempt_keys = None;
            opened_dynamic_obstacle_map = self.obstacle_map.clone();
            opened_dynamic_obstacle_map
                .clear_dynamic_clearance_in_cells(dynamic_clearance_exempt_cell_vec);
            Some(&opened_dynamic_obstacle_map)
        };
        let obstacle_map_prepare_time_us = prepare_start
            .as_ref()
            .map_or(0, |start| start.elapsed().as_micros());
        let mut result = if block_radius_cells > 0 {
            route_single_net_with_dynamic_expansion_config(
                &self.obstacle_map,
                &self.primitives,
                State::new(source.x, source.y, source.angle),
                State::new(target.x, target.y, target.angle),
                Some(opened_ref),
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys.as_ref(),
            )
        } else {
            route_single_net_with_config(
                search_obstacle_map.expect("zero-radius search map should be prepared"),
                &self.primitives,
                State::new(source.x, source.y, source.angle),
                State::new(target.x, target.y, target.angle),
                Some(opened_ref),
                &cfg,
            )
        }
        .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;

        if collect_timing {
            result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
        }
        let commit_prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let route_cells = route_commit_cells(
            &result.cells,
            block_radius_cells,
            commit_radius_cells.unwrap_or(block_radius_cells),
            clearance_exempt_cells.as_deref(),
            self.grid.width as i32,
            self.grid.height as i32,
        );
        let core_cells = route_core_cells(
            &result.cells,
            core_radius_cells.unwrap_or(block_radius_cells),
            self.grid.width as i32,
            self.grid.height as i32,
        );
        if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
            result.stats.commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
        }
        let commit_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let committed = self.obstacle_map.commit_route_with_clearance_overlap(
            net_id,
            &core_cells,
            &route_cells,
            clearance_exempt_cells.as_deref().unwrap_or(&[]),
        );
        if let Some(commit_start) = commit_start.as_ref() {
            result.stats.commit_time_us += commit_start.elapsed().as_micros();
        }
        if !committed {
            return Err(PyRuntimeError::new_err(
                "Failed to commit routed cells to obstacle map",
            ));
        }
        self.invalidate_meander_base_prefix();

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

    #[pyo3(signature=(route,source_port_um=None,target_port_um=None))]
    fn route_port_corrected_centerline(
        &self,
        route: &PyRouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        route_to_port_corrected_centerline_rs(
            &r,
            &self.primitives,
            &grid,
            source_port_um,
            target_port_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(net_id,route,width_um,clearance_radius_cells,core_radius_cells,opened_cells=None,clearance_exempt_cells=None,source_port_um=None,target_port_um=None))]
    fn route_port_corrected_centerline_checked_and_commit(
        &mut self,
        py: Python<'_>,
        net_id: u64,
        route: &PyRouteResult,
        width_um: f64,
        clearance_radius_cells: i32,
        core_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
        clearance_exempt_cells: Option<Vec<(i32, i32)>>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> PyResult<Py<PyDict>> {
        let _ = clearance_radius_cells;
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let candidates = full_straight_offset_bump_candidates_rs(
            &r,
            &self.primitives,
            &grid,
            source_port_um,
            target_port_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        if candidates.is_empty() {
            let centerline = route_to_port_corrected_centerline_rs(
                &r,
                &self.primitives,
                &grid,
                source_port_um,
                target_port_um,
            )
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
            let d = PyDict::new_bound(py);
            d.set_item("centerline", centerline)?;
            d.set_item("committed_bump", false)?;
            d.set_item("candidate_index", py.None())?;
            return Ok(d.into());
        }

        let static_grid = static_grid_from_py_grid(&self.grid);
        let width = self.grid.width as i32;
        let height = self.grid.height as i32;
        let opened_cell_vec = opened_cells.unwrap_or_default();
        let opened_keys = pack_cells(&opened_cell_vec);
        let clearance_exempt_cell_vec = clearance_exempt_cells.unwrap_or_default();
        let clearance_exempt_keys = pack_cells(&clearance_exempt_cell_vec);
        let old_blocked_cells: Vec<(i32, i32)> = self
            .obstacle_map
            .get_net_cells(net_id)
            .map(|cells| cells.iter().copied().map(unpack_xy).collect())
            .unwrap_or_default();
        let old_core_cells = route_core_cells(&r.cells, core_radius_cells, width, height);
        let commit_clearance_exempt_cell_vec = unique_cells(
            clearance_exempt_cell_vec
                .iter()
                .copied()
                .chain(old_core_cells.iter().copied()),
        );
        let commit_clearance_exempt_keys = pack_cells(&commit_clearance_exempt_cell_vec);
        let mut rejection_details = Vec::new();

        for (candidate_index, candidate) in candidates.into_iter().enumerate() {
            let candidate_label = candidate.label;
            let centerline = candidate.centerline;
            let bump_centerline = compact_bump_portion(&centerline, candidate.placement_is_start);
            let candidate_core_cells =
                centerline_core_cells(bump_centerline, width_um, &static_grid)
                    .map_err(|err| PyValueError::new_err(err.to_string()))?;
            if candidate_core_cells.is_empty() {
                rejection_details.push(format!(
                    "#{candidate_index} {candidate_label}: empty core footprint"
                ));
                continue;
            }
            // A case-4 bump is an endpoint adapter, so its compact footprint is
            // the local static opening for that port. Dynamic ownership is
            // still checked below against other committed nets.
            let local_endpoint_open_keys = pack_cells(&candidate_core_cells);
            let out_of_bounds: Vec<(i32, i32)> = candidate_core_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let static_blockers: Vec<(i32, i32)> = candidate_core_cells
                .iter()
                .copied()
                .filter(|&(x, y)| {
                    let key = pack_xy(x, y);
                    self.obstacle_map.in_bounds(x, y)
                        && self.obstacle_map.is_static_blocked(x, y)
                        && !opened_keys.contains(&key)
                        && !local_endpoint_open_keys.contains(&key)
                })
                .collect();
            let dynamic_blockers = cells_with_other_dynamic_owner(
                &self.obstacle_map,
                &candidate_core_cells,
                &clearance_exempt_keys,
                net_id,
            );
            if !out_of_bounds.is_empty()
                || !static_blockers.is_empty()
                || !dynamic_blockers.is_empty()
            {
                let mut reasons = Vec::new();
                if !out_of_bounds.is_empty() {
                    reasons.push(format!(
                        "out_of_bounds={} bbox={} sample={}",
                        out_of_bounds.len(),
                        format_bbox(&out_of_bounds),
                        format_cell_sample(&out_of_bounds, 8)
                    ));
                }
                if !static_blockers.is_empty() {
                    reasons.push(format!(
                        "static_overlap={} bbox={} sample={}",
                        static_blockers.len(),
                        format_bbox(&static_blockers),
                        format_cell_sample(&static_blockers, 8)
                    ));
                }
                if !dynamic_blockers.is_empty() {
                    let owners = sorted_other_owners_for_cells(
                        &self.obstacle_map,
                        &dynamic_blockers,
                        net_id,
                    );
                    reasons.push(format!(
                        "dynamic_overlap={} owners={owners:?} bbox={} sample={}",
                        dynamic_blockers.len(),
                        format_bbox(&dynamic_blockers),
                        format_cell_sample(&dynamic_blockers, 8)
                    ));
                }
                rejection_details.push(format!(
                    "#{candidate_index} {candidate_label}: {} core_cells={} core_bbox={}",
                    reasons.join(", "),
                    candidate_core_cells.len(),
                    format_bbox(&candidate_core_cells)
                ));
                continue;
            }

            let candidate_blocked_cells =
                inflate_route_cells(&candidate_core_cells, core_radius_cells, width, height);
            let merged_core_cells = unique_cells(
                old_core_cells
                    .iter()
                    .copied()
                    .chain(candidate_core_cells.iter().copied()),
            );
            let merged_blocked_cells = unique_cells(
                old_blocked_cells
                    .iter()
                    .copied()
                    .chain(candidate_blocked_cells.iter().copied()),
            );

            if self.obstacle_map.commit_route_with_clearance_overlap(
                net_id,
                &merged_core_cells,
                &merged_blocked_cells,
                &commit_clearance_exempt_cell_vec,
            ) {
                self.invalidate_meander_base_prefix();
                let d = PyDict::new_bound(py);
                d.set_item("centerline", centerline)?;
                d.set_item("committed_bump", true)?;
                d.set_item("candidate_index", candidate_index)?;
                d.set_item("candidate_label", candidate_label)?;
                return Ok(d.into());
            }
            let commit_dynamic_blockers = cells_with_other_dynamic_owner(
                &self.obstacle_map,
                &merged_core_cells,
                &commit_clearance_exempt_keys,
                net_id,
            );
            let commit_out_of_bounds: Vec<(i32, i32)> = merged_blocked_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let commit_owners =
                sorted_other_owners_for_cells(&self.obstacle_map, &commit_dynamic_blockers, net_id);
            rejection_details.push(format!(
                "#{candidate_index} {candidate_label}: commit_rejected dynamic_overlap={} owners={commit_owners:?} dynamic_bbox={} dynamic_sample={} out_of_bounds={} out_of_bounds_bbox={} core_cells={} core_bbox={}",
                commit_dynamic_blockers.len(),
                format_bbox(&commit_dynamic_blockers),
                format_cell_sample(&commit_dynamic_blockers, 8),
                commit_out_of_bounds.len(),
                format_bbox(&commit_out_of_bounds),
                candidate_core_cells.len(),
                format_bbox(&candidate_core_cells)
            ));
        }

        Err(PyRuntimeError::new_err(format!(
            "No collision-free port endpoint case-4 bump placement found; candidates: {}",
            rejection_details.join("; ")
        )))
    }

    #[pyo3(signature=(route,width_um,source_port_um=None,target_port_um=None))]
    fn realize_route_polygon_with_endpoint_correction(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        realize_route_polygon_with_endpoint_correction_rs(
            &r,
            &self.primitives,
            &grid,
            width_um,
            source_port_um,
            target_port_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    fn centerline_length_um(&self, centerline: Vec<(f64, f64)>) -> PyResult<f64> {
        centerline_length_um_rs(&centerline).map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(centerline,width_um))]
    fn realize_centerline_polygon(
        &self,
        centerline: Vec<(f64, f64)>,
        width_um: f64,
    ) -> PyResult<Vec<(f64, f64)>> {
        generate_waveguide_polygon_rs(&centerline, width_um)
            .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(centerline,width_um,route,source_enabled=true,target_enabled=true))]
    fn realize_centerline_polygon_with_terminal_tangents(
        &self,
        centerline: Vec<(f64, f64)>,
        width_um: f64,
        route: &PyRouteResult,
        source_enabled: bool,
        target_enabled: bool,
    ) -> PyResult<Vec<(f64, f64)>> {
        let r = to_route_result(route);
        realize_centerline_polygon_with_terminal_tangents_rs(
            &centerline,
            &r,
            width_um,
            source_enabled,
            target_enabled,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(centerline,width_um,selected_run_start_index,selected_run_end_index,meander_centerline))]
    fn realize_centerline_polygon_from_planned_auto_meander(
        &self,
        centerline: Vec<(f64, f64)>,
        width_um: f64,
        selected_run_start_index: usize,
        selected_run_end_index: usize,
        meander_centerline: Vec<(f64, f64)>,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        let _ = centerline_length_um_rs(&centerline)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let meander_points: Vec<PhysicalPoint> = meander_centerline
            .into_iter()
            .map(|(x_um, y_um)| PhysicalPoint { x_um, y_um })
            .collect();
        let spliced = splice_meander_into_centerline_range_rs(
            &centerline,
            selected_run_start_index,
            selected_run_end_index,
            &meander_points,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        generate_waveguide_polygon_rs(&spliced, width_um)
            .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(centerline,width_um,route,selected_run_start_index,selected_run_end_index,meander_centerline,source_enabled=true,target_enabled=true))]
    fn realize_centerline_polygon_from_planned_auto_meander_with_terminal_tangents(
        &self,
        centerline: Vec<(f64, f64)>,
        width_um: f64,
        route: &PyRouteResult,
        selected_run_start_index: usize,
        selected_run_end_index: usize,
        meander_centerline: Vec<(f64, f64)>,
        source_enabled: bool,
        target_enabled: bool,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        let _ = centerline_length_um_rs(&centerline)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let meander_points: Vec<PhysicalPoint> = meander_centerline
            .into_iter()
            .map(|(x_um, y_um)| PhysicalPoint { x_um, y_um })
            .collect();
        let spliced = splice_meander_into_centerline_range_rs(
            &centerline,
            selected_run_start_index,
            selected_run_end_index,
            &meander_points,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        realize_centerline_polygon_with_terminal_tangents_rs(
            &spliced,
            &r,
            width_um,
            source_enabled,
            target_enabled,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route,width_um,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,side="left",available_box=None,planning_mode="fill_box_multi_bump"))]
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

    #[pyo3(signature=(route,width_um,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,side="left",available_box=None,clearance_radius_cells=0,opened_cells=None,planning_mode="fill_box_multi_bump"))]
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
            endpoint_inset_um: 0.0,
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

        auto_meander_plan_to_py_object(
            py,
            &plan,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
        )
    }

    #[pyo3(signature=(route,requested_extra_length_um,box_depths_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,min_segment_length_um=10.0,endpoint_inset_um=0.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump",extra_blocked_cells=None))]
    fn plan_auto_analytic_meander_for_route_depth_sweep(
        &self,
        py: Python<'_>,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        box_depths_um: Vec<f64>,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        endpoint_inset_um: f64,
        clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
        extra_blocked_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<PyObject> {
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if box_depths_um.is_empty() {
            return Err(PyValueError::new_err("box_depths_um must not be empty"));
        }
        if box_depths_um.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(PyValueError::new_err(
                "box_depths_um values must be finite and > 0",
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
        if min_segment_length_um <= 0.0 {
            return Err(PyValueError::new_err("min_segment_length_um must be > 0"));
        }
        if endpoint_inset_um < 0.0 {
            return Err(PyValueError::new_err("endpoint_inset_um must be >= 0"));
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
            box_depth_um: box_depths_um[0],
            min_segment_length_um,
            endpoint_inset_um,
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
        let extra_blocked_owned;
        let extra_blocked_ref: Option<&FxHashSet<CellKey>> =
            if let Some(cells) = extra_blocked_cells.as_ref() {
                extra_blocked_owned = pack_cells(cells);
                Some(&extra_blocked_owned)
            } else {
                None
            };
        self.ensure_meander_base_prefix();
        let plm = self.registered_plm.borrow();
        let base_prefix = plm
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let plan = plan_auto_analytic_meander_for_route_depth_sweep_with_prefix_rs(
            &r,
            &self.primitives,
            &grid,
            base_prefix,
            opened_ref,
            None,
            extra_blocked_ref,
            None,
            None,
            &cfg,
            &box_depths_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        auto_meander_plan_to_py_object(
            py,
            &plan,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
        )
    }

    #[pyo3(signature=(route,requested_extra_length_um,box_depths_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,min_segment_length_um=10.0,endpoint_inset_um=0.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump",extra_blocked_cells=None))]
    fn probe_auto_analytic_meander_for_route_depth_sweep(
        &self,
        py: Python<'_>,
        route: &PyRouteResult,
        requested_extra_length_um: f64,
        box_depths_um: Vec<f64>,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        endpoint_inset_um: f64,
        clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
        extra_blocked_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<PyObject> {
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if box_depths_um.is_empty() {
            return Err(PyValueError::new_err("box_depths_um must not be empty"));
        }
        if box_depths_um.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(PyValueError::new_err(
                "box_depths_um values must be finite and > 0",
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
        if min_segment_length_um <= 0.0 {
            return Err(PyValueError::new_err("min_segment_length_um must be > 0"));
        }
        if endpoint_inset_um < 0.0 {
            return Err(PyValueError::new_err("endpoint_inset_um must be >= 0"));
        }
        if clearance_radius_cells < 0 {
            return Err(PyValueError::new_err("clearance_radius_cells must be >= 0"));
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
            box_depth_um: box_depths_um[0],
            min_segment_length_um,
            endpoint_inset_um,
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
        let extra_blocked_owned;
        let extra_blocked_ref: Option<&FxHashSet<CellKey>> =
            if let Some(cells) = extra_blocked_cells.as_ref() {
                extra_blocked_owned = pack_cells(cells);
                Some(&extra_blocked_owned)
            } else {
                None
            };
        self.ensure_meander_base_prefix();
        let plm = self.registered_plm.borrow();
        let base_prefix = plm
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let probe = probe_auto_analytic_meander_for_route_depth_sweep_with_prefix_rs(
            &r,
            &self.primitives,
            &grid,
            base_prefix,
            opened_ref,
            extra_blocked_ref,
            &cfg,
            &box_depths_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        auto_meander_probe_to_py_object(py, &probe)
    }

    #[pyo3(signature=(centerline,requested_extra_length_um,box_depths_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,min_segment_length_um=10.0,endpoint_inset_um=0.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump",extra_blocked_cells=None))]
    fn plan_auto_analytic_meander_for_centerline_depth_sweep(
        &self,
        py: Python<'_>,
        centerline: Vec<(f64, f64)>,
        requested_extra_length_um: f64,
        box_depths_um: Vec<f64>,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        endpoint_inset_um: f64,
        clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
        extra_blocked_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<PyObject> {
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if box_depths_um.is_empty() {
            return Err(PyValueError::new_err("box_depths_um must not be empty"));
        }
        if box_depths_um.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(PyValueError::new_err(
                "box_depths_um values must be finite and > 0",
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
        if min_segment_length_um <= 0.0 {
            return Err(PyValueError::new_err("min_segment_length_um must be > 0"));
        }
        if endpoint_inset_um < 0.0 {
            return Err(PyValueError::new_err("endpoint_inset_um must be >= 0"));
        }
        if clearance_radius_cells < 0 {
            return Err(PyValueError::new_err("clearance_radius_cells must be >= 0"));
        }
        let _ = centerline_length_um_rs(&centerline)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
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
            box_depth_um: box_depths_um[0],
            min_segment_length_um,
            endpoint_inset_um,
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
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };
        let extra_blocked_owned;
        let extra_blocked_ref: Option<&FxHashSet<CellKey>> =
            if let Some(cells) = extra_blocked_cells.as_ref() {
                extra_blocked_owned = pack_cells(cells);
                Some(&extra_blocked_owned)
            } else {
                None
            };
        self.ensure_meander_base_prefix();
        let plm = self.registered_plm.borrow();
        let base_prefix = plm
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let plan = plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix_rs(
            &centerline,
            &grid,
            base_prefix,
            opened_ref,
            None,
            extra_blocked_ref,
            None,
            None,
            &cfg,
            &box_depths_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        auto_meander_plan_to_py_object(
            py,
            &plan,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
        )
    }

    #[pyo3(signature=(candidate_geometry_indices,candidate_requested_extra_lengths_um,min_bend_radius_um=None,min_straight_um=0.0,max_meander_height_um=20.0,min_segment_length_um=10.0,auto_endpoint_inset_um=None,clearance_radius_cells=0,side_policy="both",planning_mode="fill_box_multi_bump"))]
    #[allow(clippy::too_many_arguments)]
    fn plan_auto_analytic_meander_requirement_candidate_indices_registered_opened_auto_config(
        &self,
        py: Python<'_>,
        candidate_geometry_indices: Vec<Vec<usize>>,
        candidate_requested_extra_lengths_um: Vec<f64>,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        auto_endpoint_inset_um: Option<f64>,
        clearance_radius_cells: i32,
        side_policy: &str,
        planning_mode: &str,
    ) -> PyResult<PyObject> {
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let box_depths_um = default_meander_box_depths_um(max_meander_height_um)?;
        let endpoint_insets_um = default_endpoint_insets_um(
            effective_radius_um,
            min_segment_length_um,
            auto_endpoint_inset_um,
        )?;
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            self.primitive_cfg.bend_radius_cells,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        self.ensure_meander_base_prefix();
        self.ensure_meander_registered_reserved_index();
        let plm = self.registered_plm.borrow();
        let base_prefix = plm
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let result = plan_registered_geometry_requirement_candidates(
            &candidate_geometry_indices,
            &candidate_requested_extra_lengths_um,
            &plm.geometries,
            &plm.open_cells,
            &plm.open_indices,
            base_prefix,
            plm.reserved_index.as_ref(),
            &grid,
            self.grid.width as i32,
            self.grid.height as i32,
            &box_depths_um,
            &endpoint_insets_um,
            auto_endpoint_inset_um.is_some(),
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            policy,
            mode,
        )
        .map_err(PyValueError::new_err)?;

        registered_requirement_result_to_py_object(
            py,
            result,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            min_straight_um,
            min_segment_length_um,
            max_meander_height_um,
        )
    }

    #[pyo3(signature=(geometry_indices,requested_extra_lengths_um,min_bend_radius_um=None,min_straight_um=0.0,max_meander_height_um=20.0,min_segment_length_um=10.0,auto_endpoint_inset_um=None,clearance_radius_cells=0,side_policy="both",planning_mode="fill_box_multi_bump"))]
    #[allow(clippy::too_many_arguments)]
    fn plan_auto_analytic_meander_geometry_sequence_registered_opened_auto_config(
        &self,
        py: Python<'_>,
        geometry_indices: Vec<usize>,
        requested_extra_lengths_um: Vec<f64>,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        auto_endpoint_inset_um: Option<f64>,
        clearance_radius_cells: i32,
        side_policy: &str,
        planning_mode: &str,
    ) -> PyResult<PyObject> {
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let box_depths_um = default_meander_box_depths_um(max_meander_height_um)?;
        let endpoint_insets_um = default_endpoint_insets_um(
            effective_radius_um,
            min_segment_length_um,
            auto_endpoint_inset_um,
        )?;
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            self.primitive_cfg.bend_radius_cells,
            self.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        self.ensure_meander_base_prefix();
        self.ensure_meander_registered_reserved_index();
        let plm = self.registered_plm.borrow();
        let base_prefix = plm
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let result = plan_registered_geometry_request_sequence(
            &geometry_indices,
            &requested_extra_lengths_um,
            &plm.geometries,
            &plm.open_cells,
            &plm.open_indices,
            base_prefix,
            plm.reserved_index.as_ref(),
            &grid,
            self.grid.width as i32,
            self.grid.height as i32,
            &box_depths_um,
            &endpoint_insets_um,
            auto_endpoint_inset_um.is_some(),
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            policy,
            mode,
        )
        .map_err(PyValueError::new_err)?;

        registered_requirement_result_to_py_object(
            py,
            result,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            min_straight_um,
            min_segment_length_um,
            max_meander_height_um,
        )
    }

    #[pyo3(signature=(centerline,requested_extra_length_um,box_depths_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,min_segment_length_um=10.0,endpoint_inset_um=0.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump",extra_blocked_cells=None))]
    fn probe_auto_analytic_meander_for_centerline_depth_sweep(
        &self,
        py: Python<'_>,
        centerline: Vec<(f64, f64)>,
        requested_extra_length_um: f64,
        box_depths_um: Vec<f64>,
        min_bend_radius_um: Option<f64>,
        min_straight_um: f64,
        max_bumps: usize,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        endpoint_inset_um: f64,
        clearance_radius_cells: i32,
        side_policy: &str,
        opened_cells: Option<Vec<(i32, i32)>>,
        planning_mode: &str,
        extra_blocked_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<PyObject> {
        if requested_extra_length_um <= 0.0 {
            return Err(PyValueError::new_err(
                "requested_extra_length_um must be > 0",
            ));
        }
        if box_depths_um.is_empty() {
            return Err(PyValueError::new_err("box_depths_um must not be empty"));
        }
        if box_depths_um.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(PyValueError::new_err(
                "box_depths_um values must be finite and > 0",
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
        if min_segment_length_um <= 0.0 {
            return Err(PyValueError::new_err("min_segment_length_um must be > 0"));
        }
        if endpoint_inset_um < 0.0 {
            return Err(PyValueError::new_err("endpoint_inset_um must be >= 0"));
        }
        if clearance_radius_cells < 0 {
            return Err(PyValueError::new_err("clearance_radius_cells must be >= 0"));
        }
        let _ = centerline_length_um_rs(&centerline)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let effective_radius_um = self.effective_bend_radius_um(min_bend_radius_um)?;
        let cfg = AutoMeanderConfig {
            requested_extra_length_um,
            min_bend_radius_um: effective_radius_um,
            min_straight_um,
            max_bumps,
            max_meander_height_um,
            box_depth_um: box_depths_um[0],
            min_segment_length_um,
            endpoint_inset_um,
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
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.port_open_cells)
        };
        let extra_blocked_owned;
        let extra_blocked_ref: Option<&FxHashSet<CellKey>> =
            if let Some(cells) = extra_blocked_cells.as_ref() {
                extra_blocked_owned = pack_cells(cells);
                Some(&extra_blocked_owned)
            } else {
                None
            };
        self.ensure_meander_base_prefix();
        let plm = self.registered_plm.borrow();
        let base_prefix = plm
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let probe = probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix_rs(
            &centerline,
            &grid,
            base_prefix,
            opened_ref,
            extra_blocked_ref,
            &cfg,
            &box_depths_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        auto_meander_probe_to_py_object(py, &probe)
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
            endpoint_inset_um: 0.0,
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

    #[pyo3(signature=(route,width_um,selected_run_start_index,selected_run_end_index,meander_centerline))]
    fn realize_route_polygon_from_planned_auto_meander(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        selected_run_start_index: usize,
        selected_run_end_index: usize,
        meander_centerline: Vec<(f64, f64)>,
    ) -> PyResult<Vec<(f64, f64)>> {
        if width_um <= 0.0 {
            return Err(PyValueError::new_err("width_um must be > 0"));
        }
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let base_centerline = route_to_primitive_centerline_rs(&r, &self.primitives, &grid)
            .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let meander_points: Vec<PhysicalPoint> = meander_centerline
            .into_iter()
            .map(|(x_um, y_um)| PhysicalPoint { x_um, y_um })
            .collect();
        let spliced = splice_meander_into_centerline_range_rs(
            &base_centerline,
            selected_run_start_index,
            selected_run_end_index,
            &meander_points,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        generate_waveguide_polygon_rs(&spliced, width_um)
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
            endpoint_inset_um: 0.0,
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

        let cfg = astar_config_from_py(&self.astar_cfg, &self.primitive_cfg, None, None, None)?;

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
            endpoint_inset_um: 0.0,
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
        for (x, y) in route_cells
            .into_iter()
            .chain(reserved_cells.iter().copied())
        {
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
        self.invalidate_meander_base_prefix();

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
        d.set_item(
            "inserted_extra_length_um",
            auto_plan.plan.inserted_extra_length_um,
        )?;
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

    #[pyo3(signature=(route,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,side="left",available_box=None,planning_mode="fill_box_multi_bump"))]
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
                (
                    plan.selected_segment.end.x_um,
                    plan.selected_segment.end.y_um,
                ),
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
        d.set_item(
            "inserted_extra_length_um",
            plan.plan.inserted_extra_length_um,
        )?;
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
    m.add_function(wrap_pyfunction!(auto_meander_search_config_rs, m)?)?;
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
        last_window_min_x: r.stats.last_window_min_x,
        last_window_max_x: r.stats.last_window_max_x,
        last_window_min_y: r.stats.last_window_min_y,
        last_window_max_y: r.stats.last_window_max_y,
        last_window_area_cells: r.stats.last_window_area_cells,
        expanded_states: r.stats.expanded_states,
        generated_neighbors: r.stats.generated_neighbors,
        heap_pushes: r.stats.heap_pushes,
        heap_pops: r.stats.heap_pops,
        skipped_duplicate_heap_entries: r.stats.skipped_duplicate_heap_entries,
        stale_generation_heap_entries: r.stats.stale_generation_heap_entries,
        closed_heap_entries: r.stats.closed_heap_entries,
        max_heap_size: r.stats.max_heap_size,
        dense_search_states: r.stats.dense_search_states,
        dense_search_storage_bytes: r.stats.dense_search_storage_bytes,
        best_cost_updates: r.stats.best_cost_updates,
        parent_updates: r.stats.parent_updates,
        obstacle_clearance_checks: r.stats.obstacle_clearance_checks,
        window_rejects: r.stats.window_rejects,
        footprint_rejects: r.stats.footprint_rejects,
        primitive_generated_by_class: r.stats.primitive_generated_by_class.to_vec(),
        primitive_bounds_rejects_by_class: r.stats.primitive_bounds_rejects_by_class.to_vec(),
        primitive_closed_rejects_by_class: r.stats.primitive_closed_rejects_by_class.to_vec(),
        primitive_cost_pruned_by_class: r.stats.primitive_cost_pruned_by_class.to_vec(),
        primitive_footprint_checks_by_class: r.stats.primitive_footprint_checks_by_class.to_vec(),
        primitive_footprint_rejects_by_class: r.stats.primitive_footprint_rejects_by_class.to_vec(),
        primitive_accepted_by_class: r.stats.primitive_accepted_by_class.to_vec(),
        dense_grid_build_failures: r.stats.dense_grid_build_failures,
        max_window_area_cells: r.stats.max_window_area_cells,
        primitive_footprint_checks: r.stats.primitive_footprint_checks,
        primitive_footprint_cells_tested: r.stats.primitive_footprint_cells_tested,
        primitive_footprint_rect_checks: r.stats.primitive_footprint_rect_checks,
        primitive_footprint_rect_rejects: r.stats.primitive_footprint_rect_rejects,
        dense_grid_cells: r.stats.dense_grid_cells,
        dense_grid_build_time_us: {
            let clamped = r.stats.dense_grid_build_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        search_loop_time_us: {
            let clamped = r.stats.search_loop_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        obstacle_map_prepare_time_us: {
            let clamped = r.stats.obstacle_map_prepare_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        simple_route_time_us: {
            let clamped = r.stats.simple_route_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        commit_prepare_time_us: {
            let clamped = r.stats.commit_prepare_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        commit_time_us: {
            let clamped = r.stats.commit_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        neighbor_generation_time_us: {
            let clamped = r.stats.neighbor_generation_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        heap_operation_time_us: {
            let clamped = r.stats.heap_operation_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        legality_check_time_us: {
            let clamped = r.stats.legality_check_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        reconstruction_time_us: {
            let clamped = r.stats.reconstruction_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        jps4_requested: r.stats.jps4_requested,
        jps4_eligible: r.stats.jps4_eligible,
        jps4_used: r.stats.jps4_used,
        jps4_fallbacks: r.stats.jps4_fallbacks,
        jps4_fallback_reason: r.stats.jps4_fallback_reason.clone(),
    })
}

fn vec_to_primitive_counter_array(values: &[usize]) -> [usize; 4] {
    let mut counters = [0usize; 4];
    for (idx, value) in values.iter().copied().take(4).enumerate() {
        counters[idx] = value;
    }
    counters
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
            last_window_min_x: route.last_window_min_x,
            last_window_max_x: route.last_window_max_x,
            last_window_min_y: route.last_window_min_y,
            last_window_max_y: route.last_window_max_y,
            last_window_area_cells: route.last_window_area_cells,
            expanded_states: route.expanded_states,
            generated_neighbors: route.generated_neighbors,
            heap_pushes: route.heap_pushes,
            heap_pops: route.heap_pops,
            skipped_duplicate_heap_entries: route.skipped_duplicate_heap_entries,
            stale_generation_heap_entries: route.stale_generation_heap_entries,
            closed_heap_entries: route.closed_heap_entries,
            max_heap_size: route.max_heap_size,
            dense_search_states: route.dense_search_states,
            dense_search_storage_bytes: route.dense_search_storage_bytes,
            best_cost_updates: route.best_cost_updates,
            parent_updates: route.parent_updates,
            obstacle_clearance_checks: route.obstacle_clearance_checks,
            window_rejects: route.window_rejects,
            footprint_rejects: route.footprint_rejects,
            primitive_generated_by_class: vec_to_primitive_counter_array(
                &route.primitive_generated_by_class,
            ),
            primitive_bounds_rejects_by_class: vec_to_primitive_counter_array(
                &route.primitive_bounds_rejects_by_class,
            ),
            primitive_closed_rejects_by_class: vec_to_primitive_counter_array(
                &route.primitive_closed_rejects_by_class,
            ),
            primitive_cost_pruned_by_class: vec_to_primitive_counter_array(
                &route.primitive_cost_pruned_by_class,
            ),
            primitive_footprint_checks_by_class: vec_to_primitive_counter_array(
                &route.primitive_footprint_checks_by_class,
            ),
            primitive_footprint_rejects_by_class: vec_to_primitive_counter_array(
                &route.primitive_footprint_rejects_by_class,
            ),
            primitive_accepted_by_class: vec_to_primitive_counter_array(
                &route.primitive_accepted_by_class,
            ),
            dense_grid_build_failures: route.dense_grid_build_failures,
            max_window_area_cells: route.max_window_area_cells,
            primitive_footprint_checks: route.primitive_footprint_checks,
            primitive_footprint_cells_tested: route.primitive_footprint_cells_tested,
            primitive_footprint_rect_checks: route.primitive_footprint_rect_checks,
            primitive_footprint_rect_rejects: route.primitive_footprint_rect_rejects,
            dense_grid_cells: route.dense_grid_cells,
            dense_grid_build_time_us: u128::from(route.dense_grid_build_time_us),
            search_loop_time_us: u128::from(route.search_loop_time_us),
            obstacle_map_prepare_time_us: u128::from(route.obstacle_map_prepare_time_us),
            simple_route_time_us: u128::from(route.simple_route_time_us),
            commit_prepare_time_us: u128::from(route.commit_prepare_time_us),
            commit_time_us: u128::from(route.commit_time_us),
            neighbor_generation_time_us: u128::from(route.neighbor_generation_time_us),
            heap_operation_time_us: u128::from(route.heap_operation_time_us),
            legality_check_time_us: u128::from(route.legality_check_time_us),
            reconstruction_time_us: u128::from(route.reconstruction_time_us),
            jps4_requested: route.jps4_requested,
            jps4_eligible: route.jps4_eligible,
            jps4_used: route.jps4_used,
            jps4_fallbacks: route.jps4_fallbacks,
            jps4_fallback_reason: route.jps4_fallback_reason.clone(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
            ),
        );
        assert!(!router.primitives.get_primitives_for_angle(0).is_empty());
    }

    #[test]
    fn route_commit_cells_exempts_clearance_but_keeps_core() {
        let center_cells = vec![(4, 4), (5, 4)];
        let exempt_cells = vec![(4, 3), (4, 4), (4, 5)];
        let committed = route_commit_cells(&center_cells, 0, 1, Some(&exempt_cells), 10, 10);
        let committed_keys: FxHashSet<CellKey> =
            committed.iter().map(|(x, y)| pack_xy(*x, *y)).collect();

        assert!(committed_keys.contains(&pack_xy(4, 4)));
        assert!(committed_keys.contains(&pack_xy(5, 4)));
        assert!(!committed_keys.contains(&pack_xy(4, 3)));
        assert!(!committed_keys.contains(&pack_xy(4, 5)));
        assert!(committed_keys.contains(&pack_xy(5, 3)));
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                "fill_box_multi_bump",
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                "fill_box_multi_bump",
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
                "fill_box_multi_bump",
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                    "fill_box_multi_bump",
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
    fn checked_analytic_meander_box_method_fails_on_blocked_box() {
        pyo3::prepare_freethreaded_python();
        let grid = PyGridSpec::new(20, 20, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, 1.0, true),
            PyAStarConfig::new(
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                "fill_box_multi_bump",
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
            let none_bumps: usize = none_d
                .get_item("bumps")
                .unwrap()
                .unwrap()
                .extract()
                .unwrap();
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
            last_window_min_x: 0,
            last_window_max_x: 0,
            last_window_min_y: 0,
            last_window_max_y: 0,
            last_window_area_cells: 0,
            expanded_states: 0,
            generated_neighbors: 0,
            heap_pushes: 0,
            heap_pops: 0,
            skipped_duplicate_heap_entries: 0,
            stale_generation_heap_entries: 0,
            closed_heap_entries: 0,
            max_heap_size: 0,
            dense_search_states: 0,
            dense_search_storage_bytes: 0,
            best_cost_updates: 0,
            parent_updates: 0,
            obstacle_clearance_checks: 0,
            window_rejects: 0,
            footprint_rejects: 0,
            primitive_generated_by_class: vec![0; 4],
            primitive_bounds_rejects_by_class: vec![0; 4],
            primitive_closed_rejects_by_class: vec![0; 4],
            primitive_cost_pruned_by_class: vec![0; 4],
            primitive_footprint_checks_by_class: vec![0; 4],
            primitive_footprint_rejects_by_class: vec![0; 4],
            primitive_accepted_by_class: vec![0; 4],
            dense_grid_build_failures: 0,
            max_window_area_cells: 0,
            primitive_footprint_checks: 0,
            primitive_footprint_cells_tested: 0,
            primitive_footprint_rect_checks: 0,
            primitive_footprint_rect_rejects: 0,
            dense_grid_cells: 0,
            dense_grid_build_time_us: 0,
            search_loop_time_us: 0,
            obstacle_map_prepare_time_us: 0,
            simple_route_time_us: 0,
            commit_prepare_time_us: 0,
            commit_time_us: 0,
            neighbor_generation_time_us: 0,
            heap_operation_time_us: 0,
            legality_check_time_us: 0,
            reconstruction_time_us: 0,
            jps4_requested: false,
            jps4_eligible: false,
            jps4_used: false,
            jps4_fallbacks: 0,
            jps4_fallback_reason: String::new(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
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
