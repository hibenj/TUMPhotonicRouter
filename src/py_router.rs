use std::cell::RefCell;
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use rustc_hash::{FxHashMap, FxHashSet};

use crate::astar::{
    export_route_svg_with_port_open_cells, route_single_net_with_collision_crossing_config,
    route_single_net_with_config, route_single_net_with_crossing_config,
    route_single_net_with_dynamic_expansion_config, try_simple_route_with_dynamic_expansion_config,
    AStarConfig, CrossingSearchConfig, CrossingSearchPartner, HeapTieBreaker, HeuristicMode,
    PrimitiveOrdering, RouteResult, RouteSearchStats, State,
};
use crate::crossings::{CrossingConfig, CrossingConstraint, CrossingContext};
use crate::geometry_realization::{
    build_port_access as build_port_access_rs, build_port_accesses as build_port_accesses_rs,
    cells_in_grid_rect as cells_in_grid_rect_rs, centerline_length_um as centerline_length_um_rs,
    check_meander_box_free_with_prefix as check_meander_box_free_with_prefix_rs,
    compress_grid_waypoints as compress_grid_waypoints_rs,
    full_straight_offset_bump_candidates as full_straight_offset_bump_candidates_rs,
    generate_waveguide_polygon as generate_waveguide_polygon_rs,
    grid_path_to_centerline as grid_path_to_centerline_rs,
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
    route_to_grid_path as route_to_grid_path_rs,
    route_to_port_corrected_centerline_with_options as route_to_port_corrected_centerline_with_options_rs,
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
    plan_registered_geometry_final_requests, plan_registered_geometry_request_sequence,
    plan_registered_geometry_requirement_candidates, plan_registered_geometry_split_request,
    MeanderPlanningProfileTotals, MeanderWrapperProfileTotals, RegisteredMeanderGeometry,
    RegisteredPlmContext, RegisteredRequirementResult,
};
use crate::primitives::{
    create_grid4_unit_grid_primitive_library, create_jps4_unit_grid_primitive_library,
    create_photonic_primitive_library, Primitive, PrimitiveGeometry, PrimitiveLibrary,
    PrimitiveLibraryConfig, DIRECTIONS,
};
use crate::static_obstacle_builder::{
    physical_to_grid, rasterize_polygon, PortInput, PyStaticCellSet, StaticGridSpec,
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
    pub proactive_congestion_weight: f64,
    #[pyo3(get, set)]
    pub proactive_congestion_radius_cells: i32,
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
    pub heuristic_weight: f64,
    #[pyo3(get, set)]
    pub heap_tie_breaker: String,
}
#[pymethods]
impl PyAStarConfig {
    #[new]
    #[pyo3(signature=(max_iterations=100_000,bend_weight=1.0,target_tolerance_cells=0,require_target_angle=true,allowed_target_angles=None,use_routing_window=true,routing_window_min_margin_cells=12,routing_window_scale=0.35,routing_window_max_expansions=3,routing_window_fallback_full_grid=true,routing_window_growth=0.5,max_dense_obstacle_cells=10_000_000,ignore_dynamic_obstacles=false,history_weight=0.0,proactive_congestion_weight=0.0,proactive_congestion_radius_cells=0,collect_detailed_timing=false,use_indexed_heap=false,primitive_ordering="library".to_string(),heuristic_mode="heading_aware".to_string(),heuristic_weight=1.0))]
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
        proactive_congestion_weight: f64,
        proactive_congestion_radius_cells: i32,
        collect_detailed_timing: bool,
        use_indexed_heap: bool,
        primitive_ordering: String,
        heuristic_mode: String,
        heuristic_weight: f64,
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
            proactive_congestion_weight,
            proactive_congestion_radius_cells,
            collect_detailed_timing,
            enable_jps4: false,
            use_indexed_heap,
            primitive_ordering,
            heuristic_mode,
            heuristic_weight,
            heap_tie_breaker: "smaller_g".to_string(),
        }
    }
}

#[pyclass(name = "CrossingConfig")]
#[derive(Clone)]
pub struct PyCrossingConfig {
    #[pyo3(get, set)]
    pub enabled: bool,
    #[pyo3(get, set)]
    pub crossing_loss: f64,
    #[pyo3(get, set)]
    pub crossing_half_size_cells: i32,
    #[pyo3(get, set)]
    pub min_straight_cells_per_crossing: i32,
    #[pyo3(get, set)]
    pub allow_only_expected_pairs: bool,
}

impl From<&PyCrossingConfig> for CrossingConfig {
    fn from(value: &PyCrossingConfig) -> Self {
        Self {
            enabled: value.enabled,
            crossing_loss: value.crossing_loss,
            crossing_half_size_cells: value.crossing_half_size_cells,
            min_straight_cells_per_crossing: value.min_straight_cells_per_crossing,
            allow_only_expected_pairs: value.allow_only_expected_pairs,
        }
    }
}

impl From<&CrossingConfig> for PyCrossingConfig {
    fn from(value: &CrossingConfig) -> Self {
        Self {
            enabled: value.enabled,
            crossing_loss: value.crossing_loss,
            crossing_half_size_cells: value.crossing_half_size_cells,
            min_straight_cells_per_crossing: value.min_straight_cells_per_crossing,
            allow_only_expected_pairs: value.allow_only_expected_pairs,
        }
    }
}

fn validate_crossing_config(config: &PyCrossingConfig) -> PyResult<()> {
    if !config.crossing_loss.is_finite() || config.crossing_loss < 0.0 {
        return Err(PyValueError::new_err(
            "crossing_loss must be finite and non-negative",
        ));
    }
    if config.crossing_half_size_cells < 0 {
        return Err(PyValueError::new_err(
            "crossing_half_size_cells must be non-negative",
        ));
    }
    if config.min_straight_cells_per_crossing < 0 {
        return Err(PyValueError::new_err(
            "min_straight_cells_per_crossing must be non-negative",
        ));
    }
    Ok(())
}

#[pymethods]
impl PyCrossingConfig {
    #[new]
    #[pyo3(signature=(enabled=false,crossing_loss=0.0,crossing_half_size_cells=0,min_straight_cells_per_crossing=0,allow_only_expected_pairs=true))]
    fn new(
        enabled: bool,
        crossing_loss: f64,
        crossing_half_size_cells: i32,
        min_straight_cells_per_crossing: i32,
        allow_only_expected_pairs: bool,
    ) -> PyResult<Self> {
        let config = Self {
            enabled,
            crossing_loss,
            crossing_half_size_cells,
            min_straight_cells_per_crossing,
            allow_only_expected_pairs,
        };
        validate_crossing_config(&config)?;
        Ok(config)
    }
}

#[pyclass(name = "CrossingConstraint")]
#[derive(Clone)]
pub struct PyCrossingConstraint {
    #[pyo3(get, set)]
    pub net_id: u64,
    #[pyo3(get, set)]
    pub partner_net_id: u64,
    #[pyo3(get, set)]
    pub level: u32,
    #[pyo3(get, set)]
    pub source_depth: u32,
    #[pyo3(get, set)]
    pub target_depth: u32,
}

impl From<&PyCrossingConstraint> for CrossingConstraint {
    fn from(value: &PyCrossingConstraint) -> Self {
        Self {
            net_id: value.net_id,
            partner_net_id: value.partner_net_id,
            level: value.level,
            source_depth: value.source_depth,
            target_depth: value.target_depth,
        }
    }
}

impl From<&CrossingConstraint> for PyCrossingConstraint {
    fn from(value: &CrossingConstraint) -> Self {
        Self {
            net_id: value.net_id,
            partner_net_id: value.partner_net_id,
            level: value.level,
            source_depth: value.source_depth,
            target_depth: value.target_depth,
        }
    }
}

fn validate_crossing_constraint(constraint: &PyCrossingConstraint) -> PyResult<()> {
    if constraint.net_id == constraint.partner_net_id {
        return Err(PyValueError::new_err(
            "crossing constraint requires two different net ids",
        ));
    }
    Ok(())
}

#[pymethods]
impl PyCrossingConstraint {
    #[new]
    #[pyo3(signature=(net_id,partner_net_id,level=0,source_depth=0,target_depth=0))]
    fn new(
        net_id: u64,
        partner_net_id: u64,
        level: u32,
        source_depth: u32,
        target_depth: u32,
    ) -> PyResult<Self> {
        let constraint = Self {
            net_id,
            partner_net_id,
            level,
            source_depth,
            target_depth,
        };
        validate_crossing_constraint(&constraint)?;
        Ok(constraint)
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
    pub crossing_candidate_checks: usize,
    #[pyo3(get)]
    pub crossing_accepted: usize,
    #[pyo3(get)]
    pub crossing_reject_non_straight: usize,
    #[pyo3(get)]
    pub crossing_reject_not_perpendicular: usize,
    #[pyo3(get)]
    pub crossing_reject_margin: usize,
    #[pyo3(get)]
    pub crossing_reject_wrong_order: usize,
    #[pyo3(get)]
    pub crossing_reject_unexpected_owner: usize,
    #[pyo3(get)]
    pub crossing_reject_unmatched_owner: usize,
    #[pyo3(get)]
    pub crossing_reject_unmatched_centerline: usize,
    #[pyo3(get)]
    pub crossing_reject_unmatched_footprint: usize,
    #[pyo3(get)]
    pub crossing_reject_unmatched_route_centerline: usize,
    #[pyo3(get)]
    pub crossing_reject_unmatched_route_footprint: usize,
    #[pyo3(get)]
    pub crossing_reject_pending_straight: usize,
    #[pyo3(get)]
    pub dense_grid_cells: usize,
    #[pyo3(get)]
    pub route_search_total_time_us: u64,
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
    astar_cfg_cached: Result<AStarConfig, String>,
    obstacle_map: ObstacleMap,
    primitives: PrimitiveLibrary,
    crossing_context: CrossingContext,
    committed_center_routes: FxHashMap<u64, Vec<(i32, i32)>>,
    committed_realized_center_routes: FxHashMap<u64, Vec<(f64, f64)>>,
    committed_opened_cell_keys: FxHashMap<u64, FxHashSet<CellKey>>,
    crossing_events: Vec<CrossingEvent>,
    use_collision_crossing_routing: bool,
    static_cells: FxHashSet<CellKey>,
    port_open_cells: FxHashSet<CellKey>,
    registered_plm: RefCell<RegisteredPlmContext>,
    last_meander_registration_profile: RefCell<Option<MeanderRegistrationProfile>>,
}

#[derive(Clone)]
struct NativeRouteJob {
    net_id: u64,
    source: PyState,
    target: PyState,
    opened_cells: Vec<(i32, i32)>,
    opened_cell_keys: FxHashSet<CellKey>,
    clearance_exempt_cells: Vec<(i32, i32)>,
    clearance_exempt_cell_keys: FxHashSet<CellKey>,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
}

impl NativeRouteJob {
    #[allow(clippy::too_many_arguments)]
    fn new(
        net_id: u64,
        source: PyState,
        target: PyState,
        opened_cells: Vec<(i32, i32)>,
        clearance_exempt_cells: Vec<(i32, i32)>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Self {
        let opened_cell_keys = pack_cells(&opened_cells);
        let clearance_exempt_cell_keys = pack_cells(&clearance_exempt_cells);
        Self {
            net_id,
            source,
            target,
            opened_cells,
            opened_cell_keys,
            clearance_exempt_cells,
            clearance_exempt_cell_keys,
            source_port_um,
            target_port_um,
        }
    }
}

#[derive(Clone)]
struct NativeRouteAttempt {
    bucket_name: &'static str,
    net_id: u64,
    route: Option<RouteResult>,
    failed: bool,
    error: Option<String>,
    repair_round: Option<u32>,
    candidate_blockers: Vec<u64>,
    ripup_ids: Vec<u64>,
}

#[derive(Clone)]
struct NativeRepairTraceEvent {
    event_name: &'static str,
    route_order: Option<&'static str>,
    action: Option<&'static str>,
    net_id: u64,
    repair_round: Option<u32>,
    repair_set_index: Option<u64>,
    candidate_blockers: Vec<u64>,
    ripup_ids: Vec<u64>,
    victim_order: Vec<u64>,
    victim_first: Option<bool>,
    reverse_victim_order: Option<bool>,
    success: Option<bool>,
    error: Option<String>,
}

#[allow(clippy::too_many_arguments)]
fn push_native_repair_trace(
    repair_trace: &mut Vec<NativeRepairTraceEvent>,
    event_name: &'static str,
    route_order: Option<&'static str>,
    action: Option<&'static str>,
    net_id: u64,
    repair_round: Option<u32>,
    repair_set_index: Option<u64>,
    candidate_blockers: &[u64],
    ripup_ids: &[u64],
    victim_order: &[u64],
    victim_first: Option<bool>,
    reverse_victim_order: Option<bool>,
    success: Option<bool>,
    error: Option<String>,
) {
    repair_trace.push(NativeRepairTraceEvent {
        event_name,
        route_order,
        action,
        net_id,
        repair_round,
        repair_set_index,
        candidate_blockers: candidate_blockers.to_vec(),
        ripup_ids: ripup_ids.to_vec(),
        victim_order: victim_order.to_vec(),
        victim_first,
        reverse_victim_order,
        success,
        error,
    });
}

#[derive(Clone, Debug)]
struct CrossingEvent {
    net_id: u64,
    partner_net_id: u64,
    point: (f64, f64),
    route_segment: ((i32, i32), (i32, i32)),
    partner_segment: ((i32, i32), (i32, i32)),
    route_angle: u8,
    partner_angle: u8,
    reservation_keys: FxHashSet<CellKey>,
}

#[derive(Clone, Debug)]
struct InvalidCrossingIntersection {
    net_id: u64,
    partner_net_id: u64,
    point: (f64, f64),
    reason: &'static str,
}

#[derive(Default)]
struct CrossingReservationBlockers {
    has_static_blocker: bool,
    dynamic_blockers: FxHashSet<u64>,
}

const CROSSING_SPACING_HISTORY_AMOUNT: u32 = 1;
const CROSSING_LOCAL_RIPUP_MIN_SOURCE_DEPTH: u32 = 2;
const CROSSING_LOCAL_RIPUP_MIN_OVERLAP_CELLS: u32 = 64;
const CROSSING_LOCAL_RIPUP_MIN_EXTRA_SUM: u64 = 64;

impl CrossingReservationBlockers {
    fn is_clear(&self) -> bool {
        !self.has_static_blocker && self.dynamic_blockers.is_empty()
    }
}

#[derive(Default)]
struct NativeBatchTimings {
    route_job_unpack_us: u128,
    obstacle_map_prepare_us: u128,
    route_search_total_us: u128,
    simple_route_candidate_us: u128,
    dense_astar_us: u128,
    commit_cell_build_us: u128,
    commit_update_dynamic_map_us: u128,
    normal_route_wall_us: u128,
    probe_route_wall_us: u128,
    repair_failed_net_wall_us: u128,
    reroute_victims_wall_us: u128,
    normal_route_failed_wall_us: u128,
    probe_route_failed_wall_us: u128,
    repair_failed_net_failed_wall_us: u128,
    reroute_victims_failed_wall_us: u128,
    repair_probe_victim_selection_us: u128,
    repair_state_reset_us: u128,
    ripup_us: u128,
    history_update_us: u128,
    route_result_construction_us: u128,
    python_return_dict_us: u128,
}

impl NativeBatchTimings {
    fn add_route_result_stats(&mut self, route: &RouteResult) {
        self.obstacle_map_prepare_us += route.stats.obstacle_map_prepare_time_us;
        self.route_search_total_us += route.stats.route_search_total_time_us;
        self.simple_route_candidate_us += route.stats.simple_route_time_us;
        self.dense_astar_us += route.stats.search_loop_time_us;
        self.commit_cell_build_us += route.stats.commit_prepare_time_us;
        self.commit_update_dynamic_map_us += route.stats.commit_time_us;
    }

    fn add_route_result_stats_if(&mut self, enabled: bool, route: &RouteResult) {
        if enabled {
            self.add_route_result_stats(route);
        }
    }
}

struct NativeEndpointCorrection {
    centerline: Vec<(f64, f64)>,
    committed_bump: bool,
    candidate_index: Option<usize>,
    candidate_label: Option<String>,
}

fn pack_cells(cells: &[(i32, i32)]) -> FxHashSet<CellKey> {
    cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect()
}

fn opened_cells_excluding_keepout(
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

fn route_orientation_to_angle(orientation: Option<f64>) -> u8 {
    let value = orientation.unwrap_or(0.0).rem_euclid(360.0);
    (value / 45.0).round().rem_euclid(8.0) as u8
}

fn route_angle_to_step(angle: u8) -> (i32, i32) {
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

fn route_in_bounds(x: i32, y: i32, grid: &StaticGridSpec) -> bool {
    x >= 0 && x < grid.width && y >= 0 && y < grid.height
}

fn route_collect_inflated_step_cells(
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

fn route_port_state_cell(
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

fn route_port_access_cells(
    grid: &StaticGridSpec,
    x_um: f64,
    y_um: f64,
    orientation: Option<f64>,
    access_length_um: Option<f64>,
    access_width_um: Option<f64>,
    port_entry_length_cells: i32,
    port_entry_half_width_cells: i32,
    port_lane_length_cells: i32,
    port_lane_half_width_cells: i32,
) -> FxHashSet<CellKey> {
    let angle = route_orientation_to_angle(orientation);
    let (sx, sy) = route_angle_to_step(angle);
    let (base_x, base_y) = route_port_state_cell(grid, x_um, y_um, orientation);
    let mut cells = if access_length_um.is_some() || access_width_um.is_some() {
        let length_cells = ((access_length_um.unwrap_or(0.0).max(0.0)) / grid.grid_size_um)
            .ceil()
            .max(1.0) as i32;
        let half_width_cells = (((access_width_um.unwrap_or(0.0).max(0.0)) / 2.0)
            / grid.grid_size_um)
            .ceil()
            .max(0.0) as i32;
        route_collect_inflated_step_cells(
            grid,
            base_x,
            base_y,
            sx,
            sy,
            length_cells,
            half_width_cells,
        )
    } else {
        let mut entry_zone = route_collect_inflated_step_cells(
            grid,
            base_x,
            base_y,
            sx,
            sy,
            port_entry_length_cells,
            port_entry_half_width_cells,
        );
        let lane_zone = route_collect_inflated_step_cells(
            grid,
            base_x,
            base_y,
            sx,
            sy,
            port_lane_length_cells,
            port_lane_half_width_cells,
        );
        entry_zone.extend(lane_zone);
        entry_zone
    };
    if route_in_bounds(base_x, base_y, grid) {
        cells.insert(pack_xy(base_x, base_y));
    }
    cells
}

fn route_base_port_open_cells(
    grid: &StaticGridSpec,
    x_um: f64,
    y_um: f64,
    port_open_radius_cells: i32,
) -> FxHashSet<CellKey> {
    let (base_x, base_y) = physical_to_grid(x_um, y_um, grid);
    route_collect_inflated_step_cells(grid, base_x, base_y, 0, 0, 1, port_open_radius_cells)
}

fn route_port_runway_cells(
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

fn route_cell_in_raw_static(
    key: CellKey,
    raw_static_keys: &FxHashSet<CellKey>,
    raw_static_rects: &[(i32, i32, i32, i32)],
) -> bool {
    if raw_static_keys.contains(&key) {
        return true;
    }
    let (x, y) = unpack_xy(key);
    raw_static_rects
        .iter()
        .any(|(x0, y0, x1, y1)| x >= *x0 && x <= *x1 && y >= *y0 && y <= *y1)
}

fn route_dynamic_clearance_exempt_cells(
    source: PyState,
    target: PyState,
    allow_45_degree_turns: bool,
    bend_radius_cells: i32,
    commit_radius_cells: i32,
    width: i32,
    height: i32,
) -> FxHashSet<CellKey> {
    let _ = (allow_45_degree_turns, bend_radius_cells);
    let mut cells = FxHashSet::default();
    let radius = commit_radius_cells.clamp(0, 1);
    for anchor in [source, target] {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                let x = anchor.x + dx;
                let y = anchor.y + dy;
                if x >= 0 && x < width && y >= 0 && y < height {
                    cells.insert(pack_xy(x, y));
                }
            }
        }
    }
    cells
}

fn sorted_cells(cells: FxHashSet<CellKey>) -> Vec<(i32, i32)> {
    let mut out: Vec<(i32, i32)> = cells.into_iter().map(unpack_xy).collect();
    out.sort_unstable();
    out
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

fn endpoint_contact_open_keys(
    corrected_core_cells: &[(i32, i32)],
    grid: &GeometryGridSpec,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
    width_um: f64,
) -> FxHashSet<CellKey> {
    let mut open_keys = FxHashSet::default();
    let radius_um = (grid.grid_size_um * 0.75).max(width_um.max(0.0) + grid.grid_size_um * 0.25);
    let radius_sq = radius_um * radius_um;
    for port in [source_port_um, target_port_um].into_iter().flatten() {
        if !port.0.is_finite() || !port.1.is_finite() {
            continue;
        }
        for &(x, y) in corrected_core_cells {
            let center = grid.cell_center(x, y);
            let dx = center.0 - port.0;
            let dy = center.1 - port.1;
            if dx * dx + dy * dy <= radius_sq {
                open_keys.insert(pack_xy(x, y));
            }
        }
    }
    open_keys
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

fn direction_angle_between_cells(a: (i32, i32), b: (i32, i32)) -> Option<u8> {
    let step = ((b.0 - a.0).signum(), (b.1 - a.1).signum());
    DIRECTIONS
        .iter()
        .position(|dir| *dir == step)
        .map(|idx| idx as u8)
}

fn axes_are_perpendicular(angle_a: u8, angle_b: u8) -> bool {
    (i16::from(angle_a % 4) - i16::from(angle_b % 4)).rem_euclid(4) == 2
}

fn segment_length_cells(a: (i32, i32), b: (i32, i32)) -> f64 {
    f64::from((a.0 - b.0).abs().max((a.1 - b.1).abs()))
}

fn segment_intersection_with_params(
    a0: (i32, i32),
    a1: (i32, i32),
    b0: (i32, i32),
    b1: (i32, i32),
) -> Option<(f64, f64, f64, f64)> {
    let ax = f64::from(a1.0 - a0.0);
    let ay = f64::from(a1.1 - a0.1);
    let bx = f64::from(b1.0 - b0.0);
    let by = f64::from(b1.1 - b0.1);
    let denom = ax * by - ay * bx;
    if denom.abs() < 1e-9 {
        return None;
    }

    let cx = f64::from(b0.0 - a0.0);
    let cy = f64::from(b0.1 - a0.1);
    let t = (cx * by - cy * bx) / denom;
    let u = (cx * ay - cy * ax) / denom;
    let eps = 1e-9;
    if (-eps..=(1.0 + eps)).contains(&t) && (-eps..=(1.0 + eps)).contains(&u) {
        Some((f64::from(a0.0) + t * ax, f64::from(a0.1) + t * ay, t, u))
    } else {
        None
    }
}

fn physical_segment_length(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    (dx * dx + dy * dy).sqrt()
}

fn physical_point_near_centerline_endpoint(
    point: (f64, f64),
    centerline: &[(f64, f64)],
    tolerance_um: f64,
) -> bool {
    let Some(first) = centerline.first() else {
        return false;
    };
    if physical_segment_length(point, *first) <= tolerance_um {
        return true;
    }
    centerline
        .last()
        .map(|last| physical_segment_length(point, *last) <= tolerance_um)
        .unwrap_or(false)
}

fn physical_segments_are_perpendicular(
    a0: (f64, f64),
    a1: (f64, f64),
    b0: (f64, f64),
    b1: (f64, f64),
) -> bool {
    let ax = a1.0 - a0.0;
    let ay = a1.1 - a0.1;
    let bx = b1.0 - b0.0;
    let by = b1.1 - b0.1;
    (ax * bx + ay * by).abs() < 1e-6
}

fn physical_segment_intersection_with_params(
    a0: (f64, f64),
    a1: (f64, f64),
    b0: (f64, f64),
    b1: (f64, f64),
) -> Option<(f64, f64, f64, f64)> {
    let ax = a1.0 - a0.0;
    let ay = a1.1 - a0.1;
    let bx = b1.0 - b0.0;
    let by = b1.1 - b0.1;
    let denom = ax * by - ay * bx;
    if denom.abs() < 1e-9 {
        return None;
    }

    let cx = b0.0 - a0.0;
    let cy = b0.1 - a0.1;
    let t = (cx * by - cy * bx) / denom;
    let u = (cx * ay - cy * ax) / denom;
    let eps = 1e-9;
    if (-eps..=(1.0 + eps)).contains(&t) && (-eps..=(1.0 + eps)).contains(&u) {
        Some((a0.0 + t * ax, a0.1 + t * ay, t, u))
    } else {
        None
    }
}

fn physical_collinear_segment_overlap_midpoint(
    a0: (f64, f64),
    a1: (f64, f64),
    b0: (f64, f64),
    b1: (f64, f64),
) -> Option<(f64, f64)> {
    let ax = a1.0 - a0.0;
    let ay = a1.1 - a0.1;
    let bx = b1.0 - b0.0;
    let by = b1.1 - b0.1;
    let eps = 1e-9;
    if ax.abs() < eps && ay.abs() < eps {
        return None;
    }
    if bx.abs() < eps && by.abs() < eps {
        return None;
    }
    if (ax * by - ay * bx).abs() >= eps {
        return None;
    }
    let offset_x = b0.0 - a0.0;
    let offset_y = b0.1 - a0.1;
    if (offset_x * ay - offset_y * ax).abs() >= eps {
        return None;
    }

    let use_x_axis = ax.abs() >= ay.abs();
    let a_denom = if use_x_axis { ax } else { ay };
    let b0_coord = if use_x_axis { b0.0 } else { b0.1 };
    let b1_coord = if use_x_axis { b1.0 } else { b1.1 };
    let a0_coord = if use_x_axis { a0.0 } else { a0.1 };
    if a_denom.abs() < eps {
        return None;
    }

    let b_t0 = (b0_coord - a0_coord) / a_denom;
    let b_t1 = (b1_coord - a0_coord) / a_denom;
    let t_start = 0.0_f64.max(b_t0.min(b_t1));
    let t_end = 1.0_f64.min(b_t0.max(b_t1));
    if t_end - t_start <= eps {
        return None;
    }

    let midpoint_t = 0.5 * (t_start + t_end);
    Some((a0.0 + midpoint_t * ax, a0.1 + midpoint_t * ay))
}

fn physical_points_are_collinear(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    let ab = (b.0 - a.0, b.1 - a.1);
    let bc = (c.0 - b.0, c.1 - b.1);
    let cross = ab.0 * bc.1 - ab.1 * bc.0;
    let dot = ab.0 * bc.0 + ab.1 * bc.1;
    cross.abs() < 1e-9 && dot >= -1e-9
}

fn compress_physical_centerline(points: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    if points.len() < 3 {
        return points;
    }
    let mut out = Vec::with_capacity(points.len());
    for point in points {
        if out.len() >= 2 {
            let prev = out[out.len() - 1];
            let prev_prev = out[out.len() - 2];
            if physical_points_are_collinear(prev_prev, prev, point) {
                out.pop();
            }
        }
        if out.last().copied() != Some(point) {
            out.push(point);
        }
    }
    out
}

fn illegal_crossing_net_ids_from_error(error: &str) -> Vec<u64> {
    const PREFIX: &str = "Illegal realized crossing: net ";
    let Some(rest) = error.strip_prefix(PREFIX) else {
        return Vec::new();
    };
    let Some((first, rest)) = rest.split_once(" intersects net ") else {
        return Vec::new();
    };
    let first_id = first.trim().parse::<u64>().ok();
    let second_id = rest
        .split_whitespace()
        .next()
        .and_then(|value| value.trim().parse::<u64>().ok());
    first_id.into_iter().chain(second_id).collect()
}

fn dynamic_commit_error_overlap_owner_ids(error: &str) -> Vec<u64> {
    let Some((_, rest)) = error.split_once("dynamic_overlap_owners=[") else {
        return Vec::new();
    };
    let Some((owner_text, _)) = rest.split_once(']') else {
        return Vec::new();
    };
    owner_text
        .split(',')
        .filter_map(|value| value.trim().parse::<u64>().ok())
        .collect()
}

fn enqueue_targeted_illegal_crossing_repair_set(
    repair_victim_sets: &mut Vec<(u32, Vec<u64>)>,
    candidate_blockers: &mut Vec<u64>,
    final_routes: &FxHashMap<u64, RouteResult>,
    current_net_id: u64,
    ripup_ids: &[u64],
    error: &str,
    round_idx: u32,
    max_rounds: u32,
    max_victims: usize,
) {
    const MAX_ADAPTIVE_REPAIR_SETS: usize = 8;
    let mut learned_ids = Vec::new();
    for extra_id in illegal_crossing_net_ids_from_error(error) {
        if extra_id == current_net_id
            || ripup_ids.contains(&extra_id)
            || !final_routes.contains_key(&extra_id)
        {
            continue;
        }
        if !candidate_blockers.contains(&extra_id) {
            candidate_blockers.push(extra_id);
        }
        if !learned_ids.contains(&extra_id) {
            learned_ids.push(extra_id);
        }
    }
    if learned_ids.is_empty() {
        return;
    }
    if round_idx >= max_rounds
        || ripup_ids.len() >= max_victims
        || repair_victim_sets.len() >= MAX_ADAPTIVE_REPAIR_SETS
    {
        return;
    }

    let mut next = ripup_ids.to_vec();
    for extra_id in learned_ids {
        if next.contains(&extra_id) {
            continue;
        }
        next.push(extra_id);
        if next.len() > max_victims {
            return;
        }
    }
    if repair_victim_sets
        .iter()
        .any(|(_, existing)| existing == &next)
    {
        return;
    }
    repair_victim_sets.push((round_idx.saturating_add(1), next));
}

fn enqueue_learned_keepout_repair_retry(
    repair_victim_sets: &mut Vec<(u32, Vec<u64>)>,
    retry_counts: &mut FxHashMap<Vec<u64>, usize>,
    ripup_ids: &[u64],
    round_idx: u32,
    next_repair_set_index: usize,
) {
    const MAX_LEARNED_KEEP_OUT_REPAIR_SETS: usize = 12;
    const MAX_LEARNED_KEEP_OUT_RETRIES_PER_SET: usize = 1;
    if ripup_ids.is_empty() || repair_victim_sets.len() >= MAX_LEARNED_KEEP_OUT_REPAIR_SETS {
        return;
    }
    let retry_key = ripup_ids.to_vec();
    if retry_counts.get(&retry_key).copied().unwrap_or(0)
        >= MAX_LEARNED_KEEP_OUT_RETRIES_PER_SET
    {
        return;
    }
    if repair_victim_sets
        .iter()
        .skip(next_repair_set_index)
        .any(|(_, existing)| existing.as_slice() == ripup_ids)
    {
        return;
    }
    *retry_counts.entry(retry_key.clone()).or_default() += 1;
    repair_victim_sets.push((round_idx, retry_key));
}

fn crossing_reservation_window_keys(
    center_x: f64,
    center_y: f64,
    half_size_cells: i32,
    width: i32,
    height: i32,
) -> FxHashSet<CellKey> {
    let mut keys = FxHashSet::default();
    if half_size_cells < 0 {
        return keys;
    }
    insert_crossing_reservation_window(
        &mut keys,
        center_x,
        center_y,
        half_size_cells,
        width,
        height,
    );
    keys
}

fn crossing_required_margin_cells(
    crossing_half_size_cells: i32,
    _min_straight_cells: i32,
    bend_runout_cells: i32,
) -> i32 {
    crossing_half_size_cells.max(0) + bend_runout_cells.max(0)
}

fn realized_crossing_margin_um(crossing_half_size_cells: i32, grid_size_um: f64) -> f64 {
    grid_size_um * f64::from(crossing_half_size_cells.max(0))
}

fn crossing_events_for_partner(
    net_id: u64,
    partner_net_id: u64,
    route_waypoints: &[(i32, i32)],
    partner_waypoints: &[(i32, i32)],
    min_straight_cells: i32,
    half_size_cells: i32,
    bend_runout_cells: i32,
    width: i32,
    height: i32,
) -> Vec<CrossingEvent> {
    if route_waypoints.len() < 2 || partner_waypoints.len() < 2 {
        return Vec::new();
    }
    let required_margin = f64::from(crossing_required_margin_cells(
        half_size_cells,
        min_straight_cells,
        bend_runout_cells,
    ));
    let mut events = Vec::new();
    let mut seen_centers = FxHashSet::default();
    for seg_a in route_waypoints.windows(2) {
        let Some(angle_a) = direction_angle_between_cells(seg_a[0], seg_a[1]) else {
            continue;
        };
        let len_a = segment_length_cells(seg_a[0], seg_a[1]);
        if len_a <= 0.0 {
            continue;
        }
        for seg_b in partner_waypoints.windows(2) {
            let Some(angle_b) = direction_angle_between_cells(seg_b[0], seg_b[1]) else {
                continue;
            };
            if !axes_are_perpendicular(angle_a, angle_b) {
                continue;
            }
            let len_b = segment_length_cells(seg_b[0], seg_b[1]);
            if len_b <= 0.0 {
                continue;
            }
            let Some((x, y, t, u)) =
                segment_intersection_with_params(seg_a[0], seg_a[1], seg_b[0], seg_b[1])
            else {
                continue;
            };
            let margin_a = (t * len_a).min((1.0 - t) * len_a);
            let margin_b = (u * len_b).min((1.0 - u) * len_b);
            if margin_a + 1e-9 < required_margin || margin_b + 1e-9 < required_margin {
                continue;
            }
            let rounded_center = (
                (x * 1_000_000.0).round() as i64,
                (y * 1_000_000.0).round() as i64,
            );
            if !seen_centers.insert(rounded_center) {
                continue;
            }
            let reservation_keys =
                crossing_reservation_window_keys(x, y, half_size_cells, width, height);
            events.push(CrossingEvent {
                net_id,
                partner_net_id,
                point: (x, y),
                route_segment: (seg_a[0], seg_a[1]),
                partner_segment: (seg_b[0], seg_b[1]),
                route_angle: angle_a,
                partner_angle: angle_b,
                reservation_keys,
            });
        }
    }
    events
}

fn crossing_candidate_keys_for_partner(
    partner_waypoints: &[(i32, i32)],
    min_straight_cells: i32,
    half_size_cells: i32,
    bend_runout_cells: i32,
    width: i32,
    height: i32,
) -> FxHashSet<CellKey> {
    let mut keys = FxHashSet::default();
    if partner_waypoints.len() < 2 {
        return keys;
    }
    let required_margin =
        crossing_required_margin_cells(half_size_cells, min_straight_cells, bend_runout_cells);
    for segment in partner_waypoints.windows(2) {
        if direction_angle_between_cells(segment[0], segment[1]).is_none() {
            continue;
        }
        let dx = (segment[1].0 - segment[0].0).signum();
        let dy = (segment[1].1 - segment[0].1).signum();
        let steps = (segment[1].0 - segment[0].0)
            .abs()
            .max((segment[1].1 - segment[0].1).abs());
        if steps <= 0 || steps < 2 * required_margin {
            continue;
        }
        for step in required_margin..=(steps - required_margin) {
            let x = segment[0].0 + dx * step;
            let y = segment[0].1 + dy * step;
            if x >= 0 && x < width && y >= 0 && y < height {
                keys.insert(pack_xy(x, y));
            }
        }
    }
    keys
}

fn crossing_spacing_history_cells_for_route(
    route_waypoints: &[(i32, i32)],
    min_straight_cells: i32,
    half_size_cells: i32,
    bend_runout_cells: i32,
    width: i32,
    height: i32,
) -> Vec<(i32, i32)> {
    let mut keys = FxHashSet::default();
    if route_waypoints.len() < 2 {
        return Vec::new();
    }
    let required_margin =
        crossing_required_margin_cells(half_size_cells, min_straight_cells, bend_runout_cells);
    for segment in route_waypoints.windows(2) {
        if direction_angle_between_cells(segment[0], segment[1]).is_none() {
            continue;
        }
        let dx = (segment[1].0 - segment[0].0).signum();
        let dy = (segment[1].1 - segment[0].1).signum();
        let steps = (segment[1].0 - segment[0].0)
            .abs()
            .max((segment[1].1 - segment[0].1).abs());
        if steps <= 0 || steps < 2 * required_margin {
            continue;
        }
        for step in required_margin..=(steps - required_margin) {
            let x = segment[0].0 + dx * step;
            let y = segment[0].1 + dy * step;
            insert_crossing_reservation_window(
                &mut keys,
                f64::from(x),
                f64::from(y),
                half_size_cells,
                width,
                height,
            );
        }
    }
    keys.into_iter().map(unpack_xy).collect()
}

fn insert_crossing_reservation_window(
    keys: &mut FxHashSet<CellKey>,
    center_x: f64,
    center_y: f64,
    half_size_cells: i32,
    width: i32,
    height: i32,
) {
    let min_x = (center_x - f64::from(half_size_cells)).floor() as i32;
    let max_x = (center_x + f64::from(half_size_cells)).ceil() as i32;
    let min_y = (center_y - f64::from(half_size_cells)).floor() as i32;
    let max_y = (center_y + f64::from(half_size_cells)).ceil() as i32;
    for x in min_x..=max_x {
        if x < 0 || x >= width {
            continue;
        }
        for y in min_y..=max_y {
            if y < 0 || y >= height {
                continue;
            }
            keys.insert(pack_xy(x, y));
        }
    }
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

fn crossing_event_svg_overlay(events: &[CrossingEvent], height: i32) -> String {
    if events.is_empty() {
        return String::new();
    }
    let mut out = String::new();
    out.push_str(r##"<g id="crossing-events">"##);
    for event in events {
        out.push_str(&format!(
            r##"<g class="crossing-event" data-net-id="{}" data-partner-net-id="{}">"##,
            event.net_id, event.partner_net_id
        ));
        out.push_str(&format!(
            "<title>crossing: net {} x net {} at ({:.3}, {:.3})</title>",
            event.net_id, event.partner_net_id, event.point.0, event.point.1
        ));
        let mut reservation_cells: Vec<CellKey> = event.reservation_keys.iter().copied().collect();
        reservation_cells.sort_unstable();
        for key in reservation_cells {
            let (x, y) = unpack_xy(key);
            let svg_y = height - y - 1;
            out.push_str(&format!(
                r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#8a8a8a" opacity="0.45" />"##
            ));
        }
        out.push_str("</g>");
    }
    out.push_str("</g>");
    out
}

fn append_crossing_event_svg_overlay(
    mut svg: String,
    events: &[CrossingEvent],
    height: i32,
) -> String {
    let overlay = crossing_event_svg_overlay(events, height);
    if overlay.is_empty() {
        return svg;
    }
    if let Some(index) = svg.rfind("</svg>") {
        svg.insert_str(index, &overlay);
    } else {
        svg.push_str(&overlay);
    }
    svg
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
        "diagonal_aware" => Ok(HeuristicMode::DiagonalAware),
        _ => Err(PyValueError::new_err(
            "heuristic_mode must be one of 'distance', 'heading_aware', or 'diagonal_aware'",
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
    if !astar_cfg.proactive_congestion_weight.is_finite()
        || astar_cfg.proactive_congestion_weight < 0.0
    {
        return Err(PyValueError::new_err(
            "proactive_congestion_weight must be finite and non-negative",
        ));
    }
    if astar_cfg.proactive_congestion_radius_cells < 0 {
        return Err(PyValueError::new_err(
            "proactive_congestion_radius_cells must be non-negative",
        ));
    }
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
        proactive_congestion_weight: astar_cfg.proactive_congestion_weight,
        proactive_congestion_radius_cells: astar_cfg.proactive_congestion_radius_cells,
        collect_detailed_timing: astar_cfg.collect_detailed_timing,
        enable_jps4: astar_cfg.enable_jps4,
        use_indexed_heap: astar_cfg.use_indexed_heap,
        primitive_ordering,
        heuristic_mode,
        heuristic_weight: astar_cfg.heuristic_weight,
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

impl PyPhotonicRouter {
    fn astar_config(
        &self,
        ignore_dynamic_obstacles: Option<bool>,
        enable_simple_routes: Option<bool>,
        history_weight: Option<f64>,
    ) -> Result<AStarConfig, String> {
        let mut cfg = self
            .astar_cfg_cached
            .as_ref()
            .map_err(|err| err.clone())?
            .clone();
        if let Some(ignore_dynamic_obstacles) = ignore_dynamic_obstacles {
            cfg.ignore_dynamic_obstacles = ignore_dynamic_obstacles;
        }
        if let Some(enable_simple_routes) = enable_simple_routes {
            cfg.enable_simple_routes = enable_simple_routes;
        }
        if let Some(history_weight) = history_weight {
            cfg.history_weight = history_weight;
        }
        Ok(cfg)
    }

    fn net_has_crossing_requirements(&self, net_id: u64) -> bool {
        self.crossing_context.is_enabled()
            && self.crossing_context.expected_crossing_count(net_id) > 0
    }

    fn add_crossing_spacing_history_for_route(&mut self, net_id: u64, route: &RouteResult) {
        if !self.net_has_crossing_requirements(net_id) {
            return;
        }
        let config = self.crossing_context.config();
        let cells = crossing_spacing_history_cells_for_route(
            &route.compressed_waypoints,
            config.min_straight_cells_per_crossing,
            config.crossing_half_size_cells,
            self.primitive_cfg.bend_radius_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        self.add_history_cells(cells, CROSSING_SPACING_HISTORY_AMOUNT);
    }

    fn crossing_local_ripup_candidates(&self, net_id: u64, limit: usize) -> Vec<u64> {
        if limit == 0 || !self.crossing_context.is_enabled() {
            return Vec::new();
        }
        let min_source_depth = self
            .crossing_context
            .ordered_constraints_for(net_id)
            .into_iter()
            .map(|constraint| constraint.source_depth)
            .min()
            .unwrap_or(0);
        if min_source_depth < CROSSING_LOCAL_RIPUP_MIN_SOURCE_DEPTH {
            return Vec::new();
        }
        let config = self.crossing_context.config();
        let mut scored: Vec<(u32, u32, u64, u64)> = self
            .crossing_allowed_partner_set(net_id)
            .into_iter()
            .filter_map(|partner_id| {
                let waypoints = self.committed_center_routes.get(&partner_id)?;
                let keys = crossing_candidate_keys_for_partner(
                    waypoints,
                    config.min_straight_cells_per_crossing,
                    config.crossing_half_size_cells,
                    self.primitive_cfg.bend_radius_cells,
                    self.grid.width as i32,
                    self.grid.height as i32,
                );
                if keys.is_empty() {
                    return None;
                }
                let mut max_extra_history = 0u32;
                let mut overlap_count = 0u32;
                let mut extra_sum = 0u64;
                for key in keys {
                    let (x, y) = unpack_xy(key);
                    let extra = self.obstacle_map.get_history_cost(x, y).saturating_sub(1);
                    if extra == 0 {
                        continue;
                    }
                    max_extra_history = max_extra_history.max(extra);
                    overlap_count = overlap_count.saturating_add(1);
                    extra_sum = extra_sum.saturating_add(u64::from(extra));
                }
                if max_extra_history == 0
                    || overlap_count < CROSSING_LOCAL_RIPUP_MIN_OVERLAP_CELLS
                    || extra_sum < CROSSING_LOCAL_RIPUP_MIN_EXTRA_SUM
                {
                    return None;
                }
                Some((max_extra_history, overlap_count, extra_sum, partner_id))
            })
            .collect();
        scored.sort_unstable_by(|a, b| {
            b.0.cmp(&a.0)
                .then_with(|| b.1.cmp(&a.1))
                .then_with(|| b.2.cmp(&a.2))
                .then_with(|| a.3.cmp(&b.3))
        });
        scored
            .into_iter()
            .take(limit)
            .map(|(_, _, _, partner_id)| partner_id)
            .collect()
    }

    fn crossing_allowed_partner_set(&self, net_id: u64) -> FxHashSet<u64> {
        if !self.crossing_context.is_enabled() {
            return FxHashSet::default();
        }
        if !self.crossing_context.config().allow_only_expected_pairs {
            return self
                .obstacle_map
                .net_route_entries()
                .map(|(partner_id, _)| partner_id)
                .filter(|partner_id| *partner_id != net_id)
                .collect();
        }
        self.crossing_context
            .allowed_partners_for(net_id)
            .into_iter()
            .filter(|partner_id| self.obstacle_map.get_net_cells(*partner_id).is_some())
            .collect()
    }

    fn crossing_events_for_route(
        &self,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
    ) -> Vec<CrossingEvent> {
        let config = self.crossing_context.config();
        let mut events = Vec::new();
        for partner_id in partner_ids {
            let Some(partner_waypoints) = self.committed_center_routes.get(partner_id) else {
                continue;
            };
            events.extend(crossing_events_for_partner(
                net_id,
                *partner_id,
                &route.compressed_waypoints,
                partner_waypoints,
                config.min_straight_cells_per_crossing,
                config.crossing_half_size_cells,
                self.primitive_cfg.bend_radius_cells,
                self.grid.width as i32,
                self.grid.height as i32,
            ));
        }
        events
    }

    fn realized_crossing_events_for_route(
        &self,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Vec<CrossingEvent> {
        if partner_ids.is_empty() || !self.crossing_context.is_enabled() {
            return Vec::new();
        }
        let route_centerline =
            self.routing_centerline_for_route(route, source_port_um, target_port_um);
        let Ok(route_centerline) = route_centerline else {
            return self.crossing_events_for_route(net_id, route, partner_ids);
        };
        if route_centerline.len() < 2 {
            return Vec::new();
        }
        let config = self.crossing_context.config();
        let required_margin_um =
            realized_crossing_margin_um(config.crossing_half_size_cells, self.grid.grid_size_um);
        let mut events = Vec::new();
        let mut seen = FxHashSet::default();
        for route_segment in route_centerline.windows(2) {
            let route_len = physical_segment_length(route_segment[0], route_segment[1]);
            if route_len <= 0.0 {
                continue;
            }
            let Some(route_grid_segment) = self.physical_segment_to_grid_segment(
                route_segment[0],
                route_segment[1],
            ) else {
                continue;
            };
            let Some(route_angle) =
                direction_angle_between_cells(route_grid_segment.0, route_grid_segment.1)
            else {
                continue;
            };
            for partner_id in partner_ids {
                let Some(partner_centerline) = self
                    .committed_realized_center_routes
                    .get(partner_id)
                    .cloned()
                    .or_else(|| {
                        self.committed_center_routes
                            .get(partner_id)
                            .map(|waypoints| self.grid_waypoints_to_centerline(waypoints))
                    })
                else {
                    continue;
                };
                for partner_segment in partner_centerline.windows(2) {
                    let partner_len =
                        physical_segment_length(partner_segment[0], partner_segment[1]);
                    if partner_len <= 0.0 {
                        continue;
                    }
                    if !physical_segments_are_perpendicular(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) {
                        continue;
                    }
                    let Some((x, y, t, u)) = physical_segment_intersection_with_params(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        continue;
                    };
                    let route_margin = (t * route_len).min((1.0 - t) * route_len);
                    let partner_margin = (u * partner_len).min((1.0 - u) * partner_len);
                    if route_margin + 1e-9 < required_margin_um
                        || partner_margin + 1e-9 < required_margin_um
                    {
                        continue;
                    }
                    if !self.crossing_context.allows_pair(net_id, *partner_id) {
                        continue;
                    }
                    if self
                        .crossing_footprint_unrelated_dynamic_owner(net_id, *partner_id, (x, y))
                        .is_some()
                    {
                        continue;
                    }
                    let pair_key = (
                        net_id.min(*partner_id),
                        net_id.max(*partner_id),
                        (x * 1_000_000.0).round() as i64,
                        (y * 1_000_000.0).round() as i64,
                    );
                    if !seen.insert(pair_key) {
                        continue;
                    }
                    let Some((center_x, center_y)) = self.grid_cell_for_physical_point((x, y))
                    else {
                        continue;
                    };
                    let Some(partner_grid_segment) = self.physical_segment_to_grid_segment(
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        continue;
                    };
                    let Some(partner_angle) =
                        direction_angle_between_cells(partner_grid_segment.0, partner_grid_segment.1)
                    else {
                        continue;
                    };
                    events.push(CrossingEvent {
                        net_id,
                        partner_net_id: *partner_id,
                        point: (x, y),
                        route_segment: route_grid_segment,
                        partner_segment: partner_grid_segment,
                        route_angle,
                        partner_angle,
                        reservation_keys: crossing_reservation_window_keys(
                            f64::from(center_x),
                            f64::from(center_y),
                            config.crossing_half_size_cells,
                            self.grid.width as i32,
                            self.grid.height as i32,
                        ),
                    });
                }
            }
        }
        events
    }

    fn crossing_candidate_keys_for_partners(
        &self,
        partner_ids: &FxHashSet<u64>,
    ) -> FxHashSet<CellKey> {
        let config = self.crossing_context.config();
        let mut keys = FxHashSet::default();
        for partner_id in partner_ids {
            let Some(partner_waypoints) = self.committed_center_routes.get(partner_id) else {
                continue;
            };
            keys.extend(crossing_candidate_keys_for_partner(
                partner_waypoints,
                config.min_straight_cells_per_crossing,
                config.crossing_half_size_cells,
                self.primitive_cfg.bend_radius_cells,
                self.grid.width as i32,
                self.grid.height as i32,
            ));
        }
        keys
    }

    fn crossing_partner_ids_from_events(events: &[CrossingEvent]) -> FxHashSet<u64> {
        events.iter().map(|event| event.partner_net_id).collect()
    }

    fn crossing_partner_ids_for_net(events: &[CrossingEvent], net_id: u64) -> FxHashSet<u64> {
        let mut partner_ids = FxHashSet::default();
        for event in events {
            if event.net_id == net_id {
                partner_ids.insert(event.partner_net_id);
            } else if event.partner_net_id == net_id {
                partner_ids.insert(event.net_id);
            }
        }
        partner_ids
    }

    fn crossing_events_cover_partners(
        events: &[CrossingEvent],
        partner_ids: &FxHashSet<u64>,
    ) -> bool {
        if partner_ids.is_empty() {
            return true;
        }
        let crossed_partner_ids = Self::crossing_partner_ids_from_events(events);
        partner_ids
            .iter()
            .all(|partner_id| crossed_partner_ids.contains(partner_id))
    }

    fn crossing_events_have_disjoint_reservations(events: &[CrossingEvent]) -> bool {
        let mut seen = FxHashSet::default();
        for event in events {
            for key in &event.reservation_keys {
                if !seen.insert(*key) {
                    return false;
                }
            }
        }
        true
    }

    fn crossing_partners_with_overlapping_reservations(events: &[CrossingEvent]) -> FxHashSet<u64> {
        let mut owner_by_key: FxHashMap<CellKey, u64> = FxHashMap::default();
        let mut overlapping_partners = FxHashSet::default();
        for event in events {
            for key in &event.reservation_keys {
                if let Some(previous_partner_id) = owner_by_key.insert(*key, event.partner_net_id) {
                    overlapping_partners.insert(previous_partner_id);
                    overlapping_partners.insert(event.partner_net_id);
                }
            }
        }
        overlapping_partners
    }

    fn crossing_reservation_blockers(
        &self,
        net_id: u64,
        events: &[CrossingEvent],
    ) -> CrossingReservationBlockers {
        let mut blockers = CrossingReservationBlockers::default();
        for event in events {
            for key in &event.reservation_keys {
                let (x, y) = unpack_xy(*key);
                if !self.obstacle_map.in_bounds(x, y) {
                    blockers.has_static_blocker = true;
                    continue;
                }
                if self.obstacle_map.is_static_blocked(x, y) {
                    blockers.has_static_blocker = true;
                }
                for owner in self.obstacle_map.dynamic_owners_for_cells(&[(x, y)]) {
                    if owner != net_id && owner != event.partner_net_id {
                        blockers.dynamic_blockers.insert(owner);
                    }
                }
            }
        }
        blockers
    }

    fn grid_cell_for_physical_point(&self, point: (f64, f64)) -> Option<(i32, i32)> {
        if !point.0.is_finite() || !point.1.is_finite() || self.grid.grid_size_um <= 0.0 {
            return None;
        }
        let x = ((point.0 - self.grid.origin_x_um) / self.grid.grid_size_um).floor() as i32;
        let y = ((point.1 - self.grid.origin_y_um) / self.grid.grid_size_um).floor() as i32;
        self.obstacle_map.in_bounds(x, y).then_some((x, y))
    }

    fn physical_segment_to_grid_segment(
        &self,
        start: (f64, f64),
        end: (f64, f64),
    ) -> Option<((i32, i32), (i32, i32))> {
        Some((
            self.grid_cell_for_physical_point(start)?,
            self.grid_cell_for_physical_point(end)?,
        ))
    }

    fn crossing_repair_keepout_radius_cells(&self) -> i32 {
        let config = self.crossing_context.config();
        crossing_required_margin_cells(
            config.crossing_half_size_cells,
            config.min_straight_cells_per_crossing,
            self.primitive_cfg.bend_radius_cells,
        )
        .max(1)
    }

    fn crossing_repair_keepout_radius_for_reason(&self, reason: &str) -> i32 {
        if reason == "crossing_footprint_contains_route_geometry"
            || reason == "crossing_footprint_overlap"
        {
            return self
                .crossing_context
                .config()
                .crossing_half_size_cells
                .saturating_add(1)
                .max(1);
        }
        if reason == "not_perpendicular" {
            return self
                .crossing_context
                .config()
                .crossing_half_size_cells
                .saturating_add(1)
                .max(1);
        }
        self.crossing_repair_keepout_radius_cells()
    }

    fn crossing_physical_violation_repair_keepout_keys(
        &self,
        violations: &[InvalidCrossingIntersection],
        partner_ids: &[u64],
    ) -> FxHashSet<CellKey> {
        if violations.is_empty() || partner_ids.is_empty() {
            return FxHashSet::default();
        }
        let partners: FxHashSet<u64> = partner_ids.iter().copied().collect();
        let mut keys = FxHashSet::default();
        for violation in violations {
            if !partners.contains(&violation.partner_net_id) {
                continue;
            }
            let radius = self.crossing_repair_keepout_radius_for_reason(violation.reason);
            let Some((center_x, center_y)) = self.grid_cell_for_physical_point(violation.point)
            else {
                continue;
            };
            for y in center_y.saturating_sub(radius)..=center_y.saturating_add(radius) {
                for x in center_x.saturating_sub(radius)..=center_x.saturating_add(radius) {
                    if self.obstacle_map.in_bounds(x, y) {
                        keys.insert(pack_xy(x, y));
                    }
                }
            }
        }
        keys
    }

    fn crossing_grid_violation_repair_keepout_keys(
        &self,
        violations: &[InvalidCrossingIntersection],
        partner_ids: &[u64],
    ) -> FxHashSet<CellKey> {
        if violations.is_empty() || partner_ids.is_empty() {
            return FxHashSet::default();
        }
        let partners: FxHashSet<u64> = partner_ids.iter().copied().collect();
        let radius = self.crossing_repair_keepout_radius_cells();
        let mut keys = FxHashSet::default();
        for violation in violations {
            if !partners.contains(&violation.partner_net_id)
                || !violation.point.0.is_finite()
                || !violation.point.1.is_finite()
            {
                continue;
            }
            let center_x = violation.point.0.floor() as i32;
            let center_y = violation.point.1.floor() as i32;
            if !self.obstacle_map.in_bounds(center_x, center_y) {
                continue;
            }
            for y in center_y.saturating_sub(radius)..=center_y.saturating_add(radius) {
                for x in center_x.saturating_sub(radius)..=center_x.saturating_add(radius) {
                    if self.obstacle_map.in_bounds(x, y) {
                        keys.insert(pack_xy(x, y));
                    }
                }
            }
        }
        keys
    }

    fn crossing_error_repair_keepout_keys(&self, error: &str) -> FxHashSet<CellKey> {
        self.crossing_error_repair_keepout_keys_with_options(error, true)
    }

    fn crossing_error_repair_keepout_keys_with_options(
        &self,
        error: &str,
        tight_not_perpendicular: bool,
    ) -> FxHashSet<CellKey> {
        const PREFIX: &str = "Illegal realized crossing: net ";
        if !error.starts_with(PREFIX) {
            return FxHashSet::default();
        }
        let Some((_, point_and_rest)) = error.split_once(" at (") else {
            return FxHashSet::default();
        };
        let Some((point_text, _)) = point_and_rest.split_once(')') else {
            return FxHashSet::default();
        };
        let Some((x_text, y_text)) = point_text.split_once(',') else {
            return FxHashSet::default();
        };
        let Ok(point_x_um) = x_text.trim().parse::<f64>() else {
            return FxHashSet::default();
        };
        let Ok(point_y_um) = y_text.trim().parse::<f64>() else {
            return FxHashSet::default();
        };
        let Some((center_x, center_y)) =
            self.grid_cell_for_physical_point((point_x_um, point_y_um))
        else {
            return FxHashSet::default();
        };
        let radius = if error.contains("(crossing_footprint_contains_route_geometry)")
            || error.contains("(crossing_footprint_overlap)")
        {
            self.crossing_repair_keepout_radius_for_reason(
                "crossing_footprint_contains_route_geometry",
            )
        } else if tight_not_perpendicular && error.contains("(not_perpendicular)") {
            1
        } else {
            self.crossing_repair_keepout_radius_cells()
        };
        let mut keys = FxHashSet::default();
        for y in center_y.saturating_sub(radius)..=center_y.saturating_add(radius) {
            for x in center_x.saturating_sub(radius)..=center_x.saturating_add(radius) {
                if self.obstacle_map.in_bounds(x, y) {
                    keys.insert(pack_xy(x, y));
                }
            }
        }
        keys
    }

    fn dynamic_commit_error_repair_keepout_keys(&self, error: &str) -> FxHashSet<CellKey> {
        let Some((_, rest)) = error.split_once("dynamic_overlap_bbox=(") else {
            return FxHashSet::default();
        };
        let Some((bbox_text, _)) = rest.split_once(')') else {
            return FxHashSet::default();
        };
        let values: Vec<i32> = bbox_text
            .split(',')
            .filter_map(|value| value.trim().parse::<i32>().ok())
            .collect();
        if values.len() != 4 {
            return FxHashSet::default();
        }
        let (min_x, max_x, min_y, max_y) = (values[0], values[1], values[2], values[3]);
        if min_x > max_x || min_y > max_y {
            return FxHashSet::default();
        }
        let bbox_width = max_x.saturating_sub(min_x).saturating_add(1);
        let bbox_height = max_y.saturating_sub(min_y).saturating_add(1);
        let (keepout_min_x, keepout_max_x, keepout_min_y, keepout_max_y) =
            if bbox_width.saturating_mul(bbox_height) <= 256 {
                (
                    min_x.saturating_sub(1),
                    max_x.saturating_add(1),
                    min_y.saturating_sub(1),
                    max_y.saturating_add(1),
                )
            } else {
                let center_x = min_x.saturating_add(max_x) / 2;
                let center_y = min_y.saturating_add(max_y) / 2;
                (
                    center_x.saturating_sub(3),
                    center_x.saturating_add(3),
                    center_y.saturating_sub(3),
                    center_y.saturating_add(3),
                )
            };
        let mut keys = FxHashSet::default();
        for y in keepout_min_y..=keepout_max_y {
            for x in keepout_min_x..=keepout_max_x {
                if self.obstacle_map.in_bounds(x, y) {
                    keys.insert(pack_xy(x, y));
                }
            }
        }
        keys
    }

    fn augmented_crossing_error_repair_keepout(
        &self,
        base_keepout: &FxHashSet<CellKey>,
        error: &str,
    ) -> (FxHashSet<CellKey>, FxHashSet<CellKey>) {
        let mut error_keepout = self.crossing_error_repair_keepout_keys(error);
        error_keepout.extend(self.dynamic_commit_error_repair_keepout_keys(error));
        if error_keepout.is_empty() {
            return (base_keepout.clone(), FxHashSet::default());
        }
        let mut merged = base_keepout.clone();
        let mut extra = FxHashSet::default();
        for key in error_keepout {
            if merged.insert(key) {
                extra.insert(key);
            }
        }
        (merged, extra)
    }

    fn remember_crossing_error_repair_keepout(
        &self,
        learned_keepout: &mut FxHashSet<CellKey>,
        error: &str,
    ) -> bool {
        let mut learned_new_key = false;
        for key in self.crossing_error_repair_keepout_keys(error) {
            if learned_keepout.insert(key) {
                learned_new_key = true;
            }
        }
        learned_new_key
    }

    fn remember_local_repair_error_keepout(
        &self,
        learned_keepout: &mut FxHashSet<CellKey>,
        error: &str,
    ) -> bool {
        let mut learned_new_key =
            self.remember_crossing_error_repair_keepout(learned_keepout, error);
        for key in self.dynamic_commit_error_repair_keepout_keys(error) {
            if learned_keepout.insert(key) {
                learned_new_key = true;
            }
        }
        learned_new_key
    }

    fn remember_victim_repair_error_keepout(
        &self,
        learned_keepout: &mut FxHashSet<CellKey>,
        victim_only_keepout: &mut FxHashSet<CellKey>,
        error: &str,
        current_net_id: u64,
    ) -> bool {
        let mut learned_new_key = false;
        let current_in_crossing = illegal_crossing_net_ids_from_error(error)
            .into_iter()
            .any(|net_id| net_id == current_net_id);
        let crossing_target_keepout = if current_in_crossing {
            &mut *victim_only_keepout
        } else {
            &mut *learned_keepout
        };
        for key in self.crossing_error_repair_keepout_keys(error) {
            if crossing_target_keepout.insert(key) {
                learned_new_key = true;
            }
        }

        let dynamic_keys = self.dynamic_commit_error_repair_keepout_keys(error);
        if dynamic_keys.is_empty() {
            return learned_new_key;
        }
        let current_owned_overlap =
            dynamic_commit_error_overlap_owner_ids(error).contains(&current_net_id);
        let target_keepout = if current_owned_overlap {
            victim_only_keepout
        } else {
            learned_keepout
        };
        for key in dynamic_keys {
            if target_keepout.insert(key) {
                learned_new_key = true;
            }
        }
        learned_new_key
    }

    fn crossing_route_satisfies_partner_constraints(
        &self,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
        crossing_events: &[CrossingEvent],
    ) -> bool {
        self.invalid_crossing_intersections_for_route(net_id, route, partner_ids)
            .is_empty()
            && self.crossing_events_satisfy_partner_constraints(
                net_id,
                partner_ids,
                crossing_events,
            )
    }

    fn crossing_events_satisfy_partner_constraints(
        &self,
        net_id: u64,
        partner_ids: &FxHashSet<u64>,
        crossing_events: &[CrossingEvent],
    ) -> bool {
        if partner_ids.is_empty() {
            return true;
        }
        let reservation_blockers = self.crossing_reservation_blockers(net_id, crossing_events);
        !crossing_events.is_empty()
            && Self::crossing_events_have_disjoint_reservations(crossing_events)
            && reservation_blockers.is_clear()
            && (!self.crossing_context.config().allow_only_expected_pairs
                || Self::crossing_events_cover_partners(crossing_events, partner_ids))
    }

    #[allow(clippy::too_many_arguments)]
    fn try_route_with_collision_crossings(
        &self,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Option<(RouteResult, Vec<CrossingEvent>)> {
        self.try_route_with_collision_crossings_with_loss(
            net_id,
            source,
            target,
            opened_ref,
            search_cfg,
            block_radius_cells,
            dynamic_clearance_exempt_keys,
            partner_ids,
            source_port_um,
            target_port_um,
            opened_cell_keys,
            None,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn try_route_with_collision_crossings_with_loss(
        &self,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        crossing_loss_override: Option<f64>,
    ) -> Option<(RouteResult, Vec<CrossingEvent>)> {
        self.try_route_with_collision_crossings_using_primitives(
            &self.primitives,
            net_id,
            source,
            target,
            opened_ref,
            search_cfg,
            block_radius_cells,
            dynamic_clearance_exempt_keys,
            partner_ids,
            source_port_um,
            target_port_um,
            opened_cell_keys,
            crossing_loss_override,
        )
    }

    fn opened_cells_without_dynamic_overlap(
        &self,
        opened_ref: &FxHashSet<CellKey>,
        source: State,
        target: State,
    ) -> Option<FxHashSet<CellKey>> {
        let source_key = pack_xy(source.x, source.y);
        let target_key = pack_xy(target.x, target.y);
        let filtered: FxHashSet<CellKey> = opened_ref
            .iter()
            .copied()
            .filter(|&key| {
                key == source_key || key == target_key || {
                    let (x, y) = unpack_xy(key);
                    !self.obstacle_map.is_dynamic_blocked(x, y)
                        && self.obstacle_map.dynamic_owners_for_cells(&[(x, y)]).is_empty()
                }
            })
            .collect();
        (filtered.len() != opened_ref.len()).then_some(filtered)
    }

    #[allow(clippy::too_many_arguments)]
    fn try_route_with_collision_crossings_using_primitives(
        &self,
        primitives: &PrimitiveLibrary,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        crossing_loss_override: Option<f64>,
    ) -> Option<(RouteResult, Vec<CrossingEvent>)> {
        if !self.crossing_context.is_enabled() || partner_ids.is_empty() {
            return None;
        }
        let crossing_cfg = self.crossing_context.config();
        let crossing_partners: Vec<CrossingSearchPartner> =
            if crossing_cfg.allow_only_expected_pairs {
                self.crossing_context
                    .ordered_constraints_for(net_id)
                    .into_iter()
                    .filter_map(|constraint| {
                        let partner_id = if constraint.net_id == net_id {
                            constraint.partner_net_id
                        } else {
                            constraint.net_id
                        };
                        if !partner_ids.contains(&partner_id) {
                            return None;
                        }
                        self.committed_center_routes
                            .get(&partner_id)
                            .map(|waypoints| CrossingSearchPartner {
                                net_id: partner_id,
                                waypoints: waypoints.clone(),
                            })
                    })
                    .collect()
            } else {
                partner_ids
                    .iter()
                    .filter_map(|partner_id| {
                        self.committed_center_routes
                            .get(partner_id)
                            .map(|waypoints| CrossingSearchPartner {
                                net_id: *partner_id,
                                waypoints: waypoints.clone(),
                            })
                    })
                    .collect()
            };
        if crossing_partners.is_empty() {
            return None;
        }
        let crossing_search = CrossingSearchConfig {
            net_id,
            partners: crossing_partners,
            min_straight_cells: crossing_cfg.min_straight_cells_per_crossing,
            crossing_half_size_cells: crossing_cfg.crossing_half_size_cells,
            bend_runout_cells: self.primitive_cfg.bend_radius_cells,
            crossing_loss: crossing_loss_override.unwrap_or(crossing_cfg.crossing_loss),
            require_all_partners: crossing_cfg.allow_only_expected_pairs,
        };
        let mut crossing_search_cfg = search_cfg.clone();
        crossing_search_cfg.enable_simple_routes = false;
        crossing_search_cfg.enable_jps4 = false;
        crossing_search_cfg.routing_window_fallback_full_grid = false;
        let trace_crossing = std::env::var("PHOTONIC_ROUTER_TRACE_CROSSING_NET")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .map_or_else(
                || std::env::var_os("PHOTONIC_ROUTER_TRACE_CROSSING").is_some(),
                |trace_net_id| trace_net_id == net_id,
            );
        if trace_crossing {
            eprintln!(
                "collision-crossing start net={} partners={:?} block_radius={} min_straight={} half_size={}",
                net_id,
                crossing_search
                    .partners
                    .iter()
                    .map(|partner| partner.net_id)
                    .collect::<Vec<_>>(),
                block_radius_cells,
                crossing_search.min_straight_cells,
                crossing_search.crossing_half_size_cells,
            );
            for partner in &crossing_search.partners {
                self.trace_committed_partner_centerline_compare(net_id, partner.net_id);
            }
        }
        let result = route_single_net_with_collision_crossing_config(
            &self.obstacle_map,
            primitives,
            source,
            target,
            Some(opened_ref),
            &crossing_search_cfg,
            block_radius_cells.max(0),
            dynamic_clearance_exempt_keys,
            &crossing_search,
        )?;
        if trace_crossing {
            eprintln!(
                "collision-crossing result net={} expanded={} generated={} accepted={} candidates={} cost={} waypoints={:?}",
                net_id,
                result.stats.expanded_states,
                result.stats.generated_neighbors,
                result.stats.crossing_accepted,
                result.stats.crossing_candidate_checks,
                result.total_cost,
                result.compressed_waypoints,
            );
        }
        let crossing_events = self.realized_crossing_events_for_route(
            net_id,
            &result,
            partner_ids,
            source_port_um,
            target_port_um,
        );
        if trace_crossing {
            eprintln!(
                "collision-crossing events net={} events={:?}",
                net_id,
                crossing_events
                    .iter()
                    .map(|event| (
                        event.partner_net_id,
                        event.point,
                        event.route_angle,
                        event.partner_angle
                    ))
                    .collect::<Vec<_>>(),
            );
        }
        let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
        let required_partner_ids = if crossing_cfg.allow_only_expected_pairs {
            partner_ids
        } else {
            &crossed_partner_ids
        };
        let satisfies = self.crossing_events_satisfy_partner_constraints(
            net_id,
            required_partner_ids,
            &crossing_events,
        );
        let realized_violations = self.crossing_violations_for_route_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
            opened_cell_keys,
        );
        if trace_crossing {
            eprintln!(
                "collision-crossing validation net={} crossed={:?} satisfies={} realized_violations={:?}",
                net_id,
                crossed_partner_ids,
                satisfies,
                realized_violations
                    .iter()
                    .map(|violation| (violation.partner_net_id, violation.point, violation.reason))
                    .collect::<Vec<_>>(),
            );
        }
        if satisfies && realized_violations.is_empty() {
            return Some((result, crossing_events));
        }
        None
    }

    #[allow(clippy::too_many_arguments)]
    fn try_route_through_collision_partner_set(
        &self,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Option<(RouteResult, Vec<CrossingEvent>)> {
        if !self.crossing_context.is_enabled() || partner_ids.is_empty() {
            return None;
        }
        let crossing_cfg = self.crossing_context.config();
        let crossing_partners: Vec<CrossingSearchPartner> = partner_ids
            .iter()
            .filter_map(|partner_id| {
                self.committed_center_routes
                    .get(partner_id)
                    .map(|waypoints| CrossingSearchPartner {
                        net_id: *partner_id,
                        waypoints: waypoints.clone(),
                    })
            })
            .collect();
        if crossing_partners.is_empty() {
            return None;
        }
        let crossing_search = CrossingSearchConfig {
            net_id,
            partners: crossing_partners,
            min_straight_cells: crossing_cfg.min_straight_cells_per_crossing,
            crossing_half_size_cells: crossing_cfg.crossing_half_size_cells,
            bend_runout_cells: self.primitive_cfg.bend_radius_cells,
            crossing_loss: crossing_cfg.crossing_loss,
            require_all_partners: true,
        };
        let mut crossing_search_cfg = search_cfg.clone();
        crossing_search_cfg.require_terminal_straights = false;
        crossing_search_cfg.enable_simple_routes = false;
        crossing_search_cfg.enable_jps4 = false;
        let trace_crossing = std::env::var("PHOTONIC_ROUTER_TRACE_CROSSING_NET")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .map_or_else(
                || std::env::var_os("PHOTONIC_ROUTER_TRACE_CROSSING").is_some(),
                |trace_net_id| trace_net_id == net_id,
            );
        if trace_crossing {
            eprintln!(
                "guided-collision-crossing start net={} partners={:?} max_iterations={} block_radius={} min_straight={} half_size={}",
                net_id,
                crossing_search
                    .partners
                    .iter()
                    .map(|partner| partner.net_id)
                    .collect::<Vec<_>>(),
                crossing_search_cfg.max_iterations,
                block_radius_cells,
                crossing_search.min_straight_cells,
                crossing_search.crossing_half_size_cells,
            );
            for partner in &crossing_search.partners {
                self.trace_committed_partner_centerline_compare(net_id, partner.net_id);
            }
        }
        let mut search_map = self.obstacle_map.clone();
        search_map.clear_dynamic_blocking_for_nets(partner_ids);
        let partner_vec: Vec<u64> = partner_ids.iter().copied().collect();
        let mut guided_keepout = FxHashSet::default();
        const MAX_GUIDED_CROSSING_VALIDATION_RETRIES: usize = 4;
        for retry_idx in 0..=MAX_GUIDED_CROSSING_VALIDATION_RETRIES {
            let result = route_single_net_with_crossing_config(
                &search_map,
                &self.primitives,
                source,
                target,
                Some(opened_ref),
                &crossing_search_cfg,
                block_radius_cells.max(0),
                dynamic_clearance_exempt_keys,
                &crossing_search,
            )?;
            let crossing_events = self.realized_crossing_events_for_route(
                net_id,
                &result,
                partner_ids,
                source_port_um,
                target_port_um,
            );
            let satisfies = self.crossing_events_satisfy_partner_constraints(
                net_id,
                partner_ids,
                &crossing_events,
            );
            let realized_violations = self.crossing_violations_for_route_with_ports(
                net_id,
                &result,
                source_port_um,
                target_port_um,
                opened_cell_keys,
            );
            let covers_requested_partners =
                Self::crossing_events_cover_partners(&crossing_events, partner_ids);
            if trace_crossing {
                eprintln!(
                    "guided-collision-crossing result net={} retry={} expanded={} generated={} events={} satisfies={} covers_partners={} realized_violations={:?} cost={} waypoints={:?}",
                    net_id,
                    retry_idx,
                    result.stats.expanded_states,
                    result.stats.generated_neighbors,
                    crossing_events.len(),
                    satisfies,
                    covers_requested_partners,
                    realized_violations
                        .iter()
                        .map(|violation| (violation.partner_net_id, violation.point, violation.reason))
                        .collect::<Vec<_>>(),
                    result.total_cost,
                    result.compressed_waypoints,
                );
            }
            if !crossing_events.is_empty()
                && satisfies
                && covers_requested_partners
                && realized_violations.is_empty()
            {
                return Some((result, crossing_events));
            }

            if retry_idx == MAX_GUIDED_CROSSING_VALIDATION_RETRIES {
                break;
            }
            let retry_keepout = self.crossing_physical_violation_repair_keepout_keys(
                &realized_violations,
                &partner_vec,
            );
            let new_keepout: FxHashSet<CellKey> = retry_keepout
                .into_iter()
                .filter(|key| guided_keepout.insert(*key))
                .collect();
            if new_keepout.is_empty() {
                break;
            }
            let added = search_map.add_static_keys(&new_keepout);
            if trace_crossing {
                eprintln!(
                    "guided-collision-crossing retry-keepout net={} retry={} keys={} added={}",
                    net_id,
                    retry_idx + 1,
                    new_keepout.len(),
                    added,
                );
            }
        }
        None
    }

    fn invalid_crossing_intersections_for_route(
        &self,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
    ) -> Vec<InvalidCrossingIntersection> {
        if route.compressed_waypoints.len() < 2 || partner_ids.is_empty() {
            return Vec::new();
        }
        let config = self.crossing_context.config();
        let required_margin = f64::from(crossing_required_margin_cells(
            config.crossing_half_size_cells,
            config.min_straight_cells_per_crossing,
            self.primitive_cfg.bend_radius_cells,
        ));
        let mut invalid = Vec::new();
        let mut seen_centers = FxHashSet::default();
        for route_segment in route.compressed_waypoints.windows(2) {
            let Some(route_angle) =
                direction_angle_between_cells(route_segment[0], route_segment[1])
            else {
                continue;
            };
            let route_len = segment_length_cells(route_segment[0], route_segment[1]);
            if route_len <= 0.0 {
                continue;
            }
            for partner_id in partner_ids {
                let Some(partner_waypoints) = self.committed_center_routes.get(partner_id) else {
                    continue;
                };
                for partner_segment in partner_waypoints.windows(2) {
                    let Some(partner_angle) =
                        direction_angle_between_cells(partner_segment[0], partner_segment[1])
                    else {
                        continue;
                    };
                    if !axes_are_perpendicular(route_angle, partner_angle) {
                        continue;
                    }
                    let partner_len = segment_length_cells(partner_segment[0], partner_segment[1]);
                    if partner_len <= 0.0 {
                        continue;
                    }
                    let Some((x, y, t, u)) = segment_intersection_with_params(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        continue;
                    };
                    let route_margin = (t * route_len).min((1.0 - t) * route_len);
                    let partner_margin = (u * partner_len).min((1.0 - u) * partner_len);
                    if route_margin + 1e-9 >= required_margin
                        && partner_margin + 1e-9 >= required_margin
                    {
                        continue;
                    }
                    let rounded_center = (
                        (x * 1_000_000.0).round() as i64,
                        (y * 1_000_000.0).round() as i64,
                    );
                    if !seen_centers.insert(rounded_center) {
                        continue;
                    }
                    invalid.push(InvalidCrossingIntersection {
                        net_id,
                        partner_net_id: *partner_id,
                        point: (x, y),
                        reason: "insufficient_straight_margin",
                    });
                }
            }
        }
        invalid
    }

    fn geometry_grid(&self) -> Result<GeometryGridSpec, String> {
        GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| err.to_string())
    }

    fn grid_waypoints_to_centerline(&self, waypoints: &[(i32, i32)]) -> Vec<(f64, f64)> {
        let grid = GeometryGridSpec {
            grid_size_um: self.grid.grid_size_um,
            origin_x_um: self.grid.origin_x_um,
            origin_y_um: self.grid.origin_y_um,
        };
        waypoints
            .iter()
            .map(|(x, y)| grid.cell_center(*x, *y))
            .collect()
    }

    fn trace_committed_partner_centerline_compare(&self, net_id: u64, partner_id: u64) {
        let Some(requested_partner_id) = std::env::var("PHOTONIC_ROUTER_TRACE_PARTNER_NET")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
        else {
            return;
        };
        if requested_partner_id != partner_id {
            return;
        }
        let grid_waypoints = self.committed_center_routes.get(&partner_id);
        let realized_centerline = self.committed_realized_center_routes.get(&partner_id);
        let grid_centerline_um = grid_waypoints
            .map(|waypoints| self.grid_waypoints_to_centerline(waypoints))
            .unwrap_or_default();
        eprintln!(
            "committed-centerline-compare net={} partner={} grid_waypoints={:?} grid_centerline_um={:?} realized_centerline_um={:?}",
            net_id,
            partner_id,
            grid_waypoints,
            grid_centerline_um,
            realized_centerline,
        );
    }

    fn route_obstacle_center_cells(&self, route: &RouteResult) -> Vec<(i32, i32)> {
        route_to_grid_path_rs(route, &self.primitives).unwrap_or_else(|_| route.cells.clone())
    }

    fn route_dynamic_center_cells(
        &self,
        route: &RouteResult,
        _source_port_um: Option<(f64, f64)>,
        _target_port_um: Option<(f64, f64)>,
    ) -> Vec<(i32, i32)> {
        self.route_obstacle_center_cells(route)
    }

    #[allow(clippy::too_many_arguments)]
    fn route_commit_and_core_cells(
        &self,
        route: &RouteResult,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<&[(i32, i32)]>,
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> (Vec<(i32, i32)>, Vec<(i32, i32)>) {
        let center_cells = self.route_dynamic_center_cells(route, source_port_um, target_port_um);
        let route_cells = route_commit_cells(
            &center_cells,
            block_radius_cells,
            commit_radius_cells.unwrap_or(block_radius_cells),
            clearance_exempt_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        let core_cells = route_core_cells(
            &center_cells,
            core_radius_cells.unwrap_or(block_radius_cells),
            self.grid.width as i32,
            self.grid.height as i32,
        );
        (route_cells, core_cells)
    }

    fn dynamic_commit_rejection_error(
        &self,
        net_id: u64,
        core_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
    ) -> String {
        let clearance_exempt_keys = pack_cells(clearance_exempt_cells);
        let dynamic_blockers =
            cells_with_other_dynamic_owner(&self.obstacle_map, core_cells, &clearance_exempt_keys, net_id);
        if dynamic_blockers.is_empty() {
            return "Failed to commit routed cells to obstacle map".to_string();
        }
        let owners = sorted_other_owners_for_cells(&self.obstacle_map, &dynamic_blockers, net_id);
        format!(
            "Failed to commit routed cells to obstacle map: dynamic_overlap_count={} dynamic_overlap_owners={owners:?} dynamic_overlap_bbox={} dynamic_overlap_sample={}",
            dynamic_blockers.len(),
            format_bbox(&dynamic_blockers),
            format_cell_sample(&dynamic_blockers, 8),
        )
    }

    fn realized_centerline_for_route(
        &self,
        route: &RouteResult,
    ) -> Result<Vec<(f64, f64)>, String> {
        let grid = self.geometry_grid()?;
        match route_to_primitive_centerline_rs(route, &self.primitives, &grid) {
            Ok(centerline) => Ok(compress_physical_centerline(centerline)),
            Err(_) => {
                let path = route_to_grid_path_rs(route, &self.primitives)
                    .unwrap_or_else(|_| route.compressed_waypoints.clone());
                grid_path_to_centerline_rs(&path, &grid)
                    .map(compress_physical_centerline)
                    .map_err(|err| err.to_string())
            }
        }
    }

    fn routing_centerline_for_route(
        &self,
        route: &RouteResult,
        _source_port_um: Option<(f64, f64)>,
        _target_port_um: Option<(f64, f64)>,
    ) -> Result<Vec<(f64, f64)>, String> {
        self.realized_centerline_for_route(route)
    }

    fn remember_committed_route_centerlines_with_ports(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<(), String> {
        let centerline = self.routing_centerline_for_route(route, source_port_um, target_port_um)?;
        let grid_path = self.route_obstacle_center_cells(route);
        let grid_waypoints = compress_grid_waypoints_rs(&grid_path);
        self.committed_center_routes
            .insert(net_id, grid_waypoints);
        self.committed_realized_center_routes
            .insert(net_id, centerline);
        Ok(())
    }

    fn remember_committed_route_opened_cells(
        &mut self,
        net_id: u64,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) {
        if let Some(opened_cell_keys) = opened_cell_keys {
            self.committed_opened_cell_keys
                .insert(net_id, opened_cell_keys.clone());
        } else {
            self.committed_opened_cell_keys.remove(&net_id);
        }
    }

    fn crossing_violations_for_route_with_ports(
        &self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Vec<InvalidCrossingIntersection> {
        let route_centerline =
            self.routing_centerline_for_route(route, source_port_um, target_port_um);
        let Ok(route_centerline) = route_centerline else {
            return Vec::new();
        };
        let mut violations =
            self.crossing_violations_for_realized_centerline(net_id, &route_centerline);
        violations.retain(|violation| {
            let Some((x, y)) = self.grid_cell_for_physical_point(violation.point) else {
                return true;
            };
            let key = pack_xy(x, y);
            let current_endpoint_access = opened_cell_keys
                .map(|opened| opened.contains(&key))
                .unwrap_or(false);
            let partner_endpoint_access = self
                .committed_opened_cell_keys
                .get(&violation.partner_net_id)
                .map(|partner_opened| partner_opened.contains(&key))
                .unwrap_or(false);
            if !(current_endpoint_access || partner_endpoint_access) {
                return true;
            }
            let tolerance_um = self.grid.grid_size_um.max(1.0e-9);
            let current_is_endpoint = physical_point_near_centerline_endpoint(
                violation.point,
                &route_centerline,
                tolerance_um,
            );
            let partner_is_endpoint = self
                .committed_realized_center_routes
                .get(&violation.partner_net_id)
                .map(|centerline| {
                    physical_point_near_centerline_endpoint(
                        violation.point,
                        centerline,
                        tolerance_um,
                    )
                })
                .unwrap_or(false);
            !(current_is_endpoint || partner_is_endpoint)
        });
        violations
    }

    fn crossing_violations_for_realized_centerline(
        &self,
        net_id: u64,
        route_centerline: &[(f64, f64)],
    ) -> Vec<InvalidCrossingIntersection> {
        if !self.crossing_context.is_enabled() {
            return Vec::new();
        }
        let config = self.crossing_context.config();
        let required_margin =
            realized_crossing_margin_um(config.crossing_half_size_cells, self.grid.grid_size_um);
        if route_centerline.len() < 2 {
            return Vec::new();
        }
        let partner_centerlines: Vec<(u64, Vec<(f64, f64)>)> = self
            .committed_center_routes
            .iter()
            .filter_map(|(partner_id, partner_grid_waypoints)| {
                if *partner_id == net_id {
                    return None;
                }
                let centerline = self
                    .committed_realized_center_routes
                    .get(partner_id)
                    .cloned()
                    .unwrap_or_else(|| self.grid_waypoints_to_centerline(partner_grid_waypoints));
                if centerline.len() < 2 {
                    None
                } else {
                    Some((*partner_id, centerline))
                }
            })
            .collect();
        if partner_centerlines.is_empty() {
            return Vec::new();
        }
        let mut invalid = Vec::new();
        let mut seen = FxHashSet::default();
        for route_segment in route_centerline.windows(2) {
            let route_len = physical_segment_length(route_segment[0], route_segment[1]);
            if route_len <= 0.0 {
                continue;
            }
            for (partner_id, partner_centerline) in &partner_centerlines {
                for partner_segment in partner_centerline.windows(2) {
                    let partner_len =
                        physical_segment_length(partner_segment[0], partner_segment[1]);
                    if partner_len <= 0.0 {
                        continue;
                    }
                    if let Some((x, y)) = physical_collinear_segment_overlap_midpoint(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) {
                        let pair_key = (
                            net_id.min(*partner_id),
                            net_id.max(*partner_id),
                            (x * 1_000_000.0).round() as i64,
                            (y * 1_000_000.0).round() as i64,
                        );
                        if !seen.insert(pair_key) {
                            continue;
                        }
                        invalid.push(InvalidCrossingIntersection {
                            net_id,
                            partner_net_id: *partner_id,
                            point: (x, y),
                            reason: "collinear_route_overlap",
                        });
                        continue;
                    }
                    let Some((x, y, t, u)) = physical_segment_intersection_with_params(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        continue;
                    };
                    let pair_key = (
                        net_id.min(*partner_id),
                        net_id.max(*partner_id),
                        (x * 1_000_000.0).round() as i64,
                        (y * 1_000_000.0).round() as i64,
                    );
                    if !seen.insert(pair_key) {
                        continue;
                    }
                    let perpendicular = physical_segments_are_perpendicular(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    );
                    let route_margin = (t * route_len).min((1.0 - t) * route_len);
                    let partner_margin = (u * partner_len).min((1.0 - u) * partner_len);
                    let pair_allowed = self.crossing_context.allows_pair(net_id, *partner_id);
                    let footprint_blocker = if self.crossing_context.config().allow_only_expected_pairs
                        && pair_allowed
                        && perpendicular
                        && route_margin + 1e-9 >= required_margin
                        && partner_margin + 1e-9 >= required_margin
                    {
                        self.crossing_footprint_unrelated_dynamic_owner(
                            net_id,
                            *partner_id,
                            (x, y),
                        )
                    } else {
                        None
                    };
                    let footprint_has_blocker = footprint_blocker.is_some();
                    if pair_allowed
                        && perpendicular
                        && route_margin + 1e-9 >= required_margin
                        && partner_margin + 1e-9 >= required_margin
                        && !footprint_has_blocker
                    {
                        continue;
                    }
                    let reason = if !pair_allowed {
                        "unexpected_pair"
                    } else if !perpendicular {
                        "not_perpendicular"
                    } else if footprint_has_blocker {
                        "crossing_footprint_contains_route_geometry"
                    } else {
                        "insufficient_straight_margin"
                    };
                    invalid.push(InvalidCrossingIntersection {
                        net_id,
                        partner_net_id: footprint_blocker.unwrap_or(*partner_id),
                        point: (x, y),
                        reason,
                    });
                }
            }
        }
        invalid
    }

    fn crossing_footprint_unrelated_dynamic_owner(
        &self,
        net_id: u64,
        partner_id: u64,
        point: (f64, f64),
    ) -> Option<u64> {
        let Some((center_x, center_y)) = self.grid_cell_for_physical_point(point) else {
            return None;
        };
        let config = self.crossing_context.config();
        let keys = crossing_reservation_window_keys(
            f64::from(center_x),
            f64::from(center_y),
            config.crossing_half_size_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        for key in keys {
            let (x, y) = unpack_xy(key);
            for owner in self.obstacle_map.dynamic_owners_for_cells(&[(x, y)]) {
                if owner != net_id && owner != partner_id {
                    return Some(owner);
                }
            }
        }
        None
    }

    fn validate_committed_crossings_for_route_with_ports(
        &self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Result<(), String> {
        let violations = self.crossing_violations_for_route_with_ports(
            net_id,
            route,
            source_port_um,
            target_port_um,
            opened_cell_keys,
        );
        if violations.is_empty() {
            return Ok(());
        }
        let violation = &violations[0];
        Err(format!(
            "Illegal realized crossing: net {} intersects net {} at ({:.3}, {:.3}) ({})",
            violation.net_id,
            violation.partner_net_id,
            violation.point.0,
            violation.point.1,
            violation.reason
        ))
    }

    fn rollback_committed_route(&mut self, net_id: u64) {
        self.remove_crossing_events_for_net(net_id);
        self.obstacle_map.ripup_route(net_id);
        self.committed_center_routes.remove(&net_id);
        self.committed_realized_center_routes.remove(&net_id);
        self.committed_opened_cell_keys.remove(&net_id);
        self.invalidate_meander_base_prefix();
    }

    fn register_geometric_crossing_events_for_route(&mut self, net_id: u64, route: &RouteResult) {
        if !self.crossing_context.is_enabled() {
            return;
        }
        let partner_ids = self.crossing_allowed_partner_set(net_id);
        if partner_ids.is_empty() {
            return;
        }
        let crossing_events = self.crossing_events_for_route(net_id, route, &partner_ids);
        if crossing_events.is_empty() {
            return;
        }
        self.add_crossing_events(crossing_events);
    }

    fn crossing_reservation_keys_for_events(events: &[CrossingEvent]) -> FxHashSet<CellKey> {
        let mut keys = FxHashSet::default();
        for event in events {
            keys.extend(event.reservation_keys.iter().copied());
        }
        keys
    }

    fn remove_crossing_events_for_net(&mut self, net_id: u64) {
        if self.crossing_events.is_empty() {
            return;
        }
        let mut remaining = Vec::with_capacity(self.crossing_events.len());
        let mut removed_keys = FxHashSet::default();
        for event in self.crossing_events.drain(..) {
            if event.net_id == net_id || event.partner_net_id == net_id {
                removed_keys.extend(event.reservation_keys.iter().copied());
            } else {
                remaining.push(event);
            }
        }
        self.crossing_events = remaining;
        if removed_keys.is_empty() {
            return;
        }
        self.obstacle_map.remove_static_keys(&removed_keys);
        for key in removed_keys {
            let (x, y) = unpack_xy(key);
            if !self.obstacle_map.is_static_blocked(x, y) {
                self.static_cells.remove(&key);
            }
        }
        self.invalidate_meander_base_prefix();
    }

    fn remove_crossing_events_for_all_routes(&mut self) {
        if self.crossing_events.is_empty() {
            return;
        }
        let mut removed_keys = FxHashSet::default();
        for event in self.crossing_events.drain(..) {
            removed_keys.extend(event.reservation_keys.iter().copied());
        }
        if removed_keys.is_empty() {
            return;
        }
        self.obstacle_map.remove_static_keys(&removed_keys);
        for key in removed_keys {
            let (x, y) = unpack_xy(key);
            if !self.obstacle_map.is_static_blocked(x, y) {
                self.static_cells.remove(&key);
            }
        }
        self.invalidate_meander_base_prefix();
    }

    fn add_crossing_events(&mut self, events: Vec<CrossingEvent>) {
        if events.is_empty() {
            return;
        }
        let reservation_keys = Self::crossing_reservation_keys_for_events(&events);
        if !reservation_keys.is_empty() {
            self.obstacle_map.add_static_keys(&reservation_keys);
            self.static_cells.extend(reservation_keys);
            self.invalidate_meander_base_prefix();
        }
        self.crossing_events.extend(events);
    }

    #[allow(clippy::too_many_arguments)]
    fn try_route_through_expected_crossing_partner(
        &self,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
    ) -> Option<(RouteResult, Vec<CrossingEvent>)> {
        let partner_ids = self.crossing_allowed_partner_set(net_id);
        if partner_ids.is_empty() {
            return None;
        }
        let require_all_expected_partners =
            self.crossing_context.config().allow_only_expected_pairs;

        let mut crossing_search_cfg = search_cfg.clone();
        crossing_search_cfg.require_terminal_straights = false;
        crossing_search_cfg.enable_simple_routes = false;
        if crossing_search_cfg.history_weight > 0.0 {
            crossing_search_cfg.routing_window_max_expansions =
                crossing_search_cfg.routing_window_max_expansions.min(1);
            crossing_search_cfg.routing_window_fallback_full_grid = false;
        }
        let crossing_cfg = self.crossing_context.config();
        let crossing_partners: Vec<CrossingSearchPartner> = self
            .crossing_context
            .ordered_constraints_for(net_id)
            .into_iter()
            .filter_map(|constraint| {
                let partner_id = if constraint.net_id == net_id {
                    constraint.partner_net_id
                } else {
                    constraint.net_id
                };
                if !partner_ids.contains(&partner_id) {
                    return None;
                }
                self.committed_center_routes
                    .get(&partner_id)
                    .map(|waypoints| CrossingSearchPartner {
                        net_id: partner_id,
                        waypoints: waypoints.clone(),
                    })
            })
            .collect();
        if crossing_partners.is_empty() {
            return None;
        }
        let crossing_search = CrossingSearchConfig {
            net_id,
            partners: crossing_partners,
            min_straight_cells: crossing_cfg.min_straight_cells_per_crossing,
            crossing_half_size_cells: crossing_cfg.crossing_half_size_cells,
            bend_runout_cells: self.primitive_cfg.bend_radius_cells,
            crossing_loss: crossing_cfg.crossing_loss,
            require_all_partners: require_all_expected_partners,
        };
        let trace_crossing = std::env::var("PHOTONIC_ROUTER_TRACE_CROSSING_NET")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .map_or_else(
                || std::env::var_os("PHOTONIC_ROUTER_TRACE_CROSSING").is_some(),
                |trace_net_id| trace_net_id == net_id,
            );
        if trace_crossing {
            eprintln!(
                "crossing-search start net={} partners={:?} max_iterations={} block_radius={} min_straight={} half_size={}",
                net_id,
                crossing_search
                    .partners
                    .iter()
                    .map(|partner| partner.net_id)
                    .collect::<Vec<_>>(),
                crossing_search_cfg.max_iterations,
                block_radius_cells,
                crossing_search.min_straight_cells,
                crossing_search.crossing_half_size_cells,
            );
        }

        let crossing_candidate_keys = self.crossing_candidate_keys_for_partners(&partner_ids);
        if !crossing_candidate_keys.is_empty() {
            let mut search_map = self.obstacle_map.clone();
            search_map
                .clear_dynamic_blocking_in_cells_for_nets(&crossing_candidate_keys, &partner_ids);
            if trace_crossing {
                eprintln!(
                    "crossing-search candidate-phase net={} candidate_keys={}",
                    net_id,
                    crossing_candidate_keys.len()
                );
            }
            if let Some(result) = route_single_net_with_crossing_config(
                &search_map,
                &self.primitives,
                source,
                target,
                Some(opened_ref),
                &crossing_search_cfg,
                block_radius_cells.max(0),
                dynamic_clearance_exempt_keys,
                &crossing_search,
            ) {
                if trace_crossing {
                    eprintln!(
                        "crossing-search candidate-result net={} expanded={} generated={} heap={} events={}",
                        net_id,
                        result.stats.expanded_states,
                        result.stats.generated_neighbors,
                        result.stats.max_heap_size,
                        self.crossing_events_for_route(net_id, &result, &partner_ids).len(),
                    );
                }
                let crossing_events = self.crossing_events_for_route(net_id, &result, &partner_ids);
                if self.crossing_route_satisfies_partner_constraints(
                    net_id,
                    &result,
                    &partner_ids,
                    &crossing_events,
                ) {
                    return Some((result, crossing_events));
                }
            }
        }

        let mut search_map = self.obstacle_map.clone();
        search_map.clear_dynamic_blocking_for_nets(&partner_ids);
        if trace_crossing {
            eprintln!("crossing-search broad-phase net={}", net_id);
        }
        let result = route_single_net_with_crossing_config(
            &search_map,
            &self.primitives,
            source,
            target,
            Some(opened_ref),
            &crossing_search_cfg,
            block_radius_cells.max(0),
            dynamic_clearance_exempt_keys,
            &crossing_search,
        )?;
        if trace_crossing {
            eprintln!(
                "crossing-search broad-result net={} expanded={} generated={} heap={} events={}",
                net_id,
                result.stats.expanded_states,
                result.stats.generated_neighbors,
                result.stats.max_heap_size,
                self.crossing_events_for_route(net_id, &result, &partner_ids)
                    .len(),
            );
        }
        let crossing_events = self.crossing_events_for_route(net_id, &result, &partner_ids);
        if self.crossing_route_satisfies_partner_constraints(
            net_id,
            &result,
            &partner_ids,
            &crossing_events,
        ) {
            return Some((result, crossing_events));
        }
        None
    }

    #[allow(clippy::too_many_arguments)]
    fn route_single_net_and_commit_native(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: Option<&[(i32, i32)]>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<&[(i32, i32)]>,
        clearance_exempt_cell_keys: Option<&FxHashSet<CellKey>>,
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err("target_tolerance_cells must be >= 0".to_string());
        }
        let opened_owned;
        let opened_default_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(keys) = opened_cell_keys {
            keys
        } else if let Some(cells) = opened_cells {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            opened_default_owned = self.port_open_cells.clone();
            &opened_default_owned
        };
        let validation_opened_cell_keys = opened_ref.clone();
        let mut cfg = self.astar_config(None, None, None)?;
        cfg.require_terminal_straights = true;
        let dynamic_clearance_exempt_cell_vec = clearance_exempt_cells.unwrap_or(&[]);
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
        let dynamic_clearance_exempt_keys_owned: Option<FxHashSet<CellKey>> = if block_radius_cells
            > 0
            && !dynamic_clearance_exempt_cell_vec.is_empty()
            && clearance_exempt_cell_keys.is_none()
        {
            Some(pack_cells(dynamic_clearance_exempt_cell_vec))
        } else {
            None
        };
        let dynamic_clearance_exempt_keys =
            if block_radius_cells > 0 && !dynamic_clearance_exempt_cell_vec.is_empty() {
                clearance_exempt_cell_keys.or(dynamic_clearance_exempt_keys_owned.as_ref())
            } else {
                None
            };
        if let Some(prepare_start) = prepare_start.as_ref() {
            obstacle_map_prepare_time_us += prepare_start.elapsed().as_micros();
        }
        let source_state = State::new(source.x, source.y, source.angle);
        let target_state = State::new(target.x, target.y, target.angle);
        let opened_search_owned = self.opened_cells_without_dynamic_overlap(
            opened_ref,
            source_state,
            target_state,
        );
        let opened_search_ref = opened_search_owned.as_ref().unwrap_or(opened_ref);
        let expected_crossing_partner_ids = self.crossing_allowed_partner_set(net_id);
        let require_crossing_compliant_route = self.crossing_context.is_enabled()
            && !self.use_collision_crossing_routing
            && self.crossing_context.config().allow_only_expected_pairs
            && !expected_crossing_partner_ids.is_empty();
        let collision_partner_ids = if self.use_collision_crossing_routing {
            self.crossing_allowed_partner_set(net_id)
        } else {
            FxHashSet::default()
        };
        let used_collision_crossing_attempt =
            self.use_collision_crossing_routing && !collision_partner_ids.is_empty();
        let crossing_attempt = if used_collision_crossing_attempt {
            self.try_route_with_collision_crossings(
                net_id,
                source_state,
                target_state,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
                &collision_partner_ids,
                source_port_um,
                target_port_um,
                Some(&validation_opened_cell_keys),
            )
        } else {
            self.try_route_through_expected_crossing_partner(
                net_id,
                source_state,
                target_state,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
            )
        };
        if let Some((mut crossing_result, crossing_events)) = crossing_attempt {
            let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
            let allowed_crossing_core_keys =
                Self::crossing_reservation_keys_for_events(&crossing_events);
            let commit_prepare_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            let (route_cells, core_cells) = self.route_commit_and_core_cells(
                &crossing_result,
                block_radius_cells,
                commit_radius_cells,
                clearance_exempt_cells,
                core_radius_cells,
                source_port_um,
                target_port_um,
            );
            if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
                commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
            }
            let commit_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            let committed = self
                .obstacle_map
                .commit_route_with_clearance_and_allowed_core_overlap_cells(
                    net_id,
                    &core_cells,
                    &route_cells,
                    clearance_exempt_cells.unwrap_or(&[]),
                    &crossed_partner_ids,
                    Some(&allowed_crossing_core_keys),
                );
            if let Some(commit_start) = commit_start.as_ref() {
                commit_time_us += commit_start.elapsed().as_micros();
            }
            if committed {
                self.remove_crossing_events_for_net(net_id);
                self.add_crossing_events(crossing_events);
                if collect_timing {
                    crossing_result.stats.obstacle_map_prepare_time_us +=
                        obstacle_map_prepare_time_us;
                    crossing_result.stats.commit_prepare_time_us += commit_prepare_time_us;
                    crossing_result.stats.commit_time_us += commit_time_us;
                }
                if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                    net_id,
                    &crossing_result,
                    source_port_um,
                    target_port_um,
                ) {
                    self.rollback_committed_route(net_id);
                    return Err(error);
                }
                self.remember_committed_route_opened_cells(
                    net_id,
                    Some(&validation_opened_cell_keys),
                );
                self.add_crossing_spacing_history_for_route(net_id, &crossing_result);
                self.invalidate_meander_base_prefix();
                if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                    net_id,
                    &crossing_result,
                    source_port_um,
                    target_port_um,
                    Some(&validation_opened_cell_keys),
                ) {
                    self.rollback_committed_route(net_id);
                    self.remove_crossing_events_for_net(net_id);
                    if !self.use_collision_crossing_routing {
                        return Err(error);
                    }
                } else {
                    return Ok(crossing_result);
                }
            }
        }
        // A failed local collision-crossing attempt means this specific
        // crossing placement is unavailable. It must not make the whole net
        // unroutable: normal A* can still route around the blocker, and only a
        // true dynamic blockage should enter rip-up/repair.
        if require_crossing_compliant_route {
            return Err("No crossing-compliant route found".to_string());
        }
        if block_radius_cells > 0 {
            let simple_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            let simple_result = {
                try_simple_route_with_dynamic_expansion_config(
                    &self.obstacle_map,
                    &self.primitives,
                    source_state,
                    target_state,
                    Some(opened_search_ref),
                    &cfg,
                    block_radius_cells,
                    dynamic_clearance_exempt_keys,
                )
            };
            if let Some(mut result) = simple_result {
                if let Some(simple_start) = simple_start.as_ref() {
                    simple_route_time_us += simple_start.elapsed().as_micros();
                }
                let commit_prepare_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let (route_cells, core_cells) = self.route_commit_and_core_cells(
                    &result,
                    block_radius_cells,
                    commit_radius_cells,
                    clearance_exempt_cells,
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                );
                if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
                    commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
                }
                let commit_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let allowed_partner_ids = FxHashSet::default();
                let committed = self
                    .obstacle_map
                    .commit_route_with_clearance_and_allowed_core_overlaps(
                        net_id,
                        &core_cells,
                        &route_cells,
                        clearance_exempt_cells.unwrap_or(&[]),
                        &allowed_partner_ids,
                    );
                if let Some(commit_start) = commit_start.as_ref() {
                    commit_time_us += commit_start.elapsed().as_micros();
                }
                if committed {
                    self.remove_crossing_events_for_net(net_id);
                    self.register_geometric_crossing_events_for_route(net_id, &result);
                    if collect_timing {
                        result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
                        result.stats.simple_route_time_us += simple_route_time_us;
                        result.stats.commit_prepare_time_us += commit_prepare_time_us;
                        result.stats.commit_time_us += commit_time_us;
                    }
                    if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                        net_id,
                        &result,
                        source_port_um,
                        target_port_um,
                    ) {
                        self.rollback_committed_route(net_id);
                        return Err(error);
                    }
                    self.remember_committed_route_opened_cells(
                        net_id,
                        Some(&validation_opened_cell_keys),
                    );
                    self.add_crossing_spacing_history_for_route(net_id, &result);
                    self.invalidate_meander_base_prefix();
                    if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                        net_id,
                        &result,
                        source_port_um,
                        target_port_um,
                        Some(&validation_opened_cell_keys),
                    ) {
                        self.rollback_committed_route(net_id);
                        return Err(error);
                    }
                    return Ok(result);
                }
            } else if let Some(simple_start) = simple_start.as_ref() {
                simple_route_time_us += simple_start.elapsed().as_micros();
            }
        }
        let search_cfg = if block_radius_cells > 0 {
            let mut search_cfg = self.astar_config(None, Some(false), None)?;
            search_cfg.require_terminal_straights = true;
            search_cfg
        } else {
            cfg
        };
        let zero_radius_overlay = block_radius_cells <= 0
            && dynamic_clearance_exempt_keys.is_some()
            && !search_cfg.enable_jps4;
        let mut opened_dynamic_obstacle_map;
        let mut result = if block_radius_cells > 0 || zero_radius_overlay {
            route_single_net_with_dynamic_expansion_config(
                &self.obstacle_map,
                &self.primitives,
                source_state,
                target_state,
                Some(opened_search_ref),
                &search_cfg,
                block_radius_cells.max(0),
                dynamic_clearance_exempt_keys,
            )
        } else {
            let search_obstacle_map = if dynamic_clearance_exempt_keys.is_some() {
                opened_dynamic_obstacle_map = self.obstacle_map.clone();
                opened_dynamic_obstacle_map
                    .clear_dynamic_clearance_in_cells(dynamic_clearance_exempt_cell_vec);
                &opened_dynamic_obstacle_map
            } else {
                &self.obstacle_map
            };
            route_single_net_with_config(
                search_obstacle_map,
                &self.primitives,
                source_state,
                target_state,
                Some(opened_search_ref),
                &search_cfg,
            )
        }
        .ok_or_else(|| "No route found".to_string())?;

        if collect_timing {
            result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
            result.stats.simple_route_time_us += simple_route_time_us;
        }
        let commit_prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let (route_cells, core_cells) = self.route_commit_and_core_cells(
            &result,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
        );
        if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
            result.stats.commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
        }
        let commit_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let allowed_partner_ids = FxHashSet::default();
        let committed = self
            .obstacle_map
            .commit_route_with_clearance_and_allowed_core_overlaps(
                net_id,
                &core_cells,
                &route_cells,
                clearance_exempt_cells.unwrap_or(&[]),
                &allowed_partner_ids,
            );
        if let Some(commit_start) = commit_start.as_ref() {
            result.stats.commit_time_us += commit_start.elapsed().as_micros();
        }
        if !committed {
            let error = self.dynamic_commit_rejection_error(
                net_id,
                &core_cells,
                clearance_exempt_cells.unwrap_or(&[]),
            );
            let retry_keepout = self.dynamic_commit_error_repair_keepout_keys(&error);
            if !retry_keepout.is_empty() {
                self.obstacle_map.add_static_keys(&retry_keepout);
                let retry_opened_cells: Vec<(i32, i32)> = validation_opened_cell_keys
                    .iter()
                    .copied()
                    .map(unpack_xy)
                    .collect();
                let retry_result = self.route_single_net_and_commit_repair_native(
                    net_id,
                    source,
                    target,
                    block_radius_cells,
                    Some(&retry_opened_cells),
                    Some(&validation_opened_cell_keys),
                    0.0,
                    commit_radius_cells,
                    clearance_exempt_cells,
                    clearance_exempt_cell_keys,
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                );
                self.obstacle_map.remove_static_keys(&retry_keepout);
                if retry_result.is_ok() {
                    return retry_result;
                }
            }
            return Err(error);
        }
        self.remove_crossing_events_for_net(net_id);
        self.register_geometric_crossing_events_for_route(net_id, &result);
        if let Err(error) = self.remember_committed_route_centerlines_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }
        self.remember_committed_route_opened_cells(net_id, Some(&validation_opened_cell_keys));
        self.add_crossing_spacing_history_for_route(net_id, &result);
        self.invalidate_meander_base_prefix();
        if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
            Some(&validation_opened_cell_keys),
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }

        Ok(result)
    }

    #[allow(clippy::too_many_arguments)]
    fn route_single_net_and_commit_repair_native(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: Option<&[(i32, i32)]>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        history_weight: f64,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<&[(i32, i32)]>,
        clearance_exempt_cell_keys: Option<&FxHashSet<CellKey>>,
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err("target_tolerance_cells must be >= 0".to_string());
        }
        let opened_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(keys) = opened_cell_keys {
            keys
        } else if let Some(cells) = opened_cells {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            &self.port_open_cells
        };
        let validation_opened_cell_keys = opened_ref.clone();
        let mut cfg = self.astar_config(Some(false), Some(false), Some(history_weight))?;
        cfg.require_terminal_straights = true;
        let dynamic_clearance_exempt_cell_vec = clearance_exempt_cells.unwrap_or(&[]);
        let collect_timing = self.astar_cfg.collect_detailed_timing;
        let prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let dynamic_clearance_exempt_keys_owned: Option<FxHashSet<CellKey>> = if block_radius_cells
            > 0
            && !dynamic_clearance_exempt_cell_vec.is_empty()
            && clearance_exempt_cell_keys.is_none()
        {
            Some(pack_cells(dynamic_clearance_exempt_cell_vec))
        } else {
            None
        };
        let dynamic_clearance_exempt_keys =
            if block_radius_cells > 0 && !dynamic_clearance_exempt_cell_vec.is_empty() {
                clearance_exempt_cell_keys.or(dynamic_clearance_exempt_keys_owned.as_ref())
            } else {
                None
            };
        let obstacle_map_prepare_time_us = prepare_start
            .as_ref()
            .map_or(0, |start| start.elapsed().as_micros());
        let source_state = State::new(source.x, source.y, source.angle);
        let target_state = State::new(target.x, target.y, target.angle);
        let opened_search_owned = self.opened_cells_without_dynamic_overlap(
            opened_ref,
            source_state,
            target_state,
        );
        let opened_search_ref = opened_search_owned.as_ref().unwrap_or(opened_ref);
        let expected_crossing_partner_ids = self.crossing_allowed_partner_set(net_id);
        let require_crossing_compliant_route = self.crossing_context.is_enabled()
            && !self.use_collision_crossing_routing
            && self.crossing_context.config().allow_only_expected_pairs
            && !expected_crossing_partner_ids.is_empty();
        let collision_partner_ids = if self.use_collision_crossing_routing {
            self.crossing_allowed_partner_set(net_id)
        } else {
            FxHashSet::default()
        };
        let used_collision_crossing_attempt =
            self.use_collision_crossing_routing && !collision_partner_ids.is_empty();
        let crossing_attempt = if used_collision_crossing_attempt {
            self.try_route_with_collision_crossings(
                net_id,
                source_state,
                target_state,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
                &collision_partner_ids,
                source_port_um,
                target_port_um,
                Some(&validation_opened_cell_keys),
            )
        } else {
            self.try_route_through_expected_crossing_partner(
                net_id,
                source_state,
                target_state,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
            )
        };
        if let Some((mut crossing_result, crossing_events)) = crossing_attempt {
            let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
            let allowed_crossing_core_keys =
                Self::crossing_reservation_keys_for_events(&crossing_events);
            let commit_prepare_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            let (route_cells, core_cells) = self.route_commit_and_core_cells(
                &crossing_result,
                block_radius_cells,
                commit_radius_cells,
                clearance_exempt_cells,
                core_radius_cells,
                source_port_um,
                target_port_um,
            );
            if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
                crossing_result.stats.commit_prepare_time_us +=
                    commit_prepare_start.elapsed().as_micros();
            }
            let commit_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            let committed = self
                .obstacle_map
                .commit_route_with_clearance_and_allowed_core_overlap_cells(
                    net_id,
                    &core_cells,
                    &route_cells,
                    clearance_exempt_cells.unwrap_or(&[]),
                    &crossed_partner_ids,
                    Some(&allowed_crossing_core_keys),
                );
            if let Some(commit_start) = commit_start.as_ref() {
                crossing_result.stats.commit_time_us += commit_start.elapsed().as_micros();
            }
            if committed {
                self.remove_crossing_events_for_net(net_id);
                self.add_crossing_events(crossing_events);
                if collect_timing {
                    crossing_result.stats.obstacle_map_prepare_time_us +=
                        obstacle_map_prepare_time_us;
                }
                if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                    net_id,
                    &crossing_result,
                    source_port_um,
                    target_port_um,
                ) {
                    self.rollback_committed_route(net_id);
                    return Err(error);
                }
                self.remember_committed_route_opened_cells(
                    net_id,
                    Some(&validation_opened_cell_keys),
                );
                self.add_crossing_spacing_history_for_route(net_id, &crossing_result);
                self.invalidate_meander_base_prefix();
                if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                    net_id,
                    &crossing_result,
                    source_port_um,
                    target_port_um,
                    Some(&validation_opened_cell_keys),
                ) {
                    self.rollback_committed_route(net_id);
                    return Err(error);
                }
                return Ok(crossing_result);
            }
        }
        // Keep collision-crossing as a preferred fast path, but fall back to
        // ordinary A* when the local legal crossing candidate is rejected.
        if require_crossing_compliant_route {
            return Err("No crossing-compliant route found".to_string());
        }
        let zero_radius_overlay =
            block_radius_cells <= 0 && dynamic_clearance_exempt_keys.is_some() && !cfg.enable_jps4;
        let mut opened_dynamic_obstacle_map;
        let mut result = if block_radius_cells > 0 || zero_radius_overlay {
            route_single_net_with_dynamic_expansion_config(
                &self.obstacle_map,
                &self.primitives,
                source_state,
                target_state,
                Some(opened_search_ref),
                &cfg,
                block_radius_cells.max(0),
                dynamic_clearance_exempt_keys,
            )
        } else {
            let search_obstacle_map = if dynamic_clearance_exempt_keys.is_some() {
                opened_dynamic_obstacle_map = self.obstacle_map.clone();
                opened_dynamic_obstacle_map
                    .clear_dynamic_clearance_in_cells(dynamic_clearance_exempt_cell_vec);
                &opened_dynamic_obstacle_map
            } else {
                &self.obstacle_map
            };
            route_single_net_with_config(
                search_obstacle_map,
                &self.primitives,
                source_state,
                target_state,
                Some(opened_search_ref),
                &cfg,
            )
        }
        .ok_or_else(|| "No route found".to_string())?;

        if collect_timing {
            result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
        }
        let commit_prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let (route_cells, core_cells) = self.route_commit_and_core_cells(
            &result,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
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
            clearance_exempt_cells.unwrap_or(&[]),
        );
        if let Some(commit_start) = commit_start.as_ref() {
            result.stats.commit_time_us += commit_start.elapsed().as_micros();
        }
        if !committed {
            return Err(self.dynamic_commit_rejection_error(
                net_id,
                &core_cells,
                clearance_exempt_cells.unwrap_or(&[]),
            ));
        }
        self.remove_crossing_events_for_net(net_id);
        self.register_geometric_crossing_events_for_route(net_id, &result);
        if let Err(error) = self.remember_committed_route_centerlines_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }
        self.remember_committed_route_opened_cells(net_id, Some(&validation_opened_cell_keys));
        self.add_crossing_spacing_history_for_route(net_id, &result);
        self.invalidate_meander_base_prefix();
        if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
            Some(&validation_opened_cell_keys),
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }

        Ok(result)
    }

    #[allow(clippy::too_many_arguments)]
    fn route_single_net_and_commit_native_with_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        clearance_exempt_cell_keys: &FxHashSet<CellKey>,
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        if repair_keepout.is_empty() {
            return self.route_single_net_and_commit_native(
                net_id,
                source,
                target,
                block_radius_cells,
                Some(opened_cells),
                Some(opened_cell_keys),
                commit_radius_cells,
                Some(clearance_exempt_cells),
                Some(clearance_exempt_cell_keys),
                core_radius_cells,
                source_port_um,
                target_port_um,
            );
        }
        let filtered_opened =
            opened_cells_excluding_keepout(opened_cells, repair_keepout, source, target);
        if filtered_opened.len() == opened_cells.len() {
            return self.route_single_net_and_commit_native(
                net_id,
                source,
                target,
                block_radius_cells,
                Some(opened_cells),
                Some(opened_cell_keys),
                commit_radius_cells,
                Some(clearance_exempt_cells),
                Some(clearance_exempt_cell_keys),
                core_radius_cells,
                source_port_um,
                target_port_um,
            );
        }
        let filtered_opened_keys = pack_cells(&filtered_opened);
        let result = self.route_single_net_and_commit_native(
            net_id,
            source,
            target,
            block_radius_cells,
            Some(&filtered_opened),
            Some(&filtered_opened_keys),
            commit_radius_cells,
            Some(clearance_exempt_cells),
            Some(clearance_exempt_cell_keys),
            core_radius_cells,
            source_port_um,
            target_port_um,
        );
        if matches!(result, Err(ref error) if error == "No route found") {
            self.route_single_net_and_commit_native(
                net_id,
                source,
                target,
                block_radius_cells,
                Some(opened_cells),
                Some(opened_cell_keys),
                commit_radius_cells,
                Some(clearance_exempt_cells),
                Some(clearance_exempt_cell_keys),
                core_radius_cells,
                source_port_um,
                target_port_um,
            )
        } else {
            result
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn route_single_net_and_commit_repair_native_with_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        history_weight: f64,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        clearance_exempt_cell_keys: &FxHashSet<CellKey>,
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        let mut active_keepout = repair_keepout.clone();
        let mut internally_added_keepout = FxHashSet::default();
        let mut feedback_attempts = 0usize;
        loop {
            let result = if active_keepout.is_empty() {
                self.route_single_net_and_commit_repair_native(
                    net_id,
                    source,
                    target,
                    block_radius_cells,
                    Some(opened_cells),
                    Some(opened_cell_keys),
                    history_weight,
                    commit_radius_cells,
                    Some(clearance_exempt_cells),
                    Some(clearance_exempt_cell_keys),
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                )
            } else {
                let filtered_opened =
                    opened_cells_excluding_keepout(opened_cells, &active_keepout, source, target);
                if filtered_opened.len() == opened_cells.len() {
                    self.route_single_net_and_commit_repair_native(
                        net_id,
                        source,
                        target,
                        block_radius_cells,
                        Some(opened_cells),
                        Some(opened_cell_keys),
                        history_weight,
                        commit_radius_cells,
                        Some(clearance_exempt_cells),
                        Some(clearance_exempt_cell_keys),
                        core_radius_cells,
                        source_port_um,
                        target_port_um,
                    )
                } else {
                    let filtered_opened_keys = pack_cells(&filtered_opened);
                    let result = self.route_single_net_and_commit_repair_native(
                        net_id,
                        source,
                        target,
                        block_radius_cells,
                        Some(&filtered_opened),
                        Some(&filtered_opened_keys),
                        history_weight,
                        commit_radius_cells,
                        Some(clearance_exempt_cells),
                        Some(clearance_exempt_cell_keys),
                        core_radius_cells,
                        source_port_um,
                        target_port_um,
                    );
                    if matches!(result, Err(ref error) if error == "No route found") {
                        self.route_single_net_and_commit_repair_native(
                            net_id,
                            source,
                            target,
                            block_radius_cells,
                            Some(opened_cells),
                            Some(opened_cell_keys),
                            history_weight,
                            commit_radius_cells,
                            Some(clearance_exempt_cells),
                            Some(clearance_exempt_cell_keys),
                            core_radius_cells,
                            source_port_um,
                            target_port_um,
                        )
                    } else {
                        result
                    }
                }
            };

            match result {
                Ok(route) => {
                    if !internally_added_keepout.is_empty() {
                        self.obstacle_map
                            .remove_static_keys(&internally_added_keepout);
                    }
                    return Ok(route);
                }
                Err(error) => {
                    if feedback_attempts >= 4 {
                        if !internally_added_keepout.is_empty() {
                            self.obstacle_map
                                .remove_static_keys(&internally_added_keepout);
                        }
                        return Err(error);
                    }
                    let error_keepout = self.crossing_error_repair_keepout_keys(&error);
                    let extra_keepout: FxHashSet<CellKey> = error_keepout
                        .into_iter()
                        .filter(|key| !active_keepout.contains(key))
                        .collect();
                    if extra_keepout.is_empty() {
                        if !internally_added_keepout.is_empty() {
                            self.obstacle_map
                                .remove_static_keys(&internally_added_keepout);
                        }
                        return Err(error);
                    }
                    self.obstacle_map.add_static_keys(&extra_keepout);
                    active_keepout.extend(extra_keepout.iter().copied());
                    internally_added_keepout.extend(extra_keepout);
                    feedback_attempts = feedback_attempts.saturating_add(1);
                }
            }
        }
    }

    fn orthogonal_repair_primitives(&self) -> PrimitiveLibrary {
        let primitives_per_angle: Vec<Vec<Primitive>> = (0u8..8u8)
            .map(|angle| {
                if angle % 2 != 0 {
                    return Vec::new();
                }
                self.primitives
                    .get_primitives_for_angle(angle)
                    .iter()
                    .filter(|primitive| match primitive.geometry {
                        PrimitiveGeometry::Straight { .. } => primitive.end_angle % 2 == 0,
                        PrimitiveGeometry::Bend { angle_delta, .. } => {
                            primitive.end_angle % 2 == 0 && angle_delta.unsigned_abs() == 2
                        }
                    })
                    .cloned()
                    .collect()
            })
            .collect();
        PrimitiveLibrary::new(primitives_per_angle, self.primitives.grid_size_um())
    }

    #[allow(clippy::too_many_arguments)]
    fn route_single_net_and_commit_orthogonal_native_with_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        let filtered_opened;
        let filtered_opened_keys;
        let (_, opened_keys_for_route) = if repair_keepout.is_empty() {
            (opened_cells, opened_cell_keys)
        } else {
            filtered_opened =
                opened_cells_excluding_keepout(opened_cells, repair_keepout, source, target);
            if filtered_opened.len() == opened_cells.len() {
                (opened_cells, opened_cell_keys)
            } else {
                filtered_opened_keys = pack_cells(&filtered_opened);
                (filtered_opened.as_slice(), &filtered_opened_keys)
            }
        };
        let mut cfg = self.astar_config(Some(false), Some(false), Some(0.0))?;
        cfg.require_terminal_straights = false;
        cfg.enable_simple_routes = false;
        cfg.enable_jps4 = false;
        let orthogonal_primitives = self.orthogonal_repair_primitives();
        let route = route_single_net_with_config(
            &self.obstacle_map,
            &orthogonal_primitives,
            State::new(source.x, source.y, source.angle),
            State::new(target.x, target.y, target.angle),
            Some(opened_keys_for_route),
            &cfg,
        )
        .ok_or_else(|| "No route found".to_string())?;
        if self.commit_native_route_with_clearance(
            net_id,
            &route,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
            Some(opened_keys_for_route),
        ) {
            Ok(route)
        } else {
            Err("Failed to commit orthogonal repair route".to_string())
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        clearance_exempt_cell_keys: &FxHashSet<CellKey>,
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        prefer_orthogonal: bool,
    ) -> Result<RouteResult, String> {
        if prefer_orthogonal {
            if let Ok(route) = self.route_single_net_and_commit_orthogonal_native_with_repair_keepout(
                net_id,
                source,
                target,
                block_radius_cells,
                opened_cells,
                opened_cell_keys,
                commit_radius_cells,
                clearance_exempt_cells,
                core_radius_cells,
                repair_keepout,
                source_port_um,
                target_port_um,
            ) {
                return Ok(route);
            }
        }
        self.route_single_net_and_commit_native_with_repair_keepout(
            net_id,
            source,
            target,
            block_radius_cells,
            opened_cells,
            opened_cell_keys,
            commit_radius_cells,
            clearance_exempt_cells,
            clearance_exempt_cell_keys,
            core_radius_cells,
            repair_keepout,
            source_port_um,
            target_port_um,
        )
    }

    fn route_single_net_ignore_dynamic_native(
        &self,
        source: PyState,
        target: PyState,
        opened_cells: Option<&[(i32, i32)]>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Result<RouteResult, String> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err("target_tolerance_cells must be >= 0".to_string());
        }
        let opened_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(keys) = opened_cell_keys {
            keys
        } else if let Some(cells) = opened_cells {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            &self.port_open_cells
        };
        let mut cfg = self.astar_config(Some(true), Some(false), Some(0.0))?;
        cfg.require_terminal_straights = true;
        let mut static_only_obstacle_map = self.obstacle_map.clone();
        static_only_obstacle_map.clear_dynamic();
        route_single_net_with_config(
            &static_only_obstacle_map,
            &self.primitives,
            State::new(source.x, source.y, source.angle),
            State::new(target.x, target.y, target.angle),
            Some(opened_ref),
            &cfg,
        )
        .ok_or_else(|| "No route found".to_string())
    }

    fn commit_native_route_with_clearance(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        self.commit_native_route_with_clearance_internal(
            net_id,
            route,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
            opened_cell_keys,
            true,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn commit_native_route_with_clearance_without_crossing_validation(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        self.commit_native_route_with_clearance_internal(
            net_id,
            route,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
            opened_cell_keys,
            false,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn commit_native_route_with_clearance_allowing_core_overlap(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        allowed_overlap_net_ids: &[u64],
        validate_crossings: bool,
    ) -> Result<bool, String> {
        let allowed_overlap_nets: FxHashSet<u64> = allowed_overlap_net_ids
            .iter()
            .copied()
            .filter(|owner| *owner != net_id)
            .collect();
        if allowed_overlap_nets.is_empty() {
            return Ok(self.commit_native_route_with_clearance_without_crossing_validation(
                net_id,
                route,
                block_radius_cells,
                commit_radius_cells,
                clearance_exempt_cells,
                core_radius_cells,
                source_port_um,
                target_port_um,
                opened_cell_keys,
            ));
        }

        let (route_cells, core_cells) = self.route_commit_and_core_cells(
            route,
            block_radius_cells,
            commit_radius_cells,
            Some(clearance_exempt_cells),
            core_radius_cells,
            source_port_um,
            target_port_um,
        );
        let mut allowed_overlap_core_keys = FxHashSet::default();
        for &(x, y) in &core_cells {
            let owners = self.obstacle_map.dynamic_owners_at(x, y);
            if owners.is_empty() {
                continue;
            }
            let other_owners: Vec<u64> = owners
                .into_iter()
                .filter(|owner| *owner != net_id)
                .collect();
            if other_owners.is_empty() {
                continue;
            }
            if !other_owners
                .iter()
                .all(|owner| allowed_overlap_nets.contains(owner))
            {
                return Ok(false);
            }
            allowed_overlap_core_keys.insert(pack_xy(x, y));
        }

        let committed = self
            .obstacle_map
            .commit_route_with_clearance_and_allowed_core_overlap_cells(
                net_id,
                &core_cells,
                &route_cells,
                clearance_exempt_cells,
                &allowed_overlap_nets,
                Some(&allowed_overlap_core_keys),
            );
        if committed {
            self.remove_crossing_events_for_net(net_id);
            self.register_geometric_crossing_events_for_route(net_id, route);
            if self
                .remember_committed_route_centerlines_with_ports(
                    net_id,
                    route,
                    source_port_um,
                    target_port_um,
                )
                .is_err()
            {
                self.rollback_committed_route(net_id);
                return Err("Failed to record committed route centerline".to_string());
            }
            self.remember_committed_route_opened_cells(net_id, opened_cell_keys);
            self.add_crossing_spacing_history_for_route(net_id, route);
            self.invalidate_meander_base_prefix();
            if validate_crossings {
                if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                    net_id,
                    route,
                    source_port_um,
                    target_port_um,
                    opened_cell_keys,
                ) {
                    self.rollback_committed_route(net_id);
                    return Err(error);
                }
            }
        }
        Ok(committed)
    }

    #[allow(clippy::too_many_arguments)]
    fn commit_native_route_with_clearance_internal(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        validate_crossings: bool,
    ) -> bool {
        let (route_cells, core_cells) = self.route_commit_and_core_cells(
            route,
            block_radius_cells,
            commit_radius_cells,
            Some(clearance_exempt_cells),
            core_radius_cells,
            source_port_um,
            target_port_um,
        );
        let committed = self.obstacle_map.commit_route_with_clearance_overlap(
            net_id,
            &core_cells,
            &route_cells,
            clearance_exempt_cells,
        );
        if committed {
            self.remove_crossing_events_for_net(net_id);
            self.register_geometric_crossing_events_for_route(net_id, route);
            if self
                .remember_committed_route_centerlines_with_ports(
                    net_id,
                    route,
                    source_port_um,
                    target_port_um,
                )
                .is_err()
            {
                self.rollback_committed_route(net_id);
                return false;
            }
            self.remember_committed_route_opened_cells(net_id, opened_cell_keys);
            self.add_crossing_spacing_history_for_route(net_id, route);
            self.invalidate_meander_base_prefix();
            if validate_crossings
                && self
                .validate_committed_crossings_for_route_with_ports(
                    net_id,
                    route,
                    source_port_um,
                    target_port_um,
                    opened_cell_keys,
                )
                .is_err()
            {
                self.rollback_committed_route(net_id);
                return false;
            }
        }
        committed
    }

    fn dynamic_owners_for_native_route(
        &self,
        route: &RouteResult,
        block_radius_cells: i32,
    ) -> Vec<u64> {
        let cells = inflate_route_cells(
            &route.cells,
            block_radius_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        let mut owners: Vec<u64> = self
            .obstacle_map
            .dynamic_owners_for_cells(&cells)
            .into_iter()
            .collect();
        owners.sort_unstable();
        owners
    }

    fn add_history_for_native_route(
        &mut self,
        route: &RouteResult,
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

    #[allow(clippy::too_many_arguments)]
    fn route_port_corrected_centerline_checked_and_commit_native(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        width_um: f64,
        core_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        allow_unchecked_fallback: bool,
    ) -> Result<NativeEndpointCorrection, String> {
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| err.to_string())?;
        let static_grid = static_grid_from_py_grid(&self.grid);
        let width = self.grid.width as i32;
        let height = self.grid.height as i32;
        let opened_keys = pack_cells(opened_cells);
        let clearance_exempt_keys = pack_cells(clearance_exempt_cells);
        let candidates = full_straight_offset_bump_candidates_rs(
            route,
            &self.primitives,
            &grid,
            source_port_um,
            target_port_um,
        )
        .map_err(|err| err.to_string())?;

        if candidates.is_empty() {
            let centerline = route_to_port_corrected_centerline_with_options_rs(
                route,
                &self.primitives,
                &grid,
                source_port_um,
                target_port_um,
                allow_unchecked_fallback,
            )
            .map_err(|err| err.to_string())?;
            let corrected_core_cells = centerline_core_cells(&centerline, width_um, &static_grid)
                .map_err(|err| err.to_string())?;
            if corrected_core_cells.is_empty() {
                return Ok(NativeEndpointCorrection {
                    centerline,
                    committed_bump: false,
                    candidate_index: None,
                    candidate_label: None,
                });
            }
            let local_endpoint_open_keys = endpoint_contact_open_keys(
                &corrected_core_cells,
                &grid,
                source_port_um,
                target_port_um,
                width_um,
            );
            let out_of_bounds: Vec<(i32, i32)> = corrected_core_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let static_blockers: Vec<(i32, i32)> = corrected_core_cells
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
            if !out_of_bounds.is_empty() || !static_blockers.is_empty() {
                return Err(format!(
                    "Endpoint correction commit rejected: out_of_bounds={} out_of_bounds_bbox={} static_overlap={} static_bbox={} core_cells={} core_bbox={}",
                    out_of_bounds.len(),
                    format_bbox(&out_of_bounds),
                    static_blockers.len(),
                    format_bbox(&static_blockers),
                    corrected_core_cells.len(),
                    format_bbox(&corrected_core_cells),
                ));
            }
            let mut allowed_overlap_nets = FxHashSet::default();
            let mut allowed_overlap_core_keys = FxHashSet::default();
            for &(x, y) in &corrected_core_cells {
                let key = pack_xy(x, y);
                let other_owners: Vec<u64> = self
                    .obstacle_map
                    .dynamic_owners_at(x, y)
                    .into_iter()
                    .filter(|owner| *owner != net_id)
                    .collect();
                if other_owners.is_empty() {
                    continue;
                }
                allowed_overlap_core_keys.insert(key);
                for owner in other_owners {
                    allowed_overlap_nets.insert(owner);
                }
            }
            let corrected_blocked_cells =
                inflate_route_cells(&corrected_core_cells, core_radius_cells, width, height);
            if !self
                .obstacle_map
                .commit_route_with_clearance_and_allowed_core_overlap_cells(
                    net_id,
                    &corrected_core_cells,
                    &corrected_blocked_cells,
                    clearance_exempt_cells,
                    &allowed_overlap_nets,
                    Some(&allowed_overlap_core_keys),
                )
            {
                let commit_dynamic_blockers = cells_with_other_dynamic_owner(
                    &self.obstacle_map,
                    &corrected_core_cells,
                    &clearance_exempt_keys,
                    net_id,
                );
                let owners = sorted_other_owners_for_cells(
                    &self.obstacle_map,
                    &commit_dynamic_blockers,
                    net_id,
                );
                return Err(format!(
                    "Endpoint correction commit rejected after validation: dynamic_overlap={} owners={owners:?} dynamic_bbox={} core_cells={} core_bbox={}",
                    commit_dynamic_blockers.len(),
                    format_bbox(&commit_dynamic_blockers),
                    corrected_core_cells.len(),
                    format_bbox(&corrected_core_cells),
                ));
            }
            self.remove_crossing_events_for_net(net_id);
            self.register_geometric_crossing_events_for_route(net_id, route);
            if self
                .remember_committed_route_centerlines_with_ports(
                    net_id,
                    route,
                    source_port_um,
                    target_port_um,
                )
                .is_err()
            {
                self.rollback_committed_route(net_id);
                return Err("Failed to record endpoint-corrected route centerline".to_string());
            }
            self.remember_committed_route_opened_cells(net_id, Some(&opened_keys));
            self.add_crossing_spacing_history_for_route(net_id, route);
            if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                net_id,
                route,
                source_port_um,
                target_port_um,
                Some(&opened_keys),
            ) {
                self.rollback_committed_route(net_id);
                return Err(error);
            }
            self.invalidate_meander_base_prefix();
            return Ok(NativeEndpointCorrection {
                centerline,
                committed_bump: false,
                candidate_index: None,
                candidate_label: None,
            });
        }

        let old_blocked_cells: Vec<(i32, i32)> = self
            .obstacle_map
            .get_net_cells(net_id)
            .map(|cells| cells.iter().copied().map(unpack_xy).collect())
            .unwrap_or_default();
        let old_core_cells = route_core_cells(&route.cells, core_radius_cells, width, height);
        let commit_clearance_exempt_cell_vec = unique_cells(
            clearance_exempt_cells
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
                    .map_err(|err| err.to_string())?;
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
                self.remove_crossing_events_for_net(net_id);
                self.register_geometric_crossing_events_for_route(net_id, route);
                if self
                    .remember_committed_route_centerlines_with_ports(
                        net_id,
                        route,
                        source_port_um,
                        target_port_um,
                    )
                    .is_err()
                {
                    self.rollback_committed_route(net_id);
                    return Err(
                        "Failed to record endpoint-corrected bump route centerline".to_string()
                    );
                }
                self.remember_committed_route_opened_cells(net_id, Some(&opened_keys));
                self.add_crossing_spacing_history_for_route(net_id, route);
                if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                    net_id,
                    route,
                    source_port_um,
                    target_port_um,
                    Some(&opened_keys),
                ) {
                    self.rollback_committed_route(net_id);
                    return Err(error);
                }
                self.invalidate_meander_base_prefix();
                return Ok(NativeEndpointCorrection {
                    centerline,
                    committed_bump: true,
                    candidate_index: Some(candidate_index),
                    candidate_label: Some(candidate_label),
                });
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

        Err(format!(
            "No collision-free port endpoint case-4 bump placement found; candidates: {}",
            rejection_details.join("; ")
        ))
    }
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
        let astar_cfg_cached =
            astar_config_from_py(&astar_config, &primitive_config, None, None, None)
                .map_err(|err| err.to_string());
        Self {
            obstacle_map: ObstacleMap::new(grid_spec.width as i32, grid_spec.height as i32),
            grid: grid_spec,
            primitive_cfg: primitive_config,
            astar_cfg: astar_config,
            astar_cfg_cached,
            primitives,
            crossing_context: CrossingContext::default(),
            committed_center_routes: FxHashMap::default(),
            committed_realized_center_routes: FxHashMap::default(),
            committed_opened_cell_keys: FxHashMap::default(),
            crossing_events: Vec::new(),
            use_collision_crossing_routing: false,
            static_cells: FxHashSet::default(),
            port_open_cells: FxHashSet::default(),
            registered_plm: RefCell::new(RegisteredPlmContext::default()),
            last_meander_registration_profile: RefCell::new(None),
        }
    }

    fn crossing_config(&self) -> PyCrossingConfig {
        PyCrossingConfig::from(self.crossing_context.config())
    }

    fn set_crossing_config(&mut self, config: PyCrossingConfig) -> PyResult<()> {
        validate_crossing_config(&config)?;
        self.crossing_context
            .set_config(CrossingConfig::from(&config));
        Ok(())
    }

    fn crossing_constraints(&self) -> Vec<PyCrossingConstraint> {
        self.crossing_context
            .constraints()
            .iter()
            .map(PyCrossingConstraint::from)
            .collect()
    }

    fn set_crossing_constraints(&mut self, constraints: Vec<PyCrossingConstraint>) -> PyResult<()> {
        for constraint in &constraints {
            validate_crossing_constraint(constraint)?;
        }
        self.crossing_context
            .replace_constraints(constraints.iter().map(CrossingConstraint::from).collect());
        Ok(())
    }

    fn clear_crossing_constraints(&mut self) {
        self.crossing_context.clear_constraints();
    }

    fn crossing_expected_count(&self, net_id: u64) -> u32 {
        self.crossing_context.expected_crossing_count(net_id)
    }

    fn crossing_has_expected_pair(&self, net_id: u64, partner_net_id: u64) -> bool {
        self.crossing_context
            .has_expected_pair(net_id, partner_net_id)
    }

    fn crossing_allows_pair(&self, net_id: u64, partner_net_id: u64) -> bool {
        self.crossing_context.allows_pair(net_id, partner_net_id)
    }

    fn set_collision_crossing_routing(&mut self, enabled: bool) {
        self.use_collision_crossing_routing = enabled;
    }

    fn crossing_events(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        let mut out = Vec::with_capacity(self.crossing_events.len());
        for event in &self.crossing_events {
            let d = PyDict::new_bound(py);
            let mut reservation_cells: Vec<(i32, i32)> = event
                .reservation_keys
                .iter()
                .copied()
                .map(unpack_xy)
                .collect();
            reservation_cells.sort_unstable();
            d.set_item("net_id", event.net_id)?;
            d.set_item("partner_net_id", event.partner_net_id)?;
            d.set_item("point", event.point)?;
            d.set_item("route_segment", event.route_segment)?;
            d.set_item("partner_segment", event.partner_segment)?;
            d.set_item("route_angle", event.route_angle)?;
            d.set_item("partner_angle", event.partner_angle)?;
            d.set_item("reservation_cells", reservation_cells)?;
            out.push(d.into());
        }
        Ok(out)
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
        self.committed_center_routes.clear();
        self.committed_realized_center_routes.clear();
        self.committed_opened_cell_keys.clear();
        self.crossing_events.clear();
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
        self.committed_center_routes.clear();
        self.committed_realized_center_routes.clear();
        self.committed_opened_cell_keys.clear();
        self.crossing_events.clear();
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
        self.committed_center_routes.clear();
        self.committed_realized_center_routes.clear();
        self.committed_opened_cell_keys.clear();
        self.crossing_events.clear();
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

    #[pyo3(signature=(
        ports,
        raw_static_cells=None,
        raw_static_rects=None,
        route_clearance_um=0.0,
        port_open_radius_um=0.5,
        bend_radius_cells=2,
        commit_radius_cells=0,
        port_entry_length_cells=4,
        port_entry_half_width_cells=1,
        port_lane_length_cells=6,
        port_lane_half_width_cells=1
    ))]
    fn build_route_port_openings(
        &self,
        ports: Vec<(
            String,
            f64,
            f64,
            Option<f64>,
            Option<String>,
            Option<f64>,
            Option<f64>,
        )>,
        raw_static_cells: Option<Vec<(i32, i32)>>,
        raw_static_rects: Option<Vec<(i32, i32, i32, i32)>>,
        route_clearance_um: f64,
        port_open_radius_um: f64,
        bend_radius_cells: i32,
        commit_radius_cells: i32,
        port_entry_length_cells: i32,
        port_entry_half_width_cells: i32,
        port_lane_length_cells: i32,
        port_lane_half_width_cells: i32,
    ) -> Vec<(String, Vec<(i32, i32)>, Vec<(i32, i32)>, Vec<(i32, i32)>)> {
        let grid = StaticGridSpec {
            width: self.grid.width as i32,
            height: self.grid.height as i32,
            grid_size_um: self.grid.grid_size_um,
            origin: (self.grid.origin_x_um, self.grid.origin_y_um),
            die_bbox: (0.0, 0.0, 0.0, 0.0),
        };
        let raw_static_keys: FxHashSet<CellKey> = raw_static_cells
            .unwrap_or_default()
            .into_iter()
            .map(|(x, y)| pack_xy(x, y))
            .collect();
        let raw_rects: Vec<(i32, i32, i32, i32)> = raw_static_rects
            .unwrap_or_default()
            .into_iter()
            .map(|(x0, y0, x1, y1)| (x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1)))
            .collect();
        let port_open_radius_cells =
            (port_open_radius_um.max(0.0) / self.grid.grid_size_um).ceil() as i32;
        let runway_length_cells = bend_radius_cells.max(0) + 1;
        let runway_half_width_cells = commit_radius_cells.max(0);

        ports
            .into_iter()
            .map(
                |(spec, x_um, y_um, orientation, port_type, access_length_um, access_width_um)| {
                    let is_custom_access = access_length_um.is_some() || access_width_um.is_some();
                    if !is_custom_access
                        && port_type.as_deref().is_some_and(|kind| kind != "optical")
                    {
                        return (spec, Vec::new(), Vec::new(), Vec::new());
                    }

                    let candidate_cells = route_port_access_cells(
                        &grid,
                        x_um,
                        y_um,
                        orientation,
                        access_length_um,
                        access_width_um,
                        port_entry_length_cells,
                        port_entry_half_width_cells,
                        port_lane_length_cells,
                        port_lane_half_width_cells,
                    );
                    if is_custom_access {
                        let cells = sorted_cells(candidate_cells);
                        return (spec, cells.clone(), cells, Vec::new());
                    }

                    let base_open_cells =
                        route_base_port_open_cells(&grid, x_um, y_um, port_open_radius_cells);
                    let runway_cells = route_port_runway_cells(
                        &grid,
                        x_um,
                        y_um,
                        orientation,
                        runway_length_cells,
                        runway_half_width_cells,
                    );
                    let runway_open_cells: FxHashSet<CellKey> = runway_cells
                        .iter()
                        .copied()
                        .filter(|key| !route_cell_in_raw_static(*key, &raw_static_keys, &raw_rects))
                        .collect();

                    let mut candidate_and_runway = candidate_cells.clone();
                    candidate_and_runway.extend(runway_cells.iter().copied());

                    let effective_cells: FxHashSet<CellKey> = if route_clearance_um <= 0.0 {
                        candidate_cells
                            .intersection(&base_open_cells)
                            .copied()
                            .chain(runway_open_cells)
                            .collect()
                    } else {
                        candidate_cells
                            .iter()
                            .copied()
                            .filter(|key| {
                                !route_cell_in_raw_static(*key, &raw_static_keys, &raw_rects)
                            })
                            .chain(candidate_cells.intersection(&base_open_cells).copied())
                            .chain(runway_open_cells)
                            .collect()
                    };

                    (
                        spec,
                        sorted_cells(effective_cells),
                        sorted_cells(candidate_and_runway),
                        sorted_cells(runway_cells),
                    )
                },
            )
            .collect()
    }

    #[pyo3(signature=(jobs,allow_45_degree_turns,bend_radius_cells,commit_radius_cells))]
    fn build_dynamic_clearance_exempt_cells_for_routes(
        &self,
        jobs: Vec<(u64, PyState, PyState)>,
        allow_45_degree_turns: bool,
        bend_radius_cells: i32,
        commit_radius_cells: i32,
    ) -> Vec<(u64, Vec<(i32, i32)>)> {
        let width = self.grid.width as i32;
        let height = self.grid.height as i32;
        jobs.into_iter()
            .map(|(net_id, source, target)| {
                let cells = route_dynamic_clearance_exempt_cells(
                    source,
                    target,
                    allow_45_degree_turns,
                    bend_radius_cells,
                    commit_radius_cells,
                    width,
                    height,
                );
                (net_id, sorted_cells(cells))
            })
            .collect()
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
        let cfg = self
            .astar_config(None, None, None)
            .map_err(PyValueError::new_err)?;
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
        let result = self
            .route_single_net_and_commit_native(
                net_id,
                source,
                target,
                block_radius_cells,
                opened_cells.as_deref(),
                None,
                commit_radius_cells,
                clearance_exempt_cells.as_deref(),
                None,
                core_radius_cells,
                None,
                None,
            )
            .map_err(PyRuntimeError::new_err)?;
        Py::new(py, convert_result(py, &self.primitives, &result)?)
    }

    #[pyo3(signature=(jobs,block_radius_cells=0,commit_radius_cells=None,core_radius_cells=None))]
    fn route_many_normal_and_commit(
        &mut self,
        py: Python<'_>,
        jobs: Vec<(
            u64,
            PyState,
            PyState,
            Vec<(i32, i32)>,
            Vec<(i32, i32)>,
            Option<(f64, f64)>,
            Option<(f64, f64)>,
        )>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
    ) -> PyResult<PyObject> {
        let collect_native_timing = self.astar_cfg.collect_detailed_timing;
        let mut timings = NativeBatchTimings::default();
        let unpack_start = native_batch_timer(collect_native_timing);
        let native_jobs: Vec<NativeRouteJob> = jobs
            .into_iter()
            .map(
                |(
                    net_id,
                    source,
                    target,
                    opened_cells,
                    clearance_exempt_cells,
                    source_port_um,
                    target_port_um,
                )| {
                    NativeRouteJob::new(
                        net_id,
                        source,
                        target,
                        opened_cells,
                        clearance_exempt_cells,
                        source_port_um,
                        target_port_um,
                    )
                },
            )
            .collect();
        timings.route_job_unpack_us += native_batch_elapsed_us(unpack_start);
        let result_dict = PyDict::new_bound(py);
        let route_entries = PyList::empty_bound(py);
        for job in &native_jobs {
            let route_start = native_batch_timer(collect_native_timing);
            let route_result = self.route_single_net_and_commit_native(
                job.net_id,
                job.source.clone(),
                job.target.clone(),
                block_radius_cells,
                Some(&job.opened_cells),
                Some(&job.opened_cell_keys),
                commit_radius_cells,
                Some(&job.clearance_exempt_cells),
                Some(&job.clearance_exempt_cell_keys),
                core_radius_cells,
                job.source_port_um,
                job.target_port_um,
            );
            let route_elapsed_us = native_batch_elapsed_us(route_start);
            timings.normal_route_wall_us += route_elapsed_us;
            match route_result {
                Ok(route_result) => {
                    timings.add_route_result_stats_if(collect_native_timing, &route_result);
                    let entry = PyDict::new_bound(py);
                    let route_construct_start = native_batch_timer(collect_native_timing);
                    let route_obj =
                        Py::new(py, convert_result(py, &self.primitives, &route_result)?)?;
                    timings.route_result_construction_us +=
                        native_batch_elapsed_us(route_construct_start);
                    let dict_start = native_batch_timer(collect_native_timing);
                    entry.set_item("net_id", job.net_id)?;
                    entry.set_item("route", route_obj)?;
                    route_entries.append(entry)?;
                    timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
                }
                Err(error) => {
                    timings.normal_route_failed_wall_us += route_elapsed_us;
                    let dict_start = native_batch_timer(collect_native_timing);
                    result_dict.set_item("status", "failed")?;
                    result_dict.set_item("failed_net_id", job.net_id)?;
                    result_dict.set_item("error", error)?;
                    result_dict.set_item("routes", route_entries)?;
                    timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
                    result_dict
                        .set_item("timings_s", native_batch_timings_to_py_dict(py, &timings)?)?;
                    return Ok(result_dict.into());
                }
            }
        }
        let dict_start = native_batch_timer(collect_native_timing);
        result_dict.set_item("status", "routed")?;
        result_dict.set_item("failed_net_id", py.None())?;
        result_dict.set_item("error", py.None())?;
        result_dict.set_item("routes", route_entries)?;
        timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
        result_dict.set_item("timings_s", native_batch_timings_to_py_dict(py, &timings)?)?;
        Ok(result_dict.into())
    }

    #[pyo3(signature=(jobs,block_radius_cells=0,commit_radius_cells=None,core_radius_cells=None,max_rounds=4,max_victims_per_failure=8,history_weight=2.0,history_increment=1))]
    #[allow(clippy::too_many_arguments)]
    fn route_many_with_repair_and_commit(
        &mut self,
        py: Python<'_>,
        jobs: Vec<(
            u64,
            PyState,
            PyState,
            Vec<(i32, i32)>,
            Vec<(i32, i32)>,
            Option<(f64, f64)>,
            Option<(f64, f64)>,
        )>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        max_rounds: u32,
        max_victims_per_failure: usize,
        history_weight: f64,
        history_increment: u32,
    ) -> PyResult<PyObject> {
        let collect_native_timing = self.astar_cfg.collect_detailed_timing;
        let mut timings = NativeBatchTimings::default();
        let unpack_start = native_batch_timer(collect_native_timing);
        let native_jobs: Vec<NativeRouteJob> = jobs
            .into_iter()
            .map(
                |(
                    net_id,
                    source,
                    target,
                    opened_cells,
                    clearance_exempt_cells,
                    source_port_um,
                    target_port_um,
                )| {
                    NativeRouteJob::new(
                        net_id,
                        source,
                        target,
                        opened_cells,
                        clearance_exempt_cells,
                        source_port_um,
                        target_port_um,
                    )
                },
            )
            .collect();
        timings.route_job_unpack_us += native_batch_elapsed_us(unpack_start);
        let order_by_id: FxHashMap<u64, usize> = native_jobs
            .iter()
            .enumerate()
            .map(|(index, job)| (job.net_id, index))
            .collect();
        let job_by_id: FxHashMap<u64, NativeRouteJob> = native_jobs
            .iter()
            .cloned()
            .map(|job| (job.net_id, job))
            .collect();
        let mut final_routes: FxHashMap<u64, RouteResult> = FxHashMap::default();
        let mut attempts: Vec<NativeRouteAttempt> = Vec::new();
        let mut repair_trace: Vec<NativeRepairTraceEvent> = Vec::new();
        let mut repair_count = 0u32;
        let mut failed_net_id: Option<u64> = None;
        let mut failed_error: Option<String> = None;
        let trace_native_progress = std::env::var_os("PHOTONIC_ROUTER_NATIVE_PROGRESS").is_some();
        let trace_native_repair = std::env::var_os("PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG").is_some();
        let mut trace_last_route_start: Option<Instant> = None;

        'route_jobs: for (job_index, job) in native_jobs.iter().enumerate() {
            if trace_native_progress {
                let now = Instant::now();
                if let Some(last_start) = trace_last_route_start.replace(now) {
                    eprintln!(
                        "native_route_elapsed previous_index={} elapsed_s={:.6}",
                        job_index,
                        last_start.elapsed().as_secs_f64()
                    );
                }
                eprintln!(
                    "native_route_start index={} net_id={} source=({}, {}, {}) target=({}, {}, {})",
                    job_index + 1,
                    job.net_id,
                    job.source.x,
                    job.source.y,
                    job.source.angle,
                    job.target.x,
                    job.target.y,
                    job.target.angle
                );
            }
            let preemptive_crossing_victims =
                self.crossing_local_ripup_candidates(job.net_id, max_victims_per_failure.min(4));
            if !preemptive_crossing_victims.is_empty() {
                let base_map = self.obstacle_map.clone();
                let base_center_routes = self.committed_center_routes.clone();
                let base_realized_center_routes = self.committed_realized_center_routes.clone();
                let base_opened_cell_keys = self.committed_opened_cell_keys.clone();
                let base_crossing_events = self.crossing_events.clone();
                let base_routes = final_routes.clone();

                for victim_id in preemptive_crossing_victims {
                    let Some(victim_job) = job_by_id.get(&victim_id) else {
                        continue;
                    };

                    self.obstacle_map = base_map.clone();
                    self.committed_center_routes = base_center_routes.clone();
                    self.committed_realized_center_routes = base_realized_center_routes.clone();
                    self.committed_opened_cell_keys = base_opened_cell_keys.clone();
                    self.crossing_events = base_crossing_events.clone();
                    final_routes = base_routes.clone();
                    self.invalidate_meander_base_prefix();

                    if let Some(old_route) = final_routes.get(&victim_id).cloned() {
                        let history_start = native_batch_timer(collect_native_timing);
                        self.add_history_for_native_route(
                            &old_route,
                            block_radius_cells,
                            history_increment,
                        );
                        timings.history_update_us += native_batch_elapsed_us(history_start);
                    }
                    let ripup_start = native_batch_timer(collect_native_timing);
                    self.remove_crossing_events_for_net(victim_id);
                    self.obstacle_map.ripup_route(victim_id);
                    self.committed_center_routes.remove(&victim_id);
                    self.committed_realized_center_routes.remove(&victim_id);
                    self.committed_opened_cell_keys.remove(&victim_id);
                    final_routes.remove(&victim_id);
                    timings.ripup_us += native_batch_elapsed_us(ripup_start);

                    let route_start = native_batch_timer(collect_native_timing);
                    let route_result = self.route_single_net_and_commit_native(
                        job.net_id,
                        job.source,
                        job.target,
                        block_radius_cells,
                        Some(&job.opened_cells),
                        Some(&job.opened_cell_keys),
                        commit_radius_cells,
                        Some(&job.clearance_exempt_cells),
                        Some(&job.clearance_exempt_cell_keys),
                        core_radius_cells,
                        job.source_port_um,
                        job.target_port_um,
                    );
                    let route_elapsed_us = native_batch_elapsed_us(route_start);
                    timings.repair_failed_net_wall_us += route_elapsed_us;
                    let current_route = match route_result {
                        Ok(route) => {
                            timings.add_route_result_stats_if(collect_native_timing, &route);
                            route
                        }
                        Err(error) => {
                            timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                            attempts.push(NativeRouteAttempt {
                                bucket_name: "preemptive_crossing_ripup",
                                net_id: job.net_id,
                                route: None,
                                failed: true,
                                error: Some(error),
                                repair_round: Some(0),
                                candidate_blockers: vec![victim_id],
                                ripup_ids: vec![victim_id],
                            });
                            continue;
                        }
                    };

                    let reroute_start = native_batch_timer(collect_native_timing);
                    let reroute_result = self.route_single_net_and_commit_native(
                        victim_job.net_id,
                        victim_job.source,
                        victim_job.target,
                        block_radius_cells,
                        Some(&victim_job.opened_cells),
                        Some(&victim_job.opened_cell_keys),
                        commit_radius_cells,
                        Some(&victim_job.clearance_exempt_cells),
                        Some(&victim_job.clearance_exempt_cell_keys),
                        core_radius_cells,
                        victim_job.source_port_um,
                        victim_job.target_port_um,
                    );
                    let reroute_elapsed_us = native_batch_elapsed_us(reroute_start);
                    timings.reroute_victims_wall_us += reroute_elapsed_us;
                    let victim_route = match reroute_result {
                        Ok(route) => {
                            timings.add_route_result_stats_if(collect_native_timing, &route);
                            route
                        }
                        Err(normal_error) => {
                            timings.reroute_victims_failed_wall_us += reroute_elapsed_us;
                            let repair_start = native_batch_timer(collect_native_timing);
                            let repair_result = self.route_single_net_and_commit_repair_native(
                                victim_job.net_id,
                                victim_job.source,
                                victim_job.target,
                                block_radius_cells,
                                Some(&victim_job.opened_cells),
                                Some(&victim_job.opened_cell_keys),
                                history_weight,
                                commit_radius_cells,
                                Some(&victim_job.clearance_exempt_cells),
                                Some(&victim_job.clearance_exempt_cell_keys),
                                core_radius_cells,
                                victim_job.source_port_um,
                                victim_job.target_port_um,
                            );
                            let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                            timings.reroute_victims_wall_us += repair_elapsed_us;
                            match repair_result {
                                Ok(route) => {
                                    timings
                                        .add_route_result_stats_if(collect_native_timing, &route);
                                    route
                                }
                                Err(error) => {
                                    timings.reroute_victims_failed_wall_us += repair_elapsed_us;
                                    attempts.push(NativeRouteAttempt {
                                        bucket_name: "preemptive_crossing_ripup",
                                        net_id: victim_job.net_id,
                                        route: None,
                                        failed: true,
                                        error: Some(format!(
                                            "{normal_error}; repair fallback: {error}"
                                        )),
                                        repair_round: Some(0),
                                        candidate_blockers: vec![victim_id],
                                        ripup_ids: vec![victim_id],
                                    });
                                    continue;
                                }
                            }
                        }
                    };

                    attempts.push(NativeRouteAttempt {
                        bucket_name: "preemptive_crossing_ripup",
                        net_id: job.net_id,
                        route: Some(current_route.clone()),
                        failed: false,
                        error: None,
                        repair_round: Some(0),
                        candidate_blockers: vec![victim_id],
                        ripup_ids: vec![victim_id],
                    });
                    attempts.push(NativeRouteAttempt {
                        bucket_name: "preemptive_crossing_ripup",
                        net_id: victim_job.net_id,
                        route: Some(victim_route.clone()),
                        failed: false,
                        error: None,
                        repair_round: Some(0),
                        candidate_blockers: vec![victim_id],
                        ripup_ids: vec![victim_id],
                    });
                    final_routes.insert(job.net_id, current_route);
                    final_routes.insert(victim_job.net_id, victim_route);
                    repair_count = repair_count.saturating_add(1);
                    continue 'route_jobs;
                }

                self.obstacle_map = base_map;
                self.committed_center_routes = base_center_routes;
                self.committed_realized_center_routes = base_realized_center_routes;
                self.committed_opened_cell_keys = base_opened_cell_keys;
                self.crossing_events = base_crossing_events;
                final_routes = base_routes;
                self.invalidate_meander_base_prefix();
            }

            let route_start = native_batch_timer(collect_native_timing);
            let route_result = self.route_single_net_and_commit_native(
                job.net_id,
                job.source,
                job.target,
                block_radius_cells,
                Some(&job.opened_cells),
                Some(&job.opened_cell_keys),
                commit_radius_cells,
                Some(&job.clearance_exempt_cells),
                Some(&job.clearance_exempt_cell_keys),
                core_radius_cells,
                job.source_port_um,
                job.target_port_um,
            );
            let route_elapsed_us = native_batch_elapsed_us(route_start);
            timings.normal_route_wall_us += route_elapsed_us;
            match route_result {
                Ok(route) => {
                    timings.add_route_result_stats_if(collect_native_timing, &route);
                    attempts.push(NativeRouteAttempt {
                        bucket_name: "normal_route",
                        net_id: job.net_id,
                        route: Some(route.clone()),
                        failed: false,
                        error: None,
                        repair_round: None,
                        candidate_blockers: Vec::new(),
                        ripup_ids: Vec::new(),
                    });
                    final_routes.insert(job.net_id, route);
                    continue;
                }
                Err(error) => {
                    timings.normal_route_failed_wall_us += route_elapsed_us;
                    attempts.push(NativeRouteAttempt {
                        bucket_name: "normal_route",
                        net_id: job.net_id,
                        route: None,
                        failed: true,
                        error: Some(error.clone()),
                        repair_round: None,
                        candidate_blockers: Vec::new(),
                        ripup_ids: Vec::new(),
                    });
                }
            }

            let probe_start = native_batch_timer(collect_native_timing);
            let probe_result = self.route_single_net_ignore_dynamic_native(
                job.source,
                job.target,
                Some(&job.opened_cells),
                Some(&job.opened_cell_keys),
            );
            let probe_elapsed_us = native_batch_elapsed_us(probe_start);
            timings.probe_route_wall_us += probe_elapsed_us;
            let probe_route = match probe_result {
                Ok(route) => {
                    timings.add_route_result_stats_if(collect_native_timing, &route);
                    attempts.push(NativeRouteAttempt {
                        bucket_name: "probe_route",
                        net_id: job.net_id,
                        route: Some(route.clone()),
                        failed: false,
                        error: None,
                        repair_round: None,
                        candidate_blockers: Vec::new(),
                        ripup_ids: Vec::new(),
                    });
                    route
                }
                Err(error) => {
                    timings.probe_route_failed_wall_us += probe_elapsed_us;
                    attempts.push(NativeRouteAttempt {
                        bucket_name: "probe_route",
                        net_id: job.net_id,
                        route: None,
                        failed: true,
                        error: Some(error.clone()),
                        repair_round: None,
                        candidate_blockers: Vec::new(),
                        ripup_ids: Vec::new(),
                    });
                    failed_net_id = Some(job.net_id);
                    failed_error = Some(error);
                    break;
                }
            };

            let victim_selection_start = native_batch_timer(collect_native_timing);
            let owner_lookup_radius_cells =
                block_radius_cells.max(commit_radius_cells.unwrap_or(block_radius_cells));
            let dynamic_probe_owners =
                self.dynamic_owners_for_native_route(&probe_route, owner_lookup_radius_cells);
            let mut candidate_blocker_priority: FxHashMap<u64, u8> = FxHashMap::default();
            let mut add_candidate_blocker = |owner: u64, priority: u8| {
                if owner == job.net_id || !final_routes.contains_key(&owner) {
                    return;
                }
                candidate_blocker_priority
                    .entry(owner)
                    .and_modify(|existing| *existing = (*existing).min(priority))
                    .or_insert(priority);
            };
            let crossing_repair_enabled = self.crossing_context.is_enabled();
            let allowed_crossing_partners: FxHashSet<u64> = if crossing_repair_enabled {
                self.crossing_allowed_partner_set(job.net_id)
                    .into_iter()
                    .filter(|partner_id| final_routes.contains_key(partner_id))
                    .collect()
            } else {
                FxHashSet::default()
            };
            let probe_crossing_events = if crossing_repair_enabled
                && !allowed_crossing_partners.is_empty()
            {
                self.crossing_events_for_route(job.net_id, &probe_route, &allowed_crossing_partners)
            } else {
                Vec::new()
            };
            let strict_expected_crossing_probe = crossing_repair_enabled
                && self.crossing_context.config().allow_only_expected_pairs
                && !allowed_crossing_partners.is_empty();
            let probe_crossing_compliant = crossing_repair_enabled
                && self.crossing_route_satisfies_partner_constraints(
                    job.net_id,
                    &probe_route,
                    &allowed_crossing_partners,
                    &probe_crossing_events,
                );
            let probe_realized_crossing_violations = if crossing_repair_enabled {
                self.crossing_violations_for_route_with_ports(
                    job.net_id,
                    &probe_route,
                    job.source_port_um,
                    job.target_port_um,
                    Some(&job.opened_cell_keys),
                )
            } else {
                Vec::new()
            };
            let probe_grid_crossing_violations = if crossing_repair_enabled
                && !allowed_crossing_partners.is_empty()
            {
                self.invalid_crossing_intersections_for_route(
                    job.net_id,
                    &probe_route,
                    &allowed_crossing_partners,
                )
            } else {
                Vec::new()
            };
            let allowed_crossing_partner_list: Vec<u64> =
                allowed_crossing_partners.iter().copied().collect();
            let probe_repair_keepout_keys = if crossing_repair_enabled {
                let mut keys = self.crossing_physical_violation_repair_keepout_keys(
                    &probe_realized_crossing_violations,
                    &allowed_crossing_partner_list,
                );
                keys.extend(self.crossing_grid_violation_repair_keepout_keys(
                    &probe_grid_crossing_violations,
                    &allowed_crossing_partner_list,
                ));
                keys
            } else {
                FxHashSet::default()
            };
            if crossing_repair_enabled {
                let legal_crossed_partners =
                    Self::crossing_partner_ids_from_events(&probe_crossing_events);
                for owner in dynamic_probe_owners {
                    if !legal_crossed_partners.contains(&owner) {
                        add_candidate_blocker(owner, 0);
                    }
                }
                for invalid in &probe_grid_crossing_violations {
                    add_candidate_blocker(invalid.partner_net_id, 1);
                }
                for invalid in &probe_realized_crossing_violations {
                    add_candidate_blocker(invalid.partner_net_id, 0);
                }
                for partner_id in
                    Self::crossing_partners_with_overlapping_reservations(&probe_crossing_events)
                {
                    add_candidate_blocker(partner_id, 1);
                }
                let reservation_blockers =
                    self.crossing_reservation_blockers(job.net_id, &probe_crossing_events);
                for owner in reservation_blockers.dynamic_blockers {
                    add_candidate_blocker(owner, 0);
                }
                if reservation_blockers.has_static_blocker {
                    for event in &probe_crossing_events {
                        add_candidate_blocker(event.partner_net_id, 1);
                    }
                }
                if self.crossing_context.config().allow_only_expected_pairs {
                    for partner_id in &allowed_crossing_partners {
                        if !legal_crossed_partners.contains(partner_id)
                            && final_routes.contains_key(partner_id)
                        {
                            add_candidate_blocker(*partner_id, 2);
                        }
                    }
                }
            } else {
                for owner in dynamic_probe_owners {
                    add_candidate_blocker(owner, 0);
                }
            }
            let mut candidate_blockers: Vec<u64> =
                candidate_blocker_priority.keys().copied().collect();
            candidate_blockers.sort_unstable_by_key(|owner| {
                (
                    candidate_blocker_priority
                        .get(owner)
                        .copied()
                        .unwrap_or(u8::MAX),
                    order_by_id.get(owner).copied().unwrap_or(usize::MAX),
                )
            });
            if trace_native_repair {
                eprintln!(
                    "native_repair_probe net={} allowed_partners={} crossing_events={} grid_violations={} realized_violations={} realized_reasons={:?} keepout_keys={} candidate_blockers={:?}",
                    job.net_id,
                    allowed_crossing_partners.len(),
                    probe_crossing_events.len(),
                    probe_grid_crossing_violations.len(),
                    probe_realized_crossing_violations.len(),
                    probe_realized_crossing_violations
                        .iter()
                        .map(|violation| (violation.partner_net_id, violation.reason))
                        .collect::<Vec<_>>(),
                    probe_repair_keepout_keys.len(),
                    candidate_blockers,
                );
            }
            timings.repair_probe_victim_selection_us +=
                native_batch_elapsed_us(victim_selection_start);
            let guided_collision_crossing_enabled =
                std::env::var_os("PHOTONIC_ROUTER_ENABLE_GUIDED_COLLISION_CROSSING").is_some()
                    && std::env::var_os("PHOTONIC_ROUTER_DISABLE_GUIDED_COLLISION_CROSSING")
                        .is_none();
            if crossing_repair_enabled
                && self.use_collision_crossing_routing
                && guided_collision_crossing_enabled
                && !candidate_blockers.is_empty()
            {
                let guided_start = native_batch_timer(collect_native_timing);
                let mut guided_partner_ids = FxHashSet::default();
                for owner in candidate_blockers
                    .iter()
                    .take(max_victims_per_failure.max(1).min(2))
                {
                    if allowed_crossing_partners.contains(owner) {
                        guided_partner_ids.insert(*owner);
                    }
                }
                if !guided_partner_ids.is_empty() {
                    let source_state =
                        State::new(job.source.x, job.source.y, job.source.angle);
                    let target_state =
                        State::new(job.target.x, job.target.y, job.target.angle);
                    let opened_search_owned = self.opened_cells_without_dynamic_overlap(
                        &job.opened_cell_keys,
                        source_state,
                        target_state,
                    );
                    let opened_search_ref =
                        opened_search_owned.as_ref().unwrap_or(&job.opened_cell_keys);
                    let dynamic_clearance_exempt_keys = if block_radius_cells > 0
                        && !job.clearance_exempt_cells.is_empty()
                    {
                        Some(&job.clearance_exempt_cell_keys)
                    } else {
                        None
                    };
                    let mut guided_cfg = self
                        .astar_config(None, None, None)
                        .map_err(PyRuntimeError::new_err)?;
                    guided_cfg.require_terminal_straights = false;
                    let guided_result = self.try_route_through_collision_partner_set(
                        job.net_id,
                        source_state,
                        target_state,
                        opened_search_ref,
                        &guided_cfg,
                        block_radius_cells,
                        dynamic_clearance_exempt_keys,
                        &guided_partner_ids,
                        job.source_port_um,
                        job.target_port_um,
                        Some(&job.opened_cell_keys),
                    );
                    timings.repair_failed_net_wall_us +=
                        native_batch_elapsed_us(guided_start);
                    if let Some((route, crossing_events)) = guided_result {
                        let crossed_partner_ids =
                            Self::crossing_partner_ids_from_events(&crossing_events);
                        let crossed_partner_vec: Vec<u64> =
                            crossed_partner_ids.iter().copied().collect();
                        match self.commit_native_route_with_clearance_allowing_core_overlap(
                            job.net_id,
                            &route,
                            block_radius_cells,
                            commit_radius_cells,
                            &job.clearance_exempt_cells,
                            core_radius_cells,
                            job.source_port_um,
                            job.target_port_um,
                            Some(&job.opened_cell_keys),
                            &crossed_partner_vec,
                            true,
                        ) {
                            Ok(true) => {
                                if trace_native_repair {
                                    eprintln!(
                                        "native_repair_guided_crossing net={} partners={:?} events={} cost={} waypoints={:?}",
                                        job.net_id,
                                        crossed_partner_ids,
                                        crossing_events.len(),
                                        route.total_cost,
                                        route.compressed_waypoints,
                                    );
                                }
                                timings.add_route_result_stats_if(collect_native_timing, &route);
                                attempts.push(NativeRouteAttempt {
                                    net_id: job.net_id,
                                    bucket_name: "guided_collision_crossing",
                                    route: Some(route.clone()),
                                    failed: false,
                                    error: None,
                                    repair_round: Some(0),
                                    candidate_blockers: candidate_blockers.clone(),
                                    ripup_ids: Vec::new(),
                                });
                                final_routes.insert(job.net_id, route);
                                continue 'route_jobs;
                            }
                            Ok(false) => {}
                            Err(error) => {
                                if trace_native_repair {
                                    eprintln!(
                                        "native_repair_guided_crossing_commit_failed net={} error={}",
                                        job.net_id, error
                                    );
                                }
                            }
                        }
                    }
                }
            }
            let invalid_collision_crossing_candidate = crossing_repair_enabled
                && self.use_collision_crossing_routing
                && !probe_realized_crossing_violations.is_empty();
            if crossing_repair_enabled
                && !invalid_collision_crossing_candidate
                && !probe_repair_keepout_keys.is_empty()
                && !candidate_blockers.is_empty()
            {
                let localized_probe_keepout = probe_repair_keepout_keys.clone();
                let added_keepout = self.obstacle_map.add_static_keys(&localized_probe_keepout);
                if trace_native_repair {
                    eprintln!(
                        "native_repair_local_keepout net={} keys={} added={} candidate_blockers={:?}",
                        job.net_id,
                        localized_probe_keepout.len(),
                        added_keepout,
                        candidate_blockers,
                    );
                }

                let mut local_retry_route: Option<RouteResult> = None;
                let route_start = native_batch_timer(collect_native_timing);
                let prefer_orthogonal_local_retry =
                    !probe_realized_crossing_violations.is_empty();
                let route_result = self
                    .route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                        job.net_id,
                        job.source,
                        job.target,
                        block_radius_cells,
                        &job.opened_cells,
                        &job.opened_cell_keys,
                        commit_radius_cells,
                        &job.clearance_exempt_cells,
                        &job.clearance_exempt_cell_keys,
                        core_radius_cells,
                        &localized_probe_keepout,
                        job.source_port_um,
                        job.target_port_um,
                        prefer_orthogonal_local_retry,
                    );
                let route_elapsed_us = native_batch_elapsed_us(route_start);
                timings.repair_failed_net_wall_us += route_elapsed_us;
                match route_result {
                    Ok(route) => {
                        timings.add_route_result_stats_if(collect_native_timing, &route);
                        attempts.push(NativeRouteAttempt {
                            bucket_name: "localized_crossing_keepout",
                            net_id: job.net_id,
                            route: Some(route.clone()),
                            failed: false,
                            error: None,
                            repair_round: Some(0),
                            candidate_blockers: candidate_blockers.clone(),
                            ripup_ids: Vec::new(),
                        });
                        local_retry_route = Some(route);
                    }
                    Err(normal_error) => {
                        let (repair_keepout, extra_repair_keepout) = self
                            .augmented_crossing_error_repair_keepout(
                                &localized_probe_keepout,
                                &normal_error,
                            );
                        if !extra_repair_keepout.is_empty() {
                            self.obstacle_map.add_static_keys(&extra_repair_keepout);
                        }
                        timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                        attempts.push(NativeRouteAttempt {
                            bucket_name: "localized_crossing_keepout",
                            net_id: job.net_id,
                            route: None,
                            failed: true,
                            error: Some(normal_error),
                            repair_round: Some(0),
                            candidate_blockers: candidate_blockers.clone(),
                            ripup_ids: Vec::new(),
                        });
                        let repair_start = native_batch_timer(collect_native_timing);
                        let repair_result =
                            self.route_single_net_and_commit_repair_native_with_repair_keepout(
                                job.net_id,
                                job.source,
                                job.target,
                                block_radius_cells,
                                &job.opened_cells,
                                &job.opened_cell_keys,
                                history_weight,
                                commit_radius_cells,
                                &job.clearance_exempt_cells,
                                &job.clearance_exempt_cell_keys,
                                core_radius_cells,
                                &repair_keepout,
                                job.source_port_um,
                                job.target_port_um,
                            );
                        let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                        if !extra_repair_keepout.is_empty() {
                            self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                        }
                        timings.repair_failed_net_wall_us += repair_elapsed_us;
                        match repair_result {
                            Ok(route) => {
                                timings.add_route_result_stats_if(collect_native_timing, &route);
                                attempts.push(NativeRouteAttempt {
                                    bucket_name: "localized_crossing_keepout",
                                    net_id: job.net_id,
                                    route: Some(route.clone()),
                                    failed: false,
                                    error: None,
                                    repair_round: Some(0),
                                    candidate_blockers: candidate_blockers.clone(),
                                    ripup_ids: Vec::new(),
                                });
                                local_retry_route = Some(route);
                            }
                            Err(error) => {
                                timings.repair_failed_net_failed_wall_us += repair_elapsed_us;
                                attempts.push(NativeRouteAttempt {
                                    bucket_name: "localized_crossing_keepout",
                                    net_id: job.net_id,
                                    route: None,
                                    failed: true,
                                    error: Some(error),
                                    repair_round: Some(0),
                                    candidate_blockers: candidate_blockers.clone(),
                                    ripup_ids: Vec::new(),
                                });
                            }
                        }
                    }
                }
                self.obstacle_map
                    .remove_static_keys(&localized_probe_keepout);
                if let Some(route) = local_retry_route {
                    final_routes.insert(job.net_id, route);
                    repair_count = repair_count.saturating_add(1);
                    continue 'route_jobs;
                }
            }
            if candidate_blockers.is_empty() {
                if probe_crossing_compliant {
                    {
                        let commit_start = native_batch_timer(collect_native_timing);
                        let crossed_partner_ids =
                            Self::crossing_partner_ids_from_events(&probe_crossing_events);
                        let allowed_crossing_core_keys =
                            Self::crossing_reservation_keys_for_events(&probe_crossing_events);
                        let (route_cells, core_cells) = self.route_commit_and_core_cells(
                            &probe_route,
                            block_radius_cells,
                            commit_radius_cells,
                            Some(&job.clearance_exempt_cells),
                            core_radius_cells,
                            job.source_port_um,
                            job.target_port_um,
                        );
                        if self
                            .obstacle_map
                            .commit_route_with_clearance_and_allowed_core_overlap_cells(
                                job.net_id,
                                &core_cells,
                                &route_cells,
                                &job.clearance_exempt_cells,
                                &crossed_partner_ids,
                                Some(&allowed_crossing_core_keys),
                            )
                        {
                            timings.commit_update_dynamic_map_us +=
                                native_batch_elapsed_us(commit_start);
                            self.remove_crossing_events_for_net(job.net_id);
                            self.add_crossing_events(probe_crossing_events);
                            if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                                job.net_id,
                                &probe_route,
                                job.source_port_um,
                                job.target_port_um,
                            ) {
                                self.rollback_committed_route(job.net_id);
                                failed_net_id = Some(job.net_id);
                                failed_error = Some(error);
                                break;
                            }
                            self.remember_committed_route_opened_cells(
                                job.net_id,
                                Some(&job.opened_cell_keys),
                            );
                            self.add_crossing_spacing_history_for_route(job.net_id, &probe_route);
                            self.invalidate_meander_base_prefix();
                            if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                                job.net_id,
                                &probe_route,
                                job.source_port_um,
                                job.target_port_um,
                                Some(&job.opened_cell_keys),
                            ) {
                                self.rollback_committed_route(job.net_id);
                                failed_net_id = Some(job.net_id);
                                failed_error = Some(error);
                                break;
                            }
                            final_routes.insert(job.net_id, probe_route);
                            continue;
                        }
                        timings.commit_update_dynamic_map_us +=
                            native_batch_elapsed_us(commit_start);
                    }
                }
                if strict_expected_crossing_probe {
                    failed_net_id = Some(job.net_id);
                    failed_error =
                        Some("Probe route violates expected crossing constraints".to_string());
                    break;
                }
                let commit_start = native_batch_timer(collect_native_timing);
                if self.commit_native_route_with_clearance(
                    job.net_id,
                    &probe_route,
                    block_radius_cells,
                    commit_radius_cells,
                    &job.clearance_exempt_cells,
                    core_radius_cells,
                    job.source_port_um,
                    job.target_port_um,
                    Some(&job.opened_cell_keys),
                ) {
                    timings.commit_update_dynamic_map_us += native_batch_elapsed_us(commit_start);
                    final_routes.insert(job.net_id, probe_route);
                    continue;
                }
                timings.commit_update_dynamic_map_us += native_batch_elapsed_us(commit_start);
                failed_net_id = Some(job.net_id);
                failed_error = Some("Failed to commit static-only probe route".to_string());
                break;
            }

            let history_start = native_batch_timer(collect_native_timing);
            self.add_history_for_native_route(&probe_route, block_radius_cells, history_increment);
            timings.history_update_us += native_batch_elapsed_us(history_start);
            let max_rounds = max_rounds.max(1);
            let max_victims = max_victims_per_failure.max(1);
            let mut repaired = false;
            let round_base_map = self.obstacle_map.clone();
            let round_base_center_routes = self.committed_center_routes.clone();
            let round_base_realized_center_routes = self.committed_realized_center_routes.clone();
            let round_base_opened_cell_keys = self.committed_opened_cell_keys.clone();
            let round_base_crossing_events = self.crossing_events.clone();
            let round_base_routes = final_routes.clone();

            let mut repair_victim_sets: Vec<(u32, Vec<u64>)> = Vec::new();
            if crossing_repair_enabled {
                if !probe_realized_crossing_violations.is_empty() && candidate_blockers.len() > 2 {
                    let single_victim_limit = candidate_blockers.len().min(max_victims);
                    for owner in candidate_blockers.iter().take(single_victim_limit) {
                        repair_victim_sets.push((1, vec![*owner]));
                    }
                    if max_victims >= 2 {
                        let top_pair = vec![candidate_blockers[0], candidate_blockers[1]];
                        if !repair_victim_sets
                            .iter()
                            .any(|(_, existing)| existing == &top_pair)
                        {
                            repair_victim_sets.push((1, top_pair));
                        }
                    }
                } else {
                    let single_victim_limit = candidate_blockers.len().min(max_victims);
                    for owner in candidate_blockers.iter().take(single_victim_limit) {
                        repair_victim_sets.push((1, vec![*owner]));
                    }
                }
            }
            for round_idx in 1..=max_rounds {
                let ripup_ids: Vec<u64> = candidate_blockers
                    .iter()
                    .take((max_victims * round_idx as usize).min(candidate_blockers.len()))
                    .copied()
                    .collect();
                if !ripup_ids.is_empty()
                    && !repair_victim_sets
                        .iter()
                        .any(|(_, existing)| existing == &ripup_ids)
                {
                    repair_victim_sets.push((round_idx, ripup_ids));
                }
            }

            let prefer_orthogonal_repair = crossing_repair_enabled
                && !probe_realized_crossing_violations.is_empty()
                && (probe_grid_crossing_violations.is_empty() || candidate_blockers.len() > 2);
            let mut learned_repair_keepouts_by_ripup: FxHashMap<Vec<u64>, FxHashSet<CellKey>> =
                FxHashMap::default();
            let mut learned_victim_only_keepouts_by_ripup: FxHashMap<
                Vec<u64>,
                FxHashSet<CellKey>,
            > = FxHashMap::default();
            let mut learned_repair_retry_counts: FxHashMap<Vec<u64>, usize> =
                FxHashMap::default();
            let mut repair_set_index = 0usize;
            while repair_set_index < repair_victim_sets.len() {
                let active_repair_set_index = repair_set_index;
                let (round_idx, ripup_ids) = repair_victim_sets[active_repair_set_index].clone();
                repair_set_index += 1;
                for victim_first in [false, true] {
                    for reverse_victim_order in [false, true] {
                        if reverse_victim_order && ripup_ids.len() <= 1 {
                            continue;
                        }
                    let reset_start = native_batch_timer(collect_native_timing);
                    self.obstacle_map = round_base_map.clone();
                    self.committed_center_routes = round_base_center_routes.clone();
                    self.committed_realized_center_routes =
                        round_base_realized_center_routes.clone();
                    self.committed_opened_cell_keys = round_base_opened_cell_keys.clone();
                    self.crossing_events = round_base_crossing_events.clone();
                    self.invalidate_meander_base_prefix();
                    final_routes = round_base_routes.clone();
                    timings.repair_state_reset_us += native_batch_elapsed_us(reset_start);
                    let mut victim_reroute_ids = ripup_ids.clone();
                    if reverse_victim_order {
                        victim_reroute_ids.reverse();
                    }
                    let route_order = if victim_first {
                        "victim_first"
                    } else {
                        "current_first"
                    };
                    push_native_repair_trace(
                        &mut repair_trace,
                        "repair_mode_start",
                        Some(route_order),
                        None,
                        job.net_id,
                        Some(round_idx),
                        Some(active_repair_set_index as u64),
                        &candidate_blockers,
                        &ripup_ids,
                        &victim_reroute_ids,
                        Some(victim_first),
                        Some(reverse_victim_order),
                        None,
                        None,
                    );

                    let mut victim_first_probe_reservation = FxHashSet::default();
                    let mut temporary_probe_reservation: FxHashSet<CellKey> = if victim_first {
                        let mut conflict_keys = self.crossing_physical_violation_repair_keepout_keys(
                            &probe_realized_crossing_violations,
                            &ripup_ids,
                        );
                        conflict_keys.extend(
                            self.crossing_grid_violation_repair_keepout_keys(
                                &probe_grid_crossing_violations,
                                &ripup_ids,
                            ),
                        );
                        if let Some(learned_keepout) =
                            learned_repair_keepouts_by_ripup.get(&ripup_ids)
                        {
                            conflict_keys.extend(learned_keepout.iter().copied());
                        }
                        if let Some(victim_only_keepout) =
                            learned_victim_only_keepouts_by_ripup.get(&ripup_ids)
                        {
                            for key in victim_only_keepout {
                                if conflict_keys.insert(*key) {
                                    victim_first_probe_reservation.insert(*key);
                                }
                            }
                        }
                        let probe_keys: FxHashSet<CellKey> =
                            probe_route.cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect();
                        for old_id in &ripup_ids {
                            if let Some(old_route) = final_routes.get(old_id) {
                                for (x, y) in &old_route.cells {
                                    let key = pack_xy(*x, *y);
                                    if probe_keys.contains(&key) {
                                        if conflict_keys.insert(key) {
                                            victim_first_probe_reservation.insert(key);
                                        }
                                    }
                                }
                            }
                        }
                        conflict_keys
                    } else {
                        let mut conflict_keys = FxHashSet::default();
                        if let Some(learned_keepout) =
                            learned_repair_keepouts_by_ripup.get(&ripup_ids)
                        {
                            conflict_keys.extend(learned_keepout.iter().copied());
                        }
                        conflict_keys
                    };
                    if trace_native_repair && !temporary_probe_reservation.is_empty() {
                        eprintln!(
                            "native_repair_keepout net={} ripup={:?} victim_first={} reverse={} keys={}",
                            job.net_id,
                            ripup_ids,
                            victim_first,
                            reverse_victim_order,
                            temporary_probe_reservation.len(),
                        );
                    }

                    let lidar_pure_crossing_repair = crossing_repair_enabled
                        && self.use_collision_crossing_routing
                        && !self.crossing_context.config().allow_only_expected_pairs;
                    for old_id in &ripup_ids {
                        if let Some(old_route) = final_routes.get(old_id).cloned() {
                            if !lidar_pure_crossing_repair {
                                let history_start = native_batch_timer(collect_native_timing);
                                self.add_history_for_native_route(
                                    &old_route,
                                    block_radius_cells,
                                    history_increment,
                                );
                                timings.history_update_us +=
                                    native_batch_elapsed_us(history_start);
                            }
                        }
                        let ripup_start = native_batch_timer(collect_native_timing);
                        self.remove_crossing_events_for_net(*old_id);
                        self.obstacle_map.ripup_route(*old_id);
                        self.committed_center_routes.remove(old_id);
                        self.committed_realized_center_routes.remove(old_id);
                        self.committed_opened_cell_keys.remove(old_id);
                        timings.ripup_us += native_batch_elapsed_us(ripup_start);
                        final_routes.remove(old_id);
                    }

                    let mut temporary_probe_reservation_added =
                        if !temporary_probe_reservation.is_empty() {
                            self.obstacle_map
                                .add_static_keys(&temporary_probe_reservation);
                            true
                        } else {
                            false
                        };
                    let mut victim_reroute_only_reservation = FxHashSet::default();
                    let mut victim_reroute_only_reservation_added = false;
                    let mut mode_failed = false;
                    let mut repaired_route: Option<RouteResult> = None;
                    if !victim_first {
                        let route_start = native_batch_timer(collect_native_timing);
                        let route_result = self
                            .route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                            job.net_id,
                            job.source,
                            job.target,
                            block_radius_cells,
                            &job.opened_cells,
                            &job.opened_cell_keys,
                            commit_radius_cells,
                            &job.clearance_exempt_cells,
                            &job.clearance_exempt_cell_keys,
                            core_radius_cells,
                            &temporary_probe_reservation,
                            job.source_port_um,
                            job.target_port_um,
                            prefer_orthogonal_repair,
                        );
                        let route_elapsed_us = native_batch_elapsed_us(route_start);
                        timings.repair_failed_net_wall_us += route_elapsed_us;
                        match route_result {
                            Ok(route) => {
                                timings.add_route_result_stats_if(collect_native_timing, &route);
                                push_native_repair_trace(
                                    &mut repair_trace,
                                    "current_route",
                                    Some(route_order),
                                    Some("normal_route"),
                                    job.net_id,
                                    Some(round_idx),
                                    Some(active_repair_set_index as u64),
                                    &candidate_blockers,
                                    &ripup_ids,
                                    &victim_reroute_ids,
                                    Some(victim_first),
                                    Some(reverse_victim_order),
                                    Some(true),
                                    None,
                                );
                                attempts.push(NativeRouteAttempt {
                                    bucket_name: "repair_failed_net",
                                    net_id: job.net_id,
                                    route: Some(route.clone()),
                                    failed: false,
                                    error: None,
                                    repair_round: Some(round_idx),
                                    candidate_blockers: candidate_blockers.clone(),
                                    ripup_ids: ripup_ids.clone(),
                                });
                                final_routes.insert(job.net_id, route.clone());
                                repaired_route = Some(route);
                            }
                            Err(normal_error) => {
                                enqueue_targeted_illegal_crossing_repair_set(
                                    &mut repair_victim_sets,
                                    &mut candidate_blockers,
                                    &round_base_routes,
                                    job.net_id,
                                    &ripup_ids,
                                    &normal_error,
                                    round_idx,
                                    max_rounds,
                                    max_victims,
                                );
                                let learned_repair_keepout =
                                    learned_repair_keepouts_by_ripup
                                        .entry(ripup_ids.clone())
                                        .or_default();
                                if self.remember_local_repair_error_keepout(
                                    learned_repair_keepout,
                                    &normal_error,
                                ) {
                                    enqueue_learned_keepout_repair_retry(
                                        &mut repair_victim_sets,
                                        &mut learned_repair_retry_counts,
                                        &ripup_ids,
                                        round_idx,
                                        repair_set_index,
                                    );
                                }
                                let (repair_keepout, extra_repair_keepout) = self
                                    .augmented_crossing_error_repair_keepout(
                                        &temporary_probe_reservation,
                                        &normal_error,
                                    );
                                if !extra_repair_keepout.is_empty() {
                                    self.obstacle_map.add_static_keys(&extra_repair_keepout);
                                }
                                timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                                push_native_repair_trace(
                                    &mut repair_trace,
                                    "current_route",
                                    Some(route_order),
                                    Some("normal_route"),
                                    job.net_id,
                                    Some(round_idx),
                                    Some(active_repair_set_index as u64),
                                    &candidate_blockers,
                                    &ripup_ids,
                                    &victim_reroute_ids,
                                    Some(victim_first),
                                    Some(reverse_victim_order),
                                    Some(false),
                                    Some(normal_error.clone()),
                                );
                                attempts.push(NativeRouteAttempt {
                                    bucket_name: "repair_failed_net",
                                    net_id: job.net_id,
                                    route: None,
                                    failed: true,
                                    error: Some(normal_error),
                                    repair_round: Some(round_idx),
                                    candidate_blockers: candidate_blockers.clone(),
                                    ripup_ids: ripup_ids.clone(),
                                });
                                let repair_start = native_batch_timer(collect_native_timing);
                                let repair_result = self
                                    .route_single_net_and_commit_repair_native_with_repair_keepout(
                                    job.net_id,
                                    job.source,
                                    job.target,
                                    block_radius_cells,
                                    &job.opened_cells,
                                    &job.opened_cell_keys,
                                    history_weight,
                                    commit_radius_cells,
                                    &job.clearance_exempt_cells,
                                    &job.clearance_exempt_cell_keys,
                                    core_radius_cells,
                                    &repair_keepout,
                                    job.source_port_um,
                                    job.target_port_um,
                                );
                                let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                                if !extra_repair_keepout.is_empty() {
                                    self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                                }
                                timings.repair_failed_net_wall_us += repair_elapsed_us;
                                match repair_result {
                                    Ok(route) => {
                                        timings.add_route_result_stats_if(
                                            collect_native_timing,
                                            &route,
                                        );
                                        push_native_repair_trace(
                                            &mut repair_trace,
                                            "current_route",
                                            Some(route_order),
                                            Some("repair_fallback"),
                                            job.net_id,
                                            Some(round_idx),
                                            Some(active_repair_set_index as u64),
                                            &candidate_blockers,
                                            &ripup_ids,
                                            &victim_reroute_ids,
                                            Some(victim_first),
                                            Some(reverse_victim_order),
                                            Some(true),
                                            None,
                                        );
                                        attempts.push(NativeRouteAttempt {
                                            bucket_name: "repair_failed_net",
                                            net_id: job.net_id,
                                            route: Some(route.clone()),
                                            failed: false,
                                            error: None,
                                            repair_round: Some(round_idx),
                                            candidate_blockers: candidate_blockers.clone(),
                                            ripup_ids: ripup_ids.clone(),
                                        });
                                        final_routes.insert(job.net_id, route.clone());
                                        repaired_route = Some(route);
                                    }
                                    Err(error) => {
                                        enqueue_targeted_illegal_crossing_repair_set(
                                            &mut repair_victim_sets,
                                            &mut candidate_blockers,
                                            &round_base_routes,
                                            job.net_id,
                                            &ripup_ids,
                                            &error,
                                            round_idx,
                                            max_rounds,
                                            max_victims,
                                        );
                                        let learned_repair_keepout =
                                            learned_repair_keepouts_by_ripup
                                                .entry(ripup_ids.clone())
                                                .or_default();
                                        if self.remember_local_repair_error_keepout(
                                            learned_repair_keepout,
                                            &error,
                                        ) {
                                            enqueue_learned_keepout_repair_retry(
                                                &mut repair_victim_sets,
                                                &mut learned_repair_retry_counts,
                                                &ripup_ids,
                                                round_idx,
                                                repair_set_index,
                                            );
                                        }
                                        timings.repair_failed_net_failed_wall_us +=
                                            repair_elapsed_us;
                                        push_native_repair_trace(
                                            &mut repair_trace,
                                            "current_route",
                                            Some(route_order),
                                            Some("repair_fallback"),
                                            job.net_id,
                                            Some(round_idx),
                                            Some(active_repair_set_index as u64),
                                            &candidate_blockers,
                                            &ripup_ids,
                                            &victim_reroute_ids,
                                            Some(victim_first),
                                            Some(reverse_victim_order),
                                            Some(false),
                                            Some(error.clone()),
                                        );
                                        attempts.push(NativeRouteAttempt {
                                            bucket_name: "repair_failed_net",
                                            net_id: job.net_id,
                                            route: None,
                                            failed: true,
                                            error: Some(error),
                                            repair_round: Some(round_idx),
                                            candidate_blockers: candidate_blockers.clone(),
                                            ripup_ids: ripup_ids.clone(),
                                        });
                                        mode_failed = true;
                                    }
                                }
                            }
                        }
                    }

                    if !mode_failed
                        && !victim_first
                        && temporary_probe_reservation_added
                        && (candidate_blockers.len() > 2
                            || (probe_grid_crossing_violations.is_empty()
                                && !probe_realized_crossing_violations.is_empty()))
                    {
                        self.obstacle_map
                            .remove_static_keys(&temporary_probe_reservation);
                        temporary_probe_reservation_added = false;
                    }

                    if !mode_failed && !victim_first {
                        if let Some(victim_only_keepout) =
                            learned_victim_only_keepouts_by_ripup.get(&ripup_ids)
                        {
                            for key in victim_only_keepout {
                                if !temporary_probe_reservation.contains(key) {
                                    victim_reroute_only_reservation.insert(*key);
                                }
                            }
                            if !victim_reroute_only_reservation.is_empty() {
                                self.obstacle_map
                                    .add_static_keys(&victim_reroute_only_reservation);
                                victim_reroute_only_reservation_added = true;
                            }
                        }
                    }

                    if !mode_failed {
                        for old_id in &victim_reroute_ids {
                            let Some(victim_job) = job_by_id.get(old_id) else {
                                mode_failed = true;
                                break;
                            };
                            let mut guided_victim_route: Option<RouteResult> = None;
                            let mut lidar_crossing_partners_available = false;
                            if lidar_pure_crossing_repair
                                && !victim_first
                                && repaired_route.is_some()
                            {
                                let victim_source_state = State::new(
                                    victim_job.source.x,
                                    victim_job.source.y,
                                    victim_job.source.angle,
                                );
                                let victim_target_state = State::new(
                                    victim_job.target.x,
                                    victim_job.target.y,
                                    victim_job.target.angle,
                                );
                                let opened_search_owned = self.opened_cells_without_dynamic_overlap(
                                    &victim_job.opened_cell_keys,
                                    victim_source_state,
                                    victim_target_state,
                                );
                                let opened_search_ref = opened_search_owned
                                    .as_ref()
                                    .unwrap_or(&victim_job.opened_cell_keys);
                                let dynamic_clearance_exempt_keys = if block_radius_cells > 0
                                    && !victim_job.clearance_exempt_cells.is_empty()
                                {
                                    Some(&victim_job.clearance_exempt_cell_keys)
                                } else {
                                    None
                                };
                                let mut seeded_partner_ids = Self::crossing_partner_ids_for_net(
                                    &round_base_crossing_events,
                                    victim_job.net_id,
                                );
                                seeded_partner_ids.retain(|partner_id| {
                                    *partner_id != victim_job.net_id
                                        && !ripup_ids.contains(partner_id)
                                        && self.committed_center_routes.contains_key(partner_id)
                                });
                                if self.committed_center_routes.contains_key(&job.net_id) {
                                    seeded_partner_ids.insert(job.net_id);
                                }
                                if seeded_partner_ids.len() > 1 {
                                    let mut seeded_cfg = self
                                        .astar_config(None, None, Some(0.0))
                                        .map_err(PyRuntimeError::new_err)?;
                                    seeded_cfg.require_terminal_straights = false;
                                    let seeded_start =
                                        native_batch_timer(collect_native_timing);
                                    let seeded_result =
                                        self.try_route_through_collision_partner_set(
                                            victim_job.net_id,
                                            victim_source_state,
                                            victim_target_state,
                                            opened_search_ref,
                                            &seeded_cfg,
                                            block_radius_cells,
                                            dynamic_clearance_exempt_keys,
                                            &seeded_partner_ids,
                                            victim_job.source_port_um,
                                            victim_job.target_port_um,
                                            Some(&victim_job.opened_cell_keys),
                                        );
                                    timings.reroute_victims_wall_us +=
                                        native_batch_elapsed_us(seeded_start);
                                    if let Some((route, crossing_events)) = seeded_result {
                                        let crossed_partner_ids =
                                            Self::crossing_partner_ids_from_events(
                                                &crossing_events,
                                            );
                                        let crossed_partner_vec: Vec<u64> =
                                            crossed_partner_ids.iter().copied().collect();
                                        match self
                                            .commit_native_route_with_clearance_allowing_core_overlap(
                                                victim_job.net_id,
                                                &route,
                                                block_radius_cells,
                                                commit_radius_cells,
                                                &victim_job.clearance_exempt_cells,
                                                core_radius_cells,
                                                victim_job.source_port_um,
                                                victim_job.target_port_um,
                                                Some(&victim_job.opened_cell_keys),
                                                &crossed_partner_vec,
                                                true,
                                            ) {
                                            Ok(true) => {
                                                timings.add_route_result_stats_if(
                                                    collect_native_timing,
                                                    &route,
                                                );
                                                push_native_repair_trace(
                                                    &mut repair_trace,
                                                    "victim_reroute",
                                                    Some(route_order),
                                                    Some("lidar_seeded_collision_crossing"),
                                                    victim_job.net_id,
                                                    Some(round_idx),
                                                    Some(active_repair_set_index as u64),
                                                    &candidate_blockers,
                                                    &ripup_ids,
                                                    &victim_reroute_ids,
                                                    Some(victim_first),
                                                    Some(reverse_victim_order),
                                                    Some(true),
                                                    None,
                                                );
                                                attempts.push(NativeRouteAttempt {
                                                    bucket_name: "reroute_victims",
                                                    net_id: victim_job.net_id,
                                                    route: Some(route.clone()),
                                                    failed: false,
                                                    error: None,
                                                    repair_round: Some(round_idx),
                                                    candidate_blockers: candidate_blockers.clone(),
                                                    ripup_ids: ripup_ids.clone(),
                                                });
                                                guided_victim_route = Some(route);
                                            }
                                            Ok(false) => {}
                                            Err(error) => {
                                                if trace_native_repair {
                                                    eprintln!(
                                                        "native_repair_lidar_seeded_crossing_commit_failed net={} error={}",
                                                        victim_job.net_id, error
                                                    );
                                                }
                                            }
                                        }
                                    }
                                }
                                let collision_partner_ids =
                                    self.crossing_allowed_partner_set(victim_job.net_id);
                                lidar_crossing_partners_available =
                                    !collision_partner_ids.is_empty();
                                if guided_victim_route.is_none()
                                    && lidar_crossing_partners_available
                                {
                                    let mut crossing_cfg = self
                                        .astar_config(None, None, Some(0.0))
                                        .map_err(PyRuntimeError::new_err)?;
                                    crossing_cfg.require_terminal_straights = false;
                                    let crossing_start =
                                        native_batch_timer(collect_native_timing);
                                    let crossing_result =
                                        self.try_route_with_collision_crossings_with_loss(
                                        victim_job.net_id,
                                        victim_source_state,
                                        victim_target_state,
                                        opened_search_ref,
                                        &crossing_cfg,
                                        block_radius_cells,
                                        dynamic_clearance_exempt_keys,
                                        &collision_partner_ids,
                                        victim_job.source_port_um,
                                        victim_job.target_port_um,
                                        Some(&victim_job.opened_cell_keys),
                                        Some(0.0),
                                    );
                                    timings.reroute_victims_wall_us +=
                                        native_batch_elapsed_us(crossing_start);
                                    if let Some((route, crossing_events)) = crossing_result {
                                        let crossed_partner_ids =
                                            Self::crossing_partner_ids_from_events(
                                                &crossing_events,
                                            );
                                        let crossed_partner_vec: Vec<u64> =
                                            crossed_partner_ids.iter().copied().collect();
                                        match self
                                            .commit_native_route_with_clearance_allowing_core_overlap(
                                                victim_job.net_id,
                                                &route,
                                                block_radius_cells,
                                                commit_radius_cells,
                                                &victim_job.clearance_exempt_cells,
                                                core_radius_cells,
                                                victim_job.source_port_um,
                                                victim_job.target_port_um,
                                                Some(&victim_job.opened_cell_keys),
                                                &crossed_partner_vec,
                                                true,
                                            ) {
                                            Ok(true) => {
                                                timings.add_route_result_stats_if(
                                                    collect_native_timing,
                                                    &route,
                                                );
                                                push_native_repair_trace(
                                                    &mut repair_trace,
                                                    "victim_reroute",
                                                    Some(route_order),
                                                    Some("lidar_collision_crossing"),
                                                    victim_job.net_id,
                                                    Some(round_idx),
                                                    Some(active_repair_set_index as u64),
                                                    &candidate_blockers,
                                                    &ripup_ids,
                                                    &victim_reroute_ids,
                                                    Some(victim_first),
                                                    Some(reverse_victim_order),
                                                    Some(true),
                                                    None,
                                                );
                                                attempts.push(NativeRouteAttempt {
                                                    bucket_name: "reroute_victims",
                                                    net_id: victim_job.net_id,
                                                    route: Some(route.clone()),
                                                    failed: false,
                                                    error: None,
                                                    repair_round: Some(round_idx),
                                                    candidate_blockers: candidate_blockers.clone(),
                                                    ripup_ids: ripup_ids.clone(),
                                                });
                                                guided_victim_route = Some(route);
                                            }
                                            Ok(false) => {}
                                            Err(error) => {
                                                if trace_native_repair {
                                                    eprintln!(
                                                        "native_repair_lidar_crossing_commit_failed net={} error={}",
                                                        victim_job.net_id, error
                                                    );
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            if crossing_repair_enabled
                                && self.use_collision_crossing_routing
                                && guided_collision_crossing_enabled
                                && !victim_first
                                && repaired_route.is_some()
                                && self.committed_center_routes.contains_key(&job.net_id)
                                && guided_victim_route.is_none()
                            {
                                let mut guided_partner_ids = FxHashSet::default();
                                guided_partner_ids.insert(job.net_id);
                                let victim_source_state = State::new(
                                    victim_job.source.x,
                                    victim_job.source.y,
                                    victim_job.source.angle,
                                );
                                let victim_target_state = State::new(
                                    victim_job.target.x,
                                    victim_job.target.y,
                                    victim_job.target.angle,
                                );
                                let opened_search_owned = self.opened_cells_without_dynamic_overlap(
                                    &victim_job.opened_cell_keys,
                                    victim_source_state,
                                    victim_target_state,
                                );
                                let opened_search_ref = opened_search_owned
                                    .as_ref()
                                    .unwrap_or(&victim_job.opened_cell_keys);
                                let dynamic_clearance_exempt_keys = if block_radius_cells > 0
                                    && !victim_job.clearance_exempt_cells.is_empty()
                                {
                                    Some(&victim_job.clearance_exempt_cell_keys)
                                } else {
                                    None
                                };
                                let mut guided_cfg = self
                                    .astar_config(None, None, None)
                                    .map_err(PyRuntimeError::new_err)?;
                                guided_cfg.require_terminal_straights = false;
                                let guided_start = native_batch_timer(collect_native_timing);
                                let guided_result = self.try_route_through_collision_partner_set(
                                    victim_job.net_id,
                                    victim_source_state,
                                    victim_target_state,
                                    opened_search_ref,
                                    &guided_cfg,
                                    block_radius_cells,
                                    dynamic_clearance_exempt_keys,
                                    &guided_partner_ids,
                                    victim_job.source_port_um,
                                    victim_job.target_port_um,
                                    Some(&victim_job.opened_cell_keys),
                                );
                                timings.reroute_victims_wall_us +=
                                    native_batch_elapsed_us(guided_start);
                                if let Some((route, crossing_events)) = guided_result {
                                    let crossed_partner_ids =
                                        Self::crossing_partner_ids_from_events(&crossing_events);
                                    let crossed_partner_vec: Vec<u64> =
                                        crossed_partner_ids.iter().copied().collect();
                                    match self.commit_native_route_with_clearance_allowing_core_overlap(
                                        victim_job.net_id,
                                        &route,
                                        block_radius_cells,
                                        commit_radius_cells,
                                        &victim_job.clearance_exempt_cells,
                                        core_radius_cells,
                                        victim_job.source_port_um,
                                        victim_job.target_port_um,
                                        Some(&victim_job.opened_cell_keys),
                                        &crossed_partner_vec,
                                        true,
                                    ) {
                                        Ok(true) => {
                                            timings.add_route_result_stats_if(
                                                collect_native_timing,
                                                &route,
                                            );
                                            push_native_repair_trace(
                                                &mut repair_trace,
                                                "victim_reroute",
                                                Some(route_order),
                                                Some("guided_collision_crossing"),
                                                victim_job.net_id,
                                                Some(round_idx),
                                                Some(active_repair_set_index as u64),
                                                &candidate_blockers,
                                                &ripup_ids,
                                                &victim_reroute_ids,
                                                Some(victim_first),
                                                Some(reverse_victim_order),
                                                Some(true),
                                                None,
                                            );
                                            attempts.push(NativeRouteAttempt {
                                                bucket_name: "reroute_victims",
                                                net_id: victim_job.net_id,
                                                route: Some(route.clone()),
                                                failed: false,
                                                error: None,
                                                repair_round: Some(round_idx),
                                                candidate_blockers: candidate_blockers.clone(),
                                                ripup_ids: ripup_ids.clone(),
                                            });
                                            guided_victim_route = Some(route);
                                        }
                                        Ok(false) => {}
                                        Err(error) => {
                                            if trace_native_repair {
                                                eprintln!(
                                                    "native_repair_guided_victim_crossing_commit_failed net={} partner={} error={}",
                                                    victim_job.net_id, job.net_id, error
                                                );
                                            }
                                        }
                                    }
                                }
                            }
                            if let Some(route) = guided_victim_route {
                                final_routes.insert(victim_job.net_id, route.clone());
                                continue;
                            }
                            if lidar_crossing_partners_available {
                                if trace_native_repair {
                                    eprintln!(
                                        "native_repair_lidar_crossing_blocked_plain_victim_fallback net={} partners_available=true",
                                        victim_job.net_id
                                    );
                                }
                                mode_failed = true;
                                break;
                            }
                            let reroute_start = native_batch_timer(collect_native_timing);
                            let reroute_result =
                                self.route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                                victim_job.net_id,
                                victim_job.source,
                                victim_job.target,
                                block_radius_cells,
                                &victim_job.opened_cells,
                                &victim_job.opened_cell_keys,
                                commit_radius_cells,
                                &victim_job.clearance_exempt_cells,
                                &victim_job.clearance_exempt_cell_keys,
                                core_radius_cells,
                                &temporary_probe_reservation,
                                victim_job.source_port_um,
                                victim_job.target_port_um,
                                prefer_orthogonal_repair,
                            );
                            let reroute_elapsed_us = native_batch_elapsed_us(reroute_start);
                            timings.reroute_victims_wall_us += reroute_elapsed_us;
                            let route = match reroute_result {
                                Ok(route) => {
                                    timings
                                        .add_route_result_stats_if(collect_native_timing, &route);
                                    push_native_repair_trace(
                                        &mut repair_trace,
                                        "victim_reroute",
                                        Some(route_order),
                                        Some("normal_route"),
                                        victim_job.net_id,
                                        Some(round_idx),
                                        Some(active_repair_set_index as u64),
                                        &candidate_blockers,
                                        &ripup_ids,
                                        &victim_reroute_ids,
                                        Some(victim_first),
                                        Some(reverse_victim_order),
                                        Some(true),
                                        None,
                                    );
                                    route
                                }
                                Err(normal_error) => {
                                    enqueue_targeted_illegal_crossing_repair_set(
                                        &mut repair_victim_sets,
                                        &mut candidate_blockers,
                                        &round_base_routes,
                                        victim_job.net_id,
                                        &ripup_ids,
                                        &normal_error,
                                        round_idx,
                                        max_rounds,
                                        max_victims,
                                    );
                                    let learned_repair_keepout =
                                        learned_repair_keepouts_by_ripup
                                            .entry(ripup_ids.clone())
                                            .or_default();
                                    let victim_only_keepout =
                                        learned_victim_only_keepouts_by_ripup
                                            .entry(ripup_ids.clone())
                                            .or_default();
                                    if self.remember_victim_repair_error_keepout(
                                        learned_repair_keepout,
                                        victim_only_keepout,
                                        &normal_error,
                                        job.net_id,
                                    ) {
                                        enqueue_learned_keepout_repair_retry(
                                            &mut repair_victim_sets,
                                            &mut learned_repair_retry_counts,
                                            &ripup_ids,
                                            round_idx,
                                            repair_set_index,
                                        );
                                    }
                                    let (repair_keepout, extra_repair_keepout) = self
                                        .augmented_crossing_error_repair_keepout(
                                            &temporary_probe_reservation,
                                            &normal_error,
                                        );
                                    if !extra_repair_keepout.is_empty() {
                                        self.obstacle_map.add_static_keys(&extra_repair_keepout);
                                    }
                                    timings.reroute_victims_failed_wall_us += reroute_elapsed_us;
                                    push_native_repair_trace(
                                        &mut repair_trace,
                                        "victim_reroute",
                                        Some(route_order),
                                        Some("normal_route"),
                                        victim_job.net_id,
                                        Some(round_idx),
                                        Some(active_repair_set_index as u64),
                                        &candidate_blockers,
                                        &ripup_ids,
                                        &victim_reroute_ids,
                                        Some(victim_first),
                                        Some(reverse_victim_order),
                                        Some(false),
                                        Some(normal_error.clone()),
                                    );
                                    attempts.push(NativeRouteAttempt {
                                        bucket_name: "reroute_victims",
                                        net_id: victim_job.net_id,
                                        route: None,
                                        failed: true,
                                        error: Some(normal_error),
                                        repair_round: Some(round_idx),
                                        candidate_blockers: candidate_blockers.clone(),
                                        ripup_ids: ripup_ids.clone(),
                                    });
                                    let repair_start = native_batch_timer(collect_native_timing);
                                    let repair_result = self
                                        .route_single_net_and_commit_repair_native_with_repair_keepout(
                                            victim_job.net_id,
                                            victim_job.source,
                                            victim_job.target,
                                            block_radius_cells,
                                            &victim_job.opened_cells,
                                            &victim_job.opened_cell_keys,
                                            history_weight,
                                            commit_radius_cells,
                                            &victim_job.clearance_exempt_cells,
                                            &victim_job.clearance_exempt_cell_keys,
                                            core_radius_cells,
                                            &repair_keepout,
                                            victim_job.source_port_um,
                                            victim_job.target_port_um,
                                        );
                                    let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                                    if !extra_repair_keepout.is_empty() {
                                        self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                                    }
                                    timings.reroute_victims_wall_us += repair_elapsed_us;
                                    match repair_result {
                                        Ok(route) => {
                                            timings.add_route_result_stats_if(
                                                collect_native_timing,
                                                &route,
                                            );
                                            push_native_repair_trace(
                                                &mut repair_trace,
                                                "victim_reroute",
                                                Some(route_order),
                                                Some("repair_fallback"),
                                                victim_job.net_id,
                                                Some(round_idx),
                                                Some(active_repair_set_index as u64),
                                                &candidate_blockers,
                                                &ripup_ids,
                                                &victim_reroute_ids,
                                                Some(victim_first),
                                                Some(reverse_victim_order),
                                                Some(true),
                                                None,
                                            );
                                            route
                                        }
                                        Err(error) => {
                                            enqueue_targeted_illegal_crossing_repair_set(
                                                &mut repair_victim_sets,
                                                &mut candidate_blockers,
                                                &round_base_routes,
                                                victim_job.net_id,
                                                &ripup_ids,
                                                &error,
                                                round_idx,
                                                max_rounds,
                                                max_victims,
                                            );
                                            let learned_repair_keepout =
                                                learned_repair_keepouts_by_ripup
                                                    .entry(ripup_ids.clone())
                                                    .or_default();
                                            let victim_only_keepout =
                                                learned_victim_only_keepouts_by_ripup
                                                    .entry(ripup_ids.clone())
                                                    .or_default();
                                            if self.remember_victim_repair_error_keepout(
                                                learned_repair_keepout,
                                                victim_only_keepout,
                                                &error,
                                                job.net_id,
                                            ) {
                                                enqueue_learned_keepout_repair_retry(
                                                    &mut repair_victim_sets,
                                                    &mut learned_repair_retry_counts,
                                                    &ripup_ids,
                                                    round_idx,
                                                    repair_set_index,
                                                );
                                            }
                                            timings.reroute_victims_failed_wall_us +=
                                                repair_elapsed_us;
                                            push_native_repair_trace(
                                                &mut repair_trace,
                                                "victim_reroute",
                                                Some(route_order),
                                                Some("repair_fallback"),
                                                victim_job.net_id,
                                                Some(round_idx),
                                                Some(active_repair_set_index as u64),
                                                &candidate_blockers,
                                                &ripup_ids,
                                                &victim_reroute_ids,
                                                Some(victim_first),
                                                Some(reverse_victim_order),
                                                Some(false),
                                                Some(error.clone()),
                                            );
                                            attempts.push(NativeRouteAttempt {
                                                bucket_name: "reroute_victims",
                                                net_id: victim_job.net_id,
                                                route: None,
                                                failed: true,
                                                error: Some(error),
                                                repair_round: Some(round_idx),
                                                candidate_blockers: candidate_blockers.clone(),
                                                ripup_ids: ripup_ids.clone(),
                                            });
                                            mode_failed = true;
                                            break;
                                        }
                                    }
                                }
                            };
                            attempts.push(NativeRouteAttempt {
                                bucket_name: "reroute_victims",
                                net_id: victim_job.net_id,
                                route: Some(route.clone()),
                                failed: false,
                                error: None,
                                repair_round: Some(round_idx),
                                candidate_blockers: candidate_blockers.clone(),
                                ripup_ids: ripup_ids.clone(),
                            });
                            final_routes.insert(victim_job.net_id, route);
                        }
                    }

                    if !mode_failed
                        && victim_first
                        && temporary_probe_reservation_added
                        && !victim_first_probe_reservation.is_empty()
                    {
                        self.obstacle_map
                            .remove_static_keys(&victim_first_probe_reservation);
                        for key in &victim_first_probe_reservation {
                            temporary_probe_reservation.remove(key);
                        }
                        if temporary_probe_reservation.is_empty() {
                            temporary_probe_reservation_added = false;
                        }
                    }

                    if !mode_failed && victim_first {
                        let route_start = native_batch_timer(collect_native_timing);
                        let normal_result =
                            self.route_single_net_and_commit_native_with_optional_orthogonal_repair_keepout(
                            job.net_id,
                            job.source,
                            job.target,
                            block_radius_cells,
                            &job.opened_cells,
                            &job.opened_cell_keys,
                            commit_radius_cells,
                            &job.clearance_exempt_cells,
                            &job.clearance_exempt_cell_keys,
                            core_radius_cells,
                            &temporary_probe_reservation,
                            job.source_port_um,
                            job.target_port_um,
                            prefer_orthogonal_repair,
                        );
                        let route_elapsed_us = native_batch_elapsed_us(route_start);
                        timings.repair_failed_net_wall_us += route_elapsed_us;
                        let route = match normal_result {
                            Ok(route) => {
                                timings.add_route_result_stats_if(collect_native_timing, &route);
                                push_native_repair_trace(
                                    &mut repair_trace,
                                    "current_route",
                                    Some(route_order),
                                    Some("normal_route"),
                                    job.net_id,
                                    Some(round_idx),
                                    Some(active_repair_set_index as u64),
                                    &candidate_blockers,
                                    &ripup_ids,
                                    &victim_reroute_ids,
                                    Some(victim_first),
                                    Some(reverse_victim_order),
                                    Some(true),
                                    None,
                                );
                                route
                            }
                            Err(normal_error) => {
                                enqueue_targeted_illegal_crossing_repair_set(
                                    &mut repair_victim_sets,
                                    &mut candidate_blockers,
                                    &round_base_routes,
                                    job.net_id,
                                    &ripup_ids,
                                    &normal_error,
                                    round_idx,
                                    max_rounds,
                                    max_victims,
                                );
                                let learned_repair_keepout =
                                    learned_repair_keepouts_by_ripup
                                        .entry(ripup_ids.clone())
                                        .or_default();
                                if self.remember_local_repair_error_keepout(
                                    learned_repair_keepout,
                                    &normal_error,
                                ) {
                                    enqueue_learned_keepout_repair_retry(
                                        &mut repair_victim_sets,
                                        &mut learned_repair_retry_counts,
                                        &ripup_ids,
                                        round_idx,
                                        repair_set_index,
                                    );
                                }
                                let (repair_keepout, extra_repair_keepout) = self
                                    .augmented_crossing_error_repair_keepout(
                                        &temporary_probe_reservation,
                                        &normal_error,
                                    );
                                if !extra_repair_keepout.is_empty() {
                                    self.obstacle_map.add_static_keys(&extra_repair_keepout);
                                }
                                timings.repair_failed_net_failed_wall_us += route_elapsed_us;
                                push_native_repair_trace(
                                    &mut repair_trace,
                                    "current_route",
                                    Some(route_order),
                                    Some("normal_route"),
                                    job.net_id,
                                    Some(round_idx),
                                    Some(active_repair_set_index as u64),
                                    &candidate_blockers,
                                    &ripup_ids,
                                    &victim_reroute_ids,
                                    Some(victim_first),
                                    Some(reverse_victim_order),
                                    Some(false),
                                    Some(normal_error.clone()),
                                );
                                attempts.push(NativeRouteAttempt {
                                    bucket_name: "repair_failed_net",
                                    net_id: job.net_id,
                                    route: None,
                                    failed: true,
                                    error: Some(normal_error),
                                    repair_round: Some(round_idx),
                                    candidate_blockers: candidate_blockers.clone(),
                                    ripup_ids: ripup_ids.clone(),
                                });
                                let repair_start = native_batch_timer(collect_native_timing);
                                let repair_result = self
                                    .route_single_net_and_commit_repair_native_with_repair_keepout(
                                    job.net_id,
                                    job.source,
                                    job.target,
                                    block_radius_cells,
                                    &job.opened_cells,
                                    &job.opened_cell_keys,
                                    history_weight,
                                    commit_radius_cells,
                                    &job.clearance_exempt_cells,
                                    &job.clearance_exempt_cell_keys,
                                    core_radius_cells,
                                    &repair_keepout,
                                    job.source_port_um,
                                    job.target_port_um,
                                );
                                let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                                if !extra_repair_keepout.is_empty() {
                                    self.obstacle_map.remove_static_keys(&extra_repair_keepout);
                                }
                                timings.repair_failed_net_wall_us += repair_elapsed_us;
                                match repair_result {
                                    Ok(route) => {
                                        timings.add_route_result_stats_if(
                                            collect_native_timing,
                                            &route,
                                        );
                                        push_native_repair_trace(
                                            &mut repair_trace,
                                            "current_route",
                                            Some(route_order),
                                            Some("repair_fallback"),
                                            job.net_id,
                                            Some(round_idx),
                                            Some(active_repair_set_index as u64),
                                            &candidate_blockers,
                                            &ripup_ids,
                                            &victim_reroute_ids,
                                            Some(victim_first),
                                            Some(reverse_victim_order),
                                            Some(true),
                                            None,
                                        );
                                        route
                                    }
                                    Err(error) => {
                                        enqueue_targeted_illegal_crossing_repair_set(
                                            &mut repair_victim_sets,
                                            &mut candidate_blockers,
                                            &round_base_routes,
                                            job.net_id,
                                            &ripup_ids,
                                            &error,
                                            round_idx,
                                            max_rounds,
                                            max_victims,
                                        );
                                        let learned_repair_keepout =
                                            learned_repair_keepouts_by_ripup
                                                .entry(ripup_ids.clone())
                                                .or_default();
                                        if self.remember_local_repair_error_keepout(
                                            learned_repair_keepout,
                                            &error,
                                        ) {
                                            enqueue_learned_keepout_repair_retry(
                                                &mut repair_victim_sets,
                                                &mut learned_repair_retry_counts,
                                                &ripup_ids,
                                                round_idx,
                                                repair_set_index,
                                            );
                                        }
                                        timings.repair_failed_net_failed_wall_us +=
                                            repair_elapsed_us;
                                        push_native_repair_trace(
                                            &mut repair_trace,
                                            "current_route",
                                            Some(route_order),
                                            Some("repair_fallback"),
                                            job.net_id,
                                            Some(round_idx),
                                            Some(active_repair_set_index as u64),
                                            &candidate_blockers,
                                            &ripup_ids,
                                            &victim_reroute_ids,
                                            Some(victim_first),
                                            Some(reverse_victim_order),
                                            Some(false),
                                            Some(error.clone()),
                                        );
                                        attempts.push(NativeRouteAttempt {
                                            bucket_name: "repair_failed_net",
                                            net_id: job.net_id,
                                            route: None,
                                            failed: true,
                                            error: Some(error),
                                            repair_round: Some(round_idx),
                                            candidate_blockers: candidate_blockers.clone(),
                                            ripup_ids: ripup_ids.clone(),
                                        });
                                        if temporary_probe_reservation_added {
                                            self.obstacle_map
                                                .remove_static_keys(&temporary_probe_reservation);
                                        }
                                        continue;
                                    }
                                }
                            }
                        };
                        attempts.push(NativeRouteAttempt {
                            bucket_name: "repair_failed_net",
                            net_id: job.net_id,
                            route: Some(route.clone()),
                            failed: false,
                            error: None,
                            repair_round: Some(round_idx),
                            candidate_blockers: candidate_blockers.clone(),
                            ripup_ids: ripup_ids.clone(),
                        });
                        final_routes.insert(job.net_id, route.clone());
                        repaired_route = Some(route);
                    }

                    if victim_reroute_only_reservation_added {
                        self.obstacle_map
                            .remove_static_keys(&victim_reroute_only_reservation);
                    }

                    if temporary_probe_reservation_added {
                        self.obstacle_map
                            .remove_static_keys(&temporary_probe_reservation);
                    }

                    push_native_repair_trace(
                        &mut repair_trace,
                        "repair_mode_result",
                        Some(route_order),
                        None,
                        job.net_id,
                        Some(round_idx),
                        Some(active_repair_set_index as u64),
                        &candidate_blockers,
                        &ripup_ids,
                        &victim_reroute_ids,
                        Some(victim_first),
                        Some(reverse_victim_order),
                        Some(!mode_failed && repaired_route.is_some()),
                        if mode_failed {
                            Some("mode_failed".to_string())
                        } else if repaired_route.is_none() {
                            Some("no_repaired_route".to_string())
                        } else {
                            None
                        },
                    );

                    if !mode_failed && repaired_route.is_some() {
                        repaired = true;
                        repair_count += 1;
                        break;
                    }
                    }
                    if repaired {
                        break;
                    }
                }
                if repaired {
                    break;
                }
            }

            if !repaired {
                let reset_start = native_batch_timer(collect_native_timing);
                self.obstacle_map = round_base_map;
                self.committed_center_routes = round_base_center_routes;
                self.committed_realized_center_routes = round_base_realized_center_routes;
                self.committed_opened_cell_keys = round_base_opened_cell_keys;
                self.crossing_events = round_base_crossing_events;
                self.invalidate_meander_base_prefix();
                final_routes = round_base_routes;
                timings.repair_state_reset_us += native_batch_elapsed_us(reset_start);
                let allow_lidar_pure_probe_commit = crossing_repair_enabled
                    && !self.crossing_context.config().allow_only_expected_pairs
                    && probe_grid_crossing_violations.is_empty()
                    && !probe_realized_crossing_violations.is_empty();
                if allow_lidar_pure_probe_commit {
                    let validate_lidar_pure_probe_commit = true;
                    let commit_start = native_batch_timer(collect_native_timing);
                    let commit_result = self.commit_native_route_with_clearance_allowing_core_overlap(
                        job.net_id,
                        &probe_route,
                        block_radius_cells,
                        commit_radius_cells,
                        &job.clearance_exempt_cells,
                        core_radius_cells,
                        job.source_port_um,
                        job.target_port_um,
                        Some(&job.opened_cell_keys),
                        &candidate_blockers,
                        validate_lidar_pure_probe_commit,
                    );
                    let commit_elapsed_us = native_batch_elapsed_us(commit_start);
                    timings.commit_update_dynamic_map_us += commit_elapsed_us;
                    match commit_result {
                        Ok(true) => {
                            attempts.push(NativeRouteAttempt {
                                bucket_name: "lidar_pure_probe_commit",
                                net_id: job.net_id,
                                route: Some(probe_route.clone()),
                                failed: false,
                                error: None,
                                repair_round: Some(max_rounds),
                                candidate_blockers: candidate_blockers.clone(),
                                ripup_ids: Vec::new(),
                            });
                            final_routes.insert(job.net_id, probe_route);
                            repair_count = repair_count.saturating_add(1);
                            continue 'route_jobs;
                        }
                        Err(error) if validate_lidar_pure_probe_commit => {
                            attempts.push(NativeRouteAttempt {
                                bucket_name: "lidar_pure_probe_commit",
                                net_id: job.net_id,
                                route: None,
                                failed: true,
                                error: Some(error.clone()),
                                repair_round: Some(max_rounds),
                                candidate_blockers: candidate_blockers.clone(),
                                ripup_ids: Vec::new(),
                            });
                            let mut validation_keepout =
                                self.crossing_error_repair_keepout_keys_with_options(
                                    &error,
                                    true,
                                );
                            if !validation_keepout.is_empty() {
                                let repair_start = native_batch_timer(collect_native_timing);
                                let mut repair_result = Err(error.clone());
                                for _feedback_attempt in 0..12 {
                                    self.obstacle_map.add_static_keys(&validation_keepout);
                                    let probe_result = self.route_single_net_ignore_dynamic_native(
                                        job.source,
                                        job.target,
                                        Some(&job.opened_cells),
                                        Some(&job.opened_cell_keys),
                                    );
                                    self.obstacle_map.remove_static_keys(&validation_keepout);

                                    let attempt_result = match probe_result {
                                        Ok(route) => {
                                            match self.commit_native_route_with_clearance_allowing_core_overlap(
                                                job.net_id,
                                                &route,
                                                block_radius_cells,
                                                commit_radius_cells,
                                                &job.clearance_exempt_cells,
                                                core_radius_cells,
                                                job.source_port_um,
                                                job.target_port_um,
                                                Some(&job.opened_cell_keys),
                                                &candidate_blockers,
                                                true,
                                            ) {
                                                Ok(true) => Ok(route),
                                                Ok(false) => Err(
                                                    "Failed to commit validation-feedback route"
                                                        .to_string(),
                                                ),
                                                Err(error) => Err(error),
                                            }
                                        }
                                        Err(error) => Err(error),
                                    };

                                    match attempt_result {
                                        Ok(route) => {
                                            repair_result = Ok(route);
                                            break;
                                        }
                                        Err(retry_error) => {
                                            let extra_keepout = self
                                                .crossing_error_repair_keepout_keys_with_options(
                                                    &retry_error,
                                                    true,
                                                );
                                            let mut added_any = false;
                                            for key in extra_keepout {
                                                if validation_keepout.insert(key) {
                                                    added_any = true;
                                                }
                                            }
                                            repair_result = Err(retry_error);
                                            if !added_any {
                                                break;
                                            }
                                        }
                                    }
                                }
                                let repair_elapsed_us = native_batch_elapsed_us(repair_start);
                                timings.repair_failed_net_wall_us += repair_elapsed_us;
                                match repair_result {
                                    Ok(route) => {
                                        timings
                                            .add_route_result_stats_if(collect_native_timing, &route);
                                        attempts.push(NativeRouteAttempt {
                                            bucket_name: "repair_failed_net",
                                            net_id: job.net_id,
                                            route: Some(route.clone()),
                                            failed: false,
                                            error: None,
                                            repair_round: Some(max_rounds),
                                            candidate_blockers: candidate_blockers.clone(),
                                            ripup_ids: Vec::new(),
                                        });
                                        final_routes.insert(job.net_id, route);
                                        repair_count = repair_count.saturating_add(1);
                                        continue 'route_jobs;
                                    }
                                    Err(retry_error) => {
                                        timings.repair_failed_net_failed_wall_us +=
                                            repair_elapsed_us;
                                        attempts.push(NativeRouteAttempt {
                                            bucket_name: "repair_failed_net",
                                            net_id: job.net_id,
                                            route: None,
                                            failed: true,
                                            error: Some(retry_error),
                                            repair_round: Some(max_rounds),
                                            candidate_blockers: candidate_blockers.clone(),
                                            ripup_ids: Vec::new(),
                                        });
                                    }
                                }
                            }
                        }
                        Ok(false) | Err(_) => {}
                    }
                }
                failed_net_id = Some(job.net_id);
                let recent_errors: Vec<String> = attempts
                    .iter()
                    .rev()
                    .filter(|attempt| {
                        attempt.repair_round.is_some()
                            && (attempt.net_id == job.net_id
                                || candidate_blockers.contains(&attempt.net_id))
                    })
                    .filter_map(|attempt| {
                        attempt.error.as_ref().map(|error| {
                            format!(
                                "{}:net{}:round{:?}:rip{:?}:{}",
                                attempt.bucket_name,
                                attempt.net_id,
                                attempt.repair_round,
                                attempt.ripup_ids,
                                error
                            )
                        })
                    })
                    .take(8)
                    .collect();
                failed_error = Some(format!(
                    "No repair route found; candidate_blockers={candidate_blockers:?}; recent_errors={recent_errors:?}"
                ));
                break;
            }
        }
        if trace_native_progress {
            if let Some(last_start) = trace_last_route_start {
                eprintln!(
                    "native_route_elapsed previous_index={} elapsed_s={:.6}",
                    native_jobs.len(),
                    last_start.elapsed().as_secs_f64()
                );
            }
        }

        let result_dict = PyDict::new_bound(py);
        let route_entries = PyList::empty_bound(py);
        for job in &native_jobs {
            if let Some(route_result) = final_routes.get(&job.net_id) {
                let entry = PyDict::new_bound(py);
                let route_construct_start = native_batch_timer(collect_native_timing);
                let route_obj = Py::new(py, convert_result(py, &self.primitives, route_result)?)?;
                timings.route_result_construction_us +=
                    native_batch_elapsed_us(route_construct_start);
                let dict_start = native_batch_timer(collect_native_timing);
                entry.set_item("net_id", job.net_id)?;
                entry.set_item("route", route_obj)?;
                route_entries.append(entry)?;
                timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
            }
        }
        let attempt_entries = PyList::empty_bound(py);
        for attempt in attempts {
            let entry = PyDict::new_bound(py);
            let route_obj = if let Some(route) = attempt.route.as_ref() {
                let route_construct_start = native_batch_timer(collect_native_timing);
                let route_obj = Py::new(py, convert_result(py, &self.primitives, route)?)?;
                timings.route_result_construction_us +=
                    native_batch_elapsed_us(route_construct_start);
                Some(route_obj)
            } else {
                None
            };
            let dict_start = native_batch_timer(collect_native_timing);
            entry.set_item("bucket_name", attempt.bucket_name)?;
            entry.set_item("net_id", attempt.net_id)?;
            entry.set_item("failed", attempt.failed)?;
            entry.set_item("error", attempt.error)?;
            entry.set_item("repair_round", attempt.repair_round)?;
            entry.set_item("candidate_blockers", attempt.candidate_blockers)?;
            entry.set_item("ripup_ids", attempt.ripup_ids)?;
            if let Some(route_obj) = route_obj {
                entry.set_item("route", route_obj)?;
            } else {
                entry.set_item("route", py.None())?;
            }
            attempt_entries.append(entry)?;
            timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
        }
        let repair_trace_entries = PyList::empty_bound(py);
        for event in repair_trace {
            let entry = PyDict::new_bound(py);
            let dict_start = native_batch_timer(collect_native_timing);
            entry.set_item("event", event.event_name)?;
            entry.set_item("route_order", event.route_order)?;
            entry.set_item("action", event.action)?;
            entry.set_item("net_id", event.net_id)?;
            entry.set_item("repair_round", event.repair_round)?;
            entry.set_item("repair_set_index", event.repair_set_index)?;
            entry.set_item("candidate_blockers", event.candidate_blockers)?;
            entry.set_item("ripup_ids", event.ripup_ids)?;
            entry.set_item("victim_order", event.victim_order)?;
            entry.set_item("victim_first", event.victim_first)?;
            entry.set_item("reverse_victim_order", event.reverse_victim_order)?;
            entry.set_item("success", event.success)?;
            entry.set_item("error", event.error)?;
            repair_trace_entries.append(entry)?;
            timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
        }
        let dict_start = native_batch_timer(collect_native_timing);
        result_dict.set_item(
            "status",
            if failed_net_id.is_some() {
                "failed"
            } else {
                "routed"
            },
        )?;
        result_dict.set_item("failed_net_id", failed_net_id)?;
        result_dict.set_item("error", failed_error)?;
        result_dict.set_item("repair_count", repair_count)?;
        result_dict.set_item("routes", route_entries)?;
        result_dict.set_item("attempts", attempt_entries)?;
        result_dict.set_item("repair_trace", repair_trace_entries)?;
        timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
        result_dict.set_item("timings_s", native_batch_timings_to_py_dict(py, &timings)?)?;
        Ok(result_dict.into())
    }

    fn ripup_route(&mut self, net_id: u64) -> bool {
        self.remove_crossing_events_for_net(net_id);
        let removed = self.obstacle_map.ripup_route(net_id);
        if removed {
            self.committed_center_routes.remove(&net_id);
            self.committed_realized_center_routes.remove(&net_id);
            self.committed_opened_cell_keys.remove(&net_id);
            self.invalidate_meander_base_prefix();
        }
        removed
    }

    fn clear_dynamic(&mut self) {
        self.remove_crossing_events_for_all_routes();
        self.obstacle_map.clear_dynamic();
        self.committed_center_routes.clear();
        self.committed_realized_center_routes.clear();
        self.committed_opened_cell_keys.clear();
        self.crossing_events.clear();
        self.invalidate_meander_base_prefix();
    }

    fn get_net_cells(&self, net_id: u64) -> Vec<(i32, i32)> {
        self.obstacle_map
            .get_net_cells(net_id)
            .map(|cells| cells.iter().copied().map(unpack_xy).collect())
            .unwrap_or_default()
    }

    fn get_net_core_cells(&self, net_id: u64) -> Vec<(i32, i32)> {
        self.obstacle_map
            .get_net_core_cells(net_id)
            .map(|cells| cells.iter().copied().map(unpack_xy).collect())
            .unwrap_or_default()
    }

    fn raw_dynamic_obstacle_cells(&self) -> Vec<(i32, i32, u16)> {
        let mut cells: Vec<(i32, i32, u16)> = self
            .obstacle_map
            .dynamic_obstacle_entries()
            .map(|(key, refs)| {
                let (x, y) = unpack_xy(key);
                (x, y, refs)
            })
            .collect();
        cells.sort_unstable();
        cells
    }

    fn raw_static_obstacle_cells(&self) -> Vec<(i32, i32)> {
        let mut cells: Vec<(i32, i32)> = self
            .obstacle_map
            .static_obstacle_keys()
            .map(unpack_xy)
            .collect();
        cells.sort_unstable();
        cells
    }

    fn raw_dynamic_core_cells(&self) -> Vec<(i32, i32, u16)> {
        let mut cells: Vec<(i32, i32, u16)> = self
            .obstacle_map
            .dynamic_core_obstacle_entries()
            .map(|(key, refs)| {
                let (x, y) = unpack_xy(key);
                (x, y, refs)
            })
            .collect();
        cells.sort_unstable();
        cells
    }

    fn all_net_route_cells(&self) -> Vec<(u64, Vec<(i32, i32)>)> {
        let mut routes: Vec<(u64, Vec<(i32, i32)>)> = self
            .obstacle_map
            .net_route_entries()
            .map(|(net_id, cells)| {
                (
                    net_id,
                    cells.iter().copied().map(unpack_xy).collect::<Vec<_>>(),
                )
            })
            .collect();
        routes.sort_unstable_by_key(|(net_id, _)| *net_id);
        routes
    }

    fn all_net_core_cells(&self) -> Vec<(u64, Vec<(i32, i32)>)> {
        let mut routes: Vec<(u64, Vec<(i32, i32)>)> = self
            .obstacle_map
            .net_route_entries()
            .filter_map(|(net_id, _)| {
                self.obstacle_map.get_net_core_cells(net_id).map(|cells| {
                    (
                        net_id,
                        cells.iter().copied().map(unpack_xy).collect::<Vec<_>>(),
                    )
                })
            })
            .collect();
        routes.sort_unstable_by_key(|(net_id, _)| *net_id);
        routes
    }

    fn commit_route_cells(&mut self, net_id: u64, cells: Vec<(i32, i32)>) -> bool {
        let committed = self.obstacle_map.commit_route(net_id, &cells);
        if committed {
            self.invalidate_meander_base_prefix();
        }
        committed
    }

    #[pyo3(signature=(net_id, cells, block_radius_cells=0, commit_radius_cells=None, clearance_exempt_cells=None, core_radius_cells=None))]
    fn commit_route_with_clearance(
        &mut self,
        net_id: u64,
        cells: Vec<(i32, i32)>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<Vec<(i32, i32)>>,
        core_radius_cells: Option<i32>,
    ) -> bool {
        let route_cells = route_commit_cells(
            &cells,
            block_radius_cells,
            commit_radius_cells.unwrap_or(block_radius_cells),
            clearance_exempt_cells.as_deref(),
            self.grid.width as i32,
            self.grid.height as i32,
        );
        let core_cells = route_core_cells(
            &cells,
            core_radius_cells.unwrap_or(block_radius_cells),
            self.grid.width as i32,
            self.grid.height as i32,
        );
        let committed = self.obstacle_map.commit_route_with_clearance_overlap(
            net_id,
            &core_cells,
            &route_cells,
            clearance_exempt_cells.as_deref().unwrap_or(&[]),
        );
        if committed {
            self.remove_crossing_events_for_net(net_id);
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

    fn export_debug_svg(&self, route: &PyRouteResult) -> String {
        let r = to_route_result(route);
        append_crossing_event_svg_overlay(
            export_route_svg_with_port_open_cells(
                &self.obstacle_map,
                &r,
                Some(&self.port_open_cells),
            ),
            &self.crossing_events,
            self.grid.height as i32,
        )
    }

    fn export_debug_svg_with_obstacle_cells(
        &self,
        route: &PyRouteResult,
        obstacle_cells: Vec<(i32, i32)>,
    ) -> String {
        let r = to_route_result(route);
        let mut obstacle_map = self.obstacle_map.clone();
        obstacle_map.clear_dynamic();
        obstacle_map.add_static_cells(&obstacle_cells);
        append_crossing_event_svg_overlay(
            export_route_svg_with_port_open_cells(&obstacle_map, &r, Some(&self.port_open_cells)),
            &self.crossing_events,
            self.grid.height as i32,
        )
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

    #[pyo3(signature=(route,source_port_um=None,target_port_um=None,allow_unchecked_bumps=true))]
    fn route_port_corrected_centerline(
        &self,
        route: &PyRouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        allow_unchecked_bumps: bool,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        route_to_port_corrected_centerline_with_options_rs(
            &r,
            &self.primitives,
            &grid,
            source_port_um,
            target_port_um,
            allow_unchecked_bumps,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route))]
    fn route_primitive_centerline(&self, route: &PyRouteResult) -> PyResult<Vec<(f64, f64)>> {
        let r = to_route_result(route);
        self.realized_centerline_for_route(&r)
            .map_err(PyValueError::new_err)
    }

    #[pyo3(signature=(net_id,route,width_um,clearance_radius_cells,core_radius_cells,opened_cells=None,clearance_exempt_cells=None,source_port_um=None,target_port_um=None,allow_unchecked_fallback=true))]
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
        allow_unchecked_fallback: bool,
    ) -> PyResult<Py<PyDict>> {
        let _ = clearance_radius_cells;
        let r = to_route_result(route);
        let opened_cell_vec = opened_cells.unwrap_or_default();
        let clearance_exempt_cell_vec = clearance_exempt_cells.unwrap_or_default();
        let correction = self
            .route_port_corrected_centerline_checked_and_commit_native(
                net_id,
                &r,
                width_um,
                core_radius_cells,
                &opened_cell_vec,
                &clearance_exempt_cell_vec,
                source_port_um,
                target_port_um,
                allow_unchecked_fallback,
            )
            .map_err(PyRuntimeError::new_err)?;
        let d = PyDict::new_bound(py);
        d.set_item("centerline", correction.centerline)?;
        d.set_item("committed_bump", correction.committed_bump)?;
        d.set_item("candidate_index", correction.candidate_index)?;
        d.set_item("candidate_label", correction.candidate_label)?;
        Ok(d.into())
    }

    #[pyo3(signature=(jobs,width_um,clearance_radius_cells,core_radius_cells,allow_unchecked_fallback=true))]
    fn apply_checked_endpoint_corrections_and_commit(
        &mut self,
        py: Python<'_>,
        jobs: Vec<(
            u64,
            Py<PyRouteResult>,
            Vec<(i32, i32)>,
            Vec<(i32, i32)>,
            Option<(f64, f64)>,
            Option<(f64, f64)>,
        )>,
        width_um: f64,
        clearance_radius_cells: i32,
        core_radius_cells: i32,
        allow_unchecked_fallback: bool,
    ) -> PyResult<PyObject> {
        let _ = clearance_radius_cells;
        let entries = PyList::empty_bound(py);
        for (
            net_id,
            route_obj,
            opened_cells,
            clearance_exempt_cells,
            source_port_um,
            target_port_um,
        ) in jobs
        {
            let entry = PyDict::new_bound(py);
            entry.set_item("net_id", net_id)?;
            let route_ref = route_obj.bind(py).borrow();
            let route = to_route_result(&route_ref);
            drop(route_ref);
            match self.route_port_corrected_centerline_checked_and_commit_native(
                net_id,
                &route,
                width_um,
                core_radius_cells,
                &opened_cells,
                &clearance_exempt_cells,
                source_port_um,
                target_port_um,
                allow_unchecked_fallback,
            ) {
                Ok(correction) => {
                    let total_length_um = centerline_length_um_rs(&correction.centerline)
                        .map_err(|err| PyValueError::new_err(err.to_string()))?;
                    entry.set_item("error", py.None())?;
                    entry.set_item("centerline", correction.centerline)?;
                    entry.set_item("total_length_um", total_length_um)?;
                    entry.set_item("committed_bump", correction.committed_bump)?;
                    entry.set_item("candidate_index", correction.candidate_index)?;
                    entry.set_item("candidate_label", correction.candidate_label)?;
                }
                Err(error) => {
                    entry.set_item("error", error)?;
                    entry.set_item("centerline", py.None())?;
                    entry.set_item("total_length_um", py.None())?;
                    entry.set_item("committed_bump", false)?;
                    entry.set_item("candidate_index", py.None())?;
                    entry.set_item("candidate_label", py.None())?;
                }
            }
            entries.append(entry)?;
        }
        Ok(entries.into())
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

    #[pyo3(signature=(geometry_index,requested_extra_length_um,min_insertable_extra_length_um,max_parts=8,min_bend_radius_um=None,min_straight_um=0.0,max_meander_height_um=20.0,min_segment_length_um=10.0,auto_endpoint_inset_um=None,clearance_radius_cells=0,side_policy="both",planning_mode="fill_box_multi_bump"))]
    #[allow(clippy::too_many_arguments)]
    fn plan_auto_analytic_meander_split_request_registered_opened_auto_config(
        &self,
        py: Python<'_>,
        geometry_index: usize,
        requested_extra_length_um: f64,
        min_insertable_extra_length_um: f64,
        max_parts: usize,
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
        let result = plan_registered_geometry_split_request(
            geometry_index,
            requested_extra_length_um,
            min_insertable_extra_length_um,
            max_parts,
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

    #[pyo3(signature=(geometry_indices,requested_extra_lengths_um,min_insertable_extra_length_um,max_split_parts=8,min_bend_radius_um=None,min_straight_um=0.0,max_meander_height_um=20.0,min_segment_length_um=10.0,auto_endpoint_inset_um=None,clearance_radius_cells=0,side_policy="both",planning_mode="fill_box_multi_bump"))]
    #[allow(clippy::too_many_arguments)]
    fn plan_auto_analytic_meander_final_requests_registered_opened_auto_config(
        &self,
        py: Python<'_>,
        geometry_indices: Vec<usize>,
        requested_extra_lengths_um: Vec<f64>,
        min_insertable_extra_length_um: f64,
        max_split_parts: usize,
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
        let final_result = plan_registered_geometry_final_requests(
            &geometry_indices,
            &requested_extra_lengths_um,
            min_insertable_extra_length_um,
            max_split_parts,
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

        let py_result = registered_requirement_result_to_py_object(
            py,
            final_result.result,
            min_bend_radius_um,
            effective_radius_um,
            self.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            min_straight_um,
            min_segment_length_um,
            max_meander_height_um,
        )?;
        let py_result_dict = py_result.bind(py).downcast::<PyDict>()?;
        py_result_dict.set_item("planning_mode", final_result.planning_mode)?;
        py_result_dict.set_item("plan_input_indices", final_result.plan_input_indices)?;
        Ok(py_result)
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

        let cfg = self
            .astar_config(None, None, None)
            .map_err(PyValueError::new_err)?;

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
    m.add_class::<PyCrossingConfig>()?;
    m.add_class::<PyCrossingConstraint>()?;
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

fn native_batch_seconds(us: u128) -> f64 {
    us as f64 / 1_000_000.0
}

fn native_batch_timer(enabled: bool) -> Option<Instant> {
    enabled.then(Instant::now)
}

fn native_batch_elapsed_us(start: Option<Instant>) -> u128 {
    start.map_or(0, |start| start.elapsed().as_micros())
}

fn native_batch_timings_to_py_dict(
    py: Python<'_>,
    timings: &NativeBatchTimings,
) -> PyResult<PyObject> {
    let d = pyo3::types::PyDict::new_bound(py);
    d.set_item(
        "route_job_unpack",
        native_batch_seconds(timings.route_job_unpack_us),
    )?;
    d.set_item(
        "obstacle_map_prepare",
        native_batch_seconds(timings.obstacle_map_prepare_us),
    )?;
    d.set_item(
        "route_search_total",
        native_batch_seconds(timings.route_search_total_us),
    )?;
    d.set_item(
        "simple_route_candidate",
        native_batch_seconds(timings.simple_route_candidate_us),
    )?;
    d.set_item("dense_astar", native_batch_seconds(timings.dense_astar_us))?;
    d.set_item(
        "commit_cell_build",
        native_batch_seconds(timings.commit_cell_build_us),
    )?;
    d.set_item(
        "commit_update_dynamic_map",
        native_batch_seconds(timings.commit_update_dynamic_map_us),
    )?;
    d.set_item(
        "normal_route_wall",
        native_batch_seconds(timings.normal_route_wall_us),
    )?;
    d.set_item(
        "probe_route_wall",
        native_batch_seconds(timings.probe_route_wall_us),
    )?;
    d.set_item(
        "repair_failed_net_wall",
        native_batch_seconds(timings.repair_failed_net_wall_us),
    )?;
    d.set_item(
        "reroute_victims_wall",
        native_batch_seconds(timings.reroute_victims_wall_us),
    )?;
    d.set_item(
        "normal_route_failed_wall",
        native_batch_seconds(timings.normal_route_failed_wall_us),
    )?;
    d.set_item(
        "probe_route_failed_wall",
        native_batch_seconds(timings.probe_route_failed_wall_us),
    )?;
    d.set_item(
        "repair_failed_net_failed_wall",
        native_batch_seconds(timings.repair_failed_net_failed_wall_us),
    )?;
    d.set_item(
        "reroute_victims_failed_wall",
        native_batch_seconds(timings.reroute_victims_failed_wall_us),
    )?;
    d.set_item(
        "repair_probe_victim_selection",
        native_batch_seconds(timings.repair_probe_victim_selection_us),
    )?;
    d.set_item(
        "repair_state_reset",
        native_batch_seconds(timings.repair_state_reset_us),
    )?;
    d.set_item("ripup", native_batch_seconds(timings.ripup_us))?;
    d.set_item(
        "history_update",
        native_batch_seconds(timings.history_update_us),
    )?;
    d.set_item(
        "route_result_construction",
        native_batch_seconds(timings.route_result_construction_us),
    )?;
    d.set_item(
        "python_return_dict",
        native_batch_seconds(timings.python_return_dict_us),
    )?;
    Ok(d.into())
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
        crossing_candidate_checks: r.stats.crossing_candidate_checks,
        crossing_accepted: r.stats.crossing_accepted,
        crossing_reject_non_straight: r.stats.crossing_reject_non_straight,
        crossing_reject_not_perpendicular: r.stats.crossing_reject_not_perpendicular,
        crossing_reject_margin: r.stats.crossing_reject_margin,
        crossing_reject_wrong_order: r.stats.crossing_reject_wrong_order,
        crossing_reject_unexpected_owner: r.stats.crossing_reject_unexpected_owner,
        crossing_reject_unmatched_owner: r.stats.crossing_reject_unmatched_owner,
        crossing_reject_unmatched_centerline: r.stats.crossing_reject_unmatched_centerline,
        crossing_reject_unmatched_footprint: r.stats.crossing_reject_unmatched_footprint,
        crossing_reject_unmatched_route_centerline: r
            .stats
            .crossing_reject_unmatched_route_centerline,
        crossing_reject_unmatched_route_footprint: r
            .stats
            .crossing_reject_unmatched_route_footprint,
        crossing_reject_pending_straight: r.stats.crossing_reject_pending_straight,
        dense_grid_cells: r.stats.dense_grid_cells,
        route_search_total_time_us: {
            let clamped = r.stats.route_search_total_time_us.min(u64::MAX as u128);
            clamped as u64
        },
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
            crossing_candidate_checks: route.crossing_candidate_checks,
            crossing_accepted: route.crossing_accepted,
            crossing_reject_non_straight: route.crossing_reject_non_straight,
            crossing_reject_not_perpendicular: route.crossing_reject_not_perpendicular,
            crossing_reject_margin: route.crossing_reject_margin,
            crossing_reject_wrong_order: route.crossing_reject_wrong_order,
            crossing_reject_unexpected_owner: route.crossing_reject_unexpected_owner,
            crossing_reject_unmatched_owner: route.crossing_reject_unmatched_owner,
            crossing_reject_unmatched_centerline: route.crossing_reject_unmatched_centerline,
            crossing_reject_unmatched_footprint: route.crossing_reject_unmatched_footprint,
            crossing_reject_unmatched_route_centerline: route
                .crossing_reject_unmatched_route_centerline,
            crossing_reject_unmatched_route_footprint: route
                .crossing_reject_unmatched_route_footprint,
            crossing_reject_pending_straight: route.crossing_reject_pending_straight,
            dense_grid_cells: route.dense_grid_cells,
            route_search_total_time_us: u128::from(route.route_search_total_time_us),
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

    fn empty_test_route() -> RouteResult {
        RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: Vec::new(),
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(0, 0, 0),
            reached_target: State::new(0, 0, 0),
            stats: RouteSearchStats::default(),
        }
    }

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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        assert!(!router.primitives.get_primitives_for_angle(0).is_empty());
    }

    #[test]
    fn targeted_illegal_crossing_repair_promotes_learned_blocker() {
        let mut final_routes = FxHashMap::default();
        final_routes.insert(31, empty_test_route());
        final_routes.insert(33, empty_test_route());
        final_routes.insert(36, empty_test_route());

        let mut repair_victim_sets = vec![(1, vec![36]), (1, vec![31])];
        let mut candidate_blockers = vec![36, 31];
        enqueue_targeted_illegal_crossing_repair_set(
            &mut repair_victim_sets,
            &mut candidate_blockers,
            &final_routes,
            36,
            &[36, 31],
            "Illegal realized crossing: net 36 intersects net 33 at (0.000, 0.000) (not_perpendicular)",
            2,
            4,
            8,
        );

        assert_eq!(candidate_blockers, vec![36, 31, 33]);
        assert!(repair_victim_sets
            .iter()
            .any(|(round, ids)| *round == 3 && ids == &vec![36, 31, 33]));
    }

    #[test]
    fn dynamic_commit_error_creates_bounded_repair_keepout() {
        let router = PyPhotonicRouter::new(
            PyGridSpec::new(40, 40, 0.5, 0.0, 0.0).unwrap(),
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );

        let keepout = router.dynamic_commit_error_repair_keepout_keys(
            "Failed to commit routed cells to obstacle map: dynamic_overlap_count=2 dynamic_overlap_owners=[36] dynamic_overlap_bbox=(10,12,20,21) dynamic_overlap_sample=(10,20),(12,21)",
        );

        assert!(keepout.contains(&pack_xy(9, 19)));
        assert!(keepout.contains(&pack_xy(13, 22)));
        assert!(!keepout.contains(&pack_xy(8, 18)));
    }

    #[test]
    fn local_repair_error_keepout_learns_dynamic_overlap_for_capped_retry() {
        let router = PyPhotonicRouter::new(
            PyGridSpec::new(40, 40, 0.5, 0.0, 0.0).unwrap(),
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );

        let mut learned_keepout = FxHashSet::default();
        let learned = router.remember_local_repair_error_keepout(
            &mut learned_keepout,
            "Failed to commit routed cells to obstacle map: dynamic_overlap_count=1 dynamic_overlap_owners=[36] dynamic_overlap_bbox=(10,10,20,20) dynamic_overlap_sample=(10,20)",
        );

        assert!(learned);
        assert!(learned_keepout.contains(&pack_xy(9, 19)));
        assert!(learned_keepout.contains(&pack_xy(11, 21)));
        assert!(!learned_keepout.contains(&pack_xy(8, 18)));

        let mut repair_victim_sets = Vec::new();
        let mut retry_counts = FxHashMap::default();
        enqueue_learned_keepout_repair_retry(
            &mut repair_victim_sets,
            &mut retry_counts,
            &[36, 31],
            1,
            0,
        );
        enqueue_learned_keepout_repair_retry(
            &mut repair_victim_sets,
            &mut retry_counts,
            &[36, 31],
            1,
            0,
        );

        assert_eq!(repair_victim_sets, vec![(1, vec![36, 31])]);
        assert_eq!(
            retry_counts.get(&vec![36, 31]).copied(),
            Some(1)
        );
    }

    #[test]
    fn victim_repair_error_keepout_routes_current_owned_dynamic_overlap_to_victim_only() {
        let router = PyPhotonicRouter::new(
            PyGridSpec::new(40, 40, 0.5, 0.0, 0.0).unwrap(),
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        let error = "Failed to commit routed cells to obstacle map: dynamic_overlap_count=1 dynamic_overlap_owners=[33] dynamic_overlap_bbox=(10,10,20,20) dynamic_overlap_sample=(10,20)";
        assert_eq!(dynamic_commit_error_overlap_owner_ids(error), vec![33]);

        let mut shared_keepout = FxHashSet::default();
        let mut victim_only_keepout = FxHashSet::default();
        let learned = router.remember_victim_repair_error_keepout(
            &mut shared_keepout,
            &mut victim_only_keepout,
            error,
            33,
        );

        assert!(learned);
        assert!(shared_keepout.is_empty());
        assert!(victim_only_keepout.contains(&pack_xy(9, 19)));
        assert!(victim_only_keepout.contains(&pack_xy(11, 21)));

        let mut shared_for_other_owner = FxHashSet::default();
        let mut victim_only_for_other_owner = FxHashSet::default();
        let learned_other = router.remember_victim_repair_error_keepout(
            &mut shared_for_other_owner,
            &mut victim_only_for_other_owner,
            "Failed to commit routed cells to obstacle map: dynamic_overlap_count=1 dynamic_overlap_owners=[36] dynamic_overlap_bbox=(10,10,20,20) dynamic_overlap_sample=(10,20)",
            33,
        );

        assert!(learned_other);
        assert!(shared_for_other_owner.contains(&pack_xy(9, 19)));
        assert!(victim_only_for_other_owner.is_empty());

        let mut crossing_shared = FxHashSet::default();
        let mut crossing_victim_only = FxHashSet::default();
        let learned_crossing = router.remember_victim_repair_error_keepout(
            &mut crossing_shared,
            &mut crossing_victim_only,
            "Illegal realized crossing: net 36 intersects net 33 at (5.000, 5.000) (not_perpendicular)",
            33,
        );

        assert!(learned_crossing);
        assert!(crossing_shared.is_empty());
        assert!(crossing_victim_only.contains(&pack_xy(10, 10)));
    }

    #[test]
    fn targeted_illegal_crossing_repair_promotes_blocker_when_queue_capped() {
        let mut final_routes = FxHashMap::default();
        final_routes.insert(31, empty_test_route());
        final_routes.insert(33, empty_test_route());
        final_routes.insert(36, empty_test_route());

        let mut repair_victim_sets = vec![
            (1, vec![36]),
            (1, vec![31]),
            (2, vec![36, 31]),
            (2, vec![31, 36]),
            (3, vec![36]),
            (3, vec![31]),
            (4, vec![36, 31]),
            (4, vec![31, 36]),
        ];
        let mut candidate_blockers = vec![36, 31];
        enqueue_targeted_illegal_crossing_repair_set(
            &mut repair_victim_sets,
            &mut candidate_blockers,
            &final_routes,
            36,
            &[36, 31],
            "Illegal realized crossing: net 36 intersects net 33 at (0.000, 0.000) (not_perpendicular)",
            2,
            4,
            8,
        );

        assert_eq!(candidate_blockers, vec![36, 31, 33]);
        assert_eq!(repair_victim_sets.len(), 8);
    }

    #[test]
    fn learned_keepout_retry_is_capped_per_ripup_set() {
        let mut repair_victim_sets = Vec::new();
        let mut retry_counts = FxHashMap::default();
        for _ in 0..8 {
            let next_repair_set_index = repair_victim_sets.len();
            enqueue_learned_keepout_repair_retry(
                &mut repair_victim_sets,
                &mut retry_counts,
                &[36, 31],
                1,
                next_repair_set_index,
            );
        }

        assert_eq!(repair_victim_sets.len(), 1);
        assert_eq!(
            retry_counts.get(&vec![36, 31]).copied(),
            Some(1)
        );
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
    fn opened_cells_excluding_keepout_preserves_terminals() {
        let opened = vec![(1, 1), (2, 2), (3, 3), (4, 4)];
        let keepout: FxHashSet<CellKey> =
            [(2, 2), (3, 3), (4, 4)].into_iter().map(|(x, y)| pack_xy(x, y)).collect();
        let filtered = opened_cells_excluding_keepout(
            &opened,
            &keepout,
            PyState::new(2, 2, 0),
            PyState::new(4, 4, 0),
        );

        assert!(filtered.contains(&(2, 2)));
        assert!(filtered.contains(&(4, 4)));
        assert!(!filtered.contains(&(3, 3)));
        assert!(filtered.contains(&(1, 1)));
    }

    #[test]
    fn crossing_events_require_straight_margin_around_intersection() {
        let partner = vec![(10, 5), (10, 24)];
        let clean =
            crossing_events_for_partner(2, 1, &[(3, 12), (24, 12)], &partner, 2, 2, 0, 32, 32);
        assert_eq!(clean.len(), 1);
        assert_eq!(clean[0].point, (10.0, 12.0));

        let bend_endpoint = crossing_events_for_partner(
            2,
            1,
            &[(3, 12), (10, 12), (10, 20)],
            &partner,
            2,
            2,
            0,
            32,
            32,
        );
        assert!(bend_endpoint.is_empty());
    }

    #[test]
    fn crossing_events_allow_bend_after_crossing_runout() {
        let partner = vec![(8, 5), (8, 24)];
        let route = vec![(3, 12), (13, 12), (16, 15)];

        let events = crossing_events_for_partner(2, 1, &route, &partner, 2, 2, 3, 32, 32);

        assert_eq!(events.len(), 1);
        assert_eq!(events[0].point, (8.0, 12.0));
    }

    #[test]
    fn invalid_crossing_intersections_block_kink_crossings() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            min_straight_cells_per_crossing: 2,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(10, 5), (10, 24)]);
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);

        let clean_route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (24, 12)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(24, 12, 0),
            reached_target: State::new(24, 12, 0),
            stats: RouteSearchStats::default(),
        };
        assert!(router
            .invalid_crossing_intersections_for_route(2, &clean_route, &partner_ids)
            .is_empty());

        let kink_route = RouteResult {
            compressed_waypoints: vec![(3, 12), (10, 12), (10, 20)],
            ..clean_route
        };
        let invalid = router.invalid_crossing_intersections_for_route(2, &kink_route, &partner_ids);
        assert_eq!(invalid.len(), 1);
        assert_eq!(invalid[0].partner_net_id, 1);
    }

    #[test]
    fn committed_crossing_validation_rejects_expected_bend_crossing() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context = CrossingContext::new(
            CrossingConfig {
                enabled: true,
                crossing_half_size_cells: 2,
                min_straight_cells_per_crossing: 2,
                ..CrossingConfig::default()
            },
            vec![CrossingConstraint {
                net_id: 1,
                partner_net_id: 2,
                level: 0,
                source_depth: 0,
                target_depth: 1,
            }],
        );
        router
            .committed_center_routes
            .insert(1, vec![(10, 5), (10, 24)]);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (10, 12), (10, 20)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(10, 20, 2),
            reached_target: State::new(10, 20, 2),
            stats: RouteSearchStats::default(),
        };

        let error = router
            .validate_committed_crossings_for_route_with_ports(2, &route, None, None, None)
            .unwrap_err();
        assert!(error.contains("Illegal realized crossing"));
        assert!(error.contains("insufficient_straight_margin"));
    }

    #[test]
    fn committed_crossing_validation_rejects_opened_cell_crossing_away_from_endpoint() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context = CrossingContext::new(
            CrossingConfig {
                enabled: true,
                allow_only_expected_pairs: false,
                crossing_half_size_cells: 2,
                min_straight_cells_per_crossing: 2,
                ..CrossingConfig::default()
            },
            Vec::new(),
        );
        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(10.5, 0.5), (10.5, 20.5)]);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(7, 10), (20, 10)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(20, 10, 0),
            reached_target: State::new(20, 10, 0),
            stats: RouteSearchStats::default(),
        };
        let opened_cell_keys: FxHashSet<CellKey> = [(10, 10)]
            .into_iter()
            .map(|(x, y)| pack_xy(x, y))
            .collect();

        let violations = router.crossing_violations_for_route_with_ports(
            2,
            &route,
            None,
            None,
            Some(&opened_cell_keys),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].partner_net_id, 1);
        assert_eq!(violations[0].reason, "insufficient_straight_margin");
    }

    #[test]
    fn realized_crossing_validation_rejects_collinear_route_overlap() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context = CrossingContext::new(
            CrossingConfig {
                enabled: true,
                allow_only_expected_pairs: false,
                ..CrossingConfig::default()
            },
            Vec::new(),
        );
        router
            .committed_center_routes
            .insert(1, vec![(0, 5), (12, 5)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(0.0, 5.0), (12.0, 5.0)]);

        let violations =
            router.crossing_violations_for_realized_centerline(2, &[(4.0, 5.0), (16.0, 5.0)]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].partner_net_id, 1);
        assert_eq!(violations[0].reason, "collinear_route_overlap");
    }

    #[test]
    fn realized_crossing_validation_rejects_lidar_pure_angle_and_margin() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context = CrossingContext::new(
            CrossingConfig {
                enabled: true,
                allow_only_expected_pairs: false,
                min_straight_cells_per_crossing: 2,
                ..CrossingConfig::default()
            },
            Vec::new(),
        );
        router
            .committed_center_routes
            .insert(1, vec![(0, 10), (20, 10)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(0.0, 10.0), (20.0, 10.0)]);

        let not_perpendicular =
            router.crossing_violations_for_realized_centerline(2, &[(0.0, 0.0), (20.0, 20.0)]);

        assert_eq!(not_perpendicular.len(), 1);
        assert_eq!(not_perpendicular[0].partner_net_id, 1);
        assert_eq!(not_perpendicular[0].reason, "not_perpendicular");

        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(10.0, 0.0), (10.0, 20.0)]);
        let insufficient_margin =
            router.crossing_violations_for_realized_centerline(2, &[(9.0, 10.0), (20.0, 10.0)]);

        assert_eq!(insufficient_margin.len(), 1);
        assert_eq!(insufficient_margin[0].partner_net_id, 1);
        assert_eq!(insufficient_margin[0].reason, "insufficient_straight_margin");
    }

    #[test]
    fn collision_crossing_route_without_event_is_not_accepted() {
        let grid = PyGridSpec::new(64, 64, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context = CrossingContext::new(
            CrossingConfig {
                enabled: true,
                allow_only_expected_pairs: false,
                min_straight_cells_per_crossing: 2,
                ..CrossingConfig::default()
            },
            Vec::new(),
        );
        router
            .committed_center_routes
            .insert(1, vec![(20, 20), (30, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(20.0, 20.0), (30.0, 20.0)]);

        let opened = FxHashSet::default();
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);
        let result = router.try_route_with_collision_crossings(
            2,
            State::new(0, 0, 0),
            State::new(12, 0, 0),
            &opened,
            &router.astar_config(None, None, None).unwrap(),
            0,
            None,
            &partner_ids,
            None,
            None,
            None,
        );

        assert!(
            result.is_none(),
            "collision-crossing helper must not accept routes with zero crossing events"
        );
    }

    #[test]
    fn core_overlap_commit_rejects_lidar_pure_collinear_overlap() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context = CrossingContext::new(
            CrossingConfig {
                enabled: true,
                allow_only_expected_pairs: false,
                ..CrossingConfig::default()
            },
            Vec::new(),
        );
        let partner_cells: Vec<(i32, i32)> = (0..=12).map(|x| (x, 5)).collect();
        assert!(router.obstacle_map.commit_route(1, &partner_cells));
        router
            .committed_center_routes
            .insert(1, vec![(0, 5), (12, 5)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(0.5, 5.5), (12.5, 5.5)]);

        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: (4..=16).map(|x| (x, 5)).collect(),
            compressed_waypoints: vec![(4, 5), (16, 5)],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: State::new(16, 5, 0),
            reached_target: State::new(16, 5, 0),
            stats: RouteSearchStats::default(),
        };

        let result = router.commit_native_route_with_clearance_allowing_core_overlap(
            2,
            &route,
            0,
            Some(0),
            &[],
            Some(0),
            None,
            None,
            None,
            &[1],
            true,
        );

        assert!(
            !matches!(result, Ok(true)),
            "collinear overlap commit unexpectedly succeeded"
        );
        assert!(router.obstacle_map.get_net_cells(2).is_none());
    }

    #[test]
    fn realized_crossing_validation_defers_lidar_pure_footprint_blocker() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context = CrossingContext::new(
            CrossingConfig {
                enabled: true,
                allow_only_expected_pairs: false,
                crossing_half_size_cells: 2,
                min_straight_cells_per_crossing: 2,
                ..CrossingConfig::default()
            },
            Vec::new(),
        );
        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(10.0, 0.0), (10.0, 20.0)]);
        assert!(router.obstacle_map.commit_route(3, &[(10, 10)]));

        let violations =
            router.crossing_violations_for_realized_centerline(2, &[(0.0, 10.0), (20.0, 10.0)]);

        assert!(violations.is_empty());
    }

    #[test]
    fn realized_crossing_violations_create_targeted_repair_keepouts() {
        let grid = PyGridSpec::new(32, 32, 0.5, 100.0, 200.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 1, 1.0, true),
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            min_straight_cells_per_crossing: 2,
            ..CrossingConfig::default()
        });
        let violation = InvalidCrossingIntersection {
            net_id: 2,
            partner_net_id: 1,
            point: (104.25, 206.75),
            reason: "not_perpendicular",
        };

        let keys = router.crossing_physical_violation_repair_keepout_keys(
            std::slice::from_ref(&violation),
            &[1],
        );
        let center = router
            .grid_cell_for_physical_point((104.25, 206.75))
            .expect("violation point is in bounds");
        assert!(keys.contains(&pack_xy(center.0, center.1)));
        assert!(keys.contains(&pack_xy(center.0 + 1, center.1)));
        assert!(keys.contains(&pack_xy(center.0, center.1 - 1)));
        assert!(keys.contains(&pack_xy(center.0 + 2, center.1)));
        assert!(keys.contains(&pack_xy(center.0 + 3, center.1)));
        assert!(!keys.contains(&pack_xy(center.0 + 4, center.1)));

        let grid_violation = InvalidCrossingIntersection {
            net_id: 2,
            partner_net_id: 1,
            point: (8.5, 13.5),
            reason: "insufficient_straight_margin",
        };
        let grid_keys = router.crossing_grid_violation_repair_keepout_keys(
            std::slice::from_ref(&grid_violation),
            &[1],
        );
        assert!(grid_keys.contains(&pack_xy(8, 13)));

        let wrong_partner_keys = router.crossing_physical_violation_repair_keepout_keys(
            std::slice::from_ref(&violation),
            &[3],
        );
        assert!(wrong_partner_keys.is_empty());

    }

    #[test]
    fn router_discovered_crossing_partner_set_uses_committed_routes() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            allow_only_expected_pairs: false,
            ..CrossingConfig::default()
        });
        assert!(router.obstacle_map.commit_route(1, &[(4, 4)]));
        assert!(router.obstacle_map.commit_route(2, &[(6, 6)]));

        let partners = router.crossing_allowed_partner_set(3);
        assert!(partners.contains(&1));
        assert!(partners.contains(&2));
    }

    #[test]
    fn crossing_events_reject_overlapping_reservation_footprints() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            allow_only_expected_pairs: true,
            crossing_half_size_cells: 2,
            min_straight_cells_per_crossing: 2,
            ..CrossingConfig::default()
        });

        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (24, 12)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(24, 12, 0),
            reached_target: State::new(24, 12, 0),
            stats: RouteSearchStats::default(),
        };
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);
        partner_ids.insert(2);

        let mut first_reservation = FxHashSet::default();
        first_reservation.insert(pack_xy(10, 12));
        first_reservation.insert(pack_xy(11, 12));
        let mut second_reservation = FxHashSet::default();
        second_reservation.insert(pack_xy(11, 12));
        second_reservation.insert(pack_xy(12, 12));
        let events = vec![
            CrossingEvent {
                net_id: 3,
                partner_net_id: 1,
                point: (10.0, 12.0),
                route_segment: ((3, 12), (24, 12)),
                partner_segment: ((10, 5), (10, 24)),
                route_angle: 0,
                partner_angle: 2,
                reservation_keys: first_reservation,
            },
            CrossingEvent {
                net_id: 3,
                partner_net_id: 2,
                point: (12.0, 12.0),
                route_segment: ((3, 12), (24, 12)),
                partner_segment: ((12, 5), (12, 24)),
                route_angle: 0,
                partner_angle: 2,
                reservation_keys: second_reservation,
            },
        ];

        assert!(PyPhotonicRouter::crossing_events_cover_partners(
            &events,
            &partner_ids
        ));
        assert!(!PyPhotonicRouter::crossing_events_have_disjoint_reservations(&events));
        assert!(!router.crossing_route_satisfies_partner_constraints(
            3,
            &route,
            &partner_ids,
            &events
        ));
        assert_eq!(
            PyPhotonicRouter::crossing_partners_with_overlapping_reservations(&events).len(),
            2
        );
    }

    #[test]
    fn crossing_events_reject_static_and_unrelated_dynamic_reservation_blockers() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            allow_only_expected_pairs: true,
            crossing_half_size_cells: 2,
            min_straight_cells_per_crossing: 2,
            ..CrossingConfig::default()
        });
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (24, 12)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(24, 12, 0),
            reached_target: State::new(24, 12, 0),
            stats: RouteSearchStats::default(),
        };
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);
        let mut reservation = FxHashSet::default();
        reservation.insert(pack_xy(10, 12));
        reservation.insert(pack_xy(10, 13));
        let events = vec![CrossingEvent {
            net_id: 3,
            partner_net_id: 1,
            point: (10.0, 12.0),
            route_segment: ((3, 12), (24, 12)),
            partner_segment: ((10, 5), (10, 24)),
            route_angle: 0,
            partner_angle: 2,
            reservation_keys: reservation,
        }];

        assert!(router.crossing_route_satisfies_partner_constraints(
            3,
            &route,
            &partner_ids,
            &events
        ));

        router.obstacle_map.add_static_cells(&[(10, 13)]);
        assert!(!router.crossing_route_satisfies_partner_constraints(
            3,
            &route,
            &partner_ids,
            &events
        ));
        let static_cleanup = [pack_xy(10, 13)].into_iter().collect();
        router.obstacle_map.remove_static_keys(&static_cleanup);

        assert!(router.obstacle_map.commit_route(4, &[(10, 13)]));
        let blockers = router.crossing_reservation_blockers(3, &events);
        assert!(!blockers.is_clear());
        assert!(blockers.dynamic_blockers.contains(&4));
        assert!(!router.crossing_route_satisfies_partner_constraints(
            3,
            &route,
            &partner_ids,
            &events
        ));
    }

    #[test]
    fn crossing_candidate_keys_keep_partner_bends_blocked() {
        let partner = vec![(10, 5), (10, 15), (18, 15)];
        let keys = crossing_candidate_keys_for_partner(&partner, 2, 2, 0, 32, 32);

        assert!(keys.contains(&pack_xy(10, 10)));
        assert!(keys.contains(&pack_xy(10, 11)));
        assert!(!keys.contains(&pack_xy(10, 5)));
        assert!(!keys.contains(&pack_xy(10, 14)));
        assert!(!keys.contains(&pack_xy(10, 15)));
        assert!(keys.contains(&pack_xy(14, 15)));
        assert!(!keys.contains(&pack_xy(17, 15)));
        assert!(!keys.contains(&pack_xy(18, 15)));
    }

    #[test]
    fn crossing_spacing_history_uses_valid_straight_windows() {
        let route = vec![(2, 10), (12, 10), (12, 16)];
        let cells = crossing_spacing_history_cells_for_route(&route, 2, 1, 0, 32, 32);
        let keys: FxHashSet<CellKey> = cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect();

        assert!(keys.contains(&pack_xy(6, 9)));
        assert!(keys.contains(&pack_xy(6, 10)));
        assert!(keys.contains(&pack_xy(6, 11)));
        assert!(keys.contains(&pack_xy(11, 14)));
        assert!(keys.contains(&pack_xy(12, 14)));
        assert!(keys.contains(&pack_xy(13, 14)));
        assert!(!keys.contains(&pack_xy(2, 10)));
        assert!(!keys.contains(&pack_xy(12, 10)));
        assert!(!keys.contains(&pack_xy(12, 16)));
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
            crossing_candidate_checks: 0,
            crossing_accepted: 0,
            crossing_reject_non_straight: 0,
            crossing_reject_not_perpendicular: 0,
            crossing_reject_margin: 0,
            crossing_reject_wrong_order: 0,
            crossing_reject_unexpected_owner: 0,
            crossing_reject_unmatched_owner: 0,
            crossing_reject_unmatched_centerline: 0,
            crossing_reject_unmatched_footprint: 0,
            crossing_reject_unmatched_route_centerline: 0,
            crossing_reject_unmatched_route_footprint: 0,
            crossing_reject_pending_straight: 0,
            dense_grid_cells: 0,
            route_search_total_time_us: 0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
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
