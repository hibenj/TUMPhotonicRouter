use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::config::{
    CrossingEngineConfig, KernelDiagnostics, NegotiationConfig, NetNameTrace, RouterConfig,
    SearchOverrides, SEARCH_ENGINE_ASTAR, SEARCH_ENGINE_GRID_DIJKSTRA,
};
use crate::crossings::{CrossingConfig, CrossingConstraint};
use crate::geometry_realization::PortAccess;
use crate::meander::{
    actual_bend_radius_um_from_cells as actual_bend_radius_um_from_cells_rs,
    bend_radius_cells_from_min_radius as bend_radius_cells_from_min_radius_rs,
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
    pub(crate) fn new(
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
    pub(crate) fn new(
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
    pub(crate) fn bend_radius_cells_from_min_radius(
        min_bend_radius_um: f64,
        grid_size_um: f64,
    ) -> PyResult<i32> {
        bend_radius_cells_from_min_radius_rs(min_bend_radius_um, grid_size_um)
            .map_err(PyValueError::new_err)
    }

    #[staticmethod]
    pub(crate) fn actual_bend_radius_um_from_cells(
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
    #[pyo3(get, set)]
    pub max_search_time_ms: u64,
}

#[pymethods]
impl PyAStarConfig {
    #[new]
    #[pyo3(signature=(max_iterations=100_000,bend_weight=1.0,target_tolerance_cells=0,require_target_angle=true,allowed_target_angles=None,use_routing_window=true,routing_window_min_margin_cells=12,routing_window_scale=0.35,routing_window_max_expansions=3,routing_window_fallback_full_grid=true,routing_window_growth=0.5,max_dense_obstacle_cells=10_000_000,ignore_dynamic_obstacles=false,history_weight=0.0,proactive_congestion_weight=0.0,proactive_congestion_radius_cells=0,collect_detailed_timing=false,use_indexed_heap=false,primitive_ordering="library".to_string(),heuristic_mode="heading_aware".to_string(),heuristic_weight=1.0))]
    pub(crate) fn new(
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
            max_search_time_ms: 0,
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

pub(crate) fn validate_crossing_config(config: &PyCrossingConfig) -> PyResult<()> {
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
    pub(crate) fn new(
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

pub(crate) fn validate_crossing_constraint(constraint: &PyCrossingConstraint) -> PyResult<()> {
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
    pub(crate) fn new(
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
#[derive(Clone, Copy, Default)]
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
    pub(crate) fn new(x: i32, y: i32, angle: u8) -> Self {
        Self {
            x,
            y,
            angle: angle % 8,
        }
    }
}

#[pyclass(name = "RouteResult")]
#[derive(Default)]
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
    pub crossing_hotpath_no_contact: usize,
    #[pyo3(get)]
    pub crossing_hotpath_contact_checks: usize,
    #[pyo3(get)]
    pub crossing_hotpath_static_rejects: usize,
    #[pyo3(get)]
    pub crossing_hotpath_no_owner_contacts: usize,
    #[pyo3(get)]
    pub crossing_hotpath_single_owner_contacts: usize,
    #[pyo3(get)]
    pub crossing_hotpath_multi_owner_contacts: usize,
    #[pyo3(get)]
    pub crossing_hotpath_witness_cells_scanned: usize,
    #[pyo3(get)]
    pub crossing_hotpath_partner_segment_checks: usize,
    #[pyo3(get)]
    pub crossing_hotpath_partner_segment_bbox_rejects: usize,
    #[pyo3(get)]
    pub crossing_hotpath_intersection_hits: usize,
    #[pyo3(get)]
    pub crossing_hotpath_total_time_us: u64,
    #[pyo3(get)]
    pub crossing_hotpath_owner_scan_time_us: u64,
    #[pyo3(get)]
    pub crossing_hotpath_segment_time_us: u64,
    #[pyo3(get)]
    pub crossing_hotpath_reservation_time_us: u64,
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
    pub crossing_accepted_planned: usize,
    #[pyo3(get)]
    pub crossing_accepted_over_budget: usize,
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
    pub(crate) inner: PortAccess,
}

#[pymethods]
impl PyPortAccess {
    #[getter]
    pub(crate) fn port_name(&self) -> String {
        self.inner.port_name.clone()
    }

    #[getter]
    pub(crate) fn port_point_um(&self) -> (f64, f64) {
        self.inner.port_point_um
    }

    #[getter]
    pub(crate) fn anchor_cell(&self) -> (i32, i32) {
        self.inner.anchor_cell
    }

    #[getter]
    pub(crate) fn anchor_point_um(&self) -> (f64, f64) {
        self.inner.anchor_point_um
    }

    #[getter]
    pub(crate) fn entry_angle(&self) -> u8 {
        self.inner.entry_angle
    }
    #[getter]
    pub(crate) fn port_angle(&self) -> u8 {
        self.inner.port_angle
    }
    #[getter]
    pub(crate) fn anchor_angle(&self) -> u8 {
        self.inner.anchor_angle
    }

    #[getter]
    pub(crate) fn access_centerline_um(&self) -> Vec<(f64, f64)> {
        self.inner.access_centerline_um.clone()
    }
}

/// Binding for `crate::config::RouterConfig` (Milestone 1 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`):
/// every field the Python side used to set via a `PHOTONIC_ROUTER_*`
/// environment variable, now passed explicitly. Field names are flat,
/// prefixed by group (`negotiation_*`, `search_*`, `crossing_*`,
/// `diag_*`), and every default equals today's environment-unset default.
#[pyclass(name = "RouterConfig")]
#[derive(Clone)]
pub struct PyRouterConfig {
    #[pyo3(get)]
    pub negotiation_budget_first: u64,
    #[pyo3(get)]
    pub negotiation_budget_first_retry: u64,
    #[pyo3(get)]
    pub negotiation_budget_retry: u64,
    #[pyo3(get)]
    pub negotiation_braid_escalation: bool,
    #[pyo3(get)]
    pub negotiation_crossing_free_unplanned: bool,
    #[pyo3(get)]
    pub negotiation_disable_braid_repair: bool,
    #[pyo3(get)]
    pub negotiation_pending_straight_ripup_threshold: usize,
    #[pyo3(get)]
    pub search_astar_timeout_ms: Option<u64>,
    #[pyo3(get)]
    pub search_max_dense_states: Option<usize>,
    #[pyo3(get)]
    pub search_long_straight_congestion_weight: Option<f64>,
    /// `RouterConfig::search.engine`: `"astar"` or `"grid-dijkstra"`,
    /// validated in `new` below.
    #[pyo3(get)]
    pub search_engine: String,
    #[pyo3(get)]
    pub crossing_enable_guided_collision_crossing: bool,
    #[pyo3(get)]
    pub crossing_disable_guided_collision_crossing: bool,
    #[pyo3(get)]
    pub crossing_disable_rust_crossing_validation: bool,
    #[pyo3(get)]
    pub diag_native_progress: bool,
    #[pyo3(get)]
    pub diag_native_repair_diag: bool,
    #[pyo3(get)]
    pub diag_search_failure_diag: bool,
    #[pyo3(get)]
    pub diag_search_failure_map: Option<Vec<i32>>,
    #[pyo3(get)]
    pub diag_chain_diag: bool,
    #[pyo3(get)]
    pub diag_hot_loop_timing: bool,
    #[pyo3(get)]
    pub diag_move_diag: bool,
    #[pyo3(get)]
    pub diag_move_diag_cell: Option<(i32, i32)>,
    #[pyo3(get)]
    pub diag_pop_diag_below_y: Option<i32>,
    #[pyo3(get)]
    pub diag_probe_cells: Vec<(i32, i32)>,
    #[pyo3(get)]
    pub diag_trace_crossing: bool,
    #[pyo3(get)]
    pub diag_trace_crossing_net: Option<u64>,
    #[pyo3(get)]
    pub diag_trace_crossing_candidates: bool,
    #[pyo3(get)]
    pub diag_trace_crossing_candidate_max: usize,
    #[pyo3(get)]
    pub diag_trace_crossing_level1: bool,
    #[pyo3(get)]
    pub diag_trace_crossing_pending: bool,
    #[pyo3(get)]
    pub diag_trace_crossing_pending_threshold: Option<usize>,
    #[pyo3(get)]
    pub diag_trace_crossing_perp_reject_threshold: Option<usize>,
    #[pyo3(get)]
    pub diag_trace_partner_net: Option<u64>,
    #[pyo3(get)]
    pub diag_trace_plain_route_net: Option<u64>,
    #[pyo3(get)]
    pub diag_trace_endpoint_bump_nets: Option<Vec<String>>,
    #[pyo3(get)]
    pub diag_trace_endpoint_bump_all_nets: bool,
    #[pyo3(get)]
    pub diag_trace_endpoint_correction_net: Option<u64>,
    #[pyo3(get)]
    pub diag_crossing_mismatch_dump: bool,
    #[pyo3(get)]
    pub diag_crossing_mismatch_dump_net: Option<u64>,
    #[pyo3(get)]
    pub diag_crossing_mismatch_fatal: bool,
    #[pyo3(get)]
    pub diag_analysis_crossing_partner_counters: bool,
}

#[pymethods]
impl PyRouterConfig {
    #[new]
    #[pyo3(signature=(
        negotiation_budget_first=2_000_000,
        negotiation_budget_first_retry=10_000_000,
        negotiation_budget_retry=30_000_000,
        negotiation_braid_escalation=false,
        negotiation_crossing_free_unplanned=true,
        negotiation_disable_braid_repair=false,
        negotiation_pending_straight_ripup_threshold=100,
        search_astar_timeout_ms=None,
        search_max_dense_states=None,
        search_long_straight_congestion_weight=None,
        search_engine=SEARCH_ENGINE_ASTAR.to_string(),
        crossing_enable_guided_collision_crossing=false,
        crossing_disable_guided_collision_crossing=false,
        crossing_disable_rust_crossing_validation=false,
        diag_native_progress=false,
        diag_native_repair_diag=false,
        diag_search_failure_diag=false,
        diag_search_failure_map=None,
        diag_chain_diag=false,
        diag_hot_loop_timing=false,
        diag_move_diag=false,
        diag_move_diag_cell=None,
        diag_pop_diag_below_y=None,
        diag_probe_cells=Vec::new(),
        diag_trace_crossing=false,
        diag_trace_crossing_net=None,
        diag_trace_crossing_candidates=false,
        diag_trace_crossing_candidate_max=120,
        diag_trace_crossing_level1=false,
        diag_trace_crossing_pending=false,
        diag_trace_crossing_pending_threshold=None,
        diag_trace_crossing_perp_reject_threshold=None,
        diag_trace_partner_net=None,
        diag_trace_plain_route_net=None,
        diag_trace_endpoint_bump_nets=None,
        diag_trace_endpoint_bump_all_nets=false,
        diag_trace_endpoint_correction_net=None,
        diag_crossing_mismatch_dump=false,
        diag_crossing_mismatch_dump_net=None,
        diag_crossing_mismatch_fatal=false,
        diag_analysis_crossing_partner_counters=false,
    ))]
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        negotiation_budget_first: u64,
        negotiation_budget_first_retry: u64,
        negotiation_budget_retry: u64,
        negotiation_braid_escalation: bool,
        negotiation_crossing_free_unplanned: bool,
        negotiation_disable_braid_repair: bool,
        negotiation_pending_straight_ripup_threshold: usize,
        search_astar_timeout_ms: Option<u64>,
        search_max_dense_states: Option<usize>,
        search_long_straight_congestion_weight: Option<f64>,
        search_engine: String,
        crossing_enable_guided_collision_crossing: bool,
        crossing_disable_guided_collision_crossing: bool,
        crossing_disable_rust_crossing_validation: bool,
        diag_native_progress: bool,
        diag_native_repair_diag: bool,
        diag_search_failure_diag: bool,
        diag_search_failure_map: Option<Vec<i32>>,
        diag_chain_diag: bool,
        diag_hot_loop_timing: bool,
        diag_move_diag: bool,
        diag_move_diag_cell: Option<(i32, i32)>,
        diag_pop_diag_below_y: Option<i32>,
        diag_probe_cells: Vec<(i32, i32)>,
        diag_trace_crossing: bool,
        diag_trace_crossing_net: Option<u64>,
        diag_trace_crossing_candidates: bool,
        diag_trace_crossing_candidate_max: usize,
        diag_trace_crossing_level1: bool,
        diag_trace_crossing_pending: bool,
        diag_trace_crossing_pending_threshold: Option<usize>,
        diag_trace_crossing_perp_reject_threshold: Option<usize>,
        diag_trace_partner_net: Option<u64>,
        diag_trace_plain_route_net: Option<u64>,
        diag_trace_endpoint_bump_nets: Option<Vec<String>>,
        diag_trace_endpoint_bump_all_nets: bool,
        diag_trace_endpoint_correction_net: Option<u64>,
        diag_crossing_mismatch_dump: bool,
        diag_crossing_mismatch_dump_net: Option<u64>,
        diag_crossing_mismatch_fatal: bool,
        diag_analysis_crossing_partner_counters: bool,
    ) -> PyResult<Self> {
        if search_engine != SEARCH_ENGINE_ASTAR && search_engine != SEARCH_ENGINE_GRID_DIJKSTRA {
            return Err(PyValueError::new_err(format!(
                "search_engine must be \"{SEARCH_ENGINE_ASTAR}\" or \"{SEARCH_ENGINE_GRID_DIJKSTRA}\", got \"{search_engine}\""
            )));
        }
        Ok(Self {
            negotiation_budget_first,
            negotiation_budget_first_retry,
            negotiation_budget_retry,
            negotiation_braid_escalation,
            negotiation_crossing_free_unplanned,
            negotiation_disable_braid_repair,
            negotiation_pending_straight_ripup_threshold,
            search_astar_timeout_ms,
            search_max_dense_states,
            search_long_straight_congestion_weight,
            search_engine,
            crossing_enable_guided_collision_crossing,
            crossing_disable_guided_collision_crossing,
            crossing_disable_rust_crossing_validation,
            diag_native_progress,
            diag_native_repair_diag,
            diag_search_failure_diag,
            diag_search_failure_map,
            diag_chain_diag,
            diag_hot_loop_timing,
            diag_move_diag,
            diag_move_diag_cell,
            diag_pop_diag_below_y,
            diag_probe_cells,
            diag_trace_crossing,
            diag_trace_crossing_net,
            diag_trace_crossing_candidates,
            diag_trace_crossing_candidate_max,
            diag_trace_crossing_level1,
            diag_trace_crossing_pending,
            diag_trace_crossing_pending_threshold,
            diag_trace_crossing_perp_reject_threshold,
            diag_trace_partner_net,
            diag_trace_plain_route_net,
            diag_trace_endpoint_bump_nets,
            diag_trace_endpoint_bump_all_nets,
            diag_trace_endpoint_correction_net,
            diag_crossing_mismatch_dump,
            diag_crossing_mismatch_dump_net,
            diag_crossing_mismatch_fatal,
            diag_analysis_crossing_partner_counters,
        })
    }
}

impl From<&PyRouterConfig> for RouterConfig {
    fn from(cfg: &PyRouterConfig) -> Self {
        let trace_endpoint_bump_nets = if cfg.diag_trace_endpoint_bump_all_nets {
            NetNameTrace::All
        } else if let Some(names) = cfg.diag_trace_endpoint_bump_nets.clone() {
            NetNameTrace::Names(names)
        } else {
            NetNameTrace::None
        };
        RouterConfig {
            negotiation: NegotiationConfig {
                budget_first: cfg.negotiation_budget_first,
                budget_first_retry: cfg.negotiation_budget_first_retry,
                budget_retry: cfg.negotiation_budget_retry,
                braid_escalation: cfg.negotiation_braid_escalation,
                crossing_free_unplanned: cfg.negotiation_crossing_free_unplanned,
                disable_braid_repair: cfg.negotiation_disable_braid_repair,
                pending_straight_ripup_threshold: cfg.negotiation_pending_straight_ripup_threshold,
            },
            search: SearchOverrides {
                astar_timeout_ms: cfg.search_astar_timeout_ms,
                max_dense_states: cfg.search_max_dense_states,
                long_straight_congestion_weight: cfg.search_long_straight_congestion_weight,
                engine: cfg.search_engine.clone(),
            },
            crossing: CrossingEngineConfig {
                enable_guided_collision_crossing: cfg.crossing_enable_guided_collision_crossing,
                disable_guided_collision_crossing: cfg.crossing_disable_guided_collision_crossing,
                disable_rust_crossing_validation: cfg.crossing_disable_rust_crossing_validation,
            },
            diagnostics: KernelDiagnostics {
                native_progress: cfg.diag_native_progress,
                native_repair_diag: cfg.diag_native_repair_diag,
                search_failure_diag: cfg.diag_search_failure_diag,
                search_failure_map: cfg.diag_search_failure_map.clone(),
                chain_diag: cfg.diag_chain_diag,
                hot_loop_timing: cfg.diag_hot_loop_timing,
                move_diag: cfg.diag_move_diag,
                move_diag_cell: cfg.diag_move_diag_cell,
                pop_diag_below_y: cfg.diag_pop_diag_below_y,
                probe_cells: cfg.diag_probe_cells.clone(),
                trace_crossing: cfg.diag_trace_crossing,
                trace_crossing_net: cfg.diag_trace_crossing_net,
                trace_crossing_candidates: cfg.diag_trace_crossing_candidates,
                trace_crossing_candidate_max: cfg.diag_trace_crossing_candidate_max,
                trace_crossing_level1: cfg.diag_trace_crossing_level1,
                trace_crossing_pending: cfg.diag_trace_crossing_pending,
                trace_crossing_pending_threshold: cfg.diag_trace_crossing_pending_threshold,
                trace_crossing_perp_reject_threshold: cfg.diag_trace_crossing_perp_reject_threshold,
                trace_partner_net: cfg.diag_trace_partner_net,
                trace_plain_route_net: cfg.diag_trace_plain_route_net,
                trace_endpoint_bump_nets,
                trace_endpoint_correction_net: cfg.diag_trace_endpoint_correction_net,
                crossing_mismatch_dump: cfg.diag_crossing_mismatch_dump,
                crossing_mismatch_dump_net: cfg.diag_crossing_mismatch_dump_net,
                crossing_mismatch_fatal: cfg.diag_crossing_mismatch_fatal,
                analysis_crossing_partner_counters: cfg.diag_analysis_crossing_partner_counters,
            },
        }
    }
}
