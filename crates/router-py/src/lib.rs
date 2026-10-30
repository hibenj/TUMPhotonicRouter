// pyo3 0.22's `#[pymethods]` / `#[pyfunction]` expansion converts the returned
// `PyErr` into `PyErr`, which clippy reports at every binding; nothing here to fix.
#![allow(clippy::useless_conversion)]

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use rustc_hash::FxHashSet;

use router_core::crossings::{CrossingConfig, CrossingGuidance};
use router_core::geometry_realization::{
    centerline_length_um as centerline_length_um_rs,
    generate_waveguide_polygon as generate_waveguide_polygon_rs,
    realize_centerline_polygon_with_terminal_tangents as realize_centerline_polygon_with_terminal_tangents_rs,
    realize_route_polygon_from_auto_plan as realize_route_polygon_from_auto_plan_rs,
    realize_route_polygon_from_primitives as realize_route_polygon_from_primitives_rs,
    realize_route_polygon_with_endpoint_correction as realize_route_polygon_with_endpoint_correction_rs,
    route_to_port_corrected_centerline_with_options as route_to_port_corrected_centerline_with_options_rs,
    route_to_primitive_centerline as route_to_primitive_centerline_rs,
    splice_meander_into_centerline_range as splice_meander_into_centerline_range_rs,
    GeometryGridSpec,
};
use router_core::meander::geometry::{
    actual_bend_radius_um_from_cells as actual_bend_radius_um_from_cells_rs, MeanderSide,
    PhysicalPoint,
};
use router_core::meander::search::{
    cells_in_grid_rect as cells_in_grid_rect_rs,
    plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix as plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix_rs,
    AutoMeanderConfig, DenseOccupancyPrefix,
};
use router_core::obstacle_map::{pack_xy, unpack_xy, CellKey};
use router_core::search::astar::svg::export_route_svg_with_port_open_cells;
use router_core::search::state::State;
use router_core::search::{SearchEnvironment, SearchRequest};

use router_core::config::{GridSpec, RouterConfig};
use router_core::engine::negotiation::loop_::NegotiationParams;
use router_core::engine::*;
use router_core::error::RouterError;
use router_core::primitives::PrimitiveLibraryConfig;
use router_core::search::astar::config::AStarParams;
pub(crate) mod convert;
pub(crate) mod meander_py;
pub(crate) mod static_obstacle;
#[cfg(test)]
mod tests;
pub(crate) mod types;
pub(crate) use convert::*;
pub(crate) use meander_py::*;
pub(crate) use static_obstacle::*;
pub(crate) use types::*;

/// The Python exception of each core error kind (phase 8 slice B of the
/// 2026-10 do-over): the kinds and messages the core raised directly before.
/// A function, not `impl From<RouterError> for PyErr`: both types are
/// foreign to this crate, so the orphan rule forbids that impl here.
pub(crate) fn py_err(err: RouterError) -> PyErr {
    match err {
        RouterError::Config(message) => PyValueError::new_err(message),
    }
}

// The Python router: one core `Router` (`router_core::engine`), with every
// Python conversion done in the methods below. (A plain comment, not a doc
// comment: the class has no docstring and the stub must not change.)
#[pyclass(name = "PyPhotonicRouter")]
pub struct PyPhotonicRouter {
    pub(crate) inner: Router,
}

#[pymethods]
impl PyPhotonicRouter {
    #[new]
    #[pyo3(signature=(grid_spec, primitive_config, astar_config, router_config=None))]
    pub(crate) fn new(
        grid_spec: PyGridSpec,
        primitive_config: PyPrimitiveLibraryConfig,
        astar_config: PyAStarConfig,
        router_config: Option<PyRouterConfig>,
    ) -> Self {
        Self {
            inner: Router::new(
                GridSpec::from(&grid_spec),
                PrimitiveLibraryConfig::from(&primitive_config),
                AStarParams::from(&astar_config),
                router_config
                    .as_ref()
                    .map(RouterConfig::from)
                    .unwrap_or_default(),
            ),
        }
    }

    /// Physical waveguide width for the commit validation's
    /// parallel-overlap check (see the `route_width_um` field).
    pub(crate) fn set_route_width_um(&mut self, width_um: f64) -> PyResult<()> {
        self.inner.set_route_width_um(width_um).map_err(py_err)
    }

    pub(crate) fn crossing_config(&self) -> PyCrossingConfig {
        PyCrossingConfig::from(self.inner.crossing_context.config())
    }

    pub(crate) fn set_crossing_config(&mut self, config: PyCrossingConfig) -> PyResult<()> {
        validate_crossing_config(&config)?;
        self.inner
            .crossing_context
            .set_config(CrossingConfig::from(&config));
        Ok(())
    }

    /// Contribution 1 (crossing-guided search): planned pairs from the
    /// topology plan and the search price of a planned crossing. Soft
    /// guidance only -- pricing in `crossing_search_config`; never a
    /// whitelist (unplanned crossings stay possible at `crossing_loss`).
    #[pyo3(signature=(planned_pairs, planned_crossing_loss=0.0, single_discounted_crossing_per_pair=true))]
    pub(crate) fn set_crossing_guidance(
        &mut self,
        planned_pairs: Vec<(u64, u64)>,
        planned_crossing_loss: f64,
        single_discounted_crossing_per_pair: bool,
    ) -> PyResult<()> {
        if !planned_crossing_loss.is_finite() || planned_crossing_loss < 0.0 {
            return Err(PyValueError::new_err(
                "planned_crossing_loss must be finite and non-negative",
            ));
        }
        self.inner.crossing_context.set_guidance(
            CrossingGuidance::new(&planned_pairs, planned_crossing_loss)
                .with_single_discounted_crossing_per_pair(single_discounted_crossing_per_pair),
        );
        Ok(())
    }

    pub(crate) fn set_long_straight_exempt_net_ids(&mut self, net_ids: Vec<u64>) {
        self.inner.set_long_straight_exempt_net_ids(net_ids)
    }

    pub(crate) fn crossing_events(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        let mut out = Vec::with_capacity(self.inner.crossing_events.len());
        for event in &self.inner.crossing_events {
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

    /// The unified predicate's verdict for every realized intersection the
    /// post-commit validation assessed (do-over phase 4 slice B; empty unless
    /// `RouterConfig.crossing_unified_legality`). One dict per
    /// (net, partner, point): the point in um, the two grid segments, `legal`,
    /// `reason` (`legal`, `not_perpendicular`, `margin_short_<side>`,
    /// `too_close_<other>`, `footprint_blocked`, `missing_event`),
    /// `perpendicular`, the four straight runs and the two required margins
    /// in grid cells, the spacing inputs (slice B-2: `required_spacing_cells`,
    /// `nearest_same_route_cells` / `nearest_registered_cells` as Chebyshev
    /// centre distances or `None`, `too_close_other` as `same_route` /
    /// `registered` / `None`), `footprint_blocked`, `event_registered`, and
    /// `margins_from_grid` (false when the grid route had no intersection near
    /// the point and the realized straights were measured instead).
    pub(crate) fn crossing_verdicts(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        let mut out = Vec::with_capacity(self.inner.crossing_verdicts.len());
        for record in &self.inner.crossing_verdicts {
            let d = PyDict::new_bound(py);
            let candidate = &record.candidate;
            d.set_item("net_id", record.net_id)?;
            d.set_item("partner_net_id", record.partner_net_id)?;
            d.set_item("point", record.point)?;
            d.set_item("route_segment", record.route_segment)?;
            d.set_item("partner_segment", record.partner_segment)?;
            d.set_item("legal", record.verdict.is_legal())?;
            d.set_item("reason", record.verdict.reason())?;
            d.set_item(
                "margin_side",
                record.verdict.margin_side().map(|side| side.name()),
            )?;
            d.set_item(
                "perpendicular",
                directions_are_perpendicular(
                    candidate.route_direction,
                    candidate.partner_direction,
                ),
            )?;
            d.set_item("route_before_cells", candidate.route_before_cells)?;
            d.set_item("route_after_cells", candidate.route_after_cells)?;
            d.set_item("partner_before_cells", candidate.partner_before_cells)?;
            d.set_item("partner_after_cells", candidate.partner_after_cells)?;
            d.set_item("required_margin_cells", record.rule.required_margin_cells())?;
            d.set_item("required_after_cells", record.rule.required_after_cells())?;
            d.set_item(
                "required_spacing_cells",
                record.rule.required_spacing_cells(),
            )?;
            d.set_item(
                "nearest_same_route_cells",
                candidate.nearest_same_route_cells(),
            )?;
            d.set_item(
                "nearest_registered_cells",
                candidate.nearest_registered_cells(),
            )?;
            d.set_item(
                "too_close_other",
                record.verdict.too_close_other().map(|other| other.name()),
            )?;
            d.set_item("footprint_blocked", candidate.footprint_blocked)?;
            d.set_item("event_registered", candidate.event_registered)?;
            d.set_item("margins_from_grid", record.margins_from_grid)?;
            d.set_item("grid_size_um", self.inner.grid.grid_size_um)?;
            out.push(d.into());
        }
        Ok(out)
    }

    pub(crate) fn add_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.inner.add_static_cells(cells)
    }
    pub(crate) fn set_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.inner.set_static_cells(cells)
    }
    /// Add compact static rectangles on top of the current static map (rects
    /// and cells alike stay in place). Used for the chip-boundary keepout.
    pub(crate) fn add_static_rects(&mut self, rects: Vec<(i32, i32, i32, i32)>) {
        self.inner.add_static_rects(rects)
    }
    pub(crate) fn set_static_rects(&mut self, rects: Vec<(i32, i32, i32, i32)>) {
        self.inner.set_static_rects(rects)
    }

    #[pyo3(signature=(ports))]
    pub(crate) fn build_port_footprint_cells(
        &self,
        ports: Vec<(String, f64, f64, Option<f64>, i32, i32)>,
    ) -> Vec<(String, Vec<(i32, i32)>)> {
        self.inner.build_port_footprint_cells(ports)
    }

    #[pyo3(signature=(jobs,commit_radius_cells,min_run_in_length_cells))]
    pub(crate) fn build_dynamic_clearance_exempt_cells_for_routes(
        &self,
        jobs: Vec<(u64, PyState, PyState, Vec<(i32, i32)>)>,
        commit_radius_cells: i32,
        min_run_in_length_cells: i32,
    ) -> Vec<(u64, Vec<(i32, i32)>)> {
        self.inner.build_dynamic_clearance_exempt_cells_for_routes(
            jobs.into_iter()
                .map(|(net_id, source, target, opened_cells)| {
                    (
                        net_id,
                        State::from(source),
                        State::from(target),
                        opened_cells,
                    )
                })
                .collect(),
            commit_radius_cells,
            min_run_in_length_cells,
        )
    }

    pub(crate) fn add_port_open_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.inner.add_port_open_cells(cells)
    }
    #[pyo3(signature=(source,target,opened_cells=None))]
    pub(crate) fn route_single_net(
        &self,
        py: Python<'_>,
        source: PyState,
        target: PyState,
        opened_cells: Option<Vec<(i32, i32)>>,
    ) -> PyResult<Py<PyRouteResult>> {
        if self.inner.astar_cfg.target_tolerance_cells < 0 {
            return Err(PyValueError::new_err("target_tolerance_cells must be >= 0"));
        }
        let opened_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            &self.inner.port_open_cells
        };
        let cfg = self
            .inner
            .astar_config(None, None, None, None, None)
            .map_err(PyValueError::new_err)?;
        let env = SearchEnvironment {
            obstacle_map: &self.inner.obstacle_map,
            primitives: &self.inner.primitives,
        };
        let request = SearchRequest {
            source: State::new(source.x, source.y, source.angle),
            target: State::new(target.x, target.y, target.angle),
            port_open_cells: Some(opened_ref),
            dynamic_expansion: None,
            crossing: None,
            config: &cfg,
        };
        let result = self
            .inner
            .search_engine
            .search(&env, &request)
            .route
            .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;
        Py::new(py, convert_result(py, &self.inner.primitives, &result)?)
    }

    #[pyo3(signature=(net_id,source,target,block_radius_cells=0,opened_cells=None,commit_radius_cells=None,clearance_exempt_cells=None,core_radius_cells=None))]
    pub(crate) fn route_single_net_and_commit(
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
        // The job the negotiated loop would build for this net: the opened
        // cells default to the router's port openings, as this method
        // always did; no static-cleanup cells and no port positions.
        let opened_cells = opened_cells.unwrap_or_else(|| {
            self.inner
                .port_open_cells
                .iter()
                .copied()
                .map(unpack_xy)
                .collect()
        });
        let job = NativeRouteJob::new(
            net_id,
            State::from(source),
            State::from(target),
            opened_cells,
            clearance_exempt_cells.unwrap_or_default(),
            Vec::new(),
            None,
            None,
        );
        let radii = SearchRadii::new(block_radius_cells, commit_radius_cells, core_radius_cells);
        let result = self
            .inner
            .route_and_commit(&SearchCall::plain(&job, radii))
            .map_err(PyRuntimeError::new_err)?;
        Py::new(py, convert_result(py, &self.inner.primitives, &result)?)
    }

    /// The negotiated rip-up-and-repair loop, the repository's only batch
    /// routing entry point since Milestone 8 of
    /// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`
    /// (2026-09-24) removed the non-repair batch binding and the older
    /// 17-strategy repair chain.
    ///
    /// `no_ripup = True` switches the LiDAR-style local rip-up rule off, so
    /// a blocked net simply fails instead of displacing committed nets. With `max_rounds = 1` that is the no-repair mode
    /// (`RipupRerouteConfig(enabled=False)`), which used to be the separate
    /// non-repair binding; every other caller leaves it at its default.
    #[pyo3(signature=(jobs,block_radius_cells=0,commit_radius_cells=None,core_radius_cells=None,max_rounds=8,history_weight=2.0,history_increment=1,no_ripup=false))]
    #[allow(clippy::too_many_arguments)]
    fn route_many_with_negotiated_repair_and_commit(
        &mut self,
        py: Python<'_>,
        jobs: Vec<PyRef<'_, PyRouteJob>>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        max_rounds: u32,
        history_weight: f64,
        history_increment: u32,
        no_ripup: bool,
    ) -> PyResult<PyObject> {
        let jobs = native_jobs_from_py(&jobs);
        let params = NegotiationParams {
            block_radius_cells,
            commit_radius_cells,
            core_radius_cells,
            max_rounds,
            history_weight,
            history_increment,
            no_ripup,
        };
        // Read before the loop runs, as `SearchArgs` has always read it;
        // no step of the loop writes `astar_cfg`.
        let collect_native_timing = self.inner.astar_cfg.collect_detailed_timing;
        let (mut batch, _counters) = self
            .inner
            .run_negotiated_batch(jobs.clone(), &params)
            .map_err(py_err)?;
        self.build_native_batch_result_dict(py, &jobs, &mut batch, collect_native_timing)
    }

    pub(crate) fn ripup_route(&mut self, net_id: u64) -> bool {
        self.inner.ripup_route(net_id)
    }

    pub(crate) fn get_net_cells(&self, net_id: u64) -> Vec<(i32, i32)> {
        self.inner.get_net_cells(net_id)
    }

    pub(crate) fn get_net_core_cells(&self, net_id: u64) -> Vec<(i32, i32)> {
        self.inner.get_net_core_cells(net_id)
    }

    pub(crate) fn raw_dynamic_obstacle_cells(&self) -> Vec<(i32, i32, u16)> {
        self.inner.raw_dynamic_obstacle_cells()
    }

    pub(crate) fn raw_static_obstacle_cells(&self) -> Vec<(i32, i32)> {
        self.inner.raw_static_obstacle_cells()
    }

    pub(crate) fn raw_dynamic_core_cells(&self) -> Vec<(i32, i32, u16)> {
        self.inner.raw_dynamic_core_cells()
    }

    pub(crate) fn all_net_route_cells(&self) -> Vec<(u64, Vec<(i32, i32)>)> {
        self.inner.all_net_route_cells()
    }

    pub(crate) fn commit_route_cells(&mut self, net_id: u64, cells: Vec<(i32, i32)>) -> bool {
        self.inner.commit_route_cells(net_id, cells)
    }

    pub(crate) fn export_debug_svg(&self, route: &PyRouteResult) -> String {
        let r = to_route_result(route);
        append_crossing_event_svg_overlay(
            export_route_svg_with_port_open_cells(
                &self.inner.obstacle_map,
                &r,
                Some(&self.inner.port_open_cells),
            ),
            &self.inner.crossing_events,
            self.inner.grid.height as i32,
        )
    }

    pub(crate) fn export_debug_svg_with_obstacle_cells(
        &self,
        route: &PyRouteResult,
        obstacle_cells: Vec<(i32, i32)>,
    ) -> String {
        let r = to_route_result(route);
        let mut obstacle_map = self.inner.obstacle_map.clone();
        obstacle_map.clear_dynamic();
        obstacle_map.add_static_cells(&obstacle_cells);
        append_crossing_event_svg_overlay(
            export_route_svg_with_port_open_cells(
                &obstacle_map,
                &r,
                Some(&self.inner.port_open_cells),
            ),
            &self.inner.crossing_events,
            self.inner.grid.height as i32,
        )
    }

    #[pyo3(signature=(route,width_um))]
    pub(crate) fn realize_route_polygon(
        &self,
        route: &PyRouteResult,
        width_um: f64,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.inner.grid.grid_size_um,
            self.inner.grid.origin_x_um,
            self.inner.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        realize_route_polygon_from_primitives_rs(&r, &self.inner.primitives, &grid, width_um)
            .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route,source_port_um=None,target_port_um=None,allow_unchecked_bumps=true))]
    pub(crate) fn route_port_corrected_centerline(
        &self,
        route: &PyRouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        allow_unchecked_bumps: bool,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.inner.grid.grid_size_um,
            self.inner.grid.origin_x_um,
            self.inner.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        route_to_port_corrected_centerline_with_options_rs(
            &r,
            &self.inner.primitives,
            &grid,
            source_port_um,
            target_port_um,
            allow_unchecked_bumps,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(route))]
    pub(crate) fn route_primitive_centerline(
        &self,
        route: &PyRouteResult,
    ) -> PyResult<Vec<(f64, f64)>> {
        let r = to_route_result(route);
        self.inner
            .realized_centerline_for_route(&r)
            .map_err(PyValueError::new_err)
    }

    #[pyo3(signature=(net_id,route,width_um,clearance_radius_cells,core_radius_cells,opened_cells=None,clearance_exempt_cells=None,source_port_um=None,target_port_um=None,allow_unchecked_fallback=true))]
    pub(crate) fn route_port_corrected_centerline_checked_and_commit(
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
            .inner
            .route_port_corrected_centerline_checked_and_commit_compat_native(
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

    #[pyo3(signature=(net_id,route,width_um,clearance_radius_cells,core_radius_cells,opened_cells=None,clearance_exempt_cells=None,source_port_um=None,target_port_um=None,allow_unchecked_fallback=true))]
    pub(crate) fn route_port_corrected_centerline_checked(
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
            .inner
            .route_port_corrected_centerline_checked_native(
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
    pub(crate) fn apply_checked_endpoint_corrections_and_commit(
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
            match self
                .inner
                .route_port_corrected_centerline_checked_and_commit_compat_native(
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

    #[pyo3(signature=(jobs,width_um,clearance_radius_cells,core_radius_cells,allow_unchecked_fallback=true))]
    pub(crate) fn apply_checked_endpoint_corrections(
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
            match self.inner.route_port_corrected_centerline_checked_native(
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
                    // Later jobs in this batch must validate their candidates
                    // against this net's *corrected* geometry, not its stale
                    // grid centerline: two sibling diagonals each corrected
                    // against the other's uncorrected line can end up
                    // physically overlapping (multiportmmi_8x8 n_13/n_14 at
                    // heuristic weight 1.0, 0.44 um apart after correction).
                    self.inner
                        .remember_corrected_realized_centerline(net_id, &correction.centerline);
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
    pub(crate) fn realize_route_polygon_with_endpoint_correction(
        &self,
        route: &PyRouteResult,
        width_um: f64,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> PyResult<Vec<(f64, f64)>> {
        let grid = GeometryGridSpec::new(
            self.inner.grid.grid_size_um,
            self.inner.grid.origin_x_um,
            self.inner.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        realize_route_polygon_with_endpoint_correction_rs(
            &r,
            &self.inner.primitives,
            &grid,
            width_um,
            source_port_um,
            target_port_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))
    }

    pub(crate) fn centerline_length_um(&self, centerline: Vec<(f64, f64)>) -> PyResult<f64> {
        centerline_length_um_rs(&centerline).map_err(|err| PyValueError::new_err(err.to_string()))
    }

    #[pyo3(signature=(centerline,width_um,route,source_enabled=true,target_enabled=true))]
    pub(crate) fn realize_centerline_polygon_with_terminal_tangents(
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

    #[pyo3(signature=(route,width_um,selected_run_start_index,selected_run_end_index,meander_centerline))]
    pub(crate) fn realize_route_polygon_from_planned_auto_meander(
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
            self.inner.grid.grid_size_um,
            self.inner.grid.origin_x_um,
            self.inner.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let r = to_route_result(route);
        let base_centerline = route_to_primitive_centerline_rs(&r, &self.inner.primitives, &grid)
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

    #[pyo3(signature=(net_id,source,target,width_um,requested_extra_length_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,box_depth_um=20.0,min_segment_length_um=10.0,route_block_radius_cells=0,meander_clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump"))]
    pub(crate) fn route_single_net_with_auto_meander_and_commit(
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
        if self.inner.astar_cfg.target_tolerance_cells < 0 {
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
        let effective_radius_um = self
            .inner
            .effective_bend_radius_um(min_bend_radius_um)
            .map_err(py_err)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            self.inner.primitive_cfg.bend_radius_cells,
            self.inner.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;

        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&self.inner.port_open_cells)
        };

        let cfg = self
            .inner
            .astar_config(None, None, None, None, None)
            .map_err(PyValueError::new_err)?;

        let env = SearchEnvironment {
            obstacle_map: &self.inner.obstacle_map,
            primitives: &self.inner.primitives,
        };
        let request = SearchRequest {
            source: State::new(source.x, source.y, source.angle),
            target: State::new(target.x, target.y, target.angle),
            port_open_cells: opened_ref,
            dynamic_expansion: None,
            crossing: None,
            config: &cfg,
        };
        let result = self
            .inner
            .search_engine
            .search(&env, &request)
            .route
            .ok_or_else(|| PyRuntimeError::new_err("No route found"))?;

        let grid = GeometryGridSpec::new(
            self.inner.grid.grid_size_um,
            self.inner.grid.origin_x_um,
            self.inner.grid.origin_y_um,
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
        let primitive_centerline =
            route_to_primitive_centerline_rs(&result, &self.inner.primitives, &grid)
                .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let base_prefix =
            DenseOccupancyPrefix::from_obstacle_map(&self.inner.obstacle_map, opened_ref);
        let auto_plan = plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix_rs(
            &primitive_centerline,
            &grid,
            &base_prefix,
            opened_ref,
            None,
            None,
            None,
            None,
            &auto_cfg,
            &[auto_cfg.box_depth_um],
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        let polygon = realize_route_polygon_from_auto_plan_rs(
            &result,
            &self.inner.primitives,
            &grid,
            width_um,
            &auto_plan,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        let route_cells = inflate_route_cells(
            &result.cells,
            route_block_radius_cells,
            self.inner.grid.width as i32,
            self.inner.grid.height as i32,
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
        if !self.inner.obstacle_map.commit_route(net_id, &merged) {
            return Err(PyRuntimeError::new_err(
                "Failed to commit merged route and meander reservation cells",
            ));
        }
        self.inner.invalidate_meander_base_prefix();

        let py_route = Py::new(py, convert_result(py, &self.inner.primitives, &result)?)?;
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
            self.inner.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            Some(box_depth_um),
        )?;
        Ok(d.into())
    }
    #[pyo3(signature=(net_id,centerline,width_um,clearance_radius_cells,core_radius_cells,opened_cells=None,clearance_exempt_cells=None,source_port_um=None,target_port_um=None))]
    pub(crate) fn centerline_port_corrected_checked(
        &mut self,
        py: Python<'_>,
        net_id: u64,
        centerline: Vec<(f64, f64)>,
        width_um: f64,
        clearance_radius_cells: i32,
        core_radius_cells: i32,
        opened_cells: Option<Vec<(i32, i32)>>,
        clearance_exempt_cells: Option<Vec<(i32, i32)>>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> PyResult<Py<PyDict>> {
        let _ = clearance_radius_cells;
        let opened_cell_vec = opened_cells.unwrap_or_default();
        let clearance_exempt_cell_vec = clearance_exempt_cells.unwrap_or_default();
        let correction = self
            .inner
            .centerline_port_corrected_checked_native(
                net_id,
                &centerline,
                width_um,
                core_radius_cells,
                &opened_cell_vec,
                &clearance_exempt_cell_vec,
                source_port_um,
                target_port_um,
            )
            .map_err(PyRuntimeError::new_err)?;
        let d = PyDict::new_bound(py);
        d.set_item("centerline", correction.centerline)?;
        d.set_item("committed_bump", correction.committed_bump)?;
        d.set_item("candidate_index", correction.candidate_index)?;
        d.set_item("candidate_label", correction.candidate_label)?;
        Ok(d.into())
    }

    pub(crate) fn describe_primitives(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        describe_primitives(py, &self.inner.primitives)
    }
}

// The binding-side helpers the `#[pymethods]` block shares; Python never
// sees them.
impl PyPhotonicRouter {
    pub(crate) fn long_straight_congestion_records(
        &self,
        py: Python<'_>,
    ) -> PyResult<Vec<PyObject>> {
        let mut out = Vec::with_capacity(self.inner.long_straight_congestion_records.len());
        for record in &self.inner.long_straight_congestion_records {
            let d = PyDict::new_bound(py);
            d.set_item("net_id", record.net_id)?;
            d.set_item("start", record.start)?;
            d.set_item("end", record.end)?;
            d.set_item("length_um", record.length_um)?;
            d.set_item("marked_cells", record.marked_cells)?;
            out.push(d.into());
        }
        Ok(out)
    }
}

pub fn register_py_router(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyGridSpec>()?;
    m.add_class::<PyPrimitiveLibraryConfig>()?;
    m.add_class::<PyAStarConfig>()?;
    m.add_class::<PyRouterConfig>()?;
    m.add_class::<PyCrossingConfig>()?;
    m.add_class::<PyState>()?;
    m.add_class::<PyRouteJob>()?;
    m.add_class::<PyRouteResult>()?;
    m.add_class::<PyPhotonicRouter>()?;
    m.add_class::<PyMeanderPlanner>()?;
    m.add_function(wrap_pyfunction!(auto_meander_search_config_rs, m)?)?;
    Ok(())
}

/// Python extension module used by `python/photonic_router/static_obstacle_builder.py`.
#[pymodule]
pub fn _rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    register_py_router(m)?;
    m.add_class::<PyStaticCellSet>()?;
    m.add_function(wrap_pyfunction!(build_static_obstacle_map_rs, m)?)?;
    Ok(())
}

#[pymodule]
pub fn photonic_router_rust(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    register_py_router(m)
}
