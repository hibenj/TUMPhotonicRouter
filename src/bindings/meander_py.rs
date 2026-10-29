// pyo3 0.22's `#[pymethods]` / `#[pyfunction]` expansion converts the returned
// `PyErr` into `PyErr`, which clippy reports at every binding; nothing here to fix.
#![allow(clippy::useless_conversion)]

use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use rustc_hash::FxHashSet;

use crate::engine::{
    build_registered_open_indices, build_registered_open_sets, collect_meander_route_cell_sets,
    pack_cells, MeanderRegistrationProfile, PyPhotonicRouter,
};
use crate::geometry_realization::{
    centerline_length_um as centerline_length_um_rs,
    realize_centerline_polygon_with_terminal_tangents as realize_centerline_polygon_with_terminal_tangents_rs,
    splice_meander_into_centerline_range as splice_meander_into_centerline_range_rs,
    GeometryGridSpec,
};
use crate::meander::geometry::{
    actual_bend_radius_um_from_cells as actual_bend_radius_um_from_cells_rs, MeanderPlanningMode,
    MeanderSide, PhysicalPoint,
};
use crate::meander::plan::{
    plan_registered_geometry_final_requests, plan_registered_geometry_requirement_requests,
    MeanderPlanningProfileTotals, MeanderWrapperProfileTotals, RegisteredMeanderGeometry,
    RegisteredPlmContext, RegisteredRequirementResult, SplitFallbackRequest,
};
use crate::meander::search::{
    plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix as plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix_rs,
    AutoMeanderConfig, AutoMeanderPlanningProfile, AutoMeanderSidePolicy,
};
use crate::obstacle_map::{unpack_xy, CellKey, ObstacleMap};
use crate::static_obstacle_builder::PyStaticCellSet;

use super::{to_route_result, PyRouteResult};

pub(crate) fn parse_auto_meander_side_policy(side_policy: &str) -> PyResult<AutoMeanderSidePolicy> {
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

pub(crate) fn parse_meander_planning_mode(mode: &str) -> PyResult<MeanderPlanningMode> {
    match mode.trim().to_ascii_lowercase().as_str() {
        "fill_box_multi_bump" => Ok(MeanderPlanningMode::FillBoxMultiBump),
        _ => Err(PyValueError::new_err(
            "planning_mode must be 'fill_box_multi_bump'",
        )),
    }
}

pub(crate) fn planning_mode_to_str(mode: MeanderPlanningMode) -> &'static str {
    let _ = mode;
    "fill_box_multi_bump"
}

pub(crate) fn add_bend_radius_debug_metadata(
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

pub(crate) fn set_meander_planning_profile_totals_item(
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

pub(crate) fn set_meander_wrapper_profile_totals_item(
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

pub(crate) fn auto_meander_planning_profile_to_py_object(
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

pub(crate) fn auto_meander_plan_to_py_object(
    py: Python<'_>,
    plan: &crate::meander::search::AutoRouteAnalyticMeanderPlan,
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

pub(crate) fn annotate_endpoint_inset_sweep_result(
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

pub(crate) const DEFAULT_MEANDER_DEPTH_CANDIDATES_UM: [f64; 12] = [
    40.0, 30.0, 24.0, 20.0, 16.0, 12.0, 10.0, 8.0, 6.0, 4.0, 3.0, 2.0,
];

pub(crate) const MEANDER_POLICY_DEDUPE_EPS_UM: f64 = 1.0e-9;

pub(crate) fn default_meander_box_depths_um(max_meander_height_um: f64) -> PyResult<Vec<f64>> {
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

pub(crate) fn default_endpoint_insets_um(
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

pub(crate) fn annotate_auto_meander_search_policy(
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

pub(crate) fn auto_meander_search_config_to_py_dict<'py>(
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
pub(crate) fn registered_requirement_result_to_py_object(
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
pub(crate) fn auto_meander_search_config_rs(
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

/// The path-length-matching planner of one router (phase 6 slice C of the
/// 2026-10 do-over): the routes registered as occupied cells with each
/// route's own cells open to its meanders, the route geometries, the cells
/// the planned meanders reserve, and the two caches the planning calls
/// rebuild on demand -- the router's dense occupancy prefix and the sparse
/// index of the reserved cells. `MeanderPlanner(router)` starts empty;
/// `register` then `register_geometries` fill it, `plan_requests` and
/// `plan_final` decide the meanders, `reserve_rect` books a planned box.
///
/// The router keeps two counters the planner syncs on at every call:
/// `obstacle_generation` moves whenever the obstacle map changes and drops
/// the prefix, `static_reset_generation` moves with `clear_static_cells`
/// and drops the registration -- exactly what the router's own context did
/// while this state lived on `PyPhotonicRouter`.
#[pyclass(name = "MeanderPlanner")]
pub struct PyMeanderPlanner {
    router: Py<PyPhotonicRouter>,
    context: RegisteredPlmContext,
    base_prefix_generation: u64,
    static_generation: u64,
    profile: Option<MeanderRegistrationProfile>,
}

impl PyMeanderPlanner {
    /// Drop the registration when the router's static cells were cleared
    /// since the last call (the router did this in `clear_static_cells`
    /// before the state moved here).
    fn sync_with_router(&mut self, router: &PyPhotonicRouter) {
        if self.static_generation != router.static_reset_generation {
            self.context.clear_registered_routes();
            self.context.clear_reserved_cells_and_invalidate_index();
            self.static_generation = router.static_reset_generation;
        }
    }

    fn ensure_base_prefix(&mut self, router: &PyPhotonicRouter) {
        if self.base_prefix_generation != router.obstacle_generation {
            self.context.invalidate_base_prefix();
            self.base_prefix_generation = router.obstacle_generation;
        }
        self.context
            .ensure_base_prefix_from_obstacle_map(&router.obstacle_map);
    }

    fn ensure_reserved_index(&mut self, router: &PyPhotonicRouter) {
        self.context
            .ensure_reserved_index(router.grid.width as i32, router.grid.height as i32);
    }

    /// The registration that adds the routes to the router's current static
    /// map (`reset=False`).
    fn register_on_current_map(
        &mut self,
        router: &mut PyPhotonicRouter,
        routes: &Bound<'_, PyList>,
        base_static_cells: Vec<(i32, i32)>,
        route_clearance_radius_cells: i32,
    ) -> PyResult<(Vec<usize>, Vec<usize>, usize)> {
        let total_start = Instant::now();
        let mut profile = MeanderRegistrationProfile {
            route_count: routes.len(),
            base_static_cell_count: base_static_cells.len(),
            ..MeanderRegistrationProfile::default()
        };
        let reset_start = Instant::now();
        self.context.clear_reserved_cells_and_invalidate_index();
        profile.reset_s += reset_start.elapsed().as_secs_f64();
        let base_static_pack_start = Instant::now();
        let base_static_keys = pack_cells(&base_static_cells);
        profile.base_static_pack_s += base_static_pack_start.elapsed().as_secs_f64();
        let (route_cell_sets, route_cell_refcounts, unique_route_cells) =
            collect_meander_route_cell_sets(
                routes,
                route_clearance_radius_cells,
                router.grid.width as i32,
                router.grid.height as i32,
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
            router.add_static_cells(route_cell_list);
            profile.route_static_add_s += route_static_add_start.elapsed().as_secs_f64();
        }

        let indices: Vec<usize> = (0..registered_open_sets.len()).collect();
        let registered_store_start = Instant::now();
        self.ensure_base_prefix(router);
        let base_prefix = self
            .context
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let registered_open_indices =
            build_registered_open_indices(base_prefix, &registered_open_sets);
        self.context.open_cells = registered_open_sets;
        self.context.open_indices = registered_open_indices;
        profile.registered_store_s += registered_store_start.elapsed().as_secs_f64();
        profile.total_s = total_start.elapsed().as_secs_f64();
        self.profile = Some(profile);
        Ok((indices, open_counts, unique_route_cell_count))
    }

    /// The registration the flow uses: the router starts over from the
    /// base static cells and the routes join its static map.
    fn register_with_reset(
        &mut self,
        router: &mut PyPhotonicRouter,
        routes: &Bound<'_, PyList>,
        base_static_cells: Vec<(i32, i32)>,
        route_clearance_radius_cells: i32,
    ) -> PyResult<(Vec<usize>, Vec<usize>, usize)> {
        let total_start = Instant::now();
        let mut profile = MeanderRegistrationProfile {
            route_count: routes.len(),
            base_static_cell_count: base_static_cells.len(),
            ..MeanderRegistrationProfile::default()
        };
        let reset_start = Instant::now();
        router.invalidate_meander_base_prefix();
        router.obstacle_map = ObstacleMap::new(router.grid.width as i32, router.grid.height as i32);
        router.committed_center_routes.clear();
        router.committed_realized_center_routes.clear();
        router.committed_target_terminal_bump_guards.clear();
        router.committed_opened_cell_keys.clear();
        router.crossing_events.clear();
        router.crossing_verdicts.clear();
        profile.reset_s += reset_start.elapsed().as_secs_f64();
        let base_static_pack_start = Instant::now();
        let base_static_keys = pack_cells(&base_static_cells);
        profile.base_static_pack_s += base_static_pack_start.elapsed().as_secs_f64();
        router.static_cells = base_static_keys.clone();
        self.context.clear_registered_routes();
        self.context.clear_reserved_cells_and_invalidate_index();
        if !base_static_cells.is_empty() {
            let base_static_add_start = Instant::now();
            router.obstacle_map.add_static_cells(&base_static_cells);
            profile.base_static_obstacle_add_s += base_static_add_start.elapsed().as_secs_f64();
        }

        let (route_cell_sets, route_cell_refcounts, unique_route_cells) =
            collect_meander_route_cell_sets(
                routes,
                route_clearance_radius_cells,
                router.grid.width as i32,
                router.grid.height as i32,
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
        router.static_cells.extend(unique_route_cells);
        if !route_cell_list.is_empty() {
            let route_static_add_start = Instant::now();
            router.obstacle_map.add_static_cells(&route_cell_list);
            profile.route_static_add_s += route_static_add_start.elapsed().as_secs_f64();
        }

        let indices: Vec<usize> = (0..registered_open_sets.len()).collect();
        let registered_store_start = Instant::now();
        self.ensure_base_prefix(router);
        let base_prefix = self
            .context
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let registered_open_indices =
            build_registered_open_indices(base_prefix, &registered_open_sets);
        self.context.open_cells = registered_open_sets;
        self.context.open_indices = registered_open_indices;
        profile.registered_store_s += registered_store_start.elapsed().as_secs_f64();
        profile.total_s = total_start.elapsed().as_secs_f64();
        self.profile = Some(profile);
        Ok((indices, open_counts, unique_route_cell_count))
    }

    /// The reset registration from a Rust static-cell handle: the prefix is
    /// built from the handle's keys plus the route cells directly.
    fn register_with_reset_from_handle(
        &mut self,
        router: &mut PyPhotonicRouter,
        routes: &Bound<'_, PyList>,
        base_static_keys: &FxHashSet<CellKey>,
        route_clearance_radius_cells: i32,
    ) -> PyResult<(Vec<usize>, Vec<usize>, usize)> {
        let total_start = Instant::now();
        let mut profile = MeanderRegistrationProfile {
            route_count: routes.len(),
            base_static_cell_count: base_static_keys.len(),
            ..MeanderRegistrationProfile::default()
        };
        let reset_start = Instant::now();
        router.invalidate_meander_base_prefix();
        router.obstacle_map = ObstacleMap::new(router.grid.width as i32, router.grid.height as i32);
        router.committed_center_routes.clear();
        router.committed_realized_center_routes.clear();
        router.committed_target_terminal_bump_guards.clear();
        router.committed_opened_cell_keys.clear();
        router.crossing_events.clear();
        router.crossing_verdicts.clear();
        router.static_cells = base_static_keys.clone();
        self.context.clear_registered_routes();
        self.context.clear_reserved_cells_and_invalidate_index();
        profile.reset_s += reset_start.elapsed().as_secs_f64();

        let (route_cell_sets, route_cell_refcounts, unique_route_cells) =
            collect_meander_route_cell_sets(
                routes,
                route_clearance_radius_cells,
                router.grid.width as i32,
                router.grid.height as i32,
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
        router.static_cells = base_prefix_keys.clone();
        self.context.set_base_prefix_from_keys(
            router.grid.width as i32,
            router.grid.height as i32,
            &base_prefix_keys,
        );
        self.base_prefix_generation = router.obstacle_generation;
        profile.base_prefix_build_s += base_prefix_build_start.elapsed().as_secs_f64();

        let indices: Vec<usize> = (0..registered_open_sets.len()).collect();
        let registered_store_start = Instant::now();
        let base_prefix = self
            .context
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let registered_open_indices =
            build_registered_open_indices(base_prefix, &registered_open_sets);
        self.context.open_cells = registered_open_sets;
        self.context.open_indices = registered_open_indices;
        profile.registered_store_s += registered_store_start.elapsed().as_secs_f64();
        profile.total_s = total_start.elapsed().as_secs_f64();
        self.profile = Some(profile);
        Ok((indices, open_counts, unique_route_cell_count))
    }
}

#[pymethods]
impl PyMeanderPlanner {
    #[new]
    fn new(py: Python<'_>, router: Py<PyPhotonicRouter>) -> PyResult<Self> {
        let (obstacle_generation, static_reset_generation) = {
            let router = router.bind(py).try_borrow()?;
            (router.obstacle_generation, router.static_reset_generation)
        };
        Ok(Self {
            router,
            context: RegisteredPlmContext::default(),
            base_prefix_generation: obstacle_generation,
            static_generation: static_reset_generation,
            profile: None,
        })
    }

    /// Register the routes as occupied cells (inflated by
    /// `route_clearance_radius_cells`), each route's own cells open to its
    /// meanders where no other route or base static cell claims them.
    /// Returns the registered route indices, their open-cell counts and the
    /// number of distinct route cells. With `reset` (the flow's form) the
    /// router starts over from `base_static_cells` -- or from the Rust
    /// handle `base_static_cell_handle`, whose keys then also build the
    /// occupancy prefix directly -- and drops its committed routes and
    /// crossing events; `reset=False` adds the routes to the router's
    /// current static map and keeps the rest.
    #[pyo3(signature=(routes,base_static_cells=None,base_static_cell_handle=None,route_clearance_radius_cells=0,reset=true))]
    fn register(
        &mut self,
        py: Python<'_>,
        routes: &Bound<'_, PyList>,
        base_static_cells: Option<Vec<(i32, i32)>>,
        base_static_cell_handle: Option<PyRef<'_, PyStaticCellSet>>,
        route_clearance_radius_cells: i32,
        reset: bool,
    ) -> PyResult<(Vec<usize>, Vec<usize>, usize)> {
        if route_clearance_radius_cells < 0 {
            return Err(PyValueError::new_err(
                "route_clearance_radius_cells must be >= 0",
            ));
        }
        let mut router = self.router.bind(py).try_borrow_mut()?;
        self.sync_with_router(&router);
        match (base_static_cells, base_static_cell_handle, reset) {
            (Some(cells), None, true) => {
                self.register_with_reset(&mut router, routes, cells, route_clearance_radius_cells)
            }
            (Some(cells), None, false) => self.register_on_current_map(
                &mut router,
                routes,
                cells,
                route_clearance_radius_cells,
            ),
            (None, Some(handle), true) => self.register_with_reset_from_handle(
                &mut router,
                routes,
                handle.keys(),
                route_clearance_radius_cells,
            ),
            (None, Some(_), false) => Err(PyValueError::new_err(
                "base_static_cell_handle registers with reset=True only",
            )),
            _ => Err(PyValueError::new_err(
                "register takes exactly one of base_static_cells and base_static_cell_handle",
            )),
        }
    }

    /// Register one centerline per route to plan meanders on: its index
    /// among the registered routes (from `register`) and its bump cap.
    /// Returns the geometry indices the planning calls take.
    fn register_geometries(
        &mut self,
        py: Python<'_>,
        centerlines: Vec<Vec<(f64, f64)>>,
        registered_opened_cell_indices: Vec<usize>,
        max_bumps_by_edge: Vec<usize>,
    ) -> PyResult<Vec<usize>> {
        {
            let router = self.router.bind(py).try_borrow()?;
            self.sync_with_router(&router);
        }
        let count = centerlines.len();
        if count != registered_opened_cell_indices.len() || count != max_bumps_by_edge.len() {
            return Err(PyValueError::new_err(
                "registered meander geometry inputs must have matching lengths",
            ));
        }
        let open_cell_count = self.context.open_cells.len();
        let mut geometries = Vec::with_capacity(count);
        for ((centerline, registered_open_index), max_bumps) in centerlines
            .into_iter()
            .zip(registered_opened_cell_indices)
            .zip(max_bumps_by_edge)
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
        self.context.geometries = geometries;
        Ok(indices)
    }

    /// Reserve a planned meander's grid rectangle for the following plans.
    /// Returns the number of newly reserved cells.
    fn reserve_rect(
        &mut self,
        py: Python<'_>,
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
        let router = self.router.bind(py).try_borrow()?;
        self.sync_with_router(&router);
        Ok(self.context.add_reserved_grid_rect(
            min_x,
            max_x,
            min_y,
            max_y,
            router.grid.width as i32,
        ))
    }

    fn clear_reserved(&mut self, py: Python<'_>) -> PyResult<()> {
        let router = self.router.bind(py).try_borrow()?;
        self.sync_with_router(&router);
        self.context.clear_reserved_cells(router.grid.height as i32);
        Ok(())
    }

    fn open_cell_count(&mut self, py: Python<'_>, index: usize) -> PyResult<usize> {
        {
            let router = self.router.bind(py).try_borrow()?;
            self.sync_with_router(&router);
        }
        self.context
            .open_cells
            .get(index)
            .map(FxHashSet::len)
            .ok_or_else(|| PyValueError::new_err("registered meander route index is out of range"))
    }

    /// The timers and counts of the last `register` call (empty before one).
    fn profile(&self, py: Python<'_>) -> PyResult<PyObject> {
        let d = PyDict::new_bound(py);
        if let Some(profile) = self.profile.as_ref() {
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

    /// Plan one path-length requirement from its request vector (phase 6
    /// slice B): `candidate_geometry_indices[i]` is the i-th candidate bundle
    /// of registered geometries and `candidate_requested_extra_lengths_um[i]`
    /// its requested extra length per edge (the edge's new total where it
    /// already carries a planned meander). The candidates are tried in order
    /// at each endpoint inset; the first bundle that plans as one sequence
    /// wins. When none plans and `split_geometry_index` is given, that
    /// geometry is tried as 2..`max_split_parts` same-edge chunks of
    /// `split_requested_extra_length_um`, each at least
    /// `min_insertable_extra_length_um`. The result's `planning_mode` says
    /// which branch planned (`rust_registered_candidate`,
    /// `rust_registered_split_route_runs`, `none`), `selected_candidate_index`
    /// the planned candidate (the split's part-count attempts follow the
    /// candidates) and `split_part_count` the chunks of a planned split.
    #[pyo3(signature=(candidate_geometry_indices,candidate_requested_extra_lengths_um,split_geometry_index=None,split_requested_extra_length_um=None,min_insertable_extra_length_um=0.0,max_split_parts=8,min_bend_radius_um=None,min_straight_um=0.0,max_meander_height_um=20.0,min_segment_length_um=10.0,auto_endpoint_inset_um=None,clearance_radius_cells=0,side_policy="both",planning_mode="fill_box_multi_bump"))]
    #[allow(clippy::too_many_arguments)]
    fn plan_requests(
        &mut self,
        py: Python<'_>,
        candidate_geometry_indices: Vec<Vec<usize>>,
        candidate_requested_extra_lengths_um: Vec<Vec<f64>>,
        split_geometry_index: Option<usize>,
        split_requested_extra_length_um: Option<f64>,
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
        let router = self.router.bind(py).try_borrow()?;
        self.sync_with_router(&router);
        let effective_radius_um = router.effective_bend_radius_um(min_bend_radius_um)?;
        let box_depths_um = default_meander_box_depths_um(max_meander_height_um)?;
        let endpoint_insets_um = default_endpoint_insets_um(
            effective_radius_um,
            min_segment_length_um,
            auto_endpoint_inset_um,
        )?;
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            router.primitive_cfg.bend_radius_cells,
            router.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let grid = GeometryGridSpec::new(
            router.grid.grid_size_um,
            router.grid.origin_x_um,
            router.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let split_fallback = match (split_geometry_index, split_requested_extra_length_um) {
            (Some(geometry_index), Some(requested_extra_length_um)) => Some(SplitFallbackRequest {
                geometry_index,
                requested_extra_length_um,
                min_insertable_extra_um: min_insertable_extra_length_um,
                max_parts: max_split_parts,
            }),
            (None, None) => None,
            _ => {
                return Err(PyValueError::new_err(
                    "split_geometry_index and split_requested_extra_length_um go together",
                ))
            }
        };

        self.ensure_base_prefix(&router);
        self.ensure_reserved_index(&router);
        let plm = &self.context;
        let base_prefix = plm
            .base_prefix
            .as_ref()
            .expect("meander base prefix should be initialized");
        let requests_result = plan_registered_geometry_requirement_requests(
            &candidate_geometry_indices,
            &candidate_requested_extra_lengths_um,
            split_fallback,
            &plm.geometries,
            &plm.open_cells,
            &plm.open_indices,
            base_prefix,
            plm.reserved_index.as_ref(),
            &grid,
            router.grid.width as i32,
            router.grid.height as i32,
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
            requests_result.result,
            min_bend_radius_um,
            effective_radius_um,
            router.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
            min_straight_um,
            min_segment_length_um,
            max_meander_height_um,
        )?;
        let py_result_dict = py_result.bind(py).downcast::<PyDict>()?;
        py_result_dict.set_item("planning_mode", requests_result.planning_mode)?;
        py_result_dict.set_item("split_part_count", requests_result.split_part_count)?;
        Ok(py_result)
    }

    /// Plan every physical edge's total in one call: the whole sequence,
    /// then each edge direct, then the same-edge split. The result's
    /// `planning_mode` names the branch that planned and
    /// `plan_input_indices` the input edge of each plan.
    #[pyo3(signature=(geometry_indices,requested_extra_lengths_um,min_insertable_extra_length_um,max_split_parts=8,min_bend_radius_um=None,min_straight_um=0.0,max_meander_height_um=20.0,min_segment_length_um=10.0,auto_endpoint_inset_um=None,clearance_radius_cells=0,side_policy="both",planning_mode="fill_box_multi_bump"))]
    #[allow(clippy::too_many_arguments)]
    fn plan_final(
        &mut self,
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
        let router = self.router.bind(py).try_borrow()?;
        self.sync_with_router(&router);
        let effective_radius_um = router.effective_bend_radius_um(min_bend_radius_um)?;
        let box_depths_um = default_meander_box_depths_um(max_meander_height_um)?;
        let endpoint_insets_um = default_endpoint_insets_um(
            effective_radius_um,
            min_segment_length_um,
            auto_endpoint_inset_um,
        )?;
        let policy = parse_auto_meander_side_policy(side_policy)?;
        let mode = parse_meander_planning_mode(planning_mode)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            router.primitive_cfg.bend_radius_cells,
            router.grid.grid_size_um,
        )
        .map_err(PyValueError::new_err)?;
        let grid = GeometryGridSpec::new(
            router.grid.grid_size_um,
            router.grid.origin_x_um,
            router.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;

        self.ensure_base_prefix(&router);
        self.ensure_reserved_index(&router);
        let plm = &self.context;
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
            router.grid.width as i32,
            router.grid.height as i32,
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
            router.primitive_cfg.bend_radius_cells,
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

    /// The depth sweep over one centerline against the router's occupancy
    /// (its port-open cells unless `opened_cells` is given): the first box
    /// depth, run and side that take `requested_extra_length_um` exactly.
    #[pyo3(signature=(centerline,requested_extra_length_um,box_depths_um,min_bend_radius_um=None,min_straight_um=0.0,max_bumps=8,max_meander_height_um=20.0,min_segment_length_um=10.0,endpoint_inset_um=0.0,clearance_radius_cells=0,side_policy="both",opened_cells=None,planning_mode="fill_box_multi_bump",extra_blocked_cells=None))]
    #[allow(clippy::too_many_arguments)]
    fn plan_centerline(
        &mut self,
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
        let router = self.router.bind(py).try_borrow()?;
        self.sync_with_router(&router);
        let effective_radius_um = router.effective_bend_radius_um(min_bend_radius_um)?;
        let primitive_bend_radius_um = actual_bend_radius_um_from_cells_rs(
            router.primitive_cfg.bend_radius_cells,
            router.grid.grid_size_um,
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
            router.grid.grid_size_um,
            router.grid.origin_x_um,
            router.grid.origin_y_um,
        )
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
        let opened_owned;
        let opened_ref: Option<&FxHashSet<CellKey>> = if let Some(cells) = opened_cells.as_ref() {
            opened_owned = pack_cells(cells);
            Some(&opened_owned)
        } else {
            Some(&router.port_open_cells)
        };
        let extra_blocked_owned;
        let extra_blocked_ref: Option<&FxHashSet<CellKey>> =
            if let Some(cells) = extra_blocked_cells.as_ref() {
                extra_blocked_owned = pack_cells(cells);
                Some(&extra_blocked_owned)
            } else {
                None
            };
        self.ensure_base_prefix(&router);
        let base_prefix = self
            .context
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
            router.primitive_cfg.bend_radius_cells,
            primitive_bend_radius_um,
            mode,
        )
    }

    /// The polygon of a port-corrected centerline with a planned meander
    /// spliced into its run `selected_run_start_index..=selected_run_end_index`,
    /// the terminal tangents taken from the grid route. Pure geometry: no
    /// router state, so a static method.
    #[staticmethod]
    #[pyo3(signature=(centerline,width_um,route,selected_run_start_index,selected_run_end_index,meander_centerline,source_enabled=true,target_enabled=true))]
    #[allow(clippy::too_many_arguments)]
    fn realize(
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
}
