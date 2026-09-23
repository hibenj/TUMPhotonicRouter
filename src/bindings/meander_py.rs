use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use crate::auto_meander::{AutoMeanderPlanningProfile, AutoMeanderSidePolicy};
use crate::meander::{MeanderPlanningMode, MeanderSide};
use crate::plm::{
    MeanderPlanningProfileTotals, MeanderWrapperProfileTotals, RegisteredRequirementResult,
};

pub(crate) fn parse_meander_side(side: &str) -> PyResult<MeanderSide> {
    let normalized = side.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "left" => Ok(MeanderSide::Left),
        "right" => Ok(MeanderSide::Right),
        _ => Err(PyValueError::new_err(
            "side must be either 'left' or 'right'",
        )),
    }
}

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
    plan: &crate::auto_meander::AutoRouteAnalyticMeanderPlan,
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

pub(crate) fn auto_meander_probe_to_py_object(
    py: Python<'_>,
    probe: &crate::auto_meander::AutoRouteAnalyticMeanderProbe,
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
