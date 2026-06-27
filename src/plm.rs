use std::time::Instant;

use rustc_hash::FxHashSet;

use crate::geometry_realization::{
    cell_count_in_grid_rect, plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix,
    AutoMeanderConfig, AutoMeanderPlanningProfile, AutoMeanderSidePolicy,
    AutoRouteAnalyticMeanderPlan, DenseOccupancyPrefix, GeometryGridSpec, SparseCellIndex,
};
use crate::meander::MeanderPlanningMode;
use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};

#[derive(Clone)]
pub struct RegisteredMeanderGeometry {
    pub centerline: Vec<(f64, f64)>,
    pub registered_open_index: usize,
    pub max_bumps: usize,
}

#[derive(Default)]
pub struct RegisteredPlmContext {
    pub base_prefix: Option<DenseOccupancyPrefix>,
    pub open_cells: Vec<FxHashSet<CellKey>>,
    pub open_indices: Vec<SparseCellIndex>,
    pub geometries: Vec<RegisteredMeanderGeometry>,
    pub reserved_cells: FxHashSet<CellKey>,
    pub reserved_index: Option<SparseCellIndex>,
}

impl RegisteredPlmContext {
    pub fn invalidate_base_prefix(&mut self) {
        self.base_prefix = None;
    }

    pub fn ensure_base_prefix_from_obstacle_map(&mut self, obstacle_map: &ObstacleMap) {
        if self.base_prefix.is_none() {
            self.base_prefix = Some(DenseOccupancyPrefix::from_obstacle_map(obstacle_map, None));
        }
    }

    pub fn set_base_prefix_from_keys(
        &mut self,
        width: i32,
        height: i32,
        keys: &FxHashSet<CellKey>,
    ) {
        self.base_prefix = Some(DenseOccupancyPrefix::from_blocked_keys(width, height, keys));
    }

    pub fn clear_registered_routes(&mut self) {
        self.open_cells.clear();
        self.open_indices.clear();
        self.geometries.clear();
    }

    pub fn clear_reserved_cells(&mut self, grid_height: i32) {
        self.reserved_cells.clear();
        self.reserved_index = Some(SparseCellIndex::empty(grid_height));
    }

    pub fn clear_reserved_cells_and_invalidate_index(&mut self) {
        self.reserved_cells.clear();
        self.reserved_index = None;
    }

    pub fn invalidate_reserved_index(&mut self) {
        self.reserved_index = None;
    }

    pub fn ensure_reserved_index(&mut self, width: i32, height: i32) {
        if self.reserved_index.is_none() {
            self.reserved_index = Some(SparseCellIndex::from_cells(
                width,
                height,
                self.reserved_cells.iter().copied(),
            ));
        }
    }

    pub fn add_reserved_cells(&mut self, cells: &[(i32, i32)], width: i32) -> usize {
        let before = self.reserved_cells.len();
        let packed_cells: Vec<CellKey> = cells.iter().map(|(x, y)| pack_xy(*x, *y)).collect();
        self.reserved_cells.extend(packed_cells.iter().copied());
        let added = self.reserved_cells.len().saturating_sub(before);
        if let Some(index) = self.reserved_index.as_mut() {
            index.insert_cells(width, packed_cells);
        }
        added
    }

    pub fn add_reserved_grid_rect(
        &mut self,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
        width: i32,
    ) -> usize {
        let before = self.reserved_cells.len();
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                self.reserved_cells.insert(pack_xy(x, y));
            }
        }
        let added = self.reserved_cells.len().saturating_sub(before);
        if let Some(index) = self.reserved_index.as_mut() {
            index.insert_rect(min_x, max_x, min_y, max_y, width);
        }
        added
    }
}

#[derive(Clone, Default)]
pub struct MeanderPlanningProfileTotals {
    pub total_s: f64,
    pub run_extraction_s: f64,
    pub footprint_s: f64,
    pub free_interval_s: f64,
    pub box_check_s: f64,
    pub analytic_plan_s: f64,
    pub replacement_check_s: f64,
    pub depth_count: usize,
    pub run_side_checks: usize,
    pub box_checks: usize,
    pub analytic_plan_calls: usize,
    pub plan_calls: usize,
}

impl MeanderPlanningProfileTotals {
    pub fn add(&mut self, profile: &AutoMeanderPlanningProfile) {
        self.total_s += profile.total_s;
        self.run_extraction_s += profile.run_extraction_s;
        self.footprint_s += profile.footprint_s;
        self.free_interval_s += profile.free_interval_s;
        self.box_check_s += profile.box_check_s;
        self.analytic_plan_s += profile.analytic_plan_s;
        self.replacement_check_s += profile.replacement_check_s;
        self.depth_count += profile.depth_count;
        self.run_side_checks += profile.run_side_checks;
        self.box_checks += profile.box_checks;
        self.analytic_plan_calls += profile.analytic_plan_calls;
        self.plan_calls += 1;
    }
}

#[derive(Clone, Default)]
pub struct MeanderWrapperProfileTotals {
    pub reserved_snapshot_s: f64,
    pub planner_call_s: f64,
    pub selected_rect_cells_s: f64,
    pub candidate_reserved_update_s: f64,
    pub py_plan_conversion_s: f64,
    pub py_plan_append_s: f64,
    pub py_candidate_result_build_s: f64,
    pub py_result_build_s: f64,
    pub extra_blocked_prepare_calls: usize,
    pub selected_rect_cell_count: usize,
    pub py_plan_count: usize,
    pub candidate_result_count: usize,
}

impl MeanderWrapperProfileTotals {
    pub fn add(&mut self, other: &MeanderWrapperProfileTotals) {
        self.reserved_snapshot_s += other.reserved_snapshot_s;
        self.planner_call_s += other.planner_call_s;
        self.selected_rect_cells_s += other.selected_rect_cells_s;
        self.candidate_reserved_update_s += other.candidate_reserved_update_s;
        self.py_plan_conversion_s += other.py_plan_conversion_s;
        self.py_plan_append_s += other.py_plan_append_s;
        self.py_candidate_result_build_s += other.py_candidate_result_build_s;
        self.py_result_build_s += other.py_result_build_s;
        self.extra_blocked_prepare_calls += other.extra_blocked_prepare_calls;
        self.selected_rect_cell_count += other.selected_rect_cell_count;
        self.py_plan_count += other.py_plan_count;
        self.candidate_result_count += other.candidate_result_count;
    }
}

pub struct RegisteredRequirementEdgePlan {
    pub plan: AutoRouteAnalyticMeanderPlan,
    pub endpoint_inset_um: f64,
}

pub struct RegisteredRequirementCandidateResult {
    pub candidate_index: usize,
    pub plans: Vec<RegisteredRequirementEdgePlan>,
    pub candidate_runs: usize,
    pub candidate_intervals: usize,
    pub rejected_box_blocked: usize,
    pub rejected_planning_failed: usize,
    pub rejected_exact_length_mismatch: usize,
    pub rejected_too_short: usize,
    pub planner_profile_total: MeanderPlanningProfileTotals,
    pub wrapper_profile_total: MeanderWrapperProfileTotals,
    pub failed_reason: Option<String>,
    pub failed_edge_index: Option<usize>,
}

impl RegisteredRequirementCandidateResult {
    pub fn status(&self) -> &'static str {
        if self.failed_reason.is_some() {
            "no_candidate"
        } else {
            "planned"
        }
    }
}

pub struct RegisteredRequirementResult {
    pub selected_candidate_index: Option<usize>,
    pub candidate_results: Vec<RegisteredRequirementCandidateResult>,
    pub planner_profile_total: MeanderPlanningProfileTotals,
    pub wrapper_profile_total: MeanderWrapperProfileTotals,
    pub endpoint_inset_um: f64,
    pub attempted_endpoint_insets_um: Vec<f64>,
    pub box_depths_um: Vec<f64>,
    pub endpoint_insets_um: Vec<f64>,
    pub fixed_endpoint_inset: bool,
}

impl RegisteredRequirementResult {
    pub fn status(&self) -> &'static str {
        if self.selected_candidate_index.is_some() {
            "planned"
        } else {
            "no_candidate"
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn plan_registered_geometry_requirement_candidates(
    candidate_geometry_indices: &[Vec<usize>],
    candidate_requested_extra_lengths_um: &[f64],
    registered_geometries: &[RegisteredMeanderGeometry],
    registered_open_cells: &[FxHashSet<CellKey>],
    registered_open_indices: &[SparseCellIndex],
    base_prefix: &DenseOccupancyPrefix,
    reserved_index: Option<&SparseCellIndex>,
    grid: &GeometryGridSpec,
    grid_width: i32,
    grid_height: i32,
    box_depths_um: &[f64],
    endpoint_insets_um: &[f64],
    fixed_endpoint_inset: bool,
    effective_radius_um: f64,
    min_straight_um: f64,
    max_meander_height_um: f64,
    min_segment_length_um: f64,
    clearance_radius_cells: i32,
    side_policy: AutoMeanderSidePolicy,
    mode: MeanderPlanningMode,
) -> Result<RegisteredRequirementResult, String> {
    let candidate_count = candidate_geometry_indices.len();
    if candidate_count != candidate_requested_extra_lengths_um.len() {
        return Err(
            "candidate geometry and requested length inputs must have matching lengths".into(),
        );
    }
    if candidate_count == 0 {
        return Err("candidate list must not be empty".into());
    }
    if box_depths_um.is_empty() {
        return Err("box_depths_um must not be empty".into());
    }
    if box_depths_um.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err("box_depths_um values must be finite and > 0".into());
    }
    if endpoint_insets_um.is_empty() {
        return Err("endpoint_insets_um must not be empty".into());
    }
    if endpoint_insets_um
        .iter()
        .any(|value| !value.is_finite() || *value < 0.0)
    {
        return Err("endpoint_insets_um values must be finite and >= 0".into());
    }
    if candidate_requested_extra_lengths_um
        .iter()
        .any(|value| *value <= 0.0)
    {
        return Err("candidate requested lengths must be > 0".into());
    }
    if min_straight_um < 0.0 {
        return Err("min_straight_um must be >= 0".into());
    }
    if max_meander_height_um <= 0.0 {
        return Err("max_meander_height_um must be > 0".into());
    }
    if min_segment_length_um <= 0.0 {
        return Err("min_segment_length_um must be > 0".into());
    }
    if clearance_radius_cells < 0 {
        return Err("clearance_radius_cells must be >= 0".into());
    }
    if candidate_geometry_indices
        .iter()
        .any(|candidate| candidate.is_empty())
    {
        return Err("candidate bundles must not be empty".into());
    }

    let mut attempted_endpoint_insets_um: Vec<f64> = Vec::with_capacity(endpoint_insets_um.len());
    let mut last_result: Option<RegisteredRequirementResult> = None;

    for endpoint_inset_um in endpoint_insets_um {
        attempted_endpoint_insets_um.push(*endpoint_inset_um);
        let reserved_snapshot_start = Instant::now();
        let mut call_wrapper_profile = MeanderWrapperProfileTotals::default();
        call_wrapper_profile.reserved_snapshot_s += reserved_snapshot_start.elapsed().as_secs_f64();
        let mut call_profile_totals = MeanderPlanningProfileTotals::default();
        let mut candidate_results = Vec::with_capacity(candidate_count);
        let mut selected_candidate_index: Option<usize> = None;

        for candidate_index in 0..candidate_count {
            let geometry_indices = &candidate_geometry_indices[candidate_index];
            let requested_extra_length_um = candidate_requested_extra_lengths_um[candidate_index];
            let mut candidate_reserved_index = SparseCellIndex::empty(grid_height);
            let mut candidate_reserved_has_cells = false;
            let mut candidate_result = RegisteredRequirementCandidateResult {
                candidate_index,
                plans: Vec::with_capacity(geometry_indices.len()),
                candidate_runs: 0,
                candidate_intervals: 0,
                rejected_box_blocked: 0,
                rejected_planning_failed: 0,
                rejected_exact_length_mismatch: 0,
                rejected_too_short: 0,
                planner_profile_total: MeanderPlanningProfileTotals::default(),
                wrapper_profile_total: MeanderWrapperProfileTotals::default(),
                failed_reason: None,
                failed_edge_index: None,
            };

            for (edge_index, geometry_index) in geometry_indices.iter().enumerate() {
                let geometry = registered_geometries.get(*geometry_index).ok_or_else(|| {
                    "registered meander geometry index is out of range".to_string()
                })?;
                let opened_ref = registered_open_cells
                    .get(geometry.registered_open_index)
                    .ok_or_else(|| "registered meander route index is out of range".to_string())?;
                let opened_index_ref = registered_open_indices
                    .get(geometry.registered_open_index)
                    .ok_or_else(|| "registered meander route index is out of range".to_string())?;
                candidate_result
                    .wrapper_profile_total
                    .extra_blocked_prepare_calls += 1;
                let candidate_reserved_index_ref =
                    candidate_reserved_has_cells.then_some(&candidate_reserved_index);
                let cfg = AutoMeanderConfig {
                    requested_extra_length_um,
                    min_bend_radius_um: effective_radius_um,
                    min_straight_um,
                    max_bumps: geometry.max_bumps,
                    max_meander_height_um,
                    box_depth_um: box_depths_um[0],
                    min_segment_length_um,
                    endpoint_inset_um: *endpoint_inset_um,
                    clearance_radius_cells,
                    side_policy,
                    mode,
                };
                let planner_call_start = Instant::now();
                let plan = match plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
                    &geometry.centerline,
                    grid,
                    base_prefix,
                    Some(opened_ref),
                    Some(opened_index_ref),
                    None,
                    reserved_index,
                    candidate_reserved_index_ref,
                    &cfg,
                    box_depths_um,
                ) {
                    Ok(plan) => {
                        candidate_result.wrapper_profile_total.planner_call_s +=
                            planner_call_start.elapsed().as_secs_f64();
                        plan
                    }
                    Err(err) => {
                        candidate_result.wrapper_profile_total.planner_call_s +=
                            planner_call_start.elapsed().as_secs_f64();
                        candidate_result.failed_reason = Some(err.to_string());
                        candidate_result.failed_edge_index = Some(edge_index);
                        break;
                    }
                };
                candidate_result.planner_profile_total.add(&plan.profile);
                call_profile_totals.add(&plan.profile);
                candidate_result.candidate_runs += plan.candidate_runs;
                candidate_result.candidate_intervals += plan.candidate_intervals;
                candidate_result.rejected_box_blocked += plan.rejected_box_blocked;
                candidate_result.rejected_planning_failed += plan.rejected_planning_failed;
                candidate_result.rejected_exact_length_mismatch +=
                    plan.rejected_exact_length_mismatch;
                candidate_result.rejected_too_short += plan.rejected_too_short;
                let selected_rect_start = Instant::now();
                let selected_rect_cell_count = cell_count_in_grid_rect(plan.selected_grid_rect);
                candidate_result.wrapper_profile_total.selected_rect_cells_s +=
                    selected_rect_start.elapsed().as_secs_f64();
                candidate_result
                    .wrapper_profile_total
                    .selected_rect_cell_count += selected_rect_cell_count;
                let candidate_reserved_update_start = Instant::now();
                candidate_reserved_index.insert_rect(
                    plan.selected_grid_rect.min_x,
                    plan.selected_grid_rect.max_x,
                    plan.selected_grid_rect.min_y,
                    plan.selected_grid_rect.max_y,
                    grid_width,
                );
                candidate_reserved_has_cells = true;
                candidate_result
                    .wrapper_profile_total
                    .candidate_reserved_update_s +=
                    candidate_reserved_update_start.elapsed().as_secs_f64();
                candidate_result.plans.push(RegisteredRequirementEdgePlan {
                    plan,
                    endpoint_inset_um: *endpoint_inset_um,
                });
            }

            let planned = candidate_result.failed_reason.is_none();
            call_wrapper_profile.add(&candidate_result.wrapper_profile_total);
            candidate_results.push(candidate_result);
            if planned {
                selected_candidate_index = Some(candidate_index);
                break;
            }
        }

        let result = RegisteredRequirementResult {
            selected_candidate_index,
            candidate_results,
            planner_profile_total: call_profile_totals,
            wrapper_profile_total: call_wrapper_profile,
            endpoint_inset_um: *endpoint_inset_um,
            attempted_endpoint_insets_um: attempted_endpoint_insets_um.clone(),
            box_depths_um: box_depths_um.to_vec(),
            endpoint_insets_um: endpoint_insets_um.to_vec(),
            fixed_endpoint_inset,
        };
        if result.selected_candidate_index.is_some() {
            return Ok(result);
        }
        last_result = Some(result);
    }

    last_result.ok_or_else(|| "endpoint inset sweep produced no result".to_string())
}

#[allow(clippy::too_many_arguments)]
pub fn plan_registered_geometry_request_sequence(
    geometry_indices: &[usize],
    requested_extra_lengths_um: &[f64],
    registered_geometries: &[RegisteredMeanderGeometry],
    registered_open_cells: &[FxHashSet<CellKey>],
    registered_open_indices: &[SparseCellIndex],
    base_prefix: &DenseOccupancyPrefix,
    reserved_index: Option<&SparseCellIndex>,
    grid: &GeometryGridSpec,
    grid_width: i32,
    grid_height: i32,
    box_depths_um: &[f64],
    endpoint_insets_um: &[f64],
    fixed_endpoint_inset: bool,
    effective_radius_um: f64,
    min_straight_um: f64,
    max_meander_height_um: f64,
    min_segment_length_um: f64,
    clearance_radius_cells: i32,
    side_policy: AutoMeanderSidePolicy,
    mode: MeanderPlanningMode,
) -> Result<RegisteredRequirementResult, String> {
    if geometry_indices.len() != requested_extra_lengths_um.len() {
        return Err(
            "geometry and requested length inputs must have matching lengths".into(),
        );
    }
    if geometry_indices.is_empty() {
        return Err("geometry sequence must not be empty".into());
    }
    if box_depths_um.is_empty() {
        return Err("box_depths_um must not be empty".into());
    }
    if box_depths_um.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err("box_depths_um values must be finite and > 0".into());
    }
    if endpoint_insets_um.is_empty() {
        return Err("endpoint_insets_um must not be empty".into());
    }
    if endpoint_insets_um
        .iter()
        .any(|value| !value.is_finite() || *value < 0.0)
    {
        return Err("endpoint_insets_um values must be finite and >= 0".into());
    }
    if requested_extra_lengths_um
        .iter()
        .any(|value| *value <= 0.0)
    {
        return Err("requested lengths must be > 0".into());
    }
    if min_straight_um < 0.0 {
        return Err("min_straight_um must be >= 0".into());
    }
    if max_meander_height_um <= 0.0 {
        return Err("max_meander_height_um must be > 0".into());
    }
    if min_segment_length_um <= 0.0 {
        return Err("min_segment_length_um must be > 0".into());
    }
    if clearance_radius_cells < 0 {
        return Err("clearance_radius_cells must be >= 0".into());
    }

    let mut attempted_endpoint_insets_um: Vec<f64> = Vec::with_capacity(endpoint_insets_um.len());
    let mut last_result: Option<RegisteredRequirementResult> = None;

    for endpoint_inset_um in endpoint_insets_um {
        attempted_endpoint_insets_um.push(*endpoint_inset_um);
        let reserved_snapshot_start = Instant::now();
        let mut call_wrapper_profile = MeanderWrapperProfileTotals::default();
        call_wrapper_profile.reserved_snapshot_s += reserved_snapshot_start.elapsed().as_secs_f64();
        let mut call_profile_totals = MeanderPlanningProfileTotals::default();
        let mut candidate_reserved_index = SparseCellIndex::empty(grid_height);
        let mut candidate_reserved_has_cells = false;
        let mut candidate_result = RegisteredRequirementCandidateResult {
            candidate_index: 0,
            plans: Vec::with_capacity(geometry_indices.len()),
            candidate_runs: 0,
            candidate_intervals: 0,
            rejected_box_blocked: 0,
            rejected_planning_failed: 0,
            rejected_exact_length_mismatch: 0,
            rejected_too_short: 0,
            planner_profile_total: MeanderPlanningProfileTotals::default(),
            wrapper_profile_total: MeanderWrapperProfileTotals::default(),
            failed_reason: None,
            failed_edge_index: None,
        };

        for (edge_index, (geometry_index, requested_extra_length_um)) in geometry_indices
            .iter()
            .zip(requested_extra_lengths_um.iter())
            .enumerate()
        {
            let geometry = registered_geometries
                .get(*geometry_index)
                .ok_or_else(|| "registered meander geometry index is out of range".to_string())?;
            let opened_ref = registered_open_cells
                .get(geometry.registered_open_index)
                .ok_or_else(|| "registered meander route index is out of range".to_string())?;
            let opened_index_ref = registered_open_indices
                .get(geometry.registered_open_index)
                .ok_or_else(|| "registered meander route index is out of range".to_string())?;
            candidate_result
                .wrapper_profile_total
                .extra_blocked_prepare_calls += 1;
            let candidate_reserved_index_ref =
                candidate_reserved_has_cells.then_some(&candidate_reserved_index);
            let cfg = AutoMeanderConfig {
                requested_extra_length_um: *requested_extra_length_um,
                min_bend_radius_um: effective_radius_um,
                min_straight_um,
                max_bumps: geometry.max_bumps,
                max_meander_height_um,
                box_depth_um: box_depths_um[0],
                min_segment_length_um,
                endpoint_inset_um: *endpoint_inset_um,
                clearance_radius_cells,
                side_policy,
                mode,
            };
            let planner_call_start = Instant::now();
            let plan = match plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
                &geometry.centerline,
                grid,
                base_prefix,
                Some(opened_ref),
                Some(opened_index_ref),
                None,
                reserved_index,
                candidate_reserved_index_ref,
                &cfg,
                box_depths_um,
            ) {
                Ok(plan) => {
                    candidate_result.wrapper_profile_total.planner_call_s +=
                        planner_call_start.elapsed().as_secs_f64();
                    plan
                }
                Err(err) => {
                    candidate_result.wrapper_profile_total.planner_call_s +=
                        planner_call_start.elapsed().as_secs_f64();
                    candidate_result.failed_reason = Some(err.to_string());
                    candidate_result.failed_edge_index = Some(edge_index);
                    break;
                }
            };
            candidate_result.planner_profile_total.add(&plan.profile);
            call_profile_totals.add(&plan.profile);
            candidate_result.candidate_runs += plan.candidate_runs;
            candidate_result.candidate_intervals += plan.candidate_intervals;
            candidate_result.rejected_box_blocked += plan.rejected_box_blocked;
            candidate_result.rejected_planning_failed += plan.rejected_planning_failed;
            candidate_result.rejected_exact_length_mismatch +=
                plan.rejected_exact_length_mismatch;
            candidate_result.rejected_too_short += plan.rejected_too_short;
            let selected_rect_start = Instant::now();
            let selected_rect_cell_count = cell_count_in_grid_rect(plan.selected_grid_rect);
            candidate_result.wrapper_profile_total.selected_rect_cells_s +=
                selected_rect_start.elapsed().as_secs_f64();
            candidate_result
                .wrapper_profile_total
                .selected_rect_cell_count += selected_rect_cell_count;
            let candidate_reserved_update_start = Instant::now();
            candidate_reserved_index.insert_rect(
                plan.selected_grid_rect.min_x,
                plan.selected_grid_rect.max_x,
                plan.selected_grid_rect.min_y,
                plan.selected_grid_rect.max_y,
                grid_width,
            );
            candidate_reserved_has_cells = true;
            candidate_result
                .wrapper_profile_total
                .candidate_reserved_update_s +=
                candidate_reserved_update_start.elapsed().as_secs_f64();
            candidate_result.plans.push(RegisteredRequirementEdgePlan {
                plan,
                endpoint_inset_um: *endpoint_inset_um,
            });
        }

        let selected_candidate_index = candidate_result
            .failed_reason
            .is_none()
            .then_some(0);
        call_wrapper_profile.add(&candidate_result.wrapper_profile_total);
        let result = RegisteredRequirementResult {
            selected_candidate_index,
            candidate_results: vec![candidate_result],
            planner_profile_total: call_profile_totals,
            wrapper_profile_total: call_wrapper_profile,
            endpoint_inset_um: *endpoint_inset_um,
            attempted_endpoint_insets_um: attempted_endpoint_insets_um.clone(),
            box_depths_um: box_depths_um.to_vec(),
            endpoint_insets_um: endpoint_insets_um.to_vec(),
            fixed_endpoint_inset,
        };
        if result.selected_candidate_index.is_some() {
            return Ok(result);
        }
        last_result = Some(result);
    }

    last_result.ok_or_else(|| "endpoint inset sweep produced no result".to_string())
}
