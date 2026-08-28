use std::time::Instant;

use rustc_hash::FxHashSet;

use crate::auto_meander::{
    cell_count_in_grid_rect, plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix,
    AutoMeanderConfig, AutoMeanderPlanningProfile, AutoMeanderSidePolicy,
    AutoRouteAnalyticMeanderPlan, DenseOccupancyPrefix, SparseCellIndex,
};
use crate::geometry_realization::GeometryGridSpec;
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

    pub fn add_totals(&mut self, other: &MeanderPlanningProfileTotals) {
        self.total_s += other.total_s;
        self.run_extraction_s += other.run_extraction_s;
        self.footprint_s += other.footprint_s;
        self.free_interval_s += other.free_interval_s;
        self.box_check_s += other.box_check_s;
        self.analytic_plan_s += other.analytic_plan_s;
        self.replacement_check_s += other.replacement_check_s;
        self.depth_count += other.depth_count;
        self.run_side_checks += other.run_side_checks;
        self.box_checks += other.box_checks;
        self.analytic_plan_calls += other.analytic_plan_calls;
        self.plan_calls += other.plan_calls;
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

pub struct RegisteredFinalPlanningResult {
    pub result: RegisteredRequirementResult,
    pub planning_mode: &'static str,
    pub plan_input_indices: Vec<usize>,
}

#[allow(clippy::too_many_arguments)]
fn plan_registered_geometry_sequence_at_endpoint(
    candidate_index: usize,
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
    endpoint_inset_um: f64,
    effective_radius_um: f64,
    min_straight_um: f64,
    max_meander_height_um: f64,
    min_segment_length_um: f64,
    clearance_radius_cells: i32,
    side_policy: AutoMeanderSidePolicy,
    mode: MeanderPlanningMode,
    call_profile_totals: &mut MeanderPlanningProfileTotals,
) -> Result<RegisteredRequirementCandidateResult, String> {
    if geometry_indices.len() != requested_extra_lengths_um.len() {
        return Err("geometry and requested length inputs must have matching lengths".into());
    }
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
            endpoint_inset_um,
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
        candidate_result.rejected_exact_length_mismatch += plan.rejected_exact_length_mismatch;
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
            .candidate_reserved_update_s += candidate_reserved_update_start.elapsed().as_secs_f64();
        candidate_result.plans.push(RegisteredRequirementEdgePlan {
            plan,
            endpoint_inset_um,
        });
    }

    Ok(candidate_result)
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
            let requested_extra_lengths_um =
                vec![requested_extra_length_um; geometry_indices.len()];
            let candidate_result = plan_registered_geometry_sequence_at_endpoint(
                candidate_index,
                geometry_indices,
                &requested_extra_lengths_um,
                registered_geometries,
                registered_open_cells,
                registered_open_indices,
                base_prefix,
                reserved_index,
                grid,
                grid_width,
                grid_height,
                box_depths_um,
                *endpoint_inset_um,
                effective_radius_um,
                min_straight_um,
                max_meander_height_um,
                min_segment_length_um,
                clearance_radius_cells,
                side_policy,
                mode,
                &mut call_profile_totals,
            )?;

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
        return Err("geometry and requested length inputs must have matching lengths".into());
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
    if requested_extra_lengths_um.iter().any(|value| *value <= 0.0) {
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
        let candidate_result = plan_registered_geometry_sequence_at_endpoint(
            0,
            geometry_indices,
            requested_extra_lengths_um,
            registered_geometries,
            registered_open_cells,
            registered_open_indices,
            base_prefix,
            reserved_index,
            grid,
            grid_width,
            grid_height,
            box_depths_um,
            *endpoint_inset_um,
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy,
            mode,
            &mut call_profile_totals,
        )?;

        let selected_candidate_index = candidate_result.failed_reason.is_none().then_some(0);
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

#[allow(clippy::too_many_arguments)]
pub fn plan_registered_geometry_split_request(
    geometry_index: usize,
    requested_extra_length_um: f64,
    min_insertable_extra_um: f64,
    max_parts: usize,
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
    if requested_extra_length_um <= 0.0 || !requested_extra_length_um.is_finite() {
        return Err("requested extra length must be finite and > 0".into());
    }
    if min_insertable_extra_um <= 0.0 || !min_insertable_extra_um.is_finite() {
        return Err("minimum insertable extra length must be finite and > 0".into());
    }
    if max_parts < 2 {
        return Err("max_parts must be >= 2".into());
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

    let largest_part_count =
        max_parts.min((requested_extra_length_um / min_insertable_extra_um) as usize);
    if largest_part_count < 2 {
        return Err("requested extra length cannot be split into legal chunks".into());
    }

    let mut attempted_endpoint_insets_um: Vec<f64> = Vec::with_capacity(endpoint_insets_um.len());
    let mut last_result: Option<RegisteredRequirementResult> = None;

    for endpoint_inset_um in endpoint_insets_um {
        attempted_endpoint_insets_um.push(*endpoint_inset_um);
        let reserved_snapshot_start = Instant::now();
        let mut call_wrapper_profile = MeanderWrapperProfileTotals::default();
        call_wrapper_profile.reserved_snapshot_s += reserved_snapshot_start.elapsed().as_secs_f64();
        let mut call_profile_totals = MeanderPlanningProfileTotals::default();
        let mut candidate_results = Vec::with_capacity(largest_part_count - 1);
        let mut selected_candidate_index: Option<usize> = None;

        for part_count in 2..=largest_part_count {
            let chunk_request = requested_extra_length_um / part_count as f64;
            if chunk_request < min_insertable_extra_um {
                continue;
            }
            let geometry_indices = vec![geometry_index; part_count];
            let requested_lengths = vec![chunk_request; part_count];
            let candidate_result = plan_registered_geometry_sequence_at_endpoint(
                part_count,
                &geometry_indices,
                &requested_lengths,
                registered_geometries,
                registered_open_cells,
                registered_open_indices,
                base_prefix,
                reserved_index,
                grid,
                grid_width,
                grid_height,
                box_depths_um,
                *endpoint_inset_um,
                effective_radius_um,
                min_straight_um,
                max_meander_height_um,
                min_segment_length_um,
                clearance_radius_cells,
                side_policy,
                mode,
                &mut call_profile_totals,
            )?;

            let planned = candidate_result.failed_reason.is_none();
            call_wrapper_profile.add(&candidate_result.wrapper_profile_total);
            candidate_results.push(candidate_result);
            if planned {
                selected_candidate_index = Some(part_count);
                break;
            }
        }

        if candidate_results.is_empty() {
            continue;
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

    last_result.ok_or_else(|| "split request produced no candidate result".to_string())
}

#[allow(clippy::too_many_arguments)]
pub fn plan_registered_geometry_final_requests(
    geometry_indices: &[usize],
    requested_extra_lengths_um: &[f64],
    min_insertable_extra_um: f64,
    max_split_parts: usize,
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
) -> Result<RegisteredFinalPlanningResult, String> {
    if geometry_indices.len() != requested_extra_lengths_um.len() {
        return Err("geometry and requested length inputs must have matching lengths".into());
    }
    if geometry_indices.is_empty() {
        return Ok(RegisteredFinalPlanningResult {
            result: RegisteredRequirementResult {
                selected_candidate_index: Some(0),
                candidate_results: vec![RegisteredRequirementCandidateResult {
                    candidate_index: 0,
                    plans: Vec::new(),
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
                }],
                planner_profile_total: MeanderPlanningProfileTotals::default(),
                wrapper_profile_total: MeanderWrapperProfileTotals::default(),
                endpoint_inset_um: endpoint_insets_um.first().copied().unwrap_or(0.0),
                attempted_endpoint_insets_um: Vec::new(),
                box_depths_um: box_depths_um.to_vec(),
                endpoint_insets_um: endpoint_insets_um.to_vec(),
                fixed_endpoint_inset,
            },
            planning_mode: "none",
            plan_input_indices: Vec::new(),
        });
    }

    let sequence_result = plan_registered_geometry_request_sequence(
        geometry_indices,
        requested_extra_lengths_um,
        registered_geometries,
        registered_open_cells,
        registered_open_indices,
        base_prefix,
        reserved_index,
        grid,
        grid_width,
        grid_height,
        box_depths_um,
        endpoint_insets_um,
        fixed_endpoint_inset,
        effective_radius_um,
        min_straight_um,
        max_meander_height_um,
        min_segment_length_um,
        clearance_radius_cells,
        side_policy,
        mode,
    )?;
    if sequence_result.selected_candidate_index.is_some() {
        return Ok(RegisteredFinalPlanningResult {
            result: sequence_result,
            planning_mode: "rust_registered_sequence",
            plan_input_indices: (0..geometry_indices.len()).collect(),
        });
    }

    let mut aggregate_candidate = RegisteredRequirementCandidateResult {
        candidate_index: 0,
        plans: Vec::new(),
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
    let mut aggregate_planner_profile = MeanderPlanningProfileTotals::default();
    let mut aggregate_wrapper_profile = MeanderWrapperProfileTotals::default();
    let mut plan_input_indices = Vec::new();
    let mut planning_mode = "rust_registered_per_edge_fallback";
    let mut attempted_endpoint_insets_um = Vec::new();

    for (input_index, (geometry_index, requested_extra_length_um)) in geometry_indices
        .iter()
        .zip(requested_extra_lengths_um.iter())
        .enumerate()
    {
        let direct_result = plan_registered_geometry_request_sequence(
            &[*geometry_index],
            &[*requested_extra_length_um],
            registered_geometries,
            registered_open_cells,
            registered_open_indices,
            base_prefix,
            reserved_index,
            grid,
            grid_width,
            grid_height,
            box_depths_um,
            endpoint_insets_um,
            fixed_endpoint_inset,
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy,
            mode,
        )?;
        aggregate_planner_profile.add_totals(&direct_result.planner_profile_total);
        aggregate_wrapper_profile.add(&direct_result.wrapper_profile_total);
        attempted_endpoint_insets_um
            .extend(direct_result.attempted_endpoint_insets_um.iter().copied());
        if let Some(candidate) = direct_result.selected_candidate_index.and_then(|selected| {
            direct_result
                .candidate_results
                .iter()
                .find(|c| c.candidate_index == selected)
        }) {
            aggregate_candidate.candidate_runs += candidate.candidate_runs;
            aggregate_candidate.candidate_intervals += candidate.candidate_intervals;
            aggregate_candidate.rejected_box_blocked += candidate.rejected_box_blocked;
            aggregate_candidate.rejected_planning_failed += candidate.rejected_planning_failed;
            aggregate_candidate.rejected_exact_length_mismatch +=
                candidate.rejected_exact_length_mismatch;
            aggregate_candidate.rejected_too_short += candidate.rejected_too_short;
            aggregate_candidate
                .planner_profile_total
                .add_totals(&candidate.planner_profile_total);
            aggregate_candidate
                .wrapper_profile_total
                .add(&candidate.wrapper_profile_total);
            for plan in &candidate.plans {
                aggregate_candidate
                    .plans
                    .push(RegisteredRequirementEdgePlan {
                        plan: plan.plan.clone(),
                        endpoint_inset_um: plan.endpoint_inset_um,
                    });
                plan_input_indices.push(input_index);
            }
            continue;
        }

        let split_result = plan_registered_geometry_split_request(
            *geometry_index,
            *requested_extra_length_um,
            min_insertable_extra_um,
            max_split_parts,
            registered_geometries,
            registered_open_cells,
            registered_open_indices,
            base_prefix,
            reserved_index,
            grid,
            grid_width,
            grid_height,
            box_depths_um,
            endpoint_insets_um,
            fixed_endpoint_inset,
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy,
            mode,
        )?;
        aggregate_planner_profile.add_totals(&split_result.planner_profile_total);
        aggregate_wrapper_profile.add(&split_result.wrapper_profile_total);
        attempted_endpoint_insets_um
            .extend(split_result.attempted_endpoint_insets_um.iter().copied());
        if let Some(candidate) = split_result.selected_candidate_index.and_then(|selected| {
            split_result
                .candidate_results
                .iter()
                .find(|c| c.candidate_index == selected)
        }) {
            planning_mode = "rust_registered_split_route_runs";
            aggregate_candidate.candidate_runs += candidate.candidate_runs;
            aggregate_candidate.candidate_intervals += candidate.candidate_intervals;
            aggregate_candidate.rejected_box_blocked += candidate.rejected_box_blocked;
            aggregate_candidate.rejected_planning_failed += candidate.rejected_planning_failed;
            aggregate_candidate.rejected_exact_length_mismatch +=
                candidate.rejected_exact_length_mismatch;
            aggregate_candidate.rejected_too_short += candidate.rejected_too_short;
            aggregate_candidate
                .planner_profile_total
                .add_totals(&candidate.planner_profile_total);
            aggregate_candidate
                .wrapper_profile_total
                .add(&candidate.wrapper_profile_total);
            for plan in &candidate.plans {
                aggregate_candidate
                    .plans
                    .push(RegisteredRequirementEdgePlan {
                        plan: plan.plan.clone(),
                        endpoint_inset_um: plan.endpoint_inset_um,
                    });
                plan_input_indices.push(input_index);
            }
            continue;
        }

        let failed_reason = split_result
            .candidate_results
            .last()
            .and_then(|candidate| candidate.failed_reason.clone())
            .or_else(|| {
                direct_result
                    .candidate_results
                    .last()
                    .and_then(|candidate| candidate.failed_reason.clone())
            })
            .unwrap_or_else(|| "no exact final aggregate meander candidate found".to_string());
        aggregate_candidate.failed_reason = Some(failed_reason);
        aggregate_candidate.failed_edge_index = Some(input_index);
        break;
    }

    let selected_candidate_index = aggregate_candidate.failed_reason.is_none().then_some(0);
    Ok(RegisteredFinalPlanningResult {
        result: RegisteredRequirementResult {
            selected_candidate_index,
            candidate_results: vec![aggregate_candidate],
            planner_profile_total: aggregate_planner_profile,
            wrapper_profile_total: aggregate_wrapper_profile,
            endpoint_inset_um: endpoint_insets_um.first().copied().unwrap_or(0.0),
            attempted_endpoint_insets_um,
            box_depths_um: box_depths_um.to_vec(),
            endpoint_insets_um: endpoint_insets_um.to_vec(),
            fixed_endpoint_inset,
        },
        planning_mode,
        plan_input_indices,
    })
}

/// Interface for substituting a different registered-meander planning implementation
/// behind future call sites. `RustRegisteredMeanderPlanner` delegates to the current
/// free functions, leaving production callers unchanged.
pub trait RegisteredMeanderPlanner {
    #[allow(clippy::too_many_arguments)]
    fn plan_requirement_candidates(
        &self,
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
    ) -> Result<RegisteredRequirementResult, String>;

    #[allow(clippy::too_many_arguments)]
    fn plan_request_sequence(
        &self,
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
    ) -> Result<RegisteredRequirementResult, String>;

    #[allow(clippy::too_many_arguments)]
    fn plan_split_request(
        &self,
        geometry_index: usize,
        requested_extra_length_um: f64,
        min_insertable_extra_um: f64,
        max_parts: usize,
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
    ) -> Result<RegisteredRequirementResult, String>;

    #[allow(clippy::too_many_arguments)]
    fn plan_final_requests(
        &self,
        geometry_indices: &[usize],
        requested_extra_lengths_um: &[f64],
        min_insertable_extra_um: f64,
        max_split_parts: usize,
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
    ) -> Result<RegisteredFinalPlanningResult, String>;
}

pub struct RustRegisteredMeanderPlanner;

impl RegisteredMeanderPlanner for RustRegisteredMeanderPlanner {
    #[allow(clippy::too_many_arguments)]
    fn plan_requirement_candidates(
        &self,
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
        plan_registered_geometry_requirement_candidates(
            candidate_geometry_indices,
            candidate_requested_extra_lengths_um,
            registered_geometries,
            registered_open_cells,
            registered_open_indices,
            base_prefix,
            reserved_index,
            grid,
            grid_width,
            grid_height,
            box_depths_um,
            endpoint_insets_um,
            fixed_endpoint_inset,
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy,
            mode,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_request_sequence(
        &self,
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
        plan_registered_geometry_request_sequence(
            geometry_indices,
            requested_extra_lengths_um,
            registered_geometries,
            registered_open_cells,
            registered_open_indices,
            base_prefix,
            reserved_index,
            grid,
            grid_width,
            grid_height,
            box_depths_um,
            endpoint_insets_um,
            fixed_endpoint_inset,
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy,
            mode,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_split_request(
        &self,
        geometry_index: usize,
        requested_extra_length_um: f64,
        min_insertable_extra_um: f64,
        max_parts: usize,
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
        plan_registered_geometry_split_request(
            geometry_index,
            requested_extra_length_um,
            min_insertable_extra_um,
            max_parts,
            registered_geometries,
            registered_open_cells,
            registered_open_indices,
            base_prefix,
            reserved_index,
            grid,
            grid_width,
            grid_height,
            box_depths_um,
            endpoint_insets_um,
            fixed_endpoint_inset,
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy,
            mode,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_final_requests(
        &self,
        geometry_indices: &[usize],
        requested_extra_lengths_um: &[f64],
        min_insertable_extra_um: f64,
        max_split_parts: usize,
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
    ) -> Result<RegisteredFinalPlanningResult, String> {
        plan_registered_geometry_final_requests(
            geometry_indices,
            requested_extra_lengths_um,
            min_insertable_extra_um,
            max_split_parts,
            registered_geometries,
            registered_open_cells,
            registered_open_indices,
            base_prefix,
            reserved_index,
            grid,
            grid_width,
            grid_height,
            box_depths_um,
            endpoint_insets_um,
            fixed_endpoint_inset,
            effective_radius_um,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            side_policy,
            mode,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GRID_WIDTH: i32 = 30;
    const GRID_HEIGHT: i32 = 30;
    const DEFAULT_BOX_DEPTHS_UM: &[f64] = &[1.6];
    const DEFAULT_ENDPOINT_INSETS_UM: &[f64] = &[0.0];
    const DEFAULT_EFFECTIVE_RADIUS_UM: f64 = 0.2;
    const DEFAULT_MIN_STRAIGHT_UM: f64 = 0.1;
    const DEFAULT_MAX_MEANDER_HEIGHT_UM: f64 = 20.0;
    const DEFAULT_MIN_SEGMENT_LENGTH_UM: f64 = 1.0;
    const DEFAULT_CLEARANCE_RADIUS_CELLS: i32 = 0;

    struct PlanningFixture {
        registered_geometries: Vec<RegisteredMeanderGeometry>,
        registered_open_cells: Vec<FxHashSet<CellKey>>,
        registered_open_indices: Vec<SparseCellIndex>,
        base_prefix: DenseOccupancyPrefix,
        grid: GeometryGridSpec,
        grid_width: i32,
        grid_height: i32,
    }

    fn grid() -> GeometryGridSpec {
        GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap()
    }

    fn straight_centerline_at(y_um: f64) -> Vec<(f64, f64)> {
        vec![(1.5, y_um), (5.5, y_um)]
    }

    fn registered_geometry(
        centerline: Vec<(f64, f64)>,
        registered_open_index: usize,
    ) -> RegisteredMeanderGeometry {
        RegisteredMeanderGeometry {
            centerline,
            registered_open_index,
            max_bumps: 2,
        }
    }

    fn empty_open_index() -> SparseCellIndex {
        SparseCellIndex::empty(GRID_HEIGHT)
    }

    impl PlanningFixture {
        fn from_map_and_geometries(
            map: &ObstacleMap,
            registered_geometries: Vec<RegisteredMeanderGeometry>,
        ) -> Self {
            let route_count = registered_geometries
                .iter()
                .map(|geometry| geometry.registered_open_index)
                .max()
                .map(|index| index + 1)
                .unwrap_or(0);
            Self {
                registered_geometries,
                registered_open_cells: vec![FxHashSet::default(); route_count],
                registered_open_indices: (0..route_count).map(|_| empty_open_index()).collect(),
                base_prefix: DenseOccupancyPrefix::from_obstacle_map(map, None),
                grid: grid(),
                grid_width: map.width(),
                grid_height: map.height(),
            }
        }

        fn empty_one() -> Self {
            let map = ObstacleMap::new(GRID_WIDTH, GRID_HEIGHT);
            Self::from_map_and_geometries(
                &map,
                vec![registered_geometry(straight_centerline_at(2.5), 0)],
            )
        }
    }

    fn assert_string_error<T>(result: Result<T, String>, expected: &str) {
        match result {
            Ok(_) => panic!("expected error: {expected}"),
            Err(err) => assert_eq!(err, expected),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn call_requirement_candidates(
        fixture: &PlanningFixture,
        candidate_geometry_indices: &[Vec<usize>],
        candidate_requested_extra_lengths_um: &[f64],
        box_depths_um: &[f64],
        endpoint_insets_um: &[f64],
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
    ) -> Result<RegisteredRequirementResult, String> {
        plan_registered_geometry_requirement_candidates(
            candidate_geometry_indices,
            candidate_requested_extra_lengths_um,
            &fixture.registered_geometries,
            &fixture.registered_open_cells,
            &fixture.registered_open_indices,
            &fixture.base_prefix,
            None,
            &fixture.grid,
            fixture.grid_width,
            fixture.grid_height,
            box_depths_um,
            endpoint_insets_um,
            false,
            DEFAULT_EFFECTIVE_RADIUS_UM,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            AutoMeanderSidePolicy::Both,
            MeanderPlanningMode::FillBoxMultiBump,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn assert_requirement_candidates_error(
        candidate_geometry_indices: &[Vec<usize>],
        candidate_requested_extra_lengths_um: &[f64],
        box_depths_um: &[f64],
        endpoint_insets_um: &[f64],
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
        expected: &str,
    ) {
        let fixture = PlanningFixture::empty_one();
        assert_string_error(
            call_requirement_candidates(
                &fixture,
                candidate_geometry_indices,
                candidate_requested_extra_lengths_um,
                box_depths_um,
                endpoint_insets_um,
                min_straight_um,
                max_meander_height_um,
                min_segment_length_um,
                clearance_radius_cells,
            ),
            expected,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn call_request_sequence(
        fixture: &PlanningFixture,
        geometry_indices: &[usize],
        requested_extra_lengths_um: &[f64],
        box_depths_um: &[f64],
        endpoint_insets_um: &[f64],
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
    ) -> Result<RegisteredRequirementResult, String> {
        plan_registered_geometry_request_sequence(
            geometry_indices,
            requested_extra_lengths_um,
            &fixture.registered_geometries,
            &fixture.registered_open_cells,
            &fixture.registered_open_indices,
            &fixture.base_prefix,
            None,
            &fixture.grid,
            fixture.grid_width,
            fixture.grid_height,
            box_depths_um,
            endpoint_insets_um,
            false,
            DEFAULT_EFFECTIVE_RADIUS_UM,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            AutoMeanderSidePolicy::Both,
            MeanderPlanningMode::FillBoxMultiBump,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn assert_request_sequence_error(
        geometry_indices: &[usize],
        requested_extra_lengths_um: &[f64],
        box_depths_um: &[f64],
        endpoint_insets_um: &[f64],
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
        expected: &str,
    ) {
        let fixture = PlanningFixture::empty_one();
        assert_string_error(
            call_request_sequence(
                &fixture,
                geometry_indices,
                requested_extra_lengths_um,
                box_depths_um,
                endpoint_insets_um,
                min_straight_um,
                max_meander_height_um,
                min_segment_length_um,
                clearance_radius_cells,
            ),
            expected,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn call_split_request(
        fixture: &PlanningFixture,
        requested_extra_length_um: f64,
        min_insertable_extra_um: f64,
        max_parts: usize,
        box_depths_um: &[f64],
        endpoint_insets_um: &[f64],
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
    ) -> Result<RegisteredRequirementResult, String> {
        plan_registered_geometry_split_request(
            0,
            requested_extra_length_um,
            min_insertable_extra_um,
            max_parts,
            &fixture.registered_geometries,
            &fixture.registered_open_cells,
            &fixture.registered_open_indices,
            &fixture.base_prefix,
            None,
            &fixture.grid,
            fixture.grid_width,
            fixture.grid_height,
            box_depths_um,
            endpoint_insets_um,
            false,
            DEFAULT_EFFECTIVE_RADIUS_UM,
            min_straight_um,
            max_meander_height_um,
            min_segment_length_um,
            clearance_radius_cells,
            AutoMeanderSidePolicy::Both,
            MeanderPlanningMode::FillBoxMultiBump,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn assert_split_request_error(
        requested_extra_length_um: f64,
        min_insertable_extra_um: f64,
        max_parts: usize,
        box_depths_um: &[f64],
        endpoint_insets_um: &[f64],
        min_straight_um: f64,
        max_meander_height_um: f64,
        min_segment_length_um: f64,
        clearance_radius_cells: i32,
        expected: &str,
    ) {
        let fixture = PlanningFixture::empty_one();
        assert_string_error(
            call_split_request(
                &fixture,
                requested_extra_length_um,
                min_insertable_extra_um,
                max_parts,
                box_depths_um,
                endpoint_insets_um,
                min_straight_um,
                max_meander_height_um,
                min_segment_length_um,
                clearance_radius_cells,
            ),
            expected,
        );
    }

    fn call_final_requests(
        fixture: &PlanningFixture,
        geometry_indices: &[usize],
        requested_extra_lengths_um: &[f64],
    ) -> Result<RegisteredFinalPlanningResult, String> {
        call_final_requests_with_split_config(
            fixture,
            geometry_indices,
            requested_extra_lengths_um,
            1.0,
            4,
        )
    }

    fn call_final_requests_with_split_config(
        fixture: &PlanningFixture,
        geometry_indices: &[usize],
        requested_extra_lengths_um: &[f64],
        min_insertable_extra_um: f64,
        max_split_parts: usize,
    ) -> Result<RegisteredFinalPlanningResult, String> {
        plan_registered_geometry_final_requests(
            geometry_indices,
            requested_extra_lengths_um,
            min_insertable_extra_um,
            max_split_parts,
            &fixture.registered_geometries,
            &fixture.registered_open_cells,
            &fixture.registered_open_indices,
            &fixture.base_prefix,
            None,
            &fixture.grid,
            fixture.grid_width,
            fixture.grid_height,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            false,
            DEFAULT_EFFECTIVE_RADIUS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            AutoMeanderSidePolicy::Both,
            MeanderPlanningMode::FillBoxMultiBump,
        )
    }

    #[test]
    fn requirement_candidates_rejects_mismatched_candidate_and_length_inputs() {
        assert_requirement_candidates_error(
            &[vec![0], vec![0]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "candidate geometry and requested length inputs must have matching lengths",
        );
    }

    #[test]
    fn requirement_candidates_rejects_empty_candidate_list() {
        assert_requirement_candidates_error(
            &[],
            &[],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "candidate list must not be empty",
        );
    }

    #[test]
    fn requirement_candidates_rejects_empty_box_depths() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            &[],
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "box_depths_um must not be empty",
        );
    }

    #[test]
    fn requirement_candidates_rejects_nonpositive_box_depth() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            &[0.0],
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "box_depths_um values must be finite and > 0",
        );
    }

    #[test]
    fn requirement_candidates_rejects_empty_endpoint_insets() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            &[],
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "endpoint_insets_um must not be empty",
        );
    }

    #[test]
    fn requirement_candidates_rejects_negative_endpoint_inset() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            &[-1.0],
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "endpoint_insets_um values must be finite and >= 0",
        );
    }

    #[test]
    fn requirement_candidates_rejects_nonpositive_requested_length() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[0.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "candidate requested lengths must be > 0",
        );
    }

    #[test]
    fn requirement_candidates_rejects_negative_min_straight() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            -1.0,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "min_straight_um must be >= 0",
        );
    }

    #[test]
    fn requirement_candidates_rejects_nonpositive_max_meander_height() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            0.0,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "max_meander_height_um must be > 0",
        );
    }

    #[test]
    fn requirement_candidates_rejects_nonpositive_min_segment_length() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            0.0,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "min_segment_length_um must be > 0",
        );
    }

    #[test]
    fn requirement_candidates_rejects_negative_clearance_radius() {
        assert_requirement_candidates_error(
            &[vec![0]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            -1,
            "clearance_radius_cells must be >= 0",
        );
    }

    #[test]
    fn requirement_candidates_rejects_empty_candidate_bundle() {
        assert_requirement_candidates_error(
            &[vec![]],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "candidate bundles must not be empty",
        );
    }

    #[test]
    fn request_sequence_rejects_mismatched_geometry_and_length_inputs() {
        assert_request_sequence_error(
            &[0, 0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "geometry and requested length inputs must have matching lengths",
        );
    }

    #[test]
    fn request_sequence_rejects_empty_geometry_sequence() {
        assert_request_sequence_error(
            &[],
            &[],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "geometry sequence must not be empty",
        );
    }

    #[test]
    fn request_sequence_rejects_empty_box_depths() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            &[],
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "box_depths_um must not be empty",
        );
    }

    #[test]
    fn request_sequence_rejects_nonpositive_box_depth() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            &[0.0],
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "box_depths_um values must be finite and > 0",
        );
    }

    #[test]
    fn request_sequence_rejects_empty_endpoint_insets() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            &[],
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "endpoint_insets_um must not be empty",
        );
    }

    #[test]
    fn request_sequence_rejects_negative_endpoint_inset() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            &[-1.0],
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "endpoint_insets_um values must be finite and >= 0",
        );
    }

    #[test]
    fn request_sequence_rejects_nonpositive_requested_length() {
        assert_request_sequence_error(
            &[0],
            &[0.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "requested lengths must be > 0",
        );
    }

    #[test]
    fn request_sequence_rejects_negative_min_straight() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            -1.0,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "min_straight_um must be >= 0",
        );
    }

    #[test]
    fn request_sequence_rejects_nonpositive_max_meander_height() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            0.0,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "max_meander_height_um must be > 0",
        );
    }

    #[test]
    fn request_sequence_rejects_nonpositive_min_segment_length() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            0.0,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "min_segment_length_um must be > 0",
        );
    }

    #[test]
    fn request_sequence_rejects_negative_clearance_radius() {
        assert_request_sequence_error(
            &[0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            -1,
            "clearance_radius_cells must be >= 0",
        );
    }

    #[test]
    fn split_request_rejects_nonpositive_requested_extra_length() {
        assert_split_request_error(
            0.0,
            1.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "requested extra length must be finite and > 0",
        );
    }

    #[test]
    fn split_request_rejects_nonpositive_min_insertable_extra() {
        assert_split_request_error(
            4.0,
            0.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "minimum insertable extra length must be finite and > 0",
        );
    }

    #[test]
    fn split_request_rejects_max_parts_less_than_two() {
        assert_split_request_error(
            4.0,
            1.0,
            1,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "max_parts must be >= 2",
        );
    }

    #[test]
    fn split_request_rejects_empty_box_depths() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            &[],
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "box_depths_um must not be empty",
        );
    }

    #[test]
    fn split_request_rejects_nonpositive_box_depth() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            &[0.0],
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "box_depths_um values must be finite and > 0",
        );
    }

    #[test]
    fn split_request_rejects_empty_endpoint_insets() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            &[],
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "endpoint_insets_um must not be empty",
        );
    }

    #[test]
    fn split_request_rejects_negative_endpoint_inset() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            &[-1.0],
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "endpoint_insets_um values must be finite and >= 0",
        );
    }

    #[test]
    fn split_request_rejects_negative_min_straight() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            -1.0,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "min_straight_um must be >= 0",
        );
    }

    #[test]
    fn split_request_rejects_nonpositive_max_meander_height() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            0.0,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "max_meander_height_um must be > 0",
        );
    }

    #[test]
    fn split_request_rejects_nonpositive_min_segment_length() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            0.0,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "min_segment_length_um must be > 0",
        );
    }

    #[test]
    fn split_request_rejects_negative_clearance_radius() {
        assert_split_request_error(
            4.0,
            1.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            -1,
            "clearance_radius_cells must be >= 0",
        );
    }

    #[test]
    fn split_request_rejects_unsplittable_legal_chunks() {
        assert_split_request_error(
            1.0,
            10.0,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            "requested extra length cannot be split into legal chunks",
        );
    }

    #[test]
    fn final_requests_rejects_mismatched_geometry_and_length_inputs() {
        let fixture = PlanningFixture::empty_one();
        assert_string_error(
            call_final_requests(&fixture, &[0, 0], &[1.0]),
            "geometry and requested length inputs must have matching lengths",
        );
    }

    #[test]
    fn request_sequence_plans_single_geometry_on_empty_map() {
        let fixture = PlanningFixture::empty_one();
        let result = call_request_sequence(
            &fixture,
            &[0],
            &[1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
        )
        .expect("empty map should support registered sequence planning");

        assert_eq!(result.selected_candidate_index, Some(0));
        assert_eq!(result.status(), "planned");
        assert_eq!(result.candidate_results.len(), 1);
        assert!(!result.candidate_results[0].plans.is_empty());
    }

    #[test]
    fn registered_meander_planner_delegates_request_sequence() {
        let fixture = PlanningFixture::empty_one();
        let planner = RustRegisteredMeanderPlanner;
        let trait_result = planner
            .plan_request_sequence(
                &[0],
                &[1.0],
                &fixture.registered_geometries,
                &fixture.registered_open_cells,
                &fixture.registered_open_indices,
                &fixture.base_prefix,
                None,
                &fixture.grid,
                fixture.grid_width,
                fixture.grid_height,
                DEFAULT_BOX_DEPTHS_UM,
                DEFAULT_ENDPOINT_INSETS_UM,
                false,
                DEFAULT_EFFECTIVE_RADIUS_UM,
                DEFAULT_MIN_STRAIGHT_UM,
                DEFAULT_MAX_MEANDER_HEIGHT_UM,
                DEFAULT_MIN_SEGMENT_LENGTH_UM,
                DEFAULT_CLEARANCE_RADIUS_CELLS,
                AutoMeanderSidePolicy::Both,
                MeanderPlanningMode::FillBoxMultiBump,
            )
            .expect("trait planner should match direct registered sequence planning");
        let direct_result = plan_registered_geometry_request_sequence(
            &[0],
            &[1.0],
            &fixture.registered_geometries,
            &fixture.registered_open_cells,
            &fixture.registered_open_indices,
            &fixture.base_prefix,
            None,
            &fixture.grid,
            fixture.grid_width,
            fixture.grid_height,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            false,
            DEFAULT_EFFECTIVE_RADIUS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
            AutoMeanderSidePolicy::Both,
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .expect("direct registered sequence planning should succeed");

        assert_eq!(
            trait_result.selected_candidate_index,
            direct_result.selected_candidate_index
        );
        assert_eq!(trait_result.status(), direct_result.status());
    }

    #[test]
    fn requirement_candidates_falls_back_to_later_feasible_candidate() {
        let mut map = ObstacleMap::new(GRID_WIDTH, GRID_HEIGHT);
        for x in 1..=5 {
            assert!(map.add_static_cell(x, 3));
            assert!(map.add_static_cell(x, 1));
        }
        let fixture = PlanningFixture::from_map_and_geometries(
            &map,
            vec![
                registered_geometry(straight_centerline_at(2.5), 0),
                registered_geometry(straight_centerline_at(10.5), 1),
            ],
        );

        let result = call_requirement_candidates(
            &fixture,
            &[vec![0], vec![1]],
            &[1.0, 1.0],
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
        )
        .expect("second registered geometry should remain feasible");

        assert_eq!(result.selected_candidate_index, Some(1));
        assert_eq!(result.candidate_results.len(), 2);
        assert_eq!(result.candidate_results[0].status(), "no_candidate");
        assert_eq!(result.candidate_results[1].status(), "planned");
    }

    #[test]
    fn split_request_success_reports_part_count_as_selected_candidate_index() {
        let map = ObstacleMap::new(GRID_WIDTH, GRID_HEIGHT);
        let fixture = PlanningFixture::from_map_and_geometries(
            &map,
            vec![registered_geometry(vec![(1.5, 10.5), (15.5, 10.5)], 0)],
        );
        let result = call_split_request(
            &fixture,
            2.0,
            0.5,
            4,
            DEFAULT_BOX_DEPTHS_UM,
            DEFAULT_ENDPOINT_INSETS_UM,
            DEFAULT_MIN_STRAIGHT_UM,
            DEFAULT_MAX_MEANDER_HEIGHT_UM,
            DEFAULT_MIN_SEGMENT_LENGTH_UM,
            DEFAULT_CLEARANCE_RADIUS_CELLS,
        )
        .expect("empty map should support the first legal split");

        assert_eq!(result.selected_candidate_index, Some(2));
        assert_eq!(result.candidate_results.len(), 1);
        assert_eq!(result.candidate_results[0].candidate_index, 2);
        assert_eq!(result.candidate_results[0].plans.len(), 2);
    }

    #[test]
    fn final_requests_empty_input_uses_none_fast_path() {
        let fixture = PlanningFixture::empty_one();
        let result = call_final_requests(&fixture, &[], &[])
            .expect("empty final request input should use the none fast path");

        assert_eq!(result.planning_mode, "none");
        assert!(result.plan_input_indices.is_empty());
        assert_eq!(result.result.selected_candidate_index, Some(0));
        assert_eq!(result.result.candidate_results.len(), 1);
        assert!(result.result.candidate_results[0].plans.is_empty());
    }

    #[test]
    fn final_requests_direct_sequence_success_reports_sequence_mode() {
        let fixture = PlanningFixture::empty_one();
        let geometry_indices = [0];
        let result = call_final_requests(&fixture, &geometry_indices, &[1.0])
            .expect("empty map should support final request sequence planning");

        assert_eq!(result.planning_mode, "rust_registered_sequence");
        assert_eq!(
            result.plan_input_indices,
            (0..geometry_indices.len()).collect::<Vec<_>>()
        );
    }

    #[test]
    fn final_requests_falls_back_to_split_route_runs_when_direct_sequence_fails() {
        let map = ObstacleMap::new(GRID_WIDTH, GRID_HEIGHT);
        let fixture = PlanningFixture::from_map_and_geometries(
            &map,
            vec![registered_geometry(vec![(1.5, 10.5), (15.5, 10.5)], 0)],
        );

        // 4 um in one meander needs A = 4/2 + r(4 - pi) = 2.17 um of depth,
        // more than the 1.6 um box; two 2 um chunks need 1.17 um each.
        let result = call_final_requests_with_split_config(&fixture, &[0], &[4.0], 0.5, 4)
            .expect("split fallback should plan two insertable chunks");

        assert_eq!(result.planning_mode, "rust_registered_split_route_runs");
        assert_eq!(result.plan_input_indices, vec![0, 0]);
        assert_eq!(result.result.selected_candidate_index, Some(0));
        assert_eq!(result.result.candidate_results.len(), 1);
        assert_eq!(result.result.candidate_results[0].plans.len(), 2);
    }
}
