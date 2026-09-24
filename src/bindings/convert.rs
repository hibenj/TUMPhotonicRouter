use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use rustc_hash::FxHashMap;

use crate::primitives::{Primitive, PrimitiveLibrary};
use crate::search::state::{RouteResult, RouteSearchStats, State};

use crate::bindings::*;

pub(crate) fn primitive_kind(p: &Primitive) -> String {
    let d = ((p.end_angle as i16 - p.start_angle as i16).rem_euclid(8)) as i16;
    if d == 0 {
        "straight".into()
    } else if d == 1 || d == 7 {
        "turn45".into()
    } else {
        "turn90".into()
    }
}

pub(crate) fn describe_primitives(
    py: Python<'_>,
    lib: &PrimitiveLibrary,
) -> PyResult<Vec<PyObject>> {
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

pub(crate) fn native_batch_timings_to_py_dict(
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

pub(crate) fn convert_result(
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
        crossing_hotpath_no_contact: r.stats.crossing_hotpath_no_contact,
        crossing_hotpath_contact_checks: r.stats.crossing_hotpath_contact_checks,
        crossing_hotpath_static_rejects: r.stats.crossing_hotpath_static_rejects,
        crossing_hotpath_no_owner_contacts: r.stats.crossing_hotpath_no_owner_contacts,
        crossing_hotpath_single_owner_contacts: r.stats.crossing_hotpath_single_owner_contacts,
        crossing_hotpath_multi_owner_contacts: r.stats.crossing_hotpath_multi_owner_contacts,
        crossing_hotpath_witness_cells_scanned: r.stats.crossing_hotpath_witness_cells_scanned,
        crossing_hotpath_partner_segment_checks: r.stats.crossing_hotpath_partner_segment_checks,
        crossing_hotpath_partner_segment_bbox_rejects: r
            .stats
            .crossing_hotpath_partner_segment_bbox_rejects,
        crossing_hotpath_intersection_hits: r.stats.crossing_hotpath_intersection_hits,
        crossing_hotpath_total_time_us: {
            let clamped = r.stats.crossing_hotpath_total_time_us.min(u64::MAX as u128);
            clamped as u64
        },
        crossing_hotpath_owner_scan_time_us: {
            let clamped = r
                .stats
                .crossing_hotpath_owner_scan_time_us
                .min(u64::MAX as u128);
            clamped as u64
        },
        crossing_hotpath_segment_time_us: {
            let clamped = r
                .stats
                .crossing_hotpath_segment_time_us
                .min(u64::MAX as u128);
            clamped as u64
        },
        crossing_hotpath_reservation_time_us: {
            let clamped = r
                .stats
                .crossing_hotpath_reservation_time_us
                .min(u64::MAX as u128);
            clamped as u64
        },
        crossing_candidate_checks: r.stats.crossing_candidate_checks,
        crossing_accepted: r.stats.crossing_accepted,
        crossing_reject_non_straight: r.stats.crossing_reject_non_straight,
        crossing_reject_not_perpendicular: r.stats.crossing_reject_not_perpendicular,
        crossing_reject_margin: r.stats.crossing_reject_margin,
        crossing_reject_wrong_order: r.stats.crossing_reject_wrong_order,
        crossing_reject_unexpected_owner: r.stats.crossing_reject_unexpected_owner,
        crossing_accepted_planned: r.stats.crossing_accepted_planned,
        crossing_accepted_over_budget: r.stats.crossing_accepted_over_budget,
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

pub(crate) fn vec_to_primitive_counter_array(values: &[usize]) -> [usize; 4] {
    let mut counters = [0usize; 4];
    for (idx, value) in values.iter().copied().take(4).enumerate() {
        counters[idx] = value;
    }
    counters
}

pub(crate) fn to_route_result(route: &PyRouteResult) -> RouteResult {
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
            diagonal_halo_contacts: 0,
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
            crossing_hotpath_no_contact: route.crossing_hotpath_no_contact,
            crossing_hotpath_contact_checks: route.crossing_hotpath_contact_checks,
            crossing_hotpath_static_rejects: route.crossing_hotpath_static_rejects,
            crossing_hotpath_no_owner_contacts: route.crossing_hotpath_no_owner_contacts,
            crossing_hotpath_single_owner_contacts: route.crossing_hotpath_single_owner_contacts,
            crossing_hotpath_multi_owner_contacts: route.crossing_hotpath_multi_owner_contacts,
            crossing_hotpath_witness_cells_scanned: route.crossing_hotpath_witness_cells_scanned,
            crossing_hotpath_partner_segment_checks: route.crossing_hotpath_partner_segment_checks,
            crossing_hotpath_partner_segment_bbox_rejects: route
                .crossing_hotpath_partner_segment_bbox_rejects,
            crossing_hotpath_intersection_hits: route.crossing_hotpath_intersection_hits,
            crossing_hotpath_total_time_us: u128::from(route.crossing_hotpath_total_time_us),
            crossing_hotpath_owner_scan_time_us: u128::from(
                route.crossing_hotpath_owner_scan_time_us,
            ),
            crossing_hotpath_segment_time_us: u128::from(route.crossing_hotpath_segment_time_us),
            crossing_hotpath_reservation_time_us: u128::from(
                route.crossing_hotpath_reservation_time_us,
            ),
            crossing_candidate_checks: route.crossing_candidate_checks,
            crossing_accepted: route.crossing_accepted,
            crossing_reject_non_straight: route.crossing_reject_non_straight,
            crossing_reject_not_perpendicular: route.crossing_reject_not_perpendicular,
            crossing_reject_margin: route.crossing_reject_margin,
            crossing_reject_wrong_order: route.crossing_reject_wrong_order,
            crossing_reject_unexpected_owner: route.crossing_reject_unexpected_owner,
            crossing_accepted_planned: route.crossing_accepted_planned,
            crossing_accepted_over_budget: route.crossing_accepted_over_budget,
            crossing_reject_unmatched_owner: route.crossing_reject_unmatched_owner,
            crossing_reject_unmatched_centerline: route.crossing_reject_unmatched_centerline,
            crossing_reject_unmatched_footprint: route.crossing_reject_unmatched_footprint,
            crossing_reject_unmatched_route_centerline: route
                .crossing_reject_unmatched_route_centerline,
            crossing_reject_unmatched_route_footprint: route
                .crossing_reject_unmatched_route_footprint,
            crossing_reject_pending_straight: route.crossing_reject_pending_straight,
            crossing_reject_reservation_overlap: 0,
            crossing_perpendicular_reject_by_partner: FxHashMap::default(),
            crossing_after_margin_by_partner: FxHashMap::default(),
            crossing_pending_straight_by_partner: FxHashMap::default(),
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
            // A `PyRouteResult` always describes a route that was found, so
            // it can never carry an unsupported-request marker.
            unsupported_request: 0,
        },
    }
}

impl PyPhotonicRouter {
    /// Batch-result serialization for `route_many_with_negotiated_repair_and_commit`:
    /// the Python-level result shape
    /// (`status`/`failed_net_id`/`error`/`routes`/`attempts`/`repair_trace`/
    /// `long_straight_congestion`/`timings_s`). It was shared with the older
    /// repair chain, so that the two engines were interchangeable for direct
    /// A/B comparison, until Milestone 8 of
    /// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`
    /// deleted that chain. See
    /// `.agent/execplans/2026-08-25-negotiated-repair-engine.md` Milestone 5.
    pub(crate) fn build_native_batch_result_dict(
        &mut self,
        py: Python<'_>,
        native_jobs: &[NativeRouteJob],
        batch: &mut RepairBatchState,
        collect_native_timing: bool,
    ) -> PyResult<PyObject> {
        let result_dict = PyDict::new_bound(py);
        let route_entries = PyList::empty_bound(py);
        for job in native_jobs {
            if let Some(route_result) = batch.final_routes.get(&job.net_id) {
                let entry = PyDict::new_bound(py);
                let route_construct_start = native_batch_timer(collect_native_timing);
                let route_obj = Py::new(py, convert_result(py, &self.primitives, route_result)?)?;
                batch.timings.route_result_construction_us +=
                    native_batch_elapsed_us(route_construct_start);
                let dict_start = native_batch_timer(collect_native_timing);
                entry.set_item("net_id", job.net_id)?;
                entry.set_item("route", route_obj)?;
                route_entries.append(entry)?;
                batch.timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
            }
        }
        let attempt_entries = PyList::empty_bound(py);
        for attempt in std::mem::take(&mut batch.attempts) {
            let entry = PyDict::new_bound(py);
            let route_obj = if let Some(route) = attempt.route.as_ref() {
                let route_construct_start = native_batch_timer(collect_native_timing);
                let route_obj = Py::new(py, convert_result(py, &self.primitives, route)?)?;
                batch.timings.route_result_construction_us +=
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
            batch.timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
        }
        let repair_trace_entries = PyList::empty_bound(py);
        for event in std::mem::take(&mut batch.repair_trace) {
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
            batch.timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
        }
        let dict_start = native_batch_timer(collect_native_timing);
        result_dict.set_item(
            "status",
            if batch.failed_net_id.is_some() {
                "failed"
            } else {
                "routed"
            },
        )?;
        result_dict.set_item("failed_net_id", batch.failed_net_id)?;
        result_dict.set_item("error", batch.failed_error.clone())?;
        result_dict.set_item("repair_count", batch.repair_count)?;
        result_dict.set_item("deferred_count", batch.deferred_count)?;
        result_dict.set_item("routes", route_entries)?;
        result_dict.set_item("attempts", attempt_entries)?;
        result_dict.set_item("repair_trace", repair_trace_entries)?;
        result_dict.set_item(
            "long_straight_congestion",
            self.long_straight_congestion_records(py)?,
        )?;
        batch.timings.python_return_dict_us += native_batch_elapsed_us(dict_start);
        result_dict.set_item(
            "timings_s",
            native_batch_timings_to_py_dict(py, &batch.timings)?,
        )?;
        Ok(result_dict.into())
    }
}
