use std::time::Instant;

use crate::search::state::RouteResult;

use crate::engine::*;

pub(crate) fn cells_bbox(cells: &[(i32, i32)]) -> Option<(i32, i32, i32, i32)> {
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

pub(crate) fn format_bbox(cells: &[(i32, i32)]) -> String {
    cells_bbox(cells)
        .map(|(min_x, max_x, min_y, max_y)| format!("({min_x},{max_x},{min_y},{max_y})"))
        .unwrap_or_else(|| "none".to_string())
}

pub(crate) fn format_cell_sample(cells: &[(i32, i32)], limit: usize) -> String {
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

#[allow(clippy::too_many_arguments)]
pub(crate) fn push_native_repair_trace(
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

/// Formats the `t=<seconds since batch start, one decimal> ` field led by
/// every `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG` trace line emitted by
/// `route_many_with_negotiated_repair_and_commit` and its helpers --
/// `""` when `start` is `None` (outside that loop, as it was for a trace
/// line shared with the older repair chain, deleted in Milestone 8 of
/// `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`), so
/// a caller can always write `eprintln!("{}native_repair_...", trace_t(start))`
/// without a branch. See
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
/// Milestone 5.
pub(crate) fn trace_t(start: Option<Instant>) -> String {
    match start {
        Some(start) => format!("t={:.1} ", start.elapsed().as_secs_f64()),
        None => String::new(),
    }
}

/// Formats a `native_negotiated_search` trace line's `budget=` field:
/// the expansion cap as a plain integer, or `full` for `None` (the
/// benchmark's own `max_iterations`, unbounded by this milestone's fail-fast
/// budget). See Milestone 5 of
/// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`.
pub(crate) fn trace_budget_str(budget: Option<u64>) -> String {
    match budget {
        Some(budget) => budget.to_string(),
        None => "full".to_string(),
    }
}

pub(crate) fn native_batch_seconds(us: u128) -> f64 {
    us as f64 / 1_000_000.0
}

pub(crate) fn native_batch_timer(enabled: bool) -> Option<Instant> {
    enabled.then(Instant::now)
}

pub(crate) fn native_batch_elapsed_us(start: Option<Instant>) -> u128 {
    start.map_or(0, |start| start.elapsed().as_micros())
}

impl PyPhotonicRouter {
    /// Diagnostic guard (owner request 2026-09-01): a route the crossing-aware
    /// search accepted as legal must never fail realized-crossing validation --
    /// when it does, the search's grid-polyline partner model and the realized
    /// centerlines disagree. This dump classifies every violation point for the
    /// three-way question "search wrong / validation wrong / geometry destroyed
    /// along the way": `grid_hit` reports whether the two GRID polylines also
    /// intersect near the point (search-side miss), the `route`/`partner`
    /// blocks report each side's grid-vs-realized local distance and segment
    /// angle at the point (who moved between search and realization), and the
    /// realized angles expose near-perpendicular cases (validator strictness).
    /// Enable with PHOTONIC_ROUTER_CROSSING_MISMATCH_DUMP (optionally
    /// =<net_id> to filter).
    pub(crate) fn dump_crossing_mismatch(
        &self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        violations: &[InvalidCrossingIntersection],
    ) {
        if !self.router_config.diagnostics.crossing_mismatch_dump {
            return;
        }
        if violations.is_empty() {
            return;
        }
        if let Some(requested) = self.router_config.diagnostics.crossing_mismatch_dump_net {
            if requested != net_id {
                return;
            }
        }
        let route_grid_centerline = self.grid_waypoints_to_centerline(&route.compressed_waypoints);
        let route_realized = self
            .routing_centerline_for_route(route, source_port_um, target_port_um)
            .unwrap_or_default();
        let radius_um = 12.0 * self.grid.grid_size_um;
        for violation in violations {
            let vp = violation.point;
            let partner_grid_centerline = self
                .committed_center_routes
                .get(&violation.partner_net_id)
                .map(|waypoints| self.grid_waypoints_to_centerline(waypoints))
                .unwrap_or_default();
            let partner_realized = self
                .committed_realized_center_routes
                .get(&violation.partner_net_id)
                .cloned()
                .unwrap_or_default();
            let grid_hit = nearest_polyline_intersection_distance(
                &route_grid_centerline,
                &partner_grid_centerline,
                vp,
                radius_um,
            );
            let (route_grid_dist, route_grid_angle) =
                nearest_segment_info(&route_grid_centerline, vp);
            let (route_real_dist, route_real_angle) = nearest_segment_info(&route_realized, vp);
            let (partner_grid_dist, partner_grid_angle) =
                nearest_segment_info(&partner_grid_centerline, vp);
            let (partner_real_dist, partner_real_angle) =
                nearest_segment_info(&partner_realized, vp);
            eprintln!(
                "crossing-mismatch net={} partner={} reason={} point=({:.3},{:.3}) \
                 grid_intersection_within_{:.1}um={} \
                 route[grid_dist={:.3} grid_angle={:.1} real_dist={:.3} real_angle={:.1}] \
                 partner[grid_dist={:.3} grid_angle={:.1} real_dist={:.3} real_angle={:.1}]",
                net_id,
                violation.partner_net_id,
                violation.reason,
                vp.0,
                vp.1,
                radius_um,
                grid_hit.map_or_else(|| "NO".to_string(), |d| format!("{d:.3}")),
                route_grid_dist,
                route_grid_angle,
                route_real_dist,
                route_real_angle,
                partner_grid_dist,
                partner_grid_angle,
                partner_real_dist,
                partner_real_angle,
            );
        }
    }

    pub(crate) fn trace_committed_partner_centerline_compare(&self, net_id: u64, partner_id: u64) {
        let Some(requested_partner_id) = self.router_config.diagnostics.trace_partner_net else {
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trace_t_is_empty_for_none_and_a_seconds_field_for_some() {
        assert_eq!(trace_t(None), "");
        let start = Instant::now() - std::time::Duration::from_millis(12_300);
        let formatted = trace_t(Some(start));
        assert!(
            formatted.starts_with("t=12.") && formatted.ends_with(' '),
            "expected a leading \"t=12.<something> \" field, got {formatted:?}"
        );
    }

    #[test]
    fn trace_budget_str_formats_none_as_full_and_some_as_the_number() {
        assert_eq!(trace_budget_str(None), "full");
        assert_eq!(trace_budget_str(Some(2_000_000)), "2000000");
    }
}
