use rustc_hash::FxHashSet;

use crate::astar::{RouteResult, State, TerminalBumpAxis, TerminalBumpGuard};
use crate::config::NetNameTrace;
use crate::geometry_realization::{
    anchored_tilt_scale_candidate as anchored_tilt_scale_candidate_rs,
    centerline_to_port_corrected_centerline_with_options as centerline_to_port_corrected_centerline_with_options_rs,
    distance as distance_rs,
    full_straight_offset_bump_candidates as full_straight_offset_bump_candidates_rs,
    full_straight_offset_bump_candidates_for_centerline as full_straight_offset_bump_candidates_for_centerline_rs,
    generate_waveguide_polygon as generate_waveguide_polygon_rs,
    route_to_port_corrected_centerline_with_options_and_collision_check,
    route_to_primitive_centerline as route_to_primitive_centerline_rs, GeometryError,
    GeometryGridSpec, OffsetBumpCandidate,
};
use crate::obstacle_map::{pack_xy, unpack_xy, CellKey};
use crate::static_obstacle_builder::{rasterize_polygon, sample_polyline_cells, StaticGridSpec};

use crate::engine::*;
#[cfg(test)]
use crate::meander::MeanderSide;
#[cfg(test)]
use pyo3::types::PyDict;

/// Cells a realized centerline occupies: the waveguide polygon's cell-center
/// rasterization unioned with every cell the centerline itself passes through.
///
/// The center-only rasterization alone misses a thin guide realized between
/// cell centers (an endpoint correction that slides a 290 um vertical 2.5 um
/// sideways produced exactly that in `benes_16x16` grid mode, 2026-08-27: the
/// candidate rasterized to zero cells, the collision check saw nothing, and
/// two nets were committed on top of each other).
/// Grid footprint of a realized centerline at two granularities.
///
/// `center` is the waveguide polygon's cell-center rasterization -- the
/// granularity every other static-obstacle decision (openings, primitive
/// footprints) is made at, so it is what static checks compare against.
/// `occupied` additionally contains every cell the centerline itself passes
/// through, so a thin guide realized between cell centers is still visible to
/// dynamic (other-net) checks and gets registered when it is committed.
pub(crate) struct CenterlineCells {
    pub(crate) center: Vec<(i32, i32)>,
    pub(crate) occupied: Vec<(i32, i32)>,
}

impl CenterlineCells {
    pub(crate) fn is_center_cell(&self, cell: (i32, i32)) -> bool {
        self.center.binary_search(&cell).is_ok()
    }
}

pub(crate) fn centerline_core_cells(
    centerline: &[(f64, f64)],
    width_um: f64,
    static_grid: &StaticGridSpec,
) -> Result<CenterlineCells, GeometryError> {
    let polygon = generate_waveguide_polygon_rs(centerline, width_um)?;
    let center_keys = rasterize_polygon(&polygon, static_grid);
    let mut occupied_keys = center_keys.clone();
    occupied_keys.extend(sample_polyline_cells(centerline, static_grid));
    let mut center: Vec<(i32, i32)> = center_keys.into_iter().map(unpack_xy).collect();
    center.sort_unstable();
    let mut occupied: Vec<(i32, i32)> = occupied_keys.into_iter().map(unpack_xy).collect();
    occupied.sort_unstable();
    Ok(CenterlineCells { center, occupied })
}

pub(crate) fn compact_bump_portion(
    centerline: &[(f64, f64)],
    placement_is_start: bool,
) -> &[(f64, f64)] {
    if centerline.len() <= 2 {
        return centerline;
    }
    if placement_is_start {
        &centerline[..centerline.len() - 1]
    } else {
        &centerline[1..]
    }
}

impl PyPhotonicRouter {
    pub(crate) fn terminal_bump_guard_for_target(
        &self,
        target: State,
        target_port_um: Option<(f64, f64)>,
    ) -> Option<TerminalBumpGuard> {
        let target_port_um = target_port_um?;
        let grid_size = self.grid.grid_size_um;
        if !grid_size.is_finite() || grid_size <= 0.0 {
            return None;
        }
        let port_grid_x = (target_port_um.0 - self.grid.origin_x_um) / grid_size;
        let port_grid_y = (target_port_um.1 - self.grid.origin_y_um) / grid_size;
        if !port_grid_x.is_finite() || !port_grid_y.is_finite() {
            return None;
        }

        let target_center_x = f64::from(target.x) + 0.5;
        let target_center_y = f64::from(target.y) + 0.5;
        let required_bump_cells = 4 * self.primitive_cfg.bend_radius_cells.max(0);
        if required_bump_cells <= 0 {
            return None;
        }

        let eps = 1.0e-6;
        match target.angle % 8 {
            // Horizontal approach: only vertical grid snap offset requires a terminal bump.
            0 | 4 if (port_grid_y - target_center_y).abs() > eps => Some(TerminalBumpGuard {
                axis: TerminalBumpAxis::Horizontal,
                target_axis_coord: f64::from(target.y),
                target_along_coord: f64::from(target.x),
                required_bump_cells,
            }),
            // Vertical approach: only horizontal grid snap offset requires a terminal bump.
            2 | 6 if (port_grid_x - target_center_x).abs() > eps => Some(TerminalBumpGuard {
                axis: TerminalBumpAxis::Vertical,
                target_axis_coord: f64::from(target.x),
                target_along_coord: f64::from(target.y),
                required_bump_cells,
            }),
            _ => None,
        }
    }

    pub(crate) fn route_port_corrected_centerline_checked_and_commit_native(
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
        commit_to_router: bool,
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
            // Give any candidate-trying strategy inside this call
            // (currently only try_apply_45_degree_endpoint_delta_correction)
            // the ability to skip a candidate that dynamically collides with
            // another net's already-committed geometry and try its next
            // candidate instead, rather than only being caught by this
            // function's own post-construction check below (which can only
            // reject, not retry) -- confirmed via direct tracing
            // (multiportmmi_16x16's n_196/n_203) that this exact strategy is
            // what previously produced a colliding candidate with no
            // alternative to fall back to. See
            // .agent/execplans/2026-08-19-collision-avoiding-endpoint-correction.md.
            // The post-construction check below is left unchanged and still
            // runs as a defense-in-depth backstop.
            let trace_net =
                self.router_config.diagnostics.trace_endpoint_correction_net == Some(net_id);
            let candidate_collision_check = |candidate: &[(f64, f64)]| -> bool {
                let Ok(candidate_cells) = centerline_core_cells(candidate, width_um, &static_grid)
                else {
                    return false;
                };
                let blockers = self
                    .realized_dynamic_blockers(
                        net_id,
                        candidate,
                        &candidate_cells,
                        &clearance_exempt_keys,
                        width_um,
                        &static_grid,
                    )
                    .blockers;
                if trace_net {
                    let owners =
                        sorted_other_owners_for_cells(&self.obstacle_map, &blockers, net_id);
                    eprintln!(
                        "endpoint_correction_trace net={net_id} candidate_points={} core_cells={} dynamic_blockers={} owners={owners:?} blocker_bbox={} candidate_bbox=({:.3},{:.3})-({:.3},{:.3})",
                        candidate.len(),
                        candidate_cells.occupied.len(),
                        blockers.len(),
                        format_bbox(&blockers),
                        candidate.iter().map(|p| p.0).fold(f64::INFINITY, f64::min),
                        candidate.iter().map(|p| p.1).fold(f64::INFINITY, f64::min),
                        candidate.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max),
                        candidate.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max),
                    );
                }
                blockers.is_empty()
            };
            let centerline = route_to_port_corrected_centerline_with_options_and_collision_check(
                route,
                &self.primitives,
                &grid,
                source_port_um,
                target_port_um,
                allow_unchecked_fallback,
                Some(&candidate_collision_check),
            )
            .map_err(|err| err.to_string())?;
            let corrected_cells = centerline_core_cells(&centerline, width_um, &static_grid)
                .map_err(|err| err.to_string())?;
            let corrected_core_cells = corrected_cells.occupied.clone();
            if corrected_core_cells.is_empty() {
                return Ok(NativeEndpointCorrection {
                    centerline,
                    committed_bump: false,
                    candidate_index: None,
                    candidate_label: None,
                });
            }
            let old_core_cells =
                match route_to_primitive_centerline_rs(route, &self.primitives, &grid) {
                    Ok(old_centerline) => {
                        centerline_core_cells(&old_centerline, width_um, &static_grid)
                            .map(|cells| cells.occupied)
                            .unwrap_or_else(|_| {
                                route_core_cells(&route.cells, core_radius_cells, width, height)
                            })
                    }
                    Err(_) => route_core_cells(&route.cells, core_radius_cells, width, height),
                };
            let old_core_keys = pack_cells(&old_core_cells);
            let out_of_bounds: Vec<(i32, i32)> = corrected_core_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let static_blockers: Vec<(i32, i32)> = corrected_cells
                .center
                .iter()
                .copied()
                .filter(|&(x, y)| {
                    let key = pack_xy(x, y);
                    self.obstacle_map.in_bounds(x, y)
                        && self.obstacle_map.is_static_blocked(x, y)
                        && !opened_keys.contains(&key)
                        && !old_core_keys.contains(&key)
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
            // This branch (no full-straight-offset-bump candidates, i.e. a
            // diagonal or already-bent baseline) is only ever reached for a
            // net the caller has already excluded from crossing handling
            // (`_apply_checked_endpoint_corrections_for_net_ids`/
            // `_apply_checked_fanout_stub_endpoint_corrections_for_net_ids`
            // both skip any net_id in `crossing_net_ids` before calling
            // here), so there is no expected-partner concept to allow for:
            // any other net occupying this candidate's own corrected core
            // cells is a genuine, un-anticipated collision, not a legitimate
            // crossing partner. `commit_route_with_clearance_and_allowed_core_overlap_cells`
            // exists precisely to allow overlap with an independently-known
            // partner set (see its own doc comment); previously this call
            // site built that allow-list by scanning the candidate's own
            // cells for whoever already occupied them, which self-authorizes
            // exactly the collision the check exists to catch (found via
            // multiportmmi_16x16's n_196/n_197 cross_net_waveguide_overlap,
            // .agent/execplans/2026-08-19-restructure-port-endpoint-correction.md
            // Milestone 0.5). Use the strict variant instead, matching the
            // case-4 bump candidate loop below, which never allows overlap.
            let corrected_blocked_cells =
                inflate_route_cells(&corrected_core_cells, core_radius_cells, width, height);
            let realized_blockers = self.realized_dynamic_blockers(
                net_id,
                &centerline,
                &corrected_cells,
                &clearance_exempt_keys,
                width_um,
                &static_grid,
            );
            let commit_dynamic_blockers = realized_blockers.blockers.clone();
            let commit_ok = commit_dynamic_blockers.is_empty()
                && if commit_to_router {
                    self.obstacle_map
                        .commit_route_with_clearance_and_allowed_core_overlap_cells(
                            net_id,
                            &corrected_core_cells,
                            &corrected_blocked_cells,
                            clearance_exempt_cells,
                            &realized_blockers.shared_clear_nets,
                            realized_blockers.allowed_overlap_cells(),
                        )
                } else {
                    self.obstacle_map
                        .can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                            net_id,
                            &corrected_core_cells,
                            &corrected_blocked_cells,
                            clearance_exempt_cells,
                            &realized_blockers.shared_clear_nets,
                            realized_blockers.allowed_overlap_cells(),
                        )
                };
            if !commit_ok {
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
            if commit_to_router {
                self.remove_crossing_events_for_net(net_id);
                self.register_geometric_crossing_events_for_route(
                    net_id,
                    route,
                    source_port_um,
                    target_port_um,
                );
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
                self.remember_corrected_realized_centerline(net_id, &centerline);
                self.remember_committed_route_opened_cells(net_id, Some(&opened_keys));
                self.add_post_commit_guidance_for_route(net_id, route);
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
            }
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
        let endpoint_bump_trace_net_id = net_id.to_string();
        let trace_endpoint_bumps = match &self.router_config.diagnostics.trace_endpoint_bump_nets {
            NetNameTrace::All => true,
            NetNameTrace::Names(names) => {
                names.iter().any(|name| *name == endpoint_bump_trace_net_id)
            }
            NetNameTrace::None => false,
        };

        for (candidate_index, candidate) in candidates.into_iter().enumerate() {
            let candidate_label = candidate.label;
            let centerline = candidate.centerline;
            let bump_centerline = compact_bump_portion(&centerline, candidate.placement_is_start);
            let candidate_cells = centerline_core_cells(bump_centerline, width_um, &static_grid)
                .map_err(|err| err.to_string())?;
            let candidate_core_cells = candidate_cells.occupied.clone();
            if candidate_core_cells.is_empty() {
                let detail = format!("#{candidate_index} {candidate_label}: empty core footprint");
                if trace_endpoint_bumps {
                    println!(
                        "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=reject {detail}"
                    );
                }
                rejection_details.push(detail);
                continue;
            }
            let candidate_blocked_cells =
                inflate_route_cells(&candidate_core_cells, core_radius_cells, width, height);
            // A case-4 bump candidate is accepted or rejected by its actual
            // waveguide core footprint. The inflated cells are only the
            // reservation committed after the core is known to be legal; using
            // them as hard blockers rejects visually legal local bumps.
            let out_of_bounds: Vec<(i32, i32)> = candidate_core_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let static_blockers: Vec<(i32, i32)> = candidate_cells
                .center
                .iter()
                .copied()
                .filter(|&(x, y)| {
                    let key = pack_xy(x, y);
                    self.obstacle_map.in_bounds(x, y)
                        && self.obstacle_map.is_static_blocked(x, y)
                        && !opened_keys.contains(&key)
                })
                .collect();
            let realized_blockers = self.realized_dynamic_blockers(
                net_id,
                bump_centerline,
                &candidate_cells,
                &clearance_exempt_keys,
                width_um,
                &static_grid,
            );
            let dynamic_blockers = realized_blockers.blockers.clone();
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
                let detail = format!(
                    "#{candidate_index} {candidate_label}: {} core_cells={} core_bbox={} blocked_cells={} blocked_bbox={}",
                    reasons.join(", "),
                    candidate_core_cells.len(),
                    format_bbox(&candidate_core_cells),
                    candidate_blocked_cells.len(),
                    format_bbox(&candidate_blocked_cells)
                );
                if trace_endpoint_bumps {
                    println!(
                        "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=reject {detail}"
                    );
                }
                rejection_details.push(detail);
                continue;
            }

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

            let commit_ok = if commit_to_router {
                self.obstacle_map
                    .commit_route_with_clearance_and_allowed_core_overlap_cells(
                        net_id,
                        &merged_core_cells,
                        &merged_blocked_cells,
                        &commit_clearance_exempt_cell_vec,
                        &realized_blockers.shared_clear_nets,
                        realized_blockers.allowed_overlap_cells(),
                    )
            } else {
                self.obstacle_map
                    .can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                        net_id,
                        &merged_core_cells,
                        &merged_blocked_cells,
                        &commit_clearance_exempt_cell_vec,
                        &realized_blockers.shared_clear_nets,
                        realized_blockers.allowed_overlap_cells(),
                    )
            };
            if commit_ok {
                if trace_endpoint_bumps {
                    println!(
                        "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=accept core_cells={} core_bbox={} blocked_cells={} blocked_bbox={}",
                        candidate_core_cells.len(),
                        format_bbox(&candidate_core_cells),
                        candidate_blocked_cells.len(),
                        format_bbox(&candidate_blocked_cells)
                    );
                }
                if commit_to_router {
                    self.remove_crossing_events_for_net(net_id);
                    self.register_geometric_crossing_events_for_route(
                        net_id,
                        route,
                        source_port_um,
                        target_port_um,
                    );
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
                    self.remember_corrected_realized_centerline(net_id, &centerline);
                    self.remember_committed_route_opened_cells(net_id, Some(&opened_keys));
                    self.add_post_commit_guidance_for_route(net_id, route);
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
                }
                return Ok(NativeEndpointCorrection {
                    centerline,
                    committed_bump: commit_to_router,
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
            let detail = format!(
                "#{candidate_index} {candidate_label}: commit_rejected dynamic_overlap={} owners={commit_owners:?} dynamic_bbox={} dynamic_sample={} out_of_bounds={} out_of_bounds_bbox={} core_cells={} core_bbox={}",
                commit_dynamic_blockers.len(),
                format_bbox(&commit_dynamic_blockers),
                format_cell_sample(&commit_dynamic_blockers, 8),
                commit_out_of_bounds.len(),
                format_bbox(&commit_out_of_bounds),
                candidate_core_cells.len(),
                format_bbox(&candidate_core_cells)
            );
            if trace_endpoint_bumps {
                println!(
                    "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=reject {detail}"
                );
            }
            rejection_details.push(detail);
        }

        Err(format!(
            "No collision-free port endpoint case-4 bump placement found; candidates: {}",
            rejection_details.join("; ")
        ))
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_port_corrected_centerline_checked_native(
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
        self.route_port_corrected_centerline_checked_and_commit_native(
            net_id,
            route,
            width_um,
            core_radius_cells,
            opened_cells,
            clearance_exempt_cells,
            source_port_um,
            target_port_um,
            allow_unchecked_fallback,
            false,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn centerline_port_corrected_checked_native(
        &mut self,
        net_id: u64,
        centerline: &[(f64, f64)],
        width_um: f64,
        core_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<NativeEndpointCorrection, String> {
        let static_grid = static_grid_from_py_grid(&self.grid);
        let width = self.grid.width as i32;
        let height = self.grid.height as i32;
        let opened_keys = pack_cells(opened_cells);
        let clearance_exempt_keys = pack_cells(clearance_exempt_cells);
        let endpoint_bump_trace_net_id = net_id.to_string();
        let trace_endpoint_bumps = match &self.router_config.diagnostics.trace_endpoint_bump_nets {
            NetNameTrace::All => true,
            NetNameTrace::Names(names) => {
                names.iter().any(|name| *name == endpoint_bump_trace_net_id)
            }
            NetNameTrace::None => false,
        };

        if let Ok(corrected_centerline) = centerline_to_port_corrected_centerline_with_options_rs(
            centerline,
            &self.primitives,
            source_port_um,
            target_port_um,
            true,
        ) {
            let corrected_cells =
                centerline_core_cells(&corrected_centerline, width_um, &static_grid)
                    .map_err(|err| err.to_string())?;
            let corrected_core_cells = corrected_cells.occupied.clone();
            let corrected_blocked_cells =
                inflate_route_cells(&corrected_core_cells, core_radius_cells, width, height);
            let old_core_cells = centerline_core_cells(centerline, width_um, &static_grid)
                .map(|cells| cells.occupied)
                .unwrap_or_default();
            let old_core_keys = pack_cells(&old_core_cells);
            let correction_clearance_exempt_cells = unique_cells(
                clearance_exempt_cells
                    .iter()
                    .copied()
                    .chain(old_core_cells.iter().copied()),
            );
            let correction_clearance_exempt_keys = pack_cells(&correction_clearance_exempt_cells);
            let out_of_bounds: Vec<(i32, i32)> = corrected_core_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let static_blockers: Vec<(i32, i32)> = corrected_cells
                .center
                .iter()
                .copied()
                .filter(|&(x, y)| {
                    let key = pack_xy(x, y);
                    self.obstacle_map.in_bounds(x, y)
                        && self.obstacle_map.is_static_blocked(x, y)
                        && !opened_keys.contains(&key)
                        && !old_core_keys.contains(&key)
                })
                .collect();
            let realized_blockers = self.realized_dynamic_blockers(
                net_id,
                &corrected_centerline,
                &corrected_cells,
                &correction_clearance_exempt_keys,
                width_um,
                &static_grid,
            );
            let dynamic_blockers = realized_blockers.blockers.clone();
            if out_of_bounds.is_empty() && static_blockers.is_empty() && dynamic_blockers.is_empty()
            {
                if self
                    .obstacle_map
                    .can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                        net_id,
                        &corrected_core_cells,
                        &corrected_blocked_cells,
                        &correction_clearance_exempt_cells,
                        &realized_blockers.shared_clear_nets,
                        realized_blockers.allowed_overlap_cells(),
                    )
                {
                    if trace_endpoint_bumps {
                        println!(
                            "endpoint_bump_trace net_id={net_id} candidate=normal_segment label=normal_segment status=accept core_cells={} core_bbox={} blocked_cells={} blocked_bbox={}",
                            corrected_core_cells.len(),
                            format_bbox(&corrected_core_cells),
                            corrected_blocked_cells.len(),
                            format_bbox(&corrected_blocked_cells)
                        );
                    }
                    return Ok(NativeEndpointCorrection {
                        centerline: corrected_centerline,
                        committed_bump: false,
                        candidate_index: None,
                        candidate_label: Some("normal_segment".to_string()),
                    });
                }
            } else if trace_endpoint_bumps {
                println!(
                    "endpoint_bump_trace net_id={net_id} candidate=normal_segment label=normal_segment status=reject out_of_bounds={} static_overlap={} dynamic_overlap={} core_cells={} core_bbox={}",
                    out_of_bounds.len(),
                    static_blockers.len(),
                    dynamic_blockers.len(),
                    corrected_core_cells.len(),
                    format_bbox(&corrected_core_cells)
                );
            }
        }

        let mut candidates = full_straight_offset_bump_candidates_for_centerline_rs(
            centerline,
            &self.primitives,
            source_port_um,
            target_port_um,
        )
        .map_err(|err| err.to_string())?;

        if candidates.is_empty() {
            // No offset bump fits (for example a terminal axis-aligned run
            // far shorter than 4*bend_radius). Last resort: a small rigid
            // rotate+scale of the whole segment about whichever end is the
            // fixed anchor, landing the moving end exactly on the true port.
            // It still runs through the same collision/commit checks below.
            let anchor_side = match (source_port_um, target_port_um) {
                (Some(source), Some(target)) if centerline.len() >= 2 => {
                    if distance_rs(source, centerline[0]) <= 1.0e-6 {
                        Some((true, target))
                    } else if distance_rs(target, centerline[centerline.len() - 1]) <= 1.0e-6 {
                        Some((false, source))
                    } else {
                        None
                    }
                }
                _ => None,
            };
            let extra = anchor_side.map(|(anchor_at_start, moving_target)| {
                (
                    anchor_at_start,
                    anchored_tilt_scale_candidate_rs(centerline, anchor_at_start, moving_target),
                )
            });
            match extra {
                Some((anchor_at_start, Some(mapped_centerline))) => {
                    candidates.push(OffsetBumpCandidate {
                        label: "anchored_tilt_scale".to_string(),
                        centerline: mapped_centerline,
                        // `compact_bump_portion` drops the port-side endpoint
                        // from the collision footprint: the moving (port) end
                        // is the last point when the anchor is at the start,
                        // the first point otherwise.
                        placement_is_start: anchor_at_start,
                    })
                }
                _ => {
                    return Err(
                        "No port endpoint correction candidates found for centerline segment"
                            .to_string(),
                    );
                }
            }
        }

        // Mirror the full-route corrector's old-core exemption: cells the
        // original (uncorrected) segment already occupies are not new static
        // conflicts a candidate introduces. Without this, an
        // anchored_tilt_scale candidate is rejected for the anchor cell the
        // route itself sits on (benes_16x16 n_s0_6_o0_to_s1_3_i0).
        let old_segment_core_keys: FxHashSet<CellKey> =
            centerline_core_cells(centerline, width_um, &static_grid)
                .map(|cells| pack_cells(&cells.occupied))
                .unwrap_or_default();

        let mut rejection_details = Vec::new();
        let mut first_accepted: Option<NativeEndpointCorrection> = None;
        for (candidate_index, candidate) in candidates.into_iter().enumerate() {
            let candidate_label = candidate.label;
            let candidate_centerline = candidate.centerline;
            let bump_centerline =
                compact_bump_portion(&candidate_centerline, candidate.placement_is_start);
            let candidate_cells = centerline_core_cells(bump_centerline, width_um, &static_grid)
                .map_err(|err| err.to_string())?;
            let candidate_core_cells = candidate_cells.occupied.clone();
            if candidate_core_cells.is_empty() {
                let detail = format!("#{candidate_index} {candidate_label}: empty core footprint");
                if trace_endpoint_bumps {
                    println!(
                        "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=reject {detail}"
                    );
                }
                rejection_details.push(detail);
                continue;
            }
            let candidate_blocked_cells =
                inflate_route_cells(&candidate_core_cells, core_radius_cells, width, height);
            let out_of_bounds: Vec<(i32, i32)> = candidate_core_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let static_blockers: Vec<(i32, i32)> = candidate_cells
                .center
                .iter()
                .copied()
                .filter(|&(x, y)| {
                    let key = pack_xy(x, y);
                    self.obstacle_map.in_bounds(x, y)
                        && self.obstacle_map.is_static_blocked(x, y)
                        && !opened_keys.contains(&key)
                        && !old_segment_core_keys.contains(&key)
                })
                .collect();
            let realized_blockers = self.realized_dynamic_blockers(
                net_id,
                bump_centerline,
                &candidate_cells,
                &clearance_exempt_keys,
                width_um,
                &static_grid,
            );
            let dynamic_blockers = realized_blockers.blockers.clone();
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
                let detail = format!(
                    "#{candidate_index} {candidate_label}: {} core_cells={} core_bbox={} blocked_cells={} blocked_bbox={}",
                    reasons.join(", "),
                    candidate_core_cells.len(),
                    format_bbox(&candidate_core_cells),
                    candidate_blocked_cells.len(),
                    format_bbox(&candidate_blocked_cells)
                );
                if trace_endpoint_bumps {
                    println!(
                        "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=reject {detail}"
                    );
                }
                rejection_details.push(detail);
                continue;
            }

            let commit_ok = self
                .obstacle_map
                .can_commit_route_with_clearance_and_allowed_core_overlap_cells(
                    net_id,
                    &candidate_core_cells,
                    &candidate_blocked_cells,
                    clearance_exempt_cells,
                    &realized_blockers.shared_clear_nets,
                    realized_blockers.allowed_overlap_cells(),
                );
            if commit_ok {
                if trace_endpoint_bumps {
                    println!(
                        "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=accept core_cells={} core_bbox={} blocked_cells={} blocked_bbox={}",
                        candidate_core_cells.len(),
                        format_bbox(&candidate_core_cells),
                        candidate_blocked_cells.len(),
                        format_bbox(&candidate_blocked_cells)
                    );
                }
                if first_accepted.is_none() {
                    first_accepted = Some(NativeEndpointCorrection {
                        centerline: candidate_centerline,
                        committed_bump: false,
                        candidate_index: Some(candidate_index),
                        candidate_label: Some(candidate_label),
                    });
                }
                continue;
            }

            let commit_dynamic_blockers = cells_with_other_dynamic_owner(
                &self.obstacle_map,
                &candidate_core_cells,
                &clearance_exempt_keys,
                net_id,
            );
            let commit_out_of_bounds: Vec<(i32, i32)> = candidate_blocked_cells
                .iter()
                .copied()
                .filter(|&(x, y)| !self.obstacle_map.in_bounds(x, y))
                .collect();
            let commit_owners =
                sorted_other_owners_for_cells(&self.obstacle_map, &commit_dynamic_blockers, net_id);
            let detail = format!(
                "#{candidate_index} {candidate_label}: commit_rejected dynamic_overlap={} owners={commit_owners:?} dynamic_bbox={} dynamic_sample={} out_of_bounds={} out_of_bounds_bbox={} core_cells={} core_bbox={}",
                commit_dynamic_blockers.len(),
                format_bbox(&commit_dynamic_blockers),
                format_cell_sample(&commit_dynamic_blockers, 8),
                commit_out_of_bounds.len(),
                format_bbox(&commit_out_of_bounds),
                candidate_core_cells.len(),
                format_bbox(&candidate_core_cells)
            );
            if trace_endpoint_bumps {
                println!(
                    "endpoint_bump_trace net_id={net_id} candidate={candidate_index} label={candidate_label} status=reject {detail}"
                );
            }
            rejection_details.push(detail);
        }

        if let Some(correction) = first_accepted {
            return Ok(correction);
        }

        Err(format!(
            "No collision-free port endpoint case-4 bump placement found for centerline segment; candidates: {}",
            rejection_details.join("; ")
        ))
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_port_corrected_centerline_checked_and_commit_compat_native(
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
        self.route_port_corrected_centerline_checked_and_commit_native(
            net_id,
            route,
            width_um,
            core_radius_cells,
            opened_cells,
            clearance_exempt_cells,
            source_port_um,
            target_port_um,
            allow_unchecked_fallback,
            true,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            None,
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
            None,
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            ..PyRouteResult::default()
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
            None,
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
            None,
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
            None,
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
