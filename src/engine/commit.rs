use rustc_hash::FxHashSet;

use crate::geometry_realization::{
    compress_grid_waypoints as compress_grid_waypoints_rs,
    route_to_grid_path as route_to_grid_path_rs,
};
use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::search::state::RouteResult;
use crate::static_obstacle_builder::{grid_cell_center, StaticGridSpec};

#[cfg(test)]
use crate::crossings::CrossingConfig;
use crate::engine::*;
#[cfg(test)]
use crate::search::state::RouteSearchStats;
#[cfg(test)]
use crate::search::state::State;

/// Cells of a candidate that other nets own, split by whether the other net's
/// *realized* geometry actually comes within one waveguide width of the
/// candidate there.
///
/// Cell ownership is a routing-time approximation: a primitive's footprint
/// claims whole cells, and at a bend it claims cells the realized Euler bend
/// never touches, while a neighbouring net's bend legitimately sweeps through
/// them. Two adjacent Benes switch stubs interleave exactly like that at the
/// switch, so judging an endpoint-correction candidate by ownership alone
/// rejected the physically clean candidate (its bend ran over a cell the
/// neighbour's footprint claimed) and then accepted a physically colliding one
/// (a thin guide realized between cell centers, which the cell-center
/// rasterization could not see at all) -- `benes_16x16` grid mode, nets 132/134,
/// 2026-08-27. `blockers` are the cells where the geometry really collides;
/// `shared_clear_cells` / `shared_clear_nets` are the cells (and their owners)
/// that are shared on the grid but clear in geometry, which a commit may allow
/// as core overlap.
pub(crate) struct RealizedDynamicBlockers {
    pub(crate) blockers: Vec<(i32, i32)>,
    pub(crate) shared_clear_cells: FxHashSet<CellKey>,
    pub(crate) shared_clear_nets: FxHashSet<u64>,
}

impl RealizedDynamicBlockers {
    pub(crate) fn allowed_overlap_cells(&self) -> Option<&FxHashSet<CellKey>> {
        (!self.shared_clear_cells.is_empty()).then_some(&self.shared_clear_cells)
    }
}

pub(crate) fn remove_success_static_cleanup(obstacle_map: &mut ObstacleMap, job: &NativeRouteJob) {
    if job.static_cleanup_cell_keys.is_empty() {
        return;
    }
    obstacle_map.remove_static_keys(&job.static_cleanup_cell_keys);
}

pub(crate) fn dynamic_commit_error_overlap_owner_ids(error: &str) -> Vec<u64> {
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

impl PyPhotonicRouter {
    pub(crate) fn add_crossing_spacing_history_for_route(
        &mut self,
        net_id: u64,
        route: &RouteResult,
    ) {
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

    pub(crate) fn add_long_straight_congestion_for_route(
        &mut self,
        net_id: u64,
        route: &RouteResult,
    ) {
        let grid_size_um = self.grid.grid_size_um;
        if !grid_size_um.is_finite() || grid_size_um <= 0.0 {
            return;
        }
        let width = self.grid.width as i32;
        let height = self.grid.height as i32;
        for segment in route.compressed_waypoints.windows(2) {
            let start = segment[0];
            let end = segment[1];
            let dx = end.0 - start.0;
            let dy = end.1 - start.1;
            let step_x = dx.signum();
            let step_y = dy.signum();
            if step_x == 0 && step_y == 0 {
                continue;
            }
            let abs_dx = dx.abs();
            let abs_dy = dy.abs();
            let is_axis_aligned = abs_dx == 0 || abs_dy == 0;
            let is_diagonal = abs_dx == abs_dy;
            if !is_axis_aligned && !is_diagonal {
                continue;
            }
            let steps = abs_dx.max(abs_dy);
            if steps <= 0 {
                continue;
            }
            let step_length_um = if is_diagonal {
                grid_size_um * 2.0_f64.sqrt()
            } else {
                grid_size_um
            };
            let length_um = f64::from(steps) * step_length_um;
            if length_um < LONG_STRAIGHT_CONGESTION_MIN_UM {
                continue;
            }

            let middle_start = (f64::from(steps) * 0.25).ceil() as i32;
            let middle_end = (f64::from(steps) * 0.75).floor() as i32;
            if middle_start > middle_end {
                continue;
            }

            let (perp_x, perp_y) = if step_x == 0 {
                (1, 0)
            } else if step_y == 0 {
                (0, 1)
            } else {
                (-step_y, step_x)
            };

            let mut cells: FxHashSet<(i32, i32)> = FxHashSet::default();
            for t in middle_start..=middle_end {
                let center_x = start.0 + step_x * t;
                let center_y = start.1 + step_y * t;
                for offset in -LONG_STRAIGHT_CONGESTION_LATERAL_RADIUS_CELLS
                    ..=LONG_STRAIGHT_CONGESTION_LATERAL_RADIUS_CELLS
                {
                    let x = center_x + perp_x * offset;
                    let y = center_y + perp_y * offset;
                    if x >= 0 && x < width && y >= 0 && y < height {
                        cells.insert((x, y));
                    }
                }
            }
            let marked_cells = cells.len();
            if marked_cells == 0 {
                continue;
            }
            for (x, y) in cells {
                self.obstacle_map
                    .add_congestion_cost(x, y, LONG_STRAIGHT_CONGESTION_AMOUNT);
                let entry = self
                    .long_straight_congestion_cells
                    .entry(pack_xy(x, y))
                    .or_insert(0);
                *entry = entry.saturating_add(LONG_STRAIGHT_CONGESTION_AMOUNT);
            }
            self.long_straight_congestion_records
                .push(LongStraightCongestionRecord {
                    net_id,
                    start,
                    end,
                    length_um,
                    marked_cells,
                });
        }
    }

    pub(crate) fn add_post_commit_guidance_for_route(&mut self, net_id: u64, route: &RouteResult) {
        self.add_crossing_spacing_history_for_route(net_id, route);
        self.add_long_straight_congestion_for_route(net_id, route);
        self.add_repair_history_for_route(route);
    }

    /// Systematic negotiated-congestion history cost: every successfully
    /// committed route penalizes the cells it used, unconditionally,
    /// matching LiDAR's `registar_net`/`updateHistoryMap`
    /// (`drgridroute.py:443`/`703`). Replaces the previous mechanism of
    /// penalizing a losing net's route right before ripping it up (3 narrow
    /// call sites, only reachable mid-repair) -- that penalized *losers*,
    /// discouraging an immediate identical retry; this penalizes *every*
    /// user of a cell, discouraging future congestion on an already-popular
    /// corridor regardless of who used it, which is the actual PathFinder-
    /// family negotiated-congestion signal. `commit_history_increment` is 0
    /// (a no-op) outside `route_many_with_repair_and_commit`, so a plain
    /// commit made via `route_many_normal_and_commit` or a single-net
    /// diagnostic call never accumulates history. See
    /// `.agent/execplans/2026-08-25-negotiated-repair-engine.md` Milestone 4.
    pub(crate) fn add_repair_history_for_route(&mut self, route: &RouteResult) {
        if self.commit_history_increment == 0 {
            return;
        }
        self.add_history_for_native_route(
            route,
            self.commit_history_block_radius_cells,
            self.commit_history_increment,
        );
    }

    pub(crate) fn route_obstacle_center_cells(&self, route: &RouteResult) -> Vec<(i32, i32)> {
        route_to_grid_path_rs(route, &self.primitives).unwrap_or_else(|_| route.cells.clone())
    }

    pub(crate) fn route_dynamic_center_cells(
        &self,
        route: &RouteResult,
        _source_port_um: Option<(f64, f64)>,
        _target_port_um: Option<(f64, f64)>,
    ) -> Vec<(i32, i32)> {
        self.route_obstacle_center_cells(route)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_commit_and_core_cells(
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

    pub(crate) fn dynamic_commit_rejection_error(
        &self,
        net_id: u64,
        core_cells: &[(i32, i32)],
        clearance_exempt_cells: &[(i32, i32)],
    ) -> String {
        let clearance_exempt_keys = pack_cells(clearance_exempt_cells);
        let dynamic_blockers = cells_with_other_dynamic_owner(
            &self.obstacle_map,
            core_cells,
            &clearance_exempt_keys,
            net_id,
        );
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

    pub(crate) fn commit_native_route_with_clearance(
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
    pub(crate) fn commit_native_route_with_clearance_without_crossing_validation(
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
    pub(crate) fn commit_native_route_with_clearance_allowing_core_overlap(
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
            return Ok(
                self.commit_native_route_with_clearance_without_crossing_validation(
                    net_id,
                    route,
                    block_radius_cells,
                    commit_radius_cells,
                    clearance_exempt_cells,
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                    opened_cell_keys,
                ),
            );
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
                return Err("Failed to record committed route centerline".to_string());
            }
            self.remember_committed_route_opened_cells(net_id, opened_cell_keys);
            self.add_post_commit_guidance_for_route(net_id, route);
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
    pub(crate) fn commit_native_route_with_clearance_internal(
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
                return false;
            }
            self.remember_committed_route_opened_cells(net_id, opened_cell_keys);
            self.add_post_commit_guidance_for_route(net_id, route);
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

    pub(crate) fn dynamic_owners_for_native_route(
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

    pub(crate) fn add_history_for_native_route(
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
    /// See `RealizedDynamicBlockers`. `clearance_exempt_keys` follows
    /// `cells_with_other_dynamic_owner`.
    pub(crate) fn realized_dynamic_blockers(
        &self,
        net_id: u64,
        candidate: &[(f64, f64)],
        candidate_cells: &CenterlineCells,
        clearance_exempt_keys: &FxHashSet<CellKey>,
        width_um: f64,
        static_grid: &StaticGridSpec,
    ) -> RealizedDynamicBlockers {
        let mut result = RealizedDynamicBlockers {
            blockers: Vec::new(),
            shared_clear_cells: FxHashSet::default(),
            shared_clear_nets: FxHashSet::default(),
        };
        let cell_blockers = cells_with_other_dynamic_owner(
            &self.obstacle_map,
            &candidate_cells.occupied,
            clearance_exempt_keys,
            net_id,
        );
        let reach = static_grid.grid_size_um + width_um;
        for cell in cell_blockers {
            let owners: Vec<u64> = self
                .obstacle_map
                .dynamic_owners_for_cells(&[cell])
                .into_iter()
                .filter(|owner| *owner != net_id)
                .collect();
            let (cx, cy) = grid_cell_center(cell.0, cell.1, static_grid);
            let region = (cx - reach, cy - reach, cx + reach, cy + reach);
            // Without a remembered geometry for the owner the only available
            // judgement is the grid-level one, at the granularity it was always
            // made: a cell-center hit blocks, a sampled-only cell does not.
            let collides =
                owners.iter().any(
                    |owner| match self.committed_realized_center_routes.get(owner) {
                        Some(other) => polylines_closer_than(candidate, other, width_um, region),
                        None => candidate_cells.is_center_cell(cell),
                    },
                );
            if collides {
                result.blockers.push(cell);
            } else {
                result.shared_clear_cells.insert(pack_xy(cell.0, cell.1));
                result.shared_clear_nets.extend(owners);
            }
        }
        // The cell-ownership prefilter above is structurally blind to one
        // geometry: a candidate shifted into the corner-cell gap between two
        // adjacent committed diagonals occupies only cells that no net owns
        // (committed diagonals own their own cells, never the corners), so no
        // blocker cell exists to inspect -- yet the realized waveguides
        // physically overlap (multiportmmi_8x8 n_13/n_14 at heuristic weight
        // 1.0: corrected centerlines 0.44 um apart with 0.5 um width). Sweep
        // the candidate directly against every committed realized centerline
        // whose bounding box comes near it.
        if let Some((min_x, min_y, max_x, max_y)) = polyline_bbox(candidate) {
            let reach = width_um;
            for (owner, other) in &self.committed_realized_center_routes {
                if *owner == net_id {
                    continue;
                }
                let Some((o_min_x, o_min_y, o_max_x, o_max_y)) = polyline_bbox(other) else {
                    continue;
                };
                if o_min_x > max_x + reach
                    || o_max_x < min_x - reach
                    || o_min_y > max_y + reach
                    || o_max_y < min_y - reach
                {
                    continue;
                }
                if let Some(point) =
                    polylines_parallel_overlap_point(candidate, other, width_um - 1.0e-6)
                {
                    if let Some(cell) = self.grid_cell_for_physical_point(point) {
                        result.blockers.push(cell);
                    }
                }
            }
        }
        result
    }

    /// Record the geometry a corrected commit actually realized, so later
    /// candidates of other nets are judged against it rather than the raw
    /// primitive centerline `remember_committed_route_centerlines_with_ports`
    /// derives from the route alone.
    pub(crate) fn remember_corrected_realized_centerline(
        &mut self,
        net_id: u64,
        centerline: &[(f64, f64)],
    ) {
        if centerline.len() >= 2 {
            self.committed_realized_center_routes
                .insert(net_id, compress_physical_centerline(centerline.to_vec()));
        }
    }

    pub(crate) fn remember_committed_route_centerlines_with_ports(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<(), String> {
        let centerline =
            self.routing_centerline_for_route(route, source_port_um, target_port_um)?;
        let grid_path = self.route_obstacle_center_cells(route);
        let grid_waypoints = compress_grid_waypoints_rs(&grid_path);
        self.committed_center_routes.insert(net_id, grid_waypoints);
        self.committed_realized_center_routes
            .insert(net_id, centerline);
        if let Some(guard) =
            self.terminal_bump_guard_for_target(route.reached_target, target_port_um)
        {
            self.committed_target_terminal_bump_guards
                .insert(net_id, guard);
        } else {
            self.committed_target_terminal_bump_guards.remove(&net_id);
        }
        Ok(())
    }

    pub(crate) fn remember_committed_route_opened_cells(
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

    pub(crate) fn rollback_committed_route(&mut self, net_id: u64) {
        self.remove_crossing_events_for_net(net_id);
        self.obstacle_map.ripup_route(net_id);
        self.committed_center_routes.remove(&net_id);
        self.committed_realized_center_routes.remove(&net_id);
        self.committed_target_terminal_bump_guards.remove(&net_id);
        self.committed_opened_cell_keys.remove(&net_id);
        self.invalidate_meander_base_prefix();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;

    /// Milestone 4 of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`:
    /// a probe that reports itself clean (`probe_crossing_compliant = true`,
    /// no candidate blockers) but never registered the crossing event for
    /// its one real intersection -- the exact shape that made
    /// `try_commit_clean_probe` commit-then-reject on the 64x64 mesh (net
    /// 575 vs net 573). Uses the B1 kernel fixture
    /// (`missing_crossing_event_fixture_router`/`install_diagonal_partner`/
    /// `crossing_diagonal_centerline`, shared with
    /// `intersection_without_crossing_event_is_a_violation` above) rather
    /// than `crossing_conflict_fixture`: that fixture's three nets are
    /// *actually* committed on `obstacle_map` with radius-1 clearance, so a
    /// hand-built probe that drops the real crossing event also loses the
    /// legal exemption the real search would have carried for the other
    /// two committed nets, and the low-level grid commit rejects for an
    /// unrelated reason before the post-commit geometric check this test
    /// targets ever runs. The B1 fixture books its one committed partner
    /// (net 3) only into `committed_center_routes`/`committed_realized_center_routes`
    /// (the geometric-check bookkeeping `try_commit_clean_probe`'s
    /// post-commit validation reads) and never into `obstacle_map`, so the
    /// grid-level commit here has nothing to conflict with and always
    /// succeeds -- isolating the geometric rejection this test is about.
    #[test]
    fn rejected_commit_records_partners_and_does_not_abort() {
        let mut router = missing_crossing_event_fixture_router();
        install_diagonal_partner(&mut router, 3);

        let probe_route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: (0..=40).map(|i| (10 + i, 50 - i)).collect(),
            compressed_waypoints: vec![(10, 50), (50, 10)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(50, 10, 0),
            reached_target: State::new(50, 10, 0),
            stats: RouteSearchStats::default(),
        };
        let job = NativeRouteJob::new(
            4,
            PyState::new(10, 50, 0),
            PyState::new(50, 10, 0),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            None,
            None,
        );
        let rejecting_probe = ProbeState {
            probe_route,
            crossing_repair_enabled: true,
            allowed_crossing_partners: FxHashSet::default(),
            // Blanked: this is the bug shape -- net 3's crossing was never
            // registered, so the post-commit geometric check below must
            // find it as a `missing_crossing_event` violation.
            probe_crossing_events: Vec::new(),
            strict_expected_crossing_probe: false,
            // Forced true: the probe believes the route is clean.
            probe_crossing_compliant: true,
            probe_realized_crossing_violations: Vec::new(),
            probe_grid_crossing_violations: Vec::new(),
            probe_repair_keepout_keys: FxHashSet::default(),
            // Blanked: this is what routes `try_commit_clean_probe` into
            // its clean-commit branch instead of returning `NotResolved`.
            candidate_blockers: Vec::new(),
            probe_intersecting_partners: Vec::new(),
        };
        let mut batch = fresh_repair_batch_state();

        let result = router.try_commit_clean_probe(
            &mut batch,
            &rejecting_probe,
            &job,
            0,
            Some(0),
            Some(0),
            false,
        );

        assert!(
            matches!(result, Err(())),
            "the post-commit validation must reject this route, got Ok"
        );
        assert_eq!(batch.failed_net_id, Some(4));
        assert!(
            batch.last_rejected_commit_partners.contains(&3),
            "the diagonal net must be recorded as the rejection's culprit: {:?}",
            batch.last_rejected_commit_partners
        );
        assert!(
            router.obstacle_map.get_net_cells(4).is_none(),
            "the rejected commit must be rolled back off the obstacle map"
        );
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
            None,
        );

        let keepout = router.dynamic_commit_error_repair_keepout_keys(
            "Failed to commit routed cells to obstacle map: dynamic_overlap_count=2 dynamic_overlap_owners=[36] dynamic_overlap_bbox=(10,12,20,21) dynamic_overlap_sample=(10,20),(12,21)",
        );

        assert!(keepout.contains(&pack_xy(9, 19)));
        assert!(keepout.contains(&pack_xy(13, 22)));
        assert!(!keepout.contains(&pack_xy(8, 18)));
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
            None,
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
}
