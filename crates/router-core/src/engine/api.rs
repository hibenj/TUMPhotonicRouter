//! The router's plain-data operations: static cells and rectangles, port
//! openings, net cells and the raw obstacle views. The Python
//! `PyPhotonicRouter` methods of the same names forward here (phase 8
//! slice B of the 2026-10 do-over moved the bodies out of the bindings).

use crate::error::RouterError;
use crate::obstacle_map::{pack_xy, unpack_xy, GridRect};
use crate::search::state::State;
use crate::static_obstacle_builder::StaticGridSpec;

use crate::engine::*;

impl Router {
    pub fn set_route_width_um(&mut self, width_um: f64) -> Result<(), RouterError> {
        if !width_um.is_finite() || width_um <= 0.0 {
            return Err(RouterError::Config(
                "route width must be finite and > 0".to_string(),
            ));
        }
        self.route_width_um = width_um;
        Ok(())
    }

    pub fn set_long_straight_exempt_net_ids(&mut self, net_ids: Vec<u64>) {
        self.long_straight_exempt_net_ids = net_ids.into_iter().collect();
    }

    pub fn add_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.invalidate_meander_base_prefix();
        for (x, y) in &cells {
            self.static_cells.insert(pack_xy(*x, *y));
        }
        self.obstacle_map.add_static_cells(&cells);
    }

    pub fn set_static_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.clear_static_cells();
        self.add_static_cells(cells);
    }

    pub fn add_static_rects(&mut self, rects: Vec<(i32, i32, i32, i32)>) {
        self.invalidate_meander_base_prefix();
        let obstacle_rects: Vec<GridRect> = rects
            .into_iter()
            .map(|(x_min, y_min, x_max, y_max)| GridRect {
                x_min,
                y_min,
                x_max,
                y_max,
            })
            .collect();
        self.obstacle_map.add_static_rects(&obstacle_rects);
    }

    pub fn set_static_rects(&mut self, rects: Vec<(i32, i32, i32, i32)>) {
        self.clear_static_cells();
        let obstacle_rects: Vec<GridRect> = rects
            .into_iter()
            .map(|(x_min, y_min, x_max, y_max)| GridRect {
                x_min,
                y_min,
                x_max,
                y_max,
            })
            .collect();
        self.obstacle_map.set_static_rects(&obstacle_rects);
        self.invalidate_meander_base_prefix();
    }

    pub fn build_port_footprint_cells(
        &self,
        ports: Vec<(String, f64, f64, Option<f64>, i32, i32)>,
    ) -> Vec<(String, Vec<(i32, i32)>)> {
        let grid = StaticGridSpec {
            width: self.grid.width as i32,
            height: self.grid.height as i32,
            grid_size_um: self.grid.grid_size_um,
            origin: (self.grid.origin_x_um, self.grid.origin_y_um),
            die_bbox: (0.0, 0.0, 0.0, 0.0),
        };

        ports
            .into_iter()
            .map(
                |(spec, x_um, y_um, orientation, length_cells, half_width_cells)| {
                    let cells = route_port_footprint_cells(
                        &grid,
                        x_um,
                        y_um,
                        orientation,
                        length_cells,
                        half_width_cells,
                    );
                    (spec, sorted_cells(cells))
                },
            )
            .collect()
    }

    pub fn build_dynamic_clearance_exempt_cells_for_routes(
        &self,
        jobs: Vec<(u64, State, State, Vec<(i32, i32)>)>,
        commit_radius_cells: i32,
        min_run_in_length_cells: i32,
    ) -> Vec<(u64, Vec<(i32, i32)>)> {
        let grid = static_grid_from_grid_spec(&self.grid);
        jobs.into_iter()
            .map(|(net_id, source, target, opened_cells)| {
                let opened_cells = pack_cells(&opened_cells);
                let cells = route_dynamic_clearance_exempt_cells(
                    &grid,
                    &opened_cells,
                    source,
                    target,
                    commit_radius_cells,
                    min_run_in_length_cells,
                );
                (net_id, sorted_cells(cells))
            })
            .collect()
    }

    pub fn add_port_open_cells(&mut self, cells: Vec<(i32, i32)>) {
        self.port_open_cells.extend(pack_cells(&cells));
    }

    pub fn ripup_route(&mut self, net_id: u64) -> bool {
        self.remove_crossing_events_for_net(net_id);
        let removed = self.obstacle_map.ripup_route(net_id);
        if removed {
            self.committed_center_routes.remove(&net_id);
            self.committed_realized_center_routes.remove(&net_id);
            self.committed_target_terminal_bump_guards.remove(&net_id);
            self.committed_opened_cell_keys.remove(&net_id);
            self.invalidate_meander_base_prefix();
        }
        removed
    }

    pub fn get_net_cells(&self, net_id: u64) -> Vec<(i32, i32)> {
        self.obstacle_map
            .get_net_cells(net_id)
            .map(|cells| cells.iter().copied().map(unpack_xy).collect())
            .unwrap_or_default()
    }

    pub fn get_net_core_cells(&self, net_id: u64) -> Vec<(i32, i32)> {
        self.obstacle_map
            .get_net_core_cells(net_id)
            .map(|cells| cells.iter().copied().map(unpack_xy).collect())
            .unwrap_or_default()
    }

    pub fn raw_dynamic_obstacle_cells(&self) -> Vec<(i32, i32, u16)> {
        let mut cells: Vec<(i32, i32, u16)> = self
            .obstacle_map
            .dynamic_obstacle_entries()
            .map(|(key, refs)| {
                let (x, y) = unpack_xy(key);
                (x, y, refs)
            })
            .collect();
        cells.sort_unstable();
        cells
    }

    pub fn raw_static_obstacle_cells(&self) -> Vec<(i32, i32)> {
        let mut cells: Vec<(i32, i32)> = self
            .obstacle_map
            .static_obstacle_keys()
            .map(unpack_xy)
            .collect();
        cells.sort_unstable();
        cells
    }

    pub fn raw_dynamic_core_cells(&self) -> Vec<(i32, i32, u16)> {
        let mut cells: Vec<(i32, i32, u16)> = self
            .obstacle_map
            .dynamic_core_obstacle_entries()
            .map(|(key, refs)| {
                let (x, y) = unpack_xy(key);
                (x, y, refs)
            })
            .collect();
        cells.sort_unstable();
        cells
    }

    pub fn all_net_route_cells(&self) -> Vec<(u64, Vec<(i32, i32)>)> {
        let mut routes: Vec<(u64, Vec<(i32, i32)>)> = self
            .obstacle_map
            .net_route_entries()
            .map(|(net_id, cells)| {
                (
                    net_id,
                    cells.iter().copied().map(unpack_xy).collect::<Vec<_>>(),
                )
            })
            .collect();
        routes.sort_unstable_by_key(|(net_id, _)| *net_id);
        routes
    }

    pub fn commit_route_cells(&mut self, net_id: u64, cells: Vec<(i32, i32)>) -> bool {
        let committed = self.obstacle_map.commit_route(net_id, &cells);
        if committed {
            self.invalidate_meander_base_prefix();
        }
        committed
    }
}
