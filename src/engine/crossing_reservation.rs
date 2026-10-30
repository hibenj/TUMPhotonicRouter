use rustc_hash::{FxHashMap, FxHashSet};

use crate::geometry_realization::{
    grid_path_to_centerline as grid_path_to_centerline_rs,
    route_to_grid_path as route_to_grid_path_rs,
    route_to_primitive_centerline as route_to_primitive_centerline_rs,
};
use crate::obstacle_map::{pack_xy, unpack_xy, CellKey};
use crate::search::state::RouteResult;
use crate::static_obstacle_builder::floor_snap_to_grid;

#[cfg(test)]
use crate::crossings::CrossingConfig;
use crate::engine::*;
#[cfg(test)]
use crate::search::state::RouteSearchStats;

#[derive(Clone, Debug)]
pub(crate) struct CrossingEvent {
    pub(crate) net_id: u64,
    pub(crate) partner_net_id: u64,
    pub(crate) point: (f64, f64),
    pub(crate) route_segment: ((i32, i32), (i32, i32)),
    pub(crate) partner_segment: ((i32, i32), (i32, i32)),
    pub(crate) route_angle: u8,
    pub(crate) partner_angle: u8,
    pub(crate) reservation_keys: FxHashSet<CellKey>,
}

#[derive(Clone, Debug)]
pub(crate) struct InvalidCrossingIntersection {
    pub(crate) net_id: u64,
    pub(crate) partner_net_id: u64,
    pub(crate) point: (f64, f64),
    pub(crate) reason: &'static str,
}

/// One realized intersection as the unified predicate assessed it
/// (do-over phase 4 slice B, `RouterConfig::crossing.unified_legality`):
/// the point in um, the two grid waypoint segments the margins were read
/// from (`None` when the grid route had no segment there), the candidate
/// the predicate saw, the rule, and the verdict. Kept on the router
/// (`crossing_verdicts`) for the `crossing_verdicts()` binding.
#[derive(Clone, Debug)]
pub(crate) struct CrossingVerdictRecord {
    pub(crate) net_id: u64,
    pub(crate) partner_net_id: u64,
    pub(crate) point: (f64, f64),
    pub(crate) route_segment: Option<((i32, i32), (i32, i32))>,
    pub(crate) partner_segment: Option<((i32, i32), (i32, i32))>,
    pub(crate) candidate: CrossingCandidate,
    pub(crate) rule: CrossingLegalityRule,
    pub(crate) verdict: CrossingVerdict,
    /// False when `grid_crossing_near` found no grid intersection within 1.5
    /// cells of the point and the realized straights were measured instead.
    pub(crate) margins_from_grid: bool,
}

/// A record plus the `CrossingEvent` the intersection would register
/// (`None` when no grid direction could be derived for it).
pub(crate) struct CrossingAssessment {
    pub(crate) event: Option<CrossingEvent>,
    pub(crate) record: CrossingVerdictRecord,
}

/// Everything `unified_crossing_assessment` finds on one route: the
/// proper intersections with their verdicts, and the collinear / parallel
/// overlaps, which are violations of a different kind (two waveguides
/// merging), not crossings.
#[derive(Default)]
pub(crate) struct UnifiedCrossingAssessment {
    pub(crate) crossings: Vec<CrossingAssessment>,
    pub(crate) overlaps: Vec<InvalidCrossingIntersection>,
}

/// The grid route's intersection nearest a realized point: the two
/// waypoint segments and the four straight runs in cells.
struct GridCrossingNear {
    route_segment: ((i32, i32), (i32, i32)),
    partner_segment: ((i32, i32), (i32, i32)),
    route_before_cells: f64,
    route_after_cells: f64,
    partner_before_cells: f64,
    partner_after_cells: f64,
}

#[derive(Default)]
pub(crate) struct CrossingReservationBlockers {
    pub(crate) has_static_blocker: bool,
    pub(crate) dynamic_blockers: FxHashSet<u64>,
}

impl CrossingReservationBlockers {
    pub(crate) fn is_clear(&self) -> bool {
        !self.has_static_blocker && self.dynamic_blockers.is_empty()
    }
}

pub(crate) fn crossing_reservation_window_keys(
    center_x: f64,
    center_y: f64,
    half_size_cells: i32,
    width: i32,
    height: i32,
) -> FxHashSet<CellKey> {
    let mut keys = FxHashSet::default();
    if half_size_cells < 0 {
        return keys;
    }
    let min_x = (center_x - f64::from(half_size_cells)).floor() as i32;
    let max_x = (center_x + f64::from(half_size_cells)).ceil() as i32;
    let min_y = (center_y - f64::from(half_size_cells)).floor() as i32;
    let max_y = (center_y + f64::from(half_size_cells)).ceil() as i32;
    for x in min_x..=max_x {
        if x < 0 || x >= width {
            continue;
        }
        for y in min_y..=max_y {
            if y < 0 || y >= height {
                continue;
            }
            keys.insert(pack_xy(x, y));
        }
    }
    keys
}

pub(crate) fn crossing_required_margin_cells(
    crossing_half_size_cells: i32,
    bend_runout_cells: i32,
) -> i32 {
    crossing_half_size_cells.max(0) + bend_runout_cells.max(0)
}

pub(crate) fn realized_crossing_margin_um(crossing_half_size_cells: i32, grid_size_um: f64) -> f64 {
    grid_size_um * f64::from(crossing_half_size_cells.max(0))
}

pub(crate) fn crossing_events_for_partner(
    net_id: u64,
    partner_net_id: u64,
    route_waypoints: &[(i32, i32)],
    partner_waypoints: &[(i32, i32)],
    half_size_cells: i32,
    bend_runout_cells: i32,
    width: i32,
    height: i32,
) -> Vec<CrossingEvent> {
    if route_waypoints.len() < 2 || partner_waypoints.len() < 2 {
        return Vec::new();
    }
    let required_margin = f64::from(crossing_required_margin_cells(
        half_size_cells,
        bend_runout_cells,
    ));
    let mut events = Vec::new();
    let mut seen_centers = FxHashSet::default();
    for seg_a in route_waypoints.windows(2) {
        let Some(angle_a) = direction_angle_between_cells(seg_a[0], seg_a[1]) else {
            continue;
        };
        let len_a = segment_length_cells(seg_a[0], seg_a[1]);
        if len_a <= 0.0 {
            continue;
        }
        for seg_b in partner_waypoints.windows(2) {
            let Some(angle_b) = direction_angle_between_cells(seg_b[0], seg_b[1]) else {
                continue;
            };
            if !axes_are_perpendicular(angle_a, angle_b) {
                continue;
            }
            let len_b = segment_length_cells(seg_b[0], seg_b[1]);
            if len_b <= 0.0 {
                continue;
            }
            let Some((x, y, t, u)) =
                segment_intersection_with_params(seg_a[0], seg_a[1], seg_b[0], seg_b[1])
            else {
                continue;
            };
            let margin_a = (t * len_a).min((1.0 - t) * len_a);
            let margin_b = (u * len_b).min((1.0 - u) * len_b);
            if margin_a + 1e-9 < required_margin || margin_b + 1e-9 < required_margin {
                continue;
            }
            let rounded_center = (
                (x * 1_000_000.0).round() as i64,
                (y * 1_000_000.0).round() as i64,
            );
            if !seen_centers.insert(rounded_center) {
                continue;
            }
            let reservation_keys =
                crossing_reservation_window_keys(x, y, half_size_cells, width, height);
            events.push(CrossingEvent {
                net_id,
                partner_net_id,
                point: (x, y),
                route_segment: (seg_a[0], seg_a[1]),
                partner_segment: (seg_b[0], seg_b[1]),
                route_angle: angle_a,
                partner_angle: angle_b,
                reservation_keys,
            });
        }
    }
    events
}

pub(crate) fn crossing_event_svg_overlay(events: &[CrossingEvent], height: i32) -> String {
    if events.is_empty() {
        return String::new();
    }
    let mut out = String::new();
    out.push_str(r##"<g id="crossing-events">"##);
    for event in events {
        out.push_str(&format!(
            r##"<g class="crossing-event" data-net-id="{}" data-partner-net-id="{}">"##,
            event.net_id, event.partner_net_id
        ));
        out.push_str(&format!(
            "<title>crossing: net {} x net {} at ({:.3}, {:.3})</title>",
            event.net_id, event.partner_net_id, event.point.0, event.point.1
        ));
        let mut reservation_cells: Vec<CellKey> = event.reservation_keys.iter().copied().collect();
        reservation_cells.sort_unstable();
        for key in reservation_cells {
            let (x, y) = unpack_xy(key);
            let svg_y = height - y - 1;
            out.push_str(&format!(
                r##"<rect x="{x}" y="{svg_y}" width="1" height="1" fill="#8a8a8a" opacity="0.45" />"##
            ));
        }
        out.push_str("</g>");
    }
    out.push_str("</g>");
    out
}

pub(crate) fn append_crossing_event_svg_overlay(
    mut svg: String,
    events: &[CrossingEvent],
    height: i32,
) -> String {
    let overlay = crossing_event_svg_overlay(events, height);
    if overlay.is_empty() {
        return svg;
    }
    if let Some(index) = svg.rfind("</svg>") {
        svg.insert_str(index, &overlay);
    } else {
        svg.push_str(&overlay);
    }
    svg
}

impl PyPhotonicRouter {
    /// Every currently-committed net other than `net_id`, unfiltered by any
    /// spatial window. In lidar-pure mode this is not a whitelist of nets a
    /// route is allowed to cross -- it is the full centerline lookup
    /// database a collision-crossing search attempt can consult once it
    /// discovers, from actual dynamic obstacle cells while expanding moves,
    /// which other net it just collided with.
    pub(crate) fn lidar_pure_full_map_partner_set(&self, net_id: u64) -> FxHashSet<u64> {
        self.obstacle_map
            .net_route_entries()
            .map(|(partner_id, _)| partner_id)
            .filter(|partner_id| *partner_id != net_id)
            .collect()
    }

    /// Every currently-committed net (other than `net_id`) with at least one
    /// cell inside the axis-aligned box `[min_x, max_x] x [min_y, max_y]`.
    /// Used by `lidar_route_result_partner_lookup_set` (box derived from an
    /// actual route's cells).
    pub(crate) fn partners_within_bbox(
        &self,
        net_id: u64,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
    ) -> FxHashSet<u64> {
        self.obstacle_map
            .net_route_entries()
            .filter_map(|(partner_id, cells)| {
                if partner_id == net_id {
                    return None;
                }
                cells
                    .iter()
                    .any(|key| {
                        let (x, y) = unpack_xy(*key);
                        x >= min_x && x <= max_x && y >= min_y && y <= max_y
                    })
                    .then_some(partner_id)
            })
            .collect()
    }

    pub(crate) fn lidar_probe_partner_lookup_set(
        &self,
        net_id: u64,
        probe_route: &RouteResult,
        owner_lookup_radius_cells: i32,
        final_routes: &FxHashMap<u64, RouteResult>,
    ) -> FxHashSet<u64> {
        self.dynamic_owners_for_native_route(probe_route, owner_lookup_radius_cells)
            .into_iter()
            .filter(|owner| *owner != net_id && final_routes.contains_key(owner))
            .collect()
    }

    pub(crate) fn lidar_route_result_partner_lookup_set(
        &self,
        net_id: u64,
        route: &RouteResult,
        extra_radius_cells: i32,
    ) -> FxHashSet<u64> {
        if !self.crossing_context.is_enabled() {
            return FxHashSet::default();
        }
        let points: Vec<(i32, i32)> = if route.cells.is_empty() {
            route.compressed_waypoints.clone()
        } else {
            route.cells.clone()
        };
        if points.is_empty() {
            return FxHashSet::default();
        }
        let extra = extra_radius_cells.max(0);
        let min_x = points
            .iter()
            .map(|(x, _)| *x)
            .min()
            .unwrap_or(0)
            .saturating_sub(extra);
        let max_x = points
            .iter()
            .map(|(x, _)| *x)
            .max()
            .unwrap_or(0)
            .saturating_add(extra);
        let min_y = points
            .iter()
            .map(|(_, y)| *y)
            .min()
            .unwrap_or(0)
            .saturating_sub(extra);
        let max_y = points
            .iter()
            .map(|(_, y)| *y)
            .max()
            .unwrap_or(0)
            .saturating_add(extra);
        self.partners_within_bbox(net_id, min_x, max_x, min_y, max_y)
    }

    pub(crate) fn crossing_partner_lookup_set_for_result(
        &self,
        net_id: u64,
        route: &RouteResult,
    ) -> FxHashSet<u64> {
        if !self.crossing_context.is_enabled() {
            return FxHashSet::default();
        }
        let config = self.crossing_context.config();
        let margin = crossing_required_margin_cells(
            config.crossing_half_size_cells,
            self.primitive_cfg.bend_radius_cells,
        );
        self.lidar_route_result_partner_lookup_set(
            net_id,
            route,
            margin
                .saturating_mul(2)
                .saturating_add(self.primitive_cfg.bend_radius_cells),
        )
    }

    /// The collision-crossing partner set to use for an *upfront* (not
    /// deferred) search attempt, given `try_order`.
    ///
    /// For `PlainFirst` callers this deliberately returns an empty set: the
    /// caller is expected to try its own plain attempt first and only
    /// consult `lidar_pure_full_map_partner_set` directly, a second time, if
    /// that plain attempt fails (matching `route_and_commit`'s two-stage
    /// structure for a plain call, which this function does not otherwise
    /// change). For `CrossingFirst` callers the full partner set is returned
    /// immediately, since that case has no deferred second stage. With
    /// crossings disabled the set is empty either way.
    pub(crate) fn upfront_collision_crossing_partner_ids(
        &self,
        net_id: u64,
        try_order: CollisionCrossingTryOrder,
    ) -> FxHashSet<u64> {
        if !self.crossing_context.is_enabled() {
            return FxHashSet::default();
        }
        match try_order {
            CollisionCrossingTryOrder::PlainFirst => FxHashSet::default(),
            CollisionCrossingTryOrder::CrossingFirst => {
                self.lidar_pure_full_map_partner_set(net_id)
            }
        }
    }

    pub(crate) fn crossing_events_for_route(
        &self,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
    ) -> Vec<CrossingEvent> {
        let config = self.crossing_context.config();
        let mut events = Vec::new();
        for partner_id in partner_ids {
            let Some(partner_waypoints) = self.committed_center_routes.get(partner_id) else {
                continue;
            };
            events.extend(crossing_events_for_partner(
                net_id,
                *partner_id,
                &route.compressed_waypoints,
                partner_waypoints,
                config.crossing_half_size_cells,
                self.primitive_cfg.bend_radius_cells,
                self.grid.width as i32,
                self.grid.height as i32,
            ));
        }
        events
    }

    pub(crate) fn realized_crossing_events_for_route(
        &self,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Vec<CrossingEvent> {
        if partner_ids.is_empty() || !self.crossing_context.is_enabled() {
            return Vec::new();
        }
        let route_centerline =
            self.routing_centerline_for_route(route, source_port_um, target_port_um);
        let Ok(route_centerline) = route_centerline else {
            return self.crossing_events_for_route(net_id, route, partner_ids);
        };
        if route_centerline.len() < 2 {
            return Vec::new();
        }
        let config = self.crossing_context.config();
        let required_margin_um =
            realized_crossing_margin_um(config.crossing_half_size_cells, self.grid.grid_size_um);
        let mut events = Vec::new();
        let mut seen = FxHashSet::default();
        for route_segment in route_centerline.windows(2) {
            let route_len = physical_segment_length(route_segment[0], route_segment[1]);
            if route_len <= 0.0 {
                continue;
            }
            let Some(route_grid_segment) =
                self.physical_segment_to_grid_segment(route_segment[0], route_segment[1])
            else {
                continue;
            };
            let Some(route_angle) =
                direction_angle_between_cells(route_grid_segment.0, route_grid_segment.1)
            else {
                continue;
            };
            for partner_id in partner_ids {
                let Some(partner_centerline) = self
                    .committed_realized_center_routes
                    .get(partner_id)
                    .cloned()
                    .or_else(|| {
                        self.committed_center_routes
                            .get(partner_id)
                            .map(|waypoints| self.grid_waypoints_to_centerline(waypoints))
                    })
                else {
                    continue;
                };
                for partner_segment in partner_centerline.windows(2) {
                    let partner_len =
                        physical_segment_length(partner_segment[0], partner_segment[1]);
                    if partner_len <= 0.0 {
                        continue;
                    }
                    if !physical_segments_are_perpendicular(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) {
                        continue;
                    }
                    let Some((x, y, t, u)) = physical_segment_intersection_with_params(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        continue;
                    };
                    let route_margin = (t * route_len).min((1.0 - t) * route_len);
                    let partner_margin = (u * partner_len).min((1.0 - u) * partner_len);
                    if route_margin + 1e-9 < required_margin_um
                        || partner_margin + 1e-9 < required_margin_um
                    {
                        continue;
                    }
                    if self
                        .crossing_footprint_unrelated_dynamic_owner(net_id, *partner_id, (x, y))
                        .is_some()
                    {
                        continue;
                    }
                    let pair_key = (
                        net_id.min(*partner_id),
                        net_id.max(*partner_id),
                        (x * 1_000_000.0).round() as i64,
                        (y * 1_000_000.0).round() as i64,
                    );
                    if !seen.insert(pair_key) {
                        continue;
                    }
                    let Some((center_x, center_y)) = self.grid_cell_for_physical_point((x, y))
                    else {
                        continue;
                    };
                    let Some(partner_grid_segment) = self
                        .physical_segment_to_grid_segment(partner_segment[0], partner_segment[1])
                    else {
                        continue;
                    };
                    let Some(partner_angle) = direction_angle_between_cells(
                        partner_grid_segment.0,
                        partner_grid_segment.1,
                    ) else {
                        continue;
                    };
                    events.push(CrossingEvent {
                        net_id,
                        partner_net_id: *partner_id,
                        point: (x, y),
                        route_segment: route_grid_segment,
                        partner_segment: partner_grid_segment,
                        route_angle,
                        partner_angle,
                        reservation_keys: crossing_reservation_window_keys(
                            f64::from(center_x),
                            f64::from(center_y),
                            config.crossing_half_size_cells,
                            self.grid.width as i32,
                            self.grid.height as i32,
                        ),
                    });
                }
            }
        }
        events
    }

    pub(crate) fn crossing_partner_ids_from_events(events: &[CrossingEvent]) -> FxHashSet<u64> {
        events.iter().map(|event| event.partner_net_id).collect()
    }

    pub(crate) fn crossing_events_have_disjoint_reservations(events: &[CrossingEvent]) -> bool {
        let mut seen = FxHashSet::default();
        for event in events {
            for key in &event.reservation_keys {
                if !seen.insert(*key) {
                    return false;
                }
            }
        }
        true
    }

    pub(crate) fn crossing_partners_with_overlapping_reservations(
        events: &[CrossingEvent],
    ) -> FxHashSet<u64> {
        let mut owner_by_key: FxHashMap<CellKey, u64> = FxHashMap::default();
        let mut overlapping_partners = FxHashSet::default();
        for event in events {
            for key in &event.reservation_keys {
                if let Some(previous_partner_id) = owner_by_key.insert(*key, event.partner_net_id) {
                    overlapping_partners.insert(previous_partner_id);
                    overlapping_partners.insert(event.partner_net_id);
                }
            }
        }
        overlapping_partners
    }

    pub(crate) fn crossing_reservation_blockers(
        &self,
        net_id: u64,
        events: &[CrossingEvent],
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> CrossingReservationBlockers {
        let mut blockers = CrossingReservationBlockers::default();
        for event in events {
            for key in &event.reservation_keys {
                let (x, y) = unpack_xy(*key);
                if !self.obstacle_map.in_bounds(x, y) {
                    blockers.has_static_blocker = true;
                    continue;
                }
                let is_opened_static = opened_cell_keys
                    .map(|opened| opened.contains(key))
                    .unwrap_or(false);
                if self.obstacle_map.is_static_blocked(x, y) && !is_opened_static {
                    blockers.has_static_blocker = true;
                }
                for owner in self.obstacle_map.dynamic_owners_for_cells(&[(x, y)]) {
                    if owner != net_id && owner != event.partner_net_id {
                        blockers.dynamic_blockers.insert(owner);
                    }
                }
            }
        }
        blockers
    }

    pub(crate) fn grid_cell_for_physical_point(&self, point: (f64, f64)) -> Option<(i32, i32)> {
        if !point.0.is_finite() || !point.1.is_finite() || self.grid.grid_size_um <= 0.0 {
            return None;
        }
        let x = floor_snap_to_grid(point.0, self.grid.origin_x_um, self.grid.grid_size_um);
        let y = floor_snap_to_grid(point.1, self.grid.origin_y_um, self.grid.grid_size_um);
        self.obstacle_map.in_bounds(x, y).then_some((x, y))
    }

    pub(crate) fn physical_segment_to_grid_segment(
        &self,
        start: (f64, f64),
        end: (f64, f64),
    ) -> Option<((i32, i32), (i32, i32))> {
        Some((
            self.grid_cell_for_physical_point(start)?,
            self.grid_cell_for_physical_point(end)?,
        ))
    }

    pub(crate) fn dynamic_commit_error_repair_keepout_keys(
        &self,
        error: &str,
    ) -> FxHashSet<CellKey> {
        let Some((_, rest)) = error.split_once("dynamic_overlap_bbox=(") else {
            return FxHashSet::default();
        };
        let Some((bbox_text, _)) = rest.split_once(')') else {
            return FxHashSet::default();
        };
        let values: Vec<i32> = bbox_text
            .split(',')
            .filter_map(|value| value.trim().parse::<i32>().ok())
            .collect();
        if values.len() != 4 {
            return FxHashSet::default();
        }
        let (min_x, max_x, min_y, max_y) = (values[0], values[1], values[2], values[3]);
        if min_x > max_x || min_y > max_y {
            return FxHashSet::default();
        }
        let bbox_width = max_x.saturating_sub(min_x).saturating_add(1);
        let bbox_height = max_y.saturating_sub(min_y).saturating_add(1);
        let (keepout_min_x, keepout_max_x, keepout_min_y, keepout_max_y) =
            if bbox_width.saturating_mul(bbox_height) <= 256 {
                (
                    min_x.saturating_sub(1),
                    max_x.saturating_add(1),
                    min_y.saturating_sub(1),
                    max_y.saturating_add(1),
                )
            } else {
                let center_x = min_x.saturating_add(max_x) / 2;
                let center_y = min_y.saturating_add(max_y) / 2;
                (
                    center_x.saturating_sub(3),
                    center_x.saturating_add(3),
                    center_y.saturating_sub(3),
                    center_y.saturating_add(3),
                )
            };
        let mut keys = FxHashSet::default();
        for y in keepout_min_y..=keepout_max_y {
            for x in keepout_min_x..=keepout_max_x {
                if self.obstacle_map.in_bounds(x, y) {
                    keys.insert(pack_xy(x, y));
                }
            }
        }
        keys
    }

    pub(crate) fn crossing_events_satisfy_partner_constraints(
        &self,
        net_id: u64,
        partner_ids: &FxHashSet<u64>,
        crossing_events: &[CrossingEvent],
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        if partner_ids.is_empty() {
            return true;
        }
        let reservation_blockers =
            self.crossing_reservation_blockers(net_id, crossing_events, opened_cell_keys);
        !crossing_events.is_empty()
            && Self::crossing_events_have_disjoint_reservations(crossing_events)
            && reservation_blockers.is_clear()
    }

    pub(crate) fn invalid_grid_crossing_error_for_route(
        &self,
        net_id: u64,
        route: &RouteResult,
    ) -> Option<String> {
        if self.router_config.crossing.disable_rust_crossing_validation {
            return None;
        }
        if !self.crossing_context.is_enabled() {
            return None;
        }
        let partner_ids = self.crossing_partner_lookup_set_for_result(net_id, route);
        if partner_ids.is_empty() {
            return None;
        }
        let invalid = self.invalid_crossing_intersections_for_route(net_id, route, &partner_ids);
        if invalid.is_empty() {
            return None;
        }
        let violation = &invalid[0];
        Some(format!(
            "Illegal grid crossing: net {} intersects net {} at ({:.3}, {:.3}) ({})",
            violation.net_id,
            violation.partner_net_id,
            violation.point.0,
            violation.point.1,
            violation.reason
        ))
    }

    pub(crate) fn format_realized_crossing_violation_error(
        net_id: u64,
        violations: &[InvalidCrossingIntersection],
    ) -> String {
        let details = violations
            .iter()
            .map(|violation| {
                format!(
                    "partner={} point=({:.3},{:.3}) reason={}",
                    violation.partner_net_id,
                    violation.point.0,
                    violation.point.1,
                    violation.reason
                )
            })
            .collect::<Vec<_>>()
            .join("; ");
        format!(
            "Realized crossing validation failed for A*-accepted crossing route on net {}: {}",
            net_id, details
        )
    }

    pub(crate) fn invalid_crossing_intersections_for_route(
        &self,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
    ) -> Vec<InvalidCrossingIntersection> {
        if route.compressed_waypoints.len() < 2 || partner_ids.is_empty() {
            return Vec::new();
        }
        let config = self.crossing_context.config();
        // Same rule as the search kernel (`crossing_move_outcome_with_segments`)
        // and the realized validator: BEFORE the crossing point the counted
        // straight run must reach `required_margin` (half_size + bend radius,
        // compensating a preceding bend arm counted as straight); AFTER it
        // only the crossing element's own `half_size` pure straight cells are
        // required (Point 2, 2026-09-03) -- a bend may follow. Demanding
        // `required_margin` on both sides discarded kernel-legal,
        // realized-clean routes (benes_32x32 net 273).
        let required_margin = f64::from(crossing_required_margin_cells(
            config.crossing_half_size_cells,
            self.primitive_cfg.bend_radius_cells,
        ));
        let required_after = f64::from(config.crossing_half_size_cells.max(0));
        let unified_legality = self.router_config.crossing.unified_legality;
        let rule = self.crossing_legality_rule();
        let mut invalid = Vec::new();
        let mut seen_centers = FxHashSet::default();
        // Slice B-2: under the switch every grid intersection is collected
        // first, so each one is judged against the route's other crossings
        // (`nearest_same_route`) as well as the registered elements.
        let mut unified_candidates: Vec<(u64, CrossingCandidate)> = Vec::new();
        for route_segment in route.compressed_waypoints.windows(2) {
            let Some(route_angle) =
                direction_angle_between_cells(route_segment[0], route_segment[1])
            else {
                continue;
            };
            let route_len = segment_length_cells(route_segment[0], route_segment[1]);
            if route_len <= 0.0 {
                continue;
            }
            for partner_id in partner_ids {
                let Some(partner_waypoints) = self.committed_center_routes.get(partner_id) else {
                    continue;
                };
                for partner_segment in partner_waypoints.windows(2) {
                    let Some(partner_angle) =
                        direction_angle_between_cells(partner_segment[0], partner_segment[1])
                    else {
                        continue;
                    };
                    if !axes_are_perpendicular(route_angle, partner_angle) {
                        continue;
                    }
                    let partner_len = segment_length_cells(partner_segment[0], partner_segment[1]);
                    if partner_len <= 0.0 {
                        continue;
                    }
                    let Some((x, y, t, u)) = segment_intersection_with_params(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        continue;
                    };
                    if unified_legality {
                        // Phase 4 slice B: the one predicate decides, on the
                        // same grid numbers the legacy check below reads;
                        // the neighbours are filled in after the scan.
                        unified_candidates.push((
                            *partner_id,
                            CrossingCandidate {
                                route_direction: grid_angle_direction(route_angle),
                                partner_direction: grid_angle_direction(partner_angle),
                                route_before_cells: t * route_len,
                                route_after_cells: (1.0 - t) * route_len,
                                partner_before_cells: u * partner_len,
                                partner_after_cells: (1.0 - u) * partner_len,
                                point: (x, y),
                                nearest_same_route: None,
                                nearest_registered: None,
                                footprint_blocked: self.footprint_blocked_at_grid_point(
                                    net_id,
                                    *partner_id,
                                    x,
                                    y,
                                ),
                                event_registered: None,
                            },
                        ));
                        continue;
                    }
                    let route_before = t * route_len;
                    let route_after = (1.0 - t) * route_len;
                    let partner_margin = (u * partner_len).min((1.0 - u) * partner_len);
                    if route_before + 1e-9 >= required_margin
                        && route_after + 1e-9 >= required_after
                        && partner_margin + 1e-9 >= required_margin
                    {
                        continue;
                    }
                    let rounded_center = (
                        (x * 1_000_000.0).round() as i64,
                        (y * 1_000_000.0).round() as i64,
                    );
                    if !seen_centers.insert(rounded_center) {
                        continue;
                    }
                    invalid.push(InvalidCrossingIntersection {
                        net_id,
                        partner_net_id: *partner_id,
                        point: (x, y),
                        reason: "insufficient_straight_margin",
                    });
                }
            }
        }
        if unified_legality && !unified_candidates.is_empty() {
            let registered = self.registered_crossing_centres_of_other_nets(net_id);
            let centres: Vec<(f64, f64)> = unified_candidates
                .iter()
                .map(|(_, candidate)| candidate.point)
                .collect();
            for (partner_id, candidate) in &unified_candidates {
                let candidate = CrossingCandidate {
                    nearest_same_route: nearest_crossing_neighbour(
                        candidate.point,
                        rule.half_size_cells,
                        centres.iter().copied(),
                    ),
                    nearest_registered: nearest_crossing_neighbour(
                        candidate.point,
                        rule.half_size_cells,
                        registered.iter().copied(),
                    ),
                    ..*candidate
                };
                let verdict = crossing_legality(&candidate, &rule);
                if verdict.is_legal() {
                    continue;
                }
                let (x, y) = candidate.point;
                let rounded_center = (
                    (x * 1_000_000.0).round() as i64,
                    (y * 1_000_000.0).round() as i64,
                );
                if !seen_centers.insert(rounded_center) {
                    continue;
                }
                invalid.push(InvalidCrossingIntersection {
                    net_id,
                    partner_net_id: *partner_id,
                    point: (x, y),
                    reason: verdict.reason(),
                });
            }
        }
        invalid
    }

    pub(crate) fn realized_centerline_for_route(
        &self,
        route: &RouteResult,
    ) -> Result<Vec<(f64, f64)>, String> {
        let grid = self.geometry_grid()?;
        match route_to_primitive_centerline_rs(route, &self.primitives, &grid) {
            Ok(centerline) => Ok(compress_physical_centerline(centerline)),
            Err(_) => {
                let path = route_to_grid_path_rs(route, &self.primitives)
                    .unwrap_or_else(|_| route.compressed_waypoints.clone());
                grid_path_to_centerline_rs(&path, &grid)
                    .map(compress_physical_centerline)
                    .map_err(|err| err.to_string())
            }
        }
    }

    pub(crate) fn routing_centerline_for_route(
        &self,
        route: &RouteResult,
        _source_port_um: Option<(f64, f64)>,
        _target_port_um: Option<(f64, f64)>,
    ) -> Result<Vec<(f64, f64)>, String> {
        self.realized_centerline_for_route(route)
    }

    /// `require_registered_events`: see
    /// [`Self::crossing_violations_for_realized_centerline`]. `true` for the
    /// post-commit validator, `false` for every pre-commit search/probe
    /// caller.
    pub(crate) fn crossing_violations_for_route_with_ports(
        &self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        require_registered_events: bool,
    ) -> Vec<InvalidCrossingIntersection> {
        let route_centerline =
            self.routing_centerline_for_route(route, source_port_um, target_port_um);
        let Ok(route_centerline) = route_centerline else {
            return Vec::new();
        };
        let mut violations = if self.router_config.crossing.unified_legality {
            Self::unified_violations(&self.unified_crossing_assessment(
                net_id,
                &route_centerline,
                &route.compressed_waypoints,
                require_registered_events,
            ))
        } else {
            self.crossing_violations_for_realized_centerline(
                net_id,
                &route_centerline,
                require_registered_events,
            )
        };
        self.retain_violations_outside_endpoint_access(
            &route_centerline,
            opened_cell_keys,
            &mut violations,
        );
        violations
    }

    /// Drops the violations the endpoint-access rule excuses: an
    /// intersection on an opened (port-access) cell of either net that lies
    /// at an endpoint of either realized centerline. Shared by the legacy
    /// and the unified validation.
    pub(crate) fn retain_violations_outside_endpoint_access(
        &self,
        route_centerline: &[(f64, f64)],
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        violations: &mut Vec<InvalidCrossingIntersection>,
    ) {
        violations.retain(|violation| {
            let Some((x, y)) = self.grid_cell_for_physical_point(violation.point) else {
                return true;
            };
            let key = pack_xy(x, y);
            let current_endpoint_access = opened_cell_keys
                .map(|opened| opened.contains(&key))
                .unwrap_or(false);
            let partner_endpoint_access = self
                .committed_opened_cell_keys
                .get(&violation.partner_net_id)
                .map(|partner_opened| partner_opened.contains(&key))
                .unwrap_or(false);
            if !(current_endpoint_access || partner_endpoint_access) {
                return true;
            }
            let tolerance_um = self.grid.grid_size_um.max(1.0e-9);
            let current_is_endpoint = physical_point_near_centerline_endpoint(
                violation.point,
                route_centerline,
                tolerance_um,
            );
            let partner_is_endpoint = self
                .committed_realized_center_routes
                .get(&violation.partner_net_id)
                .map(|centerline| {
                    physical_point_near_centerline_endpoint(
                        violation.point,
                        centerline,
                        tolerance_um,
                    )
                })
                .unwrap_or(false);
            !(current_is_endpoint || partner_is_endpoint)
        });
    }

    /// True when `self.crossing_events` already has an event for the
    /// unordered pair {net_id, partner_id} whose recorded point lies within
    /// `tolerance_um` of `point`. Used to tell an intended, registered
    /// crossing apart from a bare geometric intersection that never went
    /// through the crossing-event machinery (see the call site below).
    pub(crate) fn has_registered_crossing_event(
        &self,
        net_id: u64,
        partner_id: u64,
        point: (f64, f64),
        tolerance_um: f64,
    ) -> bool {
        self.crossing_events.iter().any(|event| {
            let pair_matches = (event.net_id == net_id && event.partner_net_id == partner_id)
                || (event.net_id == partner_id && event.partner_net_id == net_id);
            if !pair_matches {
                return false;
            }
            let dx = event.point.0 - point.0;
            let dy = event.point.1 - point.1;
            (dx * dx + dy * dy).sqrt() <= tolerance_um
        })
    }

    /// `require_registered_events`: whether a geometrically legal
    /// intersection with no registered [`CrossingEvent`] on record for the
    /// pair is itself a violation (`missing_crossing_event`, B1 of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`).
    ///
    /// This must be `true` only in the post-commit context
    /// (`validate_committed_crossings_for_route_with_ports`, called after
    /// the route's own crossing events are already registered via
    /// `register_geometric_crossing_events_for_route`/`add_crossing_events`)
    /// and `false` in every pre-commit context (a search trying a candidate
    /// route, or `probe_net_for_repair`'s diagnostic probe): a route that
    /// has not committed yet has by definition not registered its events,
    /// so `self.crossing_events` can never contain them regardless of
    /// whether the crossing is legal -- checking there would misreport
    /// every legal, not-yet-committed crossing as a violation. Pre-commit
    /// callers already have their own view of which crossings a candidate
    /// route would register (`crossing_events`/`probe_crossing_events`).
    pub(crate) fn crossing_violations_for_realized_centerline(
        &self,
        net_id: u64,
        route_centerline: &[(f64, f64)],
        require_registered_events: bool,
    ) -> Vec<InvalidCrossingIntersection> {
        if !self.crossing_context.is_enabled() {
            return Vec::new();
        }
        let config = self.crossing_context.config();
        let required_margin =
            realized_crossing_margin_um(config.crossing_half_size_cells, self.grid.grid_size_um);
        if route_centerline.len() < 2 {
            return Vec::new();
        }
        // Bounding-box prefilter (same idea as the photonic verifier's
        // 2026-08-28 self-intersection prefilter): a partner whose whole
        // centerline stays farther than the waveguide width from this
        // route's bounding box can produce neither an intersection nor a
        // parallel-overlap violation, so it is skipped before the O(n*m)
        // segment-pair loop below ever sees it.
        let Some((route_min_x, route_min_y, route_max_x, route_max_y)) =
            polyline_bbox(route_centerline)
        else {
            return Vec::new();
        };
        let bbox_reach = self.route_width_um.max(1.0e-6);
        let partner_centerlines: Vec<(u64, Vec<(f64, f64)>)> = self
            .committed_center_routes
            .iter()
            .filter_map(|(partner_id, partner_grid_waypoints)| {
                if *partner_id == net_id {
                    return None;
                }
                let centerline = self
                    .committed_realized_center_routes
                    .get(partner_id)
                    .cloned()
                    .unwrap_or_else(|| self.grid_waypoints_to_centerline(partner_grid_waypoints));
                if centerline.len() < 2 {
                    return None;
                }
                let (p_min_x, p_min_y, p_max_x, p_max_y) = polyline_bbox(&centerline)?;
                if p_min_x > route_max_x + bbox_reach
                    || p_max_x < route_min_x - bbox_reach
                    || p_min_y > route_max_y + bbox_reach
                    || p_max_y < route_min_y - bbox_reach
                {
                    return None;
                }
                Some((*partner_id, centerline))
            })
            .collect();
        if partner_centerlines.is_empty() {
            return Vec::new();
        }
        let mut invalid = Vec::new();
        let mut seen = FxHashSet::default();
        for route_segment in route_centerline.windows(2) {
            let route_len = physical_segment_length(route_segment[0], route_segment[1]);
            if route_len <= 0.0 {
                continue;
            }
            for (partner_id, partner_centerline) in &partner_centerlines {
                for partner_segment in partner_centerline.windows(2) {
                    let partner_len =
                        physical_segment_length(partner_segment[0], partner_segment[1]);
                    if partner_len <= 0.0 {
                        continue;
                    }
                    if let Some((x, y)) = physical_collinear_segment_overlap_midpoint(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) {
                        let pair_key = (
                            net_id.min(*partner_id),
                            net_id.max(*partner_id),
                            (x * 1_000_000.0).round() as i64,
                            (y * 1_000_000.0).round() as i64,
                        );
                        if !seen.insert(pair_key) {
                            continue;
                        }
                        invalid.push(InvalidCrossingIntersection {
                            net_id,
                            partner_net_id: *partner_id,
                            point: (x, y),
                            reason: "collinear_route_overlap",
                        });
                        continue;
                    }
                    let Some((x, y, t, u)) = physical_segment_intersection_with_params(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        // No intersection. Near-parallel segments closer than
                        // the waveguide width still overlap as polygons: two
                        // routes on adjacent diagonal cells share no grid cell
                        // and never intersect, but their realized waveguides
                        // physically merge (multiportmmi_8x8 n_13/n_14 at
                        // heuristic weight 1.0). Only near-parallel pairs are
                        // tested so the chords around a legal perpendicular
                        // crossing stay exempt. Cheap bbox reject first: most
                        // pairs are nowhere near each other.
                        let seg_reach = self.route_width_um;
                        if route_segment[0].0.min(route_segment[1].0)
                            > partner_segment[0].0.max(partner_segment[1].0) + seg_reach
                            || route_segment[0].0.max(route_segment[1].0)
                                < partner_segment[0].0.min(partner_segment[1].0) - seg_reach
                            || route_segment[0].1.min(route_segment[1].1)
                                > partner_segment[0].1.max(partner_segment[1].1) + seg_reach
                            || route_segment[0].1.max(route_segment[1].1)
                                < partner_segment[0].1.min(partner_segment[1].1) - seg_reach
                        {
                            continue;
                        }
                        let route_dir = (
                            (route_segment[1].0 - route_segment[0].0) / route_len,
                            (route_segment[1].1 - route_segment[0].1) / route_len,
                        );
                        let partner_dir = (
                            (partner_segment[1].0 - partner_segment[0].0) / partner_len,
                            (partner_segment[1].1 - partner_segment[0].1) / partner_len,
                        );
                        let cross_sin =
                            (route_dir.0 * partner_dir.1 - route_dir.1 * partner_dir.0).abs();
                        if cross_sin < 0.5 {
                            let distance = segment_to_segment_distance(
                                route_segment[0],
                                route_segment[1],
                                partner_segment[0],
                                partner_segment[1],
                            );
                            if distance < self.route_width_um - 1.0e-6 {
                                let mid_x = (route_segment[0].0 + route_segment[1].0) / 2.0;
                                let mid_y = (route_segment[0].1 + route_segment[1].1) / 2.0;
                                // Coarse (1 um) dedupe key: a long parallel run
                                // yields one violation per micron, not one per
                                // sampled chord pair.
                                let pair_key = (
                                    net_id.min(*partner_id),
                                    net_id.max(*partner_id),
                                    (mid_x.round() as i64) * 1_000_000,
                                    (mid_y.round() as i64) * 1_000_000,
                                );
                                if seen.insert(pair_key) {
                                    invalid.push(InvalidCrossingIntersection {
                                        net_id,
                                        partner_net_id: *partner_id,
                                        point: (mid_x, mid_y),
                                        reason: "parallel_route_overlap",
                                    });
                                }
                            }
                        }
                        continue;
                    };
                    let pair_key = (
                        net_id.min(*partner_id),
                        net_id.max(*partner_id),
                        (x * 1_000_000.0).round() as i64,
                        (y * 1_000_000.0).round() as i64,
                    );
                    if !seen.insert(pair_key) {
                        continue;
                    }
                    let perpendicular = physical_segments_are_perpendicular(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    );
                    let route_margin = (t * route_len).min((1.0 - t) * route_len);
                    let partner_margin = (u * partner_len).min((1.0 - u) * partner_len);
                    if perpendicular
                        && route_margin + 1e-9 >= required_margin
                        && partner_margin + 1e-9 >= required_margin
                    {
                        // Geometrically legal is not enough in the
                        // post-commit context: a crossing is only real
                        // design intent when the two nets also registered a
                        // CrossingEvent for it. Without this, two diagonals
                        // that cross at a cell corner (no shared grid cell,
                        // no crossing structure in the GDS) pass silently --
                        // this is how try_commit_clean_probe committed 8
                        // bare intersections on benes_16x16
                        // (2026-09-14 13:10). Gated by
                        // `require_registered_events` (see this function's
                        // doc comment): a pre-commit caller's route has not
                        // registered anything yet, so this check would
                        // misfire on every legal crossing for it.
                        if require_registered_events {
                            let event_tolerance_um = 1.5 * self.grid.grid_size_um;
                            if self.has_registered_crossing_event(
                                net_id,
                                *partner_id,
                                (x, y),
                                event_tolerance_um,
                            ) {
                                continue;
                            }
                            invalid.push(InvalidCrossingIntersection {
                                net_id,
                                partner_net_id: *partner_id,
                                point: (x, y),
                                reason: "missing_crossing_event",
                            });
                            continue;
                        }
                        continue;
                    }
                    let reason = if !perpendicular {
                        "not_perpendicular"
                    } else {
                        "insufficient_straight_margin"
                    };
                    invalid.push(InvalidCrossingIntersection {
                        net_id,
                        partner_net_id: *partner_id,
                        point: (x, y),
                        reason,
                    });
                }
            }
        }
        invalid
    }

    pub(crate) fn crossing_footprint_unrelated_dynamic_owner(
        &self,
        net_id: u64,
        partner_id: u64,
        point: (f64, f64),
    ) -> Option<u64> {
        let (center_x, center_y) = self.grid_cell_for_physical_point(point)?;
        let config = self.crossing_context.config();
        let keys = crossing_reservation_window_keys(
            f64::from(center_x),
            f64::from(center_y),
            config.crossing_half_size_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        for key in keys {
            let (x, y) = unpack_xy(key);
            for owner in self.obstacle_map.dynamic_owners_for_cells(&[(x, y)]) {
                if owner != net_id && owner != partner_id {
                    return Some(owner);
                }
            }
        }
        None
    }

    /// The margin rule the unified predicate applies on this router: the
    /// crossing element's half size and the primitive bend radius as the
    /// run-out -- the two numbers the search's `CrossingSearchConfig`
    /// carries, so search and validation read one rule.
    pub(crate) fn crossing_legality_rule(&self) -> CrossingLegalityRule {
        CrossingLegalityRule {
            half_size_cells: self.crossing_context.config().crossing_half_size_cells,
            bend_runout_cells: self.primitive_cfg.bend_radius_cells,
        }
    }

    /// `footprint_blocked` of the unified predicate at a point in grid
    /// coordinates: a dynamic cell of a net other than the two crossing nets
    /// inside the crossing element's `half_size` window -- the obstacle-map
    /// form of the Python verifier's footprint-blocker check (another net's
    /// route geometry inside the component footprint). Static cells are not
    /// blockers here: after the commit the element's own reservation is
    /// static, and before it the search's reservation-window check already
    /// refused static cells.
    pub(crate) fn footprint_blocked_at_grid_point(
        &self,
        net_id: u64,
        partner_id: u64,
        x: f64,
        y: f64,
    ) -> bool {
        let keys = crossing_reservation_window_keys(
            x,
            y,
            self.crossing_context.config().crossing_half_size_cells,
            self.grid.width as i32,
            self.grid.height as i32,
        );
        keys.into_iter().any(|key| {
            let (cell_x, cell_y) = unpack_xy(key);
            self.obstacle_map
                .dynamic_owners_for_cells(&[(cell_x, cell_y)])
                .into_iter()
                .any(|owner| owner != net_id && owner != partner_id)
        })
    }

    /// The centre, in grid coordinates, a registered crossing's reservation
    /// is built around: the snapped cell of its point (`crossing_
    /// reservation_window_keys(center_x, center_y, ..)` in
    /// `unified_crossing_assessment` and `realized_crossing_events_for_route`).
    fn registered_crossing_centre(&self, event: &CrossingEvent) -> Option<(f64, f64)> {
        self.grid_cell_for_physical_point(event.point)
            .map(|(x, y)| (f64::from(x), f64::from(y)))
    }

    /// Slice B-2, the search's view: the centres of the crossing elements
    /// registered on `partner_id` with nets other than `net_id` (the
    /// `nearest_registered` input of the unified predicate in the search
    /// hook). Empty with the switch off, so the legacy search config is
    /// unchanged.
    pub(crate) fn registered_crossing_centres_on_partner(
        &self,
        net_id: u64,
        partner_id: u64,
    ) -> Vec<(f64, f64)> {
        if !self.router_config.crossing.unified_legality {
            return Vec::new();
        }
        self.crossing_events
            .iter()
            .filter(|event| {
                (event.net_id == partner_id || event.partner_net_id == partner_id)
                    && event.net_id != net_id
                    && event.partner_net_id != net_id
            })
            .filter_map(|event| self.registered_crossing_centre(event))
            .collect()
    }

    /// Slice B-2, the commit's and the validation's view: the centres of
    /// every crossing element registered by nets other than `net_id` (the
    /// route's own events, registered or not, are its same-route
    /// neighbours instead).
    fn registered_crossing_centres_of_other_nets(&self, net_id: u64) -> Vec<(f64, f64)> {
        self.crossing_events
            .iter()
            .filter(|event| event.net_id != net_id && event.partner_net_id != net_id)
            .filter_map(|event| self.registered_crossing_centre(event))
            .collect()
    }

    /// The grid route's intersection nearest `point_um`, within 1.5 cells,
    /// among the waypoint-segment pairs that carry the realized segments
    /// (each grid segment parallel to its realized segment): the two
    /// waypoint segments and the four straight runs in cells, as the search
    /// and the commit check measure them (a bend contributes its arms as
    /// straight cells; the rule's `bend_runout` pays for that). The
    /// parallelism filter keeps a neighbouring arm of a bend, which also
    /// intersects near the point but at another angle, from lending its
    /// short runs to the crossing (multiportmmi_8x8 net 110 x 111).
    fn grid_crossing_near(
        &self,
        route_waypoints: &[(i32, i32)],
        partner_waypoints: &[(i32, i32)],
        point_um: (f64, f64),
        route_direction: (f64, f64),
        partner_direction: (f64, f64),
    ) -> Option<GridCrossingNear> {
        if self.grid.grid_size_um <= 0.0 {
            return None;
        }
        let target = (
            (point_um.0 - self.grid.origin_x_um) / self.grid.grid_size_um - 0.5,
            (point_um.1 - self.grid.origin_y_um) / self.grid.grid_size_um - 0.5,
        );
        let parallel = |a: (i32, i32), b: (i32, i32), direction: (f64, f64)| {
            let grid_direction = (f64::from(b.0 - a.0), f64::from(b.1 - a.1));
            let cross = grid_direction.0 * direction.1 - grid_direction.1 * direction.0;
            let scale = grid_direction.0.hypot(grid_direction.1) * direction.0.hypot(direction.1);
            scale > 0.0 && cross.abs() <= 1e-6 * scale
        };
        let mut best: Option<(f64, GridCrossingNear)> = None;
        for route_segment in route_waypoints.windows(2) {
            let route_len = segment_length_cells(route_segment[0], route_segment[1]);
            if route_len <= 0.0 || !parallel(route_segment[0], route_segment[1], route_direction) {
                continue;
            }
            for partner_segment in partner_waypoints.windows(2) {
                let partner_len = segment_length_cells(partner_segment[0], partner_segment[1]);
                if partner_len <= 0.0
                    || !parallel(partner_segment[0], partner_segment[1], partner_direction)
                {
                    continue;
                }
                let Some((x, y, t, u)) = segment_intersection_with_params(
                    route_segment[0],
                    route_segment[1],
                    partner_segment[0],
                    partner_segment[1],
                ) else {
                    continue;
                };
                let distance = ((x - target.0).powi(2) + (y - target.1).powi(2)).sqrt();
                if distance > 1.5 || best.as_ref().is_some_and(|(d, _)| distance >= *d) {
                    continue;
                }
                best = Some((
                    distance,
                    GridCrossingNear {
                        route_segment: (route_segment[0], route_segment[1]),
                        partner_segment: (partner_segment[0], partner_segment[1]),
                        route_before_cells: t * route_len,
                        route_after_cells: (1.0 - t) * route_len,
                        partner_before_cells: u * partner_len,
                        partner_after_cells: (1.0 - u) * partner_len,
                    },
                ));
            }
        }
        best.map(|(_, near)| near)
    }

    /// The unified crossing assessment of one route (phase 4 slice B): every
    /// proper intersection of its realized centerline with a committed
    /// partner's, judged by `crossing_legality` with this router's rule, plus
    /// the collinear / parallel overlaps. The directions are the realized
    /// segments', the margins the grid route's (`grid_crossing_near`; the
    /// realized straights in cells when the grid has no intersection near
    /// the point), `footprint_blocked` reads the obstacle map, and
    /// `event_registered` is looked up only when `require_registered_events`
    /// (post-commit; a route that has not committed has registered nothing).
    /// Registration keeps the legal assessments' events; validation turns
    /// the illegal ones into violations; both see one verdict per point.
    pub(crate) fn unified_crossing_assessment(
        &self,
        net_id: u64,
        route_centerline: &[(f64, f64)],
        route_grid_waypoints: &[(i32, i32)],
        require_registered_events: bool,
    ) -> UnifiedCrossingAssessment {
        let mut out = UnifiedCrossingAssessment::default();
        if !self.crossing_context.is_enabled() || route_centerline.len() < 2 {
            return out;
        }
        let rule = self.crossing_legality_rule();
        let half_size_cells = self.crossing_context.config().crossing_half_size_cells;
        let Some((route_min_x, route_min_y, route_max_x, route_max_y)) =
            polyline_bbox(route_centerline)
        else {
            return out;
        };
        let bbox_reach = self.route_width_um.max(1.0e-6);
        let mut partner_ids: Vec<u64> = self
            .committed_center_routes
            .keys()
            .copied()
            .filter(|partner_id| *partner_id != net_id)
            .collect();
        partner_ids.sort_unstable();
        let partners: Vec<(u64, Vec<(f64, f64)>)> = partner_ids
            .into_iter()
            .filter_map(|partner_id| {
                let centerline = self
                    .committed_realized_center_routes
                    .get(&partner_id)
                    .cloned()
                    .unwrap_or_else(|| {
                        self.grid_waypoints_to_centerline(
                            &self.committed_center_routes[&partner_id],
                        )
                    });
                if centerline.len() < 2 {
                    return None;
                }
                let (p_min_x, p_min_y, p_max_x, p_max_y) = polyline_bbox(&centerline)?;
                if p_min_x > route_max_x + bbox_reach
                    || p_max_x < route_min_x - bbox_reach
                    || p_min_y > route_max_y + bbox_reach
                    || p_max_y < route_min_y - bbox_reach
                {
                    return None;
                }
                Some((partner_id, centerline))
            })
            .collect();
        if partners.is_empty() {
            return out;
        }
        let grid_size_um = self.grid.grid_size_um.max(1.0e-9);
        let event_tolerance_um = 1.5 * self.grid.grid_size_um;
        let mut seen = FxHashSet::default();
        for route_segment in route_centerline.windows(2) {
            let route_len = physical_segment_length(route_segment[0], route_segment[1]);
            if route_len <= 0.0 {
                continue;
            }
            for (partner_id, partner_centerline) in &partners {
                let partner_grid_waypoints = &self.committed_center_routes[partner_id];
                for partner_segment in partner_centerline.windows(2) {
                    let partner_len =
                        physical_segment_length(partner_segment[0], partner_segment[1]);
                    if partner_len <= 0.0 {
                        continue;
                    }
                    if let Some((x, y)) = physical_collinear_segment_overlap_midpoint(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) {
                        let pair_key = (
                            net_id.min(*partner_id),
                            net_id.max(*partner_id),
                            (x * 1_000_000.0).round() as i64,
                            (y * 1_000_000.0).round() as i64,
                        );
                        if seen.insert(pair_key) {
                            out.overlaps.push(InvalidCrossingIntersection {
                                net_id,
                                partner_net_id: *partner_id,
                                point: (x, y),
                                reason: "collinear_route_overlap",
                            });
                        }
                        continue;
                    }
                    let Some((x, y, t, u)) = physical_segment_intersection_with_params(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    ) else {
                        // Near-parallel segments closer than the waveguide
                        // width merge as polygons without intersecting (same
                        // rule as the legacy validator).
                        let seg_reach = self.route_width_um;
                        if route_segment[0].0.min(route_segment[1].0)
                            > partner_segment[0].0.max(partner_segment[1].0) + seg_reach
                            || route_segment[0].0.max(route_segment[1].0)
                                < partner_segment[0].0.min(partner_segment[1].0) - seg_reach
                            || route_segment[0].1.min(route_segment[1].1)
                                > partner_segment[0].1.max(partner_segment[1].1) + seg_reach
                            || route_segment[0].1.max(route_segment[1].1)
                                < partner_segment[0].1.min(partner_segment[1].1) - seg_reach
                        {
                            continue;
                        }
                        let route_dir = (
                            (route_segment[1].0 - route_segment[0].0) / route_len,
                            (route_segment[1].1 - route_segment[0].1) / route_len,
                        );
                        let partner_dir = (
                            (partner_segment[1].0 - partner_segment[0].0) / partner_len,
                            (partner_segment[1].1 - partner_segment[0].1) / partner_len,
                        );
                        let cross_sin =
                            (route_dir.0 * partner_dir.1 - route_dir.1 * partner_dir.0).abs();
                        if cross_sin < 0.5
                            && segment_to_segment_distance(
                                route_segment[0],
                                route_segment[1],
                                partner_segment[0],
                                partner_segment[1],
                            ) < self.route_width_um - 1.0e-6
                        {
                            let mid_x = (route_segment[0].0 + route_segment[1].0) / 2.0;
                            let mid_y = (route_segment[0].1 + route_segment[1].1) / 2.0;
                            let pair_key = (
                                net_id.min(*partner_id),
                                net_id.max(*partner_id),
                                (mid_x.round() as i64) * 1_000_000,
                                (mid_y.round() as i64) * 1_000_000,
                            );
                            if seen.insert(pair_key) {
                                out.overlaps.push(InvalidCrossingIntersection {
                                    net_id,
                                    partner_net_id: *partner_id,
                                    point: (mid_x, mid_y),
                                    reason: "parallel_route_overlap",
                                });
                            }
                        }
                        continue;
                    };
                    let pair_key = (
                        net_id.min(*partner_id),
                        net_id.max(*partner_id),
                        (x * 1_000_000.0).round() as i64,
                        (y * 1_000_000.0).round() as i64,
                    );
                    if !seen.insert(pair_key) {
                        continue;
                    }
                    let point = (x, y);
                    let route_direction = (
                        route_segment[1].0 - route_segment[0].0,
                        route_segment[1].1 - route_segment[0].1,
                    );
                    let partner_direction = (
                        partner_segment[1].0 - partner_segment[0].0,
                        partner_segment[1].1 - partner_segment[0].1,
                    );
                    let near = self.grid_crossing_near(
                        route_grid_waypoints,
                        partner_grid_waypoints,
                        point,
                        route_direction,
                        partner_direction,
                    );
                    let (route_grid_segment, partner_grid_segment, margins, margins_from_grid) =
                        match near {
                            Some(near) => (
                                Some(near.route_segment),
                                Some(near.partner_segment),
                                (
                                    near.route_before_cells,
                                    near.route_after_cells,
                                    near.partner_before_cells,
                                    near.partner_after_cells,
                                ),
                                true,
                            ),
                            None => (
                                self.physical_segment_to_grid_segment(
                                    route_segment[0],
                                    route_segment[1],
                                ),
                                self.physical_segment_to_grid_segment(
                                    partner_segment[0],
                                    partner_segment[1],
                                ),
                                (
                                    t * route_len / grid_size_um,
                                    (1.0 - t) * route_len / grid_size_um,
                                    u * partner_len / grid_size_um,
                                    (1.0 - u) * partner_len / grid_size_um,
                                ),
                                false,
                            ),
                        };
                    // The centre the element's reservation is built around:
                    // the snapped cell of the point (the event's window
                    // below and `registered_crossing_centre` agree on it);
                    // the exact grid position when the point is off the map.
                    let centre_cell = self.grid_cell_for_physical_point(point);
                    let grid_point = centre_cell
                        .map(|(cx, cy)| (f64::from(cx), f64::from(cy)))
                        .unwrap_or((
                            (point.0 - self.grid.origin_x_um) / grid_size_um - 0.5,
                            (point.1 - self.grid.origin_y_um) / grid_size_um - 0.5,
                        ));
                    let candidate = CrossingCandidate {
                        route_direction,
                        partner_direction,
                        route_before_cells: margins.0,
                        route_after_cells: margins.1,
                        partner_before_cells: margins.2,
                        partner_after_cells: margins.3,
                        point: grid_point,
                        nearest_same_route: None,
                        nearest_registered: None,
                        footprint_blocked: self
                            .crossing_footprint_unrelated_dynamic_owner(net_id, *partner_id, point)
                            .is_some(),
                        event_registered: require_registered_events.then(|| {
                            self.has_registered_crossing_event(
                                net_id,
                                *partner_id,
                                point,
                                event_tolerance_um,
                            )
                        }),
                    };
                    let event = match (route_grid_segment, partner_grid_segment, centre_cell) {
                        (Some(route_grid), Some(partner_grid), Some((center_x, center_y))) => {
                            match (
                                direction_angle_between_cells(route_grid.0, route_grid.1),
                                direction_angle_between_cells(partner_grid.0, partner_grid.1),
                            ) {
                                (Some(route_angle), Some(partner_angle)) => Some(CrossingEvent {
                                    net_id,
                                    partner_net_id: *partner_id,
                                    point,
                                    route_segment: route_grid,
                                    partner_segment: partner_grid,
                                    route_angle,
                                    partner_angle,
                                    reservation_keys: crossing_reservation_window_keys(
                                        f64::from(center_x),
                                        f64::from(center_y),
                                        half_size_cells,
                                        self.grid.width as i32,
                                        self.grid.height as i32,
                                    ),
                                }),
                                _ => None,
                            }
                        }
                        _ => None,
                    };
                    out.crossings.push(CrossingAssessment {
                        event,
                        record: CrossingVerdictRecord {
                            net_id,
                            partner_net_id: *partner_id,
                            point,
                            route_segment: route_grid_segment,
                            partner_segment: partner_grid_segment,
                            candidate,
                            rule,
                            // Judged below, once every intersection of the
                            // route is known (slice B-2: the same-route
                            // neighbours).
                            verdict: CrossingVerdict::Legal,
                            margins_from_grid,
                        },
                    });
                }
            }
        }
        // Slice B-2: the spacing inputs and the verdicts. A crossing's
        // same-route neighbours are the route's other intersections (their
        // reservation centres); its registered neighbours the elements of
        // other nets already on the map. The route's own events, when they
        // are registered (post-commit), are its same-route neighbours, not
        // registered ones.
        let registered = self.registered_crossing_centres_of_other_nets(net_id);
        let centres: Vec<(f64, f64)> = out
            .crossings
            .iter()
            .map(|crossing| crossing.record.candidate.point)
            .collect();
        for crossing in &mut out.crossings {
            let candidate = &mut crossing.record.candidate;
            candidate.nearest_same_route = nearest_crossing_neighbour(
                candidate.point,
                rule.half_size_cells,
                centres.iter().copied(),
            );
            candidate.nearest_registered = nearest_crossing_neighbour(
                candidate.point,
                rule.half_size_cells,
                registered.iter().copied(),
            );
            crossing.record.verdict = crossing_legality(candidate, &rule);
        }
        out
    }

    /// The violations of an assessment: every crossing the predicate did not
    /// call legal (reason = the verdict's), then the overlaps.
    pub(crate) fn unified_violations(
        assessment: &UnifiedCrossingAssessment,
    ) -> Vec<InvalidCrossingIntersection> {
        assessment
            .crossings
            .iter()
            .filter(|crossing| !crossing.record.verdict.is_legal())
            .map(|crossing| InvalidCrossingIntersection {
                net_id: crossing.record.net_id,
                partner_net_id: crossing.record.partner_net_id,
                point: crossing.record.point,
                reason: crossing.record.verdict.reason(),
            })
            .chain(assessment.overlaps.iter().cloned())
            .collect()
    }

    /// Replaces this net's verdict records with the assessment's (the
    /// records of a pair belong to the net that committed last; a rip-up of
    /// either net drops them in `remove_crossing_events_for_net`).
    pub(crate) fn record_crossing_verdicts(
        &mut self,
        net_id: u64,
        assessment: &UnifiedCrossingAssessment,
    ) {
        self.crossing_verdicts
            .retain(|record| record.net_id != net_id);
        self.crossing_verdicts.extend(
            assessment
                .crossings
                .iter()
                .map(|crossing| crossing.record.clone()),
        );
    }

    pub(crate) fn validate_committed_crossings_for_route_with_ports(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Result<(), String> {
        if self.router_config.crossing.disable_rust_crossing_validation {
            return Ok(());
        }
        // Post-commit context: this net's own crossing events were already
        // registered before this call at every one of this function's
        // call sites, so a geometrically legal intersection with no
        // matching event is a real bug, not a pre-commit search artifact.
        let violations = if self.router_config.crossing.unified_legality {
            match self.routing_centerline_for_route(route, source_port_um, target_port_um) {
                Ok(route_centerline) => {
                    let assessment = self.unified_crossing_assessment(
                        net_id,
                        &route_centerline,
                        &route.compressed_waypoints,
                        true,
                    );
                    self.record_crossing_verdicts(net_id, &assessment);
                    if self.router_config.diagnostics.crossing_mismatch_dump {
                        for crossing in &assessment.crossings {
                            if !crossing.record.verdict.is_legal() {
                                eprintln!(
                                    "crossing-verdict net={} partner={} point=({:.3},{:.3}) {:?} {:?} route_segment={:?} partner_segment={:?} margins_from_grid={}",
                                    net_id,
                                    crossing.record.partner_net_id,
                                    crossing.record.point.0,
                                    crossing.record.point.1,
                                    crossing.record.verdict,
                                    crossing.record.candidate,
                                    crossing.record.route_segment,
                                    crossing.record.partner_segment,
                                    crossing.record.margins_from_grid,
                                );
                            }
                        }
                    }
                    let mut violations = Self::unified_violations(&assessment);
                    self.retain_violations_outside_endpoint_access(
                        &route_centerline,
                        opened_cell_keys,
                        &mut violations,
                    );
                    violations
                }
                Err(_) => Vec::new(),
            }
        } else {
            self.crossing_violations_for_route_with_ports(
                net_id,
                route,
                source_port_um,
                target_port_um,
                opened_cell_keys,
                true,
            )
        };
        if violations.is_empty() {
            return Ok(());
        }
        self.dump_crossing_mismatch(net_id, route, source_port_um, target_port_um, &violations);
        if self.router_config.diagnostics.crossing_mismatch_fatal {
            panic!(
                "crossing mismatch fatal (net {}): the crossing-aware search returned a route \
                 that fails realized-crossing validation; first violation: net {} intersects \
                 net {} at ({:.3}, {:.3}) ({})",
                net_id,
                violations[0].net_id,
                violations[0].partner_net_id,
                violations[0].point.0,
                violations[0].point.1,
                violations[0].reason
            );
        }
        let violation = &violations[0];
        Err(format!(
            "Illegal realized crossing: net {} intersects net {} at ({:.3}, {:.3}) ({})",
            violation.net_id,
            violation.partner_net_id,
            violation.point.0,
            violation.point.1,
            violation.reason
        ))
    }

    pub(crate) fn register_geometric_crossing_events_for_route(
        &mut self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) {
        if !self.crossing_context.is_enabled() {
            return;
        }
        if self.router_config.crossing.unified_legality {
            // Phase 4 slice B: the events are the legal assessments of the
            // one predicate (pre-commit view: no event can be registered
            // yet). The grid centerline stands in when the realized one
            // cannot be built, as the legacy grid fallback did.
            let route_centerline = self
                .routing_centerline_for_route(route, source_port_um, target_port_um)
                .unwrap_or_else(|_| self.grid_waypoints_to_centerline(&route.compressed_waypoints));
            let assessment = self.unified_crossing_assessment(
                net_id,
                &route_centerline,
                &route.compressed_waypoints,
                false,
            );
            let events: Vec<CrossingEvent> = assessment
                .crossings
                .into_iter()
                .filter(|crossing| crossing.record.verdict.is_legal())
                .filter_map(|crossing| crossing.event)
                .collect();
            self.add_crossing_events(events);
            return;
        }
        let partner_ids = self.crossing_partner_lookup_set_for_result(net_id, route);
        if partner_ids.is_empty() {
            return;
        }
        let mut crossing_events = self.realized_crossing_events_for_route(
            net_id,
            route,
            &partner_ids,
            source_port_um,
            target_port_um,
        );
        if crossing_events.is_empty() {
            crossing_events = self.crossing_events_for_route(net_id, route, &partner_ids);
        }
        if crossing_events.is_empty() {
            return;
        }
        self.add_crossing_events(crossing_events);
    }

    pub(crate) fn crossing_reservation_keys_for_events(
        events: &[CrossingEvent],
    ) -> FxHashSet<CellKey> {
        let mut keys = FxHashSet::default();
        for event in events {
            keys.extend(event.reservation_keys.iter().copied());
        }
        keys
    }

    pub(crate) fn remove_crossing_events_for_net(&mut self, net_id: u64) {
        if !self.crossing_verdicts.is_empty() {
            self.crossing_verdicts
                .retain(|record| record.net_id != net_id && record.partner_net_id != net_id);
        }
        if self.crossing_events.is_empty() {
            return;
        }
        let mut remaining = Vec::with_capacity(self.crossing_events.len());
        let mut removed_keys = FxHashSet::default();
        for event in self.crossing_events.drain(..) {
            if event.net_id == net_id || event.partner_net_id == net_id {
                removed_keys.extend(event.reservation_keys.iter().copied());
            } else {
                remaining.push(event);
            }
        }
        self.crossing_events = remaining;
        if removed_keys.is_empty() {
            return;
        }
        self.obstacle_map.remove_static_keys(&removed_keys);
        for key in removed_keys {
            let (x, y) = unpack_xy(key);
            if !self.obstacle_map.is_static_blocked(x, y) {
                self.static_cells.remove(&key);
            }
        }
        self.invalidate_meander_base_prefix();
    }

    pub(crate) fn add_crossing_events(&mut self, events: Vec<CrossingEvent>) {
        if events.is_empty() {
            return;
        }
        let reservation_keys = Self::crossing_reservation_keys_for_events(&events);
        if !reservation_keys.is_empty() {
            self.obstacle_map.add_static_keys(&reservation_keys);
            self.static_cells.extend(reservation_keys);
            self.invalidate_meander_base_prefix();
        }
        self.crossing_events.extend(events);
    }

    #[allow(clippy::too_many_arguments)]
    /// Committed nets (other than `net_id`) whose realized centerline the
    /// route's centerline intersects (proper crossing or collinear overlap),
    /// regardless of the crossing context -- the geometric core of
    /// `crossing_violations_for_realized_centerline` without its legality
    /// classification. Empty when the route has no centerline.
    pub(crate) fn committed_partners_intersecting_route(
        &self,
        net_id: u64,
        route: &RouteResult,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Vec<u64> {
        let Ok(route_centerline) =
            self.routing_centerline_for_route(route, source_port_um, target_port_um)
        else {
            return Vec::new();
        };
        if route_centerline.len() < 2 {
            return Vec::new();
        }
        let Some((route_min_x, route_min_y, route_max_x, route_max_y)) =
            polyline_bbox(&route_centerline)
        else {
            return Vec::new();
        };
        let bbox_reach = self.route_width_um.max(1.0e-6);
        let mut partners: Vec<u64> = Vec::new();
        let mut ids: Vec<&u64> = self.committed_center_routes.keys().collect();
        ids.sort_unstable();
        for partner_id in ids {
            if *partner_id == net_id {
                continue;
            }
            let centerline = self
                .committed_realized_center_routes
                .get(partner_id)
                .cloned()
                .unwrap_or_else(|| {
                    self.grid_waypoints_to_centerline(&self.committed_center_routes[partner_id])
                });
            if centerline.len() < 2 {
                continue;
            }
            let Some((p_min_x, p_min_y, p_max_x, p_max_y)) = polyline_bbox(&centerline) else {
                continue;
            };
            if p_min_x > route_max_x + bbox_reach
                || p_max_x < route_min_x - bbox_reach
                || p_min_y > route_max_y + bbox_reach
                || p_max_y < route_min_y - bbox_reach
            {
                continue;
            }
            let intersects = route_centerline.windows(2).any(|route_segment| {
                centerline.windows(2).any(|partner_segment| {
                    physical_collinear_segment_overlap_midpoint(
                        route_segment[0],
                        route_segment[1],
                        partner_segment[0],
                        partner_segment[1],
                    )
                    .is_some()
                        || physical_segment_intersection_with_params(
                            route_segment[0],
                            route_segment[1],
                            partner_segment[0],
                            partner_segment[1],
                        )
                        .is_some()
                })
            });
            if intersects {
                partners.push(*partner_id);
            }
        }
        partners
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;
    use crate::search::state::State;

    /// The grid-level and the event-level crossing check together, as the
    /// commit path applies them (`invalid_grid_crossing_error_for_route`
    /// followed by `crossing_events_satisfy_partner_constraints`). Lives here
    /// because only these tests need the two as one predicate.
    fn route_satisfies_partner_constraints(
        router: &PyPhotonicRouter,
        net_id: u64,
        route: &RouteResult,
        partner_ids: &FxHashSet<u64>,
        crossing_events: &[CrossingEvent],
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> bool {
        router
            .invalid_crossing_intersections_for_route(net_id, route, partner_ids)
            .is_empty()
            && router.crossing_events_satisfy_partner_constraints(
                net_id,
                partner_ids,
                crossing_events,
                opened_cell_keys,
            )
    }

    #[test]
    fn crossing_conflict_fixture_blocks_the_vertical_net_with_a_non_perpendicular_crossing() {
        let (mut router, jobs) = crossing_conflict_fixture();
        let vertical = jobs[3].clone();
        assert_eq!(vertical.net_id, 4);
        let order_by_id: FxHashMap<u64, usize> = jobs
            .iter()
            .enumerate()
            .map(|(index, job)| (job.net_id, index))
            .collect();

        let mut batch = RepairBatchState {
            final_routes: FxHashMap::default(),
            attempts: Vec::new(),
            repair_trace: Vec::new(),
            repair_count: 0,
            failed_net_id: None,
            failed_error: None,
            timings: NativeBatchTimings::default(),
        };
        // probe_net_for_repair only checks *membership* of an owner id in
        // `batch.final_routes`, never the stored RouteResult itself, to
        // decide whether a net the search actually hit in the (real,
        // already-committed) obstacle map counts as a candidate blocker --
        // see `lidar_probe_partner_lookup_set` and its `add_candidate_blocker`
        // caller. The fixture's three nets are truly committed on
        // `router.obstacle_map`; this batch only needs their ids present.
        for net_id in [1u64, 2, 3] {
            batch.final_routes.insert(net_id, empty_test_route());
        }

        let plain_outcome = router.try_plain_normal_route(
            &mut batch,
            &SearchCall::plain(
                &vertical,
                SearchRadii::new(
                    FIXTURE_CLEARANCE_RADIUS_CELLS,
                    Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                    Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                ),
            ),
            false,
        );
        assert!(
            matches!(plain_outcome, PlainRouteOutcome::NotResolved),
            "the vertical net must not find a legal plain route: every path from \
             (30, 0) to (30, 59) has to cross the diagonal net 3 non-perpendicularly"
        );

        let probe = router
            .probe_net_for_repair(
                &mut batch,
                &vertical,
                &order_by_id,
                &test_search_args(fixture_radii(FIXTURE_CLEARANCE_RADIUS_CELLS), true),
            )
            .expect("probe search itself must succeed (ignoring dynamic obstacles)");

        assert!(
            probe.candidate_blockers.contains(&3),
            "diagonal net 3 must be reported as a candidate blocker: {:?}",
            probe.candidate_blockers
        );
        assert!(
            probe
                .probe_realized_crossing_violations
                .iter()
                .any(|violation| violation.partner_net_id == 3
                    && violation.reason == "not_perpendicular"),
            "expected a not_perpendicular realized crossing violation against net 3: {:?}",
            probe
                .probe_realized_crossing_violations
                .iter()
                .map(|violation| (violation.partner_net_id, violation.reason))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn intersection_without_crossing_event_is_a_violation() {
        let mut router = missing_crossing_event_fixture_router();
        install_diagonal_partner(&mut router, 1);
        let centerline = crossing_diagonal_centerline();

        // require_registered_events = true: this is the post-commit
        // context the check is meant for (coordinator decision,
        // 2026-09-14 -- see the doc comment on
        // `crossing_violations_for_realized_centerline`).
        let violations = router.crossing_violations_for_realized_centerline(2, &centerline, true);

        assert_eq!(
            violations.len(),
            1,
            "expected exactly one violation, got {:?}",
            violations
                .iter()
                .map(|v| (v.partner_net_id, v.reason, v.point))
                .collect::<Vec<_>>()
        );
        let violation = &violations[0];
        assert_eq!(violation.reason, "missing_crossing_event");
        assert_eq!(violation.partner_net_id, 1);
        assert!(
            (violation.point.0 - 30.0).abs() < 1.0e-6 && (violation.point.1 - 30.0).abs() < 1.0e-6,
            "expected the intersection at (30, 30), got {:?}",
            violation.point
        );
    }

    #[test]
    fn intersection_with_a_registered_crossing_event_is_accepted() {
        let mut router = missing_crossing_event_fixture_router();
        install_diagonal_partner(&mut router, 1);
        let centerline = crossing_diagonal_centerline();

        router.add_crossing_events(vec![CrossingEvent {
            net_id: 1,
            partner_net_id: 2,
            point: (30.0, 30.0),
            route_segment: ((10, 10), (50, 50)),
            partner_segment: ((10, 50), (50, 10)),
            route_angle: 1,
            partner_angle: 3,
            reservation_keys: FxHashSet::default(),
        }]);

        let violations = router.crossing_violations_for_realized_centerline(2, &centerline, true);

        assert!(
            violations.is_empty(),
            "a registered crossing event must clear the intersection with no other reason \
             surfacing either: {:?}",
            violations
                .iter()
                .map(|v| (v.partner_net_id, v.reason, v.point))
                .collect::<Vec<_>>()
        );
    }

    /// 2026-09-15 00:30 owner decision (a): `probe_net_for_repair`'s
    /// crossing reconstruction must be geometric
    /// (`realized_crossing_events_for_route`), not grid-waypoint-based
    /// (`crossing_events_for_route`), because the grid reconstruction can
    /// miss a real intersection. This is not a quantization artifact of
    /// the route's own geometry (both functions read the same `route`) --
    /// it is the two functions' different required-crossing-margin
    /// formulas: `crossing_events_for_partner`'s grid margin reserves a
    /// full bend runout (`crossing_required_margin_cells` adds
    /// `bend_radius_cells`), while `realized_crossing_events_for_route`'s
    /// physical margin (`realized_crossing_margin_um`) does not. A crossing
    /// that lands exactly on a cell corner between two one-cell-long
    /// 45-degree diagonals -- both segments meeting exactly at their own
    /// midpoint, (30.5, 30.5), the corner shared by cells (30,30)/(30,31)/
    /// (31,30)/(31,31) -- has only half a cell of margin on each segment:
    /// enough for the physical check (margin >= 0) but short of the grid
    /// check's required 1 cell (`crossing_half_size_cells` 0 +
    /// `bend_radius_cells` 1). Tested directly on this corner geometry, as
    /// the brief permits, rather than through a full probe search: driving
    /// a real probe to reproduce this exact margin shortfall would need a
    /// route within one cell of a committed partner at the moment they
    /// cross, which is a much larger fixture to construct than the
    /// underlying margin-formula mismatch this test isolates.
    #[test]
    fn realized_crossing_events_finds_a_corner_crossing_the_grid_reconstruction_misses() {
        let mut router = missing_crossing_event_fixture_router();
        // Two one-cell 45-degree diagonals meeting exactly at their shared
        // midpoint corner (30.5, 30.5), not at any cell center.
        router
            .committed_center_routes
            .insert(1, vec![(30, 31), (31, 30)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(30.0, 31.0), (31.0, 30.0)]);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(30, 30), (31, 31)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(31, 31, 0),
            reached_target: State::new(31, 31, 0),
            stats: RouteSearchStats::default(),
        };
        let partner_ids: FxHashSet<u64> = std::iter::once(1).collect();

        let grid_events = router.crossing_events_for_route(2, &route, &partner_ids);
        assert!(
            grid_events.is_empty(),
            "the grid-waypoint reconstruction's bend-runout margin should reject this \
             half-cell-margin corner crossing: {grid_events:?}"
        );

        let realized_events =
            router.realized_crossing_events_for_route(2, &route, &partner_ids, None, None);
        assert_eq!(
            realized_events.len(),
            1,
            "the geometric reconstruction should find the corner crossing the grid \
             reconstruction missed: {realized_events:?}"
        );
        assert_eq!(realized_events[0].partner_net_id, 1);
    }

    #[test]
    fn second_vertical_job_blocked_by_diagonal_also_fails_plain_search() {
        let (mut router, _jobs) = crossing_conflict_fixture();
        let second_vertical = second_vertical_job_blocked_by_diagonal();

        let mut batch = fresh_repair_batch_state();
        for net_id in [1u64, 2, 3] {
            batch.final_routes.insert(net_id, empty_test_route());
        }

        let plain_outcome = router.try_plain_normal_route(
            &mut batch,
            &SearchCall::plain(
                &second_vertical,
                SearchRadii::new(
                    FIXTURE_CLEARANCE_RADIUS_CELLS,
                    Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                    Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                ),
            ),
            false,
        );
        assert!(
            matches!(plain_outcome, PlainRouteOutcome::NotResolved),
            "the second vertical job (net 5, x=40) must also be blocked by the diagonal \
             net 3, the same as net 4 (x=30)"
        );
    }

    #[test]
    fn crossing_events_require_straight_margin_around_intersection() {
        let partner = vec![(10, 5), (10, 24)];
        let clean = crossing_events_for_partner(2, 1, &[(3, 12), (24, 12)], &partner, 2, 0, 32, 32);
        assert_eq!(clean.len(), 1);
        assert_eq!(clean[0].point, (10.0, 12.0));

        let bend_endpoint = crossing_events_for_partner(
            2,
            1,
            &[(3, 12), (10, 12), (10, 20)],
            &partner,
            2,
            0,
            32,
            32,
        );
        assert!(bend_endpoint.is_empty());
    }

    #[test]
    fn crossing_events_allow_bend_after_crossing_runout() {
        let partner = vec![(8, 5), (8, 24)];
        let route = vec![(3, 12), (13, 12), (16, 15)];

        let events = crossing_events_for_partner(2, 1, &route, &partner, 2, 3, 32, 32);

        assert_eq!(events.len(), 1);
        assert_eq!(events[0].point, (8.0, 12.0));
    }

    /// Phase 4 slice B: the unified assessment on the diagonal fixture.
    /// Net 2's 135-degree centerline crosses the 45-degree partner (net 1)
    /// at (30, 30); before the event is registered the verdict is
    /// `MissingEvent`, afterwards `Legal`, and the margins come from the
    /// grid route (20 cells on every side of a 40-cell diagonal).
    #[test]
    fn unified_assessment_reports_missing_event_until_the_event_is_registered() {
        let mut router = missing_crossing_event_fixture_router();
        router.router_config.crossing.unified_legality = true;
        install_diagonal_partner(&mut router, 1);
        let centerline = crossing_diagonal_centerline();
        let waypoints = vec![(10, 50), (50, 10)];

        let before = router.unified_crossing_assessment(2, &centerline, &waypoints, true);
        assert_eq!(before.crossings.len(), 1, "one intersection at (30, 30)");
        assert!(before.overlaps.is_empty());
        let record = &before.crossings[0].record;
        assert_eq!(record.verdict, CrossingVerdict::MissingEvent);
        assert!(record.margins_from_grid);
        assert!((record.candidate.route_before_cells - 20.0).abs() < 1e-9);
        assert!((record.candidate.partner_after_cells - 20.0).abs() < 1e-9);
        assert_eq!(record.candidate.event_registered, Some(false));
        let violations = PyPhotonicRouter::unified_violations(&before);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].reason, "missing_event");

        let event = before.crossings[0]
            .event
            .clone()
            .expect("the assessment derives the grid event");
        assert_eq!((event.route_angle, event.partner_angle), (7, 1));
        router.add_crossing_events(vec![event]);
        let after = router.unified_crossing_assessment(2, &centerline, &waypoints, true);
        assert_eq!(after.crossings[0].record.verdict, CrossingVerdict::Legal);
        assert!(PyPhotonicRouter::unified_violations(&after).is_empty());
        // Pre-commit callers never ask about events.
        let pre_commit = router.unified_crossing_assessment(2, &centerline, &waypoints, false);
        assert_eq!(
            pre_commit.crossings[0].record.candidate.event_registered,
            None
        );
        assert!(pre_commit.crossings[0].record.verdict.is_legal());
    }

    /// Phase 4 slice B: under the switch, registration keeps the legal
    /// assessment's event, the post-commit validation records the verdict
    /// for `crossing_verdicts()`, and a rip-up drops both.
    #[test]
    fn unified_registration_and_validation_share_one_verdict() {
        let mut router = missing_crossing_event_fixture_router();
        router.router_config.crossing.unified_legality = true;
        install_diagonal_partner(&mut router, 1);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(10, 50), (50, 10)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(50, 10, 7),
            reached_target: State::new(50, 10, 7),
            stats: RouteSearchStats::default(),
        };

        router.register_geometric_crossing_events_for_route(2, &route, None, None);
        assert_eq!(router.crossing_events.len(), 1);
        assert_eq!(router.crossing_events[0].partner_net_id, 1);
        assert!(
            router.crossing_verdicts.is_empty(),
            "registration records nothing"
        );

        router
            .validate_committed_crossings_for_route_with_ports(2, &route, None, None, None)
            .expect("the registered crossing is legal");
        assert_eq!(router.crossing_verdicts.len(), 1);
        let record = &router.crossing_verdicts[0];
        assert_eq!((record.net_id, record.partner_net_id), (2, 1));
        assert!(record.verdict.is_legal());
        assert_eq!(record.candidate.event_registered, Some(true));
        assert_eq!(record.rule.required_margin_cells(), 1);

        // Without the event the same route fails validation with the
        // predicate's reason, and the record says so too.
        router.remove_crossing_events_for_net(2);
        assert!(
            router.crossing_verdicts.is_empty(),
            "the rip-up drops the record"
        );
        let error = router
            .validate_committed_crossings_for_route_with_ports(2, &route, None, None, None)
            .expect_err("a crossing without an event is a violation");
        assert!(error.contains("missing_event"), "{error}");
        assert_eq!(router.crossing_verdicts.len(), 1);
        assert_eq!(
            router.crossing_verdicts[0].verdict,
            CrossingVerdict::MissingEvent
        );
    }

    /// Phase 4 slice B: the commit-time grid check under the switch names
    /// the short side; the legacy check rejects the same route under its
    /// own name. Half size 2 and bend radius 1: margin 3 before and on the
    /// partner, 2 after.
    #[test]
    fn unified_grid_check_names_the_short_side_the_legacy_check_also_rejects() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(10, 5), (10, 24)]);
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            // Two cells of straight before the partner at x = 10: one short
            // of the margin 3.
            compressed_waypoints: vec![(8, 12), (24, 12)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(24, 12, 0),
            reached_target: State::new(24, 12, 0),
            stats: RouteSearchStats::default(),
        };

        let legacy = router.invalid_crossing_intersections_for_route(2, &route, &partner_ids);
        assert_eq!(legacy.len(), 1);
        assert_eq!(legacy[0].reason, "insufficient_straight_margin");

        router.router_config.crossing.unified_legality = true;
        let unified = router.invalid_crossing_intersections_for_route(2, &route, &partner_ids);
        assert_eq!(unified.len(), 1);
        assert_eq!(unified[0].reason, "margin_short_route_before");
        assert_eq!(unified[0].point, (10.0, 12.0));

        // Three cells before: exactly at the limit, legal in both modes.
        let at_limit = RouteResult {
            compressed_waypoints: vec![(7, 12), (24, 12)],
            ..route
        };
        assert!(router
            .invalid_crossing_intersections_for_route(2, &at_limit, &partner_ids)
            .is_empty());
    }

    /// Slice B-2: two perpendicular partners four cells apart give the
    /// route two grid crossings whose 5x5 windows (half 2) share a column;
    /// the unified commit check calls the second one `too_close_same_route`
    /// (the legacy check has no spacing term of its own -- the collision
    /// path rejects the same route through
    /// `crossing_events_have_disjoint_reservations`). Five cells apart both
    /// are legal.
    #[test]
    fn unified_grid_check_rejects_two_crossings_of_one_route_closer_than_the_spacing() {
        let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        router.router_config.crossing.unified_legality = true;
        router
            .committed_center_routes
            .insert(1, vec![(14, 5), (14, 34)]);
        router
            .committed_center_routes
            .insert(3, vec![(18, 5), (18, 34)]);
        let partner_ids: FxHashSet<u64> = [1u64, 3u64].into_iter().collect();
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(2, 20), (36, 20)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(36, 20, 0),
            reached_target: State::new(36, 20, 0),
            stats: RouteSearchStats::default(),
        };

        let invalid = router.invalid_crossing_intersections_for_route(2, &route, &partner_ids);
        assert_eq!(invalid.len(), 2, "{invalid:?}");
        assert!(invalid
            .iter()
            .all(|violation| violation.reason == "too_close_same_route"));
        let mut points: Vec<(f64, f64)> = invalid.iter().map(|v| v.point).collect();
        points.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(points, vec![(14.0, 20.0), (18.0, 20.0)]);

        // Five cells apart: windows [12, 16] and [17, 21] are disjoint.
        router
            .committed_center_routes
            .insert(3, vec![(19, 5), (19, 34)]);
        assert!(router
            .invalid_crossing_intersections_for_route(2, &route, &partner_ids)
            .is_empty());

        // A registered crossing element of two other nets three cells from
        // the route's crossing of net 1 is `too_close_registered`; the same
        // element five cells away is not.
        let registered = |point: (f64, f64)| CrossingEvent {
            net_id: 7,
            partner_net_id: 1,
            point,
            route_segment: ((5, 17), (30, 17)),
            partner_segment: ((14, 5), (14, 34)),
            route_angle: 0,
            partner_angle: 2,
            reservation_keys: FxHashSet::default(),
        };
        router.add_crossing_events(vec![registered((14.0, 17.0))]);
        let invalid = router.invalid_crossing_intersections_for_route(2, &route, &partner_ids);
        assert_eq!(invalid.len(), 1, "{invalid:?}");
        assert_eq!(invalid[0].reason, "too_close_registered");
        assert_eq!(invalid[0].point, (14.0, 20.0));
        router.remove_crossing_events_for_net(7);
        router.add_crossing_events(vec![registered((14.0, 15.0))]);
        assert!(router
            .invalid_crossing_intersections_for_route(2, &route, &partner_ids)
            .is_empty());
    }

    /// Slice B-2, post-commit: the unified assessment judges a realized
    /// crossing against the other nets' registered elements; the route's
    /// own registered events are its same-route neighbours (the one at the
    /// point itself is skipped, so a legal single crossing stays legal after
    /// its own registration).
    #[test]
    fn unified_assessment_reports_a_registered_component_in_the_window() {
        let mut router = missing_crossing_event_fixture_router();
        router.router_config.crossing.unified_legality = true;
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        install_diagonal_partner(&mut router, 1);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(10, 50), (50, 10)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(50, 10, 7),
            reached_target: State::new(50, 10, 7),
            stats: RouteSearchStats::default(),
        };
        router.register_geometric_crossing_events_for_route(2, &route, None, None);
        assert_eq!(router.crossing_events.len(), 1);
        router
            .validate_committed_crossings_for_route_with_ports(2, &route, None, None, None)
            .expect("the route's own event is not a neighbour");
        let record = &router.crossing_verdicts[0];
        assert!(record.verdict.is_legal());
        assert_eq!(record.candidate.nearest_same_route, None);
        assert_eq!(record.candidate.nearest_registered, None);

        // Another net's element on the partner, three cells along the
        // diagonal from (30, 30): its window shares cells with the route's.
        router.add_crossing_events(vec![CrossingEvent {
            net_id: 9,
            partner_net_id: 1,
            point: (33.0, 33.0),
            route_segment: ((13, 53), (53, 13)),
            partner_segment: ((10, 10), (50, 50)),
            route_angle: 7,
            partner_angle: 1,
            reservation_keys: FxHashSet::default(),
        }]);
        let error = router
            .validate_committed_crossings_for_route_with_ports(2, &route, None, None, None)
            .expect_err("the registered element is inside the window");
        assert!(error.contains("too_close_registered"), "{error}");
        let record = &router.crossing_verdicts[0];
        assert_eq!(
            record.verdict,
            CrossingVerdict::TooClose {
                other: CrossingNeighbour::Registered,
                have_cells: 3.0,
                need_cells: 5.0,
            }
        );
        assert_eq!(record.candidate.nearest_registered_cells(), Some(3.0));
    }

    #[test]
    fn invalid_crossing_intersections_block_kink_crossings() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(10, 5), (10, 24)]);
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);

        let clean_route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (24, 12)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(24, 12, 0),
            reached_target: State::new(24, 12, 0),
            stats: RouteSearchStats::default(),
        };
        assert!(router
            .invalid_crossing_intersections_for_route(2, &clean_route, &partner_ids)
            .is_empty());

        let kink_route = RouteResult {
            compressed_waypoints: vec![(3, 12), (10, 12), (10, 20)],
            ..clean_route
        };
        let invalid = router.invalid_crossing_intersections_for_route(2, &kink_route, &partner_ids);
        assert_eq!(invalid.len(), 1);
        assert_eq!(invalid[0].partner_net_id, 1);
    }

    #[test]
    fn committed_crossing_validation_rejects_expected_bend_crossing() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(10, 5), (10, 24)]);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (10, 12), (10, 20)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(10, 20, 2),
            reached_target: State::new(10, 20, 2),
            stats: RouteSearchStats::default(),
        };

        let error = router
            .validate_committed_crossings_for_route_with_ports(2, &route, None, None, None)
            .unwrap_err();
        assert!(error.contains("Illegal realized crossing"));
        assert!(error.contains("insufficient_straight_margin"));
    }

    #[test]
    fn committed_crossing_validation_rejects_opened_cell_crossing_away_from_endpoint() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 4,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(10.5, 0.5), (10.5, 20.5)]);
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(7, 10), (20, 10)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(20, 10, 0),
            reached_target: State::new(20, 10, 0),
            stats: RouteSearchStats::default(),
        };
        let opened_cell_keys: FxHashSet<CellKey> =
            [(10, 10)].into_iter().map(|(x, y)| pack_xy(x, y)).collect();

        // Post-commit validation context (this test's sibling above calls
        // `validate_committed_crossings_for_route_with_ports` directly;
        // this one drops to the lower-level helper for a more specific
        // scenario), so `require_registered_events = true`.
        let violations = router.crossing_violations_for_route_with_ports(
            2,
            &route,
            None,
            None,
            Some(&opened_cell_keys),
            true,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].partner_net_id, 1);
        assert_eq!(violations[0].reason, "insufficient_straight_margin");
    }

    #[test]
    fn realized_crossing_validation_rejects_collinear_route_overlap() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(0, 5), (12, 5)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(0.0, 5.0), (12.0, 5.0)]);

        let violations =
            router.crossing_violations_for_realized_centerline(2, &[(4.0, 5.0), (16.0, 5.0)], true);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].partner_net_id, 1);
        assert_eq!(violations[0].reason, "collinear_route_overlap");
    }

    #[test]
    fn realized_crossing_validation_rejects_lidar_pure_angle_and_margin() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(0, 10), (20, 10)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(0.0, 10.0), (20.0, 10.0)]);

        let not_perpendicular = router.crossing_violations_for_realized_centerline(
            2,
            &[(0.0, 0.0), (20.0, 20.0)],
            true,
        );

        assert_eq!(not_perpendicular.len(), 1);
        assert_eq!(not_perpendicular[0].partner_net_id, 1);
        assert_eq!(not_perpendicular[0].reason, "not_perpendicular");

        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(10.0, 0.0), (10.0, 20.0)]);
        let insufficient_margin = router.crossing_violations_for_realized_centerline(
            2,
            &[(9.0, 10.0), (20.0, 10.0)],
            true,
        );

        assert_eq!(insufficient_margin.len(), 1);
        assert_eq!(insufficient_margin[0].partner_net_id, 1);
        assert_eq!(
            insufficient_margin[0].reason,
            "insufficient_straight_margin"
        );
    }

    /// Owner-requested check (2026-09-03, .agent/execplans/2026-09-03-eager-diagonal-crossing-insertion.md):
    /// after a crossing the search now inserts exactly `crossing_half_size_cells`
    /// of pure straight (the crossing element's own extent) and then lets a
    /// bend follow. This pins, at the realized level, that a 90-degree arc
    /// starting two cells after the crossing point yields a valid crossing
    /// (the crossing lies on a straight segment with margin >= half_size on
    /// both sides), while an arc starting one cell after it does not.
    #[test]
    fn realized_crossing_accepts_bend_two_cells_after_crossing_and_rejects_one() {
        fn route_with_bend_after(cells_after_crossing: usize) -> Vec<(f64, f64)> {
            // Horizontal from x=0 through the crossing at x=10, straight for
            // `cells_after_crossing`, then a quarter circle of radius 3
            // turning north (sampled like the realizer does), then vertical.
            let bend_start_x = 10.0 + cells_after_crossing as f64;
            let radius = 3.0;
            let mut points = vec![(0.0, 10.0), (bend_start_x, 10.0)];
            let samples = 12;
            for step in 1..=samples {
                let theta = std::f64::consts::FRAC_PI_2 * step as f64 / samples as f64;
                points.push((
                    bend_start_x + radius * theta.sin(),
                    10.0 + radius * (1.0 - theta.cos()),
                ));
            }
            points.push((bend_start_x + radius, 25.0));
            points
        }

        let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 3, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        // Partner: vertical through x=10, long enough for its own margin.
        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 30)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(10.0, 0.0), (10.0, 30.0)]);

        // Pre-commit-style call (`require_registered_events = false`): this
        // test exercises bend/margin legality only, not the post-commit
        // missing-crossing-event check (B1 of
        // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`),
        // so no crossing event is registered here.
        let valid =
            router.crossing_violations_for_realized_centerline(2, &route_with_bend_after(2), false);
        assert!(
            valid.is_empty(),
            "two pure straight cells after the crossing point, then a bend, must realize a valid crossing; got {valid:?}"
        );

        let invalid =
            router.crossing_violations_for_realized_centerline(2, &route_with_bend_after(1), false);
        assert_eq!(
            invalid.len(),
            1,
            "one straight cell after the crossing is inside the crossing element"
        );
        assert_eq!(invalid[0].partner_net_id, 1);
        assert_eq!(invalid[0].reason, "insufficient_straight_margin");
    }

    /// Post-search grid-level check (`invalid_crossing_intersections_for_route`)
    /// must apply the same rule as the search kernel and the realized
    /// validator: after the crossing point the route needs `half_size` pure
    /// straight cells (Point 2), not `half_size + bend_radius`. Found on
    /// benes_32x32 net 273 (2026-09-03): the search accepted a crossing 3.5
    /// cells before a 45-degree corner, the realized centerline was clean,
    /// and this check discarded the whole route with
    /// `insufficient_straight_margin` because it demanded 5.
    #[test]
    fn grid_crossing_check_accepts_half_size_straight_after_crossing_before_a_bend() {
        let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 3, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 30)]);
        let partner_ids: FxHashSet<u64> = [1u64].into_iter().collect();
        let route_with_corner_at = |corner_x: i32| RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            // eastbound straight from x=0 crossing the vertical partner at
            // x=10, then a 45-degree corner at `corner_x` and a diagonal
            compressed_waypoints: vec![(0, 10), (corner_x, 10), (corner_x + 8, 18)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(corner_x + 8, 18, 1),
            reached_target: State::new(corner_x + 8, 18, 1),
            stats: RouteSearchStats::default(),
        };

        // 3 cells after the crossing point: >= half_size (2) -> legal, as the
        // kernel and the realized validator already say
        let invalid = router.invalid_crossing_intersections_for_route(
            2,
            &route_with_corner_at(13),
            &partner_ids,
        );
        assert!(
            invalid.is_empty(),
            "3 straight cells after the crossing must pass the grid check; got {invalid:?}"
        );
        // exactly half_size: still legal
        let invalid = router.invalid_crossing_intersections_for_route(
            2,
            &route_with_corner_at(12),
            &partner_ids,
        );
        assert!(
            invalid.is_empty(),
            "2 straight cells after the crossing must pass the grid check; got {invalid:?}"
        );
        // 1 cell: inside the crossing element -> invalid
        let invalid = router.invalid_crossing_intersections_for_route(
            2,
            &route_with_corner_at(11),
            &partner_ids,
        );
        assert_eq!(invalid.len(), 1);
        assert_eq!(invalid[0].reason, "insufficient_straight_margin");
        // before the crossing the kernel still counts required_margin (5):
        // a route starting 3 cells before the partner is rejected there too
        let short_before = RouteResult {
            compressed_waypoints: vec![(7, 10), (20, 10)],
            ..route_with_corner_at(13)
        };
        let invalid =
            router.invalid_crossing_intersections_for_route(2, &short_before, &partner_ids);
        assert_eq!(
            invalid.len(),
            1,
            "3 cells before the crossing are less than required_margin"
        );
    }

    #[test]
    fn collision_crossing_route_without_event_is_not_accepted() {
        let grid = PyGridSpec::new(64, 64, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(20, 20), (30, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(20.0, 20.0), (30.0, 20.0)]);

        let opened = FxHashSet::default();
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);
        let result = router
            .try_route_with_collision_crossings(
                2,
                State::new(0, 0, 0),
                State::new(12, 0, 0),
                &opened,
                &router.astar_config(None, None, None, None, None).unwrap(),
                0,
                None,
                &partner_ids,
                None,
                None,
                None,
                None,
                false,
            )
            .unwrap();

        assert!(
            result.is_none(),
            "collision-crossing helper must not accept routes with zero crossing events \
             unless the caller opts into clean-zero-event acceptance"
        );

        let accepted = router
            .try_route_with_collision_crossings(
                2,
                State::new(0, 0, 0),
                State::new(12, 0, 0),
                &opened,
                &router.astar_config(None, None, None, None, None).unwrap(),
                0,
                None,
                &partner_ids,
                None,
                None,
                None,
                None,
                true,
            )
            .unwrap();
        let (_, events) = accepted.expect(
            "with accept_clean_zero_event the same clean crossing-free route \
             must be returned as an ordinary route (owner decision 2026-09-02)",
        );
        assert!(
            events.is_empty(),
            "the accepted clean route must carry no crossing events"
        );
    }

    #[test]
    fn realized_crossing_validation_defers_lidar_pure_footprint_blocker() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context = CrossingContext::new(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        router
            .committed_center_routes
            .insert(1, vec![(10, 0), (10, 20)]);
        router
            .committed_realized_center_routes
            .insert(1, vec![(10.0, 0.0), (10.0, 20.0)]);
        assert!(router.obstacle_map.commit_route(3, &[(10, 10)]));

        // Pre-commit-style call (`require_registered_events = false`):
        // this test exercises footprint-blocker deferral only, not B1's
        // post-commit missing-crossing-event check, so no crossing event
        // is registered here.
        let violations = router.crossing_violations_for_realized_centerline(
            2,
            &[(0.0, 10.0), (20.0, 10.0)],
            false,
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn crossing_events_reject_overlapping_reservation_footprints() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });

        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (24, 12)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(24, 12, 0),
            reached_target: State::new(24, 12, 0),
            stats: RouteSearchStats::default(),
        };
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);
        partner_ids.insert(2);

        let mut first_reservation = FxHashSet::default();
        first_reservation.insert(pack_xy(10, 12));
        first_reservation.insert(pack_xy(11, 12));
        let mut second_reservation = FxHashSet::default();
        second_reservation.insert(pack_xy(11, 12));
        second_reservation.insert(pack_xy(12, 12));
        let events = vec![
            CrossingEvent {
                net_id: 3,
                partner_net_id: 1,
                point: (10.0, 12.0),
                route_segment: ((3, 12), (24, 12)),
                partner_segment: ((10, 5), (10, 24)),
                route_angle: 0,
                partner_angle: 2,
                reservation_keys: first_reservation,
            },
            CrossingEvent {
                net_id: 3,
                partner_net_id: 2,
                point: (12.0, 12.0),
                route_segment: ((3, 12), (24, 12)),
                partner_segment: ((12, 5), (12, 24)),
                route_angle: 0,
                partner_angle: 2,
                reservation_keys: second_reservation,
            },
        ];

        assert!(!PyPhotonicRouter::crossing_events_have_disjoint_reservations(&events));
        assert!(!route_satisfies_partner_constraints(
            &router,
            3,
            &route,
            &partner_ids,
            &events,
            None,
        ));
        assert_eq!(
            PyPhotonicRouter::crossing_partners_with_overlapping_reservations(&events).len(),
            2
        );
    }

    #[test]
    fn crossing_events_reject_static_and_unrelated_dynamic_reservation_blockers() {
        let grid = PyGridSpec::new(32, 32, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 1, 4, 1, true),
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
            ),
            None,
        );
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            crossing_half_size_cells: 2,
            ..CrossingConfig::default()
        });
        let route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: Vec::new(),
            compressed_waypoints: vec![(3, 12), (24, 12)],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(24, 12, 0),
            reached_target: State::new(24, 12, 0),
            stats: RouteSearchStats::default(),
        };
        let mut partner_ids = FxHashSet::default();
        partner_ids.insert(1);
        let mut reservation = FxHashSet::default();
        reservation.insert(pack_xy(10, 12));
        reservation.insert(pack_xy(10, 13));
        let events = vec![CrossingEvent {
            net_id: 3,
            partner_net_id: 1,
            point: (10.0, 12.0),
            route_segment: ((3, 12), (24, 12)),
            partner_segment: ((10, 5), (10, 24)),
            route_angle: 0,
            partner_angle: 2,
            reservation_keys: reservation,
        }];

        assert!(route_satisfies_partner_constraints(
            &router,
            3,
            &route,
            &partner_ids,
            &events,
            None,
        ));

        router.obstacle_map.add_static_cells(&[(10, 13)]);
        assert!(!route_satisfies_partner_constraints(
            &router,
            3,
            &route,
            &partner_ids,
            &events,
            None,
        ));
        let opened_static = [pack_xy(10, 13)].into_iter().collect();
        assert!(route_satisfies_partner_constraints(
            &router,
            3,
            &route,
            &partner_ids,
            &events,
            Some(&opened_static),
        ));
        let static_cleanup = [pack_xy(10, 13)].into_iter().collect();
        router.obstacle_map.remove_static_keys(&static_cleanup);

        assert!(router.obstacle_map.commit_route(4, &[(10, 13)]));
        let blockers = router.crossing_reservation_blockers(3, &events, None);
        assert!(!blockers.is_clear());
        assert!(blockers.dynamic_blockers.contains(&4));
        assert!(!route_satisfies_partner_constraints(
            &router,
            3,
            &route,
            &partner_ids,
            &events,
            Some(&opened_static),
        ));
    }

    #[test]
    fn polylines_closer_than_sees_parallel_guides_only_when_they_overlap() {
        // Two vertical 0.5 um guides: at 2.0 um center spacing they are clear,
        // at 0.5 um (the benes_16x16 nets 132/134 case) they overlap.
        let a = vec![(0.0, 0.0), (0.0, 100.0)];
        let far = vec![(2.0, 0.0), (2.0, 100.0)];
        let near = vec![(0.5, 0.0), (0.5, 100.0)];
        let region = (-3.0, 40.0, 3.0, 60.0);
        assert!(!polylines_closer_than(&a, &far, 0.5, region));
        assert!(!polylines_closer_than(&a, &near, 0.5, region));
        assert!(polylines_closer_than(&a, &near, 0.5 + 1.0e-6, region));
        assert!(polylines_closer_than(
            &a,
            &[(0.3, 0.0), (0.3, 100.0)],
            0.5,
            region
        ));
    }

    #[test]
    fn polylines_closer_than_only_looks_inside_the_region() {
        let a = vec![(0.0, 0.0), (0.0, 100.0)];
        let crossing = vec![(-5.0, 90.0), (5.0, 90.0)];
        assert!(polylines_closer_than(
            &a,
            &crossing,
            0.5,
            (-3.0, 85.0, 3.0, 95.0)
        ));
        assert!(!polylines_closer_than(
            &a,
            &crossing,
            0.5,
            (-3.0, 40.0, 3.0, 60.0)
        ));
    }

    #[test]
    fn segment_to_segment_distance_handles_crossing_and_disjoint_segments() {
        assert_eq!(
            segment_to_segment_distance((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)),
            0.0
        );
        let d = segment_to_segment_distance((0.0, 0.0), (1.0, 0.0), (3.0, 0.0), (4.0, 0.0));
        assert!((d - 2.0).abs() < 1.0e-9);
        let d = segment_to_segment_distance((0.0, 0.0), (1.0, 0.0), (0.5, 1.0), (0.5, 3.0));
        assert!((d - 1.0).abs() < 1.0e-9);
    }
}
