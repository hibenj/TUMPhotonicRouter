//! Fast route-to-polygon realization.
//!
//! This module converts a routed state/primitive sequence into one closed
//! waveguide polygon in physical coordinates.

use std::error::Error;
use std::fmt;

use crate::astar::{RouteResult, State};
use crate::primitives::{PrimitiveGeometry, PrimitiveLibrary};
use crate::static_obstacle_builder::{
    grid_cell_center, physical_to_grid, PortInput, StaticGridSpec,
};

const EPS: f64 = 1.0e-9;
const MITER_LIMIT: f64 = 4.0;
const DEFAULT_BEND_SAMPLES_PER_90_DEG: usize = 16;

/// Minimal grid information required to convert grid cells to physical points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeometryGridSpec {
    pub grid_size_um: f64,
    pub origin_x_um: f64,
    pub origin_y_um: f64,
}

impl GeometryGridSpec {
    pub fn new(
        grid_size_um: f64,
        origin_x_um: f64,
        origin_y_um: f64,
    ) -> Result<Self, GeometryError> {
        if !grid_size_um.is_finite() || grid_size_um <= 0.0 {
            return Err(GeometryError::InvalidGridSize(grid_size_um));
        }
        if !origin_x_um.is_finite() || !origin_y_um.is_finite() {
            return Err(GeometryError::NonFiniteCoordinate);
        }

        Ok(Self {
            grid_size_um,
            origin_x_um,
            origin_y_um,
        })
    }

    pub fn cell_center(&self, x: i32, y: i32) -> (f64, f64) {
        (
            self.origin_x_um + (x as f64 + 0.5) * self.grid_size_um,
            self.origin_y_um + (y as f64 + 0.5) * self.grid_size_um,
        )
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum GeometryError {
    InvalidGridSize(f64),
    InvalidWidth(f64),
    EmptyRoute,
    DegenerateRoute,
    InvalidRouteTopology {
        states: usize,
        primitives: usize,
    },
    MissingPrimitive {
        id: u16,
        start_angle: u8,
    },
    ZeroLengthSegment,
    NonFiniteCoordinate,
    RouteStartDoesNotMatchSourceAnchor {
        route_start: (f64, f64),
        source_anchor: (f64, f64),
    },
    RouteEndDoesNotMatchTargetAnchor {
        route_end: (f64, f64),
        target_anchor: (f64, f64),
    },
    PrimitiveEndpointMismatch {
        primitive_id: u16,
        expected: (f64, f64),
        actual: (f64, f64),
    },
    PortAccess(PortAccessError),
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeometryError::InvalidGridSize(value) => {
                write!(f, "grid_size_um must be finite and > 0, got {value}")
            }
            GeometryError::InvalidWidth(value) => {
                write!(f, "width_um must be finite and > 0, got {value}")
            }
            GeometryError::EmptyRoute => write!(f, "route has no states"),
            GeometryError::DegenerateRoute => {
                write!(f, "route must contain at least two distinct centerline points")
            }
            GeometryError::InvalidRouteTopology { states, primitives } => write!(
                f,
                "invalid route topology: expected states = primitives + 1, got states={states}, primitives={primitives}"
            ),
            GeometryError::MissingPrimitive { id, start_angle } => write!(
                f,
                "route references unknown primitive id={id} for start_angle={start_angle}"
            ),
            GeometryError::ZeroLengthSegment => write!(f, "centerline contains a zero-length segment"),
            GeometryError::NonFiniteCoordinate => write!(f, "geometry contains a non-finite coordinate"),
            GeometryError::RouteStartDoesNotMatchSourceAnchor {
                route_start,
                source_anchor,
            } => write!(
                f,
                "route start point {route_start:?} does not match source anchor point {source_anchor:?}"
            ),
            GeometryError::RouteEndDoesNotMatchTargetAnchor {
                route_end,
                target_anchor,
            } => write!(
                f,
                "route end point {route_end:?} does not match target anchor point {target_anchor:?}"
            ),
            GeometryError::PrimitiveEndpointMismatch {
                primitive_id,
                expected,
                actual,
            } => write!(
                f,
                "primitive id={primitive_id} endpoint mismatch: expected {expected:?}, got {actual:?}"
            ),
            GeometryError::PortAccess(err) => write!(f, "{err}"),
        }
    }
}

impl Error for GeometryError {}

impl From<PortAccessError> for GeometryError {
    fn from(value: PortAccessError) -> Self {
        GeometryError::PortAccess(value)
    }
}

/// Configuration for deterministic local port-access geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PortAccessConfig {
    pub min_straight_um: f64,
    pub max_anchor_search_cells: i32,
    pub min_bend_radius_um: f64,
}

impl Default for PortAccessConfig {
    fn default() -> Self {
        Self {
            min_straight_um: 0.0,
            max_anchor_search_cells: 8,
            min_bend_radius_um: 0.0,
        }
    }
}

/// Deterministic pre-routing connector from physical port to grid anchor cell.
#[derive(Clone, Debug, PartialEq)]
pub struct PortAccess {
    pub port_name: String,
    pub port_point_um: (f64, f64),
    pub anchor_cell: (i32, i32),
    pub anchor_point_um: (f64, f64),
    pub port_angle: u8,
    pub anchor_angle: u8,
    /// Compatibility alias; equals `anchor_angle`.
    pub entry_angle: u8,
    pub access_centerline_um: Vec<(f64, f64)>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PortAccessError {
    NonFinitePortCoordinate {
        port_name: String,
    },
    InvalidConfig(String),
    AnchorOutOfBounds {
        port_name: String,
        anchor_cell: (i32, i32),
    },
    AnchorSearchFailed {
        port_name: String,
    },
    ZeroLengthAccess {
        port_name: String,
    },
    NonFiniteAccessCoordinate {
        port_name: String,
    },
}

impl fmt::Display for PortAccessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PortAccessError::NonFinitePortCoordinate { port_name } => {
                write!(f, "port '{port_name}' has non-finite physical coordinates")
            }
            PortAccessError::InvalidConfig(msg) => write!(f, "invalid port-access config: {msg}"),
            PortAccessError::AnchorOutOfBounds {
                port_name,
                anchor_cell,
            } => write!(
                f,
                "port '{port_name}' selected out-of-bounds anchor cell {anchor_cell:?}"
            ),
            PortAccessError::AnchorSearchFailed { port_name } => {
                write!(f, "failed to find valid anchor cell for port '{port_name}'")
            }
            PortAccessError::ZeroLengthAccess { port_name } => {
                write!(f, "port '{port_name}' access connector has zero length")
            }
            PortAccessError::NonFiniteAccessCoordinate { port_name } => write!(
                f,
                "port '{port_name}' access connector contains non-finite coordinates"
            ),
        }
    }
}

impl Error for PortAccessError {}

/// Compress route states by keeping only heading-change points.
pub fn compress_route_waypoints(states: &[State]) -> Vec<(i32, i32)> {
    if states.is_empty() {
        return Vec::new();
    }

    let mut waypoints = Vec::with_capacity(states.len());
    waypoints.push((states[0].x, states[0].y));

    for i in 1..states.len() {
        if states[i].angle != states[i - 1].angle {
            push_if_different(&mut waypoints, (states[i].x, states[i].y));
        }
    }

    if let Some(last) = states.last() {
        push_if_different(&mut waypoints, (last.x, last.y));
    }

    waypoints
}

/// Build a grid-cell polyline by following each primitive footprint in order.
pub fn route_to_grid_path(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
) -> Result<Vec<(i32, i32)>, GeometryError> {
    if route.states.is_empty() {
        return Err(GeometryError::EmptyRoute);
    }
    if route.states.len() != route.primitives.len() + 1 {
        return Err(GeometryError::InvalidRouteTopology {
            states: route.states.len(),
            primitives: route.primitives.len(),
        });
    }

    let mut path = Vec::new();
    push_if_different(&mut path, (route.states[0].x, route.states[0].y));

    for (index, primitive_id) in route.primitives.iter().enumerate() {
        let origin = route.states[index];
        let primitive = primitives
            .get_primitives_for_angle(origin.angle)
            .iter()
            .find(|p| p.id == *primitive_id)
            .ok_or(GeometryError::MissingPrimitive {
                id: *primitive_id,
                start_angle: origin.angle,
            })?;

        for (dx, dy) in primitive.footprint.iter().copied() {
            let cell = (origin.x + dx, origin.y + dy);
            push_if_different(&mut path, cell);
        }
    }

    let last_state = route.states[route.states.len() - 1];
    push_if_different(&mut path, (last_state.x, last_state.y));

    if path.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    Ok(path)
}

/// Compress a grid-cell polyline by keeping only heading-change points.
pub fn compress_grid_waypoints(path: &[(i32, i32)]) -> Vec<(i32, i32)> {
    if path.is_empty() {
        return Vec::new();
    }
    if path.len() == 1 {
        return vec![path[0]];
    }

    let mut waypoints = Vec::with_capacity(path.len());
    push_if_different(&mut waypoints, path[0]);

    let mut prev_dir = direction(path[0], path[1]);
    for i in 2..path.len() {
        let curr_dir = direction(path[i - 1], path[i]);
        if curr_dir != prev_dir {
            push_if_different(&mut waypoints, path[i - 1]);
        }
        prev_dir = curr_dir;
    }

    push_if_different(&mut waypoints, path[path.len() - 1]);
    waypoints
}

/// Convert ordered grid cells to physical centerline points at cell centers.
pub fn grid_path_to_centerline(
    path: &[(i32, i32)],
    grid: &GeometryGridSpec,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let mut centerline = Vec::with_capacity(path.len());

    for &(x, y) in path {
        let point = grid.cell_center(x, y);
        if !is_finite_point(point) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        push_physical_if_different(&mut centerline, point);
    }

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    Ok(centerline)
}

/// Convert a routed primitive sequence to a physical centerline by replaying
/// primitive geometry metadata.
pub fn route_to_primitive_centerline(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    if route.states.is_empty() {
        return Err(GeometryError::EmptyRoute);
    }
    if route.states.len() != route.primitives.len() + 1 {
        return Err(GeometryError::InvalidRouteTopology {
            states: route.states.len(),
            primitives: route.primitives.len(),
        });
    }

    let mut centerline = Vec::with_capacity(route.states.len());
    let start = grid.cell_center(route.states[0].x, route.states[0].y);
    if !is_finite_point(start) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    push_physical_if_different(&mut centerline, start);

    for (index, primitive_id) in route.primitives.iter().enumerate() {
        let state = route.states[index];
        let next_state = route.states[index + 1];
        let primitive = primitives
            .get_primitives_for_angle(state.angle)
            .iter()
            .find(|p| p.id == *primitive_id)
            .ok_or(GeometryError::MissingPrimitive {
                id: *primitive_id,
                start_angle: state.angle,
            })?;

        let expected = grid.cell_center(next_state.x, next_state.y);
        if !is_finite_point(expected) {
            return Err(GeometryError::NonFiniteCoordinate);
        }

        match primitive.geometry {
            PrimitiveGeometry::Straight { .. } => {
                push_physical_if_different(&mut centerline, expected);
            }
            PrimitiveGeometry::Bend {
                radius_um,
                angle_delta,
            } => {
                let start_point = *centerline.last().ok_or(GeometryError::DegenerateRoute)?;
                append_circular_bend_centerline(
                    &mut centerline,
                    start_point,
                    state.angle,
                    expected,
                    next_state.angle,
                    radius_um,
                    angle_delta,
                    DEFAULT_BEND_SAMPLES_PER_90_DEG,
                )?;
            }
        }

        let actual = *centerline.last().ok_or(GeometryError::DegenerateRoute)?;
        if distance(actual, expected) > EPS {
            return Err(GeometryError::PrimitiveEndpointMismatch {
                primitive_id: primitive.id,
                expected,
                actual,
            });
        }
        if !is_finite_point(actual) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        push_physical_if_different(&mut centerline, actual);
    }

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }

    Ok(centerline)
}

/// Replace first/last centerline points with explicit source/target physical points.
pub fn snap_centerline_endpoints(
    centerline: &mut Vec<(f64, f64)>,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    if let Some(source) = source_port_um {
        if !is_finite_point(source) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        centerline[0] = source;
    }
    if let Some(target) = target_port_um {
        if !is_finite_point(target) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let last = centerline.len() - 1;
        centerline[last] = target;
    }

    let mut deduped = Vec::with_capacity(centerline.len());
    for point in centerline.iter().copied() {
        push_physical_if_different(&mut deduped, point);
    }
    *centerline = deduped;

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    Ok(())
}

/// Build deterministic local port access from a physical port to a grid anchor.
pub fn build_port_access(
    port: &PortInput,
    grid: &StaticGridSpec,
    config: &PortAccessConfig,
) -> Result<PortAccess, PortAccessError> {
    validate_port_access_config(config)?;
    if !port.x.is_finite() || !port.y.is_finite() {
        return Err(PortAccessError::NonFinitePortCoordinate {
            port_name: port.name.clone(),
        });
    }

    let port_angle = orientation_to_angle(port.orientation);
    let (anchor_cell, anchor_angle, mut access_centerline_um) =
        select_anchor_and_build_access(port, grid, config)?;
    let anchor_point_um = grid_cell_center(anchor_cell.0, anchor_cell.1, grid);

    if access_centerline_um.iter().any(|&p| !is_finite_point(p)) {
        return Err(PortAccessError::NonFiniteAccessCoordinate {
            port_name: port.name.clone(),
        });
    }
    if access_centerline_um.len() < 2 {
        return Err(PortAccessError::ZeroLengthAccess {
            port_name: port.name.clone(),
        });
    }
    if access_centerline_um
        .windows(2)
        .any(|w| distance(w[0], w[1]) <= EPS)
    {
        return Err(PortAccessError::ZeroLengthAccess {
            port_name: port.name.clone(),
        });
    }

    access_centerline_um[0] = (port.x, port.y);
    let last = access_centerline_um.len() - 1;
    access_centerline_um[last] = anchor_point_um;

    Ok(PortAccess {
        port_name: port.name.clone(),
        port_point_um: (port.x, port.y),
        anchor_cell,
        anchor_point_um,
        port_angle,
        anchor_angle,
        entry_angle: anchor_angle,
        access_centerline_um,
    })
}

/// Build deterministic local port accesses for all physical ports.
pub fn build_port_accesses(
    ports: &[PortInput],
    grid: &StaticGridSpec,
    config: &PortAccessConfig,
) -> Result<Vec<PortAccess>, PortAccessError> {
    ports
        .iter()
        .map(|port| build_port_access(port, grid, config))
        .collect()
}

/// Generate one closed mitered/beveled waveguide polygon from a centerline.
pub fn generate_waveguide_polygon(
    centerline: &[(f64, f64)],
    width_um: f64,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    if !width_um.is_finite() || width_um <= 0.0 {
        return Err(GeometryError::InvalidWidth(width_um));
    }
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    let half_width = width_um / 2.0;
    let segment_count = centerline.len() - 1;

    let mut normals = Vec::with_capacity(segment_count);
    for i in 0..segment_count {
        let dir = sub(centerline[i + 1], centerline[i]);
        let len = length(dir);
        if len <= EPS {
            return Err(GeometryError::ZeroLengthSegment);
        }

        let unit = (dir.0 / len, dir.1 / len);
        normals.push((-unit.1, unit.0));
    }

    let mut left = Vec::with_capacity(centerline.len() + 4);
    let mut right = Vec::with_capacity(centerline.len() + 4);

    left.push(add(centerline[0], scale(normals[0], half_width)));
    right.push(sub(centerline[0], scale(normals[0], half_width)));

    for i in 1..centerline.len() - 1 {
        append_join(
            &mut left,
            centerline[i],
            normals[i - 1],
            normals[i],
            half_width,
            true,
        );
        append_join(
            &mut right,
            centerline[i],
            scale(normals[i - 1], -1.0),
            scale(normals[i], -1.0),
            half_width,
            false,
        );
    }

    let last_index = centerline.len() - 1;
    let last_normal = normals[normals.len() - 1];
    left.push(add(centerline[last_index], scale(last_normal, half_width)));
    right.push(sub(centerline[last_index], scale(last_normal, half_width)));

    let mut polygon = Vec::with_capacity(left.len() + right.len() + 1);
    polygon.extend(left);
    polygon.extend(right.into_iter().rev());
    if polygon.len() < 3 {
        return Err(GeometryError::DegenerateRoute);
    }

    polygon.push(polygon[0]);
    Ok(polygon)
}

/// Exact grid-footprint route realization.
///
/// This preserves the exact A* state/primitive footprint as a polyline through
/// occupied grid-cell centers, then expands that centerline into a waveguide
/// polygon. The Python flow uses primitive replay instead so bend geometry is
/// realized as sampled curves.
pub fn realize_route_polygon(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    _source_port_um: Option<(f64, f64)>,
    _target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let path = route_to_grid_path(route, primitives)?;
    let centerline = grid_path_to_centerline(&path, grid)?;
    generate_waveguide_polygon(&centerline, width_um)
}

/// Realize route polygon from primitive-replay centerline generation.
pub fn realize_route_polygon_from_primitives(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    generate_waveguide_polygon(&centerline, width_um)
}

/// Full route realization pipeline with explicit source/target port access.
pub fn realize_route_polygon_with_port_access(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    source_access: Option<&PortAccess>,
    target_access: Option<&PortAccess>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let primitive_centerline = route_to_primitive_centerline(route, primitives, grid)?;

    if let Some(source) = source_access {
        if distance(primitive_centerline[0], source.anchor_point_um) > EPS {
            return Err(GeometryError::RouteStartDoesNotMatchSourceAnchor {
                route_start: primitive_centerline[0],
                source_anchor: source.anchor_point_um,
            });
        }
    }
    if let Some(target) = target_access {
        let route_end = primitive_centerline[primitive_centerline.len() - 1];
        if distance(route_end, target.anchor_point_um) > EPS {
            return Err(GeometryError::RouteEndDoesNotMatchTargetAnchor {
                route_end,
                target_anchor: target.anchor_point_um,
            });
        }
    }

    let mut centerline = Vec::with_capacity(
        primitive_centerline.len()
            + source_access
                .map(|a| a.access_centerline_um.len())
                .unwrap_or(0)
            + target_access
                .map(|a| a.access_centerline_um.len())
                .unwrap_or(0),
    );

    if let Some(source) = source_access {
        for point in source.access_centerline_um.iter().copied() {
            push_physical_if_different(&mut centerline, point);
        }
    }
    for point in primitive_centerline.iter().copied() {
        push_physical_if_different(&mut centerline, point);
    }
    if let Some(target) = target_access {
        for point in target.access_centerline_um.iter().rev().copied() {
            push_physical_if_different(&mut centerline, point);
        }
    }

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    generate_waveguide_polygon(&centerline, width_um)
}

fn validate_port_access_config(config: &PortAccessConfig) -> Result<(), PortAccessError> {
    if !config.min_straight_um.is_finite() || config.min_straight_um < 0.0 {
        return Err(PortAccessError::InvalidConfig(
            "min_straight_um must be finite and >= 0".to_string(),
        ));
    }
    if config.max_anchor_search_cells < 0 {
        return Err(PortAccessError::InvalidConfig(
            "max_anchor_search_cells must be >= 0".to_string(),
        ));
    }
    if !config.min_bend_radius_um.is_finite() || config.min_bend_radius_um < 0.0 {
        return Err(PortAccessError::InvalidConfig(
            "min_bend_radius_um must be finite and >= 0".to_string(),
        ));
    }
    Ok(())
}

fn select_anchor_and_build_access(
    port: &PortInput,
    grid: &StaticGridSpec,
    config: &PortAccessConfig,
) -> Result<((i32, i32), u8, Vec<(f64, f64)>), PortAccessError> {
    let base_cell = physical_to_grid(port.x, port.y, grid);
    let port_angle = orientation_to_angle(port.orientation);
    let dir = angle_to_unit_vector(port_angle);
    let lateral_dir = rotate_left(dir);
    let mut candidates: Vec<(i32, i32, i32, i32, f64, f64)> = Vec::new();
    for radius in 1..=config.max_anchor_search_cells.max(1) {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                if dx.abs().max(dy.abs()) != radius {
                    continue;
                }
                let candidate = (base_cell.0 + dx, base_cell.1 + dy);
                if !in_bounds(candidate, grid) {
                    continue;
                }
                let anchor_point = grid_cell_center(candidate.0, candidate.1, grid);
                let delta = sub(anchor_point, (port.x, port.y));
                let forward = dot(delta, dir);
                if forward <= EPS {
                    continue;
                }
                let lateral = dot(delta, lateral_dir).abs();
                candidates.push((radius, dx, dy, candidate.0, lateral, forward));
            }
        }
    }
    candidates.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.4.total_cmp(&b.4))
            .then_with(|| b.5.total_cmp(&a.5))
            .then_with(|| a.3.cmp(&b.3))
            .then_with(|| a.2.cmp(&b.2))
    });
    for (_, dx, dy, _, _, _) in candidates {
        let candidate = (base_cell.0 + dx, base_cell.1 + dy);
        let anchor_point = grid_cell_center(candidate.0, candidate.1, grid);
        let anchor_angle = port_angle;
        if let Ok(centerline) = build_access_centerline(
            (port.x, port.y),
            anchor_point,
            port_angle,
            anchor_angle,
            &port.name,
            config,
        ) {
            return Ok((candidate, anchor_angle, centerline));
        }
    }

    Err(PortAccessError::AnchorSearchFailed {
        port_name: port.name.clone(),
    })
}

fn build_access_centerline(
    port_point: (f64, f64),
    anchor_point: (f64, f64),
    port_angle: u8,
    anchor_angle: u8,
    port_name: &str,
    config: &PortAccessConfig,
) -> Result<Vec<(f64, f64)>, PortAccessError> {
    if !is_finite_point(port_point) || !is_finite_point(anchor_point) {
        return Err(PortAccessError::NonFiniteAccessCoordinate {
            port_name: port_name.to_string(),
        });
    }

    if anchor_angle != port_angle {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }

    let u = angle_to_unit_vector(port_angle);
    let v = rotate_left(u);
    let delta = sub(anchor_point, port_point);
    let local_dx = dot(delta, u);
    let local_dy = dot(delta, v);
    if local_dx <= EPS {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }

    let launch_straight = config.min_straight_um.max(EPS * 10.0);
    let landing_straight = config.min_straight_um.max(EPS * 10.0);
    let mut out = Vec::new();

    // Build in local frame (port at origin, +x along port tangent, +y left-normal), then map back.
    let to_world =
        |p: (f64, f64)| -> (f64, f64) { add(port_point, add(scale(u, p.0), scale(v, p.1))) };
    push_physical_if_different(&mut out, to_world((0.0, 0.0)));

    if local_dy.abs() <= EPS {
        let x1 = launch_straight.min(local_dx);
        push_physical_if_different(&mut out, to_world((x1, 0.0)));
        push_physical_if_different(&mut out, to_world((local_dx, 0.0)));
        if out.len() < 2 {
            return Err(PortAccessError::ZeroLengthAccess {
                port_name: port_name.to_string(),
            });
        }
        return Ok(out);
    }

    let radius = if config.min_bend_radius_um <= EPS {
        (local_dy.abs() / 2.0).max(EPS * 10.0)
    } else {
        config.min_bend_radius_um
    };
    if local_dy.abs() + EPS < 2.0 * radius {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }
    let required_dx = launch_straight + landing_straight + 2.0 * radius;
    if local_dx + EPS < required_dx {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }
    let final_straight = local_dx - required_dx;
    if final_straight < -EPS {
        return Err(PortAccessError::AnchorSearchFailed {
            port_name: port_name.to_string(),
        });
    }

    let sign = if local_dy >= 0.0 { 1.0 } else { -1.0 };
    let abs_dy = local_dy.abs();
    let p1 = (launch_straight, 0.0);
    let p2 = (launch_straight + radius, sign * radius);
    let p3 = (launch_straight + radius, sign * (abs_dy - radius));
    let p4 = (launch_straight + 2.0 * radius, sign * abs_dy);
    let p5 = (
        launch_straight + 2.0 * radius + final_straight,
        sign * abs_dy,
    );
    let p6 = (local_dx - landing_straight, sign * abs_dy);
    let p7 = (local_dx, sign * abs_dy);

    push_physical_if_different(&mut out, to_world(p1));
    append_arc_samples(
        &mut out,
        to_world(p1),
        to_world(p2),
        to_world((launch_straight, sign * radius)),
        sign > 0.0,
    );
    push_physical_if_different(&mut out, to_world(p3));
    append_arc_samples(
        &mut out,
        to_world(p3),
        to_world(p4),
        to_world((launch_straight + 2.0 * radius, sign * (abs_dy - radius))),
        sign < 0.0,
    );
    push_physical_if_different(&mut out, to_world(p5));
    push_physical_if_different(&mut out, to_world(p6));
    push_physical_if_different(&mut out, to_world(p7));

    if out.iter().any(|p| !is_finite_point(*p))
        || out.windows(2).any(|w| distance(w[0], w[1]) <= EPS)
    {
        return Err(PortAccessError::NonFiniteAccessCoordinate {
            port_name: port_name.to_string(),
        });
    }
    Ok(out)
}

fn append_arc_samples(
    out: &mut Vec<(f64, f64)>,
    start: (f64, f64),
    end: (f64, f64),
    center: (f64, f64),
    ccw: bool,
) {
    let radius = distance(start, center);
    if radius <= EPS {
        push_physical_if_different(out, end);
        return;
    }
    let a0 = (start.1 - center.1).atan2(start.0 - center.0);
    let mut a1 = (end.1 - center.1).atan2(end.0 - center.0);
    if ccw {
        while a1 <= a0 {
            a1 += std::f64::consts::TAU;
        }
    } else {
        while a1 >= a0 {
            a1 -= std::f64::consts::TAU;
        }
    }
    let steps = 4usize;
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        let a = a0 + (a1 - a0) * t;
        push_physical_if_different(
            out,
            (center.0 + radius * a.cos(), center.1 + radius * a.sin()),
        );
    }
    push_physical_if_different(out, end);
}

fn in_bounds(cell: (i32, i32), grid: &StaticGridSpec) -> bool {
    cell.0 >= 0 && cell.0 < grid.width && cell.1 >= 0 && cell.1 < grid.height
}

fn orientation_to_angle(orientation: Option<f64>) -> u8 {
    let value = orientation.unwrap_or(0.0).rem_euclid(360.0);
    (value / 45.0).round().rem_euclid(8.0) as u8
}

fn append_join(
    out: &mut Vec<(f64, f64)>,
    point: (f64, f64),
    normal_a: (f64, f64),
    normal_b: (f64, f64),
    half_width: f64,
    positive_side: bool,
) {
    let offset_a = add(point, scale(normal_a, half_width));
    let offset_b = add(point, scale(normal_b, half_width));

    let bisector = add(normal_a, normal_b);
    let bisector_len = length(bisector);

    if bisector_len <= EPS {
        out.push(offset_a);
        out.push(offset_b);
        return;
    }

    let miter_dir = (bisector.0 / bisector_len, bisector.1 / bisector_len);
    let denom = dot(miter_dir, normal_a);

    if denom.abs() <= EPS {
        out.push(offset_a);
        out.push(offset_b);
        return;
    }

    let miter_len = half_width / denom;
    if !miter_len.is_finite() || miter_len.abs() > MITER_LIMIT * half_width {
        if positive_side {
            out.push(offset_a);
            out.push(offset_b);
        } else {
            out.push(offset_b);
            out.push(offset_a);
        }
        return;
    }

    out.push(add(point, scale(miter_dir, miter_len)));
}

fn push_if_different(points: &mut Vec<(i32, i32)>, point: (i32, i32)) {
    if points.last().copied() != Some(point) {
        points.push(point);
    }
}

fn push_physical_if_different(points: &mut Vec<(f64, f64)>, point: (f64, f64)) {
    if points
        .last()
        .map(|&last| distance(last, point) > EPS)
        .unwrap_or(true)
    {
        points.push(point);
    }
}

fn is_finite_point(point: (f64, f64)) -> bool {
    point.0.is_finite() && point.1.is_finite()
}

fn direction(a: (i32, i32), b: (i32, i32)) -> (i32, i32) {
    ((b.0 - a.0).signum(), (b.1 - a.1).signum())
}

fn add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

fn sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

fn scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

fn dot(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn cross(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

fn length(a: (f64, f64)) -> f64 {
    dot(a, a).sqrt()
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    length(sub(a, b))
}

fn append_circular_bend_centerline(
    out: &mut Vec<(f64, f64)>,
    start_point: (f64, f64),
    start_angle: u8,
    end_point: (f64, f64),
    end_angle: u8,
    radius_um: f64,
    angle_delta: i8,
    samples_per_90_deg: usize,
) -> Result<(), GeometryError> {
    if !start_point.0.is_finite()
        || !start_point.1.is_finite()
        || !end_point.0.is_finite()
        || !end_point.1.is_finite()
        || !radius_um.is_finite()
        || radius_um <= 0.0
    {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    let start_dir = angle_to_unit_vector(start_angle);
    let end_dir = angle_to_unit_vector(end_angle);
    let chord = sub(end_point, start_point);
    let denom = cross(start_dir, end_dir);
    if denom.abs() <= EPS {
        push_physical_if_different(out, end_point);
        return Ok(());
    }
    let in_len = cross(chord, end_dir) / denom;
    let out_len = cross(start_dir, chord) / denom;
    if !in_len.is_finite() || !out_len.is_finite() || in_len <= EPS || out_len <= EPS {
        push_physical_if_different(out, end_point);
        return Ok(());
    }
    let corner = add(start_point, scale(start_dir, in_len));
    let turn_abs = (angle_delta as f64).abs() * (std::f64::consts::PI / 4.0);
    let trim = radius_um * (turn_abs / 2.0).tan();

    let trim_eff = trim.min(in_len - EPS).min(out_len - EPS);
    if !trim_eff.is_finite() || trim_eff <= EPS {
        push_physical_if_different(out, end_point);
        return Ok(());
    }

    let t_in = sub(corner, scale(start_dir, trim_eff));
    let t_out = add(corner, scale(end_dir, trim_eff));

    push_physical_if_different(out, t_in);

    let left_turn = angle_delta > 0;
    let n_start = if left_turn {
        rotate_left(start_dir)
    } else {
        rotate_right(start_dir)
    };
    let n_end = if left_turn {
        rotate_left(end_dir)
    } else {
        rotate_right(end_dir)
    };
    let c0 = add(t_in, scale(n_start, radius_um));
    let c1 = add(t_out, scale(n_end, radius_um));
    let center = scale(add(c0, c1), 0.5);

    let a0 = (t_in.1 - center.1).atan2(t_in.0 - center.0);
    let mut a1 = (t_out.1 - center.1).atan2(t_out.0 - center.0);
    if left_turn {
        while a1 <= a0 {
            a1 += std::f64::consts::TAU;
        }
    } else {
        while a1 >= a0 {
            a1 -= std::f64::consts::TAU;
        }
    }

    let arc_span = (a1 - a0).abs();
    let mut steps =
        ((arc_span / (std::f64::consts::PI / 2.0)) * samples_per_90_deg as f64).ceil() as usize;
    steps = steps.max(2);
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        let a = a0 + (a1 - a0) * t;
        let p = (
            center.0 + radius_um * a.cos(),
            center.1 + radius_um * a.sin(),
        );
        push_physical_if_different(out, p);
    }

    push_physical_if_different(out, t_out);
    push_physical_if_different(out, end_point);
    Ok(())
}

fn angle_to_unit_vector(angle: u8) -> (f64, f64) {
    let a = angle_to_radians(angle);
    (a.cos(), a.sin())
}

fn angle_to_radians(angle: u8) -> f64 {
    (angle as f64) * (std::f64::consts::PI / 4.0)
}

fn rotate_left(v: (f64, f64)) -> (f64, f64) {
    (-v.1, v.0)
}

fn rotate_right(v: (f64, f64)) -> (f64, f64) {
    (v.1, -v.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};
    use crate::static_obstacle_builder::make_grid_spec;

    fn grid() -> GeometryGridSpec {
        GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap()
    }

    fn test_lib() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
        })
    }

    #[test]
    fn compresses_straight_route_to_two_waypoints() {
        let states = vec![
            State::new(1, 2, 0),
            State::new(2, 2, 0),
            State::new(5, 2, 0),
        ];

        assert_eq!(compress_route_waypoints(&states), vec![(1, 2), (5, 2)]);
    }

    #[test]
    fn primitive_path_preserves_ninety_degree_turn_corner() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(2, 2, 2)],
            primitives: vec![4],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(2, 2, 2),
            reached_target: State::new(2, 2, 2),
            stats: Default::default(),
        };

        let path = route_to_grid_path(&route, &lib).unwrap();
        assert_eq!(path, vec![(1, 1), (2, 1), (2, 2)]);

        let waypoints = compress_grid_waypoints(&path);
        assert_eq!(waypoints, vec![(1, 1), (2, 1), (2, 2)]);
    }

    #[test]
    fn centerline_uses_cell_centers() {
        let centerline = grid_path_to_centerline(&[(0, 0), (2, 0)], &grid()).unwrap();

        assert_eq!(centerline, vec![(0.5, 0.5), (2.5, 0.5)]);
    }

    #[test]
    fn primitive_replay_straight_centerline_uses_state_endpoints() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let centerline = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        assert_eq!(centerline, vec![(1.5, 2.5), (5.5, 2.5)]);
    }

    #[test]
    fn primitive_replay_bend_centerline_dispatches_bend_branch() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(2, 2, 2)],
            primitives: vec![4],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(2, 2, 2),
            reached_target: State::new(2, 2, 2),
            stats: Default::default(),
        };
        let centerline = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        assert_eq!(centerline.first().copied(), Some((1.5, 1.5)));
        assert_eq!(centerline.last().copied(), Some((2.5, 2.5)));
        assert!(centerline.len() > 2);
        assert!(centerline
            .iter()
            .all(|(x, y)| x.is_finite() && y.is_finite()));
        assert!(centerline.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn primitive_replay_45_and_right_bends_are_sampled_and_oriented() {
        let lib = test_lib();
        let left_45_pid = lib.get_primitives_for_angle(0)[2].id;
        let right_45_pid = lib.get_primitives_for_angle(0)[3].id;
        let left_45 = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(3, 2, 1)],
            primitives: vec![left_45_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(3, 2, 1),
            reached_target: State::new(3, 2, 1),
            stats: Default::default(),
        };
        let right_45 = RouteResult {
            states: vec![State::new(1, 1, 0), State::new(3, 0, 7)],
            primitives: vec![right_45_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(3, 0, 7),
            reached_target: State::new(3, 0, 7),
            stats: Default::default(),
        };
        let left = route_to_primitive_centerline(&left_45, &lib, &grid()).unwrap();
        let right = route_to_primitive_centerline(&right_45, &lib, &grid()).unwrap();
        assert!(left.len() > 2);
        assert!(right.len() > 2);
        assert_eq!(left.first().copied(), Some((1.5, 1.5)));
        assert_eq!(left.last().copied(), Some((3.5, 2.5)));
        assert_eq!(right.first().copied(), Some((1.5, 1.5)));
        assert_eq!(right.last().copied(), Some((3.5, 0.5)));

        let left_mid = left[left.len() / 2];
        let right_mid = right[right.len() / 2];
        assert!(left_mid.1 > 1.5);
        assert!(right_mid.1 < 1.5);
    }

    #[test]
    fn primitive_replay_diagonal_start_bend_tail_follows_end_angle() {
        let lib = test_lib();
        let left_45_pid = lib.get_primitives_for_angle(1)[2].id;
        let route = RouteResult {
            states: vec![State::new(1, 1, 1), State::new(2, 3, 2)],
            primitives: vec![left_45_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(2, 3, 2),
            reached_target: State::new(2, 3, 2),
            stats: Default::default(),
        };

        let centerline = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        let tail_start = centerline[centerline.len() - 2];
        let tail_end = centerline[centerline.len() - 1];

        assert!((tail_end.0 - tail_start.0).abs() <= EPS);
        assert!(tail_end.1 > tail_start.1);
    }

    #[test]
    fn primitive_replay_rejects_invalid_topology() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let err = route_to_primitive_centerline(&route, &lib, &grid()).unwrap_err();
        assert!(matches!(err, GeometryError::InvalidRouteTopology { .. }));
    }

    #[test]
    fn primitive_replay_rejects_missing_primitive_id() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![9999],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let err = route_to_primitive_centerline(&route, &lib, &grid()).unwrap_err();
        assert!(matches!(err, GeometryError::MissingPrimitive { .. }));
    }

    #[test]
    fn snapped_endpoints_override_cell_centers() {
        let mut centerline = vec![(0.5, 0.5), (2.5, 0.5)];
        snap_centerline_endpoints(&mut centerline, Some((0.0, 0.0)), Some((3.0, 0.0))).unwrap();
        assert_eq!(centerline, vec![(0.0, 0.0), (3.0, 0.0)]);
    }

    fn static_grid() -> StaticGridSpec {
        make_grid_spec((0.0, 0.0, 10.0, 10.0), 1.0).unwrap()
    }

    fn tiny_grid() -> StaticGridSpec {
        make_grid_spec((0.0, 0.0, 1.0, 1.0), 1.0).unwrap()
    }

    #[test]
    fn port_at_grid_center_has_direct_access() {
        let port = PortInput::new("p0".to_string(), 0.5, 0.5, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert!(access.anchor_cell.0 >= 0 && access.anchor_cell.1 >= 0);
        assert_eq!(
            access.access_centerline_um.first().copied(),
            Some((0.5, 0.5))
        );
        assert_eq!(
            access.access_centerline_um.last().copied(),
            Some(access.anchor_point_um)
        );
        assert!(access.access_centerline_um.len() >= 2);
    }

    #[test]
    fn off_grid_port_ends_access_at_anchor_center() {
        let port = PortInput::new("p1".to_string(), 0.2, 0.7, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert!(access.anchor_cell.0 >= 0 && access.anchor_cell.1 >= 0);
        assert_eq!(
            access.access_centerline_um.first().copied(),
            Some((0.2, 0.7))
        );
        assert_eq!(
            access.access_centerline_um.last().copied(),
            Some(access.anchor_point_um)
        );
        assert!(access.access_centerline_um.len() > 2);
    }

    #[test]
    fn access_first_segment_follows_port_orientation() {
        let port = PortInput::new("p2".to_string(), 1.0, 1.0, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        let p0 = access.access_centerline_um[0];
        let p1 = access.access_centerline_um[1];
        assert!(p1.0 > p0.0);
        assert!((p1.1 - p0.1).abs() <= 1e-9);
    }

    #[test]
    fn access_last_segment_follows_anchor_angle() {
        let port = PortInput::new("p2b".to_string(), 1.0, 1.0, Some(0.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        let n = access.access_centerline_um.len();
        let p0 = access.access_centerline_um[n - 2];
        let p1 = access.access_centerline_um[n - 1];
        let seg = sub(p1, p0);
        let dir = angle_to_unit_vector(access.anchor_angle);
        let perp = rotate_left(dir);
        assert!(dot(seg, dir) > EPS);
        assert!(dot(seg, perp).abs() <= 1e-6);
        assert_eq!(access.anchor_angle, access.entry_angle);
    }

    #[test]
    fn access_points_are_finite_and_non_degenerate() {
        let port = PortInput::new("p3".to_string(), 0.3, 0.8, Some(45.0));
        let access =
            build_port_access(&port, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert!(access
            .access_centerline_um
            .iter()
            .all(|p| p.0.is_finite() && p.1.is_finite()));
        assert!(access
            .access_centerline_um
            .windows(2)
            .all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn infeasible_access_returns_explicit_error() {
        let sg = tiny_grid();
        let port = PortInput::new("p4".to_string(), 0.1, 0.1, Some(0.0));
        let cfg = PortAccessConfig {
            min_straight_um: 5.0,
            max_anchor_search_cells: 1,
            min_bend_radius_um: 5.0,
        };
        let err = build_port_access(&port, &sg, &cfg).unwrap_err();
        assert!(matches!(err, PortAccessError::AnchorSearchFailed { .. }));
    }

    #[test]
    fn batch_build_port_accesses_returns_one_per_port() {
        let ports = vec![
            PortInput::new("b0".to_string(), 0.2, 0.7, Some(0.0)),
            PortInput::new("b1".to_string(), 0.3, 0.8, Some(45.0)),
        ];
        let accesses =
            build_port_accesses(&ports, &static_grid(), &PortAccessConfig::default()).unwrap();
        assert_eq!(accesses.len(), ports.len());
    }

    #[test]
    fn straight_route_generates_closed_rectangle() {
        let centerline = vec![(0.5, 0.5), (4.5, 0.5)];
        let polygon = generate_waveguide_polygon(&centerline, 1.0).unwrap();

        assert_eq!(polygon.first(), polygon.last());
        assert_eq!(polygon.len(), 5);
        assert!(polygon.contains(&(0.5, 1.0)));
        assert!(polygon.contains(&(4.5, 0.0)));
    }

    #[test]
    fn ninety_degree_route_generates_closed_polygon() {
        let centerline = vec![(0.5, 0.5), (4.5, 0.5), (4.5, 4.5)];
        let polygon = generate_waveguide_polygon(&centerline, 1.0).unwrap();

        assert_eq!(polygon.first(), polygon.last());
        assert!(polygon.len() >= 5);
        assert!(polygon.iter().all(|&p| p.0.is_finite() && p.1.is_finite()));
    }

    #[test]
    fn realizes_route_polygon_without_port_access() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let polygon = realize_route_polygon(
            &route,
            &lib,
            &grid(),
            1.0,
            Some((1.0, 2.0)),
            Some((6.0, 2.0)),
        )
        .unwrap();

        assert_eq!(polygon.first(), polygon.last());
        let min_x = polygon.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_x = polygon
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(min_x <= 1.5 + 1e-9);
        assert!(max_x >= 5.5 - 1e-9);
    }

    #[test]
    fn realizes_route_polygon_with_explicit_port_access_endpoints() {
        let lib = test_lib();
        let sg = static_grid();
        let access_cfg = PortAccessConfig {
            min_straight_um: 0.0,
            max_anchor_search_cells: 8,
            min_bend_radius_um: 0.5,
        };
        let source = build_port_access(
            &PortInput::new("s".to_string(), 1.0, 2.5, Some(0.0)),
            &sg,
            &access_cfg,
        )
        .unwrap();
        let target = build_port_access(
            &PortInput::new("t".to_string(), 6.0, 2.5, Some(180.0)),
            &sg,
            &access_cfg,
        )
        .unwrap();

        let source_state = State::new(
            source.anchor_cell.0,
            source.anchor_cell.1,
            source.entry_angle,
        );
        let target_state = State::new(
            target.anchor_cell.0,
            target.anchor_cell.1,
            target.entry_angle,
        );
        let straight_pid = lib
            .get_primitives_for_angle(source_state.angle)
            .iter()
            .find(|p| {
                p.end_angle == source_state.angle
                    && p.dx == (target_state.x - source_state.x)
                    && p.dy == (target_state.y - source_state.y)
            })
            .map(|p| p.id)
            .unwrap_or(lib.get_primitives_for_angle(source_state.angle)[0].id);

        let route = RouteResult {
            states: vec![source_state, target_state],
            primitives: vec![straight_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: target_state,
            reached_target: target_state,
            stats: Default::default(),
        };

        let polygon = realize_route_polygon_with_port_access(
            &route,
            &lib,
            &grid(),
            1.0,
            Some(&source),
            Some(&target),
        )
        .unwrap();
        assert_eq!(polygon.first(), polygon.last());
        let min_x = polygon.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_x = polygon
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(min_x <= 1.0 + 1e-9);
        assert!(max_x >= 6.0 - 1e-9);
    }

    #[test]
    fn realizes_route_polygon_from_primitives_closed_polygon() {
        let lib = test_lib();
        let bend_pid = lib.get_primitives_for_angle(0)[4].id;
        let straight_north_pid = lib.get_primitives_for_angle(2)[0].id;
        let route = RouteResult {
            states: vec![
                State::new(1, 1, 0),
                State::new(2, 2, 2),
                State::new(2, 3, 2),
            ],
            primitives: vec![bend_pid, straight_north_pid],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let polygon = realize_route_polygon_from_primitives(&route, &lib, &grid(), 1.0).unwrap();
        assert_eq!(polygon.first(), polygon.last());
    }

    #[test]
    fn rejects_out_of_bounds_anchor_search() {
        let sg = static_grid();
        let config = PortAccessConfig {
            max_anchor_search_cells: 0,
            ..Default::default()
        };
        let port = PortInput::new("oob".to_string(), -10.0, -10.0, Some(180.0));
        let err = build_port_access(&port, &sg, &config).unwrap_err();
        assert!(matches!(err, PortAccessError::AnchorSearchFailed { .. }));
    }

    #[test]
    fn rejects_zero_length_access_connector() {
        let sg = tiny_grid();
        let port = PortInput::new("zero".to_string(), 0.5, 0.5, Some(0.0));
        let err = build_port_access(&port, &sg, &PortAccessConfig::default()).unwrap_err();
        assert!(matches!(
            err,
            PortAccessError::ZeroLengthAccess { .. } | PortAccessError::AnchorSearchFailed { .. }
        ));
    }

    #[test]
    fn rejects_mismatched_route_anchor_endpoints() {
        let lib = test_lib();
        let route = RouteResult {
            states: vec![State::new(1, 2, 0), State::new(5, 2, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0,
            total_cost: 4.0,
            requested_target: State::new(5, 2, 0),
            reached_target: State::new(5, 2, 0),
            stats: Default::default(),
        };
        let bad_source = PortAccess {
            port_name: "bad".to_string(),
            port_point_um: (0.0, 0.0),
            anchor_cell: (0, 0),
            anchor_point_um: (0.5, 0.5),
            port_angle: 0,
            anchor_angle: 0,
            entry_angle: 0,
            access_centerline_um: vec![(0.0, 0.0), (0.5, 0.5)],
        };
        let err = realize_route_polygon_with_port_access(
            &route,
            &lib,
            &grid(),
            1.0,
            Some(&bad_source),
            None,
        )
        .unwrap_err();
        assert!(matches!(
            err,
            GeometryError::RouteStartDoesNotMatchSourceAnchor { .. }
        ));
    }

    #[test]
    fn rejects_degenerate_centerline() {
        let err = generate_waveguide_polygon(&[(0.0, 0.0)], 1.0).unwrap_err();

        assert_eq!(err, GeometryError::DegenerateRoute);
    }

    #[test]
    fn rejects_zero_width() {
        let err = generate_waveguide_polygon(&[(0.0, 0.0), (1.0, 0.0)], 0.0).unwrap_err();

        assert_eq!(err, GeometryError::InvalidWidth(0.0));
    }
}
