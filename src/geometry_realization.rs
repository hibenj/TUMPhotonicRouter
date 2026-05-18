//! Fast route-to-polygon realization.
//!
//! This module converts a routed state/primitive sequence into one closed
//! waveguide polygon in physical coordinates.

use std::error::Error;
use std::fmt;

use crate::astar::{RouteResult, State};
use crate::primitives::PrimitiveLibrary;

const EPS: f64 = 1.0e-9;
const MITER_LIMIT: f64 = 4.0;

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
    InvalidRouteTopology { states: usize, primitives: usize },
    MissingPrimitive { id: u16, start_angle: u8 },
    ZeroLengthSegment,
    NonFiniteCoordinate,
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
        }
    }
}

impl Error for GeometryError {}

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

/// Full route realization pipeline.
pub fn realize_route_polygon(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let path = route_to_grid_path(route, primitives)?;
    let mut centerline = grid_path_to_centerline(&path, grid)?;
    snap_centerline_endpoints(&mut centerline, source_port_um, target_port_um)?;
    generate_waveguide_polygon(&centerline, width_um)
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

fn length(a: (f64, f64)) -> f64 {
    dot(a, a).sqrt()
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    length(sub(a, b))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{create_photonic_primitive_library, PrimitiveLibraryConfig};

    fn grid() -> GeometryGridSpec {
        GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap()
    }

    fn test_lib() -> PrimitiveLibrary {
        create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
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
    fn snapped_endpoints_override_cell_centers() {
        let mut centerline = vec![(0.5, 0.5), (2.5, 0.5)];
        snap_centerline_endpoints(&mut centerline, Some((0.0, 0.0)), Some((3.0, 0.0))).unwrap();
        assert_eq!(centerline, vec![(0.0, 0.0), (3.0, 0.0)]);
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
    fn realizes_route_polygon_with_port_snapping() {
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
        assert!(min_x <= 1.0 + 1e-9);
        assert!(max_x >= 6.0 - 1e-9);
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
