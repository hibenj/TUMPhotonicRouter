//! Fast route-to-polygon realization.
//!
//! This module converts a routed state/primitive sequence into one closed
//! waveguide polygon in physical coordinates.

use std::error::Error;
use std::fmt;
use std::time::Instant;

use rustc_hash::FxHashSet;

use crate::astar::{RouteResult, State};
#[cfg(test)]
use crate::auto_meander::{
    cells_in_grid_rect, extract_axis_aligned_straight_runs, meander_box_to_grid_rect,
    AutoMeanderSidePolicy,
};
use crate::auto_meander::{
    check_meander_box_free, plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix,
    probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix, AutoMeanderConfig,
    AutoMeanderRejectionDetail, AutoRouteAnalyticMeanderPlan, AutoRouteAnalyticMeanderProbe,
    DenseOccupancyPrefix, MeanderGridRect, SparseCellIndex,
};
use crate::meander::{
    plan_analytic_meander, AnalyticMeanderConfig, AnalyticMeanderPlan, MeanderBox,
    MeanderPlanningError, MeanderPlanningMode, MeanderSide, PhysicalPoint, StraightSegment,
};
use crate::obstacle_map::{CellKey, ObstacleMap};
use crate::primitives::{PrimitiveGeometry, PrimitiveLibrary};
use crate::static_obstacle_builder::{
    cell_center_coordinate, grid_cell_center, physical_to_grid, PortInput, StaticGridSpec,
};

pub(crate) const EPS: f64 = 1.0e-9;
const MITER_LIMIT: f64 = 4.0;
/// Floor for the number of polyline samples per 90 degrees of bend arc.
const DEFAULT_BEND_SAMPLES_PER_90_DEG: usize = 16;
/// Largest allowed deviation (sagitta) between a sampled arc chord and the
/// true arc, in micrometers. 1 nm is one GDS database unit -- the same
/// density gdsfactory's `bend_circular` uses (a vertex every 20 nm of arc) -- so realized
/// bends and meander U-turns read as curves rather than as chamfered
/// corners, and their polyline length stays within ~1e-5 of the true arc
/// length that the path-length model books.
pub const MAX_ARC_SAGITTA_UM: f64 = 0.001;

/// Length of a quarter arc of `radius_um` *as realized*: the polyline of
/// `arc_samples_per_90_deg` chords, which is what the GDS carries and what a
/// path length measured in it will show. Slightly shorter than `pi * r / 2`
/// (by ~1 nm at r = 10 um). The path-length model books this, not the ideal
/// arc, so matched paths are equal in the layout, not just on paper.
pub fn sampled_quarter_arc_length_um(radius_um: f64) -> f64 {
    let samples = arc_samples_per_90_deg(radius_um) as f64;
    samples * 2.0 * radius_um * (std::f64::consts::FRAC_PI_4 / samples).sin()
}

/// Number of polyline samples per 90 degrees of arc at `radius_um` so that
/// no chord deviates from the arc by more than [`MAX_ARC_SAGITTA_UM`], never
/// fewer than [`DEFAULT_BEND_SAMPLES_PER_90_DEG`]. Every arc builder in the
/// realization (primitive bends, endpoint-correction jogs, meanders) samples
/// through this so a bend looks the same wherever it comes from.
pub fn arc_samples_per_90_deg(radius_um: f64) -> usize {
    if !radius_um.is_finite() || radius_um <= MAX_ARC_SAGITTA_UM {
        return DEFAULT_BEND_SAMPLES_PER_90_DEG;
    }
    let max_step_rad = 2.0 * (1.0 - MAX_ARC_SAGITTA_UM / radius_um).acos();
    if !max_step_rad.is_finite() || max_step_rad <= 0.0 {
        return DEFAULT_BEND_SAMPLES_PER_90_DEG;
    }
    let needed = (std::f64::consts::FRAC_PI_2 / max_step_rad).ceil() as usize;
    needed.max(DEFAULT_BEND_SAMPLES_PER_90_DEG)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AxisAlignedRunKind {
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AxisAlignedRun {
    start_index: usize,
    end_index: usize,
    kind: AxisAlignedRunKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DirectionalStraightRun {
    start_index: usize,
    end_index: usize,
    angle: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DoglegAbsorber {
    transition_index: usize,
    kind: AxisAlignedRunKind,
    dir: i8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EndpointDeltaMutation {
    Unchanged,
    CoordinatesOnly,
    InsertedPoint { after_index: usize },
    RebuiltCenterline,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum OffsetBumpPlacement {
    Start,
    End,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OffsetBumpCandidate {
    pub label: String,
    pub centerline: Vec<(f64, f64)>,
    pub placement_is_start: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct PrimitiveCenterlineReplay {
    centerline: Vec<(f64, f64)>,
    straight_runs: Vec<AxisAlignedRun>,
    directional_runs: Vec<DirectionalStraightRun>,
    dogleg_absorbers: Vec<DoglegAbsorber>,
}

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
            cell_center_coordinate(x, self.origin_x_um, self.grid_size_um),
            cell_center_coordinate(y, self.origin_y_um, self.grid_size_um),
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
    InvalidMeanderBox,
    MeanderBoxOutOfBounds(MeanderGridRect),
    MeanderBoxBlocked {
        rect: MeanderGridRect,
        blocked_count: u32,
    },
    NoAutoMeanderCandidate {
        candidate_runs: usize,
        candidate_intervals: usize,
        rejected_box_blocked: usize,
        rejected_planning_failed: usize,
        rejected_exact_length_mismatch: usize,
        rejected_too_short: usize,
        first_rejection: Option<AutoMeanderRejectionDetail>,
    },
    NoMeanderCandidateSegment,
    PortEndpointCorrectionRequiresUnsupportedStub,
    MeanderPlanningFailed(MeanderPlanningError),
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
            GeometryError::InvalidMeanderBox => write!(f, "invalid meander box"),
            GeometryError::MeanderBoxOutOfBounds(rect) => {
                write!(f, "meander box out of bounds after grid conversion: {rect:?}")
            }
            GeometryError::MeanderBoxBlocked {
                rect,
                blocked_count,
            } => write!(
                f,
                "meander box overlaps blocked cells: rect={rect:?}, blocked_count={blocked_count}"
            ),
            GeometryError::NoAutoMeanderCandidate {
                candidate_runs,
                candidate_intervals,
                rejected_box_blocked,
                rejected_planning_failed,
                rejected_exact_length_mismatch,
                rejected_too_short,
                first_rejection,
            } => write!(
                f,
                "no legal auto-analytic meander candidate found (candidate_runs={candidate_runs}, candidate_intervals={candidate_intervals}, rejected_box_blocked={rejected_box_blocked}, rejected_planning_failed={rejected_planning_failed}, rejected_exact_length_mismatch={rejected_exact_length_mismatch}, rejected_too_short={rejected_too_short}"
            )
            .and_then(|_| {
                if let Some(detail) = first_rejection {
                    write!(
                        f,
                        ", first_rejection={{reason={}, side={:?}, depth_um={:.3}, run_start_index={}, run_end_index={}, run_length_um={:.3}, allowed_interval_length_um={:.3}, required_interval_length_um={:.3}, run_start=({:.3},{:.3}), run_end=({:.3},{:.3}), strip_grid_rect={:?}, strip_blocked_count={:?}, strip_grid_error={:?}, planning_error={:?}}}",
                        detail.reason,
                        detail.side,
                        detail.depth_um,
                        detail.run_start_index,
                        detail.run_end_index,
                        detail.run_length_um,
                        detail.allowed_interval_length_um,
                        detail.required_interval_length_um,
                        detail.run_start.0,
                        detail.run_start.1,
                        detail.run_end.0,
                        detail.run_end.1,
                        detail.strip_grid_rect,
                        detail.strip_blocked_count,
                        detail.strip_grid_error,
                        detail.planning_error,
                    )?;
                }
                write!(f, ")")
            }),
            GeometryError::NoMeanderCandidateSegment => {
                write!(f, "no axis-aligned centerline segment is suitable for meander insertion")
            }
            GeometryError::PortEndpointCorrectionRequiresUnsupportedStub => write!(
                f,
                "port endpoint correction would require an unsupported terminal stub"
            ),
            GeometryError::MeanderPlanningFailed(err) => {
                write!(f, "analytic meander planning failed: {err:?}")
            }
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

    // Simple pre-routes may return grid-polyline results without primitive IDs.
    // In that case we can still realize geometry by walking the state polyline.
    if route.primitives.is_empty() {
        let mut path = Vec::with_capacity(route.states.len());
        for state in &route.states {
            push_if_different(&mut path, (state.x, state.y));
        }
        if path.len() < 2 {
            return Err(GeometryError::DegenerateRoute);
        }
        return Ok(path);
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
    Ok(route_to_primitive_centerline_with_runs(route, primitives, grid)?.centerline)
}

fn route_to_primitive_centerline_with_runs(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
) -> Result<PrimitiveCenterlineReplay, GeometryError> {
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
    let mut straight_runs = Vec::new();
    let mut directional_runs = Vec::new();
    let mut dogleg_absorbers = Vec::new();
    let mut previous_was_bend = false;
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

        let current_is_bend = matches!(primitive.geometry, PrimitiveGeometry::Bend { .. });
        if previous_was_bend && current_is_bend {
            if let Some((kind, dir)) = dogleg_absorber_for_angle(state.angle) {
                dogleg_absorbers.push(DoglegAbsorber {
                    transition_index: centerline.len() - 1,
                    kind,
                    dir,
                });
            }
        }

        match primitive.geometry {
            PrimitiveGeometry::Straight { .. } => {
                let start_index = centerline.len() - 1;
                push_physical_if_different(&mut centerline, expected);
                let end_index = centerline.len() - 1;
                if end_index > start_index {
                    directional_runs.push(DirectionalStraightRun {
                        start_index,
                        end_index,
                        angle: state.angle % 8,
                    });
                    let a = centerline[start_index];
                    let b = centerline[end_index];
                    let kind = if is_horizontal_segment(a, b) {
                        Some(AxisAlignedRunKind::Horizontal)
                    } else if is_vertical_segment(a, b) {
                        Some(AxisAlignedRunKind::Vertical)
                    } else {
                        None
                    };
                    if let Some(kind) = kind {
                        straight_runs.push(AxisAlignedRun {
                            start_index,
                            end_index,
                            kind,
                        });
                    }
                }
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
                    arc_samples_per_90_deg(radius_um),
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
        previous_was_bend = current_is_bend;
    }

    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }

    Ok(PrimitiveCenterlineReplay {
        centerline,
        straight_runs,
        directional_runs,
        dogleg_absorbers,
    })
}

fn dogleg_absorber_for_angle(angle: u8) -> Option<(AxisAlignedRunKind, i8)> {
    match angle % 8 {
        0 => Some((AxisAlignedRunKind::Horizontal, 1)),
        2 => Some((AxisAlignedRunKind::Vertical, 1)),
        4 => Some((AxisAlignedRunKind::Horizontal, -1)),
        6 => Some((AxisAlignedRunKind::Vertical, -1)),
        _ => None,
    }
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

/// Replay route primitives, then adjust physical endpoints to real ports.
///
/// This keeps the existing primitive-replay centerline and waveguide polygon
/// generator as the only geometry realization path. For a single straight this
/// becomes the exact physical straight between ports; for multi-segment routes
/// it anchors the realized centerline to the physical port coordinates while
/// preserving the routed interior points for later meander planning/splicing.
pub fn route_to_port_corrected_centerline(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    route_to_port_corrected_centerline_with_options(
        route,
        primitives,
        grid,
        source_port_um,
        target_port_um,
        true,
    )
}

pub fn route_to_port_corrected_centerline_with_options(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
    allow_unchecked_bumps: bool,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    route_to_port_corrected_centerline_with_options_and_collision_check(
        route,
        primitives,
        grid,
        source_port_um,
        target_port_um,
        allow_unchecked_bumps,
        None,
    )
}

/// Same as `route_to_port_corrected_centerline_with_options`, with an
/// additional, optional collision check consulted by any strategy in this
/// chain that already tries multiple candidates before picking one
/// (currently just `try_apply_45_degree_endpoint_delta_correction`). See
/// that function's own doc comment for what the check must do and why it
/// exists (2026-08-19,
/// .agent/execplans/2026-08-19-collision-avoiding-endpoint-correction.md).
/// `None` reproduces `route_to_port_corrected_centerline_with_options`'s
/// exact prior behavior; every caller except
/// `route_port_corrected_centerline_checked_and_commit_native`
/// (`src/py_router.rs`) passes `None`.
pub fn route_to_port_corrected_centerline_with_options_and_collision_check(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
    allow_unchecked_bumps: bool,
    collision_check: Option<&dyn Fn(&[(f64, f64)]) -> bool>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let replay = route_to_primitive_centerline_with_runs(route, primitives, grid)?;
    let mut centerline = replay.centerline;
    if !try_apply_full_straight_port_correction(&mut centerline, source_port_um, target_port_um)? {
        if !try_apply_full_diagonal_straight_port_correction(
            &mut centerline,
            source_port_um,
            target_port_um,
        )? {
            try_apply_shared_axis_port_shift(&mut centerline, source_port_um, target_port_um)?;
            if !try_apply_45_degree_endpoint_delta_correction(
                &mut centerline,
                &replay.straight_runs,
                &replay.directional_runs,
                &replay.dogleg_absorbers,
                primitives,
                route,
                source_port_um,
                target_port_um,
                false,
                collision_check,
            )? {
                let mut existing_axis_candidate = centerline.clone();
                if absorb_endpoint_delta_into_existing_axis_runs(
                    &mut existing_axis_candidate,
                    &replay.straight_runs,
                    source_port_um,
                    target_port_um,
                )
                .is_ok()
                {
                    centerline = existing_axis_candidate;
                } else if allow_unchecked_bumps {
                    let mut bump_candidate = centerline.clone();
                    match try_apply_full_straight_offset_bump_correction(
                        &mut bump_candidate,
                        route,
                        primitives,
                        source_port_um,
                        target_port_um,
                    ) {
                        Ok(true) => centerline = bump_candidate,
                        Ok(false) | Err(_) => absorb_endpoint_delta_into_axis_runs(
                            &mut centerline,
                            &replay.straight_runs,
                            &replay.dogleg_absorbers,
                            primitives,
                            source_port_um,
                            target_port_um,
                        )?,
                    }
                } else {
                    absorb_endpoint_delta_into_existing_axis_runs(
                        &mut centerline,
                        &replay.straight_runs,
                        source_port_um,
                        target_port_um,
                    )?;
                }
            }
        }
    }
    let mut deduped = Vec::with_capacity(centerline.len());
    for point in centerline.iter().copied() {
        push_physical_if_different(&mut deduped, point);
    }
    centerline = deduped;
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    Ok(centerline)
}

fn dedupe_centerline(centerline: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut deduped = Vec::with_capacity(centerline.len());
    for point in centerline.iter().copied() {
        push_physical_if_different(&mut deduped, point);
    }
    deduped
}

pub fn centerline_to_port_corrected_centerline_with_options(
    centerline: &[(f64, f64)],
    primitives: &PrimitiveLibrary,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
    allow_unchecked_bumps: bool,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    // Crossing-aware endpoint correction calls this on source->first-crossing
    // and last-crossing->target slices. The crossing point is passed as the
    // opposite virtual port, so the crossing itself stays fixed while normal
    // endpoint correction can still use existing straights/diagonals first.
    let mut centerline = dedupe_centerline(centerline);
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&point| !is_finite_point(point)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    let source_angle = angle_for_centerline_segment(centerline[0], centerline[1]);
    let last_index = centerline.len() - 1;
    let target_angle =
        angle_for_centerline_segment(centerline[last_index - 1], centerline[last_index]);
    let straight_runs = axis_runs_from_centerline(&centerline);
    let directional_runs = directional_runs_from_centerline(&centerline);
    let dogleg_absorbers: Vec<DoglegAbsorber> = Vec::new();

    if !try_apply_full_straight_port_correction(&mut centerline, source_port_um, target_port_um)? {
        if !try_apply_full_diagonal_straight_port_correction(
            &mut centerline,
            source_port_um,
            target_port_um,
        )? {
            try_apply_shared_axis_port_shift(&mut centerline, source_port_um, target_port_um)?;
            if !try_apply_45_degree_centerline_endpoint_delta_correction(
                &mut centerline,
                &straight_runs,
                &directional_runs,
                &dogleg_absorbers,
                primitives,
                source_port_um,
                target_port_um,
                false,
                source_angle,
                target_angle,
            )? {
                let mut existing_axis_candidate = centerline.clone();
                if absorb_endpoint_delta_into_existing_axis_runs(
                    &mut existing_axis_candidate,
                    &straight_runs,
                    source_port_um,
                    target_port_um,
                )
                .is_ok()
                {
                    centerline = existing_axis_candidate;
                } else if allow_unchecked_bumps {
                    let mut bump_candidate = centerline.clone();
                    match try_apply_full_straight_offset_bump_correction_for_centerline(
                        &mut bump_candidate,
                        primitives,
                        source_port_um,
                        target_port_um,
                    ) {
                        Ok(true) => centerline = bump_candidate,
                        Ok(false) | Err(_) => absorb_endpoint_delta_into_axis_runs(
                            &mut centerline,
                            &straight_runs,
                            &dogleg_absorbers,
                            primitives,
                            source_port_um,
                            target_port_um,
                        )?,
                    }
                } else {
                    absorb_endpoint_delta_into_existing_axis_runs(
                        &mut centerline,
                        &straight_runs,
                        source_port_um,
                        target_port_um,
                    )?;
                }
            }
        }
    }

    centerline = dedupe_centerline(&centerline);
    validate_corrected_centerline_endpoint_candidate(
        &centerline,
        source_port_um,
        target_port_um,
        source_angle,
        target_angle,
    )?;
    Ok(centerline)
}

/// Replay the primitive centerline and connect physical ports only at the
/// terminal endpoints. Unlike the general endpoint correction, this deliberately
/// does not shift existing straight or diagonal runs. Crossing-aware routing
/// uses this when the grid route must remain the geometry authority.
pub fn route_to_grid_locked_port_centerline(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let mut centerline = route_to_primitive_centerline(route, primitives, grid)?;
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&point| !is_finite_point(point)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    if let Some(source) = source_port_um {
        if !is_finite_point(source) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        if distance(source, centerline[0]) > EPS {
            centerline.insert(0, source);
        } else {
            centerline[0] = source;
        }
    }
    if let Some(target) = target_port_um {
        if !is_finite_point(target) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let last = centerline.len() - 1;
        if distance(target, centerline[last]) > EPS {
            centerline.push(target);
        } else {
            centerline[last] = target;
        }
    }

    let mut deduped = Vec::with_capacity(centerline.len());
    for point in centerline.iter().copied() {
        push_physical_if_different(&mut deduped, point);
    }
    if deduped.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if deduped.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    Ok(deduped)
}

pub fn full_straight_offset_bump_candidates(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<OffsetBumpCandidate>, GeometryError> {
    let (Some(source), Some(target)) = (source_port_um, target_port_um) else {
        return Ok(Vec::new());
    };
    if !is_finite_point(source) || !is_finite_point(target) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if route.states.is_empty() {
        return Err(GeometryError::DegenerateRoute);
    }

    let replay = route_to_primitive_centerline_with_runs(route, primitives, grid)?;
    let centerline = replay.centerline;
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    let start_angle = route.states[0].angle % 8;
    let end_angle = route.states[route.states.len() - 1].angle % 8;
    if start_angle != end_angle {
        return Ok(Vec::new());
    }

    let offset_candidates = if is_full_horizontal_centerline(&centerline)
        && matches!(start_angle, 0 | 4)
        && (source.1 - target.1).abs() > EPS
    {
        [(2u8, "top"), (6u8, "bottom")]
    } else if is_full_vertical_centerline(&centerline)
        && matches!(start_angle, 2 | 6)
        && (source.0 - target.0).abs() > EPS
    {
        [(0u8, "right"), (4u8, "left")]
    } else {
        return Ok(Vec::new());
    };

    let radius_um = infer_90_bend_radius_um(primitives)?;
    let route_dir = angle_to_unit_vector(start_angle);
    let mut candidates = Vec::new();
    for placement in [OffsetBumpPlacement::Start, OffsetBumpPlacement::End] {
        let placement_label = match placement {
            OffsetBumpPlacement::Start => "start",
            OffsetBumpPlacement::End => "end",
        };
        for (offset_angle, side_label) in offset_candidates {
            if let Ok(centerline) = build_compact_four_bend_offset_bump(
                source,
                target,
                start_angle,
                offset_angle,
                radius_um,
                placement,
            ) {
                validate_bump_endpoint_tangents(&centerline, route_dir)?;
                candidates.push(OffsetBumpCandidate {
                    label: format!("{placement_label}/{side_label}"),
                    centerline,
                    placement_is_start: matches!(placement, OffsetBumpPlacement::Start),
                });
            }
        }
    }

    Ok(candidates)
}

pub fn full_straight_offset_bump_candidates_for_centerline(
    centerline: &[(f64, f64)],
    primitives: &PrimitiveLibrary,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<OffsetBumpCandidate>, GeometryError> {
    let (Some(source), Some(target)) = (source_port_um, target_port_um) else {
        return Ok(Vec::new());
    };
    if !is_finite_point(source) || !is_finite_point(target) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    let start = centerline[0];
    let end = centerline[centerline.len() - 1];
    let start_angle = if is_full_horizontal_centerline(centerline) {
        if end.0 >= start.0 {
            0u8
        } else {
            4u8
        }
    } else if is_full_vertical_centerline(centerline) {
        if end.1 >= start.1 {
            2u8
        } else {
            6u8
        }
    } else {
        return Ok(Vec::new());
    };

    let offset_candidates = if matches!(start_angle, 0 | 4) && (source.1 - target.1).abs() > EPS {
        [(2u8, "top"), (6u8, "bottom")]
    } else if matches!(start_angle, 2 | 6) && (source.0 - target.0).abs() > EPS {
        [(0u8, "right"), (4u8, "left")]
    } else {
        return Ok(Vec::new());
    };

    let radius_um = infer_90_bend_radius_um(primitives)?;
    let route_dir = angle_to_unit_vector(start_angle);
    let mut candidates = Vec::new();
    for placement in [OffsetBumpPlacement::Start, OffsetBumpPlacement::End] {
        let placement_label = match placement {
            OffsetBumpPlacement::Start => "start",
            OffsetBumpPlacement::End => "end",
        };
        for (offset_angle, side_label) in offset_candidates {
            if let Ok(centerline) = build_compact_four_bend_offset_bump(
                source,
                target,
                start_angle,
                offset_angle,
                radius_um,
                placement,
            ) {
                validate_bump_endpoint_tangents(&centerline, route_dir)?;
                candidates.push(OffsetBumpCandidate {
                    label: format!("{placement_label}/{side_label}"),
                    centerline,
                    placement_is_start: matches!(placement, OffsetBumpPlacement::Start),
                });
            }
        }
    }

    Ok(candidates)
}

fn try_apply_shared_axis_port_shift(
    centerline: &mut [(f64, f64)],
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<bool, GeometryError> {
    let (Some(source), Some(target)) = (source_port_um, target_port_um) else {
        return Ok(false);
    };
    if !is_finite_point(source) || !is_finite_point(target) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    let start = centerline[0];
    let end = centerline[centerline.len() - 1];
    let dy = source.1 - start.1;
    if (source.1 - target.1).abs() <= EPS && (target.1 - end.1 - dy).abs() <= EPS {
        for point in centerline.iter_mut() {
            point.1 += dy;
        }
        return Ok(true);
    }

    let dx = source.0 - start.0;
    if (source.0 - target.0).abs() <= EPS && (target.0 - end.0 - dx).abs() <= EPS {
        for point in centerline.iter_mut() {
            point.0 += dx;
        }
        return Ok(true);
    }

    Ok(false)
}

fn try_apply_full_straight_port_correction(
    centerline: &mut [(f64, f64)],
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<bool, GeometryError> {
    let (Some(source), Some(target)) = (source_port_um, target_port_um) else {
        return Ok(false);
    };
    if !is_finite_point(source) || !is_finite_point(target) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    if is_full_horizontal_centerline(centerline) && (source.1 - target.1).abs() <= EPS {
        for point in centerline.iter_mut() {
            point.1 = source.1;
        }
        centerline[0] = source;
        let last = centerline.len() - 1;
        centerline[last] = target;
        validate_full_straight_centerline(centerline, AxisAlignedRunKind::Horizontal)?;
        return Ok(true);
    }

    if is_full_vertical_centerline(centerline) && (source.0 - target.0).abs() <= EPS {
        for point in centerline.iter_mut() {
            point.0 = source.0;
        }
        centerline[0] = source;
        let last = centerline.len() - 1;
        centerline[last] = target;
        validate_full_straight_centerline(centerline, AxisAlignedRunKind::Vertical)?;
        return Ok(true);
    }

    Ok(false)
}

fn try_apply_full_diagonal_straight_port_correction(
    centerline: &mut [(f64, f64)],
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<bool, GeometryError> {
    let (Some(source), Some(target)) = (source_port_um, target_port_um) else {
        return Ok(false);
    };
    if !is_finite_point(source) || !is_finite_point(target) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.len() < 2 || !is_full_diagonal_centerline(centerline) {
        return Ok(false);
    }

    let base_start = centerline[0];
    let base_end = centerline[centerline.len() - 1];
    let base_delta = sub(base_end, base_start);
    let Some(angle) = diagonal_angle_for_segment(base_start, base_end) else {
        return Ok(false);
    };
    let dir = angle_to_unit_vector(angle);
    let base_forward = dot(base_delta, dir);
    let target_delta = sub(target, source);
    let target_forward = dot(target_delta, dir);
    let residual = sub(target_delta, scale(dir, target_forward));
    if base_forward <= EPS || target_forward <= EPS || length(residual) > 1.0e-6 {
        return Ok(false);
    }

    for point in centerline.iter_mut() {
        let forward = dot(sub(*point, base_start), dir);
        let t = forward / base_forward;
        *point = add(source, scale(dir, target_forward * t));
    }
    centerline[0] = source;
    let last = centerline.len() - 1;
    centerline[last] = target;
    validate_full_diagonal_centerline(centerline, angle)?;
    Ok(true)
}

fn try_apply_full_straight_offset_bump_correction(
    centerline: &mut Vec<(f64, f64)>,
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<bool, GeometryError> {
    let (Some(source), Some(target)) = (source_port_um, target_port_um) else {
        return Ok(false);
    };
    if !is_finite_point(source) || !is_finite_point(target) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.len() < 2 || route.states.is_empty() {
        return Err(GeometryError::DegenerateRoute);
    }

    let start_angle = route.states[0].angle % 8;
    let end_angle = route.states[route.states.len() - 1].angle % 8;
    if start_angle != end_angle {
        return Ok(false);
    }

    let radius_um = infer_90_bend_radius_um(primitives)?;
    let corrected = if is_full_horizontal_centerline(centerline)
        && matches!(start_angle, 0 | 4)
        && (source.1 - target.1).abs() > EPS
    {
        build_ordered_compact_offset_bump(source, target, start_angle, [2, 6], radius_um)?
    } else if is_full_vertical_centerline(centerline)
        && matches!(start_angle, 2 | 6)
        && (source.0 - target.0).abs() > EPS
    {
        build_ordered_compact_offset_bump(source, target, start_angle, [0, 4], radius_um)?
    } else {
        return Ok(false);
    };

    *centerline = corrected;
    Ok(true)
}

fn try_apply_full_straight_offset_bump_correction_for_centerline(
    centerline: &mut Vec<(f64, f64)>,
    primitives: &PrimitiveLibrary,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<bool, GeometryError> {
    let mut candidates = full_straight_offset_bump_candidates_for_centerline(
        centerline,
        primitives,
        source_port_um,
        target_port_um,
    )?;
    let Some(candidate) = candidates.drain(..).next() else {
        return Ok(false);
    };
    *centerline = candidate.centerline;
    Ok(true)
}

fn infer_90_bend_radius_um(primitives: &PrimitiveLibrary) -> Result<f64, GeometryError> {
    for angle in 0..8u8 {
        for primitive in primitives.get_primitives_for_angle(angle) {
            if let PrimitiveGeometry::Bend {
                radius_um,
                angle_delta,
            } = primitive.geometry
            {
                if angle_delta.unsigned_abs() == 2 && radius_um.is_finite() && radius_um > 0.0 {
                    return Ok(radius_um);
                }
            }
        }
    }
    Err(GeometryError::NoMeanderCandidateSegment)
}

fn build_ordered_compact_offset_bump(
    source: (f64, f64),
    target: (f64, f64),
    route_angle: u8,
    offset_angles: [u8; 2],
    radius_um: f64,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let route_dir = angle_to_unit_vector(route_angle);
    for placement in [OffsetBumpPlacement::Start, OffsetBumpPlacement::End] {
        for offset_angle in offset_angles {
            if let Ok(centerline) = build_compact_four_bend_offset_bump(
                source,
                target,
                route_angle,
                offset_angle,
                radius_um,
                placement,
            ) {
                validate_bump_endpoint_tangents(&centerline, route_dir)?;
                return Ok(centerline);
            }
        }
    }

    Err(GeometryError::NoMeanderCandidateSegment)
}

fn build_compact_offset_bump_with_side_retry(
    source: (f64, f64),
    target: (f64, f64),
    route_angle: u8,
    radius_um: f64,
    placement: OffsetBumpPlacement,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let route_dir = angle_to_unit_vector(route_angle);
    let left_angle = ((route_angle + 2) % 8) as u8;
    let right_angle = ((route_angle + 6) % 8) as u8;
    let left_dir = angle_to_unit_vector(left_angle);
    let required_offset = dot(sub(target, source), left_dir);
    let primary_offset_angle = if required_offset >= 0.0 {
        left_angle
    } else {
        right_angle
    };
    let secondary_offset_angle = ((primary_offset_angle + 4) % 8) as u8;

    for offset_angle in [primary_offset_angle, secondary_offset_angle] {
        if let Ok(centerline) = build_compact_four_bend_offset_bump(
            source,
            target,
            route_angle,
            offset_angle,
            radius_um,
            placement,
        ) {
            validate_bump_endpoint_tangents(&centerline, route_dir)?;
            return Ok(centerline);
        }
    }

    Err(GeometryError::NoMeanderCandidateSegment)
}

fn build_compact_four_bend_offset_bump(
    source: (f64, f64),
    target: (f64, f64),
    route_angle: u8,
    first_offset_angle: u8,
    radius_um: f64,
    placement: OffsetBumpPlacement,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    if !radius_um.is_finite() || radius_um <= 0.0 {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    let route_dir = angle_to_unit_vector(route_angle);
    let offset_dir = angle_to_unit_vector(first_offset_angle);
    if dot(route_dir, offset_dir).abs() > EPS {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }

    let source_to_target = sub(target, source);
    let forward_len = dot(source_to_target, route_dir);
    let offset_len = dot(source_to_target, offset_dir);
    let residual = sub(
        source_to_target,
        add(scale(route_dir, forward_len), scale(offset_dir, offset_len)),
    );
    if forward_len <= EPS || offset_len.abs() <= EPS || length(residual) > 1.0e-6 {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }

    let tail_len = forward_len - 4.0 * radius_um;
    if tail_len < -EPS {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let first_leg_len = offset_len.max(0.0);
    let second_leg_len = (-offset_len).max(0.0);
    let opposite_offset_angle = ((first_offset_angle + 4) % 8) as u8;
    let bump_start = match placement {
        OffsetBumpPlacement::Start => source,
        OffsetBumpPlacement::End => sub(
            sub(target, scale(route_dir, 4.0 * radius_um)),
            scale(offset_dir, offset_len),
        ),
    };
    let bump_end = add(
        add(bump_start, scale(route_dir, 4.0 * radius_um)),
        scale(offset_dir, offset_len),
    );

    let mut out = Vec::new();
    if matches!(placement, OffsetBumpPlacement::End) {
        push_physical_if_different(&mut out, source);
    }
    push_physical_if_different(&mut out, bump_start);

    let p1 = add(
        add(bump_start, scale(route_dir, radius_um)),
        scale(offset_dir, radius_um),
    );
    append_bump_bend(
        &mut out,
        bump_start,
        route_angle,
        p1,
        first_offset_angle,
        radius_um,
    )?;

    let p2 = add(p1, scale(offset_dir, first_leg_len));
    push_physical_if_different(&mut out, p2);

    let p3 = add(
        add(p2, scale(offset_dir, radius_um)),
        scale(route_dir, radius_um),
    );
    append_bump_bend(&mut out, p2, first_offset_angle, p3, route_angle, radius_um)?;

    let p4 = add(
        add(p3, scale(route_dir, radius_um)),
        scale(offset_dir, -radius_um),
    );
    append_bump_bend(
        &mut out,
        p3,
        route_angle,
        p4,
        opposite_offset_angle,
        radius_um,
    )?;

    let p5 = add(p4, scale(offset_dir, -second_leg_len));
    push_physical_if_different(&mut out, p5);

    let p6 = add(
        add(p5, scale(offset_dir, -radius_um)),
        scale(route_dir, radius_um),
    );
    append_bump_bend(
        &mut out,
        p5,
        opposite_offset_angle,
        p6,
        route_angle,
        radius_um,
    )?;

    if distance(p6, bump_end) > 1.0e-6 {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    if matches!(placement, OffsetBumpPlacement::Start) {
        push_physical_if_different(&mut out, target);
    } else {
        push_physical_if_different(&mut out, bump_end);
    }
    validate_bump_endpoint_tangents(&out, route_dir)?;
    if out.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    Ok(out)
}

fn append_bump_bend(
    out: &mut Vec<(f64, f64)>,
    start_point: (f64, f64),
    start_angle: u8,
    end_point: (f64, f64),
    end_angle: u8,
    radius_um: f64,
) -> Result<(), GeometryError> {
    append_circular_bend_centerline(
        out,
        start_point,
        start_angle,
        end_point,
        end_angle,
        radius_um,
        signed_angle_delta(start_angle, end_angle)?,
        arc_samples_per_90_deg(radius_um),
    )
}

fn signed_angle_delta(start_angle: u8, end_angle: u8) -> Result<i8, GeometryError> {
    let mut delta = (end_angle % 8) as i8 - (start_angle % 8) as i8;
    while delta > 4 {
        delta -= 8;
    }
    while delta <= -4 {
        delta += 8;
    }
    if delta == 0 {
        return Err(GeometryError::DegenerateRoute);
    }
    Ok(delta)
}

fn validate_bump_endpoint_tangents(
    centerline: &[(f64, f64)],
    route_dir: (f64, f64),
) -> Result<(), GeometryError> {
    validate_source_tangent(centerline, route_dir)?;
    validate_target_tangent(centerline, route_dir)?;
    Ok(())
}

fn absorb_endpoint_delta_into_axis_runs(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    dogleg_absorbers: &[DoglegAbsorber],
    primitives: &PrimitiveLibrary,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    let mut runs = straight_runs.to_vec();
    let mut doglegs = dogleg_absorbers.to_vec();

    if let Some(source) = source_port_um {
        if !is_finite_point(source) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let start = centerline[0];
        let dx = source.0 - start.0;
        let dy = source.1 - start.1;
        let mutation = absorb_source_x_delta(centerline, &runs, &mut doglegs, primitives, dx)?;
        apply_endpoint_delta_mutation(centerline, &mut runs, &mut doglegs, mutation);
        let mutation = absorb_source_y_delta(centerline, &runs, &mut doglegs, primitives, dy)?;
        apply_endpoint_delta_mutation(centerline, &mut runs, &mut doglegs, mutation);
    }
    if let Some(target) = target_port_um {
        if !is_finite_point(target) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let last = centerline[centerline.len() - 1];
        let dx = target.0 - last.0;
        let dy = target.1 - last.1;
        let mutation = absorb_target_x_delta(centerline, &runs, &mut doglegs, primitives, dx)?;
        apply_endpoint_delta_mutation(centerline, &mut runs, &mut doglegs, mutation);
        let mutation = absorb_target_y_delta(centerline, &runs, &mut doglegs, primitives, dy)?;
        apply_endpoint_delta_mutation(centerline, &mut runs, &mut doglegs, mutation);
    }
    Ok(())
}

fn absorb_endpoint_delta_into_existing_axis_runs(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }

    if let Some(source) = source_port_um {
        if !is_finite_point(source) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let start = centerline[0];
        shift_source_existing_axis_run_x(centerline, straight_runs, source.0 - start.0)?;
        shift_source_existing_axis_run_y(centerline, straight_runs, source.1 - centerline[0].1)?;
    }
    if let Some(target) = target_port_um {
        if !is_finite_point(target) {
            return Err(GeometryError::NonFiniteCoordinate);
        }
        let last_index = centerline.len() - 1;
        shift_target_existing_axis_run_x(
            centerline,
            straight_runs,
            target.0 - centerline[last_index].0,
        )?;
        shift_target_existing_axis_run_y(
            centerline,
            straight_runs,
            target.1 - centerline[last_index].1,
        )?;
    }
    Ok(())
}

fn shift_source_existing_axis_run_x(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dx: f64,
) -> Result<(), GeometryError> {
    if dx.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = first_run(straight_runs, AxisAlignedRunKind::Horizontal) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().take(run.start_index + 1) {
        point.0 += dx;
    }
    Ok(())
}

fn shift_source_existing_axis_run_y(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dy: f64,
) -> Result<(), GeometryError> {
    if dy.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = first_run(straight_runs, AxisAlignedRunKind::Vertical) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().take(run.start_index + 1) {
        point.1 += dy;
    }
    Ok(())
}

fn shift_target_existing_axis_run_x(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dx: f64,
) -> Result<(), GeometryError> {
    if dx.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = last_run(straight_runs, AxisAlignedRunKind::Horizontal) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().skip(run.end_index) {
        point.0 += dx;
    }
    Ok(())
}

fn shift_target_existing_axis_run_y(
    centerline: &mut [(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    dy: f64,
) -> Result<(), GeometryError> {
    if dy.abs() <= EPS {
        return Ok(());
    }
    let Some(run) = last_run(straight_runs, AxisAlignedRunKind::Vertical) else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    for point in centerline.iter_mut().skip(run.end_index) {
        point.1 += dy;
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct DiagonalEndpointAdjustment {
    run: DirectionalStraightRun,
    delta: (f64, f64),
}

#[allow(clippy::too_many_arguments)]
fn try_apply_45_degree_endpoint_delta_correction(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    directional_runs: &[DirectionalStraightRun],
    dogleg_absorbers: &[DoglegAbsorber],
    primitives: &PrimitiveLibrary,
    route: &RouteResult,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
    allow_unchecked_bumps: bool,
    collision_check: Option<&dyn Fn(&[(f64, f64)]) -> bool>,
) -> Result<bool, GeometryError> {
    // `collision_check`, when given, is called on every candidate centerline
    // this function is about to accept, in addition to the existing
    // `validate_corrected_endpoint_candidate` geometric/kinematic check. It
    // must return true when the candidate is safe to accept (no collision
    // with another net's already-committed geometry) and false when the
    // candidate should be rejected so this function's existing "try the
    // next candidate" loop moves on, exactly the same way a failed
    // `validate_corrected_endpoint_candidate` already does. This function
    // was identified (2026-08-19,
    // .agent/execplans/2026-08-19-collision-avoiding-endpoint-correction.md,
    // Milestone 1) as the actual strategy that produced
    // `multiportmmi_16x16`'s n_196/n_203 collisions: it already tries
    // multiple candidates (the axis-only absorption, then every
    // (source_adjustment, target_adjustment) pair below) and accepts the
    // first one that is internally consistent, but had no way to know one
    // of those candidates physically overlapped a different net until the
    // caller checked the final, already-chosen answer after the fact, too
    // late to try another candidate. Only the caller
    // (`route_port_corrected_centerline_checked_and_commit_native`,
    // `src/py_router.rs`) has the obstacle-map access needed to build a
    // real check; every other caller of this function passes `None`,
    // preserving today's behavior exactly.
    if !directional_runs.iter().any(|run| run.angle % 2 == 1) {
        return Ok(false);
    }

    let mut axis_only = centerline.clone();
    let axis_only_result = if allow_unchecked_bumps {
        absorb_endpoint_delta_into_axis_runs(
            &mut axis_only,
            straight_runs,
            dogleg_absorbers,
            primitives,
            source_port_um,
            target_port_um,
        )
    } else {
        absorb_endpoint_delta_into_existing_axis_runs(
            &mut axis_only,
            straight_runs,
            source_port_um,
            target_port_um,
        )
    };
    if axis_only_result.is_ok()
        && validate_corrected_endpoint_candidate(&axis_only, route, source_port_um, target_port_um)
            .is_ok()
        && collision_check.is_none_or(|check| check(&axis_only))
    {
        *centerline = axis_only;
        return Ok(true);
    }

    let source_adjustments =
        diagonal_endpoint_adjustments(centerline, directional_runs, source_port_um, true)?;
    let target_adjustments =
        diagonal_endpoint_adjustments(centerline, directional_runs, target_port_um, false)?;

    for source_adjustment in &source_adjustments {
        for target_adjustment in &target_adjustments {
            if source_adjustment.is_none() && target_adjustment.is_none() {
                continue;
            }
            let mut candidate = centerline.clone();
            if let Some(adjustment) = source_adjustment {
                if apply_diagonal_endpoint_adjustment(&mut candidate, *adjustment, true).is_err() {
                    continue;
                }
            }
            if let Some(adjustment) = target_adjustment {
                if apply_diagonal_endpoint_adjustment(&mut candidate, *adjustment, false).is_err() {
                    continue;
                }
            }
            let residual_result = if allow_unchecked_bumps {
                absorb_endpoint_delta_into_axis_runs(
                    &mut candidate,
                    straight_runs,
                    dogleg_absorbers,
                    primitives,
                    source_port_um,
                    target_port_um,
                )
            } else {
                absorb_endpoint_delta_into_existing_axis_runs(
                    &mut candidate,
                    straight_runs,
                    source_port_um,
                    target_port_um,
                )
            };
            if residual_result.is_err() {
                continue;
            }
            if validate_corrected_endpoint_candidate(
                &candidate,
                route,
                source_port_um,
                target_port_um,
            )
            .is_err()
            {
                continue;
            }
            if !collision_check.is_none_or(|check| check(&candidate)) {
                continue;
            }
            *centerline = candidate;
            return Ok(true);
        }
    }

    Ok(false)
}

#[allow(clippy::too_many_arguments)]
fn try_apply_45_degree_centerline_endpoint_delta_correction(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    directional_runs: &[DirectionalStraightRun],
    dogleg_absorbers: &[DoglegAbsorber],
    primitives: &PrimitiveLibrary,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
    allow_unchecked_bumps: bool,
    source_angle: Option<u8>,
    target_angle: Option<u8>,
) -> Result<bool, GeometryError> {
    if !directional_runs.iter().any(|run| run.angle % 2 == 1) {
        return Ok(false);
    }

    let mut axis_only = centerline.clone();
    let axis_only_result = if allow_unchecked_bumps {
        absorb_endpoint_delta_into_axis_runs(
            &mut axis_only,
            straight_runs,
            dogleg_absorbers,
            primitives,
            source_port_um,
            target_port_um,
        )
    } else {
        absorb_endpoint_delta_into_existing_axis_runs(
            &mut axis_only,
            straight_runs,
            source_port_um,
            target_port_um,
        )
    };
    if axis_only_result.is_ok()
        && validate_corrected_centerline_endpoint_candidate(
            &axis_only,
            source_port_um,
            target_port_um,
            source_angle,
            target_angle,
        )
        .is_ok()
    {
        *centerline = axis_only;
        return Ok(true);
    }

    let source_adjustments =
        diagonal_endpoint_adjustments(centerline, directional_runs, source_port_um, true)?;
    let target_adjustments =
        diagonal_endpoint_adjustments(centerline, directional_runs, target_port_um, false)?;

    for source_adjustment in &source_adjustments {
        for target_adjustment in &target_adjustments {
            if source_adjustment.is_none() && target_adjustment.is_none() {
                continue;
            }
            let mut candidate = centerline.clone();
            if let Some(adjustment) = source_adjustment {
                if apply_diagonal_endpoint_adjustment(&mut candidate, *adjustment, true).is_err() {
                    continue;
                }
            }
            if let Some(adjustment) = target_adjustment {
                if apply_diagonal_endpoint_adjustment(&mut candidate, *adjustment, false).is_err() {
                    continue;
                }
            }
            let residual_result = if allow_unchecked_bumps {
                absorb_endpoint_delta_into_axis_runs(
                    &mut candidate,
                    straight_runs,
                    dogleg_absorbers,
                    primitives,
                    source_port_um,
                    target_port_um,
                )
            } else {
                absorb_endpoint_delta_into_existing_axis_runs(
                    &mut candidate,
                    straight_runs,
                    source_port_um,
                    target_port_um,
                )
            };
            if residual_result.is_err() {
                continue;
            }
            if validate_corrected_centerline_endpoint_candidate(
                &candidate,
                source_port_um,
                target_port_um,
                source_angle,
                target_angle,
            )
            .is_err()
            {
                continue;
            }
            *centerline = candidate;
            return Ok(true);
        }
    }

    Ok(false)
}

fn diagonal_endpoint_adjustments(
    centerline: &[(f64, f64)],
    directional_runs: &[DirectionalStraightRun],
    port_um: Option<(f64, f64)>,
    source_side: bool,
) -> Result<Vec<Option<DiagonalEndpointAdjustment>>, GeometryError> {
    let mut out = vec![None];
    let Some(port) = port_um else {
        return Ok(out);
    };
    if !is_finite_point(port) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    let current = if source_side {
        centerline[0]
    } else {
        centerline[centerline.len() - 1]
    };
    let endpoint_delta = sub(port, current);
    if length(endpoint_delta) <= EPS {
        return Ok(out);
    }

    for run in directional_runs
        .iter()
        .copied()
        .filter(|run| run.angle % 2 == 1)
    {
        for delta in diagonal_absorption_vectors(endpoint_delta, run.angle) {
            let candidate = Some(DiagonalEndpointAdjustment { run, delta });
            if !out.contains(&candidate) {
                out.push(candidate);
            }
        }
    }
    Ok(out)
}

fn diagonal_absorption_vectors(endpoint_delta: (f64, f64), angle: u8) -> Vec<(f64, f64)> {
    let Some((sx, sy)) = diagonal_signs(angle) else {
        return Vec::new();
    };
    let sx = sx as f64;
    let sy = sy as f64;
    let mut out = Vec::with_capacity(2);
    let kx = endpoint_delta.0 / sx;
    if kx.is_finite() && kx.abs() > EPS {
        out.push((sx * kx, sy * kx));
    }
    let ky = endpoint_delta.1 / sy;
    if ky.is_finite() && ky.abs() > EPS {
        let delta = (sx * ky, sy * ky);
        if !out.iter().any(|&existing| distance(existing, delta) <= EPS) {
            out.push(delta);
        }
    }
    out
}

fn diagonal_signs(angle: u8) -> Option<(i8, i8)> {
    match angle % 8 {
        1 => Some((1, 1)),
        3 => Some((-1, 1)),
        5 => Some((-1, -1)),
        7 => Some((1, -1)),
        _ => None,
    }
}

fn apply_diagonal_endpoint_adjustment(
    centerline: &mut [(f64, f64)],
    adjustment: DiagonalEndpointAdjustment,
    source_side: bool,
) -> Result<(), GeometryError> {
    if adjustment.run.end_index >= centerline.len()
        || adjustment.run.start_index >= adjustment.run.end_index
    {
        return Err(GeometryError::DegenerateRoute);
    }
    if source_side {
        for point in centerline.iter_mut().take(adjustment.run.start_index + 1) {
            *point = add(*point, adjustment.delta);
        }
    } else {
        for point in centerline.iter_mut().skip(adjustment.run.end_index) {
            *point = add(*point, adjustment.delta);
        }
    }
    validate_directional_run_segment(centerline, adjustment.run)
}

fn validate_directional_run_segment(
    centerline: &[(f64, f64)],
    run: DirectionalStraightRun,
) -> Result<(), GeometryError> {
    if run.end_index >= centerline.len() || run.start_index >= run.end_index {
        return Err(GeometryError::DegenerateRoute);
    }
    let delta = sub(centerline[run.end_index], centerline[run.start_index]);
    if segment_is_aligned_with_dir(delta, angle_to_unit_vector(run.angle)) {
        Ok(())
    } else {
        Err(GeometryError::DegenerateRoute)
    }
}

fn validate_corrected_endpoint_candidate(
    centerline: &[(f64, f64)],
    route: &RouteResult,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 || route.states.is_empty() {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&point| !is_finite_point(point)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    if let Some(source) = source_port_um {
        if distance(centerline[0], source) > 1.0e-6 {
            return Err(GeometryError::PortEndpointCorrectionRequiresUnsupportedStub);
        }
        validate_source_tangent(centerline, angle_to_unit_vector(route.states[0].angle))?;
    }
    if let Some(target) = target_port_um {
        if distance(centerline[centerline.len() - 1], target) > 1.0e-6 {
            return Err(GeometryError::PortEndpointCorrectionRequiresUnsupportedStub);
        }
        validate_target_tangent(
            centerline,
            angle_to_unit_vector(route.states[route.states.len() - 1].angle),
        )?;
    }
    Ok(())
}

fn validate_corrected_centerline_endpoint_candidate(
    centerline: &[(f64, f64)],
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
    source_angle: Option<u8>,
    target_angle: Option<u8>,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&point| !is_finite_point(point)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if centerline.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    if let Some(source) = source_port_um {
        if distance(centerline[0], source) > 1.0e-6 {
            return Err(GeometryError::PortEndpointCorrectionRequiresUnsupportedStub);
        }
        if let Some(source_angle) = source_angle {
            validate_source_tangent(centerline, angle_to_unit_vector(source_angle))?;
        }
    }
    if let Some(target) = target_port_um {
        if distance(centerline[centerline.len() - 1], target) > 1.0e-6 {
            return Err(GeometryError::PortEndpointCorrectionRequiresUnsupportedStub);
        }
        if let Some(target_angle) = target_angle {
            validate_target_tangent(centerline, angle_to_unit_vector(target_angle))?;
        }
    }
    Ok(())
}

fn insert_terminal_tangent_stubs(
    centerline: &mut Vec<(f64, f64)>,
    route: &RouteResult,
    _stub_len_um: f64,
    source_enabled: bool,
    target_enabled: bool,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 || route.states.is_empty() || !_stub_len_um.is_finite() {
        return Err(GeometryError::DegenerateRoute);
    }

    if source_enabled {
        let dir = angle_to_unit_vector(route.states[0].angle);
        validate_source_tangent(centerline, dir)?;
    }
    if target_enabled {
        let dir = angle_to_unit_vector(route.states[route.states.len() - 1].angle);
        validate_target_tangent(centerline, dir)?;
    }
    Ok(())
}

fn apply_endpoint_delta_mutation(
    centerline: &[(f64, f64)],
    runs: &mut Vec<AxisAlignedRun>,
    doglegs: &mut Vec<DoglegAbsorber>,
    mutation: EndpointDeltaMutation,
) {
    match mutation {
        EndpointDeltaMutation::Unchanged | EndpointDeltaMutation::CoordinatesOnly => {}
        EndpointDeltaMutation::InsertedPoint { after_index } => {
            update_axis_runs_after_insert(runs, after_index);
        }
        EndpointDeltaMutation::RebuiltCenterline => {
            *runs = axis_runs_from_centerline(centerline);
            doglegs.clear();
        }
    }
}

fn terminal_tangent_stub_len_um(width_um: f64) -> Result<f64, GeometryError> {
    if !width_um.is_finite() || width_um <= 0.0 {
        return Err(GeometryError::InvalidWidth(width_um));
    }
    Ok((width_um * 0.25).max(EPS))
}

fn validate_source_tangent(
    centerline: &[(f64, f64)],
    dir: (f64, f64),
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    let start = centerline[0];
    let next = centerline[1];
    let delta = sub(next, start);
    if segment_is_aligned_with_dir(delta, dir) {
        return Ok(());
    }
    if centerline.len() >= 3
        && sampled_arc_endpoint_tangent_aligned(centerline[0], centerline[1], centerline[2], dir)
    {
        return Ok(());
    }
    Err(GeometryError::PortEndpointCorrectionRequiresUnsupportedStub)
}

fn validate_target_tangent(
    centerline: &[(f64, f64)],
    dir: (f64, f64),
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    let last_index = centerline.len() - 1;
    let prev = centerline[last_index - 1];
    let end = centerline[last_index];
    let delta = sub(end, prev);
    if segment_is_aligned_with_dir(delta, dir) {
        return Ok(());
    }
    if centerline.len() >= 3
        && sampled_arc_endpoint_tangent_aligned(
            centerline[last_index],
            centerline[last_index - 1],
            centerline[last_index - 2],
            scale(dir, -1.0),
        )
    {
        return Ok(());
    }
    Err(GeometryError::PortEndpointCorrectionRequiresUnsupportedStub)
}

fn segment_is_aligned_with_dir(delta: (f64, f64), dir: (f64, f64)) -> bool {
    let len = length(delta);
    if len <= EPS {
        return false;
    }
    let projection = dot(delta, dir);
    projection > EPS && cross(delta, dir).abs() <= EPS * len.max(1.0)
}

fn sampled_arc_endpoint_tangent_aligned(
    endpoint: (f64, f64),
    next: (f64, f64),
    next_next: (f64, f64),
    dir: (f64, f64),
) -> bool {
    let Some(center) = circumcenter(endpoint, next, next_next) else {
        return false;
    };
    let radius_vec = sub(endpoint, center);
    let radius_len = length(radius_vec);
    if radius_len <= EPS {
        return false;
    }
    let chord = sub(next, endpoint);
    let tangent_left = scale(rotate_left(radius_vec), 1.0 / radius_len);
    let tangent_right = scale(rotate_right(radius_vec), 1.0 / radius_len);
    let tangent = if dot(tangent_left, chord) >= dot(tangent_right, chord) {
        tangent_left
    } else {
        tangent_right
    };
    dot(tangent, dir) > 0.0 && cross(tangent, dir).abs() <= 1.0e-6
}

fn circumcenter(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<(f64, f64)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if !d.is_finite() || d.abs() <= EPS {
        return None;
    }
    let aa = a.0 * a.0 + a.1 * a.1;
    let bb = b.0 * b.0 + b.1 * b.1;
    let cc = c.0 * c.0 + c.1 * c.1;
    let ux = (aa * (b.1 - c.1) + bb * (c.1 - a.1) + cc * (a.1 - b.1)) / d;
    let uy = (aa * (c.0 - b.0) + bb * (a.0 - c.0) + cc * (b.0 - a.0)) / d;
    if ux.is_finite() && uy.is_finite() {
        Some((ux, uy))
    } else {
        None
    }
}

fn absorb_source_x_delta(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    dogleg_absorbers: &mut Vec<DoglegAbsorber>,
    primitives: &PrimitiveLibrary,
    dx: f64,
) -> Result<EndpointDeltaMutation, GeometryError> {
    if dx.abs() <= EPS {
        return Ok(EndpointDeltaMutation::Unchanged);
    }
    let Some(run) = first_run(straight_runs, AxisAlignedRunKind::Horizontal) else {
        if let Ok(after_index) = insert_source_delta_dogleg(
            centerline,
            dogleg_absorbers,
            AxisAlignedRunKind::Horizontal,
            dx,
        ) {
            return Ok(EndpointDeltaMutation::InsertedPoint { after_index });
        }
        insert_source_delta_bump(
            centerline,
            straight_runs,
            primitives,
            AxisAlignedRunKind::Vertical,
            dx,
        )?;
        return Ok(EndpointDeltaMutation::RebuiltCenterline);
    };
    for point in centerline.iter_mut().take(run.start_index + 1) {
        point.0 += dx;
    }
    Ok(EndpointDeltaMutation::CoordinatesOnly)
}

fn absorb_source_y_delta(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    dogleg_absorbers: &mut Vec<DoglegAbsorber>,
    primitives: &PrimitiveLibrary,
    dy: f64,
) -> Result<EndpointDeltaMutation, GeometryError> {
    if dy.abs() <= EPS {
        return Ok(EndpointDeltaMutation::Unchanged);
    }
    let Some(run) = first_run(straight_runs, AxisAlignedRunKind::Vertical) else {
        if let Ok(after_index) = insert_source_delta_dogleg(
            centerline,
            dogleg_absorbers,
            AxisAlignedRunKind::Vertical,
            dy,
        ) {
            return Ok(EndpointDeltaMutation::InsertedPoint { after_index });
        }
        insert_source_delta_bump(
            centerline,
            straight_runs,
            primitives,
            AxisAlignedRunKind::Horizontal,
            dy,
        )?;
        return Ok(EndpointDeltaMutation::RebuiltCenterline);
    };
    for point in centerline.iter_mut().take(run.start_index + 1) {
        point.1 += dy;
    }
    Ok(EndpointDeltaMutation::CoordinatesOnly)
}

fn absorb_target_x_delta(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    dogleg_absorbers: &mut Vec<DoglegAbsorber>,
    primitives: &PrimitiveLibrary,
    dx: f64,
) -> Result<EndpointDeltaMutation, GeometryError> {
    if dx.abs() <= EPS {
        return Ok(EndpointDeltaMutation::Unchanged);
    }
    let Some(run) = last_run(straight_runs, AxisAlignedRunKind::Horizontal) else {
        if let Ok(after_index) = insert_target_delta_dogleg(
            centerline,
            dogleg_absorbers,
            AxisAlignedRunKind::Horizontal,
            dx,
        ) {
            return Ok(EndpointDeltaMutation::InsertedPoint { after_index });
        }
        insert_target_delta_bump(
            centerline,
            straight_runs,
            primitives,
            AxisAlignedRunKind::Vertical,
            dx,
        )?;
        return Ok(EndpointDeltaMutation::RebuiltCenterline);
    };
    for point in centerline.iter_mut().skip(run.end_index) {
        point.0 += dx;
    }
    Ok(EndpointDeltaMutation::CoordinatesOnly)
}

fn absorb_target_y_delta(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    dogleg_absorbers: &mut Vec<DoglegAbsorber>,
    primitives: &PrimitiveLibrary,
    dy: f64,
) -> Result<EndpointDeltaMutation, GeometryError> {
    if dy.abs() <= EPS {
        return Ok(EndpointDeltaMutation::Unchanged);
    }
    let Some(run) = last_run(straight_runs, AxisAlignedRunKind::Vertical) else {
        if let Ok(after_index) = insert_target_delta_dogleg(
            centerline,
            dogleg_absorbers,
            AxisAlignedRunKind::Vertical,
            dy,
        ) {
            return Ok(EndpointDeltaMutation::InsertedPoint { after_index });
        }
        insert_target_delta_bump(
            centerline,
            straight_runs,
            primitives,
            AxisAlignedRunKind::Horizontal,
            dy,
        )?;
        return Ok(EndpointDeltaMutation::RebuiltCenterline);
    };
    for point in centerline.iter_mut().skip(run.end_index) {
        point.1 += dy;
    }
    Ok(EndpointDeltaMutation::CoordinatesOnly)
}

fn first_run(straight_runs: &[AxisAlignedRun], kind: AxisAlignedRunKind) -> Option<AxisAlignedRun> {
    straight_runs.iter().copied().find(|run| run.kind == kind)
}

fn last_run(straight_runs: &[AxisAlignedRun], kind: AxisAlignedRunKind) -> Option<AxisAlignedRun> {
    straight_runs.iter().copied().rfind(|run| run.kind == kind)
}

fn insert_source_delta_dogleg(
    centerline: &mut Vec<(f64, f64)>,
    dogleg_absorbers: &mut Vec<DoglegAbsorber>,
    kind: AxisAlignedRunKind,
    delta: f64,
) -> Result<usize, GeometryError> {
    if delta.abs() <= EPS {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let required_dir = direction_sign(-delta);
    let Some(pos) = dogleg_absorbers
        .iter()
        .position(|absorber| absorber.kind == kind && absorber.dir == required_dir)
    else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    let transition_index = dogleg_absorbers[pos].transition_index;
    if transition_index >= centerline.len() {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }

    let original_transition = centerline[transition_index];
    match kind {
        AxisAlignedRunKind::Horizontal => {
            for point in centerline.iter_mut().take(transition_index + 1) {
                point.0 += delta;
            }
        }
        AxisAlignedRunKind::Vertical => {
            for point in centerline.iter_mut().take(transition_index + 1) {
                point.1 += delta;
            }
        }
    }
    centerline.insert(transition_index + 1, original_transition);
    update_dogleg_absorbers_after_insert(dogleg_absorbers, transition_index);
    Ok(transition_index)
}

fn insert_target_delta_dogleg(
    centerline: &mut Vec<(f64, f64)>,
    dogleg_absorbers: &mut Vec<DoglegAbsorber>,
    kind: AxisAlignedRunKind,
    delta: f64,
) -> Result<usize, GeometryError> {
    if delta.abs() <= EPS {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let required_dir = direction_sign(delta);
    let Some(pos) = dogleg_absorbers
        .iter()
        .rposition(|absorber| absorber.kind == kind && absorber.dir == required_dir)
    else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };
    let transition_index = dogleg_absorbers[pos].transition_index;
    if transition_index >= centerline.len() {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }

    let shifted_transition = match kind {
        AxisAlignedRunKind::Horizontal => {
            for point in centerline.iter_mut().skip(transition_index + 1) {
                point.0 += delta;
            }
            (
                centerline[transition_index].0 + delta,
                centerline[transition_index].1,
            )
        }
        AxisAlignedRunKind::Vertical => {
            for point in centerline.iter_mut().skip(transition_index + 1) {
                point.1 += delta;
            }
            (
                centerline[transition_index].0,
                centerline[transition_index].1 + delta,
            )
        }
    };
    centerline.insert(transition_index + 1, shifted_transition);
    update_dogleg_absorbers_after_insert(dogleg_absorbers, transition_index);
    Ok(transition_index)
}

fn direction_sign(delta: f64) -> i8 {
    if delta >= 0.0 {
        1
    } else {
        -1
    }
}

fn update_dogleg_absorbers_after_insert(
    dogleg_absorbers: &mut Vec<DoglegAbsorber>,
    transition_index: usize,
) {
    dogleg_absorbers.retain(|absorber| absorber.transition_index != transition_index);
    for absorber in dogleg_absorbers.iter_mut() {
        if absorber.transition_index > transition_index {
            absorber.transition_index += 1;
        }
    }
}

fn update_axis_runs_after_insert(runs: &mut [AxisAlignedRun], after_index: usize) {
    for run in runs.iter_mut() {
        if run.start_index > after_index {
            run.start_index += 1;
        }
        if run.end_index > after_index {
            run.end_index += 1;
        }
    }
}

fn axis_runs_from_centerline(centerline: &[(f64, f64)]) -> Vec<AxisAlignedRun> {
    let mut runs = Vec::new();
    let mut active: Option<(AxisAlignedRun, i8, f64)> = None;
    for (idx, window) in centerline.windows(2).enumerate() {
        let segment = if is_horizontal_segment(window[0], window[1]) {
            let dx = window[1].0 - window[0].0;
            Some((
                AxisAlignedRunKind::Horizontal,
                if dx > 0.0 { 1 } else { -1 },
                window[0].1,
            ))
        } else if is_vertical_segment(window[0], window[1]) {
            let dy = window[1].1 - window[0].1;
            Some((
                AxisAlignedRunKind::Vertical,
                if dy > 0.0 { 1 } else { -1 },
                window[0].0,
            ))
        } else {
            None
        };

        let Some((kind, dir, line_coord)) = segment else {
            if let Some((run, _, _)) = active.take() {
                runs.push(run);
            }
            continue;
        };

        if let Some((mut run, active_dir, active_line_coord)) = active.take() {
            if run.kind == kind
                && active_dir == dir
                && (active_line_coord - line_coord).abs() <= EPS
            {
                run.end_index = idx + 1;
                active = Some((run, active_dir, active_line_coord));
            } else {
                runs.push(run);
                active = Some((
                    AxisAlignedRun {
                        start_index: idx,
                        end_index: idx + 1,
                        kind,
                    },
                    dir,
                    line_coord,
                ));
            }
        } else {
            active = Some((
                AxisAlignedRun {
                    start_index: idx,
                    end_index: idx + 1,
                    kind,
                },
                dir,
                line_coord,
            ));
        }
    }
    if let Some((run, _, _)) = active {
        runs.push(run);
    }
    runs
}

fn directional_runs_from_centerline(centerline: &[(f64, f64)]) -> Vec<DirectionalStraightRun> {
    let mut runs = Vec::new();
    let mut active: Option<DirectionalStraightRun> = None;
    for (idx, window) in centerline.windows(2).enumerate() {
        let Some(angle) = angle_for_centerline_segment(window[0], window[1]) else {
            if let Some(run) = active.take() {
                runs.push(run);
            }
            continue;
        };
        if let Some(mut run) = active.take() {
            if run.angle == angle {
                run.end_index = idx + 1;
                active = Some(run);
            } else {
                runs.push(run);
                active = Some(DirectionalStraightRun {
                    start_index: idx,
                    end_index: idx + 1,
                    angle,
                });
            }
        } else {
            active = Some(DirectionalStraightRun {
                start_index: idx,
                end_index: idx + 1,
                angle,
            });
        }
    }
    if let Some(run) = active {
        runs.push(run);
    }
    runs
}

fn insert_source_delta_bump(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    primitives: &PrimitiveLibrary,
    carrier_kind: AxisAlignedRunKind,
    delta: f64,
) -> Result<(), GeometryError> {
    let Some(run) = first_viable_delta_bump_run(
        centerline,
        straight_runs,
        primitives,
        carrier_kind,
        delta,
        true,
    )?
    else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };

    let bump = delta_bump_for_run(centerline, run, primitives, carrier_kind, delta, true)?;

    match carrier_kind {
        AxisAlignedRunKind::Horizontal => {
            for point in centerline.iter_mut().take(run.start_index + 1) {
                point.1 += delta;
            }
        }
        AxisAlignedRunKind::Vertical => {
            for point in centerline.iter_mut().take(run.start_index + 1) {
                point.0 += delta;
            }
        }
    }
    splice_precomputed_delta_bump(centerline, run, bump)
}

fn insert_target_delta_bump(
    centerline: &mut Vec<(f64, f64)>,
    straight_runs: &[AxisAlignedRun],
    primitives: &PrimitiveLibrary,
    carrier_kind: AxisAlignedRunKind,
    delta: f64,
) -> Result<(), GeometryError> {
    let Some(run) = first_viable_delta_bump_run(
        centerline,
        straight_runs,
        primitives,
        carrier_kind,
        delta,
        false,
    )?
    else {
        return Err(GeometryError::NoMeanderCandidateSegment);
    };

    let bump = delta_bump_for_run(centerline, run, primitives, carrier_kind, delta, false)?;

    match carrier_kind {
        AxisAlignedRunKind::Horizontal => {
            for point in centerline.iter_mut().skip(run.end_index) {
                point.1 += delta;
            }
        }
        AxisAlignedRunKind::Vertical => {
            for point in centerline.iter_mut().skip(run.end_index) {
                point.0 += delta;
            }
        }
    }
    splice_precomputed_delta_bump(centerline, run, bump)
}

fn first_viable_delta_bump_run(
    centerline: &[(f64, f64)],
    straight_runs: &[AxisAlignedRun],
    primitives: &PrimitiveLibrary,
    carrier_kind: AxisAlignedRunKind,
    delta: f64,
    source_side: bool,
) -> Result<Option<AxisAlignedRun>, GeometryError> {
    for run in straight_runs
        .iter()
        .copied()
        .filter(|run| run.kind == carrier_kind)
    {
        if delta_bump_for_run(
            centerline,
            run,
            primitives,
            carrier_kind,
            delta,
            source_side,
        )
        .is_ok()
        {
            return Ok(Some(run));
        }
    }
    Ok(None)
}

fn splice_precomputed_delta_bump(
    centerline: &mut Vec<(f64, f64)>,
    run: AxisAlignedRun,
    bump: Vec<(f64, f64)>,
) -> Result<(), GeometryError> {
    let mut replacement = Vec::with_capacity(centerline.len() + bump.len());
    replacement.extend_from_slice(&centerline[..run.start_index]);
    replacement.extend(bump);
    replacement.extend_from_slice(&centerline[(run.end_index + 1)..]);
    *centerline = replacement;
    Ok(())
}

fn delta_bump_for_run(
    centerline: &[(f64, f64)],
    run: AxisAlignedRun,
    primitives: &PrimitiveLibrary,
    carrier_kind: AxisAlignedRunKind,
    delta: f64,
    source_side: bool,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    if run.end_index >= centerline.len() || delta.abs() <= EPS {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let start = centerline[run.start_index];
    let end = centerline[run.end_index];
    let shifted_start = match carrier_kind {
        AxisAlignedRunKind::Horizontal => (start.0, start.1 + delta),
        AxisAlignedRunKind::Vertical => (start.0 + delta, start.1),
    };
    let shifted_end = match carrier_kind {
        AxisAlignedRunKind::Horizontal => (end.0, end.1 + delta),
        AxisAlignedRunKind::Vertical => (end.0 + delta, end.1),
    };
    let (bump_start, bump_end) = if source_side {
        (shifted_start, end)
    } else {
        (start, shifted_end)
    };
    let route_angle = route_angle_for_axis_segment(start, end, carrier_kind)?;
    build_compact_offset_bump_with_side_retry(
        bump_start,
        bump_end,
        route_angle,
        infer_90_bend_radius_um(primitives)?,
        if source_side {
            OffsetBumpPlacement::Start
        } else {
            OffsetBumpPlacement::End
        },
    )
}

fn route_angle_for_axis_segment(
    start: (f64, f64),
    end: (f64, f64),
    kind: AxisAlignedRunKind,
) -> Result<u8, GeometryError> {
    match kind {
        AxisAlignedRunKind::Horizontal => {
            if end.0 > start.0 + EPS {
                Ok(0)
            } else if end.0 < start.0 - EPS {
                Ok(4)
            } else {
                Err(GeometryError::ZeroLengthSegment)
            }
        }
        AxisAlignedRunKind::Vertical => {
            if end.1 > start.1 + EPS {
                Ok(2)
            } else if end.1 < start.1 - EPS {
                Ok(6)
            } else {
                Err(GeometryError::ZeroLengthSegment)
            }
        }
    }
}

fn is_full_horizontal_centerline(centerline: &[(f64, f64)]) -> bool {
    centerline
        .windows(2)
        .all(|w| is_horizontal_segment(w[0], w[1]))
}

fn is_full_vertical_centerline(centerline: &[(f64, f64)]) -> bool {
    centerline
        .windows(2)
        .all(|w| is_vertical_segment(w[0], w[1]))
}

fn is_full_diagonal_centerline(centerline: &[(f64, f64)]) -> bool {
    if centerline.len() < 2 {
        return false;
    }
    let Some(angle) = diagonal_angle_for_segment(centerline[0], centerline[1]) else {
        return false;
    };
    centerline
        .windows(2)
        .all(|w| diagonal_angle_for_segment(w[0], w[1]) == Some(angle))
}

fn validate_full_straight_centerline(
    centerline: &[(f64, f64)],
    kind: AxisAlignedRunKind,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    let first_delta = axis_delta(centerline[0], centerline[1], kind);
    if first_delta.abs() <= EPS {
        return Err(GeometryError::ZeroLengthSegment);
    }
    for window in centerline.windows(2) {
        let delta = axis_delta(window[0], window[1], kind);
        if delta.abs() <= EPS {
            return Err(GeometryError::ZeroLengthSegment);
        }
        if first_delta * delta <= 0.0 {
            return Err(GeometryError::DegenerateRoute);
        }
        match kind {
            AxisAlignedRunKind::Horizontal => {
                if (window[1].1 - window[0].1).abs() > EPS {
                    return Err(GeometryError::DegenerateRoute);
                }
            }
            AxisAlignedRunKind::Vertical => {
                if (window[1].0 - window[0].0).abs() > EPS {
                    return Err(GeometryError::DegenerateRoute);
                }
            }
        }
    }
    Ok(())
}

fn validate_full_diagonal_centerline(
    centerline: &[(f64, f64)],
    angle: u8,
) -> Result<(), GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    for window in centerline.windows(2) {
        if diagonal_angle_for_segment(window[0], window[1]) != Some(angle) {
            return Err(GeometryError::DegenerateRoute);
        }
        if distance(window[0], window[1]) <= EPS {
            return Err(GeometryError::ZeroLengthSegment);
        }
    }
    Ok(())
}

fn axis_delta(a: (f64, f64), b: (f64, f64), kind: AxisAlignedRunKind) -> f64 {
    match kind {
        AxisAlignedRunKind::Horizontal => b.0 - a.0,
        AxisAlignedRunKind::Vertical => b.1 - a.1,
    }
}

fn diagonal_angle_for_segment(a: (f64, f64), b: (f64, f64)) -> Option<u8> {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    if dx.abs() <= EPS || dy.abs() <= EPS || (dx.abs() - dy.abs()).abs() > 1.0e-6 {
        return None;
    }
    match (dx > 0.0, dy > 0.0) {
        (true, true) => Some(1),
        (false, true) => Some(3),
        (false, false) => Some(5),
        (true, false) => Some(7),
    }
}

fn angle_for_centerline_segment(a: (f64, f64), b: (f64, f64)) -> Option<u8> {
    if is_horizontal_segment(a, b) {
        if b.0 > a.0 {
            Some(0)
        } else {
            Some(4)
        }
    } else if is_vertical_segment(a, b) {
        if b.1 > a.1 {
            Some(2)
        } else {
            Some(6)
        }
    } else {
        diagonal_angle_for_segment(a, b)
    }
}

fn is_horizontal_segment(a: (f64, f64), b: (f64, f64)) -> bool {
    (b.1 - a.1).abs() <= EPS && (b.0 - a.0).abs() > EPS
}

fn is_vertical_segment(a: (f64, f64), b: (f64, f64)) -> bool {
    (b.0 - a.0).abs() <= EPS && (b.1 - a.1).abs() > EPS
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

/// Realize route polygon after anchoring endpoints to physical ports.
pub fn realize_route_polygon_with_endpoint_correction(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    source_port_um: Option<(f64, f64)>,
    target_port_um: Option<(f64, f64)>,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let mut centerline = route_to_port_corrected_centerline(
        route,
        primitives,
        grid,
        source_port_um,
        target_port_um,
    )?;
    insert_terminal_tangent_stubs(
        &mut centerline,
        route,
        terminal_tangent_stub_len_um(width_um)?,
        source_port_um.is_some(),
        target_port_um.is_some(),
    )?;
    generate_waveguide_polygon(&centerline, width_um)
}

pub(crate) fn realize_centerline_polygon_with_terminal_tangents(
    centerline: &[(f64, f64)],
    route: &RouteResult,
    width_um: f64,
    source_enabled: bool,
    target_enabled: bool,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let mut adjusted = centerline.to_vec();
    insert_terminal_tangent_stubs(
        &mut adjusted,
        route,
        terminal_tangent_stub_len_um(width_um)?,
        source_enabled,
        target_enabled,
    )?;
    generate_waveguide_polygon(&adjusted, width_um)
}

pub fn centerline_length_um(centerline: &[(f64, f64)]) -> Result<f64, GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if centerline.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    let mut total = 0.0;
    for window in centerline.windows(2) {
        let segment_length = distance(window[0], window[1]);
        if segment_length <= EPS {
            return Err(GeometryError::ZeroLengthSegment);
        }
        total += segment_length;
    }
    Ok(total)
}

#[derive(Clone, Debug, PartialEq)]
pub struct RouteAnalyticMeanderPlan {
    pub selected_segment_index: usize,
    pub selected_segment: StraightSegment,
    pub plan: AnalyticMeanderPlan,
}

pub(crate) fn select_meander_segment(
    centerline: &[(f64, f64)],
    available_box: MeanderBox,
) -> Result<usize, GeometryError> {
    if centerline.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }

    let mut best: Option<(usize, f64)> = None;
    for (i, w) in centerline.windows(2).enumerate() {
        let p0 = w[0];
        let p1 = w[1];
        let dx = (p1.0 - p0.0).abs();
        let dy = (p1.1 - p0.1).abs();
        let is_horizontal = dy <= EPS && dx > EPS;
        let is_vertical = dx <= EPS && dy > EPS;
        if !(is_horizontal || is_vertical) {
            continue;
        }
        let in0 = p0.0 >= available_box.min_x_um - EPS
            && p0.0 <= available_box.max_x_um + EPS
            && p0.1 >= available_box.min_y_um - EPS
            && p0.1 <= available_box.max_y_um + EPS;
        let in1 = p1.0 >= available_box.min_x_um - EPS
            && p1.0 <= available_box.max_x_um + EPS
            && p1.1 >= available_box.min_y_um - EPS
            && p1.1 <= available_box.max_y_um + EPS;
        if !(in0 && in1) {
            continue;
        }
        let len = distance(p0, p1);
        match best {
            Some((_, cur)) if cur >= len => {}
            _ => best = Some((i, len)),
        }
    }

    best.map(|(idx, _)| idx)
        .ok_or(GeometryError::NoMeanderCandidateSegment)
}

pub fn plan_analytic_meander_for_route(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    requested_extra_length_um: f64,
    min_bend_radius_um: f64,
    min_straight_um: f64,
    max_bumps: usize,
    meander_side: MeanderSide,
    available_box: MeanderBox,
    mode: MeanderPlanningMode,
) -> Result<RouteAnalyticMeanderPlan, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let selected_segment_index = select_meander_segment(&centerline, available_box)?;
    let selected_segment = StraightSegment {
        start: PhysicalPoint {
            x_um: centerline[selected_segment_index].0,
            y_um: centerline[selected_segment_index].1,
        },
        end: PhysicalPoint {
            x_um: centerline[selected_segment_index + 1].0,
            y_um: centerline[selected_segment_index + 1].1,
        },
    };
    let cfg = AnalyticMeanderConfig {
        requested_extra_length_um,
        min_bend_radius_um,
        min_straight_um,
        max_bumps,
        max_meander_height_um: 1.0e12,
        side: meander_side,
        mode,
    };
    let plan = plan_analytic_meander(selected_segment, available_box, &cfg)
        .map_err(GeometryError::MeanderPlanningFailed)?;

    Ok(RouteAnalyticMeanderPlan {
        selected_segment_index,
        selected_segment,
        plan,
    })
}

pub fn plan_auto_analytic_meander_for_route(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
) -> Result<AutoRouteAnalyticMeanderPlan, GeometryError> {
    plan_auto_analytic_meander_for_route_depth_sweep(
        route,
        primitives,
        grid,
        obstacle_map,
        opened_cells,
        config,
        &[config.box_depth_um],
    )
}

pub fn plan_auto_analytic_meander_for_route_depth_sweep(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
    box_depths_um: &[f64],
) -> Result<AutoRouteAnalyticMeanderPlan, GeometryError> {
    let total_start = Instant::now();
    if !config.requested_extra_length_um.is_finite()
        || config.requested_extra_length_um <= 0.0
        || !config.min_bend_radius_um.is_finite()
        || config.min_bend_radius_um <= 0.0
        || !config.min_straight_um.is_finite()
        || config.min_straight_um < 0.0
        || config.max_bumps == 0
        || !config.max_meander_height_um.is_finite()
        || config.max_meander_height_um <= 0.0
        || !config.box_depth_um.is_finite()
        || config.box_depth_um <= 0.0
        || !config.min_segment_length_um.is_finite()
        || config.min_segment_length_um <= 0.0
        || !config.endpoint_inset_um.is_finite()
        || config.endpoint_inset_um < 0.0
        || config.clearance_radius_cells < 0
        || box_depths_um.is_empty()
    {
        return Err(GeometryError::InvalidMeanderBox);
    }
    for depth_um in box_depths_um {
        if !depth_um.is_finite() || *depth_um <= 0.0 {
            return Err(GeometryError::InvalidMeanderBox);
        }
    }
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let prefix = DenseOccupancyPrefix::from_obstacle_map(obstacle_map, opened_cells);
    let mut plan = plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
        &centerline,
        grid,
        &prefix,
        opened_cells,
        None,
        None,
        None,
        None,
        config,
        box_depths_um,
    )?;
    plan.profile.total_s = total_start.elapsed().as_secs_f64();
    Ok(plan)
}

pub fn plan_auto_analytic_meander_for_route_depth_sweep_with_prefix(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    base_prefix: &DenseOccupancyPrefix,
    opened_cells: Option<&FxHashSet<CellKey>>,
    opened_index: Option<&SparseCellIndex>,
    extra_blocked_cells: Option<&FxHashSet<CellKey>>,
    extra_blocked_index: Option<&SparseCellIndex>,
    extra_blocked_overlay_index: Option<&SparseCellIndex>,
    config: &AutoMeanderConfig,
    box_depths_um: &[f64],
) -> Result<AutoRouteAnalyticMeanderPlan, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    plan_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
        &centerline,
        grid,
        base_prefix,
        opened_cells,
        opened_index,
        extra_blocked_cells,
        extra_blocked_index,
        extra_blocked_overlay_index,
        config,
        box_depths_um,
    )
}

pub fn probe_auto_analytic_meander_for_route_depth_sweep_with_prefix(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    base_prefix: &DenseOccupancyPrefix,
    opened_cells: Option<&FxHashSet<CellKey>>,
    extra_blocked_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
    box_depths_um: &[f64],
) -> Result<AutoRouteAnalyticMeanderProbe, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix(
        &centerline,
        grid,
        base_prefix,
        opened_cells,
        extra_blocked_cells,
        config,
        box_depths_um,
    )
}

pub(crate) fn splice_meander_into_centerline(
    centerline: &[(f64, f64)],
    segment_index: usize,
    meander: &[PhysicalPoint],
) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(centerline.len() + meander.len());
    for p in centerline.iter().take(segment_index + 1).copied() {
        push_physical_if_different(&mut out, p);
    }
    for p in meander.iter().copied() {
        push_physical_if_different(&mut out, (p.x_um, p.y_um));
    }
    for p in centerline.iter().skip(segment_index + 1).copied() {
        push_physical_if_different(&mut out, p);
    }
    out
}

pub(crate) fn splice_meander_into_centerline_range(
    centerline: &[(f64, f64)],
    start_index: usize,
    end_index: usize,
    meander: &[PhysicalPoint],
) -> Result<Vec<(f64, f64)>, GeometryError> {
    if centerline.len() < 2 || start_index >= end_index || end_index >= centerline.len() {
        return Err(GeometryError::NoMeanderCandidateSegment);
    }
    let mut out = Vec::with_capacity(centerline.len() + meander.len());
    for p in centerline.iter().take(start_index).copied() {
        push_physical_if_different(&mut out, p);
    }
    for p in meander.iter().copied() {
        push_physical_if_different(&mut out, (p.x_um, p.y_um));
    }
    for p in centerline.iter().skip(end_index + 1).copied() {
        push_physical_if_different(&mut out, p);
    }
    Ok(out)
}

pub fn realize_route_polygon_with_analytic_meander(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    requested_extra_length_um: f64,
    min_bend_radius_um: f64,
    min_straight_um: f64,
    max_bumps: usize,
    meander_side: MeanderSide,
    available_box: MeanderBox,
    mode: MeanderPlanningMode,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let route_plan = plan_analytic_meander_for_route(
        route,
        primitives,
        grid,
        requested_extra_length_um,
        min_bend_radius_um,
        min_straight_um,
        max_bumps,
        meander_side,
        available_box,
        mode,
    )?;

    let modified = splice_meander_into_centerline(
        &centerline,
        route_plan.selected_segment_index,
        &route_plan.plan.centerline,
    );
    if modified.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if modified.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if modified.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    generate_waveguide_polygon(&modified, width_um)
}

pub fn realize_route_polygon_with_checked_analytic_meander_box(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    requested_extra_length_um: f64,
    min_bend_radius_um: f64,
    min_straight_um: f64,
    max_bumps: usize,
    meander_side: MeanderSide,
    available_box: MeanderBox,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    clearance_radius_cells: i32,
    mode: MeanderPlanningMode,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let _rect = check_meander_box_free(
        available_box,
        grid,
        obstacle_map,
        opened_cells,
        clearance_radius_cells,
    )?;
    realize_route_polygon_with_analytic_meander(
        route,
        primitives,
        grid,
        width_um,
        requested_extra_length_um,
        min_bend_radius_um,
        min_straight_um,
        max_bumps,
        meander_side,
        available_box,
        mode,
    )
}

pub fn realize_route_polygon_with_auto_checked_analytic_meander(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    obstacle_map: &ObstacleMap,
    opened_cells: Option<&FxHashSet<CellKey>>,
    config: &AutoMeanderConfig,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let auto = plan_auto_analytic_meander_for_route(
        route,
        primitives,
        grid,
        obstacle_map,
        opened_cells,
        config,
    )?;
    let modified = splice_meander_into_centerline_range(
        &centerline,
        auto.selected_run_start_index,
        auto.selected_run_end_index,
        &auto.replacement_centerline,
    )?;
    if modified.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if modified.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if modified.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    generate_waveguide_polygon(&modified, width_um)
}

pub fn realize_route_polygon_from_auto_plan(
    route: &RouteResult,
    primitives: &PrimitiveLibrary,
    grid: &GeometryGridSpec,
    width_um: f64,
    auto_plan: &AutoRouteAnalyticMeanderPlan,
) -> Result<Vec<(f64, f64)>, GeometryError> {
    let centerline = route_to_primitive_centerline(route, primitives, grid)?;
    let modified = splice_meander_into_centerline_range(
        &centerline,
        auto_plan.selected_run_start_index,
        auto_plan.selected_run_end_index,
        &auto_plan.replacement_centerline,
    )?;
    if modified.len() < 2 {
        return Err(GeometryError::DegenerateRoute);
    }
    if modified.iter().any(|&p| !is_finite_point(p)) {
        return Err(GeometryError::NonFiniteCoordinate);
    }
    if modified.windows(2).any(|w| distance(w[0], w[1]) <= EPS) {
        return Err(GeometryError::ZeroLengthSegment);
    }
    generate_waveguide_polygon(&modified, width_um)
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
    let arc_span = (a1 - a0).abs();
    let steps = ((arc_span / std::f64::consts::FRAC_PI_2) * arc_samples_per_90_deg(radius) as f64)
        .ceil()
        .max(2.0) as usize;
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

pub(crate) fn push_physical_if_different(points: &mut Vec<(f64, f64)>, point: (f64, f64)) {
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

pub(crate) fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
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

    let trim_eff = trim.min(in_len).min(out_len);
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
    use rustc_hash::FxHashSet;

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

    fn primitive_id_for<F>(lib: &PrimitiveLibrary, start_angle: u8, predicate: F) -> u16
    where
        F: Fn(&crate::primitives::Primitive) -> bool,
    {
        lib.get_primitives_for_angle(start_angle)
            .iter()
            .find(|p| predicate(p))
            .expect("missing primitive for test setup")
            .id
    }

    fn states_from_primitives(
        lib: &PrimitiveLibrary,
        start: State,
        primitive_ids: &[u16],
    ) -> Vec<State> {
        let mut out = Vec::with_capacity(primitive_ids.len() + 1);
        out.push(start);
        let mut cur = start;
        for primitive_id in primitive_ids {
            let primitive = lib
                .get_primitives_for_angle(cur.angle)
                .iter()
                .find(|p| p.id == *primitive_id)
                .expect("primitive id must exist for current angle");
            cur = State::new(
                cur.x + primitive.dx,
                cur.y + primitive.dy,
                primitive.end_angle,
            );
            out.push(cur);
        }
        out
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
    fn arc_samples_keep_every_chord_within_the_sagitta_tolerance() {
        for radius_um in [0.5, 2.0, 5.0, 10.0, 25.0, 100.0] {
            let samples = arc_samples_per_90_deg(radius_um);
            assert!(samples >= DEFAULT_BEND_SAMPLES_PER_90_DEG);
            let step = std::f64::consts::FRAC_PI_2 / samples as f64;
            let sagitta = radius_um * (1.0 - (step / 2.0).cos());
            assert!(
                sagitta <= MAX_ARC_SAGITTA_UM + 1.0e-12,
                "radius {radius_um}: sagitta {sagitta} with {samples} samples"
            );
            // Not absurdly dense either: one step short would already violate
            // the tolerance (or we are at the floor).
            if samples > DEFAULT_BEND_SAMPLES_PER_90_DEG {
                let coarser = std::f64::consts::FRAC_PI_2 / (samples - 1) as f64;
                assert!(radius_um * (1.0 - (coarser / 2.0).cos()) > MAX_ARC_SAGITTA_UM);
            }
        }
        // A 10 um bend used to be 4 chords in the endpoint-correction jog and
        // 8 in a meander; it is 56 everywhere now (gdsfactory: ~62).
        assert_eq!(arc_samples_per_90_deg(10.0), 56);
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

    #[test]
    fn grid_locked_port_centerline_keeps_primitive_interior_on_grid() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let route = RouteResult {
            states: vec![
                State::new(1, 2, 0),
                State::new(5, 2, 0),
                State::new(9, 2, 0),
            ],
            primitives: vec![straight_east, straight_east],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0,
            total_cost: 8.0,
            requested_target: State::new(9, 2, 0),
            reached_target: State::new(9, 2, 0),
            stats: Default::default(),
        };

        let corrected = route_to_grid_locked_port_centerline(
            &route,
            &lib,
            &grid(),
            Some((0.8, 2.75)),
            Some((9.9, 2.75)),
        )
        .unwrap();

        assert_eq!(
            corrected,
            vec![(0.8, 2.75), (1.5, 2.5), (5.5, 2.5), (9.5, 2.5), (9.9, 2.75)]
        );
    }

    #[test]
    fn port_corrected_single_straight_absorbs_x_offsets_without_new_points() {
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

        let centerline = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((1.2, 2.5)),
            Some((6.0, 2.5)),
        )
        .unwrap();

        assert_eq!(centerline, vec![(1.2, 2.5), (6.0, 2.5)]);
        let polygon = realize_route_polygon_with_endpoint_correction(
            &route,
            &lib,
            &grid(),
            0.5,
            Some((1.2, 2.5)),
            Some((6.0, 2.5)),
        )
        .unwrap();
        assert_eq!(
            polygon,
            generate_waveguide_polygon(&centerline, 0.5).unwrap()
        );
        assert_eq!(polygon.first(), polygon.last());
    }

    #[test]
    fn port_corrected_full_horizontal_straight_shifts_line_and_adjusts_length() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let primitive_ids = vec![straight_east, straight_east];
        let states = states_from_primitives(&lib, State::new(1, 2, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0,
            total_cost: 8.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((1.2, 2.75)),
            Some((9.9, 2.75)),
        )
        .unwrap();

        assert_eq!(corrected, vec![(1.2, 2.75), (5.5, 2.75), (9.9, 2.75)]);
        assert!((centerline_length_um(&corrected).unwrap() - 8.7).abs() <= 1.0e-9);
        assert!(corrected
            .windows(2)
            .all(|w| is_horizontal_segment(w[0], w[1])));
    }

    #[test]
    fn port_corrected_near_horizontal_straight_inserts_four_bend_offset_bump() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let primitive_ids = vec![straight_east, straight_east, straight_east];
        let states = states_from_primitives(&lib, State::new(1, 2, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((1.0, 2.5)),
            Some((13.5, 2.0)),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some((1.0, 2.5)));
        assert_eq!(corrected.last().copied(), Some((13.5, 2.0)));
        assert!(corrected.len() > 8);
        validate_source_tangent(&corrected, angle_to_unit_vector(0)).unwrap();
        validate_target_tangent(&corrected, angle_to_unit_vector(0)).unwrap();
        assert!(corrected
            .iter()
            .any(|point| point.1 < 2.0 - EPS || point.1 > 2.5 + EPS));
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn port_corrected_safe_mode_rejects_unchecked_offset_bump() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let primitive_ids = vec![straight_east, straight_east, straight_east];
        let states = states_from_primitives(&lib, State::new(1, 2, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let err = route_to_port_corrected_centerline_with_options(
            &route,
            &lib,
            &grid(),
            Some((1.0, 2.5)),
            Some((13.5, 2.0)),
            false,
        )
        .unwrap_err();

        assert!(matches!(err, GeometryError::NoMeanderCandidateSegment));
    }

    #[test]
    fn port_corrected_near_vertical_straight_inserts_four_bend_offset_bump() {
        let lib = test_lib();
        let straight_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_north, straight_north, straight_north];
        let states = states_from_primitives(&lib, State::new(2, 1, 2), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((2.5, 1.0)),
            Some((3.0, 13.5)),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some((2.5, 1.0)));
        assert_eq!(corrected.last().copied(), Some((3.0, 13.5)));
        assert!(corrected.len() > 8);
        validate_source_tangent(&corrected, angle_to_unit_vector(2)).unwrap();
        validate_target_tangent(&corrected, angle_to_unit_vector(2)).unwrap();
        assert!(corrected
            .iter()
            .any(|point| point.0 < 2.5 - EPS || point.0 > 3.0 + EPS));
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn compact_offset_bump_has_minimal_length() {
        let radius_um = 1.0;
        let forward_len = 20.0;
        let offset_len = 0.5;
        let centerline = build_compact_four_bend_offset_bump(
            (0.0, 0.0),
            (forward_len, offset_len),
            0,
            2,
            radius_um,
            OffsetBumpPlacement::Start,
        )
        .unwrap();

        validate_source_tangent(&centerline, angle_to_unit_vector(0)).unwrap();
        validate_target_tangent(&centerline, angle_to_unit_vector(0)).unwrap();
        assert_eq!(centerline.first().copied(), Some((0.0, 0.0)));
        assert_eq!(centerline.last().copied(), Some((forward_len, offset_len)));

        let samples = arc_samples_per_90_deg(radius_um) as f64;
        let sampled_quarter_arc_len =
            samples * 2.0 * radius_um * (std::f64::consts::PI / (4.0 * samples)).sin();
        let expected_len =
            (forward_len - 4.0 * radius_um) + offset_len + 4.0 * sampled_quarter_arc_len;
        assert!((centerline_length_um(&centerline).unwrap() - expected_len).abs() < 1.0e-6);

        let mirrored = build_compact_four_bend_offset_bump(
            (0.0, 0.0),
            (forward_len, offset_len),
            0,
            6,
            radius_um,
            OffsetBumpPlacement::Start,
        )
        .unwrap();
        assert!((centerline_length_um(&mirrored).unwrap() - expected_len).abs() < 1.0e-3);
    }

    #[test]
    fn port_corrected_full_vertical_straight_shifts_line_and_adjusts_length() {
        let lib = test_lib();
        let straight_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_north, straight_north];
        let states = states_from_primitives(&lib, State::new(3, 1, 2), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0,
            total_cost: 8.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((3.25, 1.1)),
            Some((3.25, 9.8)),
        )
        .unwrap();

        assert_eq!(corrected, vec![(3.25, 1.1), (3.25, 5.5), (3.25, 9.8)]);
        assert!((centerline_length_um(&corrected).unwrap() - 8.7).abs() <= 1.0e-9);
        assert!(corrected
            .windows(2)
            .all(|w| is_vertical_segment(w[0], w[1])));
    }

    #[test]
    fn port_corrected_full_diagonal_straight_shifts_line_and_adjusts_length() {
        let lib = test_lib();
        let straight_ne = primitive_id_for(&lib, 1, |p| {
            p.start_angle == 1 && p.end_angle == 1 && p.dx == 4 && p.dy == 4
        });
        let primitive_ids = vec![straight_ne, straight_ne];
        let states = states_from_primitives(&lib, State::new(1, 1, 1), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0 * std::f64::consts::SQRT_2,
            total_cost: 8.0 * std::f64::consts::SQRT_2,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((1.2, 1.2)),
            Some((10.0, 10.0)),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some((1.2, 1.2)));
        assert_eq!(corrected.last().copied(), Some((10.0, 10.0)));
        validate_source_tangent(&corrected, angle_to_unit_vector(1)).unwrap();
        validate_target_tangent(&corrected, angle_to_unit_vector(1)).unwrap();
        assert!(corrected
            .windows(2)
            .all(|w| diagonal_angle_for_segment(w[0], w[1]) == Some(1)));
    }

    #[test]
    fn port_corrected_45_route_uses_diagonal_then_vertical_residual() {
        let lib = test_lib();
        let straight_ne = primitive_id_for(&lib, 1, |p| {
            p.start_angle == 1 && p.end_angle == 1 && p.dx == 4 && p.dy == 4
        });
        let bend_left = primitive_id_for(&lib, 1, |p| p.start_angle == 1 && p.end_angle == 2);
        let short_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 1
        });
        let primitive_ids = vec![straight_ne, bend_left, short_north];
        let states = states_from_primitives(&lib, State::new(1, 1, 1), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let base = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        let source_port = (base[0].0 + 0.25, base[0].1 + 0.55);
        let target_port = *base.last().unwrap();
        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        validate_source_tangent(&corrected, angle_to_unit_vector(1)).unwrap();
        validate_target_tangent(&corrected, angle_to_unit_vector(2)).unwrap();
        assert_eq!(corrected.len(), base.len());
        assert_eq!(
            diagonal_angle_for_segment(corrected[0], corrected[1]),
            Some(1)
        );
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn port_corrected_45_safe_mode_uses_diagonal_then_existing_vertical_residual() {
        let lib = test_lib();
        let straight_ne = primitive_id_for(&lib, 1, |p| {
            p.start_angle == 1 && p.end_angle == 1 && p.dx == 4 && p.dy == 4
        });
        let bend_left = primitive_id_for(&lib, 1, |p| p.start_angle == 1 && p.end_angle == 2);
        let short_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 1
        });
        let primitive_ids = vec![straight_ne, bend_left, short_north];
        let states = states_from_primitives(&lib, State::new(1, 1, 1), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let base = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        let source_port = (base[0].0 + 0.25, base[0].1 + 0.55);
        let target_port = *base.last().unwrap();
        let corrected = route_to_port_corrected_centerline_with_options(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
            false,
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        validate_source_tangent(&corrected, angle_to_unit_vector(1)).unwrap();
        validate_target_tangent(&corrected, angle_to_unit_vector(2)).unwrap();
        assert_eq!(corrected.len(), base.len());
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn port_corrected_45_degree_delta_correction_retries_a_rejected_candidate() {
        // Regression test for the collision-avoidance fix in
        // .agent/execplans/2026-08-19-collision-avoiding-endpoint-correction.md:
        // try_apply_45_degree_endpoint_delta_correction (via this fixture's
        // (0.25, 0.55) delta, the same fixture and delta the sibling test
        // above uses) has more than one internal way to reach a valid
        // corrected centerline (multiple (source_adjustment, target_adjustment)
        // pairs, each with its own residual-absorption attempt). This test
        // rejects whichever candidate it picks by default via a
        // caller-supplied collision_check and asserts a *different*, still
        // fully valid centerline comes back, proving the new collision_check
        // parameter makes this function retry rather than being limited to
        // "accept the first internally-consistent candidate or fail
        // outright" -- the limitation that previously let the caller-side
        // check in route_port_corrected_centerline_checked_and_commit_native
        // (src/py_router.rs) only ever reject a collision, never route
        // around it.
        let lib = test_lib();
        let straight_ne = primitive_id_for(&lib, 1, |p| {
            p.start_angle == 1 && p.end_angle == 1 && p.dx == 4 && p.dy == 4
        });
        let bend_left = primitive_id_for(&lib, 1, |p| p.start_angle == 1 && p.end_angle == 2);
        let long_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_ne, bend_left, long_north];
        let states = states_from_primitives(&lib, State::new(1, 1, 1), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let base = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        let source_port = (base[0].0 + 0.25, base[0].1 + 0.55);
        let target_port = *base.last().unwrap();

        let unconstrained = route_to_port_corrected_centerline_with_options_and_collision_check(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
            true,
            None,
        )
        .unwrap();

        let forbidden = unconstrained.clone();
        let reject_default_candidate = move |candidate: &[(f64, f64)]| -> bool {
            candidate.len() != forbidden.len()
                || candidate
                    .iter()
                    .zip(forbidden.iter())
                    .any(|(a, b)| distance(*a, *b) > EPS)
        };
        let retried = route_to_port_corrected_centerline_with_options_and_collision_check(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
            true,
            Some(&reject_default_candidate),
        )
        .unwrap();

        assert!(distance(*retried.first().unwrap(), source_port) <= EPS);
        assert!(distance(*retried.last().unwrap(), target_port) <= EPS);
        validate_source_tangent(&retried, angle_to_unit_vector(1)).unwrap();
        validate_target_tangent(&retried, angle_to_unit_vector(2)).unwrap();
        assert!(retried.windows(2).all(|w| distance(w[0], w[1]) > EPS));
        assert_ne!(
            retried, unconstrained,
            "collision_check rejecting the default candidate must produce a different \
             centerline, not silently return the same one or fail outright"
        );
    }

    #[test]
    fn port_corrected_45_route_rejects_unequal_delta_without_residual_run() {
        let lib = test_lib();
        let straight_ne = primitive_id_for(&lib, 1, |p| {
            p.start_angle == 1 && p.end_angle == 1 && p.dx == 4 && p.dy == 4
        });
        let primitive_ids = vec![straight_ne];
        let states = states_from_primitives(&lib, State::new(1, 1, 1), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 4.0 * std::f64::consts::SQRT_2,
            total_cost: 4.0 * std::f64::consts::SQRT_2,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };
        let base = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();

        let err = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some((base[0].0 + 0.25, base[0].1 + 0.55)),
            Some(*base.last().unwrap()),
        )
        .unwrap_err();

        assert!(matches!(
            err,
            GeometryError::NoMeanderCandidateSegment
                | GeometryError::PortEndpointCorrectionRequiresUnsupportedStub
        ));
    }

    #[test]
    fn port_corrected_mixed_xy_route_anchors_endpoints_and_keeps_interior() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let bend_left = primitive_id_for(&lib, 0, |p| p.start_angle == 0 && p.end_angle == 2);
        let straight_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_east, bend_left, straight_north];
        let states = states_from_primitives(&lib, State::new(1, 1, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let base = route_to_primitive_centerline(&route, &lib, &grid()).unwrap();
        assert!(base.len() > 2);
        let source_port = (1.1, 1.25);
        let target_port = (base.last().unwrap().0 + 0.35, base.last().unwrap().1 - 0.2);
        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        assert_eq!(corrected.len(), base.len());
        assert!(is_horizontal_segment(corrected[0], corrected[1]));
        assert!(is_vertical_segment(
            corrected[corrected.len() - 2],
            corrected[corrected.len() - 1],
        ));
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));

        let polygon = realize_route_polygon_with_endpoint_correction(
            &route,
            &lib,
            &grid(),
            0.5,
            Some(source_port),
            Some(target_port),
        )
        .unwrap();
        assert_eq!(
            polygon,
            generate_waveguide_polygon(&corrected, 0.5).unwrap()
        );
        assert_eq!(polygon.first(), polygon.last());
    }

    #[test]
    fn port_corrected_dogleg_inserts_absorber_straight_instead_of_terminal_diagonal() {
        let lib = test_lib();
        let straight_east_0 = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let bend_left = primitive_id_for(&lib, 0, |p| p.start_angle == 0 && p.end_angle == 2);
        let bend_right = primitive_id_for(&lib, 2, |p| p.start_angle == 2 && p.end_angle == 0);
        let straight_east_1 = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let primitive_ids = vec![straight_east_0, bend_left, bend_right, straight_east_1];
        let states = states_from_primitives(&lib, State::new(1, 1, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let source_port = (1.25, 1.0);
        let target_port = (12.75, 3.0);
        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        validate_source_tangent(&corrected, angle_to_unit_vector(0)).unwrap();
        validate_target_tangent(&corrected, angle_to_unit_vector(0)).unwrap();
        assert!(corrected.len() > route.primitives.len() + 1);
        assert!(corrected.windows(2).any(|w| {
            is_vertical_segment(w[0], w[1]) && (distance(w[0], w[1]) - 0.5).abs() <= 1.0e-9
        }));
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn port_correction_absorbs_xy_offsets_only_in_straight_primitives() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            p.start_angle == 0 && p.end_angle == 0 && p.dx == 4 && p.dy == 0
        });
        let bend_left = primitive_id_for(&lib, 0, |p| p.start_angle == 0 && p.end_angle == 2);
        let straight_north = primitive_id_for(&lib, 2, |p| {
            p.start_angle == 2 && p.end_angle == 2 && p.dx == 0 && p.dy == 4
        });
        let primitive_ids = vec![straight_east, bend_left, straight_north];
        let states = states_from_primitives(&lib, State::new(1, 1, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let replay = route_to_primitive_centerline_with_runs(&route, &lib, &grid()).unwrap();
        let base = replay.centerline;
        let horizontal_run = first_run(&replay.straight_runs, AxisAlignedRunKind::Horizontal)
            .expect("expected horizontal straight primitive");
        let vertical_run = first_run(&replay.straight_runs, AxisAlignedRunKind::Vertical)
            .expect("expected vertical straight primitive");
        assert!(horizontal_run.end_index < vertical_run.start_index);

        let source_dx = 0.2;
        let source_dy = 0.25;
        let target_dx = -0.3;
        let target_dy = -0.15;
        let source_port = (base[0].0 + source_dx, base[0].1 + source_dy);
        let target_port = (
            base[base.len() - 1].0 + target_dx,
            base[base.len() - 1].1 + target_dy,
        );

        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
        )
        .unwrap();

        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        validate_source_tangent(&corrected, angle_to_unit_vector(0)).unwrap();
        validate_target_tangent(&corrected, angle_to_unit_vector(2)).unwrap();
        assert!(corrected.len() >= base.len());
        assert!(horizontal_run.end_index < vertical_run.start_index);
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn port_correction_shifts_nonstraight_route_when_ports_share_y() {
        let lib = test_lib();
        let straight_east = primitive_id_for(&lib, 0, |p| {
            matches!(p.geometry, PrimitiveGeometry::Straight { .. }) && p.dx == 4 && p.dy == 0
        });
        let bend_left_from_east = primitive_id_for(&lib, 0, |p| {
            matches!(p.geometry, PrimitiveGeometry::Bend { angle_delta: 2, .. })
        });
        let bend_left_from_north = primitive_id_for(&lib, 2, |p| {
            matches!(p.geometry, PrimitiveGeometry::Bend { angle_delta: 2, .. })
        });
        let bend_left_from_west = primitive_id_for(&lib, 4, |p| {
            matches!(p.geometry, PrimitiveGeometry::Bend { angle_delta: 2, .. })
        });
        let bend_left_from_south = primitive_id_for(&lib, 6, |p| {
            matches!(p.geometry, PrimitiveGeometry::Bend { angle_delta: 2, .. })
        });
        let primitive_ids = vec![
            straight_east,
            bend_left_from_east,
            bend_left_from_north,
            bend_left_from_west,
            bend_left_from_south,
            straight_east,
        ];
        let states = states_from_primitives(&lib, State::new(1, 1, 0), &primitive_ids);
        let requested_target = *states.last().unwrap();
        let route = RouteResult {
            states,
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target,
            reached_target: requested_target,
            stats: Default::default(),
        };

        let replay = route_to_primitive_centerline_with_runs(&route, &lib, &grid()).unwrap();
        let base = replay.centerline;
        assert!(replay
            .straight_runs
            .iter()
            .all(|run| run.kind != AxisAlignedRunKind::Vertical));
        assert!((base[0].1 - base[base.len() - 1].1).abs() <= EPS);

        let shared_dy = 0.25;
        let source_port = (base[0].0 + 0.2, base[0].1 + shared_dy);
        let target_port = (base[base.len() - 1].0 - 0.3, base[0].1 + shared_dy);
        let corrected = route_to_port_corrected_centerline(
            &route,
            &lib,
            &grid(),
            Some(source_port),
            Some(target_port),
        )
        .unwrap();

        assert_eq!(corrected.len(), base.len());
        assert_eq!(corrected.first().copied(), Some(source_port));
        assert_eq!(corrected.last().copied(), Some(target_port));
        for (base_point, corrected_point) in base
            .iter()
            .zip(corrected.iter())
            .skip(1)
            .take(base.len().saturating_sub(2))
        {
            assert!((corrected_point.1 - base_point.1 - shared_dy).abs() <= 1.0e-9);
        }
        assert!(corrected.windows(2).all(|w| distance(w[0], w[1]) > EPS));
    }

    #[test]
    fn terminal_tangent_realization_rejects_unsupported_stub() {
        let route = RouteResult {
            states: vec![State::new(0, 0, 0), State::new(1, 0, 0)],
            primitives: vec![],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: State::new(1, 0, 0),
            reached_target: State::new(1, 0, 0),
            stats: Default::default(),
        };
        let mut realized_centerline = vec![(0.0, 0.0), (0.8, 0.1), (1.0, 0.11)];
        assert!(!is_horizontal_segment(
            realized_centerline[realized_centerline.len() - 2],
            realized_centerline[realized_centerline.len() - 1]
        ));
        let err = insert_terminal_tangent_stubs(
            &mut realized_centerline,
            &route,
            terminal_tangent_stub_len_um(0.5).unwrap(),
            false,
            true,
        )
        .unwrap_err();
        assert!(matches!(
            err,
            GeometryError::PortEndpointCorrectionRequiresUnsupportedStub
        ));
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
    fn splice_meander_preserves_endpoints_and_increases_length() {
        let centerline = vec![(0.0, 0.0), (10.0, 0.0), (15.0, 0.0)];
        let meander = vec![
            PhysicalPoint {
                x_um: 0.0,
                y_um: 0.0,
            },
            PhysicalPoint {
                x_um: 3.0,
                y_um: 2.0,
            },
            PhysicalPoint {
                x_um: 7.0,
                y_um: 2.0,
            },
            PhysicalPoint {
                x_um: 10.0,
                y_um: 0.0,
            },
        ];
        let spliced = splice_meander_into_centerline(&centerline, 0, &meander);
        assert_eq!(spliced.first().copied(), Some((0.0, 0.0)));
        assert_eq!(spliced.last().copied(), Some((15.0, 0.0)));
        assert!(spliced.windows(2).all(|w| distance(w[0], w[1]) > EPS));
        let old_len = centerline
            .windows(2)
            .map(|w| distance(w[0], w[1]))
            .sum::<f64>();
        let new_len = spliced
            .windows(2)
            .map(|w| distance(w[0], w[1]))
            .sum::<f64>();
        assert!(new_len > old_len);
        let b = MeanderBox {
            min_x_um: -1.0,
            max_x_um: 11.0,
            min_y_um: -1.0,
            max_y_um: 3.0,
        };
        for p in meander {
            assert!(p.x_um >= b.min_x_um - EPS && p.x_um <= b.max_x_um + EPS);
            assert!(p.y_um >= b.min_y_um - EPS && p.y_um <= b.max_y_um + EPS);
        }
    }

    #[test]
    fn select_meander_segment_prefers_longest_fully_contained_axis_aligned() {
        let centerline = vec![(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (7.0, 1.0), (9.0, 3.0)];
        let idx = select_meander_segment(
            &centerline,
            MeanderBox {
                min_x_um: 1.0,
                max_x_um: 8.0,
                min_y_um: -1.0,
                max_y_um: 2.0,
            },
        )
        .unwrap();
        assert_eq!(idx, 2);
    }

    #[test]
    fn extract_straight_runs_merges_consecutive_collinear_segments() {
        let centerline = vec![(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0)];
        let runs = extract_axis_aligned_straight_runs(&centerline, 25.0);
        assert_eq!(runs.len(), 1);
        let r = runs[0];
        assert_eq!(r.start_index, 0);
        assert_eq!(r.end_index, 3);
        assert_eq!(r.start, (0.0, 0.0));
        assert_eq!(r.end, (30.0, 0.0));
        assert!((r.length_um - 30.0).abs() < EPS);
    }

    #[test]
    fn extract_straight_runs_splits_on_axis_change() {
        let centerline = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 15.0)];
        let runs = extract_axis_aligned_straight_runs(&centerline, 5.0);
        assert_eq!(runs.len(), 2);
        assert_eq!(runs[0].start_index, 0);
        assert_eq!(runs[0].end_index, 1);
        assert!((runs[0].length_um - 10.0).abs() < EPS);
        assert_eq!(runs[1].start_index, 1);
        assert_eq!(runs[1].end_index, 3);
        assert!((runs[1].length_um - 15.0).abs() < EPS);
    }

    #[test]
    fn plan_analytic_meander_for_route_returns_plan_and_segment() {
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
        let plan = plan_analytic_meander_for_route(
            &route,
            &lib,
            &grid(),
            1.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap();
        assert_eq!(plan.selected_segment_index, 0);
        assert!(plan.plan.bumps >= 1);
        assert!(plan.plan.inserted_extra_length_um > 0.0);
        assert!(!plan.plan.centerline.is_empty());
    }

    #[test]
    fn realizes_route_polygon_with_analytic_meander_success() {
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
        let poly = realize_route_polygon_with_analytic_meander(
            &route,
            &lib,
            &grid(),
            1.0,
            1.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap();
        assert!(poly.len() >= 4);
        assert!(poly.iter().all(|p| p.0.is_finite() && p.1.is_finite()));
    }

    #[test]
    fn analytic_meander_realization_returns_no_candidate_error() {
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
        let err = realize_route_polygon_with_analytic_meander(
            &route,
            &lib,
            &grid(),
            1.0,
            2.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 100.0,
                max_x_um: 110.0,
                min_y_um: 100.0,
                max_y_um: 110.0,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap_err();
        assert_eq!(err, GeometryError::NoMeanderCandidateSegment);
    }

    #[test]
    fn analytic_meander_realization_propagates_planning_failure() {
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
        let err = realize_route_polygon_with_analytic_meander(
            &route,
            &lib,
            &grid(),
            1.0,
            50.0,
            0.5,
            0.5,
            1,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 2.9,
            },
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap_err();
        assert!(matches!(
            err,
            GeometryError::MeanderPlanningFailed(
                MeanderPlanningError::AvailableBoxTooSmall
                    | MeanderPlanningError::RequestedExtraLengthDoesNotFit
            )
        ));
    }

    #[test]
    fn meander_box_converts_to_expected_grid_rect() {
        let g = GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap();
        let rect = meander_box_to_grid_rect(
            MeanderBox {
                min_x_um: 1.0,
                max_x_um: 3.0,
                min_y_um: 2.0,
                max_y_um: 4.0,
            },
            &g,
            0,
        )
        .unwrap();
        assert_eq!(
            rect,
            MeanderGridRect {
                min_x: 1,
                max_x: 2,
                min_y: 2,
                max_y: 3
            }
        );
    }

    #[test]
    fn meander_box_out_of_bounds_is_error() {
        let map = ObstacleMap::new(5, 5);
        let g = GeometryGridSpec::new(1.0, 0.0, 0.0).unwrap();
        let err = check_meander_box_free(
            MeanderBox {
                min_x_um: -1.0,
                max_x_um: 1.0,
                min_y_um: 0.0,
                max_y_um: 1.0,
            },
            &g,
            &map,
            None,
            0,
        )
        .unwrap_err();
        assert!(matches!(err, GeometryError::MeanderBoxOutOfBounds(_)));
    }

    #[test]
    fn checked_analytic_meander_box_realization_succeeds_when_free() {
        let lib = test_lib();
        let map = ObstacleMap::new(20, 20);
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
        let empty = FxHashSet::default();
        let poly = realize_route_polygon_with_checked_analytic_meander_box(
            &route,
            &lib,
            &grid(),
            1.0,
            1.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            &map,
            Some(&empty),
            0,
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap();
        assert!(poly.len() >= 4);
    }

    #[test]
    fn checked_analytic_meander_box_realization_fails_when_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(20, 20);
        assert!(map.add_static_cell(3, 3));
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
        let err = realize_route_polygon_with_checked_analytic_meander_box(
            &route,
            &lib,
            &grid(),
            1.0,
            3.0,
            0.2,
            0.1,
            2,
            MeanderSide::Left,
            MeanderBox {
                min_x_um: 1.4,
                max_x_um: 5.6,
                min_y_um: 2.4,
                max_y_um: 4.0,
            },
            &map,
            None,
            0,
            MeanderPlanningMode::FillBoxMultiBump,
        )
        .unwrap_err();
        assert!(matches!(err, GeometryError::MeanderBoxBlocked { .. }));
    }

    #[test]
    fn cells_in_grid_rect_count_matches_area() {
        let rect = MeanderGridRect {
            min_x: 2,
            max_x: 4,
            min_y: 1,
            max_y: 3,
        };
        assert_eq!(cells_in_grid_rect(rect).len(), 9);
    }

    #[test]
    fn auto_meander_planner_selects_valid_box_on_empty_map() {
        let lib = test_lib();
        let map = ObstacleMap::new(20, 20);
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
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert!(!p.plan.centerline.is_empty());
    }

    #[test]
    fn auto_meander_chooses_right_when_left_box_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(20, 20);
        for x in 1..=5 {
            assert!(map.add_static_cell(x, 3));
        }
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
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.plan.side, MeanderSide::Right);
    }

    #[test]
    fn auto_meander_uses_middle_interval_when_endpoint_boxes_are_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(30, 20);
        for x in [1, 2, 3, 15, 16, 17] {
            assert!(map.add_static_cell(x, 3));
        }
        for x in 1..=17 {
            assert!(map.add_static_cell(x, 1));
        }
        let route = RouteResult {
            states: vec![
                State::new(1, 2, 0),
                State::new(5, 2, 0),
                State::new(9, 2, 0),
                State::new(13, 2, 0),
                State::new(17, 2, 0),
            ],
            primitives: vec![1, 1, 1, 1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 16.0,
            total_cost: 16.0,
            requested_target: State::new(17, 2, 0),
            reached_target: State::new(17, 2, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 8,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };

        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.plan.side, MeanderSide::Left);
        assert!(p.selected_segment.start.x_um > 3.0);
        assert!(p.selected_segment.end.x_um <= 15.0);
        assert!(p.selected_interval_length_um < p.selected_run_length_um);
        assert!(p.candidate_intervals >= 1);
        assert_eq!(
            p.replacement_centerline.first().copied(),
            Some(PhysicalPoint {
                x_um: 1.5,
                y_um: 2.5
            })
        );
        assert_eq!(
            p.replacement_centerline.last().copied(),
            Some(PhysicalPoint {
                x_um: 17.5,
                y_um: 2.5
            })
        );
        assert_eq!(
            p.plan.centerline.first().copied(),
            Some(p.selected_segment.start)
        );
    }

    #[test]
    fn auto_meander_tries_later_runs_when_first_run_is_blocked_on_both_sides() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(40, 40);

        // Build a route with multiple straight runs:
        // long east run -> bend -> north run -> bend -> east run.
        let east_long = primitive_id_for(&lib, 0, |p| {
            matches!(p.geometry, PrimitiveGeometry::Straight { .. }) && p.dx == 4 && p.dy == 0
        });
        let bend_left_from_east = primitive_id_for(&lib, 0, |p| {
            matches!(p.geometry, PrimitiveGeometry::Bend { angle_delta: 2, .. })
        });
        let north_long = primitive_id_for(&lib, 2, |p| {
            matches!(p.geometry, PrimitiveGeometry::Straight { .. }) && p.dx == 0 && p.dy == 4
        });
        let bend_right_from_north = primitive_id_for(&lib, 2, |p| {
            matches!(
                p.geometry,
                PrimitiveGeometry::Bend {
                    angle_delta: -2,
                    ..
                }
            )
        });

        let primitive_ids = vec![
            east_long,
            east_long,
            bend_left_from_east,
            north_long,
            bend_right_from_north,
            east_long,
        ];
        let states = states_from_primitives(&lib, State::new(2, 10, 0), &primitive_ids);
        let route = RouteResult {
            states: states.clone(),
            primitives: primitive_ids,
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 0.0,
            total_cost: 0.0,
            requested_target: *states.last().unwrap(),
            reached_target: *states.last().unwrap(),
            stats: Default::default(),
        };

        // Block both sides of the first long eastbound run (y=10).
        for x in 2..=10 {
            assert!(map.add_static_cell(x, 11));
            assert!(map.add_static_cell(x, 9));
        }

        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 8,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };

        let plan =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert!(plan.selected_run_start_index > 0);
    }

    #[test]
    fn auto_meander_returns_error_when_both_sides_blocked() {
        let lib = test_lib();
        let mut map = ObstacleMap::new(20, 20);
        for x in 1..=5 {
            assert!(map.add_static_cell(x, 3));
            assert!(map.add_static_cell(x, 1));
        }
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
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let err = plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg)
            .unwrap_err();
        assert!(matches!(
            err,
            GeometryError::NoAutoMeanderCandidate {
                candidate_runs: 1,
                rejected_box_blocked: 2,
                ..
            }
        ));
    }

    #[test]
    fn auto_meander_is_deterministic_for_tie() {
        let lib = test_lib();
        let map = ObstacleMap::new(30, 30);
        let route = RouteResult {
            states: vec![
                State::new(2, 10, 0),
                State::new(6, 10, 0),
                State::new(10, 10, 0),
            ],
            primitives: vec![1, 1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 8.0,
            total_cost: 8.0,
            requested_target: State::new(10, 10, 0),
            reached_target: State::new(10, 10, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 2,
            max_meander_height_um: 20.0,
            box_depth_um: 1.6,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.selected_segment_index, 0);
        assert_eq!(p.selected_run_start_index, 0);
        assert_eq!(p.selected_run_end_index, 2);
        assert_eq!(p.plan.side, MeanderSide::Left);
    }

    #[test]
    fn splice_meander_range_replaces_full_run_and_preserves_endpoints() {
        let centerline = vec![
            (0.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
            (15.0, 0.0),
            (20.0, 0.0),
        ];
        let meander = vec![
            PhysicalPoint {
                x_um: 5.0,
                y_um: 0.0,
            },
            PhysicalPoint {
                x_um: 7.0,
                y_um: 2.0,
            },
            PhysicalPoint {
                x_um: 10.0,
                y_um: 0.0,
            },
            PhysicalPoint {
                x_um: 13.0,
                y_um: -2.0,
            },
            PhysicalPoint {
                x_um: 15.0,
                y_um: 0.0,
            },
        ];
        let spliced = splice_meander_into_centerline_range(&centerline, 1, 3, &meander)
            .expect("valid splice");
        assert_eq!(spliced.first().copied(), Some((0.0, 0.0)));
        assert_eq!(spliced.last().copied(), Some((20.0, 0.0)));
        assert!(!spliced.windows(2).any(|w| distance(w[0], w[1]) <= EPS));
        assert!(spliced.contains(&(7.0, 2.0)));
        assert!(spliced.contains(&(13.0, -2.0)));
    }

    #[test]
    fn auto_meander_fill_mode_prefers_more_bumps() {
        let lib = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 12,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
        });
        let map = ObstacleMap::new(40, 40);
        let route = RouteResult {
            states: vec![State::new(2, 10, 0), State::new(14, 10, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: State::new(14, 10, 0),
            reached_target: State::new(14, 10, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.1,
            max_bumps: 6,
            max_meander_height_um: 20.0,
            box_depth_um: 8.0,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert!(p.plan.bumps >= 1);
    }

    #[test]
    fn auto_meander_finds_candidate_on_merged_multi_segment_straight_run() {
        let lib = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: 1.0,
            straight_short_cells: 1,
            straight_long_cells: 4,
            bend_radius_cells: 1,
            allow_45_degree_turns: true,
        });
        let map = ObstacleMap::new(40, 40);
        let route = RouteResult {
            states: vec![
                State::new(2, 10, 0),
                State::new(6, 10, 0),
                State::new(10, 10, 0),
                State::new(14, 10, 0),
            ],
            primitives: vec![1, 1, 1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 12.0,
            total_cost: 12.0,
            requested_target: State::new(14, 10, 0),
            reached_target: State::new(14, 10, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 1.0,
            min_bend_radius_um: 0.2,
            min_straight_um: 0.0,
            max_bumps: 6,
            max_meander_height_um: 20.0,
            box_depth_um: 8.0,
            min_segment_length_um: 10.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let p =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid(), &map, None, &cfg).unwrap();
        assert_eq!(p.selected_run_start_index, 0);
        assert_eq!(p.selected_run_end_index, 3);
    }

    #[test]
    fn fill_box_plan_stays_inside_box_and_uses_primitive_radius() {
        let grid = GeometryGridSpec::new(0.5, 0.0, 0.0).unwrap();
        let primitive_bend_radius_cells = 2;
        let primitive_bend_radius_um = primitive_bend_radius_cells as f64 * grid.grid_size_um;
        let lib = create_photonic_primitive_library(PrimitiveLibraryConfig {
            grid_size_um: grid.grid_size_um,
            straight_short_cells: 1,
            straight_long_cells: 60,
            bend_radius_cells: primitive_bend_radius_cells,
            allow_45_degree_turns: true,
        });
        let map = ObstacleMap::new(120, 120);
        let route = RouteResult {
            states: vec![State::new(2, 20, 0), State::new(62, 20, 0)],
            primitives: vec![1],
            cells: vec![],
            compressed_waypoints: vec![],
            total_length_um: 30.0,
            total_cost: 30.0,
            requested_target: State::new(62, 20, 0),
            reached_target: State::new(62, 20, 0),
            stats: Default::default(),
        };
        let cfg = AutoMeanderConfig {
            requested_extra_length_um: 5.0,
            min_bend_radius_um: primitive_bend_radius_um,
            min_straight_um: 0.0,
            max_bumps: 20,
            max_meander_height_um: 20.0,
            box_depth_um: 8.0,
            min_segment_length_um: 1.0,
            endpoint_inset_um: 0.0,
            clearance_radius_cells: 0,
            side_policy: AutoMeanderSidePolicy::Both,
            mode: MeanderPlanningMode::FillBoxMultiBump,
        };
        let plan =
            plan_auto_analytic_meander_for_route(&route, &lib, &grid, &map, None, &cfg).unwrap();
        assert!(plan.plan.bumps >= 1);
        assert!((cfg.min_bend_radius_um - primitive_bend_radius_um).abs() < EPS);
        for p in &plan.plan.centerline {
            assert!(p.x_um >= plan.selected_box.min_x_um - EPS);
            assert!(p.x_um <= plan.selected_box.max_x_um + EPS);
            assert!(p.y_um >= plan.selected_box.min_y_um - EPS);
            assert!(p.y_um <= plan.selected_box.max_y_um + EPS);
        }
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
