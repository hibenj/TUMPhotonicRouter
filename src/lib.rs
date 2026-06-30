//! Rust foundations for a photonic grid router.
//!
//! This crate currently provides only the routing database and obstacle-map
//! infrastructure. It intentionally does not implement A* yet.

pub mod astar;
pub mod crossings;
pub mod geometry_realization;
pub mod meander;
pub mod obstacle_map;
pub mod plm;
pub mod primitives;
pub mod py_router;
pub mod simple_routes;
pub mod static_obstacle_builder;

pub use crate::simple_routes::{
    check_simple_candidate, compact_offset_order, direction_between,
    expand_candidate_to_grid_points, grid_point_from_state, heading_delta_is_perpendicular,
    is_cardinal_heading, opposite_heading, state_from_grid_point, try_l_candidate,
    try_straight_candidate, try_straight_l_or_z_candidate,
    try_straight_l_or_z_candidate_with_config, try_straight_or_l_candidate, try_z_candidate,
    try_z_candidate_with_config, GridPoint, Segment, SimpleRouteCandidate, SimpleRouteKind,
    SimpleZRouteConfig,
};
pub use astar::{
    export_route_svg, export_route_svg_with_port_open_cells, route_single_net,
    route_single_net_with_config, AStarConfig, RouteResult, State,
};
pub use crossings::{CrossingConfig, CrossingConstraint, CrossingContext, CrossingPair};
pub use geometry_realization::{
    build_port_access, build_port_accesses, cells_in_grid_rect, check_meander_box_free,
    check_meander_box_free_with_prefix, compress_grid_waypoints, compress_route_waypoints,
    generate_waveguide_polygon, grid_path_to_centerline, meander_box_to_grid_rect,
    plan_analytic_meander_for_route, plan_auto_analytic_meander_for_route,
    plan_auto_analytic_meander_for_route_depth_sweep,
    probe_auto_analytic_meander_for_centerline_depth_sweep_with_prefix,
    probe_auto_analytic_meander_for_route_depth_sweep_with_prefix, realize_route_polygon,
    realize_route_polygon_from_auto_plan, realize_route_polygon_from_primitives,
    realize_route_polygon_with_analytic_meander,
    realize_route_polygon_with_auto_checked_analytic_meander,
    realize_route_polygon_with_checked_analytic_meander_box,
    realize_route_polygon_with_port_access, route_to_grid_path, route_to_primitive_centerline,
    snap_centerline_endpoints, AutoMeanderConfig, AutoMeanderSidePolicy,
    AutoRouteAnalyticMeanderPlan, AutoRouteAnalyticMeanderProbe, DenseOccupancyPrefix,
    GeometryError, GeometryGridSpec, GridRect, PortAccess, PortAccessConfig, PortAccessError,
    RouteAnalyticMeanderPlan,
};
pub use meander::{
    actual_bend_radius_um_from_cells, bend_radius_cells_from_min_radius, plan_analytic_meander,
    plan_fill_box_multi_bump_meander, AnalyticMeanderConfig, AnalyticMeanderPlan, MeanderBox,
    MeanderPlanningError, MeanderPlanningMode, MeanderSide, PhysicalPoint, StraightSegment,
};
pub use obstacle_map::{
    pack_xy, unpack_xy, CellKey, ClearanceMetric, NetId, ObstacleMap, WaveguideFootprint,
};
pub use primitives::{
    create_photonic_primitive_library, Primitive, PrimitiveGeometry, PrimitiveLibrary,
    PrimitiveLibraryConfig, DIRECTIONS,
};
pub use static_obstacle_builder::{
    build_static_obstacle_map_from_geometry, compute_bbox, expand_bbox, grid_cell_center,
    make_grid_spec, physical_to_grid, rasterize_polygon, BBox, GridCell, Point, Polygon, PortInput,
    PyStaticCellSet, StaticGridSpec, StaticObstacleBuildConfig, StaticObstacleBuildMode,
    StaticObstacleBuildResult, StaticObstacleBuildStats,
};

pub use static_obstacle_builder::_rust;
