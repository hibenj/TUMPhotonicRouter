//! Rust foundations for a photonic grid router.
//!
//! This crate currently provides only the routing database and obstacle-map
//! infrastructure. It intentionally does not implement A* yet.

pub mod astar;
pub mod geometry_realization;
pub mod meander;
pub mod obstacle_map;
pub mod primitives;
pub mod py_router;
pub mod static_obstacle_builder;

pub use astar::{
    export_route_svg, route_single_net, route_single_net_with_config, AStarConfig, RouteResult,
    State,
};
pub use geometry_realization::{
    AutoMeanderConfig, AutoMeanderSidePolicy, AutoRouteAnalyticMeanderPlan,
    build_port_access, build_port_accesses, cells_in_grid_rect, check_meander_box_free,
    check_meander_box_free_with_prefix, compress_grid_waypoints, compress_route_waypoints,
    generate_waveguide_polygon, grid_path_to_centerline, meander_box_to_grid_rect,
    plan_analytic_meander_for_route, plan_auto_analytic_meander_for_route, realize_route_polygon,
    realize_route_polygon_from_primitives, realize_route_polygon_with_analytic_meander,
    realize_route_polygon_with_auto_checked_analytic_meander,
    realize_route_polygon_from_auto_plan,
    realize_route_polygon_with_checked_analytic_meander_box, realize_route_polygon_with_port_access,
    route_to_grid_path, route_to_primitive_centerline, snap_centerline_endpoints,
    DenseOccupancyPrefix, GeometryError, GeometryGridSpec, GridRect, PortAccess, PortAccessConfig,
    PortAccessError, RouteAnalyticMeanderPlan,
};
pub use meander::{
    plan_analytic_meander, AnalyticMeanderConfig, AnalyticMeanderPlan, MeanderBox,
    MeanderPlanningError, MeanderSide, PhysicalPoint, StraightSegment,
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
    StaticGridSpec, StaticObstacleBuildConfig, StaticObstacleBuildResult, StaticObstacleBuildStats,
};

pub use static_obstacle_builder::_rust;
