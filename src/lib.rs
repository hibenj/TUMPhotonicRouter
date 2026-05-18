//! Rust foundations for a photonic grid router.
//!
//! This crate currently provides only the routing database and obstacle-map
//! infrastructure. It intentionally does not implement A* yet.

pub mod astar;
pub mod geometry_realization;
pub mod obstacle_map;
pub mod primitives;
pub mod py_router;
pub mod static_obstacle_builder;

pub use astar::{
    export_route_svg, route_single_net, route_single_net_with_config, AStarConfig, RouteResult,
    State,
};
pub use geometry_realization::{
    build_port_access, build_port_accesses, compress_grid_waypoints, compress_route_waypoints,
    generate_waveguide_polygon, grid_path_to_centerline, realize_route_polygon,
    realize_route_polygon_from_primitives, realize_route_polygon_with_port_access,
    route_to_grid_path, route_to_primitive_centerline, snap_centerline_endpoints, GeometryError,
    GeometryGridSpec, PortAccess, PortAccessConfig, PortAccessError,
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
