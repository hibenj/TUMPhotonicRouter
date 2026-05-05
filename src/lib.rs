//! Rust foundations for a photonic grid router.
//!
//! This crate currently provides only the routing database and obstacle-map
//! infrastructure. It intentionally does not implement A* yet.

pub mod astar;
pub mod obstacle_map;
pub mod py_router;
pub mod primitives;
pub mod static_obstacle_builder;

pub use astar::{
    export_route_svg, route_single_net, route_single_net_with_config, AStarConfig, RouteResult,
    State,
};
pub use obstacle_map::{
    pack_xy, unpack_xy, CellKey, ClearanceMetric, NetId, ObstacleMap, WaveguideFootprint,
};
pub use primitives::{
    create_photonic_primitive_library, Primitive, PrimitiveLibrary, PrimitiveLibraryConfig,
    DIRECTIONS,
};
pub use static_obstacle_builder::{
    build_static_obstacle_map_from_geometry, compute_bbox, expand_bbox, grid_cell_center,
    make_grid_spec, physical_to_grid, rasterize_polygon, BBox, GridCell, Point, Polygon, PortInput,
    StaticGridSpec, StaticObstacleBuildConfig, StaticObstacleBuildResult, StaticObstacleBuildStats,
};

pub use static_obstacle_builder::_rust;
