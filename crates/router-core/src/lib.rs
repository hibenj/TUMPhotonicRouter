//! The Rust core of the photonic waveguide router.
//!
//! `obstacle_map` is the routing database, `search` the single-net A*
//! kernel with its crossing rules and the simple-route probe, `engine` the
//! negotiated rip-up-and-reroute loop, `crossings`/`geometry_realization`/
//! `meander` the crossing, geometry and path-length-matching stages.
//! The crate has no Python dependency; the PyO3 surface the Python package
//! drives is the `router-py` crate (phase 8 slice B of the 2026-10 do-over).

pub mod config;
pub mod crossings;
pub mod engine;
pub mod error;
pub mod geometry_realization;
pub mod meander;
pub mod obstacle_map;
pub mod primitives;
pub mod search;
pub mod static_obstacle_builder;

pub use config::{
    CrossingEngineConfig, GridSpec, KernelDiagnostics, NegotiationConfig, NetNameTrace,
    RouterConfig, SearchOverrides,
};
pub use crossings::{CrossingConfig, CrossingContext, CrossingPair};
pub use engine::Router;
pub use error::RouterError;
pub use geometry_realization::{
    compress_grid_waypoints, compress_route_waypoints, generate_waveguide_polygon,
    grid_path_to_centerline, realize_route_polygon, realize_route_polygon_from_auto_plan,
    realize_route_polygon_from_primitives, route_to_grid_path, route_to_primitive_centerline,
    snap_centerline_endpoints, GeometryError, GeometryGridSpec,
};
pub use meander::geometry::{
    actual_bend_radius_um_from_cells, bend_radius_cells_from_min_radius, plan_analytic_meander,
    plan_fill_box_multi_bump_meander, AnalyticMeanderConfig, AnalyticMeanderPlan, MeanderBox,
    MeanderPlanningError, MeanderPlanningMode, MeanderSide, PhysicalPoint, StraightSegment,
};
pub use meander::search::{
    cells_in_grid_rect, check_meander_box_free_with_prefix, meander_box_to_grid_rect,
    AutoMeanderConfig, AutoMeanderSidePolicy, AutoRouteAnalyticMeanderPlan, DenseOccupancyPrefix,
    MeanderGridRect, SparseCellIndex,
};
pub use obstacle_map::{
    pack_xy, unpack_xy, CellKey, ClearanceMetric, NetId, ObstacleMap, WaveguideFootprint,
};
pub use primitives::{
    create_photonic_primitive_library, Primitive, PrimitiveGeometry, PrimitiveLibrary,
    PrimitiveLibraryConfig, DIRECTIONS,
};
pub use search::astar::config::{AStarConfig, AStarOverrides, AStarParams};
pub use search::astar::svg::{export_route_svg, export_route_svg_with_port_open_cells};
pub use search::astar::{route_single_net, route_single_net_with_config};
pub use search::state::{RouteResult, State};
pub use search::{
    AStarSearch, CrossingSearch, DynamicExpansion, NetSearch, SearchEnvironment, SearchOutcome,
    SearchRequest,
};
pub use static_obstacle_builder::{
    build_static_obstacle_map_from_geometry, compute_bbox, expand_bbox, grid_cell_center,
    make_grid_spec, parse_clearance_metric, parse_obstacle_mode, physical_to_grid,
    rasterize_polygon, BBox, GridCell, Point, Polygon, PortInput, StaticCellSet, StaticGridSpec,
    StaticObstacleBuildConfig, StaticObstacleBuildMode, StaticObstacleBuildResult,
    StaticObstacleBuildStats,
};
