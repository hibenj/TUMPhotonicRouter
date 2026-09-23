//! One interface for a single-net search: `NetSearch`. `AStarSearch`
//! (`astar/mod.rs`) is today's, and only, implementation -- it dispatches to
//! the four unified-kernel wrappers in `crate::search::astar::kernel`
//! unchanged, so this slice is the interface itself, not a behaviour
//! change. See Milestone 3, Slices 1 and 2 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
//!
//! `state` and `geometry` hold the search-state/result types and the
//! grid-polyline geometry helpers shared by `astar` and any future second
//! engine; `astar` is the A* implementation's own module tree.

pub mod astar;
pub mod geometry;
pub mod state;
#[cfg(test)]
pub(crate) mod test_support;

use crate::obstacle_map::{CellKey, ObstacleMap};
use crate::primitives::PrimitiveLibrary;
use astar::config::AStarConfig;
use astar::crossing_rules::CrossingSearchConfig;
use rustc_hash::FxHashSet;
use state::{RouteResult, RouteSearchStats, State};

pub use astar::AStarSearch;

/// The obstacle map and primitive library a search runs against. Neither
/// switches on anything by itself -- every request against the same
/// environment sees the same map and the same primitive set.
pub struct SearchEnvironment<'a> {
    pub obstacle_map: &'a ObstacleMap,
    pub primitives: &'a PrimitiveLibrary,
}

/// Present on a `SearchRequest` to widen the search past its normal
/// clearance halo (used by repair to let a route brush past a net it is
/// about to rip up). `radius_cells` is the widened halo radius;
/// `clearance_exempt_cells` are the cells exempt from the ordinary
/// clearance check regardless of radius. Absent (`SearchRequest::dynamic_expansion
/// == None`) means the plain clearance rule applies, which also unlocks the
/// JPS4 and simple-route shortcuts that the dynamic-expansion path forgoes.
pub struct DynamicExpansion<'a> {
    pub radius_cells: i32,
    pub clearance_exempt_cells: Option<&'a FxHashSet<CellKey>>,
}

/// Present on a `SearchRequest` to search for a route that crosses one or
/// more named partner nets, per `config`. `reservation_open_cells`
/// distinguishes the two crossing search variants that exist today:
/// `Some(_)` selects the collision-crossing path (an explicit reservation
/// anchor set, independent of `SearchRequest::port_open_cells`); `None`
/// selects the crossing-config path, whose own wrapper always reuses
/// `port_open_cells` as the reservation anchor set too -- behaviourally the
/// same thing `Some(port_open_cells)` would produce on the collision path,
/// confirmed by reading both wrappers (see `AStarSearch::search`'s doc
/// comment). Absent (`SearchRequest::crossing == None`) means no crossing
/// support at all: `NoCrossingHook` rather than `LiveCrossingHook`.
pub struct CrossingSearch<'a> {
    pub config: &'a CrossingSearchConfig,
    pub reservation_open_cells: Option<&'a FxHashSet<CellKey>>,
}

/// One single-net search request. `port_open_cells` are the cells the
/// search may treat as open regardless of ownership (typically the two
/// endpoints' own port footprints); it is read by every variant. See
/// `DynamicExpansion` and `CrossingSearch` for what their presence switches
/// on.
pub struct SearchRequest<'a> {
    pub source: State,
    pub target: State,
    pub port_open_cells: Option<&'a FxHashSet<CellKey>>,
    pub dynamic_expansion: Option<DynamicExpansion<'a>>,
    pub crossing: Option<CrossingSearch<'a>>,
    pub config: &'a AStarConfig,
}

/// The result of one search call: `route` is `None` on failure regardless
/// of cause (infeasible, timed out, budget exhausted); `stats` is always
/// populated (a variant that cannot expose real counters returns
/// `RouteSearchStats::default()` -- see `AStarSearch::search`'s doc comment
/// for which variant that is today and why).
pub struct SearchOutcome {
    pub route: Option<RouteResult>,
    pub stats: RouteSearchStats,
}

/// A pluggable single-net search algorithm. `AStarSearch` is the only
/// implementation today; Milestone 3 Slice 4 adds a second (a grid
/// Dijkstra) to prove the seam.
pub trait NetSearch {
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome;
}
