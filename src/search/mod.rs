//! One interface for a single-net search: `NetSearch`. See Milestone 3 of
//! `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
//!
//! # Search engines
//!
//! Every single-net search in this crate goes through the `NetSearch` trait
//! (one method, `search(&SearchEnvironment, &SearchRequest) -> SearchOutcome`).
//! Two implementations exist:
//!
//! * `AStarSearch` (`astar/mod.rs`), the production engine: the full A*
//!   with its heuristic, routing windows, JPS4 and simple-route shortcuts
//!   and crossing support, all reached through one `run_search` body.
//! * `GridDijkstraSearch` (`grid_dijkstra.rs`), a uniform-cost search over
//!   the same state space, built by calling the A* kernel's own building
//!   blocks with a zero heuristic, no routing window and no crossing hook.
//!   It proves the seam is real and serves as an optimality oracle in
//!   tests; it does not serve crossing requests.
//!
//! To add a third engine: implement `NetSearch` in its own module here,
//! and give it a name in `PyPhotonicRouter::construct` (`src/engine/mod.rs`),
//! which boxes the engine `RouterConfig::search.engine` selects. The
//! accepted names are validated in `PyRouterConfig::new`.
//!
//! `state` and `geometry` hold the search-state/result types and the
//! grid-polyline geometry helpers both engines share; `astar` is the A*
//! implementation's own module tree.

pub mod astar;
pub mod geometry;
pub mod grid_dijkstra;
pub mod state;
#[cfg(test)]
pub(crate) mod test_support;

use crate::obstacle_map::{pack_xy, CellKey, ObstacleMap};
use crate::primitives::PrimitiveLibrary;
use astar::config::AStarConfig;
use astar::crossing_rules::CrossingSearchConfig;
use rustc_hash::FxHashSet;
use state::{RouteResult, RouteSearchStats, State};

pub use astar::AStarSearch;
pub use grid_dijkstra::GridDijkstraSearch;

/// The cells a search may treat as open regardless of ownership: the
/// request's own `port_open_cells` plus the source and target cells. Both
/// engines build this set the same way and hand it to the kernel's dense
/// grid construction as its `port_open_cells` argument.
pub(crate) fn anchor_open_cells(
    source: State,
    target: State,
    port_open_cells: Option<&FxHashSet<CellKey>>,
) -> FxHashSet<CellKey> {
    let mut anchor_open_cells = FxHashSet::default();
    if let Some(port_open_cells) = port_open_cells {
        anchor_open_cells.extend(port_open_cells.iter().copied());
    }
    anchor_open_cells.insert(pack_xy(source.x, source.y));
    anchor_open_cells.insert(pack_xy(target.x, target.y));
    anchor_open_cells
}

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
/// selects the crossing-config path, which reuses the `port_open_cells`
/// anchor set as the reservation anchor set too -- behaviourally the same
/// thing `Some(port_open_cells)` produces on the collision path (see
/// `run_search`'s doc comment and
/// `crossing_reservation_none_matches_an_equal_reservation_anchor_set`).
/// Absent (`SearchRequest::crossing == None`) means no crossing support at
/// all: `NoCrossingHook` rather than `LiveCrossingHook`.
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
/// of cause (infeasible, timed out, budget exhausted); `stats` carries the
/// search's real counters on every path (a request rejected by an entry
/// guard reports `RouteSearchStats::default()`, as it always has).
pub struct SearchOutcome {
    pub route: Option<RouteResult>,
    pub stats: RouteSearchStats,
}

/// A pluggable single-net search algorithm: `AStarSearch` (production) and
/// `GridDijkstraSearch` (the oracle that proves the seam). See the module
/// doc comment above for how to add another.
pub trait NetSearch {
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome;
}
