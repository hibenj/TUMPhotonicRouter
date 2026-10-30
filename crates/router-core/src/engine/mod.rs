use std::time::Instant;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::config::{GridSpec, RouterConfig, SEARCH_ENGINE_GRID_DIJKSTRA};
use crate::crossings::CrossingContext;
use crate::error::RouterError;
use crate::meander::geometry::{
    actual_bend_radius_um_from_cells, bend_radius_cells_from_min_radius,
};
use crate::obstacle_map::{CellKey, ObstacleMap};
use crate::primitives::{
    create_photonic_primitive_library, PrimitiveLibrary, PrimitiveLibraryConfig,
};
use crate::search::astar::config::{AStarConfig, AStarOverrides, AStarParams};
use crate::search::astar::crossing_rules::TerminalBumpGuard;
use crate::search::{AStarSearch, GridDijkstraSearch, NetSearch};

pub mod api;
pub mod cells;
pub(crate) mod commit;
pub mod crossing_geometry;
pub mod crossing_reservation;
pub mod diagnostics;
pub(crate) mod endpoint_correction;
pub mod jobs;
pub mod negotiation;
pub mod search_call;
pub(crate) mod search_calls;
#[cfg(test)]
pub(crate) mod test_support;

pub use cells::*;
pub(crate) use commit::*;
pub use crossing_geometry::*;
pub use crossing_reservation::*;
pub use diagnostics::*;
pub(crate) use endpoint_correction::*;
pub use jobs::*;
pub use negotiation::*;
pub use search_call::*;
pub(crate) use search_calls::*;

#[derive(Clone, Default)]
pub struct MeanderRegistrationProfile {
    pub total_s: f64,
    pub reset_s: f64,
    pub base_static_pack_s: f64,
    pub base_static_obstacle_add_s: f64,
    pub base_prefix_build_s: f64,
    pub route_extract_s: f64,
    pub route_cell_collect_s: f64,
    pub open_set_build_s: f64,
    pub route_cell_list_s: f64,
    pub route_static_add_s: f64,
    pub registered_store_s: f64,
    pub route_count: usize,
    pub base_static_cell_count: usize,
    pub unique_route_cell_count: usize,
    pub registered_open_cell_count: usize,
}

/// The routing engine: the obstacle map, the committed routes and their
/// crossing records, and the configuration every search reads. The Python
/// `PyPhotonicRouter` class (`router-py`) wraps one.
pub struct Router {
    pub grid: GridSpec,
    pub primitive_cfg: PrimitiveLibraryConfig,
    pub astar_cfg: AStarParams,
    pub astar_cfg_cached: Result<AStarConfig, String>,
    // Typed configuration tree replacing the `PHOTONIC_ROUTER_*`
    // environment variables read inside `src/astar.rs` and
    // `src/py_router.rs` (Milestone 1 of
    // `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`).
    // Set once at construction from the optional `router_config` argument
    // of `PyPhotonicRouter::new` (`RouterConfig::default()` when omitted)
    // and read wherever this module used to read the environment directly.
    pub router_config: RouterConfig,
    pub obstacle_map: ObstacleMap,
    pub primitives: PrimitiveLibrary,
    pub crossing_context: CrossingContext,
    pub committed_center_routes: FxHashMap<u64, Vec<(i32, i32)>>,
    pub committed_realized_center_routes: FxHashMap<u64, Vec<(f64, f64)>>,
    pub committed_target_terminal_bump_guards: FxHashMap<u64, TerminalBumpGuard>,
    pub committed_opened_cell_keys: FxHashMap<u64, FxHashSet<CellKey>>,
    pub crossing_events: Vec<CrossingEvent>,
    /// The verdicts of the unified crossing-legality predicate for every
    /// realized intersection the post-commit validation assessed, one per
    /// (net, partner, point); empty unless `router_config.crossing.
    /// unified_legality` is on. Read by the `crossing_verdicts()` binding.
    pub crossing_verdicts: Vec<CrossingVerdictRecord>,
    pub static_cells: FxHashSet<CellKey>,
    pub port_open_cells: FxHashSet<CellKey>,
    /// Bumped whenever the obstacle map changes (every commit, rip-up,
    /// static-cell edit: `invalidate_meander_base_prefix`). A
    /// `MeanderPlanner` on this router rebuilds its dense occupancy prefix
    /// when the counter moved since it built the prefix; before phase 6
    /// slice C the prefix lived here and the same call sites set it to
    /// `None`.
    pub obstacle_generation: u64,
    /// Bumped by `clear_static_cells`, which also dropped the registered
    /// routes and reserved cells while they lived here; a `MeanderPlanner`
    /// drops its registration when it sees the counter moved.
    pub static_reset_generation: u64,
    pub long_straight_congestion_cells: FxHashMap<CellKey, u32>,
    pub long_straight_congestion_records: Vec<LongStraightCongestionRecord>,
    // Set once at the top of the negotiated repair loop from that call's
    // own `history_increment`/`block_radius_cells` arguments, reset
    // to 0 (a no-op amount) when the batch finishes. Read unconditionally by
    // the successful-commit chokepoint (`add_post_commit_guidance_for_route`,
    // `commit.rs`) so every
    // committed route -- not just 3 narrow repair call sites -- accumulates
    // history cost, matching LiDAR's `registar_net`/`updateHistoryMap`
    // (`drgridroute.py:443`/`703`, unconditional on every successful
    // commit). See `.agent/execplans/2026-08-25-negotiated-repair-engine.md`
    // Milestone 4.
    pub commit_history_increment: u32,
    pub commit_history_block_radius_cells: i32,
    // Physical waveguide width used by the realized-crossing commit
    // validation's parallel-overlap check: two committed centerlines closer
    // than this overlap as polygons even when they never intersect (the
    // multiportmmi_8x8 n_13/n_14 adjacent-diagonal case). Settable from
    // Python via `set_route_width_um`; 0.5 um is the repository-wide
    // realization default (`realize_routed_net_records`).
    pub route_width_um: f64,
    // Wall-clock instant the negotiated repair loop
    // (`route_many_with_negotiated_repair_and_commit`) started, `None`
    // outside it. Used only to timestamp `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG`
    // trace lines (`trace_t`): set to `Some(Instant::now())` at the top of
    // that loop and back to `None` before it returns. One of its helpers
    // (`probe_net_for_repair`) was shared with the older repair chain,
    // deleted in Milestone 8 of
    // `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`,
    // which never set this field, so its trace lines printed no `t=`
    // field there. See
    // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    // Milestone 5.
    pub negotiated_batch_start: Option<Instant>,
    // Nets searched with `long_straight_congestion_weight = 0` by the
    // negotiated loop (2026-09-17, multiportmmi_128x128): the fan-in /
    // fan-out bands of dense multi-port instances must pack their lanes
    // tightly in parallel, which the penalty forbids (with it: bundle
    // rip-ups every round; without it: 0 rip-ups in both bands). Filled by
    // `set_long_straight_exempt_net_ids` (Python: the nets with a static
    // fan-out anchor, env PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT=1).
    pub long_straight_exempt_net_ids: FxHashSet<u64>,
    // Expanded-states count of the most recent single-net search a plain
    // `route_and_commit` call ran, success or failure. Set
    // right after that search returns (before its `Option<RouteResult>` is
    // turned into the function's `Result`, so a failed search's expansion
    // count survives past the `?` that discards its `RouteResult`) and read
    // only by `route_many_with_negotiated_repair_and_commit`'s
    // `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG` trace lines, to print a real
    // `expanded=` count for `outcome=failed` instead of `expanded=?`. Not
    // reset between calls, so it always holds the last search's count
    // regardless of which kind (plain/post_ripup_plain) ran it. See
    // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    // Milestone 5.
    pub last_search_expanded_states: u64,
    // The single-net search algorithm every production call site now
    // reaches through `NetSearch::search` instead of the deleted
    // four-method single-net-search trait. Set once at construction from
    // `RouterConfig::search.engine` (`search_engine_for` below) and never
    // reassigned: `AStarSearch` by default, `GridDijkstraSearch` for
    // `"grid-dijkstra"`. See Milestone 3, Slices 1 and 4 of
    // `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`.
    pub search_engine: Box<dyn NetSearch + Send + Sync>,
}

#[derive(Clone, Debug)]
pub struct LongStraightCongestionRecord {
    pub net_id: u64,
    pub start: (i32, i32),
    pub end: (i32, i32),
    pub length_um: f64,
    pub marked_cells: usize,
}

/// The `NetSearch` implementation `RouterConfig::search.engine` names. Add
/// a new engine's name here (see `crate::search`'s module doc comment).
/// Unknown values keep A*: the Python binding rejects them at construction
/// (`PyRouterConfig::new`), so a `RouterConfig` built directly in Rust is
/// the only way to get one here.
pub(crate) fn search_engine_for(config: &RouterConfig) -> Box<dyn NetSearch + Send + Sync> {
    if config.search.engine == SEARCH_ENGINE_GRID_DIJKSTRA {
        Box::new(GridDijkstraSearch)
    } else {
        Box::new(AStarSearch)
    }
}

impl Router {
    /// A router on an empty grid. The A* settings are resolved once here;
    /// an invalid setting does not fail construction but every search call
    /// (`astar_config`) then returns its message, as the Python binding has
    /// always done.
    pub fn new(
        grid_spec: GridSpec,
        primitive_config: PrimitiveLibraryConfig,
        astar_config: AStarParams,
        router_config: RouterConfig,
    ) -> Self {
        let primitives = create_photonic_primitive_library(primitive_config.clone());
        // The cached message keeps the `"ValueError: <message>"` text of the
        // former `PyErr::to_string()`, which is what a search call reports.
        let astar_cfg_cached =
            AStarConfig::from_parts(&astar_config, &AStarOverrides::default(), &router_config)
                .map_err(|err| format!("ValueError: {err}"));
        let search_engine = search_engine_for(&router_config);
        Self {
            obstacle_map: ObstacleMap::new(grid_spec.width as i32, grid_spec.height as i32),
            grid: grid_spec,
            primitive_cfg: primitive_config,
            astar_cfg: astar_config,
            astar_cfg_cached,
            router_config,
            primitives,
            crossing_context: CrossingContext::default(),
            committed_center_routes: FxHashMap::default(),
            committed_realized_center_routes: FxHashMap::default(),
            committed_target_terminal_bump_guards: FxHashMap::default(),
            committed_opened_cell_keys: FxHashMap::default(),
            crossing_events: Vec::new(),
            crossing_verdicts: Vec::new(),
            static_cells: FxHashSet::default(),
            port_open_cells: FxHashSet::default(),
            obstacle_generation: 0,
            static_reset_generation: 0,
            long_straight_congestion_cells: FxHashMap::default(),
            long_straight_congestion_records: Vec::new(),
            commit_history_increment: 0,
            commit_history_block_radius_cells: 0,
            route_width_um: 0.5,
            negotiated_batch_start: None,
            long_straight_exempt_net_ids: FxHashSet::default(),
            last_search_expanded_states: 0,
            search_engine,
        }
    }
}

/// The helpers the bindings and the engine share (phase 3 slice E of the
/// 2026-10 do-over put them outside the `#[pymethods]` block; phase 8
/// slice B moved them here with the engine).
impl Router {
    pub fn effective_bend_radius_um(
        &self,
        min_bend_radius_um: Option<f64>,
    ) -> Result<f64, RouterError> {
        let grid_size_um = self.grid.grid_size_um;
        if !grid_size_um.is_finite() || grid_size_um <= 0.0 {
            return Err(RouterError::Config(
                "grid_size_um must be finite and > 0".to_string(),
            ));
        }
        match min_bend_radius_um {
            None => {
                let cells = self.primitive_cfg.bend_radius_cells;
                actual_bend_radius_um_from_cells(cells, grid_size_um).map_err(RouterError::Config)
            }
            Some(v) => {
                if !v.is_finite() || v <= 0.0 {
                    return Err(RouterError::Config(
                        "min_bend_radius_um must be finite and > 0 when provided".to_string(),
                    ));
                }
                let cells = bend_radius_cells_from_min_radius(v, grid_size_um)
                    .map_err(RouterError::Config)?;
                actual_bend_radius_um_from_cells(cells, grid_size_um).map_err(RouterError::Config)
            }
        }
    }

    /// The obstacle map changed: a `MeanderPlanner` rebuilds its dense
    /// occupancy prefix at its next call (see `obstacle_generation`).
    pub fn invalidate_meander_base_prefix(&mut self) {
        self.obstacle_generation += 1;
    }

    pub fn clear_static_cells(&mut self) {
        self.invalidate_meander_base_prefix();
        self.static_reset_generation += 1;
        self.obstacle_map = ObstacleMap::new(self.grid.width as i32, self.grid.height as i32);
        self.static_cells.clear();
        self.committed_center_routes.clear();
        self.committed_realized_center_routes.clear();
        self.committed_target_terminal_bump_guards.clear();
        self.committed_opened_cell_keys.clear();
        self.crossing_events.clear();
        self.crossing_verdicts.clear();
        self.long_straight_congestion_cells.clear();
        self.long_straight_congestion_records.clear();
    }

    pub fn add_history_cells(&mut self, cells: Vec<(i32, i32)>, amount: u32) {
        for (x, y) in cells {
            self.obstacle_map.add_history_cost(x, y, amount);
        }
    }
}
