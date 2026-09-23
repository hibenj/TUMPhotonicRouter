use std::cell::RefCell;
use std::time::Instant;

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::astar::{AStarConfig, TerminalBumpGuard};
use crate::config::RouterConfig;
use crate::crossings::CrossingContext;
use crate::obstacle_map::{CellKey, ObstacleMap};
use crate::plm::RegisteredPlmContext;
use crate::primitives::{
    create_grid4_unit_grid_primitive_library, create_jps4_unit_grid_primitive_library,
    create_photonic_primitive_library, PrimitiveLibrary, PrimitiveLibraryConfig,
};

use crate::bindings::*;

pub(crate) mod cells;
pub(crate) mod commit;
pub(crate) mod crossing_geometry;
pub(crate) mod crossing_reservation;
pub(crate) mod diagnostics;
pub(crate) mod endpoint_correction;
pub(crate) mod jobs;
pub(crate) mod legacy_repair;
pub(crate) mod negotiation;
pub(crate) mod search_calls;
#[cfg(test)]
pub(crate) mod test_support;

pub(crate) use cells::*;
pub(crate) use commit::*;
pub(crate) use crossing_geometry::*;
pub(crate) use crossing_reservation::*;
pub(crate) use diagnostics::*;
pub(crate) use endpoint_correction::*;
pub(crate) use jobs::*;
pub(crate) use negotiation::*;
pub(crate) use search_calls::*;

#[derive(Clone, Default)]
pub(crate) struct MeanderRegistrationProfile {
    pub(crate) total_s: f64,
    pub(crate) reset_s: f64,
    pub(crate) base_static_pack_s: f64,
    pub(crate) base_static_obstacle_add_s: f64,
    pub(crate) base_prefix_build_s: f64,
    pub(crate) route_extract_s: f64,
    pub(crate) route_cell_collect_s: f64,
    pub(crate) open_set_build_s: f64,
    pub(crate) route_cell_list_s: f64,
    pub(crate) route_static_add_s: f64,
    pub(crate) registered_store_s: f64,
    pub(crate) route_count: usize,
    pub(crate) base_static_cell_count: usize,
    pub(crate) unique_route_cell_count: usize,
    pub(crate) registered_open_cell_count: usize,
}

#[pyclass(name = "PyPhotonicRouter")]
pub struct PyPhotonicRouter {
    pub(crate) grid: PyGridSpec,
    pub(crate) primitive_cfg: PyPrimitiveLibraryConfig,
    pub(crate) astar_cfg: PyAStarConfig,
    pub(crate) astar_cfg_cached: Result<AStarConfig, String>,
    // Typed configuration tree replacing the `PHOTONIC_ROUTER_*`
    // environment variables read inside `src/astar.rs` and
    // `src/py_router.rs` (Milestone 1 of
    // `.agent/execplans/2026-09-22-modular-readable-router-restructure.md`).
    // Set once at construction from the optional `router_config` argument
    // of `PyPhotonicRouter::new` (`RouterConfig::default()` when omitted)
    // and read wherever this module used to read the environment directly.
    pub(crate) router_config: RouterConfig,
    pub(crate) obstacle_map: ObstacleMap,
    pub(crate) primitives: PrimitiveLibrary,
    pub(crate) crossing_context: CrossingContext,
    pub(crate) committed_center_routes: FxHashMap<u64, Vec<(i32, i32)>>,
    pub(crate) committed_realized_center_routes: FxHashMap<u64, Vec<(f64, f64)>>,
    pub(crate) committed_target_terminal_bump_guards: FxHashMap<u64, TerminalBumpGuard>,
    pub(crate) committed_opened_cell_keys: FxHashMap<u64, FxHashSet<CellKey>>,
    pub(crate) crossing_events: Vec<CrossingEvent>,
    pub(crate) use_collision_crossing_routing: bool,
    pub(crate) static_cells: FxHashSet<CellKey>,
    pub(crate) port_open_cells: FxHashSet<CellKey>,
    pub(crate) registered_plm: RefCell<RegisteredPlmContext>,
    pub(crate) last_meander_registration_profile: RefCell<Option<MeanderRegistrationProfile>>,
    pub(crate) last_pending_straight_victim: RefCell<Option<PendingStraightVictimHint>>,
    pub(crate) long_straight_congestion_cells: FxHashMap<CellKey, u32>,
    pub(crate) long_straight_congestion_records: Vec<LongStraightCongestionRecord>,
    // Set once at the top of `route_many_with_repair_and_commit` from that
    // call's own `history_increment`/`block_radius_cells` arguments, reset
    // to 0 (a no-op amount) when the batch finishes. Read unconditionally by
    // every successful-commit chokepoint
    // (`commit_native_route_with_clearance_internal`,
    // `commit_native_route_with_clearance_allowing_core_overlap`) so every
    // committed route -- not just 3 narrow repair call sites -- accumulates
    // history cost, matching LiDAR's `registar_net`/`updateHistoryMap`
    // (`drgridroute.py:443`/`703`, unconditional on every successful
    // commit). See `.agent/execplans/2026-08-25-negotiated-repair-engine.md`
    // Milestone 4.
    pub(crate) commit_history_increment: u32,
    pub(crate) commit_history_block_radius_cells: i32,
    // Set once at the top of `route_many_with_negotiated_repair_and_commit`
    // from that call's own `history_weight` argument, reset to 0.0 (a no-op
    // override, matching `AStarConfig`'s own baseline) everywhere else.
    // Read by `route_single_net_and_commit_native` to weight its A* search
    // by accumulated history cost -- closing Milestone 5's documented gap
    // (history cost accumulated via Milestone 4 but never fed back into
    // search cost, root-caused as why the negotiated loop did not converge
    // on `benes_16x16` in practical time). See
    // `.agent/execplans/2026-08-25-negotiated-repair-engine.md` Milestone 6.
    pub(crate) commit_history_weight: f64,
    // Physical waveguide width used by the realized-crossing commit
    // validation's parallel-overlap check: two committed centerlines closer
    // than this overlap as polygons even when they never intersect (the
    // multiportmmi_8x8 n_13/n_14 adjacent-diagonal case). Settable from
    // Python via `set_route_width_um`; 0.5 um is the repository-wide
    // realization default (`realize_routed_net_records`).
    pub(crate) route_width_um: f64,
    // Wall-clock instant the negotiated repair loop
    // (`route_many_with_negotiated_repair_and_commit`) started, `None`
    // outside it. Used only to timestamp `PHOTONIC_ROUTER_NATIVE_REPAIR_DIAG`
    // trace lines (`trace_t`): set to `Some(Instant::now())` at the top of
    // that loop and back to `None` before it returns. Two of its helpers
    // (`probe_net_for_repair`, `try_lidar_direct_crossing_subset`) are
    // shared with the older `route_many_with_repair_and_commit` chain, which
    // never sets this field, so their trace lines print no `t=` field there
    // and stay byte-identical to before this milestone. See
    // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    // Milestone 5.
    pub(crate) negotiated_batch_start: Option<Instant>,
    // Fail-fast expansion budget for the *next* search only: `None` outside
    // `route_many_with_negotiated_repair_and_commit`. That loop sets this to
    // `Some(NEGOTIATED_BUDGET_FIRST_ATTEMPT | NEGOTIATED_BUDGET_RETRY)`
    // immediately before a plain/direct-crossing search call and back to
    // `None` immediately after, so `astar_config` (the one place this field
    // is read, applied to `AStarConfig::total_expansion_budget`) only ever
    // sees it non-`None` for that one call -- never for the probe search
    // (`route_single_net_ignore_dynamic_native`, also built through
    // `astar_config`, must stay unbounded per Milestone 5's profile: it
    // costs about a second regardless) and never for the older repair chain,
    // which never touches this field and so always reads `None` here. See
    // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    // Milestone 5.
    pub(crate) negotiated_search_budget: Option<u64>,
    // Contribution 1's crossing-free search for unplanned nets (owner
    // decision 2026-09-16 00:10, see
    // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`
    // Milestone 7): `true` only while
    // `route_many_with_negotiated_repair_and_commit` searches a net whose
    // topology plan has no crossing at all. `route_single_net_and_commit_native`
    // then skips its deferred lidar-pure crossing search entirely, so the
    // net's plain search is the ordinary crossing-free A* -- every committed
    // net stays an obstacle. Never set by the older chain, never set for
    // the probe search, and reset to `false` right after every search that
    // set it.
    pub(crate) negotiated_crossing_free_search: bool,
    // Nets searched with `long_straight_congestion_weight = 0` by the
    // negotiated loop (2026-09-17, multiportmmi_128x128): the fan-in /
    // fan-out bands of dense multi-port instances must pack their lanes
    // tightly in parallel, which the penalty forbids (with it: bundle
    // rip-ups every round; without it: 0 rip-ups in both bands). Filled by
    // `set_long_straight_exempt_net_ids` (Python: the nets with a static
    // fan-out anchor, env PHOTONIC_ROUTER_LONG_STRAIGHT_EXEMPT_DENSE_FANOUT=1).
    pub(crate) long_straight_exempt_net_ids: FxHashSet<u64>,
    // `Some(weight)` while the negotiated loop searches an exempt net:
    // `astar_config` then uses it instead of the environment's weight.
    pub(crate) long_straight_weight_override: Option<f64>,
    // Expanded-states count of the most recent single-net search
    // `route_single_net_and_commit_native` ran, success or failure. Set
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
    pub(crate) last_search_expanded_states: u64,
}

#[derive(Clone, Debug)]
pub(crate) struct PendingStraightVictimHint {
    pub(crate) net_id: u64,
    pub(crate) victim_net_id: u64,
    pub(crate) count: usize,
}

#[derive(Clone, Debug)]
pub(crate) struct LongStraightCongestionRecord {
    pub(crate) net_id: u64,
    pub(crate) start: (i32, i32),
    pub(crate) end: (i32, i32),
    pub(crate) length_um: f64,
    pub(crate) marked_cells: usize,
}

impl PyPhotonicRouter {
    pub(crate) fn construct(
        grid_spec: PyGridSpec,
        primitive_config: PyPrimitiveLibraryConfig,
        astar_config: PyAStarConfig,
        router_config: Option<PyRouterConfig>,
    ) -> Self {
        let router_config: RouterConfig = router_config
            .as_ref()
            .map(RouterConfig::from)
            .unwrap_or_default();
        let primitives = if primitive_config.jps4_unit_grid {
            create_jps4_unit_grid_primitive_library(primitive_config.grid_size_um)
        } else if primitive_config.grid4_unit_grid {
            create_grid4_unit_grid_primitive_library(primitive_config.grid_size_um)
        } else {
            create_photonic_primitive_library(PrimitiveLibraryConfig {
                grid_size_um: primitive_config.grid_size_um,
                straight_short_cells: primitive_config.straight_short_cells,
                straight_long_cells: primitive_config.straight_long_cells,
                bend_radius_cells: primitive_config.bend_radius_cells,
                allow_45_degree_turns: primitive_config.allow_45_degree_turns,
            })
        };
        let astar_cfg_cached = astar_config_from_py(
            &astar_config,
            &primitive_config,
            None,
            None,
            None,
            None,
            &router_config,
        )
        .map_err(|err| err.to_string());
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
            use_collision_crossing_routing: false,
            static_cells: FxHashSet::default(),
            port_open_cells: FxHashSet::default(),
            registered_plm: RefCell::new(RegisteredPlmContext::default()),
            last_meander_registration_profile: RefCell::new(None),
            last_pending_straight_victim: RefCell::new(None),
            long_straight_congestion_cells: FxHashMap::default(),
            long_straight_congestion_records: Vec::new(),
            commit_history_increment: 0,
            commit_history_block_radius_cells: 0,
            commit_history_weight: 0.0,
            route_width_um: 0.5,
            negotiated_batch_start: None,
            negotiated_search_budget: None,
            negotiated_crossing_free_search: false,
            long_straight_exempt_net_ids: FxHashSet::default(),
            long_straight_weight_override: None,
            last_search_expanded_states: 0,
        }
    }
}
