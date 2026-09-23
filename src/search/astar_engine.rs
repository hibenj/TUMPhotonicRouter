//! `AStarSearch`, the single implementation of `NetSearch` today. Its body
//! only dispatches to the four existing `unified_kernel` wrappers in
//! `crate::astar` -- unchanged, byte for byte -- so this slice introduces
//! the interface without moving or rewriting the kernel itself. Slice 2
//! moves this file into `search/astar/mod.rs`; Slice 3 rewrites the kernel
//! loop the wrappers call.

use crate::astar::unified_kernel;
use crate::astar::RouteSearchStats;

use super::{NetSearch, SearchEnvironment, SearchOutcome, SearchRequest};

/// The kernel today's production callers have always used, now reached
/// through `NetSearch` instead of the deleted four-method single-net-search
/// trait.
pub struct AStarSearch;

impl NetSearch for AStarSearch {
    /// Reproduces the dispatch the old trait's four methods encoded as four
    /// distinct call sites: which wrapper runs depends only on whether the
    /// request carries a `crossing` part and,
    /// if so, whether that part carries its own `reservation_open_cells`.
    ///
    /// `crossing: None` (dynamic_expansion either way) selects between the
    /// plain wrapper (dynamic_expansion: None -- the only variant that
    /// tries the JPS4 and simple-route shortcuts before falling back to the
    /// windowed kernel) and the dynamic-expansion wrapper (dynamic_expansion:
    /// Some(_) -- no shortcuts, a widened clearance halo instead).
    ///
    /// `crossing: Some(c)` always reaches one of the two crossing wrappers.
    /// Reading both (`route_single_net_with_unified_kernel_collision_crossing_config_with_stats`
    /// and `route_single_net_with_unified_kernel_crossing_config`,
    /// `src/astar.rs`) shows they differ in exactly one place: the collision
    /// wrapper builds a `reservation_anchor_open_cells` set from
    /// `reservation_open_cells.or(port_open_cells)` plus the source/target
    /// cells and passes it as the hook's *reservation* anchor slot, while
    /// the *port* anchor slot always gets the `port_open_cells`-derived set;
    /// the crossing-config wrapper has no `reservation_open_cells` parameter
    /// at all and passes its one `port_open_cells`-derived set for *both*
    /// hook slots. When `c.reservation_open_cells` is `None`, the collision
    /// wrapper's `.or(port_open_cells)` fallback makes
    /// `reservation_anchor_open_cells` built from the exact same source
    /// (`port_open_cells` plus source/target) as its `anchor_open_cells` --
    /// so the two sets are equal in content, and the collision wrapper's
    /// hook call becomes identical to the crossing-config wrapper's. Every
    /// other step (guard clauses, `CrossingHookContext::build`, the
    /// windowed kernel call) is already the same code path in both
    /// wrappers. So "reservation_open_cells absent" *is* behaviourally
    /// identical to what the crossing-config wrapper does today -- no
    /// `crossing_variant` compatibility flag is needed on `CrossingSearch`.
    ///
    /// That leaves a choice of *which* wrapper to call for
    /// `reservation_open_cells: None`: the crossing-config wrapper named by
    /// the dispatch rule, or the (proven-equivalent) collision wrapper. This
    /// picks the crossing-config wrapper, literally, because: (1) it is the
    /// wrapper the three `search_with_crossing_config` production call
    /// sites reached before this slice, so naming it here keeps that fact
    /// visible in the dispatch instead of relying only on the equivalence
    /// proof above; (2) calling the collision wrapper instead would leave
    /// the crossing-config wrapper referenced only from tests, which is a
    /// `dead_code` warning risk in a non-test build the gate checks for
    /// (`cargo build --release` warning count). Its cost is stats: the
    /// crossing-config wrapper's signature is `-> Option<RouteResult>` with
    /// no stats output at all, and this slice does not change wrapper
    /// bodies, so there is truly no way to plumb real counters out of it
    /// without touching its signature. `SearchOutcome::stats` is therefore
    /// `RouteSearchStats::default()` for this one variant. This is zero
    /// behaviour change: all three call sites that reach this branch
    /// (`src/engine/search_calls.rs`) already discarded the trait's
    /// `search_with_crossing_config` return value's stats too -- that
    /// method never had a stats output either, old or new.
    fn search(&self, env: &SearchEnvironment, request: &SearchRequest) -> SearchOutcome {
        if let Some(crossing) = &request.crossing {
            let dynamic_expansion_radius_cells = request
                .dynamic_expansion
                .as_ref()
                .map_or(0, |dynamic| dynamic.radius_cells);
            let dynamic_clearance_exempt_cells = request
                .dynamic_expansion
                .as_ref()
                .and_then(|dynamic| dynamic.clearance_exempt_cells);
            return if let Some(reservation_open_cells) = crossing.reservation_open_cells {
                let (route, stats) =
                    unified_kernel::route_single_net_with_unified_kernel_collision_crossing_config_with_stats(
                        env.obstacle_map,
                        env.primitives,
                        request.source,
                        request.target,
                        request.port_open_cells,
                        Some(reservation_open_cells),
                        request.config,
                        dynamic_expansion_radius_cells,
                        dynamic_clearance_exempt_cells,
                        crossing.config,
                    );
                SearchOutcome { route, stats }
            } else {
                let route = unified_kernel::route_single_net_with_unified_kernel_crossing_config(
                    env.obstacle_map,
                    env.primitives,
                    request.source,
                    request.target,
                    request.port_open_cells,
                    request.config,
                    dynamic_expansion_radius_cells,
                    dynamic_clearance_exempt_cells,
                    crossing.config,
                );
                SearchOutcome {
                    route,
                    stats: RouteSearchStats::default(),
                }
            };
        }

        if let Some(dynamic) = &request.dynamic_expansion {
            let mut stats = RouteSearchStats::default();
            let route = unified_kernel::route_single_net_with_unified_kernel_dynamic_expansion_config_reporting_stats(
                env.obstacle_map,
                env.primitives,
                request.source,
                request.target,
                request.port_open_cells,
                request.config,
                dynamic.radius_cells,
                dynamic.clearance_exempt_cells,
                &mut stats,
            );
            return SearchOutcome { route, stats };
        }

        let mut stats = RouteSearchStats::default();
        let route = unified_kernel::route_single_net_with_unified_kernel_config_reporting_stats(
            env.obstacle_map,
            env.primitives,
            request.source,
            request.target,
            request.port_open_cells,
            request.config,
            &mut stats,
        );
        SearchOutcome { route, stats }
    }
}
