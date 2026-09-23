use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::astar::{
    try_simple_route_with_config, try_simple_route_with_dynamic_expansion_config, AStarConfig,
    CrossingSearchConfig, CrossingSearchPartner, HeapTieBreaker, HeuristicMode, PrimitiveOrdering,
    RouteResult, RouteSearchStats, State,
};
use crate::config::RouterConfig;
use crate::crossings::{CrossingConfig, CrossingGuidance};
use crate::geometry_realization::GeometryGridSpec;
use crate::obstacle_map::{pack_xy, unpack_xy, CellKey};
use crate::primitives::PrimitiveLibrary;
use crate::search::{CrossingSearch, DynamicExpansion, SearchEnvironment, SearchRequest};

#[cfg(test)]
use crate::config::SearchOverrides;
#[cfg(test)]
use crate::crossings::CrossingConstraint;
use crate::engine::*;
#[cfg(test)]
use crate::static_obstacle_builder::StaticGridSpec;

pub(crate) fn allowed_angles_to_mask(angles: Option<&Vec<u8>>) -> PyResult<Option<u8>> {
    let Some(angles) = angles else {
        return Ok(None);
    };
    if angles.is_empty() {
        return Err(PyValueError::new_err(
            "allowed_target_angles must not be empty when provided",
        ));
    }
    let mut mask = 0u8;
    for &angle in angles {
        if angle > 7 {
            return Err(PyValueError::new_err(
                "allowed_target_angles entries must be in [0, 7]",
            ));
        }
        mask |= 1u8 << angle;
    }
    Ok(Some(mask))
}

pub(crate) fn parse_primitive_ordering(value: &str) -> PyResult<PrimitiveOrdering> {
    match value.trim().to_ascii_lowercase().as_str() {
        "library" => Ok(PrimitiveOrdering::Library),
        "long_straight_first" => Ok(PrimitiveOrdering::LongStraightFirst),
        "target_biased" => Ok(PrimitiveOrdering::TargetBiased),
        _ => Err(PyValueError::new_err(
            "primitive_ordering must be one of 'library', 'long_straight_first', or 'target_biased'",
        )),
    }
}

pub(crate) fn parse_heuristic_mode(value: &str) -> PyResult<HeuristicMode> {
    match value.trim().to_ascii_lowercase().as_str() {
        "distance" => Ok(HeuristicMode::Distance),
        "heading_aware" => Ok(HeuristicMode::HeadingAware),
        "diagonal_aware" => Ok(HeuristicMode::DiagonalAware),
        _ => Err(PyValueError::new_err(
            "heuristic_mode must be one of 'distance', 'heading_aware', or 'diagonal_aware'",
        )),
    }
}

pub(crate) fn parse_heap_tie_breaker(value: &str) -> PyResult<HeapTieBreaker> {
    match value.trim().to_ascii_lowercase().as_str() {
        "smaller_g" => Ok(HeapTieBreaker::SmallerG),
        "larger_g" => Ok(HeapTieBreaker::LargerG),
        _ => Err(PyValueError::new_err(
            "heap_tie_breaker must be one of 'smaller_g' or 'larger_g'",
        )),
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn astar_config_from_py(
    astar_cfg: &PyAStarConfig,
    primitive_cfg: &PyPrimitiveLibraryConfig,
    ignore_dynamic_obstacles: Option<bool>,
    enable_simple_routes: Option<bool>,
    history_weight: Option<f64>,
    long_straight_weight_override: Option<f64>,
    router_config: &RouterConfig,
) -> PyResult<AStarConfig> {
    let allowed_target_angles_mask =
        allowed_angles_to_mask(astar_cfg.allowed_target_angles.as_ref())?;
    let primitive_ordering = parse_primitive_ordering(&astar_cfg.primitive_ordering)?;
    let heuristic_mode = parse_heuristic_mode(&astar_cfg.heuristic_mode)?;
    let heap_tie_breaker = parse_heap_tie_breaker(&astar_cfg.heap_tie_breaker)?;
    let max_search_time_ms = router_config
        .search
        .astar_timeout_ms
        .unwrap_or(astar_cfg.max_search_time_ms);
    let long_straight_congestion_weight = match long_straight_weight_override {
        Some(weight) => weight,
        None => router_config
            .search
            .long_straight_congestion_weight
            .unwrap_or(0.0),
    };
    if !astar_cfg.proactive_congestion_weight.is_finite()
        || astar_cfg.proactive_congestion_weight < 0.0
    {
        return Err(PyValueError::new_err(
            "proactive_congestion_weight must be finite and non-negative",
        ));
    }
    if astar_cfg.proactive_congestion_radius_cells < 0 {
        return Err(PyValueError::new_err(
            "proactive_congestion_radius_cells must be non-negative",
        ));
    }
    Ok(AStarConfig {
        diagnostics: router_config.diagnostics.clone(),
        max_iterations: astar_cfg.max_iterations,
        bend_weight: astar_cfg.bend_weight * primitive_cfg.bend_weight,
        target_tolerance_cells: astar_cfg.target_tolerance_cells,
        require_target_angle: astar_cfg.require_target_angle,
        allowed_target_angles_mask,
        use_routing_window: astar_cfg.use_routing_window,
        routing_window_min_margin_cells: astar_cfg.routing_window_min_margin_cells,
        routing_window_scale: astar_cfg.routing_window_scale,
        routing_window_max_expansions: astar_cfg.routing_window_max_expansions,
        routing_window_fallback_full_grid: astar_cfg.routing_window_fallback_full_grid,
        routing_window_growth: astar_cfg.routing_window_growth,
        max_dense_states: router_config
            .search
            .max_dense_states
            .filter(|n| *n > 0)
            .unwrap_or_else(|| AStarConfig::default().max_dense_states),
        max_dense_obstacle_cells: astar_cfg.max_dense_obstacle_cells,
        enable_simple_routes: enable_simple_routes.unwrap_or(astar_cfg.enable_simple_routes),
        simple_route_max_offset_cells: astar_cfg.simple_route_max_offset_cells,
        simple_route_min_leg_len_cells: astar_cfg.simple_route_min_leg_len_cells,
        ignore_dynamic_obstacles: ignore_dynamic_obstacles
            .unwrap_or(astar_cfg.ignore_dynamic_obstacles),
        history_weight: history_weight.unwrap_or(astar_cfg.history_weight),
        long_straight_congestion_weight,
        proactive_congestion_weight: astar_cfg.proactive_congestion_weight,
        proactive_congestion_radius_cells: astar_cfg.proactive_congestion_radius_cells,
        collect_detailed_timing: astar_cfg.collect_detailed_timing,
        enable_jps4: astar_cfg.enable_jps4,
        use_indexed_heap: astar_cfg.use_indexed_heap,
        primitive_ordering,
        heuristic_mode,
        heuristic_weight: astar_cfg.heuristic_weight,
        heap_tie_breaker,
        require_terminal_straights: false,
        max_search_time_ms,
        // Never set from Python: only
        // `route_many_with_negotiated_repair_and_commit` ever wants a
        // fail-fast budget, and it applies one via
        // `PyPhotonicRouter::negotiated_search_budget`/`astar_config`, not
        // through this conversion. See Milestone 5 of
        // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`.
        total_expansion_budget: None,
    })
}

/// Milestone 2 of
/// .agent/execplans/2026-08-19-restructure-crossing-partner-discovery.md:
/// names the one real behavioral difference Milestone 0's divergence audit
/// found between `route_single_net_and_commit_native` and
/// `route_single_net_and_commit_repair_native` -- both confirmed
/// intentional by the repository owner and required to stay different, but
/// previously implicit in each function's own, independently written
/// control flow rather than named or documented anywhere. See
/// `PyPhotonicRouter::upfront_collision_crossing_partner_ids`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CollisionCrossingTryOrder {
    /// Try the plain, non-crossing A* attempt first; a lidar-pure
    /// collision-crossing search (the full, unwindowed partner set) is
    /// only attempted afterward, if that plain attempt fails. Used by
    /// fresh/native routing, where crossings are the exception, not the
    /// default outcome, so the cheaper non-crossing case should be tried
    /// first.
    PlainFirst,
    /// Consult collision-crossing partners immediately, before any plain
    /// fallback. Used by repair, where the very fact that repair is
    /// running already means normal routing already failed once for this
    /// net, making congestion (and therefore a needed crossing) more
    /// likely from the start.
    CrossingFirst,
}

impl PyPhotonicRouter {
    pub(crate) fn astar_config(
        &self,
        ignore_dynamic_obstacles: Option<bool>,
        enable_simple_routes: Option<bool>,
        history_weight: Option<f64>,
    ) -> Result<AStarConfig, String> {
        let mut cfg = self
            .astar_cfg_cached
            .as_ref()
            .map_err(|err| err.clone())?
            .clone();
        if let Some(ignore_dynamic_obstacles) = ignore_dynamic_obstacles {
            cfg.ignore_dynamic_obstacles = ignore_dynamic_obstacles;
        }
        if let Some(enable_simple_routes) = enable_simple_routes {
            cfg.enable_simple_routes = enable_simple_routes;
        }
        if let Some(history_weight) = history_weight {
            cfg.history_weight = history_weight;
        }
        if let Some(budget) = self.negotiated_search_budget {
            cfg.total_expansion_budget = Some(budget);
        }
        if let Some(weight) = self.long_straight_weight_override {
            cfg.long_straight_congestion_weight = weight;
        }
        Ok(cfg)
    }

    /// Builds the per-search crossing config. Contribution 1 (crossing-guided
    /// search) enters here and only here: when guidance is set, every partner
    /// that forms a planned pair with `net_id` gets the planned crossing
    /// price as its `crossing_loss_override`; without guidance (lidar-pure)
    /// no partner carries an override and pricing is the baseline's.
    pub(crate) fn crossing_search_config(
        &self,
        net_id: u64,
        mut partners: Vec<CrossingSearchPartner>,
        crossing_cfg: &CrossingConfig,
        target: State,
        target_port_um: Option<(f64, f64)>,
        crossing_loss_override: Option<f64>,
        require_all_partners_override: Option<bool>,
    ) -> CrossingSearchConfig {
        if let Some(guidance) = self.crossing_context.guidance() {
            for partner in &mut partners {
                if guidance.is_planned_pair(net_id, partner.net_id) {
                    partner.crossing_loss_override = Some(guidance.planned_crossing_loss);
                    partner.single_discounted_crossing =
                        guidance.single_discounted_crossing_per_pair;
                }
            }
        }
        CrossingSearchConfig {
            diagnostics: self.router_config.diagnostics.clone(),
            net_id,
            partners,
            min_straight_cells: crossing_cfg.min_straight_cells_per_crossing,
            crossing_half_size_cells: crossing_cfg.crossing_half_size_cells,
            bend_runout_cells: self.primitive_cfg.bend_radius_cells,
            crossing_loss: crossing_loss_override.unwrap_or(crossing_cfg.crossing_loss),
            require_all_partners: require_all_partners_override
                .unwrap_or(crossing_cfg.allow_only_expected_pairs),
            terminal_bump_guard: self.terminal_bump_guard_for_target(target, target_port_um),
        }
    }

    /// The temporary guidance for one probe-guided search
    /// (`route_many_with_negotiated_repair_and_commit`'s probe-guided step):
    /// the union of the existing guidance's planned pairs, if any (the
    /// topology plan, under contribution 1's lidar-guided mode; empty under
    /// lidar-pure), with `(net_id, p)` for every `p` in
    /// `probe_partner_ids` (the probe's own free-path crossing partners).
    /// `CrossingGuidance` carries a single loss for every pair it holds, so
    /// this temporary guidance also re-prices the plan's own pairs at
    /// `min(existing_loss, NEGOTIATED_PROBE_GUIDANCE_LOSS)` for the
    /// duration of this one search -- a no-op under contribution 1's
    /// default (the plan's pairs are already priced at 0).
    pub(crate) fn temporary_probe_guided_guidance(
        &self,
        net_id: u64,
        probe_partner_ids: &[u64],
    ) -> CrossingGuidance {
        let existing = self.crossing_context.guidance();
        let mut pairs: Vec<(u64, u64)> = existing.map(CrossingGuidance::pairs).unwrap_or_default();
        pairs.extend(
            probe_partner_ids
                .iter()
                .map(|&partner_id| (net_id, partner_id)),
        );
        let loss = existing
            .map(|guidance| {
                guidance
                    .planned_crossing_loss
                    .min(NEGOTIATED_PROBE_GUIDANCE_LOSS)
            })
            .unwrap_or(NEGOTIATED_PROBE_GUIDANCE_LOSS);
        let single_discounted = existing
            .map(|guidance| guidance.single_discounted_crossing_per_pair)
            .unwrap_or(true);
        CrossingGuidance::new(&pairs, loss)
            .with_single_discounted_crossing_per_pair(single_discounted)
    }

    /// Runs `f` with `temporary_probe_guided_guidance(net_id,
    /// probe_partner_ids)` installed as `crossing_context`'s guidance,
    /// restoring whatever guidance existed before (or clearing it, if none
    /// did) once `f` returns -- on every path, since `f` returns a plain
    /// value rather than propagating `?`, so there is no early return for
    /// the restore to miss.
    pub(crate) fn with_probe_guided_guidance<T>(
        &mut self,
        net_id: u64,
        probe_partner_ids: &[u64],
        f: impl FnOnce(&mut Self) -> T,
    ) -> T {
        let saved = self.crossing_context.guidance().cloned();
        let temporary = self.temporary_probe_guided_guidance(net_id, probe_partner_ids);
        self.crossing_context.set_guidance(temporary);
        let result = f(self);
        match saved {
            Some(guidance) => self.crossing_context.set_guidance(guidance),
            None => self.crossing_context.clear_guidance(),
        }
        result
    }

    #[allow(clippy::too_many_arguments)]
    /// Top of a 3-level default-argument chain
    /// (`try_route_with_collision_crossings` ->
    /// [`Self::try_route_with_collision_crossings_with_loss`] ->
    /// [`Self::try_route_with_collision_crossings_using_primitives`]) --
    /// a shared low-level lidar-pure collision-crossing search primitive,
    /// not an independent repair strategy in its own right.
    pub(crate) fn try_route_with_collision_crossings(
        &self,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        accept_clean_zero_event: bool,
    ) -> Result<Option<(RouteResult, Vec<CrossingEvent>)>, String> {
        self.try_route_with_collision_crossings_with_loss(
            net_id,
            source,
            target,
            opened_ref,
            search_cfg,
            block_radius_cells,
            dynamic_clearance_exempt_keys,
            partner_ids,
            source_port_um,
            target_port_um,
            opened_cell_keys,
            None,
            accept_clean_zero_event,
        )
    }

    #[allow(clippy::too_many_arguments)]
    /// Middle of the 3-level collision-crossing search chain (see
    /// [`Self::try_route_with_collision_crossings`]); adds loss/config
    /// resolution before delegating to
    /// [`Self::try_route_with_collision_crossings_using_primitives`].
    pub(crate) fn try_route_with_collision_crossings_with_loss(
        &self,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        crossing_loss_override: Option<f64>,
        accept_clean_zero_event: bool,
    ) -> Result<Option<(RouteResult, Vec<CrossingEvent>)>, String> {
        self.try_route_with_collision_crossings_using_primitives(
            &self.primitives,
            net_id,
            source,
            target,
            opened_ref,
            search_cfg,
            block_radius_cells,
            dynamic_clearance_exempt_keys,
            partner_ids,
            source_port_um,
            target_port_um,
            opened_cell_keys,
            crossing_loss_override,
            accept_clean_zero_event,
        )
    }

    #[allow(clippy::too_many_arguments)]
    /// Bottom of the 3-level collision-crossing search chain (see
    /// [`Self::try_route_with_collision_crossings`]) -- the actual A* call
    /// site for lidar-pure collision-crossing routing.
    pub(crate) fn try_route_with_collision_crossings_using_primitives(
        &self,
        primitives: &PrimitiveLibrary,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        crossing_loss_override: Option<f64>,
        accept_clean_zero_event: bool,
    ) -> Result<Option<(RouteResult, Vec<CrossingEvent>)>, String> {
        if !self.crossing_context.is_enabled() || partner_ids.is_empty() {
            return Ok(None);
        }
        self.clear_pending_straight_victim_hint(net_id);
        let crossing_cfg = self.crossing_context.config();
        let crossing_partners: Vec<CrossingSearchPartner> =
            if crossing_cfg.allow_only_expected_pairs {
                self.crossing_context
                    .ordered_constraints_for(net_id)
                    .into_iter()
                    .filter_map(|constraint| {
                        let partner_id = if constraint.net_id == net_id {
                            constraint.partner_net_id
                        } else {
                            constraint.net_id
                        };
                        if !partner_ids.contains(&partner_id) {
                            return None;
                        }
                        self.committed_center_routes
                            .get(&partner_id)
                            .map(|waypoints| CrossingSearchPartner {
                                net_id: partner_id,
                                waypoints: waypoints.clone(),
                                target_terminal_bump_guard: self
                                    .committed_target_terminal_bump_guards
                                    .get(&partner_id)
                                    .copied(),
                                crossing_loss_override: None,
                                single_discounted_crossing: false,
                            })
                    })
                    .collect()
            } else {
                partner_ids
                    .iter()
                    .filter_map(|partner_id| {
                        self.committed_center_routes
                            .get(partner_id)
                            .map(|waypoints| CrossingSearchPartner {
                                net_id: *partner_id,
                                waypoints: waypoints.clone(),
                                target_terminal_bump_guard: self
                                    .committed_target_terminal_bump_guards
                                    .get(partner_id)
                                    .copied(),
                                crossing_loss_override: None,
                                single_discounted_crossing: false,
                            })
                    })
                    .collect()
            };
        if crossing_partners.is_empty() {
            return Ok(None);
        }
        let crossing_search = self.crossing_search_config(
            net_id,
            crossing_partners,
            crossing_cfg,
            target,
            target_port_um,
            crossing_loss_override,
            None,
        );
        let search_partner_ids: FxHashSet<u64> = crossing_search
            .partners
            .iter()
            .map(|partner| partner.net_id)
            .collect();
        let mut crossing_search_cfg = search_cfg.clone();
        crossing_search_cfg.enable_simple_routes = false;
        crossing_search_cfg.enable_jps4 = false;
        crossing_search_cfg.routing_window_fallback_full_grid = false;
        let trace_crossing = self
            .router_config
            .diagnostics
            .trace_crossing_net
            .map_or_else(
                || self.router_config.diagnostics.trace_crossing,
                |trace_net_id| trace_net_id == net_id,
            );
        if trace_crossing {
            let mut lookup_ids: Vec<u64> = partner_ids.iter().copied().collect();
            lookup_ids.sort_unstable();
            let mut center_route_ids: Vec<u64> =
                self.committed_center_routes.keys().copied().collect();
            center_route_ids.sort_unstable();
            eprintln!(
                "collision-crossing start net={} partners={:?} block_radius={} min_straight={} half_size={} lookup_partner_ids={:?} committed_center_routes={:?} net_route_entries={}",
                net_id,
                crossing_search
                    .partners
                    .iter()
                    .map(|partner| partner.net_id)
                    .collect::<Vec<_>>(),
                block_radius_cells,
                crossing_search.min_straight_cells,
                crossing_search.crossing_half_size_cells,
                lookup_ids,
                center_route_ids,
                self.obstacle_map.net_route_entries().count(),
            );
            for partner in &crossing_search.partners {
                self.trace_committed_partner_centerline_compare(net_id, partner.net_id);
            }
        }
        let env = SearchEnvironment {
            obstacle_map: &self.obstacle_map,
            primitives,
        };
        let request = SearchRequest {
            source,
            target,
            port_open_cells: Some(opened_ref),
            dynamic_expansion: Some(DynamicExpansion {
                radius_cells: block_radius_cells.max(0),
                clearance_exempt_cells: dynamic_clearance_exempt_keys,
            }),
            crossing: Some(CrossingSearch {
                config: &crossing_search,
                reservation_open_cells: opened_cell_keys,
            }),
            config: &crossing_search_cfg,
        };
        let outcome = self.search_engine.search(&env, &request);
        let (route_result, failed_stats) = (outcome.route, outcome.stats);
        let Some(result) = route_result else {
            self.remember_pending_straight_victim_hint(net_id, &failed_stats);
            return Ok(None);
        };
        if trace_crossing {
            eprintln!(
                "collision-crossing result net={} expanded={} generated={} accepted={} candidates={} cost={} waypoints={:?}",
                net_id,
                result.stats.expanded_states,
                result.stats.generated_neighbors,
                result.stats.crossing_accepted,
                result.stats.crossing_candidate_checks,
                result.total_cost,
                result.compressed_waypoints,
            );
        }
        let crossing_events = self.realized_crossing_events_for_route(
            net_id,
            &result,
            partner_ids,
            source_port_um,
            target_port_um,
        );
        if trace_crossing {
            eprintln!(
                "collision-crossing events net={} events={:?}",
                net_id,
                crossing_events
                    .iter()
                    .map(|event| (
                        event.partner_net_id,
                        event.point,
                        event.route_angle,
                        event.partner_angle
                    ))
                    .collect::<Vec<_>>(),
            );
        }
        let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
        let required_partner_ids = if crossing_cfg.allow_only_expected_pairs {
            partner_ids
        } else {
            &crossed_partner_ids
        };
        let skip_rust_level2_validation =
            self.router_config.crossing.disable_rust_crossing_validation;
        let route_has_no_unresolved_grid_crossings = skip_rust_level2_validation
            || self
                .invalid_crossing_intersections_for_route(net_id, &result, &search_partner_ids)
                .is_empty();
        // This function's whole purpose is to find a route that achieves a
        // crossing with at least one of `partner_ids` -- a route with zero
        // crossing events never engaged with any partner at all, so it must
        // never be accepted here, regardless of what `required_partner_ids`
        // evaluates to. Without this, the `!crossing_cfg.allow_only_expected_pairs`
        // branch above derives `required_partner_ids` from `crossing_events`
        // itself (`crossed_partner_ids`), so zero events yields an empty
        // required set, which `crossing_events_satisfy_partner_constraints`'s
        // own `partner_ids.is_empty() => true` case (itself correct and
        // needed by an unrelated repair-probe caller, `src/py_router.rs`
        // around line 9316, where "no partners required" legitimately means
        // "trivially compliant") then vacuously satisfies -- exactly the bug
        // traced to commit 9e0a927 and pinned by the regression test
        // `collision_crossing_route_without_event_is_not_accepted`.
        let satisfies = !crossing_events.is_empty()
            && route_has_no_unresolved_grid_crossings
            && self.crossing_events_satisfy_partner_constraints(
                net_id,
                required_partner_ids,
                &crossing_events,
                opened_cell_keys,
            );
        let realized_violations = if skip_rust_level2_validation {
            Vec::new()
        } else {
            // Pre-commit search primitive: `result` has not been
            // committed, so it cannot have registered crossing events yet.
            self.crossing_violations_for_route_with_ports(
                net_id,
                &result,
                source_port_um,
                target_port_um,
                opened_cell_keys,
                false,
            )
        };
        self.dump_crossing_mismatch(
            net_id,
            &result,
            source_port_um,
            target_port_um,
            &realized_violations,
        );
        if trace_crossing {
            let unresolved = if skip_rust_level2_validation {
                Vec::new()
            } else {
                self.invalid_crossing_intersections_for_route(net_id, &result, &search_partner_ids)
            };
            eprintln!(
                "collision-crossing validation net={} crossed={:?} satisfies={} no_unresolved_grid_crossings={} partner_constraints={} unresolved={:?} realized_violations={:?}",
                net_id,
                crossed_partner_ids,
                satisfies,
                route_has_no_unresolved_grid_crossings,
                self.crossing_events_satisfy_partner_constraints(
                    net_id,
                    required_partner_ids,
                    &crossing_events,
                    opened_cell_keys,
                ),
                unresolved
                    .iter()
                    .map(|v| (v.partner_net_id, v.point, v.reason))
                    .collect::<Vec<_>>(),
                realized_violations
                    .iter()
                    .map(|violation| (violation.partner_net_id, violation.point, violation.reason))
                    .collect::<Vec<_>>(),
            );
            // Sub-conditions of `crossing_events_satisfy_partner_constraints`,
            // so a post-search reject names its rule (harness step 5).
            let blockers =
                self.crossing_reservation_blockers(net_id, &crossing_events, opened_cell_keys);
            let overlapping =
                Self::crossing_partners_with_overlapping_reservations(&crossing_events);
            let mut static_blocker_cells: Vec<(i32, i32)> = Vec::new();
            let mut dynamic_blocker_cells: Vec<((i32, i32), u64)> = Vec::new();
            for event in &crossing_events {
                for key in &event.reservation_keys {
                    let (x, y) = unpack_xy(*key);
                    let is_opened = opened_cell_keys.is_some_and(|opened| opened.contains(key));
                    if self.obstacle_map.in_bounds(x, y)
                        && self.obstacle_map.is_static_blocked(x, y)
                        && !is_opened
                    {
                        static_blocker_cells.push((x, y));
                    }
                    for owner in self.obstacle_map.dynamic_owners_for_cells(&[(x, y)]) {
                        if owner != net_id && owner != event.partner_net_id {
                            dynamic_blocker_cells.push(((x, y), owner));
                        }
                    }
                }
            }
            static_blocker_cells.sort_unstable();
            static_blocker_cells.dedup();
            dynamic_blocker_cells.sort_unstable();
            dynamic_blocker_cells.dedup();
            eprintln!(
                "collision-crossing partner-constraints net={} events={} disjoint_reservations={} overlapping_partners={:?} static_blocker={} static_cells={:?} dynamic_blockers={:?} dynamic_cells={:?} events_by_partner={:?}",
                net_id,
                crossing_events.len(),
                Self::crossing_events_have_disjoint_reservations(&crossing_events),
                overlapping,
                blockers.has_static_blocker,
                static_blocker_cells,
                blockers.dynamic_blockers,
                dynamic_blocker_cells,
                crossing_events
                    .iter()
                    .map(|event| (event.partner_net_id, event.point))
                    .collect::<Vec<_>>(),
            );
        }
        if satisfies && realized_violations.is_empty() {
            return Ok(Some((result, crossing_events)));
        }
        // Owner decision 2026-09-02: on the MAIN routing path (not the
        // partner-set/victim strategies, whose purpose is to engage specific
        // partners), a zero-event result that is clean at BOTH validation
        // levels is simply a valid ordinary route -- the search discovered
        // that no crossing is needed. Discarding it (the strict zero-event
        // rule) short-circuited plain A* and sent dense-ramp fan-in nets
        // (multiportmmi_32x32 o24 class, the clean detour found and thrown
        // away six times per cascade) into an unwinnable repair loop. The
        // strict rule stays the default; callers opt in explicitly.
        if accept_clean_zero_event
            && crossing_events.is_empty()
            && route_has_no_unresolved_grid_crossings
            && realized_violations.is_empty()
        {
            return Ok(Some((result, crossing_events)));
        }
        if !crossing_events.is_empty() && satisfies && !realized_violations.is_empty() {
            return Err(Self::format_realized_crossing_violation_error(
                net_id,
                &realized_violations,
            ));
        }
        self.remember_pending_straight_victim_hint(net_id, &result.stats);
        Ok(None)
    }

    #[allow(clippy::too_many_arguments)]
    /// Shared low-level search primitive: routes through a caller-supplied
    /// set of collision-crossing partner nets. Used by
    /// [`Self::try_guided_collision_crossing`] (guided/2-partner case) and
    /// by [`Self::try_crossing_aware_victim_reroute`]'s "seeded" and
    /// "guided" per-victim attempts (see that method's own doc comment) --
    /// not an independent top-level repair strategy itself.
    pub(crate) fn try_route_through_collision_partner_set(
        &self,
        net_id: u64,
        source: State,
        target: State,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
        partner_ids: &FxHashSet<u64>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Result<Option<(RouteResult, Vec<CrossingEvent>)>, String> {
        if !self.crossing_context.is_enabled() || partner_ids.is_empty() {
            return Ok(None);
        }
        let crossing_cfg = self.crossing_context.config();
        let crossing_partners: Vec<CrossingSearchPartner> = partner_ids
            .iter()
            .filter_map(|partner_id| {
                self.committed_center_routes
                    .get(partner_id)
                    .map(|waypoints| CrossingSearchPartner {
                        net_id: *partner_id,
                        waypoints: waypoints.clone(),
                        target_terminal_bump_guard: self
                            .committed_target_terminal_bump_guards
                            .get(partner_id)
                            .copied(),
                        crossing_loss_override: None,
                        single_discounted_crossing: false,
                    })
            })
            .collect();
        if crossing_partners.is_empty() {
            return Ok(None);
        }
        let crossing_search = self.crossing_search_config(
            net_id,
            crossing_partners,
            crossing_cfg,
            target,
            target_port_um,
            None,
            Some(true),
        );
        let mut crossing_search_cfg = search_cfg.clone();
        crossing_search_cfg.require_terminal_straights = false;
        crossing_search_cfg.enable_simple_routes = false;
        crossing_search_cfg.enable_jps4 = false;
        // This search requires the route to cross *every* partner in
        // `partner_ids` simultaneously (`require_all_partners=true` below).
        // When that joint constraint is infeasible, A* has no early-exit
        // signal and must exhaust the search space before concluding
        // failure -- confirmed via direct measurement on multiportmmi_8x8
        // (net 32 repair, 2026-08-26): two such failed searches took 47.9s
        // and 36.6s each against the uncapped default (5,000,000), while
        // every real success this call has ever produced completed in
        // well under 200ms (expanded well under 30,000 states). Capped 10x
        // lower as a circuit breaker for the infeasible case -- still ~15x
        // more headroom than any observed real success.
        crossing_search_cfg.max_iterations = crossing_search_cfg.max_iterations.min(500_000);
        let trace_crossing = self
            .router_config
            .diagnostics
            .trace_crossing_net
            .map_or_else(
                || self.router_config.diagnostics.trace_crossing,
                |trace_net_id| trace_net_id == net_id,
            );
        if trace_crossing {
            eprintln!(
                "guided-collision-crossing start net={} partners={:?} max_iterations={} block_radius={} min_straight={} half_size={}",
                net_id,
                crossing_search
                    .partners
                    .iter()
                    .map(|partner| partner.net_id)
                    .collect::<Vec<_>>(),
                crossing_search_cfg.max_iterations,
                block_radius_cells,
                crossing_search.min_straight_cells,
                crossing_search.crossing_half_size_cells,
            );
            for partner in &crossing_search.partners {
                self.trace_committed_partner_centerline_compare(net_id, partner.net_id);
            }
        }
        let mut search_map = self.obstacle_map.clone();
        search_map.clear_dynamic_blocking_for_nets(partner_ids);
        let partner_vec: Vec<u64> = partner_ids.iter().copied().collect();
        let mut guided_keepout = FxHashSet::default();
        const MAX_GUIDED_CROSSING_VALIDATION_RETRIES: usize = 4;
        for retry_idx in 0..=MAX_GUIDED_CROSSING_VALIDATION_RETRIES {
            let env = SearchEnvironment {
                obstacle_map: &search_map,
                primitives: &self.primitives,
            };
            let request = SearchRequest {
                source,
                target,
                port_open_cells: Some(opened_ref),
                dynamic_expansion: Some(DynamicExpansion {
                    radius_cells: block_radius_cells.max(0),
                    clearance_exempt_cells: dynamic_clearance_exempt_keys,
                }),
                crossing: Some(CrossingSearch {
                    config: &crossing_search,
                    reservation_open_cells: None,
                }),
                config: &crossing_search_cfg,
            };
            let Some(result) = self.search_engine.search(&env, &request).route else {
                return Ok(None);
            };
            let crossing_events = self.realized_crossing_events_for_route(
                net_id,
                &result,
                partner_ids,
                source_port_um,
                target_port_um,
            );
            let satisfies = self.crossing_route_satisfies_partner_constraints(
                net_id,
                &result,
                partner_ids,
                &crossing_events,
                opened_cell_keys,
            );
            // Pre-commit search primitive (guided-collision retry loop):
            // `result` has not been committed, so it cannot have registered
            // crossing events yet.
            let realized_violations = self.crossing_violations_for_route_with_ports(
                net_id,
                &result,
                source_port_um,
                target_port_um,
                opened_cell_keys,
                false,
            );
            self.dump_crossing_mismatch(
                net_id,
                &result,
                source_port_um,
                target_port_um,
                &realized_violations,
            );
            let covers_requested_partners =
                Self::crossing_events_cover_partners(&crossing_events, partner_ids);
            if trace_crossing {
                eprintln!(
                    "guided-collision-crossing result net={} retry={} expanded={} generated={} events={} satisfies={} covers_partners={} realized_violations={:?} cost={} waypoints={:?}",
                    net_id,
                    retry_idx,
                    result.stats.expanded_states,
                    result.stats.generated_neighbors,
                    crossing_events.len(),
                    satisfies,
                    covers_requested_partners,
                    realized_violations
                        .iter()
                        .map(|violation| (violation.partner_net_id, violation.point, violation.reason))
                        .collect::<Vec<_>>(),
                    result.total_cost,
                    result.compressed_waypoints,
                );
            }
            if !crossing_events.is_empty()
                && satisfies
                && covers_requested_partners
                && realized_violations.is_empty()
            {
                return Ok(Some((result, crossing_events)));
            }
            if !crossing_events.is_empty()
                && satisfies
                && covers_requested_partners
                && !realized_violations.is_empty()
            {
                return Err(Self::format_realized_crossing_violation_error(
                    net_id,
                    &realized_violations,
                ));
            }

            if retry_idx == MAX_GUIDED_CROSSING_VALIDATION_RETRIES {
                break;
            }
            let retry_keepout = self.crossing_physical_violation_repair_keepout_keys(
                &realized_violations,
                &partner_vec,
            );
            let new_keepout: FxHashSet<CellKey> = retry_keepout
                .into_iter()
                .filter(|key| guided_keepout.insert(*key))
                .collect();
            if new_keepout.is_empty() {
                break;
            }
            let added = search_map.add_static_keys(&new_keepout);
            if trace_crossing {
                eprintln!(
                    "guided-collision-crossing retry-keepout net={} retry={} keys={} added={}",
                    net_id,
                    retry_idx + 1,
                    new_keepout.len(),
                    added,
                );
            }
        }
        Ok(None)
    }

    #[allow(clippy::too_many_arguments)]
    /// Shared low-level search primitive for the "window" (topology
    /// expected-partner) crossing mode -- the sibling of
    /// [`Self::try_route_through_collision_partner_set`] for nets whose
    /// crossing partners come from `crossing_allowed_partner_set` rather
    /// than lidar-pure collision detection. Not part of the
    /// `route_many_with_repair_and_commit` main loop's own call chain;
    /// called from the native route/repair entry points that resolve
    /// crossing mode before dispatching into repair.
    // candidate for removal, see Milestone 8 of .agent/execplans/2026-09-22-modular-readable-router-restructure.md
    pub(crate) fn try_route_through_expected_crossing_partner(
        &self,
        net_id: u64,
        source: State,
        target: State,
        target_port_um: Option<(f64, f64)>,
        opened_ref: &FxHashSet<CellKey>,
        search_cfg: &AStarConfig,
        block_radius_cells: i32,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
    ) -> Option<(RouteResult, Vec<CrossingEvent>)> {
        let partner_ids = self.crossing_allowed_partner_set(net_id);
        if partner_ids.is_empty() {
            return None;
        }
        let require_all_expected_partners =
            self.crossing_context.config().allow_only_expected_pairs;

        let mut crossing_search_cfg = search_cfg.clone();
        crossing_search_cfg.require_terminal_straights = false;
        crossing_search_cfg.enable_simple_routes = false;
        if crossing_search_cfg.history_weight > 0.0 {
            crossing_search_cfg.routing_window_max_expansions =
                crossing_search_cfg.routing_window_max_expansions.min(1);
            crossing_search_cfg.routing_window_fallback_full_grid = false;
        }
        let crossing_cfg = self.crossing_context.config();
        let crossing_partners: Vec<CrossingSearchPartner> = self
            .crossing_context
            .ordered_constraints_for(net_id)
            .into_iter()
            .filter_map(|constraint| {
                let partner_id = if constraint.net_id == net_id {
                    constraint.partner_net_id
                } else {
                    constraint.net_id
                };
                if !partner_ids.contains(&partner_id) {
                    return None;
                }
                self.committed_center_routes
                    .get(&partner_id)
                    .map(|waypoints| CrossingSearchPartner {
                        net_id: partner_id,
                        waypoints: waypoints.clone(),
                        target_terminal_bump_guard: self
                            .committed_target_terminal_bump_guards
                            .get(&partner_id)
                            .copied(),
                        crossing_loss_override: None,
                        single_discounted_crossing: false,
                    })
            })
            .collect();
        if crossing_partners.is_empty() {
            return None;
        }
        let crossing_search = self.crossing_search_config(
            net_id,
            crossing_partners,
            crossing_cfg,
            target,
            target_port_um,
            None,
            Some(require_all_expected_partners),
        );
        let trace_crossing = self
            .router_config
            .diagnostics
            .trace_crossing_net
            .map_or_else(
                || self.router_config.diagnostics.trace_crossing,
                |trace_net_id| trace_net_id == net_id,
            );
        if trace_crossing {
            eprintln!(
                "crossing-search start net={} partners={:?} max_iterations={} block_radius={} min_straight={} half_size={}",
                net_id,
                crossing_search
                    .partners
                    .iter()
                    .map(|partner| partner.net_id)
                    .collect::<Vec<_>>(),
                crossing_search_cfg.max_iterations,
                block_radius_cells,
                crossing_search.min_straight_cells,
                crossing_search.crossing_half_size_cells,
            );
        }

        let crossing_candidate_keys = self.crossing_candidate_keys_for_partners(&partner_ids);
        if !crossing_candidate_keys.is_empty() {
            let mut search_map = self.obstacle_map.clone();
            search_map
                .clear_dynamic_blocking_in_cells_for_nets(&crossing_candidate_keys, &partner_ids);
            if trace_crossing {
                eprintln!(
                    "crossing-search candidate-phase net={} candidate_keys={}",
                    net_id,
                    crossing_candidate_keys.len()
                );
            }
            let env = SearchEnvironment {
                obstacle_map: &search_map,
                primitives: &self.primitives,
            };
            let request = SearchRequest {
                source,
                target,
                port_open_cells: Some(opened_ref),
                dynamic_expansion: Some(DynamicExpansion {
                    radius_cells: block_radius_cells.max(0),
                    clearance_exempt_cells: dynamic_clearance_exempt_keys,
                }),
                crossing: Some(CrossingSearch {
                    config: &crossing_search,
                    reservation_open_cells: None,
                }),
                config: &crossing_search_cfg,
            };
            if let Some(result) = self.search_engine.search(&env, &request).route {
                if trace_crossing {
                    eprintln!(
                        "crossing-search candidate-result net={} expanded={} generated={} heap={} events={}",
                        net_id,
                        result.stats.expanded_states,
                        result.stats.generated_neighbors,
                        result.stats.max_heap_size,
                        self.crossing_events_for_route(net_id, &result, &partner_ids).len(),
                    );
                }
                let crossing_events = self.crossing_events_for_route(net_id, &result, &partner_ids);
                if self.crossing_route_satisfies_partner_constraints(
                    net_id,
                    &result,
                    &partner_ids,
                    &crossing_events,
                    Some(opened_ref),
                ) {
                    return Some((result, crossing_events));
                }
            }
        }

        let mut search_map = self.obstacle_map.clone();
        search_map.clear_dynamic_blocking_for_nets(&partner_ids);
        if trace_crossing {
            eprintln!("crossing-search broad-phase net={}", net_id);
        }
        let env = SearchEnvironment {
            obstacle_map: &search_map,
            primitives: &self.primitives,
        };
        let request = SearchRequest {
            source,
            target,
            port_open_cells: Some(opened_ref),
            dynamic_expansion: Some(DynamicExpansion {
                radius_cells: block_radius_cells.max(0),
                clearance_exempt_cells: dynamic_clearance_exempt_keys,
            }),
            crossing: Some(CrossingSearch {
                config: &crossing_search,
                reservation_open_cells: None,
            }),
            config: &crossing_search_cfg,
        };
        let result = self.search_engine.search(&env, &request).route?;
        if trace_crossing {
            eprintln!(
                "crossing-search broad-result net={} expanded={} generated={} heap={} events={}",
                net_id,
                result.stats.expanded_states,
                result.stats.generated_neighbors,
                result.stats.max_heap_size,
                self.crossing_events_for_route(net_id, &result, &partner_ids)
                    .len(),
            );
        }
        let crossing_events = self.crossing_events_for_route(net_id, &result, &partner_ids);
        if self.crossing_route_satisfies_partner_constraints(
            net_id,
            &result,
            &partner_ids,
            &crossing_events,
            Some(opened_ref),
        ) {
            return Some((result, crossing_events));
        }
        None
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_single_net_and_commit_native(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: Option<&[(i32, i32)]>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<&[(i32, i32)]>,
        clearance_exempt_cell_keys: Option<&FxHashSet<CellKey>>,
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err("target_tolerance_cells must be >= 0".to_string());
        }
        let opened_owned;
        let opened_default_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(keys) = opened_cell_keys {
            keys
        } else if let Some(cells) = opened_cells {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            opened_default_owned = self.port_open_cells.clone();
            &opened_default_owned
        };
        let validation_opened_cell_keys = opened_ref.clone();
        // `commit_history_weight` is 0.0 (a no-op override, matching
        // `AStarConfig`'s own baseline -- this codebase never constructs a
        // router with a nonzero baseline `history_weight`) outside
        // `route_many_with_negotiated_repair_and_commit`, so this is
        // behaviorally neutral for every other caller of this function. See
        // `.agent/execplans/2026-08-25-negotiated-repair-engine.md`
        // Milestone 6.
        let mut cfg = self.astar_config(None, None, Some(self.commit_history_weight))?;
        cfg.require_terminal_straights = true;
        let dynamic_clearance_exempt_cell_vec = clearance_exempt_cells.unwrap_or(&[]);
        let collect_timing = self.astar_cfg.collect_detailed_timing;
        let mut obstacle_map_prepare_time_us = 0u128;
        let mut simple_route_time_us = 0u128;
        let mut commit_prepare_time_us = 0u128;
        let mut commit_time_us = 0u128;
        let prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let dynamic_clearance_exempt_keys_owned: Option<FxHashSet<CellKey>> = if block_radius_cells
            > 0
            && !dynamic_clearance_exempt_cell_vec.is_empty()
            && clearance_exempt_cell_keys.is_none()
        {
            Some(pack_cells(dynamic_clearance_exempt_cell_vec))
        } else {
            None
        };
        let dynamic_clearance_exempt_keys =
            if block_radius_cells > 0 && !dynamic_clearance_exempt_cell_vec.is_empty() {
                clearance_exempt_cell_keys.or(dynamic_clearance_exempt_keys_owned.as_ref())
            } else {
                None
            };
        if let Some(prepare_start) = prepare_start.as_ref() {
            obstacle_map_prepare_time_us += prepare_start.elapsed().as_micros();
        }
        let source_state = State::new(source.x, source.y, source.angle);
        let target_state = State::new(target.x, target.y, target.angle);
        let opened_search_owned = self.opened_cells_without_dynamic_overlap(
            opened_ref,
            source_state,
            target_state,
            dynamic_clearance_exempt_keys,
        );
        let opened_search_ref = opened_search_owned.as_ref().unwrap_or(opened_ref);
        let expected_crossing_partner_ids =
            if self.crossing_context.config().allow_only_expected_pairs {
                self.crossing_allowed_partner_set(net_id)
            } else {
                FxHashSet::default()
            };
        let require_crossing_compliant_route = self.crossing_context.is_enabled()
            && !self.use_collision_crossing_routing
            && self.crossing_context.config().allow_only_expected_pairs
            && !expected_crossing_partner_ids.is_empty();
        // `negotiated_crossing_free_search`: contribution 1's crossing-free
        // search for a net without any planned crossing -- the deferred
        // lidar-pure crossing search below is skipped, so a failed simple
        // probe falls through to the ordinary (crossing-free) search.
        let lidar_pure_crossing =
            self.lidar_pure_crossing_enabled() && !self.negotiated_crossing_free_search;
        // Fresh/native routing: try the plain, non-crossing attempt first
        // (see the simple-route block below); a lidar-pure collision-crossing
        // search only happens later, deferred, if that plain attempt fails
        // (the second, separate `lidar_pure_full_map_partner_set` call further
        // down). PlainFirst is why this upfront call returns empty in
        // lidar-pure mode. See `CollisionCrossingTryOrder`'s own doc comment.
        let collision_partner_ids = self.upfront_collision_crossing_partner_ids(
            net_id,
            source_state,
            target_state,
            CollisionCrossingTryOrder::PlainFirst,
        );
        let used_collision_crossing_attempt =
            self.use_collision_crossing_routing && !collision_partner_ids.is_empty();
        let crossing_attempt = if used_collision_crossing_attempt {
            self.try_route_with_collision_crossings(
                net_id,
                source_state,
                target_state,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
                &collision_partner_ids,
                source_port_um,
                target_port_um,
                Some(&validation_opened_cell_keys),
                true,
            )?
        } else if self.crossing_context.config().allow_only_expected_pairs {
            self.try_route_through_expected_crossing_partner(
                net_id,
                source_state,
                target_state,
                target_port_um,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
            )
        } else {
            None
        };
        let mut deferred_crossing_attempt = crossing_attempt;
        // A failed local collision-crossing attempt means this specific
        // crossing placement is unavailable. It must not make the whole net
        // unroutable: normal A* can still route around the blocker, and only a
        // true dynamic blockage should enter rip-up/repair.
        if require_crossing_compliant_route {
            return Err("No crossing-compliant route found".to_string());
        }
        // Set when the simple probe found a route whose commit was rejected
        // only by other nets' clearance halos, never by another net's core
        // (see the probe commit below). Such a conflict is a spacing problem,
        // not a collision that a crossing could resolve, so lidar-pure must
        // not stop at its collision-crossing search: the ordinary plain A*
        // further down, which honors this net's clearance exemptions, gets
        // its turn -- and the unwindowed lidar-pure collision-crossing search
        // is skipped, since there is nothing to cross. With clearance 0 the
        // halo *is* the core, so this can never be set there and the
        // crossing benchmarks are unaffected.
        let mut probe_halo_only_conflict = false;
        {
            let simple_start = collect_timing.then(Instant::now);
            let simple_result = if block_radius_cells > 0 {
                try_simple_route_with_dynamic_expansion_config(
                    &self.obstacle_map,
                    &self.primitives,
                    source_state,
                    target_state,
                    Some(opened_search_ref),
                    &cfg,
                    block_radius_cells,
                    dynamic_clearance_exempt_keys,
                )
            } else {
                try_simple_route_with_config(
                    &self.obstacle_map,
                    &self.primitives,
                    source_state,
                    target_state,
                    Some(opened_search_ref),
                    &cfg,
                )
            };
            let trace_plain = self.router_config.diagnostics.trace_plain_route_net == Some(net_id);
            if trace_plain {
                eprintln!(
                    "trace_plain_route net={net_id} stage=simple_probe found={}",
                    simple_result.is_some()
                );
            }
            if let Some(mut result) = simple_result {
                if let Some(simple_start) = simple_start.as_ref() {
                    simple_route_time_us += simple_start.elapsed().as_micros();
                }
                let commit_prepare_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let (route_cells, core_cells) = self.route_commit_and_core_cells(
                    &result,
                    block_radius_cells,
                    commit_radius_cells,
                    clearance_exempt_cells,
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                );
                if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
                    commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
                }
                let commit_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let allowed_partner_ids = FxHashSet::default();
                let committed = self
                    .obstacle_map
                    .commit_route_with_clearance_and_allowed_core_overlaps(
                        net_id,
                        &core_cells,
                        &route_cells,
                        clearance_exempt_cells.unwrap_or(&[]),
                        &allowed_partner_ids,
                    );
                if let Some(commit_start) = commit_start.as_ref() {
                    commit_time_us += commit_start.elapsed().as_micros();
                }
                if !committed {
                    probe_halo_only_conflict = !core_cells
                        .iter()
                        .any(|&(x, y)| self.obstacle_map.is_dynamic_core_blocked(x, y));
                }
                if trace_plain {
                    let offending = core_cells.iter().copied().find(|&(x, y)| {
                        self.obstacle_map.is_dynamic_blocked(x, y)
                            && !clearance_exempt_cells.is_some_and(|cells| cells.contains(&(x, y)))
                    });
                    eprintln!(
                        "trace_plain_route net={net_id} stage=simple_commit committed={committed} \
                         core_cells={} first_non_exempt_dynamic_core_cell={offending:?} \
                         halo_only_conflict={probe_halo_only_conflict}",
                        core_cells.len()
                    );
                }
                if committed {
                    self.remove_crossing_events_for_net(net_id);
                    self.register_geometric_crossing_events_for_route(
                        net_id,
                        &result,
                        source_port_um,
                        target_port_um,
                    );
                    if collect_timing {
                        result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
                        result.stats.simple_route_time_us += simple_route_time_us;
                        result.stats.commit_prepare_time_us += commit_prepare_time_us;
                        result.stats.commit_time_us += commit_time_us;
                    }
                    if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                        net_id,
                        &result,
                        source_port_um,
                        target_port_um,
                    ) {
                        self.rollback_committed_route(net_id);
                        return Err(error);
                    }
                    self.remember_committed_route_opened_cells(
                        net_id,
                        Some(&validation_opened_cell_keys),
                    );
                    self.add_post_commit_guidance_for_route(net_id, &result);
                    self.invalidate_meander_base_prefix();
                    if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                        net_id,
                        &result,
                        source_port_um,
                        target_port_um,
                        Some(&validation_opened_cell_keys),
                    ) {
                        self.rollback_committed_route(net_id);
                        return Err(error);
                    }
                    return Ok(result);
                }
            } else if let Some(simple_start) = simple_start.as_ref() {
                simple_route_time_us += simple_start.elapsed().as_micros();
            }
        }
        let mut lidar_pure_crossing_attempted = false;
        // The deferred half of PlainFirst (see `upfront_collision_crossing_partner_ids`
        // above and `CollisionCrossingTryOrder`'s own doc comment): the plain
        // attempt above has now failed, so it is time to pay for the full,
        // unwindowed lidar-pure collision-crossing search.
        if lidar_pure_crossing && deferred_crossing_attempt.is_none() && !probe_halo_only_conflict {
            let owner_lookup_partner_ids = self.lidar_pure_owner_lookup_partner_set(net_id);
            if !owner_lookup_partner_ids.is_empty() {
                lidar_pure_crossing_attempted = true;
                deferred_crossing_attempt = self.try_route_with_collision_crossings(
                    net_id,
                    source_state,
                    target_state,
                    opened_search_ref,
                    &cfg,
                    block_radius_cells,
                    dynamic_clearance_exempt_keys,
                    &owner_lookup_partner_ids,
                    source_port_um,
                    target_port_um,
                    Some(&validation_opened_cell_keys),
                    true,
                )?;
            }
        }
        let mut ordinary_collision_fallback: Option<RouteResult> = None;
        if let Some((mut crossing_result, crossing_events)) = deferred_crossing_attempt {
            let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
            if crossed_partner_ids.is_empty() && !lidar_pure_crossing {
                ordinary_collision_fallback = Some(crossing_result);
            } else {
                let allowed_crossing_core_keys =
                    Self::crossing_reservation_keys_for_events(&crossing_events);
                let commit_prepare_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let (route_cells, core_cells) = self.route_commit_and_core_cells(
                    &crossing_result,
                    block_radius_cells,
                    commit_radius_cells,
                    clearance_exempt_cells,
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                );
                if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
                    commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
                }
                let commit_start = if collect_timing {
                    Some(Instant::now())
                } else {
                    None
                };
                let committed = self
                    .obstacle_map
                    .commit_route_with_clearance_and_allowed_core_overlap_cells(
                        net_id,
                        &core_cells,
                        &route_cells,
                        clearance_exempt_cells.unwrap_or(&[]),
                        &crossed_partner_ids,
                        Some(&allowed_crossing_core_keys),
                    );
                if let Some(commit_start) = commit_start.as_ref() {
                    commit_time_us += commit_start.elapsed().as_micros();
                }
                if committed {
                    self.remove_crossing_events_for_net(net_id);
                    self.add_crossing_events(crossing_events);
                    if collect_timing {
                        crossing_result.stats.obstacle_map_prepare_time_us +=
                            obstacle_map_prepare_time_us;
                        crossing_result.stats.simple_route_time_us += simple_route_time_us;
                        crossing_result.stats.commit_prepare_time_us += commit_prepare_time_us;
                        crossing_result.stats.commit_time_us += commit_time_us;
                    }
                    if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                        net_id,
                        &crossing_result,
                        source_port_um,
                        target_port_um,
                    ) {
                        self.rollback_committed_route(net_id);
                        return Err(error);
                    }
                    self.remember_committed_route_opened_cells(
                        net_id,
                        Some(&validation_opened_cell_keys),
                    );
                    self.add_post_commit_guidance_for_route(net_id, &crossing_result);
                    self.invalidate_meander_base_prefix();
                    if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                        net_id,
                        &crossing_result,
                        source_port_um,
                        target_port_um,
                        Some(&validation_opened_cell_keys),
                    ) {
                        self.rollback_committed_route(net_id);
                        self.remove_crossing_events_for_net(net_id);
                        if !self.use_collision_crossing_routing || lidar_pure_crossing {
                            return Err(error);
                        }
                    } else {
                        return Ok(crossing_result);
                    }
                } else if lidar_pure_crossing {
                    return Err(self.dynamic_commit_rejection_error(
                        net_id,
                        &core_cells,
                        clearance_exempt_cells.unwrap_or(&[]),
                    ));
                }
            }
        }
        if lidar_pure_crossing_attempted && !probe_halo_only_conflict {
            return Err("No legal LiDAR crossing route found".to_string());
        }
        if self.router_config.diagnostics.trace_plain_route_net == Some(net_id) {
            eprintln!(
                "trace_plain_route net={net_id} stage=ordinary_search lidar_pure_attempted={lidar_pure_crossing_attempted} halo_only_conflict={probe_halo_only_conflict}"
            );
        }
        let search_cfg = if block_radius_cells > 0 {
            let mut search_cfg = self.astar_config(None, Some(false), None)?;
            search_cfg.require_terminal_straights = true;
            search_cfg
        } else {
            cfg
        };
        let zero_radius_overlay = block_radius_cells <= 0
            && dynamic_clearance_exempt_keys.is_some()
            && !search_cfg.enable_jps4;
        let mut opened_dynamic_obstacle_map;
        let outcome = if block_radius_cells > 0 || zero_radius_overlay {
            let env = SearchEnvironment {
                obstacle_map: &self.obstacle_map,
                primitives: &self.primitives,
            };
            let request = SearchRequest {
                source: source_state,
                target: target_state,
                port_open_cells: Some(opened_search_ref),
                dynamic_expansion: Some(DynamicExpansion {
                    radius_cells: block_radius_cells.max(0),
                    clearance_exempt_cells: dynamic_clearance_exempt_keys,
                }),
                crossing: None,
                config: &search_cfg,
            };
            self.search_engine.search(&env, &request)
        } else {
            let search_obstacle_map = if dynamic_clearance_exempt_keys.is_some() {
                opened_dynamic_obstacle_map = self.obstacle_map.clone();
                opened_dynamic_obstacle_map
                    .clear_dynamic_clearance_in_cells(dynamic_clearance_exempt_cell_vec);
                &opened_dynamic_obstacle_map
            } else {
                &self.obstacle_map
            };
            let env = SearchEnvironment {
                obstacle_map: search_obstacle_map,
                primitives: &self.primitives,
            };
            let request = SearchRequest {
                source: source_state,
                target: target_state,
                port_open_cells: Some(opened_search_ref),
                dynamic_expansion: None,
                crossing: None,
                config: &search_cfg,
            };
            self.search_engine.search(&env, &request)
        };
        let fallback_search_stats = outcome.stats;
        let search_option = outcome.route;
        // Recorded before `.ok_or_else` below turns a failed search into an
        // `Err` that discards `fallback_search_stats` entirely -- see
        // `last_search_expanded_states`'s doc comment.
        self.last_search_expanded_states = fallback_search_stats.expanded_states as u64;
        let mut result = search_option.ok_or_else(|| {
            format!(
                "No route found (expanded_states={}, generated_neighbors={}, window_attempts={}, used_full_grid_fallback={}, last_window_area_cells={}, max_window_area_cells={})",
                fallback_search_stats.expanded_states,
                fallback_search_stats.generated_neighbors,
                fallback_search_stats.window_attempts,
                fallback_search_stats.used_full_grid_fallback,
                fallback_search_stats.last_window_area_cells,
                fallback_search_stats.max_window_area_cells,
            )
        })?;
        if let Some(fallback) = ordinary_collision_fallback {
            if fallback.total_cost + 1.0e-9 < result.total_cost {
                result = fallback;
            }
        }

        if collect_timing {
            result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
            result.stats.simple_route_time_us += simple_route_time_us;
        }
        if let Some(error) = self.invalid_grid_crossing_error_for_route(net_id, &result) {
            return Err(error);
        }
        if used_collision_crossing_attempt
            && !self
                .crossing_events_for_route(net_id, &result, &collision_partner_ids)
                .is_empty()
        {
            return Err(format!(
                "No crossing-compliant route found for net {net_id}: fallback route contains unreserved crossings"
            ));
        }
        let commit_prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let (route_cells, core_cells) = self.route_commit_and_core_cells(
            &result,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
        );
        if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
            result.stats.commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
        }
        let commit_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let allowed_partner_ids = FxHashSet::default();
        let committed = self
            .obstacle_map
            .commit_route_with_clearance_and_allowed_core_overlaps(
                net_id,
                &core_cells,
                &route_cells,
                clearance_exempt_cells.unwrap_or(&[]),
                &allowed_partner_ids,
            );
        if let Some(commit_start) = commit_start.as_ref() {
            result.stats.commit_time_us += commit_start.elapsed().as_micros();
        }
        if !committed {
            let error = self.dynamic_commit_rejection_error(
                net_id,
                &core_cells,
                clearance_exempt_cells.unwrap_or(&[]),
            );
            let retry_keepout = self.dynamic_commit_error_repair_keepout_keys(&error);
            if !retry_keepout.is_empty() {
                self.obstacle_map.add_static_keys(&retry_keepout);
                let retry_opened_cells: Vec<(i32, i32)> = validation_opened_cell_keys
                    .iter()
                    .copied()
                    .map(unpack_xy)
                    .collect();
                let retry_result = self.route_single_net_and_commit_repair_native(
                    net_id,
                    source,
                    target,
                    block_radius_cells,
                    Some(&retry_opened_cells),
                    Some(&validation_opened_cell_keys),
                    0.0,
                    commit_radius_cells,
                    clearance_exempt_cells,
                    clearance_exempt_cell_keys,
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                );
                self.obstacle_map.remove_static_keys(&retry_keepout);
                if retry_result.is_ok() {
                    return retry_result;
                }
            }
            return Err(error);
        }
        self.remove_crossing_events_for_net(net_id);
        self.register_geometric_crossing_events_for_route(
            net_id,
            &result,
            source_port_um,
            target_port_um,
        );
        if let Err(error) = self.remember_committed_route_centerlines_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }
        self.remember_committed_route_opened_cells(net_id, Some(&validation_opened_cell_keys));
        self.add_post_commit_guidance_for_route(net_id, &result);
        self.invalidate_meander_base_prefix();
        if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
            Some(&validation_opened_cell_keys),
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }

        Ok(result)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_single_net_and_commit_repair_native(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: Option<&[(i32, i32)]>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
        history_weight: f64,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: Option<&[(i32, i32)]>,
        clearance_exempt_cell_keys: Option<&FxHashSet<CellKey>>,
        core_radius_cells: Option<i32>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err("target_tolerance_cells must be >= 0".to_string());
        }
        let opened_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(keys) = opened_cell_keys {
            keys
        } else if let Some(cells) = opened_cells {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            &self.port_open_cells
        };
        let validation_opened_cell_keys = opened_ref.clone();
        let mut cfg = self.astar_config(Some(false), Some(false), Some(history_weight))?;
        cfg.require_terminal_straights = true;
        let dynamic_clearance_exempt_cell_vec = clearance_exempt_cells.unwrap_or(&[]);
        let collect_timing = self.astar_cfg.collect_detailed_timing;
        let prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let dynamic_clearance_exempt_keys_owned: Option<FxHashSet<CellKey>> = if block_radius_cells
            > 0
            && !dynamic_clearance_exempt_cell_vec.is_empty()
            && clearance_exempt_cell_keys.is_none()
        {
            Some(pack_cells(dynamic_clearance_exempt_cell_vec))
        } else {
            None
        };
        let dynamic_clearance_exempt_keys =
            if block_radius_cells > 0 && !dynamic_clearance_exempt_cell_vec.is_empty() {
                clearance_exempt_cell_keys.or(dynamic_clearance_exempt_keys_owned.as_ref())
            } else {
                None
            };
        let obstacle_map_prepare_time_us = prepare_start
            .as_ref()
            .map_or(0, |start| start.elapsed().as_micros());
        let source_state = State::new(source.x, source.y, source.angle);
        let target_state = State::new(target.x, target.y, target.angle);
        let opened_search_owned = self.opened_cells_without_dynamic_overlap(
            opened_ref,
            source_state,
            target_state,
            dynamic_clearance_exempt_keys,
        );
        let opened_search_ref = opened_search_owned.as_ref().unwrap_or(opened_ref);
        let expected_crossing_partner_ids =
            if self.crossing_context.config().allow_only_expected_pairs {
                self.crossing_allowed_partner_set(net_id)
            } else {
                FxHashSet::default()
            };
        let require_crossing_compliant_route = self.crossing_context.is_enabled()
            && !self.use_collision_crossing_routing
            && self.crossing_context.config().allow_only_expected_pairs
            && !expected_crossing_partner_ids.is_empty();
        // Repair: consult collision-crossing partners immediately (see
        // `CollisionCrossingTryOrder::CrossingFirst`'s own doc comment) --
        // repair only runs after this net already failed to route normally,
        // so trying the collision-crossing path first is a reasonable prior
        // specifically in this context, unlike fresh/native routing above.
        let collision_partner_ids = self.upfront_collision_crossing_partner_ids(
            net_id,
            source_state,
            target_state,
            CollisionCrossingTryOrder::CrossingFirst,
        );
        let used_collision_crossing_attempt =
            self.use_collision_crossing_routing && !collision_partner_ids.is_empty();
        let crossing_attempt = if used_collision_crossing_attempt {
            self.try_route_with_collision_crossings(
                net_id,
                source_state,
                target_state,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
                &collision_partner_ids,
                source_port_um,
                target_port_um,
                Some(&validation_opened_cell_keys),
                true,
            )?
        } else if self.crossing_context.config().allow_only_expected_pairs {
            self.try_route_through_expected_crossing_partner(
                net_id,
                source_state,
                target_state,
                target_port_um,
                opened_search_ref,
                &cfg,
                block_radius_cells,
                dynamic_clearance_exempt_keys,
            )
        } else {
            None
        };
        if let Some((mut crossing_result, crossing_events)) = crossing_attempt {
            let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
            let allowed_crossing_core_keys =
                Self::crossing_reservation_keys_for_events(&crossing_events);
            let commit_prepare_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            let (route_cells, core_cells) = self.route_commit_and_core_cells(
                &crossing_result,
                block_radius_cells,
                commit_radius_cells,
                clearance_exempt_cells,
                core_radius_cells,
                source_port_um,
                target_port_um,
            );
            if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
                crossing_result.stats.commit_prepare_time_us +=
                    commit_prepare_start.elapsed().as_micros();
            }
            let commit_start = if collect_timing {
                Some(Instant::now())
            } else {
                None
            };
            let committed = self
                .obstacle_map
                .commit_route_with_clearance_and_allowed_core_overlap_cells(
                    net_id,
                    &core_cells,
                    &route_cells,
                    clearance_exempt_cells.unwrap_or(&[]),
                    &crossed_partner_ids,
                    Some(&allowed_crossing_core_keys),
                );
            if let Some(commit_start) = commit_start.as_ref() {
                crossing_result.stats.commit_time_us += commit_start.elapsed().as_micros();
            }
            if committed {
                self.remove_crossing_events_for_net(net_id);
                self.add_crossing_events(crossing_events);
                if collect_timing {
                    crossing_result.stats.obstacle_map_prepare_time_us +=
                        obstacle_map_prepare_time_us;
                }
                if let Err(error) = self.remember_committed_route_centerlines_with_ports(
                    net_id,
                    &crossing_result,
                    source_port_um,
                    target_port_um,
                ) {
                    self.rollback_committed_route(net_id);
                    return Err(error);
                }
                self.remember_committed_route_opened_cells(
                    net_id,
                    Some(&validation_opened_cell_keys),
                );
                self.add_post_commit_guidance_for_route(net_id, &crossing_result);
                self.invalidate_meander_base_prefix();
                if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
                    net_id,
                    &crossing_result,
                    source_port_um,
                    target_port_um,
                    Some(&validation_opened_cell_keys),
                ) {
                    self.rollback_committed_route(net_id);
                    return Err(error);
                }
                return Ok(crossing_result);
            }
        }
        // Keep collision-crossing as a preferred fast path, but fall back to
        // ordinary A* when the local legal crossing candidate is rejected.
        if require_crossing_compliant_route {
            return Err("No crossing-compliant route found".to_string());
        }
        let zero_radius_overlay =
            block_radius_cells <= 0 && dynamic_clearance_exempt_keys.is_some() && !cfg.enable_jps4;
        let mut opened_dynamic_obstacle_map;
        let mut result = if block_radius_cells > 0 || zero_radius_overlay {
            let env = SearchEnvironment {
                obstacle_map: &self.obstacle_map,
                primitives: &self.primitives,
            };
            let request = SearchRequest {
                source: source_state,
                target: target_state,
                port_open_cells: Some(opened_search_ref),
                dynamic_expansion: Some(DynamicExpansion {
                    radius_cells: block_radius_cells.max(0),
                    clearance_exempt_cells: dynamic_clearance_exempt_keys,
                }),
                crossing: None,
                config: &cfg,
            };
            self.search_engine.search(&env, &request).route
        } else {
            let search_obstacle_map = if dynamic_clearance_exempt_keys.is_some() {
                opened_dynamic_obstacle_map = self.obstacle_map.clone();
                opened_dynamic_obstacle_map
                    .clear_dynamic_clearance_in_cells(dynamic_clearance_exempt_cell_vec);
                &opened_dynamic_obstacle_map
            } else {
                &self.obstacle_map
            };
            let env = SearchEnvironment {
                obstacle_map: search_obstacle_map,
                primitives: &self.primitives,
            };
            let request = SearchRequest {
                source: source_state,
                target: target_state,
                port_open_cells: Some(opened_search_ref),
                dynamic_expansion: None,
                crossing: None,
                config: &cfg,
            };
            self.search_engine.search(&env, &request).route
        }
        .ok_or_else(|| "No route found".to_string())?;

        if collect_timing {
            result.stats.obstacle_map_prepare_time_us += obstacle_map_prepare_time_us;
        }
        if let Some(error) = self.invalid_grid_crossing_error_for_route(net_id, &result) {
            return Err(error);
        }
        let commit_prepare_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let (route_cells, core_cells) = self.route_commit_and_core_cells(
            &result,
            block_radius_cells,
            commit_radius_cells,
            clearance_exempt_cells,
            core_radius_cells,
            source_port_um,
            target_port_um,
        );
        if let Some(commit_prepare_start) = commit_prepare_start.as_ref() {
            result.stats.commit_prepare_time_us += commit_prepare_start.elapsed().as_micros();
        }
        let commit_start = if collect_timing {
            Some(Instant::now())
        } else {
            None
        };
        let committed = self.obstacle_map.commit_route_with_clearance_overlap(
            net_id,
            &core_cells,
            &route_cells,
            clearance_exempt_cells.unwrap_or(&[]),
        );
        if let Some(commit_start) = commit_start.as_ref() {
            result.stats.commit_time_us += commit_start.elapsed().as_micros();
        }
        if !committed {
            return Err(self.dynamic_commit_rejection_error(
                net_id,
                &core_cells,
                clearance_exempt_cells.unwrap_or(&[]),
            ));
        }
        self.remove_crossing_events_for_net(net_id);
        self.register_geometric_crossing_events_for_route(
            net_id,
            &result,
            source_port_um,
            target_port_um,
        );
        if let Err(error) = self.remember_committed_route_centerlines_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }
        self.remember_committed_route_opened_cells(net_id, Some(&validation_opened_cell_keys));
        self.add_post_commit_guidance_for_route(net_id, &result);
        self.invalidate_meander_base_prefix();
        if let Err(error) = self.validate_committed_crossings_for_route_with_ports(
            net_id,
            &result,
            source_port_um,
            target_port_um,
            Some(&validation_opened_cell_keys),
        ) {
            self.rollback_committed_route(net_id);
            return Err(error);
        }

        Ok(result)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_single_net_and_commit_native_with_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        clearance_exempt_cell_keys: &FxHashSet<CellKey>,
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        if repair_keepout.is_empty() {
            return self.route_single_net_and_commit_native(
                net_id,
                source,
                target,
                block_radius_cells,
                Some(opened_cells),
                Some(opened_cell_keys),
                commit_radius_cells,
                Some(clearance_exempt_cells),
                Some(clearance_exempt_cell_keys),
                core_radius_cells,
                source_port_um,
                target_port_um,
            );
        }
        let filtered_opened =
            opened_cells_excluding_keepout(opened_cells, repair_keepout, source, target);
        if filtered_opened.len() == opened_cells.len() {
            return self.route_single_net_and_commit_native(
                net_id,
                source,
                target,
                block_radius_cells,
                Some(opened_cells),
                Some(opened_cell_keys),
                commit_radius_cells,
                Some(clearance_exempt_cells),
                Some(clearance_exempt_cell_keys),
                core_radius_cells,
                source_port_um,
                target_port_um,
            );
        }
        let filtered_opened_keys = pack_cells(&filtered_opened);
        let result = self.route_single_net_and_commit_native(
            net_id,
            source,
            target,
            block_radius_cells,
            Some(&filtered_opened),
            Some(&filtered_opened_keys),
            commit_radius_cells,
            Some(clearance_exempt_cells),
            Some(clearance_exempt_cell_keys),
            core_radius_cells,
            source_port_um,
            target_port_um,
        );
        if matches!(result, Err(ref error) if error == "No route found") {
            self.route_single_net_and_commit_native(
                net_id,
                source,
                target,
                block_radius_cells,
                Some(opened_cells),
                Some(opened_cell_keys),
                commit_radius_cells,
                Some(clearance_exempt_cells),
                Some(clearance_exempt_cell_keys),
                core_radius_cells,
                source_port_um,
                target_port_um,
            )
        } else {
            result
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn route_single_net_and_commit_repair_native_with_repair_keepout(
        &mut self,
        net_id: u64,
        source: PyState,
        target: PyState,
        block_radius_cells: i32,
        opened_cells: &[(i32, i32)],
        opened_cell_keys: &FxHashSet<CellKey>,
        history_weight: f64,
        commit_radius_cells: Option<i32>,
        clearance_exempt_cells: &[(i32, i32)],
        clearance_exempt_cell_keys: &FxHashSet<CellKey>,
        core_radius_cells: Option<i32>,
        repair_keepout: &FxHashSet<CellKey>,
        source_port_um: Option<(f64, f64)>,
        target_port_um: Option<(f64, f64)>,
    ) -> Result<RouteResult, String> {
        let mut active_keepout = repair_keepout.clone();
        let mut internally_added_keepout = FxHashSet::default();
        let mut feedback_attempts = 0usize;
        loop {
            let result = if active_keepout.is_empty() {
                self.route_single_net_and_commit_repair_native(
                    net_id,
                    source,
                    target,
                    block_radius_cells,
                    Some(opened_cells),
                    Some(opened_cell_keys),
                    history_weight,
                    commit_radius_cells,
                    Some(clearance_exempt_cells),
                    Some(clearance_exempt_cell_keys),
                    core_radius_cells,
                    source_port_um,
                    target_port_um,
                )
            } else {
                let filtered_opened =
                    opened_cells_excluding_keepout(opened_cells, &active_keepout, source, target);
                if filtered_opened.len() == opened_cells.len() {
                    self.route_single_net_and_commit_repair_native(
                        net_id,
                        source,
                        target,
                        block_radius_cells,
                        Some(opened_cells),
                        Some(opened_cell_keys),
                        history_weight,
                        commit_radius_cells,
                        Some(clearance_exempt_cells),
                        Some(clearance_exempt_cell_keys),
                        core_radius_cells,
                        source_port_um,
                        target_port_um,
                    )
                } else {
                    let filtered_opened_keys = pack_cells(&filtered_opened);
                    let result = self.route_single_net_and_commit_repair_native(
                        net_id,
                        source,
                        target,
                        block_radius_cells,
                        Some(&filtered_opened),
                        Some(&filtered_opened_keys),
                        history_weight,
                        commit_radius_cells,
                        Some(clearance_exempt_cells),
                        Some(clearance_exempt_cell_keys),
                        core_radius_cells,
                        source_port_um,
                        target_port_um,
                    );
                    if matches!(result, Err(ref error) if error == "No route found") {
                        self.route_single_net_and_commit_repair_native(
                            net_id,
                            source,
                            target,
                            block_radius_cells,
                            Some(opened_cells),
                            Some(opened_cell_keys),
                            history_weight,
                            commit_radius_cells,
                            Some(clearance_exempt_cells),
                            Some(clearance_exempt_cell_keys),
                            core_radius_cells,
                            source_port_um,
                            target_port_um,
                        )
                    } else {
                        result
                    }
                }
            };

            match result {
                Ok(route) => {
                    if !internally_added_keepout.is_empty() {
                        self.obstacle_map
                            .remove_static_keys(&internally_added_keepout);
                    }
                    return Ok(route);
                }
                Err(error) => {
                    if feedback_attempts >= 4 {
                        if !internally_added_keepout.is_empty() {
                            self.obstacle_map
                                .remove_static_keys(&internally_added_keepout);
                        }
                        return Err(error);
                    }
                    let error_keepout = self.crossing_error_repair_keepout_keys(&error);
                    let extra_keepout: FxHashSet<CellKey> = error_keepout
                        .into_iter()
                        .filter(|key| !active_keepout.contains(key))
                        .collect();
                    if extra_keepout.is_empty() {
                        if !internally_added_keepout.is_empty() {
                            self.obstacle_map
                                .remove_static_keys(&internally_added_keepout);
                        }
                        return Err(error);
                    }
                    self.obstacle_map.add_static_keys(&extra_keepout);
                    active_keepout.extend(extra_keepout.iter().copied());
                    internally_added_keepout.extend(extra_keepout);
                    feedback_attempts = feedback_attempts.saturating_add(1);
                }
            }
        }
    }

    pub(crate) fn route_single_net_ignore_dynamic_native(
        &self,
        source: PyState,
        target: PyState,
        opened_cells: Option<&[(i32, i32)]>,
        opened_cell_keys: Option<&FxHashSet<CellKey>>,
    ) -> Result<RouteResult, String> {
        if self.astar_cfg.target_tolerance_cells < 0 {
            return Err("target_tolerance_cells must be >= 0".to_string());
        }
        let opened_owned;
        let opened_ref: &FxHashSet<CellKey> = if let Some(keys) = opened_cell_keys {
            keys
        } else if let Some(cells) = opened_cells {
            opened_owned = pack_cells(cells);
            &opened_owned
        } else {
            &self.port_open_cells
        };
        let mut cfg = self.astar_config(Some(true), Some(false), Some(0.0))?;
        cfg.require_terminal_straights = true;
        let mut static_only_obstacle_map = self.obstacle_map.clone();
        static_only_obstacle_map.clear_dynamic();
        let env = SearchEnvironment {
            obstacle_map: &static_only_obstacle_map,
            primitives: &self.primitives,
        };
        let request = SearchRequest {
            source: State::new(source.x, source.y, source.angle),
            target: State::new(target.x, target.y, target.angle),
            port_open_cells: Some(opened_ref),
            dynamic_expansion: None,
            crossing: None,
            config: &cfg,
        };
        self.search_engine
            .search(&env, &request)
            .route
            .ok_or_else(|| "No route found".to_string())
    }

    #[allow(clippy::too_many_arguments)]
    /// The baseline attempt: routes the net directly via
    /// `route_single_net_and_commit_native`, with no victims, no keepouts,
    /// and no repair. Every other `try_*` method in this file exists
    /// because this one failed.
    pub(crate) fn try_plain_normal_route(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
    ) -> PlainRouteOutcome {
        let route_start = native_batch_timer(collect_native_timing);
        let route_result = self.route_single_net_and_commit_native(
            job.net_id,
            job.source,
            job.target,
            block_radius_cells,
            Some(&job.opened_cells),
            Some(&job.opened_cell_keys),
            commit_radius_cells,
            Some(&job.clearance_exempt_cells),
            Some(&job.clearance_exempt_cell_keys),
            core_radius_cells,
            job.source_port_um,
            job.target_port_um,
        );
        let route_elapsed_us = native_batch_elapsed_us(route_start);
        batch.timings.normal_route_wall_us += route_elapsed_us;
        match route_result {
            Ok(route) => {
                batch
                    .timings
                    .add_route_result_stats_if(collect_native_timing, &route);
                remove_success_static_cleanup(&mut self.obstacle_map, job);
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "normal_route",
                    net_id: job.net_id,
                    route: Some(route.clone()),
                    failed: false,
                    error: None,
                    repair_round: None,
                    candidate_blockers: Vec::new(),
                    ripup_ids: Vec::new(),
                });
                batch.final_routes.insert(job.net_id, route);
                PlainRouteOutcome::Routed
            }
            Err(error) => {
                batch.timings.normal_route_failed_wall_us += route_elapsed_us;
                batch.attempts.push(NativeRouteAttempt {
                    bucket_name: "normal_route",
                    net_id: job.net_id,
                    route: None,
                    failed: true,
                    error: Some(error.clone()),
                    repair_round: None,
                    candidate_blockers: Vec::new(),
                    ripup_ids: Vec::new(),
                });
                PlainRouteOutcome::NotResolved
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    /// Attempts to route the net directly through one already-committed
    /// crossing-partner net at a time (ordered by net index), requiring
    /// terminal straights, committing only if the resulting crossing is
    /// legal -- no ripup involved. Delegates to
    /// [`Self::try_route_with_collision_crossings`]. Gated on lidar-pure
    /// collision-crossing routing being enabled.
    pub(crate) fn try_lidar_direct_crossing_subset(
        &mut self,
        batch: &mut RepairBatchState,
        job: &NativeRouteJob,
        order_by_id: &FxHashMap<u64, usize>,
        block_radius_cells: i32,
        commit_radius_cells: Option<i32>,
        core_radius_cells: Option<i32>,
        collect_native_timing: bool,
        trace_native_repair: bool,
        partner_filter: Option<&[u64]>,
    ) -> PyResult<LidarDirectCrossingOutcome> {
        if !self.lidar_pure_crossing_enabled() || !self.use_collision_crossing_routing {
            return Ok(LidarDirectCrossingOutcome::NotResolved);
        }
        let source_state = State::new(job.source.x, job.source.y, job.source.angle);
        let target_state = State::new(job.target.x, job.target.y, job.target.angle);
        let mut local_partner_ids =
            self.crossing_partner_lookup_set_for_route(job.net_id, source_state, target_state);
        local_partner_ids.retain(|partner_id| batch.final_routes.contains_key(partner_id));
        // `None` (every caller but the negotiated loop): unchanged, sweeps
        // every committed partner this net's window touches. `Some(ids)`
        // (the negotiated loop, Milestone 5 part 3): restricts the sweep to
        // the probe's own illegal-crossing partners -- the 22-partner sweep
        // that resolved nothing on net 313 of the 64x64 mesh (see this
        // milestone's Surprises & Discoveries) is why this is opt-in rather
        // than the new default.
        if let Some(filter) = partner_filter {
            local_partner_ids.retain(|partner_id| filter.contains(partner_id));
        }
        if local_partner_ids.is_empty() {
            return Ok(LidarDirectCrossingOutcome::NotResolved);
        }
        let mut local_partner_vec: Vec<u64> = local_partner_ids.iter().copied().collect();
        local_partner_vec
            .sort_unstable_by_key(|owner| order_by_id.get(owner).copied().unwrap_or(usize::MAX));
        let opened_search_owned = self.opened_cells_without_dynamic_overlap(
            &job.opened_cell_keys,
            source_state,
            target_state,
            Some(&job.clearance_exempt_cell_keys),
        );
        let opened_search_ref = opened_search_owned
            .as_ref()
            .unwrap_or(&job.opened_cell_keys);
        let dynamic_clearance_exempt_keys =
            if block_radius_cells > 0 && !job.clearance_exempt_cells.is_empty() {
                Some(&job.clearance_exempt_cell_keys)
            } else {
                None
            };
        let mut subset_cfg = self
            .astar_config(None, None, None)
            .map_err(PyRuntimeError::new_err)?;
        subset_cfg.require_terminal_straights = true;
        for partner_id in local_partner_vec {
            let mut subset_partner_ids = FxHashSet::default();
            subset_partner_ids.insert(partner_id);
            let subset_start = native_batch_timer(collect_native_timing);
            let subset_result = self
                .try_route_with_collision_crossings(
                    job.net_id,
                    source_state,
                    target_state,
                    opened_search_ref,
                    &subset_cfg,
                    block_radius_cells,
                    dynamic_clearance_exempt_keys,
                    &subset_partner_ids,
                    job.source_port_um,
                    job.target_port_um,
                    Some(&job.opened_cell_keys),
                    false,
                )
                .map_err(PyRuntimeError::new_err)?;
            let subset_elapsed_us = native_batch_elapsed_us(subset_start);
            batch.timings.repair_failed_net_wall_us += subset_elapsed_us;
            let Some((route, crossing_events)) = subset_result else {
                continue;
            };
            let crossed_partner_ids = Self::crossing_partner_ids_from_events(&crossing_events);
            if crossed_partner_ids.is_empty() {
                continue;
            }
            let crossed_partner_vec: Vec<u64> = crossed_partner_ids.iter().copied().collect();
            match self.commit_native_route_with_clearance_allowing_core_overlap(
                job.net_id,
                &route,
                block_radius_cells,
                commit_radius_cells,
                &job.clearance_exempt_cells,
                core_radius_cells,
                job.source_port_um,
                job.target_port_um,
                Some(&job.opened_cell_keys),
                &crossed_partner_vec,
                true,
            ) {
                Ok(true) => {
                    batch
                        .timings
                        .add_route_result_stats_if(collect_native_timing, &route);
                    if trace_native_repair {
                        eprintln!(
                            "{}native_repair_lidar_direct_crossing net={} crossed={:?}",
                            trace_t(self.negotiated_batch_start),
                            job.net_id,
                            crossed_partner_vec
                        );
                    }
                    batch.attempts.push(NativeRouteAttempt {
                        bucket_name: "lidar_direct_crossing",
                        net_id: job.net_id,
                        route: Some(route.clone()),
                        failed: false,
                        error: None,
                        repair_round: None,
                        candidate_blockers: Vec::new(),
                        ripup_ids: Vec::new(),
                    });
                    batch.final_routes.insert(job.net_id, route);
                    return Ok(LidarDirectCrossingOutcome::Routed);
                }
                Ok(false) => {}
                Err(error) => {
                    if trace_native_repair {
                        eprintln!(
                            "native_repair_lidar_direct_crossing_commit_failed net={} partner={} error={}",
                            job.net_id, partner_id, error
                        );
                    }
                }
            }
        }
        Ok(LidarDirectCrossingOutcome::NotResolved)
    }

    pub(crate) fn clear_pending_straight_victim_hint(&self, net_id: u64) {
        let mut hint = self.last_pending_straight_victim.borrow_mut();
        if hint.as_ref().is_some_and(|hint| hint.net_id == net_id) {
            *hint = None;
        }
    }

    pub(crate) fn remember_pending_straight_victim_hint(
        &self,
        net_id: u64,
        stats: &RouteSearchStats,
    ) {
        let threshold = self
            .router_config
            .negotiation
            .pending_straight_ripup_threshold;
        if threshold == 0 {
            return;
        }
        let Some((&victim_net_id, &count)) = stats
            .crossing_pending_straight_by_partner
            .iter()
            .filter(|(_, count)| **count >= threshold)
            .max_by_key(|(_, count)| *count)
        else {
            return;
        };
        *self.last_pending_straight_victim.borrow_mut() = Some(PendingStraightVictimHint {
            net_id,
            victim_net_id,
            count,
        });
        if self.router_config.diagnostics.native_repair_diag {
            eprintln!(
                "native_repair_pending_straight_hint net={} victim={} count={} threshold={}",
                net_id, victim_net_id, count, threshold
            );
        }
    }

    pub(crate) fn pending_straight_victim_hint_for(
        &self,
        net_id: u64,
    ) -> Option<PendingStraightVictimHint> {
        self.last_pending_straight_victim
            .borrow()
            .as_ref()
            .filter(|hint| hint.net_id == net_id)
            .cloned()
    }

    pub(crate) fn geometry_grid(&self) -> Result<GeometryGridSpec, String> {
        GeometryGridSpec::new(
            self.grid.grid_size_um,
            self.grid.origin_x_um,
            self.grid.origin_y_um,
        )
        .map_err(|err| err.to_string())
    }

    pub(crate) fn grid_waypoints_to_centerline(&self, waypoints: &[(i32, i32)]) -> Vec<(f64, f64)> {
        let grid = GeometryGridSpec {
            grid_size_um: self.grid.grid_size_um,
            origin_x_um: self.grid.origin_x_um,
            origin_y_um: self.grid.origin_y_um,
        };
        waypoints
            .iter()
            .map(|(x, y)| grid.cell_center(*x, *y))
            .collect()
    }

    /// The opened cells a search may use: the port openings minus every
    /// cell another net already occupies -- except cells in this net's
    /// clearance-exempt set that only carry another net's clearance halo
    /// (not its core). Those stay open: the exemption exists so that two
    /// nets serving ports closer together than the clearance can approach
    /// side by side, and an approach cell is usually static underneath
    /// (port-lane reservation), so dropping it from the opened set here would
    /// re-block it before the search's own exemption check ever runs.
    pub(crate) fn opened_cells_without_dynamic_overlap(
        &self,
        opened_ref: &FxHashSet<CellKey>,
        source: State,
        target: State,
        dynamic_clearance_exempt_keys: Option<&FxHashSet<CellKey>>,
    ) -> Option<FxHashSet<CellKey>> {
        let source_key = pack_xy(source.x, source.y);
        let target_key = pack_xy(target.x, target.y);
        let filtered: FxHashSet<CellKey> = opened_ref
            .iter()
            .copied()
            .filter(|&key| {
                key == source_key || key == target_key || {
                    let (x, y) = unpack_xy(key);
                    let exempt_halo_only = dynamic_clearance_exempt_keys
                        .is_some_and(|keys| keys.contains(&key))
                        && !self.obstacle_map.is_dynamic_core_blocked(x, y);
                    exempt_halo_only
                        || (!self.obstacle_map.is_dynamic_blocked(x, y)
                            && self
                                .obstacle_map
                                .dynamic_owners_for_cells(&[(x, y)])
                                .is_empty())
                }
            })
            .collect();
        (filtered.len() != opened_ref.len()).then_some(filtered)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::test_support::*;

    /// Milestone 5 of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`:
    /// `negotiated_search_budget` (and therefore
    /// `AStarConfig::total_expansion_budget`) must stay `None` on every path
    /// the older repair chain (`route_many_with_repair_and_commit` and the
    /// plain-routing entry points it shares helpers with) uses to build a
    /// search config -- only `route_many_with_negotiated_repair_and_commit`
    /// ever sets it, and only for the duration of one search call. A fresh
    /// router has never run either engine, so `astar_config` here stands in
    /// for what every chain call sees.
    #[test]
    fn astar_config_leaves_total_expansion_budget_none_by_default() {
        let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
            PyAStarConfig::new(
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
            None,
        );
        assert_eq!(router.negotiated_search_budget, None);
        let cfg = router
            .astar_config(None, None, None)
            .expect("cached config must be valid");
        assert_eq!(
            cfg.total_expansion_budget, None,
            "the chain's config builder must never see a fail-fast budget"
        );
    }

    /// 2026-09-15 11:54 trace finding
    /// (`.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`):
    /// before this fix, `try_braid_repair`'s two searches (the current
    /// net's reroute and the partner's reroute, both issued via
    /// `try_ripup_single_victim_and_reroute` ->
    /// `route_single_net_and_commit_native` -> `self.astar_config(None,
    /// None, Some(self.commit_history_weight))`) ran with the router's
    /// default config because `route_many_with_negotiated_repair_and_commit`
    /// never set `negotiated_search_budget` around its `try_braid_repair`
    /// calls, so one failing partner reroute could run the benchmark's full
    /// `max_iterations` (20 M) unbounded -- ~13 minutes on the 64x64 mesh.
    /// The fix sets `self.negotiated_search_budget =
    /// Some(NEGOTIATED_BUDGET_BRAID)` around each of the negotiated loop's
    /// three `try_braid_repair` calls (visible in the source at each call
    /// site, just above the call) and clears it immediately after; since
    /// both of `try_braid_repair`'s searches share the exact same
    /// `astar_config` seam as every other native search, exercising that
    /// seam directly with the field set is the narrowest correct test --
    /// a full instrumented run through the negotiated loop would need a
    /// real two-net braid (two committed nets crossing each other twice)
    /// constructed end to end, which is impractical to add as a unit test
    /// here; see `astar_config_leaves_total_expansion_budget_none_by_default`
    /// above for the same seam with the field unset.
    #[test]
    fn astar_config_applies_negotiated_search_budget_when_set() {
        let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
            PyAStarConfig::new(
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
            None,
        );
        router.negotiated_search_budget = Some(NEGOTIATED_BUDGET_BRAID);
        let cfg = router
            .astar_config(None, None, None)
            .expect("cached config must be valid");
        assert_eq!(
            cfg.total_expansion_budget,
            Some(NEGOTIATED_BUDGET_BRAID),
            "every search issued while negotiated_search_budget is set (including \
             try_braid_repair's current-net and partner reroutes) must carry that \
             budget, not run unbounded"
        );
    }

    /// Probe-guided search helper (a): with an existing guidance of
    /// `{(1, 2)}` at loss 0 and probe partners `{5, 6}` for net 4, the
    /// temporary guidance installed by `with_probe_guided_guidance` must
    /// contain `(1, 2)`, `(4, 5)` and `(4, 6)` -- the union -- and once the
    /// closure returns, `crossing_context` must hold exactly the original
    /// guidance again (see the 2026-09-15 15:00 Decision Log entry of
    /// `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`).
    #[test]
    fn probe_guided_guidance_unions_existing_pairs_and_restores_after() {
        let mut router = small_test_router();
        router
            .crossing_context
            .set_guidance(CrossingGuidance::new(&[(1, 2)], 0.0));

        let observed = router.with_probe_guided_guidance(4, &[5, 6], |router| {
            let guidance = router
                .crossing_context
                .guidance()
                .expect("temporary guidance must be installed");
            (
                guidance.is_planned_pair(1, 2),
                guidance.is_planned_pair(4, 5),
                guidance.is_planned_pair(4, 6),
                guidance.planned_pair_count(),
                guidance.planned_crossing_loss,
            )
        });
        assert_eq!(observed, (true, true, true, 3, 0.0));

        let restored = router
            .crossing_context
            .guidance()
            .expect("the original guidance must be restored, not cleared");
        assert!(restored.is_planned_pair(1, 2));
        assert_eq!(restored.planned_pair_count(), 1);
        assert_eq!(restored.planned_crossing_loss, 0.0);
    }

    /// Probe-guided search helper (a), no-existing-guidance case
    /// (lidar-pure): the temporary guidance contains only the probe's own
    /// pairs, and the guidance is cleared (not left as `Some`) once the
    /// closure returns.
    #[test]
    fn probe_guided_guidance_with_no_existing_guidance_builds_probe_pairs_only_and_clears_after() {
        let mut router = small_test_router();
        assert!(router.crossing_context.guidance().is_none());

        let observed = router.with_probe_guided_guidance(4, &[5, 6], |router| {
            let guidance = router
                .crossing_context
                .guidance()
                .expect("temporary guidance must be installed");
            (
                guidance.is_planned_pair(4, 5),
                guidance.is_planned_pair(4, 6),
                guidance.planned_pair_count(),
                guidance.planned_crossing_loss,
            )
        });
        assert_eq!(observed, (true, true, 2, 0.0));
        assert!(
            router.crossing_context.guidance().is_none(),
            "no guidance existed before, so none must remain after"
        );
    }

    /// Probe-guided search helper (b): under the temporary guidance,
    /// `crossing_search_config`'s per-partner override
    /// (`crossing_loss_override`) must be `Some(0.0)` for the probe's
    /// partners (5 and 6) and `None` for an unrelated partner (7) -- the
    /// same override mechanism S1's
    /// `planned_partner_crossing_uses_its_own_price_and_unplanned_pays_crossing_loss`
    /// (`astar.rs`) exercises directly on `CrossingSearchConfig`, checked
    /// here one level up where guidance is actually applied.
    #[test]
    fn crossing_search_config_prices_probe_guided_partners_at_zero_and_leaves_others_unpriced() {
        let mut router = small_test_router();
        router
            .crossing_context
            .set_guidance(CrossingGuidance::new(&[(1, 2)], 0.0));

        let partner = |net_id: u64| CrossingSearchPartner {
            net_id,
            waypoints: Vec::new(),
            target_terminal_bump_guard: None,
            crossing_loss_override: None,
            single_discounted_crossing: false,
        };
        let crossing_cfg = CrossingConfig {
            enabled: true,
            crossing_loss: 200.0,
            ..CrossingConfig::default()
        };

        let observed = router.with_probe_guided_guidance(4, &[5, 6], |router| {
            let config = router.crossing_search_config(
                4,
                vec![partner(5), partner(6), partner(7)],
                &crossing_cfg,
                State::new(0, 0, 0),
                None,
                None,
                None,
            );
            config
                .partners
                .into_iter()
                .map(|partner| (partner.net_id, partner.crossing_loss_override))
                .collect::<Vec<_>>()
        });
        assert_eq!(observed, vec![(5, Some(0.0)), (6, Some(0.0)), (7, None)]);
    }

    /// Config path (replacing `PHOTONIC_ROUTER_MAX_DENSE_STATES`):
    /// `RouterConfig.search.max_dense_states` overrides `AStarConfig`'s own
    /// default only when it is `Some` and positive.
    #[test]
    fn router_config_max_dense_states_overrides_only_when_positive() {
        let default = AStarConfig::default().max_dense_states;
        assert_eq!(default, 100_000_000);
        let astar_cfg = PyAStarConfig::new(
            10000,
            1.0,
            0,
            true,
            None,
            true,
            12,
            0.35,
            3,
            true,
            0.5,
            10_000_000,
            false,
            0.0,
            0.0,
            0,
            false,
            false,
            "library".to_string(),
            "distance".to_string(),
            1.0,
        );
        let primitive_cfg = PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true);

        let unset = RouterConfig::default();
        assert_eq!(
            astar_config_from_py(&astar_cfg, &primitive_cfg, None, None, None, None, &unset)
                .unwrap()
                .max_dense_states,
            default
        );

        let zero = RouterConfig {
            search: SearchOverrides {
                max_dense_states: Some(0),
                ..SearchOverrides::default()
            },
            ..RouterConfig::default()
        };
        assert_eq!(
            astar_config_from_py(&astar_cfg, &primitive_cfg, None, None, None, None, &zero)
                .unwrap()
                .max_dense_states,
            default
        );

        let overridden = RouterConfig {
            search: SearchOverrides {
                max_dense_states: Some(30_000_000),
                ..SearchOverrides::default()
            },
            ..RouterConfig::default()
        };
        assert_eq!(
            astar_config_from_py(
                &astar_cfg,
                &primitive_cfg,
                None,
                None,
                None,
                None,
                &overridden
            )
            .unwrap()
            .max_dense_states,
            30_000_000
        );
    }

    #[test]
    fn crossing_free_search_flag_skips_the_lidar_pure_crossing_search() {
        // The flag's observable effect on a fresh route: the vertical net
        // can only reach its target across net 1. Crossing-free, its search
        // is the ordinary A* (fails with the plain "No route found ..."
        // diagnostics, nothing committed, no crossing event); with the flag
        // off the same call reaches the deferred lidar-pure crossing search
        // ("No legal LiDAR crossing route found" -- this 60x60 fixture is
        // too tight for a legal crossing at the default crossing config,
        // same as `crossing_conflict_fixture`'s own NOTE; the search's
        // *dispatch* is what this test pins).
        let (mut router, mut batch, vertical) = single_horizontal_crossing_fixture();

        router.negotiated_crossing_free_search = true;
        let crossing_free_outcome = router.try_plain_normal_route(
            &mut batch,
            &vertical,
            FIXTURE_CLEARANCE_RADIUS_CELLS,
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            false,
        );
        router.negotiated_crossing_free_search = false;
        let crossing_free_error = batch
            .attempts
            .last()
            .and_then(|a| a.error.clone())
            .unwrap_or_default();
        assert!(
            matches!(crossing_free_outcome, PlainRouteOutcome::NotResolved),
            "with the crossing-free flag the vertical net must not route through net 1"
        );
        assert!(
            crossing_free_error.starts_with("No route found"),
            "the crossing-free search is the ordinary A*: {crossing_free_error}"
        );
        assert!(
            router.obstacle_map.get_net_cells(2).is_none(),
            "a refused crossing-free search commits nothing"
        );
        assert!(
            !router.crossing_events.iter().any(|event| event.net_id == 2),
            "a refused crossing-free search registers no crossing event"
        );

        let priced_outcome = router.try_plain_normal_route(
            &mut batch,
            &vertical,
            FIXTURE_CLEARANCE_RADIUS_CELLS,
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
            false,
        );
        let priced_error = batch
            .attempts
            .last()
            .and_then(|a| a.error.clone())
            .unwrap_or_default();
        assert!(
            matches!(priced_outcome, PlainRouteOutcome::Routed)
                || priced_error == "No legal LiDAR crossing route found",
            "without the flag the same call dispatches the lidar-pure crossing search: {priced_error}"
        );
    }

    #[test]
    fn try_lidar_direct_crossing_subset_partner_filter_restricts_the_candidates_tried() {
        // Milestone 5 part 3 of
        // `.agent/execplans/2026-09-14-lidar-style-negotiated-ripup-endgame.md`:
        // `partner_filter` must actually restrict which committed partners
        // this sweep tries, not just reorder or prefer them. Net 4
        // (vertical)'s raw candidate set from `crossing_partner_lookup_set_for_route`
        // in this fixture is exactly {1, 2, 3} (all three other committed
        // nets are within its window); none of the three can resolve the
        // net via this mechanism alone (net 3's crossing is illegal by
        // construction -- "not_perpendicular", see
        // `crossing_conflict_fixture_blocks_the_vertical_net_with_a_non_perpendicular_crossing`
        // above -- and the vertical net structurally has to cross all of
        // 1, 2 and 3, which this one-partner-at-a-time sweep can never do
        // in a single attempt), so success/failure of the whole sweep
        // cannot distinguish "tried" from "not tried" here. Whether an
        // attempt was actually made can still be observed directly: this
        // function costs wall time into `batch.timings.repair_failed_net_wall_us`
        // (with `collect_native_timing=true`) for every partner it tries,
        // and only for those, before ever computing a route -- a filter
        // whose ids don't overlap the raw candidate set at all must leave
        // that counter at exactly zero (the sweep returns immediately, per
        // the `if local_partner_ids.is_empty()` guard, without trying
        // anything), while a filter that keeps one real candidate must
        // leave it strictly positive (one real search was attempted).
        let (mut router, jobs) = crossing_conflict_fixture();
        let vertical = jobs[3].clone();
        assert_eq!(vertical.net_id, 4);
        let order_by_id: FxHashMap<u64, usize> = jobs
            .iter()
            .enumerate()
            .map(|(index, job)| (job.net_id, index))
            .collect();

        let mut batch = RepairBatchState {
            final_routes: FxHashMap::default(),
            attempts: Vec::new(),
            repair_trace: Vec::new(),
            repair_count: 0,
            failed_net_id: None,
            failed_error: None,
            retried_source_layers: FxHashSet::default(),
            timings: NativeBatchTimings::default(),
            trace_last_route_start: None,
            deferred_job_indices: Vec::new(),
            deferred_count: 0,
            last_rejected_commit_partners: Vec::new(),
        };
        for net_id in [1u64, 2, 3] {
            batch.final_routes.insert(net_id, empty_test_route());
        }

        let source_state = State::new(vertical.source.x, vertical.source.y, vertical.source.angle);
        let target_state = State::new(vertical.target.x, vertical.target.y, vertical.target.angle);
        let raw_candidates = router.crossing_partner_lookup_set_for_route(
            vertical.net_id,
            source_state,
            target_state,
        );
        assert_eq!(
            raw_candidates,
            [1u64, 2, 3].into_iter().collect::<FxHashSet<u64>>(),
            "test assumption: this fixture's raw candidate set for net 4 is {{1, 2, 3}}"
        );

        let filtered_to_absent_id = router
            .try_lidar_direct_crossing_subset(
                &mut batch,
                &vertical,
                &order_by_id,
                FIXTURE_CLEARANCE_RADIUS_CELLS,
                Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                true,
                false,
                Some(&[999u64]),
            )
            .expect("subset search must not itself error");
        assert!(matches!(
            filtered_to_absent_id,
            LidarDirectCrossingOutcome::NotResolved
        ));
        assert_eq!(
            batch.timings.repair_failed_net_wall_us, 0,
            "an id absent from the raw candidate set must leave the candidate              set empty and try nothing at all"
        );

        let filtered_to_one_real_candidate = router
            .try_lidar_direct_crossing_subset(
                &mut batch,
                &vertical,
                &order_by_id,
                FIXTURE_CLEARANCE_RADIUS_CELLS,
                Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                Some(FIXTURE_CLEARANCE_RADIUS_CELLS),
                true,
                false,
                Some(&[1u64]),
            )
            .expect("subset search must not itself error");
        assert!(matches!(
            filtered_to_one_real_candidate,
            LidarDirectCrossingOutcome::NotResolved
        ));
        assert!(
            batch.timings.repair_failed_net_wall_us > 0,
            "net 1 is a real member of the raw candidate set and must be              tried (spending wall time on a real search), even though this              one-partner sweep still cannot resolve the net through it alone"
        );
    }

    #[test]
    fn route_port_footprint_cells_matches_directional_box_geometry() {
        let grid = StaticGridSpec {
            width: 12,
            height: 12,
            grid_size_um: 1.0,
            origin: (0.0, 0.0),
            die_bbox: (0.0, 0.0, 12.0, 12.0),
        };

        let east = route_port_footprint_cells(&grid, 2.0, 10.0, Some(0.0), 3, 1);
        let expected_east: FxHashSet<CellKey> = [
            (3, 9),
            (3, 10),
            (3, 11),
            (4, 9),
            (4, 10),
            (4, 11),
            (5, 9),
            (5, 10),
            (5, 11),
            (6, 9),
            (6, 10),
            (6, 11),
        ]
        .into_iter()
        .map(|(x, y)| pack_xy(x, y))
        .collect();
        assert_eq!(east, expected_east);

        let north = route_port_footprint_cells(&grid, 5.0, 5.0, Some(90.0), 2, 1);
        let expected_north: FxHashSet<CellKey> = [
            (4, 6),
            (4, 7),
            (4, 8),
            (5, 6),
            (5, 7),
            (5, 8),
            (6, 6),
            (6, 7),
            (6, 8),
        ]
        .into_iter()
        .map(|(x, y)| pack_xy(x, y))
        .collect();
        assert_eq!(north, expected_north);
    }

    #[test]
    fn opened_cells_excluding_keepout_preserves_terminals() {
        let opened = vec![(1, 1), (2, 2), (3, 3), (4, 4)];
        let keepout: FxHashSet<CellKey> = [(2, 2), (3, 3), (4, 4)]
            .into_iter()
            .map(|(x, y)| pack_xy(x, y))
            .collect();
        let filtered = opened_cells_excluding_keepout(
            &opened,
            &keepout,
            PyState::new(2, 2, 0),
            PyState::new(4, 4, 0),
        );

        assert!(filtered.contains(&(2, 2)));
        assert!(filtered.contains(&(4, 4)));
        assert!(!filtered.contains(&(3, 3)));
        assert!(filtered.contains(&(1, 1)));
    }

    #[test]
    fn dynamic_clearance_exempt_cells_cover_the_opened_approach_at_both_ports() {
        let grid = PyGridSpec::new(60, 40, 1.0, 0.0, 0.0).unwrap();
        let grid = static_grid_from_py_grid(&grid);
        // Source approach opened 5 cells out along +x; target approach opened
        // 8 cells back (the route arrives heading +x, so its run-in lies at
        // smaller x).
        let opened: FxHashSet<CellKey> = (10..=14)
            .map(|x| pack_xy(x, 10))
            .chain((22..=30).map(|x| pack_xy(x, 10)))
            .collect();
        let source = PyState {
            x: 10,
            y: 10,
            angle: 0,
        };
        let target = PyState {
            x: 30,
            y: 10,
            angle: 0,
        };
        let cells = route_dynamic_clearance_exempt_cells(&grid, &opened, source, target, 2, 3);
        let has = |x: i32, y: i32| cells.contains(&pack_xy(x, y));
        // Source corridor: step cells x=10..14 (the opened run), keepout
        // half-width 2, the last step cell inflated by the radius too.
        for x in 10..=16 {
            for y in 8..=12 {
                assert!(has(x, y), "source corridor missing ({x},{y})");
            }
        }
        assert!(!has(17, 10));
        assert!(!has(12, 13));
        // Target corridor: step cells x=22..30 back from the port, inflated.
        for x in 20..=30 {
            for y in 8..=12 {
                assert!(has(x, y), "target corridor missing ({x},{y})");
            }
        }
        assert!(!has(19, 10));
        // The endpoint box, and nothing beyond it, on the far side of a port.
        assert!(has(32, 10));
        assert!(!has(33, 10));
        assert!(!has(7, 10));

        let bare = route_dynamic_clearance_exempt_cells(&grid, &opened, source, target, 0, 0);
        // Radius 0: the axis line itself, exactly the opened run.
        assert!(bare.contains(&pack_xy(10, 10)));
        assert!(bare.contains(&pack_xy(14, 10)));
        assert!(!bare.contains(&pack_xy(15, 10)));
        assert!(bare.contains(&pack_xy(22, 10)));
        assert!(!bare.contains(&pack_xy(21, 10)));
        assert!(!bare.contains(&pack_xy(10, 11)));
    }

    #[test]
    fn dynamic_clearance_exempt_corridor_is_never_shorter_than_the_minimum() {
        let grid = PyGridSpec::new(60, 40, 1.0, 0.0, 0.0).unwrap();
        let grid = static_grid_from_py_grid(&grid);
        // Nothing opened at all: the port lane minimum still applies.
        let opened: FxHashSet<CellKey> = FxHashSet::default();
        let source = PyState {
            x: 10,
            y: 10,
            angle: 0,
        };
        let target = PyState {
            x: 50,
            y: 30,
            angle: 2,
        };
        let cells = route_dynamic_clearance_exempt_cells(&grid, &opened, source, target, 1, 4);
        let has = |x: i32, y: i32| cells.contains(&pack_xy(x, y));
        assert!(has(13, 10));
        assert!(has(14, 10)); // inflation of the last step cell (x=13)
        assert!(!has(15, 10));
        // Target reached heading +y: its run-in lies at smaller y.
        assert!(has(50, 27));
        assert!(has(50, 26));
        assert!(!has(50, 25));
    }

    #[test]
    fn dynamic_clearance_exempt_corridor_follows_the_arrival_heading() {
        let grid = PyGridSpec::new(60, 40, 1.0, 0.0, 0.0).unwrap();
        let grid = static_grid_from_py_grid(&grid);
        let opened: FxHashSet<CellKey> = (20..=25)
            .map(|x| pack_xy(x, 10))
            .chain((25..=30).map(|y| pack_xy(5, y)))
            .collect();
        // Target reached heading -x: the route comes from larger x.
        let source = PyState {
            x: 5,
            y: 30,
            angle: 6,
        };
        let target = PyState {
            x: 20,
            y: 10,
            angle: 4,
        };
        let cells = route_dynamic_clearance_exempt_cells(&grid, &opened, source, target, 1, 3);
        let has = |x: i32, y: i32| cells.contains(&pack_xy(x, y));
        assert!(has(25, 10));
        assert!(has(26, 10)); // inflation of the last opened step cell
        assert!(!has(27, 10));
        assert!(!has(17, 10));
        // Source leaves heading -y: the corridor runs toward smaller y.
        assert!(has(5, 25));
        assert!(has(5, 24));
        assert!(!has(5, 23));
        assert!(!has(5, 33));
    }

    #[test]
    fn search_opened_cells_keep_exempt_cells_under_a_foreign_halo_but_not_a_core() {
        let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
        let mut router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(1.0, 2, 4, 2, 1.0, false),
            PyAStarConfig::new(
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
            None,
        );
        // Net 1 runs along y=11; with keepout radius 1 its halo covers y=10.
        let core: Vec<(i32, i32)> = (10..=30).map(|x| (x, 11)).collect();
        let mut blocked = Vec::new();
        for &(x, y) in &core {
            for dy in -1..=1 {
                blocked.push((x, y + dy));
            }
        }
        assert!(router
            .obstacle_map
            .commit_route_with_clearance_and_allowed_core_overlaps(
                1,
                &core,
                &blocked,
                &[],
                &FxHashSet::default(),
            ));
        // Net 2's opening: its approach along y=10 plus one cell on net 1's
        // core row; both are under net 1's dynamic footprint.
        let opened: FxHashSet<CellKey> = (10..=30)
            .map(|x| pack_xy(x, 10))
            .chain(std::iter::once(pack_xy(20, 11)))
            .collect();
        let exempt: FxHashSet<CellKey> = opened.clone();
        let source = State::new(5, 10, 0);
        let target = State::new(30, 10, 0);

        let without = router
            .opened_cells_without_dynamic_overlap(&opened, source, target, None)
            .expect("the halo strips every opened cell but the target");
        assert_eq!(without.len(), 1);
        assert!(without.contains(&pack_xy(30, 10)));

        let with = router
            .opened_cells_without_dynamic_overlap(&opened, source, target, Some(&exempt))
            .expect("the core cell is still stripped");
        assert_eq!(with.len(), opened.len() - 1);
        assert!(with.contains(&pack_xy(15, 10)));
        assert!(!with.contains(&pack_xy(20, 11)));
    }

    #[test]
    fn upfront_collision_crossing_partner_ids_plain_first_defers_in_lidar_pure_mode() {
        let mut router = crossing_partner_discovery_test_router();
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            allow_only_expected_pairs: false,
            ..CrossingConfig::default()
        });

        let partners = router.upfront_collision_crossing_partner_ids(
            3,
            State::new(5, 5, 0),
            State::new(10, 10, 0),
            CollisionCrossingTryOrder::PlainFirst,
        );

        assert!(
            partners.is_empty(),
            "PlainFirst must defer the lidar-pure collision-crossing lookup to a \
             second, later call after the caller's own plain attempt fails, not \
             compute it upfront"
        );
    }

    #[test]
    fn upfront_collision_crossing_partner_ids_crossing_first_returns_full_map_in_lidar_pure_mode() {
        let mut router = crossing_partner_discovery_test_router();
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            allow_only_expected_pairs: false,
            ..CrossingConfig::default()
        });

        let partners = router.upfront_collision_crossing_partner_ids(
            3,
            State::new(5, 5, 0),
            State::new(10, 10, 0),
            CollisionCrossingTryOrder::CrossingFirst,
        );

        assert!(
            partners.contains(&1) && partners.contains(&2),
            "CrossingFirst must consult the full, unwindowed lidar-pure partner \
             set immediately, including net 2 well outside the windowed lookup's \
             bounding box: {partners:?}"
        );
    }

    #[test]
    fn upfront_collision_crossing_partner_ids_uses_declared_pairs_in_expected_pairs_hybrid_mode() {
        // allow_only_expected_pairs=true with use_collision_crossing_routing=true
        // (the "collision-crossing mechanics restricted to expected pairs"
        // hybrid mode Milestone 0 found) is *not* windowed at all: since
        // lidar_pure_crossing_enabled() is false whenever
        // allow_only_expected_pairs=true regardless of
        // use_collision_crossing_routing, this mode always falls through to
        // the topology-declared expected-pairs set, for both try orders,
        // with no deferred second stage. Confirmed directly here after this
        // test's own first draft (assuming a windowed lookup, matching the
        // non-lidar-pure-but-collision-crossing case's naive reading)
        // failed against the real implementation -- corrected rather than
        // adjusted to force a pass, per this repository's own testing
        // discipline of trusting real failures over first assumptions. Net
        // 1 is declared as net 3's only expected partner; net 2 is
        // committed but never declared, so it must not appear even though
        // both are equally "physically present" in the obstacle map.
        let mut router = crossing_partner_discovery_test_router();
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            allow_only_expected_pairs: true,
            ..CrossingConfig::default()
        });
        router
            .crossing_context
            .replace_constraints(vec![CrossingConstraint {
                net_id: 3,
                partner_net_id: 1,
                level: 0,
                source_depth: 0,
                target_depth: 0,
            }]);

        for try_order in [
            CollisionCrossingTryOrder::PlainFirst,
            CollisionCrossingTryOrder::CrossingFirst,
        ] {
            let partners = router.upfront_collision_crossing_partner_ids(
                3,
                State::new(5, 5, 0),
                State::new(10, 10, 0),
                try_order,
            );
            assert!(
                partners.contains(&1),
                "expected net 1 (declared partner) for try_order {try_order:?}: {partners:?}"
            );
            assert!(
                !partners.contains(&2),
                "did not expect net 2 (committed but not a declared partner) for \
                 try_order {try_order:?}: {partners:?}"
            );
        }
    }

    #[test]
    fn upfront_collision_crossing_partner_ids_empty_when_collision_crossing_routing_disabled() {
        let mut router = crossing_partner_discovery_test_router();
        router.set_collision_crossing_routing(false);
        router.crossing_context.set_config(CrossingConfig {
            enabled: true,
            allow_only_expected_pairs: false,
            ..CrossingConfig::default()
        });

        for try_order in [
            CollisionCrossingTryOrder::PlainFirst,
            CollisionCrossingTryOrder::CrossingFirst,
        ] {
            let partners = router.upfront_collision_crossing_partner_ids(
                3,
                State::new(5, 5, 0),
                State::new(10, 10, 0),
                try_order,
            );
            assert!(
                partners.is_empty(),
                "collision-crossing routing is disabled, so no partner set should \
                 be computed regardless of try_order {try_order:?}: {partners:?}"
            );
        }
    }

    #[test]
    fn committed_partners_intersecting_route_names_the_crossed_net_with_crossings_disabled() {
        let (mut router, batch, vertical) = single_horizontal_crossing_fixture();
        router.set_collision_crossing_routing(false);
        router.crossing_context.set_config(CrossingConfig {
            enabled: false,
            ..CrossingConfig::default()
        });
        assert!(batch.final_routes.contains_key(&1));
        // A straight vertical probe route through the committed horizontal net 1.
        let probe_route = RouteResult {
            states: Vec::new(),
            primitives: Vec::new(),
            cells: (0..=59).map(|y| (30, y)).collect(),
            compressed_waypoints: vec![(30, 0), (30, 59)],
            total_length_um: 59.0,
            total_cost: 59.0,
            requested_target: State::new(30, 59, 2),
            reached_target: State::new(30, 59, 2),
            stats: RouteSearchStats::default(),
        };
        let partners =
            router.committed_partners_intersecting_route(vertical.net_id, &probe_route, None, None);
        assert_eq!(
            partners,
            vec![1],
            "the horizontal net is the one the probe must cross"
        );
        assert!(
            router
                .crossing_violations_for_route_with_ports(vertical.net_id, &probe_route, None, None, None, true)
                .is_empty(),
            "the legality helper stays silent with crossings disabled -- which is why the probe needs the geometric list"
        );
    }

    #[test]
    fn simple_route_and_describe() {
        let grid = PyGridSpec::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let router = PyPhotonicRouter::new(
            grid,
            PyPrimitiveLibraryConfig::new(0.5, 1, 4, 2, 1.0, true),
            PyAStarConfig::new(
                10000,
                1.0,
                0,
                true,
                None,
                true,
                12,
                0.35,
                3,
                true,
                0.5,
                10_000_000,
                false,
                0.0,
                0.0,
                0,
                false,
                false,
                "library".to_string(),
                "distance".to_string(),
                1.0,
            ),
            None,
        );
        assert!(!router.primitives.get_primitives_for_angle(0).is_empty());
    }
}
