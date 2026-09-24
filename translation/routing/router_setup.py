"""Phase 2: the PyO3 router, its configuration objects and the static-cell bookkeeping."""

from __future__ import annotations

from typing import Any, Iterable, cast

from translation.route_rust_obstacle_config import _grid_origin_xy
from translation.route_rust_types import (
    OpticalRouteClearancePolicy,
    _as_float,
    bend_radius_cells_from_um,
)
from translation.routing import timing
from translation.routing.settings import SessionSettings
from translation.routing.state import SessionState

DENSE_OBSTACLE_CELL_CAP_MARGIN = 2


def _fanout_int_or_default(value: int | None, default: int) -> int:
    """`value` is a resolved `FanoutAccessConfig` field: `None` (unset)
    keeps the site's own `default`; validation (non-negative, raises)
    already happened when the config was parsed from the environment."""
    return int(default) if value is None else int(value)


def configure_router_and_grid(
    settings: SessionSettings, state: SessionState, obstacle_map: Any
) -> None:
    """Build the Rust grid/primitive/A* configs, construct `self.router`, and extract raw static geometry.

    Tunes `self.astar_cfg` (JPS4, heuristic mode/weight, bend weight, heap tie-breaker,
    proactive congestion, routing-window sizing) based on `self.allow_45_degree_turns`,
    `self.crossing_mode`, and related settings, resolves `self.clearance_policy` and the
    block/commit/core radii, constructs `self.router` (`PyPhotonicRouter`), and extracts
    `self.raw_static_cells`/`self.raw_static_rects_for_openings`/
    `self.heater_opening_rects_for_openings` from `obstacle_map` for later phases to
    consume when computing port openings and keepouts.
    """
    t_router_setup_start = timing.pipeline_timer_start(settings, state)
    state.origin_x_um, state.origin_y_um = _grid_origin_xy(state.grid)
    if settings.config.diagnostics.trace_grid:
        # Path-investigation harness: the grid<->um mapping must come from
        # the tool, never from a hand conversion (see
        # .agent/PATH_INVESTIGATION_HARNESS.md). um = origin + (cell + 0.5) * grid_size.
        print(
            f"      - grid: origin=({state.origin_x_um:.3f}, {state.origin_y_um:.3f}) um "
            f"size={float(state.grid.grid_size_um):.3f} um "
            f"cells={int(state.grid.width)}x{int(state.grid.height)}",
            flush=True,
        )
    grid_spec = state.rust_backend.GridSpec(
        int(state.grid.width),
        int(state.grid.height),
        float(state.grid.grid_size_um),
        state.origin_x_um,
        state.origin_y_um,
    )
    state.bend_radius_cells = bend_radius_cells_from_um(
        settings.bend_radius_um,
        grid_size_um=float(state.grid.grid_size_um),
    )
    state.primitive_cfg = state.rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(state.grid.grid_size_um),
        bend_radius_cells=state.bend_radius_cells,
        allow_45_degree_turns=settings.allow_45_degree_turns,
    )
    state.bend_radius_cells = int(state.primitive_cfg.bend_radius_cells)
    state.astar_cfg = state.rust_backend.AStarConfig(
        max_iterations=int(settings.max_iterations)
    )
    # The crossing search (lidar modes) builds a dense grid over the full
    # routing bounds and panics when that exceeds the kernel's cap
    # (default 10M cells; benes_64x64 needs 29M, multiportmmi_64x64 83M,
    # 2026-09-13). Size the cap from the obstacle map so the whole die
    # always fits; dies below the default keep the default.
    state.astar_cfg.max_dense_obstacle_cells = dense_obstacle_cell_cap(
        int(state.grid.width), int(state.grid.height), int(state.astar_cfg.max_dense_obstacle_cells)
    )
    state.astar_cfg.enable_simple_routes = bool(settings.enable_simple_routes)
    state.astar_cfg.enable_jps4 = bool(settings.enable_jps4)
    state.astar_cfg.use_indexed_heap = bool(
        settings.use_indexed_heap or settings.allow_45_degree_turns
    )
    state.astar_cfg.collect_detailed_timing = bool(
        settings.debug_timing
        or settings.collect_route_stats
        or settings.collect_attempt_diagnostics
    )
    state.astar_cfg.primitive_ordering = str(settings.primitive_ordering)
    effective_heuristic_mode = str(settings.heuristic_mode)
    if settings.allow_45_degree_turns and effective_heuristic_mode == "heading_aware":
        effective_heuristic_mode = "diagonal_aware"
    state.astar_cfg.heuristic_mode = effective_heuristic_mode
    min_heuristic_weight = float(settings.config.search.min_heuristic_weight)
    if (
        settings.allow_45_degree_turns
        and not settings.enable_crossings
        and min_heuristic_weight > 1.0
        and hasattr(state.astar_cfg, "max_iterations")
    ):
        # This cap was sized for Weighted A* (min weight > 1.0), which
        # reaches targets in few expansions; admissible search (1.0)
        # legitimately needs the caller's full iteration budget
        # (heater_s_mod's mmi_extra_3->mmi_extra_4 net exhausts 50k).
        # The condition reads `not settings.enable_crossings` because it
        # used to read `not collision_crossing_mode`, and that was exactly
        # `enable_crossings and is_collision_mode(crossing_mode)`: with the
        # collision mode gone (Milestone 8 Slice B) the mode half is always
        # false, so the negation collapses to the crossings flag alone.
        state.astar_cfg.max_iterations = min(int(state.astar_cfg.max_iterations), 50_000)
    if settings.allow_45_degree_turns and hasattr(state.astar_cfg, "heuristic_weight"):
        # 1.0 is admissible A*: cheapest paths, no weighted-search
        # geometry artifacts (chicanes, overshoot detours), and the only
        # weight at which multiportmmi_16x16 routes completely. 1.25 is
        # the old Weighted A* behavior (fewer expanded states, up to 25%
        # suboptimal paths); benes_16x16 still pins it in its
        # STABLE_ROUTING_ENV until its no-candidate endpoint-correction
        # hole is fixed -- see
        # .agent/execplans/2026-08-31-admissible-astar-heuristic-45-degree.md.
        state.astar_cfg.heuristic_weight = max(
            float(state.astar_cfg.heuristic_weight), min_heuristic_weight
        )
    if settings.allow_45_degree_turns and hasattr(state.astar_cfg, "bend_weight"):
        # A bend costs bend_weight per angle-eighth of turn (see
        # src/primitives.rs's bend_cost field); boosting it keeps 45-degree
        # A* from spending work on short zig-zag variants. This applies
        # uniformly regardless of crossing_mode -- crossing-awareness only
        # changes whether/what a *crossing* costs (see crossing_loss,
        # translation/route_rust_crossing_plan.py), not how bends are priced.
        # Experiment gate for the heuristics audit
        # (.agent/execplans/2026-08-31-heuristics-inventory.md): the
        # magnitude 12.0 has no recorded derivation; the env var lets the
        # neutral-value ladder run without a code edit. Default unchanged.
        min_bend_weight = float(settings.config.search.min_bend_weight)
        state.astar_cfg.bend_weight = max(float(state.astar_cfg.bend_weight), min_bend_weight)
    effective_heap_tie_breaker = str(settings.heap_tie_breaker)
    if settings.allow_45_degree_turns and effective_heap_tie_breaker == "smaller_g":
        effective_heap_tie_breaker = "larger_g"
    # Experiment gate (same audit): the 45-degree-only tie-break flip is
    # unjustified in-repo; an explicit env value overrides the outcome.
    env_tie_breaker = settings.config.search.heap_tie_breaker
    if env_tie_breaker is not None:
        effective_heap_tie_breaker = env_tie_breaker
    state.astar_cfg.heap_tie_breaker = effective_heap_tie_breaker
    if hasattr(state.astar_cfg, "proactive_congestion_weight"):
        state.astar_cfg.proactive_congestion_weight = float(
            settings.proactive_congestion_weight
        )
    if hasattr(state.astar_cfg, "proactive_congestion_radius_cells"):
        state.astar_cfg.proactive_congestion_radius_cells = int(
            settings.proactive_congestion_radius_cells
        )
    if settings.routing_window_scale is not None:
        state.astar_cfg.routing_window_scale = float(settings.routing_window_scale)

    state.route_clearance_um = max(
        0.0,
        _as_float(getattr(state.resolved_obstacle_config, "clearance_um", 0.0), 0.0),
    )
    state.clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
        route_width_um=float(settings.route_width_um),
        grid_size_um=float(state.grid.grid_size_um),
        route_clearance_um=state.route_clearance_um,
    )
    state.block_radius_cells = (
        state.clearance_policy.dynamic_obstacle_search_expansion_radius_cells
    )
    state.commit_radius_cells = state.clearance_policy.dynamic_route_commit_keepout_radius_cells
    state.core_commit_radius_cells = state.clearance_policy.dynamic_route_core_radius_cells
    routing_window_min_margin_cells = max(
        int(getattr(state.astar_cfg, "routing_window_min_margin_cells", 12)),
        int((2 * state.bend_radius_cells) + state.commit_radius_cells + 2),
    )
    state.astar_cfg.routing_window_min_margin_cells = max(
        int(getattr(state.astar_cfg, "routing_window_min_margin_cells", 12)),
        routing_window_min_margin_cells,
    )
    state.astar_cfg.simple_route_max_offset_cells = max(
        int(getattr(state.astar_cfg, "simple_route_max_offset_cells", 96)),
        int(12 * state.bend_radius_cells + 2 * state.commit_radius_cells),
    )
    state.router = state.rust_backend.PyPhotonicRouter(
        grid_spec,
        state.primitive_cfg,
        state.astar_cfg,
        settings.router_config.to_rust(state.rust_backend),
    )
    if hasattr(state.router, "set_route_width_um"):
        # The commit validation's parallel-overlap check needs the
        # physical waveguide width (see py_router.rs `route_width_um`).
        state.router.set_route_width_um(float(settings.route_width_um))
    timing.record_pipeline_timing(settings, state, "router_setup", t_router_setup_start)

    state.port_lane_length_cells = max(3, 2 * state.bend_radius_cells + 2)
    state.port_lane_half_width_cells = max(
        1, state.bend_radius_cells + state.commit_radius_cells + 1
    )
    # Separate override scoped to dense-source-fanout instances only
    # (`_is_dense_source_fanout_instance`) -- e.g. a multi-port MMI
    # splitter with many stubbed ports stacked close together, where the
    # flat per-port reservation above overlaps heavily and merges into
    # one large blocked region. Every *other* port (e.g. a heater port
    # on the same benchmark, confirmed via a real regression to need
    # the full default margin for its own endpoint correction) is
    # unaffected regardless of these values, so this stays scoped
    # rather than becoming a second general default.
    #
    # Both default to `0` (no reservation at all): at a stub, the
    # runway/keepout exists to protect a port whose own waveguide isn't
    # committed yet, but a dense-source-fanout stub's waveguide *is*
    # already committed static geometry by the time this reservation
    # would matter. A crossing (or anything else) placed directly at a
    # stub's exit cell is governed by the crossing-legality rules
    # (`crossing_half_size_cells`, `min_straight_cells_per_crossing`),
    # not by this reservation, so there is nothing left for it to
    # protect at stubs specifically. Validated on `multiportmmi_8x8`'s
    # stable baseline with both knobs at `0` (previously `half_width=2`,
    # `length=port_lane_length_cells`): still routes 111/111, 0 errors,
    # 0 self-intersections, 0 cross-net overlaps.
    state.stub_port_lane_length_cells = _fanout_int_or_default(
        settings.config.fanout.stub_port_lane_length_cells,
        0,
    )
    state.stub_port_lane_half_width_cells = _fanout_int_or_default(
        settings.config.fanout.stub_port_lane_half_width_cells,
        0,
    )

    port_open_radius_um = _as_float(
        getattr(state.resolved_obstacle_config, "port_open_radius_um", 0.5),
        0.5,
    )

    raw_blocked_obj: object
    if hasattr(obstacle_map, "raw_blocked_cells"):
        raw_blocked_obj = getattr(obstacle_map, "raw_blocked_cells")
    else:
        raw_blocked_obj = obstacle_map.blocked_cells
    raw_blocked_cells = cast(Iterable[tuple[int, int]], raw_blocked_obj)
    state.raw_static_cells = {(int(cell[0]), int(cell[1])) for cell in raw_blocked_cells}
    state.static_blocked_cells_before_port_reservations = state.raw_static_cells
    state.raw_static_rects_for_openings: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "raw_static_rects"):
        for rect in cast(
            Iterable[tuple[int, int, int, int]],
            getattr(obstacle_map, "raw_static_rects"),
        ):
            if len(rect) == 4:
                state.raw_static_rects_for_openings.append(
                    (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                )
    blocked_static_rects_for_openings: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "blocked_static_rects"):
        for rect in cast(
            Iterable[tuple[int, int, int, int]],
            getattr(obstacle_map, "blocked_static_rects"),
        ):
            if len(rect) == 4:
                blocked_static_rects_for_openings.append(
                    (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                )
    state.heater_opening_rects_for_openings = (
        state.raw_static_rects_for_openings + blocked_static_rects_for_openings
    )
    state.grid_width = int(state.grid.width)
    state.grid_height = int(state.grid.height)
    state.raw_static_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
    state.heater_opening_rect_ranges_by_y: dict[int, list[tuple[int, int]]] | None = None
    state.raw_static_cells_by_y: dict[int, set[int]] | None = None


def dense_obstacle_cell_cap(grid_width: int, grid_height: int, default_cap: int) -> int:
    """Return the kernel's dense-grid cell cap sized for this obstacle map.

    The crossing hook of the lidar modes rasterizes the full routing bounds;
    the cap must therefore cover the whole map (with a margin for the
    clamped bound expansion), or the kernel refuses the grid and panics.
    Maps that fit the default cap keep it, so small benchmarks are unchanged.
    """
    return max(int(default_cap), DENSE_OBSTACLE_CELL_CAP_MARGIN * int(grid_width) * int(grid_height))
