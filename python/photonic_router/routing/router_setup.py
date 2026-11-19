"""Phase 2: the PyO3 router, its configuration objects and the static-cell bookkeeping."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any, cast

from photonic_router.records.route_rust_obstacle_config import _grid_origin_xy
from photonic_router.records.route_rust_types import (
    OpticalRouteClearancePolicy,
    _as_float,
    bend_radius_cells_from_um,
)
from photonic_router.routing import timing
from photonic_router.routing.products import (
    ClearanceRadii,
    GridFrame,
    KernelSetup,
    ObstacleContext,
    PortLanes,
    RawStaticGeometry,
)
from photonic_router.routing.settings import SessionSettings

DENSE_OBSTACLE_CELL_CAP_MARGIN = 2


def _fanout_int_or_default(value: int | None, default: int) -> int:
    """`value` is a resolved `FanoutAccessConfig` field: `None` (unset)
    keeps the site's own `default`; validation (non-negative, raises)
    already happened when the config was parsed from the environment."""
    return int(default) if value is None else int(value)


def configure_router_and_grid(
    settings: SessionSettings,
    rust_backend: Any,
    context: ObstacleContext,
    timings_s: dict[str, float],
) -> KernelSetup:
    """Phase 2: build the kernel's router and everything derived for it.

    Builds the kernel's grid, primitive and A* configurations and tunes the
    A* one from the settings (heuristic mode, bend weight, heap tie-breaker,
    proactive congestion, routing window), derives the clearance radii and
    the port lane sizes, constructs the `PyPhotonicRouter`, and reads the raw
    static geometry out of the obstacle map for the port openings of phase 4.

    Args:
        settings: the session's resolved inputs.
        rust_backend: the loaded kernel module.
        context: phase 1's product.
        timings_s: the pipeline timing buckets; "router_setup" is added.
    """
    obstacle_map = context.obstacle_map
    t_router_setup_start = timing.pipeline_timer_start(settings)
    origin_x_um, origin_y_um = _grid_origin_xy(context.grid)
    if settings.config.diagnostics.trace_grid:
        # Path-investigation harness: the grid<->um mapping must come from
        # the tool, never from a hand conversion (see
        # .agent/PATH_INVESTIGATION_HARNESS.md). um = origin + (cell + 0.5) * grid_size.
        print(
            f"      - grid: origin=({origin_x_um:.3f}, {origin_y_um:.3f}) um "
            f"size={float(context.grid.grid_size_um):.3f} um "
            f"cells={int(context.grid.width)}x{int(context.grid.height)}",
            flush=True,
        )
    grid_spec = rust_backend.GridSpec(
        int(context.grid.width),
        int(context.grid.height),
        float(context.grid.grid_size_um),
        origin_x_um,
        origin_y_um,
    )
    bend_radius_cells = bend_radius_cells_from_um(
        settings.bend_radius_um,
        grid_size_um=float(context.grid.grid_size_um),
    )
    primitive_cfg = rust_backend.PrimitiveLibraryConfig(
        grid_size_um=float(context.grid.grid_size_um),
        bend_radius_cells=bend_radius_cells,
        allow_45_degree_turns=settings.allow_45_degree_turns,
    )
    bend_radius_cells = int(primitive_cfg.bend_radius_cells)
    astar_cfg = rust_backend.AStarConfig(max_iterations=int(settings.max_iterations))
    # The crossing search (lidar modes) builds a dense grid over the full
    # routing bounds and panics when that exceeds the kernel's cap
    # (default 10M cells; benes_64x64 needs 29M, multiportmmi_64x64 83M,
    # 2026-09-13). Size the cap from the obstacle map so the whole die
    # always fits; dies below the default keep the default.
    astar_cfg.max_dense_obstacle_cells = dense_obstacle_cell_cap(
        int(context.grid.width), int(context.grid.height), int(astar_cfg.max_dense_obstacle_cells)
    )
    astar_cfg.enable_simple_routes = bool(settings.enable_simple_routes)
    astar_cfg.use_indexed_heap = bool(settings.use_indexed_heap or settings.allow_45_degree_turns)
    astar_cfg.collect_detailed_timing = bool(
        settings.debug_timing
        or settings.collect_route_stats
        or settings.collect_attempt_diagnostics
    )
    astar_cfg.primitive_ordering = str(settings.primitive_ordering)
    effective_heuristic_mode = str(settings.heuristic_mode)
    if settings.allow_45_degree_turns and effective_heuristic_mode == "heading_aware":
        effective_heuristic_mode = "diagonal_aware"
    astar_cfg.heuristic_mode = effective_heuristic_mode
    if settings.allow_45_degree_turns and hasattr(astar_cfg, "bend_weight"):
        # A bend costs bend_weight per angle-eighth of turn (see
        # src/primitives.rs's bend_cost field); boosting it keeps 45-degree
        # A* from spending work on short zig-zag variants. This applies
        # uniformly regardless of crossing_mode -- crossing-awareness only
        # changes whether/what a *crossing* costs (see crossing_loss,
        # python/photonic_router/crossings/route_rust_crossing_plan.py), not how bends are priced.
        # Experiment gate for the heuristics audit
        # (.agent/execplans/2026-08-31-heuristics-inventory.md): the
        # magnitude 12.0 has no recorded derivation; the env var lets the
        # neutral-value ladder run without a code edit. Default unchanged.
        min_bend_weight = float(settings.config.search.min_bend_weight)
        astar_cfg.bend_weight = max(float(astar_cfg.bend_weight), min_bend_weight)
    effective_heap_tie_breaker = str(settings.heap_tie_breaker)
    if settings.allow_45_degree_turns and effective_heap_tie_breaker == "smaller_g":
        effective_heap_tie_breaker = "larger_g"
    # Experiment gate (same audit): the 45-degree-only tie-break flip is
    # unjustified in-repo; an explicit env value overrides the outcome.
    env_tie_breaker = settings.config.search.heap_tie_breaker
    if env_tie_breaker is not None:
        effective_heap_tie_breaker = env_tie_breaker
    astar_cfg.heap_tie_breaker = effective_heap_tie_breaker
    if hasattr(astar_cfg, "proactive_congestion_weight"):
        astar_cfg.proactive_congestion_weight = float(settings.proactive_congestion_weight)
    if hasattr(astar_cfg, "proactive_congestion_radius_cells"):
        astar_cfg.proactive_congestion_radius_cells = int(
            settings.proactive_congestion_radius_cells
        )
    if settings.routing_window_scale is not None:
        astar_cfg.routing_window_scale = float(settings.routing_window_scale)
    # Speed pass S4 (2026-11-05): the window ladder's length and its
    # full-grid fallback, env-only strategy switches; unset keeps the
    # kernel's defaults (3 expansions, fallback on).
    search_tuning = settings.config.search
    if search_tuning.routing_window_max_expansions is not None:
        astar_cfg.routing_window_max_expansions = int(search_tuning.routing_window_max_expansions)
    if search_tuning.routing_window_fallback_full_grid is not None:
        astar_cfg.routing_window_fallback_full_grid = bool(
            search_tuning.routing_window_fallback_full_grid
        )

    route_clearance_um = max(
        0.0,
        _as_float(getattr(context.resolved_obstacle_config, "clearance_um", 0.0), 0.0),
    )
    clearance_policy = OpticalRouteClearancePolicy.from_dimensions(
        route_width_um=float(settings.route_width_um),
        grid_size_um=float(context.grid.grid_size_um),
        route_clearance_um=route_clearance_um,
    )
    block_radius_cells = clearance_policy.dynamic_obstacle_search_expansion_radius_cells
    commit_radius_cells = clearance_policy.dynamic_route_commit_keepout_radius_cells
    core_commit_radius_cells = clearance_policy.dynamic_route_core_radius_cells
    routing_window_min_margin_cells = max(
        int(getattr(astar_cfg, "routing_window_min_margin_cells", 12)),
        int((2 * bend_radius_cells) + commit_radius_cells + 2),
    )
    astar_cfg.routing_window_min_margin_cells = max(
        int(getattr(astar_cfg, "routing_window_min_margin_cells", 12)),
        routing_window_min_margin_cells,
    )
    astar_cfg.simple_route_max_offset_cells = max(
        int(getattr(astar_cfg, "simple_route_max_offset_cells", 96)),
        int(12 * bend_radius_cells + 2 * commit_radius_cells),
    )
    router = rust_backend.PyPhotonicRouter(
        grid_spec,
        primitive_cfg,
        astar_cfg,
        settings.router_config.to_rust(rust_backend),
    )
    if hasattr(router, "set_route_width_um"):
        # The commit validation's parallel-overlap check needs the
        # physical waveguide width (see py_router.rs `route_width_um`).
        router.set_route_width_um(float(settings.route_width_um))
    timing.record_elapsed(settings, timings_s, "router_setup", t_router_setup_start)

    port_lane_length_cells = max(3, 2 * bend_radius_cells + 2)
    port_lane_half_width_cells = max(1, bend_radius_cells + commit_radius_cells + 1)
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
    # (`crossing_half_size_cells` plus the bend run-out),
    # not by this reservation, so there is nothing left for it to
    # protect at stubs specifically. Validated on `multiportmmi_8x8`'s
    # stable baseline with both knobs at `0` (previously `half_width=2`,
    # `length=port_lane_length_cells`): still routes 111/111, 0 errors,
    # 0 self-intersections, 0 cross-net overlaps.
    stub_port_lane_length_cells = _fanout_int_or_default(
        settings.config.fanout.stub_port_lane_length_cells,
        0,
    )
    stub_port_lane_half_width_cells = _fanout_int_or_default(
        settings.config.fanout.stub_port_lane_half_width_cells,
        0,
    )

    raw_blocked_obj: object
    if hasattr(obstacle_map, "raw_blocked_cells"):
        raw_blocked_obj = obstacle_map.raw_blocked_cells
    else:
        raw_blocked_obj = obstacle_map.blocked_cells
    raw_blocked_cells = cast(Iterable[tuple[int, int]], raw_blocked_obj)
    raw_static_cells = {(int(cell[0]), int(cell[1])) for cell in raw_blocked_cells}
    raw_static_rects_for_openings: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "raw_static_rects"):
        for rect in cast(
            Iterable[tuple[int, int, int, int]],
            obstacle_map.raw_static_rects,
        ):
            if len(rect) == 4:
                raw_static_rects_for_openings.append(
                    (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                )
    blocked_static_rects_for_openings: list[tuple[int, int, int, int]] = []
    if hasattr(obstacle_map, "blocked_static_rects"):
        for rect in cast(
            Iterable[tuple[int, int, int, int]],
            obstacle_map.blocked_static_rects,
        ):
            if len(rect) == 4:
                blocked_static_rects_for_openings.append(
                    (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
                )
    heater_opening_rects_for_openings = (
        raw_static_rects_for_openings + blocked_static_rects_for_openings
    )
    return KernelSetup(
        router=router,
        primitive_cfg=primitive_cfg,
        astar_cfg=astar_cfg,
        frame=GridFrame(
            grid_width=int(context.grid.width),
            grid_height=int(context.grid.height),
            origin_x_um=origin_x_um,
            origin_y_um=origin_y_um,
        ),
        radii=ClearanceRadii(
            clearance_policy=clearance_policy,
            route_clearance_um=route_clearance_um,
            bend_radius_cells=bend_radius_cells,
            commit_radius_cells=commit_radius_cells,
            core_commit_radius_cells=core_commit_radius_cells,
            block_radius_cells=block_radius_cells,
        ),
        lanes=PortLanes(
            port_lane_length_cells=port_lane_length_cells,
            port_lane_half_width_cells=port_lane_half_width_cells,
            stub_port_lane_length_cells=stub_port_lane_length_cells,
            stub_port_lane_half_width_cells=stub_port_lane_half_width_cells,
        ),
        static=RawStaticGeometry(
            raw_static_cells=raw_static_cells,
            raw_static_rects_for_openings=raw_static_rects_for_openings,
            heater_opening_rects_for_openings=heater_opening_rects_for_openings,
        ),
    )


def dense_obstacle_cell_cap(grid_width: int, grid_height: int, default_cap: int) -> int:
    """Return the kernel's dense-grid cell cap sized for this obstacle map.

    The crossing hook of the lidar modes rasterizes the full routing bounds;
    the cap must therefore cover the whole map (with a margin for the
    clamped bound expansion), or the kernel refuses the grid and panics.
    Maps that fit the default cap keep it, so small benchmarks are unchanged.
    """
    return max(
        int(default_cap), DENSE_OBSTACLE_CELL_CAP_MARGIN * int(grid_width) * int(grid_height)
    )
