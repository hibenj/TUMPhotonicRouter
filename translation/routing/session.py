"""The routing session shell: construction, the nine-phase `run` and the
module-level `route_nets_rust` entry point.

Milestone 5 Slice 1 moved every phase body and its stage-private helpers into
the sibling stage modules of this package, as module-level functions taking the
session as their first argument, with a one-line forwarder per moved method on
this class. Slice 2b removed those 123 forwarders: a stage calls a helper of its
own module by name and a helper of another stage through an import from that
stage's module, so the only entry points left on the class are `__init__` and
`run`. The pipeline timers moved to `timing.py` for the same reason.

Slice 2b also split what the session carried into the two records a stage now
takes: `self.settings` (frozen `settings.SessionSettings`, Slice 2a) is the run's
inputs, `self.state` (mutable `state.SessionState`) is everything the run
computes -- one documented field per per-run attribute, with the writing and
reading phases named. The class itself owns nothing else: it validates the
keywords, builds the two records and calls the nine phases of `stages.py`, whose
Protocols declare each phase's inputs and its product."""

from __future__ import annotations

import importlib

from pathlib import Path

from gdsfactory.component import Component
from gdsfactory.schematic import Schematic

from photonic_router.config import RoutingConfig

from translation.route_rust_types import RipupRerouteConfig, RustRouteDebugArtifacts

from translation.routing.settings import (
    DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    SessionSettings,
)
from translation.routing.state import SessionState

from translation.routing import (
    obstacle_context,
    router_setup,
    route_jobs as route_jobs_stage,  # `route_jobs` is a local name in `run`
    crossing_plan_stage,
    handoff,
    dispatch,
    finalize,
    verify_repair,
    realize,
)

_sob = importlib.import_module("photonic_router.static_obstacle_builder")
GridSpec = _sob.GridSpec
StaticObstacleMapConfig = _sob.StaticObstacleMapConfig
build_static_obstacle_map = _sob.build_static_obstacle_map
_load_rust_backend = _sob._load_rust_backend


class _RouteNetsRustSession:
    def __init__(
        self,
        unrouted_layout: Component,
        schematic: Schematic,
        *,
        obstacle_config: object | None = None,
        debug_dir: str | Path | None = None,
        debug_prefix: str = "route",
        debug_route_indices: set[int] | None = None,
        debug_stop_after_route_index: int | None = None,
        route_width_um: float = 0.5,
        route_layer: tuple[int, int] = (1, 0),
        allow_45_degree_turns: bool = True,
        bend_radius_um: float | None = None,
        enable_jps4: bool = False,
        use_indexed_heap: bool = False,
        enable_simple_routes: bool = True,
        primitive_ordering: str = "library",
        heuristic_mode: str = "heading_aware",
        net_order: str = "topological",
    net_order_depth_by_node: dict[str, int] | None = None,
        heap_tie_breaker: str = "smaller_g",
        proactive_congestion_weight: float = 0.0,
        proactive_congestion_radius_cells: int = 0,
        max_iterations: int = 500_000,
        routing_window_scale: float | None = None,
        debug_timing: bool = False,
        verbose_route_diagnostics: bool = False,
        collect_route_stats: bool = False,
        collect_attempt_diagnostics: bool = False,
        enable_internal_photonic_probe_verification: bool = False,
        include_heater_obstacles: bool = False,
        ripup_reroute_config: RipupRerouteConfig | None = None,
        enable_crossings: bool = False,
        node_depths: dict[str, int] | None = None,
        node_ranks: dict[str, int] | None = None,
        edge_ranks: dict[str, dict[str, int]] | None = None,
        crossing_loss: float = 0.0,
        crossing_mode: str = "window",
        crossing_half_size_cells: int = 0,
        min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
        foreign_port_keepout_cells: int = 0,
        fanout_access_mode: str | None = None,
        allow_only_expected_crossings: bool = True,
        crossing_guidance_net_names: frozenset[str] | None = None,
        defer_realization: bool = False,
        enable_checked_endpoint_correction: bool = True,
        config: RoutingConfig | None = None,
    ):
        """Route schematic nets using Rust A* and add one polygon per routed net.

        This function routes each net by:
        1. Building a static obstacle map from the unrouted layout.
        2. Calling the Rust A* router for each net.
        3. Realizing one closed polygon in Rust and inserting it with add_polygon.
        4. Updating blocked cells for subsequent nets using width-aware inflation.

        Parameters:
            unrouted_layout: Component with placed instances but no routes.
            schematic: Schematic with net definitions.
            obstacle_config: Optional obstacle-map configuration.
            debug_dir: Directory where debug SVGs are written when provided.
            debug_prefix: Prefix used for debug SVG filenames.
            debug_route_indices: Optional 1-based net indices for per-route SVG
                export. When omitted, every route SVG is exported.
            debug_stop_after_route_index: Optional 1-based route index where routing
                stops after building full-netlist debug context. Port keepouts and
                crossing context are still derived from all schematic routes.
            route_width_um: Realized waveguide width in micrometers.
            route_layer: Target GDS layer/datatype tuple for route polygons.
            allow_45_degree_turns: If False, omit ±45-degree turn primitives.
            bend_radius_um: Minimum bend radius in micrometers. When omitted, the
                module default is used. The value is rounded up to the active grid
                cell size.
            use_indexed_heap: Benchmark-only queue experiment. Pass 8E measured
                this slower than duplicate-entry BinaryHeap queueing, so the
                production default remains False.
            enable_simple_routes: If False, skip straight/L/Z simple-route
                candidates and force routes through A* search.
            primitive_ordering: Benchmark-only dense A* primitive iteration order.
                Supported values: "library", "long_straight_first",
                "target_biased". Pass 8F keeps "library" as the default.
            heuristic_mode: Dense A* heuristic. Supported values: "distance",
                "heading_aware".
            max_iterations: Maximum A* state expansions per route attempt.
            verbose_route_diagnostics: If True, print per-net route progress and
                detailed A* timing buckets. Failures are always printed.
            enable_internal_photonic_probe_verification: If True, run the expensive
                internal realized-layout photonic probe before returning. The
                production default is False because `routing_flow.py` runs the
                authoritative Python geometry verification on the final layout.
            foreign_port_keepout_cells: Additional global keepout distance in front
                of each endpoint port. Active endpoint ports can open this region;
                dense multi-port instances can also open same-instance fanout
                keepouts, while unrelated nets cannot.
            fanout_access_mode: Dense multi-port access strategy. The default
                "legacy-runway" preserves the existing staggered source-port
                runway reservations. "off" disables those dense reservations.
                "static-stubs" pre-routes deterministic same-instance fanout
                stubs as static geometry and routes from virtual anchor ports.
            defer_realization: If True, keep routed RouteResult objects but skip
                polygon realization. This is used for pre-realization transforms
                such as path-length matching/meander insertion.
            enable_checked_endpoint_correction: If False, skip the checked
                grid-to-port correction pass used by PLM-oriented flows.
            config: Typed configuration (Milestone 1: `RoutingConfig`, whose
                `.router` field is `src/config.rs`'s `RouterConfig`, passed
                to `PyPhotonicRouter`), replacing every `PHOTONIC_ROUTER_*`
                variable this module and the kernel used to read directly.
                When omitted, `RoutingConfig.from_environment()` is used, so
                every caller that passes nothing keeps today's behavior.

        Returns:
            A tuple of (routed_layout, debug_artifacts).
            :param debug_timing:
        """
        self.settings = SessionSettings.from_arguments(
            unrouted_layout,
            schematic,
            obstacle_config=obstacle_config,
            debug_dir=debug_dir,
            debug_prefix=debug_prefix,
            debug_route_indices=debug_route_indices,
            debug_stop_after_route_index=debug_stop_after_route_index,
            route_width_um=route_width_um,
            route_layer=route_layer,
            allow_45_degree_turns=allow_45_degree_turns,
            bend_radius_um=bend_radius_um,
            enable_jps4=enable_jps4,
            use_indexed_heap=use_indexed_heap,
            enable_simple_routes=enable_simple_routes,
            primitive_ordering=primitive_ordering,
            heuristic_mode=heuristic_mode,
            net_order=net_order,
            net_order_depth_by_node=net_order_depth_by_node,
            heap_tie_breaker=heap_tie_breaker,
            proactive_congestion_weight=proactive_congestion_weight,
            proactive_congestion_radius_cells=proactive_congestion_radius_cells,
            max_iterations=max_iterations,
            routing_window_scale=routing_window_scale,
            debug_timing=debug_timing,
            verbose_route_diagnostics=verbose_route_diagnostics,
            collect_route_stats=collect_route_stats,
            collect_attempt_diagnostics=collect_attempt_diagnostics,
            enable_internal_photonic_probe_verification=(
                enable_internal_photonic_probe_verification
            ),
            include_heater_obstacles=include_heater_obstacles,
            ripup_reroute_config=ripup_reroute_config,
            enable_crossings=enable_crossings,
            node_depths=node_depths,
            node_ranks=node_ranks,
            edge_ranks=edge_ranks,
            crossing_loss=crossing_loss,
            crossing_mode=crossing_mode,
            crossing_half_size_cells=crossing_half_size_cells,
            min_straight_cells_per_crossing=min_straight_cells_per_crossing,
            foreign_port_keepout_cells=foreign_port_keepout_cells,
            fanout_access_mode=fanout_access_mode,
            allow_only_expected_crossings=allow_only_expected_crossings,
            crossing_guidance_net_names=crossing_guidance_net_names,
            defer_realization=defer_realization,
            enable_checked_endpoint_correction=enable_checked_endpoint_correction,
            config=config,
        )

        rust_backend = _load_rust_backend()
        if rust_backend is None:
            raise RuntimeError(
                "Rust router backend is not available. Build it with `cargo build` "
                "or `maturin develop` so photonic_router._rust can be imported."
            )

        routed_layout = self.settings.unrouted_layout.copy()
        routed_layout.name = "routed_layout_rust"

        # Slice 2b: everything the run computes lives in one typed record, so a
        # stage takes `(settings, state)` instead of the session itself. The
        # remaining two fields are what the constructor produces.
        self.state = SessionState(rust_backend=rust_backend, routed_layout=routed_layout)

    def run(self) -> tuple[Component, RustRouteDebugArtifacts]:
        """The nine routing phases of `stages.py`, in order.

        Each takes the run's frozen inputs and its mutable state; the value a
        phase returns is the input of a later one (`stages.py` names the types).
        """
        settings, state = self.settings, self.state
        context = obstacle_context.build_static_obstacle_context(settings, state)
        router_setup.configure_router_and_grid(settings, state, context.obstacle_map)
        jobs = route_jobs_stage.build_route_jobs_and_fanout_clustering(
            settings, state, settings.schematic.netlist.routes
        )
        route_jobs_stage.apply_long_straight_fanout_exemption(settings, state)
        planned = crossing_plan_stage.build_crossing_plan_and_port_footprints(
            settings, state, jobs.route_jobs, jobs.endpoint_port_specs_by_instance,
            jobs.dense_port_runway_length_by_spec, context.obstacle_map,
            context.crossing_device_info,
        )
        final = handoff.finalize_route_jobs_and_static_handoff(
            settings, state, planned.route_jobs,
            planned.foreign_port_keepout_cells_by_instance, context.obstacle_map,
        )
        dispatch.dispatch_native_routing(settings, state, final.route_jobs)
        routed = finalize.finalize_routing_results(
            settings, state, final.route_jobs, final.astar_start_s
        )
        verified = verify_repair.repair_and_verify_final_geometry(
            settings, state, routed.routed_net_records
        )
        debug_artifacts = realize.realize_and_assemble_debug_artifacts(
            settings, state, final.route_jobs, verified.routed_net_records,
            verified.illegal_realized_crossings, context.obstacle_map,
            context.obstacle_svg, routed.astar_elapsed_s,
        )
        return state.routed_layout, debug_artifacts

def route_nets_rust(
    unrouted_layout: Component,
    schematic: Schematic,
    *,
    obstacle_config: object | None = None,
    debug_dir: str | Path | None = None,
    debug_prefix: str = "route",
    debug_route_indices: set[int] | None = None,
    debug_stop_after_route_index: int | None = None,
    route_width_um: float = 0.5,
    route_layer: tuple[int, int] = (1, 0),
    allow_45_degree_turns: bool = True,
    bend_radius_um: float | None = None,
    enable_jps4: bool = False,
    use_indexed_heap: bool = False,
    enable_simple_routes: bool = True,
    primitive_ordering: str = "library",
    heuristic_mode: str = "heading_aware",
    net_order: str = "topological",
    net_order_depth_by_node: dict[str, int] | None = None,
    heap_tie_breaker: str = "smaller_g",
    proactive_congestion_weight: float = 0.0,
    proactive_congestion_radius_cells: int = 0,
    max_iterations: int = 500_000,
    routing_window_scale: float | None = None,
    debug_timing: bool = False,
    verbose_route_diagnostics: bool = False,
    collect_route_stats: bool = False,
    collect_attempt_diagnostics: bool = False,
    enable_internal_photonic_probe_verification: bool = False,
    include_heater_obstacles: bool = False,
    ripup_reroute_config: RipupRerouteConfig | None = None,
    enable_crossings: bool = False,
    node_depths: dict[str, int] | None = None,
    node_ranks: dict[str, int] | None = None,
    edge_ranks: dict[str, dict[str, int]] | None = None,
    crossing_loss: float = 0.0,
    crossing_mode: str = "window",
    crossing_half_size_cells: int = 0,
    min_straight_cells_per_crossing: int = DEFAULT_MIN_STRAIGHT_CELLS_PER_CROSSING,
    foreign_port_keepout_cells: int = 0,
    fanout_access_mode: str | None = None,
    allow_only_expected_crossings: bool = True,
    crossing_guidance_net_names: frozenset[str] | None = None,
    defer_realization: bool = False,
    enable_checked_endpoint_correction: bool = True,
    config: RoutingConfig | None = None,
) -> tuple[Component, RustRouteDebugArtifacts]:
    """Route schematic nets through a temporary routing session."""
    session = _RouteNetsRustSession(
        unrouted_layout=unrouted_layout,
        schematic=schematic,
        obstacle_config=obstacle_config,
        debug_dir=debug_dir,
        debug_prefix=debug_prefix,
        debug_route_indices=debug_route_indices,
        debug_stop_after_route_index=debug_stop_after_route_index,
        route_width_um=route_width_um,
        route_layer=route_layer,
        allow_45_degree_turns=allow_45_degree_turns,
        bend_radius_um=bend_radius_um,
        enable_jps4=enable_jps4,
        use_indexed_heap=use_indexed_heap,
        enable_simple_routes=enable_simple_routes,
        primitive_ordering=primitive_ordering,
        heuristic_mode=heuristic_mode,
        net_order=net_order,
        net_order_depth_by_node=net_order_depth_by_node,
        heap_tie_breaker=heap_tie_breaker,
        proactive_congestion_weight=proactive_congestion_weight,
        proactive_congestion_radius_cells=proactive_congestion_radius_cells,
        max_iterations=max_iterations,
        routing_window_scale=routing_window_scale,
        debug_timing=debug_timing,
        verbose_route_diagnostics=verbose_route_diagnostics,
        collect_route_stats=collect_route_stats,
        collect_attempt_diagnostics=collect_attempt_diagnostics,
        enable_internal_photonic_probe_verification=enable_internal_photonic_probe_verification,
        include_heater_obstacles=include_heater_obstacles,
        ripup_reroute_config=ripup_reroute_config,
        enable_crossings=enable_crossings,
        node_depths=node_depths,
        node_ranks=node_ranks,
        edge_ranks=edge_ranks,
        crossing_loss=crossing_loss,
        crossing_mode=crossing_mode,
        crossing_half_size_cells=crossing_half_size_cells,
        min_straight_cells_per_crossing=min_straight_cells_per_crossing,
        foreign_port_keepout_cells=foreign_port_keepout_cells,
        fanout_access_mode=fanout_access_mode,
        allow_only_expected_crossings=allow_only_expected_crossings,
        crossing_guidance_net_names=crossing_guidance_net_names,
        defer_realization=defer_realization,
        enable_checked_endpoint_correction=enable_checked_endpoint_correction,
        config=config,
    )
    return session.run()
